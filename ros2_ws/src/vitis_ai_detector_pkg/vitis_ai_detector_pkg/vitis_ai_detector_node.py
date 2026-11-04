#!/usr/bin/env python3

import csv
import json
import os
import select
import signal
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image

from my_interfaces.msg import Detection, DetectionArray

from vitis_ai_detector_pkg.shm_frame import ShmFrameWriter


def _default_model_path():
    """배포된 모델의 share/ 설치본 경로.

    모델이 이 패키지 안에 함께 배포되므로 /home/<user>/... 같은 경로 가정 없이
    풀린다. launch 파일도 같은 경로를 명시적으로 넘기지만, 이 기본값 덕분에
    `ros2 run`만으로도 동작한다.
    """
    try:
        share = get_package_share_directory("vitis_ai_detector_pkg")
    except Exception:
        return ""
    return os.path.join(share, "models", "yolov3_tiny_7class.xmodel")


def _default_worker_script_path():
    """이 파일과 같은 디렉터리에 있는 YOLO worker 경로."""
    return os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "vitis_ai_worker_yolo.py"
    )


# OpenCV는 BGR 형식을 사용하므로 어떤 ROS 이미지 메시지든 BGR8로 변환하는 유틸리티 함수
def ros_image_to_bgr(msg):
    encoding = msg.encoding.lower()

    if encoding in ("bgr8", "rgb8", "8uc3"):
        channels = 3
    elif encoding in ("bgra8", "rgba8"):
        channels = 4
    elif encoding in ("mono8", "8uc1"):
        channels = 1
    else:
        raise RuntimeError(f"Unsupported image encoding: {msg.encoding}")

    expected_row_bytes = msg.width * channels
    expected_size = msg.height * msg.step  #이미지에 패딩이 붙어서 오는 경우도 있으므로 .step 사용
                                           # step은 한 줄이 차지하는 Byte 수, width는 픽셀 수

    try:
        data = np.frombuffer(msg.data, dtype=np.uint8) #copy 없이 버퍼를 numpy 배열로 변환
    except TypeError:
        data = np.asarray(msg.data, dtype=np.uint8)  #리스트류로 들어오는 경우
    if data.size < expected_size:
        # 데이터가 부족하면 즉시 중단
        raise RuntimeError(f"Image data too small: {data.size} < {expected_size}")

    data = data[:expected_size]
    rows = data.reshape((msg.height, msg.step)) #width를 사용할 경우 패딩이 붙을 수 있으므로 step을 사용
    pixels = rows[:, :expected_row_bytes]

    if channels == 1:
        mono = pixels.reshape((msg.height, msg.width))
        return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

    image = pixels.reshape((msg.height, msg.width, channels))

    if encoding in ("bgr8", "8uc3"):
        return image.copy()
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)

    raise RuntimeError(f"Unsupported image encoding: {msg.encoding}")


class VitisAiDetectorNode(Node):
    def __init__(self):
        super().__init__("vitis_ai_detector_node")

        self.declare_parameter("model_path", _default_model_path())
        self.declare_parameter("worker_script_path", _default_worker_script_path())
        self.declare_parameter("detector_mode", "worker")
        self.declare_parameter("worker_log_path", "")

        # worker로 프레임을 넘길 때 stdin pipe(복사 3-4회, ipc_overhead_ms ~9-12) 대신
        # /dev/shm mmap(memcpy 1회)을 쓴다. false면 예전 inline-bytes 경로로,
        # A/B 비교용 겸 fallback으로 남겨둔 것이다(shm_frame.py).
        self.declare_parameter("worker_shm", True)
        self.declare_parameter("send_resized_input", False)
        self.declare_parameter("input_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("output_topic", "/detections")
        self.declare_parameter("process_period_sec", 0.030)
        self.declare_parameter("timeout_sec", 10.0)
        self.declare_parameter("worker_startup_timeout_sec", 30.0)
        self.declare_parameter("publish_empty_detections", True)
        self.declare_parameter("log_period_sec", 1.0)

        # 값이 있으면 프레임별 타이밍을 메모리에 모아두었다가 종료 시 이 CSV 파일로
        # 쓴다(터미널에 출력하는 대신).
        self.declare_parameter("metrics_csv_path", "")

        # >0이면 딱 이 초만큼만 자동 수집한 뒤 CSV를 자동으로 쓴다(첫 처리 프레임부터
        # 재므로 카메라/worker 기동 시간은 제외된다). 0이면 종료할 때까지 수집한다.
        self.declare_parameter("metrics_duration_sec", 0.0)

        # yield(신뢰성) 요약 — 프레임 예산 흐름 + worker 재시작/타임아웃/지연 횟수를
        # 종료 시 한 번만 쓴다. slow_worker_ms는 거의 멈출 뻔한 왕복을 표시하며,
        # 하드 타임아웃/재시작이 나기 전에 미리 잡히는 선행 지표다.
        self.declare_parameter("yield_csv_path", "")
        self.declare_parameter("slow_worker_ms", 150.0)

        self.model_path = self.get_parameter("model_path").value
        self.worker_script_path = self.get_parameter("worker_script_path").value
        self.detector_mode = str(self.get_parameter("detector_mode").value)
        self.worker_log_path = self.get_parameter("worker_log_path").value

        # 이름에 pid를 넣어 이전 작업에서 강제 종료된 실행이 남긴 파일을 다음 실행에서 재사용하지 않게 한다.
        # main()의 finally에서 unlink한다. 실행 중에는 절대 unlink하면 안 된다
        # (shm_frame.py의 inode 경고 참고 — worker가 죽은 inode를 계속 읽게 된다).
        self.shm_writer = (
            ShmFrameWriter(f"/dev/shm/vitis_ai_frame_{os.getpid()}")
            if bool(self.get_parameter("worker_shm").value) else None
        )
        self.send_resized_input = bool(
            self.get_parameter("send_resized_input").value
        )
        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.process_period_sec = float(self.get_parameter("process_period_sec").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.worker_startup_timeout_sec = float(
            self.get_parameter("worker_startup_timeout_sec").value
        )
        self.publish_empty_detections = bool(
            self.get_parameter("publish_empty_detections").value
        )

        self.log_period_sec = float(self.get_parameter("log_period_sec").value)
        self.metrics_csv_path = str(self.get_parameter("metrics_csv_path").value)

        # None이면 CSV 비활성(기존처럼 터미널 출력), list면 행을 수집한다.
        self.metrics_rows = [] if self.metrics_csv_path else None
        self.metrics_duration_sec = float(
            self.get_parameter("metrics_duration_sec").value
        )
        self.yield_csv_path = str(self.get_parameter("yield_csv_path").value)
        self.slow_worker_ms = float(self.get_parameter("slow_worker_ms").value)
        self.metrics_start_ns = None
        self.metrics_window_closed = False

        self.last_process_time_ns = None
        self.last_log_time_ns = None
        self.worker_process = None
        self.worker_input_height = None
        self.worker_input_width = None
        self.last_worker_timing = None

        # 프레임 예산 흐름 + worker 신뢰성 카운터(yield 요약용).
        self.frames_arrived = 0
        self.frames_overwritten = 0
        self.frames_gate_skipped = 0
        self.frames_processed = 0
        self.frames_empty_out = 0
        self.worker_restart_count = 0
        self.worker_timeout_count = 0
        self.worker_slow_count = 0
        self._last_detect_pub_ns = None

        # 파이프라이닝: 구독 콜백은 최신 프레임을 저장만 하고, 전용 worker 스레드가
        # 쉬지 않고 그것을 소비한다(worker_loop / process_frame). 덕분에 DPU worker가
        # 다음 콜백을 기다리며 노는 일이 없고, 검출이 바쁘다고 카메라가 굶지도 않는다.
        self.frame_lock = threading.Lock()    # latest_msg 접근 보호용
        self.latest_msg = None                # 최신 이미지 메시지
        self.frame_event = threading.Event()  # 새 프레임이 들어오면 잠들어 있던 worker_loop를 깨운다
        # worker_loop 종료 시그널. 종료 중에 pipe가 깨져도
        # worker_loop가 예외를 삼키고 종료되게 한다
        self.shutdown_event = threading.Event()
        self.worker_thread = None             # worker_loop 전용 스레드

        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(
            DetectionArray,
            self.output_topic,
            output_qos,
        )
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            image_qos,
        )

        # worker 모드만 지원한다. 다른 모드(in-process, one-shot)는 각각
        # execute_async에서 segfault, test용으로 쓰던 것이라 지금은 사용하지 않음.
        if self.detector_mode != "worker":
            raise RuntimeError(
                f"Unsupported detector_mode: {self.detector_mode!r} "
                "(only 'worker' is supported)"
            )

        # worker 프로세스(vitis_ai_worker_yolo.py) 시작
        self.start_worker()

        # worker_loop 전용 스레드 생성 후 시작
        self.worker_thread = threading.Thread(
            target=self.worker_loop, name="vitis_ai_worker_loop", daemon=True
        )
        self.worker_thread.start()

        self.get_logger().info(
            f"Vitis-AI detector started in {self.detector_mode} mode: "
            f"{self.input_topic} -> {self.output_topic}"
        )
        self.get_logger().info(
            "Low-latency policy: sensor QoS depth=1, latest frame only, "
            f"process_period_sec={self.process_period_sec:.3f}, "
            f"send_resized_input={self.send_resized_input}, "
            f"worker_shm={self.shm_writer is not None}"
        )

    # 최신 프레임만 "self.latest_msg"에 저장
    def image_callback(self, msg):
        # 가볍게 처리한다: 최신 프레임만 남기고 worker 스레드에 넘긴다.
        # 즉시 반환하므로 executor가 DPU 때문에 블록되는 일이 없다.
        with self.frame_lock:
            self.frames_arrived += 1
            if self.latest_msg is not None:
                # 이전 프레임이 소비되지 못하고 버려짐(최신 프레임만 유지 정책)
                self.frames_overwritten += 1
            self.latest_msg = msg
            self.frame_event.set()  # worker_loop를 깨워 새 프레임을 처리하게 한다

    def worker_loop(self):
        # 전용 스레드: 프레임을 쉬지 않고 처리해 DPU worker가 다음 구독 콜백을
        # 기다리며 노는 일이 없게 한다. 항상 사용 가능한 최신 프레임을 고른다
        # (최신 프레임만 유지 = 지연 최소화).
        while not self.shutdown_event.is_set():
            if not self.frame_event.wait(timeout=0.5):  # 새 프레임이 들어올 때까지 대기, 0.5초마다 깨어나 종료 시그널 확인
                continue
            with self.frame_lock:
                msg = self.latest_msg  # 최신 프레임을 지역변수 msg에 저장
                self.latest_msg = None # 인스턴스 속성 self.latest_msg를 비워 다음 프레임을 받을 준비
                self.frame_event.clear()
            if msg is None:  # 아직 최신 프레임이 없을 때
                continue
            # 워커의 속도를 조절하기 위해 프레임을 건너뛸 때
            # (vitis_ai_detector.yaml에서 process_period_sec: 0.030 = 30ms.
            #  33ms가 아니다 — color 프레임 도착 주기 33.3ms '안쪽'에 여유를 두고
            #  들어가야 jitter로 2프레임 주기(15Hz)로 되돌아가지 않는다. YAML 주석 참고)
            if not self.should_process_frame():
                self.frames_gate_skipped += 1
                continue
            self.frames_processed += 1
            self.process_frame(msg)

    def process_frame(self, msg):
        processing_start_ns = time.perf_counter_ns()
        # 처리를 시작하는 시점의 프레임 나이 = 카메라 촬영 -> 여기까지
        # (USB + realsense 노드 + DDS + 구독 큐 + 핸드오프). processing_ms보다
        # 앞에서 일어나는 구간이다.
        frame_age_in_ms = self.frame_age_ms(msg)
        log_this_frame = self.should_log()  # 터미널에 출력되는 로그 주기 설정
        try:
            image = ros_image_to_bgr(msg)  # 이미지 메시지를 OpenCV BGR 형식으로 변환
            image_ready_ns = time.perf_counter_ns()

            # worker 프로세스(vitis_ai_worker_yolo.py)를 통해 객체 검출 수행
            detections = self.detect_with_worker(image)
            detection_done_ns = time.perf_counter_ns()

            out_msg = DetectionArray()
            out_msg.header = msg.header  # 컬러 이미지의 타임스탬프 및 프레임 ID
            out_msg.detections = detections  # 검출 결과

            if len(detections) == 0:
                self.frames_empty_out += 1

            if detections or self.publish_empty_detections:
                self.publisher.publish(out_msg)
            detection_publish_done_ns = time.perf_counter_ns()

            frame_done_ns = time.perf_counter_ns()  # 프레임 처리 종료 시각

            # --- 프레임별 지표 (가벼운 산술, 매 프레임 계산) ---
            # detect_ms = img_ms + worker_call_ms
            # worker_call_ms = worker_ms(순수 연산) + ipc_overhead_ms
            # worker_ms = pre_ms + dpu_ms + post_ms (worker가 보고한 값)
            processing_ms = (frame_done_ns - processing_start_ns) / 1e6
            detection_ms = (detection_done_ns - processing_start_ns) / 1e6
            detection_publish_ms = (
                detection_publish_done_ns - detection_done_ns
            ) / 1e6
            img_ms = (image_ready_ns - processing_start_ns) / 1e6
            worker_call_ms = (detection_done_ns - image_ready_ns) / 1e6
            frame_age_ms = self.frame_age_ms(msg)
            wt = self.last_worker_timing or {}
            worker_ms = wt.get("worker_ms")
            pre_ms = wt.get("pre_ms")
            dpu_ms = wt.get("dpu_ms")
            post_ms = wt.get("post_ms")
            # 구버전) pipe 모드: 64 KB pipe로 전체 프레임을 3-4회 복사 — 실측 평균 13.4 ms.
            # 현재) shm 모드(worker_shm, 2026-08-05): /dev/shm에 memcpy 1회 +
            # JSON 왕복 + 깨우기 — 실측 평균 4.65 ms (p50 ~3.2, 꼬리가 두꺼움).
            # 2026-08-05 A/B: ipc 13.4 -> 4.65 ms, throughput 18.4 -> 21.7 Hz
            # (docs/vision/throughput.md §4 방법 0).
            # worker_call_ms - worker_ms = ipc_overhead_ms
            ipc_overhead_ms = (
                worker_call_ms - worker_ms if worker_ms is not None else None
            )

            # slow_worker_ms보다 오래 걸린 worker_call_ms를 카운트한다
            if worker_call_ms > self.slow_worker_ms:
                self.worker_slow_count += 1

            # 이번 프레임과 직전 프레임의 detect publish 간격(ms)
            detect_period_ms = None
            if self._last_detect_pub_ns is not None:
                detect_period_ms = (
                    detection_publish_done_ns - self._last_detect_pub_ns
                ) / 1e6
            self._last_detect_pub_ns = detection_publish_done_ns

            if self.metrics_rows is not None and not self.metrics_window_closed:
                if self.metrics_start_ns is None:
                    self.metrics_start_ns = processing_start_ns
                    if self.metrics_duration_sec > 0:
                        self.get_logger().info(
                            "Metrics window started: collecting "
                            f"{self.metrics_duration_sec:.0f}s from the first frame"
                        )
                self.metrics_rows.append({
                    "stamp_sec": msg.header.stamp.sec,
                    "stamp_nanosec": msg.header.stamp.nanosec,
                    "capture_sec": msg.header.stamp.sec,
                    "capture_nanosec": msg.header.stamp.nanosec,
                    "detect_period_ms": self._round(detect_period_ms),
                    "count": len(detections),
                    "image_w": msg.width,
                    "image_h": msg.height,
                    "processing_ms": self._round(processing_ms),
                    "detect_ms": self._round(detection_ms),
                    "img_ms": self._round(img_ms),
                    "worker_call_ms": self._round(worker_call_ms),
                    "ipc_overhead_ms": self._round(ipc_overhead_ms),
                    "pre_ms": self._round(pre_ms),
                    "dpu_ms": self._round(dpu_ms),
                    "post_ms": self._round(post_ms),
                    "worker_ms": self._round(worker_ms),
                    "detection_publish_ms": self._round(detection_publish_ms),
                    "age_in_ms": self._round(frame_age_in_ms),
                    "frame_age_ms": self._round(frame_age_ms),
                })
                if (
                    self.metrics_duration_sec > 0
                    and (time.perf_counter_ns() - self.metrics_start_ns) / 1e9
                    >= self.metrics_duration_sec
                ):
                    self.save_metrics_csv()
                    self.metrics_window_closed = True
                    self.get_logger().info(
                        f"Metrics window {self.metrics_duration_sec:.0f}s complete: "
                        f"{len(self.metrics_rows)} rows saved to "
                        f"{self.metrics_csv_path}. You can stop the launch now "
                        f"(Ctrl-C)."
                    )

            if log_this_frame:
                if self.metrics_rows is not None:
                    if self.metrics_window_closed:
                        self.get_logger().info(
                            f"Metrics window done: {len(self.metrics_rows)} rows "
                            f"saved. Idle (Ctrl-C to exit)."
                        )
                    else:
                        self.get_logger().info(
                            f"Collecting metrics: {len(self.metrics_rows)} rows "
                            f"(CSV -> {self.metrics_csv_path})"
                        )
                else:
                    age_text = (
                        f"{frame_age_ms:.1f}"
                        if frame_age_ms is not None else "unknown"
                    )
                    age_in_text = (
                        f"{frame_age_in_ms:.1f}"
                        if frame_age_in_ms is not None else "unknown"
                    )
                    if worker_ms is not None:
                        breakdown = (
                            f"img_ms={img_ms:.1f}, "
                            f"worker_call_ms={worker_call_ms:.1f}, "
                            f"pre_ms={pre_ms:.1f}, "
                            f"dpu_ms={dpu_ms:.1f}, "
                            f"post_ms={post_ms:.1f}, "
                            f"worker_ms={worker_ms:.1f}, "
                            f"ipc_overhead_ms={ipc_overhead_ms:.1f}, "
                        )
                    else:
                        breakdown = (
                            f"img_ms={img_ms:.1f}, "
                            f"worker_call_ms={worker_call_ms:.1f}, "
                        )
                    self.get_logger().info(
                        f"Published detections: count={len(detections)}, "
                        f"image={msg.width}x{msg.height}, mode={self.detector_mode}, "
                        f"processing_ms={processing_ms:.1f}, "
                        f"detect_ms={detection_ms:.1f}, "
                        f"{breakdown}"
                        f"detection_publish_ms={detection_publish_ms:.1f}, "
                        f"age_in_ms={age_in_text}, "
                        f"frame_age_ms={age_text}"
                    )

        # read_worker_json에서 select.select()가 타임아웃되면 subprocess.TimeoutExpired가 발생한다
        except subprocess.TimeoutExpired:
            self.worker_timeout_count += 1
            # 종료 중의 broken pipe/timeout은 정상 현상 → 에러 로그로 시끄럽게 하지 않는다.
            # (get_logger()도 context가 내려간 뒤엔 "context invalid" 경고를 유발한다.)
            if not self.shutdown_event.is_set():
                self.get_logger().error("Vitis-AI detector timeout")
        except Exception as exc:
            if not self.shutdown_event.is_set():
                self.get_logger().error(f"Detection failed: {exc}")

    def start_worker(self):
        # 프로세스가 있고 살아있으면 그냥 return
        if self.worker_process is not None and self.worker_process.poll() is None:
            return

        command = [
            sys.executable,
            self.worker_script_path,
            "--model",
            self.model_path,
        ]

        # 현재 log path 설정 없음
        if self.worker_log_path:
            command.extend(["--log-file", self.worker_log_path])

        # worker 프로세스 띄우기
        self.get_logger().info("Starting Vitis-AI worker process")
        self.worker_process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # 30초동안 worker ready 신호를 기다린다
        # 30초 안에 ready 신호가 없으면 read_worker_json이 subprocess.TimeoutExpired를
        # 발생시킨다 (worker가 이미 죽어 stdout이 닫힌 경우에는 RuntimeError)
        # ready = {"status": "ready", "input_shape": [1, 416, 416, 3]} 형태로 받는다 (YOLO worker 기준)
        try:
            ready = self.read_worker_json(
                "worker ready",
                timeout_sec=self.worker_startup_timeout_sec,
            )
        # RuntimeError, subprocess.TimeoutExpired 등 모든 예외를 잡아
        # worker를 종료하고 다시 raise
        except Exception:
            self.stop_worker()
            raise
        if ready.get("status") != "ready":
            raise RuntimeError(f"Unexpected worker ready response: {ready}")

        input_shape = ready.get("input_shape")
        if not isinstance(input_shape, list) or len(input_shape) != 4:
            raise RuntimeError(f"Invalid worker input shape: {input_shape}")
        self.worker_input_height = int(input_shape[1])
        self.worker_input_width = int(input_shape[2])

        self.get_logger().info(
            f"Vitis-AI worker ready: input_shape={input_shape}"
        )

    def stop_worker(self):
        process = self.worker_process  # 지역변수에 먼저 현재 프로세스를 저장
        self.worker_process = None
        self.worker_input_height = None
        self.worker_input_width = None

        # 현재 살아있는 worker 프로세스가 없으면 return
        if process is None:
            return

        # worker 프로세스가 살아있다면 3단계에 걸쳐 종료 시도
        if process.poll() is None:
            try:
                # 1. worker 프로세스(vitis_ai_worker_yolo.py)의 main() 루프가
                #    EOF를 감지해 스스로 종료한다
                process.stdin.close()
            except Exception:
                pass  # 만약 pipe가 이미 깨졌다면 Exception이 발생하므로 무시하고 다음 단계로 넘어간다
            try:
                process.wait(timeout=2.0) # worker가 종료될 때까지 2초 기다린다
            except subprocess.TimeoutExpired:
                process.terminate()  # 2. worker가 종료되지 않으면 SIGTERM을 보내 종료를 시도한다
                try:
                    process.wait(timeout=2.0) # SIGTERM 후에도 종료되지 않으면 2초 더 기다린다
                except subprocess.TimeoutExpired:
                    os.kill(process.pid, signal.SIGKILL)  # 3. SIGKILL을 보내 강제 종료한다

    def restart_worker(self):
        self.worker_restart_count += 1
        self.stop_worker()
        self.start_worker()

    # start_worker에서 ready 신호를 기다리거나 detect_with_worker에서 worker inference 결과를 기다릴 때 사용
    def read_worker_json(self, label, timeout_sec=None):
        process = self.worker_process
        # worker handle과 stdout pipe가 있는지만 확인
        # (죽었는지는 아래 readline이 빈 줄일 때 판정)
        if process is None or process.stdout is None:
            raise RuntimeError("Worker process is not running")

        if timeout_sec is None:
            timeout_sec = self.timeout_sec  # 현재 10초로 설정되어 있음

        ready, _, _ = select.select([process.stdout], [], [], timeout_sec)
        if not ready:
            # 시간 안에 준비되지 않으면 TimeoutExpired 예외 발생
            raise subprocess.TimeoutExpired(label, timeout_sec)

        line = process.stdout.readline()
        if not line:
            # 앞서 select가 통과시켰는데 line이 비어있으면 worker가 죽은 것이므로
            # 아래에서 RuntimeError 발생
            returncode = process.poll()
            stderr = ""
            if process.stderr is not None and returncode is not None:
                stderr = process.stderr.read().decode("utf-8", errors="replace")
            raise RuntimeError(
                f"Worker closed stdout during {label}: "
                f"returncode={returncode}, stderr={stderr}"
            )

        return json.loads(line.decode("utf-8"))

    # 프레임 한 장을 worker에 보내고 검출 결과를 받는다. worker가 죽었으면 재시작 후 다시 시도한다
    def detect_with_worker(self, image):
        # 이미지가 연속 메모리 배열인지 확인하고, 아니라면 연속 메모리 배열로 변환
        image = np.ascontiguousarray(image, dtype=np.uint8)
        if image.ndim != 3 or image.shape[2] != 3:  # 3차원 배열인지 확인
            raise RuntimeError(f"Expected BGR image with 3 channels, got {image.shape}")

        if self.worker_process is None or self.worker_process.poll() is not None:
            # 종료 중이면 worker를 되살리지 않는다: 새 worker 로드(~4.5s)가
            # launch의 5초 SIGINT 유예를 넘겨 SIGTERM 강제종료(exit -15)를 유발한다.
            if self.shutdown_event.is_set():
                raise RuntimeError("worker unavailable during shutdown")
            self.restart_worker()

        # image.shape는 (height, width, channels) 순서로 반환
        source_height, source_width, _ = image.shape
        worker_image = image

        # 현재 비활성화됨 (yaml에서 False로 설정되어 있음 / worker 프로세스에서 letterbox 처리함)
        if self.send_resized_input:
            if self.worker_input_width is None or self.worker_input_height is None:
                raise RuntimeError("Worker input dimensions are not available")
            if (
                source_width != self.worker_input_width
                or source_height != self.worker_input_height
            ):
                worker_image = cv2.resize(
                    image,
                    (self.worker_input_width, self.worker_input_height),
                    interpolation=cv2.INTER_LINEAR,
                )

        # 위에서 resize를 했을 경우를 대비하여 한번 더 연속 메모리 배열로 변환
        worker_image = np.ascontiguousarray(worker_image, dtype=np.uint8)
        height, width, channels = worker_image.shape
        request = {
            "height": int(height),
            "width": int(width),
            "channels": int(channels),
            "data_len": int(worker_image.nbytes),
            "source_height": int(source_height),
            "source_width": int(source_width),
        }

        try:
            if self.shm_writer is not None:
                # 프레임은 /dev/shm으로 보내고(memcpy 1회), pipe에는 JSON 헤더만
                # 실어 보낸다. 요청보다 버퍼를 먼저 써야 한다 — worker는 요청 줄이
                # 도착하는 즉시 버퍼를 읽기 때문이다.
                self.shm_writer.write(worker_image)  # shm에 image를 기록
                request["data_len"] = 0  # pipe에는 image 데이터는 보내지 않으므로 data_len을 0으로 설정
                # shm_path를 request에 추가하여 worker가 shm에서 데이터를 읽게 한다
                request["shm_path"] = self.shm_writer.path
                # pipe에 JSON 헤더를 보내면, worker는 이를 읽고 shm에서 데이터를 읽는다
                self.worker_process.stdin.write(
                    json.dumps(request).encode("utf-8") + b"\n")
            else:
                # worker_shm=False이면 예전처럼 pipe로 전체 프레임을 보내고, worker는
                # pipe에서 읽는다. 64 KB pipe로 전체 프레임을 3-4회 복사 — ipc 실측 평균 13.4 ms.
                # worker_shm=True이면 /dev/shm에 memcpy 1회 + JSON 왕복 + 깨우기 —
                # ipc 실측 평균 4.65 ms (2026-08-05 A/B, docs/vision/throughput.md §4 방법 0).
                self.worker_process.stdin.write(
                    json.dumps(request).encode("utf-8") + b"\n")
                self.worker_process.stdin.write(worker_image.tobytes())
            self.worker_process.stdin.flush()
            response = self.read_worker_json("worker inference")
        except (BrokenPipeError, RuntimeError, subprocess.TimeoutExpired):
            # BrokenPipeError: worker가 죽어 pipe가 깨졌을 때 발생
            # RuntimeError: read_worker_json에서 worker가 죽어 stdout이 닫혔을 때 발생
            # subprocess.TimeoutExpired: read_worker_json에서 select.select()가 타임아웃되었을 때 발생
            # 종료 중이면 깨진 pipe는 worker가 공유 SIGINT로 죽은 정상 현상이다.
            # crash로 오해해 재시작하지 않고 조용히 올려보낸다.
            if not self.shutdown_event.is_set():
                self.restart_worker()
            raise

        if response.get("error"):
            # 통신은 성공했지만 worker 내부에서 오류가 발생한 경우
            raise RuntimeError(f"worker error: {response['error']}")

        # timing = worker가 보고한 pre_ms, dpu_ms, post_ms, worker_ms
        self.last_worker_timing = response.get("timing")
        return self.json_items_to_detections(response.get("detections", []))  # ROS 메시지로 변환하여 반환

    # worker가 반환한 JSON detections(python dict 목록)를 ROS Detection 메시지 리스트로 변환
    # (header를 붙인 DetectionArray 조립은 호출부인 process_frame이 한다)
    def json_items_to_detections(self, items):
        detections = []
        for item in items:
            det = Detection()
            det.class_id = int(item["class_id"])
            det.class_name = str(item["class_name"])
            det.confidence = float(item["confidence"])
            det.center_x = float(item["center_x"])
            det.center_y = float(item["center_y"])
            det.width = float(item["width"])
            det.height = float(item["height"])
            detections.append(det)

        return detections

    def should_process_frame(self):
        # 주기가 0이면 항상 처리한다.
        # (vitis_ai_detector.yaml에서 0.030으로 설정되어 있음 — 0.033은 arrival jitter
        #  여유가 0.3ms뿐이라 2프레임 주기로 되돌아갈 위험이 있어 의도적으로 배제한 값)
        if self.process_period_sec <= 0.0:
            return True

        now_ns = self.get_clock().now().nanoseconds

        # 이전 프레임 처리 시각이 없으면 이번 프레임을 처리한다
        if self.last_process_time_ns is None:
            self.last_process_time_ns = now_ns
            return True

        # 이전 프레임 처리 시각과 현재 시각의 차이가 process_period_sec보다 작으면 이번 프레임을 건너뛴다
        elapsed_sec = (now_ns - self.last_process_time_ns) / 1e9
        if elapsed_sec < self.process_period_sec:
            return False

        # 이전 프레임 처리 시각과 현재 시각의 차이가 process_period_sec보다 크거나 같으면 이번 프레임을 처리한다
        self.last_process_time_ns = now_ns
        return True

    def should_log(self):
        now_ns = self.get_clock().now().nanoseconds

        # 이전 로그 시각이 없으면(첫 호출) 이번 프레임은 로그를 출력한다.
        # (log_period_sec는 vitis_ai_detector.yaml에서 1.0으로 설정되어 있음.
        #  should_process_frame과 달리 '주기가 0이면 통과' 분기는 없다)
        if self.last_log_time_ns is None:
            self.last_log_time_ns = now_ns
            return True

        # 이전 로그 시각과 현재 시각의 차이가 log_period_sec보다 작으면 이번 프레임은 로그를 출력하지 않는다
        elapsed_sec = (now_ns - self.last_log_time_ns) / 1e9
        if elapsed_sec < self.log_period_sec:
            return False

        # 이전 로그 시각과 현재 시각의 차이가 log_period_sec보다 크거나 같으면 이번 프레임은 로그를 출력한다
        self.last_log_time_ns = now_ns
        return True

    def frame_age_ms(self, msg):
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000  # 촬영 시각 (ROS 메시지 헤더)
        stamp_ns += int(msg.header.stamp.nanosec)  # 촬영 시각 (ROS 메시지 헤더)
        if stamp_ns <= 0:
            return None

        age_ns = self.get_clock().now().nanoseconds - stamp_ns  # 현재 시각 - 촬영 시각 = 프레임 나이 (ns)
        if age_ns < 0:
            return None
        return age_ns / 1e6  # ms로 변환

    @staticmethod  # self 없이 호출 가능
    def _round(value, ndigits=3):
        # CSV에 맞게: 실수는 반올림하고, 값이 없으면 빈 칸으로 둔다.
        if value is None:
            return ""
        return round(float(value), ndigits)

    def stop_pipeline(self):
        # worker 스레드만 안전하게 종료한다(worker 프로세스 종료는 stop_worker 담당). 종료 시 지표 CSV를 기록하기 위해
        # 지표를 내보내거나 worker 프로세스를 죽이기 전에 worker 스레드를 먼저 멈춘다.
        # 그래야 metrics_rows에 행이 더 이상 추가되지 않는다.
        self.shutdown_event.set()
        self.frame_event.set()  # 루프가 대기 중이면 깨운다
        if self.worker_thread is not None:
            self.worker_thread.join(timeout=3.0)
            self.worker_thread = None

    def write_yield_summary(self):
        # 프레임 예산 흐름 + worker 신뢰성 집계를 한 행으로, 종료 시 한 번만 쓴다.
        # yield_csv_path가 설정돼 있지 않으면 쓰지 않는다. 카운터 자체는 항상
        # 증가하며(가벼운 정수 연산), 파일로 쓰는 것만 조건부다.
        if not self.yield_csv_path or self.frames_arrived == 0:
            return
        arrived = max(self.frames_arrived, 1)
        processed = max(self.frames_processed, 1)
        row = {
            "node": "vitis_ai_detector",
            "frames_arrived": self.frames_arrived,
            "frames_overwritten_dropped": self.frames_overwritten,
            "frames_gate_skipped": self.frames_gate_skipped,
            "frames_processed": self.frames_processed,
            "process_rate_of_arrived": round(self.frames_processed / arrived, 4),
            "frames_empty_out": self.frames_empty_out,
            "empty_rate_of_processed": round(self.frames_empty_out / processed, 4),
            "worker_restart_count": self.worker_restart_count,
            "worker_timeout_count": self.worker_timeout_count,
            "worker_slow_count": self.worker_slow_count,
        }
        directory = os.path.dirname(self.yield_csv_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        # 원자적 쓰기(임시 파일 + replace) — 성능 CSV와 같은 이유(두 번째 시그널 대비)
        tmp_path = self.yield_csv_path + ".tmp"
        with open(tmp_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(row.keys()))
            writer.writeheader()
            writer.writerow(row)
        os.replace(tmp_path, self.yield_csv_path)
        self.get_logger().info(
            f"Saved detector yield summary to {self.yield_csv_path}"
        )

    def save_metrics_csv(self):
        if not self.metrics_csv_path or not self.metrics_rows:
            return
        # 수집 창이 닫혔고 이미 디스크에 있으면, 종료 시 다시 저장해봐야 같은 내용을
        # 덮어쓸 뿐이다. 종료 도중 시그널이 와도 문제없도록 건너뛴다.
        if self.metrics_window_closed and (
            getattr(self, "_metrics_saved_rows", -1) == len(self.metrics_rows)
        ):
            return
        # 컬럼을 실제 행에서 뽑아낸다(지금은 capture_sec/capture_nanosec/
        # detect_period_ms도 들어 있다). 그래야 헤더가 dict와 다시 어긋나지 않는다.
        # 예전에는 하드코딩한 목록이 새 join 키를 빠뜨려 DictWriter가 예외를 냈고,
        # 헤더만 있는 파일이 남았다.
        fieldnames = list(self.metrics_rows[0].keys())
        try:
            directory = os.path.dirname(self.metrics_csv_path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            # 원자적 쓰기(임시 파일 + os.replace): 종료 시 `finally`에서 실행되는데
            # 그 사이에 두 번째 SIGINT가 올 수 있다(그룹 Ctrl-C + launch 자체 전파).
            # 그냥 "w"로 다시 쓰다가 중간에 끊기면 CSV가 잘려나간다
            # (2026-07-31 실측: 3853행 -> 611행).
            tmp_path = self.metrics_csv_path + ".tmp"
            with open(tmp_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.metrics_rows)
            os.replace(tmp_path, self.metrics_csv_path)
            self._metrics_saved_rows = len(self.metrics_rows)
            self.get_logger().info(
                f"Saved {len(self.metrics_rows)} metric rows to "
                f"{self.metrics_csv_path}"
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to save metrics CSV: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = None # finally 블록에서 node.destroy_node()를 안전하게 호출하기 위해 None으로 초기화

    # SIGTERM(예: pkill)도 Ctrl-C처럼 동작하게 만들어 finally 블록이 실행되고
    # 종료 시 지표 CSV가 그대로 기록되도록 한다
    # SIGTERM으로 죽으면 CSV가 기록되지 않고 launch가 종료되기 때문에 SIGINT와 SIGTERM을 동일하게 처리하도록 한다
    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _on_sigterm)

    try:
        node = VitisAiDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 쓰기를 두 번째 시그널로부터 보호한다: 터미널 Ctrl-C는 프로세스
        # 그룹 전체에 전달되고 launch도 자체 SIGINT를 전파하기 때문에, Ctrl-C 입력 시 SIGINT가 두 번 들어온다
        # 예전에는 KeyboardInterrupt가 바로 이 블록 안에서 발생해 CSV 쓰기를 중간에
        # 중단시켰다(2026-07-31 파일 잘림 사고).
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.stop_pipeline()  # worker 스레드 종료
            node.save_metrics_csv()  # 성능 CSV 기록
            node.write_yield_summary()  # yield CSV 기록
            node.stop_worker()  # worker 프로세스 종료
            if node.shm_writer is not None:  # 공유메모리 파일 삭제
                node.shm_writer.unlink()   # worker가 끝났으니 inode를 지워도 안전
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
