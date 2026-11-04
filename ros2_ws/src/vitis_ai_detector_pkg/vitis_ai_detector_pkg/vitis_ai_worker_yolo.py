#!/usr/bin/env python3
# =============================================================================
# vitis_ai_worker_yolo.py — YOLOv3-tiny 6클래스 DPU worker (KV260)
#
# 노드(vitis_ai_detector_node.py)가 subprocess로 띄우고 stdin/stdout JSON 한 줄
# 프로토콜로 통신한다. 프레임 본체는 /dev/shm 경유(shm_frame.py).
#
#   - 전처리: letterbox 416 (비율 유지 + pad 114) + BGR→RGB + /255 → LUT int8
#     ※ letterbox를 worker가 수행하므로 node 설정은 send_resized_input: false
#   - decode: yolov5(v5~v7)식 grid decode. anchor/stride/class 이름은
#     xmodel 옆의 decode_meta.json에서 로드 (모델 재학습 시 meta만 교체)
#   - pre-filter: objectness raw logit >= logit(min_threshold) 인 cell만
#     sigmoid/decode/NMS 수행
#     (conf = sig(obj)*sig(cls) <= sig(obj) 이므로 안전한 필요조건)
#
# 응답 좌표는 source 해상도 픽셀 기준 (center_x, center_y, width, height).
# =============================================================================

import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np
import xir
import vart

# 이 파일은 일반 스크립트로도(node가 경로로 직접 실행 — sys.path[0]가 이 디렉터리),
# 테스트에서는 패키지 모듈로도 실행된다. 두 경우를 모두 지원한다.
try:
    from vitis_ai_detector_pkg.shm_frame import ShmFrameReader
except ImportError:
    from shm_frame import ShmFrameReader


DEFAULT_THRESHOLD = 0.50
CLASS_THRESHOLDS_BY_NAME = {
    "person": 2.0,  # 영구 OFF: score<=1.0이라 도달 불가. 주석 처리 금지(0.50 폴백 → person 부활)
    # apple threshold hack 제거(2026-07-10): D14 실물 apple 재학습으로 실물 apple 0.5→0.88 →
    #   default 0.50으로 충분. (초기 apple:0.40은 'apple 0.462'가 실은 peach였다는 D13 오판 기반)
    # person 로직은 2026-09 앱 전환(추종/접근)으로 파이프라인에서 제거됨 — 모델만 6-class 유지
}
NMS_THRESHOLD = 0.45  # YOLOv5 기본값
KEEP_TOP_K = 100  # 최대 100개 검출 결과 반환
PAD_VALUE = 114  # yolov5 letterbox 기본 회색


# Graph -> root subgraph -> child subgraph 순으로 탐색하여 DPU subgraph만 반환
# get_root_subgraph, toposort_child_subgraph, has_attr, get_attr는 XIR 객체의 매서드
def get_dpu_subgraphs(graph):
    root = graph.get_root_subgraph()
    children = root.toposort_child_subgraph()  # root의 direct child들을 위상정렬하여여 반환
    return [
        sg for sg in children
        if sg.has_attr("device") and str(sg.get_attr("device")).upper() == "DPU"
    ]


def tensor_shape(tensor):
    if hasattr(tensor, "dims"):
        return tuple(tensor.dims)  # 구버전 Vitis-AI Python runtime (v1.x) 호환
    if hasattr(tensor, "get_shape"):
        return tuple(tensor.get_shape())  # 최신 Vitis-AI Python runtime (v2.x~) 호환
    raise RuntimeError("Cannot get tensor shape")


def tensor_fix_point(tensor):
    if tensor.has_attr("fix_point"):
        return int(tensor.get_attr("fix_point"))
    return 0


# YOLO 디코드용
def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-np.clip(x, -30.0, 30.0)))


def iou(box, boxes):
    # 교집합 찾기
    # box: confidence가 높은 후보 1개 (x1, y1, x2, y2)
    # boxes: 나머지 후보 N개 (N, 4)
    x1 = np.maximum(box[0], boxes[:, 0])
    y1 = np.maximum(box[1], boxes[:, 1])
    x2 = np.minimum(box[2], boxes[:, 2])
    y2 = np.minimum(box[3], boxes[:, 3])

    # 교집합 넓이 계산
    # 겹치는 부분이 없으면 넓이는 0이 되도록 np.maximum(0.0, ...) 사용
    inter = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    area_box = np.maximum(0.0, box[2] - box[0]) * np.maximum(0.0, box[3] - box[1])
    area_boxes = np.maximum(0.0, boxes[:, 2] - boxes[:, 0]) * np.maximum(
        0.0, boxes[:, 3] - boxes[:, 1])
    return inter / np.maximum(area_box + area_boxes - inter, 1e-9)  # 후보 박스의 개수만큼 값 반환


# Non-Maximum Suppression (NMS): Confidence 1등만 남기고 나머지 후보 제거
def nms(boxes, scores, threshold):
    if len(boxes) == 0:
        return []
    order = np.argsort(scores)[::-1]  # 내림차순
    keep = []
    while order.size > 0:
        idx = order[0]
        keep.append(idx)
        if order.size == 1:
            break
        rest = order[1:]
        order = rest[iou(boxes[idx], boxes[rest]) <= threshold]
    return keep


# LUT: uint8 → [0, 1]로 정규화 → int8 양자화 입력 변환 테이블 생성
def build_input_lut(input_fix):
    """uint8 픽셀값 → int8 양자화 입력 LUT (전 채널 공통: v/255 * 2^fix)."""
    values = np.arange(256, dtype=np.float32) / 255.0
    t = np.round(values * float(2 ** input_fix))
    return np.clip(t, -128, 127).astype(np.int8)


def letterbox_lut(image, input_size, lut, out_buf, state=None):
    """letterbox + BGR→RGB + LUT 적용을 한 번에. out_buf에 기록.

    반환: (ratio, pad_x, pad_y) — 좌표 역변환용.
    cv2 경로는 기존 fancy-index 경로와 비트동일: BGR2RGB는 순수 채널
    재배열이고 cv2.LUT(x, lut) == lut[x]다. 패딩은 (nh, nw)가 바뀔 때만
    다시 칠한다(고정 해상도 입력에선 최초 1회) — 콘텐츠 영역은 매 프레임
    전체가 덮어써지므로 안전.
    """
    # 축소 비율 계산
    h, w = image.shape[:2]
    r = min(input_size / h, input_size / w)

    # 리사이즈
    nh, nw = int(round(h * r)), int(round(w * r))
    if (nh, nw) != (h, w):
        resized = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)
    else:
        resized = image
    if resized.dtype != np.uint8:
        resized = resized.astype(np.uint8)

    top = (input_size - nh) // 2
    left = (input_size - nw) // 2

    # 패딩 영역을 캐싱하여 고정, 콘텐츠 영역은 매 프레임 덮어쓰기
    if state is None or state.get('pad_key') != (nh, nw):
        out_buf.fill(lut[PAD_VALUE])
        if state is not None:
            state['pad_key'] = (nh, nw)

    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    # cv2.LUT는 int8 lut를 거부할 수 있어 uint8 뷰로 조회 후 int8로 재해석
    # (비트 패턴 보존이라 값 동일).
    mapped = cv2.LUT(rgb, lut.view(np.uint8)).view(np.int8)
    out_buf[top:top + nh, left:left + nw] = mapped
    return r, left, top


def decode_head(raw, scale, anchors_np, stride, num_classes, thr_q):
    """DPU 출력(NHWC int8) 1개 scale → (boxes_xyxy@input_size, conf[N, nc]).

    yolov5(v5~v7) decode:
      xy = (sig(t_xy)*2 - 0.5 + grid) * stride
      wh = (sig(t_wh)*2)^2 * anchor_pixel
      conf = sig(t_obj) * sig(t_cls)

    scale(=2^fix), thr_q(=min_obj_logit*scale), anchors_np(float32 배열)는
    프레임마다 값이 같아 load_model에서 1회 계산해 받는다 (값 동일).
    """
    _, H, W, C = raw.shape
    na = len(anchors_np)
    no = 5 + num_classes
    if C != na * no:
        raise RuntimeError(f"channel mismatch: {C} != {na}x{no}")

    x = raw.reshape(H, W, na, no)  # (13, 13, 3, 11) 또는 (26, 26, 3, 11)
    # pre-filter는 int8 raw 그대로 비교 (logit 임계값을 fix 스케일로 변환)
    obj_q = x[..., 4]  # 모든 예측의 objectness logit
    ys, xs, as_ = np.nonzero(obj_q >= thr_q)  # 헤드 26이면 thr_q=-0.004 / 헤드 13이면 thr_q=-0.008
    if ys.size == 0:
        return (np.zeros((0, 4), dtype=np.float32),
                np.zeros((0, num_classes), dtype=np.float32))

    sel = x[ys, xs, as_].astype(np.float32) / scale
    sig = sigmoid(sel)
    anchors = anchors_np[as_]

    cx = (sig[:, 0] * 2.0 - 0.5 + xs) * stride
    cy = (sig[:, 1] * 2.0 - 0.5 + ys) * stride
    bw = (sig[:, 2] * 2.0) ** 2 * anchors[:, 0]
    bh = (sig[:, 3] * 2.0) ** 2 * anchors[:, 1]

    boxes = np.stack([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2], axis=1)
    conf = sig[:, 4:5] * sig[:, 5:]
    return boxes, conf


# pipe로 프레임을 받을 때 사용
def read_exact(stream, size):
    chunks = bytearray()
    while len(chunks) < size:
        chunk = stream.read(size - len(chunks))
        if not chunk:
            return None
        chunks.extend(chunk)
    return bytes(chunks)


class YoloWorker:
    def __init__(self, model_path, meta_path, log_file=None):
        self.model_path = model_path
        self.log_file = log_file
        self.last_timing = None
        self.log("worker init (yolo)")

        with open(meta_path) as f:
            meta = json.load(f)
        self.names = list(meta["names"])
        self.num_classes = int(meta["num_classes"])
        self.input_size = int(meta["input_size"])
        self.strides = [int(s) for s in meta["strides"]]
        self.anchors_by_scale = meta["anchors_pixel"]

        self.thresholds = np.array(
            [CLASS_THRESHOLDS_BY_NAME.get(n, DEFAULT_THRESHOLD)
             for n in self.names],
            dtype=np.float32)
        min_thr = float(self.thresholds.min())
        # 경계값 오차로 유효 후보가 잘리지 않도록 epsilon 여유
        self.min_obj_logit = math.log(min_thr / (1.0 - min_thr)) - 1e-3

        self.load_model()

    def log(self, message):
        if not self.log_file:
            return
        with open(self.log_file, "a", encoding="utf-8") as f:
            f.write(message + "\n")
            f.flush()

    def load_model(self):
        self.log(f"load_model start: {self.model_path}")
        # graph/subgraph를 self에 유지 — 해제되면 execute_async가 segfault할 수
        # 있음 (Vitis-AI Python runtime 특성)
        self.graph = xir.Graph.deserialize(self.model_path)
        self.dpu_subgraphs = get_dpu_subgraphs(self.graph)
        if not self.dpu_subgraphs:
            raise RuntimeError("No DPU subgraph found in xmodel")

        self.runner = vart.Runner.create_runner(self.dpu_subgraphs[0], "run")
        self.input_tensors = self.runner.get_input_tensors()
        self.output_tensors = self.runner.get_output_tensors()

        self.input_tensor = self.input_tensors[0]
        self.input_shape = tensor_shape(self.input_tensor)
        self.input_fix = tensor_fix_point(self.input_tensor)
        if (self.input_shape[1] != self.input_size
                or self.input_shape[2] != self.input_size):
            raise RuntimeError(
                f"xmodel input {self.input_shape} != meta input_size "
                f"{self.input_size}")

        self.input_lut = build_input_lut(self.input_fix)
        self.input_buf = np.empty(
            (1, self.input_size, self.input_size, 3), dtype=np.int8, order="C")
        self._letterbox_state = {}   # letterbox_lut용 패딩 채움 캐시

        # 출력 텐서를 spatial 크기로 meta scale과 매칭.
        # 프레임 불변 상수(scale=2^fix, thr_q, anchors 배열)는 여기서 1회 계산.
        self.head_params = []  # (output_index, scale, anchors_np, stride, thr_q)
        for i, tensor in enumerate(self.output_tensors):
            shape = tensor_shape(tensor)
            stride = self.input_size // int(shape[1])
            if stride not in self.strides:
                raise RuntimeError(
                    f"output[{i}] stride {stride} not in meta {self.strides}")
            si = self.strides.index(stride)
            fix = tensor_fix_point(tensor)
            scale = float(2 ** fix)
            anchors_np = np.asarray(
                self.anchors_by_scale[si], dtype=np.float32)
            self.head_params.append(
                (i, scale, anchors_np, stride, self.min_obj_logit * scale))
            self.log(f"output[{i}] shape={shape} stride={stride} fix={fix}")
        if len(self.head_params) != len(self.strides):
            raise RuntimeError("output tensor count != meta scale count")

        self.output_data = [
            np.empty(tensor_shape(t), dtype=np.int8, order="C")
            for t in self.output_tensors
        ]
        self.log(f"load_model done: input_shape={self.input_shape}")

    def detect(self, image, output_width=None, output_height=None):
        t_start = time.perf_counter()
        image_h, image_w = image.shape[:2]
        if output_width is None:
            output_width = image_w
        if output_height is None:
            output_height = image_h

        ratio, pad_x, pad_y = letterbox_lut(
            image, self.input_size, self.input_lut, self.input_buf[0],
            state=self._letterbox_state)
        t_pre = time.perf_counter()

        job_id = self.runner.execute_async([self.input_buf], self.output_data)
        self.runner.wait(job_id)
        t_dpu = time.perf_counter()

        all_boxes = []
        all_conf = []
        for out_idx, scale, anchors_np, stride, thr_q in self.head_params:
            b, c = decode_head(self.output_data[out_idx], scale, anchors_np,
                               stride, self.num_classes, thr_q)
            all_boxes.append(b)
            all_conf.append(c)
        boxes = np.concatenate(all_boxes, axis=0)
        conf = np.concatenate(all_conf, axis=0)

        detections = self.make_detections(
            boxes, conf, ratio, pad_x, pad_y,
            image_w, image_h, int(output_width), int(output_height))
        t_post = time.perf_counter()

        self.last_timing = {
            "pre_ms": (t_pre - t_start) * 1e3,
            "dpu_ms": (t_dpu - t_pre) * 1e3,
            "post_ms": (t_post - t_dpu) * 1e3,
            "worker_ms": (t_post - t_start) * 1e3,
        }
        return detections

    def make_detections(self, boxes, conf, ratio, pad_x, pad_y,
                        image_w, image_h, output_width, output_height):
        # letterbox 역변환(입력 image 좌표) 후, 요청된 출력 해상도로 스케일
        # (send_resized_input: false면 image == source라 sx=sy=1)
        # 
        sx = output_width / float(image_w)
        sy = output_height / float(image_h)

        detections = []
        for cls in range(self.num_classes):
            scores = conf[:, cls]
            mask = scores >= self.thresholds[cls]
            if not np.any(mask):
                continue
            cb = boxes[mask]
            cs = scores[mask]
            for k in nms(cb, cs, NMS_THRESHOLD):
                x1 = (cb[k, 0] - pad_x) / ratio * sx
                y1 = (cb[k, 1] - pad_y) / ratio * sy
                x2 = (cb[k, 2] - pad_x) / ratio * sx
                y2 = (cb[k, 3] - pad_y) / ratio * sy
                x1 = max(0.0, min(float(output_width), x1))
                x2 = max(0.0, min(float(output_width), x2))
                y1 = max(0.0, min(float(output_height), y1))
                y2 = max(0.0, min(float(output_height), y2))
                width = x2 - x1
                height = y2 - y1
                if width <= 1.0 or height <= 1.0:
                    continue
                detections.append({
                    "class_id": int(cls),
                    "class_name": self.names[cls],
                    "confidence": float(cs[k]),
                    "center_x": float(x1 + width / 2.0),
                    "center_y": float(y1 + height / 2.0),
                    "width": float(width),
                    "height": float(height),
                })

        detections.sort(key=lambda item: item["confidence"], reverse=True)
        return detections[:KEEP_TOP_K]


def write_response(response):
    sys.stdout.buffer.write(json.dumps(response).encode("utf-8") + b"\n")
    sys.stdout.buffer.flush()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--meta", default=None,
                        help="decode_meta.json (기본: model과 같은 폴더)")
    parser.add_argument("--log-file", default=None)
    args = parser.parse_args()

    meta_path = args.meta or os.path.join(
        os.path.dirname(os.path.abspath(args.model)), "decode_meta.json")
    if not os.path.exists(meta_path):
        write_response({
            "status": "error",
            "error": f"decode_meta.json not found: {meta_path}",
        })
        sys.exit(1)

    if args.log_file:
        with open(args.log_file, "w", encoding="utf-8") as f:
            f.write("worker log start (yolo)\n")

    worker = YoloWorker(args.model, meta_path, args.log_file)
    write_response({
        "status": "ready",
        "input_shape": list(worker.input_shape),
    })
    worker.log("ready sent")

    shm_reader = ShmFrameReader()
    while True:
        line = sys.stdin.buffer.readline()
        if not line:
            worker.log("stdin closed")
            break

        try:
            request = json.loads(line.decode("utf-8"))
            height = int(request["height"])
            width = int(request["width"])
            channels = int(request["channels"])
            shm_path = request.get("shm_path")
            if shm_path:
                # 프레임은 node의 /dev/shm 매핑 안에 있다 — 읽기 전용 numpy
                # 뷰라 여기서는 복사가 0회다. 그대로 써도 안전하다: node는 우리
                # 응답을 기다리는 동안 이 버퍼에 쓰지 않는다.
                image = shm_reader.view(
                    shm_path, height * width * channels
                ).reshape((height, width, channels))
            else:
                data = read_exact(sys.stdin.buffer, int(request["data_len"]))
                if data is None:
                    worker.log("request data missing")
                    break
                image = np.frombuffer(data, dtype=np.uint8).reshape(
                    (height, width, channels))

            source_width = int(request.get("source_width", width))  # 지금은 노드에서 resize를 하지 않으므로 둘이 같음
            source_height = int(request.get("source_height", height))  # 지금은 노드에서 resize를 하지 않으므로 둘이 같음
            detections = worker.detect(
                image,
                output_width=source_width,
                output_height=source_height,
            )
            write_response({
                "detections": detections,
                "timing": worker.last_timing,
                "error": None,
            })
        except Exception as exc:
            if "worker" in locals():
                worker.log(f"python exception: {exc}")
            write_response({
                "detections": [],
                "error": str(exc),
            })


if __name__ == "__main__":
    main()
