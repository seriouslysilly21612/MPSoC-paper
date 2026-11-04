# 코드 정독 가이드 — detector node + YOLO worker

> 짝 문서: 함수들의 **시간 순 상호작용**은 [`pipeline_sequence_diagrams.md`](pipeline_sequence_diagrams.md) (파일별 mermaid sequence diagram, 2026-08-19) — 이 문서는 줄 단위 정독, 저쪽은 흐름도.

`vitis_ai_detector_node.py`(853줄)와 `vitis_ai_worker_yolo.py`(463줄)를 **코드에 나온 순서 그대로** 함수 하나씩 짚어가며 "무엇을 하고, 그 내용이 무슨 뜻인지" 설명한다.
작성 2026-07-20 · 갱신 2026-09-03 — **줄번호 전면 재계산**(작성 시점 이후 코드가 밀려 있었다) + shm IPC 반영 + overlay/SSD 잔재 제거 반영.

---

## 0. 전체 아키텍처 / 흐름

### (A) 두 프로세스 + 한 파이프

```
        /camera/camera/color/image_raw  (sensor_msgs/Image, RGB8 848x480)
                     │
                     ▼
┌──────────────────────────────────────────────────────────────┐
│  프로세스 A : vitis_ai_detector_node.py   (rclpy O = ROS 배관) │
│                                                                │
│   image_callback ──▶ [latest_msg]  ──▶ worker_loop            │
│   (구독 콜백)         (최신 1장만)      (전용 스레드)          │
│                                            │                   │
│                                            ▼                   │
│                                     process_frame              │
│                                       │        ▲               │
│                              stdin ▼  │        │ ▲ stdout      │
└────────────────────────────────────┼──┼────────┼──────────────┘
                                      │  │        │   IPC
              JSON헤더 1줄(pipe) + 프레임(/dev/shm) │  │ JSON 응답(pipe)
┌─────────────────────────────────────┼──┼────────┼────────────┐
│  프로세스 B : vitis_ai_worker_yolo.py ▼  │        │ (rclpy X)  │
│                                                                │
│   main 루프 ──▶ worker.detect ──▶ [DPU 하드웨어] ──▶ 응답      │
│   (stdin 읽기)   letterbox→추론→decode→NMS                     │
└──────────────────────────────────────────────────────────────┘
                     │
                     ▼
        /detections  (my_interfaces/DetectionArray)
```

- **프로세스 A(node)** 는 ROS를 안다(`rclpy`). 카메라를 구독하고, worker에게 일을 시키고, 결과를 `/detections`로 발행하고, 프레임마다 시간을 잰다.
- **프로세스 B(worker)** 는 ROS를 모른다. `xir`/`vart`(Vitis-AI 런타임)만 쓴다. 이미지 바이트를 받아 박스를 뱉는 순수 계산 프로세스.
- **왜 둘로 나눴나**: VART `execute_async()`가 rclpy 노드 안에서 돌면 XIR 객체 GC 문제로 segfault가 난다 → 프로세스로 격리해서 회피(`worker load_model:244`, `node start_worker:458`).
- **계약(contract)**: A→B는 `{height,width,channels,data_len,source_*,shm_path}` **JSON 한 줄**(pipe) + **프레임 본체는 `/dev/shm` mmap**. B→A는 `{detections,timing,error}` **JSON 한 줄**. 프레이밍은 개행 1개 = JSON 1개다. 이 계약 덕에 모델을 바꿔도 worker 내부만 바뀌고 node는 그대로다.
- **shm 전환(2026-08-05)**: 예전에는 프레임 raw 바이트도 pipe로 밀어넣었는데 64 KB pipe라 3~4회 복사가 나서 ipc 평균 13.4 ms였다. `/dev/shm` memcpy 1회로 바꿔 **4.65 ms**가 됐고 throughput은 18.4 → 21.7 Hz. `worker_shm: false`로 두면 옛 pipe 경로가 그대로 폴백으로 남아 있다(`shm_frame.py`).

### (B) 프레임 한 장의 여정 (함수 흐름)

```
[node] image_callback → worker_loop → process_frame
          │                               ├─ ros_image_to_bgr        (ROS Image → numpy BGR)
          │                               └─ detect_with_worker ─────┐
          │                                            shm write + JSON헤더 │
[worker]  main 루프 → shm_reader.view → worker.detect                ▼
                                     ├─ letterbox_lut   (전처리: 416 letterbox + int8)
                                     ├─ execute_async/wait  (★ DPU 추론)
                                     ├─ decode_head ×heads  (grid → 박스+conf)
                                     └─ make_detections (NMS + letterbox 역변환)
                                            │ write_response (pipe)
[node]  read_worker_json → json_items_to_detections → publish → /detections
```

아래부터는 이 함수들을 **코드에 나온 순서대로** 하나씩 본다.

---

## 1. `vitis_ai_detector_node.py` — 코드 순서대로

### 모듈 상단 (import + 상수)

- **import [3–26]**: `subprocess`(worker 프로세스 띄우기), `select`(worker 응답 타임아웃), `signal`(SIGTERM 처리), `threading`(파이프라이닝), `cv2`/`numpy`(이미지), `rclpy`(ROS). `my_interfaces`에서 커스텀 메시지 `Detection`, `DetectionArray`, `shm_frame`에서 `ShmFrameWriter`.
- **모듈 상수 없음**: 예전에 있던 `BOX_COLORS`(overlay 박스 색)는 보드 그리기 경로와 함께 삭제됐다(2026-09-03).

### `_default_model_path()` [29]
> 역할: launch 없이 `ros2 run`만 해도 모델을 찾도록 기본 xmodel 경로를 준다.

`get_package_share_directory("vitis_ai_detector_pkg")`로 패키지 설치 위치를 얻어 `models/yolov3_tiny_7class.xmodel`을 반환. 패키지 안에 모델이 동봉돼 있어 `/home/<user>/...` 같은 절대경로 가정이 없다. 실패하면 `""`.

### `_default_worker_script_path()` [43]
> 역할: 쓸 worker 스크립트의 기본 경로.

`os.path.dirname(os.path.abspath(__file__))`, 즉 **이 노드 파일과 같은 디렉터리**의 `vitis_ai_worker_yolo.py`. 그래서 node와 worker가 항상 같은 소스 트리에서 짝이 된다.

### `ros_image_to_bgr(msg)` [51]
> 역할: ROS `Image` 메시지를 OpenCV가 쓰는 numpy **BGR** 배열로 변환.

- encoding을 보고 채널 수를 정한다(bgr8/rgb8=3, bgra8=4, mono8=1) [56–65].
- `np.frombuffer(msg.data, uint8)` — **복사 없이** 바이트를 numpy로 본다 [71].
- `rows = data.reshape((height, step))` 후 `[:, :expected_row_bytes]` — **`msg.step`(한 줄의 바이트 수)로 reshape한 뒤 실제 폭만 잘라낸다.** 이게 핵심: ROS 이미지는 줄 끝에 패딩이 있을 수 있어서 width가 아니라 step으로 줄을 끊어야 한다 [78–79].
- encoding에 맞춰 `cv2.cvtColor`로 BGR 통일(rgb8이면 RGB→BGR) [87–94].

### `class VitisAiDetectorNode(Node)`

#### `__init__` [98]
> 역할: 노드 구성 전체 — 파라미터, QoS, 발행/구독, worker 프로세스, 처리 스레드를 세팅. **분기 없이 선형**이라 훑고 지나가면 된다.

순서대로:
1. `declare_parameter` **17개** [101–130] — 튜닝 가능한 값들의 선언+기본값(`process_period_sec`, `send_resized_input`, `worker_shm`, `metrics_csv_path` 등).
2. 그 값들을 `self.*`로 읽어 저장 [133–168]. ⚠️ **생성자에서 딱 한 번만 읽는다** — 런타임에 `ros2 param set`으로 바꿔도 반영되지 않고 재시작이 필요하다.
3. `ShmFrameWriter(/dev/shm/vitis_ai_frame_<pid>)` 생성 [159 부근] — 이름에 pid를 넣어 이전 실행의 잔여 파일을 재사용하지 않는다. **경로만 잡고 파일·mmap은 첫 프레임에서** 만든다.
4. `metrics_rows = [] if metrics_csv_path else None` [163] — CSV 경로가 있을 때만 프레임 계측을 메모리에 쌓는다.
5. 상태 변수·yield 카운터 초기화 [174–191] — `worker_process`, `last_worker_timing`, `frames_arrived/overwritten/gate_skipped` 등.
6. **파이프라이닝 상태** [193–199] — `frame_lock`(뮤텍스), `latest_msg`(최신 프레임), `frame_event`(새 프레임 신호), `shutdown_event`, `worker_thread`.
7. **QoS 정의**: `output_qos`=RELIABLE depth1(발행용, 놓치면 안 됨) [201], `image_qos`=BEST_EFFORT depth1(구독용, 최신 1장) [207].
8. `create_publisher(/detections)` [214].
9. `create_subscription(input_topic, self.image_callback, image_qos)` [219] — 여기서 카메라와 연결. **worker 기동보다 먼저** 만들어지지만 `spin`은 아직 시작 전이라 미완성 worker로 프레임이 갈 수 없다.
10. `detector_mode`가 `"worker"`인지 확인 [228] → `start_worker()` [235].
11. `worker_loop`를 도는 **데몬 스레드 시작** [238–241].

#### `image_callback(msg)` [255]
> 역할: 카메라 프레임이 올 때마다 실행되는 구독 콜백. **최대한 가볍게.**

`frame_lock`을 잡고 `latest_msg = msg`, `frame_event.set()` 하고 **즉시 반환**한다. 무거운 DPU 작업을 여기서 하지 않기 때문에 ROS executor가 카메라를 굶기지 않는다. 이전 프레임이 아직 안 처리됐어도 그냥 덮어써서 **항상 최신 1장만** 남긴다(latest-frame-only).

#### `worker_loop()` [266]
> 역할: 별도 스레드에서 프레임을 **연속 처리**하는 루프. DPU worker가 놀지 않게.

- `shutdown_event`가 설 때까지 반복 [260].
- `frame_event.wait(timeout=0.5)` — 새 프레임이 올 때까지 대기(0.5초마다 종료 여부 재확인) [261].
- lock 잡고 `latest_msg`를 꺼내고 비운다(집은 건 다시 안 봄) [263–266].
- `should_process_frame()` 게이트를 통과하면 `process_frame(msg)` [269–271].

#### `process_frame(msg)` [289]
> 역할: **한 프레임의 실제 처리.** 다만 160줄 중 **핵심 로직은 297–315의 19줄뿐이고, 그 아래는 전부 계측/로그**다. 처음엔 계측부를 접고 봐라.

핵심 로직 [297–315]:
- `image = ros_image_to_bgr(msg)` — ROS Image → BGR [297].
- `detections = self.detect_with_worker(image)` — worker에게 추론 시킴 [301].
- `out_msg.header = msg.header` — **입력 이미지의 헤더(타임스탬프)를 그대로 복사** [305]. 이 덕분에 나중에 검출과 원본 이미지를 stamp로 짝지을 수 있다.
- `self.publisher.publish(out_msg)` — 검출이 있거나 `publish_empty_detections`면 `/detections` 발행 [311–312].
- `frame_done_ns` [315] — 프레임 처리 종료 시각. 바로 아래 `processing_ms`(대표 성능 지표)의 기준점이다.

계측부 [317–443]:
- 시간 3층 분해: `detect_ms = img_ms + worker_call_ms`, `worker_call_ms = worker_ms + ipc_overhead_ms`, `worker_ms = pre+dpu+post` [321–345]. ⚠️ `worker_call_ms`가 **노드 실측**이고 `ipc_overhead_ms`는 거기서 worker 보고값을 뺀 **파생값**이다(인과 방향 주의).
- `metrics_csv_path`가 있으면 프레임마다 dict를 `metrics_rows`에 append [356–399], 지정 시간 지나면 CSV 저장.
- 아니면 `should_log()`일 때 한 줄 로그로 출력 [401–443].
- 예외는 `TimeoutExpired`(→ `worker_timeout_count++`)/`Exception`을 잡아 에러 로그 [448–455]. ⚠️ publish [312] **이후**에 예외가 나면 이미 발행된 프레임인데 지표 행만 없는 상태가 된다.

#### `start_worker()` [458]
> 역할: worker 서브프로세스를 띄우고 "준비됨" 핸드셰이크를 받는다.

- 이미 살아있으면 그냥 반환 [460].
- `command = [python, worker_script, --model, model_path]`(+ `--log-file`은 `worker_log_path`가 있을 때만) 구성 [463–472].
- `subprocess.Popen(..., stdin=PIPE, stdout=PIPE, stderr=PIPE)` — 파이프로 연결된 자식 프로세스 생성 [476].
- `read_worker_json("worker ready", startup_timeout)` — worker가 모델 로드(실측 ~4.2초)를 마치고 보내는 `{"status":"ready","input_shape":[1,416,416,3]}`를 기다림. 기본 타임아웃 30초 [488–495].
- `input_shape`에서 worker 입력 H/W를 뽑아 저장 [503–504].
- ⚠️ ready 검증 실패의 두 `raise` [497–501]는 위 `try` **바깥**이라 `stop_worker()`가 돌지 않는다 → `__init__`이 여기서 죽으면 살아있는 worker가 고아로 남는다(부모 종료 시 stdin EOF로 간접 종료될 뿐).

#### `stop_worker()` [510]
> 역할: worker 프로세스를 단계적으로 종료.

`process`를 지역변수로 빼고 self를 먼저 비운다(재진입 안전 — 그래야 `start_worker`의 조기 return을 통과한다). 살아있으면 **stdin close → wait 2s → terminate → wait 2s → SIGKILL** 순으로 점점 강하게 [521–535].

#### `restart_worker()` [537]
> 역할: `stop_worker()` + `start_worker()`. worker가 죽거나 파이프가 깨졌을 때 되살리는 데 쓴다.

#### `read_worker_json(label, timeout_sec)` [543]
> 역할: worker stdout에서 **JSON 한 줄**을 타임아웃과 함께 읽는다.

- `select.select([stdout], [], [], timeout)` — 지정 시간 안에 읽을 게 없으면 `TimeoutExpired` [553–556].
- `readline()`이 빈 줄이면 worker가 죽은 것 → `stderr`를 읽어 원인과 함께 예외 [558–569]. (stderr는 `returncode is not None`일 때만 붙인다.)
- 정상이면 `json.loads` [571].
- ⚠️ 이 함수는 **두 스레드에서 불린다** — 기동 핸드셰이크는 메인 스레드, 프레임 왕복은 `worker_loop` 스레드.

#### `detect_with_worker(image)` [574]
> 역할: 한 이미지를 worker에 보내고 검출 리스트를 받는 **IPC 왕복**.

- `np.ascontiguousarray(uint8)` + 3채널 확인 [576–578].
- worker가 죽었으면 `restart_worker()` [580–585]. 단 **종료 중이면 되살리지 않는다** — 새 worker 로드 ~4.5s가 launch의 5초 SIGINT 유예를 넘겨 SIGTERM 강제종료를 부른다.
- `send_resized_input`이면 worker 입력 크기로 resize(현재 YOLO는 **false**라 원본 그대로 보냄. letterbox는 worker 담당) [592–604].
- `request` dict 구성: `{height,width,channels,data_len,source_width,source_height}` [608–615].
- **shm 경로** [620–628]: `shm_writer.write(worker_image)`로 `/dev/shm`에 memcpy 1회 → `data_len=0`으로 덮고 `shm_path`를 request에 추가 → pipe엔 **JSON 헤더 한 줄만**. ⚠️ **버퍼를 요청보다 먼저 써야 한다** — worker는 요청 줄이 도착하는 즉시 버퍼를 읽는다.
- (`worker_shm:false`면 옛 경로: JSON 한 줄 + 이미지 바이트 전체를 pipe로 [629–636].)
- `flush` [637] → `read_worker_json("worker inference")`로 응답 [638].
- 파이프 깨지면 restart 후 예외 [639–650]. 응답에 error면 예외 [652–653].
- `self.last_worker_timing = response.timing` 저장, `json_items_to_detections`로 변환해 반환 [656–657].

#### `json_items_to_detections(items)` [659]
> 역할: worker가 준 dict 리스트를 ROS `Detection[]` 메시지로 변환.

각 dict의 `class_id/class_name/confidence/center_x/center_y/width/height`를 `Detection()`에 채워 리스트로 [661–671]. (header를 붙인 `DetectionArray` 조립은 호출부인 `process_frame`이 한다.)

> **삭제됨 (2026-09-03)**: `prepare_overlay` / `draw_detections` / `make_overlay_msg` / `make_compressed_overlay_msg` 4개와 `BOX_COLORS`, 그리고 `publish_overlay` 계열 파라미터 6개가 제거됐다. 보드에서 박스를 그리면 프레임당 ~44 ms가 검출(~37.6 ms) 위에 얹혀 예산을 넘겼고, 그리기는 데스크톱(`detection_viewer_pkg`, `raon_operator_gui`)이 맡는 것으로 확정됐기 때문이다. 보드는 검출과 압축 스트림만 내보낸다.

#### `should_process_frame()` [674]
> 역할: 검출 속도를 제한하는 **최소 간격 게이트**(divider 아님).

`process_period_sec<=0`이면 항상 통과 [678–679]. 아니면 마지막 처리 후 경과가 `process_period_sec` 미만이면 `False`(**프레임 폐기, 재시도 없음**) [688–690]. 통과 시에만 타임스탬프 갱신 [693–695] — 기준은 "수락된 **처리 시작** 시각"이다. → 30fps여도 45ms 게이트면 2프레임에 1번(15Hz). 현재 값 0.030은 처리(~43 ms)보다 짧아 실질적으로 바인딩되지 않는 backstop이다.

#### `should_log()` [697]
> 역할: 로그를 `log_period_sec`(기본 1초)마다 한 번만 내보내는 게이트. `should_process_frame`과 같은 패턴.

#### `frame_age_ms(msg)` [716]
> 역할: 이 프레임이 **캡처된 뒤 지금까지 얼마나 지났나**(ms). `현재시각 - header.stamp`. stamp가 0이거나 음수 age면 `None`. latency 측정의 근거.

#### `_round(value, ndigits=3)` [728]
> 역할: (staticmethod) CSV용 반올림 유틸 — 값이 `None`이면 빈 칸.

#### `stop_pipeline()` [734]
> 역할: 종료 시 **worker 스레드부터** 멈춘다. `shutdown_event.set()` + `frame_event.set()`(대기 중이면 깨움) + `worker_thread.join()`. metrics_rows에 더 이상 append가 안 되게 하는 게 목적.

#### `write_yield_summary()` [744]
> 역할: 종료 시 **신뢰성(yield) 요약**을 CSV로 한 번 쓴다. 프레임 예산 흐름(`frames_arrived → overwritten → gate_skipped → processed → empty_out`)과 worker 재시작/타임아웃/지연(`slow_worker_ms` 초과) 횟수. 하드 타임아웃이 나기 전에 미리 잡히는 선행 지표다.

#### `save_metrics_csv()` [779]
> 역할: 모아둔 `metrics_rows`를 지정 CSV로 저장. 경로/행이 없으면 아무것도 안 함.

**헤더를 행에서 뽑는다** — `fieldnames = list(metrics_rows[0].keys())`. 예전엔 하드코딩한 목록이 새 키를 빠뜨려 `DictWriter`가 예외를 냈고 헤더만 있는 파일이 남았다. 쓰기는 **임시 파일 + `os.replace`로 원자적**인데, 종료 중 두 번째 SIGINT가 오면 그냥 `"w"`로는 CSV가 잘리기 때문이다(2026-07-31 실측: 3853행 → 611행).

### `main(args)` [816]
> 역할: 진입점. `rclpy.init` → 노드 생성 → `rclpy.spin`.

특이점: **SIGTERM을 `KeyboardInterrupt`로 바꾸는 핸들러**를 건다 [823–826]. `pkill`로 종료해도 Ctrl-C처럼 `finally`가 돌아서 metrics CSV가 저장되고 worker가 정리된다.

`finally`의 **순서가 의미를 가진다** [838–850]: SIGINT/SIGTERM을 `SIG_IGN`으로 막고 → `stop_pipeline` → `save_metrics_csv` → `write_yield_summary` → `stop_worker` → `shm_writer.unlink` → `destroy_node` → `rclpy.shutdown`. unlink가 `stop_worker` **뒤**인 이유: 실행 중에 unlink하고 다시 만들면 새 inode가 생겨 worker가 죽은 inode의 낡은 프레임을 조용히 읽는다.

⚠️ 단, 핸들러는 `node = ...` 대입 **전**에 걸리므로 그 틈에 SIGTERM이 오면 `node is None`이라 위 정리가 **전부 스킵**된다.

---

## 2. `vitis_ai_worker_yolo.py` — 코드 순서대로

### 모듈 docstring + import + 상수

- **docstring [2–17]**: 이 파일이 뭔지와 핵심 3가지(letterbox 416, yolov5 grid decode, objectness pre-filter)를 요약. **제일 먼저 읽을 것.**
- **import [19–29]**: `xir`, `vart`(Vitis-AI). `rclpy` 없음 — ROS를 모르는 프로세스.
- **상수 [39–48]**: `DEFAULT_THRESHOLD=0.50`, `NMS_THRESHOLD=0.45`, `KEEP_TOP_K=100`, `PAD_VALUE=114`(letterbox 회색).
  - `CLASS_THRESHOLDS_BY_NAME = {"person": 2.0}` [40–45] — ⚠️ **2.0은 도달 불가능한 임계값이라 person은 사실상 검출 OFF**다(conf는 최대 1.0). 예전 문서의 `person: 0.30`(recall 우선)은 옛 값이다. 끄려고 주석 처리하면 `DEFAULT_THRESHOLD=0.50`으로 폴백해 되살아나므로 **반드시 2.0으로 둘 것**.

### `get_dpu_subgraphs(graph)` [53]
> 역할: xmodel 그래프에서 **DPU에서 도는 subgraph**만 골라낸다.

루트 subgraph를 위상정렬로 자식들로 펼친 뒤, `device` 속성이 `"DPU"`인 것만 필터 [54–60]. (컴파일이 잘 됐으면 DPU subgraph는 1개.)

### `tensor_shape(tensor)` [62]
> 역할: 텐서의 shape을 튜플로. `dims` 또는 `get_shape()` 중 있는 걸 씀(버전 호환).

### `tensor_fix_point(tensor)` [70]
> 역할: 텐서의 **fix_point**(양자화 소수점 위치)를 반환, 없으면 0. → 실제값 = int8값 / `2^fix_point`.

### `sigmoid(x)` [77]
> 역할: 시그모이드. `exp` 오버플로 방지로 입력을 `[-30, 30]`으로 clip.

### `iou(box, boxes)` [81]
> 역할: 한 박스와 여러 박스의 **IoU(겹침 비율)를 벡터로** 한 번에 계산. NMS의 부품.

교집합 넓이 / (합집합 넓이). 분모에 `1e-9`를 더해 0 나눗셈 방지 [98].

### `nms(boxes, scores, threshold)` [100]
> 역할: **Non-Max Suppression** — 겹치는 박스 중 점수 높은 것만 남긴다.

점수 내림차순 정렬 [103] → 맨 앞을 keep에 넣고 [106] → 그것과 IoU가 `threshold` 초과인 나머지를 버림 [112] → 반복. keep된 인덱스 리스트 반환.

빈 입력은 즉시 `[]`, 그리고 **남은 후보가 1개면 `iou`를 부르지 않고 break** [108–109] — 후보가 하나뿐이면 `iou` 호출 0회다.

### `build_input_lut(input_fix)` [116]
> 역할: **전처리 LUT** — uint8 픽셀값(0~255) → int8 양자화 입력의 256칸 표.

`round(v/255 * 2^fix)`를 `[-128,127]`로 clip [119–121]. 매 프레임 계산 대신 표 조회로 대체하려는 것. (`v/255`는 정규화, `*2^fix`는 양자화.)

### `letterbox_lut(image, input_size, lut, out_buf, state)` [123]
> 역할: **letterbox + BGR→RGB + LUT 양자화를 한 번에** 해서 `out_buf`(입력 텐서)에 기록. 좌표 역변환용 `(ratio, pad_x, pad_y)` 반환.

- `r = min(416/h, 416/w)` — 비율 유지 축소율 [134]. `nh, nw` = 축소 후 크기. **848×480이면 r=0.4906, nh=235 / nw=416, top=90 / left=0.**
- 필요하면 `cv2.resize` [138–139].
- `top/left` = 가운데 정렬 pad 위치 [145–146].
- pad(회색)는 `(nh,nw)`가 바뀔 때만 다시 칠함 — 고정 해상도면 최초 1회뿐(콘텐츠 영역은 매 프레임 덮어써지니 안전) [148–152].
- `cv2.cvtColor(BGR2RGB)` [154] 후 `cv2.LUT`로 int8 매핑해 out_buf 중앙에 넣음 [157–158]. (`cv2.LUT(x,lut)==lut[x]`. int8 lut를 거부할 수 있어 uint8 뷰로 조회 후 int8로 재해석한다.)

### `decode_head(raw, scale, anchors_np, stride, num_classes, thr_q)` [162]
> 역할: DPU 출력 한 scale(격자)을 **박스+conf로 디코드**. yolov5식.

- 출력을 `(H,W,na,no)`로 reshape [180] (na=anchor 수, no=5+클래스). 채널 수가 안 맞으면 RuntimeError [178–179].
- **objectness pre-filter** [182–187]: int8 raw 그대로 `obj_q >= thr_q`인 칸만 `np.nonzero`로 고른다. 대부분 배경이라 여기서 걸러 sigmoid/decode 비용을 아낀다. (`conf=sig(obj)*sig(cls) ≤ sig(obj)`라 안전한 필요조건.) 하나도 없으면 빈 배열 조기반환.
- 고른 것만 `/scale`로 역양자화 후 `sigmoid` [189–190].
- yolov5 decode [193–196]: `cx=(sig_x*2-0.5+격자x)*stride`, `bw=(sig_w*2)^2*anchor`. anchor는 **픽셀 단위**라 stride를 곱하지 않는다.
- `boxes`(xyxy) + `conf=sig(obj)*sig(cls)` 반환 [198–199].

### `read_exact(stream, size)` [202]
> 역할: 스트림에서 **정확히 `size` 바이트**를 읽는다(부족하면 반복, 끊기면 `None`). **shm 이전의 폴백 경로 전용** — `worker_shm:true`면 node가 `data_len=0`으로 보내므로 호출되지 않는다.

### `class YoloWorker`

#### `__init__(model_path, meta_path, log_file)` [213]
> 역할: **`decode_meta.json`을 읽어** 모델 스펙을 세팅하고 `load_model` 호출.

- meta에서 `names`, `num_classes`, `input_size`, `strides`, `anchors_pixel` 로드 [221–226]. **← 클래스 수·이름의 정본은 파일명이 아니라 여기.** 실제 6클래스: apple / orange / banana / tennis_ball / mustard_bottle / person, `input_size=416`, `strides=[16,32]`.
- 클래스별 threshold 배열 구성 [227–230] — person만 2.0(=OFF), 나머지 `DEFAULT_THRESHOLD` 0.50.
- `min_obj_logit` = 최소 threshold의 logit − ε — pre-filter 임계값의 원본 [231–233]. person이 2.0이라 **최솟값은 person이 아니라 0.50**이다.
- `self.load_model()` [235].

#### `log(message)` [237]
> 역할: `log_file`이 설정됐을 때만 파일에 append(평소 off, 파일 I/O 비용 제거).

#### `load_model()` [244]
> 역할: **1회 셋업** — "프레임마다 하면 느린 것"을 전부 여기 모아 미리 한다. 그래서 `detect`가 가볍다.

- `xir.Graph.deserialize` → `get_dpu_subgraphs` → `vart.Runner.create_runner` [248–253]. **graph/subgraph를 `self`에 붙들어 둔다** — 해제되면 execute_async가 segfault(§0의 격리 이유) [246–247 주석].
- 입력 텐서 shape/fix_point 확인, meta의 input_size와 일치 검사 [257–264].
- `build_input_lut` + 입력 버퍼 `input_buf`(1,416,416,3 int8) 준비 [266–270].
- **프레임 불변 상수 미리 계산** [273–289]: 각 출력 head의 `(index, scale=2^fix, anchors 배열, stride, thr_q=min_obj_logit*scale)`를 `head_params`에 저장. `stride`는 meta가 아니라 **텐서 shape에서 역산**(`input_size // H`)하므로 런타임 텐서 순서가 뒤바뀌어도 결과는 같다.
- 출력 버퍼 `output_data`(int8) 준비 [291–294]. 실측 텐서: 입력 fix=6, 출력 `(1,26,26,33)` fix=2 / `(1,13,13,33)` fix=3.
- **여기가 기동 시간의 대부분**이다(실측 ~4.2초). node의 `start_worker`가 30초 타임아웃으로 이걸 기다린다.

#### `detect(image, output_width, output_height)` [297]
> 역할: **한 프레임 처리의 심장.** §0-B의 흐름 그대로 + 단계별 timing.

- `letterbox_lut(...)` → `(ratio, pad_x, pad_y)`, 소요 `pre_ms` [305–308].
- `runner.execute_async([input_buf], output_data)` + `runner.wait(job_id)` — **DPU 추론**, `dpu_ms` [310–313]. 이름은 async지만 submit 직후 바로 wait하므로 **사실상 동기**이고, 전처리와 추론이 겹치지 않는다.
- 각 head를 `decode_head`로 디코드해 boxes/conf를 concat [316–322].
- `make_detections(...)`로 최종 검출, `post_ms` [324–327].
- `last_timing`에 pre/dpu/post/worker_ms 기록 후 반환 [329–335]. **node CSV의 `pre_ms`/`dpu_ms`/`post_ms`는 전부 여기가 유일한 출처**다.

#### `make_detections(boxes, conf, ratio, pad_x, pad_y, image_w, image_h, output_width, output_height)` [337]
> 역할: 디코드된 후보를 **클래스별 NMS + letterbox 역변환**으로 최종 정리해 dict 리스트로.

- `sx, sy` = 출력 해상도 스케일(현재 `send_resized_input:false`라 image==source → 1.0) [341–342].
- 클래스마다: threshold 넘는 것 mask [348–350] → 후보 0이면 nms 없이 continue [351–352] → `nms` [355] → keep된 박스마다 **letterbox 역변환** `x=(cb-pad)/ratio*s` [356–359], 화면 밖 클램프 [360–363], w나 h가 1px 이하면 skip [364–366].
- `{class_id, class_name, confidence, center_x, center_y, width, height}` dict 생성(좌표는 **원본 픽셀** 기준) [367–375].
- confidence 내림차순 정렬 후 상위 `KEEP_TOP_K`개 반환 [378–379].
- ⚠️ NMS가 **클래스별로 독립**이라 서로 다른 클래스끼리 겹치는 박스는 억제되지 않는다.

### `write_response(response)` [380]
> 역할: 응답 dict를 **JSON 한 줄 + 개행**으로 stdout에 쓰고 flush. node의 `read_worker_json`이 이걸 받는다.

### `main()` [385]
> 역할: worker 진입점 — 인자 파싱 → 모델 로드 → **stdin/stdout 무한 루프**.

- 인자: `--model`(필수), `--meta`(기본은 model 옆 `decode_meta.json`), `--log-file` [386–391]. (`--softmax`는 SSD worker 호환용이었고 2026-09-03 제거됐다.)
- meta가 없으면 stdout에 error JSON을 쓰고 `sys.exit(1)` [395–402] → node는 그걸 ready 자리에서 읽어 RuntimeError를 낸다.
- `YoloWorker` 생성 후 `{"status":"ready","input_shape":[1,416,416,3]}` 전송 + **flush** [405–411]. flush가 프로토콜의 일부다 — 빠뜨리면 node가 30초 타임아웃.
- `ShmFrameReader()` 생성 [413] — 아무 파일도 열지 않는 껍데기라, node가 첫 프레임을 쓰기 전에 worker가 떠도 무해하다.
- 루프 [414–463]: stdin `readline` [415] → 빈 바이트면(=node가 stdin을 닫음) **break로 프로세스 종료** → JSON 파싱 → `shm_reader.view(...)`로 **복사 없이** `(h,w,c)` 뷰 획득 [430–432] (`shm_path`가 없을 때만 `read_exact` 폴백 [434]) → `worker.detect(output=source 크기)` [443] → `write_response({detections,timing,error})` [448–453].
- 프레임 처리 중 **어떤 예외든** `{detections:[], error:str}`로 응답하고 **루프는 유지**한다 [455–461] — worker 프로세스는 살아남는다. 단 이 응답에는 `timing` 키가 없다.

---

## 부록: 1분 용어 사전

- **fix_point / int8 양자화**: DPU는 float 대신 int8로 계산. 실제값 = int8 / `2^fix`. 입력은 곱해서 넣고(`build_input_lut`), 출력은 나눠서 되돌린다(`decode_head`).
- **LUT**: 픽셀값이 256가지뿐이라 변환을 표로 미리 만들어 조회만. A53 CPU가 약해 크게 빠름.
- **letterbox**: 848×480을 416×416에 비율 유지로 넣고 남는 자리는 회색(114) pad. 그래서 마지막에 `(x-pad)/ratio`로 원본 좌표 복원.
- **grid decode**: YOLO 출력은 격자 칸마다 objectness+위치+크기+클래스. yolov5 공식으로 실제 박스로 푼다.
- **NMS**: 한 물체에 겹친 박스를 점수순으로 하나만 남김.
- **QoS depth=1 BEST_EFFORT**: 카메라 구독은 "최신 1장, 놓쳐도 됨" → latency 최소.
- **프로세스 격리**: VART segfault 회피용으로 worker를 별도 프로세스로.
- **`/dev/shm` mmap IPC**: 프레임 본체는 pipe가 아니라 공유메모리 파일로 넘긴다. node가 memcpy 1회로 쓰고, worker는 mmap 뷰로 **복사 없이** 읽는다. pipe로 전부 밀던 시절 대비 ipc 13.4 → 4.65 ms.
- **yield(신뢰성) 지표**: 성능(ms)과 별개로 "프레임이 어디서 몇 장 새는가"(도착 → 덮어씀 → 게이트 스킵 → 처리 → 빈 결과)와 worker 재시작/타임아웃 횟수. `write_yield_summary`가 종료 시 한 번 쓴다.

> 관련 정본: 파라미터 값의 근거 [`workflow.md`](workflow.md) · 최적화 측정 델타 [`vision_final.md`](vision_final.md) · 전체 상태 [`../STATUS.md`](../STATUS.md).
