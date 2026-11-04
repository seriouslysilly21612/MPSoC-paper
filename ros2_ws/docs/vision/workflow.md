# Pick & Place Perception Workflow (KV260)

Kria KV260(APU: Ubuntu 22.04 + ROS2 Humble) 위에서 동작하는 **카메라 → 비전 추론 → 3D 목표 좌표** 파이프라인의 전체 문서입니다.

- **이 좌표의 용도**: Indy7 TCP가 테이블 위 물체 위까지 **접근**하고, 사람이 그 물체를 옮기면 **추종(tracking)** 한다(2026-08-03 전환). 파지·이송·놓기는 **범위 밖** — 그리퍼 하드웨어 미보유. 제목·launch·토픽 이름의 "pick"은 초기 pick&place 프레이밍의 legacy다. 제어 쪽 정본은 `docs/RAON-RT/merge.md`(D15).
- **하드웨어**: Kria KV260 + Intel RealSense D435i
- **가속기**: `kv260-smartcam` 오버레이의 DPU (`DPUCZDX8G_ISA1_B3136`)
- **현재 모델**: `yolov3_tiny_7class.xmodel` — **실제는 6클래스**(apple / orange / banana / tennis_ball / mustard_bottle / person). 파일명의 "7class"는 초기 잔재이고 정본은 `decode_meta.json`. person은 threshold 2.0으로 **검출 OFF**.
  - 초기 검증용 stand-in이던 `ssd_adas_pruned_0_95`와 그 worker(`vitis_ai_worker.py`)는 **2026-09-03 코드째 제거**됐다. 아래 §2.1·§2.2의 측정은 그 SSD 시절 기록이지만, 거기서 나온 기법(LUT·사전필터)은 YOLO worker에 그대로 계승됐다.
- **통합 실행**: `ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py`

문서 구성:
1. [노드별 파이프라인](#1-노드별-파이프라인) — 각 노드의 파일 경로 / 입력·출력 / 내부 동작 / 핵심 파라미터와 그 값의 이유
2. [성능 향상 기법 (전처리·후처리)](#2-성능-향상-기법-전처리후처리)
3. [파이프라인 아키텍처](#3-파이프라인-아키텍처)

---

## 0. 전체 개요

### 데이터 흐름

```
[RealSense D435i]
   /camera/camera/color/image_raw          (sensor_msgs/Image, RGB8 848x480)
   /camera/camera/depth/image_rect_raw      (sensor_msgs/Image, 16UC1 848x480, raw=unaligned)
   /camera/camera/{depth,color}/camera_info (intrinsics)
   /camera/camera/extrinsics/depth_to_color (realsense2_camera_msgs/Extrinsics)
        │ color image
        ▼
[vitis_ai_detector_node] ─ JSON(pipe)+프레임(/dev/shm) ▶ [vitis_ai_worker_yolo.py (DPU 추론)]
   → /detections                          (my_interfaces/DetectionArray)
        │
        ▼
[pick_logic_node]
   → /pick_target                         (my_interfaces/PickTarget)   2D 필터 통과 1개
        │                + raw depth + intrinsics + extrinsics
        ▼
[pick_target_3d_node]   (single-point reverse projection)
   → /pick_target_3d                      (my_interfaces/PickTarget3D, frame=camera_depth_optical_frame)
        │   + TF (base_link → camera_link → camera_depth_optical_frame)
        ▼
[pick_target_base_node]
   → /pick_target_base                    (my_interfaces/PickTarget3D, frame=base_link)
```

### 노드 ↔ 파일 ↔ 설정 매핑

| 노드 | 패키지 / 파일 | 설정 파일 |
|---|---|---|
| `camera` (realsense2_camera) | third-party | `src/system_bringup_pkg/config/realsense_pick_place.yaml` |
| `vitis_ai_detector_node` | `src/vitis_ai_detector_pkg/vitis_ai_detector_pkg/vitis_ai_detector_node.py` | `src/system_bringup_pkg/config/vitis_ai_detector.yaml` |
| (worker 프로세스) | `src/vitis_ai_detector_pkg/vitis_ai_detector_pkg/vitis_ai_worker_yolo.py` | (위 yaml의 `worker_*`) |
| `pick_logic_node` | `src/pick_logic_pkg/pick_logic_pkg/pick_logic.py` | `src/system_bringup_pkg/config/pick_logic.yaml` |
| `pick_target_3d_node` | `src/target_3d_pkg/target_3d_pkg/pick_target_3d_node.py` | `src/system_bringup_pkg/config/target_3d.yaml` |
| `base_to_camera_tf` | `tf2_ros/static_transform_publisher` | launch 파일 내 인자 |
| `pick_target_base_node` | `src/target_3d_pkg/target_3d_pkg/pick_target_base_node.py` | `src/system_bringup_pkg/config/target_base.yaml` |
| 통합 launch | `src/system_bringup_pkg/launch/pick_place_vitis_ai.launch.py` | — |

---

## 1. 노드별 파이프라인

### 1.1 RealSense 카메라 (`realsense2_camera`)

- **역할**: 센서에서 color/depth 스트림과 카메라 파라미터(intrinsics/extrinsics)를 ROS 토픽으로 발행.
- **출력 토픽**
  - `/camera/camera/color/image_raw` — **RGB8**, 848×480 (yaml에 `color_format` 키가 없어 `rs_launch.py` 기본값. 노드는 런타임 `msg.encoding`으로 분기한다)
  - `/camera/camera/depth/image_rect_raw` — 16UC1, 848×480, **raw(정렬 안 됨)**, scale 0.001 m/unit
  - `/camera/camera/{depth,color}/camera_info` — `k`(fx,fy,cx,cy), 왜곡계수 `d`(전부 0 = rectified)
  - `/camera/camera/extrinsics/depth_to_color` — depth↔color 회전·평행이동 (baseline ≈ 15 mm)
  - `/tf_static` — `camera_link → camera_depth_optical_frame`, `→ camera_color_optical_frame`

**핵심 파라미터** (`realsense_pick_place.yaml`)

| 파라미터 | 값 | 이유 |
|---|---|---|
| `rgb_camera.color_profile` | `848x480x30` | 1280×720이면 realsense 노드가 100% busy-spin에 빠져 약 70초 후 프레임 공급이 멈추는(전체 파이프라인 silent stall) 현상이 있었음. detector가 어차피 480×360으로 resize하므로 해상도를 낮춰도 **검출 품질 영향 없음**. 둘 다 16:9라 resize aspect 동일. USB/CPU 부하 ~2.3배 감소. |
| `depth_module.depth_profile` | `848x480x30` | depth도 동일 해상도. |
| `align_depth.enable` | **`false`** | full-frame depth→color 정렬은 realsense 노드의 단일 스레드가 A53 코어 하나를 100% 점유하게 만들어 검출 throughput을 절반으로 떨굼. 우리는 bbox 중앙 1점의 depth만 필요하므로 **정렬을 끄고 `pick_target_3d_node`에서 점 1개만 reverse projection** 함 (§3.5). |
| `enable_sync` | `false` | color/depth를 독립 처리 (정렬 안 하므로 sync 불필요). |
| `enable_gyro/accel/infra` | `false` | pick-place에 불필요한 스트림 차단으로 부하 절감. |

> 참고: realsense는 **aligned 토픽을 누가 구독할 때만** 정렬을 계산한다. 과거 "카메라 단독 25Hz" 측정이 오도했던 이유 — 단독에는 구독자가 없어 정렬이 안 돌았기 때문.

---

### 1.2 비전 추론: `vitis_ai_detector_node` (+ `vitis_ai_worker_yolo.py`)

비전 추론은 **두 프로세스**로 나뉜다: ROS 래퍼 노드와 DPU 추론 worker. 분리 이유는 §3.1 참고.

#### (A) `vitis_ai_detector_node.py` — ROS 래퍼

- **입력**: `/camera/camera/color/image_raw` (`sensor_msgs/Image`)
- **출력**: `/detections` (`my_interfaces/DetectionArray`). **보드 overlay 발행은 없다** — 2026-09-03에 코드째 제거했고 박스는 데스크톱(`detection_viewer_pkg`, `raon_operator_gui`)이 그린다.
- **내부 동작**
  1. `image_callback`: 들어온 최신 프레임만 보관(경량, 즉시 반환) — §3.3 파이프라이닝
  2. `worker_loop`(전용 스레드) → `process_frame`:
     - `ros_image_to_bgr(msg)`로 ROS Image → numpy BGR 변환
     - `detect_with_worker(image)`: 프레임을 `/dev/shm`에 memcpy 1회로 쓰고 → worker stdin에 **JSON 헤더 한 줄만**(`shm_path` 포함) 전송 → worker stdout에서 detection JSON + timing 수신. resize는 안 한다(letterbox는 worker 담당).
     - JSON → `Detection[]` 변환, `DetectionArray`로 `/detections` 발행(header는 입력 이미지 것을 그대로 승계)
     - (옵션) per-frame metric을 메모리에 적재(CSV)

**핵심 파라미터** (`vitis_ai_detector.yaml`)

| 파라미터 | 값 | 이유 |
|---|---|---|
| `detector_mode` | `worker` | in-process VART는 `execute_async()`에서 segfault → **별도 worker 프로세스**로 격리 (§3.1). `oneshot`은 fallback. |
| `send_resized_input` | `false` | **YOLO worker가 직접 letterbox**(비율 유지 416 + pad)하므로 노드는 원본을 그대로 넘겨야 한다. SSD 시절엔 `true`로 두고 노드가 480×360으로 줄여 pipe payload를 81% 줄였는데, shm 전환으로 그 이유도 사라졌다. |
| `worker_shm` | `true` | 프레임 본체를 pipe 대신 `/dev/shm` mmap으로 전달(memcpy 1회). ipc 13.4 → **4.65 ms**, throughput 18.4 → 21.7 Hz (2026-08-05). `false`면 옛 pipe 경로로 폴백. |
| `process_period_sec` | `0.030` | 두 검출 **시작** 사이의 최소 간격. 처리가 ~43 ms라 이 값은 실질적으로 바인딩되지 않는 backstop이다. 0.045였을 때가 의도된 15 Hz 상한이었고, 이동 물체로 전환하며 풀었다. |
| `publish_empty_detections` | `true` | 검출 0개여도 매 프레임 빈 배열 발행 → 파이프라인 heartbeat 유지 + `topic hz`가 실제 처리율을 반영(측정 편의). 운영에서 트래픽을 줄이려면 `false`. |
| `worker_log_path` | `""` | worker가 진단 로그를 프레임당 ~18회 file open/write/flush 하던 것을 제거(SD/eMMC I/O). 운영 off. |
| `worker_startup_timeout_sec` / `timeout_sec` | `30.0` / `10.0` | worker는 XIR/VART 초기화에 수 초 걸려 **기동 타임아웃을 길게**(30s), 프레임 추론 타임아웃은 짧게(10s). |
| `metrics_csv_path` | `""` | 프로파일링 전용(운영 off). 경로를 주면 매 프레임 timing을 모아 종료 시 CSV 저장. |
| `model_path` | (yaml에 없음) | launch가 `FindPackageShare`로 `vitis_ai_detector_pkg/models/yolov3_tiny_7class.xmodel`을 주입한다. yaml에 적으면 그 주입을 덮어쓴다. |

#### (B) `vitis_ai_worker_yolo.py` — DPU 추론 프로세스 (rclpy 없음)

> 줄 단위 정독은 [`detector_worker_walkthrough.md`](detector_worker_walkthrough.md) §2. 여기는 계약과 상수만.

- **입력(stdin)**: JSON 헤더 한 줄 — `{height,width,channels,data_len,source_width,source_height,shm_path}`. **프레임 본체는 `/dev/shm`** 에 있고 worker는 mmap 뷰로 **복사 없이** 읽는다. (`shm_path`가 없으면 `data_len` 바이트를 stdin에서 읽는 옛 폴백.)
- **출력(stdout)**: `{"detections":[...], "timing":{pre_ms,dpu_ms,post_ms,worker_ms}, "error":null}` + 개행 + flush. **개행 1개 = JSON 1개**가 프레이밍이다.
- **수명주기**: 기동 시 1회만 — `decode_meta.json` 로드 → `xir.Graph.deserialize` → DPU subgraph → `vart.Runner.create_runner` → LUT/입출력 버퍼/`head_params`(anchor·stride·thr_q) 준비 → `{"status":"ready", "input_shape":[1,416,416,3]}` 전송. 실측 ~4.2초. 이후 프레임마다 재사용.
- **프레임 처리** `detect()`:
  1. `letterbox_lut()` — 비율 유지 416 letterbox + pad + BGR→RGB + LUT int8 양자화를 **한 번에** (§2.1의 LUT 기법 계승)
  2. `runner.execute_async([input]) → runner.wait()` — DPU 추론(실측 중앙값 **17.4 ms**). 이름은 async지만 submit 직후 wait이라 사실상 동기.
  3. `decode_head()` × 2 head — **objectness 사전필터**(int8 raw 상태로 비교, §2.2의 아이디어 계승) → 통과분만 sigmoid + yolov5 grid decode
  4. `make_detections()` — 클래스별 NMS + letterbox 역변환 → detection 리스트(원본 source 좌표) 반환

**핵심 상수** (`vitis_ai_worker_yolo.py` 상단 + `decode_meta.json`)

| 상수 | 값 | 의미 |
|---|---|---|
| `input_size` (meta) | `416` | YOLOv3-tiny 입력 한 변. letterbox로 비율 유지해 채운다. |
| `strides` (meta) | `[16, 32]` | 출력 head 2개의 격자 간격 → 26×26, 13×13. |
| `names` (meta) | 6개 | apple / orange / banana / tennis_ball / mustard_bottle / person. **클래스 정본은 파일명이 아니라 이 meta.** |
| `DEFAULT_THRESHOLD` | `0.50` | 클래스별 지정이 없을 때의 confidence 임계값. |
| `CLASS_THRESHOLDS_BY_NAME` | `{person: 2.0}` | ⚠️ conf 최대가 1.0이라 **2.0 = person 검출 OFF**. 주석 처리하면 0.50으로 폴백해 되살아나므로 반드시 2.0으로 둘 것. |
| `NMS_THRESHOLD` | `0.45` | 중복 박스 억제 IoU(YOLOv5 기본값). **클래스별 독립**이라 다른 클래스끼리 겹치는 건 억제 안 됨. |
| `KEEP_TOP_K` | `100` | confidence 내림차순 상위 N개만 반환. |
| `PAD_VALUE` | `114` | letterbox 여백 회색(YOLOv5 기본). |

> 핵심 버그 수정: VART runner를 만든 뒤 XIR graph/subgraph 파이썬 객체가 GC되면 이후 `execute_async()`가 segfault → **`self.graph`/`self.dpu_subgraphs`를 runner 수명 동안 보관**해서 해결.

---

### 1.3 `pick_logic_node` — 2D 필터링

- **파일**: `src/pick_logic_pkg/pick_logic_pkg/pick_logic.py`
- **입력**: `/detections` (`DetectionArray`)
- **출력**: `/pick_target` (`PickTarget` = `target_valid` + Detection 필드)
- **내부 동작(v3, 2026-09-04)**: 오퍼레이터가 지정한 클래스(`desired_class`, LIVE 파라미터)의 **인스턴스 1개**를 골라 따라간다. `detections_callback`이 ① `desired_class`가 비어 있으면 전부 invalid(`no_class_selected`) ② 배열을 `is_detection_acceptable()`로 걸러 그 클래스 후보만 남김 ③ 트랙이 없으면 confidence 최대 후보 채택, 있으면 최근접 후보로 이어받기(`_update_track`) ④ `stability_min_frames` 연속 보인 뒤부터 `target_valid=true`. 채택된 인스턴스는 **sticky** — 더 좋은 후보가 나와도 갈아타지 않고 `lost_timeout_frames` 연속 사라져야 해제. 빈 배열·전부 reject·워밍업 중이면 `target_valid=false`(이유 로그). v1은 "처음 통과한 1개", v2(2026-07-26)는 conf/중앙/면적 점수식 + hysteresis 전환이었고, 앱이 접근+추종으로 바뀌며 v3에서 점수식과 AUTO 모드를 삭제했다(근거는 `pick_logic.py` `_update_track` docstring).
- **필터 순서(L1)**: confidence → allowed class → bbox 크기>0 → **edge margin** → bbox 면적(min/max) → desired_class 일치.

**핵심 파라미터** (`pick_logic.yaml`)

| 파라미터 | 값 | 이유 |
|---|---|---|
| `min_confidence` | `0.3` | 너무 약한 검출 제거. worker 쪽 클래스 임계값(0.50)이 이미 한 번 걸러 주므로 여기는 느슨하다. |
| `allowed_classes` | `[apple, orange, banana, tennis_ball, mustard_bottle]` | pick 대상 5종. 모델의 6번째 클래스 person은 여기에도 없고 worker에서도 threshold 2.0으로 꺼져 있다. |
| `image_width / image_height` | `848 / 480` | **카메라 color 해상도와 반드시 일치** — detector가 bbox를 source(848×480) 좌표로 주기 때문에 edge/area 필터가 이 값을 기준으로 함. |
| `edge_margin_px` | `30` | 프레임 가장자리에 걸친 박스는 잘려서 중심·depth가 부정확 → 거부. |
| `min_bbox_area_px` | `400.0` | 너무 작은 박스(노이즈) 거부. |
| `max_bbox_area_ratio` | `0.5` | 화면의 절반 넘는 비정상적으로 큰 박스 거부. |
| `desired_class` | `""` | **필수** LIVE 파라미터. 로봇 앱 'p' 메뉴가 `ros2 param set`으로 세팅. 비면 전부 invalid. 로봇 앱 없이 단독 측정할 땐 launch 인자 `desired_class:=apple`. |
| `stability_min_frames` | `3` | 채택 인스턴스가 이만큼 연속 보여야 valid(1프레임 오검출 방어). |
| `stability_match_px` | `60.0` | 이어받기 거리 게이트. **같은 클래스가 2개 이상 보일 때만** 적용 — 하나뿐이면 가림 중 이동해도 즉시 재획득. |
| `lost_timeout_frames` | `5` | 이만큼 연속 사라지면 트랙 해제 후 재채택. |

---

### 1.4 `pick_target_3d_node` — 단일 점 reverse projection 3D

- **파일**: `src/target_3d_pkg/target_3d_pkg/pick_target_3d_node.py`
- **입력**:
  - `/pick_target` (`PickTarget`) — bbox 중앙 = color 픽셀
  - `/camera/camera/depth/image_rect_raw` (16UC1 raw depth)
  - `/camera/camera/depth/camera_info`, `/camera/camera/color/camera_info` (intrinsics)
  - `/camera/camera/extrinsics/depth_to_color` (extrinsics, transient_local QoS)
- **출력**: `/pick_target_3d` (`PickTarget3D`, `header.frame_id = camera_depth_optical_frame`)
- **내부 동작** (`pick_target_callback`):
  1. bbox 중앙 color 픽셀 `(u_c, v_c)`에 대해 `color_pixel_to_depth_pixel()`로 **대응 depth 픽셀** 탐색 (rs2_project_color_pixel_to_depth_pixel = epipolar 선분 탐색, §3.5)
  2. 찾은 depth 픽셀 주변 patch median으로 z(m) 계산
  3. depth intrinsics로 `deproject` → depth optical frame에서 `(x, y, z)` → 발행
- **degrade**: depth/intrinsics/extrinsics 미준비, 매칭 실패, 범위 밖 depth → `depth_valid=false`로 발행(경고 throttle).

**핵심 파라미터** (`target_3d.yaml`)

| 파라미터 | 값 | 이유 |
|---|---|---|
| `depth_topic` | `.../depth/image_rect_raw` | **raw(정렬 안 된) depth** — align을 껐으므로. |
| `extrinsics_topic` | `.../extrinsics/depth_to_color` | color↔depth 대응을 풀기 위한 외부 파라미터. |
| `depth_scale_16uc1` | `0.001` | 16UC1 1단위 = 1 mm (D435i). |
| `patch_radius` | `4` | 매칭 depth 픽셀 주변 9×9 median → 노이즈/구멍에 강건. |
| `min_depth_m / max_depth_m` | `0.19 / 1.6` | 유효 depth 범위 = reverse projection 탐색 선분의 깊이 범위이기도 함. 하한은 D435i 물리 min-z(~0.195m). 상한은 base 게이트(0.95)를 따라 조이지 **않는다** — 이유는 `target_3d.yaml` 주석. |

> 출력 frame이 `camera_color_optical_frame` → `camera_depth_optical_frame`으로 바뀜(둘 다 TF로 base_link에 연결). 메시지 계약은 동일해 다운스트림 무수정.
> rs2 extrinsics 회전은 **column-major** 라 `reshape(3,3, order='F')`로 읽어야 함(검증 완료).

---

### 1.5 `base_to_camera_tf` — 정적 TF

- **노드**: `tf2_ros/static_transform_publisher` (launch 인자)
- **역할**: `base_link → camera_link` 정적 변환 발행.
- **현재 값(실측)**: `x=0.811666, y=0.087684, z=0.924974`, quaternion `(qx, qy, qz, qw) = (0.72462829, 0.00008362, -0.68706199, 0.05347582)` — `src/system_bringup_pkg/launch/pick_place_vitis_ai.launch.py:236-264`
  - **2026-07-27 eye-to-hand 캘리브레이션** 결과(16 pose, AX=ZB, residual **1.8 cm / 1.7°**), E21 base 밀림 보정(yaw +5.14°) 포함. 등가 RPY가 pitch=90° gimbal 특이점 근처라 quaternion 형태로 넣는다. → `/pick_target_base`는 **실제 로봇 base_link 좌표**다.

---

### 1.6 `pick_target_base_node` — base_link 좌표 변환

- **파일**: `src/target_3d_pkg/target_3d_pkg/pick_target_base_node.py`
- **입력**: `/pick_target_3d` + TF
- **출력**: `/pick_target_base` (`PickTarget3D`, `frame_id = base_link`)
- **내부 동작**: 입력 유효성 검사(`target_valid`, `depth_valid`, frame 존재, z 범위) → `tf_buffer.lookup_transform(base_link, 입력frame)` → `do_transform_point`로 좌표 변환 → 발행.

**핵심 파라미터** (`target_base.yaml`)

| 파라미터 | 값 | 이유 |
|---|---|---|
| `target_frame` | `base_link` | 로봇 기준 좌표. |
| `require_depth_valid` | `true` | depth 없는 타깃은 변환 안 함. |
| `min_camera_z_m / max_camera_z_m` | `0.20 / 0.95` | **depth sanity 게이트**. ⚠️ reachability 필터가 **아니다** — 광축 z만 보고 x/y는 안 보므로 도달성을 판정할 수 없다. reachability는 브릿지 워크스페이스 박스 담당(`docs/RAON-RT/merge.md` §정합 3). 상한 0.95 유도는 `target_base.yaml` 주석 참조(2026-09-03). |
| `transform_timeout_sec` | `0.2` | TF 조회 대기 한도. |

---

### 1.7 메시지 타입 (`my_interfaces`)

```
Detection.msg       : int32 class_id, string class_name, float32 confidence,
                      float32 center_x, center_y, width, height
DetectionArray.msg  : std_msgs/Header header, Detection[] detections
PickTarget.msg      : bool target_valid + (Detection 필드들)
PickTarget3D.msg    : std_msgs/Header header, bool target_valid, bool depth_valid,
                      (Detection 필드들), float32 x, y, z
```

---

## 2. 성능 향상 기법 (전처리·후처리)

측정으로 병목을 분해(detector 콜백을 `img/pre/dpu/post/ipc`로 계측)한 뒤, 큰 leaf부터 제거했다. **DPU 추론 자체는 전체의 일부**였고, 진짜 병목은 **Python 전처리/후처리**였다.

> **읽는 법 (2026-09-03)**: 아래 §2.1·§2.2의 코드·상수·측정치는 전부 **SSD stand-in 시절**(`vitis_ai_worker.py`, 480×360, priors 16436개)의 기록이다. 그 worker는 이미 삭제됐지만 **두 기법 자체는 YOLO worker에 계승**됐다 — LUT는 `build_input_lut`/`letterbox_lut`로, 사전필터는 `decode_head`의 objectness pre-filter로. 기법의 근거를 남기려고 원문을 보존한다. 현재 코드의 수치는 §3.6 아래 최신 실측을 볼 것.

### 2.1 LUT 전처리 (`preprocess_image`) — *SSD 시절 기록, 기법은 계승*

**문제**: SSD 입력 텐서를 만들 때 픽셀마다
`int8 = clip(round((픽셀 − mean) × scale), −128, 127)` 를 480×360×3 = 518,400 픽셀에 대해 float32 배열로 여러 번 패스 + 매번 임시배열 할당 → A53(메모리 대역폭 약함)에서 **~42 ms**.

**통찰**: 입력은 `uint8`라 값이 **0~255, 256가지뿐**이고, 결과는 (값, 채널)에만 의존 → 같은 계산을 수십만 번 반복 중.

**기법 (Look-Up Table)**:
- 기동 시 1회, 채널별 256칸 int8 표를 미리 계산:
  `lut[c][v] = clip(round((v − mean[c]) × scale), −128, 127)` (`scale = 2**input_fix`)
- 매 프레임은 계산 없이 표 조회 3번:
  ```python
  arr[:, :, 0] = lut[0][resized[:, :, 0]]   # B
  arr[:, :, 1] = lut[1][resized[:, :, 1]]   # G
  arr[:, :, 2] = lut[2][resized[:, :, 2]]   # R
  ```

**결과**: `pre_ms 42 → ~12 ms`. 표는 기존 수식과 동일한 round/clip으로 만들어 **출력이 bit 단위로 동일**(검출 정확도 불변, 무작위 이미지로 검증).

### 2.2 SSD 후처리 사전필터 (`postprocess`) — *SSD 시절 기록, 기법은 계승*

**문제**: 16,436개 prior 전부에 softmax + box decode + NMS를 돌림. 대부분은 background인데도 전수 처리 → `post_ms ~20 ms`.

**기법 (안전한 necessary-condition 사전필터)**: foreground 클래스 임계값을 통과할 가능성이 있는 prior만 남기고 나머지는 버린 뒤, **살아남은 소수에만** softmax/decode/NMS:
- softmax 적용 케이스(conf=logits): `prob_c ≥ t_c` 이려면 필연적으로
  `z_c − z_background ≥ logit(t_c)` (sigmoid 상계에서 유도). 이 조건으로 1차 필터(softmax 불필요, 뺄셈·비교만). FP 경계값을 안 놓치게 ε 마진.
- 이미 확률인 케이스: `foreground prob ≥ min(threshold)` 로 필터.
- `loc` 역양자화·`decode_boxes`도 **후보에 대해서만** 수행.

**결과**: `post_ms 20 → ~7 ms`. 필터가 "필요조건"이라 통과 집합은 전수 처리의 상위집합 → 이후 정확한 클래스 임계값으로 잘라내므로 **최종 detection이 전수 처리와 동일**(무작위 데이터 30회로 검증).

**YOLO 계승판**: prior/softmax 대신 격자 셀의 **objectness raw logit**으로 같은 일을 한다 — `conf = sig(obj)·sig(cls) ≤ sig(obj)`이므로 `obj_q ≥ thr_q`(int8 그대로 비교, sigmoid조차 생략)가 안전한 필요조건이다. 통과한 셀에만 역양자화·sigmoid·decode·NMS를 돌린다.

### 2.3 기타 부하 절감

| 기법 | 효과 |
|---|---|
| 입력 resize 후 전송(`send_resized_input`) | pipe payload **−81%** (2.76 MB → 0.52 MB/frame). *SSD 시절 기법 — shm 전환 + worker letterbox로 지금은 `false`.* |
| 프레임을 `/dev/shm`으로(`worker_shm`) | ipc **13.4 → 4.65 ms**, throughput 18.4 → 21.7 Hz (2026-08-05) |
| 보드 overlay 제거 | 콜백 **−~44 ms/frame**. 2026-09-03 코드째 삭제(그리기는 데스크톱) |
| worker file-log off | 프레임당 ~18회 file I/O 제거 |
| 카메라 해상도 1280×720 → 848×480 | realsense 부하 ~2.3배↓ + busy-spin stall 해소 |
| full-frame alignment 제거 (§3.5) | realsense 코어 ~65% 회수, 검출 **~13 → ~17 Hz** |

**누적 결과**: 검출 `processing_ms 88.7 → 45.8 ms`, throughput `8.6 → 13 Hz`(compute), alignment 제거까지 더해 **~17 Hz**, end-to-end latency `349 → ~261 ms`. 모든 최적화에서 **검출 정확도 불변**.

---

## 3. 파이프라인 아키텍처

### 3.1 프로세스 분리 — detector 노드 ↔ worker 프로세스

- **이유**: VART `execute_async()`가 rclpy 노드 in-process에서 segfault/bus error. 근본 원인은 XIR graph/subgraph 객체가 GC되어 runner가 무효 상태를 참조한 것. 이를 **별도 프로세스 + 객체 수명 보장**으로 안정화.
- **구조**: ROS 래퍼(`vitis_ai_detector_node`)는 이미지 구독/전처리/발행만, **DPU는 `vitis_ai_worker_yolo.py`라는 별도 프로세스**가 담당. 제어(JSON 한 줄)는 stdin/stdout **pipe**, 프레임 본체는 **`/dev/shm` mmap**으로 넘긴다.
- **계약(JSON)**: `Detection` 호환 항목 + timing. **model-agnostic** — 모델을 바꿔도 worker 내부(전처리 상수·decode)만 바뀌고, 노드/파이프라인/3D/다운스트림은 그대로.

### 3.2 worker 수명주기

```
기동: decode_meta.json → xmodel deserialize → DPU subgraph → VART runner
      → (LUT, 입출력 버퍼, head_params) 준비 → "ready"(+ input_shape) 핸드셰이크  [~4.2s]
프레임 루프: stdin에서 JSON 헤더 → /dev/shm에서 프레임 뷰 → detect() → stdout으로 detection+timing
이상 시: 노드가 worker death 감지하면 재시작(restart_worker), 기동 타임아웃 30s
```
- xmodel/runner를 **1회만** 로드하고 출력 텐서 버퍼도 재사용 → per-frame 오버헤드 최소화.

### 3.3 콜백 파이프라이닝 (스레드 분리)

- **문제**: 기존엔 `image_callback`이 worker 응답까지 블록 → executor가 묶이고 콜백 간 idle 발생.
- **구조**:
  - `image_callback`: **최신 프레임만 저장**(락+이벤트), 즉시 반환 → executor가 카메라를 안 굶김.
  - `worker_loop`(전용 스레드) → `process_frame`: 최신 프레임을 집어 **연속 처리**(latest-frame-only, 묵은 프레임 버림).
- **GIL 안전성**: 무거운 DPU는 **별도 프로세스**(GIL 무관), 노드 스레드는 주로 pipe I/O 대기(GIL 해제) + numpy/cv2(GIL 해제) → 스레드가 효과적.
- **결과**: throughput은 그대로(시스템이 **camera-limited**임이 드러남), **latency는 ~31 ms 감소**. 깨끗한 구조 분리도 확보.

### 3.4 QoS / latest-frame-only

| 인터페이스 | QoS | 이유 |
|---|---|---|
| 카메라 color/depth 구독 | `KEEP_LAST, depth=1, BEST_EFFORT` | 항상 **최신 1장**만 — 묵은 프레임 누적 방지, latency 최소. |
| extrinsics 구독 | `RELIABLE, TRANSIENT_LOCAL` | 한 번만 latched 발행되므로 늦게 떠도 받기 위해 transient_local. |
| `/detections` 등 발행 | `RELIABLE, depth=1` | 다운스트림 누락 방지. |

### 3.5 Reverse projection (full-frame alignment 제거)

- **배경**: `target_3d`가 aligned depth를 구독하면 realsense가 매 프레임 depth 전체(≈407k 픽셀)를 color로 재투영 → 단일 스레드가 코어 100% 점유 → 카메라 12 Hz로 throttle. 그런데 우리가 쓰는 건 **bbox 중앙 1점**뿐.
- **기법 (`rs2_project_color_pixel_to_depth_pixel`)**: align을 끄고, color 픽셀 1개를 다음으로 매칭:
  1. color 픽셀을 `dmin`/`dmax`로 deproject → color→depth extrinsic 변환 → depth 영상의 **짧은 선분** 양 끝 계산
  2. 그 선분을 따라가며, 각 depth 픽셀을 다시 color로 reproject해 원래 픽셀과 가장 가까운 것을 선택
  3. 찾은 depth 픽셀의 z로 depth frame에서 `(x,y,z)` 산출
- **왜 단순 근사가 안 되나**: color↔depth는 ~15 mm baseline이라 시차(disparity)가 큼(가까운 물체에서 60~90 px). "같은 픽셀" 가정 시 엉뚱한 depth를 읽음. 선분 탐색이 이를 정확히 보정(시뮬 5점 0 px 오차, 실측 z 자로 검증).
- **결과**: realsense 코어 ~65% 회수, 검출 **~13 → ~17 Hz**, latency 감소, z 정확도 유지. 회수한 코어는 향후 RT/EtherCAT 로봇 제어 여유로 사용.

### 3.6 성능 요약 (검출 단계)

| 단계 | baseline | +LUT | +post필터 | +pipe | +reverse-proj(align off) |
|---|---|---|---|---|---|
| `pre_ms` | 42.5 | 12.7 | 12.5 | 11.8 | — |
| `post_ms` | 20.1 | 19.4 | 7.4 | 7.8 | — |
| `dpu_ms` | 12.7 | 13.0 | 12.8 | 12.5 | (하드웨어 하한) |
| `processing_ms` | 88.7 | 58.2 | 45.8 | 46.1 | ~46 |
| **publish Hz** | **8.6** | 12.2 | 13.0 | 12.7 | **~17** |
| `frame_age_ms`(latency) | 349 | 307 | 292 | 261 | ~261 |

> throughput의 최종 천장은 **카메라 공급률(realsense 노드 단일 스레드 처리)** 이다. 더 올리려면 카메라 측(노드 부하/해상도) 또는 C++ 비전 경로가 필요. 단, **정적 물체** 전제였던 당시엔 ~17 Hz로 충분하다고 판단했다 — 2026-08-04 이동 물체 접근·추종으로 방향이 바뀌며 그 전제는 폐기됐다(아래 보강).

> **⚠️ 2026-08-05 보강**: 위 문장의 "최종 천장"은 *모든 소프트웨어 상한을 걷어낸 뒤*의
> 천장이라는 뜻으로만 맞다. **지금 실제로 묶고 있는 건 카메라가 아니다** — 카메라는
> 29.4 Hz를 공급하는데 파이프라인은 15.3 Hz만 내고, 구속조건은 `process_period_sec: 0.045`
> 게이트(의도된 15 Hz 상한)다. DPU도 73% 놀고 있다. 성능을 올리는 순서와 각 단계의
> 이득·비용·위험은 **`docs/vision/throughput.md`**에 정리했다. 카메라나 DPU부터
> 손대자는 제안은 거의 항상 틀렸다.
> **08-05 후속**: 이동 물체 전환으로 게이트 0.045→0.030 재결정 + worker IPC를
> pipe→/dev/shm mmap으로 교체(`shm_frame.py`) → **21.7 Hz**, CPU 변화 0. 같은 문서 §4에 실측.

**현재 구성 실측** (`evidence/metrics/runs/shm_gate030_20260805/performance/detector.csv`, 6343프레임 중앙값):

| `pre_ms` | `dpu_ms` | `post_ms` | `worker_ms` | `ipc_overhead_ms` | `processing_ms` | `detect_period_ms` |
|---|---|---|---|---|---|---|
| 11.56 | 17.35 | 4.78 | 35.26 | 3.19 | 43.01 | 45.47 (≈22 Hz) |

> 위 baseline 표(SSD 시절)와 직접 비교하지 말 것 — 모델도 입력 크기도 IPC도 다르다.

---

## 부록: 알려진 placeholder / TODO

- ~~`base_link → camera_link` 정적 TF는 **placeholder** → camera-to-base 캘리브레이션 필요(그 전엔 `/pick_target_base`는 실제 로봇 좌표 아님).~~ **완료** — 2026-07-27 eye-to-hand 캘리브레이션(residual 1.8 cm / 1.7°)의 실측값이 들어가 있다(§1.5).
- ~~`ssd_adas`는 **검증용 stand-in** → 최종 pick-object detector로 교체 예정.~~ **완료** — YOLOv3-tiny 6클래스로 교체됐고 SSD worker는 2026-09-03 삭제됐다.
- 모델 파일명 `yolov3_tiny_7class.xmodel`의 "7class"는 **잔재**(실제 6클래스). 배포 아티팩트 이름이라 `setup.py` glob·launch·디스크 파일을 함께 옮겨야 해서 미정리.
- 로봇 제어 레이어(APU→RPU(FreeRTOS)→Indy7, Ethernet/EtherCAT)는 미구현 — CPU 코어 격리 설계 필요.
