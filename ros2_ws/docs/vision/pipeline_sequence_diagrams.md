# 파이프라인 노드별 sequence diagram — 코드 공부용

작성 2026-08-19. perception 파이프라인을 구성하는 **python 파일 6개의 내부 구조**를 파일당 1개의 Mermaid sequence diagram으로 정리한다. 함수·토픽·파라미터 이름은 전부 실제 코드의 이름이다(작성일 기준 소스와 대조함).

- **보는 법**: GitHub 웹에서는 자동 렌더. 데스크톱 VS Code는 "Markdown Preview Mermaid Support" 확장 설치 후 preview(Ctrl+Shift+V). 보드 터미널에서는 텍스트로만 보이므로 desktop에서 볼 것. 급하면 <https://mermaid.live>에 블록을 붙여넣어도 된다.
- **detector + worker를 줄 단위로 정독**하려면 `detector_worker_walkthrough.md`(코드 공부용 지도)가 정본이다. 이 문서는 "함수들이 **시간 순으로 어떻게 상호작용하는가**"를 보여주는 보완 자료다.
- PlantUML을 아는 사람을 위한 문법 브리지:

| PlantUML | Mermaid |
|---|---|
| `@startuml … @enduml` | ` ```mermaid … ``` ` 코드 블록 |
| `A -> B : msg` | `A->>B: msg` |
| `A --> B : msg` (응답/점선) | `A-->>B: msg` |
| `loop / alt / else / opt / par / end` | 동일 |
| `note over A,B` | `Note over A,B: …` |
| (없음) | `break 조건 … end` — 조기 탈출(early return) 표현 |

---

## 0. 전체 파이프라인 overview

노드 5개 + worker 프로세스 1개. **`capture_stamp`(color 캡처 시각)가 파이프라인 전체의 join key**라는 것이 핵심이다.

```mermaid
sequenceDiagram
    participant CAM as camera<br/>(realsense2_camera)
    participant DET as vitis_ai_detector_node
    participant WK as vitis_ai_worker_yolo<br/>(자식 프로세스, DPU)
    participant PL as pick_logic_node
    participant T3 as pick_target_3d_node
    participant TB as pick_target_base_node
    participant ROB as CROS2PickBridge<br/>(RAON-RT App/Indy7, APU 실운용;<br/>RPU 이관 시 출력만 seqlock→rpmsg)

    CAM->>DET: /camera/camera/color/image_raw (Image BGR 848x480 @30fps)
    CAM->>T3: /camera/camera/depth/image_rect_raw (Image 16UC1 raw @30fps)
    CAM->>T3: CameraInfo x2 + depth→color extrinsics (시작 시, latched)
    Note over T3: depth_buf(deque)에 800ms 보관 — §5
    DET->>WK: frame(/dev/shm) + JSON header(stdin) — §1·§3
    WK-->>DET: {detections, timing} (stdout JSON) — §2
    DET->>PL: /detections (DetectionArray, ~21.7Hz)
    PL->>T3: /pick_target (PickTarget) — §4
    T3->>TB: /pick_target_3d (PickTarget3D, camera_depth_optical_frame) — §5
    TB->>ROB: /pick_target_base (PickTarget3D, base_link) — §6
    Note over DET,TB: capture_stamp 전파 경로: DetectionArray.header.stamp(=color 캡처 시각)<br/>→ PickTarget.capture_stamp → 그대로 전파.<br/>§5에서 depth 매칭(nearest_by_stamp), §6에서 true_e2e_ms 계산에 사용
```

**해설**
- 노드 사이는 전부 ROS2 토픽이지만, detector↔worker 사이만 **프로세스 경계**(stdin/stdout pipe + /dev/shm)다. 프로세스를 나눈 이유는 VART `execute_async`가 rclpy 프로세스 안에서 segfault를 냈기 때문(§2 참조).
- 전체 rate(~21.7Hz)는 §1의 `process_period_sec` gate 한 곳에서 정해진다 — 성능 얘기는 `docs/vision/throughput.md`가 정본.

---

## 1. `vitis_ai_detector_node.py` (921줄) — ROS 배관 + 2-thread 구조

`src/vitis_ai_detector_pkg/vitis_ai_detector_pkg/vitis_ai_detector_node.py`. 구독 callback과 처리 루프를 **다른 thread로 분리**(callback pipelining)한 것이 구조의 전부다.

```mermaid
sequenceDiagram
    participant SUB as image_callback<br/>(rclpy executor 스레드)
    participant SLOT as latest_msg<br/>(slot + frame_lock)
    participant WL as worker_loop<br/>(전용 스레드)
    participant SHM as ShmFrameWriter<br/>(/dev/shm)
    participant WP as worker 프로세스<br/>(stdin/stdout pipe)

    Note over WL,WP: __init__ 시 start_worker() — Popen → "ready" handshake 대기(§2) → worker_loop 스레드 시작

    par 수신 스레드 (camera 30fps)
        loop 매 color frame
            SUB->>SLOT: latest_msg = msg — 이미 있으면 덮어씀(frames_overwritten 집계)
            SUB->>WL: frame_event.set()
        end
    and 처리 스레드 (~21.7Hz)
        loop worker_loop
            WL->>WL: frame_event.wait(0.5s)
            WL->>SLOT: msg 꺼내고 None으로 (frame_lock, frame_event.clear)
            WL->>WL: should_process_frame() — process_period_sec(0.030) gate, 미달이면 skip
            Note over WL: 이하 process_frame(msg)
            WL->>WL: ros_image_to_bgr(msg)
            Note over WL,WP: 이하 detect_with_worker(image) — ①→② 순서가 프로토콜 규약<br/>(worker는 header가 도착하는 즉시 shm을 읽는다)
            WL->>SHM: ① shm_writer.write(frame) — memcpy 1회
            WL->>WP: ② stdin JSON {height, width, channels, data_len=0, shm_path, source_w/h}
            alt 정상 응답
                WP-->>WL: stdout JSON {detections, timing, error} — read_worker_json(select+timeout)
                WL->>WL: json_items_to_detections() → DetectionArray(header = msg.header 그대로)
                WL->>WL: /detections publish (빈 결과는 publish_empty_detections 따름)
            else timeout / BrokenPipeError
                WL->>WP: restart_worker() — stop → Popen → ready 재대기(~4.5s)
                Note over WL: 예외는 전파되어 이번 프레임 포기. 단 shutdown 중이면 재시작하지 않음<br/>(재로드 4.5s가 launch의 SIGINT 5s 유예를 넘겨 exit -15를 유발하므로)
            end
        end
    end
```

**해설**
- `latest_msg`가 **queue가 아니라 1칸 slot**인 이유: moving object에서는 밀린 프레임을 처리하는 것 자체가 손해다. 새 프레임이 오면 덮어쓰고, 덮인 수를 drop으로 집계한다. executor는 절대 DPU를 기다리지 않는다(callback은 저장+event set만 하고 즉시 반환).
- **①shm 먼저 → ②header 나중** 순서는 규약이다. pipe에는 JSON 한 줄만 흐르고 프레임 본문은 /dev/shm mmap으로 전달(2026-08-05, ipc 13.4→4.65ms). `send_resized_input: false`라 848×480 원본이 그대로 가고 letterbox는 worker가 한다.
- `timing`(§2)과 자신의 시계를 비교해 `ipc_overhead_ms = worker_call_ms − worker_ms`를 계산 — IPC 비용이 metrics로 상시 감시된다.
- 종료는 역순: `stop_pipeline()`(shutdown_event + join) → metrics 저장 → `stop_worker()`(stdin close → terminate → SIGKILL 에스컬레이션) → `shm_writer.unlink()` — **unlink는 여기 한 곳뿐**(§3의 inode 함정).

---

## 2. `vitis_ai_worker_yolo.py` (451줄) — DPU 추론 프로세스

`src/vitis_ai_detector_pkg/vitis_ai_detector_pkg/vitis_ai_worker_yolo.py`. **ROS가 아니다** — rclpy import조차 없는 순수 프로세스이고, 통신은 stdin/stdout 뿐이다. thread도 없다(요청 1건씩 엄격히 직렬).

```mermaid
sequenceDiagram
    participant ND as detector node<br/>(부모 프로세스)
    participant MAIN as main() loop
    participant SHM as ShmFrameReader
    participant YW as YoloWorker
    participant DPU as DPU<br/>(VART runner)

    Note over MAIN,DPU: 시작: --model/--meta 파싱 → YoloWorker() — decode_meta.json 로드(6-class, input 416),<br/>load_model(): LUT·입출력 buffer·anchor/stride(head 2개) 프레임 불변량 사전 계산
    MAIN-->>ND: {"status": "ready", "input_shape": [1,416,416,3]}

    loop stdin 한 줄 = 요청 1건
        ND->>MAIN: JSON {height, width, channels, shm_path, data_len, source_w/h}
        alt shm_path 있음 (기본, worker_shm=true)
            MAIN->>SHM: view(shm_path, h·w·c) — mmap 위의 zero-copy numpy view
        else pipe fallback
            MAIN->>MAIN: read_exact(stdin, data_len) → np.frombuffer
        end
        MAIN->>YW: detect(image, output_w/h = source_w/h)
        YW->>YW: letterbox_lut() — resize+pad+정규화+int8 양자화를 LUT 한 방에 → input_buf (pre_ms)
        YW->>DPU: execute_async([input_buf], output_data) → wait(job_id) (dpu_ms)
        DPU-->>YW: output tensor 2개 (stride 16 / 32 head)
        YW->>YW: decode_head() ×2 → concat — thr_q(양자화 threshold)로 배경 사전 필터
        YW->>YW: make_detections() — per-class threshold → NMS(0.45) → letterbox 역변환·clamp → top-100 (post_ms)
        YW-->>MAIN: detections + last_timing {pre_ms, dpu_ms, post_ms, worker_ms}
        MAIN-->>ND: {"detections": [...], "timing": {...}, "error": null}
    end

    Note over MAIN: 예외 시 {"detections": [], "error": str} (timing 키 없음) / stdin EOF면 loop 종료
```

**해설**
- 프로세스로 분리한 이유: VART `execute_async`가 rclpy 프로세스 안에서 segfault → **process isolation**으로 해결. `graph`/`dpu_subgraphs`를 `self`에 계속 잡아두는 것도 같은 맥락(해제하면 execute_async가 죽을 수 있음).
- `load_model()`이 프레임 불변량(LUT, buffer, anchor, thr_q)을 전부 시작 시 1회 계산해 두므로 loop 안은 순수 연산만 남는다. preprocess(LUT)와 background 사전 필터는 naive 구현과 **bit-identical** 검증됨.
- `CLASS_THRESHOLDS_BY_NAME`에서 person은 threshold 2.0 — confidence는 1.0을 못 넘으므로 사실상 출력이 차단돼 있다. (2026-09: pick_logic의 person guard는 제거됐다 — 이 threshold를 낮춰도 되살아나는 안전장치는 없다.)
- import 특이점: node가 이 파일을 **절대경로 스크립트로 Popen**하므로 `from shm_frame import …` fallback이 있고, 테스트는 패키지 모듈(`from vitis_ai_detector_pkg.shm_frame import …`)로 import한다.

---

## 3. `shm_frame.py` (96줄) — /dev/shm 프레임 전달

`src/vitis_ai_detector_pkg/vitis_ai_detector_pkg/shm_frame.py`. 클래스 2개(96줄)가 §1↔§2 사이 프레임 본문 전달을 담당한다.

```mermaid
sequenceDiagram
    participant W as ShmFrameWriter<br/>(node 프로세스)
    participant F as /dev/shm/vitis_ai_frame_(pid)
    participant R as ShmFrameReader<br/>(worker 프로세스)

    Note over W,R: 프로토콜이 엄격히 직렬(요청 1건 → 응답 1건)이라 lock이 없다

    W->>F: write(arr) — 크기 부족할 때만 grow: _unmap → open(O_CREAT|O_RDWR) → ftruncate(n) → mmap
    W->>F: self._np[:n] = arr.reshape(-1) — memcpy 1회 (이게 전송의 전부)
    Note over W: 이후 node가 stdin으로 JSON header 전송 (§1의 ①→② 순서 규약)
    R->>F: view(path, nbytes) — path/크기 바뀔 때만 remap: open(O_RDONLY) → fstat 크기검사 → mmap(PROT_READ)
    F-->>R: self._np[:nbytes] — 복사 0회, worker는 이 view 위에서 바로 letterbox

    Note over W,F: unlink()는 shutdown에서 단 1회.<br/>중간에 파일을 지웠다 재생성하면 inode가 바뀌어 worker가 죽은 inode를 계속 읽는다<br/>(grow는 같은 파일 ftruncate + remap이라 안전)
```

**해설**
- 같은 해상도가 계속 오는 정상 운전에서는 양쪽 다 mmap을 재사용하므로, 프레임당 비용은 **writer의 memcpy 1회**뿐이다(reader는 zero-copy view).
- worker가 응답을 보내기 전에는 node가 buffer를 다시 쓰지 않는다는 직렬성 위에서만 안전한 설계다 — §1·§2의 loop가 그 직렬성을 보장한다.
- **inode 함정**: 디버깅 중에도 `/dev/shm/vitis_ai_frame_*`를 절대 지우지 말 것(모듈 docstring에도 명시).

---

## 4. `pick_logic.py` (477줄, v3 2026-09-04) — 2D 필터 ladder + 단일 인스턴스 추적

`src/pick_logic_pkg/pick_logic_pkg/pick_logic.py`. thread도 IPC도 없는 단일 callback — 구조의 전부는 **L1→L2→L3 ladder와, 모든 조기 탈출이 `_finish_invalid` 한 경로로 수렴**한다는 것이다. v3에서 v2의 점수식(`_score`)과 hysteresis 전환, AUTO 모드가 빠졌다 — 오퍼레이터가 지정한 클래스의 인스턴스 1개를 골라 잃을 때까지 붙어 있는다(sticky).

```mermaid
sequenceDiagram
    participant IN as /detections
    participant CB as detections_callback
    participant TR as _track 상태
    participant OUT as /pick_target

    IN->>CB: DetectionArray (raw)
    CB->>CB: desired_class 라이브 파라미터 재조회 — 바뀌었으면 _reset_track()
    break 빈 배열
        CB->>TR: _step_track_missing()
        CB->>OUT: _finish_invalid('empty_detection_array')
    end
    break desired_class == ''
        CB->>OUT: _finish_invalid('no_class_selected') — 오퍼레이터가 'p'로 고르기 전
    end
    CB->>CB: L1 is_detection_acceptable() — conf / allowed_classes / edge margin / bbox 면적<br/>(+ desired_class 아닌 class는 desired_class_not_visible)
    break 후보 0개
        CB->>TR: _step_track_missing()
        CB->>OUT: _finish_invalid(첫 reject 사유)
    end
    CB->>TR: L2+L3 _update_track(candidates) — 트랙 없음: confidence 최대 채택 /<br/>있음: 최근접 이어받기(match_px 게이트는 2개 이상일 때만) / 없으면 missing++ → lost_timeout
    break 이번 프레임에 트랙 인스턴스 없음 또는 stable < stability_min_frames
        CB->>OUT: _finish_invalid('target_unstable')
    end
    CB->>OUT: make_pick_target(det) → target_valid=true, capture_stamp 실어 publish

    Note over CB,OUT: 모든 조기 탈출이 _finish_invalid 하나로 수렴:<br/>_tally(사유) → publish_invalid_target(publish_invalid_targets 옵션) → _record_metrics<br/>→ 어떤 경로로 끝나도 입력 1건당 metrics 1행이 보장된다
```

**해설**
- `desired_class`는 매 frame 재조회하는 **live parameter**다(로봇 쪽 operator 메뉴가 바꾼다). 바뀌면 tracking 상태를 리셋.
- `_update_track`은 프레임 간 상태(`_track`)를 가진 유일한 곳. 트랙은 **sticky**: 한 번 채택된 인스턴스는 더 좋은 후보가 나와도 바뀌지 않고 `lost_timeout_frames` 연속 사라져야 해제된다(v2의 hysteresis 전환은 추종 중 TCP를 다른 물체로 끌고 갈 수 있어 삭제). 같은 클래스가 2개 이상일 때는 최근접 + `stability_match_px` 게이트로 구분한다 — 실험은 클래스당 1개지만 검출기가 같은 클래스 박스를 2개 낼 수 있어(긴 물체 분할, 오검출) 동작을 정의해 둔 것. 근거 전문은 그 함수의 docstring.
- `capture_stamp`(=DetectionArray.header.stamp=색상 캡처 시각)를 PickTarget에 실어 보내는 것이 §5·§6의 전제.

---

## 5. `pick_target_3d_node.py` (645줄) — depth 매칭 + 역투영

`src/target_3d_pkg/target_3d_pkg/pick_target_3d_node.py`. 독립 callback 두 개가 `depth_buf`(deque)를 사이에 두고 만난다.

```mermaid
sequenceDiagram
    participant DT as /depth/image_rect_raw
    participant DC as depth_callback
    participant BUF as depth_buf (deque)
    participant PT as /pick_target
    participant PC as pick_target_callback
    participant OUT as /pick_target_3d

    Note over DC,PC: 시작 시 1회: depth/color CameraInfo 2개 + depth→color extrinsics(latched) 수신.<br/>prune_timer(1s)가 셋 다 수신 확인 후 static sub 3개를 정리하고 스스로 소멸<br/>(자기 callback 안에서 sub를 destroy하지 않기 위한 우회)

    par depth 경로 (30fps — 저장만)
        loop 매 depth frame
            DT->>DC: Image (16UC1 raw)
            DC->>BUF: append((stamp_ns, msg)) — 디코딩은 pick 시점으로 미룸
            DC->>BUF: 800ms(depth_hist_ns)보다 오래된 항목 popleft (나이 기준 prune)
        end
    and pick 경로 (~21.7Hz)
        loop 매 PickTarget
            PT->>PC: PickTarget (capture_stamp 포함)
            break target_valid == false
                PC->>OUT: _emit — 2D 필드만 복사해 그대로 통과 (depth_valid=false)
            end
            PC->>BUF: nearest_by_stamp(capture_stamp) — 캡처 시각에 가장 가까운 depth frame 선택
            break 준비 안 됨 — no_depth_image / no_intrinsics / no_extrinsics / bad_depth_encoding
                PC->>OUT: _emit(depth_valid=false) — 사유는 fail_tally로 집계
            end
            PC->>PC: color_pixel_to_depth_pixel(center_x, center_y) — 역투영 매칭 (vectorized 구현)
            opt epipolar_ab_check=true
                PC->>PC: color_pixel_to_depth_pixel_loop(참조 구현)와 결과 A/B 비교
            end
            break 매칭 실패 (no_valid_depth)
                PC->>OUT: _emit(depth_valid=false)
            end
            PC->>PC: get_depth_m_from_patch — (2·patch_radius+1)² patch 유효값 median + min/max 범위검사
            break patch 무효 (no_valid_depth_patch)
                PC->>OUT: _emit(depth_valid=false)
            end
            PC->>PC: deproject(d_fx, d_fy, d_cx, d_cy, u, v, z) → (x, y, z)
            PC->>OUT: _emit → publish (camera_depth_optical_frame, depth_valid=true)
        end
    end
```

**해설**
- **deque(800ms 역사)이지, latest 1장이 아니다.** detection이 캡처보다 ~90ms 늦게 도착하므로 "가장 최신 depth"는 파트너보다 ~3프레임 뒤다 — `nearest_by_stamp`로 캡처 시각에 맞는 frame을 되찾아야 skew가 36ms → 4.3ms가 된다(2026-08-04). **latest 1장으로 "단순화"하면 안 된다.**
- prune이 `maxlen`이 아니라 **나이 기준**인 이유: depth fps(15/30)는 설정값이라, 길이 고정이면 fps 바꿀 때 역사 길이가 달라진다.
- depth_callback이 **저장만** 하는 이유: 디코딩을 30fps 전부에 하면 낭비 — 실제 쓰는 pick 시점(~21.7Hz)에만 `depth_msg_to_view`(cv_bridge 없는 frombuffer view, 1회 등가성 검증)로 디코딩.
- 어떤 경로로 끝나든 `_emit` 한 곳을 지나므로 입력 1건당 출력/metrics 1행이 보장된다. `_emit`은 실제 사용한 depth frame 기준 skew(`depth_vs_color_skew_ms`)와 "latest를 썼다면"의 skew(`skew_if_latest_ms`)를 같이 기록해 상시 A/B가 된다.

---

## 6. `pick_target_base_node.py` (353줄) — base_link 변환

`src/target_3d_pkg/target_3d_pkg/pick_target_base_node.py`. 마지막 단계 — camera frame의 (x,y,z)를 `base_link`로 옮긴다. 구조의 핵심은 **tf2 lookup을 frame_id당 1회만 하고 (R, t)를 cache**하는 것.

```mermaid
sequenceDiagram
    participant IN as /pick_target_3d
    participant CB as target_callback
    participant TF as tf2 Buffer<br/>(TransformListener 내부 스레드가 /tf, /tf_static 수신)
    participant CA as _tf_cache<br/>(frame_id → R, t)
    participant OUT as /pick_target_base

    IN->>CB: PickTarget3D (camera_depth_optical_frame)
    CB->>CB: is_input_acceptable() — target/depth_valid, frame_id 존재, min/max_camera_z 범위
    break reject — target_not_valid / depth_not_valid / empty_input_frame / camera_z_too_close·far
        CB->>OUT: _emit(invalid) — publish_invalid_targets일 때만 publish
    end
    alt cache hit (2번째 메시지부터, static_tf_cache=true)
        CB->>CA: (R, t) 조회
        CB->>CB: p = R·[x,y,z] + t — matmul 1회, tf2 호출 없음
        CB->>OUT: _emit → publish (base_link)
    else cache miss (frame_id당 최초 1회, slow path)
        CB->>TF: lookup_transform(base_link ← frame_id, timeout=transform_timeout_sec)
        TF-->>CB: TransformStamped
        CB->>CB: do_transform_point(point)
        CB->>OUT: _emit → publish 먼저
        CB->>CA: 그 다음 cache 채움 — quat_to_rotation_matrix로 (R, t) 저장,<br/>matmul 결과와 tf2 결과의 최대 오차를 로그로 남김
    else TransformException
        CB->>OUT: _emit(invalid, 'tf_transform_failed') — publish_invalid_targets일 때만
    end
    Note over CB,OUT: _emit: true_e2e_ms(capture_stamp → 지금, valid만)와<br/>output_period_ms(실제 publish 간격) 계산 후 publish
```

**해설**
- cache가 정당한 이유: `base_link → camera` TF는 **static**(launch 인자에 박힌 캘리브 실측 고정값)이라 매번 tf2 lookup을 하는 것이 순수 낭비다. 대신 최초 1회는 tf2 정답과 matmul 결과의 오차를 로그로 검증하고 나서 cache에 넣는다.
- slow path에서 **publish가 cache 채우기보다 먼저**다 — 첫 메시지의 지연을 늘리지 않기 위해.
- `true_e2e_ms`가 여기서 나온다: color 캡처(capture_stamp)부터 base 좌표 publish까지 — CLAUDE.md의 "capture→base E2E p50 ~118ms"가 이 값이다.
- TF는 **2026-07-27 eye-to-hand 캘리브 실측치**(16 poses, AX=ZB, 잔차 1.8 cm / 1.7°) + E21 베이스 재보정(yaw +5.14°)이다(`launch/pick_place_vitis_ai.launch.py:236-264`) — 여기서 나오는 base 좌표가 곧 로봇이 접근·추종에 쓰는 실좌표다.
