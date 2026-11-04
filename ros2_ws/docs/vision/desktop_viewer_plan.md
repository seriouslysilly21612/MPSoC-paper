# 데스크톱 bbox overlay 뷰어 — 구현 완료 (as-built 명세)

**상태: ✅ 완료·실물 검증.** 보드 검출 결과를 데스크톱 화면에 bbox overlay 영상으로 띄운다. **보드는 JPEG 압축만, 그리기는 전부 데스크톱** — 보드 부하는 순감이다.

- **2026-07-16** 최초 구현·실물 검증 (커밋 `915d86d`, 조인 100% 커밋 `8eb6338`).
- **2026-07-20** launch 통합 — 뷰잉 전용 `viewing.launch.py`/`realsense_viewing.yaml` 삭제, 전체 pick 파이프라인 하나로 흡수(§8).
- **2026-07-22** 뷰어에 **출력 FPS HUD(우상단) + FPS CSV 로깅** 추가(§4.4–4.5).

> 이 문서는 이제 "계획"이 아니라 **구현된 시스템의 명세·운영 레퍼런스**다. 코드는 `src/detection_viewer_pkg/detection_viewer_pkg/detection_viewer_node.py`가 정본이며, 아래 §4가 그 as-built 요약이다.

---

## 1. 한눈에 (운영)

| | 명령 | 위치 |
|---|---|---|
| **보드** | `ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py` | Kria `~/ros2_ws` (192.168.120.50) |
| **데스크톱** | `ros2 run detection_viewer_pkg detection_viewer_node` | `~/pp_ws/viewer_ws` (jaehyeon@jaehyeon-Raimlab) |

- 전체 파이프라인이 항상 돌고, 데스크톱 뷰어가 거기 붙는다. 압축 토픽은 **구독자가 있을 때만 인코딩**(lazy, §3)되므로 뷰어를 끄면 보드 압축 비용은 0.
- 실측: 압축 스트림 30 Hz / `/detections` ~15 Hz → **화면 출력 ~15–17 FPS**(detection 게이트), 조인 100%, 영상 버벅임 없음.
- 종료: 창에서 `q`/`ESC` 또는 터미널 `Ctrl-C`. 종료 시 FPS CSV가 자동 저장된다(§4.5).

---

## 2. 아키텍처

```
[Kria 192.168.120.50]
  realsense ─┬─→ /camera/camera/color/image_raw            (raw, 로컬 SHM으로 detector에게)
             └─→ /camera/camera/color/image_raw/compressed (JPEG — 구독자 있을 때만 인코딩)
                        │
  vitis_ai_detector ────┼──→ /detections   (RELIABLE, ~15 Hz, header = color header 바이트 복사본)
                        │
          (UDPv4 / USB NIC enxc8a362ec54c4 / FastDDS / domain 0)
                   ※ eth0는 IgH EtherCAT 전용(IP 없음)
                        ▼
[Desktop Humble]  detection_viewer_node
                    CompressedImage + DetectionArray
                    → (sec,nanosec) stamp 조인 → cv2.imdecode → 박스+HUD draw → imshow
```

**보드 그리기는 아예 없앴다(2026-09-03).** `publish_overlay` 계열 파라미터와 그리기 메서드 4개를 코드째 삭제했다. 그 근거가 된 실측:

| 항목 | 값 |
|---|---|
| `detect_ms` 평균 / p95 | 37.63 / 40.16 ms |
| 보드 overlay 비용(현 설정) | **44.14 ms/frame** |
| 합 | **81.8 ms** |
| 15 Hz 예산 | **66.6 ms** |

→ 보드에서 그리면(`publish_compressed_overlay: true`) 합이 예산을 초과해 **검출률이 조용히 15 Hz 아래로 떨어진다**(`metrics_csv_path: ""`라 눈치채기도 어렵다). 데스크톱에서 그리면 보드는 이 비용을 아예 안 치른다 — 이게 이 아키텍처의 핵심 근거다. (문서상 "overlay ~70 ms"와 재측정 15/44 ms가 다투지만, 데스크톱 렌더는 그 논쟁과 무관하게 성립한다.)

---

## 3. 왜 압축 비용이 평소 0인가 (lazy encoding)

`image_transport::Publisher::publish`는 플러그인 벡터를 돌며 `getNumSubscribers()`(vtable slot 3)를 호출하고 **0이면 `publish()`(slot 5)를 건너뛴다** — 설치된 `libimage_transport.so` 디스어셈블로 확인. 즉 **데스크톱 뷰어를 끄면 JPEG 인코딩 비용이 즉시 0**이다.

- 보드 실측(2026-07-20): 전체 6노드 기동 + `/compressed` 광고 + **Subscription 0에서 인코딩 0** + `/detections` 15.3 Hz.
- 주의: `getNumSubscribers()`는 **raw + compressed 전 transport를 합산**한다. compressed만 구독해도 realsense의 publish 경로 전체가 깨어난다(의도된 동작).

---

## 4. 구현 명세 (뷰어 노드 as-built)

**파일**: `src/detection_viewer_pkg/detection_viewer_pkg/detection_viewer_node.py`
**엔트리포인트**: `detection_viewer_pkg/detection_viewer_node:main` (`ros2 run detection_viewer_pkg detection_viewer_node`)
**의존**: `rclpy`, `sensor_msgs`, `my_interfaces`, `cv2`(opencv-python), `numpy`. 보드 전용 의존(VART/DPU/realsense) **0개** → 데스크톱에서 그대로 빌드/실행.

### 4.1 CLI 인자

| 인자 | 기본값 | 의미 |
|---|---|---|
| `--image-topic` | `/camera/camera/color/image_raw/compressed` | 압축 color 토픽 |
| `--detections-topic` | `/detections` | `my_interfaces/DetectionArray` |
| `--buffer` | `30` | color 프레임 버퍼 크기(최근 N장, stamp 조인용) |
| `--window` | `KV260 detections` | OpenCV 창 이름 |
| `--fps-dir` | `~/pp_ws/viewer_ws/fps_log` | FPS CSV 자동 저장 디렉토리 (**로깅은 항상 켜짐**) |
| `--fps-csv` | `""` | 정확한 CSV 파일 경로 지정(자동 이름 override) |

ROS 인자는 `parse_known_args`로 분리해 `rclpy.init`에 넘긴다.

### 4.2 구독 & QoS

| 토픽 | 타입 | 뷰어 구독 QoS |
|---|---|---|
| `.../color/image_raw/compressed` | `sensor_msgs/CompressedImage` | **BEST_EFFORT**, KEEP_LAST 1, VOLATILE |
| `/detections` | `my_interfaces/DetectionArray` | **RELIABLE**, KEEP_LAST 1, VOLATILE |

발행자는 `/compressed`가 RELIABLE(→ raw의 SYSTEM_DEFAULT 상속), `/detections`가 RELIABLE. RELIABLE 발행자 ↔ BEST_EFFORT 구독자는 **호환**이고, 이미지는 곧 덮어쓸 데이터라 재전송 지연을 피하려 BEST_EFFORT로 받는다(§7 QoS).

### 4.3 2-sided stamp 조인 (핵심 알고리즘)

두 스트림은 **경합**한다 — detection은 캡처 ~37 ms 뒤 발행돼 네트워크를 건너고, 압축 이미지는 독립 경로로 건넌다. **어느 쪽이든 먼저 도착할 수 있다.** 그래서 조인은 양방향이다.

- `frames`: `OrderedDict[(sec,nanosec) → 디코드된 BGR]`, 상한 `--buffer`(30). "이미지가 먼저" 경우 커버.
- `pending`: `OrderedDict[(sec,nanosec) → DetectionArray]`, 상한 `pending_len`(10). "detection이 먼저" 경우 커버.
- `on_image`: `imdecode` → `frames`에 저장 → **대기 중인 detection이 있으면**(`pending.pop(key)`) 즉시 렌더(`late`).
- `on_detections`: 같은 stamp 프레임이 있으면 렌더(`hit`), 없으면 `pending`에 보관(상한 초과분만 `drop`).
- `try_render`: **단조 가드** — `key < last_rendered`면 skip(`stale`). 화면이 뒤로 감기는 것 방지.

카운터(5초마다 `report()` 로그): `hit`/`late`/`drop`/`stale` + `pending` + 조인율. `drop=0`이면 링크가 깨끗한 것, `stale`은 병적 지연 보험(실측 항상 0).

> **왜 양방향이어야 하나(실증)**: 초기엔 detection 도착 시에만 이미지를 찾고 없으면 버려서 조인이 95~100%로 흔들렸다(30장 버퍼는 이미지가 *먼저* 올 때만 유효). `on_image`에서도 짝을 맞추게 고쳐 **전 구간 100%**(`8eb6338`). `late`로 잡히는 프레임이 예전에 버려지던 바로 그 프레임이다.

### 4.4 렌더링

- **박스 좌표 = 원본 848×480 픽셀 그대로** (역변환 금지, 근거 §7):
  ```python
  xmin = int(round(det.center_x - det.width  / 2.0))
  ymin = int(round(det.center_y - det.height / 2.0))
  xmax = int(round(det.center_x + det.width  / 2.0))
  ymax = int(round(det.center_y + det.height / 2.0))
  ```
- **색상 맵 `CLASS_COLORS`는 `class_name` 문자열로 키잉** (5클래스: apple/orange/banana/tennis_ball/mustard_bottle — 모델의 person은 발행되지 않아 미등록, 오면 `UNKNOWN_COLOR`). `class_id`로 키잉하면 `decode_meta.json` 순서가 바뀔 때 조용히 오라벨. (보드 노드에 있던 구 SSD 팔레트 `BOX_COLORS`는 2026-09-03 삭제됐다 — 이제 색 정의는 여기가 유일하다.)
- **라벨**: `"{class_name} {confidence:.2f}"`, 박스 위 채운 배경 + 검은 글자.
- **좌상단 HUD**: `"{N} det  {W}x{H}"` (흰색).
- **원본 해상도에 그린 뒤 `imshow`가 표시** — 창을 리사이즈해 그리면 848/480=1.767 aspect 때문에 y가 틀어진다.

### 4.5 출력 FPS HUD + CSV 로깅 (2026-07-22 추가)

**측정 지점**: `render()`(= `imshow` 1회) **호출 간격**을 wall clock(`time.monotonic()`)으로 잰다. 즉 "화면이 실제로 갱신되는 rate". render는 조인이 완성될 때(≈detection당 1회) 불리므로 **표시 FPS ≈ detection rate ≈ 15–17**.

- **우상단 HUD**: `inst = 1/dt`를 EMA(alpha 0.2, ~0.3 s 시정수)로 평활 → `"{:.1f} FPS"`. `getTextSize`로 폭을 재 **우측 정렬**, 밝은 물체 위에서도 읽히도록 **검은 외곽선 + 초록 글자**. 첫 프레임은 간격이 없어 공백.
- **CSV 로깅(항상 켜짐)**: 저장 위치 `~/pp_ws/viewer_ws/fps_log/`(없으면 자동 생성), 파일명 실행 시각 기반 `fps_YYYYmmdd_HHMMSS.csv`(**덮어쓰지 않음**). 시작 로그에 실제 경로(`FPS log -> …`)를 찍는다.
  - 컬럼: `elapsed_s, inst_fps, ema_fps, n_det`.
  - **성능**: 런타임 중엔 메모리 리스트 `append`만(디스크 I/O 0), **종료 시 `dump_fps_csv()`가 한 번에 flush**. hot path 무영향.
  - **종료 경로**: `q`/`ESC`/`Ctrl-C`는 `finally`를 타 flush됨. `kill -9`(SIGKILL)는 flush 못 함.
  - `dump_fps_csv()`는 요약을 `get_logger()`가 아니라 **`print()`로** 낸다 — Ctrl-C 시점엔 rclpy context가 이미 무효(SIGINT 핸들러가 먼저 내림)라 ROS 로그는 콘솔엔 찍혀도 `/rosout` 발행에 실패해 `publisher's context is invalid` 경고가 뜨기 때문. `print()`는 그 경고를 원천 차단.
  - 디렉토리 생성 실패(권한 등) 시 에러만 남기고 뷰어는 정상 동작(로깅만 비활성).
- 참고: color가 30 fps 유지(§8)라 압축 스트림은 30 Hz지만 render는 detection 게이트(~15 Hz)라 **인코딩 프레임의 절반쯤은 화면에 안 쓰인다** — pick path 신선도를 위해 감수한 트레이드오프.

---

## 5. 배포 (데스크톱 `~/pp_ws/viewer_ws`)

저장소가 **private**이라 데스크톱 `git clone`은 인증(PAT)을 요구한다. 연구실 PC에 PAT를 남기지 않도록 **보드에서 두 패키지만 `scp`**로 받고, `.msg` 무결성은 md5로 보장한다(drift 위험은 §7·§10#1).

```bash
# 데스크톱 — 최초 셋업 (my_interfaces + detection_viewer_pkg)
scp -r ubuntu@192.168.120.50:~/ros2_ws/src/{my_interfaces,detection_viewer_pkg} \
    ~/pp_ws/viewer_ws/src/
cd ~/pp_ws/viewer_ws
colcon build --packages-select my_interfaces detection_viewer_pkg --symlink-install
source install/setup.bash

# .msg 무결성 — 보드와 값이 같아야 한다 (양쪽에서 실행해 대조)
md5sum src/my_interfaces/msg/*.msg
```

뷰어 코드만 바뀌었을 때(예: FPS 기능 추가)는 노드 파일만 갱신:
```bash
scp ubuntu@192.168.120.50:~/ros2_ws/src/detection_viewer_pkg/detection_viewer_pkg/detection_viewer_node.py \
    ~/pp_ws/viewer_ws/src/detection_viewer_pkg/detection_viewer_pkg/detection_viewer_node.py
# 최초 빌드를 --symlink-install로 했으면 재빌드 없이 재실행만 해도 반영됨
```

**env는 아무것도 설정하지 말 것** — Humble 기본값이 이미 맞다(보드도 fastrtps, domain 0). 셋 다 비어 있어야 정상:
```bash
printenv RMW_IMPLEMENTATION ROS_DOMAIN_ID FASTRTPS_DEFAULT_PROFILES_FILE   # 전부 빈 값
```
> ⚠️ **역사적 교훈**: 이 문서의 전임 버전은 "보드=CycloneDDS"라는, 작성 시점엔 맞았지만 실행 시점엔 거짓이 된 전제로 `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`를 지시했다 — **정확히 반대**였다. 보드 RMW를 항상 먼저 확인하라: `grep RMW_IMPL src/system_bringup_pkg/launch/pick_place_vitis_ai.launch.py`(launch가 fastrtps로 pin).

---

## 6. 환경 전제 (검증됨)

| 항목 | 값 | 확인 명령 |
|---|---|---|
| 보드 RMW | `rmw_fastrtps_cpp` (launch가 pin) | `grep RMW_IMPL …/pick_place_vitis_ai.launch.py` |
| 데스크톱 RMW | 설정 안 함 (Humble 기본=fastrtps) | `printenv RMW_IMPLEMENTATION` → 빈 값 |
| ROS_DOMAIN_ID | 양쪽 unset = 0 | `printenv ROS_DOMAIN_ID` → 양쪽 빈 값 |
| 보드 IP | **192.168.120.50/24 on `enxc8a362ec54c4` (USB NIC)** — ⚠️ 2026-07-31 확인. **eth0는 IP 없음**(IgH EtherCAT 점유). 옛 `192.168.120.132/eth0`는 EtherCAT 이관 전 값이니 문서에 남아 있으면 무시할 것 | `ip -br addr` |
| color | 848×480×**30**, `/camera/camera/color/image_raw` | `realsense_pick_place.yaml` |
| detections | `/detections`, ~15 Hz | `vitis_ai_detector.yaml` |
| encoding | `rgb8` (JPEG엔 BGR가 담김 — §7) | `ros2 topic echo …/color/image_raw --field encoding --once` |

---

## 7. 조사로 확정된 비자명 사실 (다시 조사하지 말 것)

### bbox 좌표는 원본 848×480 픽셀 — 변환 금지
worker가 letterbox를 **이미 되돌린다**(`vitis_ai_worker_yolo.py:332-335`, `x1=(cb-pad)/ratio*sx`), 그리고 `send_resized_input: false`라 `sx=sy=1`. node는 좌표를 손대지 않고 그대로 복사. → 뷰어는 받은 숫자를 원본 프레임에 그대로 그린다. **416×416 역변환 금지.**

### stamp 조인은 유효 — header가 바이트 복사본
`vitis_ai_detector_node.py`의 `out_msg.header = msg.header`(대입, 재스탬프 아님). detection stamp == color-frame stamp 바이트 일치 → `(sec, nanosec)` 정확 일치 조인 성립.

### 색 채널 — 변환 코드 넣지 말 것
`compressed_image_transport`는 컬러 입력을 인코딩 전 `bgr8`로 변환하고 `format`에 `"rgb8; jpeg compressed bgr8"`로 기록한다. 즉 JPEG 바이트엔 BGR이 들어있고, `cv2.imdecode(..., IMREAD_COLOR)`가 BGR을 돌려주며, 그게 `cv2.imshow`가 원하는 그대로다. **채널 변환을 넣으면 오히려 반전된다**(오렌지가 파래짐 — 실물로 반증 완료).

### Humble에는 type hash가 없다 → `.msg` drift = 조용한 오염
매칭이 토픽 이름 + 타입 *이름* + QoS로만 이뤄지고 **`.msg` 내용은 비교되지 않는다**. 데스크톱 `.msg`가 한 필드만 달라도 엔드포인트는 정상 매칭되고 **그럴듯한 쓰레기 값**을 역직렬화한다(에러도 no-match도 아님). **최대 위험.** 방어책은 `md5sum src/my_interfaces/msg/*.msg` 양쪽 대조뿐(실측 4/4 일치). `.msg` 4개 전부 가져갈 것.

### DDS는 원격을 막지 않는다 — XML 수정 불필요
`useBuiltinTransports=false`는 기본 **transport**를 끄지 **locator**를 안 끈다. UDPv4가 `userTransports`에 있어 FastDDS가 거기서 metatraffic locator를 파생. **`interfaceWhiteList` 추가 금지** — eth0가 DHCP인데 FastDDS 2.6은 CIDR 미지원이라 IP 하드코딩→lease 바뀌면 조용히 깨짐.

### QoS
`/detections` RELIABLE ↔ 뷰어 RELIABLE. `/compressed` RELIABLE(상속) ↔ 뷰어 BEST_EFFORT(호환).

### ufw는 지금 안 막는다 (단, 지뢰)
`/etc/ufw/ufw.conf` `ENABLED=no`라 룰을 안 건다. `systemctl is-active ufw`가 `active`라고 **거짓말한다**. 단 `DEFAULT_INPUT_POLICY="DROP"`이라 누가 `sudo ufw enable`하면 원격만 전멸(보드는 SHM으로 멀쩡). 진짜 상태는 `sudo iptables -L -n`.

---

## 8. 보드 부하 (as-built)

**launch는 하나다.** 2026-07-20에 뷰잉 전용 launch를 `pick_place_vitis_ai.launch.py`에 흡수하고 `viewing.launch.py`·`realsense_viewing.yaml`을 삭제했다. 근거: 압축 토픽은 플러그인만 있으면 전체 파이프라인에서도 광고되고 **구독 시에만 인코딩**(§3)되므로 뷰잉 전용 launch가 애초에 불필요.

- **color는 30 fps 유지** — 프로덕션 pick path 신선도 우선. (뷰어 표시는 detection 게이트라 어차피 ~15 FPS.)
- 🚨 **launch의 `SetEnvironmentVariable` 2줄(FastDDS SHM 프로파일)은 필수.** 빠지면 보드가 조용히 SHM을 잃고 CPU를 +6.6%p 더 쓴다.
- `ros-humble-compressed-image-transport`(후보 2.5.5) 설치가 전제. 설치 후 **카메라 노드 재시작 필수**(플러그인은 pluginlib ClassLoader 생성 시 한 번만 enumerate). JPEG 품질 조정 파라미터명(실측): `camera.color.image_raw.compressed.jpeg_quality`(**틀리면 조용히 무시** → `ros2 param get`으로 되읽어 확인).

**CPU 실측**:
- 검출 전용 모드(camera+detector만)일 때 뷰어 포함 **0.79 core** — 3D 노드 제외 절감(−0.9 core)이 JPEG 비용(+7.8%p)보다 커서 **순감**. (전체 파이프라인은 ≈1.8 core + 관찰 중에만 lazy JPEG.)
- ⚠️ 수치는 커널에 민감하다. 절대값 비교 말고 **같은 커널에서 뷰어 ON/OFF**를 비교할 것(현 보드 RT 커널 `5.15.199-rt91-rt-kv260c`).

---

## 9. 성능 실측

**2026-07-16 (전 게이트 통과)**

| Gate | 결과 |
|---|---|
| 1 compressed 토픽 | ✅ `/…/color/image_raw/compressed` 생성 |
| 2 `.msg` 무결성 | ✅ md5 4/4 일치 |
| 3 원격 discovery (최초 2-host) | ✅ 데스크톱에서 3토픽 관측 |
| 4 렌더 정확성 | ✅ 5클래스 동시 검출·박스 정렬·라벨 정확 (apple 0.85/orange 0.88/banana 0.85/tennis 0.81/mustard 0.83) |
| 5 보드 부하 | ✅ 0.79 core (순감) |

이 한 장면(apple+banana+orange 동시)이 코드로는 검증 못 하는 4가지를 동시 증명: ① `decode_meta.json` 클래스 순서 정확(라벨 안 뒤바뀜) ② 색 채널 정확(오렌지가 파랗지 않음) ③ bbox 원본 좌표계 맞음 ④ stamp 조인 정확.

**2026-07-22 (FPS 기능 실측)** — 43.0 s / 719행 CSV, **avg 16.7 FPS**. `img≈150, det≈85 /5초` → 압축 스트림 30 Hz, 표시 ~16.7 FPS(detection 게이트). 조인 **100%**(`hit` 지배, `late=0 drop=0 stale=0` — 이 세션은 이미지가 먼저 도착. 07-16엔 `late=5`로 양방향 조인이 실제로 기여).

---

## 10. 함정 목록 (증상이 조용한 순)

| # | 함정 | 증상 | 판별 |
|---|---|---|---|
| 1 | **`.msg` drift** | 에러 없이 그럴듯한 쓰레기 값 | `md5sum msg/*.msg` 양쪽 대조 |
| 2 | 스테일 `FASTRTPS_DEFAULT_PROFILES_FILE` | 에러 1줄 흘러가고 동작, CPU만 +6.6%p | `test -f "$FASTRTPS_DEFAULT_PROFILES_FILE"` |
| 3 | ros2 daemon이 옛 env를 pin | CLI만 오작동, 파이프라인 멀쩡 | daemon environ 확인 → `ros2 daemon stop` |
| 4 | 짧은 discovery window | 토픽이 없는 것처럼 보임 | `--spin-time 8` 주고 **등장**을 확인 |
| 5 | `ufw enable` | 원격만 전멸, 보드는 멀쩡 | `sudo iptables -L -n` |
| 6 | 플러그인 설치 후 카메라 미재시작 | `/compressed`가 안 생김 | Gate 1 |
| 7 | 고아 프로세스 + 삭제된 SHM 세그먼트 | discovery는 되는데 데이터가 안 옴 | `grep -l 'fastrtps.*(deleted)' /proc/*/maps` |
| 8 | per-class NMS | 한 물체에 두 클래스 박스(유령) | 뷰어 버그 아님, `/detections` echo |
| 9 | per-class threshold (person 2.0 = OFF) | person 박스가 안 보임 | 정상 — worker가 person 출력을 차단 |
| 10 | `ros2 run`으로 detector 단독 실행 | `send_resized_input` 기본 True라 정확도만 조용히 하락 | detector는 항상 launch로 |

---

## 11. 연결 문제 분해 (troubleshooting)

**모든 검증을 이름의 *등장*으로 확인. 부재로 추론하지 말 것** — ros2cli 기본 discovery window가 짧아 불완전한 그래프를 정상인 양 반환한다(`--spin-time 8` 주면 찾음).

```bash
# 데스크톱
ros2 topic list --no-daemon --spin-time 8 | grep -E "detections|color/image_raw"
ros2 topic hz /camera/camera/color/image_raw/compressed
ros2 topic echo /detections my_interfaces/msg/DetectionArray --once   # 타입 명시가 가장 날카로운 시험
```

**안 보일 때 분해 순서** (ufw부터 보지 말 것 — 지금 안 막는다):
1. **멀티캐스트** — DDS 배제하고 판정: 보드 `ros2 multicast receive` | 데스크톱 `ros2 multicast send`. 실패면 스위치 IGMP snooping(보드 XML로 못 고침).
2. **domain / RMW** — 양쪽 `printenv ROS_DOMAIN_ID RMW_IMPLEMENTATION` 모두 빈 값인지.
3. **스테일 env** — `test -f "${FASTRTPS_DEFAULT_PROFILES_FILE:-/dev/null}"`.
4. **ufw** — `sudo iptables -L -n`(`systemctl is-active`는 거짓말).

뷰어 자체 진단은 5초마다 찍히는 `report()` 로그를 본다: `no images AND no detections`(discovery), `detections but NO images`(compressed 플러그인/카메라 재시작), `images but NO detections`(`my_interfaces` 커밋 불일치).

---

## 12. UDP 수신 버퍼 (프레임 드롭 시)

`net.core.rmem_max = 212992`(208 KB), MTU 1500. ~37 KB JPEG는 ~25개 데이터그램으로 쪼개지고 **하나만 잃어도 프레임 전체가 날아간다**(BEST_EFFORT라 재전송 없음). 드롭이 보이면 `nstat -az UdpRcvbufErrors`로 확인 후 `rmem_max` 상향.
