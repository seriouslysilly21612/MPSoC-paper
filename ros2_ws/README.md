# Kria KV260 Pick & Place — Perception Pipeline

RealSense D435i → DPU 물체검출(YOLOv3-tiny 6-class) → 2D 필터 → 단일점 3D 역투영 → base frame 목표점.
ROS 2 Humble, 21.7 Hz, capture→base E2E p50 ~118 ms (2026-08-05 이동 물체 전환 이후).

이 목표점을 로봇 제어 앱(`~/RAON-RT-Revision`, branch `kv260-merge`)이 받아 **Indy7 TCP를 테이블 위 물체 바로 위(+15 cm)로 접근**시키고, **사람이 물체를 옮기면 TCP가 따라갑니다(추종)**. 파지·이송·놓기는 범위 밖입니다(그리퍼 하드웨어 미보유). 제목과 `pick_*` 런치·노드·토픽 이름은 초기 pick&place 프레이밍의 **legacy**이며, 바꾸지 않습니다.

```
[D435i] → camera → vitis_ai_detector_node ─(pipe)→ vitis_ai_worker_yolo (DPU)
                          │ /detections
                          ▼
                     pick_logic_node → /pick_target
                          ▼
                   pick_target_3d_node → /pick_target_3d
                          ▼
                  pick_target_base_node → /pick_target_base   ← 최종 출력
```

---

## 빌드 (최초 1회)

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws && colcon build --symlink-install    # --symlink-install 필수
source install/setup.bash
```

## 실행

파이프라인은 **launch 하나**입니다. 보고 싶을 때 데스크톱 뷰어를 붙이면 되고, 별도의 "뷰잉 전용" launch는 없습니다(2026-07-20 통합).

**보드** (항상 이거 하나):
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py
```
카메라 → 검출 → pick_logic → 3D → base까지 전부 돕니다. 데스크톱: **할 것 없음.**

확인 (다른 터미널):
```bash
ros2 topic hz /detections        # ~21.7 Hz
ros2 topic hz /pick_target_base  # 최종 단계까지 흐르는지
```

### 데스크톱에서 bbox overlay로 관찰하기 (선택)

파이프라인을 **바꾸지 않습니다.** 위 launch를 보드에서 그대로 돌린 채, 데스크톱에서 뷰어만 붙입니다:
```bash
source /opt/ros/humble/setup.bash && source ~/pp_ws/viewer_ws/install/setup.bash
ros2 run detection_viewer_pkg detection_viewer_node
```
`q` 또는 `ESC`로 종료. 설정할 환경변수는 **없습니다** (양쪽 기본값이 이미 맞음).

- 보드는 **JPEG 압축만**, 그리기는 전부 데스크톱에서 합니다.
- 압축 스트림 `/camera/camera/color/image_raw/compressed`는 **뷰어가 붙을 때만 인코딩**됩니다(image_transport lazy). 아무도 안 보면 인코딩 비용 0 → 평상시 파이프라인엔 **부담이 없습니다.** 그래서 뷰잉 전용 launch가 애초에 불필요했습니다.
- 보는 동안엔 color 30 fps 인코딩이 detection(21.7 Hz)보다 빨라 일부 프레임은 버려지지만, 이건 **관찰하는 동안만의 비용**이고 뷰어를 끄면 사라집니다. (목표점 신선도를 위해 color는 30 fps로 둡니다 — 이동 물체 추종에서 더 중요해졌습니다.)

**전제** (최초 1회): 보드에 `sudo apt install ros-humble-compressed-image-transport`, 데스크톱에 `my_interfaces` + `detection_viewer_pkg` 빌드. 전체 절차·게이트·함정: `docs/vision/desktop_viewer_plan.md`

> `build/`와 `install/`은 `--symlink-install`로 서로 묶여 있습니다. **하나만 지우면 링크가 깨집니다** — 지울 때는 항상 둘 다 지우고 재빌드하세요.

---

## 전제조건 (git으로 받아지지 않는 것들)

이 저장소에는 **우리 코드만** 들어 있습니다. 아래는 보드에 미리 갖춰져 있어야 합니다.

| # | 항목 | 버전 | 확인 명령 |
|---|---|---|---|
| 1 | Kria KV260 + Ubuntu | 22.04 | `uname -a` |
| 2 | ROS 2 | Humble | `printenv ROS_DISTRO` |
| 3 | DPU (`kv260-smartcam` overlay) | B3136 | `xdputil query` |
| 4 | Vitis-AI runtime (VART/XIR) | 2.5.0 | `python3 -c "import vart, xir"` |
| 5 | RealSense | librealsense 2.57.7 | `rs-enumerate-devices` |
| 6 | D435i 펌웨어 | **5.17.0.10** | `rs-enumerate-devices \| grep Firmware` |
| 7 | realsense-ros | 4.57.7 | `ros2 pkg prefix realsense2_camera` |

### 1. DPU 활성화

`kv260-smartcam` overlay가 부팅 시 자동 로드되어야 합니다.

```bash
sudo systemctl enable --now kv260-smartcam
xdputil query        # DPUCZDX8G_ISA1_B3136 / fingerprint 0x101000016010406 확인
```

fingerprint가 다르면 이 모델(xmodel)은 **로드되지 않습니다**. 모델은 이 DPU에 맞춰 컴파일된 것이라 fingerprint가 일치해야 합니다.

### 2. Vitis-AI runtime

`import vart, xir`이 되면 끝입니다. 보드 이미지에 포함돼 있거나 Xilinx 패키지로 설치합니다.
(모델을 새로 만들 게 아니라면 **데스크톱 Vitis-AI 2.5 docker는 필요 없습니다.** 재학습/재컴파일 시에만 필요 — `yolo_v3_tiny_training/README.md` 참고.)

### 3. RealSense

```bash
sudo apt install ros-humble-librealsense2 ros-humble-realsense2-camera
```

**펌웨어 5.16.0.1은 쓰지 마세요.** 스트리밍 수십 초 후 RGB가 멈추는 버그가 있습니다(원인·진단법: `docs/history.md` §13). 5.17.0.10으로 업데이트:

```bash
wget https://librealsense.intel.com/Releases/RS4xx/FW/D4XX_FW_Image-5.17.0.10.bin
rs-fw-update -f D4XX_FW_Image-5.17.0.10.bin
```

### 4. realsense-ros — **clone하지 않아도 됩니다**

3번의 `apt install`로 끝입니다. `src/realsense-ros`가 있어도 **`COLCON_IGNORE`가 있어 빌드되지 않으며**(`install/`에 없음), 실제로 돌아가는 건 apt 패키지 `ros-humble-realsense2-camera` 4.57.7입니다. 소스 트리는 **읽기용 참조**일 뿐이라 clone은 선택입니다(코드를 들여다볼 때만).

> ⚠️ 그래서 `src/realsense-ros`를 **고쳐도 런타임은 안 바뀝니다.** `colcon build`는 성공했다고 말하지만 그 트리는 애초에 빌드 대상이 아닙니다.

### 5. DDS 설정 — **할 일 없음** (2026-07-15부터 자동)

노드 간 1.16 MB Image를 UDP loopback이 아니라 **shared memory**로 보냅니다. 이 배선은 launch 파일이 직접 합니다(`RMW_IMPLEMENTATION` + `FASTRTPS_DEFAULT_PROFILES_FILE`을 `SetEnvironmentVariable`로 주입, XML은 `system_bringup_pkg/config/fastdds_shm_profile.xml`에서 `FindPackageShare`로 해석). **`~/.bashrc`에 아무것도 넣지 마세요** — 셸이 launch를 이기고, 절대경로는 다른 머신에서 깨집니다.

동작 확인 (파이프라인 가동 중):
```bash
ls -la /dev/shm/ | awk '$5 > 16000000'   # 16 MB 세그먼트가 노드 수(6개)만큼 보이면 정상
```

왜 16 MB인가: 기본 SHM segment가 512 KB라 1.16 MB Image가 안 들어가고 **에러 없이 조용히 UDP로 폴백**합니다(CPU +6.6%p). 그래서 프로파일 XML이 필요합니다.

### 6. RT 커널 (선택 — EtherCAT 로봇제어 단계에서만)

비전 파이프라인만 돌릴 거면 **순정 커널로 충분합니다.** EtherCAT 통합 시에는 RT 커널이 필요하고, 직접 빌드해야 합니다:

- 최종 커널: `5.15.199-rt91-rt-kv260c`
- 필수 패치 2개가 이 저장소에 있습니다:
  - `tools/kernel_patches/radix-fix/` — Ubuntu SAUCE radix-tree revert 되돌리기 (없으면 RT에서 부팅 중 위반 253건)
  - `tools/kernel_patches/zocl/apply_zocl_uaf_fix.py` — zocl KDS use-after-free (없으면 DPU 가동 ~30초 후 커널 crash)
- 전체 절차: `docs/rt/rt_patch.md`, 사건 기록: `docs/rt/rt_kernel_postmortem.md`
- 검증 하네스: `tools/rt/` (`cyclic_rt.sh` 지연 측정, `soak_rt.sh` 소크 테스트 → 결과는 `evidence/crash_logs/`)

---

## 저장소 구조

```
README.md          ← 여기
CLAUDE.md          에이전트용 프로젝트 컨텍스트
src/               ROS 2 패키지 (colcon 규약상 위치 고정)
docs/              문서 — STATUS.md가 허브
tools/             실행하는 것 (rt/ 측정·소크, kernel_patches/ 커널 패치)
evidence/          들여다보는 것 (crash_logs/ metrics/ kernel_configs/ node_graph/)
yolo_v3_tiny_training/   모델 재생산 경로 (학습→양자화→컴파일)
```

> ⚠️ `docs/` 안 문서들의 **본문에 적힌 경로는 개편 전 기준**입니다(예: `crash_logs/…`는 지금 `evidence/crash_logs/…`). 위 표에서 파일 위치를 찾으세요. 문서 본문은 히스토리 기록이라 일부러 손대지 않았습니다.

| 문서 | 내용 |
|---|---|
| `docs/STATUS.md` | **여기부터** — 통합 허브 + 정본 라우팅 표 |
| `docs/vision/workflow.md` | 노드별 파라미터와 **그 값의 근거** |
| `docs/vision/detector_worker_walkthrough.md` | detector node + worker **코드 정독 지도** (공부용) |
| `docs/vision/vision_final.md` | 비전 모델 전체 (SSD→YOLO 학습·DPU 배포·최적화) |
| `docs/rt/rt_patch.md` / `docs/rt/rt_kernel_postmortem.md` | RT 커널 구축 / 크래시 규명 |
| `docs/vision/desktop_viewer_plan.md` | 데스크톱 bbox 뷰어 (✅ 완료) |
| `docs/history.md` | 시간순 전체 히스토리 |
| `docs/onboarding.md` | 처음 붙는 사람용 안내 |
| `docs/reference/` | 주제별 공식문서 링크 모음 |

---

## 알아둘 것

- **모델**: `src/vitis_ai_detector_pkg/models/yolov3_tiny_7class.xmodel` — 파일명은 `7class`지만 실제는 **6-class**(apple/orange/banana/tennis_ball/mustard_bottle/person). `decode_meta.json`과 **반드시 같은 디렉터리**에 있어야 합니다(worker가 xmodel 옆에서 자동 탐색). 모델이 패키지 안에 동봉돼 있어 launch가 `FindPackageShare`로 찾습니다 — **경로를 손볼 필요가 없습니다.**
- **`base_link → camera_link` TF는 실측 캘리브 값**입니다(eye-to-hand, 2026-07-27, 16 poses, AX=ZB, residual 1.8 cm / 1.7°, + E21 base 재베이스 yaw +5.14° 반영 — launch 파일 `base_to_camera_tf` 블록 주석에 근거). `/pick_target_base`는 실제 로봇 좌표이며 접근·추종이 이 값 위에서 실기 동작합니다.
- **rclpy에서 노드를 병합하지 마세요.** 시도했다가 CPU +5.4pt 역효과를 실측했습니다(rclpy executor가 매 콜백마다 waitset을 재구성). 근거와 코드: `target_3d_pkg/pick_post_stack.py`.
