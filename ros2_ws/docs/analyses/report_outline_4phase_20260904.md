# 프로젝트 정리 — 분석 · 설계 · 구현 · 검증 (2026-09-04)

> **목적**: HW팀 PPT 뼈대(`09.03.hw팀.pptx` = 분석 → 설계 → 구현 → 검증 4장)의 흐름에 맞춰
> 우리 프로젝트(`~/ros2_ws` perception + `~/RAON-RT-Revision` 로봇 앱 + RT 커널/RPU)를 정리한 것.
> 슬라이드에 그대로 옮길 수 있게 **항목형**으로 쓰고, 수치는 전부 **실측값**
> (`docs/analyses/experiment_environment_survey_20260903.md` §9 대조표 기준 — 문서의 낡은 값 아님).
>
> **정본 관계**: 이 문서는 요약본. 근거·상세는 각 트랙 정본(`docs/RAON-RT/merge.md`,
> `docs/vision/vision_final.md`, `docs/vision/throughput.md`, `docs/rt/rt_final.md`,
> `docs/rpu/rpu_guide_for_claude.md`)이 우선.

**제목 제안**: *Kria KV260 기반 비전–실시간 제어 통합 물체 접근·추종 시스템*
(RealSense → DPU 검출 → 3D → base_link → RAON-RT 1 kHz CST → Indy7)

---

## 0. 한 줄 정의

테이블 위 물체를 카메라로 인식해 3D 위치를 계산하고, Indy7 TCP가 그 물체 **꼭대기 +15 cm 호버 지점으로 원샷 접근**한 뒤, **사람이 물체를 옮기면 TCP가 따라간다(추종)**. 파지·이송·놓기는 범위 밖(그리퍼 하드웨어 미보유). "pick & place"는 초기 프레이밍이라 코드/토픽의 `pick_*` 이름은 legacy.

---

## 1. 분석

### 1.1 과제 정의와 요구사항

| 구분 | 내용 |
|---|---|
| 기능 요구 | ① 물체 **클래스·2D 위치·depth** 검출 ② 카메라 좌표 → 로봇 base 좌표 변환 ③ TCP를 목표 위로 정밀 이동 ④ 움직이는 물체 실시간 추종 |
| 비기능 요구 | 보드 단독 구동(헤드리스), EtherCAT **1 kHz** 주기 보장, 사람 옆에서 동작하는 안전성, 검출 정확도 손실 없는 최적화 |
| 범위 밖 | 파지·이송·놓기(그리퍼 없음), 커스텀 PL 설계(벤더 DPU 오버레이 사용) |
| 목표 변천 | pick & place(계획) → **접근**(2026-07-27, 그리퍼 부재 확정) → **접근 + 추종**(2026-08-03) — 증거 앞에서 목표도 조정 |

### 1.2 플랫폼·제약 분석

| 항목 | 실측 사실 | 파생 제약 |
|---|---|---|
| SoC | Zynq UltraScale+ (K26): **A53 ×4 @ 1.333 GHz 고정**(DVFS 없음), R5F ×2, PL | 비전과 실시간 제어가 **같은 4코어를 경합** → 코어 예산 설계 필요 |
| 메모리 | 4 GiB DDR4, **CMA 1000 MiB**(DPU 버퍼) | RT 커널에서도 CMA 유지 필수(x86 랩 가이드와 다름), RPU carve-out은 CMA 창 밖에 |
| 가속기 | DPU **DPUCZDX8G B3136 @ 300 MHz**(`kv260-smartcam` 오버레이), Vitis AI 2.5 | **DPU 지원 op만** 쓰는 모델이어야 단일 subgraph → 모델 선택의 1순위 기준 |
| 카메라 | RealSense **D435i**(RGB-D, USB3 APU 직결) | depth 필요 → 웹캠 불가; 컬러·depth 센서 15 mm 이격 → 역투영 필요 |
| 네트워크 | 유선 **GEM3(eth0) 1포트뿐** | EtherCAT이 가져가면 원격 접속 단절 → USB NIC로 이전 필요 |
| OS | Ubuntu 22.04.5 + ROS 2 Humble, 순정 커널 PREEMPT_VOLUNTARY/HZ 250 | 1 kHz 제어에 부적합 → RT-PREEMPT 패치 필요 |
| 로봇 | Neuromeka **Indy7** 6-DoF, STEP 컨트롤러 우회 → 드라이브 **EtherCAT 직결**(슬레이브 7: 드라이브 6 + EOAT 1) | CiA402 마스터 스택 직접 구현(IgH + RAON-RT) |

### 1.3 핵심 난제 도출

| 영역 | 난제 | 왜 어려운가 |
|---|---|---|
| 비전 | **sim-to-real 도메인 갭** | 합성 데이터만으로 학습한 클래스(peach·tennis·mustard)가 실물에서 0점 근처 |
| 비전 | A53 **전/후처리 병목** | DPU(~13 ms)보다 Python 전처리(42 ms)·후처리(20 ms)가 느림 — 메모리 대역폭 한계 |
| 비전 | **camera-limited** | 풀프레임 depth 정렬이 코어 1개를 점유해 카메라가 12 Hz로 throttle |
| 비전 | 이동 물체의 **depth-color 시간 불일치** | 검출이 ~90 ms 늦게 도착 → 최신 depth는 3프레임 뒤 → 물체가 비켜난 자리의 배경 깊이를 읽음 |
| RT | 범용 Linux **지터** + 벤더 드라이버와 RT 충돌 | radix-tree(Ubuntu SAUCE revert), zocl KDS use-after-free — RT의 넓은 선점이 잠복 레이스를 노출 |
| 제어 | **IK 브랜치 플립** | RBDL 국소 반복법이라 시드가 곧 브랜치 선택 → 목표 2 mm 차이로 어깨 180° 해로 튐(경계는 32 mm 폭 혼합 지대) |
| 제어 | **stiction(정지 마찰)** | 속도≈0에서 잔차가 남아 Kd 무효, first miss ~40 mm |
| 제어 | 손-눈 **캘리브레이션** + 베이스 물리 이동 | 카메라 좌표를 로봇 좌표로 잇는 TF가 placeholder였음; 사고로 베이스가 밀림(yaw +5.14°) |
| 추종 | 안전 게이트와 추종성의 **상충** | 게이트를 쌓을수록 정상 샘플까지 연쇄 거부 → 자멸 |
| RPU | reserved-memory·GEM3 이관 | 런타임 DT overlay로는 메모리 예약 불가; GEM3는 APU IgH가 실사용 중 |

### 1.4 선행 자산·참고 분석

- **RAON-RT**(KIST AIRI Delgado 2022 / RAIMLAB 명지대 유지보수): App → CRobot(HAL) → EMasterApp(IgH 래퍼) 계층, `RT-POSIX`(SCHED_FIFO + `clock_nanosleep`). x86 기준이라 aarch64 이식·ViSP 제거 필요.
- **IgH EtherCAT Master**: 1.5.2는 5.15 커널에서 빌드 불가 → stable-1.6(1.6.10). macb 네이티브 드라이버 없음 → generic(커널 넷스택 경유, ~1 ms 천장).
- **Vitis AI 2.5** PTQ 플로우(UG1414), `hardswish.py` 배포 버그 2건.
- 랩 x86 RT+IgH 가이드(MAN-20241113-LX02H0001): 큰 흐름만 따르고 Kria 차이(GRUB→flash-kernel, CMA 유지) 반영.

---

## 2. 설계

### 2.1 시스템 아키텍처 — 3계층 · 3통신 경계

```
[PL]  DPU B3136 ──(VART, 완성)──► [APU A53×4, Ubuntu+ROS2 Humble, RT-PREEMPT]
                                    CPU0-2: 비전 파이프라인(SCHED_OTHER) + 전 IRQ
                                    CPU3  : 격리 — RAON-RT 1 kHz 제어 페어(FIFO 97/95)
                                       │ /pick_target_base (base_link, m)
                                       ▼
                                    RAON-RT App/Indy7 ──IgH EtherCAT 1 kHz CST──► Indy7 (7 slaves)
[RPU R5F-0]  FreeRTOS (부팅 검증) ◄──rpmsg(미구현)── APU        SOEM(미구현) ──► GEM3(예약)
```

| 경계 | 상태 |
|---|---|
| PL ↔ APU (DPU 추론) | **완성** |
| APU → RPU (rpmsg 목표 전달) | 미구현 — 현재는 APU가 직접 제어 |
| RPU → Indy7 (SOEM) | 미구현 — 현재는 **APU IgH**가 담당 |

두 저장소는 **ROS2 토픽 하나로만** 결합: `ros2_ws`(perception) → `/pick_target_base` → `RAON-RT-Revision`(제어). 좌표 변환은 전부 perception 쪽에서 완료.

### 2.2 비전 파이프라인 설계

```
realsense2_camera ─► vitis_ai_detector_node ─► pick_logic_node ─► pick_target_3d_node ─► pick_target_base_node
 color/depth 848×480@30    │ JSON + /dev/shm        /pick_target      /pick_target_3d          /pick_target_base
 enable_sync ON            ▼                        (2D 필터·sticky)  (단일점 역투영,          (static TF, base_link)
                     vitis_ai_worker_yolo.py                          nearest_by_stamp)
                     (VART, 별도 프로세스)
```

| 설계 결정 | 근거 |
|---|---|
| **검출 worker를 별도 프로세스로 분리** | rclpy 안에서 VART `execute_async` segfault(XIR graph 객체 GC 수명 문제) → 격리 + 모델 1회 로드 |
| detector↔worker = **model-agnostic JSON contract** | 모델 교체 시 worker 내부만 변경, 노드·하류 무수정 |
| custom msg `Detection/DetectionArray/PickTarget/PickTarget3D` | 노드 경계 = 규격. mock → SSD → YOLO 교체에도 하류 불변 |
| **`align_depth` OFF + 단일점 reverse projection** | 40만 픽셀 정렬해 1픽셀만 쓰는 낭비 제거. epipolar 선분 탐색으로 15 mm 시차 보정 |
| depth **800 ms 히스토리 + capture stamp 최근접 선택** | 이동 물체 대응. sync만 켜면 33 ms, 둘 다 해야 4.3 ms |
| QoS: 이미지 KEEP_LAST 1 · BEST_EFFORT | 최신 프레임만 |
| RMW = **Fast DDS + SHM 16 MB**(XML) | 기본 512 KB로는 1.22 MB Image가 UDP loopback으로 폴백 |
| pick_logic v3: **오퍼레이터 지정 클래스 + 단일 인스턴스 sticky** | 점수식 자동 선택은 추종 중 다른 물체로 미끄러짐 |
| throughput 결정 지점 = **worker 루프 하나** | `max(카메라 33.3 ms, 처리시간, gate)` → 다음 프레임 도착으로 올림 |

### 2.3 검출 모델 설계

| 항목 | 설계 |
|---|---|
| 모델 | **YOLOv3-tiny**, 6-class(apple/orange/banana/tennis_ball/mustard_bottle/person), 416×416 letterbox, 2-head(26²·13²), ≈**8.68 M** 파라미터 |
| 선정 기준 | 정확도보다 **"DPU에서 전부 돌아가는가"** — conv/BN/maxpool/upsample/concat만 사용 → 단일 DPU subgraph |
| 활성화 | SiLU → **Hardswish**(D11: SiLU는 DPU 미지원 op, 재학습으로 mAP 0.758→0.766) |
| 학습 도구 | ultralytics yolov5 v7.0(PyTorch `nn.Module` → vai_q_pytorch 직결; v8은 anchor-free라 부적합) |
| 데이터 | BlenderProc 합성(YCB) + COCO + Open Images + YCB-Video + **YCB real scans**(D13) + **실카메라 apple 54장**(D14). peach 드롭(D13) |
| 양자화 | Vitis AI 2.5 **INT8 PTQ**(calib 500장), `vai_c_xir` B3136 → DPU subgraph 1 |
| 배포 기하 | 회색 optical table, **top-down 0.8 m 고정**(D10) |
| Gate 설계 | Gate 0 op 호환 검사(학습 전) → 2 학습 → 3 양자화 → 4 컴파일 → 5 실물 top-down → 6 풀 파이프라인 → 7 성능 |

### 2.4 실시간 플랫폼 설계

| 항목 | 설계 |
|---|---|
| 커널 | Ubuntu `linux-xilinx-zynqmp` 5.15.0-1070(=5.15.199) + kernel.org **`patch-5.15.197-rt91`** — 벤더 드라이버(zocl 등) 유지하며 RT만 추가 |
| config | PREEMPT_RT, HZ 1000, NO_HZ_FULL, RCU_NOCB/BOOST, IRQ_FORCED_THREADING; **CPU_FREQ/CPU_IDLE/KVM off**, DEBUG 계열 off |
| 코어 배치 | **3+1**: `isolcpus=3 nohz_full=3 rcu_nocbs=3 irqaffinity=0-2` — CPU3 = 1 kHz 페어 전용. 2+2는 비전 2.8코어 시절 기아로 폐기 |
| IRQ | eth0 IRQ는 **CPU1**(의도: 격리 코어를 IRQ로부터 보호, IRQ 스레드 FIFO 50 > 비전) |
| 원격 | GEM3 → **EtherCAT 전용**, 원격은 **USB NIC AX88179B**(192.168.120.50) |
| 부트 | U-Boot + flash-kernel `image.fit`, 순정 커널 공존(복구 경로) |
| 3단계 로드맵 | ① RT-PREEMPT(APU) → ② IgH EtherCAT(APU) → ③ RPU FreeRTOS+SOEM — **불확실성을 하나씩 제거** |

### 2.5 로봇 제어 앱 설계 (RAON-RT-Revision `kv260-merge`, `App/Indy7`)

| 계층 | 설계 |
|---|---|
| 태스크 | TASK0 Main Control **FIFO 97 · 1 ms · CPU3**, TASK1 EtherCAT **FIFO 95 · 1 ms · CPU3**, 키보드/프린터/로거 비RT(CPU0), `mlockall`, 스택 2 MiB |
| 버스 | IgH 1.6.10 generic, `CYCLE_TIME` 1 ms, 7 slaves, PDO 10 entries/드라이브(6040/607A/60FF/6071/6060 ↔ 6041/6064/606C/6077/6061), SM watchdog 100 ms |
| 제어법칙 | **CiA402 CST + RBDL computed-torque**: τ = M(q)(q̈_ref + Kd ė + Kp e) + h + g, KP 800/600/400/200/150/100, KD 80/60/40/28/25/20 |
| 정지 잔차 | **Coulomb 마찰 FF**(KF 0/0/4.0/6.0/3.0/0 Nm) + 속도공간 lag gate("뒤처질 때만 보조") + 50 ms LPF — Ki 미도입(limit-cycle 위험) |
| IK | RBDL IK + **ready-seed**(HOME 기록 자세 = 앵커, 파일 영속) + 2π 폴딩 + **8-seed multi-start** + 브랜치 플립 가드(E27/E28) + tilt 60° 캡 |
| 궤적 | 관절공간 quintic(min-jerk) 3 s, Δq > 2.0 rad면 q_ready 경유 자동 스테이징, refine 최대 6회·tol 12 mm |
| 접근(`v`) | 15프레임 수집 + 축별 std ≤ 8 mm 게이트 → goal = 물체 top + **0.15 m** → IK → quintic → CTC |
| 추종(`o`) v3 | 목표 > 0.15 m: 검증된 접근 leg 자동 발화 / **≤ 0.15 m: 1 kHz DLS 서보**(λ 0.01, EMA 0.4) / 0.25 m 도망 시 제동 → 재leg. 호버 +0.20 m |
| 안전 | **TCP 속도캡 0.20 m/s(비활성화 불가) = 실질 안전장치** + 워크스페이스 박스 + goal z ≥ 0.25 m + 상실 시 TCP 동결(500 ms) + 키 화이트리스트 + URDF effort 토크 백스톱 + NaN 게이트 + 관절 드리프트 예산 1.5 rad + 소프트리밋(fold 후 판정) |
| 인터페이스 | `CROS2PickBridge`(비RT 스레드): 구독 `/pick_target_base`, `/detections`, `/operator_*`; 발행 `/joint_states`, `/robot_state`. seqlock 샘플 파이프로 RT 태스크에 전달 |
| 조작 | `r` enable → `t` CST → `h` servo-on → `p` 메뉴 → 숫자 → `v` 접근 → `o` 추종. `x` soft stop / `e` E-STOP / `b` HOME |

### 2.6 좌표·캘리브레이션 설계

- 변환은 **perception 쪽에서 완료**(`pick_target_base_node` + static TF); RAON-RT는 base_link 좌표를 소비만.
- **Eye-to-hand(AX=ZB)**: AprilTag 36h11 65 mm, 16 poses, `cv2.aruco + solvePnP(IPPE_SQUARE)`(ViSP 바이너리 실행 불가 → OpenCV로 대체, D12). 캘리브 이미지는 파이프라인 토픽에서 캡처(librealsense 직접 열기 금지, D11).
- 베이스 이동 사고(E21) 후 프로브 2점 fit으로 TF 재베이스 → 현행 `base_link→camera_link` t(0.8117, 0.0877, 0.9250), quaternion 표기(pitch 90° 짐벌 회피).

### 2.7 RPU 이관 설계

- 목표: 1 kHz 제어를 R5F로 옮겨 A53 4코어를 비전에 온전히 돌려주고 수 µs 지터 확보.
- Phase 분할: 개발환경 → **Gate 1 FreeRTOS 부팅** → rpmsg → 제어 task 이식(EtherCAT 없는 1 kHz dry-run) → SOEM(GEM3 이관, 사용자 재결정 필요).
- DT: **부팅 시 병합**(`fdtoverlay` → `user-override.dtb`), configfs 런타임 overlay는 reserved-memory 불가라 폐기. carve-out `0x36000000` 4 MiB(관례값 0x3ed00000은 CMA 한복판이라 폐기).

### 2.8 핵심 설계 결정 요약(D-log)

| # | 결정 | 이유 |
|---|---|---|
| V-D2→D12 | 합성 데이터만 → **실패 클래스에 실데이터 추가** | 도메인 갭 실증 후 뒤집음 |
| V-D11 | SiLU → Hardswish 재학습 | DPU 미지원 op |
| V-D13 | peach **드롭**, 6-class + YCB real scans | 실물과 mesh 외형 상이; 없앨 수 있는 문제는 없앰 |
| V-D14 | apple만 실촬영 54장 타깃 재학습 | 색 도메인 갭(양 문제 아님) |
| C-D1 | CST 토크 + RBDL(CSP 직결 계획 폐기) | 원저 컨트롤러 재사용, NRMK 토크변환 정상 |
| C-D2 | 독립 make + ROS2 브릿지(colcon 밖) | ros2_ws 무수정 |
| C-D5 | sim-first 철회 → **서보-오프 통합 런** | 모션 0으로 와이어까지 검증 |
| C-D9/D10 | 1 ms 유지 · 3+1 격리 | GEM 1 ms 실측 안정; RT-POSIX CPU0 기본 pin → cfg `CPU=` 키 |
| C-D13 | ready-seed IK | 시드 = 브랜치 선택 장치 → 결정적 해 |
| C-D14 | 마찰 FF + 게이트 2종(Ki 없음) | 실패 지점이 속도≈0 |
| C-D15 | 추종 = 사장된 `SetTargetPose_Jacobian` 서보 재배선 | 재계획식은 속도 불연속; v2에서 게이트 연쇄 삭제 → 속도캡 중심 |

---

## 3. 구현

### 3.1 비전 파이프라인 (`~/ros2_ws`)

| 패키지 | 구현 |
|---|---|
| `system_bringup_pkg` | 단일 launch `pick_place_vitis_ai.launch.py`(7 프로세스), 노드별 YAML(symlink, rebuild 불필요), `fastdds_shm_profile.xml`, static TF |
| `vitis_ai_detector_pkg` | `vitis_ai_detector_node.py`(1,032줄) + `vitis_ai_worker_yolo.py`(467줄, xir/vart만) + `shm_frame.py`; xmodel·`decode_meta.json` 동봉 |
| `pick_logic_pkg` | v3: `desired_class` 필수, 최고 conf 인스턴스 채택 → 최근접 승계 → 상실까지 고정 |
| `target_3d_pkg` | `pick_target_3d_node.py`(epipolar 역투영 numpy 벡터화, 9×9 median, depth 히스토리), `pick_target_base_node.py`(TF (R,t) 캐시, z 게이트 0.20–0.95 m) |
| `my_interfaces` | Detection / DetectionArray / PickTarget / PickTarget3D |
| 데스크톱 | `detection_viewer_pkg`(압축 토픽 lazy JPEG, 보드는 그리지 않음), `raon_operator_gui`(Qt) |

**구현된 최적화(전부 무손실 검증)**
1. 카메라 1280×720 → **848×480**(busy-spin stall 제거)
2. 전처리 **LUT**(42 → 12 ms, bit-identical)
3. 후처리 **objectness 사전필터**(20 → 7 ms, 필요조건 유도 + 마진, 결과 동일)
4. 콜백 파이프라이닝(latency 292 → 261 ms)
5. **align 제거 + 단일점 역투영**(13 → 17 Hz, 코어 0.65 회수)
6. CPU 절감 phase 1+2: TF 캐시·epipolar 벡터화·**FastDDS+SHM**(UDP loopback 제거) → 76.8 → ~44 %(−1.25 코어)
7. 이동 물체 전환: `enable_sync` + depth 30 fps + `nearest_by_stamp`; gate 0.045 → **0.030** + worker IPC pipe → **/dev/shm mmap**(13.4 → 4.65 ms)

### 3.2 모델 학습·양자화·컴파일 (데스크톱 RTX 4060)

- 스크립트 `yolo_v3_tiny_training/scripts/01~13`: 데이터 병합/remap → **12a Gate 0 inspect** → 11 학습(150 ep, batch 64, SGD, hsv_v 0.5, mosaic) → 12 양자화(docker `vitis-ai-cpu:2.5.0.1260`, `hardswish.py` sed 자동 패치) → 13 컴파일(`arch_b3136.json`) → scp 보드.
- 배포 xmodel `yolov3_tiny_7class.xmodel`(9.31 MB, md5 `9bc6520c…`, 내용은 6-class D14). 보드 격리 테스트 `yolov3_tiny_image_test.py`, 단일물체 auto-label `autolabel_single_object.py`.
- 곁가지: 데스크톱 `nvidia-driver-595-open` CUDA hard-freeze → 580 proprietary 롤백 + hold.

### 3.3 RT 커널

- x86 크로스빌드(`aarch64-linux-gnu-gcc 11.4`, `bindeb-pkg`) → DTB 복사 → `dpkg -i` + `flash-kernel`.
- 소스 패치 3종: ① **radix-tree local_lock 복원**(Ubuntu SAUCE revert가 RT에서 idr_preload를 atomic화 → 위반 253 → 0) ② **zocl `kds_core.c` UAF 픽스**(타임스탬프를 submit 앞으로, 3곳) ③ PREEMPT_LAZY off.
- 최종 `5.15.199-rt91-rt-kv260c` **#11**(= #10 + `AX88179_178A=m`). 순정 커널·이전 빌드 공존, `recovery/`에 deb.
- 부가: 헤드리스 전환, DP/Mali blacklist, ASIX `ax_usb_nic` 4.1.0 크로스컴파일 + udev로 cdc_ncm 차단.

### 3.4 EtherCAT + 제어 앱 (`~/RAON-RT-Revision`, 브랜치 `kv260-merge`)

| 단계 | 구현 |
|---|---|
| Phase 0 기반 | RT-POSIX aarch64 빌드(stack-min 패치), EMasterApp aarch64, RBDL 3.1.3(rbdl-orb + urdfreader), `/opt` 배치, limits.d rtprio 98 |
| IgH | 보드 네이티브 빌드 `--enable-generic --enable-hrtimer`, `/opt/etherlab`, udev `/dev/EtherCAT0` |
| Phase 2a | **ViSP 제거** 앱 빌드(`Indy7Ctrl.out`), 가이드 §14.2 차단요소 픽스, PDO 픽스 |
| Gate 2b | `ROS2PickBridge.cpp`(rclcpp), 대화형 메뉴 `p`/숫자/`v`, `desired_class` param 왕복 |
| Phase 4~5 | 서보 arm/disarm 인터록, HOME 기록/복귀, 워크스페이스 박스, refine, E13 안전 IK(soft-R + Δq 게이트) |
| 정밀화 | ready-seed IK v1→v3 → reach map(`tools/reachmap.cpp`, 컨트롤러 `.o` 링크) → 다중 시드 IK → 마찰 FF → refine scope gate → lag gate |
| 추종 | 안전 백스톱(`9204372`: NaN 게이트·토크 새추레이션·CTC 스냅샷) → `o` v1(`4efb05d`) → v2 단순화(`2993804`) → **v3 하이브리드**(`8d64868`·`25bc044`·`38a787a`: StartTrackLeg, 속도게이트 램프, FF 50 ms LPF, 드리프트 예산) |
| 운영 | `run.sh`(rtprio/memlock 검사, **E34 대책** `ecat-op-fifo` = IgH `EtherCAT-OP` 스레드를 FIFO 40으로 자동 승격), cfg `[TASKn] CPU=` 키, `HOMING_METHOD=3`(E24) |
| 오퍼레이터 콘솔 | `/operator_key|gains|jog` 원격 채널, 정지키 공통 근원 `KillMotionSources()`(`4b2113a`, 실기 미검증) |

### 3.5 캘리브레이션·계측 도구

- `tools/calib_grab.py`(파이프라인 토픽 캡처, 앱 `s`와 lockstep) → `calib_cpo.py`(태그 pose) → 원저 `eye_to_hand_calib.py` → `base_shift_fit.py`(E21 재베이스).
- `approach_plot.py`(완주마다 CSV 1행 + 3D/실척 PNG 자동), `DataRecorder`(1접근 = DataLog 1파일) + `fk_replay`(3계열 IK입력/FK(q_ref)/FK(q_act), RT 신규비용 0), `gate0_rtposix_test.c`, `test_gate2b_bridge.py`, `test_phase3_smoke.py`.
- ros2_ws `tools/metrics/`(300 s 리소스 샘플러·perf join·plot), `tools/rt/`(cyclictest·soak), `tools/board_backup.sh`.

### 3.6 RPU (Gate 1)

- Vitis 2022.1(Windows) BSP `freertos10_xilinx`, `r5_freertos.elf`(505 KB, LED 태스크 2개).
- `rpu_v2.dtso` → `fdtoverlay` 병합 → `/boot/firmware/user-override.dtb`(U-Boot `boot.scr` 훅), `rproc@36000000` 4 MiB no-map + TCM.
- `remoteproc0`로 로드 → 커널 로그 `r5f_0 is now up`. resource table·vring·IPI는 **미구현**(rpmsg 선행 조건).

### 3.7 저장소

`~/ros2_ws`(GitHub `Capstone_Design` main) · `~/RAON-RT-Revision`(`kv260-merge`, 태그 `pre-cleanup-20260731`) · `~/RAON-RT`(랩 원본, 읽기 전용) · `~/RT-POSIX` · `~/ethercat` 1.6.10 · `~/rbdl-orb`. 증거는 `ros2_ws/evidence/{crash_logs,kernel_configs,metrics/runs,approach_accuracy,reachmap}`.

---

## 4. 검증

### 4.1 비전 — 모델

| Gate | 결과 |
|---|---|
| Gate 0 op 호환 | Hardswish 전환 후 unknown-op 0, VAIQ_WARN 0 |
| Gate 2 학습(float mAP@0.5) | 7-class SiLU 0.758 → hswish **0.766** → 6-class D13 0.748 → 배포 D14 **0.728**(apple 0.625 / mustard 0.933 / tennis 0.950) |
| Gate 3 양자화 | cosine head0/1 0.9923/0.9849(7-cls) → 0.9757/0.9615(D14), VAIQ_WARN 0 |
| Gate 4 컴파일 | **DPU subgraph 1**, 전 연산자 DPU, fingerprint 일치 |
| Gate 5 실물 top-down(thr 0.50) | **apple 0.88 / orange 0.85 / banana 0.78 / mustard 0.81 / tennis 0.68** — 6종 확정. (D13 전: mustard 0.02, apple 0.26) |
| 미측정 | **INT8 mAP** — cosine + 실물 Gate 5로 대체 |

### 4.2 비전 — 파이프라인 성능·무손실

| 항목 | 결과 |
|---|---|
| 채택 런 `shm_gate030_20260805`(300 s, 6,343 프레임) | **21.7 Hz**, capture→base E2E **p50 118 / p95 143 / p99 161 ms**, **CPU 1.60 코어**, processing 43 ms(pre 11.6 + DPU 17.4 + post 4.8), IPC 4.65 ms |
| 이력 | 07-15: 15 Hz / CPU 76.8 → ~44 % · 07-31: 17.0 Hz / 1.31 코어 · SDK align 대안 14.2 Hz / 355 ms → 기각 |
| depth-color skew | 28.5 → **4.3 ms**(89 % 정확히 0), +0.18 코어 |
| 무손실 검증 | LUT bit-identical, 사전필터 무작위 30회 동일, epipolar A/B 4,500회 mismatch 0, TF 캐시 오차 0, depth view bit-identical |
| 3D 정확도 | 시뮬레이션 0 px, z 자 실측 일치; apple base 좌표 z = 테이블 + 높이와 **4 mm** 일치 |
| pick_logic v3 | 합성 테스트 6/6(→ person 제거 후 4/4) |

### 4.3 실시간 커널

| 시험 | 결과 |
|---|---|
| cyclictest(kv260c, 4코어 stress-ng+hackbench 공유) | IDLE 60 s Max 112/84/105/134 µs · **LOADED 120 s Max 142/137/127/136 µs**, 커널 위반 0(DEBUG-on 빌드는 189–282 µs) |
| soak(DEBUG_PREEMPT/ATOMIC_SLEEP 무장, 840 s) | 위반 **0** |
| radix 픽스 | 부팅 위반 253 → **0** |
| zocl 픽스 | `slub_debug` 계측 330 s Poison 0 + 프로덕션 200 s+ Oops 0(픽스 전 ~30 s 크래시) |
| gate0 격리 A/B(RT-POSIX 1 kHz, FIFO 80, 비전 가동) | **CPU3 avg 4.3 / max 51.1 µs** vs CPU0 avg 7.3 / max 114.6 µs |
| 실운전 주기(DataLog, 토크모드 풀스택) | 23,999 주기 **mean 1000.00 / min 915 / max 1086 µs, >1.1 ms 0건** |
| 미측정 | 격리 CPU3 **운영조건**(비전+DPU+EtherCAT) cyclictest |

### 4.4 EtherCAT·제어 앱 — Phase/Gate

| Gate | 결과 |
|---|---|
| Gate 0 RT 지터(1 ms × 5000) | avg 1.3 µs / max +35 µs |
| Gate 1 버스 스캔 | 7/7 슬레이브 cfg 일치, PDO 10엔트리 **비트단위 일치**, Lost 0 |
| Gate 2a/2b | aarch64 빌드 OK · 브릿지 합성 5/5 ×2, param 왕복, std 게이트 |
| Phase 3 서보-오프 통합 런 | **300 s 무단절 OP, 정상상태 Lost 0**, 파이프라인 동시 구동, PDO=SDO 조인트 대조 일치 |
| Phase 4 서보온 | 6축 0x0237 OPERATION ENABLED, grav-comp 34 s·컴플라이언스 확인, E-stop 실효(버스 전원 차단→복구 fault 0), `a` 실모션(초기 오차 ~16 cm → refine 과제) |
| refine | **160 → 10.9 mm**(4 pass 단조수렴) |
| Phase 5 캘리브 | 16 poses, reproj 0.12–0.55 px, **residual 1.81 cm / 1.66°**; 손-프로브와 9 cm 정합 |
| Phase 5 접근 데모 | **전 5클래스 접근 성공**(접근 → HOME → 다음 물체 사이클), refine 4.9–10.6 mm |
| ready-seed / 다중 시드 IK | reach map 커버율 **95.1 → 99.4 %**, BRANCH 1,345 → 14; 지도 예측이 실기와 전 필드 일치 |
| 접근 정확도(자동 CSV) | 첫 10건 3.07–11.52 mm, 오차가 물체별로 방향까지 재현(= 보정 가능) |
| **원샷 목표** | #52–58 **7/7 refine 0회**(HOME 5물체 6.9–10.7 mm + 물체→물체 11.3 mm), #78–82 5/5(orange 6.15 mm), **2026-08-05 4건 4.21–5.93 mm**; KF=0 역검증 111 mm(마찰 인과 확증) |
| E34 대책 | EtherCAT-OP FIFO 40 후 갭 0 %, #41–46 전원 합격 |

### 4.5 RPU

- Gate 1: `remote processor ff9a0000.rf5ss:r5f_0 is now up` **2026-09-02 2회**, LED 태스크 실행(점멸은 육안 확인 — `/sys/class/leds` 미노출로 보드 재검증 불가).

### 4.6 미검증·한계 (보고서에 정직하게)

| 항목 | 상태 |
|---|---|
| 추종 `o` v3 하이브리드 | **구현 완료·실기 미검증** — 검증 사다리 8단계(정적 호버 → 상실 → 저속 → **속도캡 증명** → 경계 → 폴트 → 핸드헬드) 대기. 캡 증명 전 물체를 손으로 들지 않음 |
| 오퍼레이터 콘솔 | 실기 미검증 |
| INT8 mAP | 미측정(cosine + 실물 Gate 5로 대체) |
| 격리 CPU3 운영조건 cyclictest | 미측정(gate0 51.1 µs만 존재) |
| 충돌 모델 | 없음 — reach map PASS ≠ 안전 |
| IK 실행 시간 | RT 루프 안 최대 **58 ms**(워치독 100 ms 안이라 Lost 0이지만 off-RT 이전이 숙제) |
| 접근 오차 | −x/−z 바이어스 지배 → <5 mm는 KF_2·바이어스 보정 지렛대(그리퍼 없어 실익 보류) |
| RPU | rpmsg·제어 task 이식·SOEM 3과제 잔존; SOEM은 GEM3 이관이라 사용자 재결정 필요 |

### 4.7 주요 사고와 교훈(E-log 발췌)

| # | 사건 | 교훈 |
|---|---|---|
| E13 | 첫 접근에서 테이블 충돌 | 점-구속 IK nullspace 방랑 + 무게이트 quintic → soft-R + Δq 게이트 + T 스케일링 |
| E21 | 베이스 물리 이동(yaw +5.14°) | 캘리브는 1회성이 아님 — 프로브 fit으로 재베이스 |
| E24 | `HOMING_METHOD=1`이 cfg 값을 시작 각도로 디코드 → 손목 폭주 | pose 출력은 FK(m_Q)라 자기 0점 검증 불가 → 엔코더 카운트로 검증, `=3` 전환 |
| E27/E28/E29 | 브랜치 플립, polish 단 우회, 경계는 32 mm 혼합 지대 | 솔버를 링크한 reach map으로 사전 예측 |
| E34 | 간헐 RT 스톨 = IgH 스레드 priority inversion(PI 없는 semaphore + nice 0) | EtherCAT-OP를 FIFO 40으로, 실기 중 무거운 작업 금지 |
| E37 | lag gate 우회 → bang-bang FF + 테이블 다이브 | 게이트의 원리는 하중부 — 우회 금지, 측정 공간만 모드에 맞춤 |
| 추종 v1 | 게이트 연쇄가 추종을 자멸시킴 | 안전은 게이트 개수가 아니라 **속도캡 한 줄** |

---

## 부록 A. 보고서 수치 대조 주의

문서에 남은 낡은 값 → 실측: 커널 #10→**#11**, IgH 1.6.2→**1.6.10**, FW 5.16.0.1→**5.17.0.10**, BGR8→**RGB8**, CSP→**CST**, IP .132→**.50**, 파라미터 6.9 M→**8.68 M**, 15 Hz/81 ms→**21.7 Hz/118 ms**(정의 다름), depth 15 fps→**30 fps + sync**. 전체 표 = `experiment_environment_survey_20260903.md` §9.

## 부록 B. 슬라이드 배분 제안(4장 기준)

| 슬라이드 | 담을 것 |
|---|---|
| 분석 | §1.1 요구/범위 + §1.2 제약 표 + §1.3 난제 4~5개 |
| 설계 | §2.1 아키텍처 그림 + §2.2/2.5 파이프라인·제어 다이어그램 + §2.8 결정 5개 |
| 구현 | §3.1 최적화 7개 + §3.4 단계 표 + §3.3 커널 패치 3종 |
| 검증 | §4.2 성능 표 + §4.3 지터 표 + §4.4 원샷 정확도 + §4.6 미검증 |
