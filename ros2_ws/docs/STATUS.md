# Kria KV260 Pick & Place — 통합 진행 문서 (Integrated Progress)

> **이 문서의 목적**: 여러 세션에 흩어진 프로젝트 문서(진행 히스토리·파이프라인 상세·비전 교체 계획·RT 커널·RPU/EtherCAT 계획)를 **하나로 통합**해, 이 파일만으로 전체 상태를 파악하고 어느 트랙이든 이어서 작업할 수 있게 한다.
>
> **작성**: 2026-07-09 (통합) · **원본 문서 최종 갱신**: 2026-07-13 (RT 크래시 규명·해결 + 소크 통과 → 비전 파이프라인 최적화 세션으로 전환)
>
> **통합한 원본 문서** (각 주제의 정본은 아래 §2 문서 지도 참조):
> `progress.md`, `inst_claude.md`, `workflow.md`, `yolov3_tiny_execution_plan.md`, `rt_patch.md`, **`rt_kernel_postmortem.md`(RT 크래시 종합 보고서)**, `rt_kernel_fix_plan.md`, `rpu_plan.md`, `rpu_guide_for_claude.md`, 메모리 `yolov3-vision-swap-resume.md`, `CLAUDE.md`.
>
> **주의**: 이 통합본은 스냅샷이다. 실제 명령어·게이트 세부·최신 결정은 각 트랙의 **정본 문서**가 우선한다(§2). 상태 값이 원본과 다르면 원본을 신뢰하고 이 문서를 갱신할 것.

---

## 0. 30초 요약 (현재 상태 · 2026-07-14)

- **프로젝트**: Kria KV260 기반 **물체 접근 + 추종 시스템**. `RealSense → 비전 검출(DPU) → 2D 필터 → 단일점 3D → base_link 좌표 → RAON-RT 제어 앱(APU IgH EtherCAT) → Indy7 실기 구동`. 타깃 로봇 = Neuromeka **Indy7**. ⚠️ **"pick & place"는 초기 프레이밍이고 지금은 아니다** — 파지·이송·놓기는 하지 않는다(그리퍼 하드웨어 미보유). 이 문서 제목과 코드·런치·토픽 이름(`pick_place_vitis_ai.launch.py`, `pick_logic_pkg`, `/pick_target_base` …)은 전부 그 시절 **legacy이며 바꾸지 않는다**. 앱 정의는 §1.1, 상세는 ★ 2026-09-04 항목.
- **동시 진행 중인 트랙 2개** (리소스 충돌 없음 — DPU/카메라 vs 커널/GEM):
  1. **비전 (YOLO 교체)** — SSD ADAS stand-in → **YOLOv3-tiny 6-class**. 재학습·양자화·컴파일·보드 배치·**Gate 5(실물 top-down) 통과 = 6종 확정**. D14 apple 실물 재학습으로 **실물 apple 0.5→0.88**까지 해결 → **모델 교체(Gate 2~5) 완성**. Gate 6·7도 통과해 현재 **21.7 Hz · true E2E p50 ~118 ms**로 실운용 중. ← §4
     - **★ 2026-09-08 다음 목표 결정 — 주류 모델을 YOLOv5 계열로 교체**: YOLOv5n을 1순위로 재학습해
       주류 검출기로 올리고, **현행 YOLOv3-tiny 6-class는 삭제하지 않고 legacy로 남겨 A/B 비교**에 쓴다.
       전환의 실제 근거는 정확도가 아니라 **stride 8 head**(0.8 m top-down에서 물체가 letterbox 28 px뿐이라
       tiny의 stride 16으로는 1.7셀). 데이터는 **BlenderProc synthetic 전면 제외** + 배포 카메라 실촬영을
       5종 전부로 확대. 계획 정본 = **`docs/vision/yolov5_migration_plan.md`**(게이트·데이터·통합·함정·순서).
       ⚠️ 착수 전 코드 변경 3건 필요(계획서 §5): decode_meta 경로 분리 · launch 모델 인자화 · yaml 주석 정정.
  2. **로봇 제어 (RT 커널 → EtherCAT → RPU)** — 3단계. **✅✅ 2026-07-15 RT 트랙 종결**. 두 커널 결함 모두 해결: (1) **radix-tree**(Ubuntu SAUCE local_lock revert, RT 충돌) 3파일 원복 → 부팅 위반 253→0; (2) **zocl KDS UAF**(`kds_core.c`의 submit-후-타임스탬프, DPU 파이프라인 크래시 원인) 순서교체 픽스. 최종 커널 = **`-rt-kv260c`(#11, DEBUG off + zocl 픽스) 현재 구동**. **zocl 재현검증**: 계측(330s)·프로덕션(200s+) 양쪽 Poison/Oops 0. **cyclictest(DEBUG off)**: idle Max 134 / **load Max 142µs(kv260b DEBUG-on 282→절반)** / 부하 중 위반 0. **EtherCAT 선결조건 전부 해제** — 당시 남은 건 EtherCAT 단계의 3+1 격리 코어 실측뿐이었고, **그 뒤 IgH EtherCAT·3+1 격리·RAON-RT 앱(접근·추종)까지 실기 구동에 도달**했다(★ 2026-09-04, §5.3). 상세: **`rt_kernel_postmortem.md §12-8`**, `rt_patch.md §0/§4-4-2`. ← §5
- **현재 물리 상태**: RealSense 카메라 재연결됨. **보드는 현재 프로덕션 RT 커널 `5.15.199-rt91-rt-kv260c`(#11, 2026-07-20 빌드)로 부팅됨(realtime=1)** — zocl 픽스 포함, DPU 파이프라인 크래시 없이 정상 구동 검증됨. 순정 커널(`5.15.0-1070`)도 설치돼 있어 필요 시 `sudo flash-kernel --force 5.15.0-1070-xilinx-zynqmp && sudo reboot`로 전환. 비전 config는 YOLO로 전환됨, DPU 스택 정상.
- **★ 2026-09-04 — 앱 정의 확정: "접근 + 추종"(pick & place 아님)**: 라이브 제어 앱(`~/RAON-RT-Revision`,
  브랜치 `kv260-merge`)이 하는 일은 정확히 둘이다. ① 오퍼레이터가 `p`로 파이프라인이 본 물체를 고르고
  `v`를 누르면 TCP가 **물체 꼭대기 +15 cm 호버 지점으로 원샷 접근**(2026-07-31 원샷 7/7, 실측 최종오차
  **6~12 mm** — refine 0, 10건 6.15~11.61 mm; 2026-08-05 세션 재현은 4.2~5.9 mm), ② `o`를 누르면 **사람이 물체를 옮겨도 TCP가 따라간다**(추종, 2026-08-03). **파지·이송·놓기는
  코드에 존재하지 않는다** — 그리퍼 하드웨어 미보유라 범위 밖. 추종은 **v3 하이브리드**로, 목표가 0.15 m를
  넘으면 검증된 접근 궤적 스택이 leg를 쏘고 그 안쪽만 1 kHz DLS 서보가 따라간다. 사람이 옆에 서 있는
  시나리오라 실질 안전장치는 **TCP 선속도 캡 0.20 m/s** 하나로 단순화됐고, 그 위에 워크스페이스 박스·goal z
  하한 0.25 m·상실 시 TCP 동결·키 화이트리스트·URDF effort 토크 백스톱·관절 드리프트 예산 1.5 rad이 얹혀
  있다. ⚠️ **접근은 실기 검증 완료, 추종 v3와 operator console(2026-08-06)은 실기 미검증**. 제어 트랙 정본
  = **`docs/RAON-RT/merge.md`**.
- **★ 2026-09-04 — pick_logic v3: 오퍼레이터 지정 클래스 + 단일 인스턴스 sticky 추적**: 앱 전환(접근+추종)에 맞춰
  v2의 점수식 자동 선택(conf 0.5/중앙 0.3/면적 0.2)·hysteresis 타깃 전환·AUTO 모드를 삭제. 추종 중 물체를 화면 중앙에서
  옮기면 중앙의 다른 물체가 점수로 이겨 TCP가 그쪽으로 미끄러지는 경로였음. 이제 `desired_class` **필수**(''=invalid
  `no_class_selected`), 그 클래스의 confidence 최대 인스턴스를 채택해 최근접으로 이어받고 상실 전까지 고정. 같은 클래스
  2개 이상은 최근접+`match_px`로 구분(실험은 클래스당 1개지만 검출기가 2박스를 낼 수 있어 동작 정의 — `_update_track`
  docstring). 합성 테스트 6/6. launch 인자 `desired_class:=apple`(단독 metrics용). **로봇 쪽 후속**: 브릿지 AUTO 메뉴·GUI
  AUTO 버튼·`test_gate2b_bridge.py` C1 제거. 실기 미검증. 같은 날 문서: RPU bridge는 ros2_ws 패키지가 아니라 RAON-RT
  `CROS2PickBridge`의 출력 교체(seqlock→rpmsg)로 확정, `apu_rpu_bridge_pkg` 재생성 안 함.
- **★ 2026-09-04 — RPU 마일스톤: R5F FreeRTOS 단독 구동 성공(작업 2026-09-01~02 / 펌웨어 부팅 검증
  2026-09-02) = Gate 1 실질 통과**:
  `remoteproc`으로 R5F-0에 `r5_freertos.elf`를 올려 부팅 성공(커널 로그 `remote processor
  ff9a0000.rf5ss:r5f_0 is now up`, **2026-09-02 11:02·15:01 2회** — 09-01은 DT enumeration까지고 `is now
  up` 0건) → **판정 수단은 UART가 아니라 LED**(`led35_task`/`led36_task`가 R5에서 실행됨은 커널 로그·elf
  심볼로 확인; Linux heartbeat trigger를 놓게 한 뒤 **물리 점멸은 사용자 육안 확인** — `/sys/class/leds/`에
  미노출이라 보드에서 재검증 불가). 적용 방식은
  계획서가 권했던 runtime configfs overlay가 **아니라 부팅 DTB 병합**(`dtc`+`fdtoverlay` →
  `/boot/firmware/user-override.dtb`, U-Boot `boot.scr` 훅), carve-out은 **`rproc@36000000` 4 MiB no-map**
  (문서가 관례라 적어둔 0x3ed00000은 이 보드 CMA `0x36400000+1000MiB` **한복판이라 폐기**). Vitis 2022.1도
  설치 완료(Windows, XSA `design_1_wrapper`) → **Phase 0 종료**. **남은 것은 정확히 3가지** — ① **APU↔RPU
  통신**(rpmsg), ② **RAON-RT 실시간 제어 task의 RPU 이식**, ③ **EtherCAT SOEM의 RPU 설치**. ①은 DT에 IPI
  mailbox·vring/buffer가 없고 elf에 resource table도 없어(`no resource table found` — 단 경고만 뜨고 부팅은
  됨) **DT 재작업(재부팅 1회) + 펌웨어 재빌드**가 선행 조건. ③의 GEM3 이관은 **APU IgH가 Indy7을 실제로
  제어 중이라 사용자 재결정 전 금지**. 상세 §5.4.
- **★ 2026-08-04/05 — 이동 물체 전환: depth 시간 정합 + throughput 상향**: 파이프라인 전제가
  정적 pick → **움직이는 물체 추종**으로 변경되며 3종 개편. ① **depth-color 시간 정합**:
  `enable_sync: true` + depth 15→30fps + 3D 노드가 최신 depth 대신 **capture_stamp 최근접
  프레임 선택**(800ms 히스토리, `nearest_by_stamp`) → skew |평균| 52.5→**4.3ms**(89%가 정확히
  0.0). 두 반쪽 모두 필수 — sync만 켜면 33.3ms(같은 런 대조열 `skew_if_latest_ms`로 실증).
  align의 skew 0은 **align이 아니라 딸려온 sync의 효과**였음(재해석). ② **뷰어/GUI 영상
  30fps**: 정확 조인 → 매 프레임 렌더+최신 bbox(0.5s age gate), 검증용 정확 조인은 `--sync`.
  ③ **throughput 15.3→21.7Hz**: 병목은 카메라(29.4Hz 공급)도 DPU(73% 유휴)도 아닌
  `process_period_sec: 0.045`(의도된 15Hz 캡, 전제가 정적 물체) → **0.030** + **worker IPC를
  pipe→/dev/shm mmap**(`shm_frame.py`, ipc 13.4→4.65ms, CPU 증가 0). 게이트 단독은
  rate↔신선도 맞바꿈(E2E +14ms)이었으나 shm이 이를 해소 — **0.030+shm이 구 0.045를 전 지표
  지배**(신선도 p50 141 vs 146ms). 정본 **`docs/vision/throughput.md`**(성능 요청 시 §4 사다리
  순서로 제안; 다음 카드 = 워커 파이프라이닝 pre‖DPU → 33.3ms 목표선), 실측
  `evidence/metrics/runs/{sync30_compare_20260804,gate_ipc_20260805}.md`.
- **★ 2026-07-14 새 발견 — RT 커널 + DPU = 커널 크래시(별개 이슈, radix와 무관)**: RT 커널(kv260b)에서 **DPU 비전 파이프라인을 처음 가동하자 ~30초 만에 zocl(Xilinx DPU 드라이버) 슬랩 손상으로 커널 Oops→시스템 프리즈→하드 재부팅** 발생. 레지스터 지문으로 SLUB freelist 손상 확정(`___slab_alloc`←`kds_alloc_command[zocl]`←`zocl_execbuf_ioctl`). **메모리/하드웨어/radix 회귀 아님**(모두 배제). 결함은 RT 인프라가 아니라 **zocl 벤더 드라이버의 메모리 안전 버그**(수정 위치가 radix 버그와 다름). RT-특이(선점 레이스) vs 잠복(SMMU 부재 DMA 스크리블)은 미확정. **★★★ 07-14~15 — 근본원인 확정 + 픽스 + 검증 완료(종결)**: `slub_debug=FZPU,kmalloc-256` 계측 부팅으로 재현 → 크래시 전에 `Poison overwritten` 리포트 생포. 해독 결과 **zocl KDS의 `kds_core.c`에서 `xrt_cu_submit(); set_xcmd_timestamp(KDS_QUEUED);` 순서 결함**(CU 스레드가 커맨드를 submit 직후 완료·해제할 수 있어, 타임스탬프 기록이 **해제된 메모리에 쓰는 use-after-free**가 됨 — 그 offset이 하필 SLUB freelist pointer 자리). RT의 넓은 preemption이 이 레이스를 노출(메커니즘 1 확정, SMMU/DMA 가설 폐기; upstream XRT master에도 잔존). **수정 = 타임스탬프를 submit 앞으로 순서 교체**(의미 동일, 3곳). 패치 = `~/ros2_ws/zocl_patches/`. **✅ 07-15 검증 완료**: 패치를 rev-6(`-rt-kv260c` #10, DEBUG off) 빌드에 합류 → 설치·부팅 → 계측(330s, zocl 156클라이언트) Poison 0 + 프로덕션(계측無 200s+) Oops 0 = 픽스 확정. 상세 `rt_kernel_postmortem.md §12-8`, 메모리 `zocl-dpu-rt-kernel-crash`.
- **★ 2026-07-14 전략 결정 — RT 커널 패치 완성 우선, 비전 성능 개선(fps/latency)은 그 다음**: 비전 CPU 최적화(phase 1+2)가 완료돼 **4코어 중 ~2.2코어 여유** 확보(비전 ~1.8코어). EtherCAT(IgH) ~0.1~0.3코어 + 제어(IK·traj·SM) ~0.1~0.4코어 예상 → **3배+ 마진, CPU는 더 안 줄여도 통합 가능**(헤드룸 판정 §4.2). 따라서 다음 우선순위는 **성능 개선이 아니라 RT 커널 완성**(프로덕션 rev-6 + zocl 크래시 해소) — zocl 크래시가 RT 통합의 실선결조건이고, 성능 개선은 RT 위에서 baseline 재측정하는 게 맞음(RT 오버헤드 +5~10%p 예상). 성능 레버 카탈로그는 §4.2에 phase 3용으로 보존. RT 트랙 재개 = §5.2 + postmortem §12.
- **★ 2026-07-14 카메라 FW 이슈 해결**: D435i FW 5.16.0.1이 librealsense 스트리밍 수십 초 후 RGB 프레임 정지(+dmesg `GET_CUR ... -32` 스톨)를 일으킴 — hardware_reset/물리 replug/재부팅으로 안 나음. **FW 5.17.0.10 업데이트(`rs-fw-update`)로 완치**, 직후 3분 완주(det 15.9Hz). 진단법·상세: 메모리 `d435i-fw-rgb-wedge-fix`. realsense config에 `initial_reset: true` 추가됨(웨지 방어). **CPU 베이스라인(순정커널·신FW): 총 76.8%, target_3d 68.9 / camera 53.6 / detector 43.4 / worker 36.9 / base 14.0 / pick_logic 9.7%** → `perf/runs/stock_fw51710_baseline_20260714/`.
- **★ 2026-07-14 비전 CPU 절감 phase 1+2 완료**: **총 CPU 76.8% → ~44%** (≈3.07→1.8코어, **-1.25코어**), target_3d 68.9→17.3, camera 53.6→28.7, detector 43.4→31.3, e2e 124→81ms, det 15Hz 유지(캡 45ms). 전부 무손실 검증(epipolar A/B 4500회 mismatch 0, letterbox bit-identical, depth view bit-identical, TF 캐시 오차 0.000e+00). phase 2 최대 레버 = **FastDDS+SHM 전환**(UDP loopback→shared memory, -6.6%p). 노드 병합(rclpy) 시도→**기각**(+5.4pt 역효과, waitset 재구성). 상세 = **§4.2 "CPU 절감 phase 1·2"**. DP unbind 영구화 방법 확정(§4.2 — 모듈 blacklist).
- **새 세션이 먼저 물어볼 것**: **"어느 트랙을 이어서 할까요?"** (비전 파이프라인 vs 로봇 제어 앱=RAON-RT 접근·추종 vs RPU 트랙).
- **소통 규칙**: **한국어 답변, 기술용어는 영어 유지.** 사용자는 학부생 수준·절차 지향 → 명령어 중심 단계별 안내. 파괴적/외부영향 작업만 사전 확인, 가역적 작업은 진행.

---

## 1. 프로젝트 개요 & 시스템 아키텍처

### 1.1 무엇을 만드는가
Detect objects via vision → compute 3D position → **로봇 TCP가 그 물체 위로 접근** → **사람이 물체를 옮기면 TCP가 따라간다(추종)**. **파지·이송·놓기는 하지 않는다** — 그리퍼 하드웨어 미보유라 **그리퍼 제어는 범위 밖**. 최종 end-to-end 목표는 로봇 말단부가 물체 바로 위에서 정지하고, 물체가 움직이면 그 위를 계속 유지하는 것까지. (문서 제목과 코드·토픽 이름의 "pick & place"는 초기 프레이밍의 legacy — 이름은 바꾸지 않는다.)

### 1.2 하드웨어 / SoC
- **보드**: Kria **KV260 revB** (SOM: SMK-K26 revA), Zynq UltraScale+ MPSoC. A53 4코어, RAM 4GB, swap 27G.
- **카메라**: Intel **RealSense D435i** (USB로 APU 직결. PL 경유 아님). librealsense 2.57.7 / realsense2_camera v4.57.7, D435i FW **5.17.0.10**(5.16.0.1은 RGB 웨지 버그 — §0 2026-07-14 항목, 절대 되돌리지 말 것).
- **가속기**: `kv260-smartcam` overlay의 **DPU** (`DPUCZDX8G_ISA1_B3136`, fingerprint `0x101000016010406`, 300 MHz), VART/Vitis-AI 2.5.0. systemd(`kv260-smartcam.service`) 자동 로드.
- **타깃 로봇**: Neuromeka **Indy7** (6축 협동로봇). STEP controller 우회, 드라이브 직결. 드라이브 모드는 **CST(`DRIVE_MODE=10`)** 확정(merge.md D1 — 낡은 문서의 CiA402+CSP 서술 주의).

### 1.3 SoC 프로세서 3계층
- **APU (A53×4)**: Ubuntu 22.04.5 + ROS2 Humble. perception 파이프라인 + **IgH EtherCAT master(실운용 중)** — RAON-RT 제어 앱이 여기서 Indy7을 1 kHz로 구동한다(§5.3).
- **RPU (R5F, Real-Time)**: **FreeRTOS 단독 구동 성공(2026-09-02, LED 태스크 실행까지 — 점멸 자체는 사용자 육안 확인)**. SOEM 1kHz EtherCAT 제어와 APU↔RPU 통신은 미착수(§5.4 남은 3과제).
- **PL (FPGA)**: `kv260-smartcam` overlay의 DPU가 NN 추론 가속.

### 1.4 3개 통신 경계 (분리해서 본다)
1. **PL ↔ APU** — DPU 비전 가속: **완성**. APU가 VART로 프레임을 DPU에 먹이고 detection을 받는다.
2. **APU → RPU** — 목표점/궤적 전달 (OpenAMP rpmsg): **미구현** (ros2_ws에 별도 패키지는 두지 않는다 — 2026-09-04 확정: APU 측 어댑터는 RAON-RT `CROS2PickBridge`가 이미 `/pick_target_base`를 구독 중이고, RT 태스크가 RPU로 가면 그 출력만 seqlock→rpmsg로 바뀐다. 구 `apu_rpu_bridge_pkg`는 2026-07-15 삭제, 재생성 안 함) ← **남은 3과제 ①**. RPU 쪽 FreeRTOS는 이미 돌지만(2026-09-02), DT에 IPI mailbox·vring/buffer가 없고 펌웨어에 resource table이 없어 **DT 재작업 + 펌웨어 재빌드가 선행 조건**(§5.4).
3. **RPU → 외부** — EtherCAT으로 Indy7 드라이브 제어: **미구현**(SOEM 미설치 ← **남은 3과제 ③**).
   - ⚠️ **단 Indy7 제어 자체는 미구현이 아니다.** 현재는 **APU에서 IgH EtherCAT + RAON-RT 앱**이 1 kHz로 Indy7을 실기 구동 중이며(접근·추종, §5.3), RPU 경로는 그 제어를 **지터 때문에 옮기려는 다음 단계**다. 제어 사실의 정본은 `docs/RAON-RT/merge.md`.

### 1.5 두 트랙 개요
| 트랙 | 지금 상태 | 다음 |
|---|---|---|
| **A. 비전** (§4) | 파이프라인 완성(**21.7 Hz**, 2026-08-05 gate 0.030+shm IPC; z 검증, depth-color skew ≈0). YOLOv3-tiny 6-class·Gate 2~5 완성. **이동 물체 추종 요구(갱신 주기·depth 시간 정합)는 2026-08-04/05에 반영 완료** | 필요 시 워커 파이프라이닝(`throughput.md` §4). 노드·토픽 구조는 앱 전환으로 바뀌지 않았음 |
| **B. 로봇 제어** (§5) | **RT 커널 종결(2026-07-15, `-rt-kv260c` 구동)** → **APU IgH EtherCAT + RAON-RT 앱이 Indy7 실기 구동 중**: 접근('v')은 실기 검증 완료, 추종('o') v3는 구현 완료·실기 미검증. 3+1 격리(isolcpus=3) 재도입 완료 | 추종 v3 실기 검증 → IK를 RT 루프 밖으로 → **RPU 이관**(FreeRTOS 부팅+LED 태스크 완료, 남은 3과제 §5.4) |

---

## 2. 문서 지도 (정본 라우팅)

새 세션은 **작업할 트랙의 정본 문서**를 이 통합본과 함께 열어라. 무작정 grep 하지 말 것.

> ⚠️ **2026-07-15 구조 개편으로 경로가 바뀌었다.** 아래 표는 새 경로다. 단 **각 문서의 *본문*에 적힌 경로는 개편 전 기준**(예: `crash_logs/…` → 지금은 `evidence/crash_logs/…`)이며, 히스토리 기록물이라 일부러 고치지 않았다. 파일 위치는 이 표에서 찾아라. 개편 판단의 근거: `docs/analyses/cleanup_plan.md` §7.

| 주제 | 정본 문서 | 성격 |
|---|---|---|
| 시스템 전체 규칙·소통 | `CLAUDE.md` (루트) | 항상 우선 |
| 실행법·전제조건·저장소 구조 | `README.md` (루트) | **처음 보는 사람은 여기부터** |
| 파이프라인 상세(노드별 파라미터·기법) | `docs/vision/workflow.md` | 비전 변경 전 필독 |
| **throughput — 어디서 정해지나 / 올리는 순서** | `docs/vision/throughput.md` | **성능 향상 요청 시 필독**(2026-08-05). 병목은 카메라도 DPU도 아닌 `process_period_sec` 게이트 |
| 성능 레버 전수 조사·검증 스냅샷 | `docs/analyses/perf_levers_survey_20260901.md` | §4 사다리 검증 + 신규 레버 3건(RGB 왕복 제거·416×256 재컴파일·resize-shm 융합), 후보별 산식 검산 기록(2026-09-01) |
| **실험 환경 전수 조사(보고서용)** | `docs/analyses/experiment_environment_survey_20260903.md` | HW·OS/RT·ROS·비전·제어·툴체인 실측 표(593 facts) + **§9 문서 vs 실측 불일치 체크리스트**(커널 #11, IgH 1.6.10, FW 5.17, RGB8, CST, IP .50 …) + §10 미확인 항목(2026-09-03) |
| **비전 코드 정독(detector node + worker)** | `docs/vision/detector_worker_walkthrough.md` | 코드 공부용 지도(2026-07-20) |
| 비전 코드 — 노드별 sequence diagram(6파일+overview) | `docs/vision/pipeline_sequence_diagrams.md` | 코드 공부용 mermaid(2026-08-19). walkthrough와 짝 |
| 시간순 히스토리(결정·측정·root-cause) | `docs/history.md` ← *구 progress.md* | 과거 경위 추적 |
| **비전 전체 — 학습→DPU 배포→최적화** | `docs/vision/vision_final.md` | **비전 트랙 종합 정본**(교수님 보고용). SSD→YOLO 전 과정 |
| **비전 YOLO — 명령어·게이트** | `docs/vision/yolov3_tiny_execution_plan.md` | Phase 0~7 / Gate 0~7. 단 "7-class·합성데이터만"은 실행 중 바뀜(6-class·real 도입) → 정확한 서술은 `vision_final.md` |
| **비전 다음 작업 — YOLOv5 전환** | `docs/vision/yolov5_migration_plan.md` | **2026-09-08 결정.** 주류 모델을 YOLOv5n으로 교체하고 YOLOv3-tiny는 legacy A/B 기준선으로 보존. 데이터 재구성·게이트·공존을 위한 코드 변경 3건·함정 목록 |
| 비전 YOLO — 논리 서사 | `docs/vision/yolo_v3_process.md` | 맥락 복구 |
| **데스크톱 bbox 뷰어** | `docs/vision/desktop_viewer_plan.md` | ✅ 완료(2026-07-16). 보드는 압축만/그리기는 데스크톱 |
| 3D 역투영 원리 | `docs/vision/reprojection.md` | 단일점 reverse projection |
| **RT 커널 — 종합 정본** | `docs/rt/rt_final.md` | RT 트랙 결산(2026-07-15 종결) |
| RT 커널 — 인수인계 상세 | `docs/rt/rt_patch.md` | 전 과정·함정·복구(정정 이력 겹겹) |
| **RT 크래시 — 종합 보고서** | `docs/rt/rt_kernel_postmortem.md` | 증상→오진→진단→원인→해결→교훈. **RT 문제는 여기부터** |
| RT 크래시 — 진단 상세·전략 | `docs/rt/rt_kernel_fix_plan.md` | 원인 규명 과정·근거 링크·전략 비교 |
| **RAON-RT 병합 — 로봇제어(Indy7) 통합** | `docs/RAON-RT/merge.md` | **병합 트랙 정본(2026-07-26~) = 라이브 앱(`~/RAON-RT-Revision`, 브랜치 `kv260-merge`)의 사실 소스**. 접근('v')·추종('o') 동작, 목표/결정/Phase·Gate/버그 대장/재개 가이드 |
| **RPU/EtherCAT — 새 세션 진입점** | `docs/rpu/rpu_guide_for_claude.md` | RPU 작업이면 **여기부터**(함정 지도). ⚠️ **Gate 1은 2026-09-02 통과**(FreeRTOS+LED)이나 문서 본문의 적용 경로(configfs overlay)·메모리 배치(0x3ed00000)는 **실제 부팅 구성과 다름** → §5.4 상태 갱신 블록을 먼저 볼 것 |
| RPU 실행 계획(Gate 1~5) | `docs/rpu/rpu_freertos_soem_execution_plan.md` | 명령어·게이트. Gate 1 통과분(2026-09-02)은 §5.4 반영 |
| RPU 논리 전개 | `docs/rpu/rpu_plan.md` | 왜 이 선택인가 |
| 온보딩/인수인계 | `docs/onboarding.md` ← *구 inst_claude.md* | 30초 요약·트랙 확인 |
| 의사결정 여정 | `docs/decision_journey.md` | 왜 그렇게 갔나 |
| 분석·타당성 | `docs/analyses/` | `yolov8_vitisai35_feasibility.md`, `cleanup_plan.md` |
| 지속 메모리 | `~/.claude/projects/-home-ubuntu/memory/MEMORY.md` | 세션 간 유지되는 사실 |
| 토픽별 공식 문서 링크 | `docs/reference/reference_0X_*.md` ← *구 site_md/* | 외부 조사 전 확인 |
| 실행 도구 / 증거 | `tools/` (rt 하네스, kernel_patches) / `evidence/` (crash_logs, metrics, kernel_configs) | 각각 "돌리는 것" / "보는 것" |

---

## 3. 보드·환경 필수 사실

- **보드(작업 머신)**: Kria KV260, `ubuntu@192.168.120.50`(USB NIC), Ubuntu 22.04.5 + ROS2 Humble. 작업 루트 `~/ros2_ws/`. **프로덕션 RT 커널 `5.15.199-rt91-rt-kv260c`(#11 = #10(radix 픽스 + zocl UAF 픽스 + DEBUG off)에 `CONFIG_USB_NET_AX88179_178A=m`만 추가) 구동 중(2026-07-20 빌드)** — kv260b/kv260/-rt-kria·순정도 설치돼 있음(복구 경로). **RT 커널에서 DPU 파이프라인 정상 구동 검증 완료**(과거 §12 zocl 크래시는 kv260c에서 해결). 순정 전환은 `sudo flash-kernel --force 5.15.0-1070-xilinx-zynqmp && sudo reboot`.
  - 참고: RPU 가이드 기준으로 **Claude Code가 보드 자체에서 실행되는** 세션도 있다(`uname -a` → aarch64, hostname `kria`). 그 경우 `/proc`,`/sys`,`dmesg`로 상태 직접 확인 가능.
- **DPU**: `DPUCZDX8G_ISA1_B3136`, fingerprint `0x101000016010406`, kv260-smartcam overlay(systemd 자동), VART/VAI 2.5.0. arch: `~/vitis_ai_work/arch/arch_b3136.json`. **DPU는 한 번에 한 프로세스만 점유** (파이프라인 worker가 떠 있으면 isolated image-test 충돌).
- **카메라**: RealSense D435i, color+depth **848×480×30**, **`align_depth` OFF**. color 토픽 `/camera/camera/color/image_raw`, raw depth `/camera/camera/depth/image_rect_raw`.
- **파이프라인 실행**: `ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py` (카메라+detector+pick_logic+3D+TF). 3D는 단일점 reverse projection(`camera_depth_optical_frame`), base TF는 **실측 캘리브 값**(2026-07-27 eye-to-hand 16 pose, AX=ZB, residual 1.8 cm/1.7° + E21 base 재베이스 yaw +5.14° 반영 — launch 파일 `base_to_camera_tf` 블록 주석). ~~placeholder~~ 해소.
- **detector↔worker 경계 = model-agnostic JSON contract**. 모델 교체는 worker 전처리/decode만 건드림.
  - `vitis_ai_detector_pkg`는 **editable-install**(egg-link → src가 live), config YAML은 **symlink**(수정 즉시 반영, rebuild 불필요). **`target_3d_pkg`만** 편집 후 `colcon build --packages-select target_3d_pkg --symlink-install`.
- **데스크톱(학습/양자화/컴파일)**: `jaehyeon@jaehyeon-Raimlab`, **RTX 4060 8GB**, Ubuntu 22.04.5, RAM 16GB. 경로 `~/capstone_training/`. 학습=native venv(torch cu121), 양자화/컴파일=CPU docker `xilinx/vitis-ai-cpu:2.5.0.1260`. 보드→데스크톱 스크립트 동기화는 rsync, 데스크톱→보드 xmodel 전송은 scp.
- **빌드 PC(RT 커널/RPU elf)**: x86_64 Ubuntu, `jaehyeon@jaehyeon-Raimlab`. RT 커널 작업 경로 `~/kria-rt/`, RPU는 **Vitis 2022.1 설치 완료**(Windows `d:\xilinx\vitis\2022.1`, GCC 11.2.0 arm-none-eabi, BSP `freertos10_xilinx_psu_cortexr5_0`, XSA `design_1_wrapper`) → RPU Phase 0 종료.
- **원격 접속**: **USB NIC(AX88179B, `ax_usb_nic`) `192.168.120.50`** (과거 eth0 직결 `192.168.120.132`, 그 전엔 Tailscale over eth0). **eth0(GEM3)는 EtherCAT 전용으로 분리**돼 있어 EtherCAT/RPU 이관으로 원격이 끊기지 않는다 — GEM3 충돌은 **RPU SOEM 단계(§5.4 남은 3과제 ③)에서만** 문제(§6).

---

# 트랙 A — 비전 (Perception)

## 4.1 현재 파이프라인 아키텍처 (노드별)

### 데이터 흐름
```
[RealSense D435i]
   /camera/camera/color/image_raw          (RGB8 848×480)
   /camera/camera/depth/image_rect_raw     (16UC1 848×480, raw=unaligned, 0.001 m/unit)
   /camera/camera/{depth,color}/camera_info (intrinsics)
   /camera/camera/extrinsics/depth_to_color (baseline ≈ 15 mm)
        │ color image
        ▼
[vitis_ai_detector_node]  ── pipe(JSON+raw) ──▶ [vitis_ai_worker(_yolo).py (DPU 추론 프로세스)]
   → /detections                          (my_interfaces/DetectionArray)
        │
        ▼
[pick_logic_node]  (오퍼레이터 지정 클래스의 인스턴스 1개, sticky)
   → /pick_target                         (my_interfaces/PickTarget)
        │   + raw depth + intrinsics + extrinsics
        ▼
[pick_target_3d_node]   (single-point reverse projection)
   → /pick_target_3d                      (frame=camera_depth_optical_frame)
        │   + TF (base_link → camera_link → camera_depth_optical_frame)
        ▼
[pick_target_base_node]
   → /pick_target_base                    (frame=base_link)
```

### 노드 ↔ 파일 ↔ 설정
| 노드 | 파일 | 설정 |
|---|---|---|
| `camera` (realsense2_camera) | third-party | `config/realsense_pick_place.yaml` |
| `vitis_ai_detector_node` | `vitis_ai_detector_pkg/.../vitis_ai_detector_node.py` | `config/vitis_ai_detector.yaml` |
| (worker) | `.../vitis_ai_worker_yolo.py` | 위 yaml의 `worker_*` |
| `pick_logic_node` | `pick_logic_pkg/.../pick_logic.py` | `config/pick_logic.yaml` |
| `pick_target_3d_node` | `target_3d_pkg/.../pick_target_3d_node.py` | `config/target_3d.yaml` |
| `base_to_camera_tf` | `tf2_ros/static_transform_publisher` | launch 인자 (**실측 캘리브 값**, 2026-07-27) |
| `pick_target_base_node` | `target_3d_pkg/.../pick_target_base_node.py` | `config/target_base.yaml` |
| 통합 launch | `system_bringup_pkg/launch/pick_place_vitis_ai.launch.py` | — |

### 노드별 핵심

**1) RealSense 카메라** — color/depth 스트림 + intrinsics/extrinsics 발행.
- `color_profile: 848×480×30` (1280×720이면 realsense 노드가 busy-spin에 빠져 ~70초 후 stall. worker가 어차피 416×416 letterbox로 줄이므로 검출 품질 무영향, USB/CPU ~2.3배↓).
- `align_depth.enable: **false**` (full-frame 정렬은 A53 코어 하나 100% 점유 → throughput 절반. bbox 중앙 1점만 필요하므로 정렬 끄고 reverse projection).
- `enable_sync/gyro/accel/infra: false`.
- 함정: realsense는 **aligned 토픽 구독자가 있을 때만** 정렬 계산 (과거 "카메라 단독 25Hz" 측정이 오도한 이유).

**2) vitis_ai_detector_node + worker** (두 프로세스로 분리, 이유 §4.3):
- (A) ROS 래퍼: `image_callback`은 최신 프레임만 보관(즉시 반환), `worker_loop` 전용 스레드가 연속 처리. ROS Image→BGR numpy→worker로 전송→JSON detection 수신→`/detections` 발행.
- (B) worker(rclpy 없음): 기동 시 1회 `xir.Graph.deserialize`→DPU subgraph→`vart.Runner.create_runner`→LUT/입출력버퍼/head_params(anchor·stride·thr_q) 준비→`ready` 핸드셰이크(실측 ~4.2s). 이후 프레임마다 `detect()`: letterbox+LUT 전처리 → `execute_async→wait` → `decode_head`(objectness 사전필터 + yolov5 grid decode) → 클래스별 NMS + letterbox 역변환.
- 실측 프레임 예산(`evidence/metrics/runs/shm_gate030_20260805`, 6343프레임 중앙값): pre 11.6 / **dpu 17.4** / post 4.8 = worker 35.3 ms, ipc 3.2 ms, processing 43.0 ms, 검출 주기 45.5 ms(≈22 Hz).
- 핵심 파라미터: `detector_mode: worker`, `send_resized_input: false`(worker가 letterbox), `worker_shm: true`(프레임을 `/dev/shm`으로 — ipc 13.4→4.65 ms), `process_period_sec: 0.030`, `worker_log_path: ""`(파일 I/O 제거), `metrics_duration_sec: 0.0`(⚠️ **double이어야 함** — 정수면 InvalidParameterType로 detector 死).
- 보드 overlay 그리기는 **없다**(2026-09-03 코드째 제거). 박스는 데스크톱(`detection_viewer_pkg`, `raon_operator_gui`)에서 그린다.

**3) pick_logic_node** — 2D 필터 + 단일 인스턴스 추적(**v3, 2026-09-04**). L1 순서: confidence → allowed class → bbox 크기>0 → edge margin(30px) → bbox 면적(min 400 / max 0.5 화면) → `desired_class` 일치. `desired_class`(LIVE, 로봇 앱 'p' 메뉴가 세팅)는 **필수** — 비면 전부 invalid(`no_class_selected`). 트랙 없으면 그 클래스 중 confidence 최대 채택, 있으면 최근접 이어받기(`stability_match_px`는 2개 이상 보일 때만), 3프레임 연속 후 `target_valid=true`, 이후 **sticky**(5프레임 연속 상실 전까지 전환 없음). v2의 점수식·hysteresis·AUTO는 삭제(앱이 접근+추종이라 오퍼레이터가 클래스를 고른다). 단독 측정은 launch 인자 `desired_class:=apple`. `image_width/height`는 **카메라 해상도와 일치**해야(bbox가 source 좌표).

**4) pick_target_3d_node** — 단일점 reverse projection (§4.3.5). bbox 중앙 color 픽셀 → `rs2_project_color_pixel_to_depth_pixel`(epipolar 선분 탐색)로 대응 depth 픽셀 → patch median(9×9) z → depth intrinsics로 deproject. 출력 frame `camera_depth_optical_frame`. ⚠️ rs2 extrinsics 회전은 **column-major** (`reshape(3,3, order='F')`). depth_scale 0.001, 유효범위 **0.19~1.6 m**(`target_3d.yaml`. 옛 0.05~3.5 m는 2026-07-14 P1에서 정정됨 — D435i 물리 min-z ~0.195 m).

**5) base_to_camera_tf** — `base_link → camera_link` 정적 TF (`pick_place_vitis_ai.launch.py`). **2026-07-27 hand-eye 캘리브 + E21 재베이스 반영**: `t=(0.811666, 0.087684, 0.924974)`, `q=(0.72462829, 0.00008362, -0.68706199, 0.05347582)`. eye-to-hand 천장 마운트, 테이블면 위 0.825 m, 광축이 수직에서 5.29°. (옛 placeholder `x=0.45,y=0.10,z=0.70,rpy=0`은 대체됨.)

**6) pick_target_base_node** — `/pick_target_3d` + TF → `do_transform_point` → `/pick_target_base`(base_link). depth sanity 게이트 `min/max_camera_z_m: 0.20/0.95`(2026-09-03 유도, 근거는 `target_base.yaml` 주석), `require_depth_valid: true`. ⚠️ reachability는 이 게이트가 아니라 브릿지 워크스페이스 박스 담당.

### 메시지 타입 (`my_interfaces`)
```
Detection.msg       : int32 class_id, string class_name, float32 confidence,
                      float32 center_x, center_y, width, height
DetectionArray.msg  : std_msgs/Header header, Detection[] detections
PickTarget.msg      : bool target_valid + (Detection 필드들)
PickTarget3D.msg    : std_msgs/Header header, bool target_valid, bool depth_valid,
                      (Detection 필드들), float32 x, y, z
```

## 4.2 성능 최적화 기법 (검출 단계)

측정으로 병목 분해(`img/pre/dpu/post/ipc/overlay`) 후 큰 leaf부터 제거. **DPU 추론 자체는 ~13ms**(하드웨어 하한)였고 진짜 병목은 **Python 전/후처리**.

- **LUT 전처리** (`pre_ms 42→12`): 입력이 uint8(0~255, 256가지)이라 채널별 256칸 int8 표 미리 계산 → 매 프레임 표 조회 3번. **bit-identical 검증** (round/clip 동일).
- **SSD 후처리 사전필터** (`post_ms 20→7`): 16,436 prior 대부분 background → foreground 임계 통과 가능성 있는 prior만 필요조건(`z_c − z_bg ≥ logit(t_c)`, ε 마진)으로 1차 필터 후 소수에만 softmax/decode/NMS. **최종 detection 동일** 검증.
- 기타: 입력 resize 후 전송(pipe payload −81%), overlay off(−70ms/frame), worker file-log off(프레임당 ~18회 I/O 제거), 카메라 1280→848(busy-spin stall 해소), full-frame alignment 제거(코어 ~65% 회수).
- **누적 결과**: `processing_ms 88.7→45.8`, throughput `8.6→13 Hz`(compute), alignment 제거로 **~17 Hz**, latency `349→261ms`. **모든 최적화에서 검출 정확도 불변.** throughput 천장 = **카메라 공급률**(realsense 단일스레드), APU compute 아님.

| 단계 | baseline | +LUT | +post필터 | +pipe | +reverse-proj |
|---|---|---|---|---|---|
| pre_ms | 42.5 | 12.7 | 12.5 | 11.8 | — |
| post_ms | 20.1 | 19.4 | 7.4 | 7.8 | — |
| dpu_ms | 12.7 | 13.0 | 12.8 | 12.5 | (HW 하한) |
| processing_ms | 88.7 | 58.2 | 45.8 | 46.1 | ~46 |
| **publish Hz** | **8.6** | 12.2 | 13.0 | 12.7 | **~17** |
| frame_age_ms | 349 | 307 | 292 | 261 | ~261 |

### ★ CPU 절감 phase 1 (2026-07-14, YOLO 교체 후 · 순정커널 · FW 5.17.0.10)

목표가 throughput이 아니라 **CPU 절감**(EtherCAT+제어 코드 공존용)인 최적화 라운드. 전부 **무손실**(bit-identical/수치동일 검증) 원칙, 총 **CPU 76.8% → 53.1%** (≈3.07→2.12코어, **-0.95코어**), e2e 124→92ms 덤. 산출: `perf/runs/stock_fw51710_baseline_20260714`(전) vs `perf/runs/phase1_cpuopt_20260714`(후).

- **P1 epipolar 벡터화** (`pick_target_3d_node.py`): rs2 역투영 선분 걷기(python 루프 126회/pick)를 numpy 일괄 처리로. 오프라인 600px + **라이브 4500 pick A/B mismatch 0** 검증(`epipolar_ab_check` 파라미터로 상시 재검증 가능). + `target_3d.yaml` min/max_depth 0.05/3.5→**0.19/1.6**(D435i 물리 min-z ~0.195m). ⚠️ 당시 "base gate 0.20/1.50과 일치"라고 적었으나 1.6≠1.50이고 그 0.10 차이는 의도된 것(탐색이 게이트를 감싸야 거부가 진단 가능). 2026-09-03 게이트 상한을 0.95로 재유도했고 탐색 1.6은 **유지** — workspace 확장 시 두 yaml 같이 조정.
- **P2 정적 구독 prune** (동 파일): camera_info x2(30Hz)+extrinsics를 첫 수신 후 `destroy_subscription`(타이머로 안전하게) — intrinsics는 stream profile 고정이라 무손실.
- **P3 letterbox cv2 경로** (`vitis_ai_worker_yolo.py`): fancy-index 3회+매프레임 fill → `cvtColor(BGR2RGB)+cv2.LUT`+fill 1회 캐시. **bit-identical PASS**. `pre_ms 17.5→9.0`.
- **P4 depth 30→15fps** (`realsense_pick_place.yaml`): 3D는 pick 시점 latest-depth만 사용 → 15fps로 camera/t3d depth 경로 절반. depth 최신성 33→66ms(정적 픽 장면 무해).
- **검출률 캡** (`vitis_ai_detector.yaml process_period_sec: 0.045`): P3로 uncap 시 ~19Hz까지 오르나 절감분을 도로 소모 → 2프레임 주기(15Hz) 고정. **0.062는 지터로 3프레임 주기(13.8Hz)로 빠짐 — 45ms가 정답**. 0.0으로 되돌리면 ~19Hz(+3~5%p CPU).
  **→ 2026-08-05 재결정: 0.030으로 완화**(이동 물체 전환으로 15Hz 캡의 전제 소멸) + worker IPC shm화 → 21.7Hz, CPU 1.60코어. 위 문단은 정적 물체 시절 기록으로 보존. 현행 정본은 `docs/vision/throughput.md`.

| 노드 | 전(76.8% 총) | 후(53.1% 총) |
|---|---|---|
| target_3d | **68.9** | **12.0** |
| camera | 53.6 | 40.2 |
| detector | 43.4 | 39.8 |
| worker(DPU pre/post/IPC) | 36.9 | 25.4 |
| target_base / pick_logic | 14.0 / 9.7 | 11.1 / 7.6 |
| det_hz | 15.9 | 14.6 (중앙값 15.0, p95 16.0) |
| e2e | 124ms | 92ms |

### ★ CPU 절감 phase 2 (2026-07-14, phase 1에 이어)

**최종: 총 CPU 76.8% → ~44%** (≈3.07→1.8코어, **-1.25코어**), e2e 124→81ms, det 15Hz 유지. 산출: `perf/runs/phase2_w1_20260714`(quick-win) → `phase2_w2_fastdds_shm_20260714`(DDS) → `phase2_final_20260714`(확정).

- **W1 quick wins (채택)**: ① t3d depth lazy 변환 — depth_callback은 msg 저장만, pick 시점에 `np.frombuffer` zero-copy view (cv_bridge 대비 26x, **라이브 비트동일 True 확인**). ② base 노드 정적 TF 캐시 — 첫 lookup에서 (R,t) 캐시 후 matmul 1회로 변환 (**tf2 대비 오차 0.000e+00 확인**, `static_tf_cache` 파라미터로 off 가능). base 10.4→6.7%. ③ worker decode 상수 hoist(2^fix·thr_q·anchors 배열을 load_model 1회 계산).
- **W2 FastDDS+SHM (채택, 최대 레버)**: `.bashrc`의 `RMW_IMPLEMENTATION`을 cyclonedds→**fastrtps** + `FASTRTPS_DEFAULT_PROFILES_FILE=~/ros2_ws/fastdds_shm_profile.xml`(SHM segment 16MB). 같은 보드 안 1.22MB Image가 UDP loopback 대신 shared memory로 → **camera 40→29, detector 39→31, 총 -6.6%p**. 되돌리기: bashrc 두 줄을 cyclonedds 한 줄로.
- **W3 노드 병합 (시도→기각·롤백)**: pick_logic+t3d+base를 단일 프로세스(`pick_post_stack`, 코드는 잔존)로 병합했더니 **+5.4pt 역효과 실측** — rclpy executor가 매 콜백마다 전체 waitset을 재구성해, 엔티티 3배가 모든 콜백 dispatch 단가를 올림. **rclpy에선 병합 금지**, 실익은 rclcpp composition으로만 가능. launch는 3노드 분리로 복원됨.

| 노드 | phase1 후 | phase2 후 |
|---|---|---|
| camera | 40.2 | **28.7** |
| detector | 39.8 | **31.3** |
| worker | 25.4 | 25.7 |
| target_3d | 12~20 | 17.3 |
| target_base | 11.1 | **6.7** |
| pick_logic | 7.6 | 7.2 |
| **총(4코어 평균)** | 53.1→51.1(DP) | **~44** |

### ★ zero-copy 분석 (2026-07-15) — "SHM transport가 zero-copy인가? Iceoryx로 가야 하나?" → 아니오 / 아니오

W2 채택 후 제기된 두 질문("FastDDS SHM은 zero-copy냐", "아니면 CycloneDDS+Iceoryx가 맞지 않냐", "해상도를 고정하면 zero-copy가 되지 않냐")에 대한 결론. **현재 선택(FastDDS SHM transport)이 옳고, Iceoryx는 명확한 후퇴이며, zero-copy의 유일한 경로는 레버 ⑥(rclcpp composition)이다.**

- **FastDDS의 공유메모리는 두 가지가 별개다**:
  - **SHM transport** (= 우리가 쓰는 것): UDP를 대체하는 *transport*. 직렬화 + SHM memcpy(쓰기/읽기)가 **그대로 남는다**. 즉 -6.6%p의 정체는 *복사 제거*가 아니라 **커널 네트워크 스택(syscall·소켓·UDP/IP·loopback) 제거**다.
  - **Data Sharing** (`<data_sharing>`): 진짜 zero-copy delivery. **plain & bounded 타입을 요구**.
- **`sensor_msgs/Image`는 plain이 될 수 없다** — unbounded가 3곳: `uint8[] data` + `string encoding` + `Header.frame_id`(string). **해상도를 고정해 `uint8[1221120] data`로 바꿔도 string 2개가 남는다**(boundedness는 런타임 값이 아니라 **타입 선언**의 속성). Header를 버리면 `header.stamp` 기반 박스-이미지 매칭(현재 오차 0)을 잃는다.
- **CycloneDDS+Iceoryx는 후퇴**: Iceoryx zero-copy도 고정크기 타입을 요구 → unbounded Image에는 **관여 자체를 안 하고 네트워크 경로로 폴백** = UDP loopback 복귀(이득 0). 게다가 `iox-roudi` 데몬이 상주(프로세스·CPU·실패지점 추가). **FastDDS SHM transport는 transport라서 타입을 안 가린다** — Iceoryx가 못 붙는 자리에서 효과가 난 이유.
- **결정타 — rclpy에는 loaned message API가 없다** (2026-07-15 실측: `rclpy.publisher.Publisher`에 loan 관련 메서드 **0개** / rclcpp엔 `rclcpp/loaned_message.hpp` 존재). zero-copy는 `borrow_loaned_message()`로 DDS가 준 메모리에 직접 써야 성립 → **C++ 전용**. detector가 Python인 한 타입을 고쳐도 불가능.
- 소스도 third-party: `realsense2_camera`(C++)가 `sensor_msgs/Image`를 발행 → 커스텀 plain 타입을 쓰려면 변환 노드가 필요하고 **그 변환이 곧 복사**(포크하지 않는 한 복사를 옮길 뿐).
- **결론: Image의 진짜 zero-copy = rclcpp intra-process composition(레버 ⑥).** intra-process는 `shared_ptr`만 넘기므로 **직렬화 자체가 없고 boundedness와 무관** → `sensor_msgs/Image`를 그대로 쓰면서 복사 0. 해상도 고정도 메시지 수술도 불필요. **레버 ⑥의 근거가 여기서 강화됨** — W3(rclpy 노드 병합 +5.4pt 실패)와 뿌리가 같다: **rclpy엔 intra-process도 loan도 없다.**

**FastDDS 2.6.11(Humble) 제약 실측 → `fastdds_shm_profile.xml`은 삭제 불가**: `FASTDDS_BUILTIN_TRANSPORTS` 허용값 = `DEFAULT/DEFAULTv6/LARGE_DATA/UDPv4/UDPv6` (**SHM 옵션 없음**), 옵션 문법(`?max_msg_size=`)은 2.13+ 기능이라 미지원, segment_size 전용 env var 없음 → **segment_size는 XML로만 설정 가능**(C++ API엔 있으나 `rmw_fastrtps`가 transport descriptor를 노출하지 않음). 파일을 지우면 기본 **512KB**로 폴백 → color 1.16MB·depth 0.77MB가 안 들어가 **UDP로 조용히 회귀(-6.6%p 상실, 에러 없음)**. 이 XML은 클러터가 아니라 *CycloneDDS 시절에 없던 튜닝* 그 자체다. (배선은 `~/.bashrc:123-124`의 env var 2줄 — 저장소 밖이라 clone으로 전파되지 않음, §7 재현성 참고.)

**잔여 CPU 백로그(미적용)**: node→worker POSIX shm IPC(-3~5pt), detector raw 구독(-2~6pt), rosout off(-3~6pt, /rosout 소실 트레이드오프), (조건부·검출품질 A/B 필수) color YUYV 포맷(-17~30pt, bit-identity 깨짐). 측정 하네스에 고아 launch 래퍼 자동정리 추가됨(run_gate6_perf.sh preflight).

**DP unbind 영구화 (2026-07-14 방법 확정)**: `zynqmp-display`는 빌트인이 아니라 **모듈 `zynqmp_dpsub`**(현재 로드·사용처 0, `/boot/initrd.img`에도 복사본 존재 확인). 따라서 영구 차단 = blacklist + initramfs 갱신 (사용자 sudo 2줄):
```bash
echo 'blacklist zynqmp_dpsub' | sudo tee /etc/modprobe.d/blacklist-zynqmp-dpsub.conf
sudo update-initramfs -u
```
다음 부팅부터 모듈 로드 자체 차단(irq 11k/s→0 원천 봉쇄). 확인 `lsmod | grep dpsub`(무출력=성공). DPU(zocl)는 별개라 smartcam 파이프라인 영향 없음. 롤백(DP/HDMI 출력 필요 시) = conf 삭제 후 `update-initramfs -u` + 재부팅. (RT 커널 initrd에는 이미 dpsub blacklist 반영됨 — 이건 순정 커널용.)

**★ EtherCAT 헤드룸 판정 (2026-07-14, "CPU 더 줄여야 하나?" 답 = 아니오)**: 비전 ~1.8코어 → **여유 ~2.2코어**. 수요측 IgH EtherCAT 1kHz cyclic ~0.1~0.3코어 + 제어(IK·traj·SM, C++) ~0.1~0.4코어 → 합쳐도 여유의 1/3 이하(**3배+ 마진**). 단 3가지 주의: ① RT는 평균 아닌 **최악지연** 기준(통합 시 EtherCAT 전용코어 pin + 비전 나머지 3코어 = 평균 60%). ② RT 전환 시 threaded IRQ로 비전 +5~10%p 예상(그래도 충분). ③ **진짜 게이트는 CPU가 아니라 zocl 크래시**(§5.2, postmortem §12) + DPU DDR burst의 EtherCAT 지터 영향(CPU%에 안 보임 → 통합 시 cyclictest+파이프라인 동시측정 필요). 남은 절감 백로그(YUYV 등)는 통합 후 모자랄 때 꺼내는 예비 탱크.

**성능 개선 레버 카탈로그 (phase 3용 — RT 완성 후 착수 권장)**: 성능(fps↑/latency↓)은 총 CPU를 fps에 비례해 올리므로 진행 게이트 = **총 CPU ≤ 60%**(EtherCAT 마진 보존). phase2 최종 **e2e 81ms 분해**(중앙값): 캡처→detector도착 **40.7ms**(센서 물리, 30fps 상한 — 우리 코드 밖) / img→np 1.3 / node↔worker IPC **6.1** / pre(letterbox) 8.2 / **DPU 16.9**(모델 결정) / post 4.2 / publish~3. 처리 합계 37.8ms라 **캡만 풀면 ~26Hz 체력 이미 있음**. 레버(추천순):

| # | 레버 | 효과 | CPU 비용 | 난이도 |
|---|---|---|---|---|
| 1 | **node↔worker POSIX shm IPC** | latency -5~6ms | **-3~5pt(동반 하락)** | 중 — 유일 win-win, 1순위 |
| 2 | 캡 `process_period_sec` 조정 | 15→20Hz / uncap ~26Hz | +20pt / +45pt↑ | 설정 1줄 |
| 3 | worker **3단 pipelining**(pre‖DPU‖post, VART async) | 30Hz(카메라 상한)까지, latency 불변 | fps 비례 | 중상 |
| 4 | color 60fps 모드 | latency -15ms(readout 반감) | camera +10pt± | 설정+검증 |
| 5 | YOLOv3-tiny 7-class 교체(Gate5 트랙 합류) | dpu_ms↓ → fps·latency 동시 | — | 별도 트랙 |
| 6 | rclcpp 포팅 + composition | CPU·latency 동시 대폭 | 큰 공수 | 장기 |

**주의**: ⑤로 모델 프로파일이 바뀌기 전엔 detector 주변을 과최적화 말 것(모델 무관한 ③ pipelining은 지금 해도 무방). phase3 착수 시 **첫 작업 = perf_probe를 /detections가 아닌 최종 target 토픽까지 확장**해 full-chain latency 기준선 확보.

## 4.3 파이프라인 아키텍처 결정

- **4.3.1 프로세스 분리 (detector ↔ worker)**: VART `execute_async()`가 rclpy 노드 in-process에서 segfault/bus error. **근본 원인 = XIR graph/subgraph Python 객체가 GC되어 runner가 무효 상태 참조**. 해결 = 별도 프로세스 + `self.graph`/`self.dpu_subgraphs`를 runner 수명 동안 보관. (fallback: C++ VART / Vitis-AI Library.)
- **4.3.2 worker 수명주기**: xmodel/runner 1회 로드, 출력 버퍼 재사용. 노드가 worker death 감지 시 재시작(startup timeout 30s).
- **4.3.3 콜백 파이프라이닝**: `image_callback`=최신 프레임만 저장 즉시 반환, `worker_loop` 전용 스레드가 연속 처리(묵은 프레임 버림). GIL 안전(무거운 DPU는 별도 프로세스, 노드는 pipe I/O 대기). throughput 불변(camera-limited 드러남), **latency ~31ms 감소**.
- **4.3.4 QoS**: 카메라 구독 `KEEP_LAST depth=1 BEST_EFFORT`(최신 1장), extrinsics `RELIABLE TRANSIENT_LOCAL`(latched), 발행 `RELIABLE depth=1`.
- **4.3.5 Reverse projection (align 제거)**: aligned depth 구독 시 realsense가 매 프레임 ~407k 픽셀 재투영 → 코어 100% → 카메라 12Hz throttle. 우리는 bbox 중앙 1점만 필요. align 끄고 color 픽셀 1개를 `rs2_project_color_pixel_to_depth_pixel`(dmin/dmax deproject → 짧은 선분 → 각 depth 픽셀 reproject 최근접 선택)로 매칭. color↔depth ~15mm baseline이라 disparity 큼(60~90px)이라 단순 근사 불가. 시뮬 5점 0px 오차, 실측 z 자로 검증. **~13→17 Hz, 코어 ~65% 회수** (향후 RT/EtherCAT 여유).

## 4.4 YOLO 교체 트랙 (SSD stand-in → YOLOv3-tiny 6-class)

### 목적
`ssd_adas_pruned_0_95`(car/bicycle/person)는 **DPU+파이프라인 검증용 stand-in**. 최종 pick 물체 detector로 교체.

### 결정 로그 (Decision Log)
| # | 결정 | 근거 |
|---|---|---|
| D1 | 모델 = YOLOv3-tiny (원래 7-class → **D13에서 6-class**) | DPU 지원 op만으로 단일 subgraph |
| D2 | **실환경 촬영 이미지는 학습에 미사용**(검증만) — *D12/D13/D14에서 완화됨* | 사용자 결정 2026-07-06 |
| D3 | 학습 데이터 = synthetic(YCB mesh) + 공개 dataset(COCO/OI/YCB-Video) | D2 귀결 |
| D4 | 물리 객체: 과일=플라스틱(YCB replica), tennis/mustard=실물. mustard=Morehouse(≈YCB006) | 2026-07-06 |
| D5 | 툴체인 = **Vitis-AI 2.5 docker 고정** | 버전 skew 방지 |
| D6 | 프레임워크 = PyTorch(vai_q_pytorch) | UG1414 |
| D7 | 전처리 = **letterbox 416×416** (train/calib/worker 통일) | yolov5 표준 전처리 |
| D8 | person = safety class, threshold 낮게(0.25~0.4). pickable=[0..], safety=person | — |
| D9 | fast-finetune/QAT 써도 **학습 subset만** 사용 | D2 준수 |
| D10 | 배포 기하 = 회색 optical table, **top-down 0.8m 수직 고정** | 사용자 2026-07-06 |
| D11 | **activation SiLU → `nn.Hardswish()` 교체 후 재학습** (SiLU는 DPU 미지원) | 2026-07-07 |
| D12 | **D2 재검토 → 소량 real 학습이미지 추가** (peach/mustard 도메인 갭이 재렌더로 안 닫힘) | 사용자 2026-07-08 |
| D13 | **peach 드롭 → 6-class 재번호** + YCB 벤치마크 real 스캔 학습 도입 | 사용자 2026-07-09 |
| D14 | apple 안정화 = **배포 카메라로 apple만 top-down 실촬영** 학습 추가 | 사용자 2026-07-09 |

**현재 class**: `{0 apple, 1 orange, 2 banana, 3 tennis_ball, 4 mustard_bottle, 5 person}`.

### 시스템 통합 범위 (JSON contract 덕에 좁음)
- **변경**: `vitis_ai_worker_yolo.py`(MODEL_W/H=416, LUT, YOLO grid decode+NMS, objectness pre-filter, class threshold), `vitis_ai_detector.yaml`(model_path·worker·`send_resized_input: false`), `pick_logic`(pickable/safety 분리).
- **불변**: detector node(handshake에서 input size 자동 수신), 3D·TF·camera·pipelining·metrics.

### Phase / Gate 요약
| Gate | 통과 조건 | 상태 |
|---|---|---|
| 0 | Inspector: CPU subgraph 0 | ✅ (Hardswish 패치 후) |
| 1 | class별 수량 + contact-sheet 검수 | ✅ (train 15,799 / val 2,093 — *7-class 시절 수치; 6-class(YCB real+real_apple 추가) 재집계 수치는 소스에 미기록*) |
| 2 | float mAP + 혼동 pair + person recall + cross-domain | ✅ (배포=D14 all mAP50 0.728; D13 0.748 — 차이는 `hsv_v`↑ 대가, 실물은 오히려 개선) |
| 3 | quant 손실 ≤3%p | ✅ (cosine 0.9757/0.9615 > 0.95 기준) |
| 4 | DPU subgraph 1개 + 보드 load | ✅ (배포=D14 md5 `9bc6520c…`; 출력 33ch=(6+5)×3) |
| 5 | host 일치 + 실물 top-down 검출 | ✅ **6종 전부**(D14 후 apple 0.5→0.88) |
| 6 | `/pick_target_base` 정상 (풀 파이프라인) | ✅ 15Hz 동기·E2E 137ms (2026-07-10 3분 실측) |
| 7 | live 체크리스트 + 성능 CSV | 🟡 성능 CSV 확보(3분 실측) / live 체크리스트 미완 |

### 진행 히스토리 (D11 → D14)

**Gate 2/3/4 통과 (7-class, 2026-07-07)**: SiLU 1차 학습 → 양자화에서 `aten::silu_` float 잔류(VAIQ_WARN) → **Hardswish 재학습**(D11, all mAP50 0.758→0.766, 비용 ~0). 양자화 중 VAI 2.5 `hardswish.py` 배포 버그 2건(FixNeuronWithBackward 미정의 / fake_quantize 인자 누락) → 12/12a wrapper가 docker 안 `sed`로 자동 패치. Gate3 cosine 0.9923/0.9849, Gate4 DPU subgraph 1. **교훈: cosine 수치보다 VAIQ_WARN/unknown-op를 먼저 볼 것.**

**Gate 5 실패 → 도메인 갭 확정 (7-class, 2026-07-07~08)**: 실물 6종 중 3종 실패. 진짜 D10 top-down 재검증에서도 재현:
- banana/orange ✅, tennis_ball 0.491(경계), **apple(빨강) 0.26**(약함 + peach에 밀림), **peach→apple 오분류**, **mustard 0.02**(완전 실패).
- 패턴: **COCO-rich class(apple/orange/banana) 전이 성공**, synthetic+niche real 의존(peach/tennis/mustard) 실패. threshold 무효(0.05로도 후보 없음). peach·mustard는 **YCB mesh ≠ 실물 외형** → 재렌더로 못 닫음 → **D12(real 데이터 학습 사용) 결정**.

**D13 — peach 드롭 + YCB real 도입 (2026-07-09)**: YCB 대조 결과 **peach 드롭**(실물 분홍+잎 ≠ YCB015, apple과 최난도 혼동), **mustard 유지**(YCB006 일치, YCB N5 카메라가 top-down 커버). 7→6 class 재번호. YCB 벤치마크 real 스캔(물체당 600장+mask, N4/N5=top-down) 학습 도입 = 촬영·라벨 수작업 0.

**★ D13 결과 — Gate 5 통과 🎉 (2026-07-09)**:
- Gate2 all mAP50 **0.748** (mustard .933 / tennis .950 / orange·banana .736 / **apple .629**(최약) / person .503).
- Gate3 cosine 0.9757/0.9615, Gate4 subgraph 1 (구 7-class는 `models/*.OLD7.*` 백업).
- **실물 top-down 재검증**: mustard **0.02→0.814**(핵심 성과), tennis 0.491→0.677, orange 0.850, banana 0.777 — **5종 정상**. **apple만 3프레임 0.489/0.549/0.549 = 0.50 경계.**
- 진단: 드롭된 **peach(OOD)가 apple로 0.462 오검출** + 인접 물체 억제 → 복숭아 치우니 진짜 apple 0.216→0.549. **배포엔 peach 없으니 오검출 무의미.** 진짜 apple 경계 = **color 도메인 갭**(실물 apple이 YCB apple보다 밝음) + apple 자체 최난도(둥금). **양 문제 아님**(이미 ~6371 인스턴스).

**★ D14 — apple 실물 캡처 타깃 재학습 → Gate 5 통과 🎉 (2026-07-09~10)**:
- 완료분: `capture_color_frames.py --manual`(Enter=1장, blur 배제), `autolabel_single_object.py`(신규, DPU top-1 box → `--class-id` 강제+viz). apple만 올려 ~54장 캡처·라벨(오검출 1장 삭제) → `datasets/real_apple_yolo/`(train-only) rsync. `06_merge_split.py`에 `real_apple_yolo`+`TRAIN_ONLY_SOURCES` 추가. `hyp_pickplace.yaml` `hsv_v` 0.40→0.50(밝기 갭 완충, **hsv_h는 유지** = apple↔orange 혼동 방지).
- ⚠️ **`11_train.sh`가 데스크톱 GPU 드라이버 hang으로 2회 중단**(둘 다 epoch 0, GPU ~50℃=과열 아님). 원인 = 자동 업데이트로 설치된 `nvidia-driver-595-open`(open 커널모듈+GSP 강제, 595.71.05)의 **로그 없는 silent hard-hang**(OOM/Xid/MCE 전무; "XID 641"은 RealTek NIC 오탐). "오후엔 됨"=새 모듈이 재부팅 때 발효. **해결 = proprietary `nvidia-driver-580` 롤백**(`sudo apt purge nvidia-driver-595-open nvidia-dkms-595-open nvidia-kernel-source-595-open && sudo apt install nvidia-driver-580 && sudo reboot`; 확인=`/proc/driver/nvidia/version`에 'Open Kernel Module' 없음) + `apt-mark hold`. **드라이버 수정 완료 → 재학습 완주(hswish5, freeze 없음 = 595-open이 원인이었음 확정).** 재발 시 하드웨어(memtest/PSU/PCIe).

**★ D14 결과 — Gate 5 통과 🎉 (2026-07-10)**:
- 드라이버 롤백 후 재학습 완주(hswish5, 2.528h). 양자화/컴파일 통과(md5 `9bc6520c`, subgraph 1).
- **보드 Gate 5(실물 top-down, D13과 같은 프레임)**: **apple 0.489~0.549 → 0.876/0.899/0.875**(완전 해결). orange 0.85~0.88 / banana 0.83~0.85 / mustard 0.81~0.87(val 하락이 실물엔 무영향) / tennis 0.82~0.85 → **6종 전부 확정, 모델 교체(Gate 2~5) 완성.**
- val Gate 2 apple은 0.625로 D13(0.629)과 동일 — `real_apple_yolo`가 **train-only**라 val엔 개선이 안 잡히는 게 정상(진짜 판정은 보드 Gate 5).
- 정리: `apple:0.40` threshold를 worker+test 스크립트 모두 **0.50 default로 원복**(apple 0.88이라 불필요). orange가 노란 mustard를 0.19로 약하게 오검출하나 배포 threshold(0.50) 아래라 무해. 배포 xmodel=**D14 `9bc6520c`**(D13 백업 없이 덮음 → 필요시 hswish2 best.pt로 재생성).

### ★ Gate 6 실측 (2026-07-10, 4코어·isolcpus OFF, 3분)
- 풀 파이프라인 end-to-end 정상: camera 30Hz, detection/pick/3d/base 모두 **15Hz 동기**, **capture→detection E2E 137ms**(p95 149). **Gate 6 = PASS.**
- 비전 per-frame: **dpu_ms 18**(빠름·안정), pre 18/post 6 → 병목은 DPU 아니라 CPU측 전처리. detection은 compute-bound ~15Hz(YOLO worker가 옛 SSD보다 무거움; 예전 "camera-limited ~17Hz" 주석은 이제 무효 — 카메라는 30fps 공급).
- **CPU: 4코어의 79%(~3.17코어).** 노드별(코어1개=100%): **target_3d 68.8(최대)** / camera 56.9 / detector 49.9 / worker_dpu 36.2 / target_base 13.2 / pick_logic 8.8.
- CSV: `~/vitis_ai_work/perf/{vision_metrics,pipeline_timeseries,cpu_timeseries,gate6_summary}.csv`. 하네스: `perf_probe.py`+`run_gate6_perf.sh`.

### ▶ 남은 것 (최적화 + Gate 7)
- **최적화(3+1 EtherCAT 격리 대비)**: 현재 ~3.17코어라 3코어 격리 시 초과. **1순위 = `pick_target_3d_node`**(69% 최대) — depth를 30Hz로 매 프레임 cv_bridge 변환하나 3D 계산은 ~15Hz만 소비. 대책: **realsense depth 30→15fps**(config, 최대 효과·무코드) + depth 변환 지연(depth_callback→pick_target_callback) + 역투영 루프 scalar화. **YOLO 전/후처리는 이미 SSD LUT/사전필터가 이식됨**(worker `build_input_lut`/`letterbox_lut`, `decode_head` int8 obj 사전필터)이라 추가 여지는 작음(잔여: `send_resized_input`로 IPC↓, `out_buf.fill` 1회화).
- **Gate 7**: live 체크리스트(성능 CSV는 위 실측으로 확보).

### 데스크톱 실행 순서 (데이터 준비 후)
```bash
# 0) 스크립트 동기화 (보드 → 데스크톱). ⚠️ 디렉토리명 training → yolo_v3_tiny_training 로 변경됨
rsync -av ubuntu@192.168.120.50:~/ros2_ws/yolo_v3_tiny_training/ ~/capstone_training/training/
# 1) (D13 1회성, 이미 수행) 7-class → 6-class remap
python3 09_drop_peach_remap.py           # ⚠️ 09는 1회만 (6-class에 재실행 시 orange를 peach로 오인)
# 2) YCB real 5종 변환 (013 apple→0, 017 orange→1, 011 banana→2, 056 tennis→3, 006 mustard→4)
python3 08_ycb_real_to_yolo.py --ycb-dir ~/ycb/013_apple --class-id 0 --cameras N3,N4,N5 --stride 2 --out ~/capstone_training/datasets/ycb_real_yolo
# 3) 병합 + split → data.yaml (6-class, real_apple_yolo 포함)
python3 06_merge_split.py
# 4) 재학습 → Gate0 inspect → 재양자화 → 재컴파일
bash 11_train.sh && bash 12a_inspect_docker.sh && bash 12_run_quantize_docker.sh && bash 13_compile_docker.sh
```

### 보드 재검증 명령 (사용자 명령 없이 Claude가 직접)
```bash
# (1) 파이프라인 기동 (background)
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py > /tmp/pl.log 2>&1 &
# (2) 프레임 캡처 (카메라만 필요 — RT 커널이면 full 파이프라인 대신 카메라만 기동 권장)
python3 ~/vitis_ai_work/scripts/capture_color_frames.py --count 3 --out ~/vitis_ai_work/test_images/runN
# (3) 파이프라인 정지 → DPU 해제 (isolated 테스트 전 필수 — DPU는 1 프로세스만)
pkill -f "pick_place_vitis_ai.launch"; pkill -f realsense2_camera_node; pkill -f vitis_ai_worker_yolo; sleep 3
# (4) 격리 테스트 (물체별 confidence/bbox + overlay)
python3 ~/vitis_ai_work/scripts/yolov3_tiny_image_test.py \
  --model ~/vitis_ai_work/models/yolov3_tiny_7class.xmodel \
  --meta  ~/vitis_ai_work/models/decode_meta.json \
  --image ~/vitis_ai_work/test_images/runN/obj_00.jpg --output /tmp/ov.jpg
```
> **주의**: DPU는 한 번에 한 프로세스만. 파이프라인(worker) 떠 있으면 image-test 충돌. 캡처는 카메라 토픽만 쓰므로 공존 가능하나 isolated 추론 전엔 반드시 파이프라인 정지.

### config 현재 상태 & SSD 복구
`~/ros2_ws/src/system_bringup_pkg/config/` (install→build→src 전부 symlink, **수정 즉시 반영**):
- `vitis_ai_detector.yaml`: model_path→yolov3_tiny_7class, worker→vitis_ai_worker_yolo.py, send_resized_input→false, metrics_duration_sec→0.0.
- `pick_logic.yaml`: allowed_classes→5종(person은 detector threshold 2.0으로 억제, pick_logic에 person 로직 없음 — 2026-09 제거).
- **SSD stand-in 복구**: `vitis_ai_detector.yaml` 상단 주석의 3줄(SSD model_path/worker/resized=true)로 되돌리면 됨.

### 지뢰밭 (이미 밟아서 해결 — 반복 금지)
- yolov5 기본 activation = SiLU(DPU 미지원) → cfg에 `activation: nn.Hardswish()`. **학습 전 `12a_inspect_docker.sh`(Gate 0)로 DPU 매핑 선검증 필수.**
- VAI 2.5 `hardswish.py` 배포 버그 2건 → 12/12a wrapper가 docker 안 sed 자동 패치.
- `metrics_duration_sec`는 double(`0.0`) — 정수면 detector 死.
- `sudo bash 12_*.sh` 금지(내부에서만 sudo).
- 데스크톱 GPU: `nvidia-driver-595-open`(open+GSP)이 CUDA 부하 중 silent hard-freeze → proprietary `nvidia-driver-580`로 롤백 + `apt-mark hold`.

## 4.5 비전 트랙 시간순 히스토리 (요약)

`progress.md`가 정본(2500줄). 주요 마일스톤:
- **초기(mock)**: USB 카메라 → mock_detector(고정 bbox) → pick_logic(pass-through) → pick_target_3d(pinhole). 이후 RealSense로 교체.
- **2026-05-07~18**: RealSense 파이프라인 검증 → 통합 bringup launch → 정적 TF(base_link→camera_link) → base-frame 변환 → 2D 필터(confidence/class/edge/size) → DetectionArray. KV260 Vitis-AI DPU 셋업, 표준 SSD ADAS 검증. **ROS 통합 시 VART execute_async 크래시** → one-shot 파일 기반 detector로 우회(느림).
- **2026-05-19~21**: 크래시 **root cause = XIR graph/subgraph GC** 규명 → 객체 수명 보장으로 해결 → **long-running worker 모드** 확정(~12.9 FPS 검증).
- **2026-06-22**: 카메라 busy-spin stall root cause(1280×720 → 848×480 해결), per-stage timing 계측, CSV metrics, **LUT 전처리 + 후처리 사전필터**(bit-identical), **콜백 파이프라이닝**, **단일점 reverse projection**(align 제거) → ~17 Hz, z 검증.
- **2026-07-07~09**: YOLOv3-tiny 교체(§4.4). Gate 2/3/4 통과, Gate 5 도메인 갭 → D12/D13/D14.

---

# 트랙 B — 로봇 제어 (RT 커널 → EtherCAT → RPU)

Indy7 제어 스택을 3단계로 쌓는다. **비전 트랙과 리소스 충돌 없음**(DPU/카메라 vs 커널/GEM). 단 같은 보드라 재부팅·커널 교체 시 비전 파이프라인도 함께 내려감.

## 5.1 왜 이 순서인가
- EtherCAT 모션 제어는 매 1ms 사이클마다 프레임을 정해진 시각에 보내야 하고, jitter가 제어 품질을 결정.
- APU Linux는 범용 OS라 수십~수백µs jitter. **RT-PREEMPT로 수십µs**까지 개선.
- **RPU(R5F)**는 OS 간섭 없이 코어 전용 + TCM(지연 고정 메모리) → **수µs 이하 jitter** → 이것이 RPU 트랙의 존재 이유.
- 로드맵: **APU에서 IgH EtherCAT + 로봇 제어 먼저**(실용적 1단계) → 이후 RPU(FreeRTOS+SOEM) 이전. GEM3 포트가 하나뿐이라 APU EtherCAT과 RPU는 동시 사용 불가.

## 5.2 Phase A — RT-PREEMPT 커널 패치 (APU) ★ 지금 활성

정본: `rt_patch.md`(인수인계) · **`rt_kernel_postmortem.md`(크래시 종합 보고서 — RT 문제는 여기부터)** · `rt_kernel_fix_plan.md`(진단 상세).

### ★★ RT 커널 크래시 사건 — 규명·해결 완료 (2026-07-08 ~ 07-13)

**증상**: `-rt-kria` 커널이 겉보기엔 부팅·구동됐으나 랜덤하게 hang(전원 리셋 필요). 커널 크래시 3회(07-08 1회 + 07-10 SLUB oops·fpsimd 폭풍 2회). "무부하일 때도 죽음"이 결정적 단서 → 과부하가 아니라 **내부 상태 오염 + 타이밍** 문제로 방향 전환.

**오진들(전부 반증됨)**: 버전 미스매치(rt91=.197 vs .199) / CONFIG_PREEMPT_LAZY / fpsimd 드라이버 / config / "Ubuntu에 RT 자체가 무리" / fork 회계(0x2). — 상세 반증은 postmortem §3.

**근본 원인(2026-07-13 확정)**: **Ubuntu jammy 커널의 `UBUNTU: SAUCE: Revert "radix-tree: Use local_lock for protection"`**(커밋 05fdd32398). mainline이 v5.8에 `radix-tree` preload 보호를 `local_lock`으로 바꿨는데, Ubuntu가 **NVIDIA 독점 모듈 빌드 호환**을 위해 구식 `preempt_disable()`로 되돌림. 일반 커널엔 무해하나, **PREEMPT_RT를 얹으면** `idr_preload()`를 쓰는 모든 경로(fork `alloc_pid`·sysfs·cgroup·crng…)가 "선점 금지 상태에서 잠드는 락 잡기" = atomic-context sleep 위반이 됨. vanilla·linux-xlnx는 정상(local_lock), rt91 패치는 이 파일 무관 — **Ubuntu 베이스만의 지뢰.**

**진단 방법**: ① DEBUG_PREEMPT/DEBUG_ATOMIC_SLEEP 켠 "함정 커널"(`-rt-kv260`)로 부팅 → 위반 리포트 **253건 전수가 단일 원점 `__radix_tree_preload`**를 지목. ② 5개 트리 소스 전수 대조로 Ubuntu만 구식임을 확정.

**해결**: 영향받은 **3파일**(`lib/radix-tree.c`, `include/linux/radix-tree.h`, `include/linux/idr.h`)을 vanilla 5.15.199 원본(local_lock)으로 원복 → 재빌드(rev-5). **config로는 불가**(소스 로직), **전체 재조립(④ linux-xlnx)도 불필요**. vanilla 원본 = `~/ros2_ws/kernel_configs/vanilla-5.15.199-radix-fix/`.

**검증(2026-07-13)**: 설치 전 System.map에 `__ksymtab_radix_tree_preloads`(local_lock판 지문) 확인 → 부팅 후 검출기 완전 무장 상태에서 **부팅 위반 253 → 0건**, DPU(zocl)/카메라 정상. **RT 크래시 종결, ④ 폐기.**

**남은 것**: (a) 소크 테스트 **✅ 통과(2026-07-13; 07-14 재확인, soak v2 부하 중 radix 위반 0)** — 하네스 `soak_rt.sh`. → (b) **[유일 잔여] DEBUG 옵션 끈 프로덕션 rev-6 — 2026-07-14 PC 빌드 진행 중**(`-rt-kv260c`, DEBUG_PREEMPT/ATOMIC_SLEEP=n만 변경; radix 픽스·PREEMPT_LAZY=n 포함 나머지 전부 유지). 설치·검증(무부하+soak 반복) 통과 시 EtherCAT 선결조건 해제. 빌드 게이트·설치 절차 = `rt_patch.md §4-4-2`.

**교훈**: Ubuntu 5.15 + PREEMPT_RT 조합은 이 지뢰를 밟는다(재현성 있음). RT 브링업의 핵심 진단 도구 = DEBUG_PREEMPT/DEBUG_ATOMIC_SLEEP(위반 원점을 즉시 특정). "현장(fpsimd/zocl/SLUB) ≠ 원인". 빌드 함정 A~H는 postmortem §9.

### 목표·현 커널
- KV260 APU(A53) 커널을 RT-PREEMPT로 교체(실시간 제어 지연 보장).
- 원래 커널: `linux-xilinx-zynqmp` 5.15.0-1070.74 (업스트림 베이스 **5.15.199**), PREEMPT_VOLUNTARY / HZ=250.
- 방법: x86 PC에서 aarch64 크로스컴파일. 소스 = **Ubuntu Xilinx 커널 소스 + kernel.org RT 패치** `patch-5.15.197-rt91`(5.15.199에 가장 가까움, reject 0).

### Kria 특수사항 (랩 x86 가이드와 다른 점)
- 부팅 = **U-Boot + flash-kernel(`/boot/firmware/image.fit`), GRUB 없음.**
- **CMA 유지**(FPGA/PL 사용 중 — 랩 가이드의 CMA 비활성화 미적용). cmdline `cma=1000M`, `clk_ignore_unused`.
- **헤드리스 운용**(SSH만 — RT 지터 감소 + 부팅 프리즈 용의자 mali 제거).

### 현재 상태 (2026-07-14 갱신)
| 항목 | 상태 |
|---|---|
| ~~RT 커널 `5.15.199-rt91-rt-kria`~~ | ❌ **폐기(결함)** — radix-tree revert로 크래시. 위 사건 블록 참조 |
| RT 커널 `-rt-kv260b` (build #8, DEBUG on 검증판) | ✅ radix 253→0 + zocl E2 진단 완수. **kv260c로 대체됨**(설치는 유지=복구경로) |
| ~~`-rt-kv260` rev-5~~ | kv260b로 대체됨(개선판) |
| **zocl RT 위반 trace#1 (07-13)** | 🔶 별개·저심각 — DPU xclbin 로드 시 rcu-read 안 vmalloc(sleeping). radix 무관, config로 제어 불가(vendor 소스). 1회성, DPU 정상. 아래 치명 크래시(trace#2)의 원인 아님(공통원인) |
| **★ zocl DPU 커널 크래시 trace#2 (07-14)** | ✅✅ **해결·검증 완료(07-15).** E2(slub_debug=FZPU) 계측 재현으로 `Poison overwritten` 생포 → `kds_core.c`의 `xrt_cu_submit()` 뒤 `set_xcmd_timestamp(KDS_QUEUED)` = **submit-후-타임스탬프 UAF**(CU 스레드가 먼저 free하면 해제 메모리에 타임스탬프 씀 = SLUB freelist 오염). RT의 넓은 preemption이 노출(메커니즘1 확정, SMMU/DMA 폐기; upstream XRT master에도 잔존). **수정 = 타임스탬프↔submit 순서 교체**(3곳). kv260c(#10)에 합류 빌드 → **계측(330s)·프로덕션(200s+) 재현 Poison/Oops 0건 = 픽스 확정.** 패치 `~/ros2_ws/zocl_patches/`, 상세 `rt_kernel_postmortem.md §12-8`, 메모리 `zocl-dpu-rt-kernel-crash` |
| **✅✅ 프로덕션 `-rt-kv260c` (#11, DEBUG off + zocl 픽스) ← 현재 구동** | **RT 트랙 최종 커널(2026-07-15 #10 → 2026-07-20 #11).** #11은 #10에 `CONFIG_USB_NET_AX88179_178A=m`만 추가한 빌드라 아래 검증·cyclictest 수치는 그대로 유효. kv260b config에서 DEBUG_PREEMPT/ATOMIC_SLEEP off + zocl `kds_core.c` UAF 순서교체 패치. 빌드·설치·부팅·검증 완료(realtime=1). zocl 재현 Poison/Oops 0(계측 330s+프로덕션 200s). cyclictest load Max 142µs·위반 0. **EtherCAT 선결조건 해제** |
| cmdline (cma=1000M 등) 유지 | ✅ |
| 헤드리스(multi-user.target + gdm stop) | ✅ (rootfs 설정 — 커널 무관 공통) |
| mali GPU blacklist | ✅ (initrd 반영) |
| **DisplayPort IRQ 낭비 제거** | ✅ RT initrd에 dpsub blacklist 반영(rev-5도 포함 확인). **순정도 영구화 방법 확정(2026-07-14)** = `blacklist zynqmp_dpsub`+`update-initramfs -u`(모듈이라 initrd 갱신 필요, §4.2). 사용자 sudo 실행 대기 |
| **CPU 격리 (isolcpus)** | ✅ **3+1 재도입 완료**(D10) — cmdline `isolcpus=3 nohz_full=3 rcu_nocbs=3 irqaffinity=0-2 skew_tick=1`, 비전=CPU0-2 / 1kHz 제어 페어=CPU3. A/B 실측: 격리 CPU3 max **51.1 µs** vs 비격리 CPU0 114.6 µs. ⚠️ **eth0 IRQ가 CPU1인 것은 `irqaffinity=0-2`로 의도한 설계 — CPU3으로 옮기지 말 것**(IK 구간 동안 프레임 정지). ~~해제(A안)~~ |
| 간헐적 부팅 프리즈 | ✅ **원인 규명·해결** — radix-tree 결함의 발현이었음(rev-5에서 소멸) |
| apt-mark hold (커널 3종) | ✅ |
| IgH EtherCAT Master | ✅ **실운용 중** — `/opt/etherlab`에 stable-1.6(태그 1.6.10) + generic 드라이버 설치, RAON-RT 앱이 Indy7을 1 kHz CST로 구동(§5.3 상태 갱신) |
| 비전 파이프라인 코어 사용량 | ✅ **최적화 완료 = ~1.8코어**(phase 1+2, 76.8→~44%/4코어 · §4.2). 여유 ~2.2코어 → 3+1 격리 + EtherCAT 마진 충분(§4.2 헤드룸 판정). ~~3+1 대비 target_3d 최적화 필요~~ 해소 |
| RT 인증 소크(부하 무위반) | ✅ **통과(2026-07-13)** — 누적 radix 위반 0건(v1 load156 9.6분+v3 load50 14분, DEBUG 검출기 무장). `~/ros2_ws/soak_rt.sh`. (v3 조기종료는 빌드 무관 claude 프로세스 메모리압박 크래시, 커널·위반과 무관) |
| cyclictest RT 지연 baseline | ✅ **최종 kv260c(DEBUG off, 2026-07-15)**: idle Max 134µs(Avg 11~13) / **load Max 142µs(Avg 14~19)** / 부하 중 커널 위반 0. **DEBUG off로 kv260b load Max 282→142µs(절반)·코어편차 타이트(127~142)**. idle Max 134는 단발 outlier(Avg 낮음). 여전히 >100µs는 **격리無 4코어 공유** 조건 탓 — `<100µs`는 EtherCAT 3+1 격리 코어 몫. 교훈: 판정용은 `-S`(코어별 고정) 필수. 하네스 `cyclic_rt.sh`. 로그 `crash_logs/cyclic_20260715-013556.log` |

### ⚠️ CPU 격리 사건 (2026-07-09, 핵심 교훈)
- **2+2 격리(isolcpus=2,3)가 비전+SSH를 죽임.** 격리로 비전+전체 시스템+모든 IRQ(USB 카메라 xhci→cpu0, eth0→cpu1)를 코어 0,1에만 몰아넣었는데 비전이 **~2.8코어**를 써서 2코어에 안 들어감 → 카메라 USB 서비스 굶음(frame_age 7500ms stall) + 네트워크 굶음(SSH 끊김). 정작 격리 코어 2,3은 놀고 있었음(제어 루프 없음).
- **조치**: 격리 해제(A안). cmdline `skew_tick=1`만. RT 커널·PREEMPT_RT는 유지(문제는 격리 전략이지 커널 아님).
- **재도입 방침**: **EtherCAT 통합 시작할 때 3+1(isolcpus=3)** — 비전=0,1,2 / 제어=격리코어 1개. (1kHz EtherCAT+제어는 격리 1코어면 충분.) → ✅ **실행됨**(D10, 2026-07-28 A/B 실측 완결).

**비전 파이프라인 코어 사용량 실측 (4코어 정상, yolov3_tiny)** — 총 ~2.8코어:
| 프로세스 | 코어 | 비고 |
|---|---|---|
| **pick_target_3d_node** | **~0.73** | ★ #1 소비자. "1픽셀 reverse projection"인데 과다 → **full depth 매프레임 처리 의심, 최우선 최적화 대상** |
| realsense_camera_node | ~0.55 | USB 단일스레드 병목 |
| vitis_ai_detector_node | ~0.48 | 전/후처리·IPC |
| vitis_ai_worker (DPU) | ~0.38 | NN은 PL DPU 오프로드 |
| pick_target_base / pick_logic / USB kworker+softirq | ~0.14 / ~0.10 / ~0.4 | |

**함의**: 3+1(비전 3코어)은 2.8<3이라 되지만 마진 얇음(~0.2코어). `pick_target_3d_node` 최적화(~-0.5코어 기대)하면 여유. **궁극 해답 = RPU 이전**(제어를 R5+TCM로 옮기면 A53 4코어 전부 비전 → 경합 소멸).

### 커널 빌드 핵심 (rt_patch.md §4)
- 소스 취득: Launchpad **소스 패키지 직다운로드**(git repo는 504). `dpkg-source` 추출 후 **`chmod -R +x scripts/ debian/rules debian/scripts/`**(함정①).
- RT 패치: `xzcat patch-5.15.197-rt91.patch.xz | patch -p1` → `.rej` 없음.
- config: PREEMPT_RT=y, HZ_1000=y, **CMA=y 유지**, KVM/CPU_IDLE/CPU_FREQ off, LOCALVERSION `-rt-kria`.
  - 함정②: **KVM 켜져 있으면 PREEMPT_RT 선택 불가**(arm64 `ARCH_SUPPORTS_RT if !KVM`) → KVM off.
  - 함정③: CPU_IDLE은 ACPI_PROCESSOR가 select → 둘 다 off.
- 빌드: `make -j$(nproc) bindeb-pkg` → image/headers .deb.
- 보드 설치 함정④: flash-kernel DB의 `Kernel-Flavors: xilinx-zynqmp` 필터가 우리 커널 무시 → `/etc/flash-kernel/db`에 `Machine: ZynqMP *KV260*` + `Kernel-Flavors: any` 추가.
- 보드 설치 함정⑤: FIT 템플릿이 DTB를 `/lib/firmware/<버전>/device-tree/xilinx/`에서 참조하나 bindeb-pkg는 `/usr/lib/linux-image-<버전>/xilinx/`에 설치 → **커스텀 커널마다 DTB 복사 필수**.
- DisplayPort IRQ 제거 함정: **blacklist만으론 재부팅 후 재로드**(dpsub.ko가 initramfs에 있고 `console=tty1`로 조기 로드) → `blacklist` + **`update-initramfs -u -k <버전>` 필수**. (교훈: 모듈 blacklist가 재부팅 후 안 먹으면 initramfs 확인.)

### 커널 전환/복구 (rt_patch.md §8, §10 · postmortem §9 함정 E/F)
```bash
# 현재 커널 확인
uname -r                    # 5.15.199-rt91-rt-kv260b=RT(현행 최신) / 5.15.0-1070-xilinx-zynqmp=순정(현재 부팅)
cat /sys/kernel/realtime    # 있고 1이면 RT
# RT ↔ 순정 전환 (image.fit만 교체, rootfs 무영향, 왕복 가능)
# ★ 함정 F: 지정 커널이 "설치된 최신"이 아니면 --force 필수(아니면 조용히 무시 exit 0)
sudo flash-kernel --force 5.15.0-1070-xilinx-zynqmp && sudo reboot   # → 순정 (구버전이라 --force)
sudo flash-kernel 5.15.199-rt91-rt-kv260b && sudo reboot              # → RT (정렬상 최신이라 --force 불필요)
```
- 버전 정렬(함정 F): `-rt-kv260b` > `-rt-kv260` > `-rt-kria` > `5.15.0-1070` (문자열). auto-trigger가 최신을 고르므로, 순정 상태에서 커널/initramfs 작업 시 RT가 최신이면 조용히 RT FIT을 구울 수 있음 → 순정 전용으로 쓰려면 RT 패키지 제거하면 지뢰 소멸.
- ⚠️ **순정 커널 패키지를 절대 apt remove 하지 말 것**(복귀 수단).
- `/etc/default/flash-kernel`의 `LINUX_KERNEL_CMDLINE`은 **양쪽 커널 공통 적용**.
- 완전 제거 절차는 `rt_patch.md §10`(순정 부팅 → RT 패키지 purge → 수동 파일 되돌리기 순).

## 5.3 Phase B — IgH EtherCAT Master (APU)

> **상태 갱신 (2026-09-04)**: 아래 본문은 **착수 전 계획**이다. **Phase B는 완료를 넘어 실운용 중** —
> IgH(git stable-1.6, 태그 **1.6.10**)가 `/opt/etherlab`에 설치되어 RAON-RT 앱이 Indy7을 **1 kHz CST**
> (`DRIVE_MODE=10`, merge.md D1 — 본문의 CiA402+**CSP** 서술은 낡음)로 구동하고, 그 위에서 **접근·추종
> 앱**이 돈다(§0 ★ 2026-09-04). 원격 접속은 **USB NIC(AX88179B)로 분리**되어 eth0는 EtherCAT 전용이
> 되었으므로 아래 "네트워크 계획 필수" 경고는 **해소**됐다. **3+1 CPU 격리(isolcpus=3)도 재도입 완료**(D10,
> 격리 CPU3 max 51.1 µs vs 비격리 CPU0 114.6 µs). 제어 트랙의 현재 사실은 `docs/RAON-RT/merge.md`가 정본.

정본: `rt_patch.md §7`.
- APU에서 **IgH EtherCAT Master 1.6.2** 구동. NIC 드라이버 = igc가 아니라 **Cadence GEM(macb)** → generic 드라이버(`--enable-generic`) 필요.
- **GEM3=ff0e0000 (eth0, 유일한 활성 GEM)**. Indy7는 STEP 우회·드라이브 직결(CiA402+CSP). 사용자가 **PC에서 EtherCAT으로 Indy7 제어 성공한 코드 보유** → 이식 작업. 테스트 드라이브 = LS Mecapion **L7N**. 제어주기 1kHz 추정.
- ⚠️ **네트워크 계획 필수**: eth0 하나뿐 + 현재 원격접속이 eth0 → EtherCAT 점유 시 원격 불가. 대체 접속(USB 이더넷/WiFi/시리얼 콘솔) 확보 후 착수. 보드는 손 닿는 위치(UART 가능).
- configure 예시: `./configure --prefix=/opt/etherlab --enable-generic --disable-8139too --enable-hrtimer --enable-eoe=no` (인텔 전용 옵션 제외). 소스 = `gitlab.com/etherlab.org/ethercat.git` stable-1.6. 빌드는 보드에서 직접(headers 설치됨).
- 이후: `ethercatctl start` → `ethercat slaves`(L7N 인식) → CiA402+CSP.
- **EtherCAT 통합 시작 시 3+1 CPU 격리 재도입** + 코어3 통합부하 레이턴시 측정.
  - ⚠️ 통합부하 레이턴시는 stress-ng 합성부하로 안 드러남 — **실제 비전+DPU+EtherCAT 돌리며** 격리 코어 측정해야 보임: `sudo cyclictest -m -p 90 -i 1000 -a 3 -D 5m -q`, Max<100µs면 통과.
  - 잔존 리스크: **L2/DDR 경합**(A53 4코어가 L2 1MB+DDR 공유, 비전 메모리 대역 큼 → isolcpus로 못 막는 스파이크) — 이것이 결국 **RPU 이전의 근본 근거**. EtherCAT generic 드라이버는 net-stack 경유 → eth0 IRQ를 격리 코어 쪽에 pin 검토.

## 5.4 Phase C — RPU FreeRTOS + SOEM (이후)

정본: `rpu_guide_for_claude.md`(새 세션 진입점), `rpu_plan.md`(논리), `rpu_freertos_soem_execution_plan.md`(명령·게이트).

> **상태 갱신 (2026-09-04) — Phase 0~1 완료, 남은 것은 3가지**
>
> - **Phase 0 완료**: Vitis 2022.1 설치됨(Windows `d:\xilinx\vitis\2022.1`, GCC 11.2.0 arm-none-eabi,
>   BSP `freertos10_xilinx_psu_cortexr5_0`, XSA = 자체 Vivado BD export `design_1_wrapper`). BSP 라이브러리에
>   `gpiops`/`scugic`/**`ttcps`**(Phase 3~4의 µs 시계 재료)가 이미 포함.
> - **Gate 1 실질 통과 (작업 2026-09-01~02 / 펌웨어 부팅 검증 2026-09-02 11:02·15:01 2회)**: `remoteproc`으로
>   R5F-0에 `r5_freertos.elf` 부팅 성공(`remote processor ff9a0000.rf5ss:r5f_0 is now up`) + **LED 태스크
>   실행**(`led35_task`/`led36_task`; **물리 점멸은 사용자 육안 확인** — `/sys/class/leds/`에 미노출이라
>   보드에서 재검증 불가). ⚠️ **09-01은 DT enumeration까지**다(12:19 `could not get R5 core platform device`,
>   12:57 `r5f_0 is available`) — `is now up` 0건이므로 펌웨어 부팅은 09-02 2회가 전부.
>   ⚠️ **판정 수단은 UART가 아니라 LED였다** — 아래 "Gate 1: UART/trace" 기준은 결과적으로 미사용.
>   start/stop **반복** 안정성은 아직 미확인(부팅 2회 각각 `is now up` 1줄씩만 확인됨).
> - **적용 방식이 계획과 다르다**: configfs runtime overlay가 아니라 **부팅 DTB 병합**
>   (`dtc -@` → `fdtoverlay -i base.dtb` → `/boot/firmware/user-override.dtb`, U-Boot `boot.scr` 훅이 로드).
>   `flash-kernel`/`image.fit` 재생성 불필요. ⚠️ **재작업 시 `user-override.dtb`는 누적되지 않는다** — 항상
>   `base.dtb`에서 다시 병합할 것. 산출물은 저장소 밖 `~/rpu_work/`에 있고,
>   `tools/rpu/rpu_rproc.dtso`는 **적용된 적이 없다**. ⚠️ **둘 다 git 미추적이다** — `tools/rpu/`도
>   `git ls-files`에 없다(`git status`에 `?? tools/rpu/`).
> - **메모리 배치 확정 = `rproc@36000000` + 0x400000(4 MiB) no-map** (`/proc/device-tree/reserved-memory/`).
>   실제 사용은 ~78 KB뿐 + TCM ATCM에 `.vectors`. ⚠️ **문서가 관례라 적어둔 0x3ed00000은 이 보드의 CMA 창
>   (`0x36400000`+1000 MiB) 한복판이라 폐기**. carve-out을 4 MiB 위로 늘리면 CMA 시작을 침범한다.
> - **남은 것은 정확히 3가지** (사용자 정의):
>   | # | 과제 | 이 문서상 위치 | GEM3 필요? |
>   |---|---|---|---|
>   | ① | **APU↔RPU 통신**(rpmsg) | Phase 5의 rpmsg 부분 | **불필요 → 지금 진행 가능** |
>   | ② | **RAON-RT 실시간 제어 task의 RPU 이식** | Phase 4 중 EtherCAT 무관한 연산·주기·지터 부분 | **불필요 → 지금 진행 가능** |
>   | ③ | **EtherCAT SOEM의 RPU 설치** | Phase 3(+선행 Phase 2) | 빌드까지는 불요, **PHY 링크업·OP 도달은 필수 → 차단** |
> - **①의 선행 조건(현 부팅 구성으로는 원리적으로 불가)**: `r5f_0`에 `mboxes`/`mbox-names` 없음,
>   reserved-memory에 vring/buffer 없음, RPU용 IPI mailbox 노드 없음, 펌웨어에 `.resource_table` 없음
>   (커널 로그 `no resource table found.` — **경고만 뜨고 부팅은 됨**). → **DT 재작업(재부팅 1회) + OpenAMP
>   베이스 펌웨어 재빌드**가 함께 필요. 다행히 4 MiB carve-out 중 ~78 KB만 쓰므로 vring/buffer를
>   **0x36000000 창 안에** 넣으면 CMA를 안 건드린다. `tools/rpu/rpu_rproc.dtso`의 **노드 이름 규칙**
>   (`rpu0vdev0vring0` — 드라이버가 `name[14]`에서 번호를 파싱)과 IPI 노드 구성은 주소만 바꿔 재사용 가치가 있다.
> - **③은 사용자 재결정 전 착수 금지**: GEM3(eth0)는 APU IgH의 EtherCAT 포트이고 **RAON-RT가 Indy7을 실제로
>   제어 중**이다(`rpu_guide_for_claude.md:30`).
> - ⚠️ **재부팅이 리스크다**: ①은 DT 변경 → 재부팅 수반. 교체 전 `/boot/firmware/user-override.dtb.ok`로
>   백업하고(파일이 없으면 `boot.scr`가 FIT의 순정 DTB로 폴백), **RT 실기 세션과 반드시 분리**
>   (`systemctl is-active ethercat` 확인). CPU3은 격리 코어이므로 RPU 작업의 APU측 부하는 CPU0-2에.

### 목표 아키텍처
```
┌────────────────── KV260 (ZynqMP) ──────────────────┐
│  APU (A53×4)                RPU (R5F-0, split)      │
│  Ubuntu + ROS2              FreeRTOS 10             │
│  perception                 SOEM + 1kHz 제어 루프    │
│  CROS2PickBridge     ⇄⇄⇄   OpenAMP rpmsg           │
│                             PS GEM3 ── RJ45 ────────┼── L7N → Indy7 드라이브 체인
└─────────────────────────────────────────────────────┘
```
**설계 원칙**: 실시간이 필요한 것(1kHz PDO, CiA402 상태머신, 궤적 보간)은 전부 RPU 안에서 닫는다. APU↔RPU 경계는 "다음 목표점"과 "상태 보고"만. → APU가 바빠도 제어주기 무관.

### 확정 결정 (재논의 금지)
| 결정 | 내용 |
|---|---|
| 실행 코어 | R5F-**0**, **split** 모드 (lockstep 불필요) |
| RTOS | FreeRTOS 10 (Vitis BSP `freertos10_xilinx`) |
| EtherCAT master | **SOEM** v1.4.x (RPU) / IgH는 APU 전용 |
| Vitis 버전 | **2022.1** (kria-apps-docs 기준, 커널 5.15 세대 일치). ✅ **설치 완료**(2026-09 이전, Windows) |
| RPU 로드 | Linux **remoteproc** (BOOT.BIN 아님, 안정화 후 systemd). ✅ 검증됨(2026-09-02) |
| 코드 배치 | DDR 예약 영역 링크, 지연민감 코드만 TCM(128KB 초과). 실제 = **0x36000000 4 MiB** + `.vectors` TCM |
| DMA 버퍼 | 처음엔 **non-cacheable**(R5 MPU) — 캐시 일관성 버그 원천 차단 |
| SOEM 수신 | 처음엔 폴링, 이후 인터럽트+큐 |
| 작업 순서 | **Phase 2(GEM3 이관)는 사용자 재결정 전 금지** — IgH가 Indy7 실운용 중. Phase 0~1 ✅완료, 남은 3과제 중 ①②는 GEM3 무관이라 **지금 진행 가능** |
| 대상 로봇 | Indy7, 드라이브 직결, CiA402+**CST**(`DRIVE_MODE=10`, merge.md D1). 테스트=L7N. 주기 **1kHz 확정**(APU 실운용값) |
| 범위 | 그리퍼 제외(하드웨어 미보유), 말단부가 물체 위 정지 + **사람이 물체를 옮기면 추종**까지 |

### 핵심 난제 = "SOEM 포팅"
SOEM은 `application/core`(순수 C, 무수정) + `osal`(OS 추상화) + `oshw`(NIC 접근). Linux는 osal=POSIX, oshw=raw socket 한 줄이라 포팅 불필요. **FreeRTOS/R5엔 커널 서비스가 없어 직접 구현** = 포팅.
- **최대 난제 = DMA 캐시 일관성**: GEM MAC이 DMA master라 CPU 캐시 우회 → 캐시에만 있는 데이터/옛 데이터 전송·수신. 증상 최악(간헐적 깨짐, printf 넣으면 사라짐). **대응: DMA 버퍼/descriptor를 R5 MPU로 non-cacheable 설정하고 시작.**
- 포팅 난이도의 **80%가 Phase 2(하드웨어 계층)**, Phase 3(SOEM 자체)은 20%. 그래서 Phase 2와 3을 분리.

### Phase별 (불확실성 하나씩 제거)
```
Phase 0  Vitis 2022.1 + R5 BSP 플랫폼        "빌드/볼 수 있는가"     ✅ 완료
Phase 1  remoteproc + FreeRTOS + DT overlay   "코드 올리고 반복 가능한가" ✅ 완료(2026-09-02, 판정=LED)
Phase 2  GEM3 이관 + raw frame 왕복 ★최대난제  "이더넷 HW 소유 가능한가"  (APU IgH 완료 후)
Phase 3  SOEM 포팅 (osal/oshw)                "EtherCAT 프로토콜 도는가"
Phase 4  1kHz DC + CiA402 (L7N→Indy7 이식)     "실시간 모션 되는가"
Phase 5  rpmsg + ROS2 통합 (CROS2PickBridge)  "전체 이어지는가"
```
- **Gate 1**: UART/trace로 FreeRTOS 태스크 로그 + start/stop 안정. → ✅ **2026-09-02 실질 통과** (단 판정은 UART/trace가 아니라 **LED**로 했고, start/stop **반복** 안정성은 미확인).
- **Gate 2**: RPU↔PC 임의 L2 프레임 수천 회 무손실 왕복 (EtherType 0x88A4, Wireshark).
- **Gate 3**: L7N PREOP→SAFEOP→OP + PDO read.
- **Gate 4**: 1kHz jitter 수µs 이하 + L7N CSP 위치 제어 + Indy7 다축.
- **Gate 5(최종)**: perception → RPU → **Indy7 말단부가 물체 바로 위 정지 + 사람이 물체를 옮기면 추종**, end-to-end 1회. (현재 이 end-to-end는 **APU 경로로 이미 성립** — Gate 5는 "같은 동작을 RPU 경로로"라는 뜻이 됐다.)

**1ms 사이클 타이밍 예산**: EtherCAT 왕복(6~7 slave)~50–100µs + 제어연산~10–50µs + SOEM/non-cached~수십µs = **<200µs(예산 20% 미만, 여유 5배+)**. 1kHz는 여유 큰 목표.

### 검증된 보드 사실 (rpu_guide §3)
| 사실 | 2026-07-08 결과 |
|---|---|
| remoteproc binding = **`xlnx,zynqmp-r5-remoteproc`** (Xilinx 5.15 벤더, upstream 6.x `zynqmp-r5fss`와 다름!) | alias 확인 |
| ~~DT에 R5 노드 없음~~ → **해소(2026-09-02)**: 부팅 DTB 병합으로 `rf5ss@ff9a0000`/`r5f_0`/TCM 2개/`rproc@36000000` 상주, `zynqmp_r5_remoteproc` 자동 probe | 현재 `/sys/class/remoteproc/remoteproc0` = `ff9a0000.rf5ss:r5f_0` (state `offline`) |
| configfs overlay 동작(smartcam이 이 방식). **단 RPU는 이 방식을 쓰지 않는다** — 부팅 DTB 병합 | overlays에는 `kv260-smartcam_image_1` **하나뿐** |
| GEM3만 활성(ff0e0000, eth0), 나머지 disabled | okay/disabled |
| UART0(ff000000) disabled, UART1=Linux 콘솔(ttyPS1) | disabled |
| CMA 1000M 중 **여유 ~13MB**(DPU 점유) | CmaFree 13448 kB |
| rpmsg 모듈 존재(rpmsg_char.ko) | ✅ |
| 기존 IPI: `mailbox@ff9905c0`, **`xlnx,ipi-id = <4>`** — R5 overlay는 이와 **충돌하지 않는 id** 사용 | `<4>` |

### 금지·주의 (세션 망치는 지름길)
1. **eth0 관련 명령(macb unbind, ip link down, GEM3 DT disable)은 사전 고지 없이 금지** — 원격접속 즉사. 복구=`echo ff0e0000.ethernet > .../macb/bind` 또는 재부팅.
2. **QSPI 부트펌웨어(`xmutil bootfw_*`) 건드리지 말 것.**
3. **kv260-smartcam overlay 내리지 말 것**(`xmutil unloadapp` 금지) — DPU가 그 위에서 돎.
4. `/boot/firmware/image.fit` 직접 수정 금지 — flash-kernel 절차만.
5. RPU reserved-memory를 runtime configfs로만 잡은 채 실사용 금지(무작위 메모리 오염) — boot-time 반영 후. → ✅ **지켜짐**: 부팅 DTB 병합으로 `rproc@36000000` no-map 확보 후 사용 중.
6. upstream 6.x DT 예제(`xlnx,zynqmp-r5fss`) 쓰지 말 것 — 이 커널은 벤더 형식.
7. FreeRTOS elf에 resource table 없으면 **기능 제한** → rpmsg/trace 쓰려면 **OpenAMP echo-test 템플릿을 베이스로**. (실측 정정: 현 elf엔 resource table이 없고 커널이 `no resource table found.` **경고만** 낸 뒤 정상 부팅했다 — "로드 실패"는 과장. 대신 `trace0`가 없어 로그 수단이 LED뿐이며, **남은 과제 ①의 선행 조건**이 바로 이것이다.)

### GEM3 이관 함정 (Phase 2)
- **함정 1 (클럭 게이팅)**: macb unbind 시 자기가 켠 GEM3 클럭을 끔(`clk_ignore_unused`는 부팅 시 미사용 클럭만 관여, 드라이버 명시적 off는 못 막음) → RPU emacps 초기화 실패. 해결: (a) RPU 펌웨어가 XilPM/CRL_APB로 클럭 자립 기동(권장) 또는 (b) boot-time DT로 GEM3 disabled(SSH 끊김).
- **함정 2 (DMA 캐시)**: BD ring+버퍼를 non-cacheable로. 최적화는 Gate 2 후.
- **함정 3 (PHY DP83867)**: RGMII delay가 링크업 실패 단골.
- **함정 4 (인터럽트)**: GEM3 IRQ(SPI 63, GIC 95)를 RPU scugic에 등록. unbind 선행이면 안전.

## 5.5 남은 확인 항목 (로봇 제어)
- [x] Indy7 PC 제어 코드의 EtherCAT master 종류 → **IgH 확정**(APU에서 실운용 중) ⇒ RPU는 SOEM이므로 **번역 계층 필요 확정**.
- [x] 제어 주기 확정 → **1kHz**(APU 실운용값, cfg `[TASK0]/[TASK1]`).
- [ ] L7N 문서 PDF 수령 → `~/ros2_ws/docs/l7n/` (Phase 3 착수 시).
- [ ] KV260 캐리어의 PS UART0 물리 라우팅 (UART0 쓰려는 경우만).
- [x] RPU reserved-memory 최종 주소 → **`rproc@36000000` + 4 MiB no-map 확정**(부팅 DTB). 관례로 적어둔 0x3ed00000은 **CMA 창 한복판이라 폐기**.
- [ ] APU↔RPU rpmsg용 DT 재작업(IPI mailbox + vring/buffer, 0x36000000 창 안) + resource table 포함 펌웨어 재빌드 ← **남은 과제 ①**
- [ ] RAON-RT 제어 코드의 R5F 이식 범위 판정(전체 vs 보간·CiA402만) — C++/double·RBDL 의존이 R5F(단일 코어, VFP, TCM 128 KB)에서 성립하는지 **미확인**. 4 MiB carve-out도 재산정 대상 ← **남은 과제 ②**
- [ ] R5F start/stop **반복** 안정성(현재 각 부팅 1회씩만 확인).

---

## 6. 트랙 간·단계 간 충돌 주의

- ~~**GEM3(eth0)가 유일한 활성 이더넷** → 원격접속 끊김~~ → **해소**: 원격은 **USB NIC(AX88179B)**로 분리됐고 eth0는 EtherCAT 전용으로 예약됐다. 이 전제 위에 세워졌던 "대체 네트워크 확보 후 착수" 조건은 소멸.
- **RPU의 GEM3 이관(Phase 2, 남은 과제 ③)은 APU IgH가 Indy7을 실운용 중이라 사용자 재결정 전 금지.** GEM3 하나를 APU IgH와 RPU SOEM이 동시 사용할 수 없다. 단 **Phase 0~1은 완료**됐고, 남은 과제 ①(rpmsg)·②(제어 task 이식)는 **GEM3와 무관해 지금 진행 가능**(§5.4).
- **RPU 작업의 재부팅이 실기 세션과 충돌한다**: DT 변경은 재부팅을 수반하므로 `systemctl is-active ethercat` 확인 후, RAON-RT 실기 일정과 분리해서 진행.
- **비전 트랙과 로봇 제어 트랙은 리소스 충돌 없음**(DPU/카메라 vs 커널/GEM). 단 같은 보드라 재부팅·커널 교체 시 비전 파이프라인도 함께 내려감.
- **RT 커널 CPU 격리 부담**: **3+1 격리가 재도입된 상태**(isolcpus=3, §5.2)라 비전은 CPU0-2 3코어를 쓴다(비전 ~1.8코어라 여유는 있음). 이 조건에서는 비전 재검증 캡처를 full 파이프라인 대신 **카메라만 기동** 권장(코어 포화로 eth0/SSH stall 회피 — 과거 2+2 격리에서 frame_age 7500ms stall + SSH 끊김 사고). DPU 추론 자체는 RT 커널에서 정상 확인됨.

---

## 7. 통합 TODO 스냅샷

**비전** (§4):
- [x] D13 6-class Gate 5 통과 (5종 확정)
- [x] D14 apple 실물 재학습 완주 (hswish5; 데스크톱 nvidia 595-open→580 롤백으로 freeze 해결)
- [x] 재양자화·재컴파일·보드 Gate 5 → **실물 apple 0.5→0.88, 6종 확정 = 모델 교체(Gate 2~5) 완성**
- [x] apple threshold 0.40→0.50 원복 (worker + test 스크립트)
- [x] Gate 6 풀 파이프라인 실측 (15Hz 동기·E2E 137ms·CPU 79%/4코어; target_3d 69% 최대)
- [x] **비전 CPU 절감 phase 1+2 완료** (2026-07-14, 76.8→~44%, **-1.25코어**, 무손실 검증 · §4.2). ~2.2코어 여유 확보 → EtherCAT 통합 마진 충분(§4.2 헤드룸 판정)
- [ ] 비전 **성능** 개선(fps/latency) — phase 3 레버 카탈로그 §4.2. **RT 커널 완성 후 착수**(전략 결정 2026-07-14)
- [ ] Gate 7 live 체크리스트

**로봇 제어** (§5):
- [x] Phase A: RT-PREEMPT 커널 패치 (빌드·부팅·검증, 헤드리스, DP IRQ 제거, apt hold)
- [x] **RT 커널 크래시 근본 원인 규명·해결** (2026-07-13, Ubuntu radix-tree revert → 3파일 원복, 253→0. postmortem)
- [x] RT 인증 소크 통과 (2026-07-13, 누적 위반 0 — v1 load156 + v3 load50, DEBUG 검출기 무장)
- [x] **zocl DPU 크래시 근본원인 규명 + 픽스 작성** (2026-07-14 심야, postmortem §12-8) — E2 계측으로 KDS submit-후-타임스탬프 UAF 확정, 패치 `~/ros2_ws/zocl_patches/` 완성
- [x] **프로덕션 rev-6(`-rt-kv260c` #10, DEBUG off) + zocl UAF 픽스 빌드·설치·검증 완료 (2026-07-15)** — 계측·프로덕션 재현 Poison/Oops 0, cyclictest load Max 282→142µs. **RT 트랙 종결, EtherCAT 선결조건 해제**
- [x] 비전 CPU 최적화 완료 (phase 1+2, **-1.25코어** · §4.2) → 3+1 격리 여유 확보
- [x] **Phase B: APU IgH EtherCAT master + Indy7 제어 코드 이식 + 3+1 격리 재도입** — IgH 1.6.10 `/opt/etherlab`, RAON-RT 앱이 Indy7을 1 kHz CST로 실기 구동, D10 3+1 격리 배치 완료(격리 CPU3 max 51.1 µs vs CPU0 114.6 µs). 정본 `docs/RAON-RT/merge.md`
- [x] **앱 = 물체 위 접근('v')** — 2026-07-31 원샷 7/7, 실측 최종오차 **6~12 mm**(refine 0, 10건 6.15~11.61 mm) / 2026-08-05 세션 재현 4.2~5.9 mm (실기 검증 완료)
- [ ] **앱 = 추종('o') v3 하이브리드 실기 검증** — 구현 완료(2026-08-03), 검증 사다리 8단계(정적 호버 → 상실 → 저속 → **속도 캡 증명** → 경계 → 폴트 드릴 → 핸드헬드) 미실시. ⚠️ 속도 캡 증명 전 물체를 손으로 들지 말 것
- [ ] operator console(GUI ↔ `/operator_key`·`/operator_jog`·`/operator_gains`·`/robot_state`) 실기 검증 — 커밋 `4b2113a` 본문 자체가 "Built and linked. Not yet exercised on the robot."
- [x] **Phase C Phase 0~1: Vitis 2022.1 설치 + remoteproc + FreeRTOS 부팅 + LED 태스크 실행(점멸은 사용자 육안 확인) — Gate 1 실질 통과(작업 2026-09-01~02 / 펌웨어 부팅 검증 2026-09-02 11:02·15:01 2회)**
- [ ] Phase C ①: APU↔RPU 통신(rpmsg) — DT 재작업 + resource table 포함 펌웨어 재빌드 선행
- [ ] Phase C ②: RAON-RT 실시간 제어 task의 RPU 이식 (EtherCAT 없는 1 kHz dry-run + 지터 측정)
- [ ] Phase C ③: EtherCAT SOEM의 RPU 설치 (Phase 2~4) — **GEM3 이관은 사용자 재결정 전 금지**

**공통 시스템**:
- [x] base_link↔camera_link TF **calibration** — 2026-07-27 eye-to-hand(16 pose, AX=ZB, residual 1.8 cm/1.7°) + E21 base 재베이스(yaw +5.14°) 반영. `/pick_target_base`는 실좌표
- [ ] APU↔RPU bridge (ros2_ws 패키지 아님 — RAON-RT `CROS2PickBridge` 출력을 seqlock→rpmsg로 교체, 2026-09-04 확정)
- [ ] 최종 pickable-object detector for B3136 (진행 중 = YOLOv3-tiny)

---

## 8. 부록

### 8.1 새 세션 시작 절차 (권장)
1. 이 통합본 + `inst_claude.md`(또는 트랙별 정본)를 읽는다.
2. 사용자에게 **어느 트랙**인지 확인 (비전 / 로봇 제어).
3. 해당 트랙 정본 문서를 연다:
   - 비전 → `yolov3_tiny_execution_plan.md` 맨 아래 "D13 결과 / D14" + 필요시 `yolo_v3_process.md`.
   - RT 커널 → **`rt_kernel_postmortem.md`(크래시 전말)** → `rt_patch.md`(인수인계·복구). RPU → `rpu_guide_for_claude.md`.
4. 상태 변경(파이프라인 기동/정지, DPU 점유)은 사용자 명령 없이 Claude가 직접 하되, **물리 배치(물체 놓기·카메라 형상)와 연결 단절·비가역 작업은 사용자에게 확인**.
5. Phase/Gate 달성·중요 발견·결정 변경 시 ①정본 실행계획 갱신 ②지속 메모리 갱신. 사용자 확인 사항은 날짜와 함께 기록.

### 8.2 소통 규칙 (CLAUDE.md)
- **한국어 답변, 기술용어는 영어 유지**(calibration, depth scale, device tree 등).
- 학부생 수준·FPGA 경험 제한 가정 → 원리부터, 절차·검증·디버깅 순서 중심(추측 지양).
- 강한 주장은 high-priority 소스(AMD/Xilinx UG/PG → Kria 문서 → wiki → 공식 GitHub → 포럼 → 커뮤니티) 근거로만. 소스 불일치 시 양쪽 인용 + 공식 확인 권고.
- 답변 전 `site_md/reference_0X_*.md`의 토픽별 링크 우선 확인.

### 8.3 비전 모델 선택 우선순위 (CLAUDE.md)
1. **접근·추종 출력 효용**(class + 2D 위치 + depth) 우선 — 판정 기준 자체는 pick&place 시절과 동일하다.
2. **KV260 배포 가능성**(명확한 Vitis AI 경로).
3. **Latency**(capture→detect→3D→publish가 사이클에 맞는가).
4. **모델 정확도**(논문 성능은 후순위). 권장 순서: SSD/SSDLite-MobileNet v2 → tiny/pruned YOLOv3 → instance seg → pose. **최신 모델이 더 낫다고 가정하지 말 것.**

### 8.4 참고 링크
- RT 패치 아카이브: https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.15/older/
- 커널 소스: https://launchpad.net/ubuntu/+source/linux-xilinx-zynqmp
- IgH 문서: https://docs.etherlab.org/ethercat/1.6/pdf/ethercat_doc.pdf
- SOEM: https://github.com/OpenEtherCATsociety/SOEM (v1.4.x)
- UG1414 v2.5: `~/ug1414-vitis-ai.pdf` (Quantizer pp.86–107, Compiler pp.108–135, 지원 op Table 20)
- 랩 가이드: MAN-20241113-LX02H0001 (RAIMLAB, x86용 RT-PREEMPT+IgH — 절차 뼈대 참고)
