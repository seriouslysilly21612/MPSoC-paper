# inst_claude.md — 새 세션 온보딩 / 인수인계 문서

> **이 문서의 목적**: 새 Claude 세션이 이 파일 하나만 읽고 **마지막 지점에서 바로 이어서 작업**할 수 있게 한다. 여기서 각 주제의 정본 문서로 라우팅하므로, 추가 탐색 없이 시작 가능.
> 최종 갱신: 2026-09-04

---

## 0. 30초 요약 (지금 상태)

- **프로젝트**: Kria KV260 기반 **접근(approach) + 추종(tracking) 시스템** (RealSense → 비전 검출 → 3D 위치 → `base_link` 좌표 → Indy7 실시간 제어). 타깃 로봇 = Neuromeka Indy7.
- **앱 정의 (2026-08-03 전환 — 가장 먼저 알아야 할 것)**: 오퍼레이터가 테이블 위 물체를 고르면 **Indy7 TCP가 그 물체 꼭대기 +15 cm 호버 지점으로 접근**하고, **사람이 그 물체를 손으로 옮기면 TCP가 따라간다(추종)**. **파지·이송·놓기는 범위 밖**(3지 그리퍼 하드웨어 미보유 — 앱 코드에 파지 관련 키·상태·토픽이 아예 없다). "pick & place"는 **초기 프레이밍이고 지금은 아니다** — 런치/노드/토픽/패키지의 `pick_*` 이름은 전부 legacy이며 **바꾸지 않는다**.
- **트랙 3개**:
  1. **비전** — **완료**. YOLOv3-tiny **6-class** 배포(peach 드롭 D13, apple 재학습 D14). 라이브 **21.7 Hz**, capture→base E2E p50 ~118 ms, depth-color skew ≈0(이동 물체 대응, 2026-08-04/05). ← 이 문서의 §2 (경위), 현재 수치는 `docs/vision/throughput.md`
  2. **로봇 제어 (APU)** — **실운용 중**. RT 커널 종결 → APU IgH EtherCAT → RAON-RT가 Indy7 6축을 **1 kHz CST**로 제어. 접근(`v`)은 실기 검증 완료(one-shot 실측 4~6 mm), 추종(`o`) v3 하이브리드는 **구현 완료·실기 미검증**. 라이브 앱 = `~/RAON-RT-Revision`(브랜치 `kv260-merge`), 정본 = `docs/RAON-RT/merge.md`. ← 이 문서의 §3
  3. **RPU (R5F)** — **Gate 1 통과(작업 2026-09-01~02, 펌웨어 부팅 검증 2026-09-02 2회)**: FreeRTOS 단독 부팅(커널 로그 확인) + **LED 태스크 실행**(점멸 자체는 사용자 육안 확인 — 보드에서 재검증 불가). 남은 것은 정확히 3가지 — ① APU↔RPU 통신(rpmsg) ② RAON-RT 실시간 제어 task의 RPU 이식 ③ EtherCAT SOEM의 RPU 설치. ← 이 문서의 §3 Phase C
- **현재 물리 상태**: 비전 파이프라인·제어 앱 모두 기동 가능. RPU는 `remoteproc0` 등록됨(현재 `offline`, 부팅마다 firmware 이름 재설정 필요).
- **먼저 물어볼 것**: "어느 트랙을 이어서 할까요?" (비전 / 로봇 제어=RAON-RT 접근·추종 / RPU). **③ SOEM은 eth0(GEM3)를 RPU로 가져가야 하는데 지금 APU IgH가 그 포트로 Indy7을 제어 중** — 사용자 재결정 전 착수 금지.

---

## 1. 문서 지도 (어디에 무엇이 있나)

새 세션은 **작업할 트랙의 정본 문서부터** 읽어라. 이 지도로 충분하니 무작정 grep 하지 말 것.

| 주제 | 정본 문서 | 성격 |
|---|---|---|
| **전체 스냅샷 + 문서 라우팅** | `docs/STATUS.md` | **여기부터.** §2가 정본 라우팅 표 |
| 시스템 전체 구조·규칙 | `CLAUDE.md` | 노드/토픽/제약/소통규칙 (항상 우선) |
| **제어 트랙 정본(접근·추종·RAON-RT)** | `docs/RAON-RT/merge.md` | **로봇 앱 작업이면 여기부터.** §0 30초요약 + D결정대장 + E버그대장 |
| 파이프라인 상세(노드별 파라미터·근거) | `docs/vision/workflow.md` | 비전 변경 전 필독 |
| 비전 throughput 사다리 | `docs/vision/throughput.md` | 속도 요구가 오면 §4 사다리 순서대로 |
| 비전 트랙 결산(SSD→YOLO 전 과정) | `docs/vision/vision_final.md` | 보고용 정본 |
| 시간순 히스토리(결정·측정·root-cause) | `docs/history.md` | append-only, 과거 경위 추적용 |
| 왜 그렇게 결정했나(서사) | `docs/decision_journey.md` | 맥락 복구용 |
| **비전 YOLO — 명령어·게이트 레퍼런스** | `docs/vision/yolov3_tiny_execution_plan.md` | Phase 0~7, Gate 0~7 (모델 재학습 시) |
| **비전 YOLO — 논리 서사(왜 이 선택인가)** | `docs/vision/yolo_v3_process.md` | 맥락 복구용 |
| 3D 역투영 원리 | `docs/vision/reprojection.md` | 단일점 reverse projection |
| **RPU/EtherCAT — 새 세션 진입점** | `docs/rpu/rpu_guide_for_claude.md` | **RPU 작업이면 여기부터** (함정 지도 포함) |
| RPU 실행 계획(Gate 1~5) | `docs/rpu/rpu_freertos_soem_execution_plan.md` | 명령어·게이트 |
| RPU 논리 전개 | `docs/rpu/rpu_plan.md` | 왜 이 선택인가 |
| 실험환경 실측 전수조사 | `docs/analyses/experiment_environment_survey_20260903.md` | 보고서에 쓸 수치는 여기(문서 값이 아니라 실측값) |
| 지속 메모리 | `~/.claude/projects/-home-ubuntu/memory/MEMORY.md` | `raon-vs-merge-plan`(제어 트랙 정본), `kria-rt-preempt-project` 등 |

---

## 2. 비전 트랙 (YOLOv3-tiny → 6-class) — 이 세션의 작업

> **2026-09-04 주석**: 비전 트랙은 **종료**됐다(모델 6-class 확정, 파이프라인 라이브 **21.7 Hz**). 아래 §2.x는 그 **경위 기록**이며, 현재 수치·설정의 정본은 `docs/vision/throughput.md`·`docs/vision/workflow.md`다.

### 2.0 ★ 최신 상태 (2026-07-09) — 아래 2.1~2.4보다 이게 우선
- **7-class → 6-class**(peach 드롭, D13) 재학습·양자화·컴파일·보드 배치 완료. **Gate 5(실물 top-down) 통과**: mustard 0.02→0.814, tennis 0.677, orange 0.850, banana 0.777 — **5종 확정**. apple만 0.489~0.549(0.50 경계, color 갭).
- **apple 타깃 재학습(D14) → Gate 5 통과 🎉 (2026-07-10)**: 배포 카메라로 apple만 실촬영(`real_apple_yolo` train-only) + `hsv_v`↑ 재학습 → 실물 top-down **apple 0.5→0.88**(나머지 5종 0.81~0.88 전부 개선/유지). **모델 교체(Gate 2~5) 완성.** `apple:0.40` threshold는 0.50 default로 원복함. (재학습이 데스크톱 GPU 드라이버 hang으로 2회 중단됐던 건 §2.8 — `nvidia-driver-580` proprietary 롤백으로 해결.) **남은 것: Gate 6(풀 파이프라인)·Gate 7.**
- **정본·재개 순서**: `yolov3_tiny_execution_plan.md` 맨 아래 **"D13 결과 / D14"** 섹션. 재개 = 새 mAP(apple↑?) → `12` → `13` → scp → 보드 Gate5 → apple 강하면 `apple:0.40`→**0.50 원복** → Gate6/7.
- 신규 스크립트: 보드 `~/vitis_ai_work/scripts/autolabel_single_object.py`(단일물체 auto-label), `capture_color_frames.py --manual`(Enter=1장).
- ⚠️ 아래 **2.1~2.4는 7-class 시절 기록(참고용)** — class id·Gate5 실패표는 구버전. 현재 class = `{0 apple,1 orange,2 banana,3 tennis_ball,4 mustard_bottle,5 person}`.

### 2.1 무엇을 왜 했나 (핵심만; 논리 전체는 `yolo_v3_process.md`)
- 모델 = **YOLOv3-tiny 7-class** (`0 apple,1 peach,2 orange,3 banana,4 tennis_ball,5 mustard_bottle,6 person`). 이유: DPU 지원 op만으로 구성 → 단일 DPU subgraph 가능.
- 학습 도구 = **yolov5 v7.0 repo** (구조는 YOLOv3-tiny, PyTorch라 vai_q_pytorch 직결). **activation은 SiLU→Hardswish 교체**(D11, SiLU가 DPU 미지원).
- 데이터 = **synthetic(YCB mesh)+COCO+OpenImages+BOP ycbv**. **실환경 촬영 이미지는 학습에 미사용**(D2, 검증만 허용).
- 전처리 = **letterbox 416**(D7, train/calib/worker 통일).
- 배포 기하 = 회색 optical table, **top-down 0.8m 수직**(D10).

### 2.2 완료된 것 (재작업 불필요)
- **Gate 2** 학습 PASS: hswish run `pickplace_v3tiny_hswish`, val mAP@0.5 **all 0.766** (apple .625/peach .955/orange .734/banana .738/tennis .978/mustard .823/person .509).
- **Gate 3** 양자화 PASS: cosine head0 0.9923 / head1 0.9849, VAIQ_WARN 0건.
- **Gate 4** 컴파일 PASS: DPU subgraph 1개. `xdputil`로 fingerprint·fixpos 검증됨.
- **보드 배치 완료**: `~/vitis_ai_work/models/yolov3_tiny_7class.xmodel` + `decode_meta.json` (md5 `e2ca87c2466f715e9ecc00c43b599cc4`).
- **라이브 파이프라인 기동 성공**: dpu_ms ~17-23ms. apple/orange/banana 실물 검출 OK, 오검출 없음.

### 2.3 Gate 5 결과 — 진짜 top-down 재검증 완료 (2026-07-08): 도메인 갭 확정
실제 D10 top-down(회색 타공 optical table 수직 하방)에서 재검증한 최종 결과. **형상 탓이 아니라 진짜 sim-to-real 도메인 갭**:
| 물체 | top-down 실측 | 판정 |
|---|---|---|
| banana / orange | 0.61~0.67 / 0.88 | ✅ 정상 |
| **tennis_ball** | **0.491** (0.5 바로 아래) | ⚠️ threshold 0.4로 즉시 해결 (top-down이 도움됨) |
| **apple(실제 빨강)** | 0.26 | ❌ 약함 + peach에 밀림 |
| **peach** | apple로 0.66 / peach로 ~0 | ❌ 사과로 오분류 |
| **mustard** | **0.02** | ❌ 완전 실패 (누운 top-down + Morehouse 스퀴즈병 ≠ YCB006 경질병) |
- peach·mustard는 **YCB mesh ≠ 실물 외형** → 같은 mesh 재렌더로는 못 닫음. 그래서 **D2 재검토 결정(D12)**.

### 2.4 재개 계획 (2026-07-09~) — D12: 소량 real 학습데이터 추가 (사용자 결정 2026-07-08)
**방향 전환**: synthetic 재렌더가 아니라 **real 촬영 이미지를 학습(weight update)에도 사용**. 기존 synthetic+public 유지(banana/orange 등 OK), real은 실패 class(peach/mustard/apple) 보강.
1. **real 학습셋 수집**: 이 테이블에서 6종 위치/각도/자세 다양(특히 **mustard 눕힘+세움**, 프레임 가장자리) 수십~수백 장. 사용자 물체 배치 + `capture_color_frames.py`(카메라만 기동, RT 커널 2코어 부담 회피).
2. **라벨링**(D2가 피하던 수작업): 도구/워크플로 제안 필요 — 잘 되는 class는 현 모델 auto-label 후 교정, 실패 class는 수동. (labelImg/CVAT/Roboflow 후보.)
3. **병합 + fine-tune**: 데스크톱에서 기존 데이터셋에 real 추가 → hswish cfg 재학습(`11_train.sh`) → 재양자화(`12`) → 재컴파일(`13`) → 보드 재배치 → Gate 5 재검증.
4. **quick win(즉시)**: tennis_ball threshold 0.5→0.4 (image-test 스크립트 + worker CLASS_THRESHOLDS). 0.491이라 바로 잡힘.
- 상세: `yolov3_tiny_execution_plan.md` 맨 아래 "▶ 다음 작업" 섹션. 보존 자산: `test_images/topdown/`(실물 top-down 5장).

### 2.5 보드에서 재검증할 때 쓰는 명령 (사용자 명령 없이 Claude가 직접 실행)
```bash
# (1) 파이프라인 기동 (background)
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py > /tmp/pl.log 2>&1 &
# "RealSense Node Is Up" 뜰 때까지 대기 후:
# (2) 프레임 캡처 (카메라만 필요)
python3 ~/vitis_ai_work/scripts/capture_color_frames.py --count 3 --out ~/vitis_ai_work/test_images/runN
# (3) 파이프라인 정지 → DPU 해제 (isolated 테스트 전 필수: DPU는 1 프로세스만 점유)
pkill -f "pick_place_vitis_ai.launch"; pkill -f realsense2_camera_node; pkill -f vitis_ai_worker_yolo; sleep 3
# (4) 격리 테스트 (물체별 confidence/bbox + overlay)
python3 ~/vitis_ai_work/scripts/yolov3_tiny_image_test.py \
  --model ~/vitis_ai_work/models/yolov3_tiny_7class.xmodel \
  --meta  ~/vitis_ai_work/models/decode_meta.json \
  --image ~/vitis_ai_work/test_images/runN/obj_00.jpg --output /tmp/ov.jpg
# 저-threshold 분포 진단: /tmp/.../scratchpad/diag_lowthr.py (재작성 필요시 계획서 참고)
```
**주의**: DPU는 한 번에 한 프로세스만. 파이프라인(worker)이 떠 있으면 `yolov3_tiny_image_test.py`가 충돌한다. 캡처는 카메라 토픽만 쓰므로 파이프라인과 공존 가능하지만, isolated 추론 전엔 반드시 파이프라인 정지.

### 2.6 config 현재 상태 (중요)
`~/ros2_ws/src/system_bringup_pkg/config/` (install→build→src 전부 symlink, **수정 즉시 반영, rebuild 불필요**):
- `vitis_ai_detector.yaml`: model_path·worker_script_path 키는 **없다**(2026-07-15 — launch가 패키지 동봉 `yolov3_tiny_7class.xmodel`을 주입, 노드는 자기 옆의 `vitis_ai_worker_yolo.py` 사용), **send_resized_input: false**, **worker_shm: true**(2026-08-05 — 프레임 본문은 /dev/shm mmap, JSON 제어선만 pipe), **process_period_sec: 0.030**. metrics 키도 여기 없고 launch 플래그 `metrics:=true`가 주입한다(값은 double이어야 함 — 정수면 detector 死).
- `pick_logic.yaml`: allowed_classes→**5종**(apple/orange/banana/tennis_ball/mustard_bottle). person은 detector threshold 2.0으로 `/detections`에 오지 않고, pick_logic에 person 로직은 없다(2026-09 제거).
- ~~**SSD stand-in 복구**~~: **불가**. SSD 워커(`vitis_ai_worker.py`)와 복구용 주석이 2026-09 정리에서 삭제됐다. 되돌리려면 git 히스토리에서 파일을 꺼내야 한다.

### 2.7 학습·양자화·컴파일은 어디서 (데스크톱)
- **보드 아님**. 데스크톱 `jaehyeon@jaehyeon-Raimlab`(RTX 4060), 경로 `~/capstone_training/`.
- 스크립트 원본은 **보드 `~/ros2_ws/yolo_v3_tiny_training/`**(구 `training/`에서 개명)에 있고 데스크톱이 rsync로 당겨 씀:
  `rsync -av ubuntu@192.168.120.50:~/ros2_ws/yolo_v3_tiny_training/ ~/capstone_training/training/`
- 학습 = native venv(`~/capstone_training/venv_train`, torch cu121). 양자화/컴파일 = CPU docker `xilinx/vitis-ai-cpu:2.5.0.1260`.
- 게이트 순서·정확한 명령: `yolov3_tiny_execution_plan.md`. 스크립트 실행 순서: `training/README.md` (01~13, 12a는 학습 전 Gate0 inspect).

### 2.8 지뢰밭 (이미 밟아서 해결한 것 — 반복 금지)
- yolov5 기본 activation은 **SiLU(DPU 미지원)** → cfg에 `activation: nn.Hardswish()`. **학습 전 `12a_inspect_docker.sh`(Gate 0)로 DPU 매핑 선검증 필수.**
- VAI 2.5 `hardswish.py` 배포 버그 2건(FixNeuronWithBackward 미정의 / fake_quantize_per_tensor 인자 누락) → 12·12a wrapper가 docker 안에서 `sed`로 자동 패치.
- `cosine 수치보다 VAIQ_WARN/unknown-op 경고를 먼저 볼 것`.
- `metrics_duration_sec`는 double(`0.0`)이어야 함(정수면 detector 死).
- `sudo bash 12_*.sh` 금지(내부에서만 sudo). 스크립트는 sudo 없이 실행.
- **데스크톱 GPU 드라이버(2026-07-09)**: 자동 업데이트로 `nvidia-driver-595-open`(open 커널모듈+GSP)이 깔리면 CUDA 학습 중 **로그 없는 hard-freeze**(epoch 0에서 죽음, 온도 정상, OOM/Xid/MCE 전무). → **proprietary `nvidia-driver-580`로 롤백**(open은 GSP 강제라 회피 불가) + `apt-mark hold`로 자동 업데이트 차단. 확인=`cat /proc/driver/nvidia/version`에 'Open Kernel Module' 문구 없으면 정상. 재발 시 하드웨어(memtest/PSU/PCIe).

---

## 3. 로봇 제어 트랙 — RT 커널 패치 → EtherCAT → RPU (별도 관리)

로봇(Indy7) 제어 스택을 3단계로 쌓는다. **2026-09-04 현재: Phase A 종결 · Phase B 실운용 중 · Phase C는 Gate 1 통과 후 남은 3과제.** 아래 Phase 헤딩의 `★ 지금 활성` 표시는 **2026-07 당시 표기**이며, 각 Phase 첫 줄의 `상태:`가 현재 사실이다.

정본: 제어 앱(접근·추종) = `docs/RAON-RT/merge.md`, RPU 계획 = `docs/rpu/rpu_freertos_soem_execution_plan.md`, RPU 논리 = `docs/rpu/rpu_plan.md`, RT 커널 결산 = `docs/rt/`. **RPU 작업이면 반드시 `docs/rpu/rpu_guide_for_claude.md`부터 읽어라** (검증된 DT binding, UART0/CMA 함정, macb unbind 클럭 게이팅 등 함정 지도 포함).

### Phase A — RT-PREEMPT 커널 패치 (APU) ★ 지금 활성
- **상태: 종결(2026-07-15).** 프로덕션 커널 `5.15.199-rt91-rt-kv260c`(빌드 #11, DEBUG off, radix + zocl 픽스) 구동·검증 완료. 아래는 당시 계획 기록.
- **목표**: KV260 APU(A53) 커널을 RT-PREEMPT로 교체 (실시간 제어 지연 보장).
- **교체 전 커널**: `linux-xilinx-zynqmp` 5.15.0-1070.74 (업스트림 베이스 5.15.199), **PREEMPT_VOLUNTARY / HZ=250**.
- **방법**: x86 PC에서 **aarch64 크로스컴파일**. 소스 = Ubuntu Xilinx 커널(linux-xilinx-zynqmp) + kernel.org RT 패치(5.15.199에 가장 가까운 `patch-5.15.x-rt`).
- **Kria 특수사항**: 부팅은 **U-Boot + flash-kernel(`/boot/firmware/image.fit`), GRUB 없음**. **CMA 유지**(FPGA/PL 사용 중 — 랩 가이드의 CMA 비활성화는 적용 안 함). cmdline에 `cma=1000M`, `clk_ignore_unused`.
- **참고 가이드**: 랩(RAIMLAB, 명지대) MAN-20241113-LX02H0001 (x86 NUC용 RT-PREEMPT + IgH EtherCAT). 큰 흐름만 따르고 Kria 차이(GRUB→flash-kernel, CMA 유지) 반영.

### Phase B — IgH EtherCAT Master (APU)
- **상태: 실운용 중.** APU IgH가 Indy7 6축을 **1 kHz**로 실제 제어하고 있다. 라이브 앱 = `~/RAON-RT-Revision`(브랜치 `kv260-merge`, 디렉터리 `App/Indy7`, 기동 `run.sh`). ⚠️ `~/RAON-RT`·`~/RAON-RT-Revision-main`은 원본/학습용이라 라이브가 아니다.
- APU에서 **IgH EtherCAT Master git stable-1.6(태그 1.6.10)** 구동(1.5.2는 불가). NIC 드라이버는 igc가 아니라 **Cadence GEM(macb)** → generic 드라이버(`--enable-generic`). 설치 = `/opt/etherlab` + CLI.
- **GEM3=ff0e0000 (eth0)**. Indy7는 **STEP 우회·드라이브 직결**, 드라이브 모드는 **CST(`DRIVE_MODE=10`)** 확정(CiA402+CSP가 아님 — 낡은 문서 주의). 제어주기 **1 kHz 확정**. 테스트 드라이브 **LS Mecapion L7N**.
- **앱 조작(오퍼레이터 키)**: `r` 컨트롤러 enable → `t` CST → `h` 서보 ON → **`p`(물체 메뉴) → 숫자(선택) → `v`(접근)** → **`o`(추종 토글)**. 정지는 `x`(소프트, 중력보상 유지) / `e`(E-STOP, 서보 OFF), `b` HOME 복귀. **세션 첫 행동 = 자연 자세에서 HOME 기록**(ready-seed 앵커). 상세·안전장치는 `docs/RAON-RT/merge.md`.

### Phase C — RPU FreeRTOS + SOEM (이후)
- **상태: Gate 1 통과(작업 2026-09-01~02, 펌웨어 부팅 검증은 2026-09-02 11:02·15:01 2회 — 09-01은 DT enumeration까지).** R5F-0에 **FreeRTOS 단독 부팅 성공**(커널 로그) **+ LED 태스크 실행**(판정 수단은 UART/trace가 아니라 **LED**였다 — 현 펌웨어에 resource table이 없어 trace buffer가 안 생긴다. 점멸 자체는 **사용자 육안 확인**이며 `/sys/class/leds/`에 led35/36이 없어 보드에서 재검증 불가). 커널 로그 `remote processor ff9a0000.rf5ss:r5f_0 is now up`, 펌웨어 `/lib/firmware/r5_freertos.elf`.
- **적용 방식은 configfs가 아니라 부팅 DT 병합**: base DTB + overlay를 `fdtoverlay`로 오프라인 병합 → `/boot/firmware/user-override.dtb`(U-Boot 부트스크립트가 로드) → 재부팅. reserved-memory는 런타임 configfs로는 제대로 잡을 수 없기 때문(memblock 단계에서만 예약 가능). 실제 산출물은 git 밖 `~/rpu_work/`에 있다.
- **메모리 배치(실측)**: carve-out `rproc@36000000` + `0x400000`(4 MiB, no-map), ELF `.text @ 0x36000000`(실사용 ~78 KB) + TCM. ⚠️ 문서 곳곳의 관례값 **0x3ed00000은 쓰지 말 것** — 이 보드의 CMA 창(0x36400000 + 1000 MiB) 한복판이다.
- **빌드 환경**: Vitis **2022.1**(Windows `d:\xilinx\vitis\2022.1`, **설치 완료**), XSA `design_1_wrapper`, 도메인 `psu_cortexr5_0`, BSP `freertos10_xilinx`.
- **남은 것은 정확히 3가지**:
  1. **APU↔RPU 통신(rpmsg)** — 지금 DT에 IPI mailbox도 vring/buffer reserved-memory도 없고 펌웨어에 resource table도 없다 → **DT 재작업(재부팅 1회) + 펌웨어 재빌드**가 같이 필요. 4 MiB carve-out 중 실사용이 ~78 KB뿐이라 그 창 안에서 해결 가능. **GEM3 무관 → 지금 진행 가능.**
  2. **RAON-RT 실시간 제어 task의 RPU 이식** — EtherCAT 없이 1 kHz 주기 연산 + 지터 측정까지가 1차 범위. **GEM3 무관 → 지금 진행 가능.** (미확인: RBDL·double 연산이 R5F 단일 코어/TCM에서 어디까지 성립하는지 → 이식 범위를 먼저 판정해야 함.)
  3. **EtherCAT SOEM의 RPU 설치** — SOEM osal/oshw를 emacps 기반으로 포팅. 소스 편입·스텁 빌드까지는 GEM3 없이 가능하지만, **PHY 링크업 이후는 GEM3를 RPU로 이관해야 하므로 사용자 재결정 전 착수 금지**(APU IgH가 그 포트로 Indy7 제어 중).
- 그리퍼 없음 → end-to-end 목표는 **말단부가 물체 위에 정지 + 물체가 움직이면 추종**까지.

### 트랙 간·단계 간 충돌 주의
- **원격 접속은 eth0가 아니다.** AX88179B **USB NIC**(`192.168.120.50`)로 영구 이전됨(2026-07-20, Tailscale 제거). **eth0(GEM3)는 EtherCAT 전용 예약** — 그래서 EtherCAT 착수로 원격이 끊기지 않는다.
- **GEM3 공유 충돌은 RPU 잔여과제 ③(SOEM)에만 해당.** ①rpmsg·②제어 task 이식은 IPI·공유메모리·연산만 쓰므로 APU IgH 운용과 공존한다. ③은 GEM3를 RPU로 넘겨야 하므로 사용자 재결정 필요.
- **DT 변경은 재부팅을 수반한다** → 실기(로봇) 세션과 분리하고, 착수 전 `systemctl is-active ethercat` 확인. 교체 전 `/boot/firmware/user-override.dtb`를 백업할 것(부팅 실패 시 SD에서 그 파일만 지우면 순정 DTB로 복귀).
- **CPU3 침범 금지**: `isolcpus=3 nohz_full=3 rcu_nocbs=3 irqaffinity=0-2` — 1 kHz 제어 페어가 CPU3에 pin되어 있다. RPU 작업의 APU측 부하는 CPU0-2에.
- **실기 세션 중 보드에 무거운 작업 금지**(빌드·대량 IO·IDE 인덱싱 포함). E34(IgH 스레드 priority inversion)·측정 오염 전례 있음.
- **비전 트랙과는 리소스 충돌 없음** (DPU/카메라 vs 커널/GEM은 별개). 단 같은 보드라 재부팅·커널 교체 시 비전 파이프라인도 함께 내려감.

---

## 4. 보드·환경 필수 사실

- **보드(이 머신)**: Kria KV260, `ubuntu@192.168.120.50`(AX88179B **USB NIC**. eth0는 EtherCAT 전용 예약), Ubuntu 22.04.5 + ROS2 Humble. 작업 루트 `~/ros2_ws/`(perception), 제어 앱은 `~/RAON-RT-Revision`(브랜치 `kv260-merge`).
- **DPU**: DPUCZDX8G_ISA1_B3136, fingerprint `0x101000016010406`, kv260-smartcam overlay(systemd 자동로드), VART/VAI 2.5.0. arch: `~/vitis_ai_work/arch/arch_b3136.json`.
- **카메라**: RealSense D435i(FW **5.17.0.10**), color+depth 848×480×30, `enable_sync` **ON**(2026-08-04, 이동 물체 대응), `align_depth` OFF. color 토픽 `/camera/camera/color/image_raw`.
- **파이프라인 실행**: `ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py` (카메라+detector+pick_logic+3D+TF. **런치·노드·토픽 이름의 `pick_*`은 pick&place 시절 legacy** — 실제 동작은 접근+추종 타깃 공급). 3D는 단일점 reverse projection(`camera_depth_optical_frame`) + color capture stamp에 가장 가까운 depth 프레임 선택.
- **base TF는 더 이상 placeholder가 아니다**: 2026-07-27 eye-to-hand calibration 실측치(16 pose, AX=ZB, residual 1.8 cm / 1.7°) + E21 base 재베이스(yaw +5.14°)가 launch의 `base_to_camera_tf`에 박혀 있다. 최종 출력 `/pick_target_base`는 **진짜 로봇 좌표**이며, 로봇 앱(`ROS2PickBridge`)이 이 토픽과 `/detections`를 구독한다.
- **detector↔worker 경계 = model-agnostic JSON contract**. 모델 교체는 worker 전처리/decode만 건드림. `vitis_ai_detector_pkg`는 editable-install(src가 live), config YAML은 symlink(rebuild 불필요), `target_3d_pkg`만 편집 후 `colcon build --packages-select target_3d_pkg --symlink-install`.
- **데스크톱**: `jaehyeon@jaehyeon-Raimlab`, RTX 4060 8GB, Ubuntu 22.04.5. 학습/양자화/컴파일 담당. 보드로 xmodel 전송은 `scp ... ubuntu@192.168.120.50:~/vitis_ai_work/models/`.

---

## 5. 새 세션 시작 절차 (권장)

1. 이 문서(`inst_claude.md`)를 읽는다.
2. 사용자에게 **어느 트랙**인지 확인 (로봇 제어=접근·추종 / RPU / 비전).
3. 해당 트랙 정본 문서를 연다:
   - 로봇 제어 → `docs/RAON-RT/merge.md` (§0 30초요약 → D/E 대장).
   - RPU → `docs/rpu/rpu_guide_for_claude.md`.
   - 비전 → `docs/vision/workflow.md`(현재 설정) / `docs/vision/throughput.md`(속도 요구 시 §4 사다리 순서대로) / 모델 재학습이면 `docs/vision/yolov3_tiny_execution_plan.md`.
4. **실기(로봇) 세션이면**: 보드에 무거운 작업 금지(빌드·대량 IO·IDE 인덱싱), 재부팅을 수반하는 작업(DT 변경 등)은 분리. 세션 첫 행동은 자연 자세에서 **HOME 기록**(ready-seed 앵커). 상태 변경(파이프라인 기동/정지, DPU 점유)은 사용자 명령 없이 Claude가 직접 하되, 물리 배치(물체 놓기·손으로 물체 옮기기)는 사용자에게 요청.
5. 소통: **한국어 답변, 기술용어는 영어 유지.** 사용자는 학부생 수준·절차 지향 → 명령어 중심 단계별 안내. 파괴적/외부영향 작업만 사전 확인, 가역적 작업은 진행.

---

## 6. 미해결 TODO 스냅샷

**비전(종료)**: [D13 6-class Gate5 통과 ✔] → [D14 apple 실물 재학습 ✔] → [보드 Gate5: 실물 apple 0.5→0.88, 6종 확정 ✔] → [apple threshold 0.50 원복 ✔] → [Gate 6 실측 ✔ 15Hz·E2E137ms·CPU79%/4코어] → [CPU 절감 phase1+2 ✔ 76.8→~44%] → [이동 물체 전환 ✔ sync30+nearest-stamp로 skew 28.5→4.3 ms, 게이트 0.030+shm IPC로 15.3→**21.7 Hz**]. 남은 사다리(미실행)는 `docs/vision/throughput.md` §4 2단계 = 워커 내 파이프라이닝.
**로봇 제어(APU)**: [Phase A: RT-PREEMPT 커널 ✔ 종결] → [Phase B: APU IgH EtherCAT + RAON-RT ✔ **실운용 중**] → 현재 잔여: **추종(`o`) v3 하이브리드 실기 검증**(검증 사다리 8단계 미실시, 속도 캡 증명 전 물체를 손으로 들지 말 것) / **오퍼레이터 콘솔(`raon_operator_gui`, `/operator_key` 등) 실기 미검증** / IK를 RT 루프 밖으로(최대 58 ms). 정본=`docs/RAON-RT/merge.md`.
**RPU(Phase C)**: [Phase 0 개발환경 ✔] → [**Gate 1 remoteproc+FreeRTOS+LED ✔ 부팅 검증 2026-09-02**] → ① APU↔RPU rpmsg(DT 재작업+펌웨어 재빌드 필요) / ② RAON-RT 제어 task 이식(EtherCAT 없는 1 kHz dry-run + 지터) / ③ SOEM 설치(**GEM3 필요 → 사용자 재결정 전 금지**). 상세=`docs/rpu/rpu_freertos_soem_execution_plan.md`, 진입=`docs/rpu/rpu_guide_for_claude.md`.
**공통 시스템**: base_link↔camera_link TF calibration ✔ 완료(2026-07-27 실측), **APU↔RPU bridge 미구현**(= RPU 잔여과제 ①), 그리퍼 하드웨어 미보유 → 파지·이송·놓기는 범위 밖.
