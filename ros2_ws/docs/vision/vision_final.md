# Vision Model 최종 정리 — SSD stand-in → YOLOv3-tiny 6-class (KV260 Pick & Place)

> 작성: 2026-07-15
> 범위: 비전 모델을 SSD stand-in에서 **YOLOv3-tiny로 교체**하는 시작점(학습)부터 **DPU 탑재**, **파이프라인 최적화**까지.
> 목적: 지금까지의 전 과정 · 주요 문제의 원인/해결 · 확정 스펙을 한 문서로 학습·보고.
> 정본 출처: `yolov3_tiny_execution_plan.md`(게이트·명령), `yolo_v3_process.md`(의사결정 논리), `workflow.md`(파이프라인 구조), `integrated_progress.md §4`(최적화 수치), 소스 코드(`vitis_ai_worker_yolo.py`, `pick_target_3d_node.py`, `pick_target_base_node.py`), 메모리(`yolov3-vision-swap-resume`, `perception-cpu-opt-phase1`, `d435i-fw-rgb-wedge-fix`).

---

## 0. 한눈에 보기 (Executive Summary)

Kria KV260 시스템(테이블 위 물체 **접근 + 추종**. 파지·이송·놓기는 범위 밖 — 그리퍼 하드웨어 미보유. 제목의 "Pick & Place"는 초기 프레이밍의 legacy)의 검출 모델을, DPU/파이프라인 검증용으로 쓰던 **SSD ADAS stand-in**(car/bicycle/person)에서 실제로 접근·추종할 물체를 검출하는 **YOLOv3-tiny 6-class**(apple/orange/banana/tennis_ball/mustard_bottle/person)로 교체 완료했다.

전 과정을 **게이트(Gate 0~7)**로 나눠 각 단계 통과 조건을 두고 진행했다:

| Gate | 단계 | 결과 |
|---|---|---|
| 0 | Inspector — DPU op 매핑 사전검사 | ✅ CPU subgraph 0 (Hardswish 패치 후) |
| 1 | 데이터셋 조립 | ✅ train 15,799 / val 2,093 (7-class 시절) |
| 2 | Float 학습 (mAP) | ✅ all mAP@0.5 = **0.766**(hswish 7-class) → 배포 D14 0.728 |
| 3 | 양자화 (INT8 PTQ) | ✅ cosine 0.976/0.962 (> 0.95 기준) |
| 4 | 컴파일 (xmodel) | ✅ **DPU subgraph 1개** |
| 5 | 보드 실물 검증 | ✅ **6종 전부 0.81~0.88** (도메인 갭 3라운드 해소) |
| 6 | 풀 파이프라인 | ✅ 15Hz 동기, E2E 137ms (2026-07-10 측정. **현재 구성은 21.7 Hz · true E2E p50 ~118 ms** — §9, `throughput.md`) |
| 7 | Live + 성능 CSV | 🟡 성능 CSV 확보 / live 체크리스트 일부 잔여 |

**가장 중요한 3대 트러블슈팅**:
1. **SiLU → Hardswish** activation 교체 (DPU 미지원 op → 재학습). 이 작업 전체에서 가장 교훈적.
2. **sim-to-real 도메인 갭** — synthetic만으로 학습한 peach/mustard/apple이 실물에서 실패 → 데이터 전략 3회 수정(D12/D13/D14)으로 해소.
3. **파이프라인 CPU 절감** — YOLO 교체 후 76.8% → ~44%(4코어 평균, **-1.25코어**)로 EtherCAT 통합 헤드룸 확보.

---

## 1. 시스템 · 하드웨어 스펙 (확정본)

### 1.1 플랫폼

| 항목 | 스펙 |
|---|---|
| 보드 | Kria **KV260** (Zynq UltraScale+ MPSoC) |
| APU (PS) | ARM Cortex-A53 4코어, **Ubuntu 22.04.5 LTS**, **ROS2 Humble** |
| 가속기(PL) | DPU via **`kv260-smartcam`** 오버레이 (부팅 시 `kv260-smartcam.service` 자동 로드) |
| 카메라 | Intel **RealSense D435i** (USB 직결, APU로) |

### 1.2 DPU (검출 연산 가속기) — `xdputil_query` 실측

| 항목 | 값 |
|---|---|
| DPU Arch | **DPUCZDX8G_ISA1_B3136** |
| Fingerprint | `0x101000016010406` |
| DPU Frequency | **300 MHz** |
| DPU IP version | v4.0.0 (generation 2022-05-11, git `9bf4ccf`) |
| VART runtime | **2.5.0** (libvart-runner 2022-07-20, libxir, target-factory 2.5.0) |
| 유효 CU | cu_idx 0 = `DPUCZDX8G:DPUCZDX8G_1` (cu_idx 1 = `pp_pipeline_accel`, DPU 아님) |

> B3136 = 사이클당 peak 3136 연산(≈8×16×… PE array) 규모의 DPU 구성. 이 fingerprint가 **컴파일 타깃과 보드 런타임에서 정확히 일치해야** xmodel이 로드된다(→ 툴체인을 VAI 2.5로 고정한 근거, D5). 자세한 매커니즘은 §3.

### 1.3 카메라

| 항목 | 값 |
|---|---|
| 드라이버 | realsense2_camera **v4.57.7** / librealsense **2.57.7** |
| Color / Depth | 둘 다 **848×480×30** (RGB8 / 16UC1 raw) — yaml에 `color_format` 키가 없어 `rs_launch.py` 기본값 RGB8이 적용된다. worker는 런타임 `msg.encoding`으로 분기하므로 하드코딩 금지 |
| Depth scale | 0.001 m/unit (16UC1 1단위 = 1 mm) |
| align_depth | **OFF** (단일점 reverse projection 사용, §8.5) |
| 펌웨어 | 5.16.0.1 → **5.17.0.10** (웨지 버그 수정, §7.5) |

### 1.4 개발 툴체인 (모델 교체용)

| 항목 | 값 |
|---|---|
| 양자화/컴파일 | **Vitis-AI 2.5** docker (`xilinx/vitis-ai-cpu:2.5.0.1260`, `conda activate vitis-ai-pytorch`) |
| 양자화 프레임워크 | **PyTorch** + `vai_q_pytorch` (지원 torch 1.2~1.10.2) |
| 학습 프레임워크 | **ultralytics yolov5 v7.0** repo (구조 정의만 차용, §4) |
| 학습 하드웨어 | Host PC: Ubuntu 22.04.5 x86_64, **NVIDIA RTX 4060 8GB**, RAM 16GB |
| 근거 문서 | **UG1414 v2.5** (Vitis AI User Guide, 보드 runtime과 동일 판) |

---

## 2. 왜 교체하나 · 왜 YOLOv3-tiny인가

### 2.1 출발점

시스템은 이미 동작 중이었다: RealSense D435i → DPU 검출 → 2D 필터 → 단일점 3D(역투영) → base frame 목표점까지 ROS2로 흐른다(~17 Hz). 단, 검출 모델이 **SSD ADAS stand-in**(`ssd_adas_pruned_0_95`, 480×360, car/bicycle/person)이라 실제로 집을 물체를 못 잡는다. 이 stand-in을 **우리가 집을 물체 검출기로 교체**하는 것이 목표.

### 2.2 모델 선택 — 3대 제약이 지배

프로젝트 원칙: **"논문 정확도보다 KV260 배포 가능성이 우선"**.

1. 접근·추종엔 **class + 2D bbox**면 충분 → segmentation/pose는 과잉. (당시 라벨은 "pick&place"였지만 이 판정 기준은 앱 전환 뒤에도 그대로 유효.)
2. KV260 DPU에 **실제로 올라가야** 함 → Vitis-AI 지원 op만 사용.
3. 파이프라인 나머지(3D·TF·downstream)는 안 건드림 → model-agnostic JSON contract 덕에 교체 범위 좁음.

**YOLOv3-tiny를 고른 결정적 이유**: 구조가 `conv + BatchNorm + LeakyReLU + maxpool + nearest upsample + concat`뿐인데, 이 op들이 **UG1414 v2.5 Table 20(DPUCZDX8G 열)에서 전부 지원**된다. 즉 CPU fallback 없이 **네트워크 전체가 DPU에 통째로 올라간다** → 나중에 "단일 DPU subgraph"(Gate 4)를 가능하게 한 근거. ("op이 지원돼야 한다"의 하드웨어적 의미는 §3.4에서 설명.)

### 2.3 모델 최종 스펙 — `decode_meta.json` 실측

| 항목 | 값 |
|---|---|
| 구조 | **YOLOv3-tiny** (detection head 2개 = tiny 특성) |
| 클래스 | **6-class**: `{0 apple, 1 orange, 2 banana, 3 tennis_ball, 4 mustard_bottle, 5 person}` |
| 입력 | **letterbox 416×416** (pad value 114) |
| Stride (2 scale) | 16, 32 → 출력 그리드 **26×26** / **13×13** |
| 출력 채널 | **33 = 3 × (6 + 5)** (anchor 3개 × (nc + x,y,w,h,obj)) |
| Anchors (px) | stride16: [10,14][23,27][37,58] · stride32: [81,82][135,169][344,319] |
| Activation | **Hardswish** (원래 SiLU → 교체, §6) |
| 파라미터 | **약 8.68M** (xmodel 상수 텐서 직접 합산, 2026-09-03 재측정. 옛 "6.9M"은 추정치였다) |
| 배포 파일 | `~/vitis_ai_work/models/yolov3_tiny_7class.xmodel` (md5 `9bc6520c`, 파일명은 7-class 시절 유지, 실제 6-class D14) |

> **왜 6-class인가**: 원래 7-class(peach 포함)로 시작했으나, peach가 실물에서 apple과 근본적으로 혼동되어 **D13에서 드롭**(§7.4). 출력 채널이 36(7-class)→33(6-class)으로 바뀐 이유.

---

## 3. Python 모델이 DPU에 올라가기까지 — Vitis-AI 플로우 (FPGA 엔지니어용)

> 이 장은 **FPGA는 알지만 Vitis-AI는 처음인 독자**를 위해, PyTorch로 학습한 float 모델이 어떻게 KV260의 DPU에서 실행되는지 그 원리와 흐름을 설명한다.

### 3.1 핵심 개념 — DPU는 "재합성하는 회로"가 아니라 "고정 오버레이 프로세서"다

FPGA 경험자가 "신경망을 FPGA에 올린다"고 하면 흔히 **HLS4ML/FINN처럼 네트워크를 회로(LUT/DSP/BRAM)로 합성**하는 그림을 떠올린다. **DPUCZDX8G는 그 방식이 아니다.**

- **DPU는 이미 PL에 합성돼 올라가 있는 고정 IP(soft processor)다.** `kv260-smartcam` 오버레이(bitstream)의 일부로, 보드 부팅 시 `kv260-smartcam.service`가 로드한다. 우리가 모델을 바꿔도 **bitstream/PL은 전혀 건드리지 않는다.**
- DPU는 자체 **명령어 집합(ISA)**을 가진, conv/pooling/element-wise에 특화된 프로세서다. 개념적으로 **CPU/GPU에 가깝다** — 다만 범용이 아니라 CNN 연산 전용.
- 따라서 신경망은 **회로로 합성되는 게 아니라, DPU가 실행할 "프로그램(명령어 + 가중치)"으로 컴파일된다.** 이 프로그램이 `.xmodel` 파일이다.

**비유**: `.xmodel` ≈ DPU ISA용 실행 파일(머신코드+데이터). PyTorch 모델 → xmodel 변환 ≈ **C 소스 → 특정 CPU용 바이너리 컴파일**. HDL 합성이 아니다. 그래서 모델 교체가 "재합성(수 시간, timing closure 위험)"이 아니라 "재컴파일(수 분)"로 끝난다.

### 3.2 "프로그래밍"은 두 층에서 일어난다 (FPGA 관점 정리)

| 층 | 무엇을 | 언제 | 우리가 하나? |
|---|---|---|---|
| **PL bitstream** | DPU IP를 PL에 배치·라우팅 (DSP array, AXI, 스케줄러 등) | 보드 이미지 빌드 시 (Xilinx 제공 smartcam) | ❌ 손 안 댐 |
| **DPU 프로그램(xmodel)** | 이 신경망을 DPU 명령어 스트림으로 | 모델 교체 때마다 (우리 작업) | ✅ 이 문서의 대상 |

즉 FPGA 엔지니어에게 익숙한 "bitstream = 회로 구성"은 **이미 끝나 있고 고정**이다. 우리가 반복하는 건 그 위에서 도는 **소프트웨어(명령어) 컴파일**뿐이다.

### 3.3 왜 INT8 양자화가 필수인가 (fixed-point 이야기)

DPU의 MAC array는 **INT8 고정소수점 연산기**다. float32 가중치·활성값을 그대로 못 먹는다. FPGA에서 float→fixed-point 변환을 하는 것과 정확히 같은 문제다.

- 각 텐서는 **per-tensor 스케일** `2^fix_point`로 양자화된다. 예: 입력 텐서 `fix=6`이면 `int8 = clip(round(float × 2^6), -128, 127)`, 출력 텐서 `fix=2`이면 dequant 시 `float = int8 / 2^2`.
- 이 `fix_point`(=Q포맷의 소수부 비트수)는 **컴파일된 xmodel의 텐서 속성**으로 박혀 있고, 보드 worker가 그 값을 읽어 전/후처리 스케일에 쓴다(§8.4의 LUT·decode가 `2^fix`를 그대로 사용).
- 스케일을 정하려면 **활성값의 실제 분포**를 알아야 한다 → **calibration**: 대표 이미지 수백 장을 float 모델에 통과시켜 각 층 활성값의 범위를 관측하고 스케일을 확정한다(PTQ, Post-Training Quantization). 가중치 재학습 없음.

### 3.4 왜 "지원 op"이 제약인가 — §6 SiLU 문제의 하드웨어적 이유

DPU는 **고정 하드웨어**라 실행할 수 있는 연산 종류가 정해져 있다(UG1414 Table 20): conv2d(+BN fold), max/avg pool, element-wise add/concat, nearest-neighbor upsample, 그리고 activation은 **ReLU / LeakyReLU / ReLU6 / Hardswish / Hardsigmoid** 정도.

- 모델에 **DPU가 모르는 op**(예: `SiLU = x·sigmoid(x)`)이 있으면, 그 부분은 DPU에서 못 돌고 **PS(A53 CPU)로 넘겨야** 한다.
- 그러면 그래프가 `[DPU 구간] → [CPU op] → [DPU 구간]`으로 **쪼개지고**(subgraph 파편화), 매 경계마다 DPU↔DDR↔CPU 왕복이 생겨 느려진다.
- 그래서 목표는 **네트워크 전체가 하나의 DPU subgraph**가 되는 것(op이 전부 지원돼야 함). 이것이 §2.2에서 YOLOv3-tiny를 고른 이유이자, §6에서 SiLU를 Hardswish로 바꾼 이유다.

### 3.5 런타임 데이터 경로 (추론 1회)

```
[A53/PS]  color 프레임 → letterbox 416 + INT8 양자화(LUT) → 입력 텐서를 DDR(CMA 연속 버퍼)에 기록
   │  VART: runner.execute_async([input]) → DPU에 명령/디스크립터 포인터 전달
   ▼
[DPU/PL] AXI master로 DDR에서 [명령어 + 가중치 + 입력 활성값] 읽음
         → conv/pool/concat 실행 (INT8 MAC array) → 출력 텐서를 DDR에 기록
   │  runner.wait() 로 완료 대기
   ▼
[A53/PS]  DDR의 INT8 출력 텐서 dequant(/2^fix) → YOLO grid decode + NMS → detection
```

- **VART(Vitis AI Runtime)** = 이 오케스트레이션을 하는 유저스페이스 라이브러리. 우리는 Python `vart.Runner` API로 호출한다.
- 가중치·명령어·활성값은 전부 **DDR을 통해** 오간다(DPU는 온칩에 전부 못 담음). 그래서 **연속 물리 메모리(CMA)** 가 필요하고, cmdline에 `cma=1000M`이 잡혀 있다.
- DPU는 한 번에 한 추론(batch=1)만 — 그래서 xmodel export도 batch=1 필수.

### 3.6 툴 3단 정리 (우리가 실제로 돌리는 순서)

| 단계 | 툴 | 입력 → 출력 | 하는 일 |
|---|---|---|---|
| ① **Inspect** | `Inspector` (vai_q_pytorch) | float `nn.Module` → 리포트 | 각 op이 DPU ISA에 매핑되는지 **사전검사**. CPU subgraph 0이어야 통과(Gate 0). **학습 전에** 돌려 SiLU 같은 지뢰를 조기 발견(§6.5) |
| ② **Quantize** | `torch_quantizer` (vai_q_pytorch) | float 모델 + calib 이미지 → INT8 XIR + `quant_info` | calibration으로 텐서별 스케일 확정, INT8 양자화. `--deploy`로 `*_int.xmodel` export (§7.1) |
| ③ **Compile** | `vai_c_xir` | INT8 XIR + `arch_b3136.json` → `.xmodel` | op fusion·명령어 스케줄링·DDR 배치 → **DPU 명령어 스트림** 생성. 단일 DPU subgraph 확인(Gate 4, §7.2) |

- **arch / fingerprint 매칭**: 컴파일 시 `arch_b3136.json`(fingerprint `0x101000016010406`)을 타깃으로 준다. 이 fingerprint가 보드 DPU와 다르면 명령어가 호환되지 않아 로드가 거부된다 — **CPU ISA 버전(예: armv8)이 맞아야 바이너리가 도는 것과 같은 개념.** 그래서 **툴체인 버전(VAI 2.5)을 보드 런타임(2.5.0)에 고정**한다(D5).
- **왜 tiny 모델도 CPU docker로 충분한가**: PTQ는 순전파 몇백 장이면 되고 재학습(gradient)이 없어 GPU 불필요. 학습만 GPU(RTX 4060), 양자화·컴파일은 데스크톱 CPU docker에서 수 분.

> **한 줄 요약(FPGA 관점)**: bitstream(회로)은 고정된 DPU 오버레이로 이미 끝나 있고, 신경망은 그 프로세서용 **INT8 명령어 프로그램(xmodel)** 으로 컴파일되어 VART가 런타임에 DDR을 통해 먹여준다. "합성"이 아니라 "크로스컴파일"이다.

---

## 4. 학습 프레임워크 선택 — "구조는 YOLOv3-tiny, 도구는 yolov5"

혼동하기 쉬운 지점: **구조(YOLOv3-tiny)와 학습 도구(yolov5)는 별개**다. 양자화 도구 `vai_q_pytorch`는 **PyTorch `nn.Module`만** 입력으로 받으므로(D6), "PyTorch로 YOLOv3-tiny를 학습하는 관리된 방법"이 필요했다.

| 후보 | 결과 | 이유 |
|---|---|---|
| darknet (원조 YOLOv3, C) | 탈락 | 출력이 `.weights`(darknet 전용) → PyTorch 변환기가 숨은 지뢰. 최신 augmentation 부재 |
| **yolov5 v7.0 repo** | **채택** | YOLOv3-tiny 구조 yaml 제공 + PyTorch 직행 + COCO pretrained + augmentation 완비 |
| ultralytics v8 | 탈락 | yolov3-tiny**u**는 anchor-free head → DPU decode 계획과 불일치 |

**v7.0 태그로 고정**: 고전 anchor 기반 Detect head를 쓰는 마지막 안정 릴리스라, 보드 decode 공식과 정확히 맞는다.

**부수 효과(알고 선택)**: yolov5로 학습하면 bbox decode가 yolov5식이다. 보드 worker와 `decode_meta.json`을 처음부터 이 공식으로 작성·단위테스트했다:
```
xy   = (sigmoid(t)*2 - 0.5 + grid) * stride
wh   = (sigmoid(t)*2)^2 * anchor_pixel
conf = sigmoid(obj) * sigmoid(cls)
```

---

## 5. 데이터 전략과 학습 (Gate 1, 2)

### 5.1 데이터 전략 (D2/D3) — 촬영 없이 합성 + 공개데이터

**D2(사용자 결정)**: 실환경(D435i) 촬영 이미지를 **학습(weight update)에 쓰지 않는다**(검증엔 허용). 수작업 촬영·라벨링 회피가 목적. → 이 결정은 나중에 도메인 갭 때문에 D12/D13/D14에서 **단계적으로 완화**된다.

| class | 주력 소스 | 이유 |
|---|---|---|
| apple/orange/banana | **COCO** (real 사진 다수) | 흔한 물체라 공개 real 풍부 |
| person | **COCO** (상한 sampling) | 손/팔 부분노출 포함 → hand FP 방지 |
| peach | **synthetic** (사실상 유일) | COCO에 없음, YCB 015 mesh 의존 |
| tennis_ball | synthetic + Open Images | COCO "sports ball"은 부적합 |
| mustard_bottle | synthetic + **YCB-Video**(real) | YCB 006, 실물 Morehouse ≈ French's |

- **synthetic**: **BlenderProc**로 YCB CAD mesh 6종(006/011/013/015/017/056) 렌더링. **배포 기하(D10)를 중심으로 randomize** — 카메라 elevation 55~90°(90=수직 하방), 거리 0.55~1.05m, 회색 optical table 표면(실측 sRGB 반영), 부분 가림 포함.
- **D10 배포 기하**: 회색 optical table 위, 카메라는 테이블 정중앙 상부 **~0.8m 수직 하방(top-down) 고정**. 이 형상·색을 synthetic에 반영해 domain gap 완충.

### 5.2 전처리 통일 (D7) — letterbox 416

train / quantize calibration / board worker **세 곳 모두 letterbox 416×416**으로 통일. 이유: 세 곳 전처리가 일치해야 calibration이 유효하고 학습-배포 분포가 맞는다. 대가는 worker에 pad 역변환 1줄 추가뿐.

### 5.3 학습 결과 (Gate 2)

- COCO pretrained YOLOv3-tiny에서 fine-tuning, **150 epoch (~2h, RTX 4060)**, augmentation 최대(실환경을 못 보므로 domain gap 완충재).
- train 15,799 / val 2,093 (ycbv/COCO/OI real 다수 포함).

**Gate 2 mAP@0.5 (최종 Hardswish 7-class 버전, run `pickplace_v3tiny_hswish`)**:

| class | mAP@0.5 |
|---|---|
| all | **0.766** |
| apple | 0.625 |
| peach | 0.955 |
| orange | 0.734 |
| banana | 0.738 |
| tennis_ball | 0.978 |
| mustard_bottle | 0.823 |
| person | 0.509 |

**혼동 진단**: confusion matrix에서 유일한 class 혼동이 mustard→banana(0.39). val을 소스별로 쪼개니 — synthetic-val mAP **0.993**(배포 유사 도메인엔 혼동 없음), ycbv-val에서만 banana FP 다수 → **혼동은 real 비디오의 어수선한 장면 탓**이지 우리 top-down 환경 탓이 아니라 판정하고 통과. (단 synthetic 0.993은 in-distribution 수치라 실물 최종확인은 Gate 5로 미룸 — 이 판단이 §7에서 시험대에 오른다.)

---

## 6. 핵심 문제 ① — SiLU → Hardswish (D11) ★가장 중요

이 작업에서 **가장 중요한 트러블슈팅**. 논리 흐름이 교훈적이라 따로 정리. (하드웨어적 배경은 §3.4.)

### 6.1 문제 발견
양자화(Gate 3) 첫 시도에서 cosine 유사도는 0.99+로 통과권이었는데, 로그에 결정적 경고:
```
[VAIQ_WARN]: The quantizer recognize new op `aten::silu_` as a float operator by default.
... type: aten::silu_, is not defined in XIR
```

### 6.2 원인
yolov5 v7.0의 Conv 기본 activation이 **SiLU**인데, SiLU는 우리 DPU에서 **미지원**(§3.4). 이대로 컴파일하면 conv마다 CPU subgraph가 끼어 파편화 → Gate 4(단일 DPU subgraph) 확정 실패.

> **교훈**: cosine 수치보다 `VAIQ_WARN`/unknown-op 경고를 먼저 봐야 한다. 근본 원인은 검증 누락 — 원조 darknet YOLOv3-tiny가 LeakyReLU(DPU-safe)라 가정했는데, yolov5 재구현의 기본값이 SiLU로 바뀐 걸 **학습 전에 확인하지 않음**.

### 6.3 해결 — 왜 Hardswish인가
DPUCZDX8G가 지원하는 activation 중 **SiLU와 형태가 가장 가까운 것이 Hardswish**. quantizer(PyTorch `Hardswish`→XIR `hardswish`)·compiler 양쪽 지원, 곡선이 가까워 pretrained 전이 손실 최소.

> ⚠️ **정정(2026-09-08, 배포 xmodel 실측)**: "conv에 fuse된다"고 적었으나 **실제 배포된 xmodel은 융합하지 않는다.**
> op 인벤토리를 덤프하면 `conv2d-fix` 13개가 **전부 `nonlinear=NONE`**이고, Hardswish는
> `hard-sigmoid-fix` 11개 + `eltwise-fix` 11개 쌍으로 풀려 있다. DPU가 지원하는 op만 쓴다는 것과
> 단일 DPU subgraph가 나온다는 결론은 그대로 유효하고, 융합 여부만 사실이 아니었다. (사용자 제시 논문 *"Efficient SAR Vessel Detection for FPGA-Based On-Satellite Sensing"*도 동일 해법. fallback = LeakyReLU(0.1).)

### 6.4 왜 재학습이 필요한가
activation은 "출력에 붙는 옵션"이 아니라 **가중치가 학습될 때 전제한 수식의 일부**다. 8.68M conv 가중치가 전부 SiLU 전제로 최적화돼 있어, activation만 바꿔치기하면 층마다 출력이 어긋나고 20여 층을 지나며 누적·증폭되어 정확도가 무너진다. SiLU↔Hardswish 정확 변환 공식도 없다. 그래서 **cfg 한 줄(activation)만 바꾸고 재학습**(데이터·hyp·anchor 전부 동일, 2h 재실행).

### 6.5 재발 방지 (Gate 0 신설)
2시간 학습 후에야 발견한 실수를 막기 위해, **학습 전 Inspector로 DPU 매핑을 선검증**하는 단계(Gate 0, `12a_inspect_docker.sh`)를 파이프라인에 추가. Inspector가 *"All the operators are assigned to the DPU"*를 확정한 뒤에만 학습.

```python
from pytorch_nndct.apis import Inspector
inspector = Inspector("0x101000016010406")           # B3136 fingerprint
inspector.inspect(model, torch.randn([1, 3, 416, 416]))
```

### 6.6 결과 — activation 교체 비용 = 0

| | SiLU 1차 | Hardswish 2차 |
|---|---|---|
| all mAP50 | 0.758 | **0.766** (+0.008) |
| banana | 0.694 | 0.738 (P 0.72→0.80) |

곡선이 가까워 같은 지점에 수렴, 오히려 소폭 개선. **재학습 판단이 옳았음이 수치로 확인**.

---

## 7. 양자화 · 컴파일 · 도메인 갭 (Gate 3, 4, 5)

### 7.1 양자화 (Gate 3) — INT8 PTQ

- 모델을 **forward-only로 개조**(pre/post는 클래스 밖으로), `torch.jit.trace` 통과.
- calibration = **학습 dataset subset 500장** (bitwidth=8, PTQ; §3.3의 스케일 확정 과정).
- **Gate 3(7-class hswish)**: cosine **head[0] 0.9923 / head[1] 0.9849**, `VAIQ_WARN`/unknown-op **0건**.
- **Gate 3(6-class D13)**: cosine **0.9757 / 0.9615** (기준선 0.95 상회 → 통과, head[1]=큰물체 약함).
- 기준: cosine > 0.95, mAP 손실 ≤ 3%p. head[1]이 0.99 소폭 하회하나 DPU hardswish 고정소수점 근사까지 반영한 **정직한 수치** → 검출 최종확인은 Gate 5로.

**VAI 2.5 배포 버그 2건 (우리 코드 아님, 패키지 내부 버그 — GitHub v2.5/v3.0 소스 대조로 해결)**:
1. `pytorch_nndct/.../hardswish.py __init__`이 미정의 심볼 `FixNeuronWithBackward` 초기화(dead line) → v3.0은 줄 삭제로 수정.
2. `hardswish.py forward`가 `fake_quantize_per_tensor()`를 옛 시그니처로 호출(필수 인자 `method`/`inplace` 누락) → `method=2`/`inplace=False` 보완.
   - docker가 일회용(`--rm`)이라 컨테이너 기동 직후 `sed`로 매번 패치하게 wrapper에 넣음. (함정 또 나오면 LeakyReLU로 즉시 전환하기로 선을 그었고, 두 건으로 끝남.)

### 7.2 컴파일 (Gate 4)

```bash
vai_c_xir -x quantize_result/yolov3_tiny_int.xmodel -a arch_b3136.json \
          -o compiled_yolov3_tiny -n yolov3_tiny_7class
```

컴파일 로그:
```
Total device subgraph number 4, DPU subgraph number 1
```
**`DPU subgraph number 1` = 통과.** "4"는 [입력 전달]+[DPU 연산 1덩어리]+[출력 텐서 2개]로 나뉜 것뿐이고, **연산이 전부 담긴 DPU 덩어리는 1개**(Inspector "all DPU"와 일치).

보드 `xdputil` 재검증: fingerprint 일치, 입력 **416×416×3 (fixpos 6)**, 출력 **26×26×33 (fixpos 2) · 13×13×33 (fixpos 3)** — decode_meta와 정합. (fixpos = §3.3의 `fix_point`.) **head마다 fixpos가 다르다**는 점에 주의 — worker는 텐서별로 읽으므로 동작엔 무영향이다.

xmodel 이력: 7-class `e2ca87c2` → D13 6-class `d925c711` → **D14 배포 `9bc6520c`**.

### 7.3 보드 검증 (Gate 5) — sim-to-real 갭 발견 → 핵심 문제 ②

**기술 검증은 통과**: config를 YOLO로 전환(worker 교체, `send_resized_input: false`, pick_logic 6 pickable), 라이브 파이프라인 기동. detector가 848×480 처리(**dpu_ms ~17-23ms, SSD보다 빠름**), `/detections` 정상 발행.

**실물 6종 top-down 검증 — 3종 실패**:
- ✅ banana(0.81)·orange(0.90)·apple(0.76) — 잘 검출
- ❌ **peach** → apple로 오분류 (peach 점수 0.018 = 사실상 0)
- ❌ **tennis_ball** 미검출 (0.017)
- ❌ **mustard_bottle** 미검출 (0.001)

threshold 0.05까지 낮춰도 안 뜸 → **threshold 문제가 아니라 인식 자체 실패**. 같은 프레임에서 3종 성공/3종 실패 = 전처리·decode 버그가 아니라 **class별 도메인 갭**. 패턴이 명확: **COCO real이 풍부한 class는 전이 성공, synthetic 의존 class는 실패**. §5.3에서 우려한 리스크가 실제로 터졌다.

**재배치 진단으로 원인 3갈래 분리**:

| 물체 | 원인 | 근거 |
|---|---|---|
| mustard | **자세(pose)** — 외형은 학습됨 | 누움 0.001 → 세움 **0.80** |
| tennis_ball | 약한 신뢰도 + 가림 (marginal) | 0.004 → 간격 벌리니 0.14~0.26 |
| peach | **외형 confusion** (제일 어려움) | 재배치 무관 apple 0.54. 플라스틱 복숭아가 모델엔 사과로 보임 |

### 7.4 도메인 갭 해소 — 데이터 전략 3회 수정 (D12 → D13 → D14)

**D12 (2026-07-08)** — 진짜 top-down에서 재검증하니 형상 artifact가 아니라 진짜 갭 확정(peach 0.02 / mustard 0.02). YCB mesh와 실물 외형이 근본적으로 달라 같은 mesh 재렌더로는 못 닫음 → **실촬영 이미지를 학습에도 사용** 결정(D2 완화).

**D13 (2026-07-09) — peach 드롭 + YCB real 스캔 도입**:
- **peach 드롭 → 6-class 재번호**. 우리 실물(분홍+잎) ≠ YCB 015(주황복숭아). apple과 최난도 혼동이라 교체 아닌 **드롭**.
- **mustard 유지**: 실물 ≈ YCB 006(둘 다 노란 French's형). 실패는 자세가 아니라 top-down 뷰 부족 → **YCB 벤치마크 real 스캔**(물체당 600장 + mask, N4/N5 카메라 = 우리 top-down 뷰)을 학습 도입. 촬영·라벨 수작업 0.
- **결과**: 재학습 Gate 2 all 0.748, Gate 3 cosine 0.976/0.962, Gate 4 subgraph 1. **보드 재검증**: mustard **0.02→0.814**(핵심 증명), tennis 0.677, orange 0.850, banana 0.777 — **5종 정상. apple만 0.489~0.549 경계.**

**apple 경계의 진짜 원인 규명**: 처음엔 apple 0.462로 잡혀 threshold hack(`apple:0.40`)을 넣었으나, 그 0.462는 **드롭한 peach(OOD)가 apple로 오검출**된 것이고 진짜 사과는 0.216이었음. peach를 치우자 진짜 사과 0.549로 회복. 배포엔 peach 없어 오검출은 무의미하나, 진짜 apple이 0.50 경계라 **flicker 위험**. 근본은 **color 도메인 갭**(실물 사과가 YCB 사과보다 밝음) + apple 자체 최난도(둥글어 orange/tennis 혼동). **데이터 양 문제 아님**(이미 6371 인스턴스).

**D14 (2026-07-09~10) — apple 타깃 실촬영 재학습**:
- 테이블에 **사과만** 올려 배포 카메라로 top-down 실촬영 ~54장. 단일물체라 auto-label(DPU top-1 box를 apple로 강제)로 **라벨 수작업 0**.
- `real_apple_yolo`(train 전량)로 추가 + `hsv_v` 0.40→0.50(밝기 갭 완충, `hsv_h`는 유지=apple↔orange 혼동 방지).
- **결과 — Gate 5 통과 🎉**: apple **0.489~0.549 → 0.876/0.899/0.875**(완전 해결). 나머지도 전부 유지/개선 — orange 0.85~0.88, banana 0.83~0.85, mustard 0.81~0.87, tennis 0.82~0.85. threshold hack은 0.50 default로 원복. **모델 교체(Gate 2~5) 완성.**

### 7.5 그 외 해결한 실무 이슈

| 이슈 | 원인 · 해결 |
|---|---|
| **GPU 드라이버 hang** (D14 재학습 2회 중단, epoch 0) | 자동 업데이트로 설치된 `nvidia-driver-595-open`(open 커널모듈+GSP)의 **로그 없는 silent hard-hang**(OOM/Xid/MCE 전무). → proprietary **`nvidia-driver-580`로 롤백** + `apt-mark hold`(자동 업데이트 차단) |
| **D435i FW 웨지** | FW 5.16.0.1이 librealsense 스트리밍 수십 초 후 RGB 프레임 정지(+dmesg `GET_CUR ... -32` 스톨). replug/재부팅 무효. → **FW 5.17.0.10 업데이트(`rs-fw-update`)로 완치**. 진단법 = cv2 V4L2 직접 캡처는 정상(HW 무죄, librealsense 경유만 사망) |
| BlenderProc empty annotation | `enable_segmentation_output()` 시점 객체만 등록 → object-pool 구조로 해결 |
| opencv/numpy 충돌 | yolov5 v7.0은 numpy 1.x → `opencv-python==4.10` 고정 |
| Pillow 10 `getsize` 제거 | 플롯 crash → `Pillow==9.5.0` 고정(지표·weight 무관) |
| `metrics_duration_sec: 0` 정수 버그 | 노드가 double 선언 → 정수면 detector 死. `0.0`으로 수정(모델 교체와 무관한 기존 config 버그) |

---

## 8. 파이프라인 통합 · 아키텍처 (Gate 6)

### 8.1 model-agnostic 설계 — 교체 범위가 좁은 이유

detector node ↔ worker process를 **JSON contract**로 분리 설계했기에, 모델을 바꿔도 **worker의 전처리 상수 + decode만** 바뀌고 노드/파이프라인/3D/다운스트림은 그대로다.

- **변경**: `vitis_ai_worker_yolo.py`(MODEL 416, LUT, YOLO grid decode+NMS, objectness pre-filter, class threshold), `vitis_ai_detector.yaml`(model_path·worker·`send_resized_input:false`), `pick_logic`(pickable/safety 분리).
- **불변**: detector node(handshake에서 input size 자동 수신), 3D·TF·camera·pipelining·metrics.

### 8.2 프로세스 분리 (핵심 문제 ③) — VART segfault

**VART `execute_async()`가 rclpy 노드 in-process에서 segfault/bus error**. 근본 원인 = **XIR graph/subgraph Python 객체가 GC되어 runner가 무효 상태 참조**. 해결 = **별도 worker 프로세스 + `self.graph`/`self.dpu_subgraphs`를 runner 수명 동안 보관**. 노드↔worker는 stdin/stdout pipe IPC.

### 8.3 콜백 파이프라이닝
`image_callback`은 **최신 프레임만 저장**하고 즉시 반환, `worker_loop` 전용 스레드가 연속 처리(묵은 프레임 버림). GIL 안전(무거운 DPU는 별도 프로세스). throughput 불변(**camera-limited**임이 드러남), latency ~31ms 감소.

### 8.4 QoS / latest-frame-only
카메라 color/depth 구독 `KEEP_LAST depth=1 BEST_EFFORT`(항상 최신 1장, 묵은 프레임 누적 방지), extrinsics `RELIABLE TRANSIENT_LOCAL`(한 번만 latched 발행), `/detections` 등 발행 `RELIABLE depth=1`.

### 8.5 Reverse projection (align 제거)
aligned depth 구독 시 realsense가 매 프레임 ~407k 픽셀 재투영 → 코어 100% → 카메라 12Hz throttle. 우리는 **bbox 중앙 1점**만 필요 → align 끄고 color 픽셀 1개를 `rs2_project_color_pixel_to_depth_pixel`(epipolar 선분 탐색)로 매칭. color↔depth ~15mm baseline이라 disparity 큼(60~90px)이라 단순 근사 불가. **~13→17 Hz, 코어 ~65% 회수.** (이 epipolar 탐색이 §9.2 P1 벡터화의 대상.)

### 8.6 Gate 6 실측 (2026-07-10, 4코어, 3분)
파이프라인 **15Hz 동기 · E2E 137ms = PASS**. CPU 79%/4코어(~3.17). 노드별: **target_3d 69%(최대)** / camera 57 / detector 50 / worker_dpu 36. dpu_ms 18(병목 아님). → target_3d가 최대 소비자로 드러나 §9 최적화의 1순위가 됨.

---

## 9. 파이프라인 최적화 상세 — 무엇을·어떻게·얼마나

> 이 장은 각 최적화를 **① 무엇을 줄이려고 → ② 어떻게 구현 → ③ 얼마나 줄었나 → ④ 무손실 검증** 순으로 상술한다. 두 라운드다: SSD 시절 확립돼 YOLO worker에 이식된 **검출단계 최적화(§9.1)**, 그리고 YOLO 교체 후 EtherCAT 통합 대비 **CPU 절감 phase 1+2(§9.2~9.3)**.

측정 방식: detector 콜백을 `img/pre/dpu/post/ipc/overlay`로 계측하고, 파이프라인은 노드별 CPU%·det_hz·e2e를 CSV로 수집(`run_gate6_perf.sh`). **DPU 추론 자체는 ~13-17ms(하드웨어 하한)**였고 진짜 병목은 **Python 전/후처리와 노드 간 전송**이었다.

### 9.1 검출단계 최적화 (SSD 확립 → YOLO worker 이식)

#### (1) LUT 전처리 — `pre_ms 42 → 12`
- **무엇을**: 입력 텐서 양자화 비용. 픽셀마다 `int8 = clip(round((픽셀−mean)×scale))`을 480×360×3 = 518,400 픽셀에 float 배열로 여러 패스 + 매번 임시배열 할당 → A53(메모리 대역폭 약함)에서 ~42ms.
- **어떻게**: 입력이 uint8이라 값이 **0~255 256가지뿐**이고 결과는 (값,채널)에만 의존 → 같은 계산을 수십만 번 반복 중임을 이용. 기동 시 채널별 256칸 int8 표를 1회 계산(`lut[c][v]=clip(round((v−mean[c])×scale),−128,127)`), 매 프레임은 **표 조회 3번**(`arr[:,:,c]=lut[c][resized[:,:,c]]`). YOLO worker(`build_input_lut`)는 `v/255×2^fix` 단일 표로 동일 원리.
- **얼마나**: `pre_ms 42 → ~12ms`.
- **검증**: 표를 기존 수식과 동일한 round/clip으로 생성 → 출력 **bit-identical**(무작위 이미지).

#### (2) 후처리 사전필터 — `post_ms 20 → 7`
- **무엇을**: SSD는 16,436개 prior 전부에 softmax+box decode+NMS. 대부분 background인데 전수 처리 → ~20ms.
- **어떻게**: **안전한 필요조건(necessary-condition) 사전필터**. softmax 케이스에서 `prob_c ≥ t_c`이려면 필연적으로 `z_c − z_background ≥ logit(t_c)`(sigmoid 상계에서 유도) — 이 조건은 **뺄셈·비교만**(softmax 불필요). FP 경계값을 안 놓치게 ε 마진. 통과한 소수에만 정확한 softmax/decode/NMS, `loc` 역양자화도 후보만. YOLO worker(`decode_head`)는 `conf = sig(obj)·sig(cls) ≤ sig(obj)`라 **objectness raw logit ≥ logit(threshold)** 인 cell만 `np.nonzero`로 골라 sigmoid/decode — INT8 raw 그대로 비교(임계값을 `2^fix` 스케일로 변환).
- **얼마나**: `post_ms 20 → ~7ms`.
- **검증**: 필터가 필요조건이라 통과 집합은 전수 처리의 상위집합 → 이후 정확한 임계값으로 잘라내므로 **최종 detection 동일**(무작위 30회).

#### (3) 입력 resize 후 전송 (`send_resized_input`, SSD)
- **무엇을**: node→worker pipe IPC payload. 원본 848×480×3을 그대로 보내면 프레임당 2.76MB.
- **어떻게**: worker로 보내기 전 480×360으로 resize(검출은 어차피 480×360 입력이라 무손실, bbox는 source 좌표로 반환).
- **얼마나**: payload 2,764,800 → 518,400 byte(**−81%**). (YOLO는 letterbox를 worker가 하므로 `false` — 대신 phase 3 POSIX shm 후보.)

#### (4) 콜백 파이프라이닝
- **무엇을**: latency. 기존엔 `image_callback`이 worker 응답까지 블록 → executor가 묶여 콜백 간 idle.
- **어떻게**: `image_callback`은 최신 프레임만 저장 즉시 반환, `worker_loop` 전용 스레드가 latest-frame 연속 처리(§8.3).
- **얼마나**: **latency ~31ms 감소**, throughput 불변(시스템이 camera-limited임이 드러남).

#### (5) reverse projection (full-frame align 제거)
- **무엇을**: realsense 코어 점유. aligned depth 구독 시 매 프레임 ~407k 픽셀 재투영 → 단일 스레드가 코어 100% → 카메라 12Hz throttle.
- **어떻게**: align 끄고 bbox 중앙 1점만 epipolar 선분 탐색으로 매칭(§8.5).
- **얼마나**: realsense **코어 ~65% 회수**, 검출 **~13 → 17 Hz**.

#### (6) 잡다한 부하 제거
| 기법 | 효과 |
|---|---|
| overlay off | 콜백 **−~70ms/frame**(디버그 전용) |
| worker file-log off | 프레임당 ~18회 file I/O 제거 |
| 카메라 1280×720 → 848×480 | realsense 부하 ~2.3배↓ + busy-spin stall 해소 |

**§9.1 누적**: `processing_ms 88.7 → 45.8`, throughput `8.6 → 13 Hz`(compute), align 제거로 **~17 Hz**, latency `349 → 261ms`. **모든 최적화에서 검출 정확도 불변.**

---

### 9.2 CPU 절감 phase 1 (76.8% → 53.1%, −0.95코어)

> 목표가 throughput(fps)이 아니라 **CPU 절감**이다. RT 커널 위에서 EtherCAT + 로봇 제어가 공존하려면 비전이 코어를 비워줘야 하기 때문. 전부 **무손실** 원칙. 베이스라인(순정커널+FW 5.17): 총 76.8% (target_3d 68.9 / camera 53.6 / detector 43.4 / worker 36.9 / base 14.0 / pick 9.7).

#### P1 — epipolar 벡터화 (`pick_target_3d_node.py`) ★최대 단일 절감
- **무엇을**: `pick_target_3d_node`의 CPU. 이 노드가 **69%로 최대 소비자**였다. 원인은 `rs2_project_color_pixel_to_depth_pixel`의 선분 탐색이 **Python `for` 루프**로 구현돼, 선분 위 최대 ~126개 샘플점을 **한 점씩** 걷기(각 점마다 scalar deproject → matmul → reproject → 거리)를 **pick마다** 수행.
- **어떻게**: 루프를 numpy 일괄 연산으로 대체. `a = arange(steps+1)/steps`로 선분 전체 샘플 좌표를 한 번에 만들고 → **단일 fancy-index gather**(`depth_img[vi, ui]`)로 depth를 전부 읽고 → 유효 후보를 `(3, N)` 행렬로 back-project → `R_dc @ pts + t_dc` **단일 matmul**로 depth→color 매핑 → `argmin`으로 최근접 선택. python 루프 126회 → 벡터 연산 몇 개.
- **얼마나**: **target_3d 68.9 → 12.0%** (단일 최대 절감). 
- **검증**: `np.rint == int(round)`(둘 다 round-half-to-even), mask 순서가 후보 집합 보존, `argmin`이 루프의 strict `<`처럼 첫 최소 반환 → **수치 동일**. 오프라인 600px + **라이브 4500 pick A/B mismatch 0**. 루프 레퍼런스(`..._loop`)는 코드에 보존, `epipolar_ab_check` 파라미터로 상시 재검증 가능.

#### P2 — 정적 구독 prune (동 파일)
- **무엇을**: executor dispatch 낭비. `camera_info`×2(각 30Hz)+`extrinsics`는 stream profile이 고정이라 값이 안 바뀌는데도 매 메시지 콜백 dispatch가 계속 돎.
- **어떻게**: 첫 수신 후 `destroy_subscription`(타이머로 안전하게, `prune_static_subs`). intrinsics/extrinsics는 불변이라 무손실.
- **얼마나**: (소폭, target_3d 절감에 포함.)

#### P3 — letterbox cv2 경로 (`vitis_ai_worker_yolo.py`)
- **무엇을**: YOLO worker 전처리 `pre_ms`. letterbox + BGR→RGB + `/255` LUT를 fancy-index 3회 + **매 프레임 pad fill**로 수행.
- **어떻게**: `cv2.cvtColor(BGR2RGB)` + `cv2.LUT`(OpenCV 최적화 경로, `cv2.LUT(x,lut)==lut[x]`) + **pad fill을 (nh,nw)가 바뀔 때만 캐시**(`out_buf.fill(lut[114])` 1회 후 재사용). cv2.LUT가 int8 표를 거부해 uint8 뷰로 조회 후 int8 재해석.
- **얼마나**: `pre_ms 17.5 → 9.0`.
- **검증**: **bit-identical PASS**(cv2.LUT 결과가 fancy-index와 동일 재배열).

#### P4 — depth 30 → 15fps (`realsense_pick_place.yaml`)
- **무엇을**: camera + t3d의 depth 경로 CPU. depth를 30fps로 매 프레임 처리하나, 3D는 pick 시점의 **latest-depth만** 소비 → 절반이 낭비.
- **어떻게**: `depth_profile`을 30→15fps로. depth 최신성이 33→66ms로 늘지만 **정적 pick 장면엔 무해**.
- **얼마나**: **camera 53.6 → 40.2%** (P4가 큰 몫).

#### 검출률 캡 (`process_period_sec: 0.045`)
- **무엇을**: P3로 uncap 시 ~19Hz까지 올라 절감분을 도로 소모.
- **어떻게**: `process_period_sec 0.045`(2프레임 주기 = 15Hz 고정).
- **함정**: 0.062는 프레임 지터로 3프레임 주기(13.8Hz)로 떨어짐 → 45ms가 정답. 0.0은 ~19Hz(+3~5%p CPU).

| 노드 | 전(76.8% 총) | 후(53.1% 총) |
|---|---|---|
| target_3d | **68.9** | **12.0** |
| camera | 53.6 | 40.2 |
| detector | 43.4 | 39.8 |
| worker(DPU pre/post/IPC) | 36.9 | 25.4 |
| target_base / pick_logic | 14.0 / 9.7 | 11.1 / 7.6 |
| e2e | 124ms | 92ms |

---

### 9.3 CPU 절감 phase 2 (53.1% → ~44%, 누적 −1.25코어)

#### W1-a — depth lazy 변환 (`pick_target_3d_node.py`)
- **무엇을**: t3d의 depth 변환 CPU. `depth_callback`이 매 프레임(15fps) 도착 즉시 cv_bridge로 변환.
- **어떻게**: `depth_callback`은 msg 저장만. pick 시점에만 `depth_msg_to_view` = `np.frombuffer(msg.data, '<u2').reshape(H,W)` **zero-copy view**(복사 없음). cv_bridge passthrough와 결과 동등하나 변환 비용 자체를 소거.
- **얼마나**: cv_bridge 대비 **~26×** 빠른 변환 + pick당 1회만 수행.
- **검증**: 라이브 `np.array_equal(frombuffer_view, cv_bridge) == True`.

#### W1-b — base 정적 TF 캐시 (`pick_target_base_node.py`)
- **무엇을**: base 노드 CPU. 매 메시지 `tf_buffer.lookup_transform` + `do_transform_point`.
- **어떻게**: `base_link ← camera` TF가 static이므로 **첫 lookup에서 `(R 3×3, t 3)`을 캐시** → 이후 `R @ p + t` **단일 matmul**로 변환. `static_tf_cache` 파라미터로 off 가능(안전장치).
- **얼마나**: **base 10.4 → 6.7%**.
- **검증**: tf2 대비 **오차 0.000e+00**.

#### W1-c — worker decode 상수 hoist
- **무엇을**: worker per-frame 중복 계산. `2^fix`·`thr_q(=min_obj_logit×scale)`·anchors 배열을 매 프레임 재계산.
- **어떻게**: `load_model`의 `head_params`로 **1회 계산 후 재사용**.
- **얼마나**: 소폭(worker post 경감).

#### W2 — FastDDS + SHM ★phase 2 최대 레버
- **무엇을**: 노드 간 DDS 전송 CPU. 같은 보드 안에서도 CycloneDDS가 1.22MB color Image를 **UDP loopback**(직렬화 + 커널 소켓 왕복)으로 나름.
- **어떻게**: `.bashrc`의 `RMW_IMPLEMENTATION`을 cyclonedds → **fastrtps** + `FASTRTPS_DEFAULT_PROFILES_FILE`(SHM segment 16MB). 같은 호스트 노드 간에는 **shared memory transport**로 직접 전달 → UDP loopback 제거.
- **얼마나**: **camera 40 → 29, detector 39 → 31, 총 −6.6%p**(단일 최대 레버). 되돌리기 = bashrc 두 줄을 cyclonedds 한 줄로.

#### W3 — 노드 병합 (시도 → **기각·롤백**)
- **무엇을(의도)**: pick_logic+t3d+base 3노드를 단일 프로세스로 합쳐 executor/IPC 오버헤드 절감.
- **어떻게**: `pick_post_stack` 단일 프로세스로 병합(코드 잔존).
- **결과**: **+5.4pt 역효과 실측 → 기각.** 원인 = rclpy executor가 **매 콜백마다 전체 waitset을 재구성**해, 엔티티 3배가 모든 콜백 dispatch 단가를 올림. **rclpy에선 노드 병합으로 CPU 절감 금지** — 실익은 rclcpp composition으로만 가능. launch는 3노드 분리로 복원.

| 노드 | phase1 후 | phase2 후 |
|---|---|---|
| camera | 40.2 | **28.7** |
| detector | 39.8 | **31.3** |
| worker | 25.4 | 25.7 |
| target_3d | 12~20 | 17.3 |
| target_base | 11.1 | **6.7** |
| pick_logic | 7.6 | 7.2 |
| **총(4코어 평균)** | 53.1 | **~44** |

**최종**: 총 CPU **76.8% → ~44%**(≈3.07 → **1.8코어**, **−1.25코어**), e2e 124→81ms, **det 15Hz 유지**.

### 9.4 EtherCAT 헤드룸 판정 ("CPU 더 줄여야 하나?" = 아니오)
비전 ~1.8코어 → **여유 ~2.2코어**. IgH EtherCAT 1kHz cyclic ~0.1~0.3코어 + 제어(IK·traj·SM) ~0.1~0.4코어 → 합쳐도 여유의 1/3 이하(**3배+ 마진**). **CPU 절감은 여기서 종료**, 다음 우선순위는 성능이 아니라 RT 커널 완성이었다(RT 트랙은 2026-07-15 종결). 남은 성능 개선(fps↑/latency↓)은 RT 위에서 baseline 재측정하는 게 맞다(§10).

---

## 10. 남은 것 (phase 3 성능 레버 — RT 완성 후)

e2e 81ms 분해(중앙값): 캡처→도착 **40.7ms**(센서 물리, 30fps 상한) / IPC 6.1 / pre 8.2 / **DPU 16.9**(모델 결정) / post 4.2. **처리 합계 37.8ms라 캡만 풀면 ~26Hz 체력 이미 있음.**

| # | 레버 | 효과 | 난이도 |
|---|---|---|---|
| 1 | node↔worker POSIX shm IPC | latency −5~6ms, CPU −3~5pt (유일 win-win) | 중 |
| 2 | 캡 `process_period_sec` 조정 | 15→20Hz / uncap ~26Hz | 설정 1줄 |
| 3 | worker 3단 pipelining (pre‖DPU‖post) | 30Hz까지, latency 불변 | 중상 |
| 4 | color 60fps 모드 | latency −15ms | 설정+검증 |
| 5 | (Gate 7) live 체크리스트 완료 | 최종 acceptance | — |
| 6 | rclcpp 포팅 + composition | CPU·latency 동시 대폭 | 장기 |

> 그 외 미완: Gate 7 live 6종×위치별 검출·threshold tuning. (camera-to-base **TF 캘리브레이션**은 2026-07-27 eye-to-hand로 **완료** — 16 pose, residual 1.8 cm / 1.7°, E21 yaw +5.14° 재베이스 포함. `/pick_target_base`는 실제 로봇 base_link 좌표다.)

---

## 11. 결정 로그 요약 (D1~D14)

| # | 결정 | 한 줄 근거 |
|---|---|---|
| D1 | 모델 = YOLOv3-tiny (7→6-class) | DPU 지원 op만으로 단일 subgraph |
| D2 | 실환경 촬영은 **학습 미사용**(검증만) *→ D12~14 완화* | 수작업 라벨 회피 |
| D3 | 학습 = synthetic(YCB mesh) + 공개 dataset | D2 귀결 |
| D4 | mustard 실물 Morehouse ≈ YCB006 → YCB 소스 유지 | 배포 거리 라벨차 무시 |
| D5 | 툴체인 = **VAI 2.5 docker 고정** | 보드 runtime 버전 일치(§3.6) |
| D6 | 프레임워크 = PyTorch (vai_q_pytorch) | 양자화 도구 요구 |
| D7 | 전처리 = **letterbox 416** (train/calib/worker 통일) | 분포 정합 |
| D8 | person = safety class, threshold 낮게, pickable 분리 | recall 우선 |
| D9 | fast-finetune/QAT 시 학습 subset만 | D2 준수 |
| D10 | 배포 기하 = 회색 table, top-down 0.8m | 합성에 반영, gap 감소 |
| D11 | **activation SiLU → Hardswish 재학습** | SiLU DPU 미지원(§3.4) |
| D12 | D2 재검토 → 소량 real 학습이미지 추가 | peach/mustard 갭 |
| D13 | **peach 드롭 → 6-class** + YCB real 스캔 도입 | 실물≠mesh |
| D14 | apple **배포 카메라 실촬영** 학습 추가 | color 갭 |

---

## 부록 — 게이트별 명령 요약

```bash
# Gate 0 — Inspector (학습 전 DPU 매핑 검증)
bash 12a_inspect_docker.sh

# Gate 2 — 학습 (host GPU)
bash 11_train.sh                    # yolov3-tiny-hswish.yaml, 150 epoch

# Gate 3 — 양자화 (VAI 2.5 docker, PTQ)
python yolov3_tiny_quant.py --quant_mode calib --subset_len 500
python yolov3_tiny_quant.py --quant_mode test --subset_len 1 --batch_size 1 --deploy

# Gate 4 — 컴파일 (XIR → DPU 명령어 xmodel)
vai_c_xir -x quantize_result/yolov3_tiny_int.xmodel -a arch_b3136.json \
          -o compiled_yolov3_tiny -n yolov3_tiny_7class

# Gate 5 — 보드 정지 이미지 격리 검증
python3 ~/vitis_ai_work/scripts/yolov3_tiny_image_test.py

# Gate 6 — 풀 파이프라인 + 성능 측정
ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py
bash ~/vitis_ai_work/perf/run_gate6_perf.sh 180
```
