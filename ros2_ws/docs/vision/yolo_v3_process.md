# YOLOv3-tiny 7-Class 비전 모델 교체 — 과정과 논리

> 작성: 2026-07-07
> 목적: SSD stand-in → YOLOv3-tiny 7-class 교체 작업을 **"어떤 선택을 왜 했는가"** 중심으로 처음부터 정리.
> 명령어·게이트 상세는 `yolov3_tiny_execution_plan.md`, 파이프라인 구조는 `workflow.md` 참고.

---

## 0. 출발점과 목표

**상황**: Kria KV260 pick&place 시스템은 이미 동작 중이다. RealSense D435i → DPU 검출 → 2D 필터 → 단일점 3D(역투영) → base frame 목표점까지 ROS2로 흐른다(~17 Hz). 단, 검출 모델이 **SSD ADAS stand-in**(car/bicycle/person)이라 실제로 집을 물체를 못 잡는다.

**목표**: 이 stand-in을 우리가 실제로 집을 **7종 물체 검출기**로 교체한다.
- class 고정: `0 apple, 1 peach, 2 orange, 3 banana, 4 tennis_ball, 5 mustard_bottle, 6 person`
- 최종 산출물: DPU(DPUCZDX8G_ISA1_B3136)가 실행할 수 있는 `.xmodel`

**핵심 제약 3가지가 모든 선택을 지배한다**:
1. 검출 결과가 pick&place에 쓸모 있어야 한다 (class + 2D 위치 + 깊이).
2. KV260 DPU에 **실제로 올라가야** 한다 (Vitis-AI 지원 op만 사용).
3. 파이프라인 나머지(3D·TF·downstream)는 건드리지 않는다.

이 3번은 이미 시스템이 **model-agnostic JSON contract**(detector node ↔ worker process)로 설계돼 있어서 가능하다. 모델을 바꿔도 worker의 전처리 상수 + decode만 바뀌고 노드/파이프라인/3D는 그대로다. 그래서 교체 범위가 좁다.

---

## 1. 왜 YOLOv3-tiny인가 (모델 선택)

프로젝트 원칙은 "논문 정확도보다 **KV260 배포 가능성**이 우선"이다. 후보는 SSD/SSDLite-MobileNet, tiny/pruned YOLOv3, instance segmentation, pose estimation이었다.

- pick&place엔 **class + 2D bbox**면 충분하다 → segmentation/pose는 과잉.
- SSD는 가장 보수적이지만 이미 stand-in으로 쓰던 계열이고, 우리 물체엔 anchor/prior 튜닝 부담이 있다.
- **YOLOv3-tiny**를 고른 결정적 이유는 **DPU 지원 op만으로 구성된다는 점**이다. 구조가 `conv + BatchNorm + LeakyReLU + maxpool + nearest upsample + concat`뿐인데, UG1414 v2.5 Table 20(DPUCZDX8G 열)에서 이 op들이 전부 지원된다. 즉 CPU fallback 없이 **네트워크 전체가 DPU에 통째로 올라간다**.

이것이 나중에 "단일 DPU subgraph"(Gate 4)를 가능하게 하는 근거였다.

---

## 2. 왜 yolov5 코드로 학습하는가 (프레임워크 선택)

여기서 혼동하기 쉬운 지점: **"구조는 YOLOv3-tiny, 학습 도구는 yolov5"**다. 둘은 별개다.

우리 양자화 도구 `vai_q_pytorch`는 **PyTorch `nn.Module`만** 입력으로 받는다(D6). 그래서 "PyTorch로 YOLOv3-tiny를 학습하는 관리된 방법"이 필요했다. 선택지는 셋:

| 후보 | 결과 | 이유 |
|---|---|---|
| **darknet** (원조 YOLOv3, C) | 탈락 | 출력이 `.weights`(darknet 전용) → 어차피 PyTorch로 변환 필요(변환기가 숨은 지뢰). mosaic 등 최신 augmentation 부재 → 합성만으로 학습하는 우리 전략에 불리 |
| **ultralytics yolov5 v7.0 repo** | **채택** | YOLOv3-tiny 구조 yaml(`models/hub/yolov3-tiny.yaml`) 제공. PyTorch 직행 → 양자화 직결. COCO pretrained 제공. augmentation·검증 도구 완비 |
| **ultralytics v8 패키지** | 탈락 | yolov3-tiny**u**는 anchor-free head라 출력 구조가 달라 DPU decode 계획과 불일치 |

**v7.0 태그로 고정**한 이유: 고전 **anchor 기반 Detect head**를 쓰는 마지막 안정 릴리스라, 우리가 짜둔 보드 decode 공식과 정확히 맞는다.

**부수 효과 (알고 선택한 것)**: yolov5로 학습하면 bbox decode 공식이 darknet 원조가 아니라 yolov5식이다.
```
xy = (sigmoid(t)*2 - 0.5 + grid) * stride
wh = (sigmoid(t)*2)^2 * anchor_pixel
conf = sigmoid(obj) * sigmoid(cls)
```
보드 worker와 `decode_meta.json`을 처음부터 이 공식으로 작성하고 단위테스트까지 통과시켰다. 이름과 실체가 어긋나는 부분은 없다.

---

## 3. 왜 촬영 안 하고 합성+공개데이터인가 (데이터 전략, D2/D3)

**D2 (사용자 결정)**: 실환경(D435i) 촬영 이미지를 **학습(weight 업데이트)에 쓰지 않는다.** 검증·평가에는 써도 된다.

이유: 물체를 일일이 촬영·라벨링하는 수작업이 번거롭고, 우리 물체(과일·공·병)가 인식이 복잡한 대상이 아니다. 대신 **합성 렌더링 + 공개 데이터셋**으로 학습한다(D3).

**소스별 역할과 이유**:

| class | 주력 소스 | 왜 |
|---|---|---|
| apple/orange/banana | **COCO** (real 사진 다수) | 흔한 물체라 공개 real 데이터가 풍부 |
| person | **COCO** (상한 두고 sampling) | 손/팔 부분 노출 포함 → hand false-positive 방지 |
| peach | **synthetic** (사실상 유일) + OI 소량 | COCO에 없음. YCB 015 mesh 의존 |
| tennis_ball | synthetic + Open Images "Tennis ball" | COCO "sports ball"은 일반 공이라 부적합 |
| mustard_bottle | synthetic + **BOP ycbv**(real 비디오) | YCB 006 mesh. 실물은 Morehouse(외형이 French's=006과 거의 동일, D4) |

- **synthetic**: BlenderProc로 YCB CAD mesh 6종(006/011/013/015/017/056) 렌더링. **배포 기하(D10)를 중심으로 randomize** — 카메라 elevation 55~90°(90=수직 하방), 거리 0.55~1.05m, 회색 optical table 표면, 부분 가림 포함.
- **YCB-Video(BOP ycbv)**: mustard/banana가 **real 이미지**에서도 잡히는지 cross-domain 보강.
- **COCO/Open Images**: real 일반화 + 클래스 균형.

이 전략의 **알려진 리스크**를 처음부터 인지했다: 합성/공개 데이터의 분포와 우리 실물+촬영조건이 다르면 sim-to-real 갭이 생긴다. 계획서에 contingency ladder(threshold → 재렌더 → D2 재검토)를 미리 넣어둔 이유다. (→ 9장에서 이 리스크가 실제로 터진다.)

### 3-1. 왜 배포 기하를 데이터에 반영하나 (D10)

사용자가 실측 정보를 줬다: optical table(회색) 위 물체, 카메라는 **테이블 정중앙 상부 ~0.8m에서 수직 하방(top-down) 고정**. 테이블 표면색 sRGB 어두움(110,110,108)/평균(156,155,151)/밝음(190,190,185).

합성 렌더가 이 형상·색을 따르게 만들면 domain gap이 줄어든다. 그래서 카메라 샘플링과 테이블 material을 이 값에 맞췄다. (BlenderProc의 sRGB→linear 변환까지 반영.)

---

## 4. 왜 letterbox 전처리인가 (D7)

원래 계획은 "단순 resize"였다. 하지만 학습 프레임워크 yolov5의 **표준 전처리가 letterbox**(비율 유지 resize + 회색 pad 114)다. 프레임워크 내부와 싸우는 대신, **train / quantize calibration / board worker 세 곳 모두 letterbox 416×416으로 통일**했다.

이유: 세 곳의 전처리가 일치해야 calibration이 유효하고 학습-배포 분포가 맞는다. 대가는 worker에 pad 역변환 1줄 추가뿐이다. top-down 배치라 rotation 180°+flipud augmentation도 유효하다.

---

## 5. 왜 Vitis-AI 2.5로 고정인가 (툴체인, D5)

보드 runtime이 **2.5.0**(DPUCZDX8G_ISA1_B3136, fingerprint `0x101000016010406`)이다. 툴체인 버전이 어긋나면 컴파일된 xmodel이 안 올라간다. 그래서 **VAI 2.5 docker로 고정**한다.

- docker image = `xilinx/vitis-ai-cpu:2.5.0.1260` (그 안에서 `conda activate vitis-ai-pytorch`).
- 주의로 기록: `vitis-ai-pytorch-cpu` 분리 이미지는 3.x부터고 2.5엔 없다(초기에 잘못된 태그를 썼다가 Docker Hub API로 확인·정정).
- 실행 경로: **학습 = 데스크톱 native GPU(RTX 4060)**, **양자화/컴파일 = 데스크톱 CPU docker**(tiny 모델 PTQ는 CPU로 수 분이면 충분).

---

## 6. 실행 과정과 게이트

전 과정을 **게이트(gate)**로 나눠 각 단계에서 통과 조건을 두고 진행했다. "다음 단계로 넘어가기 전에 이건 확인됐다"를 보장하기 위해서다.

### Phase 1 — 데이터 조립 (Gate 1 통과)
합성 렌더 + COCO/OI/ycbv 추출 → remap → scene 단위 train/val split → contact-sheet 육안 검수.
결과: **train 15,799 / val 2,093**. class별 인스턴스 균형 확인.

### Phase 2 — 학습 (Gate 2 통과)
COCO pretrained YOLOv3-tiny에서 fine-tuning, 150 epoch(~2h, RTX 4060), augmentation 최대(실환경을 못 보므로 domain gap 완충).

**결과 (최종 Hardswish 버전, 7장 참고)**: val(ycbv/COCO/OI real 다수 포함) mAP@0.5 **all 0.766**.

| class | mAP@0.5 |
|---|---|
| apple | 0.625 |
| peach | 0.955 |
| orange | 0.734 |
| banana | 0.738 |
| tennis_ball | 0.978 |
| mustard_bottle | 0.823 |
| person | 0.509 |

**혼동 진단**: 전체 confusion matrix에서 유일한 class 혼동이 mustard→banana(0.39)였다. val을 소스별로 쪼개 평가하니:
- synthetic-val: mAP 0.993, mustard 대각 0.98 → 배포 유사 도메인에선 혼동 없음.
- ycbv-val: banana FP 다수 → **혼동은 ycbv 가림/클러터 한정**.

즉 우려한 혼동은 real 비디오의 어수선한 장면 탓이지 우리 환경 탓이 아니라고 판정하고 통과시켰다. (단 synthetic 0.993은 in-distribution 수치이므로 실물 최종 확인은 Phase 5로 미뤘다.)

---

## 7. 핵심 문제와 해결 — SiLU → Hardswish (D11)

이 작업에서 **가장 중요한 트러블슈팅**이다. 논리 흐름이 교훈적이라 따로 정리한다.

### 문제 발견
양자화(Gate 3) 첫 시도에서 cosine 유사도는 0.99+로 통과권이었는데, 로그에 결정적 경고가 있었다:
```
[VAIQ_WARN]: The quantizer recognize new op `aten::silu_` as a float operator by default.
... type: aten::silu_, is not defined in XIR
```

### 원인
yolov5 v7.0의 Conv 기본 activation이 **SiLU**다. 그런데 SiLU는 우리 DPU(UG1414 Table 20)에서 **미지원**이다. 이대로 컴파일하면 conv마다 CPU subgraph가 끼어 파편화 → Gate 4(단일 DPU subgraph) 확정 실패.

**교훈으로 기록**: cosine 수치보다 `VAIQ_WARN`/unknown-op 경고를 먼저 봐야 한다. 그리고 이 실수의 근본 원인은 내 검증 누락 — 원조 darknet YOLOv3-tiny가 LeakyReLU라 DPU-safe라고 가정했는데, yolov5 재구현의 기본값이 SiLU로 바뀐 걸 학습 전에 확인하지 않았다.

### 해결: 왜 Hardswish인가
UG1414 v2.5에서 DPUCZDX8G가 지원하는 activation 중 SiLU와 **형태가 가장 가까운 것이 Hardswish**다. Hardswish는 quantizer(PyTorch `Hardswish`→XIR `hardswish`)·compiler 양쪽에서 지원된다. (⚠️ 2026-09-08 정정: 당시 "conv에 fuse"라고 적었으나 **배포된 xmodel은 융합하지 않는다** — `conv2d-fix` 13개가 전부 `nonlinear=NONE`이고 Hardswish는 `hard-sigmoid-fix`+`eltwise-fix` 쌍 11조로 풀려 있다. 단일 DPU subgraph 결론은 그대로 유효.) SiLU와 곡선이 가까워 pretrained 전이 손실이 최소다. (사용자가 제시한 논문 "Efficient SAR Vessel Detection for FPGA-Based On-Satellite Sensing"도 동일 해법을 씀. fallback으로 LeakyReLU(0.1)도 준비.)

### 왜 재학습이 필요한가
activation은 "출력에 붙는 옵션"이 아니라 **가중치가 학습될 때 전제한 수식의 일부**다. 8.68M개 conv 가중치는 전부 SiLU 전제로 최적화돼 있어서, activation만 바꿔치기하면 층마다 출력이 어긋나고 20여 층을 지나며 누적·증폭돼 정확도가 무너진다. SiLU↔Hardswish 정확 변환 공식도 없다. 그래서 **cfg 한 줄(activation) 바꾸고 재학습**했다(데이터·hyp·anchor 전부 동일, 2h 재실행).

### 재발 방지
2시간 학습 후에야 발견한 실수를 막기 위해, **학습 전 Inspector로 DPU 매핑을 선검증**하는 단계(Gate 0, `12a_inspect_docker.sh`)를 파이프라인에 추가했다. Inspector가 "All operators assigned to the DPU"를 확정한 뒤에만 학습에 들어간다.

### 결과 (activation 교체 비용 = 0)
| | SiLU 1차 | Hardswish 2차 |
|---|---|---|
| all mAP50 | 0.758 | **0.766** (+0.008) |
| banana | 0.694 | 0.738 (P 0.72→0.80) |

곡선이 가까워 같은 지점에 수렴했고 오히려 소폭 개선. **재학습 판단이 옳았음이 수치로 확인**됐다.

---

## 8. 양자화·컴파일 (Gate 3/4 통과)

### VAI 2.5 배포 버그 2건 (GitHub 소스 대조로 해결)
Hardswish 모델을 양자화하려 하니 패키지 **내부**에서 두 번 터졌다. 우리 코드가 아니라 Vitis-AI 2.5 `pytorch_nndct`의 배포 버그였다:
1. `hardswish.py __init__`이 미정의 심볼 `FixNeuronWithBackward`를 초기화 (어디서도 안 쓰는 dead line).
2. `hardswish.py forward`가 `fake_quantize_per_tensor()`를 옛 시그니처로 호출 (필수 인자 `method`/`inplace` 누락).

둘 다 GitHub의 v2.5/v3.0 소스를 직접 대조해 확인했다 — v3.0은 ①을 줄 삭제로 고쳤고, ②는 `fix_ops.py`의 실제 시그니처를 보고 기본 rounding(`method=2`)+`inplace=False`로 보완. docker가 일회용(`--rm`)이라 컨테이너 기동 직후 `sed`로 매번 패치하게 wrapper에 넣었다.

**한도 설정**: hswish 경로에서 함정이 또 나오면 검증이 두터운 LeakyReLU로 즉시 전환하기로 선을 그었다(세 번째 패치는 우회로보다 비싸므로). 결과적으로 두 건으로 끝났다.

### Gate 3 — 양자화 결과
calibration 500장 완주, cosine **head[0] 0.9923 / head[1] 0.9849**, `VAIQ_WARN`/unknown-op **0건**. head[1]이 0.99를 소폭 하회하나, 이는 DPU의 hardswish 고정소수점 근사까지 반영한 **정직한 수치**이고 본 기준(mAP 손실 ≤3%p)의 프록시로는 통과권. 검출 수준 최종 확인은 보드(Gate 5)로 미뤘다.

### Gate 4 — 컴파일 결과
```
Total device subgraph number 4, DPU subgraph number 1
```
`DPU subgraph number 1` = 통과. "4"는 [입력 전달]+[DPU 연산 1덩어리]+[출력 텐서 2개]로 나뉜 것뿐이고, **연산이 전부 담긴 DPU 덩어리는 1개**다(Inspector의 "all DPU"와 일치). 보드에서 `xdputil`로 재검증: fingerprint 일치, 입력 416×416×3(fixpos 6), 출력 26×26×36·13×13×36(fixpos 2) — decode_meta와 정합.

---

## 9. 보드 검증 (Gate 5) — sim-to-real 갭 발견

### 기술 검증은 통과
config를 YOLO로 전환(worker 교체, `send_resized_input: false`, pick_logic 6 pickable)하고 라이브 파이프라인 기동. detector가 848×480을 처리(**dpu_ms ~17-23ms**, SSD보다 빠름), `/detections` 정상 발행. 기존 프레임 12장 테스트에서 **person 일반화 + 오검출 0건** 확인 → 보드 경로 자체는 건강.

(도중에 기존 config 버그 발견·수정: `metrics_duration_sec: 0` → `0.0`. 노드가 double로 선언해서 정수면 detector가 죽는다. 모델 교체와 무관하지만 어떤 launch에서든 터졌을 버그.)

### 실물 6종 검증 — 3종 실패
실물을 top-down으로 배치·캡처해 격리 테스트하니:
- ✅ banana(0.81)·orange(0.90)·apple(0.76) — 잘 검출
- ❌ **peach** → apple로 오분류 (peach 점수 0.018 = 사실상 0)
- ❌ **tennis_ball** 미검출 (0.017)
- ❌ **mustard_bottle** 미검출 (0.001)

threshold를 0.05까지 낮춰도 안 떴다. **threshold 문제가 아니라 인식 자체 실패**. 같은 프레임에서 3종은 완벽하고 3종은 0점 → 전처리·decode 버그가 아니라 class별 도메인 갭. 패턴이 명확했다: **COCO real이 풍부한 class(apple/orange/banana)는 전이 성공, synthetic 의존 class(peach/tennis/mustard)는 실패.** 3장에서 우려한 리스크가 실제로 터진 것.

### 재배치 진단으로 원인 분리
물체를 벌리고 mustard를 세워서 재테스트하니 원인이 3갈래로 갈렸다:

| 물체 | 원인 | 근거 |
|---|---|---|
| **mustard** | **자세(pose)** — 외형은 학습됨 | 누움 0.001 → 세움 **0.80** |
| **tennis_ball** | 약한 신뢰도 + 가림 (marginal) | 0.004 → 간격 벌리니 0.14~0.26 (0.5 미달) |
| **peach** | **외형 confusion** (제일 어려움) | 재배치 무관하게 apple 0.54. 플라스틱 복숭아가 모델엔 사과로 보임 |

**핵심 통찰**: mustard가 자세만 바꿔 살아난 것은 "모델이 물체를 아예 못 배운 게 아니라, **실제로 나타나는 자세/각도/외형을 synthetic이 충분히 못 담았다**"는 뜻. mustard·tennis는 재렌더로 개선 여지가 크고, peach만 진짜 난제(YCB 015 mesh 외형이 실물과 달라서 같은 mesh 재렌더로도 안 될 수 있음).

### 형상 confound (중요)
이 테스트가 **실제 top-down 배포 형상이 아니었다**(oblique 각도였음). 사용자 확인 결과 실제 배포는 top-down 수직(D10)이 맞다. top-down에선 세운 병이 뚜껑만 보이는 등 결과가 또 달라질 수 있어, **실패가 얼마나 진짜 문제이고 얼마나 형상 탓인지 아직 완전히 분리되지 않았다.**

---

## 10. 현재 상태와 다음 계획

**2026-07-07 일시중단** — 사용자가 Kria 보드 RT 커널 패치를 병행 중이고 카메라가 실 배포 형상 미세팅이라 잠시 멈춤. DPU·카메라 정리 완료(RT 작업 무방해).

**완료(재작업 불필요)**: 학습(Gate2)·양자화(Gate3)·컴파일(Gate4) 전부 통과. xmodel 보드 배치 완료. 보드 추론 정상.

**합의된 재개 방향 (synthetic 재렌더 우선, D2 유지)**:
1. **먼저 진짜 top-down 형상에서 재검증** — 이번 테스트 형상이 D10과 달랐으므로 헛수고 렌더링을 피하기 위해 먼저 확인.
2. 남는 실패만 **synthetic 재렌더 타깃**: 눕힌 병 top-down 뷰, tennis 다양화(조명/clutter), peach 텍스처 대비 + apple hard-negative.
3. 재렌더 → 재학습 → 재양자화 → 재컴파일 → 재검증.
4. peach가 재렌더로도 안 되면 **그때만** D2 재검토(소량 real 학습이미지) — 최종 판단은 사용자.

---

## 11. 결정 로그 요약 (D1–D11)

| # | 결정 | 한 줄 근거 |
|---|---|---|
| D1 | 모델 = YOLOv3-tiny 7-class | class 고정, DPU 지원 op만으로 구성 |
| D2 | 실환경 촬영 이미지는 **학습에 미사용** (검증은 허용) | 수작업 라벨 회피 (사용자 결정) |
| D3 | 학습 데이터 = synthetic + 공개 dataset | D2의 귀결 |
| D4 | mustard 실물 Morehouse (외형 French's=YCB006과 동일) → YCB 소스 유지 | 배포 거리에서 라벨 차이 무시 가능 |
| D5 | 툴체인 = VAI 2.5 docker 고정 | 보드 runtime과 버전 일치 |
| D6 | 프레임워크 = PyTorch (vai_q_pytorch) | 양자화 도구 요구사항 |
| D7 | 전처리 = letterbox 416 (train/calib/worker 통일) | 프레임워크 표준과 일치, 분포 정합 |
| D8 | person = safety class, threshold 낮게(0.3), pickable에서 분리 | recall 우선, 집는 물체 아님 |
| D9 | fast-finetune/QAT 시 학습 subset만 사용 | weight 바뀌는 단계라 D2 준수 |
| D10 | 배포 기하 = 회색 optical table, top-down 0.8m + 실측 색 | 합성 데이터에 반영해 domain gap 감소 |
| D11 | activation SiLU → **Hardswish** 교체 후 재학습 | SiLU는 DPU 미지원, Hardswish가 지원+최근접 |

---

## 부록 — 그 외 해결한 실무 이슈
- **BlenderProc empty annotation**: `enable_segmentation_output()`가 호출 시점에 존재하는 객체만 등록 → object-pool 구조(전부 생성→enable→scene마다 재배치)로 해결.
- **opencv 5.0 vs numpy 충돌**: yolov5 v7.0은 numpy 1.x → `opencv-python==4.10` 고정 (pip_constraints.txt).
- **Arial.ttf HTTP 308**: ultralytics.com 리다이렉트 → 폰트 사전 배치로 우회.
- **Pillow 10 `getsize` 제거**: yolov5 v7.0 플롯 crash → `Pillow==9.5.0` 고정. (학습 결과엔 무관, 플롯만 영향.)
- **sudo bash 시 $HOME→/root**: wrapper가 `SUDO_USER`의 HOME을 쓰도록 방어.

---

## 11. D13 재배포 후 Gate 5 통과, 그리고 apple 재학습 (D14) — 2026-07-09

D13(6-class + YCB real)으로 재학습·재배포한 모델을 실물 top-down에서 검증했다. 결과는 이번 교체의 핵심 목표를 증명했다: 지난번 완전히 실패했던 **mustard가 0.02 → 0.814**로 살아났고, tennis(0.677)·orange(0.850)·banana(0.777)까지 5종이 안정적으로 잡혔다. YCB 벤치마크 real 스캔을 학습에 넣은 D13의 판단이 옳았다.

남은 하나는 apple이었다. 처음엔 apple이 0.462로 잡혀 "threshold만 살짝 낮추면 된다"고 보고 `apple:0.40`을 넣었는데, 사용자가 그 0.462 박스가 **실제로는 복숭아(드롭한 class)** 임을 지적했다. 복숭아는 학습에서 뺐지만 물리적으로는 테이블에 있었고, out-of-distribution이라 가장 가까운 apple로 강하게 오검출되며 **진짜 사과(0.216)를 오히려 눌러버린** 것이다. 복숭아를 치우고 다시 찍자 진짜 사과는 0.549로 올라오고 다른 물체 점수까지 함께 상승했다 — 복숭아가 장면 전체를 갉아먹고 있었던 셈이다. 배포 환경엔 복숭아가 없으니 이 오검출 자체는 문제가 아니지만, **진짜 사과가 0.489~0.549로 0.50 경계에 걸터앉아 flicker할 위험**이 드러났다.

왜 apple만 약한가? 데이터 양의 문제가 아니다(이미 6371 인스턴스로 다른 물체와 동급). 근본은 **color 도메인 갭**(실물 사과가 YCB 사과보다 밝음)과 apple 자체의 난도(둥글어 orange/tennis와 형태 혼동)다. 그래서 "그냥 더 많은 데이터"는 효과가 적고 — 같은 색 데이터를 더 넣는 것뿐 — **배포 카메라로 찍은 실물 사과**가 정확한 처방이다. 고정 station이라 그 카메라·그 조명·그 사과에 맞추는 것이 오히려 정답이다.

그래서 D14: 테이블에 **사과만** 올려두고(단일물체라 auto-label이 사실상 무조건 정답 → 라벨 수작업 0) top-down으로 다양한 위치·회전·조명으로 캡처했다. 캡처는 움직임 중 흐린 프레임을 배제하려 Enter를 누른 순간만 저장하는 수동 모드를 추가했고, 라벨은 DPU 모델의 top-1 박스를 apple로 강제하는 스크립트로 자동화했다. ~54장을 `real_apple_yolo`(train 전량)로 학습에 추가하고 밝기 augmentation(`hsv_v`)만 0.5로 올렸다(`hsv_h`를 키우면 apple↔orange가 혼동되므로 유지). 이 데이터로 재학습을 돌리는 중이다. 재학습 후 apple이 0.7+로 안정되면 임시로 넣었던 threshold hack을 원복하고, 이후 풀 파이프라인(Gate 6)으로 넘어간다.
