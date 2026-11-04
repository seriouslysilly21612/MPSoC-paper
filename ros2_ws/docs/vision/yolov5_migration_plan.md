# YOLOv5 전환 계획 — 주류 비전 모델 교체, YOLOv3-tiny는 legacy 보존

> 작성: 2026-09-08
> 목표: **주류 검출 모델을 YOLOv5 계열로 교체**하고, 현행 YOLOv3-tiny 6-class는
> **삭제하지 않고 legacy로 보존**해 나중에 정량 비교(A/B)에 쓴다.
> 배경 논의: `vision_final.md`(현행 모델 정본), `throughput.md`(성능 예산 정본),
> `yolov3_tiny_execution_plan.md`(게이트 정의 원본).

---

## 0. 한 줄 요약

현행 파이프라인은 YOLOv3-tiny 6-class INT8로 21.7 Hz에서 잘 돈다. 교체의 목적은
"안 되는 걸 되게" 하는 게 아니라 **stride 8 head 확보와 학습 데이터 정리**다.
그리퍼 하드웨어 대기 중인 지금이 이 작업의 적기이며, 실패해도 legacy 모델로
즉시 되돌아갈 수 있게 두 모델을 공존시킨다.

---

## 1. 왜 v5인가, 그리고 왜 v3-tiny를 남기는가

### 1.1 전환 근거 — head 하나가 진짜 이유다

| 근거 | 내용 |
|---|---|
| **stride 8 head** | 0.8 m top-down에서 사과 75 mm ≈ 원본 56 px ≈ letterbox 28 px. tiny의 최소 stride는 16이라 물체 하나가 1.7셀에 걸친다. v5는 stride 8 head가 있어 같은 물체를 3.5셀로 본다. Gate 5에서 마진이 가장 얇았던 tennis_ball이 정확히 이 조건이었다 |
| **툴체인 재사용** | 학습 저장소가 이미 ultralytics yolov5 v7.0이고, 보드 decode 공식도 yolov5식이다. Hardswish cfg 자동 생성(D11)과 Gate 0 inspector도 그대로 쓴다 |
| **op 호환** | v5 v7.0의 구성은 conv / C3의 residual add·concat / SPPF의 maxpool k5 / nearest upsample뿐. tiny와 같은 UG1414 Table 20 집합 안이라 단일 DPU subgraph가 성립할 것으로 본다. **확정은 Gate 0 inspector 1회** |
| **데이터 정리** | BlenderProc synthetic을 학습에서 빼는 재구성(§3)을 어차피 해야 하는데, 재학습이 필요하므로 모델 교체와 한 번에 묶는 게 싸다 |

### 1.2 v5가 고쳐주지 않는 것 — 기대치 정렬

지금까지 이 프로젝트의 검출 실패는 **전부 데이터 도메인 문제**였다. mustard는 자세,
apple은 색, peach는 실물과 mesh 불일치였고 **모델 용량 부족이 원인이었던 적은 한 번도 없다**.
따라서 전환의 성패는 §3 데이터 구성이 결정한다. 아키텍처는 거들 뿐이다.

또한 v5로 바꿔도 **같은 클래스 인스턴스의 identity는 여전히 안 생긴다.**
검출기는 매 프레임을 처음 보는 그림으로 다루고 인스턴스 ID를 내지 않는다.
identity는 `pick_logic`이 공간 연속성으로 근사하며, 그 한계는 그대로 남는다.

### 1.3 legacy 보존 이유

- 현행 모델은 Gate 2~5를 전부 통과하고 실물 6종에서 0.81~0.90으로 검증된 **유일한 기준선**이다.
- 재현 가능한 비교 없이 "새 모델이 낫다"고 말할 근거가 없다. §6이 비교 프로토콜이다.
- 보드 배포는 `9bc6520c`(D14) 하나뿐이고 D13은 데스크톱 `best.pt`에서만 재생성 가능하다.
  **legacy xmodel을 지우면 기준선이 사라진다.**

---

## 2. 모델 크기 선택 — 성능 예산이 먼저다

### 2.1 현재 예산 (측정값, `shm_gate030_20260805`)

| 항목 | 값 |
|---|---|
| 처리시간(= 주기) | 45.5 ms → **21.7 Hz** |
| 그중 `dpu_ms` | 17.7 ms (점유 **약 39%**) |
| 카메라 공급 | 29.25 Hz (소비 22.34 Hz, 여유 1.31배) |
| 후보 수 | 2,535 (26²×3 + 13²×3) |

throughput.md §1의 규칙대로 **주기 = 처리시간**이므로, DPU가 느려지면 그만큼 Hz가 직접 떨어진다.
DPU 점유가 39%라는 것은 "여유가 6할"이 아니라 **처리시간 45.5 ms 안에서 DPU가 17.7 ms를 쓴다**는
뜻이고, 나머지는 전처리·후처리·IPC가 이미 쓰고 있다.

### 2.2 후보 비교

GFLOPs는 공식 640 기준값을 416으로 환산((416/640)² = 0.4225). mAP는 COCO 640 기준 참고치다.

| 모델 | GFLOPs @416 | COCO mAP50-95 | head | 후보 수 @416 | 판정 |
|---|---|---|---|---|---|
| YOLOv3-tiny (현행) | 5.6 | 17.6 | 2 (16/32) | 2,535 | 기준선 |
| **YOLOv5n** | **1.9** | **28.0** | **3 (8/16/32)** | **10,647** | **1순위** |
| YOLOv5s | 7.0 | 37.4 | 3 | 10,647 | 예산 확인 후 |
| YOLOv5m | 20.7 | 45.4 | 3 | 10,647 | 배제 |

### 2.3 왜 v5n이 1순위인가

**v5n은 현행 tiny보다 연산량이 3분의 1인데 정확도는 더 높고, stride 8 head까지 얻는다.**
DPU 시간이 GFLOPs에 대략 비례한다면 17.7 → 약 6 ms이고, 층 수와 메모리 트래픽 오버헤드를
감안해도 8~12 ms 수준으로 본다.

대신 **후보 수가 4.2배**로 늘어 후처리가 비싸진다. 현행 `post_ms` 4.78 ms가 8~12 ms로
오를 수 있다. objectness 사전필터가 대부분을 쳐내므로 선형 증가는 아니지만, decode 대상
배열 자체가 커진다.

거칠게 합치면 v5n은 **DPU에서 번 시간을 후처리에서 도로 쓰는 대략 중립**이고, 그 대가로
정확도와 작은 물체 해상도를 얻는다. v5s는 DPU가 25~35 ms로 추정되어 처리시간이
58~70 ms(14~17 Hz)로 **명백한 회귀**다.

> **주의**: 위 DPU 추정치는 계산이지 측정이 아니다. Gate 4 직후 보드에서 `dpu_ms`를
> 실측하기 전에는 크기를 확정하지 않는다. 실측이 v5s도 여유 있다고 말하면 올린다.

### 2.4 클래스 수 결정 — 5-class 권고

person은 2026-07-27부터 worker threshold 2.0으로 꺼져 있고, `person_guard`도 비활성이며,
2026-09 앱 전환으로 파이프라인에서 완전히 빠졌다. 재학습하는 김에 **person을 빼고 5-class**로
정리하는 것이 단순하다.

영향 범위(전부 같이 고쳐야 함):

- `yolo_v3_tiny_training/config/classes.yaml` — id 재번호
- 새 모델의 `decode_meta.json` — `names`, `num_classes` (자동 생성됨)
- `pick_logic.yaml` `allowed_classes` — person이 원래 없으므로 무변경
- worker `CLASS_THRESHOLDS_BY_NAME` — person 항목 삭제 가능

안전 래치(`person_guard`)를 되살릴 계획이 생기면 6-class로 되돌려야 한다. **지금은 계획 없음.**

---

## 3. 학습 데이터 구성

### 3.1 원칙 세 줄

1. **BlenderProc synthetic 전면 제외.** synthetic-val mAP 0.993이었지만 synthetic 의존 3종은
   실물에서 0.001~0.018로 전멸했다. 살려낸 것은 전부 real 데이터였다.
2. **BOP ycbv, Open Images 제외.** ycbv는 Gate 2에서 mustard→banana 혼동(0.39)을 만든 소스다.
   어수선한 실내 비디오라 고정 station에 기여가 적다.
3. **배포 카메라 실촬영을 apple 하나에서 5종 전부로 확대.** D14가 apple을 0.55 → 0.88로
   올린 그 방법을 나머지에도 적용한다.

### 3.2 물체별 구성

| 물체 | 배포 실촬영 (단일물체, auto-label) | YCB Berkeley 실스캔 | COCO | 비고 |
|---|---|---|---|---|
| apple | **40~50장, 주력** | 013, N3~N5 stride 2 | ≤1,000장 | YCB 사과가 실물보다 어두워 보조 역할. `hsv_v` 0.5 유지 |
| orange | 40~50장 | 017 | ≤1,000장 | `hsv_h`는 낮게 유지(apple 혼동 방지) |
| banana | 40~50장, 놓인 방향 다양하게 | 011 | ≤1,000장 | **ycbv 제외** |
| tennis_ball | 40~50장, 테이블 가장자리 위치 포함 | 056 | 없음 | 가장 작은 물체. COCO "sports ball"은 부적합 |
| mustard_bottle | 40~50장, **누운 자세 필수** | 006 (N5 = top-down) | 없음 | 누운 자세가 Gate 5 실패 원인이었다 |
| ~~person~~ | — | — | — | §2.4에 따라 드롭 |

### 3.3 공통 세트 (신설)

| 세트 | 양 | 목적 |
|---|---|---|
| 다중물체 장면 (5종 동시, 간격·가림 변주) | 30~50장 | 실제 접근 장면. tennis 실패가 가림·간격이었다. 현행 DPU 모델로 사전라벨 후 검수 |
| 배경 negative (빈 테이블, 케이블, 그림자, 손) | 30~50장, 라벨 없음 | FP 억제. 케이블·그림자가 conf 0.32~0.41 person으로 36% 프레임 오검출된 기록이 있다 |

### 3.4 split과 가중치

- **실촬영 일부를 val로 남긴다.** D14는 전량 train이라 배포 도메인 val 지표가 없었다.
  촬영 세션 단위로 20%를 val로 뗀다. 그러면 Gate 2 수치가 처음으로 Gate 5를 예측하게 된다.
- **실촬영을 오버샘플링한다.** 실촬영 250장이 YCB·COCO 수천 장에 묻히면 D14 효과가 희석된다.
  train 목록에 실촬영 폴더를 여러 번 등록하는 방식이 가장 단순하다.
- val split은 기존 규칙대로 **scene 단위**로 나눈다(연속 프레임이 train/val에 갈리면 안 됨).

---

## 4. 게이트 — 기존 정의 재사용

| Gate | 내용 | 통과 기준 |
|---|---|---|
| 0 | 학습 전 Inspector로 DPU 매핑 선검증 | "리포트에 CPU 배정 없음" |
| 1 | 데이터 조립 + contact-sheet 육안 검수 | 클래스별 목표 수량 충족 |
| 2 | 학습 | mAP@0.5가 현행 0.748 이상. **배포 도메인 val을 따로 볼 것** |
| 3 | 양자화 (INT8 PTQ, calib 500장) | head별 cosine > 0.95 |
| 4 | 컴파일 | `DPU subgraph number 1` |
| 4.5 | **보드 `dpu_ms` 실측 (신설)** | §2.3의 크기 확정 판단. 여기서 처리시간 예산 초과면 v5n으로 내린다 |
| 5 | 실물 top-down 5종 검출 | 전 클래스 0.7 이상 |
| 6 | 파이프라인 통합 | 기존 노드 무수정으로 동작 |
| 7 | 라이브 성능 | Hz·E2E·CPU가 현행 대비 회귀 없음 |

Gate 4.5를 새로 넣은 이유는, 지금까지 성능 예산을 Gate 6까지 안 봤는데 v5는 **크기 선택
자체가 예산에 달린** 첫 사례이기 때문이다.

---

## 5. 파이프라인 통합 — legacy 공존을 위한 코드 변경 3건

현재 구조로는 두 모델을 나란히 둘 수 없다. 아래 셋을 먼저 고쳐야 한다.

### 5.1 `decode_meta.json`이 모델 파일 옆에서 해석된다

worker는 `--meta`가 없으면 **모델과 같은 디렉터리**의 `decode_meta.json`을 쓴다
(`vitis_ai_worker_yolo.py` `main()`). 지금처럼 `models/` 하나에 두 모델을 넣으면
meta 파일이 충돌한다.

**처방**: 모델별 하위 디렉터리로 분리한다.

```
src/vitis_ai_detector_pkg/models/
  arch_b3136.json
  yolov3_tiny_6class/     ← legacy
      model.xmodel
      decode_meta.json
  yolov5n_5class/         ← 주류
      model.xmodel
      decode_meta.json
```

덤으로 현행 파일명 `yolov3_tiny_7class.xmodel`이 **실제로는 6-class D14 모델**이라는
이름-실체 불일치도 이때 해소된다.

### 5.2 launch가 모델 경로를 하드코딩하고 yaml을 덮어쓴다

`pick_place_vitis_ai.launch.py`가 `yolov3_tiny_7class.xmodel`을 `FindPackageShare`로
주입하는데, `parameters=[config, {'model_path': model_path}]` 순서라 **launch가 yaml을 이긴다.**

> ⚠️ `vitis_ai_detector.yaml`의 주석은 "여기에 `model_path`를 적으면 launch 주입을 덮어쓴다"고
> 적혀 있는데 **사실과 반대다.** 이 주석도 같이 고칠 것.

**처방**: launch 인자로 모델을 고르게 한다(`ros2 launch ... vision_model:=yolov5n_5class`).
기본값을 새 모델로 두고, legacy는 인자 하나로 되돌아갈 수 있게 한다.

### 5.3 worker는 head 3개를 이미 받는다 — 수정 불필요

`load_model()`이 `decode_meta`의 `strides` 목록으로 head를 매칭하고, 출력 텐서의
spatial 크기에서 stride를 역산해 인덱스를 맞춘다. head가 2개든 3개든 코드 변경이 없다.
**fixpos도 텐서별로 읽으므로** head마다 달라도 된다(현행도 head[0]=2, head[1]=3으로 다르다).

하류(`pick_logic`, `pick_target_3d`, `pick_target_base`)는 model-agnostic JSON contract라
전부 무영향이다.

---

## 6. 비교 실험 프로토콜 (legacy가 존재하는 이유)

같은 장면·같은 조건에서 두 모델을 번갈아 돌린다. 모델 외 변수를 고정하는 것이 전부다.

| 축 | 측정 | 방법 |
|---|---|---|
| 정확도 | 클래스별 confidence, 미검출률 | 동일한 top-down 정지 장면 세트, 물체 위치 고정 |
| 작은 물체 | tennis_ball 검출 안정성 | stride 8 head의 효과를 보는 핵심 지표 |
| 성능 | `dpu_ms` / `pre_ms` / `post_ms` / 처리시간 / Hz | 기존 metrics CSV 그대로 |
| 자원 | 노드별 CPU, CMA | 기존 resource 측정 그대로 |
| 추종 품질 | 접근 오차, 추종 중 타깃 상실 빈도 | 로봇 트랙과 합쳐서 |

⚠️ 실측 전 보드를 조용하게 만들 것. IDE에서 C++ 파일을 여는 것만으로 cpptools 인덱싱이
CPU를 150% 잡아 측정이 무효가 된 전례가 있다.

---

## 7. 함정 목록

| # | 함정 | 대응 |
|---|---|---|
| 1 | **autoanchor** — yolov5는 학습 중 앵커를 재계산할 수 있다 | `decode_meta.json`의 앵커는 반드시 학습된 `best.pt`의 Detect 모듈에서 읽는다. `12_quantize.py`가 이미 그렇게 한다(`detect.anchors[i] * detect.stride[i]`). yaml 기본값을 손으로 적지 말 것 |
| 2 | **letterbox 3중 일치** | train / quantize calibration / board worker 세 곳이 전부 letterbox 416 + pad 114여야 한다 |
| 3 | **head별 fixpos** | 두 head가 fix 2·3으로 다르듯, 3 head도 각각 다를 수 있다. 텐서에서 읽으므로 문제없지만 문서에 "전부 N"이라고 적지 말 것 |
| 4 | **Hardswish는 conv에 fuse되지 않는다** | 배포된 xmodel 실측 결과 conv2d-fix 13개 전부 `nonlinear=NONE`이고 hard-sigmoid-fix + eltwise-fix 쌍으로 풀려 있다. v5는 C3 때문에 activation 수가 더 많으니 op 수 증가를 Gate 0에서 확인 |
| 5 | **VAI 2.5 hardswish 배포 버그 2건** | docker `--rm`이라 매 실행 sed 패치가 필요하다. 기존 wrapper가 이미 처리 |
| 6 | **죽은 앵커** | 현행 stride-32 앵커 344×319는 우리 작업거리에서 도달 불가다. v5 앵커도 학습 후 실제 물체 크기 대비 도달 가능한지 확인할 것 |
| 7 | **fingerprint 고정** | `arch_b3136.json`(`0x101000016010406`) 타깃 유지. 툴체인은 VAI 2.5 고정 |
| 8 | **데스크톱 GPU 드라이버** | `nvidia-driver-*-open`은 CUDA 부하에서 무로그 hard-freeze한다. proprietary 580 + `apt-mark hold` 유지 |

---

## 8. 작업 순서

```
[데이터]
 1. 배포 카메라로 5종 단일물체 촬영 (물체당 40~50장, --manual 모드)
 2. autolabel_single_object.py로 라벨 자동화
 3. 다중물체 장면 30~50장 촬영 → 현행 DPU 모델로 사전라벨 → 검수
 4. 배경 negative 30~50장 (라벨 없음)
 5. YCB Berkeley 실스캔 5종 재사용 (기존 08 스크립트)
 6. COCO 3종 ≤1,000장씩
 7. classes.yaml 5-class 재번호 → 06_merge_split.py (실촬영 오버샘플링, 세션 단위 val 20%)

[모델]
 8. yolov5n cfg + Hardswish → Gate 0 inspector           ← 여기서 op 호환 확정
 9. 학습 (Gate 2)
10. 양자화 (Gate 3) → 컴파일 (Gate 4)

[통합]
11. models/ 하위 디렉터리 분리 + launch 인자화 + yaml 주석 정정  (§5)
12. 보드 배치 → dpu_ms 실측 (Gate 4.5)                    ← 여기서 v5s 승격 여부 결정
13. 실물 검증 (Gate 5) → 파이프라인 (Gate 6) → 라이브 (Gate 7)

[비교]
14. legacy와 A/B (§6) → 결과를 vision_final.md에 기록
```

**되돌리기**: 어느 단계에서 막히든 launch 인자를 legacy로 바꾸면 즉시 현행 구성으로 복귀한다.
그것이 legacy를 남기는 실질적 이유다.

---

## 9. 이 문서의 위치

- 현행 모델의 정본은 `vision_final.md`다. 이 계획이 완료되면 그쪽에 결과를 합치고
  이 문서는 계획 이력으로 남긴다.
- 성능 예산의 정본은 `throughput.md`다. §2의 수치가 그쪽과 어긋나면 `throughput.md`가 맞다.
- 게이트 정의의 원본은 `yolov3_tiny_execution_plan.md`다.
