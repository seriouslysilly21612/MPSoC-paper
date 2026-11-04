# YOLOv3-tiny 7-Class 교체 실행 계획 (확정본)

- 작성: 2026-07-06
- 상위 방향 문서: `yolov3_tiny_plan.md` (모델/클래스/마일스톤 정의)
- 기술 근거: **UG1414 v2.5** (`~/ug1414-vitis-ai.pdf`, 2022-06-15판 — 보드 runtime과 동일 버전), 현 시스템 구조는 `workflow.md`
- 상태: Phase 0 착수 전. Open item 2개(§8) 남음.

---

## 1. 확정된 결정 (Decision Log)

| # | 결정 | 근거 / 비고 |
|---|---|---|
| D1 | 모델 = **YOLOv3-tiny 7-class**. class id 고정: `0 apple, 1 peach, 2 orange, 3 banana, 4 tennis_ball, 5 mustard_bottle, 6 person` | `yolov3_tiny_plan.md` |
| D2 | **실환경(D435i) 촬영 이미지는 학습(weight 업데이트)에 사용하지 않는다.** 검증·평가(추론)에는 사용 가능 | 사용자 결정 2026-07-06 |
| D3 | 학습 데이터 = **synthetic(YCB CAD mesh 렌더링) + 공개 dataset**(COCO / Open Images / YCB-Video) | D2의 귀결 |
| D4 | 물리 객체: 과일 4종 = **플라스틱(YCB replica)**, tennis ball · mustard bottle = **실물**. mustard 실물은 **Morehouse 제품**(외형이 French's=YCB 006과 거의 동일) → YCB 소스 유지, 라벨 차이는 배포 거리에서 수 px 수준이라 무시. Phase 5/7에서 실물로 검증 | 사용자 확인 2026-07-06 |
| D5 | 툴체인 = **Vitis-AI 2.5 docker 고정** (보드 runtime 2.5.0 / DPUCZDX8G_ISA1_B3136 / fingerprint `0x101000016010406`) | 버전 skew 방지 |
| D6 | 프레임워크 = **PyTorch (vai_q_pytorch)**. Caffe는 2.5부터 deprecated | UG1414 p.4, p.86 |
| D7 | 전처리 = **letterbox → 416×416** (비율 유지 resize + 상하 padding). 학습·quantize calib·worker 3곳 동일 적용. (2026-07-07 변경: 원래 "단순 resize"였으나, 학습 프레임워크 yolov5의 표준 전처리가 letterbox라 이를 전 구간에 채택 — 프레임워크 내부와 싸우지 않고 train/deploy 일치 확보. worker에는 pad 역변환 1줄 추가될 뿐) | 전처리 일관성 (UG1414 p.89) |
| D8 | person = safety class. threshold 낮게(0.25~0.4, recall 우선), `pickable_classes=[0..5]` / `safety_classes=[6]` 분리. **2026-09 폐기**: 앱이 추종/접근으로 전환되며 pick_logic person guard·classes.yaml pickable/safety 키 제거. 모델은 6-class 유지, worker threshold 2.0으로 person 미발행 | `yolov3_tiny_plan.md` §7 |
| D9 | quantize 시 fast-finetune/QAT를 쓰게 되더라도 **학습 dataset subset만 사용** (weight가 바뀌는 단계이므로 D2 준수) | UG1414 p.92 |
| D10 | 배포 기하: **optical table(회색)** 위 물체, 카메라는 테이블 정중앙 상부 **~0.8 m**에서 **수직 하방(top-down) 고정**. 테이블 실측 표면색 sRGB: 어두움(110,110,108)/평균(156,155,151)/밝음(190,190,185) → linear 0.156~0.515. synthetic 카메라 샘플링(elev 55~90°, 0.55~1.05 m, 테이블 표면 50% 회색 금속)이 이를 따름 | 사용자 제공 2026-07-06 |
| D11 | **activation: SiLU → `nn.Hardswish()` 교체 후 재학습**. yolov5 v7.0의 Conv 기본 activation이 SiLU라 1차 학습 모델이 DPU 미지원 op(`aten::silu_`)로 export됨(Gate 3 시도1). Hardswish는 UG1414 v2.5에서 quantizer(PyTorch `Hardswish`→XIR `hardswish`)·compiler 모두 지원이고 (⚠️ 2026-09-08 정정: 배포 xmodel은 conv에 **융합하지 않는다** — 별도 op 쌍으로 풀린다) SiLU와 형태가 가장 가까워 pretrained 전이 손실 최소. 사용자 제시 논문("Efficient SAR Vessel Detection for FPGA-Based On-Satellite Sensing")과 동일 해법. fallback = LeakyReLU(0.1) (역시 지원). 재학습 전 Inspector 선검증(12a, Gate 0) 필수 | 2026-07-07 |

---

## 2. 시스템 통합 범위 (우리 코드에서 바뀌는 것)

JSON contract 덕분에 교체 범위가 좁다:

| 구분 | 파일 | 내용 |
|---|---|---|
| 변경 | `vitis_ai_detector_pkg/vitis_ai_worker.py` | MODEL_W/H=416, LUT 값(`x/255×input_scale`), SSD prior/decode → YOLO grid decode + NMS, objectness pre-filter, 7-class threshold dict |
| 변경 | `system_bringup_pkg/config/vitis_ai_detector.yaml` | `model_path`만 새 xmodel로 |
| 변경 | `pick_logic_pkg` + config | pickable/safety class 분리 |
| 불변 | detector node | worker handshake에서 input size 자동 수신 (`vitis_ai_detector_node.py:466`) |
| 불변 | 3D(single-point reverse projection), TF, camera 설정, pipelining, metrics CSV | — |

---

## 3. Class별 학습 데이터 소스

| class | 주력 | 보조 | 비고 |
|---|---|---|---|
| mustard_bottle | synthetic + **YCB-Video** (real) | — | YCB 006 mesh. 실물은 Morehouse(외형 거의 동일) — 소스 유지, Phase 5/7 실물 검증 |
| banana | synthetic + YCB-Video + COCO | — | YCB 011 |
| apple | synthetic + COCO | Open Images | YCB 013. 플라스틱 replica라 synthetic이 주력 |
| peach | **synthetic (사실상 유일한 신뢰 소스)** | Open Images 소량 | YCB 015. 혼동 pair 핵심 |
| orange | synthetic + COCO | Open Images | YCB 017 |
| tennis_ball | synthetic + Open Images "Tennis ball" | — | YCB 056. COCO "sports ball"은 일반 공이라 미사용 |
| person | **COCO** (상한 두고 sampling) | — | 손/팔 부분 노출 이미지 포함 |

공통 규칙:
1. 공개 데이터 class id → 우리 7-class로 **remap**
2. tight bbox 스타일 통일 (Open Images 느슨한 라벨은 필터/소량만)
3. class **balance 상한** (COCO person이 수량으로 압도하지 않게)
4. **scene 단위 train/val split** (연속/유사 frame 분리 금지)
5. val = held-out synthetic + YCB-Video frame (+ 선택: 소규모 실환경 val set — D2상 평가 용도는 허용)

---

## 4. Phase별 실행 계획

### Phase 0 — 환경 고정 + op 사전검사
- [x] Host PC 사양 확정 (2026-07-06 조사): **Ubuntu 22.04.5 LTS x86_64, RTX 4060 8GB** (driver 595.71.05 / CUDA 13.2), RAM 16GB, 디스크 여유 771GB, Docker 미설치
- 실행 경로 확정: **학습 = 데스크톱 native venv(PyTorch CUDA)** · **quantize/compile = 데스크톱 Vitis-AI 2.5 CPU docker** (tiny 모델 PTQ는 CPU로 충분, 수 분 단위). GPU docker는 dockerhub에 없어 로컬 빌드가 필요하므로 **QAT escalation 시에만** 빌드.
- [ ] Docker 설치: `sudo apt install -y docker.io && sudo usermod -aG docker $USER` (재로그인 필요)
- [ ] `docker pull xilinx/vitis-ai-cpu:2.5.0.1260` — Docker Hub 태그 실존 확인 완료(`2.5`/`2.5.0`도 동일 이미지). **주의**: `vitis-ai-pytorch-cpu` 분리 이미지는 3.x부터이며 2.5에는 없음. 2.5는 통합 CPU 이미지 안에서 `conda activate vitis-ai-pytorch` (UG1414 p.86)
- [ ] `arch_b3136.json`을 host로 복사 (원본: `~/vitis_ai_work/arch/arch_b3136.json`)
- [ ] BlenderProc 2.x 설치 + YCB CAD mesh 6종 다운로드 (006, 011, 013, 015, 017, 056)
- [ ] **Inspector로 float model 뼈대 사전검사** (UG1414 p.88, p.106):

```python
from pytorch_nndct.apis import Inspector
inspector = Inspector("0x101000016010406")          # B3136 fingerprint
inspector.inspect(model, torch.randn([1, 3, 416, 416]))
# → quantize_result/inspect_*.txt 에서 CPU로 배정된 op이 없는지 확인
```

- 참고: vai_q_pytorch는 **PyTorch 1.2~1.10.2**만 지원(UG1414 p.86). 학습은 최신 torch로 해도 되지만 **weight는 state_dict로 저장**하고, quantize는 2.5 docker의 torch에서 모델 정의를 재구성해 로드한다.
- **Gate 0**: inspect 리포트에 CPU subgraph 0개.
- op 호환성 근거 (UG1414 Table 20, DPUCZDX8G열): conv2d activation에 **LeakyReLU 지원**, max-pooling kernel/stride 범위 충분, **resize NEAREST**(upsample) 지원, concat 지원 → tiny-yolov3 전 레이어 DPU 배치 가능.

### Phase 1 — Dataset 조립 (촬영 없음, 전부 스크립트)
1. **Synthetic**: BlenderProc로 6개 mesh 렌더링. 테이블 위 1~5개 물체 + distractor, 랜덤 배경/조명, 카메라는 **배포 기하(D10) 중심 randomize** — elevation 55~90°(90=수직 하방), 거리 0.55~1.05 m, look-at은 테이블 중앙 부근(물체가 화면 가장자리에 걸리는 상황 포함), 테이블 표면 50%는 회색 금속(optical table 유사). 부분 가림 포함. 목표 **class당 2,000~5,000 인스턴스**, COCO format 자동 annotation → YOLO txt 변환.
2. **YCB-Video 추출**: mustard_bottle(006)/banana(011) 등장 frame → bbox 변환 + remap.
3. **COCO**: person subset (상한 5–10k) + apple/orange/banana 소량.
4. **Open Images**: peach / tennis_ball 소량 (라벨 품질 필터 후).
5. 병합 → balance → split. annotation **contact-sheet 자동 생성**으로 육안 검수.
- **Gate 1**: class별 목표 수량 충족 + contact-sheet 검수 통과.

### Phase 2 — Training (float, host PC)
- COCO pretrained YOLOv3-tiny에서 fine-tuning. `classes=7`, head filter `3×(7+5)=36`. (yolov5 repo v7.0의 `models/hub/yolov3-tiny.yaml` 사용 — 고전 anchor head. ultralytics v8의 "yolov3-tinyu"는 anchor-free head라 DPU decode 계획과 안 맞아 미사용) **activation은 커스텀 cfg `models/yolov3-tiny-hswish.yaml`(hub yaml + `activation: nn.Hardswish()`)로 교체 — D11. 11_train.sh가 cfg 자동 생성·검증.**
- 전처리 = letterbox 416×416 (D7).
- **Augmentation 최대 적용** (HSV jitter, blur/noise, mosaic, perspective) — 실환경을 못 보므로 domain gap 완충재.
- 지표: mAP@0.5, class별 recall/precision, **person false-negative rate** 별도 추적.
- **Gate 2**: val 전 class 검출 + 혼동 pair(apple↔peach↔orange, tennis_ball↔orange) 오류율 확인 + synthetic-학습 class가 real 이미지(YCB-V/OI)에서도 잡히는지 cross-domain 확인.
- **Gate 2 결과 (2026-07-07) — PASS**: run `pickplace_v3tiny2` (150 epoch, 2.06 h, RTX 4060).
  val(2,093장 — ycbv/COCO/OI real 이미지 다수 포함) mAP@0.5:
  all **0.758** / apple 0.627 / peach 0.943 / orange 0.734 / banana 0.694 / tennis_ball 0.980 / mustard_bottle 0.819 / person 0.509.
  - cross-domain OK: mustard(0.819)·banana(0.694)의 val instance 대부분이 real ycbv frame → synthetic-학습 class가 real에서 검출됨. peach/tennis_ball은 OI real 사진 포함 val에서 0.94+.
  - 기록해 둘 약점: ① apple 최저(0.627) — COCO 과일더미 + orange 혼동 기인, 배포 도메인(단일 플라스틱 사과·top-down)과 거리 있음 → Phase 5 실물 스냅샷에서 재확인. ② person 0.509 (mAP50-95 0.215) — COCO 군중/소형 person 기인; 안전 트리거 용도는 thr 0.3(D8) + Phase 7 live에서 최종 판정. ③ mustard R@maxF1 0.459 (P 0.987) — ycbv 가림 프레임 miss, 우리 환경은 무가림.
  - 학습 종료 시 `FreeTypeFont.getsize` AttributeError는 Pillow 10 플롯 이슈(지표·weight 무관, cosmetic) → `pip_constraints.txt`에 `Pillow==9.5.0` 핀 추가로 재발 방지.
  - **소스별 진단 (2026-07-07)**: 전체 confusion matrix에서 유일한 class 혼동이 mustard→banana 0.39이라 val을 소스별로 분리 평가.
    synth-val(640장): mAP50 **0.993**, mustard 대각 0.98/banana 칸 0.01 → 배포 유사 도메인에서는 혼동 없음.
    ycbv-val(384장): banana P 0.434(잡동사니에 banana FP 다수 — 0.39의 원인), banana R 0.911, mustard mAP50 0.779.
    → 혼동은 ycbv 가림/클러터 한정. 단 synth 0.993은 in-distribution 수치이므로 실물 전이는 Phase 5 스냅샷에서 확인(1순위: 실제 Morehouse 병이 banana로 찍히는지).
- **재학습(Hardswish, D11) Gate 2 재판정 (2026-07-07) — PASS**: run `pickplace_v3tiny_hswish` (150 epoch, 2.07 h). all mAP50 **0.766** (SiLU 1차 0.758 대비 +0.008 → activation 교체 비용 없음). banana 0.694→0.738 (P 0.721→0.804, ycbv FP 감소) 개선, 나머지 class ±0.01 이내 동일. **이 weight로 Phase 3 진행.**

### Phase 3 — Quantize (VAI 2.5 docker, UG1414 pp.86–107)
- 모델 정의를 **forward-only로 개조** (pre/post-processing은 클래스 밖으로), `torch.jit.trace` 통과 필수 (p.89).
- calibration 이미지 = **학습 dataset subset 100~1000장** (p.90; D9).

```bash
# 1) calibration
python yolov3_tiny_quant.py --quant_mode calib --subset_len 500
# 2) quantized 정확도 검증 (val 전체)
python yolov3_tiny_quant.py --quant_mode test
# 3) 배포용 xmodel export (batch=1 필수, p.90/p.102)
python yolov3_tiny_quant.py --quant_mode test --subset_len 1 --batch_size 1 --deploy
#    → quantize_result/<Model>_int.xmodel
```

스크립트 내부 API (p.89, p.105):

```python
from pytorch_nndct.apis import torch_quantizer
quantizer = torch_quantizer(quant_mode, model, (torch.randn([1, 3, 416, 416])),
                            device=device, bitwidth=8)
quant_model = quantizer.quant_model
# ... calib/val 이미지 forward ...
if quant_mode == 'calib':
    quantizer.export_quant_config()
if deploy:
    quantizer.export_xmodel(deploy_check=False)
```

- 정확도 손실 크면 escalation (p.92, p.99): **fast finetune(AdaQuant)** → **QAT**. 두 경우 모두 학습 subset만 사용(D9).
- **Gate 3**: quantized mAP 손실 ≤ 3%p, person recall 유지.
- **Gate 3 시도 1 (2026-07-07) — 구조 FAIL**: cosine head[0] 0.9931 / head[1] 0.9929로 수치는 통과권이었으나, `aten::silu_`가 quantizer에서 **float op로 잔류**(`VAIQ_WARN`) + XIR unknown-op 경고 → 컴파일 시 conv마다 CPU subgraph 파편화 확정. 원인·해결 = D11(Hardswish 재학습). 교훈: **cosine 수치보다 `VAIQ_WARN`/unknown-op 경고를 먼저 볼 것**. 재발 방지 = 12a_inspect_docker.sh(Gate 0)를 학습 전에 실행.
- **VAI 2.5 배포 버그 + 패치 (2026-07-07)**: Hardswish 모델 inspect/quantize 시 `NameError: FixNeuronWithBackward` — `pytorch_nndct/nn/modules/hardswish.py`의 `__init__`이 미정의 심볼을 초기화(사용처 없는 dead line). GitHub v2.5/v3.0 소스 대조로 확인: **v3.0은 해당 줄 삭제로 수정**, forward의 DPU식 hswish 근사(`relu6(x+3)×2731/16384` + fake-quant)는 2.5에 정상 구현됨. 대응: 12a/12 wrapper가 컨테이너 기동 직후 sed로 해당 줄을 `None` 치환(`--rm` 컨테이너라 매 실행 자동 적용). Gate 0 시도 1에서 cfg 생성·activation 검증·fingerprint 인식은 모두 정상 확인됨.
- **Gate 0 통과 (2026-07-07)**: 패치 후 Inspector 판정 = **"All the operators are assigned to the DPU"** (hswish cfg, target DPUCZDX8G_ISA1_B3136). Hardswish 구조 확정 → 재학습 진행.
- **VAI 2.5 배포 버그 ② (2026-07-07, calib에서 발견)**: `hardswish.py` forward의 `fake_quantize_per_tensor(...)` 호출에 필수 인자 `method`/`inplace` 누락(TypeError). 2.5 `fix_ops.py`의 실제 시그니처 확인 후 패키지 기본 rounding(`method=2`) + `inplace=False`로 보완하는 sed를 12/12a wrapper에 추가. 겸사겸사 12 wrapper가 시작 시 `quantize_result/`를 비워 SiLU 1차 잔재(quant_info/bias_corr)와의 혼입 차단. **한도 설정: hswish 경로에서 추가 함정이 또 나오면 LeakyReLU(0.1)로 즉시 전환.**
- **Gate 3 시도 2 (2026-07-07) — PASS**: hswish weight, 패치 후 calib 500장 완주. cosine **head[0] 0.9923 / head[1] 0.9849** (DPU hswish 근사 시뮬레이션 포함 측정), `VAIQ_WARN`/unknown-op **0건**, `DeployModel_int.xmodel` + `decode_meta.json` export 완료. head[1]이 0.99를 소폭 하회하나 본 기준(mAP 손실 ≤3%p)의 프록시로는 통과권 — 검출 수준 최종 확인은 Gate 5(보드에서 host float와 검출 비교)에서 수행, 미달 시 fast-finetune(D9: 학습 subset만 사용) 카드 유지.

### Phase 4 — Compile (UG1414 p.112)

```bash
vai_c_xir \
  -x quantize_result/yolov3_tiny_int.xmodel \
  -a arch_b3136.json \
  -o compiled_yolov3_tiny \
  -n yolov3_tiny_7class
```

- 컴파일 로그에서 **DPU subgraph 1개** 확인 (2개↑ = CPU op 혼입 → Inspector로 원인 추적).
- 보드에서 `xdputil xmodel -l yolov3_tiny_7class.xmodel`: input 416×416×3, output 2개(13×13×36, 26×26×36), **fixpoint scale 기록** → worker LUT/decode에 사용.
- **Gate 4**: 단일 DPU subgraph + 보드 load 성공.

### Phase 5 — 보드 정지 이미지 검증 (camera 앞서 수행)
- `~/vitis_ai_work/scripts/ssd_adas_image_test.py`를 템플릿으로 `yolov3_tiny_image_test.py` 작성: VART load → 정지 이미지 추론 → decode+NMS → bbox 시각화 저장.
- 테스트 이미지: 기존 `~/vitis_ai_work/realsense_frames/` + 실물 배치 스냅샷 (D2상 평가 용도 허용) → **domain gap 최초 관측 지점**.
- **Gate 5**: host float 결과와 검출 일치 + 실환경 스냅샷에서 전 class 검출.

### Phase 6 — Worker 교체 (보드) — 코드 준비 완료 2026-07-07, 전환만 남음
- **신규 worker `vitis_ai_worker_yolo.py` 작성 완료** (SSD worker는 보존 — config 전환식 교체):
  - letterbox+LUT 전처리 — float 계산과 **bit-identical 검증 완료**
  - `decode_meta.json`(xmodel 옆 자동 탐색) 기반 yolov5식 grid decode — 단위테스트 통과
  - objectness **int8-domain pre-filter** (dequantize 전에 컷, SSD pre-filter와 동일 아이디어)
  - letterbox 역변환 → source 해상도 좌표 (기존 JSON contract 완전 호환)
- Phase 5 스크립트도 준비 완료: `~/vitis_ai_work/scripts/yolov3_tiny_image_test.py`
- **전환 체크리스트** (모델 도착 + Phase 5 통과 후 실행):
  - [ ] `vitis_ai_detector.yaml`: `model_path` → `~/vitis_ai_work/models/yolov3_tiny_7class.xmodel`, `worker_script_path` → `vitis_ai_worker_yolo.py`, **`send_resized_input: false`** (letterbox는 worker가 수행 — node의 plain resize는 비율을 깨므로 필수)
  - ~~`pick_logic.yaml`: `allowed_classes` → 6개 pickable 이름 (person 제외 = D8 분리. person은 `/detections`에 계속 흐름 → 향후 safety 로직용)~~ — 2026-09 폐기: person 로직 제거, allowed 5종
  - [ ] 재빌드 불필요 (worker egg-link, config symlink)
- **Gate 6**: `ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py` → `/pick_target_base`까지 정상.

### Phase 7 — Live 검증 = 최종 acceptance gate (실환경 val이 없으므로 역할 격상)
체크리스트:
- [ ] 6개 물체 각각: 배포 기하(0.8 m top-down, D10)에서 화면 위치별(중앙/가장자리) 검출, confidence 분포, bbox 안정성
- [ ] 혼동 pair 실측: apple↔peach↔orange, tennis_ball↔orange
- ~~person 진입 검출 (손·팔만 진입 포함)~~ — 2026-09 폐기(person 로직 제거)
- [ ] threshold tuning (worker config만 — 재학습 아님)
- [ ] 기존 metrics CSV(`metrics_csv_path` + `metrics_duration_sec`)로 60초 측정 → SSD 대비 pre/dpu/post/worker_ms 비교, throughput ceiling(camera ~20 Hz) 재확인

---

## 5. Domain gap contingency (실패 시 대응 순서)

1. **class threshold 조정** — worker config만.
2. **synthetic 재생성 + 재학습** — 관측된 실패 모드(조명/각도/배경)를 렌더링 파라미터에 반영. 촬영이 아니므로 D2 준수.
3. 그래도 부족하면 → **"소량 실촬영 fine-tune" 결정(D2) 재검토 제안** 트리거.

## 6. 역할 분담

| 담당 | 작업 |
|---|---|
| Claude | 렌더링/추출/remap/변환/검수 스크립트, training·quantize 스크립트, Phase 5 테스트 스크립트, Phase 6 worker 교체, Phase 7 측정 |
| 사용자 | host PC 확보·실행(렌더링/학습/quantize), 실물 배치 live test |

## 7. Gate 요약

| Gate | 통과 조건 |
|---|---|
| 0 | Inspector: CPU subgraph 0 |
| 1 | class별 수량 + contact-sheet 검수 |
| 2 | float mAP + 혼동 pair + person recall + cross-domain 확인 |
| 3 | quant 손실 ≤3%p |
| 4 | DPU subgraph 1개 + 보드 load |
| 5 | host 일치 + 실환경 스냅샷 검출 |
| 6 | `/pick_target_base` 정상 |
| 7 | live 체크리스트 + 성능 측정 |

## 8. Open Items

- [x] ~~Host PC 사양~~ → 확정: Ubuntu 22.04.5 x86_64 / RTX 4060 8GB / RAM 16GB / 디스크 771GB 여유 / Docker 미설치(설치 예정). 경로 = native GPU 학습 + CPU docker quantize (Phase 0 참조)
- [x] ~~mustard bottle 제품 확인~~ → Morehouse로 확인, YCB 006 소스 유지 (D4)

## 9. References

- UG1414 v2.5 (`~/ug1414-vitis-ai.pdf`): Quantizer pp.86–107 · Compiler pp.108–135 · VART pp.136–141, 204–210 · 지원 op Table 20 pp.116–124
- arch: `~/vitis_ai_work/arch/arch_b3136.json` (fingerprint `0x101000016010406`)
- Phase 5 템플릿: `~/vitis_ai_work/scripts/ssd_adas_image_test.py`
- 현 파이프라인 상세: `workflow.md` / 히스토리: `progress.md`

### Gate 5 시도 1 (2026-07-07) — 부분 FAIL: sim-to-real 도메인 갭
- 보드 라이브 파이프라인 정상 기동 (config 전환 후). detector worker mode, dpu_ms ~17-23ms, processing ~50-70ms, /detections 발행 OK. (도중 발견·수정: `vitis_ai_detector.yaml`의 `metrics_duration_sec: 0` → `0.0` — 노드가 double로 선언, 정수면 InvalidParameterType로 detector 死. 모델 교체와 무관한 기존 config 버그.)
- 실물 6종 top-down 배치 캡처(848×480, obj_00~02) → 격리 테스트(yolov3_tiny_image_test.py) 결과 **일관되게**:
  - ✅ banana 0.78-0.81, orange 0.77-0.90, apple(빨간 사과) 0.66-0.76 — 잘 검출
  - ❌ peach → apple로 confident 오분류 (peach 최고 점수 0.015-0.018 = 사실상 0)
  - ❌ tennis_ball 미검출 (최고 0.004-0.017)
  - ❌ mustard_bottle 미검출 (최고 0.001)
- **threshold 문제 아님** (0.05로 낮춰도 후보 없음). 같은 프레임에서 3종 성공/3종 실패 → 전처리·decode 버그 아니라 class별 도메인 갭.
- 패턴: **COCO real 데이터가 풍부한 class(apple/orange/banana) = 전이 성공**, synthetic+niche real 의존 class(peach/tennis_ball/mustard) = 전이 실패. val mAP는 3종 다 높았음(0.82~0.98) → val의 real(OI/ycbv)과 우리 실물+촬영조건이 또 다른 분포.
- 계획 contingency ladder 도달: threshold(무효) → **다음 rung**. 원인 좁히기용 진단(물체 재배치: mustard 세우기 + 물체 간 간격) 후 D2 재검토 여부 사용자 판단 필요.

### Gate 5 시도 2 (2026-07-07) — 재배치 진단으로 원인 분리
배치2: 물체 간격 벌림 + mustard 수직 + (부수적으로 카메라가 낮은 oblique 각도로 바뀜). objects2/obj_00~02 격리+저thr 진단:
- **mustard_bottle**: 누움 0.001 → **수직 0.66~0.80 검출**. = orientation/view 문제 (외형 자체는 학습됨). synthetic이 눕힌 병·top-down 뷰를 충분히 못 담은 것. **단, D10 top-down(수직 상방)에서 세운 병은 뚜껑(원)만 보임 → 실제 배포 자세 확인 필요.**
- **tennis_ball**: 0.004→0.14~0.26. 간격 벌리니 firing 시작하나 여전히 0.5 미달. clutter + 약한 신뢰도 복합. marginal.
- **peach**: 0.02~0.047 유지 = 사실상 0. 간격 벌려도 apple로 confident 오분류(0.54). **재배치 무관한 외형 confusion (플라스틱 복숭아가 사과처럼 보임).** 3종 중 가장 어려움.
- 결론: mustard=자세(재렌더로 해결 가능), tennis=marginal(재렌더+threshold), peach=외형(synthetic-only로 어려움, real 데이터가 가장 효과적).
- **미해결 질문**: 실제 배포 카메라 각도(top-down vs oblique)와 물체 자세(병 눕힘/세움)를 확정해야 재렌더 타깃이 정해짐 → 사용자 확인 대기.

---

## ⏸ 작업 일시중단 지점 (2026-07-07)
사용자가 Kria 보드 RT 커널 패치를 병행 중이고 카메라가 아직 실제 배포 형상으로 세팅되지 않아, 여기서 잠시 멈추고 나중에 재개.

### 확정된 사실 (재개 시 다시 안 해도 됨)
- **모델 파이프라인 전부 통과**: Gate 2(mAP 0.766) / Gate 3(cosine 0.992·0.985) / Gate 4(DPU subgraph 1). xmodel 보드 배치 완료: `~/vitis_ai_work/models/yolov3_tiny_7class.xmodel` + `decode_meta.json` (md5 e2ca87c2…, fingerprint·fixpos 검증됨).
- **보드 추론 정상**: 라이브 파이프라인 기동 OK, dpu_ms ~17-23ms. apple/orange/banana 실물 잘 검출, 오검출 없음.
- **config 현재 상태 = YOLO로 전환됨** (아직 완전 검증 전):
  - `system_bringup_pkg/config/vitis_ai_detector.yaml`: model_path→yolov3_tiny_7class, worker→vitis_ai_worker_yolo.py, send_resized_input→false, metrics_duration_sec→0.0(버그수정)
  - `pick_logic.yaml`: allowed_classes→6 pickable
  - **SSD stand-in으로 되돌리려면** vitis_ai_detector.yaml 상단 주석의 SSD 3줄로 복구.

### Gate 5에서 발견된 미해결 문제 (재개 시 이어서)
실물 검증에서 6종 중 3종 실패 (단, **테스트가 실제 top-down 배포 형상이 아니었음** — oblique 각도였음):
- peach → apple 오분류 (외형 confusion, 제일 어려움)
- tennis_ball marginal (0.14~0.26, 0.5 미달)
- mustard 자세 민감 (누움 0.001 / 세움 0.80)

### 재개 시 순서 (합의된 방향: synthetic 재렌더 우선, D2 유지)
1. **먼저 실제 top-down 배포 형상으로 재검증** — 이번 테스트 형상이 D10과 달랐으므로, 진짜 top-down에서 다시 캡처→`yolov3_tiny_image_test.py`로 재판정. 실패가 형상 탓이었는지 분리.
2. 남는 실패만 **synthetic 재렌더 타깃**: 눕힌 병 top-down 뷰, tennis 다양화(조명/clutter), peach 텍스처 대비 강화 + apple hard-negative. (`02_render_synthetic.py` 수정)
3. 재렌더→재학습(11)→재양자화(12)→재컴파일(13)→보드 재검증.
4. peach가 재렌더로도 안 되면 그때 D2 재검토(소량 real 학습이미지). — 사용자 최종 판단 사항.
- 재개용 테스트 자산: `~/vitis_ai_work/test_images/objects/`(top-down 유사), `objects2/`(oblique), 캡처 헬퍼 `capture_color_frames.py`, 저-thr 진단 `scratchpad/diag_lowthr.py`.

### Gate 5 시도 3 (2026-07-08) — 진짜 top-down 재검증: 형상 아님, 도메인 갭 확정
RT 커널(5.15.199-rt91-rt-kria) 부팅 상태에서, 실제 D10 top-down(회색 타공 optical table 수직 하방)으로 카메라만 기동해 캡처(`test_images/topdown/`) → isolated 판정:
- banana 0.61-0.67 ✅ / orange 0.88 ✅
- **tennis_ball 0.491** (초록 공 위치 정확, 0.5 바로 아래) — top-down이 오히려 도움(oblique 0.14→0.49). threshold 0.4로 즉시 잡힘.
- **apple(실제 빨강) 0.26** — 이 뷰에서 약함. 게다가 peach가 apple로 0.66 검출되어 실제 사과보다 높게 밀어냄.
- **peach → apple** 오분류 유지 (peach-as-peach ~0).
- **mustard 0.02** — 완전 실패 (누운 top-down + Morehouse 스퀴즈병 ≠ YCB 006 French's 경질병).
- **결론**: 지난 실패는 형상 artifact가 아니라 진짜 sim-to-real 도메인 갭. peach·mustard는 YCB mesh와 실물 외형/형태가 근본적으로 달라 **같은 mesh 재렌더로는 못 닫을 가능성이 큼**.

| D12 | **D2 재검토 → 소량 real 학습이미지 추가**. Gate 5 실물 top-down에서 peach/mustard 도메인 갭이 synthetic 재렌더로 닫히기 어려움이 확인됨(mesh≠실물). real 촬영 이미지를 **학습(weight update)에도** 사용(기존엔 검증만). 기존 synthetic+public는 유지(banana/orange 등은 잘 됨), real은 실패 클래스 보강용. | 사용자 결정 2026-07-08 |

### ▶ 다음 작업 (2026-07-09 예정) — real 학습셋 수집·라벨·fine-tune
1. **real 학습셋 수집** (사용자 물체 배치 + `capture_color_frames.py`): 이 테이블에서 6종을 위치/각도/자세 다양하게(특히 **mustard 눕힘+세움**, 프레임 가장자리 포함) 수십~수백 장. 조명도 변주.
2. **라벨링** (D2가 피하려던 수작업 — 도구/워크플로 내일 제안: 잘 되는 class(banana/orange 등)는 현 모델로 auto-label 후 교정, 실패 class는 수동. labelImg/CVAT/Roboflow 등 후보).
3. **병합 + fine-tune**: 기존 synthetic+public 데이터에 real 추가 → hswish cfg로 재학습(`11_train.sh`). 초점 class: peach, mustard, apple.
4. **재양자화(`12_run_quantize_docker.sh`) → 재컴파일(`13_compile_docker.sh`) → 보드 재배치 → Gate 5 재검증**.
- **즉시 가능한 quick win(내일 반영)**: tennis_ball threshold를 0.5→0.4로 (image-test 스크립트/worker의 CLASS_THRESHOLDS). 0.491이라 바로 잡힘.
- 보존 자산: `test_images/topdown/`(실물 top-down 5장), `test_images/objects*/`(이전), 저-thr 진단 재생성용 코드는 이 섹션 참고.

---

## D13 — peach 드롭 (7-class → 6-class) + YCB real 데이터 도입 (2026-07-09)

| D13 | **peach 클래스 드롭 → 6-class 재번호** `{0 apple, 1 orange, 2 banana, 3 tennis_ball, 4 mustard_bottle, 5 person}`. + **YCB 벤치마크 real 스캔을 학습에 도입**(D12 real-data의 구체화). | 사용자 결정 2026-07-09 |

**근거 (YCB 대조로 확정)**:
- **peach**: 우리 실물(분홍+잎) ≠ YCB 015(주황+빨강 blush, 잎 없음). YCB 데이터로도 못 닫고 apple과 최난도 혼동 → **드롭**(교체 아님).
- **mustard**: 우리 실물 ≈ YCB 006(둘 다 노란 French's형 스퀴즈병). 실패는 자세가 아니라 **top-down 뷰 부족**이었고, **YCB N5 카메라가 거의 완벽한 top-down** → YCB 006이 우리 뷰를 커버. 교체 불필요.
- YCB 스캔 = 물체당 600장(N1~N5 × 120각) + mask(bbox 반자동) + 캘리브. **N4/N5 = 우리 배포 top-down 뷰.** 수작업 촬영·라벨 불필요.

**6-class 마이그레이션 (스크립트 반영 완료, 데스크톱 실행 대기)**:
- `classes.yaml` 6-class로 갱신(canonical). 03/04는 이를 동적 참조(자동), 02/02b/05/06/07은 하드코딩 직접 수정 완료.
- `08_ycb_real_to_yolo.py` (신규): YCB 이미지+mask → YOLO 라벨. N4/N5 위주, stride로 유사프레임 조절. bbox 추출 top-down mustard로 검증 완료(120/120, tight).
- `09_drop_peach_remap.py` (신규, **1회성**): 기존 7-class 추출물 → 6-class remap(peach 제거+재번호), peach-only 이미지 삭제. 안전가드(class 6 미발견 시 거부) 포함.
- `06_merge_split.py`: 6-class NAMES + `ycb_real_yolo` 소스 추가(전량 train — 턴테이블은 증강용, val은 board 실장면).
- 보드 `pick_logic.yaml`: pickable 5종(peach 제거). worker/decode_meta는 num_classes 자동.
- tennis_ball threshold 0.5→0.4 (classes.yaml 반영; 배포 worker에도 반영 필요).

**데스크톱 실행 순서 (데이터 준비 후)**:
```
# 0) 최신 스크립트 동기화
rsync -av ubuntu@192.168.120.50:~/ros2_ws/yolo_v3_tiny_training/ ~/capstone_training/training/
# 1) 기존 7-class 데이터 → 6-class remap (1회)
python3 09_drop_peach_remap.py --dry-run    # 먼저 확인
python3 09_drop_peach_remap.py
# 2) YCB real 5종 변환 (013 apple→0, 017 orange→1, 011 banana→2, 056 tennis→3, 006 mustard→4)
#    (YCB RGB 세트를 데스크톱 ~/ycb/<objdir> 로 받은 뒤)
python3 08_ycb_real_to_yolo.py --ycb-dir ~/ycb/013_apple --class-id 0 --cameras N3,N4,N5 --stride 2 --out ~/capstone_training/datasets/ycb_real_yolo
#    ... orange/banana/tennis/mustard 동일 (같은 --out 에 누적)
# 3) 병합 + split → data.yaml (6-class)
python3 06_merge_split.py
# 4) 재학습 → 재양자화 → 재컴파일 → 보드 재배치 → Gate 5 재검증
bash 11_train.sh && bash 12a_inspect_docker.sh && bash 12_run_quantize_docker.sh && bash 13_compile_docker.sh
```
- ⚠️ **09는 1회만**(6-class 데이터에 재실행 시 orange를 peach로 오인 — 안전가드가 막지만 주의).
- ⚠️ YCB 나머지 4종(apple/orange/banana/tennis)도 실물과 매칭되는지 대조 권장(특히 tennis·apple).

---

## D13 결과 (2026-07-09): Gate 2→5 통과, apple만 도메인갭 잔존

**재학습(6-class, YCB real 포함) → 양자화 → 컴파일**:
- Gate 2 all mAP50 **0.748** (mustard 0.933 / tennis 0.950 / orange·banana 0.736 / apple **0.629**(최약) / person 0.503).
- Gate 3 cosine **0.9757 / 0.9615**(지난 0.99보다 낮으나 스크립트 기준선 0.95 상회 → 통과; head[1]=큰물체 약함, 문제 시 fast-finetune).
- Gate 4 DPU subgraph **1** (md5 `d925c711...`, 출력 **33ch**=(6+5)×3). 구 7-class는 보드 `models/*.OLD7.*` 백업.

**Gate 5 실물 top-down 재검증**: mustard **0.02→0.814**(핵심 성과), tennis 0.491→0.677, orange 0.850, banana 0.777 — **5종 정상**. **apple만 3프레임 0.489/0.549/0.549 = 0.50 경계**.
- 진단1: 드롭된 **peach(OOD)가 apple로 0.462 오검출** + 인접 물체 억제 → 복숭아 치우니 진짜 apple 0.216→0.549, 나머지도 상승. **배포엔 peach 없으니 오검출은 무의미.**
- 진단2: 진짜 apple이 0.50 경계 = **flicker 위험**. 원인 = color 도메인 갭(실물 apple이 YCB apple보다 밝음) + apple 자체 최난도(둥금 → orange/tennis 혼동). **양 문제 아님**(이미 ~6371 인스턴스).

## D14 — apple 실물 캡처 후 타깃 재학습 → Gate 5 통과 (2026-07-09~10)

| D14 | apple 안정화 = **배포 카메라로 apple만 올린 top-down 실촬영**을 학습에 추가(단일물체 → auto-label 수작업 ≈0). "그냥 더 많은 데이터"(B') 기각 = apple은 양이 아니라 **color 갭**이 문제. | 사용자 결정 2026-07-09 |

**실행(완료분)**:
- `capture_color_frames.py`에 `--manual`(Enter=1장 저장) 모드 추가 → 움직임 중 blur 프레임 배제, 정지 순간만 캡처.
- `autolabel_single_object.py`(신규, 보드): 현재 DPU 모델로 top-1 box 추출 → `--class-id`로 라벨 강제(단일물체라 class 오인해도 box는 정확) + viz 검수 오버레이.
- apple만 올려두고 ~54장 캡처 → auto-label(obj_54 1장만 오검출 → 삭제) → `datasets/real_apple_yolo/{images,labels}`로 rsync.
- `06_merge_split.py`: `SOURCES`에 `real_apple_yolo` 추가 + `TRAIN_ONLY_SOURCES`(실촬영을 val에 넣으면 같은 station 과대평가 → train 전량).
- `hyp_pickplace.yaml`: `hsv_v` 0.40→0.50(밝기 갭 완충). **hsv_h는 의도적 유지**(키우면 apple↔orange 색 혼동).
- **데스크톱 `11_train.sh` 재학습 시작 → GPU 드라이버 hang으로 2회 중단(둘 다 epoch 0, GPU ~50℃=과열 아님)**. 원인=자동 업데이트로 설치된 `nvidia-driver-595-open`(open 커널모듈+GSP)의 **로그 없는 silent hard-hang**(OOM/Xid/MCE 전무). "오후엔 되고 저녁엔 안 됨"=새 모듈이 재부팅 때 발효되기 때문. 해결=proprietary `nvidia-driver-580`로 롤백 + `apt-mark hold`(자동 업데이트 차단). **드라이버 수정 완료 → 재학습 완주(2026-07-10).** 재발 시 하드웨어(memtest/PSU) 의심.

### ★ D14 결과 — Gate 5 통과 🎉 (2026-07-10)
- 재학습 완주(hswish5, 2.528h). Gate 2 all mAP50 0.728 — **apple val 0.625**는 real_apple이 **train-only**라 val엔 개선이 안 잡히는 게 정상(전체 소폭↓는 `hsv_v`↑의 대가). Gate 3/4 통과(md5 `9bc6520c`, DPU subgraph 1).
- **보드 Gate 5(실물 top-down, D13과 같은 프레임)**: **apple 0.489~0.549 → 0.876/0.899/0.875**(완전 해결). 나머지도 전부 ↑/유지 — orange 0.85~0.88, banana 0.83~0.85, **mustard 0.81~0.87**(val 하락이 실물엔 무영향), tennis 0.82~0.85. → **모델 교체(Gate 2~5) 완성.**
- 정리: `apple:0.40` threshold를 worker+test 스크립트 모두 **0.50 default로 원복**(apple 0.88이라 불필요; 애초 '0.462=진짜사과' 오판 기반이었고 실제 0.462는 peach였음). orange가 mustard를 0.19로 약하게 오검출하나 배포 threshold(0.50) 아래라 무해. 배포 xmodel=**D14 `9bc6520c`**(D13 백업 없이 덮음 → 필요시 hswish2 best.pt로 12→13 재생성).

**Gate 6 실측 완료(2026-07-10, 4코어)**: 파이프라인 15Hz 동기·E2E 137ms = PASS. CPU 79%/4코어, **target_3d 69% 최대**(depth 30fps 매프레임 cv_bridge 변환, 3D 계산은 15Hz만). YOLO 전/후처리는 이미 LUT(`build_input_lut`)·int8 objectness 사전필터(`decode_head`)가 적용돼 있음. **남은 것: `pick_target_3d_node` CPU 최적화(depth 30→15fps 등, 3+1 EtherCAT 격리 대비)·Gate 7 live.** perf 하네스·CSV=`~/vitis_ai_work/perf/`.
