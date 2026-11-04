# YOLOv3-tiny 7-Class Training Workspace

KV260 pick & place용 최종 detector 학습·양자화 작업 공간.
**전체 계획과 결정 사항은 `../yolov3_tiny_execution_plan.md` 참조** (이 디렉토리는 그 계획의 Phase 1~4 실행 코드).

- 실행 머신: **데스크톱** (Ubuntu 22.04, RTX 4060 8GB, Vitis-AI 2.5 docker)
- 이 디렉토리는 KV260 보드의 레포(`~/ros2_ws/yolo_v3_tiny_training/`)에서 작성·버전관리되고, 데스크톱으로 복사해서 실행한다.

## 데스크톱으로 가져오기

```bash
# 데스크톱에서 (보드 IP: 192.168.120.132)
rsync -av ubuntu@192.168.120.132:~/ros2_ws/yolo_v3_tiny_training/ ~/capstone_training/training/
# rsync 없으면: scp -r ubuntu@192.168.120.132:~/ros2_ws/yolo_v3_tiny_training ~/capstone_training/
```

## 디렉토리 구조 (데스크톱)

```
~/capstone_training/
  training/    # 이 디렉토리 (스크립트+config, 보드에서 rsync)
  assets/      # 다운로드 원본 (YCB mesh, COCO, BOP ycbv)
  synth/       # BlenderProc 렌더링 출력
  datasets/    # 최종 YOLO format dataset (train/val)
  runs/        # 학습 결과 (weights, 로그)
```

## 환경 준비 (최초 1회)

```bash
sudo apt install -y wget curl unzip python3-venv
python3 -m venv ~/capstone_training/venv
source ~/capstone_training/venv/bin/activate
pip install --upgrade pip
pip install blenderproc pycocotools fiftyone opencv-python pyyaml tqdm
```

(학습용 PyTorch CUDA 환경은 Phase 2 단계에서 별도 안내 — `runs/` 참조 예정)

## 실행 순서

| 순서 | 스크립트 | 내용 | 상태 |
|---|---|---|---|
| 01 | `scripts/01_download_assets.sh` | YCB mesh / COCO / BOP ycbv 다운로드 | **사용 가능** |
| 02 | `scripts/02_run_render.sh` (→ `02_render_synthetic.py`) | BlenderProc domain-randomized 렌더링 (6 YCB 물체) | **사용 가능** |
| 02b | `scripts/02b_synth_to_yolo.py` | 렌더링 결과 → YOLO 변환 + class별 수량 집계 | **사용 가능** |
| 03 | `scripts/03_extract_bop_ycbv.py` | BOP ycbv → YOLO (mustard/banana real) | **사용 가능** |
| 04 | `scripts/04_extract_coco.py` | COCO → YOLO (person + 과일, cap 적용) | **사용 가능** |
| 05 | `scripts/05_extract_openimages.py` | Open Images → YOLO (peach/tennis_ball) | **사용 가능** (최초 실행 시 수 GB 추가 다운로드) |
| 06 | `scripts/06_merge_split.py` | 병합·scene 단위 train/val split + data.yaml 생성 | **사용 가능** |
| 07 | `scripts/07_contactsheet.py` | annotation 육안 검수용 contact-sheet 생성 | **사용 가능** |
| 10 | `scripts/10_setup_training.sh` | 학습 환경 구축 (venv_train + torch cu121 + yolov5 v7.0 + pretrained) | **사용 가능** |
| 11 | `scripts/11_train.sh` | YOLOv3-tiny 7-class fine-tuning 실행 (D11: Hardswish cfg 자동 생성·검증) | **사용 가능** |
| 12a | `scripts/12a_inspect_docker.sh` | **Gate 0**: 학습 전 Inspector로 전 op DPU 매핑 선검증 (Vitis-AI docker) — 11번보다 먼저 실행 | **사용 가능** |
| 12 | `scripts/12_run_quantize_docker.sh` (→ `12_quantize.py`) | Phase 3 양자화: calib→test→xmodel export (Vitis-AI docker) | **사용 가능** |
| 13 | `scripts/13_compile_docker.sh` | Phase 4 컴파일: B3136용 .xmodel 생성 | **사용 가능** |

지금 시작할 수 있는 것 (다운로드가 오래 걸리므로 먼저 걸어두기):

```bash
bash ~/capstone_training/training/scripts/01_download_assets.sh --meshes     # 수백 MB, 금방
bash ~/capstone_training/training/scripts/01_download_assets.sh --coco       # ~19 GB
bash ~/capstone_training/training/scripts/01_download_assets.sh --bop-test   # ~15 GB
```

끊기면 같은 명령 재실행 (이어받기 지원).

## Synthetic 렌더링 (02)

```bash
source ~/capstone_training/venv/bin/activate
cd ~/capstone_training/training/scripts

# 1) smoke test — 2 scene × 2 view만 렌더링해서 파이프라인 동작 확인 (몇 분)
#    ※ 최초 실행 시 blenderproc가 Blender(~수백 MB)를 자동 다운로드함
blenderproc run 02_render_synthetic.py -- \
  --assets ~/capstone_training/assets --out ~/capstone_training/synth/smoke \
  --scenes 2 --views 2 --samples 16

# 2) smoke 결과 확인 (이미지를 열어 물체가 테이블 위에 놓였는지 눈으로 확인)
python3 02b_synth_to_yolo.py --synth-dir ~/capstone_training/synth/smoke --count-only
ls ~/capstone_training/synth/smoke/coco_data/images/

# 3) 본 렌더링 (기본 40 batch × 40 scene × 4 view ≈ 6,400장, GPU로 수 시간)
#    중단해도 완료된 batch는 보존, 재실행하면 이어서 진행
bash 02_run_render.sh

# 4) YOLO 변환 + class별 인스턴스 수 확인 (목표: class당 3,000)
python3 02b_synth_to_yolo.py
```

팁: 테이블/배경 텍스처로 COCO 이미지를 쓰면 다양성이 크게 좋아진다.
`--coco` 다운로드 완료 후 `unzip -q ~/capstone_training/assets/coco/train2017.zip -d ~/capstone_training/assets/coco` 를 먼저 하면 02가 자동으로 사용한다 (없으면 단색 배경으로 동작).

## BOP ycbv 추출 (03) — mustard/banana real 데이터

```bash
# 사전: ycbv_test_all.zip 압축 해제
unzip -q ~/capstone_training/assets/bop_ycbv/ycbv_test_all.zip -d ~/capstone_training/assets/bop_ycbv

python3 03_extract_bop_ycbv.py
# → datasets/ycbv_yolo/{images,labels}/ + class별 수량 출력
```

같은 frame의 다른 YCB 물체(캔·드릴 등)는 우리 class가 아니므로 라벨 없이 두어
자연스러운 distractor 역할을 한다. 렌더링(02)과 동시에 실행해도 된다.

## Phase 2 — 학습 (10~11, Gate 1 통과 후)

```bash
# 최초 1회: 학습 환경 구축 (별도 venv_train — 렌더링 venv와 분리)
bash 10_setup_training.sh

# 학습 (기본 150 epoch, batch 64 — RTX 4060에서 수 시간)
bash 11_train.sh
# 커스텀: EPOCHS=200 BATCH=48 bash 11_train.sh
```

- 결과 weight: `~/capstone_training/runs/pickplace_v3tiny*/weights/best.pt`
- 학습 곡선/지표: 같은 폴더의 `results.png`, `results.csv`
- **Gate 2**: val mAP@0.5, class별 recall, 혼동 pair(apple↔peach↔orange,
  tennis_ball↔orange) 확인 — 학습 종료 시 class별 표가 출력된다.
- augmentation은 `config/hyp_pickplace.yaml` — top-down 배포(D10)에 맞춰
  회전 180°/상하반전을 켜둔 것이 핵심.

## Phase 3~4 — 양자화 + 컴파일 (12~13, 학습 완료 후)

```bash
bash 12_run_quantize_docker.sh     # calib 500장 → 유사도 리포트 → xmodel export
bash 13_compile_docker.sh          # → compiled_yolov3_tiny/yolov3_tiny_7class.xmodel
```

- 12의 "float vs quantized cosine 유사도"가 0.99↑면 양자화 손실 미미,
  0.95 미만이면 fast-finetune/QAT escalation 검토 (계획 Phase 3).
- 13 로그의 **DPU subgraph number = 1** 확인 (Gate 4). 2 이상이면 CPU op 혼입.
- 함께 생성되는 `decode_meta.json`(anchors/strides/names)은 보드 worker의
  decode 구현이 사용한다 — xmodel과 같이 보드로 복사할 것.

## 핵심 규칙

- **class id는 `config/classes.yaml`이 유일한 기준** (0 apple … 5 person; D13 peach 드롭 이후 6-class). 모든 스크립트가 이 파일을 읽는다. 수정 금지.
- **배포 기하 (계획 D10)**: optical table, 카메라는 테이블 정중앙 상부 ~0.8 m에서 수직 하방 고정. synthetic 카메라는 이를 중심으로 randomize (elev 55~90°, 거리 0.55~1.05 m).
- 전처리 = **letterbox 416×416** (비율 유지 + 상하 padding — yolov5 표준). 학습·quantize·보드 worker 3곳 동일 (계획 D7, 2026-07-07 개정).
- **실환경(D435i) 촬영 이미지는 학습에 넣지 않는다** (계획 D2). 검증용으로만 사용.
- val split은 scene 단위로 나눈다 (연속/유사 frame이 train과 val에 갈라지면 안 됨).

## Vitis-AI docker (Phase 3~4에서 사용)

```bash
# 이미지: xilinx/vitis-ai-cpu:2.5.0.1260 (다운로드 완료 2026-07-06)
sudo docker run --rm -it -v ~/capstone_training:/workspace xilinx/vitis-ai-cpu:2.5.0.1260 bash
# 컨테이너 안에서:
conda activate vitis-ai-pytorch
```

- arch 파일: 보드의 `~/ros2_ws/src/vitis_ai_detector_pkg/models/arch_b3136.json` → `scp ubuntu@192.168.120.132:~/ros2_ws/src/vitis_ai_detector_pkg/models/arch_b3136.json ~/capstone_training/`
- quantize/compile 절차와 명령은 `../yolov3_tiny_execution_plan.md` Phase 3~4 참조 (UG1414 v2.5 근거)
