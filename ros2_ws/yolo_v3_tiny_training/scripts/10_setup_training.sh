#!/usr/bin/env bash
# =============================================================================
# 10_setup_training.sh — Phase 2 학습 환경 구축 (최초 1회, 데스크톱)
#
#   bash 10_setup_training.sh
#
# 하는 일:
#   1. 학습 전용 venv 생성 (~/capstone_training/venv_train)
#      - 렌더링용 venv와 분리: torch 버전 충돌 방지
#   2. PyTorch 2.2.2 + CUDA 12.1 설치 (RTX 4060 지원, torch 2.6의
#      weights_only 기본값 변경 이슈 회피를 위해 고정)
#   3. yolov5 repo clone + v7.0 태그 고정
#      - 이 repo의 models/hub/yolov3-tiny.yaml 로 "고전" anchor 기반
#        YOLOv3-tiny를 학습한다 (DPU decode 계획과 head 구조 일치).
#      - ultralytics v8 패키지의 yolov3-tiny"u"는 anchor-free head라
#        출력 구조가 달라져 사용하지 않음.
#   4. COCO pretrained yolov3-tiny.pt 다운로드
# =============================================================================
set -euo pipefail

BASE="$HOME/capstone_training"
VENV="$BASE/venv_train"
YOLO="$BASE/yolov5"
WEIGHTS_URL="https://github.com/ultralytics/yolov3/releases/download/v9.6.0/yolov3-tiny.pt"

echo "== 1) venv_train 생성 =="
python3 -m venv "$VENV"
# shellcheck disable=SC1091
source "$VENV/bin/activate"
pip install --upgrade pip

echo "== 2) PyTorch (CUDA 12.1) =="
pip install torch==2.2.2 torchvision==0.17.2 --index-url https://download.pytorch.org/whl/cu121

echo "== 3) yolov5 v7.0 =="
if [[ ! -d "$YOLO" ]]; then
  git clone https://github.com/ultralytics/yolov5 "$YOLO"
fi
git -C "$YOLO" fetch --tags
git -C "$YOLO" checkout v7.0
# yolov5 v7.0 호환 버전 고정: constraints로 처음부터 맞는 버전을 고르게 함
# (numpy 1.x 고정 + numpy 2를 요구하는 opencv 5.x 차단)
CONSTRAINTS="$BASE/training/config/pip_constraints.txt"
pip install "numpy==1.26.4"
pip install -r "$YOLO/requirements.txt" -c "$CONSTRAINTS"

echo "== 4) pretrained weight =="
mkdir -p "$BASE/weights"
wget -c -q --show-progress -P "$BASE/weights" "$WEIGHTS_URL"

echo "== 4b) yolov5 plot 폰트 준비 (ultralytics.com 308 redirect 이슈 회피) =="
mkdir -p "$HOME/.config/Ultralytics"
if [[ ! -s "$HOME/.config/Ultralytics/Arial.ttf" ]]; then
  curl -fsSL -o "$HOME/.config/Ultralytics/Arial.ttf" \
    https://ultralytics.com/assets/Arial.ttf \
    || cp /usr/share/fonts/truetype/dejavu/DejaVuSans.ttf \
         "$HOME/.config/Ultralytics/Arial.ttf" || true
fi

echo "== 5) GPU 확인 =="
python - <<'EOF'
import torch
print("torch:", torch.__version__, "| cuda available:", torch.cuda.is_available())
if torch.cuda.is_available():
    print("device:", torch.cuda.get_device_name(0))
EOF

echo ""
echo "완료. 다음: bash 11_train.sh"
