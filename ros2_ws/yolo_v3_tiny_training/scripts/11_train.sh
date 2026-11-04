#!/usr/bin/env bash
# =============================================================================
# 11_train.sh — YOLOv3-tiny 7-class fine-tuning 실행 (데스크톱, RTX 4060)
#
#   bash 11_train.sh                # 기본: 150 epoch, batch 64
#   EPOCHS=200 BATCH=48 bash 11_train.sh
#
# - 입력: datasets/final/data.yaml (06 출력) — 없으면 중단
# - 출력: ~/capstone_training/runs/pickplace_v3tiny*/weights/best.pt
# - 중간에 Ctrl+C 해도 last.pt가 남고, --resume 으로 이어서 학습 가능
# - D11: yolov5 v7.0의 Conv 기본 activation은 SiLU(DPU 미지원)라서,
#   `activation: nn.Hardswish()` 를 붙인 커스텀 cfg를 자동 생성/검증 후 학습한다.
#   학습 전 반드시 bash 12a_inspect_docker.sh 로 Gate 0(DPU 매핑) 통과 확인.
# =============================================================================
set -euo pipefail

BASE="$HOME/capstone_training"
DATA="$BASE/datasets/final/data.yaml"
HYP="$BASE/training/config/hyp_pickplace.yaml"
EPOCHS="${EPOCHS:-150}"
BATCH="${BATCH:-64}"
CFG="${CFG:-models/yolov3-tiny-hswish.yaml}"   # yolov5 dir 기준 상대경로

[[ -f "$DATA" ]] || { echo "data.yaml 없음 — 06_merge_split.py 먼저 실행"; exit 1; }
# shellcheck disable=SC1091
source "$BASE/venv_train/bin/activate"

cd "$BASE/yolov5"

if [[ ! -f "$CFG" ]]; then
  echo "== cfg 생성: $CFG (yolov3-tiny + activation: nn.Hardswish(), D11) =="
  cp models/hub/yolov3-tiny.yaml "$CFG"
  printf '\nactivation: nn.Hardswish()\n' >> "$CFG"
fi

echo "== activation 검증 (SiLU가 나오면 중단) =="
python - "$CFG" <<'PYEOF'
import sys
from models.yolo import Model
act = type(Model(sys.argv[1]).model[0].act).__name__
print(f"[check] Conv activation = {act}")
assert act == "Hardswish", (
    f"activation이 {act} — yolov5가 yaml의 activation key를 무시했습니다. 학습 중단")
PYEOF

python train.py \
  --img 416 \
  --batch "$BATCH" \
  --epochs "$EPOCHS" \
  --data "$DATA" \
  --weights "$BASE/weights/yolov3-tiny.pt" \
  --cfg "$CFG" \
  --hyp "$HYP" \
  --project "$BASE/runs" \
  --name pickplace_v3tiny_hswish \
  --workers 8 \
  --seed 0

echo ""
echo "학습 완료. 결과:"
echo "  weight : $BASE/runs/pickplace_v3tiny*/weights/best.pt"
echo "  곡선/지표: $BASE/runs/pickplace_v3tiny*/results.png, results.csv"
echo "  class별 mAP: 학습 마지막 로그 또는 val 재실행으로 확인"
