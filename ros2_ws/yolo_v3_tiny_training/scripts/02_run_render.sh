#!/usr/bin/env bash
# =============================================================================
# 02_run_render.sh — synthetic 렌더링 전체 실행 (batch 반복 wrapper)
#
# 용법 (데스크톱, venv 활성화 후):
#   bash 02_run_render.sh                 # 기본: 40 batch × 40 scene × 4 view ≈ 6,400장
#   bash 02_run_render.sh 10 40 4         # batch 수 / batch당 scene / scene당 view
#
# - batch마다 blender 프로세스를 새로 띄운다 (장시간 실행 시 메모리 누적 방지)
# - 중간에 중단해도 완료된 batch는 보존됨. 재시작하면 완료된 batch는 건너뜀
# - 진행 중 인스턴스 수 확인: python3 02b_synth_to_yolo.py --count-only
# =============================================================================
set -euo pipefail

N_BATCHES=${1:-40}
SCENES=${2:-40}
VIEWS=${3:-4}
ASSETS="${ASSETS_DIR:-$HOME/capstone_training/assets}"
SYNTH="${SYNTH_DIR:-$HOME/capstone_training/synth}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "== synthetic 렌더링: ${N_BATCHES} batch × ${SCENES} scene × ${VIEWS} view =="
for i in $(seq 0 $((N_BATCHES - 1))); do
  batch_dir="$SYNTH/batch_$(printf %03d "$i")"
  if [[ -f "$batch_dir/coco_data/coco_annotations.json" ]]; then
    echo "== batch $i: 이미 존재 → 건너뜀 ($batch_dir)"
    continue
  fi
  echo "== batch $((i + 1))/$N_BATCHES → $batch_dir"
  blenderproc run "$SCRIPT_DIR/02_render_synthetic.py" -- \
    --assets "$ASSETS" \
    --out "$batch_dir" \
    --scenes "$SCENES" \
    --views "$VIEWS" \
    --seed "$i"
done
echo "== 렌더링 완료. 다음: python3 $SCRIPT_DIR/02b_synth_to_yolo.py"
