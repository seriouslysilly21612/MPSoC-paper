#!/usr/bin/env bash
# churn.sh — zocl KDS 재현/검증: 파이프라인 시작/정지 반복(churn)
# 파일 분리로 self-kill 방지(호출 셸 cmdline에 pkill 패턴 미노출).
CYCLES="${1:-4}"
DUR="${2:-20}"
kill_pipeline() {
  pkill -9 -f "pick_place_vitis_ai" 2>/dev/null
  pkill -9 -f "vitis_ai_worker_yolo" 2>/dev/null
  pkill -9 -f "vitis_ai_detector_node" 2>/dev/null
  pkill -9 -f "realsense2_camera_node" 2>/dev/null
  pkill -9 -f "pick_post_stack" 2>/dev/null
  pkill -9 -f "pick_target" 2>/dev/null
  pkill -9 -f "pick_logic" 2>/dev/null
  sleep 3
}
cd "$HOME/vitis_ai_work/perf" || exit 1
kill_pipeline
for i in $(seq 1 "$CYCLES"); do
  echo "########## churn cycle $i/$CYCLES $(date +%T) ##########"
  bash run_gate6_perf.sh "$DUR" 2>&1 | grep -E "^\[run\]|^\[!\]|probe" | head -6
  kill_pipeline
done
echo "########## churn done $(date +%T) ##########"
