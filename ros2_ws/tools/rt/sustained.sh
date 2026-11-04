#!/usr/bin/env bash
# sustained.sh — 지속 부하 재현 (파일 분리로 self-kill 방지)
DUR="${1:-180}"
kill_pipeline() {
  pkill -9 -f "pick_place_vitis_ai" 2>/dev/null
  pkill -9 -f "vitis_ai_worker_yolo" 2>/dev/null
  pkill -9 -f "vitis_ai_detector_node" 2>/dev/null
  pkill -9 -f "realsense2_camera_node" 2>/dev/null
  pkill -9 -f "pick_post_stack" 2>/dev/null
  pkill -9 -f "pick_target" 2>/dev/null
  pkill -9 -f "pick_logic" 2>/dev/null
  sleep 4
}
cd "$HOME/vitis_ai_work/perf" || exit 1
echo "[정리 전 잔재] $(pgrep -fc vitis_ai_worker_yolo)개"
kill_pipeline
echo "[정리 후 잔재] $(pgrep -fc vitis_ai_worker_yolo)개 (0이어야)"
echo "=== sustained ${DUR}s 재현 ==="
bash run_gate6_perf.sh "$DUR" 2>&1 | grep -E "^\[run\]|^\[!\]|probe" | head -8
kill_pipeline
echo "=== done $(date +%T) ==="
