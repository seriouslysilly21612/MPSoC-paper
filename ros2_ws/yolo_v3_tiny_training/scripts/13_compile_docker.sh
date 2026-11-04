#!/usr/bin/env bash
# =============================================================================
# 13_compile_docker.sh — Phase 4 컴파일: quantized xmodel → B3136용 .xmodel
#
#   bash 13_compile_docker.sh
#
# UG1414 v2.5 p.112: vai_c_xir -x <quantized.xmodel> -a <arch.json> -o <out> -n <name>
# 성공 기준(계획 Gate 4): 로그에 "DPU subgraph number: 1"
# 출력: ~/capstone_training/compiled_yolov3_tiny/yolov3_tiny_7class.xmodel
# =============================================================================
set -euo pipefail

# sudo bash 로 통째로 실행해도 경로가 /root 로 튀지 않게 원 사용자 HOME 사용
RUN_HOME="$HOME"
[[ -n "${SUDO_USER:-}" ]] && RUN_HOME="$(getent passwd "$SUDO_USER" | cut -d: -f6)"
BASE="$RUN_HOME/capstone_training"
IMAGE="xilinx/vitis-ai-cpu:2.5.0.1260"
QUANT_X="/workspace/quantize/quantize_result/DeployModel_int.xmodel"
ARCH="/workspace/arch_b3136.json"

[[ -f "$BASE/quantize/quantize_result/DeployModel_int.xmodel" ]] || {
  echo "quantized xmodel 없음 — 12_run_quantize_docker.sh 먼저"; exit 1; }
[[ -f "$BASE/arch_b3136.json" ]] || {
  echo "arch_b3136.json 없음 — 보드에서 복사: scp ubuntu@192.168.120.132:~/ros2_ws/src/vitis_ai_detector_pkg/models/arch_b3136.json $BASE/"; exit 1; }

sudo docker run --rm -v "$BASE":/workspace -w /workspace "$IMAGE" bash -lc "
set -e
conda activate vitis-ai-pytorch
vai_c_xir -x '$QUANT_X' -a '$ARCH' -o /workspace/compiled_yolov3_tiny -n yolov3_tiny_7class
"

echo ""
echo "== 컴파일 결과 =="
ls -la "$BASE/compiled_yolov3_tiny/"
echo ""
echo "위 로그에서 'DPU subgraph number'가 1인지 확인하세요 (Gate 4)."
echo "다음: xmodel + decode_meta.json을 보드로 복사"
echo "  scp $BASE/compiled_yolov3_tiny/yolov3_tiny_7class.xmodel \\"
echo "      $BASE/quantize/quantize_result/decode_meta.json \\"
echo "      ubuntu@192.168.120.132:~/ros2_ws/src/vitis_ai_detector_pkg/models/"
