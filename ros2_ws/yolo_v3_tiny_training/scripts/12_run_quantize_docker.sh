#!/usr/bin/env bash
# =============================================================================
# 12_run_quantize_docker.sh — Phase 3 양자화 실행 (host에서 실행하는 wrapper)
#
#   bash 12_run_quantize_docker.sh                    # 최신 best.pt 자동 탐색
#   bash 12_run_quantize_docker.sh runs/xxx/weights/best.pt   # 직접 지정
#
# Vitis-AI 2.5 CPU docker 안에서 순서대로 실행한다:
#   ① calib (500장)  ② test (float vs quant 유사도)  ③ deploy (xmodel export)
# 출력: ~/capstone_training/quantize/quantize_result/
#         DeployModel_int.xmodel + decode_meta.json (+ quant config)
# =============================================================================
set -euo pipefail

# sudo bash 로 통째로 실행해도 경로가 /root 로 튀지 않게 원 사용자 HOME 사용
RUN_HOME="$HOME"
[[ -n "${SUDO_USER:-}" ]] && RUN_HOME="$(getent passwd "$SUDO_USER" | cut -d: -f6)"
BASE="$RUN_HOME/capstone_training"
IMAGE="xilinx/vitis-ai-cpu:2.5.0.1260"

# best.pt 찾기 (인자 없으면 runs에서 최신)
if [[ $# -ge 1 ]]; then
  WEIGHTS_HOST="$1"
  [[ "$WEIGHTS_HOST" = /* ]] || WEIGHTS_HOST="$BASE/$WEIGHTS_HOST"
else
  WEIGHTS_HOST=$(ls -t "$BASE"/runs/pickplace_v3tiny*/weights/best.pt 2>/dev/null | head -1 || true)
fi
[[ -n "${WEIGHTS_HOST:-}" && -f "$WEIGHTS_HOST" ]] || {
  echo "best.pt 없음 — 학습(11_train.sh) 완료 후 실행하거나 경로를 인자로 주세요"; exit 1; }
WEIGHTS_C="/workspace/${WEIGHTS_HOST#"$BASE"/}"
echo "== weights: $WEIGHTS_HOST"

# yolov5 코드가 import해야 하는 패키지들을 컨테이너 conda env에 설치 후 3단계 실행
sudo docker run --rm -v "$BASE":/workspace -w /workspace "$IMAGE" bash -lc "
set -e
conda activate vitis-ai-pytorch
# VAI 2.5 배포 버그 패치 2건 (pytorch hardswish 경로가 upstream에서 미정비 상태):
#  ① __init__: 미정의 심볼 FixNeuronWithBackward (미사용 dead line, v3.0은 줄 삭제로 수정)
#  ② forward: fake_quantize_per_tensor 호출에 필수 인자 method/inplace 누락
#     → 패키지 자체 기본 rounding(method=2) + inplace=False 로 보완
# --rm 컨테이너라 매 실행 적용 (치환 후엔 패턴이 없어 재실행에도 안전)
sed -i 's/self.fix_neuron = FixNeuronWithBackward()/self.fix_neuron = None/' \
  /opt/vitis_ai/conda/envs/vitis-ai-pytorch/lib/python3.7/site-packages/pytorch_nndct/nn/modules/hardswish.py
sed -i 's/fake_quantize_per_tensor(output, scale_inv=128, zero_point=0, quant_min=-128, quant_max=127)/fake_quantize_per_tensor(output, scale_inv=128, zero_point=0, quant_min=-128, quant_max=127, method=2, inplace=False)/' \
  /opt/vitis_ai/conda/envs/vitis-ai-pytorch/lib/python3.7/site-packages/pytorch_nndct/nn/modules/hardswish.py
# 이전 모델(SiLU 1차)의 quant config/bias_corr 잔재 제거 — calib부터 전부 재생성됨
rm -rf /workspace/quantize/quantize_result
pip install -q pyyaml pandas requests tqdm matplotlib seaborn 'numpy<2' 2>/dev/null || \
pip install -q pyyaml pandas requests tqdm matplotlib seaborn
echo '== ① calibration =='
python training/scripts/12_quantize.py --weights '$WEIGHTS_C' --quant_mode calib --subset 500
echo '== ② quantized vs float 유사도 =='
python training/scripts/12_quantize.py --weights '$WEIGHTS_C' --quant_mode test --subset 100
echo '== ③ xmodel export =='
python training/scripts/12_quantize.py --weights '$WEIGHTS_C' --quant_mode test --deploy
"

echo ""
echo "완료: $BASE/quantize/quantize_result/"
ls -la "$BASE/quantize/quantize_result/" || true
echo "다음: bash 13_compile_docker.sh"
