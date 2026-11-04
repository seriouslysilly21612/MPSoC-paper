#!/usr/bin/env bash
# =============================================================================
# 12a_inspect_docker.sh — Gate 0: 학습 전 DPU 매핑 선검증 (host wrapper)
#
#   bash 12a_inspect_docker.sh
#
# 2시간짜리 학습에 들어가기 전에, 모델 구조의 모든 op가 우리 DPU(B3136)에
# 올라가는지 Inspector로 몇 분 만에 확인한다 (UG1414 v2.5 p.87).
# Gate 3 시도1에서 SiLU(DPU 미지원)가 학습 후에야 발견된 사고의 재발 방지책.
#
# - cfg가 없으면 yolov5 hub의 yolov3-tiny.yaml 을 복사해
#   `activation: nn.Hardswish()` 를 붙인 커스텀 cfg를 자동 생성한다 (D11).
# - 출력: "[inspect] ..." 요약 + quantize/inspect/inspect_*.txt 리포트
# =============================================================================
set -euo pipefail

# sudo bash 로 통째로 실행해도 경로가 /root 로 튀지 않게 원 사용자 HOME 사용
RUN_HOME="$HOME"
[[ -n "${SUDO_USER:-}" ]] && RUN_HOME="$(getent passwd "$SUDO_USER" | cut -d: -f6)"
BASE="$RUN_HOME/capstone_training"
IMAGE="xilinx/vitis-ai-cpu:2.5.0.1260"
CFG_REL="yolov5/models/yolov3-tiny-hswish.yaml"   # BASE 기준 상대경로

[[ -d "$BASE/yolov5" ]] || { echo "yolov5 repo 없음 — 10_setup_training.sh 먼저"; exit 1; }
[[ -f "$BASE/arch_b3136.json" ]] || {
  echo "arch_b3136.json 없음 — 보드에서 복사: scp ubuntu@192.168.120.132:~/ros2_ws/src/vitis_ai_detector_pkg/models/arch_b3136.json $BASE/"; exit 1; }

if [[ ! -f "$BASE/$CFG_REL" ]]; then
  echo "== cfg 생성: $CFG_REL (yolov3-tiny + activation: nn.Hardswish(), D11) =="
  cp "$BASE/yolov5/models/hub/yolov3-tiny.yaml" "$BASE/$CFG_REL"
  printf '\nactivation: nn.Hardswish()\n' >> "$BASE/$CFG_REL"
fi

sudo docker run --rm -v "$BASE":/workspace -w /workspace "$IMAGE" bash -lc "
set -e
conda activate vitis-ai-pytorch
# VAI 2.5 배포 버그 패치 2건 — 상세는 12_run_quantize_docker.sh 주석 참고
sed -i 's/self.fix_neuron = FixNeuronWithBackward()/self.fix_neuron = None/' \
  /opt/vitis_ai/conda/envs/vitis-ai-pytorch/lib/python3.7/site-packages/pytorch_nndct/nn/modules/hardswish.py
sed -i 's/fake_quantize_per_tensor(output, scale_inv=128, zero_point=0, quant_min=-128, quant_max=127)/fake_quantize_per_tensor(output, scale_inv=128, zero_point=0, quant_min=-128, quant_max=127, method=2, inplace=False)/' \
  /opt/vitis_ai/conda/envs/vitis-ai-pytorch/lib/python3.7/site-packages/pytorch_nndct/nn/modules/hardswish.py
pip install -q pyyaml pandas requests tqdm matplotlib seaborn 'numpy<2' 2>/dev/null || \
pip install -q pyyaml pandas requests tqdm matplotlib seaborn
python training/scripts/12_quantize.py --inspect --cfg '/workspace/$CFG_REL'
"

echo ""
echo "판정: 위 출력의 '[cfg] Conv activation = Hardswish' 와"
echo "      '[inspect] 리포트에 CPU 배정 없음' 이 둘 다 보이면 Gate 0 통과 → bash 11_train.sh"
