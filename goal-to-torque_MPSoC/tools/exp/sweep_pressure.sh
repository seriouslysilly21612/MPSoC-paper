#!/bin/bash
# 11-27 압력 보정 sweep: FULL, S2, bench -b, 단계마다 150 s (warm-up 120 s 별도)
# 단계 순서: 상한 없음(I-108 확인 겸) -> 0 -> 200 -> 400 -> 800 -> 1200 MB/s (worker당 상한)
# 사용: cd ~/MPSoC-paper/goal-to-torque_MPSoC && sudo -v && bash ~/sweep_pressure.sh [단계...]
#   예) bash ~/sweep_pressure.sh unb        # I-108 확인만 먼저
#       bash ~/sweep_pressure.sh 0 200 400 800 1200
set -u
cd ~/MPSoC-paper/goal-to-torque_MPSoC
OUT=~/timing_logs/sweep_20261127
M320=~/models/compiled_pickplace_v5n_5class_hswish_real_320/yolov5n_5class_320.xmodel
mkdir -p "$OUT"
STEPS=("$@"); [ ${#STEPS[@]} -eq 0 ] && STEPS=(unb 0 200 400 800 1200)

for s in "${STEPS[@]}"; do
  case $s in
    unb) LOAD=(--pressure 100) ;;
    0)   LOAD=(--pressure 0) ;;
    *)   LOAD=(--memrate-mbs "$s") ;;
  esac
  D="$OUT/p_$s"
  echo "=== step $s -> $D  $(date +%T)"
  sudo -n true || { echo "sudo 시간 만료: sudo -v 다시"; exit 1; }
  # DDR APM: run 전체 동안 0.25 s마다 (warm-up 포함, 분석 때 TRACK 창만 자른다)
  mkdir -p "$D"
  sudo -n python3 tools/host/ddr_apm.py --out "$D/apm.csv" --period 0.25 > "$D/apm.txt" 2>&1 &
  APM=$!
  python3 tools/run_timing.py --condition pressure "${LOAD[@]}" \
      --arch S2 --method FULL --bus indy7/bench/L7N3_bus.cfg \
      --model-2 "$M320" --desired-class apple \
      --duration 150 --no-hl-log --out "$D" 2>&1 | tee "$D/console.txt"
  sudo -n kill -INT $APM 2>/dev/null; wait $APM 2>/dev/null
  echo "=== step $s done $(date +%T)"; tail -3 "$D/apm.txt"
  sleep 20
done
