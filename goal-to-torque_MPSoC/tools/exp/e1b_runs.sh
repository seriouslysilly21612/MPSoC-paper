#!/bin/bash
# 11-27 E1b: 압력 3단계 x {M2, FULL 고정 L_m, FULL 동적 L_m} = 9 run, 각 300 s (warm-up 120 s 별도)
# 사용: cd ~/MPSoC-paper/goal-to-torque_MPSoC && sudo -v && LEVELS="L M H" bash ~/e1b_runs.sh
#   LEVELS: worker당 memrate 상한 MB/s 3개 (0 = memrate 없음). sweep 결과로 정한 값을 넣는다.
#   예) LEVELS="0 800 1600" bash ~/e1b_runs.sh
#   한 run만 다시: ONLY="800:FULLdyn" LEVELS="800" bash ~/e1b_runs.sh
set -u
cd ~/MPSoC-paper/goal-to-torque_MPSoC
OUT=~/timing_logs/e1b_20261127
M320=~/models/compiled_pickplace_v5n_5class_hswish_real_320/yolov5n_5class_320.xmodel
read -r -a LV <<< "${LEVELS:?LEVELS=\"a b c\" 를 주세요}"
mkdir -p "$OUT"
ls /dev/uio* | grep -q . || { echo "UIO 없음: r5_record overlay 먼저"; exit 1; }
for lv in "${LV[@]}"; do
  if [ "$lv" = 0 ]; then LOAD=(--pressure 0); else LOAD=(--memrate-mbs "$lv"); fi
  for m in M2 FULLfix FULLdyn; do
    [ -n "${ONLY:-}" ] && [ "$ONLY" != "$lv:$m" ] && continue
    case $m in
      M2)      METH=(--method M2);   VARGS="" ;;
      FULLfix) METH=(--method FULL); VARGS="dynamic_latency:=false" ;;
      FULLdyn) METH=(--method FULL); VARGS="dynamic_latency:=true" ;;
    esac
    D="$OUT/p${lv}_$m"
    echo "=== $lv MB/s $m -> $D  $(date +%T)"
    sudo -n true || { echo "sudo 시간 만료: sudo -v 다시"; exit 1; }
    mkdir -p "$D"
    sudo -n python3 tools/host/ddr_apm.py --out "$D/apm.csv" --period 0.25 > "$D/apm.txt" 2>&1 &
    APM=$!
    python3 tools/run_timing.py --condition pressure "${LOAD[@]}" \
        --arch S2 "${METH[@]}" --bus indy7/bench/L7N3_bus.cfg \
        --model-2 "$M320" --desired-class apple \
        ${VARGS:+--vision-args "$VARGS"} \
        --duration 300 --out "$D" 2>&1 | tee "$D/console.txt"
    sudo -n kill -INT $APM 2>/dev/null; wait $APM 2>/dev/null
    echo "=== $lv $m done $(date +%T)"; tail -2 "$D/apm.txt"
    sleep 20
  done
done
