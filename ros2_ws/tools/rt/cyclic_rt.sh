#!/usr/bin/env bash
# =============================================================================
# cyclic_rt.sh — kv260b RT 지연(cyclictest) 측정 + 부하 중 커널 위반 감시
#
#   sudo bash cyclic_rt.sh [IDLE_SEC=120] [LOAD_SEC=300]
#
# 2단계:
#   [1] IDLE baseline  — 부하 없이 최선(best-case) 지연
#   [2] LOADED         — stress-ng(cpu/io/vm) + hackbench 루프(스케줄러 churn)로
#                        최악(worst-case) 지연 유발. RT 인증의 진짜 숫자.
# cyclictest는 SCHED_FIFO p90 → 일반 프로세스(이 세션 포함) preempt하므로 측정 보호됨.
# 동시에 DEBUG_ATOMIC_SLEEP 검출기로 radix/zocl/BUG 위반이 0인지 확인(검증 겸용).
# 주의: kv260b는 DEBUG 커널이라 수치가 프로덕션(rev-6, DEBUG off)보다 비관적(높게) 나옴.
# =============================================================================
set -o pipefail
IDLE=${1:-120}; LOAD=${2:-300}
# 저장소 상대경로로 해석 (tools/rt/ → ../.. = ros2_ws). sudo로 돌리므로 $HOME은 쓸 수 없다.
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOGDIR="$REPO_ROOT/evidence/crash_logs"
STAMP=$(date +%Y%m%d-%H%M%S)
LOG="$LOGDIR/cyclic_${STAMP}.log"
PAT="sleeping function called from invalid context|scheduling while atomic|BUG:|Preemption disabled at|WARNING.*fpsimd|Unable to handle kernel|Internal error"
viol(){ journalctl -k -b 0 2>/dev/null | grep -cE "$PAT"; }
maxlat(){ grep -E "^T:" "$1" | sed -E 's/.*Max:[[:space:]]*([0-9]+).*/\1/' | sort -n | tail -1; }

mkdir -p "$LOGDIR"
echo "=== cyclictest RT 측정: $(date) / 커널 $(uname -r) / 코어 $(nproc) ===" | tee "$LOG"
echo "검출기 무장(2=DEBUG_PREEMPT+ATOMIC_SLEEP): $(zcat /proc/config.gz 2>/dev/null | grep -cE 'CONFIG_DEBUG_(PREEMPT|ATOMIC_SLEEP)=y')" | tee -a "$LOG"
echo "cmdline: $(cat /proc/cmdline)" | tee -a "$LOG"
BASE=$(viol); echo "시작 위반 baseline: $BASE" | tee -a "$LOG"

# --- 정리 보장 ---
trap 'pkill -9 stress-ng 2>/dev/null; pkill -9 hackbench 2>/dev/null' EXIT INT TERM

echo "" | tee -a "$LOG"
echo "--- [1/2] IDLE baseline ${IDLE}s (부하 없음) ---" | tee -a "$LOG"
I1="$LOG.idle"
cyclictest -m -S -p90 -i200 -D "$IDLE" -q > "$I1" 2>&1
grep -E "^T:" "$I1" | tee -a "$LOG"
echo ">>> IDLE 최대지연: $(maxlat "$I1") us" | tee -a "$LOG"

echo "" | tee -a "$LOG"
echo "--- [2/2] LOADED ${LOAD}s (stress-ng cpu4 io2 vm1(20%) + hackbench 루프) ---" | tee -a "$LOG"
stress-ng --cpu 4 --io 2 --vm 1 --vm-bytes 20% --timeout "$((LOAD+15))s" >/dev/null 2>&1 &
( end=$((SECONDS+LOAD)); while [ "$SECONDS" -lt "$end" ]; do hackbench -l 100 >/dev/null 2>&1; done ) &
sleep 3   # 부하 예열
L1="$LOG.load"
cyclictest -m -S -p90 -i200 -D "$LOAD" -q > "$L1" 2>&1
pkill -9 stress-ng 2>/dev/null; pkill -9 hackbench 2>/dev/null; sleep 2
grep -E "^T:" "$L1" | tee -a "$LOG"
echo ">>> LOADED 최대지연: $(maxlat "$L1") us" | tee -a "$LOG"

END=$(viol); DELTA=$((END-BASE))
echo "" | tee -a "$LOG"
echo "======================================================" | tee -a "$LOG"
echo " IDLE   최대지연: $(maxlat "$I1") us" | tee -a "$LOG"
echo " LOADED 최대지연: $(maxlat "$L1") us   ← RT 인증 핵심 숫자" | tee -a "$LOG"
echo " 부하 중 커널 위반델타: ${DELTA}  (0이어야 정상)" | tee -a "$LOG"
echo " 참고: EtherCAT 1kHz 여유 기준 통상 Max<100us. kv260b는 DEBUG 커널이라 rev-6보다 높게 나옴." | tee -a "$LOG"
echo "======================================================" | tee -a "$LOG"
echo "로그: $LOG (+ .idle/.load 원본)"
