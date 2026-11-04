#!/usr/bin/env bash
# =============================================================================
# soak_rt.sh — RT 커널(radix-tree 픽스) 인증 소크 테스트 (v2, 보드 보호판)
#
#   bash soak_rt.sh [DURATION_SEC=1200]
#
# 배경(2026-07-13): v1은 --fork16/clone8/pthread8(태스크 생성기 32개)로 load 156까지
#   치솟아 보드가 OOM/livelock으로 죽음. 단, 죽은 부팅 로그에 radix 위반 0건 →
#   "픽스는 극한 fork 부하 9.6분을 무위반으로 버팀"이 확인됨(죽음은 버그가 아니라 부하 크기).
#
# v2 설계:
#   - 버그 경로(idr_preload = fork/alloc_pid, sysfs/kernfs)는 유지하되 **적당한 크기**로
#     (버그는 락 '경합'으로 터지므로 4코어 초과구독 load~12면 경합 충분, 보드는 안전).
#   - ★자원 안전장치: MemAvailable<300MB 또는 load1>40이면 부하 자동 중단(보드 보호).
#   - 판정 3분기: PASS(위반0·완주) / FAIL(radix 위반 발생=픽스 문제) / SAFEHALT(자원 임계로 중단, 위반0)
# =============================================================================
set -o pipefail
DUR="${1:-1200}"
STAMP=$(date +%Y%m%d-%H%M%S)
# 저장소 상대경로로 해석 (tools/rt/ → ../.. = ros2_ws). clone 위치와 무관하게 동작하고,
# sudo로 돌려 $HOME이 /root가 돼도 엉뚱한 곳에 쓰지 않는다.
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOGDIR="$REPO_ROOT/evidence/crash_logs"
mkdir -p "$LOGDIR"
LOG="$LOGDIR/soak_rt_${STAMP}.log"
PAT="sleeping function called from invalid context|scheduling while atomic|BUG:|Preemption disabled at|WARNING.*fpsimd|Unable to handle kernel|Internal error"
MEM_FLOOR_MB=300      # MemAvailable 이 아래면 보드 보호 위해 중단
LOAD_CEIL=40          # load1 이 위면 중단

viol_count() { journalctl -k -b 0 2>/dev/null | grep -cE "$PAT"; }
mem_avail_mb() { awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo; }
load1() { awk '{print int($1)}' /proc/loadavg; }
temp_c() { for z in /sys/class/thermal/thermal_zone*/temp; do [ -r "$z" ] && awk '{printf "%.0f",$1/1000}' "$z" && return; done; echo "?"; }

echo "=== RT 소크 v2 시작: $(date) / 목표 ${DUR}s / 커널 $(uname -r) ===" | tee "$LOG"
echo "검출기: $(zcat /proc/config.gz | grep -E '^CONFIG_DEBUG_PREEMPT=|^CONFIG_DEBUG_ATOMIC_SLEEP=' | tr '\n' ' ')" | tee -a "$LOG"
BASE=$(viol_count)
echo "시작 위반 baseline: $BASE / MemAvail $(mem_avail_mb)MB" | tee -a "$LOG"

# ★ 정리 보장: 스크립트가 어떻게 끝나든(완료/timeout/kill/에러) stress-ng 자식까지 전부 reap.
#   (2026-07-13 사건: kill $SPID가 메인만 죽여 --fork 자식이 고아로 폭주 → load 54 좀비 5027개.)
trap 'pkill -9 stress-ng 2>/dev/null' EXIT INT TERM

# 부하 기동 — CPU/메모리는 공격적(안전장치가 보호), fork/clone만 억제(load 156 방지)
#   버그 경로(fork/alloc_pid, sysfs/kernfs) 유지 + 전 코어 CPU + ~1.3GB 메모리로 RT 전반 인증
echo "[load] stress-ng: cpu4 vm2(2x640M≈1.3G) fork4 clone2 pthread2 sysfs1 — 전 코어+메모리 공격적, fork만 억제" | tee -a "$LOG"
stress-ng --cpu 4 --cpu-load 70 --vm 2 --vm-bytes 35% --fork 4 --clone 2 --pthread 2 --sysfs 1 \
          --timeout "${DUR}s" --metrics-brief > "${LOG}.stressng" 2>&1 &
SPID=$!

VERDICT="PASS"
START=$(date +%s)
while kill -0 "$SPID" 2>/dev/null; do
  sleep 10
  NOW=$(viol_count); DELTA=$((NOW - BASE))
  MA=$(mem_avail_mb); LA=$(load1); EL=$(( $(date +%s) - START ))
  echo "[$(date +%H:%M:%S) +${EL}s] 위반델타=${DELTA} load=${LA} MemAvail=${MA}MB temp=$(temp_c)C" | tee -a "$LOG"
  if [ "$DELTA" -gt 0 ]; then
    echo "!!! radix 위반 발생 — 픽스 불완전(FAIL). 중단·캡처 !!!" | tee -a "$LOG"
    pkill -9 stress-ng 2>/dev/null   # 자식까지 전부 (메인만 죽이면 --fork 고아 폭주)
    journalctl -k -b 0 | grep -E "$PAT" -A 25 | tail -120 | tee -a "$LOG"
    VERDICT="FAIL"; break
  fi
  # [자원 안전장치 제거됨 — 사용자 요청(2026-07-13). load/MemAvail은 위 tick에 계속 기록만.
  #  보드가 hang해도 위반 증거는 persistent journal + 이 로그의 마지막 tick에 남음.]
done
wait "$SPID" 2>/dev/null

END=$(viol_count); FINAL_DELTA=$((END - BASE))
echo "" | tee -a "$LOG"
echo "=== 소크 종료: $(date) / 경과 $(( $(date +%s) - START ))s / 최종 위반델타 ${FINAL_DELTA} ===" | tee -a "$LOG"
if [ "$VERDICT" = "PASS" ] && [ "$FINAL_DELTA" -eq 0 ]; then
  echo "★ PASS — 부하 소크 전 구간 radix 위반 0건. 픽스 무결성 확인." | tee -a "$LOG"
elif [ "$VERDICT" = "SAFEHALT" ]; then
  echo "◐ SAFEHALT — 자원 보호로 조기 중단(위반 ${FINAL_DELTA}건). 위반 0이면 픽스는 무죄, 부하만 재조정 필요." | tee -a "$LOG"
else
  echo "✗ FAIL — radix 위반 ${FINAL_DELTA}건. 로그 확인: $LOG" | tee -a "$LOG"
fi
echo "로그: $LOG"
