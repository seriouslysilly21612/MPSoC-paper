#!/usr/bin/env bash
# metrics run 오케스트레이터 — 호출 1회 = run 1개 (자동 기동→감시→종료→검증)
# usage: ab_one_run.sh <label> [extra launch args...]
#
# 2026-07-31 align_depth A/B 실험(evidence/metrics/runs/ab_compare_20260731.md)을
# 돌린 스크립트. A/B 스위치(camera_config:=aligned ...)는 실험 종료 후 코드에서
# 원복됐으므로 aligned run 은 이 파일이 추가된 커밋을 복원해야 한다.
# production metrics run 은 지금 코드로 그대로 쓸 수 있다:
#   bash tools/metrics/ab_one_run.sh my_label
LABEL="$1"; shift
WS=/home/ubuntu/ros2_ws
LOG="/tmp/ab_${LABEL}.log"
RUN_DIR="$WS/evidence/metrics/runs/$LABEL"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "[$(date +%T)] run '$LABEL' 기동 (extra: $*)"
# set -m: 비대화형 셸에서 &로 띄우면 SIGINT가 SIG_IGN으로 상속돼 kill -INT가
# 조용히 씹힌다(run A에서 실증 — TERM 승급 + 고아 노드 발생). job control을
# 켜면 launch가 자기 프로세스 그룹을 갖고 기본 시그널 처리로 시작되므로,
# 그룹 전체(-PID)에 INT를 보내면 터미널 Ctrl-C와 동일하게 동작한다.
set -m
# sigterm_timeout=20: 기본 5초는 노드 finally(마감 저장+yield 기록)가 끝나기 전에
# SIGTERM으로 끊어 CSV 재저장을 도중 절단시켰다(B 1차에서 실증: base 3972→1086행).
ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py \
    metrics:=true "metrics_run:=$LABEL" \
    sigterm_timeout:=20 sigkill_timeout:=10 "$@" >"$LOG" 2>&1 &
LPID=$!
set +m
echo "[$(date +%T)] launch pid=$LPID (pgid=$LPID)"

# ---- 헬스 게이트: 90초 안에 첫 프레임(=기록 시작)이 와야 한다 ----
# 노드들이 첫 row/frame에서 "...s from the first frame/row" 를 찍는다.
started=0
for _ in $(seq 1 45); do
    sleep 2
    kill -0 "$LPID" 2>/dev/null || { echo "[abort] launch 조기 종료"; tail -40 "$LOG"; exit 1; }
    grep -qE "from the first (frame|row)" "$LOG" && { started=1; break; }
done
if [ "$started" -ne 1 ]; then
    echo "[abort] 90초 내 프레임 없음 — USB/UVC wedge 의심. 콘솔 tail:"
    tail -25 "$LOG"
    echo "---- kernel uvc tail ----"
    journalctl -k --since "-3 min" --no-pager 2>/dev/null | grep -iE "uvc|usb 2-" | tail -8
    kill -INT -- "-$LPID" 2>/dev/null; sleep 10
    kill -TERM -- "-$LPID" 2>/dev/null; sleep 5
    kill -KILL -- "-$LPID" 2>/dev/null
    exit 2
fi
echo "[$(date +%T)] 기록 시작 확인 (첫 프레임 수신)"

# ---- 300초 metrics window 완료 대기 (상한 420초) ----
completed=0
for _ in $(seq 1 210); do
    sleep 2
    grep -qE "Metrics window .*complete" "$LOG" && { completed=1; break; }
    kill -0 "$LPID" 2>/dev/null || break
done
echo "[$(date +%T)] window_complete=$completed — 15초 유예 후 SIGINT"
sleep 15   # 나머지 노드들의 window 마감 + sampler flush 유예

# launch 프로세스에게만 SIGINT: launch가 자식들에게 SIGINT를 전파하므로 각
# 노드는 정확히 1발만 받는다. 그룹(-PID)에 쏘면 노드가 (그룹+launch전파) 2발을
# 받아 finally 도중 2차 KeyboardInterrupt로 CSV가 절단됐다(2026-07-31 실증).
kill -INT "$LPID" 2>/dev/null
for _ in $(seq 1 25); do sleep 2; kill -0 "$LPID" 2>/dev/null || break; done
if kill -0 "$LPID" 2>/dev/null; then
    echo "TERM 승급(그룹)"; kill -TERM -- "-$LPID" 2>/dev/null; sleep 8
fi
if kill -0 "$LPID" 2>/dev/null; then
    echo "KILL 승급(그룹)"; kill -KILL -- "-$LPID" 2>/dev/null; sleep 2
fi
wait "$LPID" 2>/dev/null
# 고아 노드 잔존 확인 — 있으면 다음 run 을 오염시키므로 반드시 정리.
# 우리 프로세스 그룹은 제외한다: pgrep -f 는 커맨드라인에 노드 "파일명"만 들어
# 있어도 잡아서, 예컨대 'flake8 ... pick_target_3d_node.py && ab_one_run.sh' 로
# 이 스크립트를 띄운 부모 셸을 죽였다(2026-08-04 실증, run 이 exit 144 로 끝남).
# 위 set -m 덕에 진짜 노드들은 launch 의 그룹(LPID)에 있고 우리 그룹에는 절대
# 없으므로, PGID 비교 하나로 조상도 자기 서브셸도 한꺼번에 걸러진다.
NODE_PAT="[r]ealsense2_camera_node|[v]itis_ai_detector_node|[p]ick_logic|[p]ick_target_3d_node|[p]ick_target_base_node|[v]itis_ai_worker_yolo"
MYPG=$(ps -o pgid= -p $$ | tr -d ' ')
find_strays() {
    pgrep -f "$NODE_PAT" | while read -r pid; do
        pg=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
        [ -n "$pg" ] && [ "$pg" != "$MYPG" ] && echo "$pid"
    done
}
strays=$(find_strays)
if [ -n "$strays" ]; then
    echo "고아 프로세스 정리: $strays"
    kill -TERM $strays 2>/dev/null; sleep 8
    strays2=$(find_strays)
    [ -n "$strays2" ] && { kill -KILL $strays2 2>/dev/null; sleep 2; }
fi

# ---- 산출물 검증 ----
np=$(ls "$RUN_DIR/performance/"*.csv 2>/dev/null | wc -l)
ny=$(ls "$RUN_DIR/yield/"*.csv 2>/dev/null | wc -l)
nr=$([ -f "$RUN_DIR/resource/run.csv" ] && wc -l <"$RUN_DIR/resource/run.csv" || echo 0)
cp "$LOG" "$RUN_DIR/launch_console.log" 2>/dev/null
echo "RESULT label=$LABEL window_complete=$completed perf_csv=$np yield_csv=$ny resource_rows=$nr"
if [ "$np" -ge 4 ] && [ "$ny" -ge 4 ]; then exit 0; else exit 3; fi
