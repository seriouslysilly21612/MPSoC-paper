#!/usr/bin/env bash
#
# plots.sh — 측정 CSV로부터 논문형 그래프(plots_paper) 생성
#
# metrics:=true 로 파이프라인을 돌려 CSV를 받은 뒤, 이 스크립트 하나만 실행하면
# evidence/metrics/latest/plots_paper/ 에 figure 8장이 생깁니다.
#
# ⚠️ 이건 ROS 프로그램이 아닙니다 — 순수 오프라인 CSV→PNG 변환입니다.
#    setup.bash source 도, 파이프라인 실행도, colcon build 도 필요 없습니다.
#    보드에서 바로 돌아갑니다(numpy 1.21.5 + matplotlib 3.5.1 이미 설치됨).
#    파이프라인이 돌고 있든 꺼져 있든 무관합니다.
#
# 대상은 기본적으로 **가장 최근 run**(evidence/metrics/latest)입니다.
#
# 사용법:
#   ~/ros2_ws/launch_script/plots.sh              # 최신 run, 논문형 8장 (기본)
#   ~/ros2_ws/launch_script/plots.sh --list       # 기록된 run 목록 보기
#   ~/ros2_ws/launch_script/plots.sh --run 20260722_231246   # 특정 run 지정
#   ~/ros2_ws/launch_script/plots.sh --all        # 모든 run 에 대해 생성
#   ~/ros2_ws/launch_script/plots.sh --quick      # 간이 대시보드 3장 (numpy 불필요)
#   ~/ros2_ws/launch_script/plots.sh --both       # 논문형 + 간이 둘 다
#   ~/ros2_ws/launch_script/plots.sh --base /tmp/metrics    # 다른 데이터셋 지정
#   (그 밖의 인자는 그대로 파이썬 스크립트에 전달됩니다)
#
# 산출물:
#   <base>/plots_paper/   1_latency_distribution 2_latency_breakdown 3_latency_timeline
#                         3b_throughput 4_cpu 4b_cpu_cores 5_memory_thermal 6_yield_funnel
#   <base>/plots/         performance / resource / yield        (--quick 일 때)
#
# 입력은 metrics 기록 run 이 만든 것:
#   <base>/performance/{detector,pick_logic,target_3d,base}.csv  (+ merged.csv 있으면 우선 사용)
#   <base>/resource/run.csv , <base>/yield/*.csv
# join_perf.py 를 안 돌렸어도 4개 노드 CSV 로 스스로 join 하므로 그냥 실행하면 됩니다.
#
# 의존: 논문형 = numpy + matplotlib / 간이 = matplotlib 만 (보드에 둘 다 설치돼 있음)
# 자세한 읽는 법: evidence/metrics/README.md

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname "${SCRIPT_DIR}")"                  # launch_script 의 부모 = 워크스페이스 루트
METRICS="${WS}/evidence/metrics"                 # 측정 데이터는 evidence/ 에 산다(저장소 규약)
PAPER_PY="${WS}/tools/metrics/plot_metrics_paper.py"
QUICK_PY="${WS}/tools/metrics/plot_metrics.py"

# --- 인자 파싱: 아래 플래그는 여기서 소비, 나머지는 파이썬에 전달 ---
MODE="paper"
BASE=""
RUN=""
DO_ALL=0
PASS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --quick) MODE="quick"; shift ;;
        --both)  MODE="both";  shift ;;
        --paper) MODE="paper"; shift ;;
        --base)  BASE="$2"; shift 2 ;;
        --base=*) BASE="${1#*=}"; shift ;;
        --run)   RUN="$2"; shift 2 ;;
        --run=*) RUN="${1#*=}"; shift ;;
        --all)   DO_ALL=1; shift ;;
        --list)
            echo "기록된 run (${METRICS}/runs):"
            if [[ -d "${METRICS}/runs" ]]; then
                cur="$(readlink "${METRICS}/latest" 2>/dev/null | sed 's#^runs/##')"
                for d in "${METRICS}"/runs/*/; do
                    [[ -d "$d" ]] || continue
                    name="$(basename "$d")"
                    n_perf=$(ls "$d/performance"/*.csv 2>/dev/null | wc -l)
                    sz=$(du -sh "$d" 2>/dev/null | cut -f1)
                    mark=""; [[ "$name" == "$cur" ]] && mark="  <- latest"
                    printf "  %-22s  perf CSV %2d개  %6s%s\n" "$name" "$n_perf" "$sz" "$mark"
                done
            else
                echo "  (없음 — 아직 metrics run 을 안 했습니다)"
            fi
            exit 0 ;;
        -h|--help)
            # shebang 다음 줄부터, 주석이 끝나는 지점까지만 출력
            awk 'NR>1 && /^#/ { sub(/^# ?/, ""); print; next } NR>1 { exit }' \
                "${BASH_SOURCE[0]}"
            exit 0 ;;
        *) PASS+=("$1"); shift ;;
    esac
done

# --- 대상 run 결정: --base > --run > latest > (구 flat 레이아웃) ---
if [[ -z "${BASE}" ]]; then
    if [[ -n "${RUN}" ]]; then
        BASE="${METRICS}/runs/${RUN}"
        [[ -d "${BASE}" ]] || {
            echo "[plots.sh] ERROR: 그런 run 이 없습니다: ${RUN}" >&2
            echo "[plots.sh] 목록: $0 --list" >&2; exit 1; }
    elif [[ -d "${METRICS}/latest" ]]; then
        BASE="$(cd "${METRICS}/latest" && pwd -P)"     # symlink 실경로로 해석
    else
        BASE="${METRICS}"                              # 구 flat 레이아웃 하위호환
    fi
fi

# 의존 모듈 확인 — 없을 때 파이썬 traceback 대신 사람이 읽을 안내를 낸다
need_mod() {
    python3 -c "import $1" 2>/dev/null && return 0
    echo "[plots.sh] ERROR: python3 모듈 '$1' 이(가) 없습니다." >&2
    echo "[plots.sh]   설치: pip3 install $1        (또는 sudo apt install python3-$1)" >&2
    [[ "$1" == "numpy" ]] && echo "[plots.sh]   numpy 없이 쓰려면: $0 --quick  (간이 대시보드 3장)" >&2
    return 1
}

# 한 run 디렉토리를 처리한다 (검사 → 생성 → 안내). 0=성공
process_one() {
    local base="$1" rc=0
    if [[ ! -d "${base}" ]]; then
        echo "[plots.sh] ERROR: 데이터 디렉토리가 없습니다: ${base}" >&2
        echo "[plots.sh] 먼저 metrics 를 켜고 파이프라인을 돌리세요:" >&2
        echo "    ${SCRIPT_DIR}/launch.sh metrics:=true" >&2
        return 1
    fi
    if [[ ! -d "${base}/performance" && ! -d "${base}/resource" && ! -d "${base}/yield" ]]; then
        echo "[plots.sh] ERROR: ${base} 안에 performance/ resource/ yield/ 가 없습니다." >&2
        echo "[plots.sh] metrics 기록 run 을 먼저 하세요: ${SCRIPT_DIR}/launch.sh metrics:=true" >&2
        return 1
    fi

    echo "[plots.sh] data base = ${base}"
    if [[ "${MODE}" == "paper" || "${MODE}" == "both" ]]; then
        [[ -f "${PAPER_PY}" ]] || { echo "[plots.sh] ERROR: 없음 ${PAPER_PY}" >&2; return 1; }
        need_mod numpy || return 1
        need_mod matplotlib || return 1
        echo "[plots.sh] 논문형 figure 생성 → ${base}/plots_paper/"
        python3 "${PAPER_PY}" --base "${base}" "${PASS[@]}" || rc=$?
    fi
    if [[ "${MODE}" == "quick" || "${MODE}" == "both" ]]; then
        [[ -f "${QUICK_PY}" ]] || { echo "[plots.sh] ERROR: 없음 ${QUICK_PY}" >&2; return 1; }
        need_mod matplotlib || return 1
        echo
        echo "[plots.sh] 간이 대시보드 생성 → ${base}/plots/"
        python3 "${QUICK_PY}" --base "${base}" "${PASS[@]}" || rc=$?
    fi

    if [[ $rc -eq 0 ]]; then
        # 안내는 실제로 생성한 디렉토리를 가리켜야 한다 (--quick 은 plots/)
        local -a outdirs
        case "${MODE}" in
            quick) outdirs=("${base}/plots") ;;
            both)  outdirs=("${base}/plots_paper" "${base}/plots") ;;
            *)     outdirs=("${base}/plots_paper") ;;
        esac
        echo
        echo "[plots.sh] 완료. PNG 보기:"
        for d in "${outdirs[@]}"; do
            echo "    (VSCode) ${d}/ 에서 파일 열기"
            echo "    (PC로)   scp -r kria:${d} ."
        done
    fi
    return $rc
}

RC=0
if [[ ${DO_ALL} -eq 1 ]]; then
    shopt -s nullglob
    runs=("${METRICS}"/runs/*/)
    shopt -u nullglob
    if [[ ${#runs[@]} -eq 0 ]]; then
        echo "[plots.sh] ERROR: ${METRICS}/runs 에 run 이 없습니다." >&2
        exit 1
    fi
    echo "[plots.sh] --all: ${#runs[@]}개 run 처리"
    for d in "${runs[@]}"; do
        echo; echo "──────── $(basename "${d%/}") ────────"
        process_one "${d%/}" || RC=$?
    done
else
    process_one "${BASE}" || RC=$?
fi
exit $RC
