#!/usr/bin/env bash
#
# viewer.sh — [데스크톱/연구실 PC에서 실행] KV260 검출 bbox overlay 뷰어 기동
#
# ⚠️ 이 스크립트는 보드가 아니라 PC(jaehyeon@jaehyeon-Raimlab)에서 실행한다.
# 보드에서 pick_place_vitis_ai.launch.py 가 돌고 있는 상태에서 이걸 실행하면
# /camera/.../compressed + /detections 를 받아 stamp 조인 후 bbox overlay 를 화면에 띄운다.
# 종료: 창에서 q/ESC 또는 터미널 Ctrl-C (종료 시 FPS CSV 가 자동 저장됨).
#
# 사용법 (PC에서):
#   ./viewer.sh                              # 기본 기동
#   ./viewer.sh --buffer 60                  # 뷰어 노드 인자를 그대로 전달
#   ./viewer.sh --image-topic /foo/compressed --detections-topic /bar
#   VIEWER_WS=/다른/경로 ./viewer.sh          # 워크스페이스 경로 override
#
# 전제 (docs/vision/desktop_viewer_plan.md §5,§6):
#   - my_interfaces + detection_viewer_pkg 를 ~/pp_ws/viewer_ws/src 에 scp 후 colcon build 완료.
#   - 보드에서 파이프라인이 돌고 있고, PC와 같은 네트워크 / 같은 domain 0.
#   - DDS env(RMW/domain/profile)는 전부 기본값이어야 함 → 이 스크립트가 깨끗한 상태로 실행한다.

# --- 뷰어 워크스페이스 (기본 ~/pp_ws/viewer_ws, VIEWER_WS 환경변수로 override) ---
WS="${VIEWER_WS:-$HOME/pp_ws/viewer_ws}"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="${WS}/install/setup.bash"
PKG="detection_viewer_pkg"
NODE="detection_viewer_node"
BOARD="ubuntu@192.168.120.132"          # 최초 셋업 안내용

# --- pre-flight ---
if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "[viewer.sh] ERROR: ROS 2 Humble 을 찾을 수 없습니다: ${ROS_SETUP}" >&2
    exit 1
fi
if [[ ! -f "${WS_SETUP}" ]]; then
    echo "[viewer.sh] ERROR: 뷰어 워크스페이스가 빌드되지 않았습니다: ${WS_SETUP}" >&2
    echo "[viewer.sh] 최초 셋업 (desktop_viewer_plan.md §5):" >&2
    echo "    mkdir -p ${WS}/src" >&2
    echo "    scp -r ${BOARD}:~/ros2_ws/src/{my_interfaces,detection_viewer_pkg} ${WS}/src/" >&2
    echo "    cd ${WS} && colcon build --packages-select my_interfaces detection_viewer_pkg --symlink-install" >&2
    echo "    # .msg 무결성 대조(양쪽 값이 같아야 함): md5sum ${WS}/src/my_interfaces/msg/*.msg" >&2
    exit 1
fi

# --- DDS env 는 반드시 '기본값(비어있음)'이어야 보드(fastrtps, domain 0)와 붙는다 ---
#     스테일 RMW/profile 이 셸에 남아 있으면 조용히 안 붙거나 CPU 만 낭비된다
#     (desktop_viewer_plan.md §5, §10 #2·#3). subshell 안에서만 비우므로 사용자 셸엔 영향 없음.
unset RMW_IMPLEMENTATION
unset ROS_DOMAIN_ID
unset FASTRTPS_DEFAULT_PROFILES_FILE

# --- 환경 source (ROS setup.bash 는 'set -u' 에서 깨지므로 -u 는 쓰지 않음) ---
echo "[viewer.sh] source ${ROS_SETUP}"
# shellcheck source=/dev/null
source "${ROS_SETUP}"
echo "[viewer.sh] source ${WS_SETUP}"
# shellcheck source=/dev/null
source "${WS_SETUP}"

# --- 기동 ---
echo "[viewer.sh] RMW=${RMW_IMPLEMENTATION:-(default fastrtps)}  DOMAIN=${ROS_DOMAIN_ID:-0}"
echo "[viewer.sh] 보드에서 pick_place_vitis_ai.launch.py 가 돌고 있어야 화면이 뜹니다."
echo "[viewer.sh] ros2 run ${PKG} ${NODE} $*"
echo "[viewer.sh] 종료: 창에서 q/ESC 또는 Ctrl-C (FPS CSV 자동 저장)"
echo
exec ros2 run "${PKG}" "${NODE}" "$@"
