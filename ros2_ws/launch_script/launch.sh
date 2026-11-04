#!/usr/bin/env bash
#
# launch.sh — Kria KV260 비전 pick & place 파이프라인 원터치 기동 스크립트
#
# 우분투 로그인 후 이 스크립트 하나만 실행하면 전체 perception 파이프라인이 뜹니다:
#   camera → vitis_ai_detector → pick_logic → pick_target_3d → static TF → pick_target_base
# 종료: Ctrl-C
#
# 사용법:
#   ~/ros2_ws/launch_script/launch.sh                 # 기본 기동
#   ~/ros2_ws/launch_script/launch.sh metrics:=true   # metrics 기록 켜고 기동
#   (모든 launch 인자는 그대로 ros2 launch 로 전달됩니다)
#
# 전제:
#   - 워크스페이스가 빌드돼 있어야 함(install/setup.bash 존재). 없으면 빌드 방법을 안내합니다.
#   - DPU smartcam overlay 는 부팅 시 systemd(kv260-smartcam.service)가 자동 로드하므로
#     이 스크립트에서 따로 로드하지 않습니다.
#   - DDS SHM 프로파일 등 환경변수는 launch 파일이 내부에서 설정하므로 여기서 안 건드립니다.

# --- 경로: 스크립트 위치에서 워크스페이스 루트를 유도 (repo 위치가 바뀌어도 동작) ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname "${SCRIPT_DIR}")"                 # launch_script 의 부모 = 워크스페이스 루트
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="${WS}/install/setup.bash"
LAUNCH_PKG="system_bringup_pkg"
LAUNCH_FILE="pick_place_vitis_ai.launch.py"

# --- pre-flight 점검 ---
if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "[launch.sh] ERROR: ROS 2 Humble 을 찾을 수 없습니다: ${ROS_SETUP}" >&2
    exit 1
fi
if [[ ! -f "${WS_SETUP}" ]]; then
    echo "[launch.sh] ERROR: 워크스페이스가 아직 빌드되지 않았습니다: ${WS_SETUP}" >&2
    echo "[launch.sh] 먼저 빌드하세요:" >&2
    echo "    cd ${WS} && source ${ROS_SETUP} && colcon build --symlink-install" >&2
    exit 1
fi

# --- ROS 2 + 워크스페이스 환경 source ---
#     (ROS 의 setup.bash 는 'set -u' 에서 unbound 변수로 깨지므로 -u 는 쓰지 않습니다)
echo "[launch.sh] source ${ROS_SETUP}"
# shellcheck source=/dev/null
source "${ROS_SETUP}"
echo "[launch.sh] source ${WS_SETUP}"
# shellcheck source=/dev/null
source "${WS_SETUP}"

# --- 파이프라인 기동 (exec: Ctrl-C 등 시그널이 ros2 launch 로 바로 전달되도록) ---
echo "[launch.sh] ros2 launch ${LAUNCH_PKG} ${LAUNCH_FILE} $*"
echo "[launch.sh] 종료하려면 Ctrl-C 를 누르세요."
echo
exec ros2 launch "${LAUNCH_PKG}" "${LAUNCH_FILE}" "$@"
