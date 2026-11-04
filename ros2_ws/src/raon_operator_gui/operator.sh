#!/usr/bin/env bash
# RAON operator station launcher — PC side.
#
#   ./operator.sh                 실행만 (빌드돼 있으면 바로 launch)
#   ./operator.sh --update        보드에서 최신 소스 scp → 클린 재빌드 → launch
#   ./operator.sh urdf:=...       나머지 인자는 ros2 launch에 그대로 전달
#
# 배치: ~/pp_ws/viewer_ws/ 루트에 복사해 두거나, scp된
# src/raon_operator_gui/operator.sh 를 그대로 실행해도 된다 — 스스로
# 워크스페이스를 찾는다. BOARD 환경변수로 보드 주소 덮어쓰기 가능.

# Whole body in main(): bash reads scripts INCREMENTALLY, and --update
# replaces this very file under src/ — without the wrapper, overwriting a
# running script executes garbage half-lines (tools/ab_one_run.sh already
# taught this project that lesson the hard way).
main() {
  # no `set -u`: ROS's setup.bash reads unbound variables
  set -eo pipefail

  local BOARD="${BOARD:-ubuntu@192.168.120.50}"
  local PKG=raon_operator_gui

  # self-locate: either <ws>/operator.sh (copied) or <ws>/src/<pkg>/operator.sh
  local SELF HERE WS
  SELF="$(readlink -f "$0")"
  HERE="$(cd "$(dirname "$SELF")" && pwd)"
  if [[ -f "$HERE/package.xml" ]]; then WS="$(cd "$HERE/../.." && pwd)"; else WS="$HERE"; fi
  cd "$WS"

  local DO_BUILD=0
  if [[ "${1:-}" == "--update" ]]; then
    shift
    echo ">>> updating $PKG from $BOARD"
    rm -rf "src/$PKG" "build/$PKG" "install/$PKG"
    scp -rq "$BOARD:~/ros2_ws/src/$PKG" src/
    # meshes are the piece a truncated scp loses first — refuse to build a
    # console that would silently render nothing
    [[ -f "src/$PKG/meshes/indy7/visual/Indy7_6.stl" ]] \
      || { echo "!!! scp incomplete — meshes missing"; exit 1; }
    DO_BUILD=1
  fi

  source /opt/ros/humble/setup.bash
  [[ -f "install/$PKG/share/$PKG/launch/operator.launch.py" ]] || DO_BUILD=1

  if [[ "$DO_BUILD" == 1 ]]; then
    colcon build --packages-select "$PKG"
  fi

  source "install/setup.bash"
  exec ros2 launch "$PKG" operator.launch.py "$@"
}

main "$@"
