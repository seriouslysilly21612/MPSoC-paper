#!/bin/sh
# control_replay.sh -- builds test/control_replay.cpp with the firmware's control task (src/ctrl) and the libraries
# from their sources, for this PC and for the R5, and runs both on a file of periods (control_check -o FILE of the
# application; see control_replay.cpp).  From r5_freertos/:
#
#   RBDL_SRC=... RUCKIG_SRC=... sh test/control_replay.sh FILE [BUILD_DIR]
#
# The arguments, the libraries and the build directory are those of test/shadow_replay.sh, which compiles the
# libraries if this script finds none there.
#
# The PC run must agree with the file exactly.  The R5 run (arm-none-eabi-g++, qemu-arm; skipped with a line if one
# is missing) within 1e-9.  Neither may allocate memory in a period.
# Between the two, on the PC: the replay once for every place at which the planner touches a slot, with the step of
# the next period in the middle of it there (SLOT_PREEMPT_TEST, see control_replay.cpp); about 1000 replays.
# Then, with COUNT=N, the instructions of the first N periods on the R5 are counted (test/count_periods.py).
# Exit 1 if a run failed, 2 if the R5 run was not made.
set -e
FILE="$1"; B="${2:-/tmp/shadow_replay}"; EIGEN="${EIGEN_DIR:-/usr/include/eigen3}"
[ -f "$FILE" ] && [ -d "$RBDL_SRC/src" ] && [ -d "$RUCKIG_SRC/src/ruckig" ] || { sed -n '2,17p' "$0"; exit 2; }
APP=../../goal-to-torque_MPSoC/indy7
INC="-Isrc/ctrl/port -Isrc/ctrl -Isrc -Itest -I$APP/src -isystem $RBDL_SRC/include -isystem $EIGEN -isystem $RUCKIG_SRC/include"
OURS="test/control_replay.cpp src/ctrl/control_ctrl.cpp src/ctrl/ControlCycle.cpp src/ctrl/TrajectoryRT.cpp src/ctrl/shadow_ctrl.cpp src/ctrl/ShadowControl.cpp src/ctrl/FullDynControllerRT.cpp src/ctrl/Controller.cpp src/ctrl/HighLevelRT.cpp src/ctrl/ctrl_model.cpp"
R5="-mcpu=cortex-r5 -mfpu=vfpv3-d16 -mfloat-abi=hard -Wno-psabi"
WRAP="-Wl,--wrap=malloc,--wrap=calloc,--wrap=realloc"     # the allocation count of control_replay.cpp

# libs <directory> <compiler> <flags>: the libraries, once (as shadow_replay.sh)
libs() {
    d="$1"; cxx="$2"; flags="$3"
    mkdir -p "$d"
    for f in "$RBDL_SRC"/src/*.cc; do
        o="$d/rbdl_$(basename "$f" .cc).o"
        [ -f "$o" ] || echo "$cxx $flags -O2 -std=gnu++17 -DNDEBUG -w $INC -c $f -o $o"
    done > "$d/jobs"
    for f in "$RUCKIG_SRC"/src/ruckig/*.cpp; do
        case "$f" in *cloud_client*) continue ;; esac
        o="$d/ruckig_$(basename "$f" .cpp).o"
        [ -f "$o" ] || echo "$cxx $flags -O2 -std=gnu++17 -DNDEBUG -w $INC -c $f -o $o"
    done >> "$d/jobs"
    xargs -r -P "$(nproc)" -d '\n' -n 1 sh -c < "$d/jobs"
}
# ours <directory> <prefix of the objects> <compiler> <flags>
ours() {
    for f in $OURS; do
        echo "$3 $4 -O2 -std=gnu++17 -DNDEBUG -Wall $INC -c $f -o $1/$2_$(basename "$f" .cpp).o"
    done | xargs -r -P "$(nproc)" -d '\n' -n 1 sh -c
}

libs "$B/pc" g++ ""
ours "$B/pc" ctl g++ ""
g++ $WRAP "$B"/pc/ctl_*.o "$B"/pc/rbdl_*.o "$B"/pc/ruckig_*.o -o "$B/pc/control_replay"
echo "PC:"
"$B/pc/control_replay" "$FILE" 0 || bad=1

mkdir -p "$B/pre"
ours "$B/pre" ctl g++ "-DSLOT_PREEMPT_TEST"
g++ $WRAP "$B"/pre/ctl_*.o "$B"/pc/rbdl_*.o "$B"/pc/ruckig_*.o -o "$B/pre/control_replay"
echo "PC, the step of a period in the middle of the planner's work:"
"$B/pre/control_replay" "$FILE" 0 || bad=1

if command -v arm-none-eabi-g++ > /dev/null && command -v qemu-arm > /dev/null; then
    libs "$B/r5" arm-none-eabi-g++ "$R5 -ffunction-sections -fdata-sections"
    ours "$B/r5" ctl arm-none-eabi-g++ "$R5 -ffunction-sections -fdata-sections"
    arm-none-eabi-g++ $R5 --specs=rdimon.specs -Wl,--gc-sections $WRAP "$B"/r5/ctl_*.o "$B"/r5/rbdl_*.o "$B"/r5/ruckig_*.o \
        -o "$B/r5/control_replay.elf"
    echo "R5 (qemu-arm):"
    qemu-arm -cpu cortex-r5f "$B/r5/control_replay.elf" "$FILE" 1e-9 || bad=1
    arm-none-eabi-size "$B/r5/control_replay.elf"
    if [ -n "$COUNT" ]; then
        M=$(arm-none-eabi-nm "$B/r5/control_replay.elf" | awk '$3 == "mark" { print $1 }')
        qemu-arm -cpu cortex-r5f -one-insn-per-tb -d exec,nochain "$B/r5/control_replay.elf" "$FILE" 1e-9 "$COUNT" 2>&1 > /dev/null |
            python3 -I test/count_periods.py "$M"
    fi
else
    echo "NOT RUN: the R5 build (no arm-none-eabi-g++ or no qemu-arm)"
    [ -z "$bad" ] && bad=2
fi
exit ${bad:-0}
