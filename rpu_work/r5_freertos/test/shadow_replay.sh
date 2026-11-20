#!/bin/sh
# shadow_replay.sh -- builds test/shadow_replay.cpp with the firmware's controller (src/ctrl) and the libraries
# from their sources, for this PC and for the R5, and runs both on a file of periods (shadow_check -o FILE of the
# application; see shadow_replay.cpp).  From r5_freertos/:
#
#   RBDL_SRC=... RUCKIG_SRC=... sh test/shadow_replay.sh FILE [BUILD_DIR]
#
#   RBDL_SRC    the sources of RBDL 3.1.3 (its src/ and include/)
#   RUCKIG_SRC  the sources of Ruckig 0.19.4 (its src/ruckig/ and include/)
#   EIGEN_DIR   Eigen 3.4 (default /usr/include/eigen3)
#   BUILD_DIR   where the objects go (default /tmp/shadow_replay); the libraries are compiled once
#
# The PC run must agree with the file exactly.  The R5 run (arm-none-eabi-g++, qemu-arm; skipped with a line if
# one is missing) within 1e-9 N m and rad: it has no fused multiply-add, the PC that wrote the file may.
# Neither may allocate memory in a period.
# Then, with COUNT=N, the instructions of the first N periods on the R5 are counted (test/count_periods.py).
# Exit 1 if a run failed, 2 if the R5 run was not made.
set -e
FILE="$1"; B="${2:-/tmp/shadow_replay}"; EIGEN="${EIGEN_DIR:-/usr/include/eigen3}"
[ -f "$FILE" ] && [ -d "$RBDL_SRC/src" ] && [ -d "$RUCKIG_SRC/src/ruckig" ] || { sed -n '2,16p' "$0"; exit 2; }
APP=../../goal-to-torque_MPSoC/indy7
INC="-Isrc/ctrl/port -Isrc/ctrl -Isrc -I$APP/src -isystem $RBDL_SRC/include -isystem $EIGEN -isystem $RUCKIG_SRC/include"
OURS="test/shadow_replay.cpp src/ctrl/shadow_ctrl.cpp src/ctrl/ShadowControl.cpp src/ctrl/FullDynControllerRT.cpp src/ctrl/Controller.cpp src/ctrl/HighLevelRT.cpp src/ctrl/ctrl_model.cpp"
R5="-mcpu=cortex-r5 -mfpu=vfpv3-d16 -mfloat-abi=hard -Wno-psabi"
WRAP="-Wl,--wrap=malloc,--wrap=calloc,--wrap=realloc"     # the allocation count of shadow_replay.cpp

# build <directory> <compiler> <flags>: the libraries (once) and the test
build() {
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
    for f in $OURS; do
        $cxx $flags -O2 -std=gnu++17 -DNDEBUG -Wall $INC -c "$f" -o "$d/ours_$(basename "$f" .cpp).o"
    done
}

build "$B/pc" g++ ""
g++ $WRAP "$B"/pc/*.o -o "$B/pc/shadow_replay"
echo "PC:"
"$B/pc/shadow_replay" "$FILE" 0 || bad=1

if command -v arm-none-eabi-g++ > /dev/null && command -v qemu-arm > /dev/null; then
    build "$B/r5" arm-none-eabi-g++ "$R5 -ffunction-sections -fdata-sections"
    arm-none-eabi-g++ $R5 --specs=rdimon.specs -Wl,--gc-sections $WRAP "$B"/r5/*.o -o "$B/r5/shadow_replay.elf"
    echo "R5 (qemu-arm):"
    qemu-arm -cpu cortex-r5f "$B/r5/shadow_replay.elf" "$FILE" 1e-9 || bad=1
    arm-none-eabi-size "$B/r5/shadow_replay.elf"
    if [ -n "$COUNT" ]; then
        M=$(arm-none-eabi-nm "$B/r5/shadow_replay.elf" | awk '$3 == "mark" { print $1 }')
        qemu-arm -cpu cortex-r5f -one-insn-per-tb -d exec,nochain "$B/r5/shadow_replay.elf" "$FILE" 1e-9 "$COUNT" 2>&1 > /dev/null |
            python3 -I test/count_periods.py "$M"
    fi
else
    echo "NOT RUN: the R5 build (no arm-none-eabi-g++ or no qemu-arm)"
    [ -z "$bad" ] && bad=2
fi
exit ${bad:-0}
