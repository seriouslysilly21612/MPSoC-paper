#!/bin/sh
# check_copies.sh -- what this firmware has from the APU application (goal-to-torque_MPSoC/indy7) must be what the
# application has now.  From r5_freertos/:
#   sh test/check_copies.sh
#   1. src/ipc/ipc_layout.h, src/ipc/ipc_slot.h and src/obs/ (the mailbox; with the control task of S2 also the
#      receiver of the camera results) are copies of the application's files: the two sides must be built from the
#      same text.  A copy is updated from the application (cp), never the other
#      way round.
#   2. src/cfg/fw_cfg_indy7.h and src/cfg/fw_cfg_l7n.h are what bin/fw_cfg_header writes from INDY7.cfg and
#      bench/L7N3_bus.cfg (cd goal-to-torque_MPSoC/indy7 && make tools).  When one differs, write it again with the
#      tool and build the firmware again; the header is not edited.
#   3. the record of a cycle (src/cycle_record.h) is the record of the application's timing log (TimingLog.h):
#      test/check_record.cpp compiles only when the fields and the flag bits are the same.
#   4. the controller of src/ctrl (the second controller of --shadow, and the control task of S2) is the
#      application's: its classes are copies as in 1.; src/ctrl/ctrl_model_indy7.h is what bin/rbdl_model_code writes from models/indy7.urdf, and
#      src/ctrl/ctrl_numbers_indy7.h what bin/ctrl_numbers_header writes from INDY7.cfg (make tools; they need the
#      RBDL of the application).
# Exit 1 if something differs, 2 if a tool is not built or there is no g++ (that part was not checked).
APP=../../goal-to-torque_MPSoC/indy7
bad=0
# same <file of the application under src/> <file here under src/>
same() {
    if cmp -s "$APP/src/$1" "src/$2"; then
        echo "same: src/$2"
    else
        echo "DIFFERENT: src/$2 and $APP/src/$1"
        bad=1
    fi
}
for f in ipc/ipc_layout.h ipc/ipc_slot.h obs/obs_mailbox.h obs/obs_receiver.h obs/obs_status.h obs/obs_ring.h \
         obs/ObsEstimate.h obs/ObsGoal.h obs/ObsMethod.h; do
    same "$f" "$f"
done
for f in Controller.h Controller.cpp FullDynControllerRT.h FullDynControllerRT.cpp HighLevelRT.h HighLevelRT.cpp \
         HighLevelLink.h shadow/ShadowControl.h shadow/ShadowControl.cpp TrajectoryRT.h TrajectoryRT.cpp GoalSlot.h \
         shadow/ShadowWriter.h shadow/ControlCycle.h shadow/ControlCycle.cpp GoalFilter.h; do
    same "$f" "ctrl/$(basename "$f")"
done
if [ ! -x "$APP/bin/fw_cfg_header" ]; then
    echo "NOT CHECKED: src/cfg/fw_cfg_*.h ($APP/bin/fw_cfg_header is not built: make tools)"
    [ $bad -eq 0 ] && bad=2
else
    tmp=$(mktemp)
    "$APP/bin/fw_cfg_header" -f "$APP/INDY7.cfg" -o "$tmp" > /dev/null 2>&1
    if cmp -s "$tmp" src/cfg/fw_cfg_indy7.h; then echo "same: src/cfg/fw_cfg_indy7.h"; else echo "DIFFERENT: src/cfg/fw_cfg_indy7.h and the cfg"; bad=1; fi
    "$APP/bin/fw_cfg_header" -f "$APP/INDY7.cfg" -B "$APP/bench/L7N3_bus.cfg" -o "$tmp" > /dev/null 2>&1
    if cmp -s "$tmp" src/cfg/fw_cfg_l7n.h; then echo "same: src/cfg/fw_cfg_l7n.h"; else echo "DIFFERENT: src/cfg/fw_cfg_l7n.h and the cfg"; bad=1; fi
    rm -f "$tmp"
fi
if [ ! -x "$APP/bin/rbdl_model_code" ]; then
    echo "NOT CHECKED: src/ctrl/ctrl_model_indy7.h ($APP/bin/rbdl_model_code is not built: make tools)"
    [ $bad -eq 0 ] && bad=2
else
    tmp=$(mktemp)
    "$APP/bin/rbdl_model_code" "$APP/models/indy7.urdf" -o "$tmp" > /dev/null 2>&1
    if cmp -s "$tmp" src/ctrl/ctrl_model_indy7.h; then echo "same: src/ctrl/ctrl_model_indy7.h"; else echo "DIFFERENT: src/ctrl/ctrl_model_indy7.h and the URDF"; bad=1; fi
    rm -f "$tmp"
fi
if [ ! -x "$APP/bin/ctrl_numbers_header" ]; then
    echo "NOT CHECKED: src/ctrl/ctrl_numbers_indy7.h ($APP/bin/ctrl_numbers_header is not built: make tools)"
    [ $bad -eq 0 ] && bad=2
else
    tmp=$(mktemp)
    "$APP/bin/ctrl_numbers_header" -f "$APP/INDY7.cfg" -o "$tmp" > /dev/null 2>&1
    if cmp -s "$tmp" src/ctrl/ctrl_numbers_indy7.h; then echo "same: src/ctrl/ctrl_numbers_indy7.h"; else echo "DIFFERENT: src/ctrl/ctrl_numbers_indy7.h and the cfg"; bad=1; fi
    rm -f "$tmp"
fi
if ! command -v g++ > /dev/null; then
    echo "NOT CHECKED: src/cycle_record.h (no g++)"
    [ $bad -eq 0 ] && bad=2
elif g++ -std=c++17 -fsyntax-only -I"$APP/src" -Isrc test/check_record.cpp; then
    echo "same: the record of src/cycle_record.h and of $APP/src/TimingLog.h"
else
    echo "DIFFERENT: the record of src/cycle_record.h and of $APP/src/TimingLog.h"
    bad=1
fi
exit $bad
