#!/bin/sh
# check_copies.sh -- the files that this firmware has from the APU application (goal-to-torque_MPSoC/indy7/src) are
# copies: the two sides must be built from the same text.  From r5_freertos/:
#   sh test/check_copies.sh
# Exit 1 if a copy differs.  A copy is updated from the application (cp), never the other way round.
APP=../../goal-to-torque_MPSoC/indy7/src
bad=0
for f in ipc/ipc_layout.h ipc/ipc_slot.h obs/obs_mailbox.h; do
    if cmp -s "$APP/$f" "src/$f"; then
        echo "same: src/$f"
    else
        echo "DIFFERENT: src/$f and $APP/$f"
        bad=1
    fi
done
exit $bad
