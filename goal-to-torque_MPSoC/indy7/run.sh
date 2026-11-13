#!/bin/bash
# run.sh - start the controller with the limits and the environment it needs
# (paper, Table 3):
#
#     ./run.sh [-f FILE] [-s | -b [-B FILE]] [-t DIR]   the options of bin/indy7_ctrl
#
# - The real-time priority limit must be at least 97 and the memlock limit
#   unlimited, e.g. in /etc/security/limits.d/99-realtime.conf
#       <user>  -  rtprio   99
#       <user>  -  memlock  unlimited
#   (log in again afterwards).
# - MALLOC_ARENA_MAX=2: the controller locks all of its memory (mlockall with
#   MCL_FUTURE), and the default arenas of glibc (up to 64 MB per thread)
#   would inflate the locked memory.
# - With the bus (the robot, or -b): the kernel thread of the IgH master in
#   its operation phase, EtherCAT-OP, is created SCHED_OTHER. It runs the
#   state machines of the master and the slaves, whose results the EtherCAT
#   task reads in every period, and under load it can wait for milliseconds.
#   It is lifted to SCHED_FIFO 40, above the load and below the real-time
#   tasks (95-97), by a fixed-command sudo wrapper that
#   tools/host/install_ecat_op_fifo.sh installs once.
# - A build with ROS 2 (the bridge of the camera application) loads the type
#   support of its messages at run time (dlopen, LD_LIBRARY_PATH only): this
#   script sources /opt/ros/humble/setup.bash and ~/ros2_ws/install/setup.bash
#   unless they are sourced already. Nothing else of the DDS setup of the
#   camera pipeline is needed (same RMW, same domain).
# - One controller at a time: the earlier one (Indy7Ctrl.out of
#   RAON-RT-Revision) pins its 1 kHz tasks to the same isolated CPU 3 and
#   takes the same EtherCAT master.
set -e
cd "$(dirname "$0")"

RT=$(ulimit -r); ML=$(ulimit -l)
if [ "$RT" != "unlimited" ] && [ "$RT" -lt 97 ]; then
    echo "run.sh: the real-time priority limit is $RT, at least 97 is needed (ulimit -r; see the head of run.sh)" >&2
    exit 1
fi
if [ "$ML" != "unlimited" ]; then
    echo "run.sh: the memlock limit is $ML, unlimited is needed (ulimit -l; see the head of run.sh)" >&2
    exit 1
fi
if [ ! -x ./bin/indy7_ctrl ]; then
    echo "run.sh: bin/indy7_ctrl is missing: make" >&2
    exit 1
fi
if [ "$(cat /proc/sys/kernel/timer_migration 2>/dev/null)" != 0 ]; then
    # nohz_full CPU 3: with migration the timers of its tasks can expire on
    # another core and wake the Dynamics task before the EtherCAT task (a
    # period without a fresh torque, '[BUS] WARNING: no fresh torque')
    echo "run.sh: WARNING: kernel.timer_migration is not 0 (paper, Table 3): sudo sysctl kernel.timer_migration=0" >&2
fi
for p in Indy7Ctrl.out indy7_ctrl; do
    if pgrep -x "$p" >/dev/null; then
        echo "run.sh: $p is running - quit it first (one controller at a time on CPU 3)" >&2
        exit 1
    fi
done

export MALLOC_ARENA_MAX=2

if ldd ./bin/indy7_ctrl 2>/dev/null | grep -q librclcpp && \
   [[ "${AMENT_PREFIX_PATH:-}" != *my_interfaces* ]]; then
    if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        source "$HOME/ros2_ws/install/setup.bash"
    else
        echo "run.sh: note: ~/ros2_ws/install/setup.bash is missing - the bridge will not start"
    fi
fi

SIM=0
for a in "$@"; do
    [ "$a" = "-s" ] && SIM=1
done
if [ "$SIM" = 0 ]; then
    (
        for _ in $(seq 1 180); do
            if pgrep -x EtherCAT-OP >/dev/null 2>&1; then
                if sudo -n /usr/local/sbin/ecat-op-fifo 2>/dev/null; then
                    echo "[run.sh] EtherCAT-OP -> SCHED_FIFO 40"
                else
                    echo "[run.sh] WARNING: EtherCAT-OP stays SCHED_OTHER: install the wrapper once with" \
                         "sudo bash ../tools/host/install_ecat_op_fifo.sh, or for this session:" \
                         "sudo chrt -f -p 40 \$(pgrep -x EtherCAT-OP)"
                fi
                break
            fi
            sleep 0.5
        done
    ) &
fi

exec ./bin/indy7_ctrl "$@"
