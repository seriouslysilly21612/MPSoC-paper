#!/bin/bash
# install_pl_qos.sh - one-time root install: the static QoS of the PL ports to the DDR for the timing runs
# (C3-4: tools/run_timing.py --pl-qos N raises it for one run and puts it back after).
#
# The AFIFM registers are reached through /dev/mem (tools/host/hp_qos.py), so root only, and a batch of runs outlasts
# the sudo timestamp. This installs
#   /usr/local/lib/hp-qos/hp_qos.py  a root-owned copy of tools/host/hp_qos.py (install again after it changes)
#   /usr/local/sbin/pl-qos           a fixed-command wrapper:
#                                      pl-qos show       every port, and whether a set is not yet restored
#                                      pl-qos set N      the read and write QoS of HPC0-1 and HP0-3 to N (0-15),
#                                                        static; their values before the first set kept in
#                                                        /var/lib/hp-qos/before.json
#                                      pl-qos restore    those values back, and the file removed
#   /etc/sudoers.d/pl-qos            lets USER run only that wrapper without a password
# The wrapper takes no path and no register value but N: the user cannot point root at a file of its own.
#
#   sudo bash tools/host/install_pl_qos.sh [USER]       (USER: default the user who runs sudo)
set -e
if [ "$(id -u)" != 0 ]; then
    echo "run with sudo: sudo bash $0" >&2
    exit 1
fi
U=${1:-${SUDO_USER:-}}
if [ -z "$U" ] || ! id "$U" >/dev/null 2>&1; then
    echo "usage: sudo bash $0 [USER] (the user who starts the timing runs)" >&2
    exit 2
fi
SRC=$(dirname "$(readlink -f "$0")")/hp_qos.py
if [ ! -f "$SRC" ]; then
    echo "no $SRC" >&2
    exit 1
fi

install -d -m 0755 /usr/local/lib/hp-qos /var/lib/hp-qos
install -m 0644 "$SRC" /usr/local/lib/hp-qos/hp_qos.py
install -m 0755 /dev/stdin /usr/local/sbin/pl-qos <<'WRAPPER'
#!/bin/sh
# pl-qos: the static QoS of the PL ports to the DDR (HPC0-1, HP0-3) for the timing runs (run_timing.py --pl-qos).
# Fixed commands on purpose: it is referenced by a NOPASSWD sudoers rule (tools/host/install_pl_qos.sh).
T=/usr/local/lib/hp-qos/hp_qos.py
S=/var/lib/hp-qos/before.json
PORTS=HPC0,HPC1,HP0,HP1,HP2,HP3
case "$1" in
show)
    [ $# -eq 1 ] || exit 2
    /usr/bin/python3 -I -B "$T" || exit 1
    if [ -f "$S" ]; then
        echo "pl-qos: raised since $(date -r "$S" '+%F %T') and not restored (pl-qos restore)"
    fi ;;
set)
    [ $# -eq 2 ] || exit 2
    case "$2" in
    [0-9]|1[0-5]) ;;
    *) echo "pl-qos: N is 0-15" >&2; exit 2 ;;
    esac
    exec /usr/bin/python3 -I -B "$T" set --port "$PORTS" --rd-qos "$2" --wr-qos "$2" --static --save "$S" ;;
restore)
    [ $# -eq 1 ] || exit 2
    if [ -f "$S" ]; then
        /usr/bin/python3 -I -B "$T" restore "$S" && rm -f "$S"
    else
        echo "pl-qos: nothing to restore"
    fi ;;
*)
    echo "usage: pl-qos show | set N | restore" >&2
    exit 2 ;;
esac
WRAPPER

echo "$U ALL=(root) NOPASSWD: /usr/local/sbin/pl-qos" > /etc/sudoers.d/pl-qos
chmod 0440 /etc/sudoers.d/pl-qos
visudo -cf /etc/sudoers.d/pl-qos

echo "installed: /usr/local/sbin/pl-qos, /usr/local/lib/hp-qos/hp_qos.py and /etc/sudoers.d/pl-qos (user $U)"
echo "check: sudo -n /usr/local/sbin/pl-qos show"
