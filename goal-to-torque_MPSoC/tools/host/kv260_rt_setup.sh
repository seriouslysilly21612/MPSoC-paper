#!/bin/bash
# kv260_rt_setup.sh - the host configuration of the paper (Sec. III-A, Table 3) on the Kria KV260:
#
#     bash tools/host/kv260_rt_setup.sh                 check every item (no root needed)
#     sudo bash tools/host/kv260_rt_setup.sh --apply    apply what differs (backups *.g2t.bak), then reboot
#
#   the paper (NUC)                                 the KV260
#   PREEMPT_RT, HRT, HZ_1000, NO_HZ_FULL,           kernel 5.15.199-rt91-rt-kv260c: the same (checked)
#     RCU_NOCB_CPU, no cpufreq/cpuidle driver
#   isolcpus=domain,managed_irq,6-7 nohz_full=6-7   isolcpus=domain,managed_irq,3 nohz_full=3 rcu_nocbs=3
#     rcu_nocbs=6-7 irqaffinity=0-5,8-15              irqaffinity=0-2 (skew_tick=1 of the board kept)
#   idle=poll                                       nohlt: the same switch on arm64 (idle loop polls, no WFI)
#   CPU 7 (the SMT sibling) offline                 the A53 has no SMT
#   CPU 6 at its largest non-turbo clock            no cpufreq driver, no turbo: every core runs at the
#                                                   1333 MHz of the K26 (measured 1332.1-1332.5 MHz on CPU 3)
#   kernel.timer_migration=0                        the same (/etc/sysctl.d/90-g2t-rt.conf)
#   NTP stopped                                     timedatectl set-ntp false
#   Wi-Fi off                                       the board has none
#   rtprio 99, memlock unlimited                    /etc/security/limits.d/99-realtime.conf
#   IgH thread EtherCAT-OP at SCHED_FIFO 40         /usr/local/sbin/ecat-op-fifo (install_ecat_op_fifo.sh)
#
# nohlt keeps every core busy in its idle loop: about +1 W and a warmer chip (the PS temperature and the
# board power are printed). Undo: restore /etc/default/flash-kernel.g2t.bak and run flash-kernel, remove
# /etc/sysctl.d/90-g2t-rt.conf, timedatectl set-ntp true, rtprio back in the limits file; reboot.
set -u
CMDLINE="skew_tick=1 isolcpus=domain,managed_irq,3 nohz_full=3 rcu_nocbs=3 irqaffinity=0-2 nohlt"
FK=/etc/default/flash-kernel
SYSCTL=/etc/sysctl.d/90-g2t-rt.conf
LIMITS=/etc/security/limits.d/99-realtime.conf
APPLY=0; [ "${1:-}" = "--apply" ] && APPLY=1
if [ $APPLY = 1 ] && [ "$(id -u)" != 0 ]; then echo "--apply needs root (sudo)" >&2; exit 1; fi

bad=0
item() {   # name, ok (0/1), found
    printf '  %-4s %-34s %s\n' "$([ "$2" = 1 ] && echo ok || echo DIFF)" "$1" "$3"
    [ "$2" = 1 ] || bad=$((bad + 1))
}
K=/boot/config-$(uname -r)
for o in PREEMPT_RT HIGH_RES_TIMERS HZ_1000 NO_HZ_FULL RCU_NOCB_CPU GENERIC_IDLE_POLL_SETUP; do
    grep -q "^CONFIG_$o=y" "$K" 2>/dev/null; item "kernel $o" $((1 - $?)) "$(grep -E "CONFIG_$o[= ]" "$K" 2>/dev/null)"
done
for o in CPU_FREQ CPU_IDLE; do
    grep -q "^CONFIG_$o=y" "$K" 2>/dev/null; item "kernel no $o" $? "$(grep -E "CONFIG_$o[= ]" "$K" 2>/dev/null)"
done
for t in $CMDLINE; do
    grep -qw -- "$t" /proc/cmdline; item "boot $t" $((1 - $?)) ""
done
item "configured cmdline ($FK)" "$(grep -q "^LINUX_KERNEL_CMDLINE=\"$CMDLINE\"" $FK && echo 1 || echo 0)" \
     "$(grep '^LINUX_KERNEL_CMDLINE=' $FK)"
tm=$(cat /proc/sys/kernel/timer_migration); item "kernel.timer_migration=0" "$([ "$tm" = 0 ] && echo 1 || echo 0)" "$tm"
ntp=$(timedatectl show -p NTP --value 2>/dev/null); item "NTP stopped" "$([ "$ntp" = no ] && echo 1 || echo 0)" "NTP=$ntp"
rt=$(ulimit -r); item "rtprio 99 (this login)" "$([ "$rt" = 99 ] && echo 1 || echo 0)" "$rt ($(grep -h rtprio $LIMITS 2>/dev/null))"
ml=$(ulimit -l); item "memlock unlimited" "$([ "$ml" = unlimited ] && echo 1 || echo 0)" "$ml"
item "EtherCAT-OP -> FIFO 40 wrapper" "$([ -x /usr/local/sbin/ecat-op-fifo ] && echo 1 || echo 0)" "/usr/local/sbin/ecat-op-fifo"
item "no cpufreq (fixed clock)" "$([ ! -e /sys/devices/system/cpu/cpu3/cpufreq ] && echo 1 || echo 0)" ""
A=/sys/bus/iio/devices/iio:device0; P=/sys/bus/iio/devices/iio:device1
if [ -r $A/in_temp0_ps_temp_raw ] && [ -r $P/in_power2_raw ]; then
    echo "  info PS $(awk -v r=$(cat $A/in_temp0_ps_temp_raw) -v o=$(cat $A/in_temp0_ps_temp_offset) \
         -v s=$(cat $A/in_temp0_ps_temp_scale) 'BEGIN{printf "%.1f C", (r+o)*s/1000}'), board $(awk \
         -v r=$(cat $P/in_power2_raw) -v s=$(cat $P/in_power2_scale) 'BEGIN{printf "%.2f W", r*s/1000}')"
fi
echo "$bad item(s) differ from the paper's configuration"
[ $APPLY = 1 ] || exit $((bad > 0))

echo "== applying"
if ! grep -q "^LINUX_KERNEL_CMDLINE=\"$CMDLINE\"" $FK; then
    cp -n $FK $FK.g2t.bak
    sed -i "s|^LINUX_KERNEL_CMDLINE=.*|LINUX_KERNEL_CMDLINE=\"$CMDLINE\"|" $FK
    flash-kernel || { echo "flash-kernel failed: $FK.g2t.bak holds the old line" >&2; exit 1; }
    grep -aq "nohlt" /boot/firmware/boot.scr.uimg && echo "boot script updated: reboot to use it"
fi
echo "kernel.timer_migration = 0" > $SYSCTL && sysctl -q -p $SYSCTL
timedatectl set-ntp false
if [ -f $LIMITS ] && ! grep -qE 'rtprio[[:space:]]+99' $LIMITS; then
    cp -n $LIMITS $LIMITS.g2t.bak
    sed -i -E 's/(rtprio[[:space:]]+)[0-9]+/\199/' $LIMITS
fi
echo "done: reboot (the command line), then log in again (rtprio) and run this script without --apply"
