# S1: the EtherCAT task on the R5, the control task on the APU

In this configuration the R5 firmware (`r5_freertos/`) runs the EtherCAT exchange and the drives, and the control
application (`../goal-to-torque_MPSoC`, started with `--arch S1`) keeps the control task. The two exchange three
slots in the first 16 KB of the OCM (`0xFFFC0000`; `ipc_layout.h`, the same file on both sides):

| Slot | Direction | Content |
|---|---|---|
| State | R5 → APU | the measurement of the period, its number, its release time on the system counter, the status of the drives, the fault counts of the R5, the check value of the firmware's constants |
| Torque | APU → R5 | the torque computed from that State, with its period number |
| Command | APU → R5 | servo on, servo off, stop |
| Shadow in | APU → R5 | with `--shadow` only: what the control task did with its controller in the period (the measured state, the goal and the calls of the Trajectory part, the mode of the law, its torque) |
| Shadow out | R5 → APU | with `--shadow` only: the torque of the R5's second controller for that period, its largest difference to the torque of the APU, the time of its two parts |

The two Shadow slots came with layout 2 of the region (2026-11-18). A side reads a slot only while the other side
has its own layout number in the head, so the firmware, the application and `r5_freertos/r5cmd.py` are updated
together: to an application of layout 2 a firmware of layout 1 is an R5 that is not up (`not up yet (no State slot
until it is)`), and that firmware takes neither its torque nor its commands.

The period of the R5 is the period of the loop. The two tasks of the application are released `WAKE_DELAY_US`
(`[R5_LINK]` of the cfg, first value 150) after the release of the R5, read the State of that period, and must have
the torque in its slot before the next release of the R5.

    R5   | receive, State(k), send |                            | receive, Torque(k) ...
    APU            | State(k) -> control law -> Torque(k) |
         0 ------- WAKE_DELAY ------------------------------- 1 ms

## The three ways to run the application with `--arch S1`

| Start | The R5 | The arm | Needs |
|---|---|---|---|
| `./run.sh -s --arch S1` | simulated: a thread of the application on another CPU, with the firmware's rules for the slots | MuJoCo, stepped by the simulated R5 | nothing of this page |
| `./run.sh -b -B bench/L7N4_bus.cfg --arch S1` | the firmware on the bus of the bench; its drives stay off (the application never asks for a servo on with `-b`) | MuJoCo, stepped by the application once per State taken | the device tree and the firmware below |
| `./run.sh --arch S1` | the firmware on the bus of the robot | the robot | the same, and a firmware built for the Indy7 from the cfg |

## The device tree: the slots as a UIO device

The application runs as the user of the board, and `/dev/mem` needs root. `ocm_link_uio.dtso` adds a node
`ocm-link@fffc0000` (16 KB, `compatible = "generic-uio"`), which Linux gives as `/dev/uioN`; the application finds
it by its name. The GEM3 buffers of the firmware start right behind the slots (`0xFFFC4000`) and are not in it.

Merge from `base.dtb`, never on top of a merged tree (as in `r5_freertos/README.md`):

    cd ~/rpu_work
    fdtoverlay -i base.dtb -o user-override.s1.dtb rpu_v2.dtbo gem3_off.dtbo ocm_link_uio.dtbo
    sha256sum ocm_link_uio.dtbo user-override.s1.dtb
    # 28edf211d046e44b78cc82620559b88a159ee693701973d627f3fb7e9abcbe4a  ocm_link_uio.dtbo
    # aa2aee99fa506ab1dce60e8d424c1b4e52e19baad13c3db4597e6c49fb6f8b92  user-override.s1.dtb

The result is the tree of the R5 configuration (`r5_freertos/user-override.dtb`) with that one node more. With the
R5 stopped and no drive enabled:

    sudo cp user-override.s1.dtb /boot/firmware/user-override.dtb.s1
    sudo cp /boot/firmware/user-override.dtb.s1 /boot/firmware/user-override.dtb    # .r5 and .p0 stay as they are
    sudo reboot

After the boot:

    for d in /sys/class/uio/uio*; do echo "$d $(cat $d/name) $(cat $d/maps/map0/addr) $(cat $d/maps/map0/size)"; done
    # /sys/class/uio/uio4 ocm-link 0x00000000fffc0000 0x0000000000004000

The number is the next free one: this board has four other UIO devices (`axi-pmon`), so the node is `uio4`. The
application and the udev rule find it by its name.

No such line: the kernel command line must have `uio_pdrv_genirq.of_id=generic-uio` (`cat /proc/cmdline`; it has on
this board), and the driver must be there (`ls /sys/bus/platform/drivers/uio_pdrv_genirq`; as a module:
`sudo modprobe uio_pdrv_genirq of_id=generic-uio`). `ls /sys/bus/platform/devices | grep ocm-link` tells whether
the node arrived.

The device belongs to root. Once:

    sudo cp 99-ocm-link.rules /etc/udev/rules.d/
    sudo udevadm control --reload && sudo udevadm trigger --subsystem-match=uio
    ls -l /dev/uio4          # crw-rw---- 1 root ubuntu (the number of the line above)

At a boot the rule and the driver (a module) come by themselves. `r5log.py` uses `/dev/mem` as root and goes on
working beside the node, which does not reserve the region.

Without a reboot, e.g. while the R5 runs: the same node as an overlay at run time. It is gone at the next boot,
and for a moment after it appears the device is still root's (udev sets the group a little later).

    sudo mkdir /configfs/device-tree/overlays/ocm_link
    sudo sh -c 'cat ocm_link_uio.dtbo > /configfs/device-tree/overlays/ocm_link/dtbo'
    cat /configfs/device-tree/overlays/ocm_link/status      # applied

## The device tree: the record of the R5's cycles as a UIO device

The firmware keeps a record of every cycle of its EtherCAT task in a ring in its DDR (`r5_freertos/README.md`).
`r5_freertos/r5rec.py` copies it into a timing log, and `tools/run_timing.py --arch S1` starts it for the length of a
run. So that it reads the ring as the user of the board, `r5_record_uio.dtso` adds a node `r5-record@36300000`
(0xF0000 bytes, `compatible = "generic-uio"`); without the node `r5rec.py` uses `/dev/mem` and needs `sudo`. The
node does not reserve the region: remoteproc and `r5log.py` go on as before.

Without a reboot (gone at the next boot):

    cd ~/rpu_work
    sudo mkdir /configfs/device-tree/overlays/r5_record
    sudo sh -c 'cat r5_record_uio.dtbo > /configfs/device-tree/overlays/r5_record/dtbo'
    cat /configfs/device-tree/overlays/r5_record/status      # applied
    sudo cp 99-ocm-link.rules /etc/udev/rules.d/             # it has the line of this device too
    sudo udevadm control --reload && sudo udevadm trigger --subsystem-match=uio
    for d in /sys/class/uio/uio*; do echo "$d $(cat $d/name) $(cat $d/maps/map0/addr) $(cat $d/maps/map0/size)"; done
    # ... /sys/class/uio/uio5 r5-record 0x0000000036300000 0x00000000000f0000
    python3 r5_freertos/r5rec.py --head                      # no sudo

In the boot tree: merge it as the fourth overlay, from `base.dtb`, and install the result as above.

    fdtoverlay -i base.dtb -o user-override.s1.dtb rpu_v2.dtbo gem3_off.dtbo ocm_link_uio.dtbo r5_record_uio.dtbo
    sha256sum r5_record_uio.dtbo user-override.s1.dtb
    # 11d092621ff0d47c5767c84d7c1f6f3afa8df2b014d81657c7f45b45c3b70871  r5_record_uio.dtbo
    # eda7a7ae03767a7c0b9ef0e1e575ad9e1ee1cabdfaa5528a51d72b93365205fb  user-override.s1.dtb

## The firmware

- It must have the OCM link and switch the servo on by command only: `main` since 2026-11-17 (it prints `[link]`
  lines at its start, and a build with `-DRUN_LEVEL` stops with an error).
- For a servo on it must be built from the cfg the application runs with: the application compares the check
  value in the State slot with the one it computes from its cfg, and refuses the servo on when they differ. A
  firmware of `main` reports 0 and is refused; the value comes with the branch `fw-fixes`. `-b` needs no servo on.
- Start and stop as in `r5_freertos/README.md` (remoteproc, `r5log.py -f`, Ctrl-C for the clean stop). Either side
  may start first, and either may be started again: the application says `the R5 side is not up yet`, then
  `[R5] following the period of the R5 ...` and `[R5] OP: ...` when the States come. When the R5 is started again
  under a running application, it says `[R5] the R5 started again (epoch a -> b)` and tells OP again.

## What to look at

The application prints these lines when it ends (`q`):

| Line | Should be | Otherwise |
|---|---|---|
| `[R5] State slot: ... N cycles without a new State (M while following)` | M = 0 | the task ran before the R5 had written the State: `WAKE_DELAY_US` too small, or the R5 late |
| `... N periods of the R5 that no cycle took` | 0 | the task here was more than a period late (Linux) |
| `... N reads failed` | 0 | `being written`: the read met the write of the R5 (`WAKE_DELAY_US` too small) |
| `[R5] following: phase error mean, min, max` | a few us around 0 | the counter of the R5 and the clock of Linux drift, or the R5's period jitters |
| `... the grid was set at once N times` | 1 | 2 or more: the phase error left 50 us for 8 periods, or the R5 started again |
| `... with a release time that did not fit` | 0 | the firmware does not write the system counter into the State slot |
| `[R5] Torque slot: ... N late` | 0 | the torque was not in its slot at the next release of the R5: `WAKE_DELAY_US` too large, or the control task too long |
| `[R5] faults the R5 reports` | 0 | the firmware's own late releases, overruns, send deadlines |

The read that sets the grid (the first State, and the first one after a restart of the R5) comes at any time in the
period: it is not among the times of the reads, and neither the period that the cycle after it may lose nor its
torque is counted. A restart of the R5 under a running application shows as cycles without a new State (the time
the R5 was down), reads that failed with `being written` (its bring-up, about 3 s: the slot is marked until the
first State), the grid set at once again, and `the R5 carried out number 0`: a new run of the firmware drops the
command that was in the slot (its drives are off).

A test of the application (`u`, `y`) counts a State that no cycle took as a late release and a late torque as a
missed send, beside the faults the R5 reports, and marks the run `TIMING FAULTS`. `tools/timing_summary.py` reads a
`-t` log of `--arch S1` by the same quantities.

The send is the R5's, so it is not in the log of the application. `tools/run_timing.py --arch S1` takes the record
of the R5 during the run (`OUT/r5/timing_r5_<stamp>.bin`) and gives it to the summary (`--r5`): the rows `R5 Delta
T_send`, `R5 t_w`, `R5 t_w + t_s` over the periods of the window, the faults of the R5 in them, and whether every
one of them is in its log. These fill the first three columns of the row of Table 6. The log of the application
marks the State whose period the R5 began without the torque of the State before it (`flags2`); with `-b` the
summary sets apart those that were late only because the step of the simulated arm ran before the control task
(the chain reached `T_s` with the step and not without it).

`--shadow` (the application's README; the firmware side in `r5_freertos/README.md`, built with `SHADOW_CONTROL`):
the R5 runs the control computation after its send, on what the control task told it. The firmware says `[shadow]
the controller of the control task is built: check value 0x...` at its start, the application prints the same
value in its first `[SHADOW]` line and, at its end, how many torques it compared with the R5's and how many
differ. `tools/run_timing.py --arch S1 --shadow` adds the times of that computation on the R5 to the summary, and
what the largest release-to-end of a cycle with it says about moving the control task to the R5 (D-14).

`WAKE_DELAY_US` is chosen on the board: long enough that the State of the period is written when the tasks wake (the
R5 took at most 88 us for a whole period on the bench), short enough that the torque is back before the next
release. Try 100, 150, 200 and 300 with `python3 tools/cfgset.py indy7/INDY7.cfg WAKE_DELAY_US=...` and compare
the lines above.

## What differs from the EtherCAT task on the APU

- What the application decides about the drives (servo on and off, the dropout rule, the servo on that does not
  complete) reaches them one period later: it travels as a command, which the R5 carries out in its next period.
- A stop command is a servo off for the firmware.
- With the servo on and the controller of the application off, the R5 gives torque 0 (the application alone leaves
  the last target in the drives).
- Should the application stop writing torques, the R5 switches the servo off after `TORQUE_STALE_LIMIT` (10)
  periods. The application then sees the drives leave Operation enabled, reports a drive dropout and adds which
  torque the R5 had (`... the torque of period P, N before it`): N = 1 is a drive or an E-stop, N at the limit is
  this rule.
- A State that the application does not take: the arm moved for a period, the reference did not. On the APU a late
  cycle ran once more and caught up; here the next cycle finds no new State and does nothing. The tracking error
  jumps by the motion of one period for every such State.

## Safety

The rules of `r5_freertos/README.md` hold: one EtherCAT master at a time; no `echo stop` while a motor is enabled;
no restart of Linux while the R5 runs the bus. Quit the application with `q` (it sends the servo off and goes on for
1 s) before stopping the R5. The first servo on of a new build: a low torque limit, the E-stop at hand.

## Checked on the board (2026-11-18, the L7N bench, `-b`)

- The node at run time and at a boot with the merged tree; the rule; the application as the user of the board.
- The R5 reads the same counter as the APU: the States were read 164.0 us after their release on average, which
  is `WAKE_DELAY_US` (150) and the 14.0 us that the task took to start.
- With the bus in OP, 132 s, the reference check (`y`, 5) on the simulated arm: every State taken, none read
  twice, no read failed, phase error -5.6 to 4.3 us, no torque late, no fault reported by the R5. From the release
  of the R5 to the torque in its slot (the step of the simulated arm left out): 224 us on average, 402 us at most.
- The clean stop (`r5log.py -f`, Ctrl-C) with the drives in OP and no axis enabled: PRE-OP after 10 ms, no alarm;
  and the R5 stopped and started again under the running application.

Not yet: a servo on through the application (the firmware of `main` reports the check value 0: `fw-fixes`), an
enabled axis at the clean stop, the load conditions of the host-timing test, the robot.
