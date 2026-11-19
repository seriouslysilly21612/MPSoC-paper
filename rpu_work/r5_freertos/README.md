# R5F-0 firmware: SOEM EtherCAT master on GEM3

The FreeRTOS application of the R5F-0 (split mode) as it ran on the KV260 on 2026-11-11: the EtherCAT
master (SOEM 1.4.0) on GEM3, which Linux leaves to the R5, driving the L7N bench (4 axes, CSP, DC on,
1 kHz). Commit `ddc6d14` holds the source of that build unchanged. Since then the application is split
into modules with new names, without a change in the generated code (below); the master went FREE_RUN on
2026-11-15 and back to DC SYNC0 on 2026-11-16 (below). Torque mode and the slots of the APU came on 2026-11-17
(commit `51a7ad8`); the send deadline, the record of every cycle and the GEM3 buffers in the OCM on 2026-11-18
(below). Since 2026-11-19 the bench is three drives ("Three drives" below).

| Path | Content | Origin |
|---|---|---|
| `src/main.c` | start-up and two tasks: `ecat` (the master, priority `configMAX_PRIORITIES - 2`) and `led` | own |
| `src/app_config.h` | the build switches and fixed parameters: axes, cycle time, `RUN_LEVEL`, timeouts, report period | own |
| `src/ecat_task.c`, `.h` | the task `ecat`: bring-up, then the 1 kHz cycle (receive, working-counter check, axis logic, send), a status report every 10 s, and the stop that Linux asks for | own, a translation of the IgH example `cia402_basic.c` |
| `src/ecat_bringup.c`, `.h` | bring-up to OP: bus scan, PDO mapping, the drives' sync parameters printed, SAFE-OP, OP with process data flowing | own, from the same example |
| `src/cia402_drive.c`, `.h` | the drive's process data (packed structs over the IOmap), PDO mapping by SDO, statusword → CiA 402 state | own, from the same example |
| `src/axis_control.c`, `.h` | what one cycle does with the axes: CiA 402 state machine (enable, fault reset), hold, sine trajectory | own, from the same example |
| `src/event_log.c`, `.h` | the loop never prints: it posts events to a queue, the task `ecatlog` prints them at low priority | own |
| `src/ecat_diag.c`, `.h` | AL states, AL status codes and SDO abort codes in words, SOEM's error list, the slave states | own |
| `src/loop_timing.c`, `.h` | the release of a cycle, its times, the send deadline and the counts of late, missed and overrun | own |
| `src/sys_counter.h` | the system counter of the chip (IOU_SCNTR, the APU's CNTVCT): the one clock of the firmware | own |
| `src/cycle_record.c`, `.h` | the record of every cycle (that of the APU's timing log) and its ring in the DDR for Linux | own |
| `src/oshw/freertos/gem3_nic.c`, `.h` | the GEM3 driver below SOEM: clock and reset, PHY reset and MDIO, DMA descriptors and buffers (in the OCM, or in the DDR as before: `GEM3_BUF_IN_OCM`), polled send and receive, a PHY-loopback self-test | own |
| `src/oshw/freertos/nicdrv.c`, `.h`, `oshw.c`, `.h` | the SOEM hardware layer for FreeRTOS | own, from the SOEM templates (GPL-2.0 with exception) |
| `src/osal/freertos/` | the SOEM OS layer for FreeRTOS | own |
| `src/oshw/freertos/r5log.c`, `.h` | `outbyte()` into a ring buffer in DDR: the BSP console (JTAG DCC) blocks for ever without a debugger, as under remoteproc. Linux reads it with `r5log.py` | own |
| `src/oshw/freertos/xemacps*` | Xilinx emacps driver 3.16 (embeddedsw `xilinx_v2022.1`), unchanged | Xilinx, MIT |
| `src/soem/`, `src/osal/osal.h` | SOEM 1.4.0, unchanged | SOEM, GPL-2.0 with exception, `LICENSE.SOEM` |
| `src/shadow_link.c`, `.h` | the second controller of `--shadow` in the cycle: after the send, the payload of the control task through the controller of `src/ctrl`, the answer into its slot ("The second controller" below) | own |
| `src/ctrl/` | that controller: the classes of the control task of the APU, copied unchanged, with what they need here in place of Linux, a file system and the URDF reader ("The second controller" below) | the application's (`goal-to-torque_MPSoC/indy7/src`), and own |
| `src/lscript.ld` | the linker script: code and data in DDR 0x3600_0000–0x362F_FFFF, stack 8 KB, heap 256 KB (8 KB before the second controller, which takes about 80 KB of it when it is built). The last 1 MB of the 4 MB that `rpu_v2.dtbo` reserves stays out of the link: the record ring (0x3630_0000, 960 KB), and in the last 64 KB the stop page (0x363F_B000) and the log ring (0x363F_C000). In the build with `GEM3_BUF_IN_OCM` 0 those 64 KB are a non-cacheable window that also holds the GEM3 descriptor rings (from 0x363F_0000), and the frame buffers are cacheable `.bss` | own, from the Vitis template |
| `bsp_ref/FreeRTOSConfig.h` | the FreeRTOS configuration the BSP generates from `system.mss` (tick 1000 Hz, heap 256 KB, 8 priorities), to read only | generated |
| `r5_freertos.elf` | the build that ran, 936,504 bytes, sha256 `5d2e82c908e3cfaad145abfcb7496b4699455c1d81fe95db70fbf9d872e188eb`: the same file as `/lib/firmware/r5_freertos.elf` on the board (checked 2026-11-12). Built from the source of `ddc6d14` (`cia402_soem.c` in one file) | build |
| `platform/design_1_wrapper.xsa` | the hardware hand-off from Vivado 2022.1 (board `kv260_som` 1.3): the PS only, no PL, GEM3 not enabled | Vivado |
| `platform/system.mss` | the BSP settings of the R5 domain (`freertos10_xilinx` 1.11 on `psu_cortexr5_0`); its md5 is the one `platform.spr` records for that domain | Vitis |
| `platform/platform.spr` | the Vitis platform: the FSBL and PMU firmware domains and the R5 domain, with their BSP compiler flags | Vitis |
| `user-override.dtb` | the device tree the board boots with this firmware, sha256 `b1f13abe9be882fb887bfd6fdc8bc540edcd0a8acdb6facbb2064befa2a34207`: byte for byte `fdtoverlay -i ../base.dtb -o user-override.dtb ../rpu_v2.dtbo ../gem3_off.dtbo` | board |
| `../gem3_off.dtso`, `.dtbo` | the overlay that disables GEM3 (`ethernet@ff0e0000`) for Linux; the `.dtbo` is `dtc -@` of the `.dtso` | own |
| `r5log.py` | reads the log ring through `/dev/mem` on Linux (single-byte loads: the carveout is Device memory); with `-f`, Ctrl-C stops the run cleanly ("Clean stop") | own |
| `r5rec.py` | reads the record ring on Linux (the UIO device `r5-record` of `../r5_record_uio.dtso`, else `/dev/mem` with `sudo`) and writes a timing log that `goal-to-torque_MPSoC/tools/timing_summary.py` reads | own |

The BSP was not modified: its `mpu.c` is the same file as in the FSBL and PMU firmware BSPs, and the three
copies of `FreeRTOSConfig.h` are identical. The BSP can be generated again from the platform files.

GEM3 is not enabled in the XSA, so the BSP has no Ethernet driver. That is why the emacps sources are copied
into `src/` and `gem3_nic.c` builds the driver configuration and sets up the GEM3 clock and reset itself; the
rest of the PS is set up by the Kria boot firmware.

## Build

Vitis 2022.1: the XSA records 2022.1, the BSP is standalone 7.7 and freertos10_xilinx 1.11 (the embeddedsw
release `xilinx_v2022.1`), and the ELF records GCC 11.2.0.

1. A platform project from `platform/design_1_wrapper.xsa`, domain `freertos10_xilinx` on `psu_cortexr5_0`,
   with the BSP settings of `platform/system.mss`: tick 1000 Hz from TTC3, heap 256 KB, every task with an FPU
   context (`use_task_fpu_support = 2`), console on `psu_coresight_0` (replaced by `r5log.c`). The BSP
   libraries are compiled with `-Os` (`platform.spr`), the application with `-O0`.
2. An application project `r5_freertos` on that domain, its `src/` replaced by this `src/`.
3. The settings of the build that ran, from its generated makefiles (not kept here: Vitis writes them, and
   they hold the paths of one PC):
   - compiler: `armr5-none-eabi-gcc -DARMR5 -Wall -O0 -g3 -fmessage-length=0 -mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16`
   - include paths: `src/osal`, `src/soem`, `src/oshw/freertos`, `src/osal/freertos`, the BSP `bspinclude/include`
   - linker: `-mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16 -Wl,--gc-sections -Wl,-T -Wl,src/lscript.ld -L <BSP>/bsplib/lib`,
     libraries (`objects.mk`) `-Wl,--start-group,-lxil,-lfreertos,-lgcc,-lc,--end-group`
4. The switches in the source, as built:
   - `ROBOT_TARGET` (`src/app_config.h`): 0 = the L7N bench (the default), 1 = the Indy7. The ELF of 2026-11-11
     was built with `RUN_LEVEL` 2 (servo on and a sine trajectory by itself); that switch is gone, and a leftover
     `-DRUN_LEVEL`, `-DCIA402_...`, `-DLOOP_TIMING` or `-DCPU_HZ` stops the build with an error.
   - `GEM3_BUF_IN_OCM` (`gem3_nic.h`): 1 = the descriptors and frame buffers in the OCM, 0xFFFC_4000–0xFFFC_DFFF
     (the default); 0 = as on 2026-11-11, the descriptor window at 0x363F_0000 (64 KB) and the buffers in `.bss`.
     Other `GEM3_*` (`gem3_nic.c`): self-test on, PHY reset on MIO 38.
   - `R5LOG_ADDR` (`r5log.c`): the log ring at 0x363F_C000, 16 KB. `RECORD_RING_ADDR` (`cycle_record.h`): the
     record ring at 0x3630_0000, 960 KB; both outside what `lscript.ld` links.
   - `SYSCNT_HZ` (`sys_counter.h`): the ticks of the system counter per second, 99,999,000 on this board.

## Run

- The device tree: this directory's `user-override.dtb` into `/boot/firmware/` (back up the old one), then
  reboot. To change an overlay, merge again from `base.dtb`, never on top of a merged tree. The
  `user-override.dtb` at the top of `rpu_work/` is the configuration before the move (`base.dtb` +
  `rpu_v2.dtbo`, the board's `user-override.dtb.bak_gem3`: Linux drives GEM3 with IgH); do not boot this
  firmware with it.
- The firmware: `r5_freertos.elf` into `/lib/firmware/`, then remoteproc0 as in the top `README.md`.
- The log: `sudo python3 r5log.py -f` on Linux. The ring survives an R5 hang; read it before starting the R5 again.
- The record of every cycle: `sudo python3 r5rec.py --out DIR` on Linux, while the R5 runs (below, "The release ...").
- The stop: Ctrl-C in `r5log.py -f` (below, "Clean stop"). It ends with `[r5log: R5 stopped ...]`.
- Safety: one EtherCAT master at a time. Do not stop the R5 with `echo stop` alone while a motor is enabled, and do
  not use `r5log.py -f` with a firmware that has no stop command, without the E-stop or the drive power off first.
  Do not restart Linux while the R5 runs the bus.

## DC SYNC0 again (2026-11-16); FREE_RUN (2026-11-15) did not run on this bench

From 2026-11-15 the master activated no distributed clocks (FREE_RUN, decision D-13 of the paper project:
FREE_RUN on every stage, as in the earlier Indy7 experiments). On this bench the drive with firmware 0.12
(see "SM2 watchdog") left OP 100 ms after OP in every FREE_RUN run, so on 2026-11-16 the DC setup of the
colleague's build came back: `ec_configdc()` and `ec_dcsync0()` on every drive with `CYCLE_NS` and
`DC_SHIFT_NS` (125000), before the SAFE-OP request. 2-1 checks that the EtherCAT task runs on the R5; the
sync mode for the measurements is decided again before they are made. In PRE-OP, after the PDO mapping and
before SYNC0 is set up, the firmware reads and prints each drive's 0x1C32 with CoE SDO reads:

    [ecat]   slave <n> 0x1C32: sync type 0x<sub 1>, cycle <sub 2> ns, supported 0x<sub 4>, minimum cycle <sub 5> ns (<k> of 4 read)

Sub 1 is the sync type in use (0 free run, 1 SM2-synchronous, 2 DC SYNC0), sub 2 the cycle time, sub 4 the
supported types (bit 0 free run, bit 1 SM2, bits 2-3 DC), sub 5 the minimum cycle time. A sub that cannot be
read prints as 0xFFFF or 4294967295, and SOEM's error list follows.

On the board (2026-11-16, FREE_RUN builds) all four drives print sync type 0x0001, cycle 0 ns, supported
0x0005, minimum cycle 200000 ns: they report SM2-synchronous although 0x0005 lists free run and DC SYNC0 only.
IgH's FREE_RUN on the APU gives the same 0x1C32:01 = 1.

On the board with DC SYNC0 (2026-11-16, builds of `48fc0af`):
- RUN_LEVEL 0 ran 27 minutes with all four drives in OP and `wkc_bad=0`. OP came 205 ms after the request, as
  in the colleague's build, and exec max was 33 to 35 us. `diag.py` read the tick timer TTC3.0 at 999.79 Hz.
- RUN_LEVEL 1, first run: the 0.12 drive left OP at servo-on (AL status 0x001B, `wkc_bad=9898`).
- RUN_LEVEL 1, second run, after a drive power cycle: all four axes stayed OPERATION_ENABLED for 10 minutes.
  The positions stayed within +-2 counts, the loop period was 999/1000/1001 us and exec max 34 to 38 us.

With SYNC0 the 0.12 drive still fails now and then at start-up; the colleague saw two failures before a pass.
When it fails, switch the drive power off and on and start again.

## Three drives (2026-11-19)

The drive with firmware 0.12 is off the bus (the cable of the second drive's OUT goes to the IN of the fourth),
and the bench target is the three drives with firmware 0.22: `AXIS_COUNT` 3 and `BUS_SLAVE_COUNT` 3 in
`src/app_config.h`, `bench/L7N3_bus.cfg` in the application (check value 0xB2DB751F; it was 0x07AE943D with
`bench/L7N4_bus.cfg`). A firmware built for four drives refuses this bus at the start, and the other way round.

Why: with the build of that day (C++ project, `-O2`, the second controller of `--shadow`) the 0.12 drive left OP
about 100 ms after the loop started in 4 starts of 5 (AL status 0x001B, `wkc_bad=9899` at 10 s), the three
others stayed in OP in all five. Of the two starts right after a drive power-on one held; the three without one
before them all failed. Why it failed more often than with the builds before is not known, and it was not
looked for: the bench gives the timing of a real bus, and the share of one drive is at most a quarter of the
96 us that a cycle of the EtherCAT task took at most with four (2026-11-18).

DC SYNC0 stays as it is. In the FREE_RUN runs of 2026-11-16 only the 0.12 drive left OP (the first three state
that the others stayed), so the bench could go back to FREE_RUN (D-13); that is not decided.

## SM2 watchdog (2026-11-16)

Before `ec_config_map()` the firmware sets the watchdog trigger (bit 6) in every drive's SM2 control byte, as
IgH's `EC_WD_ENABLE` does in the APU application (`goal-to-torque_MPSoC/indy7/src/ecat/Indy7Joint.cpp`). The
drives' SII has 0x24 there (trigger off), and SOEM writes the SII value as it is. After `ec_config_map()` the
firmware reads each drive's SM2 registers back:

    [ecat]   slave <n> SM2 @0x1800, 11 bytes, control 0x64

Why (2026-11-16, the bench of four L7NA004):
- Three drives run firmware 0.22 (0x100A), the third on the bus 0.12. In every RUN_LEVEL 0 run without the
  trigger (3 of 3, one right after a drive power cycle) the 0.12 drive left OP about 100 ms after OP: SAFE-OP +
  ERROR, AL status 0x001B (SM watchdog), statusword FAULT, AL-67 on its display (0x603F = 0x0067),
  `wkc_bad=9898` at t=10 s. The other three stayed in OP.
- The ESC watchdog is the default 100 ms on all four drives (0x0400 = 2498, 0x0420 = 1000).
- Under IgH (P0, FREE_RUN) the same drive stays in OP. There SM2 reads `@0x1800, 13 bytes, control 0x64`, with
  0x1C32:01 = 1 and the AL event mask 0x00FF0400 on the 0.12 and a 0.22 drive alike. Apart from the PDO length
  (IgH maps the two torque entries as well), the control byte is the difference found.
- AL-67 is what an L7N reports when its SM watchdog expires: the three 0.22 drives reported it too (0x603F =
  0x0067, statusword FAULT) after the R5 was stopped while they were in OP. A drive power cycle clears it.

With the trigger set (one RUN_LEVEL 0 run, 22:21) all four drives read back `control 0x64`, and the 0.12 drive
left OP at the same cycle all the same. The trigger stays, as IgH has it; it was not the cause.

## OP after process data (2026-11-16, tried and taken out)

Under FREE_RUN, one build added `ec_configdc()` (DC offsets, no SYNC0) and 200 ms of process data in SAFE-OP
before the OP request, the two other things IgH FREE_RUN and the colleague's DC build do. The 0.12 drive left OP
at the same cycle all the same (one RUN_LEVEL 0 run, 23:06). The 200 ms are gone again; `ec_configdc()` stays as
part of the DC setup. Two differences are left between IgH FREE_RUN and this firmware: the operation mode (the
APU application writes CST, 10; this firmware CSP, 8) and the PDO content (the application maps 0x6071 and
0x6077 as well, 13 bytes each way). Neither is tried.

## Torque limit (2026-11-16)

Still in PRE-OP, after the 0x1C32 read, every drive gets `MAX_TORQUE_PERMILLE` (`src/app_config.h`,
default 100 = 10 % of rated torque; the L7N's own value is 3000) in 0x6072 (max torque), 0x60E0 (positive
limit) and 0x60E1 (negative limit). The firmware reads the three back and prints them:

    [ecat]   slave <n> torque limit: 0x6072 <v>, 0x60E0 <v>, 0x60E1 <v> (wrote <value>; unit 0.1 percent of rated)

It is the low limit for the first servo-on (decision D-10, condition 1). The L7N limits the torque with the
smallest of the three (L7N manual, section 6.7). A RUN_LEVEL 1 or 2 build does not leave PRE-OP when a value
does not read back; a RUN_LEVEL 0 build warns and goes on. The firmware does not store the values (no write to
0x1010), and the drive does not keep them either: they read 3000 again after a drive power cycle.

On the board (2026-11-16, RUN_LEVEL 0) all four drives read back 100, 100, 100. The first build also printed
0x2110 because the L7NH and L7P manuals call it the torque limit function select; on the L7N it is the notch
filter frequency (0 to 1000, default 300), and it read 300. That read is gone.

## Clean stop (2026-11-17)

Before, Ctrl-C in `r5log.py -f` only ended the viewer: the R5 is a CPU of its own and the viewer only reads the
ring. `echo stop` then halted the R5 at once, in OP. 100 ms later every drive's SM watchdog expired and the L7N
showed AL-67 (I-40). Now Ctrl-C stops the run:

1. `r5log.py` writes the stop command (`'S'`) into the stop page.
2. The task `ecat` sees it in the next cycle (`ctl_stop_requested()`). It stops calling the axis logic, so the
   outputs keep the last target and a moving axis comes to rest under position control. It holds 100 ms.
3. It writes controlword 0x0000 (disable voltage) to every axis and waits until no axis is enabled (500 ms at most).
4. It asks all drives for PRE-OP, with the error acknowledge: a drive that dropped out (AL error flag set) may
   ignore a request without it. The frames keep flowing while the drives change state, so no watchdog expires on
   the way (2 s at most, read every 10 ms).
5. It sends no more frames and prints each drive's AL state and last statusword, then
   `[ecat] STOPPED: all axes off, all drives in PRE-OP, no more frames; the R5 can be stopped`. A step that ran
   out of time prints a WARN line, and the last line says `STOPPED with the WARN above`. Only then does it
   report STOPPED in the page. DS36 goes off.
6. `r5log.py` prints those lines and stops the R5 (`/sys/class/remoteproc/remoteproc0/state`).

In PRE-OP no process data is expected, so no SM watchdog runs.

If the R5 does not report STOPPED within 15 s, `r5log.py` leaves it alone and prints the E-stop way. A second
Ctrl-C stops waiting. After Ctrl-C, `r5log.py` writes to stderr, since Ctrl-C has also ended a `| tee`. To end
only the viewer, kill it (SIGTERM).

The stop page is at 0x363F_B000, the 4 KB below the log ring, in the window that `lscript.ld` leaves out of the
link. Linux writes only its first 32-byte line, the R5 only its second, so a cache write-back of one side never
undoes the other side's byte (the R5 D-cache line is 32 bytes):

| Offset | Writer | Content |
|---|---|---|
| 0x00 | Linux | `0x53` ('S') = stop; the R5 clears it when it starts |
| 0x20 | R5 | `R5ST` |
| 0x24 | R5 | `ecat_status` as int8: 0 starting, 1 running, -2 bus fault, 2 stopping, 3 stopped, -1 init failed |
| 0x28 | R5 | this run's `boot_id` from the log ring header (`r5log_boot_id()`) |

`r5log.py` takes the page as this run's only when the magic and the `boot_id` match the ring's. A firmware
without the stop command leaves an earlier run's page in place, and stopping that R5 because of an old
"stopped" there would cut the frames of drives that may be enabled.

What was tested (2026-11-17, without the board):
- A host test of `ecat_task.c` with the real axis logic, drive interface, event log and loop timing against a
  model of four drives: RUN_LEVEL 0, 1, 2 × seven cases. The cases are normal, a stale stop command, one drive
  dropped out, one axis that does not switch off, one drive that refuses PRE-OP, PRE-OP slower than the watchdog,
  and init failed. All pass.
- Three changed builds (no acknowledge; no frames while the drives go to PRE-OP, two cases) fail the test, as
  they should.
- `r5log.py` against a simulated ring, page and remoteproc file, with real SIGINTs: nine cases pass.

Not yet on the board.

## The constants of the cfg, the stale-torque stop, the copies (2026-11-18)

Three points of the review of the torque firmware (`51a7ad8`), changed without the board and without a Vitis
build; the rest of this file still describes the build of 2026-11-11 to 2026-11-17.

- **The constants come from the cfg of the APU controller.** `src/robot_target.c` (a table typed by hand) is gone.
  `src/cfg/fw_cfg_l7n.h` and `src/cfg/fw_cfg_indy7.h` are written by the controller's tool, and
  `src/robot_target.h` takes the unit constants and the identity of every slave from the one of `ROBOT_TARGET`:

      cd goal-to-torque_MPSoC/indy7 && make tools
      ./bin/fw_cfg_header -o ../../rpu_work/r5_freertos/src/cfg/fw_cfg_indy7.h
      ./bin/fw_cfg_header -B bench/L7N3_bus.cfg -o ../../rpu_work/r5_freertos/src/cfg/fw_cfg_l7n.h

  The bring-up now compares the vendor id and the product code of every slave with the cfg (it compared the
  vendor id of the drives). The State slot carries `FW_CFG_CRC`, the check value of those constants, instead of 0;
  the APU is to ask for servo on only when it equals the check value of its own cfg (not wired yet). The sync mode
  is still `ECAT_USE_DC` of `app_config.h`. An application project that still has `src/robot_target.c` does not
  compile: delete the file there.
- **The stale-torque stop looks at the request, not at the drives.** `TORQUE_STALE_LIMIT` cycles without a new
  Torque slot while the servo is requested switch it off, also when an axis is in FAULT or does not come up (the
  count used to run only while every axis was enabled, and then never started). The APU therefore writes the
  Torque slot in every cycle from the servo-on command on; `r5cmd.py run` does.
- **The copies.** `OBS_OFF_MAILBOX` is in `obs/obs_mailbox.h`, the file both sides are built from.
  `sh test/check_copies.sh` tells when one of the three files copied from the application, or one of the two
  headers of the cfg, is not what the application has now.

Host tests (no board), from this directory; each prints `all passed`:

    gcc -std=gnu11 -Itest/stub -Isrc test/test_axis_control.c src/axis_control.c -lm -o /tmp/t && /tmp/t
    gcc -std=gnu11 -DROBOT_TARGET=1 -Itest/stub -Isrc test/test_axis_control.c src/axis_control.c -lm -o /tmp/t && /tmp/t
    gcc -std=gnu11 -DHOST_TEST -Itest/stub -Isrc test/test_obs_link.c src/obs_link.c -o /tmp/t_obs && /tmp/t_obs
    python3 test/test_r5cmd.py
    sh test/check_copies.sh

Checked on a PC: the tests above; the unit conversions give the same bits as the table typed by hand for 80,000
(L7N) and 120,000 (Indy7) random inputs; the changed files compile for the R5 (arm-none-eabi-gcc 13, stand-ins for
the BSP headers). Not checked: a Vitis build, the bench, the Indy7; the comparison of the product codes has run
nowhere.

## The release and the send deadline, the record of every cycle, the GEM3 buffers in the OCM (2026-11-18)

The last point of that review (F5: no send deadline, no base for the period) and the plan's 2-2, 2-4 and 2-11,
written without the board and without a Vitis build.

- **One clock.** Every time of the firmware is the system counter of the chip (`src/sys_counter.h`: IOU_SCNTR,
  which the APU reads as CNTVCT; 99,999,000 ticks/s on this board). The PMU cycle counter is no longer used, and
  `LOOP_TIMING` and `CPU_HZ` are gone.
- **The release of a cycle is the FreeRTOS tick**, the interrupt of the TTC that wakes the task. At the start of a
  cycle the task reads how far that timer has counted: that is t_w, and the release is the start minus t_w
  (`loop_timing_begin()`). The State slot carries that release (it carried the start of the cycle), so the APU
  reads the State t_w earlier than before, a few microseconds. `loop_timing_start()` checks that the timer runs up
  in interval mode; when it does not, the firmware parks before the bus scan and says why.
- **The send deadline.** A cycle whose send returns later than `SEND_DEADLINE` of the cfg (400 us) after its
  release is counted (missed), as are a start more than a period late (late) and an end after the next release
  (overrun): the rules and names of the EtherCAT task of the APU controller. The counts are in the State slot
  (`n_late`, `n_missed`, `n_overrun`, of the cycles before the one that writes them) and in the status report:

      [ecat]   loop period min/avg/max = <a>/<b>/<c> us   t_w max = <n> us   release to send max = <n> us of 400   exec max = <n> us (run max <n> us)   late <k>, sends past the deadline <k>, overruns <k>

  The loop period is start to start; an average that is not 1000 us says that `SYSCNT_HZ` is wrong.
- **SOEM's timers run on that clock** (`osal.c`). With the FreeRTOS tick a timeout ended at a tick: the receive of
  a lost frame waited up to 1 ms, and the send of that cycle left a period late. `RX_TIMEOUT_US` is 200 now
  (500 before), so that a cycle that loses its frame still sends inside the deadline.
- **A record of every cycle** (`src/cycle_record.c`): 60 bytes with t_w, t_s, the receive, the send and the flags,
  the record of the APU's timing log (`TimingLog.h`; `test/check_record.cpp` does not compile when they differ).
  The R5 writes it into a ring of 16,382 records (16 s) at 0x3630_0000, in the DDR that Linux reserves for the R5,
  and never waits for a reader. On Linux:

      sudo python3 r5rec.py --out DIR          # until Ctrl-C; DIR/timing_r5_<stamp>.bin and .txt
      python3 ../../goal-to-torque_MPSoC/tools/timing_summary.py DIR/timing_r5_<stamp>.bin --window op

  The summary has the rows of the send (Delta T_send, t_w + t_s and its deadline) that a log of the APU with
  `--arch S1` leaves empty; the two logs carry the same period numbers. The head of the file tells the build:
  `gem3_buffers=ocm|ddr`, `optimized=0|1`, `sync_mode`, the check value of the cfg.
- **The GEM3 descriptors and frame buffers are in the OCM** (0xFFFC_4000–0xFFFC_DFFF, `GEM3_BUF_IN_OCM` 1), so
  that a frame does not pass through the DDR. The start-up prints three checks: the R5 reads back what it writes
  there, the four queue base registers, and the PHY-loopback self-test, which now also shows that the DMA of the
  GEM reaches the OCM (a DMA bus error is said in words, and the bring-up ends). With `-DGEM3_BUF_IN_OCM=0` the
  driver is the one of 2026-11-11, instruction for instruction. Build both: when the OCM does not work the DDR
  build is ready, and the two are compared later (plan 2-14).
- `lscript.ld` links 3 MB instead of 3.9 MB (the image is 0.65 MB): the record ring has the megabyte above.

Host tests added (no board), from this directory:

    gcc -std=gnu11 -DHOST_TEST -Itest/stub -Isrc test/test_loop_timing.c src/loop_timing.c -o /tmp/t_loop && /tmp/t_loop
    gcc -std=gnu11 -DHOST_TEST -Itest/stub -Isrc/osal -Isrc/osal/freertos test/test_osal_clock.c src/osal/freertos/osal.c -o /tmp/t_osal && /tmp/t_osal
    gcc -std=gnu11 -DHOST_TEST -DRECORD_RING_BYTES=1024 -Itest/stub -Isrc -Isrc/oshw/freertos test/test_cycle_record.c src/cycle_record.c -lm -o /tmp/t_rec && /tmp/t_rec
    gcc -std=gnu11 -DHOST_TEST -DRECORD_RING_BYTES=4096 -Isrc/soem -Isrc/osal -Isrc/osal/freertos -Isrc/oshw/freertos -Itest/stub -Isrc test/test_ecat_cycle.c src/loop_timing.c src/cycle_record.c -lm -o /tmp/t_cycle && /tmp/t_cycle
    python3 test/test_r5rec.py
    sh test/check_copies.sh

Checked on a PC: those tests and the five above; 67 deliberately wrong versions of the new code each fail one of
them; every source compiles for the R5 without a warning at `-O0` and `-O2`, for both targets and both places of
the GEM3 buffers (arm-none-eabi-gcc 13, the BSP headers of embeddedsw 2022.1, stand-ins for `xparameters.h` and
`FreeRTOSConfig.h`), and needs no symbol from the BSP that it did not need before; the driver with the buffers in
the OCM and in the DDR passes the model of the GEM of 2026-11-11 (the same output), and the DDR build has the
machine code of 2026-11-11. Not checked: a Vitis build and link, anything on the board. On the board only: that the
DMA of the GEM reaches the OCM, the read of the tick timer, what t_w and the send times are, that Linux reads the
ring while the R5 writes it.

## The second controller of `--shadow`: the control computation on the R5 (2026-11-18)

Plan stage 3 asks whether the control computation of the control task (the high-level controller with the inverse
kinematics and the trajectory generator, and the control law with the dynamics of the arm) fits the 1 ms of the R5
next to the EtherCAT cycle, before the control task moves there (S2). This build measures it without moving
anything: the APU keeps the arm, and tells in every period what it did with its controller; the R5 does the same
with a controller of its own, after its send, and its torque is compared with the APU's and not used.

**What runs.** The classes are those of the application, as they are: `Controller`, `FullDynControllerRT`,
`HighLevelRT` and `ShadowControl` in `src/ctrl/` are copies (`sh test/check_copies.sh` tells when one is not what
the application has now; a copy is updated from the application, never the other way round). What they take from
Linux is in `src/ctrl/port/`, headers with the application's names that the compiler finds in their place:

| File | In place of |
|---|---|
| `port/base/Common.h` | the application's `base/Common.h` (RT-POSIX, a terminal): the types, `read_timer()` and the log lines, which go to `ctrl_port.c` |
| `port/CallTiming.h` | the call timing of the application: off |
| `port/sys/mman.h` | `munlockall()`: nothing to unlock |
| `port/rbdl/addons/urdfreader/urdfreader.h` | the URDF reader: the model comes from `ctrl_model.cpp` |
| `port/rbdl/rbdl_config.h` | the header that RBDL's CMake writes: filled in by hand for a static core without logging |

| File | Content |
|---|---|
| `ctrl_model.cpp`, `.h`, `ctrl_model_indy7.h` | the RBDL model as numbers: what the URDF reader of the application built from `models/indy7.urdf`, added to an empty model body by body. The header is written by `bin/rbdl_model_code` of the application |
| `ctrl_numbers_indy7.h` | the 50 constants of the controller from `INDY7.cfg` (gains, torque limits, the parameters of the high-level controller), written by `bin/ctrl_numbers_header` of the application |
| `shadow_ctrl.cpp`, `.h` | the controller for the C side: built once, then one step per payload; an exception of a library ends as a failed build or a fault answer, never in the firmware |
| `ctrl_port.c`, `.h` | the clock of the classes (the system counter, in ns without a division), their log lines (printed while the controller is built, counted in the loop), and `_gettimeofday()`, which the trajectory generator reads through `std::chrono` and the BSP lacks |

The two generated headers each carry a check value; the check value of the controller is of both and goes back to
the APU in every answer. When `INDY7.cfg` changes a constant of the controller, or the URDF changes, the header is
written again (`cd goal-to-torque_MPSoC/indy7 && make tools`, then `./bin/ctrl_numbers_header -o
../../rpu_work/r5_freertos/src/ctrl/ctrl_numbers_indy7.h`, `./bin/rbdl_model_code models/indy7.urdf -o
../../rpu_work/r5_freertos/src/ctrl/ctrl_model_indy7.h`) and the firmware is built again.

**In the cycle** (`shadow_link.c`, called from `ecat_cycle()`): after the send returns, one look at the Shadow in
slot (`ipc/ipc_layout.h`, layout 3). A payload that is new goes through the controller, and the answer (the torque,
its largest difference to the APU's, the reference, the time of the two parts) into the Shadow out slot. The
payload a cycle finds is that of the period before: the control task is not through with the State of this cycle
when the send returns. Without a new payload nothing runs, so the firmware is as before while the APU runs without
`--shadow`.

- A cycle in which the controller ran ends with it: `t_e` of its record is start to that end, `hl_ns` and `dyn_ns`
  are the times of the high-level controller and of the control law, and the overrun rule (`t_w + t_e <= T`) takes
  it in. The send deadline is held by the send, as before: the controller runs after it.
- The controller is built at the start of the task `ecat`, before the bring-up of the bus. It takes its memory
  from the C heap then (about 80 KB of the 256 KB of `lscript.ld`) and none in a cycle. If it cannot be built, a
  `[shadow] FAIL` line says so and the cycle runs without it.
- Every status report has a line `[shadow] periods of the control task computed N (apart A, failed F), longest
  L us | payloads missed so far M, lines of the controller not printed K`. "Apart": after a payload that did not
  arrive, the motion of this controller is no longer that of the APU, until the next start of a motion; the time
  of such a period still counts, its torque is not compared.
- `SHADOW_CONTROL` (`app_config.h`): 1 = with it (the default), 0 = the firmware without it, which is C only and
  builds in the project as it was. The head of the record ring tells which (`RECORD_BUILD_SHADOW`).

**Building it: a C++ project.** The controller needs three libraries, which are not in this repository; their
sources go into the project next to `src/ctrl`:

| Library | Version | What goes where |
|---|---|---|
| RBDL | ORB-HD/rbdl-orb at `0606f2c`, what the board has (its core is that of RBDL 3.1.3, file for file) | the 12 files of its `src/` (`Constraint_Contact.cc Constraint_Loop.cc Constraints.cc Dynamics.cc Joint.cc Kinematics.cc Logging.cc Model.cc rbdl_errors.cc rbdl_mathutils.cc rbdl_utils.cc rbdl_version.cc`) into `src/third_party/rbdl/src/`, its `include/rbdl/` into `src/third_party/rbdl/include/rbdl/`. No addon: the URDF reader is not built |
| Ruckig | v0.19.4 | `src/ruckig/*.cpp` but `cloud_client.cpp` (11 files) into `src/third_party/ruckig/src/`, `include/ruckig/` into `src/third_party/ruckig/include/ruckig/` |
| Eigen | 3.4.0 | the directory `Eigen/` of headers into `src/third_party/eigen3/Eigen/` |

1. A new application project on the same platform and domain with the language C++ (template "Empty Application
   (C++)"), its `src/` replaced by this `src/` with the three libraries in it. The C files are compiled by the C
   compiler as before, and the link is by `armr5-none-eabi-g++`, which adds the C++ library and `libm`.
2. Both compilers: `-O2` (the verdict is about optimized code; the firmware ran at `-O0` so far), `-DARMR5`,
   `-mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16`, the include paths of "Build" above.
3. The C++ compiler, in addition: `-std=gnu++17` (not `-std=c++17`: the C library then has no `M_PI`),
   `-DNDEBUG`, and the include paths `src`, `src/ctrl`, `src/ctrl/port`, `src/third_party/rbdl/include`,
   `src/third_party/ruckig/include`, `src/third_party/eigen3`.
4. The linker script is `src/lscript.ld` as before; the libraries of the link `-lxil -lfreertos -lgcc -lc
   -lstdc++` in the group.

Compiled here at `-O2`, the controller and the two libraries are about 550 KB of code next to the 115 KB of the C
files, and the C++ library comes on top; the 3 MB that `lscript.ld` links hold it. A firmware of this layout needs
the application and `r5cmd.py` of the same commit: `IPC_LAYOUT` is 3, and each side refuses another number
(`test/test_r5cmd.py` compares the number of `r5cmd.py` with the header's).

Since layout 3 (2026-11-19) the controller does not solve the inverse kinematics: the IK of a goal comes with the
payload of the period in which the APU's controller took it (the APU has a planner task for it). The plan to it
is calculated here, in the cycle in which it is asked for. Both are to move into a task below the EtherCAT task.

**Checked on a PC**, from this directory (the application built first: `make` and `make tools` in
`goal-to-torque_MPSoC/indy7`):

    g++ -std=gnu++17 -O2 -Isrc/ctrl -isystem $RBDL_DIR/include -isystem /usr/include/eigen3 test/check_model.cpp \
        src/ctrl/ctrl_model.cpp -L$RBDL_DIR/lib -lrbdl -lrbdl_urdfreader -Wl,-rpath,$RBDL_DIR/lib -o /tmp/check_model \
        && /tmp/check_model ../../goal-to-torque_MPSoC/indy7/models/indy7.urdf
    (cd ../../goal-to-torque_MPSoC/indy7 && ./bin/shadow_check -o /tmp/shadow_vectors.bin)
    RBDL_SRC=<rbdl-orb> RUCKIG_SRC=<ruckig> sh test/shadow_replay.sh /tmp/shadow_vectors.bin
    gcc -std=gnu11 -Wall -DHOST_TEST -Itest/stub -Isrc -Isrc/ctrl test/test_ctrl_port.c src/ctrl/ctrl_port.c -o /tmp/t_port && /tmp/t_port
    gcc -std=gnu11 -Wall -DHOST_TEST -Itest/stub -Isrc test/test_shadow_link.c src/shadow_link.c -o /tmp/t_shadow && /tmp/t_shadow
    sh test/check_copies.sh

- `check_model`: the model from numbers is the model of the URDF reader, body by body, and gives the same gravity
  torque, inertia matrix, bias force, tool position, orientation and Jacobian at 1000 random states, bit for bit.
- `shadow_check` (the application's): a control task in small goes through gravity compensation, HOME, TRACK with
  20 goals, a stop, a joint move and the corner cases of following (12,457 periods), and the second controller
  gives the torque and the reference of the first bit for bit in every period it follows (12,018 and 10,195).
- `shadow_replay.sh`: the controller as this firmware builds it (the port headers, the two generated headers,
  `shadow_ctrl.cpp`) replays those periods. Built for the PC: 0 difference. Built for the R5 (`arm-none-eabi-g++`
  13.2, run by `qemu-arm`): the largest torque difference is 5.1e-12 N m, the largest reference difference
  1.1e-15 rad (the R5 has no fused multiply-add), and no period takes memory.
- The instructions that the R5 executes per period, counted in that run (`COUNT=N` of `shadow_replay.sh`,
  `test/count_periods.py`): 9,000 without a motion (the control law alone), 34,600 with one (median), 60,000 to
  70,000 in the period of a start, and up to 136,000 while the inverse kinematics runs its 10 iterations of a
  period (61 of the 12,457 periods above 100,000). At 533 MHz that is 65 us for a period of a motion and 256 us for
  the largest at one cycle per instruction, and three times that at three; cache misses and the pipeline are not in
  a count, so the board decides.
- On the emulated R5 the controller takes 60 KB of the heap and 3 KB of stack while it is built, and 5.5 KB of stack
  in a period; the task `ecat` has 32 KB now.
- Every C file of the firmware compiles for the R5 at `-O2` with the headers of the BSP (embeddedsw
  `xilinx_v2022.1`), and all of it together with the controller and the two libraries leaves nothing undefined but
  the BSP, FreeRTOS and the C and C++ libraries.

Not checked: a Vitis build (its compiler is GCC 11.2; the libraries were also compiled as C++17 here, so nothing of
C++20 is needed), the link against the BSP, the board. The times above are an estimate.

## The split into modules (2026-11-15)

`cia402_soem.c` (953 lines) became the modules in the table above, at the request of the firmware's
author, with names that say what a thing is. The code itself did not change. Checked with
arm-none-eabi-gcc 13.2.1 and the flags of the build plus `-Wextra`, compiled to assembly for every
`RUN_LEVEL` (0, 1, 2) with `LOOP_TIMING` 0 and 1: the 28 functions and 23 variables of `cia402_soem.c`
and `main.c` come out as the same instructions and data, names mapped, except three log lines that named a
renamed identifier and the LED task's `st > 0`, now `st == ECAT_STATUS_RUNNING` (the same for the four
values the status takes). No warnings.

| Before | Now | In |
|---|---|---|
| `control_task`, `ecat_cyclic`, `ecat_report`, `diag_al_states` | `ecat_task`, `ecat_cycle`, `post_status_report`, `post_slave_al_states` | `ecat_task.c` |
| `cia402_state` (0, 1, -1, -2) | `ecat_status` (`ECAT_STATUS_STARTING`, `_RUNNING`, `_INIT_FAILED`, `_BUS_FAULT`) | `ecat_task.c` |
| `g_wkc_bad`, `g_wkc_bad_total` | `wkc_bad_cycles`, `wkc_bad_total` | `ecat_task.c` |
| `ecat_init`, `safe_outputs`, `g_expected_wkc` | `ecat_bring_up`, `set_standstill_outputs`, `ecat_expected_wkc` | `ecat_bringup.c` |
| `rxpdo_t`, `txpdo_t`, `rx[]`, `tx[]` | `drive_outputs_t`, `drive_inputs_t`, `drive_out[]`, `drive_in[]` | `cia402_drive.c` |
| `servo_setup`, `SERVO_SDO_WRITES` | `drive_map_pdos`, `PDO_MAPPING_SDO_WRITES` | `cia402_drive.c` |
| `GetStatusVal`, `cia_name`, `CIA_*` | `cia402_state_from_statusword`, `cia402_state_name`, `CIA402_*` (the state names the log prints) | `cia402_drive.c` |
| `cyclic_logic`, `traj_sin` | `axis_control_update`, `sine_of_step` | `axis_control.c` |
| `StatusWord[]`, `PrevStatusWord[]` (they hold the decoded state) | `axis_state[]`, `axis_state_prev[]` | `axis_control.c` |
| `nFirstPos[]`, `fault_resets[]`, `enabled_cycles` | `hold_position[]`, `fault_reset_count[]`, `all_enabled_cycles` | `axis_control.c` |
| `traj_k`, `traj_started`, `traj_stopped` | `sine_step`, `sine_started`, `sine_stopped` | `axis_control.c` |
| `ev_t` (`v[]`), `EV_*`, `ev_begin`, `ev_commit`, `ev_post` | `event_t` (`value[]`), `EVENT_*`, `event_reserve`, `event_publish`, `event_post` | `event_log.c` |
| `evq`, `evq_w`, `evq_r`, `ev_dropped`, `ev_suppressed`, `ev_budget` | `event_queue`, `event_write_index`, `event_read_index`, `events_dropped`, `events_suppressed`, `event_budget` | `event_log.c` |
| `log_task`, `log_drain`, `log_print`, `CIA402_BARRIER` | `event_log_task`, `print_pending_events`, `print_event`, `MEMORY_BARRIER` | `event_log.c` |
| `al_str`, `al_state_str`, `coe_abort_str` | `al_status_code_text`, `al_state_name`, `sdo_abort_code_text` | `ecat_diag.c` |
| `print_ec_errors`, `report_states` | `print_soem_errors`, `print_slave_states` | `ecat_diag.c` |
| `cyc_init`, `cyc_now`, `CYC_PER_US`, `CYC_PER_CYCLE` | `cycle_counter_start`, `cycle_counter_read`, `CPU_CYCLES_PER_US`, `CPU_CYCLES_PER_LOOP` | `loop_timing.h` |
| `tm` (`n`, `per_*`, `ex_max`, `ex_max_all`, `late`), `tm_reset_window`, `tm_sample` | `loop_timing` (`periods`, `period_*`, `exec_max`, `exec_max_run`, `overruns`), `loop_timing_reset_window`, `loop_timing_record` | `loop_timing.c` |
| `SERVO_MOTOR_NUM`, `CIA402_MODE` | `AXIS_COUNT`, `RUN_LEVEL` (`RUN_BUS_ONLY`, `RUN_SERVO_HOLD`, `RUN_SINE`) | `app_config.h` |
| `CIA402_OP_WAIT_MS`, `CIA402_RX_TIMEOUT_US`, `CIA402_TRAJ_DELAY_MS`, `CIA402_MAX_FAULT_RESETS` | `OP_TIMEOUT_MS`, `RX_TIMEOUT_US`, `SINE_START_DELAY_MS`, `MAX_FAULT_RESETS` | `app_config.h` |
| `CIA402_BUSFAULT_CYCLES`, `CIA402_DIAG_CYCLES`, `CIA402_REPORT_S`, `CIA402_EV_PER_WINDOW` | `WKC_BUS_FAULT_CYCLES`, `WKC_DIAG_CYCLES`, `REPORT_PERIOD_S`, `EVENTS_PER_REPORT` | `app_config.h` |
| `CIA402_TIMING`, `CIA402_CPU_HZ`, `TRAJ_PERIOD_CYCLES`, `TRAJ_AMPLITUDE` | `LOOP_TIMING`, `CPU_HZ`, `SINE_PERIOD_CYCLES`, `SINE_AMPLITUDE` | `app_config.h` |
| `CIA402_HOST_TEST`, `cia402_host_cycles` | `HOST_TEST`, `host_test_cycles` | `event_log.c`, `loop_timing.h` |

Kept: `IOmap` (SOEM's own name for it), `CYCLE_NS`, `ENCODER_RESOLUTION`, `CW_*`, `MODE_CSP`, the PDO
field names (the CiA 402 object names), the task names `ecat`, `ecatlog` and `led`, and every log line but
the three above.
