# R5F-0 firmware: SOEM EtherCAT master on GEM3

The FreeRTOS application of the R5F-0 (split mode) as it ran on the KV260 on 2026-11-11: the EtherCAT
master (SOEM 1.4.0) on GEM3, which Linux leaves to the R5, driving the L7N bench (4 axes, CSP, DC on,
1 kHz). Commit `ddc6d14` holds the source of that build unchanged. Since then the application is split
into modules with new names, without a change in the generated code (below), and the master no longer
sets up distributed clocks (FREE_RUN, below). Torque mode and the GEM3 buffers in OCM come as later
commits.

| Path | Content | Origin |
|---|---|---|
| `src/main.c` | start-up and two tasks: `ecat` (the master, priority `configMAX_PRIORITIES - 2`) and `led` | own |
| `src/app_config.h` | the build switches and fixed parameters: axes, cycle time, `RUN_LEVEL`, timeouts, report period | own |
| `src/ecat_task.c`, `.h` | the task `ecat`: bring-up, then the 1 kHz cycle (receive, working-counter check, axis logic, send) and a status report every 10 s | own, a translation of the IgH example `cia402_basic.c` |
| `src/ecat_bringup.c`, `.h` | bring-up to OP: bus scan, PDO mapping, the drives' sync parameters printed, SAFE-OP, OP with process data flowing | own, from the same example |
| `src/cia402_drive.c`, `.h` | the drive's process data (packed structs over the IOmap), PDO mapping by SDO, statusword → CiA 402 state | own, from the same example |
| `src/axis_control.c`, `.h` | what one cycle does with the axes: CiA 402 state machine (enable, fault reset), hold, sine trajectory | own, from the same example |
| `src/event_log.c`, `.h` | the loop never prints: it posts events to a queue, the task `ecatlog` prints them at low priority | own |
| `src/ecat_diag.c`, `.h` | AL states, AL status codes and SDO abort codes in words, SOEM's error list, the slave states | own |
| `src/loop_timing.c`, `.h` | loop period and execution time from the PMU cycle counter | own |
| `src/oshw/freertos/gem3_nic.c`, `.h` | the GEM3 driver below SOEM: clock and reset, PHY reset and MDIO, DMA descriptors and buffers, polled send and receive, a PHY-loopback self-test | own |
| `src/oshw/freertos/nicdrv.c`, `.h`, `oshw.c`, `.h` | the SOEM hardware layer for FreeRTOS | own, from the SOEM templates (GPL-2.0 with exception) |
| `src/osal/freertos/` | the SOEM OS layer for FreeRTOS | own |
| `src/oshw/freertos/r5log.c`, `.h` | `outbyte()` into a ring buffer in DDR: the BSP console (JTAG DCC) blocks for ever without a debugger, as under remoteproc. Linux reads it with `r5log.py` | own |
| `src/oshw/freertos/xemacps*` | Xilinx emacps driver 3.16 (embeddedsw `xilinx_v2022.1`), unchanged | Xilinx, MIT |
| `src/soem/`, `src/osal/osal.h` | SOEM 1.4.0, unchanged | SOEM, GPL-2.0 with exception, `LICENSE.SOEM` |
| `src/lscript.ld` | the linker script: code and data in DDR 0x3600_0000–0x363E_FFFF, stack 8 KB, heap 8 KB. The last 64 KB of the 4 MB that `rpu_v2.dtbo` reserves stays out of the link: a non-cacheable window with the GEM3 descriptor rings (from 0x363F_0000) and the log ring (0x363F_C000). The frame buffers are cacheable `.bss` | own, from the Vitis template |
| `bsp_ref/FreeRTOSConfig.h` | the FreeRTOS configuration the BSP generates from `system.mss` (tick 1000 Hz, heap 256 KB, 8 priorities), to read only | generated |
| `r5_freertos.elf` | the build that ran, 936,504 bytes, sha256 `5d2e82c908e3cfaad145abfcb7496b4699455c1d81fe95db70fbf9d872e188eb`: the same file as `/lib/firmware/r5_freertos.elf` on the board (checked 2026-11-12). Built from the source of `ddc6d14` (`cia402_soem.c` in one file) | build |
| `platform/design_1_wrapper.xsa` | the hardware hand-off from Vivado 2022.1 (board `kv260_som` 1.3): the PS only, no PL, GEM3 not enabled | Vivado |
| `platform/system.mss` | the BSP settings of the R5 domain (`freertos10_xilinx` 1.11 on `psu_cortexr5_0`); its md5 is the one `platform.spr` records for that domain | Vitis |
| `platform/platform.spr` | the Vitis platform: the FSBL and PMU firmware domains and the R5 domain, with their BSP compiler flags | Vitis |
| `user-override.dtb` | the device tree the board boots with this firmware, sha256 `b1f13abe9be882fb887bfd6fdc8bc540edcd0a8acdb6facbb2064befa2a34207`: byte for byte `fdtoverlay -i ../base.dtb -o user-override.dtb ../rpu_v2.dtbo ../gem3_off.dtbo` | board |
| `../gem3_off.dtso`, `.dtbo` | the overlay that disables GEM3 (`ethernet@ff0e0000`) for Linux; the `.dtbo` is `dtc -@` of the `.dtso` | own |
| `r5log.py` | reads the log ring through `/dev/mem` on Linux (single-byte loads: the carveout is Device memory) | own |

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
   - `RUN_LEVEL` (`src/app_config.h`, `CIA402_MODE` before the split): 0 = bus only, 1 = + servo on and hold,
     2 = + sine trajectory (the default). The ELF was built with 2. A leftover `-DCIA402_...` stops the build
     with an error, so that an old `-DCIA402_MODE=0` cannot silently turn into level 2.
   - `GEM3_*` (`gem3_nic.c`): descriptor window 0x363F_0000 (64 KB, must match `lscript.ld`), self-test on, PHY reset on MIO 38.
   - `R5LOG_ADDR` (`r5log.c`): the log ring at 0x363F_C000, 16 KB.

## Run

- The device tree: this directory's `user-override.dtb` into `/boot/firmware/` (back up the old one), then
  reboot. To change an overlay, merge again from `base.dtb`, never on top of a merged tree. The
  `user-override.dtb` at the top of `rpu_work/` is the configuration before the move (`base.dtb` +
  `rpu_v2.dtbo`, the board's `user-override.dtb.bak_gem3`: Linux drives GEM3 with IgH); do not boot this
  firmware with it.
- The firmware: `r5_freertos.elf` into `/lib/firmware/`, then remoteproc0 as in the top `README.md`.
- The log: `sudo python3 r5log.py -f` on Linux. The ring survives an R5 hang; read it before starting the R5 again.
- Safety: one EtherCAT master at a time. Never stop the R5 while a motor moves (E-stop or drive power first).
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
