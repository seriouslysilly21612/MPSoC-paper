# R5F-0 firmware: SOEM EtherCAT master on GEM3

The FreeRTOS application of the R5F-0 (split mode) as it ran on the KV260 on 2026-11-11: the EtherCAT
master (SOEM 1.4.0) on GEM3, which Linux leaves to the R5, driving the L7N bench (4 axes, CSP, DC on,
1 kHz). This directory is the source of that build, unchanged. The changes for the paper (DC removed,
torque mode, the GEM3 buffers in OCM) come as later commits.

| Path | Content | Origin |
|---|---|---|
| `src/main.c` | start-up and two tasks: `ecat` (the master, priority `configMAX_PRIORITIES - 2`) and `led` | own |
| `src/cia402_soem.c` | the 1 kHz cycle: bring-up to OP, PDO mapping by SDO, the CiA 402 state machine, the sine trajectory, loop statistics; the task `ecatlog` prints them at low priority | own, a translation of the IgH example `cia402_basic.c` |
| `src/oshw/freertos/gem3_nic.c`, `.h` | the GEM3 driver below SOEM: clock and reset, PHY reset and MDIO, DMA descriptors and buffers, polled send and receive, a PHY-loopback self-test | own |
| `src/oshw/freertos/nicdrv.c`, `.h`, `oshw.c`, `.h` | the SOEM hardware layer for FreeRTOS | own, from the SOEM templates (GPL-2.0 with exception) |
| `src/osal/freertos/` | the SOEM OS layer for FreeRTOS | own |
| `src/oshw/freertos/r5log.c`, `.h` | `outbyte()` into a ring buffer in DDR: the BSP console (JTAG DCC) blocks for ever without a debugger, as under remoteproc. Linux reads it with `r5log.py` | own |
| `src/oshw/freertos/xemacps*` | Xilinx emacps driver 3.16 (embeddedsw `xilinx_v2022.1`), unchanged | Xilinx, MIT |
| `src/soem/`, `src/osal/osal.h` | SOEM 1.4.0, unchanged | SOEM, GPL-2.0 with exception, `LICENSE.SOEM` |
| `src/lscript.ld` | the linker script: code and data in DDR 0x3600_0000–0x363E_FFFF, stack 8 KB, heap 8 KB. Of the 4 MB that `rpu_v2.dtbo` reserves, the last 64 KB is the GEM3 DMA area | own, from the Vitis template |
| `bsp_ref/FreeRTOSConfig.h` | the FreeRTOS configuration the BSP generates from `system.mss` (tick 1000 Hz, heap 256 KB, 8 priorities), to read only | generated |
| `r5_freertos.elf` | the build that ran, 936,504 bytes, sha256 `5d2e82c908e3cfaad145abfcb7496b4699455c1d81fe95db70fbf9d872e188eb`: the same file as `/lib/firmware/r5_freertos.elf` on the board (checked 2026-11-12) | build |
| `platform/design_1_wrapper.xsa` | the hardware hand-off from Vivado 2022.1 (board `kv260_som` 1.3): the PS only, no PL, GEM3 not enabled | Vivado |
| `platform/system.mss` | the BSP settings of the R5 domain (`freertos10_xilinx` 1.11 on `psu_cortexr5_0`); its md5 is the one `platform.spr` records for that domain | Vitis |
| `platform/platform.spr` | the Vitis platform: the FSBL and PMU firmware domains and the R5 domain, with their BSP compiler flags | Vitis |

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
   - `CIA402_MODE` (`src/cia402_soem.c`): 0 = bus only, 1 = + servo on and hold, 2 = + sine trajectory. Built with 2.
   - `GEM3_*` (`gem3_nic.c`): DMA area 0x363F_0000 (64 KB, must match `lscript.ld`), self-test on, PHY reset on MIO 38.
   - `R5LOG_ADDR` (`r5log.c`): the log ring at 0x363F_C000, 16 KB.

## Run

- The device tree: `base.dtb` with `rpu_v2.dtbo` and the overlay that disables GEM3 for Linux
  (`gem3_off.dtbo`), always merged anew from `base.dtb`. The `user-override.dtb` at the top of `rpu_work/`
  is the configuration before the move (the board's `user-override.dtb.bak_gem3`, same sha256: Linux drives
  GEM3 with IgH); do not boot this firmware with it.
- The firmware: `r5_freertos.elf` into `/lib/firmware/`, then remoteproc0 as in the top `README.md`.
- The log: `sudo python3 r5log.py -f` on Linux.
- Safety: one EtherCAT master at a time. Never stop the R5 while a motor moves (E-stop or drive power first).
  Do not restart Linux while the R5 runs the bus.
