/*
 * app_config.h  --  the build switches and fixed parameters of the EtherCAT firmware, in one place.
 * Every value inside #ifndef can be set with -D<NAME>=<value> in the compiler symbols of the Vitis project.
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "FreeRTOS.h"
#include "xparameters.h"

/* These switches were renamed or removed.  A leftover -D of an old name would be ignored without a word, so it is
 * an error: the table of old names is in README.md.                                                                */
#if defined(CIA402_MODE) || defined(CIA402_OP_WAIT_MS) || defined(CIA402_RX_TIMEOUT_US) || defined(CIA402_TRAJ_DELAY_MS) || \
    defined(CIA402_MAX_FAULT_RESETS) || defined(CIA402_BUSFAULT_CYCLES) || defined(CIA402_DIAG_CYCLES) ||                \
    defined(CIA402_REPORT_S) || defined(CIA402_EV_PER_WINDOW) || defined(CIA402_TIMING) || defined(CIA402_CPU_HZ)
#error "an old CIA402_* switch is set: the switches are renamed, see README.md"
#endif
#if defined(RUN_LEVEL)
#error "RUN_LEVEL is gone: the firmware no longer moves anything by itself. Remove -DRUN_LEVEL; servo on / off come from the APU (Command slot)"
#endif
#if defined(LOOP_TIMING) || defined(CPU_HZ)
#error "LOOP_TIMING and CPU_HZ are gone: every cycle is timed, with the system counter (sys_counter.h). Remove the -D"
#endif

/* ----------------------------------------------------------------------- */
/* Which hardware this build drives.  0 = the L7N bench, 1 = the Indy7.     */
/* Change the number here (or give -DROBOT_TARGET=1).  Everything that      */
/* differs between the two lives under this switch: the number of axes and  */
/* which constants of the cfg are used (robot_target.h: identity, units).    */
/* ----------------------------------------------------------------------- */
#define TARGET_L7N_BENCH    0                 /* three LS Mecapion L7N single-axis drives, motors lying flat */
#define TARGET_INDY7        1                 /* the Indy7: six joint drives + the end-tool module */
#ifndef ROBOT_TARGET
#define ROBOT_TARGET        TARGET_L7N_BENCH
#endif

#if ROBOT_TARGET == TARGET_L7N_BENCH
#define AXIS_COUNT          3                 /* drives that are driven.  The bench has one more, with firmware 0.12, the third on the
                                                 bus until 2026-11-19: taken off, it left OP at the start of 4 runs of 5 (bench/L7N3_bus.cfg) */
#define BUS_SLAVE_COUNT     3                 /* slaves on the bus; exactly this many must answer */
#define ECAT_USE_DC         1                 /* DC SYNC0 on the drives, as the bench has run since 2026-11-16: the drive with firmware
                                                 0.12 only stayed in OP with it (I-40).  The three left (0.22) stayed in OP without it
                                                 too; whether the bench goes back to free run is open (D-13) */
#elif ROBOT_TARGET == TARGET_INDY7
#define AXIS_COUNT          6
#define BUS_SLAVE_COUNT     7                 /* + the end-tool module as slave 7 (shortcut: left as the bus scan finds it, no PDO hook, no DC) */
#define ECAT_USE_DC         0                 /* free run, as the old Indy7 build ran (INDY7.cfg has no DC_SUPPORT key, 2026-11-17) */
#else
#error "ROBOT_TARGET must be TARGET_L7N_BENCH (0) or TARGET_INDY7 (1)"
#endif

/* ----------------------------------------------------------------------- */
/* The control task on the R5 (control_link.h; S2): 1 = the control          */
/* computation runs here, after the send of every cycle, on commands and    */
/* goals of the APU, and its torque goes to the drives; the Torque slot of  */
/* the APU is not taken.  0 = the torque is the APU's (S1).  The same C++    */
/* project as SHADOW_CONTROL below; the two exclude each other.             */
/* ----------------------------------------------------------------------- */
#ifndef CONTROL_TASK
#define CONTROL_TASK        0
#endif

/* ----------------------------------------------------------------------- */
/* The second controller of --shadow (shadow_link.h; plan stage 3): 1 = the */
/* control computation of the APU's control task runs here too, after the   */
/* send of every cycle, and its torque is compared, not used.  It is C++    */
/* with RBDL, Ruckig and Eigen: a C++ project with the sources of src/ctrl  */
/* and of the three libraries (README.md).  0 = the firmware without it, C. */
/* ----------------------------------------------------------------------- */
#ifndef SHADOW_CONTROL
#define SHADOW_CONTROL      (!CONTROL_TASK)
#endif
#if CONTROL_TASK && SHADOW_CONTROL
#error "CONTROL_TASK and SHADOW_CONTROL: one controller in a build. With -DCONTROL_TASK=1 leave SHADOW_CONTROL unset"
#endif

#define CYCLE_NS            1000000u          /* 1 ms = 1 kHz            */
#define DC_SHIFT_NS         125000            /* SYNC0 shift on the DC time grid (IgH: 12500) */

#ifndef OP_TIMEOUT_MS
#define OP_TIMEOUT_MS       10000             /* how long the drives get to reach OP (process data flowing) */
#endif
#ifndef RX_TIMEOUT_US
#define RX_TIMEOUT_US       200               /* the reply to last cycle's frame is long there; 200 us = lost, and the send
                                                 of the cycle still leaves before its deadline (loop_timing.h) */
#endif
#ifndef MAX_FAULT_RESETS
#define MAX_FAULT_RESETS    3                 /* automatic CiA402 fault resets per axis and servo-on */
#endif
#ifndef TORQUE_STALE_LIMIT
#define TORQUE_STALE_LIMIT  10                /* cycles in a row with the servo requested and no new Torque slot: torque 0 and servo off (D-03, D-10) */
#endif
_Static_assert(TORQUE_STALE_LIMIT >= 1, "TORQUE_STALE_LIMIT is a number of 1 ms cycles, at least 1");
#ifndef WKC_BUS_FAULT_CYCLES
#define WKC_BUS_FAULT_CYCLES 100              /* incomplete working counter this many cycles in a row = BUS FAULT */
#endif
#ifndef WKC_DIAG_CYCLES
#define WKC_DIAG_CYCLES     10                /* ... after this many: read + print every slave's AL state / status code once */
#endif
_Static_assert(WKC_DIAG_CYCLES >= 1 && WKC_DIAG_CYCLES < WKC_BUS_FAULT_CYCLES,
               "WKC_DIAG_CYCLES must be at least 1 and below WKC_BUS_FAULT_CYCLES");
#ifndef REPORT_PERIOD_S
#define REPORT_PERIOD_S     10                /* one status report every N seconds */
#endif
#ifndef EVENTS_PER_REPORT
#define EVENTS_PER_REPORT   40                /* max. event lines per report window (flood guard) */
#endif
#ifndef MAX_TORQUE_PERMILLE
#define MAX_TORQUE_PERMILLE 100               /* torque limit set in PRE-OP, 0.1 % of rated (L7N: 1.337 Nm rated, 3000 = its default) */
#endif
_Static_assert(MAX_TORQUE_PERMILLE >= 1 && MAX_TORQUE_PERMILLE <= 1000,
               "MAX_TORQUE_PERMILLE is the limit for the first servo-on: 1..1000 (0.1 % .. 100 % of rated torque)");

/* The loop is paced with vTaskDelayUntil(1 tick) and counts 1 ms per call. */
_Static_assert(configTICK_RATE_HZ == 1000,
               "FreeRTOS tick must be 1000 Hz: set tick_rate=1000 in the freertos10_xilinx BSP settings and rebuild the platform");

#endif /* APP_CONFIG_H */
