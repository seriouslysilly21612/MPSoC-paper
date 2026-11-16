/*
 * app_config.h  --  the build switches and fixed parameters of the EtherCAT firmware, in one place.
 * Every value inside #ifndef can be set with -D<NAME>=<value> in the compiler symbols of the Vitis project.
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "FreeRTOS.h"
#include "xparameters.h"

/* These switches had a CIA402_ prefix until the split into modules (table in README.md).  A leftover -D of an old
 * name would be ignored without a word, and RUN_LEVEL would fall back to RUN_SINE: the motors would move.         */
#if defined(CIA402_MODE) || defined(CIA402_OP_WAIT_MS) || defined(CIA402_RX_TIMEOUT_US) || defined(CIA402_TRAJ_DELAY_MS) || \
    defined(CIA402_MAX_FAULT_RESETS) || defined(CIA402_BUSFAULT_CYCLES) || defined(CIA402_DIAG_CYCLES) ||                \
    defined(CIA402_REPORT_S) || defined(CIA402_EV_PER_WINDOW) || defined(CIA402_TIMING) || defined(CIA402_CPU_HZ)
#error "an old CIA402_* switch is set: the switches are renamed (CIA402_MODE is RUN_LEVEL now), see README.md"
#endif

#define AXIS_COUNT          4                 /* drives on the bus; exactly this many must answer */
#define ENCODER_RESOLUTION  524288            /* counts per motor turn */
#define CYCLE_NS            1000000u          /* 1 ms = 1 kHz            */

/* What the firmware does once all drives are OPERATIONAL.  Each level adds to the one below. */
#define RUN_BUS_ONLY        0                 /* drives to OPERATIONAL, nothing is enabled, nothing moves */
#define RUN_SERVO_HOLD      1                 /* + servo ON, hold the position the axis had when it enabled */
#define RUN_SINE            2                 /* + sine trajectory (= the IgH program) */
#ifndef RUN_LEVEL
#define RUN_LEVEL           RUN_SINE
#endif
#if (RUN_LEVEL < RUN_BUS_ONLY) || (RUN_LEVEL > RUN_SINE)
#error "RUN_LEVEL must be 0, 1 or 2"
#endif

#ifndef OP_TIMEOUT_MS
#define OP_TIMEOUT_MS       10000             /* how long the drives get to reach OP (process data flowing) */
#endif
#ifndef SAFEOP_FRAMES_MS
#define SAFEOP_FRAMES_MS    200               /* process data at the cycle rate in SAFE-OP before OP is requested */
#endif
_Static_assert(SAFEOP_FRAMES_MS < OP_TIMEOUT_MS, "SAFEOP_FRAMES_MS must be below OP_TIMEOUT_MS");
#ifndef RX_TIMEOUT_US
#define RX_TIMEOUT_US       500               /* the reply to last cycle's frame is long there; 500 us = lost */
#endif
#ifndef SINE_START_DELAY_MS
#define SINE_START_DELAY_MS 500               /* all axes enabled this long before the sine starts */
#endif
#ifndef MAX_FAULT_RESETS
#define MAX_FAULT_RESETS    3                 /* automatic CiA402 fault resets per axis and run */
#endif
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
#ifndef LOOP_TIMING
#define LOOP_TIMING         1                 /* 0 = do not touch the PMU / no timing statistics */
#endif
#ifndef MAX_TORQUE_PERMILLE
#define MAX_TORQUE_PERMILLE 100               /* torque limit set in PRE-OP, 0.1 % of rated (L7N: 1.337 Nm rated, 3000 = its default) */
#endif
_Static_assert(MAX_TORQUE_PERMILLE >= 1 && MAX_TORQUE_PERMILLE <= 1000,
               "MAX_TORQUE_PERMILLE is the limit for the first servo-on: 1..1000 (0.1 % .. 100 % of rated torque)");

#ifndef CPU_HZ
#if defined(XPAR_CPU_CORE_CLOCK_FREQ_HZ)
#define CPU_HZ              XPAR_CPU_CORE_CLOCK_FREQ_HZ
#elif defined(XPAR_PSU_CORTEXR5_0_CPU_CLK_FREQ_HZ)
#define CPU_HZ              XPAR_PSU_CORTEXR5_0_CPU_CLK_FREQ_HZ
#else
#define CPU_HZ              500000000UL
#endif
#endif

/* The loop is paced with vTaskDelayUntil(1 tick) and counts 1 ms per call. */
_Static_assert(configTICK_RATE_HZ == 1000,
               "FreeRTOS tick must be 1000 Hz: set tick_rate=1000 in the freertos10_xilinx BSP settings and rebuild the platform");

#define SINE_PERIOD_CYCLES  10000u            /* 0.1 Hz at 1 kHz */
#define SINE_AMPLITUDE      (ENCODER_RESOLUTION / 2)
_Static_assert((SINE_PERIOD_CYCLES % 4u) == 0u, "SINE_PERIOD_CYCLES must be a multiple of 4");

#endif /* APP_CONFIG_H */
