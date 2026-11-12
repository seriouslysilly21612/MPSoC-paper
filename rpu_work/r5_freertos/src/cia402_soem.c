/*
 * cia402_soem.c  --  4-axis CiA402 (CSP) servo control, 1 kHz, DC enabled.
 *
 * Translation of the IgH/EtherLab "servo_drive_dc/cia402_basic.c" to
 * SOEM + FreeRTOS, for the KV260 RPU (Cortex-R5F).  Entry point: control_task().
 *
 * What was REUSED (application logic, EtherCAT-master-agnostic):
 *   - CiA402 drive state machine (SWITCH_ON_DISABLED -> ... -> OPERATION_ENABLED), GetStatusVal()
 *   - sine target position: nFirstPos + (ENCODER_RESOLUTION/2) * sin(2*pi*0.1*t), t in 1 ms steps
 * What was TRANSLATED (IgH ecrt_* -> SOEM ec_*):
 *   - master / domain / PDO / DC setup, process-data access through packed structs over IOmap
 * What is NEW for the embedded target:
 *   - PDO mapping by CoE SDO writes (PO2SOconfig hook), FreeRTOS task instead of a Linux RT thread,
 *     r5log-friendly logging (the 1 kHz loop never prints), loop period / execution time statistics
 *
 * REV 2  (found by comparing rev 1 with the IgH original and with SOEM's simple_test)
 *   1. Bring-up order = IgH:  PRE-OP -> PDO mapping -> DC + SYNC0 -> SAFE-OP -> process data FLOWING AT
 *      THE CYCLE RATE -> OP.  rev 1 asked for OP without sending a single process-data frame while it
 *      waited: drives with DC / SM watchdog never leave SAFE-OP that way, and SOEM's DC drift
 *      compensation (the FRMW datagram inside every process-data frame) never ran.
 *   2. WKC gate (IgH: domain wc_state == complete): the CiA402 state machine and the trajectory only run
 *      on a cycle whose working counter is complete.  A lost frame pauses the trajectory, it never
 *      makes it jump.  100 incomplete cycles in a row = "BUS FAULT" (cia402_state = -2).
 *   3. PDO size check: what the drive reports must equal sizeof(rxpdo_t)/sizeof(txpdo_t), otherwise the
 *      packed structs would be written to the wrong offsets.  Init fails instead; nothing moves.
 *      Exactly SERVO_MOTOR_NUM slaves must be on the bus.
 *   4. target_position follows position_actual until an axis is OPERATION_ENABLED (no step when it
 *      enables); the sine starts only after ALL axes were enabled for CIA402_TRAJ_DELAY_MS (brake release).
 *   5. If an axis leaves OPERATION_ENABLED after the sine started (fault, quick stop, ...) the sine stops
 *      for good.  rev 1 restarted it from a stale phase after the fault reset = a step of up to half a turn.
 *      Automatic fault resets are limited to CIA402_MAX_FAULT_RESETS per axis (IgH reset forever).
 *   6. CIA402_MODE:  0 = bus only (drives to OPERATIONAL, nothing is enabled, nothing moves)
 *                    1 = + servo ON, hold the position the axis had when it enabled
 *                    2 = + sine trajectory   (default, = the IgH program)
 *   7. Logging: bring-up lines are printed directly; everything in the 1 kHz loop is posted to a small
 *      queue and printed by a low-priority task, so a slow console never delays a cycle.  xil_printf goes
 *      to the r5log ring (r5log.c MUST be in the project: the BSP's own console blocks forever without
 *      a JTAG host).  Read it on Linux with "sudo python3 r5log.py -f".
 *   8. The FPU is declared (portTASK_USES_FLOATING_POINT), sin() is computed locally (no -lm needed),
 *      the build fails with a clear message if the FreeRTOS tick is not 1000 Hz.
 *   9. Diagnostics: an incomplete working counter that lasts CIA402_DIAG_CYCLES cycles makes control_task
 *      read every slave's AL state + AL status code ONCE (ec_readstate) and print it next to the CiA402
 *      statusword.  wkc = 12 - 2 means ONE drive stopped taking outputs (it fell out of OPERATIONAL); that line
 *      says which drive and why (0x001B SM watchdog, 0x001A sync error, ...).  Nothing else changes.
 *
 * Timing: the loop is paced with vTaskDelayUntil(1 tick).  That is phase-free w.r.t. the EtherCAT
 * distributed clock (SYNC0).  The printed "period"/"exec" statistics show how good that is; a
 * hardware-timer ISR aligned to SYNC0 is the next step (see control_task()).
 */

#include "FreeRTOS.h"
#include "task.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "ethercat.h"

/* gem3_nic.c (rev4): NWSR/TXSR/RXSR, frame counters and the PHY link bits - printed when the bus scan finds nobody. */
extern void gem3_debug_dump(void);

/* ----------------------------------------------------------------------- */
/* Config                                                                  */
/* ----------------------------------------------------------------------- */
#define SERVO_MOTOR_NUM     4
#define ENCODER_RESOLUTION  524288
#define CYCLE_NS            1000000u          /* 1 ms = 1 kHz            */
#define DC_SHIFT_NS         125000            /* SYNC0 shift on the DC time grid (IgH: 12500) */

#ifndef CIA402_MODE
#define CIA402_MODE         2
#endif
#if (CIA402_MODE < 0) || (CIA402_MODE > 2)
#error "CIA402_MODE must be 0, 1 or 2"
#endif
#ifndef CIA402_OP_WAIT_MS
#define CIA402_OP_WAIT_MS       10000         /* how long the drives get to reach OP (process data flowing) */
#endif
#ifndef CIA402_RX_TIMEOUT_US
#define CIA402_RX_TIMEOUT_US    500           /* the reply to last cycle's frame is long there; 500 us = lost */
#endif
#ifndef CIA402_TRAJ_DELAY_MS
#define CIA402_TRAJ_DELAY_MS    500           /* all axes enabled this long before the sine starts */
#endif
#ifndef CIA402_MAX_FAULT_RESETS
#define CIA402_MAX_FAULT_RESETS 3             /* automatic CiA402 fault resets per axis and run */
#endif
#ifndef CIA402_BUSFAULT_CYCLES
#define CIA402_BUSFAULT_CYCLES  100           /* incomplete working counter this many cycles in a row */
#endif
#ifndef CIA402_DIAG_CYCLES
#define CIA402_DIAG_CYCLES      10            /* ... after this many: read + print every slave's AL state / status code once */
#endif
_Static_assert(CIA402_DIAG_CYCLES >= 1 && CIA402_DIAG_CYCLES < CIA402_BUSFAULT_CYCLES,
               "CIA402_DIAG_CYCLES must be at least 1 and below CIA402_BUSFAULT_CYCLES");
#ifndef CIA402_REPORT_S
#define CIA402_REPORT_S         10            /* one status report every N seconds */
#endif
#ifndef CIA402_EV_PER_WINDOW
#define CIA402_EV_PER_WINDOW    40            /* max. event lines per report window (flood guard) */
#endif
#ifndef CIA402_TIMING
#define CIA402_TIMING           1             /* 0 = do not touch the PMU / no timing statistics */
#endif

#ifndef CIA402_CPU_HZ
#if defined(XPAR_CPU_CORE_CLOCK_FREQ_HZ)
#define CIA402_CPU_HZ       XPAR_CPU_CORE_CLOCK_FREQ_HZ
#elif defined(XPAR_PSU_CORTEXR5_0_CPU_CLK_FREQ_HZ)
#define CIA402_CPU_HZ       XPAR_PSU_CORTEXR5_0_CPU_CLK_FREQ_HZ
#else
#define CIA402_CPU_HZ       500000000UL
#endif
#endif

/* The loop below is paced with vTaskDelayUntil(1 tick) and counts 1 ms per call. */
_Static_assert(configTICK_RATE_HZ == 1000,
               "FreeRTOS tick must be 1000 Hz: set tick_rate=1000 in the freertos10_xilinx BSP settings and rebuild the platform");

#define TRAJ_PERIOD_CYCLES  10000u            /* 0.1 Hz at 1 kHz */
#define TRAJ_AMPLITUDE      (ENCODER_RESOLUTION / 2)
_Static_assert((TRAJ_PERIOD_CYCLES % 4u) == 0u, "TRAJ_PERIOD_CYCLES must be a multiple of 4");

/* ----------------------------------------------------------------------- */
/* Process-data layout (must match the PDO mapping programmed below).      */
/* EtherCAT payload is little-endian, R5 is little-endian -> direct access.*/
/* The fields sit at odd offsets: plain unaligned loads/stores, fine on the R5F in normal memory  */
/* (SCTLR.A = 0 as set by the Xilinx BSP).  A data abort on one of these -> build -mno-unaligned-access. */
/* ----------------------------------------------------------------------- */
typedef struct __attribute__((__packed__)) {   /* RxPDO 0x1600 (master->slave) */
    uint16 controlword;          /* 0x6040 */
    int32  target_position;      /* 0x607A */
    int32  target_velocity;      /* 0x60FF */
    int8   modes_of_operation;   /* 0x6060 */
} rxpdo_t;

typedef struct __attribute__((__packed__)) {   /* TxPDO 0x1A00 (slave->master) */
    uint16 statusword;           /* 0x6041 */
    int32  position_actual;      /* 0x6064 */
    int32  velocity_actual;      /* 0x606C */
    int8   modes_display;        /* 0x6061 */
} txpdo_t;

_Static_assert(sizeof(rxpdo_t) == 11 && sizeof(txpdo_t) == 11, "PDO structs must be 11 bytes each");

#define SERVO_SDO_WRITES    19    /* number of ec_SDOwrite() calls in servo_setup() */

static char IOmap[4096] __attribute__((aligned(4)));
static rxpdo_t *rx[SERVO_MOTOR_NUM];   /* -> ec_slave[i+1].outputs */
static txpdo_t *tx[SERVO_MOTOR_NUM];   /* -> ec_slave[i+1].inputs  */

/* 0 = starting, 1 = running (all drives OPERATIONAL), -1 = init failed / parked, -2 = bus fault.  Read by the LED task. */
volatile int cia402_state = 0;

static int      g_expected_wkc;
static int      g_wkc_bad;             /* consecutive cycles with an incomplete working counter */
static unsigned g_wkc_bad_total;

#if defined(__arm__) && !defined(CIA402_HOST_TEST)
#define CIA402_BARRIER()    __asm__ volatile ("dmb" : : : "memory")
#else
#define CIA402_BARRIER()    __asm__ volatile ("" : : : "memory")
#endif

/* ----------------------------------------------------------------------- */
/* Cycle counter (Cortex-R5 PMU PMCCNTR) -> loop period / execution time   */
/* ----------------------------------------------------------------------- */
#if CIA402_TIMING
#if defined(__arm__) && !defined(CIA402_HOST_TEST)
static inline void cyc_init(void)
{
    uint32_t v;
    __asm__ volatile ("mrc p15, 0, %0, c9, c12, 0" : "=r"(v));      /* PMCR */
    v |= 0x5U;                                                      /* E = enable, C = reset the cycle counter */
    v &= ~0x8U;                                                     /* D = 0: count every CPU cycle */
    __asm__ volatile ("mcr p15, 0, %0, c9, c12, 0" : : "r"(v));
    __asm__ volatile ("mcr p15, 0, %0, c9, c12, 1" : : "r"(0x80000000U));   /* PMCNTENSET: cycle counter on */
}
static inline uint32_t cyc_now(void)
{
    uint32_t v;
    __asm__ volatile ("mrc p15, 0, %0, c9, c13, 0" : "=r"(v));      /* PMCCNTR */
    return v;
}
#else
extern uint32_t cia402_host_cycles(void);                          /* host test provides it */
static inline void cyc_init(void) { }
static inline uint32_t cyc_now(void) { return cia402_host_cycles(); }
#endif
#else
static inline void cyc_init(void) { }
static inline uint32_t cyc_now(void) { return 0U; }
#endif

#define CYC_PER_US      ((uint32_t)((CIA402_CPU_HZ + 500000UL) / 1000000UL))
#define CYC_PER_CYCLE   (CYC_PER_US * (CYCLE_NS / 1000u))

static struct {
    uint32_t prev_t0;
    uint32_t warm;                     /* iterations seen; the first period starts at an arbitrary point inside a tick */
    uint32_t n;                        /* period samples in this window */
    uint64_t per_sum;
    uint32_t per_min, per_max, ex_max; /* this window, CPU cycles */
    uint32_t ex_max_all;               /* whole run */
    uint32_t late;                     /* cycles whose execution took longer than one period */
} tm;

static void tm_reset_window(void)
{
    tm.n = 0U; tm.per_sum = 0U; tm.per_min = 0xFFFFFFFFu; tm.per_max = 0U; tm.ex_max = 0U;
}

static void tm_sample(uint32_t t0, uint32_t t1)
{
#if CIA402_TIMING
    uint32_t per, ex = t1 - t0;
    if (tm.warm < 2U) {
        tm.warm++;                     /* iterations 1 and 2 are not on the tick grid yet */
    } else {
        per = t0 - tm.prev_t0;
        tm.n++;
        tm.per_sum += per;
        if (per < tm.per_min) tm.per_min = per;
        if (per > tm.per_max) tm.per_max = per;
    }
    tm.prev_t0 = t0;
    if (ex > tm.ex_max) tm.ex_max = ex;
    if (ex > tm.ex_max_all) tm.ex_max_all = ex;
    if (ex > CYC_PER_CYCLE) tm.late++;
#else
    (void)t0; (void)t1;
#endif
}

/* ----------------------------------------------------------------------- */
/* Event queue: control_task posts numbers, a low-priority task prints.    */
/* Single producer (control_task) / single consumer (ecat_log task).       */
/* ----------------------------------------------------------------------- */
enum {
    EV_AXIS = 1,       /* v0 axis, v1 statusword, v2 CiA402 state                  */
    EV_WKC_LOST,       /* v0 wkc, v1 expected wkc                                   */
    EV_WKC_OK,         /* v0 number of bad cycles                                   */
    EV_TRAJ_START,
    EV_TRAJ_STOP,      /* v0 axis, v1 statusword                                    */
    EV_FAULT_LIMIT,    /* v0 axis                                                   */
    EV_BUSFAULT,       /* v0 number of bad cycles                                   */
    EV_REPORT,         /* see ecat_report()                                         */
    EV_SLAVE_AL        /* v0 slave (1-based), v1 AL state (0x10 = error flag), v2 AL status code, v3 statusword, v4 position */
};

#define EVQ_N   32U                                   /* power of two */
#define EV_NV   (10 + 2 * SERVO_MOTOR_NUM)
typedef struct { uint32_t id; int32_t v[EV_NV]; } ev_t;

static ev_t              evq[EVQ_N];
static volatile uint32_t evq_w, evq_r;
static volatile uint32_t ev_dropped;                  /* queue full */
static uint32_t          ev_suppressed;               /* over the per-window budget */
static int               ev_budget = CIA402_EV_PER_WINDOW;

/* returns a free slot (fill it, then ev_commit()) or NULL; limited = subject to the per-window budget */
static ev_t *ev_begin(uint32_t id, int limited)
{
    ev_t *e;
    int k;
    uint32_t w = evq_w;

    if (limited && ev_budget <= 0) { ev_suppressed++; return NULL; }
    if ((uint32_t)(w - evq_r) >= EVQ_N) { ev_dropped++; return NULL; }
    if (limited) ev_budget--;
    e = &evq[w % EVQ_N];
    e->id = id;
    for (k = 0; k < EV_NV; k++) e->v[k] = 0;
    return e;
}

static void ev_commit(void)
{
    CIA402_BARRIER();                  /* slot contents first, then the index */
    evq_w = evq_w + 1U;
}

static void ev_post(uint32_t id, int limited, int a, int b, int c)
{
    ev_t *e = ev_begin(id, limited);
    if (e != NULL) {
        e->v[0] = a; e->v[1] = b; e->v[2] = c;
        ev_commit();
    }
}

/* ----------------------------------------------------------------------- */
/* CiA402 state machine constants (REUSED from the IgH version, CIA_ prefix */
/* added so they cannot collide with BSP macros)                           */
/* ----------------------------------------------------------------------- */
#define CIA_NOT_READY           0x0000
#define CIA_SW_ON_DISABLED      0x0040
#define CIA_READY_TO_SWITCH_ON  0x0021
#define CIA_SWITCHED_ON         0x0023
#define CIA_OP_ENABLED          0x0027
#define CIA_QUICK_STOP          0x0007
#define CIA_FAULT_REACTION      0x000F
#define CIA_FAULT               0x0008
#define CIA_UNKNOWN             0xFFFF

#define CW_SHUTDOWN             0x0006         /* IgH: READY            */
#define CW_SWITCH_ON            0x0007
#define CW_ENABLE_OPERATION     0x000F
#define CW_FAULT_RESET          0x0080

#define MODE_CSP                0x08           /* Cyclic Synchronous Position */

static unsigned short GetStatusVal(unsigned short statusWord)
{
    if      ((statusWord & 0x4F) == 0x00) return CIA_NOT_READY;
    else if ((statusWord & 0x4F) == 0x08) return CIA_FAULT;
    else if ((statusWord & 0x4F) == 0x40) return CIA_SW_ON_DISABLED;
    else if ((statusWord & 0x6F) == 0x27) return CIA_OP_ENABLED;
    else if ((statusWord & 0x6F) == 0x23) return CIA_SWITCHED_ON;
    else if ((statusWord & 0x6F) == 0x21) return CIA_READY_TO_SWITCH_ON;
    else if ((statusWord & 0x6F) == 0x07) return CIA_QUICK_STOP;
    else if ((statusWord & 0x4F) == 0x0F) return CIA_FAULT_REACTION;
    return CIA_UNKNOWN;
}

static const char *cia_name(unsigned s)
{
    switch (s) {
    case CIA_NOT_READY:          return "NOT_READY_TO_SWITCH_ON";
    case CIA_SW_ON_DISABLED:     return "SWITCH_ON_DISABLED";
    case CIA_READY_TO_SWITCH_ON: return "READY_TO_SWITCH_ON";
    case CIA_SWITCHED_ON:        return "SWITCHED_ON";
    case CIA_OP_ENABLED:         return "OPERATION_ENABLED";
    case CIA_QUICK_STOP:         return "QUICK_STOP_ACTIVE";
    case CIA_FAULT_REACTION:     return "FAULT_REACTION_ACTIVE";
    case CIA_FAULT:              return "FAULT";
    default:                     return "unknown";
    }
}

/* EtherCAT AL status codes (ETG.1000) -- the ones a CiA402 drive usually answers with */
static const char *al_str(uint16 c)
{
    switch (c) {
    case 0x0000: return "no error";
    case 0x0001: return "unspecified error";
    case 0x0011: return "invalid requested state change";
    case 0x0016: return "invalid mailbox configuration";
    case 0x0017: return "invalid sync manager configuration";
    case 0x0018: return "no valid inputs available";
    case 0x0019: return "no valid outputs";
    case 0x001A: return "synchronization error";
    case 0x001B: return "sync manager watchdog";
    case 0x001D: return "invalid output configuration";
    case 0x001E: return "invalid input configuration";
    case 0x001F: return "invalid watchdog configuration";
    case 0x0024: return "invalid input mapping";
    case 0x0025: return "invalid output mapping";
    case 0x0026: return "inconsistent settings";
    case 0x002C: return "fatal sync error";
    case 0x002D: return "no sync error";
    case 0x0030: return "invalid DC SYNC configuration";
    case 0x0032: return "PLL error";
    case 0x0033: return "DC sync IO error";
    case 0x0034: return "DC sync timeout error";
    case 0x0035: return "DC invalid sync cycle time";
    case 0x0036: return "DC invalid sync0 cycle time";
    default:     return "see the ETG.1000 AL status code table";
    }
}

static const char *al_state_str(unsigned s)
{
    switch (s) {
    case EC_STATE_INIT:        return "INIT";
    case EC_STATE_PRE_OP:      return "PRE-OP";
    case EC_STATE_BOOT:        return "BOOT";
    case EC_STATE_SAFE_OP:     return "SAFE-OP";
    case EC_STATE_OPERATIONAL: return "OPERATIONAL";
    default:                   return "?";
    }
}

/* CoE SDO abort codes (ETG.1000.6) that a drive typically gives when it refuses a PDO mapping */
static const char *coe_abort_str(uint32_t c)
{
    switch (c) {
    case 0x05040000u: return "SDO protocol timed out";
    case 0x06010000u: return "unsupported access to an object";
    case 0x06010002u: return "write to a read-only object (fixed PDO mapping?)";
    case 0x06010003u: return "subindex cannot be written, SI0 must be 0";
    case 0x06020000u: return "object does not exist";
    case 0x06040041u: return "object cannot be mapped into a PDO";
    case 0x06040042u: return "mapped objects exceed the PDO length";
    case 0x06070010u: return "data type / length of the parameter does not match";
    case 0x06090011u: return "subindex does not exist";
    case 0x06090030u: return "value range of the parameter exceeded";
    case 0x08000000u: return "general error";
    case 0x08000020u: return "data cannot be transferred or stored";
    case 0x08000022u: return "not possible in the present device state";
    default:          return "";
    }
}

static int StatusWord[SERVO_MOTOR_NUM]     = {0};
static int PrevStatusWord[SERVO_MOTOR_NUM] = {0};
/* (some of these are only used with CIA402_MODE >= 1 / 2: "unused" keeps -Wall quiet in every mode) */
static int nFirstPos[SERVO_MOTOR_NUM]    __attribute__((unused));   /* position the axis had when it was enabled */
static int fault_resets[SERVO_MOTOR_NUM] __attribute__((unused));
static uint32_t enabled_cycles           __attribute__((unused));   /* consecutive cycles with ALL axes enabled */
static uint32_t traj_k                   __attribute__((unused));   /* sine phase, 0 .. TRAJ_PERIOD_CYCLES-1 */
static int traj_started;                                            /* sine began */
static int traj_stopped;                                            /* latched: an axis dropped out afterwards */

/* ----------------------------------------------------------------------- */
/* sin() without libm.  k = 0 .. TRAJ_PERIOD_CYCLES-1 is one full period.   */
/* Octant reduction by symmetry, then a Taylor series on 0..pi/2 (error < 1e-9). */
/* ----------------------------------------------------------------------- */
static double traj_sin(uint32_t k) __attribute__((unused));   /* only called with CIA402_MODE >= 2 */
static double traj_sin(uint32_t k)
{
    const uint32_t half    = TRAJ_PERIOD_CYCLES / 2u;
    const uint32_t quarter = TRAJ_PERIOD_CYCLES / 4u;
    double sign = 1.0, x, x2;

    if (k >= half)    { k -= half; sign = -1.0; }     /* sin(t + pi) = -sin(t) */
    if (k > quarter)  { k = half - k; }               /* sin(pi - t) =  sin(t) */
    x  = 3.14159265358979323846 * (double)k / (double)half;
    x2 = x * x;
    return sign * x * (1.0 + x2 * (-1.0 / 6.0 + x2 * (1.0 / 120.0 + x2 * (-1.0 / 5040.0
                 + x2 * (1.0 / 362880.0 + x2 * (-1.0 / 39916800.0 + x2 * (1.0 / 6227020800.0)))))));
}

/* ----------------------------------------------------------------------- */
/* Log task: the only place that prints after the 1 kHz loop has started    */
/* ----------------------------------------------------------------------- */
static void log_print(const ev_t *e)
{
    int i;

    switch (e->id) {
    case EV_AXIS:
        xil_printf("[ecat] axis %d: statusword 0x%04x = %s\r\n", (int)e->v[0], (unsigned)e->v[1], cia_name((unsigned)e->v[2]));
        break;
    case EV_WKC_LOST:
        xil_printf("[ecat] WARN working counter %d < %d (frame lost / a drive dropped out): state machine and trajectory paused\r\n",
                   (int)e->v[0], (int)e->v[1]);
        break;
    case EV_WKC_OK:
        xil_printf("[ecat] working counter OK again after %d bad cycle(s)\r\n", (int)e->v[0]);
        break;
    case EV_TRAJ_START:
        xil_printf("[ecat] all %d axes OPERATION_ENABLED for %d ms: sine trajectory starts\r\n", SERVO_MOTOR_NUM, (int)CIA402_TRAJ_DELAY_MS);
        break;
    case EV_TRAJ_STOP:
        xil_printf("[ecat] axis %d left OPERATION_ENABLED (statusword 0x%04x): trajectory STOPPED for good (restart the R5 to run it again)\r\n",
                   (int)e->v[0], (unsigned)e->v[1]);
        break;
    case EV_FAULT_LIMIT:
        xil_printf("[ecat] axis %d: fault reset limit (%d) reached - the axis stays in FAULT\r\n", (int)e->v[0], (int)CIA402_MAX_FAULT_RESETS);
        break;
    case EV_BUSFAULT:
        xil_printf("[ecat] BUS FAULT: working counter incomplete for %d cycles in a row - see the slave AL states above:"
                   " which drive left OPERATIONAL, and why (cable? drive power? drive alarm? SM watchdog?). Restart the R5.\r\n", (int)e->v[0]);
        break;
    case EV_REPORT:
        xil_printf("[ecat] t=%us wkc_bad=%d traj=%s events_lost=%d |", (unsigned)e->v[0], (int)e->v[1],
                   e->v[2] == 2 ? "STOPPED" : (e->v[2] == 1 ? "running" : "-"), (int)e->v[9]);
        for (i = 0; i < SERVO_MOTOR_NUM; i++) {
            xil_printf(" ax%d sw=%04x pos=%d", i, (unsigned)e->v[10 + i], (int)e->v[10 + SERVO_MOTOR_NUM + i]);
        }
        xil_printf("\r\n");
#if CIA402_TIMING
        xil_printf("[ecat]   loop period min/avg/max = %d/%d/%d us   exec max = %d us (run max %d us)   cycles over 1 ms: %d\r\n",
                   (int)e->v[3], (int)e->v[4], (int)e->v[5], (int)e->v[6], (int)e->v[7], (int)e->v[8]);
#endif
        break;
    case EV_SLAVE_AL: {
        unsigned st = (unsigned)e->v[1];
        if ((st & 0x0FU) == 0U) {      /* state 0 = the AL status read got no answer */
            xil_printf("[ecat]   slave %d did not answer the AL status read (cable / power?)\r\n", (int)e->v[0]);
        } else {
            xil_printf("[ecat]   slave %d AL state %s%s, AL status code 0x%04x (%s) | statusword 0x%04x = %s, pos=%d\r\n",
                       (int)e->v[0], al_state_str(st & 0x0FU), (st & (unsigned)EC_STATE_ERROR) ? " + ERROR flag" : "",
                       (unsigned)e->v[2], al_str((uint16)e->v[2]),
                       (unsigned)e->v[3], cia_name(GetStatusVal((unsigned short)e->v[3])), (int)e->v[4]);
        }
        break;
    }
    default:
        break;
    }
}

static void log_drain(void)
{
    while (evq_r != evq_w) {
        ev_t e = evq[evq_r % EVQ_N];       /* copy out, then give the slot back */
        CIA402_BARRIER();
        evq_r = evq_r + 1U;
        log_print(&e);
    }
}

static void log_task(void *arg)
{
    (void)arg;
    for (;;) {
        log_drain();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* Pop SOEM's internal error list (SDO aborts, mailbox errors, emergencies ...) and print the OLDEST `max`
 * entries -- the first error of a burst is the informative one (the list is circular and drops the oldest). */
static void print_ec_errors(int max)
{
    ec_errort e;
    int cnt = 0, more = 0;

    while (ec_poperror(&e)) {
        if (cnt >= max) { more++; continue; }
        cnt++;
        switch (e.Etype) {
        case EC_ERR_TYPE_SDO_ERROR:
            xil_printf("[ecat]   SOEM: slave %u SDO %04x:%02x aborted, code 0x%08x %s\r\n", (unsigned)e.Slave,
                       (unsigned)e.Index, (unsigned)e.SubIdx, (unsigned)e.AbortCode, coe_abort_str((uint32_t)e.AbortCode));
            break;
        case EC_ERR_TYPE_EMERGENCY:
            xil_printf("[ecat]   SOEM: slave %u EMERGENCY code 0x%04x register 0x%02x\r\n", (unsigned)e.Slave,
                       (unsigned)e.ErrorCode, (unsigned)e.ErrorReg);
            break;
        default:
            xil_printf("[ecat]   SOEM: slave %u error type %d, %04x:%02x, code 0x%08x\r\n", (unsigned)e.Slave,
                       (int)e.Etype, (unsigned)e.Index, (unsigned)e.SubIdx, (unsigned)e.AbortCode);
            break;
        }
    }
    if (more > 0) xil_printf("[ecat]   SOEM: (+%d more error(s))\r\n", more);
}

/* ----------------------------------------------------------------------- */
/* PDO mapping via CoE SDO  (translation of ecrt_slave_config_pdos).       */
/* SOEM calls this per slave during ec_config_map (PRE-OP).               */
/* VERIFY against the drive's ESI/manual: some drives use FIXED mapping    */
/* and reject these writes -- then skip this and use the default layout.   */
/* ----------------------------------------------------------------------- */
static int servo_setup(uint16 slave)
{
    int wkc = 0;
    uint8  u8;
    uint16 u16;
    uint32 u32;

    /* RxPDO 0x1600: controlword, target_pos, target_vel, mode */
    u8 = 0; wkc += ec_SDOwrite(slave, 0x1C12, 0x00, FALSE, sizeof(u8),  &u8,  EC_TIMEOUTRXM);
    u8 = 0; wkc += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u8),  &u8,  EC_TIMEOUTRXM);
    u32 = 0x60400010; wkc += ec_SDOwrite(slave, 0x1600, 0x01, FALSE, sizeof(u32), &u32, EC_TIMEOUTRXM);
    u32 = 0x607A0020; wkc += ec_SDOwrite(slave, 0x1600, 0x02, FALSE, sizeof(u32), &u32, EC_TIMEOUTRXM);
    u32 = 0x60FF0020; wkc += ec_SDOwrite(slave, 0x1600, 0x03, FALSE, sizeof(u32), &u32, EC_TIMEOUTRXM);
    u32 = 0x60600008; wkc += ec_SDOwrite(slave, 0x1600, 0x04, FALSE, sizeof(u32), &u32, EC_TIMEOUTRXM);
    u8 = 4;  wkc += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u8),  &u8,  EC_TIMEOUTRXM);
    u16 = 0x1600; wkc += ec_SDOwrite(slave, 0x1C12, 0x01, FALSE, sizeof(u16), &u16, EC_TIMEOUTRXM);
    u8 = 1;  wkc += ec_SDOwrite(slave, 0x1C12, 0x00, FALSE, sizeof(u8),  &u8,  EC_TIMEOUTRXM);

    /* TxPDO 0x1A00: statusword, pos_actual, vel_actual, mode_display */
    u8 = 0; wkc += ec_SDOwrite(slave, 0x1C13, 0x00, FALSE, sizeof(u8),  &u8,  EC_TIMEOUTRXM);
    u8 = 0; wkc += ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(u8),  &u8,  EC_TIMEOUTRXM);
    u32 = 0x60410010; wkc += ec_SDOwrite(slave, 0x1A00, 0x01, FALSE, sizeof(u32), &u32, EC_TIMEOUTRXM);
    u32 = 0x60640020; wkc += ec_SDOwrite(slave, 0x1A00, 0x02, FALSE, sizeof(u32), &u32, EC_TIMEOUTRXM);
    u32 = 0x606C0020; wkc += ec_SDOwrite(slave, 0x1A00, 0x03, FALSE, sizeof(u32), &u32, EC_TIMEOUTRXM);
    u32 = 0x60610008; wkc += ec_SDOwrite(slave, 0x1A00, 0x04, FALSE, sizeof(u32), &u32, EC_TIMEOUTRXM);
    u8 = 4;  wkc += ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(u8),  &u8,  EC_TIMEOUTRXM);
    u16 = 0x1A00; wkc += ec_SDOwrite(slave, 0x1C13, 0x01, FALSE, sizeof(u16), &u16, EC_TIMEOUTRXM);
    u8 = 1;  wkc += ec_SDOwrite(slave, 0x1C13, 0x00, FALSE, sizeof(u8),  &u8,  EC_TIMEOUTRXM);

    /* default mode = CSP so the drive is in position mode from the start */
    u8 = MODE_CSP; wkc += ec_SDOwrite(slave, 0x6060, 0x00, FALSE, sizeof(u8), &u8, EC_TIMEOUTRXM);

    if (wkc != SERVO_SDO_WRITES) {
        xil_printf("[ecat] slave %u: only %d of %d PDO-mapping SDO writes were accepted"
                   " (fixed PDO mapping?)\r\n", (unsigned)slave, wkc, SERVO_SDO_WRITES);
        print_ec_errors(3);                      /* the first aborts of this slave say which object the drive refused */
    }
    return 1;   /* SOEM ignores the value; return 1 by convention */
}

/* Outputs while the drives are brought to OP: servo OFF (controlword 0), CSP selected, target = actual position. */
static void safe_outputs(void)
{
    int i;
    for (i = 0; i < SERVO_MOTOR_NUM; i++) {
        rx[i]->modes_of_operation = MODE_CSP;
        rx[i]->target_position    = tx[i]->position_actual;
    }
}

static void report_states(const char *why)
{
    int i;
    (void)ec_readstate();
    xil_printf("[ecat] %s\r\n", why);
    for (i = 1; i <= ec_slavecount; i++) {
        xil_printf("[ecat]   slave %d \"%s\": state 0x%02x  AL status 0x%04x (%s)\r\n", i, ec_slave[i].name,
                   (unsigned)ec_slave[i].state, (unsigned)ec_slave[i].ALstatuscode,
                   al_str(ec_slave[i].ALstatuscode));
    }
    print_ec_errors(8);
}

/* ----------------------------------------------------------------------- */
/* Bring-up: ec_init -> config -> PDO map -> DC -> SAFE_OP -> OPERATIONAL  */
/* (translation of main()'s ecrt_* setup block)                            */
/* returns 1 = all drives OPERATIONAL, 0 = failed (reason already printed) */
/* ----------------------------------------------------------------------- */
static int ecat_init(void)
{
    int i, n, t, lowest = 0;
    TickType_t next;

    xil_printf("[ecat] ec_init(GEM3) ...\r\n");
    if (ec_init("GEM3") <= 0) {                           /* ifname ignored by our nicdrv */
        xil_printf("[ecat] FAIL: ec_init -- the NIC did not open (see the [gem3] lines above)\r\n");
        return 0;
    }

    n = ec_config_init(FALSE);                            /* bus scan, slaves -> PRE-OP, fills ec_slave[] */
    xil_printf("[ecat] ec_config_init: %d slave(s)\r\n", n);
    if (n <= 0) {
        xil_printf("[ecat] FAIL: nobody answered. Cable on the FIRST drive's IN port? Drive power on?"
                   " ([gem3] link UP?)\r\n");
        gem3_debug_dump();       /* tx>0 rx=0: cable / IN port.  rxFCSerr>0: RGMII delay.  TXSR 0x100 / RXSR 0x8: DMA bus error */
        return 0;
    }
    for (i = 1; i <= ec_slavecount; i++) {
        xil_printf("[ecat]   slave %d: \"%s\"  vendor 0x%08x  product 0x%08x  DC %s\r\n", i, ec_slave[i].name,
                   (unsigned)ec_slave[i].eep_man, (unsigned)ec_slave[i].eep_id, ec_slave[i].hasdc ? "yes" : "NO");
    }
    if (ec_slavecount != SERVO_MOTOR_NUM) {
        xil_printf("[ecat] FAIL: found %d slave(s) but this program drives exactly %d (SERVO_MOTOR_NUM)."
                   " Nothing will be moved.\r\n", ec_slavecount, SERVO_MOTOR_NUM);
        return 0;
    }

    /* register the PDO-mapping hook on each motor (called inside config_map) */
    for (i = 1; i <= SERVO_MOTOR_NUM; i++) {
        ec_slave[i].PO2SOconfig = &servo_setup;
    }

    /* IgH order: PDO mapping + DC/SYNC0 are in place BEFORE the drives are asked for SAFE-OP.
     * SOEM would request SAFE-OP inside ec_config_map(); manualstatechange suppresses exactly that
     * (the slaves are already in PRE-OP, so nothing else changes).                                  */
    ecx_context.manualstatechange = 1;
    n = ec_config_map(&IOmap);                            /* calls servo_setup() per slave */
    ecx_context.manualstatechange = 0;
    xil_printf("[ecat] ec_config_map: IOmap %d bytes (expected %d)\r\n", n,
               SERVO_MOTOR_NUM * (int)(sizeof(rxpdo_t) + sizeof(txpdo_t)));
    print_ec_errors(8);
    if (n <= 0) {
        xil_printf("[ecat] FAIL: ec_config_map produced no process data\r\n");
        return 0;
    }

    /* the packed structs are only valid for exactly this mapping; bind the process-data pointers */
    for (i = 0; i < SERVO_MOTOR_NUM; i++) {
        const ec_slavet *s = &ec_slave[i + 1];
        if (s->outputs == NULL || s->inputs == NULL ||
            s->Obytes != sizeof(rxpdo_t) || s->Ibytes != sizeof(txpdo_t)) {
            xil_printf("[ecat] FAIL: slave %d reports PDO sizes out %u / in %u bytes, this program needs %u / %u"
                       " (PDO-mapping SDOs rejected? fixed mapping?). Nothing will be moved.\r\n",
                       i + 1, (unsigned)s->Obytes, (unsigned)s->Ibytes,
                       (unsigned)sizeof(rxpdo_t), (unsigned)sizeof(txpdo_t));
            return 0;
        }
        rx[i] = (rxpdo_t *)s->outputs;
        tx[i] = (txpdo_t *)s->inputs;
    }

    /* distributed clocks: offsets + propagation delays, then SYNC0 at the cycle time on every DC slave */
    if (!ec_configdc()) {
        xil_printf("[ecat] WARN: no DC-capable slave found\r\n");
    }
    for (i = 1; i <= SERVO_MOTOR_NUM; i++) {
        if (ec_slave[i].hasdc) {
            ec_dcsync0((uint16)i, TRUE, CYCLE_NS, DC_SHIFT_NS);
        } else {
            xil_printf("[ecat] WARN: slave %d has no distributed clock, it runs without SYNC0\r\n", i);
        }
    }

    /* SAFE-OP, now that the drives see their SYNC0 configuration */
    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);
    (void)ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    if (ec_slave[0].state != EC_STATE_SAFE_OP) {
        report_states("FAIL: not all drives reached SAFE-OP");
        return 0;
    }
    xil_printf("[ecat] all drives in SAFE-OP\r\n");

    /* OPERATIONAL: process data must keep flowing at the cycle rate while the drives change state
     * (SM watchdog, DC drift compensation via the FRMW datagram, SYNC0 has to start first).        */
    g_expected_wkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    xil_printf("[ecat] expected working counter %d; requesting OPERATIONAL\r\n", g_expected_wkc);

    next = xTaskGetTickCount();
    for (t = 0; t < CIA402_OP_WAIT_MS; t++) {
        ec_send_processdata();
        (void)ec_receive_processdata(CIA402_RX_TIMEOUT_US);
        safe_outputs();
        if (t == 0) {                                     /* one valid frame first (SOEM simple_test), then ask for OP */
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            ec_writestate(0);
        } else {
            lowest = ec_readstate();
            for (i = 1; i <= ec_slavecount; i++) {        /* a drive that refuses OP raises the AL error flag */
                if (ec_slave[i].state & EC_STATE_ERROR) {
                    report_states("FAIL: a drive refused OPERATIONAL");
                    return 0;
                }
            }
            if (lowest == EC_STATE_OPERATIONAL) break;
        }
        vTaskDelayUntil(&next, pdMS_TO_TICKS(1));
    }
    if (lowest != EC_STATE_OPERATIONAL) {
        report_states("FAIL: not all drives reached OPERATIONAL in time");
        return 0;
    }
    xil_printf("[ecat] all drives OPERATIONAL after %d ms\r\n", t);

    ec_send_processdata();                                /* prime: the first cyclic receive finds a frame */
    return 1;
}

/* ----------------------------------------------------------------------- */
/* One 1 kHz cycle's logic: CiA402 state machine + trajectory              */
/* (translation of cyclic_task() body; CiA402 + sine logic REUSED)         */
/* Called only on a cycle with a complete working counter.                 */
/* ----------------------------------------------------------------------- */
static void cyclic_logic(void)
{
    int i, nEnabled = 0;

    for (i = 0; i < SERVO_MOTOR_NUM; i++) {
        StatusWord[i] = GetStatusVal(tx[i]->statusword);
        if (StatusWord[i] == CIA_OP_ENABLED) nEnabled++;

        /* until the axis is enabled its target follows its actual position: no step when it enables */
        if (StatusWord[i] != CIA_OP_ENABLED) {
            rx[i]->target_position = tx[i]->position_actual;
        }
        if (PrevStatusWord[i] == StatusWord[i]) continue;

        ev_post(EV_AXIS, 1, i, tx[i]->statusword, StatusWord[i]);
#if CIA402_MODE >= 1
        switch (StatusWord[i]) {
        case CIA_SW_ON_DISABLED:
            rx[i]->controlword = CW_SHUTDOWN;           break;
        case CIA_READY_TO_SWITCH_ON:
            rx[i]->controlword = CW_SWITCH_ON;          break;
        case CIA_SWITCHED_ON:
            rx[i]->controlword = CW_ENABLE_OPERATION;   break;
        case CIA_OP_ENABLED:
            nFirstPos[i] = tx[i]->position_actual;      /* hold the position the axis has right now */
            rx[i]->target_position = nFirstPos[i];
            break;
        case CIA_FAULT:
            if (fault_resets[i] < CIA402_MAX_FAULT_RESETS) {
                fault_resets[i]++;
                rx[i]->controlword = CW_FAULT_RESET;
            } else {
                ev_post(EV_FAULT_LIMIT, 0, i, 0, 0);
            }
            break;
        default:
            break;
        }
#endif
        PrevStatusWord[i] = StatusWord[i];
    }

#if CIA402_MODE >= 2
    if (nEnabled == SERVO_MOTOR_NUM) {
        if (enabled_cycles < 0xFFFFFFFFu) enabled_cycles++;
    } else {
        enabled_cycles = 0U;
    }

    /* an axis dropped out after the sine began (fault, quick stop, ...): stop for good */
    if (traj_started && !traj_stopped && nEnabled < SERVO_MOTOR_NUM) {
        int a = 0;
        traj_stopped = 1;
        while (a < SERVO_MOTOR_NUM - 1 && StatusWord[a] == CIA_OP_ENABLED) a++;
        ev_post(EV_TRAJ_STOP, 0, a, tx[a]->statusword, 0);
    }
    /* all axes enabled for a while (brakes released): cyclic synchronous position = sine trajectory */
    if (!traj_started && !traj_stopped && enabled_cycles >= CIA402_TRAJ_DELAY_MS) {
        traj_started = 1;
        traj_k = 0U;
        ev_post(EV_TRAJ_START, 0, 0, 0, 0);
    }
    if (traj_started && !traj_stopped) {
        int j;
        double s = traj_sin(traj_k);
        for (j = 0; j < SERVO_MOTOR_NUM; j++) {
            rx[j]->modes_of_operation = MODE_CSP;
            rx[j]->target_position    = nFirstPos[j] + (int)(TRAJ_AMPLITUDE * s);
        }
        traj_k = (traj_k + 1U) % TRAJ_PERIOD_CYCLES;      /* 1 ms step */
    }
#else
    (void)nEnabled;
#endif
}

/* ----------------------------------------------------------------------- */
/* An incomplete working counter that does not go away: WHICH slave left   */
/* OPERATIONAL and WHY?  One AL-status read of every slave, in control_task*/
/* (the only task that may use the NIC / SOEM); the log task prints it.    */
/* wkc 12 -> 10 = one slave stopped taking outputs (it is in SAFE-OP): its */
/* AL status code says why (0x001B SM watchdog, 0x001A sync error, ...) and*/
/* its CiA402 statusword shows whether the drive itself is in FAULT.       */
/* ----------------------------------------------------------------------- */
static void diag_al_states(void)
{
    int i;

    (void)ec_readstate();                  /* BRD of the AL status; one more datagram for all slaves if they differ or one reports an error */
    for (i = 1; i <= ec_slavecount && i <= SERVO_MOTOR_NUM; i++) {
        ev_t *e = ev_begin(EV_SLAVE_AL, 1);
        if (e == NULL) continue;
        e->v[0] = i;
        e->v[1] = (int32_t)ec_slave[i].state;
        e->v[2] = (int32_t)ec_slave[i].ALstatuscode;
        e->v[3] = (int32_t)tx[i - 1]->statusword;          /* the input image is still updated for a slave in SAFE-OP */
        e->v[4] = (int32_t)tx[i - 1]->position_actual;
        ev_commit();
    }
}

/* ----------------------------------------------------------------------- */
/* One 1 kHz cycle: receive -> logic -> send                               */
/* ----------------------------------------------------------------------- */
static void ecat_cyclic(void)
{
    uint32_t t0 = cyc_now(), t1;
    int wkc = ec_receive_processdata(CIA402_RX_TIMEOUT_US);

    if (wkc >= g_expected_wkc) {
        if (g_wkc_bad > 0) {
            ev_post(EV_WKC_OK, 1, g_wkc_bad, 0, 0);
            g_wkc_bad = 0;
            if (cia402_state == -2) cia402_state = 1;
        }
        cyclic_logic();
    } else {
        g_wkc_bad++;
        g_wkc_bad_total++;
        if (g_wkc_bad == 1) {
            ev_post(EV_WKC_LOST, 1, wkc, g_expected_wkc, 0);
        }
        if (g_wkc_bad == CIA402_DIAG_CYCLES) {
            diag_al_states();               /* once per episode; no process-data frame is in flight at this point */
        }
        if (g_wkc_bad == CIA402_BUSFAULT_CYCLES) {
            cia402_state = -2;
            ev_post(EV_BUSFAULT, 0, g_wkc_bad, 0, 0);
        }
    }

    ec_send_processdata();              /* outputs are re-sent unchanged on a bad cycle */
    t1 = cyc_now();
    tm_sample(t0, t1);
}

/* once per CIA402_REPORT_S: status + timing statistics of the window just ended */
static void ecat_report(uint32_t cycles)
{
    ev_t *e = ev_begin(EV_REPORT, 0);
    int i;

    if (e != NULL) {
        e->v[0] = (int32_t)(cycles / 1000u);
        e->v[1] = (int32_t)g_wkc_bad_total;
        e->v[2] = traj_stopped ? 2 : (traj_started ? 1 : 0);
#if CIA402_TIMING
        e->v[3] = (int32_t)(tm.n ? tm.per_min / CYC_PER_US : 0U);
        e->v[4] = (int32_t)(tm.n ? (uint32_t)(tm.per_sum / ((uint64_t)tm.n * CYC_PER_US)) : 0U);
        e->v[5] = (int32_t)(tm.per_max / CYC_PER_US);
        e->v[6] = (int32_t)(tm.ex_max / CYC_PER_US);
        e->v[7] = (int32_t)(tm.ex_max_all / CYC_PER_US);
        e->v[8] = (int32_t)tm.late;
#endif
        e->v[9] = (int32_t)(ev_dropped + ev_suppressed);
        for (i = 0; i < SERVO_MOTOR_NUM; i++) {
            e->v[10 + i]                    = (int32_t)tx[i]->statusword;
            e->v[10 + SERVO_MOTOR_NUM + i]  = (int32_t)tx[i]->position_actual;
        }
        ev_commit();
    }
    tm_reset_window();
    ev_budget = CIA402_EV_PER_WINDOW;
}

/* ----------------------------------------------------------------------- */
/* FreeRTOS control task.                                                   */
/*                                                                          */
/* TODO (hard real-time): vTaskDelayUntil is quantized to the FreeRTOS tick */
/* (1 ms here) and is NOT phase-locked to EtherCAT DC. For true 1 kHz jitter*/
/* control, drive ecat_cyclic() from a TTC hardware-timer ISR (or a task    */
/* unblocked by it), aligned to the DC SYNC0. This placeholder gets the     */
/* loop running; the "loop period" line of the report tells how much that   */
/* matters on your hardware.                                                */
/* ----------------------------------------------------------------------- */
void control_task(void *arg)
{
    TickType_t next;
    uint32_t cycles = 0;
    (void)arg;

    portTASK_USES_FLOATING_POINT();     /* traj_sin() uses the VFP registers: this task needs an FPU context */
    cyc_init();
    tm_reset_window();

    xil_printf("[ecat] control_task: %d axes, CIA402_MODE %d (0 bus only / 1 servo on + hold / 2 + sine),"
               " cycle %u us, tick %u Hz\r\n", SERVO_MOTOR_NUM, CIA402_MODE,
               (unsigned)(CYCLE_NS / 1000u), (unsigned)configTICK_RATE_HZ);
#if CIA402_MODE >= 2
    xil_printf("[ecat] the motors WILL move: sine +-%d counts, %u s period, after all %d axes are enabled\r\n",
               TRAJ_AMPLITUDE, (unsigned)(TRAJ_PERIOD_CYCLES / 1000u), SERVO_MOTOR_NUM);
#endif

    if (!ecat_init()) {
        cia402_state = -1;
        xil_printf("[ecat] init failed -> task parked, nothing is moving (the reason is in the lines above)\r\n");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (xTaskCreate(log_task, "ecatlog", 1024, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        xil_printf("[ecat] WARN: could not start the log task (FreeRTOS heap full?) - event lines will not be printed\r\n");
    }
    cia402_state = 1;
    xil_printf("[ecat] running: %u us loop started\r\n", (unsigned)(CYCLE_NS / 1000u));

    next = xTaskGetTickCount();
    for (;;) {
        ecat_cyclic();
        cycles++;
        if ((cycles % (1000u * CIA402_REPORT_S)) == 0u) {
            ecat_report(cycles);
        }
        vTaskDelayUntil(&next, pdMS_TO_TICKS(1));    /* ~1 kHz placeholder */
    }
}
