/*
 * ecat_task.c  --  CiA402 torque (CST) servo control, 1 kHz: the EtherCAT master task.
 *
 * Translation of the IgH/EtherLab "servo_drive_dc/cia402_basic.c" to SOEM + FreeRTOS, for the KV260 RPU
 * (Cortex-R5F).  Entry point: ecat_task().
 *   REUSED (application logic, EtherCAT-master-agnostic): the CiA402 drive state machine
 *     (axis_control.c, cia402_drive.c)
 *   TRANSLATED (IgH ecrt_* -> SOEM ec_*): master / domain / PDO setup, process-data access through packed
 *     structs over the IOmap (ecat_bringup.c, cia402_drive.c).  DC SYNC0 on every drive for the L7N bench, none for
 *     the Indy7 (ECAT_USE_DC in app_config.h, ecat_bringup.c).
 *   NEW for the embedded target: PDO mapping by CoE SDO writes, a FreeRTOS task instead of a Linux RT thread,
 *     a 1 kHz loop that never prints (event_log.c), loop period / execution time statistics (loop_timing.c)
 *
 * - WKC gate (IgH: domain wc_state == complete): the CiA402 state machine and the torque only run on a
 *   cycle whose working counter is complete.  On a lost frame the torque is set to 0.
 *   WKC_BUS_FAULT_CYCLES incomplete cycles in a row = "BUS FAULT" (ecat_status = ECAT_STATUS_BUS_FAULT).
 * - Diagnostics: an incomplete working counter that lasts WKC_DIAG_CYCLES cycles makes the task read every
 *   slave's AL state + AL status code ONCE (ec_readstate) and print it next to the CiA402 statusword.
 *   wkc = 12 - 2 means ONE drive stopped taking outputs (it fell out of OPERATIONAL); that line says which
 *   drive and why (0x001B SM watchdog, 0x001A sync error, ...).  Nothing else changes.
 * - Stop (2026-11-17): Linux (r5log.py -f, Ctrl-C) writes a stop command into a page next to the log ring.  The
 *   loop then sets the torque to 0, waits STOP_HOLD_MS, switches every drive off, asks the drives for PRE-OP
 *   while the frames still flow, and stops sending once they are there.  Stopping the R5 in OP instead cuts the
 *   frames there: every drive's SM watchdog expires 100 ms later (AL-67 on the L7N).
 * - Nothing moves by itself.  After the bring-up the drives stay off; the APU asks for servo on / off with the Command
 *   slot (ocm_link.c), and the State slot (drive values) is written every cycle.  ROBOT_TARGET (app_config.h) selects
 *   the L7N bench or the Indy7.
 * - The FPU is declared (portTASK_USES_FLOATING_POINT), sin() is computed locally (no -lm needed), the build
 *   fails with a clear message if the FreeRTOS tick is not 1000 Hz (app_config.h).
 *
 * Timing: the loop is paced with vTaskDelayUntil(1 tick).  The drives take the outputs at their SYNC0; the loop
 * is not phase-locked to SYNC0 (no master clock alignment), as in the colleague's DC build.  The printed
 * "period"/"exec" statistics show how steady the loop is; a hardware-timer ISR is the next step (see ecat_task()).
 */
#include "FreeRTOS.h"
#include "task.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "ethercat.h"
#include "app_config.h"
#include "ecat_task.h"
#include "ecat_bringup.h"
#include "cia402_drive.h"
#include "axis_control.h"
#include "event_log.h"
#include "loop_timing.h"
#include "r5log.h"
#include "ocm_link.h"
#include "obs_link.h"

volatile int ecat_status = ECAT_STATUS_STARTING;

static int      wkc_bad_cycles;        /* consecutive cycles with an incomplete working counter */
static unsigned wkc_bad_total;
static uint32_t cycle_index;           /* release index of the cycle, 1, 2, ... (State slot: period) */

/* ----------------------------------------------------------------------- */
/* The stop command from Linux: the page below the r5log ring, inside the  */
/* top 64 KB that lscript.ld leaves out of the link.  Linux writes only    */
/* the first 32-byte line, this task only the second (after ctl_init):     */
/* the R5 D-cache line is 32 bytes, so a write-back of one side's line can */
/* never undo the other side's byte.  The flush / invalidate are for the   */
/* time before gem3_nic.c maps the window strongly-ordered (no-ops after). */
/* r5log.py uses the same addresses.                                       */
/* ----------------------------------------------------------------------- */
#define CTL_BASE        0x363FB000UL
#define CTL_CMD         (*(volatile uint8_t  *)(CTL_BASE + 0x00U))   /* Linux -> R5 */
#define CTL_MAGIC       (*(volatile uint32_t *)(CTL_BASE + 0x20U))   /* R5 -> Linux: this firmware takes the command */
#define CTL_STATE       (*(volatile int8_t   *)(CTL_BASE + 0x24U))   /* R5 -> Linux: ecat_status */
#define CTL_BOOT        (*(volatile uint32_t *)(CTL_BASE + 0x28U))   /* R5 -> Linux: this run's r5log boot_id */
#define CTL_CMD_STOP    0x53U                                         /* 'S' */
#define CTL_MAGIC_VALUE 0x54533552UL                                  /* bytes 'R','5','S','T' */

static void ctl_publish(int status)
{
    CTL_STATE = (int8_t)status;
    Xil_DCacheFlushRange((INTPTR)(CTL_BASE + 0x20U), 32U);
}

static void ctl_init(void)                 /* a stop command left from an earlier run must not stop this one */
{
    CTL_CMD = 0U;
    Xil_DCacheFlushRange((INTPTR)CTL_BASE, 32U);
    CTL_MAGIC = CTL_MAGIC_VALUE;
    CTL_BOOT  = r5log_boot_id();       /* Linux takes the page as this run's only when it matches the ring's boot_id */
    ctl_publish(ecat_status);
}

static int ctl_stop_requested(void)
{
    Xil_DCacheInvalidateRange((INTPTR)CTL_BASE, 32U);
    return CTL_CMD == CTL_CMD_STOP;
}

static void set_status(int status)
{
    ecat_status = status;
    ctl_publish(status);
}

/* ----------------------------------------------------------------------- */
/* The stop, one step per cycle (stop_cycle).                              */
/* ----------------------------------------------------------------------- */
#define STOP_HOLD_MS        100U   /* torque 0 for this long before the servo-off (the axes coast: CST has no position hold) */
#define STOP_SERVO_OFF_MS   500U   /* the drives' time to leave OPERATION_ENABLED after CW_DISABLE_VOLTAGE */
#define STOP_PREOP_MS       2000U  /* the drives' time to reach PRE-OP (SOEM's EC_TIMEOUTSTATE is 2 s) */

static int      stop_step = STOP_NONE;
static uint32_t stop_ms;                 /* ms in the current step */
static int      stop_axes_on, stop_drives_up;

/* axes whose power stage may still drive the motor (an unknown statusword counts as on) */
static int axes_still_on(void)
{
    int i, n = 0;

    for (i = 0; i < AXIS_COUNT; i++) {
        unsigned short s = cia402_state_from_statusword(drive_in[i]->statusword);
        if (s != CIA402_SWITCH_ON_DISABLED && s != CIA402_READY_TO_SWITCH_ON && s != CIA402_SWITCHED_ON &&
            s != CIA402_NOT_READY_TO_SWITCH_ON && s != CIA402_FAULT) {
            n++;
        }
    }
    return n;
}

/* drives not yet in PRE-OP or INIT (one that does not answer counts as not there) */
static int drives_above_preop(void)
{
    int i, n = 0;

    (void)ec_readstate();
    for (i = 1; i <= ec_slavecount; i++) {
        uint16 s = ec_slave[i].state & 0x0FU;
        if (s != EC_STATE_PRE_OP && s != EC_STATE_INIT) n++;
    }
    return n;
}

/* after the receive and before the send: no process-data frame is in flight */
static void stop_cycle(void)
{
    int i;

    stop_ms++;
    switch (stop_step) {
    case STOP_HOLD:                     /* axis_control_update() no longer runs: torque 0 until the servo-off below */
        axis_control_torque_zero();
        if (stop_ms >= STOP_HOLD_MS) {
            for (i = 0; i < AXIS_COUNT; i++) drive_out[i]->controlword = CW_DISABLE_VOLTAGE;
            stop_step = STOP_SERVO_OFF;
            stop_ms = 0U;
        }
        break;
    case STOP_SERVO_OFF:
        stop_axes_on = axes_still_on();
        if (stop_axes_on == 0 || stop_ms >= STOP_SERVO_OFF_MS) {
            /* with the ack: a drive that dropped out (AL error flag set) may ignore a request without it */
            ec_slave[0].state = EC_STATE_PRE_OP | EC_STATE_ACK;
            ec_writestate(0);
            event_post(EVENT_STOP, 0, STOP_PREOP, stop_axes_on, (int)stop_ms);
            stop_step = STOP_PREOP;
            stop_ms = 0U;
        }
        break;
    case STOP_PREOP:                    /* frames go on until the drives are there: no SM watchdog expires on the way */
        if ((stop_ms % 10U) == 0U) {
            stop_drives_up = drives_above_preop();
            if (stop_drives_up == 0 || stop_ms >= STOP_PREOP_MS) {
                event_post(EVENT_STOP, 0, STOP_DONE, stop_drives_up, (int)stop_ms);
                stop_step = STOP_DONE;
            }
        }
        break;
    default:
        break;
    }
}

/* ----------------------------------------------------------------------- */
/* An incomplete working counter that does not go away: WHICH slave left   */
/* OPERATIONAL and WHY?  One AL-status read of every slave, in this task   */
/* (the only task that may use the NIC / SOEM); the log task prints it.    */
/* wkc 12 -> 10 = one slave stopped taking outputs (it is in SAFE-OP): its */
/* AL status code says why (0x001B SM watchdog, 0x001A sync error, ...) and*/
/* its CiA402 statusword shows whether the drive itself is in FAULT.       */
/* ----------------------------------------------------------------------- */
static void post_slave_al_states(void)
{
    int i;

    (void)ec_readstate();                  /* BRD of the AL status; one more datagram for all slaves if they differ or one reports an error */
    for (i = 1; i <= ec_slavecount && i <= AXIS_COUNT; i++) {
        event_t *e = event_reserve(EVENT_SLAVE_AL_STATUS, 1);
        if (e == NULL) continue;
        e->value[0] = i;
        e->value[1] = (int32_t)ec_slave[i].state;
        e->value[2] = (int32_t)ec_slave[i].ALstatuscode;
        e->value[3] = (int32_t)drive_in[i - 1]->statusword;          /* the input image is still updated for a slave in SAFE-OP */
        e->value[4] = (int32_t)drive_in[i - 1]->position_actual;
        event_publish();
    }
}

/* ----------------------------------------------------------------------- */
/* The APU's slots: Command (servo on / off) and Torque; the camera results. */
/* ----------------------------------------------------------------------- */
static void take_apu_slots(uint64_t now)
{
    uint32_t command = ocm_link_take_command();

    if (command != IPC_CMD_NONE) {
        event_post(EVENT_COMMAND, 1, (int)command, 0, 0);
        /* shortcut: IPC_CMD_STOP is a servo off here; the clean stop to PRE-OP is still r5log.py -f */
        if (command == IPC_CMD_SERVO_ON) axis_control_servo(1);
        else                             axis_control_servo(0);
    }
    link_torque_t torque;
    int fresh = ocm_link_take_torque(&torque);

    axis_control_torque_input(fresh, torque.usable, torque.tau);
    obs_link_poll(now);                 /* the camera results: taken and counted; the control task will use them (plan 4-3) */
}

/* ----------------------------------------------------------------------- */
/* One 1 kHz cycle: receive -> logic -> send                               */
/* ----------------------------------------------------------------------- */
static void ecat_cycle(void)
{
    uint32_t start = cycle_counter_read(), end;
    uint64_t t_release = ocm_link_counter();
    uint32_t link_flags = IPC_STATE_OP;
    int wkc = ec_receive_processdata(RX_TIMEOUT_US);

    cycle_index++;
    if (stop_step == STOP_NONE) take_apu_slots(t_release);     /* a command during a stop would only fight the stop */

    if (stop_step != STOP_NONE) {
        stop_cycle();                   /* the working counter falls as the drives leave OP: no bus fault then */
    } else if (wkc >= ecat_expected_wkc) {
        if (wkc_bad_cycles > 0) {
            event_post(EVENT_WKC_RECOVERED, 1, wkc_bad_cycles, 0, 0);
            wkc_bad_cycles = 0;
            if (ecat_status == ECAT_STATUS_BUS_FAULT) set_status(ECAT_STATUS_RUNNING);
        }
        axis_control_update();
    } else {
        axis_control_torque_zero();         /* update() does not run: a torque from before the loss must not stay */
        wkc_bad_cycles++;
        wkc_bad_total++;
        if (wkc_bad_cycles == 1) {
            event_post(EVENT_WKC_INCOMPLETE, 1, wkc, ecat_expected_wkc, 0);
        }
        if (wkc_bad_cycles == WKC_DIAG_CYCLES) {
            post_slave_al_states();         /* once per episode; no process-data frame is in flight at this point */
        }
        if (wkc_bad_cycles == WKC_BUS_FAULT_CYCLES) {
            set_status(ECAT_STATUS_BUS_FAULT);
            event_post(EVENT_BUS_FAULT, 0, wkc_bad_cycles, 0, 0);
        }
    }

    if (stop_step == STOP_NONE && ctl_stop_requested()) {
        stop_step = STOP_HOLD;
        stop_ms = 0U;
        set_status(ECAT_STATUS_STOPPING);
        event_post(EVENT_STOP, 0, STOP_HOLD, axes_still_on(), (int)STOP_HOLD_MS);
    }
    if (stop_step != STOP_NONE) link_flags |= IPC_STATE_STOP;
    else if (wkc < ecat_expected_wkc) link_flags |= IPC_STATE_WKC_ERR;
#if LOOP_TIMING
    ocm_link_publish_state(t_release, cycle_index, link_flags, loop_timing.overruns, wkc_bad_total);
#else
    ocm_link_publish_state(t_release, cycle_index, link_flags, 0u, wkc_bad_total);
#endif
    if (stop_step != STOP_DONE) {
        ec_send_processdata();          /* outputs are re-sent unchanged on a bad cycle */
    }
    end = cycle_counter_read();
    loop_timing_record(start, end);
}

/* once per REPORT_PERIOD_S: status + timing statistics of the window just ended */
static void post_status_report(uint32_t cycles)
{
    event_t *e = event_reserve(EVENT_STATUS_REPORT, 0);
    int i;

    if (e != NULL) {
        e->value[0] = (int32_t)(cycles / 1000u);
        e->value[1] = (int32_t)wkc_bad_total;
        e->value[2] = axis_control_servo_requested();
#if LOOP_TIMING
        e->value[3] = (int32_t)(loop_timing.periods ? loop_timing.period_min / CPU_CYCLES_PER_US : 0U);
        e->value[4] = (int32_t)(loop_timing.periods ? (uint32_t)(loop_timing.period_sum / ((uint64_t)loop_timing.periods * CPU_CYCLES_PER_US)) : 0U);
        e->value[5] = (int32_t)(loop_timing.period_max / CPU_CYCLES_PER_US);
        e->value[6] = (int32_t)(loop_timing.exec_max / CPU_CYCLES_PER_US);
        e->value[7] = (int32_t)(loop_timing.exec_max_run / CPU_CYCLES_PER_US);
        e->value[8] = (int32_t)loop_timing.overruns;
#endif
        e->value[9] = (int32_t)(events_dropped + events_suppressed);
        for (i = 0; i < AXIS_COUNT; i++) {
            e->value[10 + i]              = (int32_t)drive_in[i]->statusword;
            e->value[10 + AXIS_COUNT + i] = (int32_t)drive_in[i]->position_actual;
        }
        event_publish();
    }
    obs_link_report(ocm_link_counter());
    loop_timing_reset_window();
    event_budget = EVENTS_PER_REPORT;
}

/* ----------------------------------------------------------------------- */
/* FreeRTOS task "ecat".                                                    */
/*                                                                          */
/* TODO (hard real-time): vTaskDelayUntil is quantized to the FreeRTOS tick */
/* (1 ms here). For true 1 kHz jitter control, drive ecat_cycle() from a    */
/* TTC hardware-timer ISR (or a task unblocked by it). This placeholder     */
/* gets the loop running; the "loop period" line of the report tells how   */
/* much that matters on your hardware.                                      */
/* ----------------------------------------------------------------------- */
void ecat_task(void *arg)
{
    TickType_t next;
    uint32_t cycles = 0;
    (void)arg;

    portTASK_USES_FLOATING_POINT();     /* the unit conversions (robot_target.h) are double: this task needs an FPU context */
    ctl_init();
    if (!ocm_link_start()) {
        set_status(ECAT_STATUS_INIT_FAILED);
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    obs_link_start();
    cycle_counter_start();
    loop_timing_reset_window();

    xil_printf("[ecat] ecat_task: target %s, %d axes, cycle %u us, tick %u Hz; servo on / off come from the APU\r\n",
               ROBOT_TARGET == TARGET_INDY7 ? "Indy7" : "L7N bench", AXIS_COUNT,
               (unsigned)(CYCLE_NS / 1000u), (unsigned)configTICK_RATE_HZ);

    if (!ecat_bring_up()) {
        set_status(ECAT_STATUS_INIT_FAILED);
        xil_printf("[ecat] init failed -> task parked, nothing is moving (the reason is in the lines above)\r\n");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }


    set_status(ECAT_STATUS_RUNNING);
    xil_printf("[ecat] running: %u us loop started, drives off until the APU asks for servo on\r\n", (unsigned)(CYCLE_NS / 1000u));

    next = xTaskGetTickCount();
    while (stop_step != STOP_DONE) {
        ecat_cycle();
        cycles++;
        if ((cycles % (1000u * REPORT_PERIOD_S)) == 0u) {
            post_status_report(cycles);
        }
        vTaskDelayUntil(&next, pdMS_TO_TICKS(1));    /* ~1 kHz placeholder */
    }

    /* stopped on request; the last cycle sent no frame, so none is in flight */
    post_slave_al_states();                          /* every drive's final AL state and last statusword */
    vTaskDelay(pdMS_TO_TICKS(100));                  /* the log task (every 20 ms) prints the queued lines first */
    if (stop_axes_on == 0 && stop_drives_up == 0) {
        xil_printf("[ecat] STOPPED: all axes off, all drives in PRE-OP, no more frames; the R5 can be stopped\r\n");
    } else {
        xil_printf("[ecat] STOPPED with the WARN above: no more frames; a drive still in OP raises its SM watchdog alarm\r\n");
    }
    set_status(ECAT_STATUS_STOPPED);                 /* after the lines: r5log.py reads the ring once more, then stops the R5 */
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}
