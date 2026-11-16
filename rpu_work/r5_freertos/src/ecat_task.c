/*
 * ecat_task.c  --  4-axis CiA402 (CSP) servo control, 1 kHz, DC SYNC0: the EtherCAT master task.
 *
 * Translation of the IgH/EtherLab "servo_drive_dc/cia402_basic.c" to SOEM + FreeRTOS, for the KV260 RPU
 * (Cortex-R5F).  Entry point: ecat_task().
 *   REUSED (application logic, EtherCAT-master-agnostic): the CiA402 drive state machine and the sine target
 *     position (axis_control.c, cia402_drive.c)
 *   TRANSLATED (IgH ecrt_* -> SOEM ec_*): master / domain / PDO setup, process-data access through packed
 *     structs over the IOmap (ecat_bringup.c, cia402_drive.c).  DC as IgH's: SYNC0 on every drive (ecat_bringup.c).
 *   NEW for the embedded target: PDO mapping by CoE SDO writes, a FreeRTOS task instead of a Linux RT thread,
 *     a 1 kHz loop that never prints (event_log.c), loop period / execution time statistics (loop_timing.c)
 *
 * - WKC gate (IgH: domain wc_state == complete): the CiA402 state machine and the trajectory only run on a
 *   cycle whose working counter is complete.  A lost frame pauses the trajectory, it never makes it jump.
 *   WKC_BUS_FAULT_CYCLES incomplete cycles in a row = "BUS FAULT" (ecat_status = ECAT_STATUS_BUS_FAULT).
 * - Diagnostics: an incomplete working counter that lasts WKC_DIAG_CYCLES cycles makes the task read every
 *   slave's AL state + AL status code ONCE (ec_readstate) and print it next to the CiA402 statusword.
 *   wkc = 12 - 2 means ONE drive stopped taking outputs (it fell out of OPERATIONAL); that line says which
 *   drive and why (0x001B SM watchdog, 0x001A sync error, ...).  Nothing else changes.
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
#include "ethercat.h"
#include "app_config.h"
#include "ecat_task.h"
#include "ecat_bringup.h"
#include "cia402_drive.h"
#include "axis_control.h"
#include "event_log.h"
#include "loop_timing.h"

volatile int ecat_status = ECAT_STATUS_STARTING;

static int      wkc_bad_cycles;        /* consecutive cycles with an incomplete working counter */
static unsigned wkc_bad_total;

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
/* One 1 kHz cycle: receive -> logic -> send                               */
/* ----------------------------------------------------------------------- */
static void ecat_cycle(void)
{
    uint32_t start = cycle_counter_read(), end;
    int wkc = ec_receive_processdata(RX_TIMEOUT_US);

    if (wkc >= ecat_expected_wkc) {
        if (wkc_bad_cycles > 0) {
            event_post(EVENT_WKC_RECOVERED, 1, wkc_bad_cycles, 0, 0);
            wkc_bad_cycles = 0;
            if (ecat_status == ECAT_STATUS_BUS_FAULT) ecat_status = ECAT_STATUS_RUNNING;
        }
        axis_control_update();
    } else {
        wkc_bad_cycles++;
        wkc_bad_total++;
        if (wkc_bad_cycles == 1) {
            event_post(EVENT_WKC_INCOMPLETE, 1, wkc, ecat_expected_wkc, 0);
        }
        if (wkc_bad_cycles == WKC_DIAG_CYCLES) {
            post_slave_al_states();         /* once per episode; no process-data frame is in flight at this point */
        }
        if (wkc_bad_cycles == WKC_BUS_FAULT_CYCLES) {
            ecat_status = ECAT_STATUS_BUS_FAULT;
            event_post(EVENT_BUS_FAULT, 0, wkc_bad_cycles, 0, 0);
        }
    }

    ec_send_processdata();              /* outputs are re-sent unchanged on a bad cycle */
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
        e->value[2] = sine_stopped ? 2 : (sine_started ? 1 : 0);
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

    portTASK_USES_FLOATING_POINT();     /* the sine in axis_control.c uses the VFP registers: this task needs an FPU context */
    cycle_counter_start();
    loop_timing_reset_window();

    xil_printf("[ecat] ecat_task: %d axes, RUN_LEVEL %d (0 bus only / 1 servo on + hold / 2 + sine),"
               " cycle %u us, tick %u Hz\r\n", AXIS_COUNT, RUN_LEVEL,
               (unsigned)(CYCLE_NS / 1000u), (unsigned)configTICK_RATE_HZ);
#if RUN_LEVEL >= RUN_SINE
    xil_printf("[ecat] the motors WILL move: sine +-%d counts, %u s period, after all %d axes are enabled\r\n",
               SINE_AMPLITUDE, (unsigned)(SINE_PERIOD_CYCLES / 1000u), AXIS_COUNT);
#endif

    if (!ecat_bring_up()) {
        ecat_status = ECAT_STATUS_INIT_FAILED;
        xil_printf("[ecat] init failed -> task parked, nothing is moving (the reason is in the lines above)\r\n");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (xTaskCreate(event_log_task, "ecatlog", 1024, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        xil_printf("[ecat] WARN: could not start the log task (FreeRTOS heap full?) - event lines will not be printed\r\n");
    }
    ecat_status = ECAT_STATUS_RUNNING;
    xil_printf("[ecat] running: %u us loop started\r\n", (unsigned)(CYCLE_NS / 1000u));

    next = xTaskGetTickCount();
    for (;;) {
        ecat_cycle();
        cycles++;
        if ((cycles % (1000u * REPORT_PERIOD_S)) == 0u) {
            post_status_report(cycles);
        }
        vTaskDelayUntil(&next, pdMS_TO_TICKS(1));    /* ~1 kHz placeholder */
    }
}
