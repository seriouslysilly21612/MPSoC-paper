/*
 * event_log.c  --  the event queue of the 1 kHz loop and the low-priority task that prints it.
 * xil_printf goes to the r5log ring (r5log.c MUST be in the project: the BSP's own console blocks forever
 * without a JTAG host).  Read it on Linux with "sudo python3 r5log.py -f".
 */
#include "FreeRTOS.h"
#include "task.h"
#include "xil_printf.h"
#include "ethercat.h"
#include "event_log.h"
#include "cia402_drive.h"
#include "ecat_diag.h"

#if defined(__arm__) && !defined(HOST_TEST)
#define MEMORY_BARRIER()    __asm__ volatile ("dmb" : : : "memory")
#else
#define MEMORY_BARRIER()    __asm__ volatile ("" : : : "memory")
#endif

#define EVENT_QUEUE_LEN 32U                           /* power of two */

static event_t           event_queue[EVENT_QUEUE_LEN];
static volatile uint32_t event_write_index, event_read_index;
volatile uint32_t        events_dropped;
uint32_t                 events_suppressed;
int                      event_budget = EVENTS_PER_REPORT;

event_t *event_reserve(uint32_t id, int budgeted)
{
    event_t *e;
    int k;
    uint32_t w = event_write_index;

    if (budgeted && event_budget <= 0) { events_suppressed++; return NULL; }
    if ((uint32_t)(w - event_read_index) >= EVENT_QUEUE_LEN) { events_dropped++; return NULL; }
    if (budgeted) event_budget--;
    e = &event_queue[w % EVENT_QUEUE_LEN];
    e->id = id;
    for (k = 0; k < EVENT_VALUES; k++) e->value[k] = 0;
    return e;
}

void event_publish(void)
{
    MEMORY_BARRIER();                  /* slot contents first, then the index */
    event_write_index = event_write_index + 1U;
}

void event_post(uint32_t id, int budgeted, int a, int b, int c)
{
    event_t *e = event_reserve(id, budgeted);
    if (e != NULL) {
        e->value[0] = a; e->value[1] = b; e->value[2] = c;
        event_publish();
    }
}

/* ----------------------------------------------------------------------- */
/* The only place that prints after the 1 kHz loop has started             */
/* ----------------------------------------------------------------------- */
static void print_event(const event_t *e)
{
    int i;

    switch (e->id) {
    case EVENT_AXIS_STATE_CHANGED:
        xil_printf("[ecat] axis %d: statusword 0x%04x = %s\r\n", (int)e->value[0], (unsigned)e->value[1], cia402_state_name((unsigned)e->value[2]));
        break;
    case EVENT_WKC_INCOMPLETE:
        xil_printf("[ecat] WARN working counter %d < %d (frame lost / a drive dropped out): state machine and trajectory paused\r\n",
                   (int)e->value[0], (int)e->value[1]);
        break;
    case EVENT_WKC_RECOVERED:
        xil_printf("[ecat] working counter OK again after %d bad cycle(s)\r\n", (int)e->value[0]);
        break;
    case EVENT_SINE_STARTED:
        xil_printf("[ecat] all %d axes OPERATION_ENABLED for %d ms: sine trajectory starts\r\n", AXIS_COUNT, (int)SINE_START_DELAY_MS);
        break;
    case EVENT_SINE_STOPPED:
        xil_printf("[ecat] axis %d left OPERATION_ENABLED (statusword 0x%04x): trajectory STOPPED for good (restart the R5 to run it again)\r\n",
                   (int)e->value[0], (unsigned)e->value[1]);
        break;
    case EVENT_FAULT_RESET_LIMIT:
        xil_printf("[ecat] axis %d: fault reset limit (%d) reached - the axis stays in FAULT\r\n", (int)e->value[0], (int)MAX_FAULT_RESETS);
        break;
    case EVENT_BUS_FAULT:
        xil_printf("[ecat] BUS FAULT: working counter incomplete for %d cycles in a row - see the slave AL states above:"
                   " which drive left OPERATIONAL, and why (cable? drive power? drive alarm? SM watchdog?). Restart the R5.\r\n", (int)e->value[0]);
        break;
    case EVENT_STATUS_REPORT:
        xil_printf("[ecat] t=%us wkc_bad=%d traj=%s events_lost=%d |", (unsigned)e->value[0], (int)e->value[1],
                   e->value[2] == 2 ? "STOPPED" : (e->value[2] == 1 ? "running" : "-"), (int)e->value[9]);
        for (i = 0; i < AXIS_COUNT; i++) {
            xil_printf(" ax%d sw=%04x pos=%d", i, (unsigned)e->value[10 + i], (int)e->value[10 + AXIS_COUNT + i]);
        }
        xil_printf("\r\n");
#if LOOP_TIMING
        xil_printf("[ecat]   loop period min/avg/max = %d/%d/%d us   exec max = %d us (run max %d us)   cycles over 1 ms: %d\r\n",
                   (int)e->value[3], (int)e->value[4], (int)e->value[5], (int)e->value[6], (int)e->value[7], (int)e->value[8]);
#endif
        break;
    case EVENT_SLAVE_AL_STATUS: {
        unsigned st = (unsigned)e->value[1];
        if ((st & 0x0FU) == 0U) {      /* state 0 = the AL status read got no answer */
            xil_printf("[ecat]   slave %d did not answer the AL status read (cable / power?)\r\n", (int)e->value[0]);
        } else {
            xil_printf("[ecat]   slave %d AL state %s%s, AL status code 0x%04x (%s) | statusword 0x%04x = %s, pos=%d\r\n",
                       (int)e->value[0], al_state_name(st & 0x0FU), (st & (unsigned)EC_STATE_ERROR) ? " + ERROR flag" : "",
                       (unsigned)e->value[2], al_status_code_text((uint16)e->value[2]),
                       (unsigned)e->value[3], cia402_state_name(cia402_state_from_statusword((unsigned short)e->value[3])), (int)e->value[4]);
        }
        break;
    }
    default:
        break;
    }
}

static void print_pending_events(void)
{
    while (event_read_index != event_write_index) {
        event_t e = event_queue[event_read_index % EVENT_QUEUE_LEN];   /* copy out, then give the slot back */
        MEMORY_BARRIER();
        event_read_index = event_read_index + 1U;
        print_event(&e);
    }
}

void event_log_task(void *arg)
{
    (void)arg;
    for (;;) {
        print_pending_events();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
