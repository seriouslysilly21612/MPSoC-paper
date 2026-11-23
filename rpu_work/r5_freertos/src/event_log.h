/*
 * event_log.h  --  the 1 kHz loop never prints.  It posts numbers into a small queue; a low-priority task
 * (event_log_task, "ecatlog") prints them, so a slow console never delays a cycle.  Bring-up lines are
 * printed directly.  Single producer (the ecat task) / single consumer (the ecatlog task).
 */
#ifndef EVENT_LOG_H
#define EVENT_LOG_H

#include <stdint.h>
#include "app_config.h"

enum {
    EVENT_AXIS_STATE_CHANGED = 1, /* value[0] axis, [1] statusword, [2] CiA402 state                  */
    EVENT_WKC_INCOMPLETE,         /* [0] wkc, [1] expected wkc                                        */
    EVENT_WKC_RECOVERED,          /* [0] number of bad cycles                                         */
    EVENT_FAULT_RESET_LIMIT,      /* [0] axis                                                         */
    EVENT_BUS_FAULT,              /* [0] number of bad cycles                                         */
    EVENT_STATUS_REPORT,          /* see post_status_report() in ecat_task.c                          */
    EVENT_SLAVE_AL_STATUS,        /* [0] slave (1-based), [1] AL state (0x10 = error flag), [2] AL status code, [3] statusword, [4] position */
    EVENT_STOP,                   /* [0] the step that begins (STOP_*, ecat_task.h), [1] axes on / drives above PRE-OP, [2] ms */
    EVENT_COMMAND,                /* [0] IPC_CMD_* of a new command from the APU                       */
    EVENT_TORQUE_STALE_STOP,      /* [0] stale cycles in a row (TORQUE_STALE_LIMIT): servo switched off */
    EVENT_OBS_REPORT,             /* see obs_link_report() in obs_link.c                              */
    EVENT_TIMING_REPORT,          /* see post_status_report() in ecat_task.c                          */
    EVENT_SHADOW_REPORT,          /* see shadow_link_report() in shadow_link.c                        */
    EVENT_CONTROL_REPORT,         /* see control_link_report() in control_link.c                      */
    EVENT_OBS_COST_REPORT,        /* see obs_cost_report() in control_link.c                          */
    EVENT_CONTROL_LATE            /* [0] period of the step not done by the next release, [1] such cycles so far */
};

#define EVENT_VALUES    (10 + 2 * AXIS_COUNT)
typedef struct { uint32_t id; int32_t value[EVENT_VALUES]; } event_t;

extern volatile uint32_t events_dropped;      /* queue full */
extern uint32_t          events_suppressed;   /* over the per-window budget */
extern int               event_budget;        /* budgeted events left in this report window; post_status_report() refills it */

/* a free slot (fill it, then event_publish()) or NULL; budgeted = counts against the per-window budget */
event_t *event_reserve(uint32_t id, int budgeted);
void     event_publish(void);
void     event_post(uint32_t id, int budgeted, int a, int b, int c);

void     event_log_task(void *arg);           /* FreeRTOS task: prints the queued events */

#endif /* EVENT_LOG_H */
