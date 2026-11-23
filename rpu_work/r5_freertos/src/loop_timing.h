/*
 * loop_timing.h  --  the times of a 1 kHz cycle and what they are held to (review F5, plan 2-4), on the system
 * counter (sys_counter.h):
 *   release   the tick of the period timer that woke the task: the FreeRTOS tick, a TTC in interval mode
 *   t_w       release -> start of the cycle (how far the timer has counted when the task runs)
 *   t_s       start -> return of the send;  t_w + t_s <= t_b, the send deadline of the cfg (eq. (3) of the paper)
 *   t_e       start -> end of the cycle: the return of the send, or, in a cycle in which the second controller of
 *             --shadow ran after the send (shadow_link.h), its end, or the end of the control step of the cycle
 *             (CONTROL_TASK, control_task.h; the next cycle closes the record).
 *             The record of the times is written after it and is in neither.
 *             t_w + t_e <= T: the cycle ends before the next release
 * The names and the rules of late, missed and overrun are those of the EtherCAT task of the APU controller
 * (TaskSlots.h, TimingLog.h).
 */
#ifndef LOOP_TIMING_H
#define LOOP_TIMING_H

#include <stdint.h>
#include "app_config.h"
#include "sys_counter.h"
#include "cycle_record.h"
#include "robot_target.h"

/* [ns] -> ticks of the system counter, at compile time */
#define TICKS_OF_NS(ns)         ((uint32_t)(((uint64_t)SYSCNT_HZ * (ns) + 500000000u) / 1000000000u))
#define TICKS_PER_CYCLE         TICKS_OF_NS(CYCLE_NS)
#define TICKS_SEND_DEADLINE     TICKS_OF_NS(FW_CFG_SEND_DEADLINE_NS)

/* A lost frame must not cost the send its deadline: the receive gives up after RX_TIMEOUT_US, and the rest of the
 * cycle up to the return of the send took 0.1 ms at most on the board (2026-11-18: the whole cycle 96 us). */
_Static_assert(RX_TIMEOUT_US * 1000 + 100000 <= FW_CFG_SEND_DEADLINE_NS,
               "RX_TIMEOUT_US leaves no room before the send deadline of the cfg");

typedef struct {                        /* one cycle [ticks of the system counter] */
    uint64_t release;                   /* the release of the period */
    uint32_t wake;                      /* t_w */
    /* the low word of the counter at: the start of the cycle, the inputs in, before and after the send, the end */
    uint32_t start, recv, send_begin, send_end, end;
} cycle_times_t;

typedef struct {
    uint32_t late, missed, overruns;    /* cycles so far: ST_FAULTS of the State slot */
    /* the window of one status report [ticks] */
    uint32_t periods;                   /* start-to-start samples */
    uint64_t period_sum;
    uint32_t period_min, period_max;
    uint32_t wake_max, send_max, exec_max;      /* t_w, t_w + t_s, t_e */
    uint32_t exec_max_run;              /* t_e, whole run */
    uint32_t prev_start;
    int      have_prev;
} loop_timing_t;

extern loop_timing_t loop_timing;

/* Before the first cycle: is the period timer what loop_timing_begin() takes it for?  0 = no (reason printed). */
int      loop_timing_start(void);
void     loop_timing_reset_window(void);

/* At the start of a cycle, right after vTaskDelayUntil(): its release and t_w.  planned_tick: the tick the task
 * was to wake at. */
void     loop_timing_begin(uint32_t planned_tick, cycle_times_t *t);

/* t_w [ticks of the system counter] of a cycle that starts periods_late whole periods after its release and
 * timer_count counts of the period timer on top; the timer counts 0..timer_interval in one period, and
 * timer_interval is not 0 (loop_timing_start() refuses such a timer). */
uint32_t cycle_wake_ticks(uint32_t periods_late, uint32_t timer_count, uint32_t timer_interval);

/* At the end of a cycle: its times into the record, RECORD_LATE / _MISSED / _OVERRUN into the record's flags,
 * the counts and the window. */
void     loop_timing_end(const cycle_times_t *t, cycle_record_t *record);

#endif /* LOOP_TIMING_H */
