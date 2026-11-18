/*
 * loop_timing.c  --  the release of a 1 kHz cycle, its times and the rules they are held to (see loop_timing.h).
 */
#include <string.h>
#include "loop_timing.h"

loop_timing_t loop_timing;

void loop_timing_reset_window(void)
{
    loop_timing.periods = 0u; loop_timing.period_sum = 0u; loop_timing.period_min = 0xFFFFFFFFu;
    loop_timing.period_max = 0u; loop_timing.wake_max = 0u; loop_timing.send_max = 0u; loop_timing.exec_max = 0u;
}

uint32_t cycle_wake_ticks(uint32_t periods_late, uint32_t timer_count, uint32_t timer_interval)
{
    uint64_t wake;

    wake = (uint64_t)periods_late * TICKS_PER_CYCLE + (uint64_t)timer_count * TICKS_PER_CYCLE / timer_interval;
    return wake > 0xFFFFFFFFu ? 0xFFFFFFFFu : (uint32_t)wake;
}

void loop_timing_end(const cycle_times_t *t, cycle_record_t *record)
{
    /* low words of the counter: a wrap between two of them cancels in the difference */
    const uint32_t exec = t->send_end - t->start;                /* t_s, and t_e: the cycle ends with its send */
    const uint64_t release_to_send = (uint64_t)t->wake + exec;

    record->w_ns    = (int32_t)sys_counter_ns(t->wake);
    record->s_ns    = (int32_t)sys_counter_ns(exec);
    record->e_ns    = record->s_ns;
    record->recv_ns = (int32_t)sys_counter_ns(t->recv - t->start);
    record->send_ns = (int32_t)sys_counter_ns(t->send_end - t->send_begin);
    if (t->wake > TICKS_PER_CYCLE)             { record->flags |= RECORD_LATE;    loop_timing.late++; }
    if (release_to_send > TICKS_SEND_DEADLINE) { record->flags |= RECORD_MISSED;  loop_timing.missed++; }
    if (release_to_send > TICKS_PER_CYCLE)     { record->flags |= RECORD_OVERRUN; loop_timing.overruns++; }

    if (loop_timing.have_prev) {
        const uint32_t period = t->start - loop_timing.prev_start;

        loop_timing.periods++;
        loop_timing.period_sum += period;
        if (period < loop_timing.period_min) loop_timing.period_min = period;
        if (period > loop_timing.period_max) loop_timing.period_max = period;
    }
    loop_timing.prev_start = t->start;
    loop_timing.have_prev  = 1;
    if (t->wake > loop_timing.wake_max) loop_timing.wake_max = t->wake;
    if (release_to_send > loop_timing.send_max) {
        loop_timing.send_max = release_to_send > 0xFFFFFFFFu ? 0xFFFFFFFFu : (uint32_t)release_to_send;
    }
    if (exec > loop_timing.exec_max) loop_timing.exec_max = exec;
    if (exec > loop_timing.exec_max_run) loop_timing.exec_max_run = exec;
}

#if defined(__arm__) && !defined(HOST_TEST)
#include "FreeRTOS.h"
#include "task.h"
#include "xil_printf.h"

/* The period timer is the tick timer of FreeRTOS: a TTC that the port of the BSP (portZynqUltrascale.c) runs in
 * interval mode, counting up from 0 to its interval, with an interrupt at the wrap that is the tick.  So its count
 * is the time since the release, in parts of its interval. */
#define TIMER_REG(offset)   (*(volatile uint32_t *)((uintptr_t)configTIMER_BASEADDR + (offset)))
#define TIMER_CONTROL       TIMER_REG(0x0CU)    /* bit 0: stopped, bit 1: interval mode, bit 2: counting down */
#define TIMER_COUNT         TIMER_REG(0x18U)
#define TIMER_INTERVAL      TIMER_REG(0x24U)

static uint32_t timer_interval;

int loop_timing_start(void)
{
    const uint32_t control = TIMER_CONTROL;

    timer_interval = TIMER_INTERVAL;
    if ((control & 0x7U) != 0x2U || timer_interval == 0U) {
        xil_printf("[ecat] FAIL: the tick timer at 0x%08x is not running up in interval mode (control 0x%02x, interval %u):"
                   " the release of a cycle cannot be told from it\r\n",
                   (unsigned)configTIMER_BASEADDR, (unsigned)control, (unsigned)timer_interval);
        return 0;
    }
    memset(&loop_timing, 0, sizeof(loop_timing));
    loop_timing_reset_window();
    xil_printf("[ecat] release: the tick timer at 0x%08x, %u counts a period; system counter %u ticks/s; send deadline %u us"
               " after the release\r\n", (unsigned)configTIMER_BASEADDR, (unsigned)timer_interval, (unsigned)SYSCNT_HZ,
               (unsigned)(FW_CFG_SEND_DEADLINE_NS / 1000));
    return 1;
}

void loop_timing_begin(uint32_t planned_tick, cycle_times_t *t)
{
    uint32_t tick, count;
    uint64_t now;

    do {                                /* tick count and timer of the same period: a tick between the reads shows */
        tick  = (uint32_t)xTaskGetTickCount();
        count = TIMER_COUNT;
        now   = sys_counter_now();
    } while (tick != (uint32_t)xTaskGetTickCount());
    t->start   = (uint32_t)now;
    t->wake    = cycle_wake_ticks(tick - planned_tick, count, timer_interval);
    t->release = now - t->wake;
}
#endif
