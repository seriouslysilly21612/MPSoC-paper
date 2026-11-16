/*
 * loop_timing.c  --  loop period / execution time statistics of the 1 kHz cycle (see loop_timing.h).
 */
#include "loop_timing.h"

loop_timing_t loop_timing;

void loop_timing_reset_window(void)
{
    loop_timing.periods = 0U; loop_timing.period_sum = 0U; loop_timing.period_min = 0xFFFFFFFFu;
    loop_timing.period_max = 0U; loop_timing.exec_max = 0U;
}

void loop_timing_record(uint32_t start, uint32_t end)
{
#if LOOP_TIMING
    uint32_t period, exec = end - start;
    if (loop_timing.warmup < 2U) {
        loop_timing.warmup++;          /* iterations 1 and 2 are not on the tick grid yet */
    } else {
        period = start - loop_timing.prev_start;
        loop_timing.periods++;
        loop_timing.period_sum += period;
        if (period < loop_timing.period_min) loop_timing.period_min = period;
        if (period > loop_timing.period_max) loop_timing.period_max = period;
    }
    loop_timing.prev_start = start;
    if (exec > loop_timing.exec_max) loop_timing.exec_max = exec;
    if (exec > loop_timing.exec_max_run) loop_timing.exec_max_run = exec;
    if (exec > CPU_CYCLES_PER_LOOP) loop_timing.overruns++;
#else
    (void)start; (void)end;
#endif
}
