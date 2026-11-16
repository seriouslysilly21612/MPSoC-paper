/*
 * loop_timing.h  --  loop period and execution time of the 1 kHz cycle, in CPU cycles of the Cortex-R5 PMU
 * cycle counter (PMCCNTR).  LOOP_TIMING 0 leaves the PMU alone and records nothing.
 */
#ifndef LOOP_TIMING_H
#define LOOP_TIMING_H

#include <stdint.h>
#include "app_config.h"

#if LOOP_TIMING
#if defined(__arm__) && !defined(HOST_TEST)
static inline void cycle_counter_start(void)
{
    uint32_t v;
    __asm__ volatile ("mrc p15, 0, %0, c9, c12, 0" : "=r"(v));      /* PMCR */
    v |= 0x5U;                                                      /* E = enable, C = reset the cycle counter */
    v &= ~0x8U;                                                     /* D = 0: count every CPU cycle */
    __asm__ volatile ("mcr p15, 0, %0, c9, c12, 0" : : "r"(v));
    __asm__ volatile ("mcr p15, 0, %0, c9, c12, 1" : : "r"(0x80000000U));   /* PMCNTENSET: cycle counter on */
}
static inline uint32_t cycle_counter_read(void)
{
    uint32_t v;
    __asm__ volatile ("mrc p15, 0, %0, c9, c13, 0" : "=r"(v));      /* PMCCNTR */
    return v;
}
#else
extern uint32_t host_test_cycles(void);                            /* host test provides it */
static inline void cycle_counter_start(void) { }
static inline uint32_t cycle_counter_read(void) { return host_test_cycles(); }
#endif
#else
static inline void cycle_counter_start(void) { }
static inline uint32_t cycle_counter_read(void) { return 0U; }
#endif

#define CPU_CYCLES_PER_US   ((uint32_t)((CPU_HZ + 500000UL) / 1000000UL))
#define CPU_CYCLES_PER_LOOP (CPU_CYCLES_PER_US * (CYCLE_NS / 1000u))

typedef struct {
    uint32_t prev_start;
    uint32_t warmup;                   /* iterations seen; the first period starts at an arbitrary point inside a tick */
    uint32_t periods;                  /* period samples in this window */
    uint64_t period_sum;
    uint32_t period_min, period_max, exec_max;   /* this window, CPU cycles */
    uint32_t exec_max_run;             /* whole run */
    uint32_t overruns;                 /* cycles whose execution took longer than one period */
} loop_timing_t;

extern loop_timing_t loop_timing;

void loop_timing_reset_window(void);
void loop_timing_record(uint32_t start, uint32_t end);   /* start / end: cycle_counter_read() around one cycle */

#endif /* LOOP_TIMING_H */
