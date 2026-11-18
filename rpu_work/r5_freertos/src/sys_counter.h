/*
 * sys_counter.h  --  the system counter of the chip: the one clock of this firmware (D-08).
 *
 * The R5 reads the read-only window IOU_SCNTR (low word at 0x00, high word at 0x04); the APU reads the same counter
 * as CNTVCT.  So a time of the R5 (the release in the State slot, the record of a cycle) and a time of the APU are
 * numbers of one clock.  On the board (2026-11-18): the R5 may read the window, and the APU reports 99,999,000
 * ticks per second (CNTFRQ), about 10 ns a tick.
 *
 * SYSCNT_HZ is that number, typed here: the R5 has no register that tells it.  When it is wrong, the loop period in
 * the status report is not 1000 us (ecat_task.c).
 */
#ifndef SYS_COUNTER_H
#define SYS_COUNTER_H

#include <stdint.h>

#ifndef SYSCNT_HZ
#define SYSCNT_HZ           99999000u
#endif

#if defined(__arm__) && !defined(HOST_TEST)
#define SYSCNT_LOW          (*(volatile uint32_t *)0xFF250000UL)
#define SYSCNT_HIGH         (*(volatile uint32_t *)0xFF250004UL)

/* the low word alone: for the difference of two times less than 40 s apart (it wraps every 42.9 s) */
static inline uint32_t sys_counter_low(void)
{
    return SYSCNT_LOW;
}

static inline uint64_t sys_counter_now(void)
{
    uint32_t hi, lo, hi_again;

    do {                                             /* high, low, high: a carry between the two reads shows */
        hi = SYSCNT_HIGH;
        lo = SYSCNT_LOW;
        hi_again = SYSCNT_HIGH;
    } while (hi != hi_again);
    return ((uint64_t)hi << 32) | lo;
}
#else
extern uint64_t host_test_counter(void);             /* the host test provides it */
static inline uint64_t sys_counter_now(void) { return host_test_counter(); }
static inline uint32_t sys_counter_low(void) { return (uint32_t)host_test_counter(); }
#endif

/* [us] -> ticks */
static inline uint64_t sys_counter_ticks(uint32_t us)
{
    return (uint64_t)us * SYSCNT_HZ / 1000000u;
}

/* ticks -> [ns], rounded, for a span of 2 s at most (longer: 2 s).  A multiplication by 2^32 * 1e9 / SYSCNT_HZ
 * instead of a 64-bit division: it runs several times in every cycle.  Within 1 ns of the exact value. */
static inline uint32_t sys_counter_ns(uint32_t ticks)
{
    if (ticks > 2u * SYSCNT_HZ) ticks = 2u * SYSCNT_HZ;
    return (uint32_t)(((uint64_t)ticks * ((1000000000ull << 32) / SYSCNT_HZ) + 0x80000000u) >> 32);
}

#endif /* SYS_COUNTER_H */
