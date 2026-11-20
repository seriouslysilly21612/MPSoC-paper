/* host test of loop_timing.c and sys_counter.h (no board, no FreeRTOS).  From r5_freertos/:
 *   gcc -std=gnu11 -DHOST_TEST -Itest/stub -Isrc test/test_loop_timing.c src/loop_timing.c -o /tmp/t_loop && /tmp/t_loop
 * (add -DROBOT_TARGET=1 for the Indy7 constants)
 * It checks: counter ticks to ns against the exact division, t_w from the tick count and the period timer, the
 * times of a cycle in its record, the rules of late, missed and overrun at their limits, the counts, a wrap of the
 * low word of the counter inside a cycle, and the window of the status report. */
#include <stdio.h>
#include <string.h>
#include "loop_timing.h"

uint64_t host_test_counter(void) { return 0; }             /* loop_timing_end() takes its times as arguments */
static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

static uint32_t exact_ns(uint32_t ticks) { return (uint32_t)(((uint64_t)ticks * 1000000000ull + SYSCNT_HZ / 2u) / SYSCNT_HZ); }
static int ns_ok(uint32_t ticks) { int64_t d = (int64_t)sys_counter_ns(ticks) - (int64_t)exact_ns(ticks); return d >= -1 && d <= 1; }

/* a cycle that starts `wake` ticks after its release, with the low word of the counter at `start`, and ends `end`
 * ticks after its start */
static cycle_record_t cycle_to(uint32_t start, uint32_t wake, uint32_t recv, uint32_t send_begin, uint32_t send_end,
                               uint32_t end)
{
    cycle_times_t t = { .release = 0, .wake = wake, .start = start, .recv = start + recv,
                        .send_begin = start + send_begin, .send_end = start + send_end, .end = start + end };
    cycle_record_t r;

    memset(&r, 0, sizeof(r));
    loop_timing_end(&t, &r);
    return r;
}

/* the same, ending with its send */
static cycle_record_t cycle(uint32_t start, uint32_t wake, uint32_t recv, uint32_t send_begin, uint32_t send_end)
{
    return cycle_to(start, wake, recv, send_begin, send_end, send_end);
}

int main(void)
{
    cycle_record_t r;
    uint32_t k;

    /* the constants: 1 ms and the send deadline of the cfg in ticks */
    CHECK(TICKS_PER_CYCLE == 99999u && TICKS_SEND_DEADLINE == 40000u && FW_CFG_SEND_DEADLINE_NS == 400000);
    CHECK(sys_counter_ticks(200) == 19999u && sys_counter_ticks(1000000) == SYSCNT_HZ && sys_counter_ticks(0) == 0u);

    /* ticks -> ns: within 1 ns of the exact value, rounded; 2 s and more is 2 s */
    for (k = 0; k < 400000u && ns_ok(k); k++) { }          /* every span of a cycle and beyond: 0 .. 4 ms */
    CHECK(k == 400000u);
    for (k = 400000u; k <= 2u * SYSCNT_HZ - 7919u && ns_ok(k); k += 7919u) { }
    CHECK(k > 2u * SYSCNT_HZ - 7919u);
    CHECK(sys_counter_ns(99999u) == 1000000u && sys_counter_ns(2u * SYSCNT_HZ) == 2000000000u);
    CHECK(sys_counter_ns(0xFFFFFFFFu) == 2000000000u);

    /* t_w: the part of the period the timer has counted, plus whole periods */
    CHECK(cycle_wake_ticks(0, 0, 100000) == 0u);
    CHECK(cycle_wake_ticks(0, 500, 100000) == 499u);                    /* 5 us of a 100 MHz timer: 499.995 ticks */
    CHECK(cycle_wake_ticks(0, 50000, 100000) == TICKS_PER_CYCLE / 2u);
    CHECK(cycle_wake_ticks(0, 100000, 100000) == TICKS_PER_CYCLE);
    CHECK(cycle_wake_ticks(2, 25000, 100000) == 2u * TICKS_PER_CYCLE + TICKS_PER_CYCLE / 4u);
    CHECK(cycle_wake_ticks(0, 6250, 12500) == TICKS_PER_CYCLE / 2u);    /* a timer with a prescaler: another interval */
    CHECK(cycle_wake_ticks(0xFFFFFFFFu, 0, 100000) == 0xFFFFFFFFu);     /* no wrap: as late as can be told */

    /* an ordinary cycle: 400 ticks (4.0 us) late, the inputs in after 1500, the send from 5000 to 6200 (62000.6 ns) */
    memset(&loop_timing, 0, sizeof(loop_timing)); loop_timing_reset_window();
    r = cycle(1000000u, 400, 1500, 5000, 6200);
    CHECK(r.w_ns == 4000 && r.s_ns == 62001 && r.e_ns == 62001 && r.recv_ns == 15000 && r.send_ns == 12000);
    CHECK(r.flags == 0 && loop_timing.late == 0 && loop_timing.missed == 0 && loop_timing.overruns == 0);
    CHECK(loop_timing.periods == 0);                                    /* one start is no period yet */

    /* the send deadline: t_w + t_s up to t_b is in time, one tick more is missed */
    r = cycle(1100000u, 400, 1500, 5000, TICKS_SEND_DEADLINE - 400);
    CHECK(r.flags == 0 && loop_timing.missed == 0);
    r = cycle(1200000u, 400, 1500, 5000, TICKS_SEND_DEADLINE - 400 + 1);
    CHECK(r.flags == RECORD_MISSED && loop_timing.missed == 1 && loop_timing.late == 0 && loop_timing.overruns == 0);
    r = cycle(1300000u, TICKS_SEND_DEADLINE, 1500, 5000, 6200);         /* late enough to miss it with an ordinary cycle */
    CHECK(r.flags == RECORD_MISSED && loop_timing.missed == 2);

    /* overrun: the cycle ends after the next release; late: it starts more than a period after its own */
    r = cycle(1400000u, 400, 1500, 5000, TICKS_PER_CYCLE - 400);
    CHECK(r.flags == RECORD_MISSED && loop_timing.overruns == 0);
    r = cycle(1500000u, 400, 1500, 5000, TICKS_PER_CYCLE - 400 + 1);
    CHECK(r.flags == (RECORD_MISSED | RECORD_OVERRUN) && loop_timing.overruns == 1);
    r = cycle(1600000u, TICKS_PER_CYCLE, 1500, 5000, 6200);
    CHECK(r.flags == (RECORD_MISSED | RECORD_OVERRUN) && loop_timing.late == 0);
    r = cycle(1700000u, TICKS_PER_CYCLE + 1u, 1500, 5000, 6200);
    CHECK(r.flags == (RECORD_LATE | RECORD_MISSED | RECORD_OVERRUN) && loop_timing.late == 1);
    CHECK(loop_timing.missed == 6 && loop_timing.overruns == 3);

    /* a cycle that goes on after its send (the second controller): t_s is the send's, t_e the end's; the send
     * deadline is held by the send, the period by the end */
    r = cycle_to(1800000u, 400, 1500, 5000, 6200, 30000);
    CHECK(r.s_ns == 62001 && r.e_ns == 300003 && r.send_ns == 12000 && r.flags == 0);
    r = cycle_to(1900000u, 400, 1500, 5000, 6200, TICKS_PER_CYCLE - 400);
    CHECK(r.flags == 0 && loop_timing.missed == 6 && loop_timing.overruns == 3);
    r = cycle_to(2000000u, 400, 1500, 5000, 6200, TICKS_PER_CYCLE - 400 + 1);
    CHECK(r.flags == RECORD_OVERRUN && loop_timing.missed == 6 && loop_timing.overruns == 4);

    /* flags that the caller set stay, in a cycle in time and in one that breaks every rule */
    {
        cycle_times_t t = { .wake = 400, .start = 5, .recv = 10, .send_begin = 20, .send_end = 30, .end = 30 };
        memset(&r, 0, sizeof(r)); r.flags = RECORD_BUS | RECORD_OP;
        loop_timing_end(&t, &r);
        CHECK(r.flags == (RECORD_BUS | RECORD_OP));
        t.wake = TICKS_PER_CYCLE + 1u;
        memset(&r, 0, sizeof(r)); r.flags = RECORD_BUS | RECORD_OP;
        loop_timing_end(&t, &r);
        CHECK(r.flags == (RECORD_BUS | RECORD_OP | RECORD_LATE | RECORD_MISSED | RECORD_OVERRUN));
    }

    /* the low word of the counter wraps inside a cycle: the spans are the same */
    memset(&loop_timing, 0, sizeof(loop_timing)); loop_timing_reset_window();
    r = cycle(0xFFFFFFFFu - 3000u, 400, 1500, 5000, 6200);
    CHECK(r.w_ns == 4000 && r.s_ns == 62001 && r.recv_ns == 15000 && r.send_ns == 12000 && r.flags == 0);

    /* the window of a report: start-to-start periods, the largest t_w, release-to-send and cycle; a reset keeps
     * the counts and the largest cycle of the run */
    memset(&loop_timing, 0, sizeof(loop_timing)); loop_timing_reset_window();
    (void)cycle(0xFFFFFFF0u, 300, 1500, 5000, 6000);
    (void)cycle(0xFFFFFFF0u + 99999u, 500, 1500, 5000, 7000);           /* across the wrap */
    (void)cycle_to(0xFFFFFFF0u + 99999u + 100100u, 400, 1500, 5000, 50000, 70000);
    (void)cycle(0xFFFFFFF0u + 99999u + 100100u + 99898u, 200, 1500, 5000, 6500);
    CHECK(loop_timing.periods == 3 && loop_timing.period_min == 99898u && loop_timing.period_max == 100100u);
    CHECK(loop_timing.period_sum == 99999u + 100100u + 99898u);
    CHECK(loop_timing.wake_max == 500u && loop_timing.send_max == 50400u && loop_timing.exec_max == 70000u);
    CHECK(loop_timing.exec_max_run == 70000u && loop_timing.missed == 1);
    loop_timing_reset_window();
    CHECK(loop_timing.periods == 0 && loop_timing.wake_max == 0 && loop_timing.send_max == 0 && loop_timing.exec_max == 0);
    CHECK(loop_timing.exec_max_run == 70000u && loop_timing.missed == 1);
    (void)cycle(0xFFFFFFF0u + 99999u + 100100u + 99898u + 99999u, 100, 1500, 5000, 6000);
    CHECK(loop_timing.periods == 1 && loop_timing.period_min == 99999u);    /* the period into the new window */

    printf(fails ? "test_loop_timing: %d FAILED\n" : "test_loop_timing OK\n", fails);
    return fails != 0;
}
