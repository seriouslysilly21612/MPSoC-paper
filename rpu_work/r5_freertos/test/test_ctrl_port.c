/* host test of src/ctrl/ctrl_port.c: the clock of the controller classes and their log lines.  From r5_freertos/:
 *   gcc -std=gnu11 -Wall -DHOST_TEST -Itest/stub -Isrc -Isrc/ctrl test/test_ctrl_port.c src/ctrl/ctrl_port.c -o /tmp/t_port && /tmp/t_port
 * It checks: ticks -> ns never goes back, also at the carries of the low word of the counter, and a difference
 * is the exact time between the two counts within 2 ns, for spans from a tick to a second anywhere in a year of
 * running; read_timer() and the time of day are those of the system counter; a line is printed while the
 * controller is built (trace lines are not), and after ctrl_port_loop() nothing is printed and the warnings and
 * errors are counted. */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "sys_counter.h"
#include "ctrl_port.h"

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

static uint64_t counter;
uint64_t host_test_counter(void) { return counter; }

static char printed[400];
static int  prints;
void xil_printf(const char *format, ...)
{
    va_list args;

    va_start(args, format);
    vsnprintf(printed, sizeof(printed), format, args);
    va_end(args);
    prints++;
}

/* the difference of the values of `ticks` and of `span` ticks later, against the exact time of the span (a second
 * at most) */
static int span_ok(uint64_t ticks, uint32_t span)
{
    const int64_t exact = (int64_t)(((uint64_t)span * 1000000000ull + SYSCNT_HZ / 2u) / SYSCNT_HZ);
    const int64_t d = (int64_t)(ctrl_port_ns(ticks + span) - ctrl_port_ns(ticks)) - exact;

    return d >= -2 && d <= 2;
}

int main(void)
{
    static const uint32_t spans[] = { 1u, 37u, 999u, 99999u, 10000007u, SYSCNT_HZ };   /* a tick .. 1 ms .. 1 s */
    uint64_t t, before, rng = 20261118u;
    unsigned s;
    int k;

    CHECK(ctrl_port_ns(0) == 0 && span_ok(0, SYSCNT_HZ));
    for (t = 0; t < 300000u && span_ok(0, (uint32_t)t); t++) { }      /* every span of a cycle and beyond, from 0 */
    CHECK(t == 300000u);
    /* anywhere in a year of running (2^52 ticks), and right at the carries of the low word (43 s apart) */
    for (k = 0; k < 200000; k++) {
        rng = rng * 6364136223846793005ull + 1442695040888963407ull;
        t = (k % 2) ? (rng >> 12) : (((rng >> 44) + 1u) << 32) - spans[k % 6] / 2u;
        for (s = 0; s < 6 && span_ok(t, spans[s]); s++) { }
        if (s < 6) break;
    }
    CHECK(k == 200000);
    for (t = 0xFFFF0000u, before = 0, k = 0; k < 200000; k++, t += 1u + (uint64_t)(k % 7)) {  /* through a carry */
        const uint64_t ns = ctrl_port_ns(t);
        if (ns < before) break;
        before = ns;
    }
    CHECK(k == 200000);

    counter = 5ull * SYSCNT_HZ + 250u;
    CHECK(read_timer() == ctrl_port_ns(counter) && read_timer() >= 5000002490ull && read_timer() <= 5000002510ull);
    {
        struct timeval tv = { 0, 0 };

        counter = 7ull * SYSCNT_HZ + SYSCNT_HZ / 2u;                  /* 7.5 s */
        CHECK(_gettimeofday(&tv, NULL) == 0 && tv.tv_sec == 7 && tv.tv_usec >= 499999 && tv.tv_usec <= 500001);
    }

    ctrl_port_log(3, "(%s) gains %.2f, %d joints", "law", 2.5, 6);    /* an info line while the controller is built */
    CHECK(prints == 1 && strcmp(printed, "[ctrl] (law) gains 2.50, 6 joints\r\n") == 0);
    ctrl_port_log(0, "a trace line");
    CHECK(prints == 1 && ctrl_port_lines_held() == 0);
    ctrl_port_log(1, "a warning");
    CHECK(prints == 2);
    ctrl_port_loop();
    ctrl_port_log(3, "an info line in the loop");
    ctrl_port_log(0, "a trace line in the loop");
    CHECK(prints == 2 && ctrl_port_lines_held() == 0);
    ctrl_port_log(1, "a warning in the loop");
    ctrl_port_log(2, "an error in the loop %d", 7);
    CHECK(prints == 2 && ctrl_port_lines_held() == 2);

    printf(fails ? "test_ctrl_port: %d FAILED\n" : "test_ctrl_port OK\n", fails);
    return fails != 0;
}
