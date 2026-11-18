/* host test of the clock of the SOEM OS layer (osal.c), on a counter the test sets.  From r5_freertos/:
 *   gcc -std=gnu11 -DHOST_TEST -Itest/stub -Isrc/osal -Isrc/osal/freertos test/test_osal_clock.c src/osal/freertos/osal.c -o /tmp/t_osal && /tmp/t_osal
 * It checks: a timer ends after its microseconds and not at a FreeRTOS tick, also across the carry of the counter's
 * low word and for the long timeouts of the bring-up; the current time in seconds and microseconds. */
#include <stdio.h>
#include "osal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "../src/sys_counter.h"

static uint64_t counter;
uint64_t host_test_counter(void) { return counter; }
void vTaskDelay(TickType_t ticks) { (void)ticks; }
BaseType_t xTaskCreate(TaskFunction_t f, const char *name, configSTACK_DEPTH_TYPE depth, void *arg, UBaseType_t priority, TaskHandle_t *handle)
{ (void)f; (void)name; (void)depth; (void)arg; (void)priority; (void)handle; return pdPASS; }
static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

/* a timer of `us` started at `start`: not expired one tick before its end, expired at it */
static void timer_ends_after(uint64_t start, uint32 us)
{
    osal_timert timer;
    const uint64_t ticks = (uint64_t)us * SYSCNT_HZ / 1000000u;

    counter = start;
    osal_timer_start(&timer, us);
    CHECK(!osal_timer_is_expired(&timer));
    counter = start + ticks - 1u;
    CHECK(!osal_timer_is_expired(&timer));
    counter = start + ticks;
    CHECK(osal_timer_is_expired(&timer));
    counter = start + ticks + 1000000u;
    CHECK(osal_timer_is_expired(&timer));
}

int main(void)
{
    ec_timet t;

    timer_ends_after(12345678u, 200);                          /* the receive timeout of a cycle: 19999 ticks */
    timer_ends_after(SYSCNT_HZ * 7ull + 99990u, 200);          /* started 0.1 us before a millisecond: no tick ends it */
    timer_ends_after(0xFFFFFFFFull - 100u, 200);               /* the low word of the counter carries inside the timeout */
    timer_ends_after(0x1FFFFFFFFull - 5u, 2000);
    timer_ends_after(0xFFFFFFFFull * 3u, 8000000u);            /* EC_TIMEOUTSTATE * 4 of the bring-up: 8 s */
    {                                                          /* a timeout of 0 has ended when it starts */
        osal_timert timer;
        counter = 5u;
        osal_timer_start(&timer, 0);
        CHECK(osal_timer_is_expired(&timer));
    }

    counter = 3ull * SYSCNT_HZ + SYSCNT_HZ / 2u;               /* 3.5 s since the counter started */
    t = osal_current_time();
    CHECK(t.sec == 3u && t.usec == 500000u);
    counter = 100000ull * SYSCNT_HZ + 123u;                    /* more than 2^32 ticks */
    t = osal_current_time();
    CHECK(t.sec == 100000u && t.usec == 1u);
    counter = SYSCNT_HZ - 1u;
    t = osal_current_time();
    CHECK(t.sec == 0u && t.usec == 999999u);

    printf(fails ? "test_osal_clock: %d FAILED\n" : "test_osal_clock OK\n", fails);
    return fails != 0;
}
