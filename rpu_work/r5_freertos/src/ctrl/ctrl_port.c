/*
 * ctrl_port.c  --  see ctrl_port.h.
 */
#include <stdarg.h>
#include <stdio.h>
#include "xil_printf.h"
#include "../sys_counter.h"
#include "ctrl_port.h"

enum { LOG_TRACE = 0, LOG_WARN = 1, LOG_ERROR = 2 };      /* eLogLevel of port/base/Common.h */

static int      in_loop;
static uint32_t lines_held;

uint64_t ctrl_port_ns(uint64_t ticks)
{
    /* [ns] per tick with 28 fraction bits (about 2^31.3 at 100 MHz); the two halves of the count apart, so that
     * no product passes 64 bits before the high half reaches 2^28 (39,000 years) */
    const uint64_t ns_per_tick = ((1000000000ull << 28) + SYSCNT_HZ / 2u) / SYSCNT_HZ;

    return (((ticks >> 32) * ns_per_tick) << 4) + (((ticks & 0xFFFFFFFFu) * ns_per_tick) >> 28);
}

uint64_t read_timer(void)
{
    return ctrl_port_ns(sys_counter_now());
}

int _gettimeofday(struct timeval *tv, void *tz)
{
    const uint64_t us = read_timer() / 1000u;

    (void)tz;
    tv->tv_sec  = (time_t)(us / 1000000u);
    tv->tv_usec = (suseconds_t)(us % 1000000u);
    return 0;
}

void ctrl_port_log(int level, const char *format, ...)
{
    char line[200];
    va_list args;

    if (in_loop) {
        if (level == LOG_WARN || level == LOG_ERROR) lines_held++;
        return;
    }
    if (level == LOG_TRACE) return;
    va_start(args, format);
    (void)vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    xil_printf("[ctrl] %s\r\n", line);
}

void ctrl_port_loop(void)
{
    in_loop = 1;
}

uint32_t ctrl_port_lines_held(void)
{
    return lines_held;
}
