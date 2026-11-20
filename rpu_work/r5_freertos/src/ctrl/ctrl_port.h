/*
 * ctrl_port.h  --  what the controller classes of src/ctrl take from their surroundings, on this firmware: a clock
 * and their log lines (port/base/Common.h declares the two for them).  ctrl_port.c also has _gettimeofday(), the
 * call under gettimeofday() of the C library, which the BSP lacks: the trajectory generator reads the time through
 * it (std::chrono::steady_clock, twice in every update).
 */
#ifndef CTRL_PORT_H
#define CTRL_PORT_H

#include <stdint.h>
#include <sys/time.h>

#ifdef __cplusplus
extern "C" {
#endif

/* seconds and microseconds on the system counter; 0 */
int      _gettimeofday(struct timeval *tv, void *tz);

/* [ns] on the system counter (sys_counter.h), for the difference of two readings: see ctrl_port_ns() */
uint64_t read_timer(void);

/* A line of the classes (eLogLevel of port/base/Common.h, then printf arguments).  While the controller is built it
 * is printed, but for the trace lines; once ctrl_port_loop() was called nothing is printed (the 1 kHz loop never
 * prints): a warning or an error is counted. */
void     ctrl_port_log(int level, const char *format, ...);

void     ctrl_port_loop(void);              /* from here the lines are counted, not printed */
uint32_t ctrl_port_lines_held(void);        /* warnings and errors not printed since ctrl_port_loop() */

/* ticks of the system counter -> [ns], without a division: it never goes back, and the difference of two values
 * up to a second apart is within 2 ns of the time between them (the value itself drifts by 2 parts in 10^10, which
 * no difference of this firmware notices) */
uint64_t ctrl_port_ns(uint64_t ticks);

#ifdef __cplusplus
}
#endif

#endif /* CTRL_PORT_H */
