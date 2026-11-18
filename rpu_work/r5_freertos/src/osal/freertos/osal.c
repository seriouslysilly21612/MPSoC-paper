/*
 * SOEM OSAL port for FreeRTOS on Xilinx Cortex-R5 (KV260 RPU)
 *
 * Implements the OS-abstraction surface the SOEM core actually uses:
 *   osal_usleep, osal_current_time, osal_time_diff,
 *   osal_timer_start, osal_timer_is_expired,
 *   osal_thread_create, osal_thread_create_rt
 *
 * TIME SOURCE: the system counter of the chip (sys_counter.h), about 10 ns a tick (plan 2-4).  With the FreeRTOS
 * tick (v1) a timeout ended at a tick: the receive of a lost frame waited up to 1 ms, to the next tick, and the
 * send of that cycle left a period late.  Now a timeout ends after its microseconds.
 *   osal_usleep still sleeps in whole ticks: SOEM calls it between the steps of the bring-up, not in a cycle.
 */

#include <stddef.h>
#include "osal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "../../sys_counter.h"

/* microseconds per FreeRTOS tick */
#define OSAL_USEC_PER_TICK   (1000000UL / configTICK_RATE_HZ)

/* Priorities for SOEM helper threads. Must be < configMAX_PRIORITIES.
 * Tune later so the EtherCAT cyclic task sits above housekeeping. */
#ifndef OSAL_THREAD_PRIORITY
#define OSAL_THREAD_PRIORITY     (tskIDLE_PRIORITY + 2)
#endif
#ifndef OSAL_THREAD_PRIORITY_RT
#define OSAL_THREAD_PRIORITY_RT  (configMAX_PRIORITIES - 1)
#endif

int osal_usleep(uint32 usec)
{
   /* round up to at least one tick; sub-tick requests become one tick */
   TickType_t ticks = (usec + OSAL_USEC_PER_TICK - 1) / OSAL_USEC_PER_TICK;
   if (ticks == 0) ticks = 1;
   vTaskDelay(ticks);
   return 0;
}

/* time since the counter started, i.e. since the boot of the board */
ec_timet osal_current_time(void)
{
   ec_timet t;
   uint64_t ticks = sys_counter_now();
   t.sec  = (uint32)(ticks / SYSCNT_HZ);
   t.usec = (uint32)((ticks % SYSCNT_HZ) * 1000000u / SYSCNT_HZ);
   return t;
}

void osal_time_diff(ec_timet *start, ec_timet *end, ec_timet *diff)
{
   if (end->usec < start->usec) {
      diff->sec  = end->sec - start->sec - 1;
      diff->usec = end->usec + 1000000UL - start->usec;
   } else {
      diff->sec  = end->sec - start->sec;
      diff->usec = end->usec - start->usec;
   }
}

/* A timer keeps its stop time in ticks of the system counter, not in seconds: the high word in stop_time.sec, the
 * low word in stop_time.usec.  Only the two functions below read it, and osal_timer_is_expired() runs in the
 * loop that waits for a frame: a comparison there, no division. */
void osal_timer_start(osal_timert *self, uint32 timeout_usec)
{
   uint64_t stop = sys_counter_now() + sys_counter_ticks(timeout_usec);
   self->stop_time.sec  = (uint32)(stop >> 32);
   self->stop_time.usec = (uint32)stop;
}

boolean osal_timer_is_expired(osal_timert *self)
{
   uint64_t stop = ((uint64_t)self->stop_time.sec << 32) | self->stop_time.usec;
   return (sys_counter_now() >= stop) ? TRUE : FALSE;
}

static int osal_thread_spawn(void *thandle, int stacksize, void *func,
                             void *param, UBaseType_t prio)
{
   /* SOEM passes stack size in BYTES; FreeRTOS wants WORDS (StackType_t units) */
   configSTACK_DEPTH_TYPE depth =
       (configSTACK_DEPTH_TYPE)(stacksize / sizeof(StackType_t));
   BaseType_t ok = xTaskCreate((TaskFunction_t)func, "soem",
                               depth, param, prio, (TaskHandle_t *)thandle);
   return (ok == pdPASS) ? 1 : 0;
}

int osal_thread_create(void *thandle, int stacksize, void *func, void *param)
{
   return osal_thread_spawn(thandle, stacksize, func, param, OSAL_THREAD_PRIORITY);
}

int osal_thread_create_rt(void *thandle, int stacksize, void *func, void *param)
{
   return osal_thread_spawn(thandle, stacksize, func, param, OSAL_THREAD_PRIORITY_RT);
}
