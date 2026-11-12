/*
 * SOEM OSAL port for FreeRTOS on Xilinx Cortex-R5 (KV260 RPU)
 *
 * Implements the OS-abstraction surface the SOEM core actually uses:
 *   osal_usleep, osal_current_time, osal_time_diff,
 *   osal_timer_start, osal_timer_is_expired,
 *   osal_thread_create, osal_thread_create_rt
 *
 * TIME SOURCE (v1): FreeRTOS tick (configTICK_RATE_HZ).
 *   -> resolution = 1 tick (e.g. 1 ms at 1000 Hz).
 *   Fine for bring-up (slaveinfo / simple_test): EtherCAT frames return in
 *   microseconds, so the coarse timer only bounds timeout ceilings, not the
 *   happy path.
 *   TODO before 1 kHz hard-real-time control: replace osal_current_time()
 *   with a hardware high-resolution source - a free-running TTC channel, or
 *   the Cortex-R5 PMU cycle counter (PMCCNTR) - for microsecond resolution.
 */

#include <stddef.h>
#include "osal.h"
#include "FreeRTOS.h"
#include "task.h"

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

ec_timet osal_current_time(void)
{
   ec_timet t;
   TickType_t ticks = xTaskGetTickCount();
   t.sec  = (uint32)(ticks / configTICK_RATE_HZ);
   t.usec = (uint32)((ticks % configTICK_RATE_HZ) * OSAL_USEC_PER_TICK);
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

void osal_timer_start(osal_timert *self, uint32 timeout_usec)
{
   ec_timet now = osal_current_time();
   self->stop_time.sec  = now.sec  + timeout_usec / 1000000UL;
   self->stop_time.usec = now.usec + timeout_usec % 1000000UL;
   if (self->stop_time.usec >= 1000000UL) {
      self->stop_time.sec  += 1;
      self->stop_time.usec -= 1000000UL;
   }
}

boolean osal_timer_is_expired(osal_timert *self)
{
   ec_timet now = osal_current_time();
   if (now.sec > self->stop_time.sec) return TRUE;
   if (now.sec == self->stop_time.sec && now.usec >= self->stop_time.usec) return TRUE;
   return FALSE;
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
