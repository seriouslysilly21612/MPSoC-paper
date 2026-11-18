/*
 * r5_freertos main.c -- KV260 RPU (Cortex-R5F): FreeRTOS + SOEM EtherCAT master, CiA402 drives (CSP).
 * Which drives (the L7N bench or the Indy7) is ROBOT_TARGET in app_config.h.
 *
 * Tasks
 *   ecat_task (ecat_task.c)       prio MAX-2  EtherCAT bring-up, then the 1 kHz process-data loop; the APU's
 *                                 Command / Torque / State slots (ocm_link.c) are served inside that loop.
 *   event_log_task (event_log.c)  prio 1      prints the events the 1 kHz loop posts (it never prints itself).
 *
 * Console: every xil_printf() goes to the DDR ring buffer of r5log.c (r5log.c MUST be in the project);
 *          read it on the Linux side with  "sudo python3 r5log.py -f".
 */
#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

#include "xil_printf.h"

#include "ecat_task.h"
#include "event_log.h"

/* r5log.c (src/oshw/freertos/).  Declared here instead of #include "r5log.h" so that this file does not depend on
 * that folder being in the include path.  If r5log.c is not part of the build this is a LINK error, which is the
 * point: without it xil_printf() would block forever (BSP console = JTAG debug channel). */
extern void r5log_init(void);

int main(void)
{
   r5log_init();                        /* marks a fresh run in the ring buffer (boot_id + 1) */
   xil_printf("\r\n[main] r5_freertos starting: FreeRTOS tick %u Hz, %u priorities\r\n",
              (unsigned)configTICK_RATE_HZ, (unsigned)configMAX_PRIORITIES);

   /* SOEM needs a roomy stack (protocol + config), and the second controller of --shadow runs on it: 3 KB while it
    * is built, 5.5 KB in a cycle (measured on an emulated R5, 2026-11-18).  In WORDS: 8192 words = 32 KB. */
   if (xTaskCreate(ecat_task, "ecat", 8192, NULL, configMAX_PRIORITIES - 2, NULL) != pdPASS)
      xil_printf("[main] FAIL: could not create ecat_task (FreeRTOS heap too small? raise total_heap_size)\r\n");

   if (xTaskCreate(event_log_task, "ecatlog", 1024, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS)
      xil_printf("[main] WARN: could not start the log task (FreeRTOS heap full?) - event lines will not be printed\r\n");

   vTaskStartScheduler();

   /* only reached if the scheduler could not start (no heap for the idle task) */
   xil_printf("[main] FAIL: vTaskStartScheduler returned\r\n");
   for (;;) { }
   return 0;
}
