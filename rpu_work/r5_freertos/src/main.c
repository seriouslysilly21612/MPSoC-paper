/*
 * r5_freertos main.c -- KV260 RPU (Cortex-R5F): FreeRTOS + SOEM EtherCAT master, CiA402 drives (CSP).
 * Which drives (the L7N bench or the Indy7) is ROBOT_TARGET in app_config.h.
 *
 * Tasks
 *   ecat_task (ecat_task.c)       prio MAX-2  EtherCAT bring-up, then the 1 kHz process-data loop; the APU's
 *                                 Command / Torque / State slots (ocm_link.c) are served inside that loop.
 *   event_log_task (event_log.c)  prio 1      prints the events the 1 kHz loop posts (it never prints itself).
 *   planner_task (planner_task.c) prio 2      with SHADOW_CONTROL or CONTROL_TASK: the IK and the plans that the
 *                                 controller asks for in the loop, in the time a period leaves.
 *
 * Console: every xil_printf() goes to the DDR ring buffer of r5log.c (r5log.c MUST be in the project);
 *          read it on the Linux side with  "sudo python3 r5log.py -f".
 */
#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

#include "xil_printf.h"
#include "xreg_cortexr5.h"
#include "xpseudo_asm.h"

#include "app_config.h"
#include "ecat_task.h"
#include "event_log.h"
#include "planner_task.h"

/* r5log.c (src/oshw/freertos/).  Declared here instead of #include "r5log.h" so that this file does not depend on
 * that folder being in the include path.  If r5log.c is not part of the build this is a LINK error, which is the
 * point: without it xil_printf() would block forever (BSP console = JTAG debug channel). */
extern void r5log_init(void);

int main(void)
{
   r5log_init();                        /* marks a fresh run in the ring buffer (boot_id + 1) */
   xil_printf("\r\n[main] r5_freertos starting: FreeRTOS tick %u Hz, %u priorities\r\n",
              (unsigned)configTICK_RATE_HZ, (unsigned)configMAX_PRIORITIES);
   {
      /* the times of the loop are with these caches: the boot code of the BSP switches both on */
      const u32 control = mfcp(XREG_CP15_SYS_CONTROL);

      xil_printf("[main] caches: instruction cache %s, data cache %s\r\n",
                 (control & XREG_CP15_CONTROL_I_BIT) ? "on" : "OFF", (control & XREG_CP15_CONTROL_C_BIT) ? "on" : "OFF");
   }

   /* SOEM needs a roomy stack (protocol + config), and the second controller of --shadow runs on it: 3 KB while it
    * is built, 3.9 KB in a cycle (measured on an emulated R5, 2026-11-19).  In WORDS: 8192 words = 32 KB. */
   if (xTaskCreate(ecat_task, "ecat", 8192, NULL, configMAX_PRIORITIES - 2, NULL) != pdPASS)
      xil_printf("[main] FAIL: could not create ecat_task (FreeRTOS heap too small? raise total_heap_size)\r\n");

   if (xTaskCreate(event_log_task, "ecatlog", 1024, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS)
      xil_printf("[main] WARN: could not start the log task (FreeRTOS heap full?) - event lines will not be printed\r\n");

#if SHADOW_CONTROL || CONTROL_TASK
   /* without it the controller gets no plan: its reference stays where a motion started */
   if (xTaskCreate(planner_task, "planner", PLANNER_TASK_STACK_WORDS, NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS)
      xil_printf("[main] FAIL: could not create planner_task (FreeRTOS heap too small? raise total_heap_size)\r\n");
#endif

   vTaskStartScheduler();

   /* only reached if the scheduler could not start (no heap for the idle task) */
   xil_printf("[main] FAIL: vTaskStartScheduler returned\r\n");
   for (;;) { }
   return 0;
}
