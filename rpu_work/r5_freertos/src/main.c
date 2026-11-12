/*
 * r5_freertos main.c -- KV260 RPU (Cortex-R5F): FreeRTOS + SOEM EtherCAT master, 4-axis CiA402 (CSP).
 *
 * Tasks
 *   control_task (cia402_soem.c)  prio MAX-2  EtherCAT bring-up, then the 1 kHz process-data loop.
 *                                 It creates its own low-priority log task after the bring-up.
 *   led_task                      prio 1      heartbeat + status LEDs (no console required)
 *
 * LEDs
 *   DS35 (heartbeat)   always blinking                  -> R5 alive, scheduler running
 *   DS36 (EtherCAT)    blinks together with DS35        -> starting: NIC / bus scan / bring-up to OP
 *                      solid ON                         -> all drives OPERATIONAL, 1 kHz loop running
 *                      blinks OPPOSITE to DS35          -> failed: init failed (nobody on the bus, wrong
 *                                                          slave count, drive refused OP ...) or BUS FAULT
 *
 * Console: every xil_printf() goes to the DDR ring buffer of r5log.c (r5log.c MUST be in the project);
 *          read it on the Linux side with  "sudo python3 r5log.py -f".
 *
 * Needed in src/: this file, cia402_soem.c, r5log.c, r5log.h, plus the SOEM + oshw/osal port you already have.
 * (The old scan-only soem_task is gone: control_task now does ec_init / ec_config_init / ec_config_map
 *  itself, so two tasks must not both own the NIC and SOEM's global state.)
 */
#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

#include "xparameters.h"
#include "xgpiops.h"
#include "xil_printf.h"

/* r5log.c (src/oshw/freertos/).  Declared here instead of #include "r5log.h" so that this file does not depend on
 * that folder being in the include path.  If r5log.c is not part of the build this is a LINK error, which is the
 * point: without it xil_printf() would block forever (BSP console = JTAG debug channel). */
extern void r5log_init(void);

#define LED_HB   7    /* DS35 heartbeat   */
#define LED_ST   8    /* DS36 EtherCAT status */

/* cia402_soem.c */
extern void control_task(void *arg);
extern volatile int cia402_state;        /* 0 starting, 1 running, -1 init failed, -2 bus fault */

static XGpioPs gpio;

/* ------------------------------------------------------------------ */
/* Heartbeat + status LEDs (no console required).                      */
/* ------------------------------------------------------------------ */
static void led_task(void *arg)
{
   int hb = 0;
   (void)arg;

   for (;;)
   {
      int st = cia402_state;

      hb ^= 1;
      XGpioPs_WritePin(&gpio, LED_HB, hb);              /* alive blink */

      if (st > 0)
         XGpioPs_WritePin(&gpio, LED_ST, 1);            /* solid: all drives OPERATIONAL, loop running */
      else if (st == 0)
         XGpioPs_WritePin(&gpio, LED_ST, hb);           /* blink together: starting */
      else
         XGpioPs_WritePin(&gpio, LED_ST, hb ^ 1);       /* opposite blink: failed */

      vTaskDelay(pdMS_TO_TICKS(250));
   }
}

int main(void)
{
   XGpioPs_Config *cfg;

   r5log_init();                        /* marks a fresh run in the ring buffer (boot_id + 1) */
   xil_printf("\r\n[main] r5_freertos starting: FreeRTOS tick %u Hz, %u priorities\r\n",
              (unsigned)configTICK_RATE_HZ, (unsigned)configMAX_PRIORITIES);

   cfg = XGpioPs_LookupConfig(XPAR_XGPIOPS_0_DEVICE_ID);
   if (cfg == NULL)
   {
      xil_printf("[main] FAIL: XGpioPs_LookupConfig returned NULL\r\n");
      for (;;) { }
   }
   XGpioPs_CfgInitialize(&gpio, cfg, cfg->BaseAddr);

   XGpioPs_SetDirectionPin(&gpio, LED_HB, 1);
   XGpioPs_SetOutputEnablePin(&gpio, LED_HB, 1);
   XGpioPs_SetDirectionPin(&gpio, LED_ST, 1);
   XGpioPs_SetOutputEnablePin(&gpio, LED_ST, 1);

   /* SOEM needs a roomy stack (protocol + config). Stack is in WORDS (4096 words = 16 KB). */
   if (xTaskCreate(control_task, "ecat", 4096, NULL, configMAX_PRIORITIES - 2, NULL) != pdPASS)
      xil_printf("[main] FAIL: could not create control_task (FreeRTOS heap too small? raise total_heap_size)\r\n");
   if (xTaskCreate(led_task, "led", 512, NULL, 1, NULL) != pdPASS)
      xil_printf("[main] FAIL: could not create led_task\r\n");

   vTaskStartScheduler();

   /* only reached if the scheduler could not start (no heap for the idle task) */
   xil_printf("[main] FAIL: vTaskStartScheduler returned\r\n");
   for (;;) { }
   return 0;
}
