/*
 * control_task.h  --  the control step of every cycle in a task of its own (CONTROL_TASK of app_config.h; S2,
 * I-143): the EtherCAT task hands it a cycle after the send and wakes it (control_link.h).  The priority is below the
 * EtherCAT task, so the next release interrupts a step that runs long and the send stays on time, and above the
 * planner, which runs in what the two leave (planner_task.h).  The IK and the plans stay out of the step.
 */
#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include "FreeRTOS.h"
#include "task.h"

/* its stack [words], 32 KB: the step used 3.9 KB at most on the EtherCAT task's stack (emulated R5, 2026-11-19) */
#define CONTROL_TASK_STACK_WORDS 8192
/* the EtherCAT task is configMAX_PRIORITIES - 2, the planner tskIDLE_PRIORITY + 2 */
#define CONTROL_TASK_PRIORITY    (configMAX_PRIORITIES - 3)

extern TaskHandle_t control_task_handle;    /* from xTaskCreate() in main.c, before the scheduler starts */

void control_task(void *arg);               /* FreeRTOS task */
void control_task_wake(void);               /* the EtherCAT task: a cycle was handed over */

#endif /* CONTROL_TASK_H */
