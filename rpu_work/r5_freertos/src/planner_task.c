/*
 * planner_task.c  --  see planner_task.h.
 */
#include "FreeRTOS.h"
#include "task.h"

#include "planner_task.h"
#include "ctrl/shadow_ctrl.h"

void planner_task(void *arg)
{
    (void)arg;
    portTASK_USES_FLOATING_POINT();     /* the planner computes in double: this task needs an FPU context */
    for (;;) {
        while (shadow_ctrl_serve()) {   /* a plan before the IK, the IK one iteration at a time; 0 until the
                                           controller is built */
        }
        vTaskDelay(1);                  /* the next tick: its cycle runs first */
    }
}
