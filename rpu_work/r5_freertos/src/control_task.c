/*
 * control_task.c  --  see control_task.h.
 */
#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "control_task.h"
#include "control_link.h"

#if CONTROL_TASK
TaskHandle_t control_task_handle;

void control_task(void *arg)
{
    (void)arg;
    portTASK_USES_FLOATING_POINT();     /* the controller computes in double: this task needs an FPU context */
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (control_link_run()) {    /* one step per cycle handed over */
        }
    }
}

void control_task_wake(void)
{
    if (control_task_handle != NULL) xTaskNotifyGive(control_task_handle);
}
#endif /* CONTROL_TASK */
