typedef void (*TaskFunction_t)(void *); typedef void *TaskHandle_t;
#define tskIDLE_PRIORITY 0
void vTaskDelay(TickType_t ticks);
BaseType_t xTaskCreate(TaskFunction_t f, const char *name, configSTACK_DEPTH_TYPE depth, void *arg, UBaseType_t priority, TaskHandle_t *handle);
void vTaskDelayUntil(TickType_t *previous_wake, TickType_t increment);
TickType_t xTaskGetTickCount(void);
