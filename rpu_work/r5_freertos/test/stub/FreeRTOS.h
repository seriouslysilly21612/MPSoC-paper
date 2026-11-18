#define configTICK_RATE_HZ 1000
#define configMAX_PRIORITIES 8
#define configSTACK_DEPTH_TYPE uint16_t
#define pdPASS 1
#include <stdint.h>
typedef uint32_t TickType_t; typedef long BaseType_t; typedef unsigned long UBaseType_t; typedef uint32_t StackType_t;
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define portTASK_USES_FLOATING_POINT()
