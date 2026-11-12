/*
 * SOEM OSAL port for FreeRTOS on Xilinx Cortex-R5 (KV260 RPU)
 * osal_defs.h - compile-time definitions
 */
#ifndef _osal_defs_
#define _osal_defs_

#ifdef __cplusplus
extern "C" {
#endif

/* Enable to route SOEM debug prints to the Xilinx console (xil_printf).
 * Leave OFF for real-time operation - printf in the cyclic path kills
 * determinism. Turn on only for bring-up debugging. */
/* #define EC_DEBUG */

#ifdef EC_DEBUG
#include "xil_printf.h"
#define EC_PRINT xil_printf
#else
#define EC_PRINT(...) do {} while (0)
#endif

/* Packed-struct attribute (GCC). EtherCAT datagrams are byte-packed on the
 * wire, so the protocol structs must not be padded. */
#ifndef PACKED
#define PACKED_BEGIN
#define PACKED  __attribute__((__packed__))
#define PACKED_END
#endif

/* Thread abstraction -> FreeRTOS tasks.
 * Usage in app code:
 *     OSAL_THREAD_HANDLE h;
 *     osal_thread_create(&h, stack_bytes, func, arg);
 */
#include "FreeRTOS.h"
#include "task.h"
#define OSAL_THREAD_HANDLE   TaskHandle_t
#define OSAL_THREAD_FUNC     void
#define OSAL_THREAD_FUNC_RT  void

#ifdef __cplusplus
}
#endif

#endif
