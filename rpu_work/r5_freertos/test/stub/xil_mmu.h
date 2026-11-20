/* host-test stand-in for the BSP header: the MPU region of the OCM window is ordinary memory of the test */
#ifndef XIL_MMU_H
#define XIL_MMU_H
#include <stdint.h>
typedef intptr_t INTPTR;
typedef uint32_t u32;
u32 Xil_SetMPURegion(INTPTR addr, u32 size, u32 attrib);
#endif
