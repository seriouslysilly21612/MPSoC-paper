#include <stdint.h>
typedef intptr_t INTPTR;
void Xil_DCacheFlushRange(INTPTR adr, uint32_t len);
void Xil_DCacheInvalidateRange(INTPTR adr, uint32_t len);
