/*
 * sys/mman.h of the firmware  --  FullDynControllerRT.h includes it for munlockall() (its destructor).  The R5
 * has no paging: nothing to unlock.
 */
#ifndef CTRL_PORT_SYS_MMAN_H
#define CTRL_PORT_SYS_MMAN_H

static inline int munlockall(void) { return 0; }

#endif /* CTRL_PORT_SYS_MMAN_H */
