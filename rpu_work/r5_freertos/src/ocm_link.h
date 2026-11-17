/*
 * ocm_link.h  --  the R5's side of the shared region of the APU (ipc/ipc_layout.h): State slot out, Torque and
 * Command slots in.  Every call is one attempt and never waits for the APU.  The layout and the slot protocol are
 * the files of the APU build (src/ipc/), copied unchanged.
 */
#ifndef OCM_LINK_H
#define OCM_LINK_H

#include <stdint.h>
#include "app_config.h"
#include "ipc/ipc_layout.h"

#define OCM_LINK_BASE   0xFFFC0000UL      /* the first 16 KB of the OCM (D-09): the slots of ipc_layout.h, then the obs mailbox (obs_link.h) */
#define OCM_LINK_SIZE   0x00004000UL      /* the MPU window: a power of two, aligned */

/* Before the first cycle: the OCM window non-cacheable, the R5 half of the head block, the Command slot as it is
 * now (a command that was there before this start is not carried out).  0 = failed (reason printed). */
int      ocm_link_start(void);

uint64_t ocm_link_counter(void);                 /* system counter (IOU_SCNTR), the clock of t_release; 0 with LINK_TIMESTAMP 0 */

/* The command that is new since the last call (IPC_CMD_*), or IPC_CMD_NONE. */
uint32_t ocm_link_take_command(void);

/* The Torque slot as the last new one left it. */
typedef struct {
    int    usable;                  /* the APU marked it valid and enabled (IPC_TORQUE_VALID, IPC_TORQUE_ENABLED) */
    double tau[AXIS_COUNT];         /* [N m] */
} link_torque_t;

/* Reads the Torque slot once.  Returns 1 when it holds a torque that is new since the last call, and then *held is
 * that torque; returns 0 when it does not (nothing new, or a read that failed) and *held is the last new one. */
int      ocm_link_take_torque(link_torque_t *held);

/* Writes the State slot: this cycle's drive values.  flags: IPC_STATE_OP / _WKC_ERR / _STOP, the rest is added here. */
void     ocm_link_publish_state(uint64_t t_release, uint32_t period, uint32_t flags, uint32_t n_overrun, uint32_t n_wkc);

#endif /* OCM_LINK_H */
