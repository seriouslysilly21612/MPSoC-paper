/*
 * obs_link.h  --  the R5's side of the camera-result mailbox of the APU (obs/obs_mailbox.h): the R5 is its reader.
 * One call per cycle takes what is READY, one look per slot, never waiting for the writer.  What the control task
 * does with a result (eq. (3), the joint record, the held list: obs_receiver.h of the APU build) is not here: it
 * moves to the R5 with the control task (plan 4-3).  Until then this module keeps the newest result that is still
 * valid, counts, and reports once per status window.
 *
 * The mailbox sits in the OCM window of ocm_link.h, behind the slots of ipc_layout.h, at OBS_OFF_MAILBOX of
 * obs/obs_mailbox.h: the file that both sides are built from, so the APU's writer maps the same offset.
 */
#ifndef OBS_LINK_H
#define OBS_LINK_H

#include <stdint.h>
#include "ocm_link.h"
#include "obs/obs_mailbox.h"

IPC_ASSERT(OBS_OFF_MAILBOX + sizeof(obs_mailbox_t) <= OCM_LINK_SIZE, "the mailbox is outside the OCM window");

/* After ocm_link_start() (the MPU window): the reader's start, a line with the state of the writer. */
void obs_link_start(void);

/* Every cycle: takes the READY results.  now = the system counter (ocm_link_counter()). */
void obs_link_poll(uint64_t now);

/* The newest result that has not expired yet.  1 = *out holds it, 0 = there is none. */
int  obs_link_latest(obs_desc_t *out);

/* Once per status window: posts the counters as an event (printed by event_log_task). */
void obs_link_report(uint64_t now);

#endif /* OBS_LINK_H */
