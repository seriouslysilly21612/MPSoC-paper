/*
 * shadow_link.h  --  the second controller of --shadow in the 1 kHz cycle (plan stage 3: does the control
 * computation of the control task fit the period of the R5?).
 *
 * The control task of the APU tells every period what it did with its controller (Shadow in slot of
 * ipc/ipc_layout.h).  After the send of a cycle this module takes the payload that is new, runs the same controller
 * on it (ctrl/shadow_ctrl.h: the classes of the control task, built for this firmware), and writes the answer
 * (Shadow out slot): the torque it computed, its difference to the torque of the APU, the time of its two parts.
 * Its torque goes nowhere else: the drives get the torque of the APU, as before.  The payload it has in a cycle is
 * that of the period before: the control task is not through with the State of this cycle when the send returns.
 *
 * With SHADOW_CONTROL 0 (app_config.h) none of this is in the build.
 */
#ifndef SHADOW_LINK_H
#define SHADOW_LINK_H

#include <stdint.h>

/* Before the first cycle, from the task that will run the steps: builds the controller (C heap, the lines of its
 * classes are printed).  0 = not built (the reason is printed): the cycle runs on without it. */
int  shadow_link_start(void);

/* Every cycle, after the send: one look at the Shadow in slot.  1 = a new payload was there and the controller ran;
 * then *traj_ns and *ctrl_ns are the times of its high-level controller and of its control law [ns], -1 for a part
 * that the payload did not ask for.  0 = nothing ran, and the two are as they were. */
int  shadow_link_step(int32_t *traj_ns, int32_t *ctrl_ns);

/* Once per status window: posts the counters as an event (printed by event_log_task). */
void shadow_link_report(void);

#endif /* SHADOW_LINK_H */
