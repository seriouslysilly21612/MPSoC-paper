/*
 * control_link.h  --  the control task on the R5 (CONTROL_TASK of app_config.h; S2) in the EtherCAT cycle: the
 * control computation of ctrl/control_ctrl.h between the slots of the APU and the drives.
 *
 *   in every cycle, where the cycle without it takes the Torque slot:
 *     control_link_drives()   the drives get what the step of the cycle before decided: its torque, and a servo
 *                             on or off
 *   after the send:
 *     control_link_step()     the command and the goal of the APU that are new, the measured state of this cycle,
 *                             one period of the control task, its report to the APU
 *
 * So the torque computed from the state of a cycle goes out with the send of the next, as the torque of the
 * APU's control task does (send before compute).
 *
 * The arm is that of the bus (the State slot of this cycle), when the bus has the joints of the controller
 * (AXIS_COUNT == IPC_CTRL_DOF: the Indy7).  When the APU of this run writes the Sim state slot, the arm is the
 * one that the APU simulates: a period of the control task then runs on each new state of it, its torque goes back
 * in the Sim torque slot, and the drives of the bus stay off.  On the L7N bench there is no other arm.  The
 * report of such a period is written before its torque, and a period without a new state of that arm writes
 * none (a command that it took is answered in the next report): the APU steps the arm when it has the torque of
 * its last state, so the report of every period that had a state is there for it until it has stepped.
 */
#ifndef CONTROL_LINK_H
#define CONTROL_LINK_H

#include <stdint.h>

/* Before the first cycle, after ocm_link_start(): builds the controller (ctrl/control_ctrl.h) and prints a line.
 * 0 = it is not built; the cycle then runs without it, and the drives stay off. */
int  control_link_start(void);

/* The drives of this cycle: see above.  Call it where the cycle takes the slots of the APU, before the axes are
 * updated. */
void control_link_drives(void);

/* After the send of the cycle `period`.  bus_state: the frame of this cycle came back complete and the firmware is
 * not stopping; release: of this cycle, ticks of the system counter (the time base of the camera results).
 * Returns 1 with the times of the two parts of the control task in *traj_ns and *ctrl_ns (-1: that part did not
 * run), 0 when there is no controller. */
int  control_link_step(uint32_t period, int bus_state, uint64_t release, int32_t *traj_ns, int32_t *ctrl_ns);

/* One line into the event log (EVENT_CONTROL_REPORT): the counts of the window, which starts anew. */
void control_link_report(void);

#endif /* CONTROL_LINK_H */
