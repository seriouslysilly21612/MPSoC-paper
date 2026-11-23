/*
 * control_link.h  --  the control task on the R5 (CONTROL_TASK of app_config.h; S2) beside the EtherCAT task: the
 * control computation of ctrl/control_ctrl.h between the slots of the APU and the drives.
 *
 *   the EtherCAT task (ecat_task.c), in every cycle, where the cycle without it takes the Torque slot:
 *     control_link_drives()   the drives get what the step of the cycle before decided: its torque, and a servo
 *                             on or off; if that step is not done, they keep what they had
 *   ... after the send:
 *     control_link_give()     the cycle to the control task, which it then wakes
 *   the control task (control_task.c), below the EtherCAT task and above the planner:
 *     control_link_run()      the command and the goal of the APU that are new, the measured state of the cycle,
 *                             one period of the control task, its report to the APU
 *
 * So the torque computed from the state of a cycle goes out with the send of the next, as the torque of the
 * APU's control task does (send before compute), and the EtherCAT task is never late for a step: a step that runs
 * past the next release is interrupted there, that cycle sends the torque of the cycle before again and hands
 * nothing over, and TORQUE_STALE_LIMIT such cycles in a row switch the servo off (axis_control.c).
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

/* The EtherCAT task.  1 = the step handed over last is done (or none was), so the control task waits. */
int  control_link_idle(void);

/* The EtherCAT task: the drives of this cycle, see above.  Call it where the cycle takes the slots of the APU,
 * before the axes are updated. */
void control_link_drives(void);

/* The EtherCAT task, after the send of the cycle `period` and its State: bus_state: the frame of this cycle came
 * back complete and the firmware is not stopping; release: of this cycle, ticks of the system counter (the time
 * base of the camera results).  1 = handed over (then wake the control task); 0 = no controller, or the step of
 * the cycle before is not done. */
int  control_link_give(uint32_t period, int bus_state, uint64_t release);

/* The control task: the step of the cycle handed over, if it is new (1) */
int  control_link_run(void);

/* The EtherCAT task: is the step of `period` done?  1 with the low word of the counter at its end and the times of
 * the two parts of the control task (-1: that part did not run). */
int  control_link_result(uint32_t period, uint32_t *end, int32_t *traj_ns, int32_t *ctrl_ns);

/* The cycles so far that found the step of the cycle before not done */
uint32_t control_link_late_total(void);

/* One line into the event log (EVENT_CONTROL_REPORT): the counts of the window, which starts anew.  The EtherCAT
 * task, while the control task is in no step (control_link_idle, or a cycle handed over that it has not begun). */
void control_link_report(void);

#endif /* CONTROL_LINK_H */
