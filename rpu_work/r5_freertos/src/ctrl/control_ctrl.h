/*
 * control_ctrl.h  --  the control task on the R5 (ControlCycle.h, C++; S2) for the C side of the firmware: built once
 * from the constants of this build, then one step per period, after the EtherCAT send.
 *
 * The constants and the check value are those of the second controller (shadow_ctrl.h): the same numbers and the
 * same model.  The check value goes to the APU in every report.
 */
#ifndef CONTROL_CTRL_H
#define CONTROL_CTRL_H

#include <stdint.h>
#include "../ipc/ipc_layout.h"
#include "../obs/obs_receiver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* what a period starts from (CControlCycle::Input) */
typedef struct {
    uint32_t period;
    int      state;                 /* 1: the measured state below is of this period */
    int      sim_arm;               /* ... of the arm that the APU simulates (Sim state slot, step sim_step) */
    int      servo_on;              /* every joint in Operation enabled */
    uint32_t not_ready;             /* bit i: joint i cannot be switched on */
    uint32_t sim_step;
    uint64_t release;               /* the release of the period, ticks of the system counter */
    float    response;              /* [s] release to the end of the control computation of the period before; < 0: none */
    double   q[IPC_CTRL_DOF];       /* [rad] */
    double   qd[IPC_CTRL_DOF];      /* [rad/s] */
    double   tau[IPC_CTRL_DOF];     /* the torque the drives report [N m] */
    const ipc_command_t *command;   /* a command that is new in this period, or NULL */
    const ipc_goal_t    *goal;      /* a goal that is new in this period, or NULL */
} control_in_t;

/* what the drives get of it (CControlCycle::Output) */
typedef struct {
    int    torque;                  /* 1: the law ran, tau is the torque for the next send; 0: none */
    double tau[IPC_CTRL_DOF];       /* [N m], after the torque limit */
    int    servo;                   /* IPC_CMD_SERVO_ON or IPC_CMD_SERVO_OFF: to be asked of the drives; 0: nothing */
} control_out_t;

/* Builds the controller.  It takes memory from the C heap (_HEAP_SIZE of lscript.ld), here and never after.
 * region: the shared region of the link (ipc/ipc_layout.h), where the receiver of the camera results has its
 * mailbox, its status and its record (obs/obs_ring.h); ticks_per_s: of the counter of the releases.
 * 0 = not built: the heap is too small, or the classes refused the constants (their lines say why).  1 = built.
 * 2 = built, but the receiver cannot use its numbers of this build (CTRL_OBS_NUMBERS): IPC_CMD_OBS is refused. */
int      control_ctrl_build(volatile uint32_t *region, double ticks_per_s);
uint32_t control_ctrl_check_value(void);
/* the check value of the receiver's numbers of this build (CTRL_OBS_NUMBERS_CRC): IPC_CMD_OBS must bring it */
uint32_t control_ctrl_obs_check_value(void);
/* After control_ctrl_build() returned 1 or 2, before the periods: the clock (ticks_per_s of the build) of what the
 * receiver's work of a period takes (eq. (6)); without this call it is not measured. */
void     control_ctrl_set_obs_clock(uint64_t (*now)(void));
/* What that work took so far in the run of the receiver (obs_receiver.h, obs_cost_t).  Any task; words, no lock.
 * 0: no controller. */
int      control_ctrl_obs_cost(obs_cost_t *out);

/* One period.  Only after control_ctrl_build() returned 1.  No memory is taken.  Should a class throw, there is no
 * torque, the servo is asked off, the report has IPC_CTL_FAULT, and the cycle goes on. */
void     control_ctrl_step(const control_in_t *in, control_out_t *out, ipc_report_t *report);

/* The planner of the controller, as shadow_ctrl_serve(): from one task only, another than that of
 * control_ctrl_step() (planner_task.c), at any time.  1 = it did something, call again.  No memory is taken. */
int      control_ctrl_serve(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_CTRL_H */
