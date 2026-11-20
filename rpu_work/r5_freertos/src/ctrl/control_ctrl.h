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
 * 0 = not built: the heap is too small, or the classes refused the constants (their lines say why). */
int      control_ctrl_build(void);
uint32_t control_ctrl_check_value(void);

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
