/*****************************************************************************
*	Name: control_vectors.h
*	Description: A file of periods of the control task without its
*	operating system (shadow/ControlCycle.h): the constants it is built
*	from (the head of shadow_vectors.h, with another magic), and per period
*	what it started from and what it must give. Written by
*	tools/control_check -o FILE on a PC from the control cycle of the
*	application; replayed by the control cycle as the firmware of the R5
*	builds it (rpu_work/r5_freertos/test/control_replay.cpp). C, fixed
*	widths, little-endian: the same bytes for both.
*****************************************************************************/
#ifndef G2T_CONTROL_VECTORS_H
#define G2T_CONTROL_VECTORS_H

#include "shadow_vectors.h"

#define CONTROL_VECTORS_MAGIC	0x31564343u		/* "CCV1" */

enum
{
	CONTROL_V_STATE = 1u,		/* the period has a measured state (q, qd) */
	CONTROL_V_SERVO_ON = 2u,	/* every joint in Operation enabled */
	CONTROL_V_COMMAND = 4u,		/* `command` is new in this period */
	CONTROL_V_GOAL = 8u,		/* `goal` is new in this period */
	CONTROL_V_TORQUE = 16u		/* the output: the law ran */
};

typedef struct
{
	uint32_t		period;
	uint32_t		flags;			/* CONTROL_V_* */
	uint32_t		not_ready;
	int32_t			servo;			/* the output: IPC_CMD_SERVO_ON, IPC_CMD_SERVO_OFF, or 0 */
	double			q[IPC_CTRL_DOF], qd[IPC_CTRL_DOF];
	double			goal[3];
	ipc_command_t	command;
	double			tau[IPC_CTRL_DOF];	/* the output: the torque for the drives [N m] */
	ipc_report_t	report;			/* the report; of its times only whether the part ran (1) or not (0) */
} control_vector_t;

IPC_ASSERT(sizeof(control_vector_t) == 16 + 8 * (2 * IPC_CTRL_DOF + 3) + sizeof(ipc_command_t) + 8 * IPC_CTRL_DOF +
           sizeof(ipc_report_t), "control_vector_t");

#endif /* G2T_CONTROL_VECTORS_H */
