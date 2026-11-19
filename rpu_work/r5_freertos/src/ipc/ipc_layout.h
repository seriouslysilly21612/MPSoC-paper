/*****************************************************************************
*	Name: ipc_layout.h
*	Description: The data that the APU (Linux, the control task) and the R5
*	(FreeRTOS, the EtherCAT task) exchange once the EtherCAT task runs on the
*	R5: a head block and slots at fixed offsets of one shared region, in two
*	blocks with the mailbox of the camera results (obs/obs_mailbox.h) between.
*
*	    head     the R5 half and the APU half: magic, layout number, epoch
*	    State    R5 -> APU, every period: ST_STATE_SLOT of TaskSlots.h
*	    Torque   APU -> R5, every period: what the EtherCAT task takes of
*	             ST_TORQUE_SLOT (the control task on the APU: S1)
*	    Shadow in   APU -> R5, every period with --shadow: what the control
*	                task did with its controller, for the R5 to do the same
*	    Shadow out  R5 -> APU: what the controller of the R5 computed from it
*	  the second block:
*	    Command  APU -> R5, now and then: a numbered command, for the drives
*	             or for the control task on the R5
*	  and with the control task on the R5 (S2):
*	    Goal     APU -> R5: the latest goal of the tool (GoalSlot.h)
*	    Report   R5 -> APU, every period: what the control task did, the
*	             state of its motion and its controller, its answer to the
*	             last command, and the sample for the logs
*	    Sim state, Sim torque   APU -> R5 and back, with an arm simulated on
*	             the APU (-s, -b): the state of the arm, and the torque that
*	             the control task computed from it
*
*	C with fixed-width types, one file for both sides. Every word has one
*	writer. A slot is the two words of ipc_slot.h (sequence number, epoch of
*	the writer) and then its payload below. The sizes and offsets are
*	asserted at the end: a side built with another layout does not compile,
*	and one built from another version of this file is refused at run time
*	(IPC_LAYOUT).
*****************************************************************************/
#ifndef G2T_IPC_LAYOUT_H
#define G2T_IPC_LAYOUT_H

#include <stddef.h>
#include <stdint.h>

#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "ipc_layout.h: the A53 and the R5 are little-endian; this build is not"
#endif

#ifdef __cplusplus
#define IPC_ASSERT(cond, text) static_assert(cond, text)
#else
#define IPC_ASSERT(cond, text) _Static_assert(cond, text)
#endif

#define IPC_MAGIC	0x31435049u		/* "IPC1" */
#define IPC_LAYOUT	6u				/* + 1 with every change of this file */
enum { IPC_MAX_DOF = 8 };			/* MAX_DOF of TaskSlots.h */
enum { IPC_CTRL_DOF = 6 };			/* the joints of the controller (CHighLevelRT::DOF) */

/* the head block: each side writes its half when it starts (ipc_side_start) */
typedef struct
{
	uint32_t	magic;				/* IPC_MAGIC once the side is up */
	uint32_t	layout;				/* IPC_LAYOUT of its build */
	uint32_t	epoch;				/* + 1 with every start of the side; never 0 */
	uint32_t	reserved[5];
} ipc_side_t;

typedef struct
{
	ipc_side_t	r5;
	ipc_side_t	apu;
} ipc_head_t;

/* R5 -> APU: what the frame of this period brought */
enum
{
	IPC_STATE_OP = 1u, IPC_STATE_WKC_ERR = 2u, IPC_STATE_SERVO_ON = 4u, IPC_STATE_STOP = 8u,
	IPC_STATE_CONTROL = 16u			/* the control task runs on this R5 (S2): it takes no Torque slot */
};
typedef struct
{
	uint64_t	t_release;			/* release of the period, ticks of the system counter (IOU_SCNTR = CNTVCT) */
	uint32_t	period;				/* release index of the EtherCAT task */
	uint32_t	dof;
	uint32_t	flags;				/* IPC_STATE_*: bOp, bWkcErr, bServoOn, bStop of ST_STATE_SLOT */
	uint32_t	not_ready;			/* bit i: joint i in Switch on disabled or Not ready to switch on */
	uint32_t	cmd_done;			/* seq of the last command carried out */
	uint32_t	cfg_crc;			/* CRC-32 of the constants the firmware was built with */
	uint32_t	torque_period;		/* period of the Torque slot payload used in this period */
	uint32_t	n_torque_stale;		/* periods without a fresh torque while the drives were on, so far */
	uint16_t	status[IPC_MAX_DOF];	/* 0x6041 */
	double		q[IPC_MAX_DOF];		/* [rad] */
	double		qd[IPC_MAX_DOF];	/* [rad/s] */
	double		tau[IPC_MAX_DOF];	/* torque the drives report [N m] */
	int32_t		tau_act_raw[IPC_MAX_DOF];	/* 0x6077, drive units */
	int32_t		tau_cmd_raw[IPC_MAX_DOF];	/* 0x6071 written in this period, drive units */
	uint32_t	n_late, n_overrun, n_missed, n_wkc;	/* ST_FAULTS, so far */
} ipc_state_t;

/* APU -> R5: the torque of the control task's last cycle */
enum { IPC_TORQUE_VALID = 1u, IPC_TORQUE_ENABLED = 2u };
typedef struct
{
	uint32_t	period;				/* the State period it was computed from */
	uint32_t	flags;				/* IPC_TORQUE_*: bValid, bEnabled of ST_TORQUE_SLOT */
	double		tau[IPC_MAX_DOF];	/* [N m], after the torque limit */
} ipc_torque_t;

/* APU -> R5: the R5 carries out a command once, when (epoch, seq) changes. One at a time: the next is written
 * when the R5 has taken the last (cmd_done of the State slot). */
enum
{
	IPC_CMD_NONE = 0, IPC_CMD_SERVO_ON = 1, IPC_CMD_SERVO_OFF = 2, IPC_CMD_STOP = 3,	/* eServoCmd */
	/* for the control task on the R5 (S2; control/ControlCycle.h): the keys of the operator and the starts that
	 * the control task of the APU gives its own controller */
	IPC_CMD_CTRL_ON = 4, IPC_CMD_CTRL_OFF = 5,		/* 'r' */
	IPC_CMD_MOTION_STOP = 6,						/* 'x', 'g': the motion ends, gravity compensation */
	IPC_CMD_HOLD_ON = 7, IPC_CMD_HOLD_OFF = 8,		/* 'k' */
	IPC_CMD_START = 9								/* TRACK or HOME from the measured state (arg, delta, q_ready, target) */
};
enum { IPC_START_HOME = 1u, IPC_START_TARGET = 2u, IPC_START_FRICTION_FF = 4u };	/* arg of IPC_CMD_START */
typedef struct
{
	uint32_t	seq;
	uint32_t	cmd;				/* IPC_CMD_* */
	uint32_t	arg;				/* IPC_CMD_START: IPC_START_* (without HOME: TRACK; HOME to q_ready, or to target) */
	uint32_t	reserved;
	double		delta;				/* IPC_CMD_START: the goal threshold of the motion [m] */
	double		q_ready[IPC_CTRL_DOF];	/* ... the ready posture [rad] */
	double		target[IPC_CTRL_DOF];	/* ... IPC_START_TARGET [rad] */
} ipc_command_t;
/* what became of a command (Report slot) */
enum
{
	IPC_DONE_OK = 0,
	IPC_REFUSED_NO_STATE = 1,		/* a start, a servo on: no measured state in the period */
	IPC_REFUSED_NOT_ENABLED = 2,	/* a start, a servo on: the controller is not enabled */
	IPC_REFUSED_SERVO_OFF = 3,		/* a start: the servo is off */
	IPC_REFUSED_MODE = 4,			/* a start, a servo on: the law is not in gravity compensation */
	IPC_REFUSED_START = 5,			/* a start: CTrajectoryRT::Start() refused it (refusal_* of the report) */
	IPC_REFUSED_FAULT = 6,			/* a servo on: a controller fault is latched */
	IPC_REFUSED_NOT_READY = 7,		/* a servo on: a drive cannot be switched on */
	IPC_REFUSED_UNKNOWN = 8,		/* not a command of this control task */
	IPC_REFUSED_ARGUMENT = 9		/* a start: a number of the command is not finite or outside its range (refusal_*:
									 * joint -1 the goal threshold, else that joint of the ready posture) */
};

/* APU -> R5 with --shadow: the control task's period, as far as a second controller needs it to compute the
 * same torque (shadow/ShadowWriter.h writes it, shadow/ShadowControl.h follows it). The torque of the APU goes
 * along, so that the R5 can tell the difference in the period in which it computes. */
enum
{
	IPC_SHADOW_TRAJ        = 1u << 0,	/* the high-level controller was updated (a motion was running) ... */
	IPC_SHADOW_GOAL        = 1u << 1,	/* ... with `goal` (without: with no goal) */
	IPC_SHADOW_START       = 1u << 2,	/* a motion started in this period, from q and qd ... */
	IPC_SHADOW_START_AFTER = 1u << 3,	/* ... after the update of this period (without: before it) */
	IPC_SHADOW_START_HOME  = 1u << 4,	/* ... HOME to start_target (without: TRACK) */
	IPC_SHADOW_STARTS_LOST = 1u << 5,	/* the update of this period belonged to a motion whose start is not told */
	IPC_SHADOW_CTRL        = 1u << 6,	/* the control law was updated ... */
	IPC_SHADOW_REF         = 1u << 7,	/* ... on the reference of the high-level controller of this period */
	IPC_SHADOW_FRICTION_FF = 1u << 8,	/* ... with the friction feedforward of the motion */
	IPC_SHADOW_HOLD        = 1u << 9,	/* the hold of gravity compensation is switched on */
	IPC_SHADOW_IK          = 1u << 10,	/* the update took the IK of its goal, converged to ik_q (one that ended at
										 * N_max changes no reference and is not told) */
	IPC_SHADOW_PLAN        = 1u << 11	/* a plan came into use in the update */
};
typedef struct
{
	uint32_t	period;				/* the State period of this cycle of the control task */
	uint32_t	flags;				/* IPC_SHADOW_* */
	uint32_t	starts;				/* motions started so far, this period included */
	uint32_t	ctrl_mode;			/* eControlMode of the control law in this period */
	double		q[IPC_CTRL_DOF];	/* the measured state that the control task used, for all it did in the period [rad] */
	double		qd[IPC_CTRL_DOF];	/* [rad/s] */
	double		goal[3];			/* IPC_SHADOW_GOAL: the goal of the tool given to the high-level controller [m] */
	double		delta;				/* its goal threshold at the update [m] */
	double		q_ready[IPC_CTRL_DOF];		/* its ready posture at the update [rad] */
	double		start_target[IPC_CTRL_DOF];	/* IPC_SHADOW_START_HOME */
	double		tau[IPC_CTRL_DOF];	/* IPC_SHADOW_CTRL: the torque of the APU's control law, after the limit [N m] */
	double		ik_q[IPC_CTRL_DOF];	/* IPC_SHADOW_IK: the solution of the APU's planner [rad] */
} ipc_shadow_in_t;

/* R5 -> APU: the answer to the Shadow in payload of `period` */
enum
{
	IPC_SHADOW_OUT_TRAJ   = 1u << 0,	/* the reference below is of this period */
	IPC_SHADOW_OUT_CTRL   = 1u << 1,	/* tau and tau_diff are of this period */
	IPC_SHADOW_OUT_FAULT  = 1u << 2,	/* the high-level controller or the control law of the R5 failed the period */
	IPC_SHADOW_OUT_APART  = 1u << 3		/* a payload or a start was missed: until the next start the R5's motion is
										 * not that of the APU, and tau_diff says nothing */
};
typedef struct
{
	uint32_t	period;
	uint32_t	flags;				/* IPC_SHADOW_OUT_* */
	uint32_t	ctrl_crc;			/* check value of the controller constants and the model the firmware was built with */
	uint32_t	n_missed;			/* periods of the control task whose payload the R5 did not get, so far */
	uint32_t	traj_ns;			/* time of the high-level controller in this period */
	uint32_t	ctrl_ns;			/* time of the control law in this period */
	uint32_t	n_plans;			/* the planner of this controller, so far: the plans it calculated ... */
	uint32_t	plan_us;			/* ... the last of them in this time, from start to end [us] */
	uint32_t	n_iks;				/* ... the IK it ran to their end ... */
	uint32_t	ik_us;				/* ... the last of them in this time, from its request to its result [us] */
	uint32_t	n_plan_late;		/* plans of this controller that were not back in their period, so far */
	uint32_t	n_ik_compared;		/* goals whose IK of this planner was compared with the APU's (ik_q), so far */
	double		q_ref[IPC_CTRL_DOF];	/* the reference of the R5's high-level controller */
	double		tau[IPC_CTRL_DOF];	/* the torque of the R5's control law [N m] */
	double		tau_diff;			/* largest |tau - tau of the APU| over the joints [N m] */
	double		ik_diff;			/* the last goal compared: largest |solution - that of the APU| over the joints [rad] */
} ipc_shadow_out_t;

/* APU -> R5 with the control task on the R5: the latest goal of the tool [m]. The number of the slot tells a new one. */
typedef struct
{
	double		p[3];
} ipc_goal_t;

/* APU -> R5 with an arm that the APU simulates: its state after `step` steps. The control task takes a new one
 * as the measured state of its period, in place of the bus (whose drives stay off). */
enum { IPC_SIM_SERVO_ON = 1u };
typedef struct
{
	uint32_t	step;				/* + 1 with every step of the arm; never 0 */
	uint32_t	flags;				/* IPC_SIM_SERVO_ON: every joint of the arm is in Operation enabled */
	uint32_t	not_ready;			/* bit i: joint i cannot be switched on */
	uint32_t	reserved;
	double		q[IPC_CTRL_DOF];	/* [rad] */
	double		qd[IPC_CTRL_DOF];	/* [rad/s] */
	double		tau[IPC_CTRL_DOF];	/* the torque the drives report [N m] */
} ipc_sim_state_t;
/* R5 -> APU: what the arm gets for its next step */
enum { IPC_SIM_TORQUE_USABLE = 1u, IPC_SIM_SERVO_ASKED = 2u };
typedef struct
{
	uint32_t	step;				/* the step of the Sim state that this answers */
	uint32_t	flags;				/* IPC_SIM_*: the law ran and the controller is enabled; the drives are asked on */
	double		tau[IPC_CTRL_DOF];	/* [N m], after the torque limit */
} ipc_sim_torque_t;

/* R5 -> APU with the control task on the R5: its period */
enum
{
	IPC_CTL_STATE       = 1u << 0,	/* the period had a measured state: the motion and the law could run */
	IPC_CTL_SIM_ARM     = 1u << 1,	/* ... that of the Sim state slot (step sim_step) */
	IPC_CTL_PAYLOAD     = 1u << 2,	/* `in` describes the period for a second controller (CShadowWriter::Finish) */
	IPC_CTL_ENABLED     = 1u << 3,	/* the controller is enabled */
	IPC_CTL_FAULT       = 1u << 4,	/* a controller fault is latched */
	IPC_CTL_SERVO_ASKED = 1u << 5,	/* the drives are asked on */
	IPC_CTL_SERVO_ON    = 1u << 6,	/* every joint was in Operation enabled */
	IPC_CTL_SAMPLE      = 1u << 7	/* the high-level controller was updated: the sample below is of this period */
};
typedef struct
{
	ipc_shadow_in_t	in;				/* what the control task did, as the control task of the APU tells it with --shadow:
									 * the measured state, the goal, the starts, the mode, the torque after the limit */
	uint32_t	state;				/* IPC_CTL_* */
	uint32_t	traj_state;			/* CTrajectoryRT::eState after the period */
	uint32_t	motion;				/* CTrajectoryRT::eMotion after the period */
	uint32_t	clamp;				/* bit i: the torque limit acted on joint i */
	uint32_t	ctrl_crc;			/* check value of the controller constants and the model of the firmware */
	uint32_t	sim_step;			/* the step of the last Sim state taken */
	uint32_t	traj_ns;			/* time of the trajectory part in this period (0: it did not run) */
	uint32_t	ctrl_ns;			/* time of the control law in this period (0: it did not run) */
	uint32_t	cmd_done;			/* seq of the last command taken ... */
	uint32_t	cmd_result;			/* ... IPC_DONE_OK or IPC_REFUSED_* */
	uint32_t	start_seq;			/* the same of the last IPC_CMD_START ... */
	uint32_t	start_result;
	int32_t		refusal_why;		/* ... IPC_REFUSED_START: CTrajectoryRT::Refusal */
	int32_t		refusal_joint;
	double		refusal_value, refusal_lo, refusal_hi;
	uint32_t	n_no_state;			/* so far: periods without a measured state */
	uint32_t	n_ctrl_faults;		/* controller faults latched */
	uint32_t	n_drive_stops;		/* times the drives left Operation enabled by themselves, or a servo on ran out */
	uint32_t	n_goals;			/* goals taken from the Goal slot */
	uint32_t	n_plans, plan_us, plan_max_us;	/* the planner: plans calculated, the last and the longest [us] */
	uint32_t	n_iks, ik_us, ik_max_us;		/* IK run to their end, the last and the longest [us] */
	uint32_t	n_plan_late;		/* plans that were not back in their period */
	uint32_t	hl_ns;				/* IPC_CTL_SAMPLE, CTrajectoryRT::Sample from here on: the step of the high-level controller */
	uint32_t	hl_flags;			/* CTrajectoryRT::HL_* */
	int32_t		otg;				/* ruckig::Result */
	uint32_t	lead_reset;
	int32_t		ik_iter;
	int32_t		goal_age;			/* periods since the goal changed */
	uint32_t	n_cycle;			/* periods of the motion before this one */
	uint32_t	n_accepted, n_converged, n_rejected, n_lead_resets;	/* CHighLevelRT::Status: so far in the motion */
	double		ik_err;				/* [m] */
	double		otg_duration;		/* [s] */
	double		accepted[3];		/* the accepted goal [m] */
	double		tool[3];			/* the tool position of the measurement [m] */
	double		target[IPC_CTRL_DOF];	/* the target of the newest plan asked for or in use [rad] */
	double		q_ref[IPC_CTRL_DOF], qd_ref[IPC_CTRL_DOF], qdd_ref[IPC_CTRL_DOF];
} ipc_report_t;

/* the region: 64 bytes per writer at least, so no cache line has two writers but that of the head. 0x240 was
 * the Command slot up to layout 4. */
#define IPC_OFF_HEAD		0x000u
#define IPC_OFF_STATE		0x040u
#define IPC_OFF_TORQUE		0x1C0u
#define IPC_OFF_SHADOW_IN	0x280u
#define IPC_OFF_SHADOW_OUT	0x400u
#define IPC_OFF_END			0x4C0u
/* the second block, behind the mailbox of the camera results (obs/obs_mailbox.h: 0x500 to 0x780) */
#define IPC_OFF_COMMAND		0x800u
#define IPC_OFF_GOAL		0x880u
#define IPC_OFF_SIM_STATE	0x8C0u
#define IPC_OFF_SIM_TORQUE	0x980u
#define IPC_OFF_REPORT		0x9C0u
#define IPC_OFF_END2		0xCC0u

enum { IPC_SLOT_HEAD_WORDS = 2, IPC_WORDS_MAX = sizeof(ipc_report_t) / 4 };	/* the largest payload */

IPC_ASSERT(sizeof(ipc_side_t) == 32 && sizeof(ipc_head_t) == 64, "ipc_head_t");
IPC_ASSERT(sizeof(ipc_state_t) == 328 && offsetof(ipc_state_t, period) == 8 && offsetof(ipc_state_t, status) == 40 &&
           offsetof(ipc_state_t, q) == 56 && offsetof(ipc_state_t, tau_act_raw) == 248 &&
           offsetof(ipc_state_t, n_late) == 312, "ipc_state_t");
IPC_ASSERT(sizeof(ipc_torque_t) == 72 && offsetof(ipc_torque_t, tau) == 8, "ipc_torque_t");
IPC_ASSERT(sizeof(ipc_command_t) == 120 && offsetof(ipc_command_t, delta) == 16 && offsetof(ipc_command_t, target) == 72,
           "ipc_command_t");
IPC_ASSERT(sizeof(ipc_shadow_in_t) == 336 && offsetof(ipc_shadow_in_t, q) == 16 && offsetof(ipc_shadow_in_t, goal) == 112 &&
           offsetof(ipc_shadow_in_t, start_target) == 192 && offsetof(ipc_shadow_in_t, tau) == 240 &&
           offsetof(ipc_shadow_in_t, ik_q) == 288, "ipc_shadow_in_t");
IPC_ASSERT(sizeof(ipc_shadow_out_t) == 160 && offsetof(ipc_shadow_out_t, n_plans) == 24 &&
           offsetof(ipc_shadow_out_t, q_ref) == 48 && offsetof(ipc_shadow_out_t, tau_diff) == 144 &&
           offsetof(ipc_shadow_out_t, ik_diff) == 152, "ipc_shadow_out_t");
IPC_ASSERT(sizeof(ipc_goal_t) == 24, "ipc_goal_t");
IPC_ASSERT(sizeof(ipc_sim_state_t) == 160 && offsetof(ipc_sim_state_t, q) == 16 && offsetof(ipc_sim_state_t, tau) == 112,
           "ipc_sim_state_t");
IPC_ASSERT(sizeof(ipc_sim_torque_t) == 56 && offsetof(ipc_sim_torque_t, tau) == 8, "ipc_sim_torque_t");
IPC_ASSERT(sizeof(ipc_report_t) == 760 && offsetof(ipc_report_t, state) == 336 && offsetof(ipc_report_t, cmd_done) == 368 &&
           offsetof(ipc_report_t, refusal_value) == 392 && offsetof(ipc_report_t, n_no_state) == 416 &&
           offsetof(ipc_report_t, n_plans) == 432 && offsetof(ipc_report_t, hl_ns) == 460 &&
           offsetof(ipc_report_t, n_accepted) == 488 && offsetof(ipc_report_t, ik_err) == 504 &&
           offsetof(ipc_report_t, accepted) == 520 && offsetof(ipc_report_t, target) == 568 &&
           offsetof(ipc_report_t, q_ref) == 616, "ipc_report_t");
IPC_ASSERT(sizeof(ipc_state_t) / 4 <= IPC_WORDS_MAX && sizeof(ipc_torque_t) / 4 <= IPC_WORDS_MAX &&
           sizeof(ipc_command_t) / 4 <= IPC_WORDS_MAX && sizeof(ipc_shadow_in_t) / 4 <= IPC_WORDS_MAX &&
           sizeof(ipc_shadow_out_t) / 4 <= IPC_WORDS_MAX && sizeof(ipc_sim_state_t) / 4 <= IPC_WORDS_MAX,
           "IPC_WORDS_MAX");
IPC_ASSERT(IPC_OFF_HEAD + sizeof(ipc_head_t) <= IPC_OFF_STATE &&
           IPC_OFF_STATE + 4 * IPC_SLOT_HEAD_WORDS + sizeof(ipc_state_t) <= IPC_OFF_TORQUE &&
           IPC_OFF_TORQUE + 4 * IPC_SLOT_HEAD_WORDS + sizeof(ipc_torque_t) <= IPC_OFF_SHADOW_IN &&
           IPC_OFF_SHADOW_IN + 4 * IPC_SLOT_HEAD_WORDS + sizeof(ipc_shadow_in_t) <= IPC_OFF_SHADOW_OUT &&
           IPC_OFF_SHADOW_OUT + 4 * IPC_SLOT_HEAD_WORDS + sizeof(ipc_shadow_out_t) <= IPC_OFF_END &&
           IPC_OFF_COMMAND + 4 * IPC_SLOT_HEAD_WORDS + sizeof(ipc_command_t) <= IPC_OFF_GOAL &&
           IPC_OFF_GOAL + 4 * IPC_SLOT_HEAD_WORDS + sizeof(ipc_goal_t) <= IPC_OFF_SIM_STATE &&
           IPC_OFF_SIM_STATE + 4 * IPC_SLOT_HEAD_WORDS + sizeof(ipc_sim_state_t) <= IPC_OFF_SIM_TORQUE &&
           IPC_OFF_SIM_TORQUE + 4 * IPC_SLOT_HEAD_WORDS + sizeof(ipc_sim_torque_t) <= IPC_OFF_REPORT &&
           IPC_OFF_REPORT + 4 * IPC_SLOT_HEAD_WORDS + sizeof(ipc_report_t) <= IPC_OFF_END2, "the slots overlap");
IPC_ASSERT(IPC_OFF_STATE % 64 == 0 && IPC_OFF_TORQUE % 64 == 0 && IPC_OFF_SHADOW_IN % 64 == 0 &&
           IPC_OFF_SHADOW_OUT % 64 == 0 && IPC_OFF_END % 64 == 0 && IPC_OFF_COMMAND % 64 == 0 &&
           IPC_OFF_GOAL % 64 == 0 && IPC_OFF_SIM_STATE % 64 == 0 && IPC_OFF_SIM_TORQUE % 64 == 0 &&
           IPC_OFF_REPORT % 64 == 0 && IPC_OFF_END2 % 64 == 0, "a slot is not on a 64-byte boundary");

#endif /* G2T_IPC_LAYOUT_H */
