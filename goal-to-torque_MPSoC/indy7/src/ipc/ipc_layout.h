/*****************************************************************************
*	Name: ipc_layout.h
*	Description: The data that the APU (Linux, the control task) and the R5
*	(FreeRTOS, the EtherCAT task) exchange once the EtherCAT task runs on the
*	R5: a head block and three slots at fixed offsets of one shared region.
*
*	    head     the R5 half and the APU half: magic, layout number, epoch
*	    State    R5 -> APU, every period: ST_STATE_SLOT of TaskSlots.h
*	    Torque   APU -> R5, every period: what the EtherCAT task takes of
*	             ST_TORQUE_SLOT
*	    Command  APU -> R5, now and then: a numbered command for the drives
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
#define IPC_LAYOUT	1u				/* + 1 with every change of this file */
enum { IPC_MAX_DOF = 8 };			/* MAX_DOF of TaskSlots.h */

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
enum { IPC_STATE_OP = 1u, IPC_STATE_WKC_ERR = 2u, IPC_STATE_SERVO_ON = 4u, IPC_STATE_STOP = 8u };
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

/* APU -> R5: the R5 carries out a command once, when (epoch, seq) changes */
enum { IPC_CMD_NONE = 0, IPC_CMD_SERVO_ON = 1, IPC_CMD_SERVO_OFF = 2, IPC_CMD_STOP = 3 };	/* eServoCmd */
typedef struct
{
	uint32_t	seq;
	uint32_t	cmd;				/* IPC_CMD_* */
} ipc_command_t;

/* the region: 64 bytes per writer at least, so no cache line has two writers but that of the head */
#define IPC_OFF_HEAD	0x000u
#define IPC_OFF_STATE	0x040u
#define IPC_OFF_TORQUE	0x1C0u
#define IPC_OFF_COMMAND	0x240u
#define IPC_OFF_END		0x280u

enum { IPC_SLOT_HEAD_WORDS = 2, IPC_WORDS_MAX = sizeof(ipc_state_t) / 4 };

IPC_ASSERT(sizeof(ipc_side_t) == 32 && sizeof(ipc_head_t) == 64, "ipc_head_t");
IPC_ASSERT(sizeof(ipc_state_t) == 328 && offsetof(ipc_state_t, period) == 8 && offsetof(ipc_state_t, status) == 40 &&
           offsetof(ipc_state_t, q) == 56 && offsetof(ipc_state_t, tau_act_raw) == 248 &&
           offsetof(ipc_state_t, n_late) == 312, "ipc_state_t");
IPC_ASSERT(sizeof(ipc_torque_t) == 72 && offsetof(ipc_torque_t, tau) == 8, "ipc_torque_t");
IPC_ASSERT(sizeof(ipc_command_t) == 8, "ipc_command_t");
IPC_ASSERT(sizeof(ipc_torque_t) / 4 <= IPC_WORDS_MAX && sizeof(ipc_command_t) / 4 <= IPC_WORDS_MAX, "IPC_WORDS_MAX");
IPC_ASSERT(IPC_OFF_HEAD + sizeof(ipc_head_t) <= IPC_OFF_STATE &&
           IPC_OFF_STATE + 4 * IPC_SLOT_HEAD_WORDS + sizeof(ipc_state_t) <= IPC_OFF_TORQUE &&
           IPC_OFF_TORQUE + 4 * IPC_SLOT_HEAD_WORDS + sizeof(ipc_torque_t) <= IPC_OFF_COMMAND &&
           IPC_OFF_COMMAND + 4 * IPC_SLOT_HEAD_WORDS + sizeof(ipc_command_t) <= IPC_OFF_END, "the slots overlap");
IPC_ASSERT(IPC_OFF_STATE % 64 == 0 && IPC_OFF_TORQUE % 64 == 0 && IPC_OFF_COMMAND % 64 == 0 && IPC_OFF_END % 64 == 0,
           "a slot is not on a 64-byte boundary");

#endif /* G2T_IPC_LAYOUT_H */
