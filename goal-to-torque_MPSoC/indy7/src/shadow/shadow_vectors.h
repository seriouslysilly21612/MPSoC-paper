/*****************************************************************************
*	Name: shadow_vectors.h
*	Description: A file of periods of the control task for the second
*	controller (--shadow): the constants it is built from, and per period
*	the payload of the Shadow in slot with what the answer must be, and
*	whether its planner gets to run after the period. Written
*	by tools/shadow_check -o FILE on a PC from the controller of the
*	application; replayed by the controller as the firmware of the R5 builds
*	it (rpu_work/r5_freertos/test/shadow_replay.cpp), which has another
*	compiler, another C library and its model from numbers. C, fixed widths,
*	little-endian: the same bytes for both.
*****************************************************************************/
#ifndef G2T_SHADOW_VECTORS_H
#define G2T_SHADOW_VECTORS_H

#include "ipc/ipc_layout.h"

#define SHADOW_VECTORS_MAGIC	0x31564853u		/* "SHV1" */
enum { SHADOW_VECTORS_NUMBERS = 50 };			/* SHADOW_NUMBERS of shadow/ShadowControl.h */

typedef struct
{
	uint32_t	magic;
	uint32_t	periods;			/* shadow_vector_t after this head */
	uint32_t	crc;				/* check value of the numbers */
	uint32_t	reserved;
	double		numbers[SHADOW_VECTORS_NUMBERS];
} shadow_vectors_head_t;

typedef struct
{
	ipc_shadow_in_t	in;				/* in.tau: the torque of the controller that wrote the file */
	double			q_ref[IPC_CTRL_DOF];	/* its reference, when out_flags has IPC_SHADOW_OUT_TRAJ */
	uint32_t		out_flags;		/* IPC_SHADOW_OUT_* that the answer must have */
	uint32_t		planner_stalled;	/* 1: the planner of the second controller is not served after this period */
} shadow_vector_t;

IPC_ASSERT(sizeof(shadow_vectors_head_t) == 16 + 8 * SHADOW_VECTORS_NUMBERS, "shadow_vectors_head_t");
IPC_ASSERT(sizeof(shadow_vector_t) == sizeof(ipc_shadow_in_t) + 8 * IPC_CTRL_DOF + 8, "shadow_vector_t");

#endif /* G2T_SHADOW_VECTORS_H */
