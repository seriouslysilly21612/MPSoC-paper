/*****************************************************************************
*	Name: obs_status.h
*	Description: What the control task tells the camera side in every period
*	(paper, Sec. IV-C; D-24). The camera side decides with it whether the
*	next image is worth an inference (eq. (8)): the tolerance of the period,
*	the camera result in use with its deadline, and the response of the
*	control task. C, one file for the writer (the control task, on CPU3 now
*	and on the R5 later) and the reader (the bridge).
*
*	  slot     the sequence-number slot of ipc_slot.h. It has a head of its
*	           own: its writer is the reader of the mailbox, whose head
*	           belongs to the other side.
*	  writer   one write per period; it never waits.
*	  reader   one attempt; a read that fails leaves the last copy as it was.
*
*	No lock, no allocation, no system call.
*****************************************************************************/
#ifndef G2T_OBS_STATUS_H
#define G2T_OBS_STATUS_H

#include "../ipc/ipc_slot.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { OBS_STATUS_RESULT_IN_USE = 1u, OBS_STATUS_TARGET_LOST = 2u };	/* flags */

/* the payload: 14 words. "result": the camera result that a period used last */
typedef struct
{
	uint64_t	t_release;				/* release of the period that wrote this, ticks of the system counter */
	uint64_t	result_t_e;				/* capture of the result (0: no result used so far) */
	uint64_t	result_t_valid_until;	/* its deadline */
	uint32_t	result_n;				/* its number in the mailbox */
	uint32_t	target_seq;				/* the target it belongs to */
	float		result_error_bound;		/* [m] eq. (7) of the result in the last period that used it */
	float		tolerance;				/* [m] delta of this period */
	float		response_max;			/* [s] the longest response of the control task, release to torque, of the
										 * last one to two seconds */
	uint32_t	contract_crc;			/* check value of the contract constants of this side (ObsContract.h) */
	float		control_period;			/* [s] the period of the control task: the deadline of its response */
	uint8_t		method;					/* obs_method::eMethod of this side */
	uint8_t		flags;					/* OBS_STATUS_RESULT_IN_USE: this period had a result to use;
										 * OBS_STATUS_TARGET_LOST: none for too long */
	uint8_t		result_mode;			/* mode of the result (obs_desc_t) */
	uint8_t		reserved8;
} obs_status_t;

enum { OBS_STATUS_WORDS = 14 };

typedef struct
{
	ipc_side_t	writer;											/* the control task: magic, layout, epoch */
	uint32_t	slot[IPC_SLOT_HEAD_WORDS + OBS_STATUS_WORDS];	/* sequence number, epoch, obs_status_t */
} obs_status_region_t;

IPC_ASSERT(sizeof(obs_status_t) == 4 * OBS_STATUS_WORDS && offsetof(obs_status_t, result_n) == 24 &&
           offsetof(obs_status_t, result_error_bound) == 32 && offsetof(obs_status_t, control_period) == 48 &&
           offsetof(obs_status_t, method) == 52, "obs_status_t");
IPC_ASSERT((unsigned)OBS_STATUS_WORDS <= (unsigned)IPC_WORDS_MAX && sizeof(obs_status_region_t) == 96,
           "obs_status_region_t");

/* the control task starts (before its first obs_status_write) */
static inline void obs_status_start(volatile obs_status_region_t* region)
{
	volatile uint32_t* const slots[1] = { region->slot };

	(void)ipc_side_start(&region->writer, slots, 1u);
}

/* the control task, once per period */
static inline void obs_status_write(volatile obs_status_region_t* region, const obs_status_t* status)
{
	IPC_WRITE(region->slot, ipc_load32(&region->writer.epoch), status);
}

/* the camera side, one attempt. IPC_OK: *status is the newest status and stamp its epoch and sequence number.
 * Otherwise (ipc_slot.h) *status and stamp stay as they were. */
static inline int obs_status_read(const volatile obs_status_region_t* region, obs_status_t* status, uint32_t stamp[2])
{
	return IPC_READ(region->slot, &region->writer, status, stamp);
}

/* The largest of the newest values, without a list of them: the values are counted in windows, and the answer is
 * the largest of the last full window and of the window being filled. It covers the last `window` to
 * 2 x `window` values. */
typedef struct
{
	uint32_t	window;			/* values per window */
	uint32_t	n_filling;		/* values in the window being filled */
	float		max_filling;	/* the largest of them */
	float		max_filled;		/* the largest of the last full window (0 before the first) */
} obs_recent_max_t;

static inline void obs_recent_max_start(obs_recent_max_t* recent, uint32_t window)
{
	recent->window = window > 0u ? window : 1u;
	recent->n_filling = 0u;
	recent->max_filling = 0.0f;
	recent->max_filled = 0.0f;
}

static inline void obs_recent_max_add(obs_recent_max_t* recent, float value)
{
	if (recent->n_filling == 0u || value > recent->max_filling)
		recent->max_filling = value;
	if (++recent->n_filling >= recent->window)
	{
		recent->max_filled = recent->max_filling;
		recent->n_filling = 0u;
	}
}

static inline float obs_recent_max(const obs_recent_max_t* recent)
{
	return (recent->n_filling != 0u && recent->max_filling > recent->max_filled) ? recent->max_filling
	                                                                               : recent->max_filled;
}

#ifdef __cplusplus
}
#endif

#endif /* G2T_OBS_STATUS_H */
