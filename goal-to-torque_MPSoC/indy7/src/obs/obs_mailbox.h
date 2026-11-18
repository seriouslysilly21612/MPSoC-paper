/*****************************************************************************
*	Name: obs_mailbox.h
*	Description: The camera results on their way to the control task: the
*	descriptor of eq. (5) and the 4-state mailbox of Alg. 1 (D-07, D-17,
*	D-18). C, one file for the writer (the bridge) and the reader (the
*	control task, on CPU3 now and on the R5 later).
*
*	  slot     8 of them, one descriptor of 64 bytes each.
*	  state    one word per slot: FREE -> WRITING -> READY (the writer),
*	           READY -> READING -> FREE (the reader). Every state has one
*	           side that may change it, so plain loads and stores do: no CAS.
*	           WRITING and READING only show where a side was (the four
*	           states of the draft): with no reclaim, the reader treats
*	           WRITING as FREE and the writer treats READING as READY.
*	  writer   never touches a READY or READING slot. With no FREE slot it
*	           drops the new result; a result already past t_valid_until is
*	           not written. Both are counted in the head.
*	  reader   looks at a slot once and never waits for the writer.
*	  start    each side puts back only its own half-done states: the writer
*	           WRITING (and any word that is no state), the reader READING.
*	           The writer counts its epoch up; a result of an older epoch is
*	           copied, freed and refused (OBS_STALE).
*
*	Words, barriers and the start of the writer as in ipc_slot.h.
*	No lock, no allocation, no system call.
*****************************************************************************/
#ifndef G2T_OBS_MAILBOX_H
#define G2T_OBS_MAILBOX_H

#include "../ipc/ipc_slot.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OBS_MAGIC	0x3153424Fu		/* "OBS1" */
#define OBS_LAYOUT	2u				/* + 1 with every change of this file */
/* where the mailbox lies in the region that the APU and the R5 share (ipc_layout.h), once its reader is on the R5:
 * behind the slots. One number for both sides. */
#define OBS_OFF_MAILBOX	0x500u
enum { OBS_SLOTS = 8, OBS_DESC_WORDS = 16 };

/* one camera result (eq. (5)): 16 words */
enum { OBS_Z_TAG = 1u, OBS_TAG_LOST = 2u, OBS_Z_BASE = 4u };		/* flags */
typedef struct
{
	uint32_t	n;				/* 1, 2, ... in the writer's epoch (set by obs_put) */
	uint32_t	epoch;			/* the writer's epoch (set by obs_put) */
	uint64_t	t_e;			/* capture, ticks of the system counter */
	uint64_t	f;				/* publication, read just before READY (set by obs_put) */
	uint64_t	t_valid_until;	/* t_e + the longest age of the mode: neither written nor used after it */
	float		z[3];			/* [m] the object from the tag, or in the base frame (flags) */
	float		sigma;			/* [m] the measurement error of the mode */
	uint8_t		mode;			/* 0 = 416, 1 = 320 */
	uint8_t		flags;			/* OBS_Z_TAG, OBS_TAG_LOST, OBS_Z_BASE */
	uint16_t	cls;			/* class of the detector */
	uint32_t	target_seq;		/* + 1 when the user picks another target */
	uint32_t	s;				/* 0 in the mailbox: the reader fills its copy (the state at t_e) */
	uint32_t	n_check;		/* = n (set by obs_put): a copy that mixes two results breaks it */
} obs_desc_t;

enum { OBS_FREE = 0u, OBS_WRITING = 1u, OBS_READY = 2u, OBS_READING = 3u };

/* the region. The writer writes the head and the slots; a state word changes hands */
typedef struct
{
	uint32_t	magic;				/* OBS_MAGIC once the writer is up */
	uint32_t	layout;				/* OBS_LAYOUT of the writer's build */
	uint32_t	epoch;				/* + 1 with every start of the writer; never 0 */
	uint32_t	n_last;				/* n of the last result written in this epoch */
	uint32_t	dropped_full;		/* results of this epoch dropped: no FREE slot */
	uint32_t	dropped_expired;	/* results of this epoch not written: past t_valid_until */
	uint32_t	reserved[10];
	uint32_t	state[OBS_SLOTS][2];	/* [b][0]; [b][1] unused: one state per 8-byte ECC word of the OCM */
	uint32_t	slot[OBS_SLOTS][OBS_DESC_WORDS];
} obs_mailbox_t;

IPC_ASSERT(sizeof(obs_desc_t) == 4 * OBS_DESC_WORDS && offsetof(obs_desc_t, f) == 16 &&
           offsetof(obs_desc_t, t_valid_until) == 24 && offsetof(obs_desc_t, z) == 32 &&
           offsetof(obs_desc_t, mode) == 48 && offsetof(obs_desc_t, target_seq) == 52 &&
           offsetof(obs_desc_t, n_check) == 60, "obs_desc_t");
IPC_ASSERT(offsetof(obs_mailbox_t, state) == 64 && offsetof(obs_mailbox_t, slot) == 128 &&
           sizeof(obs_mailbox_t) == 640, "obs_mailbox_t");
IPC_ASSERT(OBS_OFF_MAILBOX % 64u == 0u && OBS_OFF_MAILBOX >= IPC_OFF_END, "the mailbox overlaps the slots");

enum
{
	OBS_OK = 0,
	OBS_NO_PEER,		/* reader: the writer is not up, or was built with another layout */
	OBS_EMPTY,			/* reader: no READY result in the slot */
	OBS_STALE,			/* reader: copied, freed and refused: another epoch, a broken n_check, or n = 0 */
	OBS_FULL,			/* writer: no FREE slot; dropped and counted */
	OBS_EXPIRED			/* writer: past t_valid_until; not written, counted */
};

/* the writer starts (before its first obs_put). Returns its epoch. */
static inline uint32_t obs_writer_start(volatile obs_mailbox_t* mb)
{
	uint32_t epoch, st;
	unsigned b;

	ipc_store32(&mb->magic, 0u);
	ipc_barrier();
	for (b = 0; b < OBS_SLOTS; b++)
	{
		st = ipc_load32(&mb->state[b][0]);
		if (st != OBS_FREE && st != OBS_READY && st != OBS_READING)		/* its own WRITING, or garbage */
			ipc_store32(&mb->state[b][0], OBS_FREE);
	}
	epoch = ipc_load32(&mb->epoch) + 1u;
	if (epoch == 0u)
		epoch = 1u;
	ipc_store32(&mb->epoch, epoch);
	ipc_store32(&mb->n_last, 0u);
	ipc_store32(&mb->dropped_full, 0u);
	ipc_store32(&mb->dropped_expired, 0u);
	ipc_store32(&mb->layout, OBS_LAYOUT);
	ipc_barrier();
	ipc_store32(&mb->magic, OBS_MAGIC);
	return epoch;
}

/* the reader starts (before its first obs_take) */
static inline void obs_reader_start(volatile obs_mailbox_t* mb)
{
	unsigned b;

	for (b = 0; b < OBS_SLOTS; b++)
		if (ipc_load32(&mb->state[b][0]) == OBS_READING)
			ipc_store32(&mb->state[b][0], OBS_FREE);
}

/* the writer (one thread only): one result. obs_put sets n, epoch, f, s and n_check of *d; now() reads the
 * system counter. */
static inline int obs_put(volatile obs_mailbox_t* mb, obs_desc_t* d, uint64_t (*now)(void))
{
	uint32_t w[OBS_DESC_WORDS];
	unsigned b, i;

	if (now() > d->t_valid_until)
	{
		ipc_store32(&mb->dropped_expired, ipc_load32(&mb->dropped_expired) + 1u);
		return OBS_EXPIRED;
	}
	for (b = 0; b < OBS_SLOTS; b++)
		if (ipc_load32(&mb->state[b][0]) == OBS_FREE)
			break;
	if (b == OBS_SLOTS)
	{
		ipc_store32(&mb->dropped_full, ipc_load32(&mb->dropped_full) + 1u);
		return OBS_FULL;
	}
	ipc_store32(&mb->state[b][0], OBS_WRITING);		/* no barrier: the stores below depend on the FREE just read */
	d->n = ipc_load32(&mb->n_last) + 1u;
	d->n_check = d->n;
	d->epoch = ipc_load32(&mb->epoch);
	d->s = 0u;
	ipc_store32(&mb->n_last, d->n);
	memcpy(w, d, sizeof(w));
	for (i = 0; i < OBS_DESC_WORDS; i++)
		if (i != 4u && i != 5u)						/* f last */
			ipc_store32(&mb->slot[b][i], w[i]);
	ipc_barrier();
	d->f = now();
	memcpy(&w[4], &d->f, 8u);
	ipc_store32(&mb->slot[b][4], w[4]);
	ipc_store32(&mb->slot[b][5], w[5]);
	ipc_barrier();
	ipc_store32(&mb->state[b][0], OBS_READY);
	return OBS_OK;
}

/* the reader: slot b (< OBS_SLOTS), one look. OBS_OK, or OBS_STALE (refuse it): *d holds the copy. Otherwise *d
 * stays as it was. */
static inline int obs_take(volatile obs_mailbox_t* mb, unsigned b, obs_desc_t* d)
{
	uint32_t w[OBS_DESC_WORDS];
	unsigned i;

	if (ipc_load32(&mb->magic) != OBS_MAGIC || ipc_load32(&mb->layout) != OBS_LAYOUT)
		return OBS_NO_PEER;
	if (ipc_load32(&mb->state[b][0]) != OBS_READY)
		return OBS_EMPTY;
	ipc_barrier();								/* the result after its READY */
	ipc_store32(&mb->state[b][0], OBS_READING);
	for (i = 0; i < OBS_DESC_WORDS; i++)
		w[i] = ipc_load32(&mb->slot[b][i]);
	ipc_barrier();								/* the copy before FREE: then the writer may reuse the slot */
	ipc_store32(&mb->state[b][0], OBS_FREE);
	memcpy(d, w, sizeof(w));
	if (d->n == 0u || d->n_check != d->n || d->epoch != ipc_load32(&mb->epoch))
		return OBS_STALE;
	return OBS_OK;
}

#ifdef __cplusplus
}
#endif

#endif /* G2T_OBS_MAILBOX_H */
