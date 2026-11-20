/*****************************************************************************
*	Name: obs_ring.h
*	Description: R5 -> APU with the receiver of the camera results on the R5
*	(S2: shadow/ControlCycle.h runs obs/ObsGoal.h): what the APU needs of
*	it besides the status (obs_status.h), in the region of the link behind
*	the slots (ipc/ipc_layout.h, IPC_OFF_END2 on).
*
*	  counts   one slot, written every period: the counts of the receiver
*	           (CObsGoal::Counts) and the entries written so far.
*	  entries  a ring of OBS_RING_CELLS slots: one entry per result that
*	           left the receiver (CObsGoal::Entry), number n in cell
*	           n % OBS_RING_CELLS. The APU takes them for the log of the run
*	           (ObsLog.h); one it takes too late is overwritten and counted
*	           as lost, not mixed up: the cell carries its number.
*
*	C with fixed-width types for both sides, as ipc_layout.h. One writer:
*	the control task.
*****************************************************************************/
#ifndef G2T_OBS_RING_H
#define G2T_OBS_RING_H

#include "obs_receiver.h"
#include "obs_status.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { OBS_RING_CELLS = 32, OBS_WORKSPACE_VERDICTS = 5 };		/* CGoalFilter::eVerdicts */

/* the entry of one result: CObsGoal::Entry */
typedef struct
{
	obs_record_t	rec;			/* the result and its verdict */
	int32_t			workspace;		/* a result used for the first time: the workspace check of its object; else -1 */
	int32_t			folded;			/* 1: the estimate took it; 2: and started again from it */
	double			obj[3];			/* its object in the base frame [m] (workspace >= 0) */
} obs_entry_t;

typedef struct
{
	uint32_t	no;					/* the number of the entry, 0, 1, ... since the writer started */
	uint32_t	reserved;
	obs_entry_t	e;
} obs_cell_t;

/* the counts of the receiver, written every period */
typedef struct
{
	uint32_t	taken;								/* results out of the mailbox */
	uint32_t	verdict[OBS_VERDICTS];
	uint32_t	workspace[OBS_WORKSPACE_VERDICTS];	/* of the objects of the results used */
	uint32_t	folded, restarts;					/* the estimate */
	uint32_t	goals;								/* object positions written into the goal slot */
	uint32_t	entries;							/* entries written into the ring: the next is number `entries` */
	uint32_t	method;								/* the method of the receiver (obs/ObsMethod.h); OBS_RING_OFF: none */
	uint32_t	flags;								/* OBS_RING_GOALS: its objects are the goals of TRACK */
} obs_counts_t;

enum { OBS_RING_OFF = 0xFFu, OBS_RING_GOALS = 1u };

typedef struct
{
	ipc_side_t	writer;
	uint32_t	counts[IPC_SLOT_HEAD_WORDS + sizeof(obs_counts_t) / 4];
	uint32_t	reserved[2];
	uint32_t	cell[OBS_RING_CELLS][IPC_SLOT_HEAD_WORDS + sizeof(obs_cell_t) / 4];
} obs_ring_t;

/* where they are in the region (ipc_layout.h) */
#define OBS_OFF_STATUS	0xD00u
#define OBS_OFF_RING	0xD80u

IPC_ASSERT(sizeof(obs_record_t) == 128 && offsetof(obs_record_t, t_out) == 112 && offsetof(obs_held_t, t_in) == 96,
           "obs_record_t");
IPC_ASSERT(sizeof(obs_entry_t) == 160 && offsetof(obs_entry_t, obj) == 136, "obs_entry_t");
IPC_ASSERT(sizeof(obs_cell_t) == 168 && sizeof(obs_cell_t) / 4 <= IPC_WORDS_MAX && sizeof(obs_counts_t) == 76,
           "obs_cell_t");
IPC_ASSERT(OBS_OFF_STATUS >= IPC_OFF_END2 && OBS_OFF_STATUS + sizeof(obs_status_region_t) <= OBS_OFF_RING &&
           OBS_OFF_RING % 64u == 0u && OBS_OFF_RING + sizeof(obs_ring_t) <= IPC_REGION_BYTES, "the region");

/* the writer starts (before its first obs_ring_counts) */
static inline void obs_ring_start(volatile obs_ring_t* ring)
{
	volatile uint32_t* slots[1 + OBS_RING_CELLS];
	unsigned i;

	slots[0] = ring->counts;
	for (i = 0; i < OBS_RING_CELLS; i++)
		slots[1 + i] = ring->cell[i];
	(void)ipc_side_start(&ring->writer, slots, 1u + OBS_RING_CELLS);
}

/* the writer: entry number `no` (counts.entries before it), then once per period the counts */
static inline void obs_ring_put(volatile obs_ring_t* ring, uint32_t no, const obs_entry_t* e)
{
	obs_cell_t c;

	memset(&c, 0, sizeof(c));
	c.no = no;
	c.e = *e;
	IPC_WRITE(ring->cell[no % OBS_RING_CELLS], ipc_load32(&ring->writer.epoch), &c);
}
static inline void obs_ring_counts(volatile obs_ring_t* ring, const obs_counts_t* counts)
{
	IPC_WRITE(ring->counts, ipc_load32(&ring->writer.epoch), counts);
}

/* the reader keeps where it is */
typedef struct
{
	uint32_t	epoch;				/* of the writer when it last read the counts; 0: not yet */
	uint32_t	next;				/* the number of the next entry to take */
	uint32_t	lost;				/* entries overwritten before it took them */
	uint32_t	stamp[2];
} obs_ring_reader_t;

/* the newest counts (IPC_OK), and from a writer that started again the entries from now on */
static inline int obs_ring_read_counts(const volatile obs_ring_t* ring, obs_ring_reader_t* r, obs_counts_t* counts)
{
	const int ok = IPC_READ(ring->counts, &ring->writer, counts, r->stamp);
	if (ok == IPC_OK && r->stamp[0] != r->epoch)
	{
		r->epoch = r->stamp[0];
		r->next = counts->entries;
	}
	return ok;
}

/* the next entry up to counts->entries (of obs_ring_read_counts): 1 with *e, 0 none now (taken, or a cell being
 * written: the next call tries again) */
static inline int obs_ring_take(const volatile obs_ring_t* ring, obs_ring_reader_t* r, const obs_counts_t* counts,
                                obs_entry_t* e)
{
	obs_cell_t c;
	uint32_t stamp[2];

	if (r->epoch == 0u || r->next == counts->entries)
		return 0;
	if (counts->entries - r->next > OBS_RING_CELLS)		/* overwritten already */
	{
		r->lost += counts->entries - OBS_RING_CELLS - r->next;
		r->next = counts->entries - OBS_RING_CELLS;
	}
	if (IPC_READ(ring->cell[r->next % OBS_RING_CELLS], &ring->writer, &c, stamp) != IPC_OK)
		return 0;
	if (c.no != r->next)			/* overwritten between the counts and now: lost */
	{
		r->lost++;
		r->next++;
		return 0;
	}
	*e = c.e;
	r->next++;
	return 1;
}

#ifdef __cplusplus
}
#endif

#endif /* G2T_OBS_RING_H */
