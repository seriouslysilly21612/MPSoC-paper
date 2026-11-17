/*****************************************************************************
*	Name: ipc_slot.h
*	Description: The single-writer slot between the APU and the R5: the
*	sequence-number slot of base/AtomicSlot.h (eq. (1)) in the form the
*	shared region needs.
*
*	  words    32-bit, aligned, loaded and stored one by one: Linux maps the
*	           region as Device memory, where memcpy may fault.
*	  writer   publishes an odd sequence number, stores its epoch and the
*	           payload, then publishes the next even number.
*	  reader   one attempt: sequence number, payload, sequence number again.
*	           It never waits for the other processor; a read that fails
*	           leaves its last copy as it was.
*	  barrier  between the sequence number and the payload, on both sides.
*	  start    a side that starts leaves the sequence numbers of its slots
*	           odd and counts its epoch up (ipc_side_start). So nothing in
*	           the region needs to be zeroed, and what a side wrote before it
*	           restarted is never read as new: the slot stays "being
*	           written" until the first write after the start. The sequence
*	           number goes on from its last value: starting again from 0, a
*	           reader stopped in a copy could meet its old number again and
*	           accept a mixed copy.
*
*	A slot does not tell that its writer has died; the reader does, from a
*	value that no longer changes. No lock, no allocation, no system call.
*****************************************************************************/
#ifndef G2T_IPC_SLOT_H
#define G2T_IPC_SLOT_H

#include <string.h>

#include "ipc_layout.h"

#ifdef __cplusplus
extern "C" {
#endif

/* a plain 32-bit load and store (ldr / str) on the A53 and the R5 */
static inline uint32_t ipc_load32(const volatile uint32_t* p) { return __atomic_load_n(p, __ATOMIC_RELAXED); }
static inline void ipc_store32(volatile uint32_t* p, uint32_t v) { __atomic_store_n(p, v, __ATOMIC_RELAXED); }

static inline void ipc_barrier(void)
{
#if defined(__aarch64__) || defined(__arm__)
	__asm__ volatile("dmb sy" : : : "memory");	/* sy, not ish: the other processor is outside the inner-shareable domain */
#else
	__atomic_thread_fence(__ATOMIC_SEQ_CST);	/* the test on a PC */
#endif
}

enum { IPC_SLOT_GEN = 0, IPC_SLOT_EPOCH = 1 };		/* the two words before the payload */

/* a side starts (before its first write): the slots it writes, n of them. Returns its epoch. */
static inline uint32_t ipc_side_start(volatile ipc_side_t* me, volatile uint32_t* const* slots, unsigned n)
{
	uint32_t epoch;
	unsigned i;

	ipc_store32(&me->magic, 0u);
	ipc_barrier();
	for (i = 0; i < n; i++)
		ipc_store32(slots[i] + IPC_SLOT_GEN, ipc_load32(slots[i] + IPC_SLOT_GEN) | 1u);
	epoch = ipc_load32(&me->epoch) + 1u;
	if (epoch == 0u)
		epoch = 1u;
	ipc_store32(&me->epoch, epoch);
	ipc_store32(&me->layout, IPC_LAYOUT);
	ipc_barrier();
	ipc_store32(&me->magic, IPC_MAGIC);
	return epoch;
}

/* the writer of the slot (one thread only): words of the payload, sizeof / 4 */
static inline void ipc_slot_write(volatile uint32_t* slot, uint32_t epoch, const void* payload, unsigned words)
{
	uint32_t gen = ipc_load32(slot + IPC_SLOT_GEN) | 1u;	/* odd already after a start, or a writer that died in a write */
	uint32_t w;
	unsigned i;

	ipc_store32(slot + IPC_SLOT_GEN, gen);
	ipc_barrier();
	ipc_store32(slot + IPC_SLOT_EPOCH, epoch);
	for (i = 0; i < words; i++)
	{
		memcpy(&w, (const unsigned char*)payload + 4u * i, 4u);
		ipc_store32(slot + IPC_SLOT_HEAD_WORDS + i, w);
	}
	ipc_barrier();
	gen += 1u;
	if (gen == 0u)
		gen = 2u;		/* the wrap: 0 is the number of a slot never written */
	ipc_store32(slot + IPC_SLOT_GEN, gen);
}

enum
{
	IPC_OK = 0,
	IPC_NO_PEER,		/* the writer's side is not up, or was built with another layout */
	IPC_BUSY,			/* being written, or not written since the writer started */
	IPC_TORN,			/* written during the copy */
	IPC_OLD_EPOCH,		/* written before the writer restarted */
	IPC_BAD_SIZE
};

/* a reader, one attempt. IPC_OK: the payload, and in stamp its epoch and sequence number, which differ from
 * those of every earlier payload. Otherwise payload and stamp stay as they were. */
static inline int ipc_slot_read(const volatile uint32_t* slot, const volatile ipc_side_t* writer, void* payload,
                                unsigned words, uint32_t stamp[2])
{
	uint32_t buf[IPC_WORDS_MAX], gen, epoch;
	unsigned i;

	if (words > IPC_WORDS_MAX)
		return IPC_BAD_SIZE;
	if (ipc_load32(&writer->magic) != IPC_MAGIC || ipc_load32(&writer->layout) != IPC_LAYOUT)
		return IPC_NO_PEER;
	gen = ipc_load32(slot + IPC_SLOT_GEN);
	if (gen & 1u)
		return IPC_BUSY;
	ipc_barrier();
	epoch = ipc_load32(slot + IPC_SLOT_EPOCH);
	for (i = 0; i < words; i++)
		buf[i] = ipc_load32(slot + IPC_SLOT_HEAD_WORDS + i);
	ipc_barrier();
	if (ipc_load32(slot + IPC_SLOT_GEN) != gen)
		return IPC_TORN;
	if (epoch != ipc_load32(&writer->epoch))
		return IPC_OLD_EPOCH;
	memcpy(payload, buf, 4u * words);
	stamp[0] = epoch;
	stamp[1] = gen;
	return IPC_OK;
}

/* the same with the size of the payload taken from its type */
#define IPC_WRITE(slot, epoch, p)			ipc_slot_write(slot, epoch, p, (unsigned)(sizeof(*(p)) / 4u))
#define IPC_READ(slot, writer, p, stamp)	ipc_slot_read(slot, writer, p, (unsigned)(sizeof(*(p)) / 4u), stamp)

#ifdef __cplusplus
}
#endif

#endif /* G2T_IPC_SLOT_H */
