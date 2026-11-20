/*
 * base/AtomicSlot.h of the firmware  --  the single-writer slot of the application's base/AtomicSlot.h (paper,
 * Sec. II-A, eq. (1): a sequence number around the payload, odd while it is written) on 32-bit words, because the
 * R5 has no 64-bit atomic without a lock.  The same calls, so that the classes that use the slot (HighLevelLink.h)
 * are the application's files unchanged; the firmware finds this header in place of the application's.
 *
 * The tasks of the firmware share one core, and that is the one difference to the application's file: a reader
 * that finds the number odd has interrupted the writer, which cannot finish while the reader runs.  Read() then
 * returns false at once (the application's reader tries again, for a writer on another core), and the reader
 * comes back in its next period.  A reader that the writer interrupts sees another number afterwards and reads
 * once more.
 */
#ifndef G2T_ATOMIC_SLOT_H
#define G2T_ATOMIC_SLOT_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

template <class T>
class CAtomicSlot
{
	static_assert(std::is_trivially_copyable<T>::value, "the payload of a slot must be trivially copyable");

public:
	enum { WORDS = (sizeof(T) + sizeof(uint32_t) - 1) / sizeof(uint32_t), MAX_ATTEMPTS = 4 };

	CAtomicSlot()
	{
		m_uSeq.store(0);
		for (size_t i = 0; i < WORDS; i++)
			m_auWord[i].store(0);
	}
	CAtomicSlot(const CAtomicSlot&) = delete;
	CAtomicSlot& operator=(const CAtomicSlot&) = delete;

	/* the 32-bit words of the slot are lock-free on this processor */
	static bool IsLockFree()
	{
		std::atomic<uint32_t> uProbe(0);
		return uProbe.is_lock_free();
	}

	/* eq. (1) */
	static bool Accept(uint32_t auS0, uint32_t auS1)
	{
		return (auS0 == auS1) && (auS0 > 0) && (auS0 % 2 == 0);
	}

	/* the writer (one task only) */
	void Write(const T& astData)
	{
		uint32_t auBuf[WORDS];
		memset(auBuf, 0, sizeof(auBuf));
		memcpy(auBuf, &astData, sizeof(T));
		const uint32_t uSeq = m_uSeq.load();
		m_uSeq.store(uSeq + 1);					// odd: being written
		for (size_t i = 0; i < WORDS; i++)
			m_auWord[i].store(auBuf[i]);
		m_uSeq.store(uSeq + 2);					// even: published
	}

	/* a reader: true and the newest payload in astData if a copy was accepted; false if nothing has been
	 * published yet, a write is under way, or every attempt was interrupted by one (astData is then unchanged).
	 * *apuCount: the number of payloads published up to the accepted copy. */
	bool Read(T& astData, uint32_t* apuCount = NULL) const
	{
		uint32_t auBuf[WORDS];
		for (int nAttempt = 0; nAttempt < (int)MAX_ATTEMPTS; nAttempt++)
		{
			const uint32_t uS0 = m_uSeq.load();
			if (uS0 == 0 || uS0 % 2 != 0)
				return false;					// never published, or the writer is interrupted in the middle
			for (size_t i = 0; i < WORDS; i++)
				auBuf[i] = m_auWord[i].load();
			const uint32_t uS1 = m_uSeq.load();
			if (Accept(uS0, uS1))
			{
				memcpy(&astData, auBuf, sizeof(T));
				if (apuCount != NULL)
					*apuCount = uS0 / 2;
				return true;
			}
		}
		return false;
	}

	/* the number of payloads published so far */
	uint32_t Count() const { return m_uSeq.load() / 2; }

private:
	std::atomic<uint32_t>	m_uSeq;
	std::atomic<uint32_t>	m_auWord[WORDS];
};

#endif /* G2T_ATOMIC_SLOT_H */
