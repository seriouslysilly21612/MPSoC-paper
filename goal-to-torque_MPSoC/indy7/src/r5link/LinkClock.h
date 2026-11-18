/*****************************************************************************
*	Name: LinkClock.h
*	Description: The time base that the R5 and the APU share (D-08): the
*	system counter of the chip. The R5 reads it as IOU_SCNTR (ocm_link.c of
*	the firmware), Linux as CNTVCT_EL0, and a State slot carries the release
*	of its period in ticks of it (ipc_layout.h, t_release).
*
*	The tasks of the controller run on CLOCK_MONOTONIC (read_timer). Linux
*	derives that clock from the same counter and slews it (NTP), so the
*	offset between the two is not a constant: CTickClock takes it again
*	with one counter read between two clock reads, and keeps the last
*	offset when the two clock reads lie too far apart (the thread was
*	interrupted between them).
*
*	On a host without that counter (the simulation on a PC) the ticks are
*	the nanoseconds of CLOCK_MONOTONIC themselves, and the offset is 0.
*****************************************************************************/
#ifndef G2T_LINK_CLOCK_H
#define G2T_LINK_CLOCK_H

#include <cstdint>

#include "base/Common.h"

namespace link_clock
{
#if defined(__aarch64__)
	enum { SAME_AS_MONOTONIC = 0 };
	/* the system counter now, and its frequency [ticks/s] */
	inline uint64_t Ticks()
	{
		uint64_t uTicks;
		__asm__ __volatile__("isb\n\tmrs %0, cntvct_el0" : "=r"(uTicks));
		return uTicks;
	}
	inline uint64_t TicksPerSecond()
	{
		uint64_t uHz;
		__asm__ __volatile__("mrs %0, cntfrq_el0" : "=r"(uHz));
		return uHz & 0xFFFFFFFFull;
	}
#else
	enum { SAME_AS_MONOTONIC = 1 };
	inline uint64_t Ticks() { return read_timer(); }
	inline uint64_t TicksPerSecond() { return 1000000000ull; }
#endif

	/* ticks -> nanoseconds and back, rounded down; exact below 2^64 (the product of the whole count would overflow) */
	inline uint64_t NsOfTicks(uint64_t auTicks, uint64_t auTicksPerSecond)
	{
		return auTicks / auTicksPerSecond * 1000000000ull + auTicks % auTicksPerSecond * 1000000000ull / auTicksPerSecond;
	}
	inline uint64_t TicksOfNs(uint64_t auNs, uint64_t auTicksPerSecond)
	{
		return auNs / 1000000000ull * auTicksPerSecond + auNs % 1000000000ull * auTicksPerSecond / 1000000000ull;
	}

	/* a time of the counter on the clock of the tasks */
	class CTickClock
	{
	public:
		enum { PAIR_MAX_NS = 20000 };	// the two clock reads around the counter read lie at most this far apart

		CTickClock() : m_uTicksPerSecond(link_clock::TicksPerSecond()), m_nOffsetNs(0), m_bValid(false) {}

		uint64_t TicksPerSecond() const { return m_uTicksPerSecond; }
		/* the offset now; FALSE: the pair was too wide, and the last offset stays */
		bool Sample()
		{
			if (SAME_AS_MONOTONIC)
			{
				m_bValid = true;
				return true;
			}
			const RTTIME tBefore = read_timer();
			const uint64_t uTicks = link_clock::Ticks();
			const RTTIME tAfter = read_timer();
			if (tAfter - tBefore > (RTTIME)PAIR_MAX_NS)
				return false;
			m_nOffsetNs = (int64_t)(tBefore + (tAfter - tBefore) / 2) - (int64_t)NsOfTicks(uTicks, m_uTicksPerSecond);
			m_bValid = true;
			return true;
		}
		/* an offset has been taken */
		bool IsValid() const { return m_bValid; }
		/* CLOCK_MONOTONIC [ns] of a counter value, and the counter value of a time, with the last offset */
		RTTIME MonotonicOf(uint64_t auTicks) const
		{
			return (RTTIME)((int64_t)NsOfTicks(auTicks, m_uTicksPerSecond) + m_nOffsetNs);
		}
		uint64_t TicksOf(RTTIME atMonotonic) const
		{
			return TicksOfNs((uint64_t)((int64_t)atMonotonic - m_nOffsetNs), m_uTicksPerSecond);
		}

	private:
		uint64_t	m_uTicksPerSecond;
		int64_t		m_nOffsetNs;
		bool		m_bValid;
	};
}

#endif // G2T_LINK_CLOCK_H
