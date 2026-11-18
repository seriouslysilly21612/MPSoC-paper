/*****************************************************************************
*	Name: R5Follow.h
*	Description: Following the period of the R5 (D-03). With the EtherCAT task
*	on the R5 its period is the period of the loop: the R5 releases period k
*	at r_k, receives, writes the State slot and sends. The tasks here are
*	released a fixed delay after r_k (cfg WAKE_DELAY_US), when that State is
*	written, and their torque must be in the Torque slot before r_k+1. They
*	sleep on a clock of their own, so the release grid is moved a little in
*	every period (CRtTask::ShiftGrid):
*
*	    phase error  e = (P - r) - delay
*	                 P: planned release of the cycle that read the State of
*	                 the period released at r; e is taken to the nearest
*	                 period, (-T/2, T/2]
*	    following    the grid moves by -e/4, at most MAX_STEP_NS per period
*	    not yet      the grid moves by -e at once: at the start, after a
*	                 restart of the R5, and when e stayed outside
*	                 LOCK_WINDOW_NS for UNLOCK_AFTER cycles in a row
*
*	A cycle without a new State changes nothing: the grid runs on. The
*	planned release counts, not the time the task woke, so the latency of
*	Linux does not move the grid; a cycle that ran late may have read the
*	State of the next period, which the nearest period covers.
*	No clock is read here: the caller gives both times on one clock.
*****************************************************************************/
#ifndef G2T_R5_FOLLOW_H
#define G2T_R5_FOLLOW_H

#include <cstdint>

struct R5Follow
{
	enum { GAIN_DIVISOR = 4, MAX_STEP_NS = 5000, LOCK_WINDOW_NS = 50000, UNLOCK_AFTER = 8 };

	int64_t	nPeriodNs;		// T
	int64_t	nDelayNs;		// the release of a cycle after that of the R5's period
	bool	bFollowing;
	int		nOutside;		// cycles in a row with the phase error outside the window
	long	nJumps;			// times the grid was set at once
	/* the phase error of the cycles that followed [ns] */
	long	nSamples;
	int64_t	nErrorMin, nErrorMax;
	double	dErrorSum;

	void Start(int64_t anPeriodNs, int64_t anDelayNs)
	{
		nPeriodNs = anPeriodNs;
		nDelayNs = anDelayNs;
		nJumps = 0;
		nSamples = 0;
		nErrorMin = nErrorMax = 0;
		dErrorSum = 0.0;
		Restart();
	}
	/* the R5 started again: its periods have a new phase */
	void Restart()
	{
		bFollowing = false;
		nOutside = 0;
	}
	/* e of a cycle [ns]: both times on one clock */
	int64_t PhaseError(uint64_t auPlannedRelease, uint64_t auR5Release) const
	{
		int64_t nError = ((int64_t)(auPlannedRelease - auR5Release) - nDelayNs) % nPeriodNs;
		if (nError > nPeriodNs / 2)
			nError -= nPeriodNs;
		else if (nError <= -nPeriodNs / 2)
			nError += nPeriodNs;
		return nError;
	}
	/* a cycle that read a new State: by how much the grid moves for the releases after it [ns] */
	int64_t Step(uint64_t auPlannedRelease, uint64_t auR5Release)
	{
		const int64_t nError = PhaseError(auPlannedRelease, auR5Release);
		if (bFollowing)
		{
			if (nSamples == 0 || nError < nErrorMin) nErrorMin = nError;
			if (nSamples == 0 || nError > nErrorMax) nErrorMax = nError;
			dErrorSum += (double)nError;
			nSamples++;
			if (nError > LOCK_WINDOW_NS || nError < -LOCK_WINDOW_NS)
			{
				if (++nOutside >= UNLOCK_AFTER)
					bFollowing = false;
			}
			else
				nOutside = 0;
		}
		if (!bFollowing)
		{
			bFollowing = true;
			nOutside = 0;
			nJumps++;
			return -nError;
		}
		const int64_t nStep = -nError / GAIN_DIVISOR, nMax = MAX_STEP_NS;
		return nStep > nMax ? nMax : nStep < -nMax ? -nMax : nStep;
	}
};

#endif // G2T_R5_FOLLOW_H
