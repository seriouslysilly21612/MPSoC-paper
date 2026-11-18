/*****************************************************************************
*	Name: r5_link_check.cpp
*	Description: The test of the exchange with the R5 (--arch S1): the APU
*	side (indy7/src/r5link/R5Link.h), the rule by which the tasks follow the
*	period of the R5 (R5Follow.h), the conversion of its counter
*	(LinkClock.h), and the R5's side of the slots in the simulation
*	(indy7/src/sim/SimR5Slots.h). One period at a time, on times that the
*	test makes up; no thread but in part 6, no simulator, no board.
*
*	    bin/r5_link_check [States of part 6, default 100000]
*
*	1. Following: the phase error and its nearest period; the first State
*	   sets the grid at once; then a quarter of the error per period, at most
*	   5 us; an error outside the window for 8 periods in a row sets the grid
*	   again; a clock that drifts, releases that jitter and periods without
*	   a State are followed; a restart of the R5 starts anew.
*	2. The counter: ticks to nanoseconds and back without the overflow of
*	   the product, and the two directions of the clock agree.
*	3. The State slot: no start without a region, with a number of joints
*	   that the slots do not hold, or with a delay outside the period;
*	   nothing before the R5 is up; a period is taken once, and how long
*	   after its release it was read;
*	   periods that no cycle took are counted; a State of another bus (the
*	   number of joints) is not taken; a release time that did not go on is
*	   not followed and the release is estimated; a restart of the R5 is seen
*	   and not read before its first State.
*	4. The Torque slot and the stale-torque rule of the R5: the torque
*	   arrives with its period and its two marks; a period in which the R5
*	   did not have the torque of the period before is counted; the R5
*	   switches the servo off after the limit, whatever the drives report,
*	   and not one period earlier; a new torque starts the count again.
*	5. The Command slot: the drives are told off at the start; servo off and
*	   stop are written once however often they are asked, a servo on every
*	   time; a servo on does not leave to a firmware with another check
*	   value, nor before a State was taken; a restart of either side.
*	6. Between two threads: a writer that publishes State after State and a
*	   reader that takes them as fast as it can, until it has that many: no
*	   State mixes two periods, and no period comes twice or goes back.
*	The test passes if no check failed.
*****************************************************************************/
#include <atomic>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

#include "r5link/R5Link.h"
#include "sim/SimR5Slots.h"

namespace {

int g_nFailed = 0;

void Expect(bool abOk, const char* apcWhat)
{
	if (abOk)
		return;
	g_nFailed++;
	printf("  FAILED: %s\n", apcWhat);
}

const int64_t PERIOD_NS = 1000000, DELAY_NS = 150000;
const uint32_t DOF = 6, CFG_CRC = 0x3896D4ECu;
/* times that passed the counter and came back: exact where the ticks are the nanoseconds. With a counter the test
 * and the link each take its offset, either within half a pair of clock reads of the true one (LinkClock.h) */
const int64_t TIME_TOLERANCE_NS = link_clock::SAME_AS_MONOTONIC ? 0 : link_clock::CTickClock::PAIR_MAX_NS + 100;

bool Near(int64_t anValue, int64_t anExpected)
{
	const int64_t nDiff = anValue - anExpected;
	return nDiff <= TIME_TOLERANCE_NS && nDiff >= -TIME_TOLERANCE_NS;
}

/* a State released at atRelease was read between atBefore and atAfter: its age is the time from the release to the read */
bool IsAgeOfRead(RTTIME atAge, RTTIME atRelease, RTTIME atBefore, RTTIME atAfter)
{
	return (int64_t)atAge >= (int64_t)(atBefore - atRelease) - TIME_TOLERANCE_NS &&
	       (int64_t)atAge <= (int64_t)(atAfter - atRelease) + TIME_TOLERANCE_NS;
}

/* pseudo-random numbers, the same in every run */
uint64_t g_uRandom = 0x9E3779B97F4A7C15ull;
uint64_t Random()
{
	g_uRandom ^= g_uRandom << 13;
	g_uRandom ^= g_uRandom >> 7;
	g_uRandom ^= g_uRandom << 17;
	return g_uRandom;
}

/* 1 */
void Following()
{
	printf("1. following the period of the R5\n");
	R5Follow st;
	st.Start(PERIOD_NS, DELAY_NS);
	const uint64_t uR5 = 5000000000ull;

	// the phase error, to the nearest period
	Expect(st.PhaseError(uR5 + DELAY_NS, uR5) == 0, "no error when the cycle is released the delay after the R5");
	Expect(st.PhaseError(uR5 + DELAY_NS + 30, uR5) == 30 && st.PhaseError(uR5 + DELAY_NS - 30, uR5) == -30,
	       "30 ns late and 30 ns early");
	Expect(st.PhaseError(uR5 + 7 * PERIOD_NS + DELAY_NS + 30, uR5) == 30 &&
	       st.PhaseError(uR5 + DELAY_NS + 30, uR5 + 7 * PERIOD_NS) == 30, "whole periods do not count");
	Expect(st.PhaseError(uR5 + DELAY_NS + PERIOD_NS / 2, uR5) == PERIOD_NS / 2 &&
	       st.PhaseError(uR5 + DELAY_NS + PERIOD_NS / 2 + 1, uR5) == -PERIOD_NS / 2 + 1 &&
	       st.PhaseError(uR5 + DELAY_NS - PERIOD_NS / 2, uR5) == PERIOD_NS / 2 &&
	       st.PhaseError(uR5 + DELAY_NS - PERIOD_NS / 2 + 1, uR5) == -PERIOD_NS / 2 + 1, "the half period");

	// the first State sets the grid at once; then a quarter of the error, at most MAX_STEP_NS
	Expect(!st.bFollowing && st.Step(uR5 + DELAY_NS + 37000, uR5) == -37000 && st.bFollowing && st.nJumps == 1,
	       "the first State: the whole error at once");
	Expect(st.nSamples == 0, "the error before the grid followed is not in the statistics");
	Expect(st.Step(uR5 + DELAY_NS + 400, uR5) == -100 && st.Step(uR5 + DELAY_NS - 400, uR5) == 100,
	       "a quarter of the error");
	Expect(st.Step(uR5 + DELAY_NS + 3, uR5) == 0, "below 4 ns nothing moves");
	Expect(st.Step(uR5 + DELAY_NS + 19999, uR5) == -4999 && st.Step(uR5 + DELAY_NS + 20000, uR5) == -5000 &&
	       st.Step(uR5 + DELAY_NS + 40000, uR5) == -5000 && st.Step(uR5 + DELAY_NS - 40000, uR5) == 5000,
	       "at most 5 us per period");
	Expect(st.nSamples == 7 && st.nErrorMin == -40000 && st.nErrorMax == 40000 && st.nJumps == 1,
	       "the statistics of the errors");
	// the smallest and the largest error are errors that occurred, also when all lie on one side
	st.Start(PERIOD_NS, DELAY_NS);
	(void)st.Step(uR5 + DELAY_NS, uR5);
	(void)st.Step(uR5 + DELAY_NS - 400, uR5);
	(void)st.Step(uR5 + DELAY_NS - 900, uR5);
	Expect(st.nErrorMin == -900 && st.nErrorMax == -400, "errors that were all early");
	st.Start(PERIOD_NS, DELAY_NS);
	(void)st.Step(uR5 + DELAY_NS, uR5);
	(void)st.Step(uR5 + DELAY_NS + 900, uR5);
	(void)st.Step(uR5 + DELAY_NS + 400, uR5);
	Expect(st.nErrorMin == 400 && st.nErrorMax == 900, "errors that were all late");

	// outside the window: 7 periods in a row are followed step by step, the 8th sets the grid again
	st.Start(PERIOD_NS, DELAY_NS);
	(void)st.Step(uR5 + DELAY_NS, uR5);
	bool bStepped = true;
	for (int n = 0; n < 7; n++)
		bStepped = bStepped && st.Step(uR5 + DELAY_NS + 300000, uR5) == -5000;
	Expect(bStepped && st.bFollowing && st.nJumps == 1, "7 periods outside the window: steps");
	Expect(st.Step(uR5 + DELAY_NS + 50000, uR5) == -5000 && st.nOutside == 0, "the edge of the window is inside");
	for (int n = 0; n < 7; n++)
		(void)st.Step(uR5 + DELAY_NS + 50001, uR5);
	Expect(st.nJumps == 1 && st.nOutside == 7, "one period inside starts the count again");
	Expect(st.Step(uR5 + DELAY_NS - 300000, uR5) == 300000 && st.nJumps == 2 && st.bFollowing && st.nOutside == 0,
	       "the 8th period outside: the whole error at once");

	// a restart of the R5: its next State sets the grid at once
	st.Restart();
	Expect(!st.bFollowing && st.Step(uR5 + DELAY_NS + 1234, uR5) == -1234 && st.nJumps == 3, "after a restart");

	// a clock that drifts, releases that jitter, periods without a State: the grid of the tasks, moved by what
	// Step() returns, stays the delay behind the R5
	struct { double dDriftPpm; int nJitterNs; int nGapEvery; int64_t nBound; const char* pcWhat; } astCase[] = {
		{ 200.0, 0, 0, 1000, "the R5 200 ppm slow: within 1 us" },
		{ -200.0, 0, 0, 1000, "the R5 200 ppm fast: within 1 us" },
		{ 50.0, 3000, 0, 7000, "releases that jitter by 3 us: within 7 us" },
		{ 200.0, 0, 20, 8000, "200 ppm with 10 of every 20 periods without a State: within 8 us" },
	};
	for (const auto& stC : astCase)
	{
		st.Start(PERIOD_NS, DELAY_NS);
		const double dR5Period = (double)PERIOD_NS * (1.0 + stC.dDriftPpm * 1e-6);
		int64_t nShift = 0, nWorst = 0;
		for (int k = 1; k <= 20000; k++)
		{
			const int64_t nJitter = stC.nJitterNs > 0 ? (int64_t)(Random() % (uint64_t)(2 * stC.nJitterNs + 1)) -
			                                            stC.nJitterNs : 0;
			const uint64_t uRelease = uR5 + (uint64_t)((double)k * dR5Period) + (uint64_t)(100000 + nJitter);
			const uint64_t uPlanned = uR5 + (uint64_t)((int64_t)k * PERIOD_NS + 777777 + nShift);
			if (stC.nGapEvery > 0 && k % stC.nGapEvery >= stC.nGapEvery / 2)
				continue;
			const int64_t nError = st.PhaseError(uPlanned, uRelease);
			nShift += st.Step(uPlanned, uRelease);
			if (k > 100 && (nError > nWorst || -nError > nWorst))
				nWorst = nError > 0 ? nError : -nError;
		}
		Expect(nWorst <= stC.nBound && st.nJumps == 1, stC.pcWhat);
	}
}

/* 2 */
void Counter()
{
	printf("2. the counter\n");
	const uint64_t auHz[] = { 1000000000ull, 99990005ull, 100000000ull, 62500000ull, 24000000ull, 33333333ull };
	bool bNs = true, bTicks = true, bBack = true;
	for (uint64_t uHz : auHz)
		for (int n = 0; n < 20000; n++)
		{
			// up to 2^62 ns: a century and more
			const uint64_t uNs = Random() >> 2;
			const uint64_t uTicks = (uint64_t)((unsigned __int128)uNs * uHz / 1000000000ull);
			bTicks = bTicks && link_clock::TicksOfNs(uNs, uHz) == uTicks;
			bNs = bNs && link_clock::NsOfTicks(uTicks, uHz) == (uint64_t)((unsigned __int128)uTicks * 1000000000ull / uHz);
			// a time that went to ticks and back lies less than a tick before itself
			const uint64_t uBack = link_clock::NsOfTicks(uTicks, uHz);
			bBack = bBack && uBack <= uNs && uNs - uBack <= 1000000000ull / uHz + 1;
		}
	Expect(bNs, "ticks to nanoseconds, as the 128-bit product gives them");
	Expect(bTicks, "nanoseconds to ticks, as the 128-bit product gives them");
	Expect(bBack, "to ticks and back: less than a tick lost");

	link_clock::CTickClock cClock;
	bool bSampled = false;
	for (int nTry = 0; nTry < 1000 && !bSampled; nTry++)
		bSampled = cClock.Sample();
	Expect(bSampled && cClock.IsValid(), "an offset of the counter is taken");
	const RTTIME tNow = read_timer();
	Expect(Near((int64_t)cClock.MonotonicOf(cClock.TicksOf(tNow)), (int64_t)tNow), "a time on the counter and back");
	const RTTIME tOfCounter = cClock.MonotonicOf(link_clock::Ticks());
	const RTTIME tAfter = read_timer();
	Expect(tOfCounter + 1000000 > tNow && tOfCounter < tAfter + 1000000, "the counter now is the clock now, within 1 ms");
}

/* the two sides on one region, one period at a time */
struct Bench
{
	alignas(64) volatile uint32_t auRegion[IPC_OFF_END / 4];
	CSimR5Slots cR5;
	CSimR5Slots::Drives stDrives;
	CR5Link cApu;
	link_clock::CTickClock cClock;
	RTTIME tGrid;			// the release of period 0 of the R5 on the clock of the tasks
	uint32_t uPeriod;		// the last period of the R5 that ran

	Bench()
	{
		for (size_t i = 0; i < sizeof(auRegion) / sizeof(auRegion[0]); i++)
			auRegion[i] = 0u;
		memset(&stDrives, 0, sizeof(stDrives));
		stDrives.uDof = DOF;
		SetStatus(0x0031);
		for (int nTry = 0; nTry < 1000 && !cClock.Sample(); nTry++)
			;
		tGrid = read_timer() / (RTTIME)PERIOD_NS * (RTTIME)PERIOD_NS - 60000 * (RTTIME)PERIOD_NS;
		uPeriod = 0;
	}
	void SetStatus(uint16_t ausStatus)
	{
		for (unsigned i = 0; i < DOF; i++)
			stDrives.ausStatus[i] = ausStatus;
	}
	RTTIME ReleaseOf(uint32_t auPeriod) const { return tGrid + (RTTIME)auPeriod * (RTTIME)PERIOD_NS; }
	void StartR5(uint32_t auCrc = CFG_CRC, int anStaleLimit = 10) { (void)cR5.Start(auRegion, auCrc, anStaleLimit); }
	bool StartApu(CR5Link& acApu)
	{
		acApu.UseRegion(auRegion, "the test");
		return acApu.Start(DOF, CFG_CRC, (RTTIME)PERIOD_NS, (RTTIME)DELAY_NS) ? true : false;
	}
	/* the next period of the R5: its measurement tells the period; true: the stale rule switched the servo off */
	bool R5Period(int64_t anReleaseErrorNs = 0)
	{
		uPeriod++;
		for (unsigned i = 0; i < DOF; i++)
		{
			stDrives.adQ[i] = 0.001 * (double)uPeriod + (double)i;
			stDrives.adQd[i] = -0.5 * (double)uPeriod;
			stDrives.adTau[i] = 2.0 * (double)i - (double)uPeriod;
		}
		const bool bStopped = cR5.TakeApuSlots(stDrives);
		cR5.PublishState(cClock.TicksOf(ReleaseOf(uPeriod) + (RTTIME)anReleaseErrorNs), uPeriod, stDrives, 0);
		return bStopped;
	}
	/* the cycle of the application for the last period of the R5, released anLateNs after the delay */
	bool ApuReceive(CR5Link& acApu, int64_t anLateNs = 0)
	{
		return acApu.Receive(ReleaseOf(uPeriod) + (RTTIME)(DELAY_NS + anLateNs)) ? true : false;
	}
	/* the Torque slot as the APU side left it */
	ipc_torque_t TorqueInSlot()
	{
		ipc_torque_t stTorque;
		uint32_t auStamp[2];
		memset(&stTorque, 0xFF, sizeof(stTorque));
		(void)IPC_READ(auRegion + IPC_OFF_TORQUE / 4, &((volatile ipc_head_t*)auRegion)->apu, &stTorque, auStamp);
		return stTorque;
	}
	void ApuTorque(CR5Link& acApu, bool abValid = true, bool abEnabled = true)
	{
		double adTau[DOF];
		for (unsigned i = 0; i < DOF; i++)
			adTau[i] = 0.25 * (double)uPeriod + (double)i;
		acApu.SendTorque(acApu.State().period, abValid, abEnabled, adTau, DOF);
	}
};

/* 3 */
void StateSlot()
{
	printf("3. the State slot\n");
	static Bench b;
	{
		// what the configuration gives is checked before anything is written
		static CR5Link s_cRefused;
		Expect(!s_cRefused.Start(DOF, CFG_CRC, (RTTIME)PERIOD_NS, (RTTIME)DELAY_NS), "no start without a region");
		s_cRefused.UseRegion(b.auRegion, "the test");
		Expect(!s_cRefused.Start(0, CFG_CRC, (RTTIME)PERIOD_NS, (RTTIME)DELAY_NS) &&
		       !s_cRefused.Start(IPC_MAX_DOF + 1, CFG_CRC, (RTTIME)PERIOD_NS, (RTTIME)DELAY_NS),
		       "nor with no joint, nor with more than the slots hold");
		Expect(!s_cRefused.Start(DOF, CFG_CRC, 0, 0) &&
		       !s_cRefused.Start(DOF, CFG_CRC, (RTTIME)PERIOD_NS, (RTTIME)PERIOD_NS), "nor with a delay of a whole period");
		Expect(ipc_load32(&((volatile ipc_head_t*)b.auRegion)->apu.magic) == 0u, "and the region is as it was");
		static Bench s_stMost;
		static CR5Link s_cMost;
		s_cMost.UseRegion(s_stMost.auRegion, "the test");
		Expect(s_cMost.Start(IPC_MAX_DOF, CFG_CRC, (RTTIME)PERIOD_NS, (RTTIME)PERIOD_NS - 1),
		       "as many joints as the slots hold, and a delay just inside the period, start");
	}
	Expect(b.StartApu(b.cApu), "the APU side starts on a region that nothing wrote yet");
	Expect(!b.ApuReceive(b.cApu) && b.cApu.GetCounts().anReadFailed[IPC_NO_PEER] == 1 &&
	       b.cApu.GetCounts().nNoState == 1 && b.cApu.GetCounts().nNoStateFollowing == 0 && !b.cApu.IsFollowing(),
	       "nothing before the R5 is up");
	b.StartR5();
	Expect(!b.ApuReceive(b.cApu) && b.cApu.GetCounts().anReadFailed[IPC_BUSY] == 1,
	       "nothing before the first State of the R5");

	// the first State: taken, with its release, and the grid set at once
	const CR5Link::Counts& c = b.cApu.GetCounts();
	(void)b.R5Period();
	RTTIME tBefore = read_timer();
	const bool bFirst = b.ApuReceive(b.cApu, 37000);
	RTTIME tAfter = read_timer();
	const RTTIME tFirstAge = c.tAgeMax;
	Expect(c.nAges == 1 && c.tAgeMin == tFirstAge && IsAgeOfRead(tFirstAge, b.ReleaseOf(1), tBefore, tAfter),
	       "how long after its release the State was read");
	Expect(bFirst && b.cApu.State().period == 1 && b.cApu.State().dof == DOF &&
	       b.cApu.State().q[5] == 0.001 + 5.0 && b.cApu.State().qd[0] == -0.5 && b.cApu.State().tau[3] == 5.0 &&
	       b.cApu.State().cfg_crc == CFG_CRC && (b.cApu.State().flags & IPC_STATE_OP) &&
	       !(b.cApu.State().flags & IPC_STATE_SERVO_ON), "the first State, as the R5 wrote it");
	Expect(Near((int64_t)b.cApu.StateRelease(), (int64_t)b.ReleaseOf(1)), "its release on the clock of the tasks");
	Expect(Near(b.cApu.GridShift(), -37000) && b.cApu.IsFollowing(), "the grid set at once");
	Expect(!b.ApuReceive(b.cApu) && b.cApu.GridShift() == 0 && b.cApu.GetCounts().nNoState == 3 &&
	       b.cApu.GetCounts().nNoStateFollowing == 1 && b.cApu.GetCounts().nStates == 1,
	       "the same period is not taken twice, and the grid stays");

	// the next periods: followed; periods that no cycle took
	(void)b.R5Period();
	tBefore = read_timer();
	const bool bSecond = b.ApuReceive(b.cApu, 400);
	tAfter = read_timer();
	Expect(bSecond && b.cApu.State().period == 2 && Near(b.cApu.GridShift(), -100) &&
	       b.cApu.GetCounts().nPeriodsMissed == 0, "the next period, a quarter of the error");
	// its release is a period later, so it was read sooner after it unless the host stopped the test for 1 ms
	const RTTIME tSecondAge = (c.tAgeMax == tFirstAge) ? c.tAgeMin : c.tAgeMax;
	Expect(c.nAges == 2 && (c.tAgeMin == tFirstAge || c.tAgeMax == tFirstAge) && c.tAgeMin <= c.tAgeMax &&
	       IsAgeOfRead(tSecondAge, b.ReleaseOf(2), tBefore, tAfter) &&
	       c.dAgeSumNs == (double)tFirstAge + (double)tSecondAge, "the shortest, the longest and the sum of two");
	(void)b.R5Period();
	(void)b.R5Period();
	(void)b.R5Period();
	Expect(b.ApuReceive(b.cApu) && b.cApu.State().period == 5 && b.cApu.GetCounts().nPeriodsMissed == 2 &&
	       Near((int64_t)b.cApu.StateRelease(), (int64_t)b.ReleaseOf(5)) && b.cApu.GetCounts().nBadTime == 0,
	       "two periods that no cycle took");

	// a State with another number of joints is not taken
	b.stDrives.uDof = 4;
	(void)b.R5Period();
	Expect(!b.ApuReceive(b.cApu) && b.cApu.GetCounts().nBadState == 1 && b.cApu.State().period == 5,
	       "a State of a bus with 4 drives is not taken");
	b.stDrives.uDof = DOF;

	// a release time that does not go on by the periods that passed (a firmware without the counter leaves it
	// where it was): the State is taken, the grid is not moved, the release is estimated
	(void)b.R5Period(-2 * PERIOD_NS);
	Expect(b.ApuReceive(b.cApu, 20000) && b.cApu.State().period == 7 && b.cApu.GetCounts().nBadTime == 1 &&
	       b.cApu.GridShift() == 0, "a release time that stayed: the State is taken, the grid is not moved");
	Expect(Near((int64_t)b.cApu.StateRelease(), (int64_t)b.ReleaseOf(7)), "and the release is the last one and 2 periods");
	(void)b.R5Period(-3 * PERIOD_NS);
	Expect(b.ApuReceive(b.cApu) && b.cApu.GetCounts().nBadTime == 2 &&
	       Near((int64_t)b.cApu.StateRelease(), (int64_t)b.ReleaseOf(8)), "it stayed again: one period more");
	// the time of the R5 is back: its first release has nothing right before it, the next one is followed
	(void)b.R5Period();
	Expect(b.ApuReceive(b.cApu, 20000) && b.cApu.GetCounts().nBadTime == 3 && b.cApu.GridShift() == 0,
	       "the first release after those is not followed yet");
	(void)b.R5Period();
	Expect(b.ApuReceive(b.cApu, 400) && b.cApu.GetCounts().nBadTime == 3 && Near(b.cApu.GridShift(), -100) &&
	       Near((int64_t)b.cApu.StateRelease(), (int64_t)b.ReleaseOf(10)), "the next one is");
	// half a period is the limit
	(void)b.R5Period((int64_t)PERIOD_NS / 2 - 5000);
	Expect(b.ApuReceive(b.cApu) && b.cApu.GetCounts().nBadTime == 3, "a release 495 us late is taken as it is");
	(void)b.R5Period();
	Expect(b.ApuReceive(b.cApu) && b.cApu.GetCounts().nBadTime == 3, "and the one after it, 495 us early by it");
	(void)b.R5Period((int64_t)PERIOD_NS / 2 + 5000);
	Expect(b.ApuReceive(b.cApu) && b.cApu.GetCounts().nBadTime == 4, "a release 505 us late is not");
	(void)b.R5Period((int64_t)PERIOD_NS / 2 + 5000);
	(void)b.ApuReceive(b.cApu);
	(void)b.R5Period();
	Expect(b.ApuReceive(b.cApu) && b.cApu.GetCounts().nBadTime == 5, "nor one 505 us early");

	// a restart of the R5: not read before its first State, then seen, and the grid set again
	const long nJumps = b.cApu.GetFollow().nJumps;
	b.StartR5();
	const long nBusy = b.cApu.GetCounts().anReadFailed[IPC_BUSY];
	Expect(!b.ApuReceive(b.cApu) && b.cApu.GetCounts().anReadFailed[IPC_BUSY] == nBusy + 1 &&
	       b.cApu.GetCounts().nR5Restarts == 0, "what the R5 wrote before it restarted is not read");
	b.uPeriod = 0;
	b.tGrid += 123456;
	(void)b.R5Period();
	Expect(b.ApuReceive(b.cApu, -60000) && b.cApu.State().period == 1 && b.cApu.GetCounts().nR5Restarts == 1 &&
	       b.cApu.GetFollow().nJumps == nJumps + 1 && Near(b.cApu.GridShift(), 60000) &&
	       Near((int64_t)b.cApu.StateRelease(), (int64_t)b.ReleaseOf(1)), "the restart is seen, and the grid set again");
	Expect(b.cApu.GetCounts().nBadTime == 5, "its first release time needs nothing before it");
}

/* 4 */
void TorqueSlot()
{
	printf("4. the Torque slot and the stale-torque rule of the R5\n");
	static Bench b;
	const int nLimit = 10;
	b.StartR5(CFG_CRC, nLimit);
	(void)b.StartApu(b.cApu);
	(void)b.R5Period();
	(void)b.ApuReceive(b.cApu);
	Expect(!b.cR5.IsTorqueUsable() && b.cApu.State().torque_period == 0, "no torque before the first one is written");

	// the torque arrives with its period and its marks
	b.ApuTorque(b.cApu);
	(void)b.R5Period();
	Expect(b.cR5.IsTorqueUsable() && b.cR5.Torque()[0] == 0.25 && b.cR5.Torque()[5] == 5.25,
	       "the torque, valid and enabled");
	Expect(b.ApuReceive(b.cApu) && b.cApu.State().torque_period == 1 && b.cApu.GetCounts().nTorqueLate == 0 &&
	       b.cApu.GetCounts().nTorquesWritten == 1, "the State tells which torque the R5 had");
	Expect(b.TorqueInSlot().flags == (IPC_TORQUE_VALID | IPC_TORQUE_ENABLED) && b.TorqueInSlot().period == 1,
	       "the slot has both marks and the period of the State");
	b.ApuTorque(b.cApu, true, false);
	(void)b.R5Period();
	Expect(!b.cR5.IsTorqueUsable() && b.TorqueInSlot().flags == IPC_TORQUE_VALID,
	       "valid but the controller off: not for the drives");
	(void)b.ApuReceive(b.cApu);
	b.ApuTorque(b.cApu, false, true);
	(void)b.R5Period();
	Expect(!b.cR5.IsTorqueUsable() && b.TorqueInSlot().flags == IPC_TORQUE_ENABLED, "not valid: not for the drives");
	(void)b.ApuReceive(b.cApu);
	Expect(b.cApu.GetCounts().nTorqueLate == 0, "three torques in time");

	// a torque that was not there in time
	(void)b.R5Period();
	Expect(b.ApuReceive(b.cApu) && b.cApu.GetCounts().nTorqueLate == 1, "a period in which the R5 did not have it");
	b.ApuTorque(b.cApu);
	(void)b.R5Period();
	Expect(b.ApuReceive(b.cApu) && b.cApu.GetCounts().nTorqueLate == 1, "the next one in time");
	b.ApuTorque(b.cApu);
	(void)b.R5Period();
	(void)b.R5Period();
	Expect(b.ApuReceive(b.cApu) && b.cApu.GetCounts().nTorqueLate == 1 && b.cApu.GetCounts().nPeriodsMissed == 1,
	       "a period that no cycle took is not a late torque");
	(void)b.R5Period();
	(void)b.R5Period();
	Expect(b.ApuReceive(b.cApu) && b.cApu.GetCounts().nTorqueLate == 2 && b.cApu.GetCounts().nPeriodsMissed == 2,
	       "but a torque that is not there two periods later is");

	// the stale-torque rule: the servo asked for, every drive enabled
	b.cApu.RequestCommand(IPC_CMD_SERVO_ON);
	Expect(b.cApu.SendCommand(), "servo on leaves");
	b.SetStatus(0x0027);
	b.ApuTorque(b.cApu);
	(void)b.R5Period();
	(void)b.ApuReceive(b.cApu);
	const uint32_t uStale0 = b.cApu.State().n_torque_stale;
	Expect(b.cR5.IsServoAsked() && (b.cApu.State().flags & IPC_STATE_SERVO_ON), "the servo is asked for and on");
	bool bStopped = false;
	for (int n = 0; n < 50; n++)
	{
		b.ApuTorque(b.cApu);
		bStopped = bStopped || b.R5Period();
		(void)b.ApuReceive(b.cApu);
	}
	Expect(!bStopped && b.cR5.IsServoAsked() && b.cApu.State().n_torque_stale == uStale0,
	       "a torque in every period: the servo stays on");
	for (int n = 0; n < nLimit - 1; n++)
		bStopped = bStopped || b.R5Period();
	Expect(!bStopped && b.cR5.IsServoAsked() && b.cR5.StaleRun() == nLimit - 1, "9 periods without a torque: still on");
	b.ApuTorque(b.cApu);
	bStopped = b.R5Period();
	Expect(!bStopped && b.cR5.StaleRun() == 0, "a new torque starts the count again");
	for (int n = 0; n < nLimit - 1; n++)
		bStopped = bStopped || b.R5Period();
	Expect(!bStopped && b.cR5.IsServoAsked(), "again 9 periods: still on");
	Expect(b.R5Period() && !b.cR5.IsServoAsked(), "the 10th period without a torque: servo off");
	(void)b.ApuReceive(b.cApu);
	Expect(b.cApu.State().n_torque_stale == uStale0 + 2u * (uint32_t)nLimit - 1u,
	       "the R5 counted those periods, the drives being on");
	Expect(!b.R5Period() && !b.cR5.IsServoAsked(), "and it stays off");
	b.ApuTorque(b.cApu);
	Expect(!b.R5Period() && !b.cR5.IsServoAsked(), "a torque alone does not switch it on again");
	// the periods without a torque while the servo is not asked for do not count toward the next stop
	for (int n = 0; n < 2 * nLimit; n++)
		(void)b.R5Period();
	b.cApu.RequestCommand(IPC_CMD_SERVO_ON);
	(void)b.cApu.SendCommand();
	Expect(!b.R5Period() && b.cR5.IsServoAsked() && b.cR5.StaleRun() == 1,
	       "asked for again without a torque: the count starts with this period");
	bStopped = false;
	for (int n = 0; n < nLimit - 2; n++)
		bStopped = bStopped || b.R5Period();
	Expect(!bStopped && b.R5Period() && !b.cR5.IsServoAsked(), "and the servo goes off after 10 periods of it");

	// the stop does not wait for every drive: one that is not enabled
	b.cApu.RequestCommand(IPC_CMD_SERVO_ON);
	(void)b.cApu.SendCommand();
	b.stDrives.ausStatus[2] = 0x0040;
	b.ApuTorque(b.cApu);
	(void)b.R5Period();
	(void)b.ApuReceive(b.cApu);
	const uint32_t uStale1 = b.cApu.State().n_torque_stale;
	Expect(b.cR5.IsServoAsked() && !(b.cApu.State().flags & IPC_STATE_SERVO_ON) && b.cApu.State().not_ready == 4u,
	       "asked for again, one drive in Switch on disabled");
	bStopped = false;
	for (int n = 0; n < nLimit - 1; n++)
		bStopped = bStopped || b.R5Period();
	Expect(!bStopped && b.R5Period() && !b.cR5.IsServoAsked(), "the 10th period: servo off, though a drive is not enabled");
	(void)b.ApuReceive(b.cApu);
	Expect(b.cApu.State().n_torque_stale == uStale1, "those periods are not in the count of the State slot");
}

/* 5 */
void CommandSlot()
{
	printf("5. the Command slot\n");
	static Bench b;
	b.StartR5();
	(void)b.StartApu(b.cApu);

	// before a State: no servo on (not even to a configuration whose check value is that of an empty slot);
	// the first command tells the drives off
	Expect(!b.cApu.IsServoOnAllowed(), "no servo on before a State was taken");
	{
		static Bench z;
		static CR5Link s_cZero;
		s_cZero.UseRegion(z.auRegion, "the test");
		(void)s_cZero.Start(DOF, 0u, (RTTIME)PERIOD_NS, (RTTIME)DELAY_NS);
		s_cZero.RequestCommand(IPC_CMD_SERVO_ON);
		Expect(!s_cZero.IsServoOnAllowed() && !s_cZero.SendCommand(), "nor with a check value of 0");
	}
	Expect(b.cApu.SendCommand() && b.cApu.GetCounts().nCommands == 1, "the start tells the drives off");
	(void)b.R5Period();
	(void)b.ApuReceive(b.cApu);
	Expect(b.cApu.State().cmd_done == 1 && !b.cR5.IsServoAsked(), "the R5 carried it out");
	Expect(b.cApu.SendCommand() && b.cApu.GetCounts().nCommands == 1, "nothing asked: nothing written");

	// servo on to a firmware of this configuration; every servo on is written
	Expect(b.cApu.IsServoOnAllowed(), "the check value of the R5 is that of the configuration");
	b.cApu.RequestCommand(IPC_CMD_SERVO_ON);
	Expect(b.cApu.SendCommand() && b.cApu.GetCounts().nCommands == 2, "servo on is written");
	(void)b.R5Period();
	(void)b.ApuReceive(b.cApu);
	Expect(b.cR5.IsServoAsked() && b.cApu.State().cmd_done == 2, "the R5 has the servo on asked for");
	(void)b.R5Period();
	Expect(b.cR5.IsServoAsked(), "a command counts once");
	Expect(b.cApu.SendCommand() && b.cApu.GetCounts().nCommands == 2, "a servo on is not written again by itself");
	b.cApu.RequestCommand(IPC_CMD_SERVO_ON);
	Expect(b.cApu.SendCommand() && b.cApu.GetCounts().nCommands == 3, "a servo on again is written again");

	// servo off and stop: once, however often they are asked for
	for (int n = 0; n < 5; n++)
	{
		b.cApu.RequestCommand(IPC_CMD_SERVO_OFF);
		(void)b.cApu.SendCommand();
		(void)b.R5Period();
	}
	Expect(b.cApu.GetCounts().nCommands == 4 && !b.cR5.IsServoAsked(), "servo off asked 5 times: written once");
	b.cApu.RequestCommand(IPC_CMD_SERVO_OFF, TRUE);
	Expect(b.cApu.SendCommand() && b.cApu.GetCounts().nCommands == 5, "asked for again on purpose: written");
	b.cApu.RequestCommand(IPC_CMD_SERVO_ON);
	(void)b.cApu.SendCommand();
	(void)b.R5Period();
	Expect(b.cR5.IsServoAsked(), "on again");
	b.cApu.RequestCommand(IPC_CMD_STOP);
	(void)b.cApu.SendCommand();
	b.cApu.RequestCommand(IPC_CMD_STOP);
	(void)b.cApu.SendCommand();
	(void)b.R5Period();
	Expect(!b.cR5.IsServoAsked() && b.cApu.GetCounts().nCommands == 7, "a stop is a servo off for the R5, written once");
	// the last request of a cycle is the one that is written
	b.cApu.RequestCommand(IPC_CMD_SERVO_ON);
	b.cApu.RequestCommand(IPC_CMD_SERVO_OFF);
	(void)b.cApu.SendCommand();
	(void)b.R5Period();
	Expect(!b.cR5.IsServoAsked() && b.cApu.GetCounts().nCommands == 8, "on, then off in one cycle: off");

	// a firmware built from another configuration: the servo on does not leave
	{
		static Bench o;
		o.StartR5(CFG_CRC ^ 1u);
		(void)o.StartApu(o.cApu);
		(void)o.cApu.SendCommand();
		(void)o.R5Period();
		(void)o.ApuReceive(o.cApu);
		const long nWritten = o.cApu.GetCounts().nCommands;
		o.cApu.RequestCommand(IPC_CMD_SERVO_ON);
		Expect(!o.cApu.IsServoOnAllowed() && !o.cApu.SendCommand() && o.cApu.GetCounts().nServoOnRefused == 1 &&
		       o.cApu.GetCounts().nCommands == nWritten, "another check value: the servo on is refused, not written");
		(void)o.R5Period();
		Expect(!o.cR5.IsServoAsked(), "and the R5 does not have it");
		Expect(o.cApu.SendCommand(), "the refused servo on is not sent later");
		(void)o.R5Period();
		Expect(!o.cR5.IsServoAsked() && o.cApu.GetCounts().nCommands == nWritten, "nothing left to send");
		o.cApu.RequestCommand(IPC_CMD_SERVO_OFF, TRUE);
		Expect(o.cApu.SendCommand() && o.cApu.GetCounts().nCommands == nWritten + 1,
		       "servo off needs no check value");
	}

	// the R5 restarts: what the slot holds was sent to its earlier run
	b.cApu.RequestCommand(IPC_CMD_SERVO_ON);
	Expect(b.cApu.SendCommand() && b.cApu.GetCounts().nCommands == 9, "a servo on in the slot");
	b.StartR5();
	b.uPeriod = 0;
	(void)b.R5Period();
	Expect(!b.cR5.IsServoAsked(), "the R5 that restarted does not carry it out");
	(void)b.ApuReceive(b.cApu);
	Expect(b.cApu.State().cmd_done == 0 && b.cApu.GetCounts().nR5Restarts == 1, "and reports no command carried out");
	b.cApu.RequestCommand(IPC_CMD_SERVO_ON);
	(void)b.cApu.SendCommand();
	(void)b.R5Period();
	(void)b.ApuReceive(b.cApu);
	Expect(b.cR5.IsServoAsked() && b.cApu.State().cmd_done == 10, "the next servo on it does carry out");

	// the application restarts: its slots are not read before it wrote them
	b.SetStatus(0x0027);
	b.ApuTorque(b.cApu);
	(void)b.R5Period();
	Expect(b.cR5.IsServoAsked() && b.cR5.IsTorqueUsable(), "servo on, with a torque");
	static CR5Link s_cApuAgain;
	(void)b.StartApu(s_cApuAgain);
	bool bStopped = false;
	int nPeriods = 0;
	while (!bStopped && nPeriods < 20)
	{
		bStopped = b.R5Period();
		nPeriods++;
	}
	Expect(bStopped && nPeriods == 10, "the application gone: no torque is new, servo off after 10 periods");

	// and its command numbers count anew: number 1 of the new run is not number 1 of the run before
	{
		static Bench a;
		static CR5Link s_cSecondRun;
		a.StartR5();
		(void)a.StartApu(a.cApu);
		(void)a.cApu.SendCommand();
		(void)a.R5Period();
		(void)a.ApuReceive(a.cApu);
		Expect(a.cApu.State().cmd_done == 1 && !a.cR5.IsServoAsked(), "the first run: its command 1, servo off");
		(void)a.StartApu(s_cSecondRun);
		(void)a.R5Period();
		(void)a.ApuReceive(s_cSecondRun);
		s_cSecondRun.RequestCommand(IPC_CMD_SERVO_ON);
		Expect(s_cSecondRun.SendCommand() && s_cSecondRun.GetCounts().nCommands == 1, "the second run: its command 1");
		a.ApuTorque(s_cSecondRun);
		(void)a.R5Period();
		(void)a.ApuReceive(s_cSecondRun);
		Expect(a.cR5.IsServoAsked() && s_cSecondRun.State().cmd_done == 1 &&
		       s_cSecondRun.State().torque_period == a.uPeriod - 1, "the R5 carries it out: servo on");
	}
}

/* 6 */
void BetweenTwoThreads(long anStates)
{
	printf("6. between two threads, %ld States\n", anStates);
	static Bench b;
	b.StartR5();
	(void)b.StartApu(b.cApu);
	(void)b.R5Period();
	std::atomic<bool> bStop{false};
	std::thread cWriter([&bStop]() {
		while (!bStop.load(std::memory_order_relaxed))
			(void)b.R5Period();
	});
	long nReads = 0, nTaken = 0, nMixed = 0, nNotNew = 0;
	uint32_t uLast = 0;
	// the host may start the writer late or stop it for a while: the States count, not the reads
	const RTTIME tGiveUp = read_timer() + 20000000000ull;
	while (nTaken < anStates && ((++nReads & 0xFFF) != 0 || read_timer() < tGiveUp))
	{
		// the planned release is any time: this part does not look at the grid
		if (!b.cApu.Receive(b.tGrid))
			continue;
		const ipc_state_t& s = b.cApu.State();
		nTaken++;
		if (s.period <= uLast)
			nNotNew++;
		uLast = s.period;
		for (unsigned i = 0; i < DOF; i++)
			if (s.q[i] != 0.001 * (double)s.period + (double)i || s.qd[i] != -0.5 * (double)s.period ||
			    s.tau[i] != 2.0 * (double)i - (double)s.period)
				nMixed++;
	}
	bStop.store(true);
	cWriter.join();
	const CR5Link::Counts& c = b.cApu.GetCounts();
	printf("   %ld reads: %ld States taken, %ld reads while the slot was written, %ld torn\n", nReads, nTaken,
	       c.anReadFailed[IPC_BUSY], c.anReadFailed[IPC_TORN]);
	Expect(nTaken == anStates, "the States were taken within 20 s");
	Expect(nMixed == 0, "no State mixes two periods");
	Expect(nNotNew == 0, "no period twice, none back");
	Expect(c.nStates == nTaken && c.nR5Restarts == 0 && c.nBadState == 0, "the counts of the link");
}

}	// namespace

int main(int argc, char** argv)
{
	const long nStates = argc > 1 ? atol(argv[1]) : 100000;
	if (nStates <= 0 || nStates > 2000000000L)
	{
		printf("usage: r5_link_check [States of part 6, default 100000]\n");
		return 2;
	}
	Following();
	Counter();
	StateSlot();
	TorqueSlot();
	CommandSlot();
	BetweenTwoThreads(nStates);
	printf("%s: %d check(s) failed\n", g_nFailed == 0 ? "PASS" : "FAIL", g_nFailed);
	return g_nFailed == 0 ? 0 : 1;
}
