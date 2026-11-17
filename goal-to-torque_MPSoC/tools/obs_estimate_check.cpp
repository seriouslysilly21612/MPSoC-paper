/*****************************************************************************
*	Name: obs_estimate_check.cpp
*	Description: The test of the object estimate of the camera application
*	(indy7/src/obs/ObsEstimate.h). A tick is 1 us.
*
*	    bin/obs_estimate_check
*
*	1. beta = 0: the estimate is, bit for bit, the moving average of the
*	   bridge (GoalFilter.h) on the same samples, with the same restarts
*	   after a gap.
*	2. Another target: the estimate restarts from the first result of the new
*	   target and does not slide over from the old one. A gap of more than
*	   dGapS between two captures restarts it too.
*	3. The object of a period: new after a folded result, the last one
*	   otherwise; a capture that is not after the last one is not folded.
*	4. beta > 0: an object that moves at 0.1 m/s. The velocity is found, a
*	   period with a result in use gets the object at its release, a period
*	   without one keeps what it has.
*	5. The target is lost after dLossS without a folded result.
*	The test passes if no check failed.
*****************************************************************************/
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "GoalFilter.h"
#include "obs/ObsEstimate.h"

namespace {

int g_nFailed = 0;

void Expect(bool abOk, const char* apcWhat)
{
	if (abOk)
		return;
	g_nFailed++;
	printf("  FAILED: %s\n", apcWhat);
}

uint64_t Us(double adS) { return (uint64_t)llround(adS * 1e6); }

CObsEstimate::Params Params(double adBeta)
{
	CObsEstimate::Params st;
	st.dAlpha = 0.4;
	st.dBeta = adBeta;
	st.dGapS = 1.0;
	st.dLossS = 1.0;
	st.dTicksPerS = 1e6;
	return st;
}

void AsTheBridge()
{
	CGoalFilter cFilter;					// its defaults: kappa 0.4, gap 1 s
	CObsEstimate cEst;
	cEst.SetParams(Params(0.0));
	uint32_t uX = 2463534242u;
	double dT = 10.0;
	long nSame = 0, nSamples = 0;
	for (int i = 0; i < 2000; i++)
	{
		uX ^= uX << 13; uX ^= uX >> 17; uX ^= uX << 5;
		dT += (uX % 97u == 0) ? 1.5 : 0.045;			// now and then a gap of more than a second
		double adZ[3], adA[3], adB[3] = { 0.0, 0.0, 0.0 };
		for (int k = 0; k < 3; k++)
		{
			uX ^= uX << 13; uX ^= uX >> 17; uX ^= uX << 5;
			adZ[k] = (k == 0 ? 0.6 : 0.1) + 0.002 * ((double)(uX % 2001u) / 1000.0 - 1.0);
		}
		if (cFilter.Push(true, adZ, dT, adA) != CGoalFilter::eAccepted)
			continue;
		nSamples++;
		const uint64_t uT = Us(dT);
		if (cEst.Fold(adZ, uT, 1, uT + 100000) && cEst.Object(uT + 100000, true, adB) && !memcmp(adA, adB, sizeof(adA)) &&
		    cEst.Restarted() == cFilter.Restarted())
			nSame++;
	}
	Expect(nSamples == 2000 && nSame == nSamples && cEst.Restarts() == cFilter.Restarts() && cEst.Restarts() > 5 &&
	       cEst.Folded() == nSamples, "beta = 0: the moving average of the bridge, bit for bit, with its restarts");
}

void AnotherTarget()
{
	const double adA[3] = { 0.5, 0.0, 0.1 }, adB[3] = { 0.5, 0.3, 0.1 };
	double adP[3] = { 0.0, 0.0, 0.0 };
	CObsEstimate cEst;
	cEst.SetParams(Params(0.0));
	bool bOk = true;
	for (int i = 0; i < 20; i++)
		bOk = bOk && cEst.Fold(adA, Us(1.0 + 0.045 * i), 7, Us(1.1 + 0.045 * i)) && cEst.Object(Us(1.1 + 0.045 * i), true, adP);
	Expect(bOk && adP[1] == 0.0 && cEst.Restarts() == 1, "twenty results of one target");
	bOk = cEst.Fold(adB, Us(2.0), 8, Us(2.1)) && cEst.Object(Us(2.1), true, adP);
	Expect(bOk && cEst.Restarted() && cEst.Restarts() == 2 && cEst.Target() == 8 && !memcmp(adP, adB, sizeof(adP)),
	       "the first result of another target: the estimate is that result");
	bOk = cEst.Fold(adA, Us(2.045), 8, Us(2.145)) && cEst.Object(Us(2.145), true, adP);
	Expect(bOk && !cEst.Restarted() && fabs(adP[1] - 0.18) < 1e-12,
	       "the same target 300 mm away: no restart, the estimate goes 40 % of the way");
}

void Gap()
{
	const double adA[3] = { 0.5, 0.0, 0.1 }, adB[3] = { 0.6, 0.0, 0.1 };
	double adP[3] = { 0.0, 0.0, 0.0 };
	CObsEstimate cEst;
	cEst.SetParams(Params(0.0));
	cEst.Fold(adA, Us(1.0), 1, Us(1.1));
	Expect(cEst.Fold(adB, Us(2.0), 1, Us(2.1)) && !cEst.Restarted() && cEst.Object(Us(2.1), true, adP) &&
	       fabs(adP[0] - 0.54) < 1e-12, "a gap of dGapS between two captures: no restart");
	Expect(cEst.Fold(adA, Us(3.0) + 1, 1, Us(3.1)) && cEst.Restarted() && cEst.Object(Us(3.1), true, adP) && adP[0] == 0.5,
	       "a longer gap: the estimate restarts from the result");
}

void ObjectOfAPeriod()
{
	const double adA[3] = { 0.5, 0.0, 0.1 }, adB[3] = { 0.6, 0.0, 0.1 };
	double adP[3] = { 9.0, 9.0, 9.0 };
	CObsEstimate cEst;
	cEst.SetParams(Params(0.0));
	Expect(!cEst.Object(Us(1.0), false, adP) && !cEst.Object(Us(1.0), true, adP) && adP[0] == 9.0 && !cEst.HasFolded(),
	       "before the first result: no object");
	Expect(cEst.Fold(adA, Us(1.0), 1, Us(1.1)) && cEst.Object(Us(1.1), true, adP) && adP[0] == 0.5, "the first result: its object");
	adP[0] = 9.0;
	Expect(!cEst.Object(Us(1.101), true, adP) && !cEst.Object(Us(1.102), false, adP) && adP[0] == 9.0,
	       "the periods after it: nothing new, the object stands");
	Expect(!cEst.Fold(adB, Us(1.0), 1, Us(1.2)) && !cEst.Fold(adB, Us(0.9), 1, Us(1.2)) && cEst.Folded() == 1 &&
	       !cEst.Object(Us(1.2), true, adP), "a capture that is not after the last one: not folded");
	Expect(cEst.Fold(adB, Us(1.045), 1, Us(1.2)) && cEst.Object(Us(1.2), false, adP) && fabs(adP[0] - 0.54) < 1e-12,
	       "a later capture: folded");

	CObsEstimate::Params st = Params(0.0);
	st.dAlpha = 0.0;
	cEst.SetParams(st);
	cEst.Fold(adA, Us(1.0), 1, Us(1.1));
	Expect(cEst.Fold(adB, Us(1.045), 1, Us(1.2)) && cEst.Object(Us(1.2), true, adP) && adP[0] == 0.6,
	       "an alpha of 0: the newest result alone");
}

void Moves()
{
	// the object moves along x at 0.1 m/s; a result every 33 ms, used 120 ms after its capture
	double adP[3] = { 0.0, 0.0, 0.0 }, adQ[3] = { 0.0, 0.0, 0.0 };
	CObsEstimate cEst, cStill;
	cEst.SetParams(Params(0.1));
	cStill.SetParams(Params(0.0));
	double dT = 1.0;
	for (int i = 0; i < 200; i++, dT += 0.033)
	{
		const double adZ[3] = { 0.5 + 0.1 * (dT - 1.0), 0.1, 0.1 };
		cEst.Fold(adZ, Us(dT), 1, Us(dT + 0.12));
		cEst.Object(Us(dT + 0.12), true, adP);
		cStill.Fold(adZ, Us(dT), 1, Us(dT + 0.12));
		cStill.Object(Us(dT + 0.12), true, adQ);
	}
	dT -= 0.033;
	const double dTrue = 0.5 + 0.1 * (dT + 0.12 - 1.0);
	Expect(fabs(adP[0] - dTrue) < 1e-4 && adP[1] == 0.1 && adP[2] == 0.1, "beta > 0: the object at the release, within 0.1 mm");
	Expect(dTrue - adQ[0] > 0.012, "beta = 0 on the same results: more than 12 mm behind");
	Expect(cEst.Object(Us(dT + 0.15), true, adP) && fabs(adP[0] - (dTrue + 0.1 * 0.03)) < 1e-4,
	       "a later period with the result in use: the object 30 ms on");
	adP[0] = 9.0;
	Expect(!cEst.Object(Us(dT + 0.2), false, adP) && adP[0] == 9.0, "a period without a result in use: the last object stands");
}

void Lost()
{
	const double adA[3] = { 0.5, 0.0, 0.1 };
	CObsEstimate cEst;
	cEst.SetParams(Params(0.0));
	Expect(!cEst.Lost(Us(100.0)), "before the first result: not lost");
	cEst.Fold(adA, Us(1.0), 1, Us(1.1));
	Expect(!cEst.Lost(Us(2.1)) && cEst.Lost(Us(2.1) + 1), "lost after dLossS without a folded result, not at it");
	cEst.Fold(adA, Us(3.0), 1, Us(3.1));
	Expect(!cEst.Lost(Us(3.5)), "a result again: not lost");
}

} // namespace

int main()
{
	AsTheBridge();
	AnotherTarget();
	Gap();
	ObjectOfAPeriod();
	Moves();
	Lost();
	printf("%s: %d checks failed\n", g_nFailed == 0 ? "PASS" : "FAIL", g_nFailed);
	return g_nFailed == 0 ? 0 : 1;
}
