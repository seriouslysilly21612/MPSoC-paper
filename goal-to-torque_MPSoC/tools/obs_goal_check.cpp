/*****************************************************************************
*	Name: obs_goal_check.cpp
*	Description: The test of the two ends of the mailbox in the control
*	application: the camera side (indy7/src/obs/ObsWriter.h) and the control
*	task's side (indy7/src/obs/ObsGoal.h), in one process. A tick is 1 ns and
*	a period 1 ms, as in the application.
*
*	    bin/obs_goal_check
*
*	The control task's side, with a clock that the test sets and a tag that
*	moves along x with the first joint angle:
*	1. Results in the base frame: the goal slot holds, bit for bit, what the
*	   bridge of M1 writes for the same samples (GoalFilter.h: the checks of
*	   the workspace, the moving average, its restarts after a gap), and a
*	   rejected sample leaves the slot as it was. (The mailbox carries the
*	   position as 32-bit floats: the samples are rounded to them.)
*	2. Results relative to the tag while the arm moves at 0.2 m/s, with joint
*	   angles that are two periods old when a period gets them: with the
*	   contract the object is where it is; with the angles of the arrival it
*	   is off by the way of the tag since the capture; with the contract but
*	   an age of 0 it is off by the way of the tag in two periods.
*	3. Another target restarts the estimate from its first result.
*	4. Two results in a period, a result whose error bound stays above the
*	   tolerance, a capture in periods without a state, an image without the
*	   tag: the entries of the period and the counts.
*	5. The target is lost after dLossS without a folded result, and back
*	   with the next one.
*	6. Settings that cannot be used are refused.
*	7. The status of a period, for the camera side (obs_status.h): the
*	   tolerance, the result used last with its deadline and its error bound,
*	   whether this period had a result to use, and the lost target.
*	The camera side, on the clock of the machine:
*	8. What a result becomes in the mailbox, the results that are not
*	   written, the counts, and a stamp of the system clock on the clock of
*	   the tasks.
*	9. Both ends: a result of the camera side is the object in the goal slot.
*	The test passes if no check failed.
*****************************************************************************/
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>

#include "obs/ObsGoal.h"
#include "obs/ObsWriter.h"

namespace {

const uint64_t T = 1000000;					// a period [ns]
const double TAG_V = 0.2;					// the tag moves at this speed [m/s] ...
const double Q_RATE = 0.2;					// ... with the first joint at this one [rad/s]
const uint64_t T_REF = 1000 * T;			// the first joint angle is 0 here

int g_nFailed = 0;
alignas(64) obs_mailbox_t g_stMb;
uint64_t g_uNow = 0;						// the clock of the test: what obs_put reads
CGoalSlot g_cSlot;
int g_nTagCalls = 0;

volatile obs_mailbox_t* Mb() { return &g_stMb; }
uint64_t Now() { return g_uNow; }

void Expect(bool abOk, const char* apcWhat)
{
	if (abOk)
		return;
	g_nFailed++;
	printf("  FAILED: %s\n", apcWhat);
}

/* the arm of the test: the first joint angle at a time, and the tag at joint angles */
double QTrue(uint64_t atT) { return Q_RATE * ((double)atT - (double)T_REF) * 1e-9; }
void TagAt(void*, const float* afQ, double* adP)
{
	g_nTagCalls++;
	adP[0] = 0.40 + (TAG_V / Q_RATE) * (double)afQ[0];
	adP[1] = 0.10;
	adP[2] = 0.30;
}
void TagTrue(uint64_t atT, double* adP)
{
	const float afQ[OBS_DOF] = { (float)QTrue(atT), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	const int nCalls = g_nTagCalls;
	TagAt(NULL, afQ, adP);
	g_nTagCalls = nCalls;
}

/* the checks and the smoothing of the test: none of them the default of GoalFilter.h or ObsEstimate.h */
const double KAPPA = 0.3, GAP_S = 0.9, LOSS_S = 0.8;
CGoalFilter::Params Filter()
{
	const CGoalFilter::Params st = { { 0.31, 0.90, -0.30, 0.60, 0.12, 0.48 }, 0.90, 0.26, 0.21, KAPPA, GAP_S };
	return st;
}

CObsGoal::Config Config(int anPolicy, int anStateAge)
{
	CObsGoal::Config st;
	memset(&st, 0, sizeof(st));
	st.stSet.policy = anPolicy;
	st.stSet.period_ticks = T;
	st.stSet.age_cap_ticks = 250 * T;
	st.stSet.ticks_per_s = 1e9;
	st.stSet.e0 = 0.001;
	st.stSet.g = 1.0;
	st.stSet.eps_cal = 0.002;
	st.dTolM = 0.010;
	st.nStateAge = anStateAge;
	st.stFilter = Filter();
	st.dLossS = LOSS_S;
	st.pTagAt = &TagAt;
	return st;
}

/* a control task of the test: periods one after the other, each with the joint angles that are anAge periods old */
struct Task
{
	CObsGoal	cGoal;
	uint32_t	uK;				// the last period
	int			nAge;

	bool Start(const CObsGoal::Config& astCfg, int anTrueAge)
	{
		memset(&g_stMb, 0, sizeof(g_stMb));
		obs_writer_start(Mb());
		uK = 2000;
		nAge = anTrueAge;
		g_uNow = Release(uK);
		return cGoal.Init(Mb(), astCfg, &g_cSlot);
	}
	static uint64_t Release(uint32_t auK) { return (uint64_t)auK * T; }
	void Period(bool abState = true)
	{
		uK++;
		const double adQ[OBS_DOF] = { QTrue(Release(uK) - (uint64_t)nAge * T), 0.0, 0.0, 0.0, 0.0, 0.0 };
		cGoal.Step(uK, Release(uK), abState ? adQ : NULL);
	}
	void Periods(int anN, bool abState = true)
	{
		for (int i = 0; i < anN; i++)
			Period(abState);
	}
};

/* a result into the mailbox: captured at atTe, published in the middle of the period after the last one */
uint32_t Put(const Task& acTask, uint64_t atTe, const double* adZ, unsigned auFlags, float afSigma = 0.002f, uint32_t auTarget = 0)
{
	obs_desc_t stD;
	memset(&stD, 0, sizeof(stD));
	stD.t_e = atTe;
	stD.t_valid_until = atTe + 250 * T;
	for (int k = 0; k < 3; k++)
		stD.z[k] = (float)adZ[k];
	stD.sigma = afSigma;
	stD.flags = (uint8_t)auFlags;
	stD.target_seq = auTarget;
	g_uNow = Task::Release(acTask.uK) + T / 2;
	Expect(obs_put(Mb(), &stD, &Now) == OBS_OK, "the result is in the mailbox");
	return stD.n;
}

bool Goal(double* adP, unsigned int* apuCount)
{
	*apuCount = 0;
	return g_cSlot.Read(adP, apuCount);
}

uint32_t Rand(uint32_t& auX)
{
	auX ^= auX << 13;
	auX ^= auX >> 17;
	auX ^= auX << 5;
	return auX;
}
double Uniform(uint32_t& auX) { return (double)Rand(auX) / 4294967296.0; }

/* 1 */
void AsTheBridge()
{
	printf("1. results in the base frame against the bridge of M1\n");
	Task cT;
	Expect(cT.Start(Config(OBS_MAX_AGE, 2), 2), "the settings are accepted");
	cT.Periods(300);
	CGoalFilter cFilter;
	cFilter.SetParams(Filter());
	uint32_t uX = 2463534242u;
	unsigned int uCount0 = 0, uCount = 0;
	double adGoal[3] = { 0.0, 0.0, 0.0 };
	Goal(adGoal, &uCount0);
	long nSame = 0, nWritten = 0, nKept = 0, nMismatch = 0;
	for (int i = 0; i < 600; i++)
	{
		double adP[3] = { 0.45 + 0.30 * Uniform(uX), 0.00 + 0.30 * Uniform(uX), 0.06 + 0.10 * Uniform(uX) };
		const uint32_t uCase = Rand(uX) % 10u;
		if (uCase == 0)
			adP[0] = 1.20;						// outside the box
		else if (uCase == 1)
			adP[2] = 0.00;						// its goal below the floor
		else if (uCase == 2)
			adP[1] = 0.615;						// outside the box of the test, inside the default of GoalFilter.h
		for (int k = 0; k < 3; k++)
			adP[k] = (double)(float)adP[k];		// as the mailbox carries it
		const uint64_t tTe = Task::Release(cT.uK) - 80 * T + 300000;
		Put(cT, tTe, adP, OBS_Z_BASE);
		cT.Period();
		double adOut[3];
		const CGoalFilter::eVerdict eV = cFilter.Push(true, adP, (double)tTe * 1e-9, adOut);
		unsigned int uNow = 0;
		const bool bHave = Goal(adGoal, &uNow);
		if (eV == CGoalFilter::eAccepted)
		{
			nWritten += (uNow == uCount + 1) ? 1 : 0;
			nSame += (bHave && memcmp(adGoal, adOut, sizeof(adOut)) == 0) ? 1 : 0;
		}
		else
			nKept += (uNow == uCount) ? 1 : 0;
		if (cT.cGoal.Entries() != 1 || cT.cGoal.GetEntry(0).stRec.verdict != OBS_V_ACCEPTED ||
		    cT.cGoal.GetEntry(0).nWorkspace != (int)eV ||
		    cT.cGoal.GetEntry(0).nFolded != (eV != CGoalFilter::eAccepted ? 0 : cFilter.Restarted() ? 2 : 1))
			nMismatch++;
		uCount = uNow;
		cT.Periods((Rand(uX) % 40u == 0) ? 1500 : 32);		// the next image, or a gap of 1.5 s
	}
	const long nAcc = cFilter.Count(CGoalFilter::eAccepted);
	CObsGoal::Counts stC;
	cT.cGoal.GetCounts(stC);
	printf("   600 samples: %ld accepted, %ld outside the box, %ld below the floor, %ld restarts\n", nAcc,
	       cFilter.Count(CGoalFilter::eOutsideBox), cFilter.Count(CGoalFilter::eBelowFloor), cFilter.Restarts());
	Expect(nAcc > 350 && cFilter.Count(CGoalFilter::eOutsideBox) > 60 && cFilter.Count(CGoalFilter::eBelowFloor) > 20 &&
	       cFilter.Restarts() > 5, "the samples cover acceptance, both rejections and restarts");
	Expect(nSame == nAcc, "every accepted sample: the goal slot holds the average of the bridge, bit for bit");
	Expect(nWritten == nAcc, "every accepted sample: one goal written");
	Expect(nKept == 600 - nAcc, "every rejected sample: the goal slot as it was");
	Expect(nMismatch == 0, "every period with a result: one entry, accepted, with the verdict of the workspace and the fold");
	Expect(stC.uTaken == 600 && stC.auVerdict[OBS_V_ACCEPTED] == 600 && stC.nFolded == nAcc &&
	       stC.nRestarts == cFilter.Restarts() && stC.nGoals == nAcc &&
	       stC.anWorkspace[CGoalFilter::eAccepted] == nAcc &&
	       stC.anWorkspace[CGoalFilter::eOutsideBox] == cFilter.Count(CGoalFilter::eOutsideBox) &&
	       stC.anWorkspace[CGoalFilter::eBelowFloor] == cFilter.Count(CGoalFilter::eBelowFloor), "the counts");
	CObsGoal::Counts stD = stC;
	stD.Minus(stC);
	Expect(stD.uTaken == 0 && stD.auVerdict[OBS_V_ACCEPTED] == 0 && stD.nFolded == 0 && stD.nRestarts == 0 &&
	       stD.nGoals == 0 && stD.anWorkspace[CGoalFilter::eAccepted] == 0, "the counts since themselves are 0");
	Expect(g_nTagCalls == 0, "a result in the base frame does not ask for the tag");
}

/* 2: the error of the object [m] with a result relative to the tag, captured 120.3 ms before the release;
 * *apnBack: the period of the state used, counted back from the period that used the result */
double TagError(int anPolicy, int anStateAge, int anTrueAge, double* adAge, int* apnBack)
{
	const double adObj[3] = { 0.60, 0.20, 0.10 };
	Task cT;
	Expect(cT.Start(Config(anPolicy, anStateAge), anTrueAge), "the settings are accepted");
	cT.Periods(300);
	const uint64_t tTe = Task::Release(cT.uK + 1) - 120300000ULL;
	double adTag[3], adZ[3];
	TagTrue(tTe, adTag);
	for (int k = 0; k < 3; k++)
		adZ[k] = adObj[k] - adTag[k];
	Put(cT, tTe, adZ, OBS_Z_TAG);
	cT.Period();
	*adAge = (double)(Task::Release(cT.uK) - tTe) * 1e-9;
	double adGoal[3] = { 0.0, 0.0, 0.0 };
	unsigned int uCount = 0;
	Expect(Goal(adGoal, &uCount) && cT.cGoal.Entries() == 1 && cT.cGoal.GetEntry(0).nFolded == 2,
	       "the result is used and starts the estimate");
	Expect(std::fabs(adGoal[1] - adObj[1]) < 1e-6 && std::fabs(adGoal[2] - adObj[2]) < 1e-6, "the object in y and z");
	*apnBack = cT.cGoal.Entries() == 1 ? (int)(cT.uK - cT.cGoal.GetEntry(0).stRec.h.d.s) : -1;
	return adGoal[0] - adObj[0];
}

void WithTheTag()
{
	printf("2. results relative to the tag while the arm moves\n");
	double dAge = 0.0;
	int nBackContract = 0, nBackArrival = 0, nBack = 0;
	const double dContract = TagError(OBS_CONTRACT, 2, 2, &dAge, &nBackContract);
	const double dArrival = TagError(OBS_MAX_AGE, 2, 2, &dAge, &nBackArrival);
	const double dNoAge = TagError(OBS_CONTRACT, 0, 2, &dAge, &nBack);
	printf("   age %.1f ms, tag at %.1f m/s: contract %+.4f mm, angles of the arrival %+.3f mm, contract with an "
	       "age of 0 for angles 2 periods old %+.4f mm\n", dAge * 1e3, TAG_V, dContract * 1e3, dArrival * 1e3,
	       dNoAge * 1e3);
	Expect(std::fabs(dContract) < 2e-6, "the contract: the object is where it is");
	/* the newest angles are those of the release minus two periods */
	Expect(std::fabs(dArrival - TAG_V * (dAge - 0.002)) < 2e-6, "the angles of the arrival: off by the way of the tag since the capture");
	Expect(std::fabs(dNoAge + TAG_V * 0.002) < 2e-6, "the contract without the age of the angles: off by the way of the tag in two periods");
	Expect(g_nTagCalls == 3, "one position of the tag per result used");
	/* the angles of a period are those of 2 periods before: the capture, 120.3 ms back, lies between the angles of
	 * the periods 119 and 118 back */
	Expect(nBackContract == 119 && nBackArrival == 0, "the period of the state used: that of the capture, or that of the arrival");
}

/* 3 */
void AnotherTarget()
{
	printf("3. another target\n");
	Task cT;
	Expect(cT.Start(Config(OBS_CONTRACT, 2), 2), "the settings are accepted");
	cT.Periods(300);
	const double adA[3] = { 0.50, 0.10, 0.10 }, adA2[3] = { 0.52, 0.10, 0.10 }, adB[3] = { 0.75, 0.40, 0.08 };
	double adGoal[3];
	unsigned int uCount = 0;
	Put(cT, Task::Release(cT.uK) - 60 * T, adA, OBS_Z_BASE, 0.002f, 7);
	cT.Periods(33);
	Put(cT, Task::Release(cT.uK) - 60 * T, adA2, OBS_Z_BASE, 0.002f, 7);
	cT.Periods(33);
	Expect(Goal(adGoal, &uCount) && std::fabs(adGoal[0] - (0.50 + KAPPA * 0.02)) < 1e-6, "the same target: averaged");
	Put(cT, Task::Release(cT.uK) - 60 * T, adB, OBS_Z_BASE, 0.002f, 8);
	cT.Period();
	Expect(Goal(adGoal, &uCount) && std::fabs(adGoal[0] - 0.75) < 1e-6 && std::fabs(adGoal[1] - 0.40) < 1e-6 &&
	       std::fabs(adGoal[2] - 0.08) < 1e-6, "another target: its first result is the object");
	Expect(cT.cGoal.Entries() == 1 && cT.cGoal.GetEntry(0).nFolded == 2, "the entry says that the estimate started again");
	/* the gap counts between two captures: 0.95 s here, with the publications 0.76 s apart */
	const double adC[3] = { 0.55, 0.20, 0.10 }, adD[3] = { 0.65, 0.20, 0.10 };
	cT.Periods(300);
	Put(cT, Task::Release(cT.uK) - 239 * T, adC, OBS_Z_BASE, 0.002f, 8);
	cT.Periods(760);
	Put(cT, Task::Release(cT.uK) - 50 * T, adD, OBS_Z_BASE, 0.002f, 8);
	cT.Period();
	Expect(Goal(adGoal, &uCount) && std::fabs(adGoal[0] - 0.65) < 1e-6 && cT.cGoal.GetEntry(0).nFolded == 2,
	       "a gap between two captures restarts the estimate, however close their publications");
}

/* 4 */
void Entries()
{
	printf("4. the entries of a period\n");
	const double adObj[3] = { 0.60, 0.20, 0.10 };
	unsigned int uCount = 0, uCount0 = 0;
	double adGoal[3];
	{
		Task cT;
		Expect(cT.Start(Config(OBS_CONTRACT, 2), 2), "the settings are accepted");
		cT.Periods(300);
		const uint32_t nOld = Put(cT, Task::Release(cT.uK) - 90 * T, adObj, OBS_Z_BASE);
		const uint32_t nNew = Put(cT, Task::Release(cT.uK) - 50 * T, adObj, OBS_Z_BASE);
		cT.Period();
		Expect(cT.cGoal.Entries() == 2, "two results in a period: two entries");
		bool bOld = false, bNew = false;
		for (unsigned i = 0; i < cT.cGoal.Entries(); i++)
		{
			const CObsGoal::Entry& stE = cT.cGoal.GetEntry(i);
			if (stE.stRec.h.d.n == nOld)
				bOld = stE.stRec.verdict == OBS_V_SUPERSEDED && stE.nWorkspace == -1 && stE.nFolded == 0;
			if (stE.stRec.h.d.n == nNew)
				bNew = stE.stRec.verdict == OBS_V_ACCEPTED && stE.nWorkspace == (int)CGoalFilter::eAccepted &&
				       stE.nFolded == 2 && std::fabs(stE.adObj[0] - adObj[0]) < 1e-6;
		}
		Expect(bOld && bNew, "the newer capture is used, the older one superseded, and only the one used has an object");
		cT.Period();
		Expect(cT.cGoal.Entries() == 0, "a period without a new verdict has no entry");
	}
	{
		/* sigma 20 mm: eq. (7) gives 23 mm, above the tolerance of 10 mm at every age */
		Task cT;
		Expect(cT.Start(Config(OBS_CONTRACT, 2), 2), "the settings are accepted");
		cT.Periods(300);
		Goal(adGoal, &uCount0);
		Put(cT, Task::Release(cT.uK) - 50 * T, adObj, OBS_Z_BASE, 0.020f);
		cT.Periods(150);
		Goal(adGoal, &uCount);
		Expect(uCount == uCount0, "the contract: a result above the tolerance is never used");
		CObsGoal::Counts stC;
		cT.cGoal.GetCounts(stC);
		Expect(stC.auVerdict[OBS_V_ERROR_EXCEEDED] == 0, "... and has no verdict before its deadline");
		cT.Periods(60);
		cT.cGoal.GetCounts(stC);
		Expect(stC.auVerdict[OBS_V_ERROR_EXCEEDED] == 1 && stC.auVerdict[OBS_V_ACCEPTED] == 0 && stC.nGoals == 0,
		       "... and leaves as error_exceeded at its deadline");
		Task cM;
		Expect(cM.Start(Config(OBS_MAX_AGE, 2), 2), "the settings are accepted");
		cM.Periods(300);
		Put(cM, Task::Release(cM.uK) - 50 * T, adObj, OBS_Z_BASE, 0.020f);
		cM.Period();
		cM.cGoal.GetCounts(stC);
		Expect(stC.auVerdict[OBS_V_ACCEPTED] == 1 && stC.nGoals == 1, "the age alone: the same result is used");
	}
	{
		Task cT;
		Expect(cT.Start(Config(OBS_CONTRACT, 2), 2), "the settings are accepted");
		cT.Periods(300);
		cT.Periods(40, false);						// the bus is not up
		cT.Periods(40);
		double adTag[3], adZ[3];
		const uint64_t tTe = Task::Release(cT.uK) - 60 * T;		// in the periods without a state
		TagTrue(tTe, adTag);
		for (int k = 0; k < 3; k++)
			adZ[k] = adObj[k] - adTag[k];
		Put(cT, tTe, adZ, OBS_Z_TAG);
		Put(cT, tTe + T, adZ, OBS_Z_TAG | OBS_TAG_LOST);
		Put(cT, tTe + 2 * T, adObj, OBS_Z_BASE);
		cT.Period();
		CObsGoal::Counts stC;
		cT.cGoal.GetCounts(stC);
		Expect(stC.auVerdict[OBS_V_STATE_UNRECOVERABLE] == 2 && stC.auVerdict[OBS_V_ACCEPTED] == 1 && stC.nFolded == 1,
		       "no state at the capture, no tag in the image: state_unrecoverable; the result in the base frame is used");
		Expect(cT.cGoal.Entries() == 3, "three results, three entries");
	}
}

/* 5 */
void Lost()
{
	printf("5. the target is lost\n");
	const double adObj[3] = { 0.60, 0.20, 0.10 };
	Task cT;
	Expect(cT.Start(Config(OBS_CONTRACT, 2), 2), "the settings are accepted");
	cT.Periods(300);
	Expect(!cT.cGoal.Lost(), "no target yet: not lost");
	Put(cT, Task::Release(cT.uK) - 50 * T, adObj, OBS_Z_BASE);
	cT.Period();
	cT.Periods(800);
	Expect(!cT.cGoal.Lost(), "0.8 s after the period that used the last result: not lost");
	cT.Period();
	Expect(cT.cGoal.Lost(), "a period later: lost");
	Put(cT, Task::Release(cT.uK) - 50 * T, adObj, OBS_Z_BASE);
	cT.Period();
	Expect(!cT.cGoal.Lost(), "the next result: the target again");
	Expect(cT.cGoal.OutsideWorkspace() == 0, "no result outside the workspace so far");
	const double adFar[3] = { 2.00, 0.20, 0.10 };			// beyond the box of the test
	Put(cT, Task::Release(cT.uK) - 50 * T, adFar, OBS_Z_BASE);
	cT.Period();
	cT.Periods(801);
	Expect(cT.cGoal.Lost() && cT.cGoal.OutsideWorkspace() == 1,
	       "a result outside the workspace does not keep the target, and is counted");
}

/* 6 */
void Refused()
{
	printf("6. settings that cannot be used\n");
	const double dNan = std::numeric_limits<double>::quiet_NaN();
	Task cT;
	CObsGoal::Config st = Config(OBS_CONTRACT, 2);
	Expect(cT.Start(st, 2), "the settings of the test are accepted");
	g_stMb.state[3][0] = OBS_READING;			// a control task that died in a copy
	Expect(cT.cGoal.Init(Mb(), st, &g_cSlot) && g_stMb.state[3][0] == OBS_FREE, "the start gives a slot left in READING back");
	Expect(!cT.cGoal.Init(NULL, st, &g_cSlot) && !cT.cGoal.Init(Mb(), st, NULL), "no mailbox, no goal slot");
	st.pTagAt = NULL;
	Expect(!cT.Start(st, 2), "no position of the tag");
	const double adTol[4] = { 0.0, -0.010, dNan, std::numeric_limits<double>::infinity() };
	for (double dTol : adTol)
	{
		st = Config(OBS_CONTRACT, 2);
		st.dTolM = dTol;
		Expect(!cT.Start(st, 2), "a tolerance that is 0, negative or not finite");
	}
	for (int nTerm = 0; nTerm < 5; nTerm++)
		for (int nBad = 0; nBad < 2; nBad++)
		{
			st = Config(OBS_CONTRACT, 2);
			double* const apTerm[5] = { &st.stSet.e0, &st.stSet.g, &st.stSet.eps_v, &st.stSet.a_max, &st.stSet.eps_cal };
			*apTerm[nTerm] = nBad ? dNan : -0.001;
			Expect(!cT.Start(st, 2), "a term of eq. (7) that is negative or not finite");
		}
	st = Config(OBS_CONTRACT, -1);
	Expect(!cT.Start(st, 2), "a negative age of the state");
	st = Config(OBS_CONTRACT, CObsGoal::MAX_STATE_AGE + 1);
	Expect(!cT.Start(st, 2), "an age of the state above 8 periods");
	st = Config(OBS_CONTRACT, CObsGoal::MAX_STATE_AGE);
	Expect(cT.Start(st, 2), "an age of the state of 8 periods");
	st = Config(OBS_CONTRACT, 2);
	st.stSet.age_cap_ticks = (OBS_STATES - 3) * T;
	Expect(!cT.Start(st, 2), "an age cap that the record of the states does not cover");
	st = Config(2, 2);
	Expect(!cT.Start(st, 2), "a policy that does not exist");
}

/* 7 */
void Status()
{
	printf("7. the status of a period\n");
	const double adObj[3] = { 0.60, 0.20, 0.10 };
	Task cT;
	Expect(cT.Start(Config(OBS_CONTRACT, 2), 2), "the settings are accepted");
	cT.Period();
	const obs_status_t& stS = cT.cGoal.Status();
	Expect(stS.t_release == Task::Release(cT.uK) && stS.tolerance == 0.010f && stS.control_period == 0.001f,
	       "the release and the tolerance of the period, and the period of the task");
	Expect(stS.flags == 0 && stS.result_n == 0 && stS.result_t_e == 0, "no result so far: none in use, not lost");

	const uint64_t tCapture = Task::Release(cT.uK) - 50 * T;
	const uint32_t uN = Put(cT, tCapture, adObj, OBS_Z_BASE, 0.002f, 7);
	cT.Period();
	Expect(stS.flags == OBS_STATUS_RESULT_IN_USE && stS.result_n == uN && stS.target_seq == 7,
	       "the period that used the result: in use, its number and its target");
	Expect(stS.result_t_e == tCapture && stS.result_t_valid_until == tCapture + 250 * T, "its capture and its deadline");
	Expect(std::fabs(stS.result_error_bound - 0.005f) < 1e-7f && stS.result_mode == 0,
	       "its error bound, e0 + g (sigma + eps_cal) = 5 mm, and its mode");

	cT.Periods(199);			// the release of the last of them is the deadline: still in use
	Expect(stS.flags == OBS_STATUS_RESULT_IN_USE && stS.t_release == stS.result_t_valid_until, "at its deadline: in use");
	cT.Period();
	Expect(stS.flags == 0 && stS.result_n == uN && stS.result_t_valid_until == tCapture + 250 * T,
	       "a period later: none in use, and the fields of the last result stay");
	cT.Periods(601);
	Expect(stS.flags == OBS_STATUS_TARGET_LOST, "0.8 s after the period that used it first: lost");
	Expect(cT.Start(Config(OBS_CONTRACT, 2), 2) && stS.flags == 0 && stS.result_n == 0 && stS.result_t_valid_until == 0,
	       "a control task that starts again starts with an empty status");
}

/* 8 */
void CameraSide()
{
	printf("8. the camera side\n");
	memset(&g_stMb, 0, sizeof(g_stMb));
	CObsWriter cW;
	const CObsWriter::Config stCfg = { 0.250, { 0.002f, 0.003f } };
	cW.Init(Mb(), stCfg);
	const double adZ[3] = { 0.5, -0.25, 0.125 };
	const int64_t tNow = (int64_t)CObsWriter::NowNs();
	Expect(cW.Put(false, adZ, OBS_Z_BASE, tNow - 100000000, 0, 3) == CObsWriter::eInvalid, "an invalid result is not written");
	for (int k = 0; k < 3; k++)
	{
		double adBad[3] = { 0.5, -0.25, 0.125 };
		adBad[k] = k == 1 ? std::numeric_limits<double>::infinity() : std::numeric_limits<double>::quiet_NaN();
		Expect(cW.Put(true, adBad, OBS_Z_BASE, tNow - 100000000, 0, 3) == CObsWriter::eNotFinite, "one that is not finite neither");
	}
	Expect(cW.Put(true, adZ, OBS_Z_BASE, tNow + 1000000000, 0, 3) == CObsWriter::eAge, "nor one from the future");
	Expect(cW.Put(true, adZ, OBS_Z_BASE, tNow - 251000000, 0, 3) == CObsWriter::eAge, "nor one older than the cap");
	obs_desc_t stD;
	Expect(obs_take(Mb(), 0, &stD) == OBS_EMPTY, "the mailbox is still empty");
	cW.NextTarget();
	cW.NextTarget();
	Expect(cW.Put(true, adZ, OBS_Z_TAG, tNow - 100000000, 1, 3) == CObsWriter::eWritten, "a result is written");
	Expect(obs_take(Mb(), 0, &stD) == OBS_OK, "... and is in the mailbox");
	Expect(stD.t_e == (uint64_t)(tNow - 100000000) && stD.t_valid_until == stD.t_e + 250000000ULL &&
	       stD.f >= (uint64_t)tNow && stD.f <= CObsWriter::NowNs(), "its capture, its deadline and its publication");
	Expect(stD.z[0] == 0.5f && stD.z[1] == -0.25f && stD.z[2] == 0.125f && stD.sigma == 0.003f && stD.mode == 1 &&
	       stD.flags == OBS_Z_TAG && stD.cls == 3 && stD.target_seq == 2, "its position, error, mode, format, class and target");
	Expect(cW.Put(true, adZ, OBS_Z_BASE, tNow - 100000000, 0, 3) == CObsWriter::eWritten && obs_take(Mb(), 0, &stD) == OBS_OK &&
	       stD.sigma == 0.002f && stD.mode == 0 && stD.flags == OBS_Z_BASE, "the error of the other mode, the other format");
	Expect(cW.Put(true, adZ, OBS_Z_BASE, tNow - 100000000, 7, 3) == CObsWriter::eWritten && obs_take(Mb(), 0, &stD) == OBS_OK &&
	       stD.mode == 0, "a mode that does not exist is mode 0");
	for (int b = 0; b < (int)OBS_SLOTS; b++)
		Expect(cW.Put(true, adZ, OBS_Z_BASE, tNow - 100000000, 0, 3) == CObsWriter::eWritten, "eight results fill the mailbox");
	Expect(cW.Put(true, adZ, OBS_Z_BASE, tNow - 100000000, 0, 3) == CObsWriter::eFull, "the ninth is dropped");
	Expect(cW.Count(CObsWriter::eWritten) == 11 && cW.Count(CObsWriter::eInvalid) == 1 && cW.Count(CObsWriter::eNotFinite) == 3 &&
	       cW.Count(CObsWriter::eAge) == 2 && cW.Count(CObsWriter::eFull) == 1 && cW.Count(CObsWriter::eExpired) == 0, "the counts");
	const uint32_t uEpoch = g_stMb.epoch;
	cW.Init(Mb(), stCfg);
	Expect(g_stMb.epoch == uEpoch + 1 && cW.Count(CObsWriter::eWritten) == 0 && cW.Count(CObsWriter::eFull) == 0,
	       "a camera side that starts again: the next epoch, its counts from 0");
	/* a stamp of the system clock on the clock of the tasks: the two clocks read one after the other here too; the
	 * best of ten, as the machine may stall between two reads */
	int64_t tBest = INT64_MAX;
	for (int i = 0; i < 10; i++)
	{
		struct timespec stReal;
		clock_gettime(CLOCK_REALTIME, &stReal);
		const int64_t tMono = (int64_t)CObsWriter::NowNs();
		const int64_t tGot = CObsWriter::MonoOfReal((int64_t)stReal.tv_sec * 1000000000LL + stReal.tv_nsec - 50000000LL);
		const int64_t tErr = tGot - (tMono - 50000000LL);
		if ((tErr < 0 ? -tErr : tErr) < tBest)
			tBest = tErr < 0 ? -tErr : tErr;
	}
	printf("   a stamp of the system clock, 50 ms old, on the clock of the tasks: off by %lld ns\n", (long long)tBest);
	Expect(tBest < 200000, "the stamp is within 0.2 ms of its time on the clock of the tasks");
	const int64_t tStamp = 1700000000123456789LL;
	const int64_t tBack = CObsWriter::RealOfMono(CObsWriter::MonoOfReal(tStamp)) - tStamp;
	Expect((tBack < 0 ? -tBack : tBack) < 200000, "and back on the system clock: within 0.2 ms of the stamp");
}

/* 9 */
void BothEnds()
{
	printf("9. both ends\n");
	memset(&g_stMb, 0, sizeof(g_stMb));
	CObsWriter cW;
	const CObsWriter::Config stCfg = { 0.250, { 0.002f, 0.003f } };
	cW.Init(Mb(), stCfg);
	CObsGoal cGoal;
	Expect(cGoal.Init(Mb(), Config(OBS_CONTRACT, 2), &g_cSlot), "the settings are accepted");
	const double adObj[3] = { 0.625, 0.25, 0.125 };
	unsigned int uCount0 = 0, uCount = 0;
	double adGoal[3] = { 0.0, 0.0, 0.0 };
	Goal(adGoal, &uCount0);
	Expect(cW.Put(true, adObj, OBS_Z_BASE, (int64_t)CObsWriter::NowNs() - 100000000, 0, 1) == CObsWriter::eWritten,
	       "the camera side writes a result");
	cGoal.Step(1, CObsWriter::NowNs(), NULL);
	Expect(Goal(adGoal, &uCount) && uCount == uCount0 + 1 && adGoal[0] == 0.625 && adGoal[1] == 0.25 && adGoal[2] == 0.125,
	       "the next period of the control task has its object in the goal slot");
	Expect(cGoal.Entries() == 1 && cGoal.GetEntry(0).stRec.verdict == OBS_V_ACCEPTED &&
	       std::fabs((double)cGoal.GetEntry(0).stRec.h.ebar - 0.005) < 1e-6, "the entry: accepted, eq. (7) at 5 mm");
	Expect(strcmp(CObsGoal::VerdictName(OBS_V_ACCEPTED), "accepted") == 0 &&
	       strcmp(CObsGoal::VerdictName(OBS_V_EVICTED), "evicted") == 0 && strcmp(CObsGoal::VerdictName(OBS_VERDICTS), "?") == 0 &&
	       strcmp(CObsGoal::VerdictName(-1), "?") == 0 &&
	       strcmp(CObsWriter::Name(CObsWriter::eWritten), "written") == 0 && strcmp(CObsWriter::Name(CObsWriter::eResults), "?") == 0 &&
	       strcmp(CObsWriter::Name((CObsWriter::eResult)-1), "?") == 0, "the names of the verdicts and of the outcomes");
}

} // namespace

int main()
{
	AsTheBridge();
	WithTheTag();
	AnotherTarget();
	Entries();
	Lost();
	Refused();
	Status();
	CameraSide();
	BothEnds();
	printf("%s: %d check(s) failed\n", g_nFailed == 0 ? "PASS" : "FAIL", g_nFailed);
	return g_nFailed == 0 ? 0 : 1;
}
