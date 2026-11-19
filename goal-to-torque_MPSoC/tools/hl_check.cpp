/*****************************************************************************
*	Name: hl_check.cpp
*	Description: Offline check of the high-level controller (CHighLevelRT,
*	indy7/src/HighLevelRT.h; paper, Sec. IV-C, eq. (9) and Table 9) before it
*	drives the simulation or the robot.
*
*	The arm is ideal: it follows the reference exactly, and the controller
*	receives the state two periods old, as on the robot. The planner
*	(CHlPlanner) runs beside the loop: what the controller asks of it is back
*	a set number of periods later, one unless the scenario says otherwise.
*	Scenarios:
*	  A  HOME from another posture
*	  B  fixed goal 0.3 m away (the fixed-target task-space test); the same
*	     with the planner in the task of the controller (CHlInline)
*	  C  goal moved 0.15 m sideways when the tool is 0.15 m from it
*	  D  goal with noise (1 mm, every 67 periods), with delta and with delta = 0
*	  E  goal out of reach (rejected), then a goal within reach
*	  F  J2 blocked: the lead bound resets its reference
*	  G  a plan that is not back in its period: asked for again; the late one
*	     comes back afterwards, also while the arm moves; every plan late
*	  H  a goal that replaces one whose IK is not back yet, or is back in the
*	     same period; a long IK
*	  I  a goal whose IK comes back while the plan of the goal before is out
*	  J  a new start while a plan is out, with an IK just back, in a motion,
*	     and while a target and a joint at its bound wait for the next plan
*	  K  a plan that comes back with an error of the generator
*	  L  TRACK without a goal; a second start
*	  M  the step baseline
*	  N  the iterations of the IK: the step, its bound, the joint limits
*	In every period it checks the reference contract: q_d is the integral of
*	qd_d (C2), the limits on velocity, acceleration and jerk except where the
*	lead bound reset a joint, and the joint limits (C3), the lead (C4); and
*	the final position of the tool, the time to reach the goal against the
*	planned duration, the turn toward a moved goal without a stop (C1), and,
*	in the malloc-count build, that neither Update() nor the planner
*	allocates (C5).
*
*	    bin/hl_check models/indy7.urdf     exit code 0 when every check passes
*	(make tools; make MALLOC_COUNT=1 tools to count the allocations)
*****************************************************************************/
#include "HighLevelRT.h"
#include "CallTiming.h"
#include "base/MallocCount.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <random>
#include <vector>

namespace {

const int N = CHighLevelRT::DOF;
const double TS = 1e-3;
int g_nFail = 0;

void Check(bool abOk, const char* astrWhat, const char* astrDetail)
{
	printf("  %-4s %-58s %s\n", abOk ? "ok" : "FAIL", astrWhat, astrDetail);
	if (!abOk) g_nFail++;
}

/* the planner beside the loop: the result of a request of period c is given to the controller before its
 * Update() of period c + delay, each result at its own time. The IK runs to its end at once; its iterations are
 * in the result. */
struct Planner
{
	CHlPlanner pl;
	int nIkDelay = 1, nPlanDelay = 1;
	int nLatePlan = 0, nLateDelay = 0;	// the plan with this number (1 the first; -1: every plan) is back after nLateDelay
	int nFirstPlanResult = 0;			// the result of the first plan, when not 0 (an error of the generator)
	uint32_t uIkSeq = 0, uPlanSeq = 0;
	long nIks = 0, nPlans = 0, nAllocs = 0;
	double dMaxPlanUs = 0;
	std::deque<std::pair<long, CHighLevelRT::IkResult>> qIk;
	std::deque<std::pair<long, CHighLevelRT::PlanResult>> qPlan;
	CHighLevelRT::IkResult stIk;		// the last of each
	CHighLevelRT::PlanResult stPlan;

	void After(CHighLevelRT& hl, long nCycle)
	{
		const bool bIk = hl.GetIkRequest().uSeq != uIkSeq, bPlan = hl.GetPlanRequest().uSeq != uPlanSeq;
		const unsigned long uA0 = malloc_count::Count();
		malloc_count::Arm(true);
		if (bIk)
		{
			uIkSeq = hl.GetIkRequest().uSeq;
			pl.IkBegin(hl.GetIkRequest());
			while (!pl.IkStep(&stIk)) { }
		}
		if (bPlan)
		{
			uPlanSeq = hl.GetPlanRequest().uSeq;
			pl.Plan(hl.GetPlanRequest(), &stPlan);
		}
		malloc_count::Arm(false);
		nAllocs += (long)(malloc_count::Count() - uA0);
		if (bIk)
		{
			qIk.push_back(std::make_pair(nCycle + nIkDelay, stIk));
			nIks++;
		}
		if (bPlan)
		{
			nPlans++;
			if (nPlans == 1 && nFirstPlanResult) stPlan.nResult = nFirstPlanResult;
			if (stPlan.dCalcUs > dMaxPlanUs) dMaxPlanUs = stPlan.dCalcUs;
			qPlan.push_back(std::make_pair(nCycle + ((nLatePlan == nPlans || nLatePlan < 0) ? nLateDelay : nPlanDelay), stPlan));
		}
	}
	void Before(CHighLevelRT& hl, long nCycle)
	{
		for (auto it = qIk.begin(); it != qIk.end(); )
			if (nCycle >= it->first) { hl.TakeIk(it->second); it = qIk.erase(it); } else ++it;
		for (auto it = qPlan.begin(); it != qPlan.end(); )
			if (nCycle >= it->first) { hl.TakePlan(it->second); it = qPlan.erase(it); } else ++it;
	}
};

/* closed loop with an ideal arm: measurement = reference two periods old */
struct Loop
{
	CHighLevelRT& hl;
	Planner* pPlanner = NULL;			// one of the two carries the requests
	CHlInline* pInline = NULL;
	std::deque<std::vector<double>> qHist, qdHist;
	double q[N], qd[N], qr[N], qdr[N], qddr[N], qrPrev[N], qdrPrev[N], qddrPrev[N];
	bool bFirst;
	/* contract */
	double dMaxVel, dMaxAcc, dMaxJerk, dMaxInteg, dMaxLead, dMinMargin;
	long nCycles, nAllocs, nResets;
	double dMaxUs;
	unsigned uBlocked;		// joints whose measurement is frozen
	double adBlockedQ[N], adBlockedQd[N];

	explicit Loop(CHighLevelRT& aHl, const double* adQ0) : hl(aHl), bFirst(true), dMaxVel(0), dMaxAcc(0), dMaxJerk(0),
		dMaxInteg(0), dMaxLead(0), dMinMargin(1e9), nCycles(0), nAllocs(0), nResets(0), dMaxUs(0), uBlocked(0)
	{
		for (int i = 0; i < N; i++) { q[i] = adQ0[i]; qd[i] = 0; qr[i] = adQ0[i]; qdr[i] = 0; qddr[i] = 0; adBlockedQ[i] = adQ0[i]; adBlockedQd[i] = 0; }
		for (int k = 0; k < 2; k++) { qHist.push_back(std::vector<double>(q, q + N)); qdHist.push_back(std::vector<double>(N, 0.0)); }
	}

	bool Step(const double* adGoal)
	{
		/* the measurement of this period: the reference of two periods ago */
		for (int i = 0; i < N; i++) { q[i] = qHist.front()[i]; qd[i] = qdHist.front()[i]; }
		for (int i = 0; i < N; i++) if (uBlocked & (1u << i)) { q[i] = adBlockedQ[i]; qd[i] = adBlockedQd[i]; }
		memcpy(qrPrev, qr, sizeof(qr)); memcpy(qdrPrev, qdr, sizeof(qdr)); memcpy(qddrPrev, qddr, sizeof(qddr));
		if (pPlanner) pPlanner->Before(hl, nCycles);
		const unsigned long uA0 = malloc_count::Count();
		malloc_count::Arm(true);
		const BOOL bOk = hl.Update(adGoal, q, qd, qr, qdr, qddr);
		malloc_count::Arm(false);
		if (nCycles > 0) nAllocs += (long)(malloc_count::Count() - uA0);
		if (pPlanner) pPlanner->After(hl, nCycles);
		if (pInline) pInline->After(hl);
		const CHighLevelRT::Status& st = hl.GetStatus();
		if (st.dUs > dMaxUs && nCycles > 0) dMaxUs = st.dUs;
		const CHighLevelRT::Params& p = hl.GetParams();
		for (int i = 0; i < N; i++)
		{
			dMaxVel = std::max(dMaxVel, std::fabs(qdr[i]));
			dMaxAcc = std::max(dMaxAcc, std::fabs(qddr[i]));
			dMinMargin = std::min(dMinMargin, std::min(qr[i] - p.adLimLo[i], p.adLimHi[i] - qr[i]));
			dMaxLead = std::max(dMaxLead, std::fabs(qr[i] - q[i]));
			if (!bFirst && !(prevReset & (1u << i)))
			{
				dMaxJerk = std::max(dMaxJerk, std::fabs(qddr[i] - qddrPrev[i]) / TS);
				/* q_d is the integral of qd_d (trapezoid; Ruckig's jerk is piecewise constant) */
				dMaxInteg = std::max(dMaxInteg, std::fabs(qr[i] - qrPrev[i] - 0.5 * TS * (qdr[i] + qdrPrev[i])));
			}
		}
		prevReset = st.uLeadReset;
		if (st.uLeadReset) nResets++;
		bFirst = false;
		qHist.pop_front(); qdHist.pop_front();
		qHist.push_back(std::vector<double>(qr, qr + N)); qdHist.push_back(std::vector<double>(qdr, qdr + N));
		nCycles++;
		return bOk;
	}
	unsigned prevReset = 0;
};

double Norm3(const double* a, const double* b)
{
	return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

void ReportContract(const Loop& l, const CHighLevelRT::Params& p)
{
	char ac[160];
	snprintf(ac, sizeof(ac), "max |qd_d| %.4f rad/s (limit %.2f)", l.dMaxVel, p.dVelMax);
	Check(l.dMaxVel <= p.dVelMax + 1e-9, "C3 velocity limit", ac);
	snprintf(ac, sizeof(ac), "max |qdd_d| %.4f rad/s^2 (limit %.2f)", l.dMaxAcc, p.dAccMax);
	Check(l.dMaxAcc <= p.dAccMax + 1e-9, "C3 acceleration limit", ac);
	snprintf(ac, sizeof(ac), "max |dqdd_d|/T_s %.4f rad/s^3 (limit %.1f)", l.dMaxJerk, p.dJerkMax);
	Check(l.dMaxJerk <= p.dJerkMax * (1 + 1e-6) + 1e-6, "C3 jerk limit (outside lead-bound resets)", ac);
	snprintf(ac, sizeof(ac), "max |q_d(k)-q_d(k-1)-T(qd_d(k)+qd_d(k-1))/2| %.2e rad", l.dMaxInteg);
	Check(l.dMaxInteg < 1e-7, "C2 q_d is the integral of qd_d", ac);
	snprintf(ac, sizeof(ac), "closest to a joint limit %.4f rad", l.dMinMargin);
	Check(l.dMinMargin > 0.0, "C3 joint limits", ac);
	snprintf(ac, sizeof(ac), "Update(): %ld allocations in %ld periods%s; max %.1f us", l.nAllocs, l.nCycles,
	         malloc_count::Enabled() ? "" : " (not counted: build with make MALLOC_COUNT=1)", l.dMaxUs);
	Check(l.nAllocs == 0, "C5 no allocation in Update()", ac);
	if (l.pPlanner != NULL)
	{
		snprintf(ac, sizeof(ac), "planner: %ld allocations in %ld IK and %ld plans; a plan at most %.1f us",
		         l.pPlanner->nAllocs, l.pPlanner->nIks, l.pPlanner->nPlans, l.pPlanner->dMaxPlanUs);
		Check(l.pPlanner->nAllocs == 0, "C5 no allocation in the planner", ac);
	}
}

/* an arm at rest whose J1 is measured at a speed above the rest threshold for 250 periods, then below it: the
 * periods at rest among the first, and the first period at rest among the second */
void MovedThenStill(Loop& l, const double* adGoal, double adRestQd, int* apnAtRest, int* apnRest)
{
	l.uBlocked = 1u;
	l.adBlockedQ[0] = l.q[0];
	l.adBlockedQd[0] = 1.2 * adRestQd;
	*apnAtRest = 0;
	for (int k = 0; k < 250; k++) { l.Step(adGoal); if (l.hl.GetStatus().bAtRest) (*apnAtRest)++; }
	l.adBlockedQd[0] = 0.8 * adRestQd;
	*apnRest = -1;
	for (int k = 0; k < 250 && *apnRest < 0; k++) { l.Step(adGoal); if (l.hl.GetStatus().bAtRest) *apnRest = k; }
}

/* the high-level controller and its planner beside the loop, both from the URDF */
struct Pair
{
	CHighLevelRT hl;
	Planner pl;
	bool Init(const char* astrURDF, const CHighLevelRT::Params& p)
	{
		return hl.Init(astrURDF, TS, p) && pl.pl.Init(astrURDF, TS, p);
	}
};

} // namespace

int main(int argc, char** argv)
{
	if (argc < 2) { fprintf(stderr, "usage: %s <urdf>   (models/indy7.urdf)\n", argv[0]); return 2; }
	CHighLevelRT::Params p = CHighLevelRT::DefaultParams();
	const int H = p.nPlanAhead;
	char ac[240];
	const double d1[3] = { 0.2121320343559643, 0.0, -0.2121320343559643 };		// 0.3 m from the tool at q_r

	/* ---------------------------------------------------------------- A */
	{
		printf("A  HOME from another posture\n");
		Pair c;
		if (!c.Init(argv[1], p)) return 2;
		const double q0[N] = { 0.4, -0.3, -1.2, 0.3, -1.2, 0.5 };
		Loop l(c.hl, q0);
		l.pPlanner = &c.pl;
		c.hl.StartHome(q0, l.qd);
		int nRest = -1, nNew = -1;
		double dHeld = 0;
		for (int k = 0; k < 20000 && nRest < 0; k++)
		{
			l.Step(NULL);
			if (c.hl.GetStatus().bOtgNewCalc && nNew < 0) nNew = k;
			if (nNew < 0) for (int i = 0; i < N; i++) dHeld = std::max(dHeld, std::fabs(l.qr[i] - q0[i]) + std::fabs(l.qdr[i]));
			if (c.hl.GetStatus().bAtRest) nRest = k;
		}
		double dMax = 0;
		for (int i = 0; i < N; i++) dMax = std::max(dMax, std::fabs(l.q[i] - p.adQr[i]));
		snprintf(ac, sizeof(ac), "plan in use from period %d (H = %d); before it the reference moved %.1e", nNew, H, dHeld);
		Check(nNew == H && dHeld == 0.0, "holds the start posture until the plan is in use, H periods later", ac);
		snprintf(ac, sizeof(ac), "at rest after %.3f s, |q - q_r| %.1e rad", nRest * TS, dMax);
		Check(nRest > 0 && dMax < 1e-9, "reaches q_r and comes to rest", ac);
		ReportContract(l, p);
	}

	/* ---------------------------------------------------------------- B, C */
	int nRestB = -1;
	for (int nCase = 0; nCase < 3; nCase++)
	{
		printf(nCase == 0 ? "B  fixed goal 0.3 m away\n" : nCase == 1 ? "B  the same, the planner in the task of the controller (CHlInline)\n"
		       : "C  goal moved 0.15 m sideways when the tool is 0.15 m from it\n");
		Pair c;
		CHlInline cInline;
		if (!c.Init(argv[1], p) || !cInline.Init(argv[1], TS, p)) return 2;
		Loop l(c.hl, p.adQr);
		if (nCase == 1) l.pInline = &cInline; else l.pPlanner = &c.pl;
		CHighLevelRT& hl = c.hl;
		hl.StartTrack(p.adQr, l.qd);
		double p0[3], goal[3], tool[3];
		hl.ToolPosition(p.adQr, p0);
		for (int k = 0; k < 3; k++) goal[k] = p0[k] + d1[k];
		bool bJumped = false;
		int nAccept = -1, nConv = -1, nNew = -1, nRest = -1, nJump = -1;
		double dPlanned = 0, dMinSpeedAfterJump = 1e9;
		for (int e = 0; e < call_timing::eN; e++) call_timing::g_aStat[e] = call_timing::Stat();
		call_timing::g_bOn = (nCase == 1);					// the calls of the planner in the task are counted
		for (int k = 0; k < 20000; k++)
		{
			l.Step(goal);
			const CHighLevelRT::Status& st = hl.GetStatus();
			if (st.bGoalAccepted && nAccept < 0) nAccept = k;		// the first goal
			if (st.bIkConverged && nConv < 0) nConv = k;
			if (st.bOtgNewCalc && nNew < 0) nNew = k;
			if (st.bOtgNewCalc) dPlanned = st.dOtgDuration;
			hl.ToolPosition(l.q, tool);
			if (nCase == 2 && !bJumped && Norm3(tool, goal) <= 0.15)
			{
				goal[1] += 0.15;
				bJumped = true;
				nJump = k;
			}
			if (nCase == 2 && bJumped && k <= nJump + 500)		// 0.5 s after the jump
			{
				double dS = 0;
				for (int i = 0; i < N; i++) dS += l.qdr[i] * l.qdr[i];
				dMinSpeedAfterJump = std::min(dMinSpeedAfterJump, std::sqrt(dS));
			}
			if (st.bAtRest && (nCase != 2 || bJumped)) { nRest = k; break; }
		}
		call_timing::g_bOn = false;
		const CHighLevelRT::Status& st = hl.GetStatus();
		hl.ToolPosition(l.q, tool);
		snprintf(ac, sizeof(ac), "first goal accepted in period %d, its IK back in period %d, its plan in use from period %d; "
		         "last goal: %d iterations", nAccept, nConv, nNew, st.nIkIter);
		Check(nAccept == 0 && nConv == 1 && nNew == 1 + H, "the IK is back one period after the goal, the plan in use H after that", ac);
		snprintf(ac, sizeof(ac), "final distance of the tool %.4f mm (tolerance %.2f mm)", Norm3(tool, goal) * 1e3, p.dTol * 1e3);
		Check(Norm3(tool, goal) < p.dTol, "tool at the goal", ac);
		if (nCase != 2)
		{
			snprintf(ac, sizeof(ac), "at rest after %.3f s; plan from %.3f s, planned duration %.3f s + 0.2 s rest window", nRest * TS,
			         nNew * TS, dPlanned);
			Check(nRest > 0 && std::fabs(nRest * TS - (nNew * TS + dPlanned + 0.2 + 2 * TS)) < 0.01, "time to reach = planned duration", ac);
			const int nEnd = nNew + (int)std::ceil(dPlanned / TS - 1e-9);		// the first period at the target
			snprintf(ac, sizeof(ac), "at rest in period %d; the plan ends in period %d, the rest window is %d periods", nRest, nEnd, p.nRestCyc);
			Check(nRest == nEnd + p.nRestCyc - 1, "at rest when the plan has been at its target for the rest window", ac);
			if (nCase == 0)
			{
				/* the reference stays where the plan ended, however long */
				double adAt[N], adR[N], adRd[N], adRdd[N], dDrift = 0, dRate = 0;
				memcpy(adAt, l.qr, sizeof(adAt));
				for (long k = 0; k < 10000000; k++) hl.Update(goal, l.q, l.qd, adR, adRd, adRdd);
				for (int i = 0; i < N; i++)
				{
					dDrift = std::max(dDrift, std::fabs(adR[i] - adAt[i]));
					dRate = std::max(dRate, std::fabs(adRd[i]) + std::fabs(adRdd[i]));
				}
				snprintf(ac, sizeof(ac), "after 10,000 s more at the target the reference is %.1e rad from where it was; its speed and "
				         "acceleration %.1e", dDrift, dRate);
				Check(dDrift == 0.0 && dRate == 0.0, "the reference stays where the plan ended, at rest", ac);
			}
			if (nCase == 0) nRestB = nRest;
			else
			{
				snprintf(ac, sizeof(ac), "at rest in period %d, beside the loop in period %d", nRest, nRestB);
				Check(nRest == nRestB, "the planner in the task gives the same motion", ac);
				const unsigned long uPlans = (unsigned long)call_timing::g_aStat[call_timing::eOtg].uN,
					uIter = (unsigned long)call_timing::g_aStat[call_timing::eIkIter].uN;
				snprintf(ac, sizeof(ac), "in %d periods: %lu plan(s) calculated, the IK called %lu times for %d iterations", nRest + 1, uPlans,
				         uIter, st.nIkIter);
				Check(uPlans == 1 && uIter == (unsigned long)st.nIkIter + 1, "it calculates each request once", ac);
			}
		}
		else
		{
			snprintf(ac, sizeof(ac), "jump at %.3f s; lowest joint speed in the next 0.5 s %.3f rad/s", nJump * TS, dMinSpeedAfterJump);
			Check(nJump > 0 && dMinSpeedAfterJump > 0.05, "C1 turns toward the new goal without stopping", ac);
		}
		snprintf(ac, sizeof(ac), "%ld accepted, %ld converged, %ld rejected, %ld lead resets, %ld plans late", st.nAccepted, st.nConverged,
		         st.nRejected, st.nLeadResets, st.nPlanLate);
		Check(st.nRejected == 0 && st.nLeadResets == 0 && st.nPlanLate == 0, "counters", ac);
		ReportContract(l, p);
	}

	/* ---------------------------------------------------------------- D */
	for (int nCase = 0; nCase < 2; nCase++)
	{
		const double dDelta = nCase == 0 ? 0.006 : 0.0;	// delta of Table 9: 6 sigma of the noise
		printf("D  goal with noise of 1 mm every 67 periods, delta = %.0f mm\n", dDelta * 1e3);
		CHighLevelRT::Params pd = p;
		pd.dDelta = dDelta;
		Pair c;
		if (!c.Init(argv[1], pd)) return 2;
		CHighLevelRT& hl = c.hl;
		Loop l(hl, pd.adQr);
		l.pPlanner = &c.pl;
		hl.StartTrack(pd.adQr, l.qd);
		double p0[3], goal[3], g0[3], tool[3];
		hl.ToolPosition(pd.adQr, p0);
		for (int k = 0; k < 3; k++) g0[k] = goal[k] = p0[k] + d1[k];
		std::mt19937 rng(1);
		std::normal_distribution<double> nd(0.0, 0.001);
		const int nT = 10000;
		double adMin[3] = { 1e9, 1e9, 1e9 }, adMax[3] = { -1e9, -1e9, -1e9 };
		for (int k = 0; k < nT; k++)
		{
			if (k % 67 == 0)
				for (int j = 0; j < 3; j++) goal[j] = g0[j] + nd(rng);
			l.Step(goal);
			if (k >= nT - 1000)
			{
				hl.ToolPosition(l.q, tool);
				for (int j = 0; j < 3; j++) { adMin[j] = std::min(adMin[j], tool[j]); adMax[j] = std::max(adMax[j], tool[j]); }
			}
		}
		double dPP = 0;
		for (int j = 0; j < 3; j++) dPP = std::max(dPP, adMax[j] - adMin[j]);
		const CHighLevelRT::Status& st = hl.GetStatus();
		snprintf(ac, sizeof(ac), "%ld goals accepted of %d samples; residual motion (last 1 s) %.3f mm", st.nAccepted, nT / 67 + 1, dPP * 1e3);
		if (nCase == 0)
			Check(dPP < 1e-6, "with delta: the tool comes to rest", ac);
		else
			Check(dPP > 1e-4, "without delta: the tool keeps moving", ac);
		ReportContract(l, pd);
	}

	/* ---------------------------------------------------------------- E */
	{
		printf("E  goal out of reach, then a goal within reach\n");
		Pair c;
		if (!c.Init(argv[1], p)) return 2;
		CHighLevelRT& hl = c.hl;
		Loop l(hl, p.adQr);
		l.pPlanner = &c.pl;
		hl.StartTrack(p.adQr, l.qd);
		double p0[3], far[3], near_[3];
		hl.ToolPosition(p.adQr, p0);
		for (int k = 0; k < 3; k++) { far[k] = p0[k]; near_[k] = p0[k]; }
		far[0] += 2.0;
		near_[2] -= 0.1;
		int nRej = -1;
		for (int k = 0; k < 100 && nRej < 0; k++) { l.Step(far); if (hl.GetStatus().bIkRejected) nRej = k; }
		double dMaxQ = 0;
		for (int i = 0; i < N; i++) dMaxQ = std::max(dMaxQ, std::fabs(hl.GetStatus().adTarget[i] - p.adQr[i]));
		snprintf(ac, sizeof(ac), "rejected in period %d (%d iterations, N_max %d), target moved %.1e rad, %ld plans asked for", nRej,
		         hl.GetStatus().nIkIter, p.nIterMax, dMaxQ, c.pl.nPlans);
		Check(nRej == 1 && hl.GetStatus().nIkIter == p.nIterMax + 1 && dMaxQ == 0.0 && c.pl.nPlans == 0,
		      "rejected after N_max iterations, previous target kept, no plan", ac);
		int nRest = -1;
		for (int k = 0; k < 20000 && nRest < 0; k++) { l.Step(near_); if (hl.GetStatus().bAtRest) nRest = k; }
		double tool[3];
		hl.ToolPosition(l.q, tool);
		snprintf(ac, sizeof(ac), "then at rest after %.3f s, %.4f mm from the goal", nRest * TS, Norm3(tool, near_) * 1e3);
		Check(nRest > 0 && Norm3(tool, near_) < p.dTol, "a goal within reach is accepted afterwards", ac);
		ReportContract(l, p);
	}

	/* ---------------------------------------------------------------- F */
	{
		printf("F  J2 blocked while the goal needs it\n");
		Pair c;
		if (!c.Init(argv[1], p)) return 2;
		CHighLevelRT& hl = c.hl;
		Loop l(hl, p.adQr);
		l.pPlanner = &c.pl;
		hl.StartTrack(p.adQr, l.qd);
		double p0[3], goal[3];
		hl.ToolPosition(p.adQr, p0);
		for (int k = 0; k < 3; k++) goal[k] = p0[k] + d1[k];
		l.uBlocked = 1u << 1;
		l.adBlockedQ[1] = p.adQr[1];
		l.adBlockedQd[1] = 0.02;			// not what a blocked joint has; it shows whose speed the plan starts with
		double dMaxLeadJ2 = 0, dJump = 0, dStartOff = 0;
		int nFlagged = 0, nRun = 0, nLongest = 0;
		for (int k = 0; k < 3000; k++)
		{
			const bool bWas = (hl.GetStatus().uLeadReset & 2u) != 0;
			const double dBefore = l.qr[1];
			l.Step(goal);
			dMaxLeadJ2 = std::max(dMaxLeadJ2, std::fabs(l.qr[1] - l.q[1]));
			const bool bIs = (hl.GetStatus().uLeadReset & 2u) != 0;
			if (bIs) { nFlagged++; nRun++; nLongest = std::max(nLongest, nRun); } else nRun = 0;
			if (bWas && !bIs)			// the cycle after the last flagged one: time 0 of the plan that restarts J2
			{
				dJump = std::max(dJump, std::fabs(l.qr[1] - dBefore));
				dStartOff = std::max(dStartOff, std::fabs(l.qr[1] - p.adQr[1]) + std::fabs(l.qdr[1] - 0.02) + std::fabs(l.qddr[1]));
			}
		}
		snprintf(ac, sizeof(ac), "%ld resets of J2 in 3 s; largest lead of J2 %.4f rad (ell %.2f)", hl.GetStatus().nLeadResets, dMaxLeadJ2, p.dLead);
		Check(hl.GetStatus().nLeadResets > 0 && dMaxLeadJ2 < p.dLead + 0.01, "C4 the lead bound holds the reference of the blocked joint back", ac);
		snprintf(ac, sizeof(ac), "J2 flagged in %d periods, at most %d in a row (H = %d); its reference then jumps by up to %.4f rad",
		         nFlagged, nLongest, H, dJump);
		Check(nFlagged == H * (int)hl.GetStatus().nLeadResets && nLongest == H && dJump > p.dLead - 0.01,
		      "a reset takes H periods: the joint is flagged in them and its reference jumps after the last", ac);
		snprintf(ac, sizeof(ac), "after a reset the reference of J2 is %.1e off (q, qd, 0) of its measurement", dStartOff);
		Check(dStartOff == 0.0, "the plan restarts the joint at its measured position and speed, without acceleration", ac);
	}

	/* ---------------------------------------------------------------- G */
	{
		printf("G  the first plan is back two periods after the period it was asked for\n");
		Pair c;
		if (!c.Init(argv[1], p)) return 2;
		CHighLevelRT& hl = c.hl;
		c.pl.nLatePlan = 1;
		c.pl.nLateDelay = H + 2;
		Loop l(hl, p.adQr);
		l.pPlanner = &c.pl;
		hl.StartTrack(p.adQr, l.qd);
		double p0[3], goal[3], tool[3];
		hl.ToolPosition(p.adQr, p0);
		for (int k = 0; k < 3; k++) goal[k] = p0[k] + d1[k];
		int nNew = -1, nRest = -1;
		for (int k = 0; k < 20000 && nRest < 0; k++)
		{
			l.Step(goal);
			if (hl.GetStatus().bOtgNewCalc && nNew < 0) nNew = k;
			if (hl.GetStatus().bAtRest) nRest = k;
		}
		hl.ToolPosition(l.q, tool);
		snprintf(ac, sizeof(ac), "%ld plan late, %ld asked for; the plan in use from period %d (1 + 2 H = %d); %.4f mm from the goal",
		         hl.GetStatus().nPlanLate, c.pl.nPlans, nNew, 1 + 2 * H, Norm3(tool, goal) * 1e3);
		Check(hl.GetStatus().nPlanLate == 1 && c.pl.nPlans == 2 && nNew == 1 + 2 * H && nRest > 0 && Norm3(tool, goal) < p.dTol,
		      "a plan that is late is not used; it is asked for again, for H periods later", ac);
		/* a result that answers an older request, or none: not taken */
		CHighLevelRT::PlanResult stOld = c.pl.stPlan;
		stOld.uSeq = hl.GetPlanRequest().uSeq - 1;
		hl.TakePlan(stOld);
		stOld.uSeq = hl.GetPlanRequest().uSeq;		// the newest request, already answered and in use
		hl.TakePlan(stOld);
		CHighLevelRT::IkResult stOldIk = c.pl.stIk;
		for (int i = 0; i < N; i++) stOldIk.adQ[i] = 0.3;
		hl.TakeIk(stOldIk);							// its IK was used long ago
		const long nPlans = c.pl.nPlans;
		double adAt[N], dMoved = 0;
		memcpy(adAt, l.qr, sizeof(adAt));
		for (int k = 0; k < 50; k++)
		{
			l.Step(goal);
			for (int i = 0; i < N; i++) dMoved = std::max(dMoved, std::fabs(l.qr[i] - adAt[i]));
		}
		snprintf(ac, sizeof(ac), "after three results nobody waits for: %ld more plans asked for, the reference moved %.1e rad", c.pl.nPlans - nPlans, dMoved);
		Check(c.pl.nPlans == nPlans && dMoved == 0.0 && hl.GetStatus().bAtRest, "a result that answers no open request is not taken", ac);
		ReportContract(l, p);
	}

	{
		printf("G  the plan of a second goal is late while the arm moves, and is back after the plan asked for in its place\n");
		Pair c;
		if (!c.Init(argv[1], p)) return 2;
		CHighLevelRT& hl = c.hl;
		c.pl.nLatePlan = 2;
		c.pl.nLateDelay = H + 2;
		Loop l(hl, p.adQr);
		l.pPlanner = &c.pl;
		hl.StartTrack(p.adQr, l.qd);
		double p0[3], goal1[3], goal2[3], tool[3];
		hl.ToolPosition(p.adQr, p0);
		for (int k = 0; k < 3; k++) { goal1[k] = p0[k] + d1[k]; goal2[k] = goal1[k]; }
		goal2[1] += 0.10;
		int nRest = -1;
		for (int k = 0; k < 20000 && nRest < 0; k++)
		{
			l.Step(k < 400 ? goal1 : goal2);
			if (k > 400 && hl.GetStatus().bAtRest) nRest = k;
		}
		hl.ToolPosition(l.q, tool);
		snprintf(ac, sizeof(ac), "%ld plan late, %ld asked for; %.4f mm from the second goal", hl.GetStatus().nPlanLate, c.pl.nPlans,
		         Norm3(tool, goal2) * 1e3);
		Check(hl.GetStatus().nPlanLate == 1 && c.pl.nPlans == 3 && nRest > 0 && Norm3(tool, goal2) < p.dTol,
		      "the late plan is not used when it is back after the one asked for in its place", ac);
		ReportContract(l, p);
	}
	{
		printf("G  every plan is back one period after the period it was asked for\n");
		Pair c;
		if (!c.Init(argv[1], p)) return 2;
		CHighLevelRT& hl = c.hl;
		c.pl.nLatePlan = -1;
		c.pl.nLateDelay = H + 1;
		Loop l(hl, p.adQr);
		l.pPlanner = &c.pl;
		hl.StartTrack(p.adQr, l.qd);
		double p0[3], goal[3], dMoved = 0;
		hl.ToolPosition(p.adQr, p0);
		for (int k = 0; k < 3; k++) goal[k] = p0[k] + d1[k];
		int nAtRest = 0, nFalse = 0;
		for (int k = 0; k < 300; k++)
		{
			if (!l.Step(goal)) nFalse++;
			if (hl.GetStatus().bAtRest) nAtRest++;
			for (int i = 0; i < N; i++) dMoved = std::max(dMoved, std::fabs(l.qr[i] - p.adQr[i]) + std::fabs(l.qdr[i]));
		}
		snprintf(ac, sizeof(ac), "in 300 periods: %ld plans late, at rest in %d, Update() FALSE in %d, the reference moved %.1e",
		         hl.GetStatus().nPlanLate, nAtRest, nFalse, dMoved);
		Check(hl.GetStatus().nPlanLate == 59 && nAtRest == 0 && nFalse == 0 && dMoved == 0.0,
		      "while every plan is late the reference holds, and the arm does not count as at rest", ac);
	}

	/* ---------------------------------------------------------------- H */
	{
		printf("H  a second goal in period 1, while the IK of the first is three periods away\n");
		Pair c;
		if (!c.Init(argv[1], p)) return 2;
		CHighLevelRT& hl = c.hl;
		c.pl.nIkDelay = 3;
		Loop l(hl, p.adQr);
		l.pPlanner = &c.pl;
		hl.StartTrack(p.adQr, l.qd);
		double p0[3], goal1[3], goal2[3], tool[3];
		hl.ToolPosition(p.adQr, p0);
		for (int k = 0; k < 3; k++) { goal1[k] = p0[k] + d1[k]; goal2[k] = goal1[k]; }
		goal2[1] += 0.10;
		l.Step(goal1);
		const CHighLevelRT::IkResult stFirst = c.pl.stIk;		// the IK of the first goal, as the planner has it
		l.Step(goal2);										// accepted: it replaces the first
		hl.TakeIk(stFirst);									// the first IK comes back now: of a goal that is no longer the accepted one
		int nConv = -1, nRest = -1;
		for (int k = 2; k < 20000 && nRest < 0; k++)
		{
			l.Step(goal2);
			if (hl.GetStatus().bIkConverged && nConv < 0) nConv = k;
			if (hl.GetStatus().bAtRest) nRest = k;
		}
		hl.ToolPosition(l.q, tool);
		const CHighLevelRT::Status& st = hl.GetStatus();
		snprintf(ac, sizeof(ac), "%ld accepted, %ld converged (in period %d), %ld plans; %.4f mm from the second goal, %.1f mm from the first",
		         st.nAccepted, st.nConverged, nConv, c.pl.nPlans, Norm3(tool, goal2) * 1e3, Norm3(tool, goal1) * 1e3);
		Check(st.nAccepted == 2 && st.nConverged == 1 && nConv == 1 + 3 && c.pl.nPlans == 1 && nRest > 0 && Norm3(tool, goal2) < p.dTol,
		      "the IK of a goal that was replaced is not used", ac);
		ReportContract(l, p);
	}

	{
		printf("H  a second goal in the period in which the IK of the first is back; then a goal whose IK takes 250 periods\n");
		Pair c;
		if (!c.Init(argv[1], p)) return 2;
		CHighLevelRT& hl = c.hl;
		Loop l(hl, p.adQr);
		l.pPlanner = &c.pl;
		hl.StartTrack(p.adQr, l.qd);
		double p0[3], goal1[3], goal2[3], goal3[3], tool[3];
		hl.ToolPosition(p.adQr, p0);
		for (int k = 0; k < 3; k++) { goal1[k] = p0[k] + d1[k]; goal2[k] = goal3[k] = goal1[k]; }
		goal2[1] += 0.10;
		goal3[1] += 0.15;
		l.Step(goal1);										// its IK is given back before the next Update()
		l.Step(goal2);										// ... which accepts the second goal
		const bool bUsed = hl.GetStatus().bIkConverged || hl.GetStatus().nConverged != 0 || !hl.GetStatus().bIkActive;
		int nConv = -1, nRest = -1;
		for (int k = 2; k < 20000 && nRest < 0; k++)
		{
			l.Step(goal2);
			if (hl.GetStatus().bIkConverged && nConv < 0) nConv = k;
			if (hl.GetStatus().bAtRest) nRest = k;
		}
		hl.ToolPosition(l.q, tool);
		snprintf(ac, sizeof(ac), "the first IK used: %d; %ld converged (in period %d), %ld plans; %.4f mm from the second goal",
		         (int)bUsed, hl.GetStatus().nConverged, nConv, c.pl.nPlans, Norm3(tool, goal2) * 1e3);
		Check(!bUsed && hl.GetStatus().nConverged == 1 && nConv == 2 && c.pl.nPlans == 1 && nRest > 0 && Norm3(tool, goal2) < p.dTol,
		      "an IK that is back in the period of a new goal is not used", ac);
		c.pl.nIkDelay = 250;
		int nAtRest = 0, nBack = -1;
		for (int k = 0; k < 20000; k++)
		{
			l.Step(goal3);
			if (hl.GetStatus().bIkConverged && nBack < 0) nBack = k;
			if (nBack < 0 && hl.GetStatus().bAtRest) nAtRest++;
			if (nBack >= 0 && hl.GetStatus().bAtRest) break;
		}
		hl.ToolPosition(l.q, tool);
		snprintf(ac, sizeof(ac), "its IK is back %d periods after the goal; at rest in %d of them; then %.4f mm from the goal",
		         nBack, nAtRest, Norm3(tool, goal3) * 1e3);
		Check(nBack == 250 && nAtRest == 0 && Norm3(tool, goal3) < p.dTol, "the arm is not at rest while the IK of the accepted goal is out", ac);
		ReportContract(l, p);
	}

	/* ---------------------------------------------------------------- I */
	{
		printf("I  a second goal whose IK is back while the plan of the first is out\n");
		Pair c;
		if (!c.Init(argv[1], p)) return 2;
		CHighLevelRT& hl = c.hl;
		Loop l(hl, p.adQr);
		l.pPlanner = &c.pl;
		hl.StartTrack(p.adQr, l.qd);
		double p0[3], goal1[3], goal2[3], tool[3];
		hl.ToolPosition(p.adQr, p0);
		for (int k = 0; k < 3; k++) { goal1[k] = p0[k] + d1[k]; goal2[k] = goal1[k]; }
		goal2[1] += 0.10;
		int anNew[4] = { -1, -1, -1, -1 }, nNews = 0, nRest = -1;
		for (int k = 0; k < 20000 && nRest < 0; k++)
		{
			l.Step(k < 2 ? goal1 : goal2);					// the second goal: accepted in period 2, its IK back in period 3
			if (hl.GetStatus().bOtgNewCalc && nNews < 4) anNew[nNews++] = k;
			if (hl.GetStatus().bAtRest) nRest = k;
		}
		hl.ToolPosition(l.q, tool);
		snprintf(ac, sizeof(ac), "plans in use from periods %d and %d (1 + H = %d, 1 + 2 H = %d), %d in all; %ld late; %.4f mm from the second goal",
		         anNew[0], anNew[1], 1 + H, 1 + 2 * H, nNews, hl.GetStatus().nPlanLate, Norm3(tool, goal2) * 1e3);
		Check(nNews == 2 && anNew[0] == 1 + H && anNew[1] == 1 + 2 * H && hl.GetStatus().nPlanLate == 0 && nRest > 0 &&
		      Norm3(tool, goal2) < p.dTol, "one plan at a time: the second is asked for when the first is in use", ac);
		ReportContract(l, p);
	}

	/* ---------------------------------------------------------------- J */
	{
		printf("J  HOME started while the plan of a goal is out\n");
		Pair c;
		if (!c.Init(argv[1], p)) return 2;
		CHighLevelRT& hl = c.hl;
		const double q0[N] = { 0.4, -0.3, -1.2, 0.3, -1.2, 0.5 };
		Loop l(hl, q0);
		l.pPlanner = &c.pl;
		hl.StartTrack(q0, l.qd);
		double p0[3], goal[3];
		hl.ToolPosition(q0, p0);
		for (int k = 0; k < 3; k++) goal[k] = p0[k];
		goal[2] += 0.10;
		for (int k = 0; k < 3; k++) l.Step(goal);			// accepted in 0, IK back in 1, plan asked for in 1 and back in 2: due in 1 + H
		const bool bOut = hl.GetStatus().bPlanAsked;
		hl.StartHome(l.q, l.qd);
		Loop l2(hl, l.q);									// the reference starts again, at the measured posture
		l2.pPlanner = &c.pl;
		int nNew = -1, nNews = 0, nRest = -1;
		double dHeld = 0;
		for (int k = 0; k < 20000 && nRest < 0; k++)
		{
			l2.Step(goal);									// HOME takes no goal
			if (hl.GetStatus().bOtgNewCalc) { nNews++; if (nNew < 0) nNew = k; }
			if (nNew < 0) for (int i = 0; i < N; i++) dHeld = std::max(dHeld, std::fabs(l2.qr[i] - q0[i]) + std::fabs(l2.qdr[i]));
			if (hl.GetStatus().bAtRest) nRest = k;
		}
		double dMax = 0;
		for (int i = 0; i < N; i++) dMax = std::max(dMax, std::fabs(l2.q[i] - p.adQr[i]));
		snprintf(ac, sizeof(ac), "a plan was out at the start: %d; then %d plan(s) in use, the first from period %d of HOME (H = %d), "
		         "before it the reference moved %.1e; |q - q_r| %.1e rad; %ld goals accepted", (int)bOut, nNews, nNew, H, dHeld, dMax,
		         hl.GetStatus().nAccepted);
		Check(bOut && nNews == 1 && nNew == H && dHeld == 0.0 && nRest > 0 && dMax < 1e-9 && hl.GetStatus().nAccepted == 0,
		      "a start drops what was asked of the planner before it", ac);
		ReportContract(l2, p);
	}

	{
		printf("J  HOME started when the IK of a goal has just come back\n");
		Pair c;
		if (!c.Init(argv[1], p)) return 2;
		CHighLevelRT& hl = c.hl;
		const double q0[N] = { 0.4, -0.3, -1.2, 0.3, -1.2, 0.5 };
		Loop l(hl, q0);
		l.pPlanner = &c.pl;
		hl.StartTrack(q0, l.qd);
		double p0[3], goal[3];
		hl.ToolPosition(q0, p0);
		for (int k = 0; k < 3; k++) goal[k] = p0[k];
		goal[2] += 0.10;
		l.Step(goal);
		c.pl.Before(hl, 1);									// the IK is given back ...
		hl.StartHome(l.q, l.qd);							// ... and the motion it belongs to ends before it is used
		Loop l2(hl, l.q);
		l2.pPlanner = &c.pl;
		int nRest = -1;
		long nConv = 0;
		for (int k = 0; k < 20000 && nRest < 0; k++)
		{
			l2.Step(NULL);
			nConv += hl.GetStatus().bIkConverged ? 1 : 0;
			if (hl.GetStatus().bAtRest) nRest = k;
		}
		double dMax = 0;
		for (int i = 0; i < N; i++) dMax = std::max(dMax, std::fabs(l2.q[i] - p.adQr[i]));
		snprintf(ac, sizeof(ac), "%ld IK used after the start, %ld plans asked for; |q - q_r| %.1e rad", nConv, c.pl.nPlans, dMax);
		Check(nConv == 0 && c.pl.nPlans == 1 && nRest > 0 && dMax < 1e-9, "an IK of the motion before the start is not used", ac);
	}
	{
		printf("J  TRACK started in the middle of HOME\n");
		Pair c;
		if (!c.Init(argv[1], p)) return 2;
		CHighLevelRT& hl = c.hl;
		const double q0[N] = { 0.4, -0.3, -1.2, 0.3, -1.2, 0.5 };
		Loop l(hl, q0);
		l.pPlanner = &c.pl;
		hl.StartHome(q0, l.qd);
		for (int k = 0; k < 300; k++) l.Step(NULL);			// its plan is in use, the arm on its way
		double adAt[N], dSpeed = 0, dMoved = 0;
		memcpy(adAt, l.q, sizeof(adAt));
		for (int i = 0; i < N; i++) dSpeed = std::max(dSpeed, std::fabs(l.qdr[i]));
		hl.StartTrack(l.q, l.qd);
		Loop l2(hl, adAt);
		l2.pPlanner = &c.pl;
		bool bFinished = true;
		for (int k = 0; k < 50; k++)
		{
			l2.Step(NULL);
			bFinished = bFinished && hl.GetStatus().nOtg == (int)ruckig::Result::Finished;
			for (int i = 0; i < N; i++) dMoved = std::max(dMoved, std::fabs(l2.qr[i] - adAt[i]) + std::fabs(l2.qdr[i]) + std::fabs(l2.qddr[i]));
		}
		snprintf(ac, sizeof(ac), "the reference had %.3f rad/s at the start; after it, it is %.1e off the measured posture at rest", dSpeed, dMoved);
		Check(dSpeed > 0.1 && dMoved == 0.0 && bFinished, "a start drops the plan in use: the reference holds the measured posture", ac);
	}

	{
		printf("J  TRACK started again while a plan is out and a target and a joint at its bound wait for the next\n");
		const double dOff = p.dLead + 1e-9;
		double dRef3 = 0;				// the reference of J3 one period after J2 came to its bound ...
		int nKd = -1;					// ... in this period
		for (int nRun = 0; nRun < 2; nRun++)
		{
			Pair c;
			if (!c.Init(argv[1], p)) return 2;
			CHighLevelRT& hl = c.hl;
			Loop l(hl, p.adQr);
			l.pPlanner = &c.pl;
			hl.StartTrack(p.adQr, l.qd);
			double p0[3], goal1[3], goal2[3];
			hl.ToolPosition(p.adQr, p0);
			for (int k = 0; k < 3; k++) { goal1[k] = p0[k] + d1[k]; goal2[k] = goal1[k]; }
			goal2[1] += 0.10;
			l.uBlocked = 1u << 1;
			l.adBlockedQ[1] = p.adQr[1];
			l.adBlockedQd[1] = 0.02;
			if (nRun == 0)
			{
				/* the first run finds the period and the reference of J3 */
				for (int k = 0; k < 3000 && nKd < 0; k++) { l.Step(goal1); if (hl.GetStatus().uLeadReset & 2u) nKd = k; }
				l.Step(goal1);
				dRef3 = l.qr[2];
				continue;
			}
			/* the second: J3 is blocked where it comes to its bound one period after J2, when the plan that restarts
			 * J2 is out; the second goal is given in the period of J2, so that its IK is back in that of J3 */
			l.uBlocked |= 1u << 2;
			l.adBlockedQ[2] = dRef3 - (dRef3 > p.adQr[2] ? dOff : -dOff);
			l.adBlockedQd[2] = 0.02;
			unsigned uAtKd = 0;
			for (int k = 0; k <= nKd + 1; k++) { l.Step(k < nKd ? goal1 : goal2); if (k == nKd) uAtKd = hl.GetStatus().uLeadReset; }
			const CHighLevelRT::Status stAt = hl.GetStatus();
			const long nPlans = c.pl.nPlans;
			double adAt[N], dMoved = 0;
			memcpy(adAt, l.q, sizeof(adAt));
			hl.StartTrack(l.q, l.qd);
			Loop l2(hl, adAt);
			l2.pPlanner = &c.pl;
			int nAsked = 0, nFlagged = 0;
			for (int k = 0; k < 50; k++)
			{
				l2.Step(NULL);
				if (hl.GetStatus().bPlanAsked) nAsked++;
				if (hl.GetStatus().uLeadReset) nFlagged++;
				for (int i = 0; i < N; i++) dMoved = std::max(dMoved, std::fabs(l2.qr[i] - adAt[i]) + std::fabs(l2.qdr[i]));
			}
			snprintf(ac, sizeof(ac), "J2 at its bound in period %d (flags %u), J3 in the next (flags %u), a plan out: %d, %ld IK converged; "
			         "after the start: a plan asked for in %d periods, a joint flagged in %d, the reference moved %.1e", nKd, uAtKd,
			         stAt.uLeadReset, (int)stAt.bPlanAsked, stAt.nConverged, nAsked, nFlagged, dMoved);
			Check(nKd > 0 && uAtKd == 2u && stAt.uLeadReset == 6u && stAt.bPlanAsked && stAt.nConverged == 2 && nAsked == 0 &&
			      nFlagged == 0 && c.pl.nPlans == nPlans && dMoved == 0.0, "a start drops what waited for the next plan", ac);
		}
	}

	/* ---------------------------------------------------------------- K */
	{
		printf("K  the plan of the goal comes back with an error of the generator\n");
		Pair c;
		if (!c.Init(argv[1], p)) return 2;
		CHighLevelRT& hl = c.hl;
		c.pl.nFirstPlanResult = -100;						// ruckig::Result::ErrorInvalidInput
		Loop l(hl, p.adQr);
		l.pPlanner = &c.pl;
		hl.StartTrack(p.adQr, l.qd);
		double p0[3], goal[3];
		hl.ToolPosition(p.adQr, p0);
		for (int k = 0; k < 3; k++) goal[k] = p0[k] + d1[k];
		int nFalse = -1, nFalses = 0;
		double dMoved = 0;
		for (int k = 0; k < 2 * H + 4; k++)
		{
			if (!l.Step(goal)) { nFalses++; if (nFalse < 0) nFalse = k; }
			if (k <= 1 + H) for (int i = 0; i < N; i++) dMoved = std::max(dMoved, std::fabs(l.qr[i] - p.adQr[i]) + std::fabs(l.qdr[i]));
		}
		snprintf(ac, sizeof(ac), "Update() FALSE in period %d (1 + H = %d), %d time(s); the reference moved %.1e up to there; "
		         "result %d, at rest %d", nFalse, 1 + H, nFalses, dMoved, hl.GetStatus().nOtg, (int)hl.GetStatus().bAtRest);
		Check(nFalse == 1 + H && nFalses == 1 && dMoved == 0.0, "the error is told in the period the plan was due, the reference stays", ac);
	}

	/* ---------------------------------------------------------------- L */
	{
		printf("L  TRACK without a goal\n");
		Pair c;
		if (!c.Init(argv[1], p)) return 2;
		CHighLevelRT& hl = c.hl;
		Loop l(hl, p.adQr);
		l.pPlanner = &c.pl;
		hl.StartTrack(p.adQr, l.qd);
		int nRest = -1;
		bool bFinished = true;
		double dMoved = 0;
		for (int k = 0; k < 250; k++)
		{
			l.Step(NULL);
			bFinished = bFinished && hl.GetStatus().nOtg == (int)ruckig::Result::Finished;
			if (hl.GetStatus().bAtRest && nRest < 0) nRest = k;
			for (int i = 0; i < N; i++) dMoved = std::max(dMoved, std::fabs(l.qr[i] - p.adQr[i]) + std::fabs(l.qdr[i]));
		}
		snprintf(ac, sizeof(ac), "at rest from period %d (rest window %d); the reference moved %.1e; %ld requests", nRest, p.nRestCyc, dMoved,
		         c.pl.nIks + c.pl.nPlans);
		Check(nRest == p.nRestCyc - 1 && bFinished && dMoved == 0.0 && c.pl.nIks + c.pl.nPlans == 0,
		      "it holds the start posture, at its target and at rest after the rest window, and asks for nothing", ac);
		hl.StartTrack(p.adQr, l.qd);						// at rest; started again
		int nRest2 = -1;
		for (int k = 0; k < 250 && nRest2 < 0; k++) { l.Step(NULL); if (hl.GetStatus().bAtRest) nRest2 = k; }
		snprintf(ac, sizeof(ac), "started again at rest: at rest from period %d after it", nRest2);
		Check(nRest2 == p.nRestCyc - 1, "a start begins the rest window again", ac);
		int nAtRest = 0, nRest3 = -1;
		MovedThenStill(l, NULL, p.dRestQd, &nAtRest, &nRest3);
		snprintf(ac, sizeof(ac), "J1 measured at 1.2 times the rest speed: at rest in %d of 250 periods; at 0.8 times: at rest from period %d",
		         nAtRest, nRest3);
		Check(nAtRest == 0 && nRest3 == p.nRestCyc - 1, "at rest only when |qd| has stayed below its threshold for the rest window", ac);
	}

	/* ---------------------------------------------------------------- M */
	{
		printf("M  the step baseline: the target as the reference\n");
		CHighLevelRT::Params ps = p;
		ps.bStepRef = true;
		Pair c;
		if (!c.Init(argv[1], ps)) return 2;
		CHighLevelRT& hl = c.hl;
		Loop l(hl, ps.adQr);
		l.pPlanner = &c.pl;
		hl.StartTrack(ps.adQr, l.qd);
		double p0[3], goal[3], tool[3];
		hl.ToolPosition(ps.adQr, p0);
		for (int k = 0; k < 3; k++) goal[k] = p0[k] + d1[k];
		int nStep = -1, nRest = -1, nAsked = 0;
		double dBefore = 0, dOff = 0;
		for (int k = 0; k < 400; k++)
		{
			l.Step(goal);
			const CHighLevelRT::Status& st = hl.GetStatus();
			if (st.bIkConverged && nStep < 0) nStep = k;
			for (int i = 0; i < N; i++)
				if (nStep < 0) dBefore = std::max(dBefore, std::fabs(l.qr[i] - ps.adQr[i]));
				else dOff = std::max(dOff, std::fabs(l.qr[i] - st.adTarget[i]) + std::fabs(l.qdr[i]) + std::fabs(l.qddr[i]));
			if (st.bPlanAsked) nAsked++;
			if (st.bAtRest && nRest < 0) nRest = k;
		}
		hl.ToolPosition(l.qr, tool);
		snprintf(ac, sizeof(ac), "the reference steps in period %d, %.1e off (target, 0, 0) from then on, moved %.1e before; the tool of the "
		         "reference %.4f mm from the goal; %d periods with a plan asked for, %ld plans", nStep, dOff, dBefore, Norm3(tool, goal) * 1e3,
		         nAsked, c.pl.nPlans);
		Check(nStep == 1 && dOff == 0.0 && dBefore == 0.0 && Norm3(tool, goal) < ps.dTol && nAsked == 0 && c.pl.nPlans == 0,
		      "the reference is the target from the period its IK is back; no plan is asked for", ac);
		const double goal2[3] = { goal[0], goal[1] + 0.10, goal[2] };
		bool bAtAccept = true;
		int nRest2 = -1;
		for (int k = 0; k < 400 && nRest2 < 0; k++)
		{
			l.Step(goal2);
			if (k == 0) bAtAccept = hl.GetStatus().bAtRest;
			else if (hl.GetStatus().bAtRest) nRest2 = k;
		}
		snprintf(ac, sizeof(ac), "at rest from period %d; in the period of a second goal: %d, and again from period %d after it (rest window "
		         "%d; |qd| of the ideal arm is 0)", nRest, (int)bAtAccept, nRest2, ps.nRestCyc);
		Check(nRest == ps.nRestCyc - 1 && !bAtAccept && nRest2 == ps.nRestCyc - 1,
		      "at rest when |qd| has stayed small for the rest window after a goal", ac);
		int nAtRest = 0, nRest3 = -1;
		MovedThenStill(l, goal2, ps.dRestQd, &nAtRest, &nRest3);
		snprintf(ac, sizeof(ac), "J1 measured at 1.2 times the rest speed: at rest in %d of 250 periods; at 0.8 times: at rest from period %d",
		         nAtRest, nRest3);
		Check(nAtRest == 0 && nRest3 == ps.nRestCyc - 1, "and only then", ac);
	}

	/* ---------------------------------------------------------------- N */
	{
		printf("N  the iterations of the IK\n");
		CHlIkN<N> ik;
		if (!ik.Init(argv[1], p.acTool)) return 2;
		double p0[3], goal[2][3];
		{
			CHighLevelRT hl;
			if (!hl.Init(argv[1], TS, p)) return 2;
			hl.ToolPosition(p.adQr, p0);
		}
		for (int k = 0; k < 3; k++) { goal[0][k] = p0[k] + d1[k]; goal[1][k] = p0[k]; }
		goal[1][0] -= 2.0;									// out of reach, behind the arm
		/* the third run: the goal out of reach again, with limits 0.30 rad around q_r, 0.04 rad after the margin */
		double adLo[3][N], adHi[3][N];
		for (int i = 0; i < N; i++)
		{
			adLo[0][i] = adLo[1][i] = p.adLimLo[i];
			adHi[0][i] = adHi[1][i] = p.adLimHi[i];
			adLo[2][i] = p.adQr[i] - 0.30;
			adHi[2][i] = p.adQr[i] + 0.30;
		}
		/* one step from q_r without its bound and far inside the limits, against the damped least-squares step from
		 * the singular values of J: sum of v_i sigma_i / (sigma_i^2 + lambda^2) u_i^T e */
		{
			double adWideLo[N], adWideHi[N];
			for (int i = 0; i < N; i++) { ik.vQk[i] = p.adQr[i]; adWideLo[i] = -1e9; adWideHi[i] = 1e9; }
			RigidBodyDynamics::Math::Vector3d vE;
			ik.Kinematics();
			ik.Error(goal[0], &vE);
			ik.Jacobian();
			const Eigen::JacobiSVD<Eigen::MatrixXd> svd(Eigen::MatrixXd(ik.mG), Eigen::ComputeThinU | Eigen::ComputeThinV);
			Eigen::VectorXd vDamped = Eigen::VectorXd::Zero(N), vPlain = Eigen::VectorXd::Zero(N);
			for (int j = 0; j < 3; j++)
			{
				const double dS = svd.singularValues()[j], dUe = svd.matrixU().col(j).dot(Eigen::Vector3d(vE));
				vDamped += svd.matrixV().col(j) * (dS / (dS * dS + p.dLambda * p.dLambda) * dUe);
				vPlain  += svd.matrixV().col(j) * (dUe / dS);
			}
			ik.Step(vE, p.dLambda, 1e9, adWideLo, adWideHi, 0.0);
			double dDiff = 0;
			for (int i = 0; i < N; i++) dDiff = std::max(dDiff, std::fabs(ik.vQk[i] - p.adQr[i] - vDamped[i]));
			snprintf(ac, sizeof(ac), "the step from q_r (%.3f rad) is %.1e rad off the damped step; the step without damping is %.1e rad "
			         "from it", vDamped.norm(), dDiff, (vDamped - vPlain).norm());
			Check(dDiff < 1e-12 && (vDamped - vPlain).norm() > 1e-6, "a step of the IK is the damped least-squares step", ac);
		}
		double dMaxStep = 0, dMinInside = 1e9;
		int nAtBound = 0, nClipped = 0, anIter[3] = { 0, 0, 0 };
		for (int g = 0; g < 3; g++)
		{
			for (int i = 0; i < N; i++) ik.vQk[i] = p.adQr[i];
			for (int n = 0; n < p.nIterMax; n++)
			{
				RigidBodyDynamics::Math::Vector3d vE;
				ik.Kinematics();
				if (ik.Error(goal[g < 2 ? g : 1], &vE) < p.dTol) break;
				ik.Jacobian();
				const RigidBodyDynamics::Math::VectorNd vBefore = ik.vQk;
				ik.Step(vE, p.dLambda, p.dStepMax, adLo[g], adHi[g], p.dMargin);
				const double dStep = (ik.vQk - vBefore).norm();
				dMaxStep = std::max(dMaxStep, dStep);
				if (dStep > p.dStepMax - 1e-9) nAtBound++;
				for (int i = 0; i < N; i++)
				{
					const double dIn = std::min(ik.vQk[i] - (adLo[g][i] + p.dMargin), (adHi[g][i] - p.dMargin) - ik.vQk[i]);
					dMinInside = std::min(dMinInside, dIn);
					if (dIn == 0.0) nClipped++;
				}
				anIter[g]++;
			}
		}
		snprintf(ac, sizeof(ac), "%d and %d iterations; the largest step %.6f rad (bound %.2f), %d at the bound", anIter[0], anIter[1], dMaxStep,
		         p.dStepMax, nAtBound);
		Check(anIter[0] < 20 && anIter[1] == p.nIterMax && dMaxStep <= p.dStepMax + 1e-12 && nAtBound > 0, "a step of the IK is at most Delta_max", ac);
		snprintf(ac, sizeof(ac), "every iterate at least %.1e rad inside the limits minus the margin; %d times a joint on that border", dMinInside,
		         nClipped);
		Check(dMinInside >= 0.0 && nClipped > 0, "the iterates stay inside the joint limits minus the margin", ac);
	}

	printf("\n%s: %d check(s) failed\n", g_nFail ? "FAIL" : "PASS", g_nFail);
	return g_nFail ? 1 : 0;
}
