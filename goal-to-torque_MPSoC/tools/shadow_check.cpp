/*****************************************************************************
*	Name: shadow_check.cpp
*	Description: The second controller of --shadow (shadow/ShadowControl.h)
*	against the controller it follows, on a PC.
*
*	    bin/shadow_check [-f FILE] [-o VECTORS]
*
*	  -f FILE     the configuration, found as by indy7_ctrl
*	  -o VECTORS  the periods of the run as a file (shadow/shadow_vectors.h)
*	              for the controller as the firmware of the R5 builds it
*
*	A control task in small: the controller of the application, set up from
*	the configuration as CRobotIndy7 sets it up and driven through every
*	period as TaskTrajectory.cpp and TaskDynamics.cpp drive it, on an arm
*	that follows its reference with a small error. It goes through gravity
*	compensation with the hold on and off, HOME, TRACK with goals in and out
*	of reach, a goal that the inverse kinematics solves in several periods
*	with a joint at its limit margin, an arm held fast (the lead bound and
*	the torque limit act), a stop, a new ready posture, a joint move, TRACK
*	with the goal threshold 0, starts before and after the update of a
*	period and in a period without one, periods with two measured states,
*	and a planner task that has no time for some periods: once for the IK
*	of a goal, once for a plan until it is late. Each period goes the way of
*	--shadow: through CShadowTask and
*	the Shadow in slot of a link (CR5Link) to the R5's side of the slots
*	(CSimR5Slots), there into a second controller built from the numbers of
*	shadow/ShadowParams.h, and its answer back through the Shadow out slot.
*
*	Checked: the torque and the reference of the second controller are those
*	of the first, bit for bit, in every period it follows, also when the
*	IK of a goal is back periods later; it is apart exactly in the periods
*	after a payload that did not arrive or was not given, in the period whose
*	start is not told, and from the period in which its plan comes into use
*	and the first controller's does not, until the next start;
*	the payloads it does not follow (a JOINT TEST, a period number that went
*	back, a torque that is no number); the numbers and their check value;
*	what the control task counts of the answers.
*	The same code and compiler on both sides here: a difference is a fault
*	of the following, not of the arithmetic.
*
*	Exit code 0 when no check failed.
*****************************************************************************/
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <set>
#include <string>
#include <vector>
#include <unistd.h>

#include "FullDynControllerRT.h"
#include "GoalSlot.h"
#include "TrajectoryRT.h"
#include "base/Config.h"
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "shadow/ModelCode.h"
#include "shadow/ShadowControl.h"
#include "shadow/ShadowParams.h"
#include "shadow/ShadowTask.h"
#include "shadow/shadow_vectors.h"
#include "sim/SimR5Slots.h"

static_assert((int)SHADOW_VECTORS_NUMBERS == (int)SHADOW_NUMBERS, "shadow_vectors.h and ShadowControl.h");

namespace {

const int N = IPC_CTRL_DOF;
typedef CControllerFullDynamicsRT C;

int g_nFailed = 0;

void Expect(bool abOk, const char* apcWhat)
{
	if (abOk)
		return;
	g_nFailed++;
	printf("  FAILED: %s\n", apcWhat);
}

/* what the scenario asks for in a period, before its update (the commands of the operator and the goal of a test
 * program) and after it (a test program that goes from HOME to TRACK), and what the State of the period is like */
enum
{
	A_TRACK = 1 << 0, A_HOME = 1 << 1, A_HOME_TO = 1 << 2, A_STOP = 1 << 3, A_GOAL = 1 << 4, A_HOLD = 1 << 5,
	A_TRACK_AFTER = 1 << 6,		// after the update: end the motion and start TRACK
	A_TRACK_AT_REST = 1 << 7,	// the same, in the period in which the motion ends by itself (HOME at rest)
	A_FRICTION = 1 << 8,		// the motion started in this period has the friction feedforward
	A_DELTA0 = 1 << 9,			// the TRACK started in this period takes every goal (goal threshold 0: test 4)
	A_READY = 1 << 10,			// the measured posture becomes the ready posture (menu 7)
	A_NOT_OP = 1 << 11,			// the State that the Trajectory part has is not operational: commands, no update
	A_SPLIT = 1 << 12			// the Trajectory part has the State of the period before, the control law this one's
};

/* the controller of the control task */
class CApu
{
public:
	~CApu() { delete m_pTask; delete m_pCtl; }

	/* as CRobotIndy7::InitController and InitTrajectory, then the link to an R5 whose side the scenario plays;
	 * auCheck: the check value of the controller */
	bool Init(const CConfig& acConfig, uint32_t auCheck)
	{
		const ST_CONFIG_SYSTEM& stS = acConfig.System();
		m_pCtl = new C(stS.strURDFPath, (unsigned)N);
		m_pCtl->EnableRTMode(TRUE);
		m_pCtl->SetDeadline(500000);
		m_pCtl->SetControlMode(C::eGravityCompensation);
		if (!m_pCtl->Init())
			return false;
		m_pCtl->SetControlGains(std::vector<double>(stS.vKp.begin(), stS.vKp.begin() + N),
		                        std::vector<double>(stS.vKd.begin(), stS.vKd.begin() + N));
		std::vector<double> vKf(N, 0.0), vB(N, 0.0), vTauMax;
		for (int i = 0; i < N && i < (int)stS.vKf.size(); i++)
			vKf[i] = stS.vKf[i];
		for (int i = 0; i < N && i < (int)stS.vRotorB.size(); i++)
			vB[i] = stS.vRotorB[i];
		m_pCtl->SetFrictionFF(vKf);
		m_pCtl->SetRotorInertia(vB);
		m_pCtl->SetFrictionFFGate(stS.nFfGate != 0);
		for (const ST_CONFIG_ECAT_SLAVE& s : acConfig.Slaves())
			if (s.nSlaveType == 0 && (int)vTauMax.size() < N)
				vTauMax.push_back(s.dTorLimitU);
		vTauMax.resize(N, 0.0);
		m_pCtl->SetTorqueLimits(vTauMax);
		m_pCtl->Enable(TRUE);

		CHighLevelRT::Params stP = CHighLevelRT::DefaultParams();
		stP.dDelta     = stS.dHlDelta;
		stP.dLambda    = stS.dHlLambda;
		stP.dStepMax   = stS.dHlStepMax;
		stP.dTol       = stS.dHlTol;
		stP.nPlanAhead = stS.nHlPlanAhead;
		stP.nIterMax   = stS.nHlIterGoal;
		stP.dVelMax    = stS.dHlVelMax;
		stP.dAccMax    = stS.dHlAccMax;
		stP.dJerkMax   = stS.dHlJerkMax;
		stP.dMargin    = stS.dHlMargin;
		stP.dLead      = stS.dHlLead;
		stP.bStepRef   = (stS.nHlStepBaseline != 0);
		for (int i = 0; i < N && i < (int)stS.vHlReadyQ.size(); i++)
			stP.adQr[i] = stS.vHlReadyQ[i];
		if (FALSE == m_cTraj.Init(stS.strURDFPath.c_str(), (double)acConfig.Tasks()[1].nPeriod * 1e-9, stP))
			return false;
		m_cTraj.SetGoalSlot(&m_cGoal);
		const uint32_t uCfgCrc = 0x600DC0DEu;
		memset((void*)m_auRegion, 0, sizeof(m_auRegion));
		(void)m_cR5.Start(m_auRegion, uCfgCrc, 10);
		ipc_shadow_out_t stOld;				// the answer to a run of the control task before this one: not taken
		memset(&stOld, 0, sizeof(stOld));
		stOld.ctrl_crc = auCheck;
		stOld.flags = IPC_SHADOW_OUT_CTRL;
		stOld.tau_diff = 5.0;
		stOld.n_missed = 1000;
		m_cR5.PublishShadow(stOld);
		m_cLink.UseRegion(m_auRegion, "the test");
		if (FALSE == m_cLink.Start((uint32_t)N, uCfgCrc, (RTTIME)acConfig.Tasks()[1].nPeriod, 150000))
			return false;
		m_pTask = new CShadowTask(&m_cLink, auCheck);
		m_dDelta = stS.dHlDelta;
		m_vQ.assign(N, 0.0);
		m_vQd.assign(N, 0.0);
		m_vTor.assign(N, 0.0);
		m_vTau.assign(N, 0.0);
		return true;
	}

	/* one period. adQ, adQd: the measured state, which the law has; adTq, adTqd: the state that the Trajectory
	 * part has (the same, or with A_SPLIT an earlier one). The payload of the period is in the Shadow in slot
	 * afterwards, unless the period had two states. *apbRef: the reference m_adQr.. is of this period. FALSE: a
	 * start was refused, or the law failed (the scenario is wrong). */
	bool Period(uint32_t auPeriod, const double* adQ, const double* adQd, const double* adTq, const double* adTqd,
	            unsigned auAsk, const double* adGoal, const double* adHomeTarget, bool* apbRef)
	{
		char acWhy[160] = "";
		bool bStarted = true;
		m_bEnded = false;
		const BOOL bFf = (auAsk & A_FRICTION) ? TRUE : FALSE;
		const bool bTrajOp = !(auAsk & A_NOT_OP);
		const bool bLawOp = bTrajOp || (auAsk & A_SPLIT);	// the State of this period, which the law has, is operational

		/* the Trajectory part (trajectory_cycle): its State, the commands, the goal, the step */
		m_pTask->TrajState(auPeriod, adTq, adTqd);
		if (auAsk & A_STOP)
			m_cTraj.End(CTrajectoryRT::eAborted);
		if (auAsk & A_HOLD)
			m_pCtl->m_bStickyEnable = !m_pCtl->m_bStickyEnable.load();
		if (auAsk & A_READY)
			m_cTraj.HighLevel().SetReady(adTq);
		if (auAsk & A_TRACK)
			bStarted = StartTrack(adTq, adTqd, auAsk, acWhy, sizeof(acWhy)) && bStarted;
		if (auAsk & A_HOME)
			bStarted = m_cTraj.Start(true, adTq, adTqd, bFf, acWhy, sizeof(acWhy)) && bStarted;
		if (auAsk & A_HOME_TO)
			bStarted = m_cTraj.Start(true, adTq, adTqd, bFf, acWhy, sizeof(acWhy), adHomeTarget) && bStarted;
		if (auAsk & A_GOAL)
			m_cGoal.Write(adGoal);
		bool bRef = false, bHome = false;
		m_bUpdated = false;
		if (bTrajOp && m_cTraj.IsRunning())
		{
			const BOOL bOk = m_cTraj.Update(adTq, adTqd, m_adQr, m_adQdr, m_adQddr);
			// the planner task, here: what the period asked for is back for the next, unless it has no time
			if (m_nStall > 0)
				m_nStall--;
			else
				while (m_cTraj.ServePlanner())
					;
			m_pTask->TrajUpdated(m_cTraj);
			m_bUpdated = true;
			bRef = (bOk != FALSE);
			bHome = (m_cTraj.GetMotion() == CTrajectoryRT::eMotionHome);
			m_bEnded = (m_cTraj.Pending() != CTrajectoryRT::eRunning);
			if (m_bEnded)
			{
				m_cTraj.End(m_cTraj.Pending());
				bRef = false;
			}
		}
		const BOOL bRefFf = m_cTraj.GetFrictionFF();
		if ((auAsk & A_TRACK_AFTER) || ((auAsk & A_TRACK_AT_REST) && m_bEnded))
		{
			m_cTraj.End(CTrajectoryRT::eAborted);
			bStarted = StartTrack(adTq, adTqd, auAsk, acWhy, sizeof(acWhy)) && bStarted;
		}
		if (!bStarted)
			printf("  the scenario asked for a start that was refused in period %u: %s\n", (unsigned)auPeriod, acWhy);

		/* the Dynamics part (dynamics_cycle), with an operational State: the mode follows the reference, then the law */
		const bool bLaw = bLawOp && m_pCtl->IsEnabled();
		BOOL bOk = TRUE;
		if (bLawOp)
		{
			const C::eControlMode eNow = m_pCtl->GetControlMode();
			if (bRef)
			{
				const C::eControlMode eWant = bHome ? C::eHome : C::eTrack;
				if (eNow != eWant)
					m_pCtl->SetControlMode(eWant);
				C::Reference stRef;
				memset(&stRef, 0, sizeof(stRef));
				for (int i = 0; i < N; i++)
				{
					stRef.adQ[i] = m_adQr[i];
					stRef.adQd[i] = m_adQdr[i];
					stRef.adQdd[i] = m_adQddr[i];
				}
				stRef.bFrictionFF = bRefFf;
				m_pCtl->SetReference(stRef);
			}
			else if (eNow == C::eTrack || eNow == C::eHome)
				m_pCtl->SetControlMode(C::eGravityCompensation);
		}
		if (bLaw)
		{
			for (int i = 0; i < N; i++)
			{
				m_vQ[i] = adQ[i];
				m_vQd[i] = adQd[i];
			}
			bOk = m_pCtl->Update(m_vQ, m_vQd, m_vTor, m_vTau);
		}
		m_uClamp = bLaw ? m_pCtl->GetLastClamp() : 0u;
		m_pTask->Finish(true, auPeriod, bLaw ? adQ : NULL, bLaw ? adQd : NULL, m_cTraj, *m_pCtl, bRef, bRefFf != FALSE,
		                m_vTau.data());
		*apbRef = bRef;
		return bStarted && bOk != FALSE;
	}

	/* a cycle in which the two parts did not both have a new State (the control task ran over a period), with
	 * nothing started or updated. adLawQ: the state of the law, NULL: it was not updated. -> a payload was told (*apIn) */
	bool Cycle(bool abTrajNew, uint32_t auTrajPeriod, const double* adTq, const double* adTqd, bool abDynNew,
	           uint32_t auDynPeriod, const double* adLawQ, const double* adLawQd, ipc_shadow_in_t* apIn)
	{
		if (abTrajNew)
			m_pTask->TrajState(auTrajPeriod, adTq, adTqd);
		m_pTask->Finish(abDynNew, auDynPeriod, adLawQ, adLawQd, m_cTraj, *m_pCtl, false, false, m_vTau.data());
		return m_cR5.TakeShadow(apIn);
	}

	/* the answer that is in the Shadow out slot now, as the next period of the control task would take it */
	void TakeAnswer()
	{
		ipc_shadow_out_t stOut;
		if (m_cLink.TakeShadow(&stOut))
			m_pTask->Take(stOut);
	}
	const CShadowTask::Counts& Counts() const { return m_pTask->GetCounts(); }

	CSimR5Slots		m_cR5;					// the R5's side of the slots, for the scenario
	CTrajectoryRT	m_cTraj;
	double			m_adQr[N] = {}, m_adQdr[N] = {}, m_adQddr[N] = {};	// 0 until the first reference: they go into the vectors
	bool			m_bEnded = false;		// the motion ended by itself in this period (HOME at rest, or an error)
	bool			m_bUpdated = false;		// the high-level controller was updated in this period
	int				m_nStall = 0;			// updates from now after which the planner is not served
	unsigned int	m_uClamp = 0;			// bit i: the torque limit acted on joint i in this period

private:
	/* as CRobotIndy7::StartHighLevelTrack: the goal threshold of the run, then the start */
	bool StartTrack(const double* adQ, const double* adQd, unsigned auAsk, char* astrWhy, size_t auLen)
	{
		m_cTraj.HighLevel().SetDelta((auAsk & A_DELTA0) ? 0.0 : m_dDelta);
		return FALSE != m_cTraj.Start(false, adQ, adQd, (auAsk & A_FRICTION) ? TRUE : FALSE, astrWhy, auLen);
	}

	alignas(64) volatile uint32_t m_auRegion[IPC_OFF_END / 4];
	CR5Link			m_cLink;
	CShadowTask*	m_pTask = NULL;
	C*				m_pCtl = NULL;
	CGoalSlot		m_cGoal;
	double			m_dDelta = 0.0;
	std::vector<double> m_vQ, m_vQd, m_vTor, m_vTau;
};

/* the answers to payloads made by hand: what the second controller does not follow, and what it tells.
 * acApuHl: the high-level controller of the control task, for its solution of a goal */
void ByHand(const char* astrURDF, const double* adNumbers, CHighLevelRT& acApuHl)
{
	CShadowControl cS;
	Expect(cS.Init(astrURDF, adNumbers, 0x12345678u) && !cS.Init(astrURDF, adNumbers, 1u), "built once, not twice");
	ipc_shadow_in_t stIn;
	ipc_shadow_out_t stOut;
	memset(&stIn, 0, sizeof(stIn));
	stIn.delta = adNumbers[SHADOW_N_DELTA];
	for (int i = 0; i < N; i++)
	{
		stIn.q[i] = adNumbers[SHADOW_N_READY + i] + 0.3;		// a posture that gravity loads
		stIn.q_ready[i] = adNumbers[SHADOW_N_READY + i];
	}
	stIn.period = 50;
	stIn.flags = IPC_SHADOW_CTRL | IPC_SHADOW_HOLD;
	stIn.ctrl_mode = (uint32_t)C::eGravityCompensation;
	cS.Step(stIn, &stOut);
	Expect(stOut.period == 50 && stOut.ctrl_crc == 0x12345678u && stOut.flags == IPC_SHADOW_OUT_CTRL && stOut.n_missed == 0,
	       "the first payload: the law, the period and the check value in the answer, nothing missed");
	Expect(stOut.tau_diff > 1.0, "a torque of the APU that is not the law's is told (0 against gravity)");
	for (int i = 0; i < N; i++)
		stIn.tau[i] = stOut.tau[i];
	stIn.period = 51;
	cS.Step(stIn, &stOut);
	Expect(stOut.flags == IPC_SHADOW_OUT_CTRL && stOut.tau_diff == 0.0, "the same state again: the same torque");
	for (int i = 0; i < N; i++)
		stIn.tau[i] = stOut.tau[i] + 2.0;
	stIn.period = 52;
	cS.Step(stIn, &stOut);
	Expect(std::fabs(stOut.tau_diff - 2.0) < 1e-9, "a torque of the APU above the law's is told as one below is");
	for (int i = 0; i < N; i++)
		stIn.tau[i] = stOut.tau[i];
	stIn.tau[N - 1] = std::nan("");
	stIn.tau[0] += 1.0;
	stIn.period = 53;
	cS.Step(stIn, &stOut);
	Expect(stOut.tau_diff != stOut.tau_diff, "a torque of the APU that is no number is no agreement, whichever joint");
	stIn.tau[N - 1] = stOut.tau[N - 1];
	stIn.tau[0] = stOut.tau[0];

	stIn.period = 54;
	stIn.ctrl_mode = (uint32_t)C::eJointTest;
	cS.Step(stIn, &stOut);
	Expect(stOut.flags == IPC_SHADOW_OUT_APART, "a JOINT TEST is not followed: no law, apart");
	stIn.period = 55;
	stIn.ctrl_mode = (uint32_t)C::eGravityCompensation;
	cS.Step(stIn, &stOut);
	Expect(stOut.flags == (IPC_SHADOW_OUT_CTRL | IPC_SHADOW_OUT_APART), "and apart after it, until a start");

	stIn.period = 55 + 4;
	cS.Step(stIn, &stOut);
	Expect(stOut.n_missed == 3, "three periods without a payload are counted");
	stIn.period = 7;
	cS.Step(stIn, &stOut);
	Expect(stOut.n_missed == 4 && (stOut.flags & IPC_SHADOW_OUT_APART), "a period number that went back counts once");

	stIn.period = 8;
	stIn.flags |= IPC_SHADOW_START;
	stIn.starts = 1;
	cS.Step(stIn, &stOut);
	Expect(stOut.flags == IPC_SHADOW_OUT_CTRL, "a start ends it");
	stIn.period = 9;
	stIn.flags &= ~(uint32_t)IPC_SHADOW_START;
	stIn.starts = 2;
	cS.Step(stIn, &stOut);
	Expect(stOut.flags & IPC_SHADOW_OUT_APART, "a start that no payload told (the count went on) leaves it apart");
	stIn.period = 10;
	cS.Step(stIn, &stOut);
	Expect(stOut.flags & IPC_SHADOW_OUT_APART, "and so in the periods after it");

	/* TRACK from this posture, then the arm half a radian away from the reference: the torque limits act */
	stIn.period = 11;
	stIn.starts = 3;
	stIn.flags = IPC_SHADOW_START | IPC_SHADOW_TRAJ | IPC_SHADOW_CTRL | IPC_SHADOW_REF | IPC_SHADOW_HOLD;
	stIn.ctrl_mode = (uint32_t)C::eTrack;
	cS.Step(stIn, &stOut);
	Expect(stOut.flags == (IPC_SHADOW_OUT_TRAJ | IPC_SHADOW_OUT_CTRL), "TRACK started and followed");
	stIn.period = 12;
	stIn.flags &= ~(uint32_t)IPC_SHADOW_START;
	for (int i = 0; i < N; i++)
		stIn.q[i] += 0.5;
	cS.Step(stIn, &stOut);
	int nAtLimit = 0, nAbove = 0;
	for (int i = 0; i < N; i++)
	{
		if (std::fabs(stOut.tau[i]) == adNumbers[SHADOW_N_TAU_MAX + i]) nAtLimit++;
		if (!(std::fabs(stOut.tau[i]) <= adNumbers[SHADOW_N_TAU_MAX + i])) nAbove++;
	}
	Expect(nAtLimit >= 3 && nAbove == 0, "the torque limits are those of the numbers: three joints or more at theirs, none above");

	CShadowControl cNone;
	cNone.Step(stIn, &stOut);
	Expect(stOut.flags == IPC_SHADOW_OUT_FAULT && stOut.period == 12, "not built: every answer is a fault");

	/* the planner: TRACK with a goal from period 0; the IK of the goal comes with the payload of period 10, and
	 * the plan to it is this controller's own, in use H periods later */
	const int H = (int)adNumbers[SHADOW_N_PLAN_AHEAD];
	for (int nCase = 0; nCase < 3; nCase++)
	{
		CShadowControl cK;
		ipc_shadow_in_t stK;
		ipc_shadow_out_t stA;
		memset(&stK, 0, sizeof(stK));
		stK.delta = adNumbers[SHADOW_N_DELTA];
		stK.starts = 1;
		stK.goal[0] = 0.4;
		stK.goal[2] = 0.5;
		double adStart[N], dToIk = 0.0;
		for (int i = 0; i < N; i++)
		{
			adStart[i] = stK.q[i] = stK.q_ready[i] = adNumbers[SHADOW_N_READY + i];
			stK.ik_q[i] = adStart[i] + 0.05;		// below the lead bound: the measured state stays where it is
		}
		int nFirstApart = -1, nFirstMoved = -1;
		bool bBuilt = (FALSE != cK.Init(astrURDF, adNumbers, 1u));
		for (int k = 0; k < 3000 && bBuilt; k++)
		{
			stK.period = 100u + (uint32_t)k;
			stK.flags = IPC_SHADOW_TRAJ | IPC_SHADOW_GOAL | (k == 0 ? (uint32_t)IPC_SHADOW_START : 0u);
			if (k == 10)
				stK.flags |= IPC_SHADOW_IK;
			if ((nCase == 0 && k == 10 + H) || (nCase == 2 && k == 12))
				stK.flags |= IPC_SHADOW_PLAN;
			cK.Step(stK, &stA);
			while (cK.ServePlanner())		// its planner task, here
				;
			if ((stA.flags & IPC_SHADOW_OUT_APART) && nFirstApart < 0)
				nFirstApart = k;
			dToIk = 0.0;
			for (int i = 0; i < N; i++)
			{
				if (stA.q_ref[i] != adStart[i] && nFirstMoved < 0)
					nFirstMoved = k;
				dToIk = std::fmax(dToIk, std::fabs(stA.q_ref[i] - stK.ik_q[i]));
			}
		}
		if (nCase == 0)
			Expect(bBuilt && nFirstApart < 0 && nFirstMoved == 10 + H + 1 && dToIk < 1e-9,
			       "the IK that the APU's update took is the target: the reference leaves H periods later, for it, and "
			       "not before (the second controller does not solve the goal itself)");
		else if (nCase == 1)
			Expect(bBuilt && nFirstApart == 10 + H, "a plan that comes into use in a period in which the APU's did not: apart");
		else
			Expect(bBuilt && nFirstApart == 12, "a plan of the APU in a period in which none comes into use here: apart");
	}

	/* the IK of a goal that its own planner found against the APU's (ik_diff): once per goal, when both solutions
	 * are there, whichever came first; not while it is apart. The APU's update takes its IK in period 10. */
	{
		double adP[3], adGoalA[3], adGoalB[3], adIkA[N], adIkB[N];
		acApuHl.ToolPosition(&adNumbers[SHADOW_N_READY], adP);
		const double adToA[3] = { 0.05, 0.03, -0.04 }, adToB[3] = { -0.06, 0.08, 0.02 };
		for (int j = 0; j < 3; j++)
		{
			adGoalA[j] = adP[j] + adToA[j];
			adGoalB[j] = adP[j] + adToB[j];
		}
		const bool bSolved = FALSE != acApuHl.SolveIk(adGoalA, adIkA, NULL, NULL) &&
		                     FALSE != acApuHl.SolveIk(adGoalB, adIkB, NULL, NULL);
		for (int nCase = 0; nCase < 4; nCase++)
		{
			CShadowControl cK;
			ipc_shadow_in_t stK;
			ipc_shadow_out_t stA;
			memset(&stK, 0, sizeof(stK));
			memset(&stA, 0, sizeof(stA));
			stK.delta = adNumbers[SHADOW_N_DELTA];
			stK.starts = 1;
			for (int i = 0; i < N; i++)
				stK.q[i] = stK.q_ready[i] = adNumbers[SHADOW_N_READY + i];
			uint32_t auCompared[20] = {};
			double adDiff[20] = {};
			uint32_t uFlags10 = 0;
			bool bBuilt = bSolved && (FALSE != cK.Init(astrURDF, adNumbers, 1u));
			for (int k = 0; k < 20 && bBuilt; k++)
			{
				const bool bGoalB = (nCase == 3 && k >= 5);		// case 3: another goal from period 5 ...
				stK.period = 100u + (uint32_t)k;
				stK.flags = IPC_SHADOW_TRAJ | IPC_SHADOW_GOAL | (k == 0 ? (uint32_t)IPC_SHADOW_START : 0u);
				for (int j = 0; j < 3; j++)
					stK.goal[j] = bGoalB ? adGoalB[j] : adGoalA[j];
				if (nCase == 2 && k >= 5)
					stK.starts = 2;								// case 2: a start that no payload told
				if (k == 10)
				{
					stK.flags |= IPC_SHADOW_IK;
					for (int i = 0; i < N; i++)
						stK.ik_q[i] = bGoalB ? adIkB[i] : adIkA[i];
					if (nCase == 1)								// case 1: the APU has another solution
					{
						stK.ik_q[1] -= 0.5;
						stK.ik_q[4] += 0.25;
					}
				}
				cK.Step(stK, &stA);
				if (!(bGoalB && k < 12))						// ... which its planner has no time for until period 12
					while (cK.ServePlanner())
						;
				auCompared[k] = stA.n_ik_compared;
				adDiff[k] = stA.ik_diff;
				if (k == 10)
					uFlags10 = stA.flags;
			}
			if (nCase == 0)
				Expect(bBuilt && auCompared[9] == 0 && auCompared[10] == 1 && adDiff[10] == 0.0 && auCompared[19] == 1,
				       "the IK of its own planner is that of the APU's: compared when the APU's update takes its IK, once");
			else if (nCase == 1)
				Expect(bBuilt && auCompared[10] == 1 && std::fabs(adDiff[10] - 0.5) < 1e-12 && adDiff[19] == adDiff[10],
				       "another solution of the APU is told: the largest difference of a joint, above or below");
			else if (nCase == 2)
				Expect(bBuilt && (uFlags10 & IPC_SHADOW_OUT_APART) && auCompared[19] == 0,
				       "apart, its goal may be another than the APU's: not compared");
			else
				Expect(bBuilt && auCompared[10] == 0 && auCompared[12] == 0 && auCompared[13] == 1 && adDiff[13] == 0.0 &&
				       auCompared[19] == 1,
				       "the IK of the APU before its own: compared when its own is back, and not with that of the goal before");
		}
	}

	/* what the control task counts of the answers (CShadowTask::Take): another check value is not compared; a
	 * difference above the limit or no number differs; apart is not compared; the times of the parts that ran */
	CShadowTask cTask(NULL, 0x1111u);
	memset(&stOut, 0, sizeof(stOut));
	stOut.n_missed = 1000;				// what the R5 counted before this run of the control task
	stOut.ctrl_crc = 0x2222u;
	stOut.flags = IPC_SHADOW_OUT_CTRL;
	stOut.tau_diff = 5.0;
	cTask.Take(stOut);
	Expect(cTask.GetCounts().nAnswers == 1 && cTask.GetCounts().nOtherCheck == 1 && cTask.GetCounts().nCompared == 0,
	       "an answer with another check value is counted and not compared");
	stOut.ctrl_crc = 0x1111u;
	stOut.tau_diff = CShadowTask::TAU_SAME;
	stOut.ctrl_ns = 40000;
	cTask.Take(stOut);
	stOut.tau_diff = 2.0 * CShadowTask::TAU_SAME;
	stOut.flags = IPC_SHADOW_OUT_CTRL | IPC_SHADOW_OUT_TRAJ;
	stOut.traj_ns = 300000;
	stOut.ctrl_ns = 50000;
	cTask.Take(stOut);
	Expect(cTask.GetCounts().nCompared == 2 && cTask.GetCounts().nDiffer == 1 &&
	       cTask.GetCounts().dDiffMax == 2.0 * CShadowTask::TAU_SAME, "a difference above the limit differs, one at it does not");
	stOut.flags = IPC_SHADOW_OUT_CTRL | IPC_SHADOW_OUT_APART | IPC_SHADOW_OUT_FAULT;
	stOut.tau_diff = 9.0;
	stOut.traj_ns = 0;
	stOut.ctrl_ns = 45000;
	stOut.n_missed = 1007;
	cTask.Take(stOut);
	Expect(cTask.GetCounts().nCompared == 2 && cTask.GetCounts().nApart == 1 && cTask.GetCounts().nFault == 1 &&
	       cTask.GetCounts().uMissed == 7 && cTask.GetCounts().dDiffMax == 2.0 * CShadowTask::TAU_SAME,
	       "an answer apart is counted and not compared");
	stOut.flags = IPC_SHADOW_OUT_CTRL;
	stOut.tau_diff = std::nan("");
	cTask.Take(stOut);
	stOut.tau_diff = 1.0;
	cTask.Take(stOut);
	Expect(cTask.GetCounts().nCompared == 4 && cTask.GetCounts().nDiffer == 3 &&
	       cTask.GetCounts().dDiffMax != cTask.GetCounts().dDiffMax, "a difference that is no number differs, and stays the largest");
	Expect(cTask.GetCounts().nTraj == 1 && cTask.GetCounts().nCtrl == 5 && cTask.GetCounts().uTrajMaxNs == 300000 &&
	       cTask.GetCounts().uCtrlMaxNs == 50000 && cTask.GetCounts().dTrajSumNs == 300000.0 &&
	       cTask.GetCounts().dCtrlSumNs == 225000.0 && cTask.GetCounts().dBothMaxNs == 350000.0,
	       "the times of the high-level controller and of the law, where they ran");
	stOut.n_missed = 2;
	cTask.Take(stOut);
	const uint32_t uAgain = cTask.GetCounts().uMissed;
	stOut.n_missed = 5;
	cTask.Take(stOut);
	Expect(uAgain == 0 && cTask.GetCounts().uMissed == 3, "a count of the R5 that went back (it started again) is counted from there");

	/* what its planner did: the counts from the first answer on, a time when its count shows a new plan or IK */
	{
		CShadowTask cPl(NULL, 0x1111u);
		ipc_shadow_out_t stP;
		memset(&stP, 0, sizeof(stP));
		stP.ctrl_crc = 0x1111u;
		stP.n_plans = 40; stP.plan_us = 5000; stP.n_iks = 20; stP.ik_us = 9000; stP.n_plan_late = 2;	// of a run before
		stP.n_ik_compared = 7; stP.ik_diff = 3.0;
		cPl.Take(stP);
		const CShadowTask::Counts& c = cPl.GetCounts();
		Expect(c.uR5Plans == 0 && c.uR5Iks == 0 && c.uR5PlanLate == 0 && c.uR5PlanMaxUs == 0 && c.uR5IkMaxUs == 0 &&
		       c.uIkCompared == 0 && c.nIkDiffer == 0 && c.dIkDiffMax == 0.0,
		       "the planner of the R5: what it did before the first answer here is not counted");
		stP.n_plans = 41; stP.plan_us = 900;
		cPl.Take(stP);
		stP.plan_us = 2500;								// no new plan: not a time of this run
		stP.n_iks = 21; stP.ik_us = 400;
		cPl.Take(stP);
		stP.n_plans = 43; stP.plan_us = 700; stP.n_iks = 22; stP.ik_us = 300; stP.n_plan_late = 3;
		cPl.Take(stP);
		stP.ik_us = 7000;								// no new IK since the last answer
		cPl.Take(stP);
		Expect(c.uR5Plans == 3 && c.uR5Iks == 2 && c.uR5PlanLate == 1 && c.uR5PlanMaxUs == 900 && c.uR5IkMaxUs == 400,
		       "then its plans, its IK and its late plans by its counts, and the longest plan and IK that an answer showed");
		stP.n_ik_compared = 8; stP.ik_diff = CShadowTask::IK_SAME;
		cPl.Take(stP);
		stP.ik_diff = 0.5;								// no goal compared since the last answer
		cPl.Take(stP);
		stP.n_ik_compared = 10; stP.ik_diff = 2.0 * CShadowTask::IK_SAME;
		cPl.Take(stP);
		Expect(c.uIkCompared == 3 && c.nIkDiffer == 1 && c.dIkDiffMax == 2.0 * CShadowTask::IK_SAME,
		       "the goals compared by its count; a difference above the limit differs, one at it does not, and one "
		       "without a new goal is none");
		stP.n_ik_compared = 11; stP.ik_diff = std::nan("");
		cPl.Take(stP);
		stP.n_ik_compared = 12; stP.ik_diff = 1.0;
		cPl.Take(stP);
		Expect(c.uIkCompared == 5 && c.nIkDiffer == 3 && c.dIkDiffMax != c.dIkDiffMax,
		       "a difference that is no number differs, and stays the largest");
		stP.ctrl_crc = 0x2222u;
		stP.n_plans = 44; stP.plan_us = 8000; stP.n_ik_compared = 13;
		cPl.Take(stP);
		Expect(c.uR5Plans == 3 && c.uR5PlanMaxUs == 900 && c.uIkCompared == 5,
		       "an answer with another check value does not count for it");
	}

	/* the check value: every number counts, each of its 8 bytes */
	const uint32_t uCrc = ShadowNumbersCrc(adNumbers);
	int nSame = 0;
	for (int i = 0; i < SHADOW_NUMBERS; i++)
		for (int nByte = 0; nByte < 8; nByte++)
		{
			double adOther[SHADOW_NUMBERS];
			memcpy(adOther, adNumbers, sizeof(adOther));
			((unsigned char*)&adOther[i])[nByte] ^= 0x10;
			if (ShadowNumbersCrc(adOther) == uCrc)
				nSame++;
		}
	Expect(nSame == 0, "the check value changes with every byte of every number");

	/* the check value of the model takes its numbers on a grid of 2^-40. The reader of the board (aarch64) gives
	 * E(1,1) and E(2,0) of the joint frames of bodies 2 and 5 as dBoard below, the reader of a PC (x86) as dPc,
	 * 1.6e-23 away (2026-11-19): the same model, whichever of the two the reader of this run gives. A frame 1e-11 m
	 * away is another. */
	{
		RigidBodyDynamics::Model cModel;
		model_code::ModelCode stHere, stPc, stBoard, stOther;
		bool bOk = RigidBodyDynamics::Addons::URDFReadFromFile(astrURDF, &cModel, false, false) &&
		           model_code::ModelCodeOf(cModel, &stHere);
		const double dBoard = -0x1.a7939a720126dp-53, dPc = -0x1.a79398p-53;
		double dApart = 0.0;		// this reader from the nearer of the two
		for (unsigned uBody = 2; bOk && uBody <= 5; uBody += 3)
			dApart = std::fmax(dApart, std::fmin(std::fabs(cModel.X_T[uBody].E(1, 1) - dBoard),
			                                     std::fabs(cModel.X_T[uBody].E(1, 1) - dPc)));
		for (unsigned uBody = 2; bOk && uBody <= 5; uBody += 3)
			cModel.X_T[uBody].E(1, 1) = cModel.X_T[uBody].E(2, 0) = dPc;
		bOk = bOk && model_code::ModelCodeOf(cModel, &stPc);
		for (unsigned uBody = 2; bOk && uBody <= 5; uBody += 3)
			cModel.X_T[uBody].E(1, 1) = cModel.X_T[uBody].E(2, 0) = dBoard;
		bOk = bOk && model_code::ModelCodeOf(cModel, &stBoard);
		if (bOk)
			cModel.X_T[2].r[2] += 1e-11;
		bOk = bOk && model_code::ModelCodeOf(cModel, &stOther);
		Expect(bOk && dApart < 1e-20 && stBoard.strBodies != stPc.strBodies && stBoard.uCrc == stPc.uCrc &&
		       stHere.uCrc == stPc.uCrc && stOther.uCrc != stPc.uCrc,
		       "the check value of the model: the same for the numbers of the board's reader and of a PC's, another for "
		       "a frame 1e-11 m away");
	}
}

/* ShadowNumbers() of the configuration astrCfg with the line of apcKey replaced by apcLine: is it refused? */
bool Refused(const TSTRING& astrCfg, const char* apcKey, const char* apcLine)
{
	char acPath[] = "/tmp/shadow_check_XXXXXX";
	const int nFd = mkstemp(acPath);
	if (nFd < 0)
		return false;
	close(nFd);
	std::ifstream cIn(astrCfg);
	std::ofstream cOut(acPath);
	for (std::string strLine; std::getline(cIn, strLine); )
		cOut << (strLine.compare(0, strlen(apcKey), apcKey) == 0 ? apcLine : strLine) << "\n";
	cOut.close();
	CConfig cConfig;
	double adNumbers[SHADOW_NUMBERS];
	const bool bRefused = (FALSE != cConfig.Load(acPath, FALSE)) && !ShadowNumbers(cConfig, adNumbers);
	unlink(acPath);
	return bRefused;
}

} // namespace

int main(int argc, char** argv)
{
	TSTRING strCfg, strOut;
	int nOpt;
	while ((nOpt = getopt(argc, argv, "f:o:")) != -1)
	{
		if (nOpt == 'f') strCfg = optarg;
		else if (nOpt == 'o') strOut = optarg;
		else
		{
			fprintf(stderr, "usage: %s [-f FILE] [-o VECTORS]\n", argv[0]);
			return 2;
		}
	}
	CConfig cConfig;
	strCfg = CConfig::Find(strCfg, argv[0]);
	if (FALSE == cConfig.Load(strCfg, FALSE))
		return 2;
	const char* const pcURDF = cConfig.System().strURDFPath.c_str();
	double adNumbers[SHADOW_NUMBERS];
	CApu cApu;
	CShadowControl cShadow;
	if (!ShadowNumbers(cConfig, adNumbers) || !cApu.Init(cConfig, ShadowNumbersCrc(adNumbers)) ||
	    !cShadow.Init(pcURDF, adNumbers, ShadowNumbersCrc(adNumbers)))
	{
		fprintf(stderr, "%s: the controller cannot be built from it\n", strCfg.c_str());
		return 2;
	}
	ByHand(pcURDF, adNumbers, cApu.m_cTraj.HighLevel());
	Expect(Refused(strCfg, "ROBOT_DOF=", "ROBOT_DOF=5") && Refused(strCfg, "NO_OF_TASKS=", "NO_OF_TASKS=1") &&
	       !Refused(strCfg, "ROBOT_DOF=", "ROBOT_DOF=6"),
	       "no numbers from a configuration with another arm or without a control task");

	/* the scenario: what is asked for in which period (0: the first) */
	const double dTs = adNumbers[SHADOW_N_TS];
	const int P_IDLE_SPLIT = 100, P_HOLD_OFF = 150, P_HOLD_ON = 220, P_HOME = 320, P_TRACK = 4500, P_STOP = P_TRACK + 4200,
	          P_READY = P_STOP + 50, P_MOVE = P_STOP + 100, P_BOTH = P_MOVE + 2500, P_DROP = P_BOTH + 300,
	          P_STOP2 = P_DROP + 300, P_TRACK2 = P_STOP2 + 50, P_SPLIT = P_TRACK2 + 420, P_STOP3 = P_SPLIT + 30,
	          P_TRACK3 = P_STOP3 + 20, P_END = P_TRACK3 + 40, FIRST = 1000;
	/* the last goal before P_STOP: its plan is late on the APU; the goal before it: its IK is back three periods later */
	const int P_PLAN_LATE = ((P_STOP - 38) / 400) * 400 + 37, P_IK_LATER = P_PLAN_LATE - 400, H = cApu.m_cTraj.HighLevel().GetParams().nPlanAhead;
	/* the goal of the TRACK that starts at P_BOTH: its plan is late on the second controller */
	const int P_R5_LATE = ((P_BOTH - 237) / 400 + 1) * 400 + 237;
	const int HARD_GOAL = 3, HELD_FROM = 150, HELD_TO = 750;	// the goal number; the arm held fast, periods after it
	const double adHard[3] = { -0.9, 0.2, -0.5 };	// from the tool position of the ready posture: behind the arm
	const CHighLevelRT::Params stP = cApu.m_cTraj.HighLevel().GetParams();
	double adQ[N], adQd[N], adQBefore[N], adQdBefore[N], adBase[N], adP0[3], adMove[N];
	for (int i = 0; i < N; i++)
	{
		adBase[i] = adNumbers[SHADOW_N_READY + i] + 0.25 * ((i % 2) ? 1.0 : -1.0);	// away from the ready posture
		adMove[i] = adNumbers[SHADOW_N_READY + i] + 0.2;
		adQ[i] = adBase[i];
		adQd[i] = 0.0;
	}
	cApu.m_cTraj.HighLevel().ToolPosition(&adNumbers[SHADOW_N_READY], adP0);

	std::vector<shadow_vector_t> vVectors;
	std::set<int> setApart, setExpected;
	long nLaw = 0, nLawDiff = 0, nTraj = 0, nTrajDiff = 0, nStartsTold = 0, nAfter = 0, nLost = 0, nGoals = 0, nFault = 0,
	     nNone = 0, nCount = 0, nState = 0, nTimes = 0, nLead = 0, nClamp = 0, nNearIgnored = 0, nNearTaken0 = 0,
	     nIkLater = 0, nIkTold = 0, nIkCompared = 0, nIkDiff = 0;
	int nAccepted = -1;			// the period of the last goal that was accepted
	int nShadowStall = 0;		// steps of the second controller from now after which its planner is not served
	ipc_shadow_out_t stLast;	// its last answer
	memset(&stLast, 0, sizeof(stLast));
	uint32_t uMissed = 0;
	bool bScenario = true, bRefBefore = false, bHardSolved = false;
	int nHomeEnd = -1, nHardAt = -1;
	uint64_t uRng = 20261118;
	for (int k = 0; k < P_END; k++)
	{
		unsigned uAsk = 0;
		double adGoal[3] = { 0.0, 0.0, 0.0 };
		if (k == P_IDLE_SPLIT) uAsk |= A_SPLIT;		// while nothing moves: the payload has the state of the law
		if (k == P_HOLD_OFF || k == P_HOLD_ON) uAsk |= A_HOLD;
		if (k == P_HOME)   uAsk |= A_HOME | A_FRICTION;
		if (k > P_HOME && nHomeEnd < 0)
			uAsk |= A_TRACK_AT_REST;		// as a test program goes from HOME to TRACK: after the update that ended HOME
		if (k == P_STOP || k == P_STOP2 || k == P_STOP3) uAsk |= A_STOP;
		if (k == P_READY)  uAsk |= A_READY;
		if (k == P_MOVE)   uAsk |= A_HOME_TO;
		if (k == P_BOTH)   uAsk |= A_HOME | A_TRACK_AFTER | A_FRICTION;	// its update is HOME's, whose start is not told
		if (k == P_TRACK2) uAsk |= A_TRACK | A_DELTA0 | A_NOT_OP;		// a start in a period without update and law
		if (k == P_SPLIT)  uAsk |= A_SPLIT;								// an update from another state than the law's
		if (k == P_TRACK3) uAsk |= A_TRACK | A_NOT_OP | A_SPLIT;		// a start from another state than the law's
		const bool bTrack = (nHomeEnd >= 0 && k > nHomeEnd && k < P_STOP) || (k > P_BOTH && k < P_STOP2) ||
		                    (k > P_TRACK2 && k < P_STOP3);
		const bool bNear = (k % 400 == 237);
		if (bTrack && (k % 400 == 37 || bNear))
		{
			// a new goal around the tool position of the ready posture; one in five out of reach, one far
			// behind the arm; the one 200 periods later within the goal threshold of it
			static double s_adLast[3];
			if (!bNear)
			{
				for (int j = 0; j < 3; j++)
				{
					uRng = uRng * 6364136223846793005ULL + 1442695040888963407ULL;
					s_adLast[j] = adP0[j] + 0.24 * ((double)(uRng >> 40) / (double)(1 << 24) - 0.5);
				}
				if (++nGoals % 5 == 4)
					s_adLast[0] = adP0[0] + 1.5;
				if (nGoals == HARD_GOAL)
				{
					for (int j = 0; j < 3; j++)
						s_adLast[j] = adP0[j] + adHard[j];
					nHardAt = k;
				}
			}
			for (int j = 0; j < 3; j++)
				adGoal[j] = s_adLast[j] + (bNear ? 0.3 * adNumbers[SHADOW_N_DELTA] : 0.0);
			uAsk |= A_GOAL;
		}

		/* the measured state: the reference of the period before with a small error; without a motion a slow
		 * swing that fades to a tremble before the next start (above and below the speed that unlocks the
		 * hold); after the far goal the arm is held fast for a while */
		const double dT = k * dTs;
		const double dSwing = ((k < 250) ? 1.0 : (k < 300 ? (300 - k) / 50.0 : 0.0)) * 0.06 + 1e-6;
		const bool bHeld = (nHardAt >= 0 && k >= nHardAt + HELD_FROM && k < nHardAt + HELD_TO);
		for (int i = 0; i < N; i++)
		{
			adQBefore[i] = adQ[i];
			adQdBefore[i] = adQd[i];
			if (bHeld)
				adQd[i] = 0.0;
			else if (bRefBefore)
			{
				adQ[i] = cApu.m_adQr[i] + 2e-4 * std::sin(13.0 * dT + i);
				adQd[i] = cApu.m_adQdr[i] + 2e-4 * 13.0 * std::cos(13.0 * dT + i);
				adBase[i] = cApu.m_adQr[i];
			}
			else
			{
				adQ[i] = adBase[i] + dSwing * std::sin(4.4 * dT * (i + 1) / 3.0);
				adQd[i] = dSwing * 4.4 * (i + 1) / 3.0 * std::cos(4.4 * dT * (i + 1) / 3.0);
			}
		}

		/* the planner task has no time: for three periods from the goal, so that its IK is back later; and for
		 * H + 1 periods from the request of a plan, so that the plan is late and asked for again */
		if (k == P_IK_LATER)
			cApu.m_nStall = 3;
		if (k == P_PLAN_LATE + 1)
			cApu.m_nStall = H + 1;
		if (k == P_R5_LATE + 1)			// the same for the planner of the second controller: the APU's plan is in time
			nShadowStall = H + 1;

		/* what the Trajectory part has: the state of the period, or with A_SPLIT that of the period before; at
		 * P_SPLIT only its angles, at P_TRACK3 only its speeds (either alone makes two states) */
		const bool bSplit = (uAsk & A_SPLIT) != 0;
		const double* const adTrajQ = (bSplit && k != P_TRACK3) ? adQBefore : adQ;
		const double* const adTrajQd = (bSplit && k != P_SPLIT) ? adQdBefore : adQd;
		bScenario = cApu.Period((uint32_t)(FIRST + k), adQ, adQd, adTrajQ, adTrajQd, uAsk, adGoal, adMove,
		                        &bRefBefore) && bScenario;
		if (cApu.m_bEnded && nHomeEnd < 0)
			nHomeEnd = k;
		if (cApu.m_uClamp != 0)
			nClamp++;
		if (cApu.m_bUpdated)
		{
			// what the scenario is to reach in the high-level controller
			const CTrajectoryRT::Sample& s = cApu.m_cTraj.GetSample();
			if (s.uLeadReset != 0)
				nLead++;
			if (s.uFlags & CTrajectoryRT::HL_GOAL_ACCEPTED)
				nAccepted = k;
			if ((s.uFlags & CTrajectoryRT::HL_IK_CONVERGED) && k > nAccepted + 1)
				nIkLater++;
			if (bNear && (uAsk & A_GOAL) && !(s.uFlags & CTrajectoryRT::HL_GOAL_ACCEPTED))
				nNearIgnored++;
			if (bNear && (uAsk & A_GOAL) && k > P_TRACK2 && (s.uFlags & CTrajectoryRT::HL_GOAL_ACCEPTED))
				nNearTaken0++;		// in the TRACK with the threshold 0, after a goal of its own
			if ((s.uFlags & CTrajectoryRT::HL_IK_CONVERGED) && s.nIkIter > stP.nIterMax / 2)
				for (int i = 0; i < N; i++)
					if (s.adTarget[i] == stP.adLimLo[i] + stP.dMargin || s.adTarget[i] == stP.adLimHi[i] - stP.dMargin)
						bHardSolved = true;
		}
		/* the R5's side: the payload of the period out of its slot, through the second controller, the answer
		 * into its slot, where the next period of the control task takes it */
		if (k == P_DROP)
			continue;				// the R5 does not take this payload: the next one replaces it in the slot
		ipc_shadow_in_t stIn;
		if (!cApu.m_cR5.TakeShadow(&stIn))
		{
			nNone++;				// the period had two measured states: no payload
			continue;
		}
		const ipc_shadow_in_t* const pIn = &stIn;
		if (pIn->flags & IPC_SHADOW_START) nStartsTold++;
		if (pIn->flags & IPC_SHADOW_START_AFTER) nAfter++;
		if (pIn->flags & IPC_SHADOW_STARTS_LOST) nLost++;
		if (pIn->starts != cApu.m_cTraj.LastStart().uCount)
			nCount++;
		{
			// the state of the payload: that of the law, or without the law that of the Trajectory part
			const bool bLaw = (pIn->flags & IPC_SHADOW_CTRL) != 0;
			if (memcmp(pIn->q, bLaw ? adQ : adTrajQ, sizeof(pIn->q)) != 0 ||
			    memcmp(pIn->qd, bLaw ? adQd : adTrajQd, sizeof(pIn->qd)) != 0)
				nState++;
		}
		ipc_shadow_out_t stOut;
		cShadow.Step(*pIn, &stOut);
		cApu.m_cR5.PublishShadow(stOut);
		// its planner task, here: what the period asked for is back for the next, unless it has no time
		const bool bStalled = (nShadowStall > 0);
		if (bStalled)
			nShadowStall--;
		else
			while (cShadow.ServePlanner())
				;
		const bool bApart = (stOut.flags & IPC_SHADOW_OUT_APART) != 0;
		if (bApart)
			setApart.insert(k);
		if ((k == P_BOTH) || (k > P_DROP && k < P_TRACK2) || k > P_SPLIT || (k >= P_PLAN_LATE + 1 + H && k < P_MOVE) ||
		    (k >= P_R5_LATE + 1 + H && k < P_TRACK2))
			setExpected.insert(k);
		if ((pIn->flags & IPC_SHADOW_IK) && setExpected.count(k) == 0)
			nIkTold++;				// a goal whose IK the APU's update took while the second controller is to follow
		if (stOut.n_ik_compared != stLast.n_ik_compared)
		{
			nIkCompared++;
			if (stOut.ik_diff != 0.0)
				nIkDiff++;
		}
		stLast = stOut;
		if (stOut.flags & IPC_SHADOW_OUT_FAULT)
			nFault++;
		if (((stOut.flags & IPC_SHADOW_OUT_TRAJ) != 0) != (stOut.traj_ns != 0) ||
		    ((stOut.flags & IPC_SHADOW_OUT_CTRL) != 0) != (stOut.ctrl_ns != 0))
			nTimes++;
		if ((stOut.flags & IPC_SHADOW_OUT_CTRL) && !bApart)
		{
			nLaw++;
			if (stOut.tau_diff != 0.0 || memcmp(stOut.tau, pIn->tau, sizeof(stOut.tau)) != 0)
				nLawDiff++;
		}
		if ((stOut.flags & IPC_SHADOW_OUT_TRAJ) && !bApart)
		{
			nTraj++;
			if (memcmp(stOut.q_ref, cApu.m_adQr, sizeof(stOut.q_ref)) != 0)
				nTrajDiff++;
		}
		uMissed = stOut.n_missed;
		shadow_vector_t stV;
		memset(&stV, 0, sizeof(stV));
		stV.in = *pIn;
		for (int i = 0; i < N; i++)
			stV.q_ref[i] = cApu.m_adQr[i];
		stV.out_flags = stOut.flags;
		stV.planner_stalled = bStalled ? 1u : 0u;
		vVectors.push_back(stV);
	}

	cApu.TakeAnswer();
	const CShadowTask::Counts& stC = cApu.Counts();
	Expect(bScenario && nHomeEnd > P_HOME && nHomeEnd < P_TRACK, "the scenario ran: every start accepted, HOME at rest in time");
	Expect(nGoals >= 10 && nFault == 0 && bHardSolved && nLead > 0 && nClamp > 0 && nNearIgnored > 0 && nNearTaken0 == 1,
	       "and reached what it is for: ten goals or more, one solved in more than half the iterations allowed with "
	       "a joint at its margin, the lead bound and the torque limit acted, a goal within the threshold ignored and, "
	       "with the threshold 0, taken");
	Expect(nIkLater == 1 && cApu.m_cTraj.PlansLate() == 1,
	       "and the planner without time: the IK of one goal back more than a period after it, one plan late");
	Expect(P_R5_LATE > P_BOTH && P_R5_LATE + 1 + H < P_DROP && stLast.n_plan_late == 1 && stLast.n_plans > 10 &&
	       stLast.n_iks == (uint32_t)cApu.m_cTraj.HighLevel().GetIkRequest().uSeq && stLast.plan_us > 0 && stLast.ik_us > 0,
	       "the planner of the second controller: one plan late, and in its answers the plans it calculated and "
	       "every goal of the APU's controller solved once more");
	Expect(nIkTold >= 10 && nIkCompared == nIkTold && stLast.n_ik_compared == (uint32_t)nIkTold && nIkDiff == 0,
	       "and for every goal whose IK the APU's update took while it follows, its own planner has the same solution");
	Expect(nLaw > 10000 && nLawDiff == 0, "the torque of the second controller is the torque of the first, in every period it follows");
	Expect(nTraj > 6000 && nTrajDiff == 0, "and so is its reference");
	Expect(nTimes == 0, "a time of the high-level controller and of the law in the periods in which they ran, and only there");
	Expect(nNone == 2, "no payload for the two periods with two measured states");
	Expect(setApart == setExpected, "apart in the period whose start is not told, after a period without a payload, and from the "
	       "period of a plan that only one of the two controllers has in time, until the next start");
	Expect(uMissed == 3, "three payloads missed");
	Expect(stC.nPayloads == (long)vVectors.size() + 1 && stC.nWithheld == 2 && stC.nAnswers == (long)vVectors.size() &&
	       stC.nOtherCheck == 0 && stC.uMissed == 3 && stC.nApart == (long)setApart.size() && stC.nFault == 0 &&
	       stC.nCompared == nLaw && stC.nDiffer == 0 && stC.dDiffMax == 0.0 && stC.nTraj > 6000 && stC.nCtrl > 10000,
	       "the control task counts the same: its payloads (one did not arrive), the answers, apart, compared, none differs");
	Expect(stC.uR5PlanLate == 1 && stC.uR5Plans == stLast.n_plans && stC.uR5Iks == stLast.n_iks && stC.uR5PlanMaxUs > 0 &&
	       stC.uR5IkMaxUs > 0 && stC.uIkCompared == (uint32_t)nIkTold && stC.nIkDiffer == 0 && stC.dIkDiffMax == 0.0,
	       "and what the planner of the second controller did: its plans, the goals it solved, each with the solution "
	       "of this one, one plan late");
	Expect(nStartsTold == 5 && nAfter == 2 && nLost == 1 && nCount == 0 && nState == 0,
	       "five starts told, two after the update of their period, one start lost; the count of the starts and "
	       "the measured state of the law in every payload");

	/* the control task ran over a period: a cycle in which only one part, or none, had a new State */
	{
		const uint32_t uOver = 4000000u;
		const long nTold = stC.nPayloads;
		double adA[N], adB[N], adZero[N] = {};
		for (int i = 0; i < N; i++)
		{
			adA[i] = 0.1 * (i + 1);
			adB[i] = -0.05 * (i + 1);
		}
		ipc_shadow_in_t stOne;
		memset(&stOne, 0, sizeof(stOne));
		Expect(cApu.Cycle(true, uOver + 1, adA, adZero, false, uOver, NULL, NULL, &stOne) && stOne.period == uOver + 1 &&
		       memcmp(stOne.q, adA, sizeof(adA)) == 0 && !(stOne.flags & IPC_SHADOW_CTRL),
		       "a new State for the Trajectory part only: its period and its state are told, no law");
		Expect(!cApu.Cycle(false, 0, NULL, NULL, false, uOver + 1, NULL, NULL, &stOne) && stC.nPayloads == nTold + 1,
		       "a new State for neither part: nothing is told");
		Expect(cApu.Cycle(false, 0, NULL, NULL, true, uOver + 3, adB, adZero, &stOne) && stOne.period == uOver + 3 &&
		       memcmp(stOne.q, adB, sizeof(adB)) == 0 && stC.nPayloads == nTold + 2,
		       "a new State for the Dynamics part only: its period and the state of the law are told");
	}
	printf("  %ld periods with the law followed (%ld differ), %ld with the reference (%ld differ); apart in %zu periods; "
	       "%ld goals, the IK of %ld compared (%ld differ); the lead bound acted in %ld periods, the torque limit in %ld; "
	       "an IK back later for %ld goal, %ld plan late\n",
	       nLaw, nLawDiff, nTraj, nTrajDiff, setApart.size(), nGoals, nIkCompared, nIkDiff, nLead, nClamp, nIkLater,
	       cApu.m_cTraj.PlansLate());

	if (!strOut.empty())
	{
		shadow_vectors_head_t stHead;
		memset(&stHead, 0, sizeof(stHead));
		stHead.magic = SHADOW_VECTORS_MAGIC;
		stHead.periods = (uint32_t)vVectors.size();
		stHead.crc = ShadowNumbersCrc(adNumbers);
		memcpy(stHead.numbers, adNumbers, sizeof(stHead.numbers));
		FILE* const pf = fopen(strOut.c_str(), "wb");
		const bool bOk = pf != NULL && fwrite(&stHead, sizeof(stHead), 1, pf) == 1 &&
		                 fwrite(vVectors.data(), sizeof(shadow_vector_t), vVectors.size(), pf) == vVectors.size();
		if (pf == NULL || fclose(pf) != 0 || !bOk)
		{
			perror(strOut.c_str());
			return 2;
		}
		printf("  %zu periods written to %s\n", vVectors.size(), strOut.c_str());
	}
	printf("%s: %d checks failed\n", g_nFailed == 0 ? "PASS" : "FAIL", g_nFailed);
	return g_nFailed == 0 ? 0 : 1;
}
