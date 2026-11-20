/*****************************************************************************
*	Name: ShadowControl.cpp
*	Description: The controller of the control task a second time. See
*	ShadowControl.h.
*****************************************************************************/
#include "ShadowControl.h"

#include <cmath>
#include <cstring>

/* the largest |a - b| over the joints; a NaN stays: no later joint takes its place */
static double
LargestDifference(const double* adA, const double* adB)
{
	double dDiff = 0.0;
	for (int i = 0; i < CShadowControl::DOF; i++)
	{
		const double d = std::fabs(adA[i] - adB[i]);
		if (d > dDiff || d != d)
			dDiff = d;
	}
	return dDiff;
}

CShadowControl::CShadowControl()
	: m_pCtl(NULL), m_nPlansLate(0), m_nLateSeen(0), m_uIkApu(0), m_uIkCompared(0), m_dIkDiff(0.0), m_uCrc(0),
	  m_uPeriod(0), m_uStarts(0), m_uMissed(0), m_bFirst(true), m_bApart(false)
{
	memset(&m_stPlanner, 0, sizeof(m_stPlanner));
	memset(&m_stIkOwn, 0, sizeof(m_stIkOwn));
	memset(&m_stRef, 0, sizeof(m_stRef));
}

CShadowControl::~CShadowControl()
{
	delete m_pCtl;
}

CHighLevelRT::Params
CShadowControl::HlParams(const double* adNumbers)
{
	const double* const n = adNumbers;
	CHighLevelRT::Params stP = CHighLevelRT::DefaultParams();
	stP.dDelta     = n[SHADOW_N_DELTA];
	stP.dLambda    = n[SHADOW_N_LAMBDA];
	stP.dStepMax   = n[SHADOW_N_STEP_MAX];
	stP.dTol       = n[SHADOW_N_TOL];
	stP.nPlanAhead = (int)n[SHADOW_N_PLAN_AHEAD];
	stP.nIterMax   = (int)n[SHADOW_N_ITER_MAX];
	stP.dVelMax    = n[SHADOW_N_VEL_MAX];
	stP.dAccMax    = n[SHADOW_N_ACC_MAX];
	stP.dJerkMax   = n[SHADOW_N_JERK_MAX];
	stP.dMargin    = n[SHADOW_N_MARGIN];
	stP.dLead      = n[SHADOW_N_LEAD];
	stP.bStepRef   = (n[SHADOW_N_STEP_REF] != 0.0);
	for (int i = 0; i < DOF; i++)
		stP.adQr[i] = n[SHADOW_N_READY + i];
	return stP;
}

CControllerFullDynamicsRT*
CShadowControl::NewLaw(const char* astrURDF, const double* adNumbers)
{
	const double* const n = adNumbers;
	CControllerFullDynamicsRT* const pCtl = new CControllerFullDynamicsRT(astrURDF, DOF);
	pCtl->EnableRTMode(TRUE);
	pCtl->SetDeadline(500000);
	pCtl->SetControlMode(CControllerFullDynamicsRT::eGravityCompensation);
	if (!pCtl->Init())
	{
		delete pCtl;
		return NULL;
	}
	pCtl->SetControlGains(std::vector<double>(n + SHADOW_N_KP, n + SHADOW_N_KP + DOF),
	                      std::vector<double>(n + SHADOW_N_KD, n + SHADOW_N_KD + DOF));
	pCtl->SetFrictionFF(std::vector<double>(n + SHADOW_N_KF, n + SHADOW_N_KF + DOF));
	pCtl->SetRotorInertia(std::vector<double>(n + SHADOW_N_ROTOR, n + SHADOW_N_ROTOR + DOF));
	pCtl->SetFrictionFFGate(n[SHADOW_N_FF_GATE] != 0.0);
	pCtl->SetTorqueLimits(std::vector<double>(n + SHADOW_N_TAU_MAX, n + SHADOW_N_TAU_MAX + DOF));
	return pCtl;
}

BOOL
CShadowControl::Init(const char* astrURDF, const double* adNumbers, uint32_t auCrc)
{
	if (m_pCtl != NULL)
		return FALSE;
	const CHighLevelRT::Params stP = HlParams(adNumbers);
	if (FALSE == m_cHl.Init(astrURDF, adNumbers[SHADOW_N_TS], stP) ||
	    FALSE == m_cPlanner.Init(astrURDF, adNumbers[SHADOW_N_TS], stP))
		return FALSE;

	/* the control law; enabled for good: whether it runs in a period is the APU's */
	CControllerFullDynamicsRT* const pCtl = NewLaw(astrURDF, adNumbers);
	if (pCtl == NULL)
		return FALSE;
	pCtl->Enable(TRUE);

	m_vQ.assign(DOF, 0.0);
	m_vQd.assign(DOF, 0.0);
	m_vTor.assign(DOF, 0.0);
	m_vTau.assign(DOF, 0.0);
	m_uCrc = auCrc;
	m_pCtl = pCtl;
	return TRUE;
}

void
CShadowControl::StartMotion(const ipc_shadow_in_t& astIn)
{
	if (astIn.flags & IPC_SHADOW_START_HOME)
		m_cHl.StartHomeTo(astIn.q, astIn.qd, astIn.start_target);
	else
		m_cHl.StartTrack(astIn.q, astIn.qd);
	m_uStarts = astIn.starts;
	m_nLateSeen = 0;			// the count of the high-level controller starts again
	m_bApart = false;			// from here both controllers have the same motion again
}

void
CShadowControl::Step(const ipc_shadow_in_t& astIn, ipc_shadow_out_t* apOut)
{
	typedef CControllerFullDynamicsRT C;
	memset(apOut, 0, sizeof(*apOut));
	apOut->period = astIn.period;
	apOut->ctrl_crc = m_uCrc;
	if (m_pCtl == NULL)
	{
		apOut->flags = IPC_SHADOW_OUT_FAULT;
		return;
	}
	uint32_t uFlags = 0;

	/* a period of the control task without its payload here: its motion went on, this one did not */
	if (!m_bFirst && astIn.period != m_uPeriod + 1u)
	{
		const uint32_t uGap = astIn.period - m_uPeriod - 1u;
		m_uMissed += (uGap < 0x80000000u) ? uGap : 1u;		// a period number that went back: the R5 started again
		m_bApart = true;
	}
	m_bFirst = false;
	m_uPeriod = astIn.period;

	/* the high-level controller */
	const bool bStart = (astIn.flags & IPC_SHADOW_START) != 0, bAfter = (astIn.flags & IPC_SHADOW_START_AFTER) != 0;
	if (bStart && !bAfter)
		StartMotion(astIn);
	bool bApart = m_bApart;		// of this period: a start after its update comes too late for it
	if (!bStart && astIn.starts != m_uStarts)
		bApart = true;			// a start that no payload told: so in every period until the next start
	if (astIn.flags & IPC_SHADOW_STARTS_LOST)
		bApart = true;			// its update belonged to a motion this controller was not told
	else if (astIn.flags & IPC_SHADOW_TRAJ)
	{
		double adQdd[DOF], adTool[3];
		m_cHl.SetDelta(astIn.delta);
		m_cHl.SetReady(astIn.q_ready);
		const RTTIME t0 = read_timer();
		m_cLink.Before(m_cHl, &m_stIkOwn);	// the plans that its planner has finished; its IK aside
		if (astIn.flags & IPC_SHADOW_IK)
		{
			/* the IK that the APU's update took, as the answer to the request of this controller */
			CHighLevelRT::IkResult stIk;
			stIk.uSeq = m_cHl.GetIkRequest().uSeq;
			stIk.bConverged = 1u;
			stIk.nIter = 0;
			stIk.dErr = 0.0;
			for (int i = 0; i < DOF; i++)
				stIk.adQ[i] = m_adIkApu[i] = astIn.ik_q[i];
			m_cHl.TakeIk(stIk);
			m_uIkApu = bApart ? 0u : stIk.uSeq;		// apart, its request may be for another goal than the APU's
		}
		if (m_uIkApu != 0u && m_stIkOwn.uSeq == m_uIkApu)
		{
			/* the two solutions of one goal are here, whichever came first */
			m_dIkDiff = LargestDifference(m_stIkOwn.adQ, m_adIkApu);
			m_uIkCompared++;
			m_uIkApu = 0u;
		}
		const BOOL bOk = m_cHl.Update((astIn.flags & IPC_SHADOW_GOAL) ? astIn.goal : NULL, astIn.q, astIn.qd,
		                              m_stRef.adQ, m_stRef.adQd, adQdd);
		if ((m_cHl.GetStatus().bOtgNewCalc ? 1 : 0) != ((astIn.flags & IPC_SHADOW_PLAN) ? 1 : 0))
			m_bApart = bApart = true;	// a plan in use from another period than the APU's
		m_cLink.After(m_cHl);				// what this period asks of it
		m_nPlansLate += m_cHl.GetStatus().nPlanLate - m_nLateSeen;
		m_nLateSeen = m_cHl.GetStatus().nPlanLate;
		// the tool position of the measurement, which the control task takes in every period of a motion for
		// its log (CTrajectoryRT::Update): part of what the period costs
		m_cHl.ToolPosition(astIn.q, adTool);
		apOut->traj_ns = (uint32_t)(read_timer() - t0);
		for (int i = 0; i < DOF; i++)
		{
			m_stRef.adQdd[i] = adQdd[i];
			apOut->q_ref[i] = m_stRef.adQ[i];
		}
		uFlags |= IPC_SHADOW_OUT_TRAJ | (bOk ? 0u : (uint32_t)IPC_SHADOW_OUT_FAULT);
	}
	if (bStart && bAfter)
		StartMotion(astIn);

	/* the control law */
	const C::eControlMode eMode = (C::eControlMode)astIn.ctrl_mode;
	if ((astIn.flags & IPC_SHADOW_CTRL) && eMode != C::eGravityCompensation && eMode != C::eTrack && eMode != C::eHome)
		m_bApart = bApart = true;	// a JOINT TEST: not followed, and the hold of this law is not where the APU's is
	else if (astIn.flags & IPC_SHADOW_CTRL)
	{
		m_pCtl->m_bStickyEnable = (astIn.flags & IPC_SHADOW_HOLD) != 0;
		if (m_pCtl->GetControlMode() != eMode)
			m_pCtl->SetControlMode(eMode);
		if (astIn.flags & IPC_SHADOW_REF)
		{
			m_stRef.bFrictionFF = (astIn.flags & IPC_SHADOW_FRICTION_FF) ? TRUE : FALSE;
			m_pCtl->SetReference(m_stRef);
		}
		for (int i = 0; i < DOF; i++)
		{
			m_vQ[i] = astIn.q[i];
			m_vQd[i] = astIn.qd[i];
		}
		const RTTIME t0 = read_timer();
		const BOOL bOk = m_pCtl->Update(m_vQ, m_vQd, m_vTor, m_vTau);
		apOut->ctrl_ns = (uint32_t)(read_timer() - t0);
		for (int i = 0; i < DOF; i++)
			apOut->tau[i] = m_vTau[i];
		apOut->tau_diff = LargestDifference(apOut->tau, astIn.tau);
		uFlags |= IPC_SHADOW_OUT_CTRL | (bOk ? 0u : (uint32_t)IPC_SHADOW_OUT_FAULT);
	}

	apOut->flags = uFlags | (bApart ? (uint32_t)IPC_SHADOW_OUT_APART : 0u);
	apOut->n_missed = m_uMissed;
	/* what its planner has done so far. On the R5 this period may have interrupted the planner task while it
	 * wrote that: it cannot be read then, and the answer has what was read last */
	m_cLink.GetStats(m_stPlanner);
	apOut->n_plans = m_stPlanner.uPlans;
	apOut->plan_us = (uint32_t)m_stPlanner.dPlanUs;
	apOut->n_iks   = m_stPlanner.uIks;
	apOut->ik_us   = (uint32_t)m_stPlanner.dIkUs;
	apOut->n_plan_late = (uint32_t)m_nPlansLate;
	apOut->n_ik_compared = m_uIkCompared;
	apOut->ik_diff = m_dIkDiff;
}
