/*****************************************************************************
*	Name: HighLevelRT.cpp
*	Description: The high-level controller (paper, Sec. IV-C). See
*	HighLevelRT.h.
*****************************************************************************/
#include "HighLevelRT.h"
#include "CallTiming.h"

#include <rbdl/addons/urdfreader/urdfreader.h>

#include <array>
#include <cmath>
#include <cstring>
#include <limits>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

/* ------------------------------------------------------------------------ */
/* Step 2 on a model: CHlIkN                                                */
/* ------------------------------------------------------------------------ */
template <int N>
BOOL
CHlIkN<N>::Init(const char* astrURDF, const char* astrTool)
{
	if (!Addons::URDFReadFromFile(astrURDF, &model, false, false))
	{
		DBG_LOG_ERROR("(CHighLevelRT) cannot read %s", astrURDF);
		return FALSE;
	}
	if (model.qdot_size != (unsigned int)N)
	{
		DBG_LOG_ERROR("(CHighLevelRT) %u joints in %s, %d expected", model.qdot_size, astrURDF, (int)N);
		return FALSE;
	}
	uTcp = model.GetBodyId(astrTool);
	if (uTcp == std::numeric_limits<unsigned int>::max())
	{
		DBG_LOG_ERROR("(CHighLevelRT) no body \"%s\" in %s", astrTool, astrURDF);
		return FALSE;
	}
	vTcpPoint.setZero();
	vQk   = VectorNd::Zero(N);
	vZero = VectorNd::Zero(N);
	mG    = MatrixNd::Zero(3, N);
	return TRUE;
}

template <int N>
double
CHlIkN<N>::Error(const double* adGoal, Vector3d* apvE)
{
	*apvE = Vector3d(adGoal[0], adGoal[1], adGoal[2]) - CalcBodyToBaseCoordinates(model, vQk, uTcp, vTcpPoint, false);
	return apvE->norm();
}

template <int N>
void
CHlIkN<N>::Jacobian()
{
	mG.setZero();				// RBDL fills only the nonzero entries
	CalcPointJacobian(model, vQk, uTcp, vTcpPoint, mG, false);
}

template <int N>
void
CHlIkN<N>::Step(const Vector3d& avE, double adLambda, double adStepMax, const double* adLimLo, const double* adLimHi,
                double adMargin)
{
	mJp = mG;					// fixed size from here on
	Eigen::Matrix3d mA = mJp * mJp.transpose();
	const double dL2 = adLambda * adLambda;
	mA(0, 0) += dL2;
	mA(1, 1) += dL2;
	mA(2, 2) += dL2;
	ldlt.compute(mA);
	Eigen::Matrix<double, N, 1> vDq = mJp.transpose() * ldlt.solve(avE);
	const double dN = vDq.norm();
	if (dN > adStepMax)
		vDq *= adStepMax / dN;
	for (int i = 0; i < N; i++)
	{
		const double dQ = vQk[i] + vDq[i];
		const double dLo = adLimLo[i] + adMargin, dHi = adLimHi[i] - adMargin;
		vQk[i] = dQ < dLo ? dLo : (dQ > dHi ? dHi : dQ);
	}
}

/* ------------------------------------------------------------------------ */
/* the 1 ms part: CHighLevelRTN                                             */
/* ------------------------------------------------------------------------ */
template <int N>
typename CHighLevelRTN<N>::Params
CHighLevelRTN<N>::DefaultParams()
{
	Params p;
	p.dDelta     = 0.006;			// the values of Table 9; the cfg sets them (HL_* keys)
	p.dLambda    = 0.01;
	p.dStepMax   = 0.2;
	p.dTol       = 1e-4;
	p.nIterMax   = 100;
	p.nPlanAhead = 5;				// on the R5 a plan takes up to 1.1 ms of the 0.66 ms per period left to the planner
	p.dVelMax    = 0.6;
	p.dAccMax    = 2.0;
	p.dJerkMax   = 10.0;
	p.dMargin    = 0.26;			// above 0.253 rad, the largest modeled overshoot of a replanned target
	p.dLead      = 0.10;
	p.dRestQd    = 0.05;
	p.nRestCyc   = 200;
	/* the arm (N = 6): q_r and the URDF joint limits of indy7.urdf, J1-J5 175 deg, J6 215 deg; other N:
	 * zero and +-pi, to be set by the caller */
	const double adQr6[6] = { 0.0, 0.0, -M_PI / 2, 0.0, -M_PI / 2, 0.0 };
	for (int i = 0; i < DOF; i++)
	{
		p.adQr[i] = (N == 6) ? adQr6[i] : 0.0;
		p.adLimHi[i] = (N == 6) ? ((i == 5) ? 3.7524578917878086 : 3.0543261909900767) : M_PI;
		p.adLimLo[i] = -p.adLimHi[i];
	}
	snprintf(p.acTool, sizeof(p.acTool), "%s", "tcp");
	p.bStepRef   = false;			// the step baseline is off
	return p;
}

template <int N>
CHighLevelRTN<N>::CHighLevelRTN()
	: m_bInit(false), m_bHome(false), m_dTs(0.001), m_uK(0), m_nPlan(0), m_bPlan(false), m_uPlanStart(0),
	  m_bIkBack(false), m_bPlanBack(false), m_nPlanResult(0), m_dPlanCalcUs(0.0), m_bWantPlan(false), m_uLeadWant(0),
	  m_uLeadAsked(0), m_nRest(0)
{
	m_p = DefaultParams();
	memset(&m_st, 0, sizeof(m_st));
	memset(&m_stIkReq, 0, sizeof(m_stIkReq));
	memset(&m_stIkRes, 0, sizeof(m_stIkRes));
	memset(&m_stPlanReq, 0, sizeof(m_stPlanReq));
	for (int i = 0; i < DOF; i++)
		m_adHold[i] = m_adRef[i] = m_adRefQd[i] = m_adRefQdd[i] = m_adLeadQ[i] = m_adLeadQd[i] = 0.0;
}

template <int N>
BOOL
CHighLevelRTN<N>::Init(const char* astrURDF, double adTs, const Params& astParams)
{
	if (FALSE == m_cIk.Init(astrURDF, astParams.acTool))
		return FALSE;
	if (!(adTs > 0.0))
		return FALSE;
	if (astParams.nPlanAhead < 1)
	{
		DBG_LOG_ERROR("(CHighLevelRT) a plan %d periods ahead: at least 1", astParams.nPlanAhead);
		return FALSE;
	}
	m_vQ  = VectorNd::Zero(DOF);
	m_dTs = adTs;
	m_p = astParams;
	m_bInit = true;

	/* the ready posture must lie inside the limits minus the margin */
	for (int i = 0; i < DOF; i++)
		if (m_p.adQr[i] < m_p.adLimLo[i] + m_p.dMargin || m_p.adQr[i] > m_p.adLimHi[i] - m_p.dMargin)
			DBG_LOG_WARN("(CHighLevelRT) q_r of J%d (%.3f rad) is not inside the limits minus %.2f rad",
			             i + 1, m_p.adQr[i], m_p.dMargin);
	DBG_LOG_INFO("(CHighLevelRT) delta %.1f mm; IK lambda %.3g m, step %.2f rad, tol %.2f mm, %d/goal; a plan %d "
	             "periods ahead; limits %.2f rad/s, %.2f rad/s^2, %.1f rad/s^3; margin %.2f rad, lead %.2f rad",
	             m_p.dDelta * 1e3, m_p.dLambda, m_p.dStepMax, m_p.dTol * 1e3, m_p.nIterMax, m_p.nPlanAhead,
	             m_p.dVelMax, m_p.dAccMax, m_p.dJerkMax, m_p.dMargin, m_p.dLead);
	return TRUE;
}

template <int N>
void
CHighLevelRTN<N>::SetReady(const double* adQr)
{
	for (int i = 0; i < DOF; i++)
		m_p.adQr[i] = adQr[i];
}

template <int N>
void
CHighLevelRTN<N>::StartFrom(const double* adQ)
{
	memset(&m_st, 0, sizeof(m_st));		// no accepted goal, nothing asked of the planner, counters zero
	for (int i = 0; i < DOF; i++)
	{
		m_adHold[i]     = adQ[i];		// hold until a plan is in use
		m_adRef[i]      = adQ[i];
		m_adRefQd[i]    = 0.0;
		m_adRefQdd[i]   = 0.0;
		m_st.adTarget[i] = adQ[i];
	}
	/* what the planner still has of the motion before is not taken: nothing is asked for now, and the next
	 * request has another number */
	m_bPlan = false;
	m_bIkBack = false;
	m_bWantPlan = false;
	m_uLeadWant = 0;
	m_uLeadAsked = 0;
	m_nRest = 0;
}

template <int N>
void
CHighLevelRTN<N>::StartTrack(const double* adQ, const double* adQd)
{
	(void)adQd;
	m_bHome = false;
	StartFrom(adQ);
}

template <int N>
void
CHighLevelRTN<N>::StartHome(const double* adQ, const double* adQd)
{
	StartHomeTo(adQ, adQd, m_p.adQr);
}

template <int N>
void
CHighLevelRTN<N>::StartHomeTo(const double* adQ, const double* adQd, const double* adTarget)
{
	(void)adQd;
	m_bHome = true;
	StartFrom(adQ);
	for (int i = 0; i < DOF; i++)
		m_st.adTarget[i] = adTarget[i];
	m_bWantPlan = true;					// asked for in the first cycle
}

template <int N>
void
CHighLevelRTN<N>::TakeIk(const IkResult& astResult)
{
	if (m_st.bIkActive && astResult.uSeq == m_stIkReq.uSeq)
	{
		m_stIkRes = astResult;
		m_bIkBack = true;
	}
}

template <int N>
void
CHighLevelRTN<N>::TakePlan(const PlanResult& astResult)
{
	if (astResult.uSeq == m_stPlanReq.uSeq)		// m_bPlanBack is read only while this request is out
	{
		m_acPlan[1 - m_nPlan] = astResult.cPlan;
		m_nPlanResult = astResult.nResult;
		m_dPlanCalcUs = astResult.dCalcUs;
		m_bPlanBack = true;
	}
}

template <int N>
bool
CHighLevelRTN<N>::Reference(uint32_t auK, double* adQ, double* adQd, double* adQdd) const
{
	if (!m_bPlan)
	{
		for (int i = 0; i < DOF; i++)
		{
			adQ[i]   = m_adHold[i];
			adQd[i]  = 0.0;
			adQdd[i] = 0.0;
		}
		return true;
	}
	const ruckig::Trajectory<DOF>& cPlan = m_acPlan[m_nPlan];
	const double dT = (double)(auK - m_uPlanStart) * m_dTs;
	std::array<double, DOF> aP, aV, aA;
	cPlan.at_time(dT, aP, aV, aA);			// past its end: Ruckig goes on from the end state, which is at rest
	for (int i = 0; i < DOF; i++)
	{
		adQ[i]   = aP[i];
		adQd[i]  = aV[i];
		adQdd[i] = aA[i];
	}
	return dT >= cPlan.get_duration();
}

template <int N>
BOOL
CHighLevelRTN<N>::Update(const double* adGoal, const double* adQ, const double* adQd,
                         double* adQRef, double* adQdRef, double* adQddRef)
{
	const RTTIME tStart = read_timer();
	const uint32_t uK = m_uK++;
	m_st.bGoalAccepted = false;
	m_st.bIkConverged  = false;
	m_st.bIkRejected   = false;
	m_st.bOtgNewCalc   = false;

	/* Step 1, goal admission: a goal within delta of the accepted goal is
	 * ignored; one farther away replaces it and asks for its IK, from q_r */
	if (!m_bHome && adGoal != NULL)
	{
		double dD2 = 0.0;
		for (int k = 0; k < 3; k++)
			dD2 += (adGoal[k] - m_st.adGoal[k]) * (adGoal[k] - m_st.adGoal[k]);
		if (!m_st.bHasGoal || dD2 > m_p.dDelta * m_p.dDelta)
		{
			for (int k = 0; k < 3; k++)
				m_st.adGoal[k] = m_stIkReq.adGoal[k] = adGoal[k];
			for (int i = 0; i < DOF; i++)
				m_stIkReq.adQr[i] = m_p.adQr[i];
			m_stIkReq.uSeq++;
			m_bIkBack          = false;		// the IK of the goal before it, if it is back, is not used
			m_st.bHasGoal      = true;
			m_st.bIkActive     = true;
			m_st.bGoalAccepted = true;
			m_st.nAccepted++;
			m_nRest = 0;						// the arm is not at rest at the new goal
		}
	}

	/* Step 2, the IK of the accepted goal, when the planner has it: converged
	 * -> the target of the next plan; N_max -> rejected (the previous target
	 * is kept) */
	if (m_bIkBack)
	{
		m_bIkBack      = false;
		m_st.bIkActive = false;
		m_st.nIkIter   = m_stIkRes.nIter;
		m_st.dIkErr    = m_stIkRes.dErr;
		if (m_stIkRes.bConverged)
		{
			m_st.bIkConverged = true;
			m_st.nConverged++;
			for (int i = 0; i < DOF; i++)
				m_st.adTarget[i] = m_stIkRes.adQ[i];
			m_bWantPlan = true;
		}
		else
		{
			m_st.bIkRejected = true;
			m_st.nRejected++;
		}
	}

	/* the step baseline (simulation only, HL_STEP_BASELINE): the target is
	 * the reference, as a step, without a plan (Step 3) and the lead bound
	 * (Step 4); the arm is at rest when |qd| has stayed small, as below */
	if (m_p.bStepRef)
	{
		double dQd2 = 0.0;
		for (int i = 0; i < DOF; i++)
		{
			adQRef[i]   = m_adRef[i] = m_st.adTarget[i];
			adQdRef[i]  = m_adRefQd[i] = 0.0;
			adQddRef[i] = m_adRefQdd[i] = 0.0;
			dQd2 += adQd[i] * adQd[i];
		}
		m_st.nOtg = (int)ruckig::Result::Finished;
		m_nRest = (dQd2 > m_p.dRestQd * m_p.dRestQd) ? 0 : (m_nRest < m_p.nRestCyc ? m_nRest + 1 : m_nRest);
		m_st.bAtRest = m_nRest >= m_p.nRestCyc;
		m_st.dUs = (double)(read_timer() - tStart) * 1e-3;
		return TRUE;
	}

	/* Step 3, the reference. The plan that was asked for is due in this period: it is used from here on, its
	 * time 0 now. One that is not back is asked for again below, for a later period. */
	if (m_st.bPlanAsked && uK == m_stPlanReq.uAdopt)
	{
		m_st.bPlanAsked = false;
		if (!m_bPlanBack)
		{
			m_st.nPlanLate++;
			m_bWantPlan = true;
			m_uLeadWant |= m_uLeadAsked;	// their measured state is still in m_adLeadQ, m_adLeadQd
		}
		else if (m_nPlanResult < 0)
		{
			/* invalid input: keep the last reference; the caller stops the motion */
			m_uLeadAsked = 0;
			for (int i = 0; i < DOF; i++)
			{
				adQRef[i]   = m_adRef[i];
				adQdRef[i]  = m_adRefQd[i];
				adQddRef[i] = m_adRefQdd[i];
			}
			m_st.nOtg = m_nPlanResult;
			m_st.uLeadReset = m_uLeadWant;
			m_st.bAtRest = false;
			m_nRest = 0;
			m_st.dUs = (double)(read_timer() - tStart) * 1e-3;
			return FALSE;
		}
		else
		{
			m_nPlan      = 1 - m_nPlan;
			m_bPlan      = true;
			m_uPlanStart = uK;
			m_st.bOtgNewCalc  = true;
			m_st.dOtgDuration = m_acPlan[m_nPlan].get_duration();
			m_st.dOtgCalcUs   = m_dPlanCalcUs;
		}
		m_uLeadAsked = 0;
	}
	const RTTIME tAt = call_timing::Now();
	const bool bFinished = Reference(uK, m_adRef, m_adRefQd, m_adRefQdd);
	call_timing::Lap(call_timing::eOtgAt, tAt);
	if (bFinished && m_bPlan)
	{
		/* the plan has ended: where it ended is held from the next period on. Ruckig would go on from the end
		 * state, at rest only as far as it rounds; and the time on a plan is a difference of period counts,
		 * which wrap after 49.7 days: an arm that stays that long would run its last plan again */
		m_bPlan = false;
		for (int i = 0; i < DOF; i++)
			m_adHold[i] = m_adRef[i];
	}
	m_st.nOtg = (int)(bFinished ? ruckig::Result::Finished : ruckig::Result::Working);
	for (int i = 0; i < DOF; i++)
	{
		adQRef[i]   = m_adRef[i];
		adQdRef[i]  = m_adRefQd[i];
		adQddRef[i] = m_adRefQdd[i];
	}

	/* Step 4, lead bound: a joint whose reference leads by ell gets its
	 * measured state as the start of the next plan */
	for (int i = 0; i < DOF; i++)
		if (std::fabs(m_adRef[i] - adQ[i]) >= m_p.dLead)
		{
			const unsigned uBit = 1u << i;
			if (!((m_uLeadWant | m_uLeadAsked) & uBit))
				m_st.nLeadResets++;			// once per reset, not per period at the bound
			if (!(m_uLeadAsked & uBit))		// the plan that is out already restarts this joint
			{
				m_uLeadWant |= uBit;
				m_adLeadQ[i]  = adQ[i];
				m_adLeadQd[i] = adQd[i];
			}
		}

	/* a new plan is asked for when none is out and the target changed or a joint is at its bound: for the
	 * period H ahead, from the state that the reference has then */
	if (!m_st.bPlanAsked && (m_bWantPlan || m_uLeadWant != 0))
	{
		m_stPlanReq.uSeq++;
		m_stPlanReq.uAdopt = uK + (uint32_t)m_p.nPlanAhead;
		(void)Reference(m_stPlanReq.uAdopt, m_stPlanReq.adQ, m_stPlanReq.adQd, m_stPlanReq.adQdd);
		for (int i = 0; i < DOF; i++)
		{
			if (m_uLeadWant & (1u << i))
			{
				m_stPlanReq.adQ[i]   = m_adLeadQ[i];
				m_stPlanReq.adQd[i]  = m_adLeadQd[i];
				m_stPlanReq.adQdd[i] = 0.0;
			}
			m_stPlanReq.adTarget[i] = m_st.adTarget[i];
		}
		m_uLeadAsked = m_uLeadWant;
		m_uLeadWant  = 0;
		m_bWantPlan  = false;
		m_bPlanBack  = false;
		m_st.bPlanAsked = true;
	}
	m_st.uLeadReset = m_uLeadWant | m_uLeadAsked;

	/* at rest: the plan in use has reached its target, nothing is asked of the planner, and |qd| has stayed small */
	if (bFinished && !m_st.bPlanAsked && !m_st.bIkActive)
	{
		double dQd2 = 0.0;
		for (int i = 0; i < DOF; i++)
			dQd2 += adQd[i] * adQd[i];
		// counted up to the rest window only: the arm may stay for longer than an int counts periods
		m_nRest = (dQd2 > m_p.dRestQd * m_p.dRestQd) ? 0 : (m_nRest < m_p.nRestCyc ? m_nRest + 1 : m_nRest);
	}
	else
		m_nRest = 0;
	m_st.bAtRest = m_nRest >= m_p.nRestCyc;
	m_st.dUs = (double)(read_timer() - tStart) * 1e-3;
	return TRUE;
}

template <int N>
void
CHighLevelRTN<N>::ToolPosition(const double* adQ, double* adP)
{
	const double adLocal[3] = { m_cIk.vTcpPoint[0], m_cIk.vTcpPoint[1], m_cIk.vTcpPoint[2] };
	ToolPoint(adQ, adLocal, adP);
}

template <int N>
void
CHighLevelRTN<N>::ToolPoint(const double* adQ, const double* adLocal, double* adP)
{
	for (int i = 0; i < DOF; i++)
		m_vQ[i] = adQ[i];
	UpdateKinematics(m_cIk.model, m_vQ, m_cIk.vZero, m_cIk.vZero);
	const Vector3d vP = CalcBodyToBaseCoordinates(m_cIk.model, m_vQ, m_cIk.uTcp, Vector3d(adLocal[0], adLocal[1], adLocal[2]), false);
	for (int k = 0; k < 3; k++)
		adP[k] = vP[k];
}

// the iterations of Step 2, as the planner runs them: the same start q_r,
// steps, clipping, tolerance, and N_max, so that a goal that converges here
// converges in TRACK
template <int N>
BOOL
CHighLevelRTN<N>::SolveIk(const double* adGoal, double* adQ, int* pnIter, double* pdErr)
{
	for (int i = 0; i < DOF; i++)
		m_cIk.vQk[i] = m_p.adQr[i];
	int nIter = 0;
	BOOL bOk = FALSE;
	double dErr = 0.0;
	for (;;)
	{
		Vector3d vE;
		m_cIk.Kinematics();
		dErr = m_cIk.Error(adGoal, &vE);
		if (dErr < m_p.dTol)
		{
			bOk = TRUE;
			break;
		}
		if (++nIter > m_p.nIterMax)
			break;
		m_cIk.Jacobian();
		m_cIk.Step(vE, m_p.dLambda, m_p.dStepMax, m_p.adLimLo, m_p.adLimHi, m_p.dMargin);
	}
	if (adQ != NULL)
		for (int i = 0; i < DOF; i++)
			adQ[i] = m_cIk.vQk[i];
	if (pnIter != NULL)
		*pnIter = nIter;
	if (pdErr != NULL)
		*pdErr = dErr;
	return bOk;
}

/* ------------------------------------------------------------------------ */
/* the planner: CHlPlannerN                                                 */
/* ------------------------------------------------------------------------ */
template <int N>
CHlPlannerN<N>::CHlPlannerN()
	: m_bInit(false), m_bIkActive(false), m_nIkIter(0), m_otg(0.001)
{
	m_p = HL::DefaultParams();
	memset(&m_stReq, 0, sizeof(m_stReq));
}

template <int N>
BOOL
CHlPlannerN<N>::Init(const char* astrURDF, double adTs, const typename HL::Params& astParams)
{
	if (FALSE == m_cIk.Init(astrURDF, astParams.acTool))
		return FALSE;
	if (!(adTs > 0.0))
		return FALSE;
	m_p = astParams;
	m_otg.delta_time = adTs;
	for (int i = 0; i < N; i++)
	{
		m_in.max_velocity[i]        = m_p.dVelMax;
		m_in.max_acceleration[i]    = m_p.dAccMax;
		m_in.max_jerk[i]            = m_p.dJerkMax;
		m_in.target_velocity[i]     = 0.0;
		m_in.target_acceleration[i] = 0.0;
	}
	m_in.synchronization = ruckig::Synchronization::Time;	// all joints arrive at once
	m_bIkActive = false;
	m_bInit = true;
	return TRUE;
}

template <int N>
void
CHlPlannerN<N>::IkBegin(const typename HL::IkRequest& astRequest)
{
	m_stReq = astRequest;
	for (int i = 0; i < N; i++)
		m_cIk.vQk[i] = astRequest.adQr[i];
	m_nIkIter = 0;
	m_bIkActive = true;
}

template <int N>
BOOL
CHlPlannerN<N>::IkStep(typename HL::IkResult* apResult)
{
	if (!m_bIkActive)
		return FALSE;
	// call timing (CallTiming.h): 0 and no clock read when off
	const RTTIME tIt0 = call_timing::Now();
	Vector3d vE;
	m_cIk.Kinematics();
	RTTIME tCt = call_timing::Lap(call_timing::eIkKin, tIt0);
	const double dErr = m_cIk.Error(m_stReq.adGoal, &vE);
	tCt = call_timing::Lap(call_timing::eIkFk, tCt);
	bool bDone = false, bConverged = false;
	if (dErr < m_p.dTol)
		bDone = bConverged = true;
	else if (++m_nIkIter > m_p.nIterMax)
		bDone = true;
	else
	{
		m_cIk.Jacobian();
		tCt = call_timing::Lap(call_timing::eIkJac, tCt);
		m_cIk.Step(vE, m_p.dLambda, m_p.dStepMax, m_p.adLimLo, m_p.adLimHi, m_p.dMargin);
		call_timing::Lap(call_timing::eIkSolve, tCt);
	}
	call_timing::Lap(call_timing::eIkIter, tIt0);
	if (!bDone)
		return FALSE;
	m_bIkActive = false;
	apResult->uSeq       = m_stReq.uSeq;
	apResult->bConverged = bConverged ? 1u : 0u;
	apResult->nIter      = m_nIkIter;
	apResult->dErr       = dErr;
	for (int i = 0; i < N; i++)
		apResult->adQ[i] = m_cIk.vQk[i];
	return TRUE;
}

template <int N>
void
CHlPlannerN<N>::Plan(const typename HL::PlanRequest& astRequest, typename HL::PlanResult* apResult)
{
	for (int i = 0; i < N; i++)
	{
		m_in.current_position[i]     = astRequest.adQ[i];
		m_in.current_velocity[i]     = astRequest.adQd[i];
		m_in.current_acceleration[i] = astRequest.adQdd[i];
		m_in.target_position[i]      = astRequest.adTarget[i];
	}
	const RTTIME tCt = call_timing::Now();
	const RTTIME t0 = read_timer();
	const ruckig::Result eRes = m_otg.calculate(m_in, apResult->cPlan);
	const RTTIME t1 = read_timer();
	call_timing::Lap(call_timing::eOtg, tCt);
	apResult->uSeq    = astRequest.uSeq;
	apResult->nResult = (int32_t)eRes;
	apResult->dCalcUs = (double)(t1 - t0) * 1e-3;
}

/* ------------------------------------------------------------------------ */
/* the planner in the task of its caller: CHlInlineN                        */
/* ------------------------------------------------------------------------ */
template <int N>
void
CHlInlineN<N>::After(HL& acHl)
{
	const typename HL::IkRequest& stIkReq = acHl.GetIkRequest();
	if (stIkReq.uSeq != m_uIkSeq)
	{
		m_uIkSeq = stIkReq.uSeq;
		m_cPlanner.IkBegin(stIkReq);
	}
	const typename HL::PlanRequest& stPlanReq = acHl.GetPlanRequest();
	if (stPlanReq.uSeq != m_uPlanSeq)			// a plan first: it has a period to be back in
	{
		m_uPlanSeq = stPlanReq.uSeq;
		m_cPlanner.Plan(stPlanReq, &m_stPlan);
		acHl.TakePlan(m_stPlan);
	}
	for (int n = 0; n < IK_STEPS && m_cPlanner.IkActive(); n++)
		if (m_cPlanner.IkStep(&m_stIk))
			acHl.TakeIk(m_stIk);
}

/* the instances: the arm (6) and a hand of the humanoid (7) */
template struct CHlIkN<6>;
template struct CHlIkN<7>;
template class CHighLevelRTN<6>;
template class CHighLevelRTN<7>;
template class CHlPlannerN<6>;
template class CHlPlannerN<7>;
template class CHlInlineN<6>;
template class CHlInlineN<7>;
