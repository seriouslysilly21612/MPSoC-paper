/*****************************************************************************
*	Name: ControlCycle.cpp
*	Description: The control task without its operating system. See
*	ControlCycle.h.
*****************************************************************************/
#include "ControlCycle.h"

#include <cmath>
#include <cstring>

#include "obs/ObsMethod.h"

static_assert((int)OBS_WORKSPACE_VERDICTS == (int)CGoalFilter::eVerdicts, "obs_counts_t.workspace");
static_assert((int)OBS_DOF == (int)CControlCycle::DOF, "the joints of a recorded state");

typedef CControllerFullDynamicsRT C;

CControlCycle::CControlCycle()
	: m_pCtl(NULL), m_uCrc(0), m_bFault(false), m_bServoAsked(false), m_bServoReached(false), m_uServoWait(0),
	  m_uServoLimit(0), m_uCmdDone(0), m_uCmdResult(IPC_DONE_OK), m_uStartSeq(0), m_uStartResult(IPC_DONE_OK),
	  m_uNoState(0), m_uFaults(0), m_uDriveStops(0), m_uGoals(0), m_uMotionPeriods(0), m_dTs(0.0), m_bObsLink(false),
	  m_uObsMethod(OBS_RING_OFF), m_bObsGoals(false), m_uObsGoalsEpoch(0), m_uObsSeen(0), m_uObsEntries(0)
{
	memset(m_adObsNumbers, 0, sizeof(m_adObsNumbers));
	memset(&m_stObs, 0, sizeof(m_stObs));
	memset(&m_stObsCfg, 0, sizeof(m_stObsCfg));
	obs_recent_max_start(&m_stResponse, 1u);
	memset(&m_stRef, 0, sizeof(m_stRef));
	memset(&m_stPlanner, 0, sizeof(m_stPlanner));
	memset(&m_stRefusal, 0, sizeof(m_stRefusal));
	memset(m_adQ, 0, sizeof(m_adQ));
	memset(m_adQd, 0, sizeof(m_adQd));
}

CControlCycle::~CControlCycle()
{
	delete m_pCtl;
}

BOOL
CControlCycle::Init(const char* astrURDF, const double* adNumbers, uint32_t auCrc)
{
	if (m_pCtl != NULL)
		return FALSE;
	if (FALSE == m_cTraj.Init(astrURDF, adNumbers[SHADOW_N_TS], CShadowControl::HlParams(adNumbers)))
		return FALSE;
	m_cTraj.SetGoalSlot(&m_cGoal);
	CControllerFullDynamicsRT* const pCtl = CShadowControl::NewLaw(astrURDF, adNumbers);
	if (pCtl == NULL)
		return FALSE;
	pCtl->Enable(FALSE);		// until IPC_CMD_CTRL_ON
	m_vQ.assign(DOF, 0.0);
	m_vQd.assign(DOF, 0.0);
	m_vTor.assign(DOF, 0.0);
	m_vTau.assign(DOF, 0.0);
	m_uCrc = auCrc;
	// the status tells the longest response of the last 1 to 2 s (obs_status.h): a window of 1 s of periods
	const double dTs = adNumbers[SHADOW_N_TS];
	m_dTs = dTs;
	obs_recent_max_start(&m_stResponse, (dTs > 0.0 && dTs < 1.0) ? (uint32_t)std::lround(1.0 / dTs) : 1u);
	m_pCtl = pCtl;
	return TRUE;
}

// the position of the tool tag at the joint angles of a result: the model of the 1 ms side of the high-level
// controller, in this task (CRobotIndy7::InitObs does the same with the APU's)
void
CControlCycle::TagAt(void* apArg, const float* afQ, double* adP)
{
	CControlCycle* const pThis = static_cast<CControlCycle*>(apArg);
	double adQ[DOF], adTag[3];
	for (int i = 0; i < DOF; i++)
		adQ[i] = afQ[i];
	for (int k = 0; k < 3; k++)
		adTag[k] = pThis->m_adObsNumbers[OBS_N_TAG + k];
	pThis->m_cTraj.HighLevel().ToolPoint(adQ, adTag, adP);
}

// The settings of the receiver from the numbers, as CRobotIndy7::InitObs takes them from the configuration, on
// the counter of the releases; the checks are those of InitObs and of CObsGoal::Init
BOOL
CControlCycle::SetObsLink(const ObsLink& astLink)
{
	m_bObsLink = false;
	if (m_pCtl == NULL || astLink.pBox == NULL || astLink.pStatus == NULL || astLink.pRing == NULL ||
	    astLink.pNumbers == NULL || !(astLink.dTicksPerS > 0.0) || !std::isfinite(astLink.dTicksPerS))
		return FALSE;
	for (int i = 0; i < OBS_NUMBERS; i++)
		m_adObsNumbers[i] = astLink.pNumbers[i];
	const double* const n = m_adObsNumbers;
	const double dTs = m_dTs;
	const double dCap = n[OBS_N_AGE_CAP], dAge = n[OBS_N_STATE_AGE];
	bool bOk = std::isfinite(dCap) && dCap > 0.0 && dCap < 1e3 && dTs > 0.0 && dAge == std::floor(dAge);
	for (int m = 0; m < 2; m++)
		bOk = bOk && std::isfinite(n[OBS_N_EPS_M + m]) && n[OBS_N_EPS_M + m] >= 0.0;
	for (int k = 0; k < 3; k++)
		bOk = bOk && std::isfinite(n[OBS_N_TAG + k]);
	if (!bOk)
		return FALSE;
	CObsGoal::Config& c = m_stObsCfg;
	memset(&c, 0, sizeof(c));
	c.stSet.policy = OBS_CONTRACT;
	c.stSet.ticks_per_s = astLink.dTicksPerS;
	c.stSet.period_ticks = (uint64_t)std::llround(dTs * astLink.dTicksPerS);
	c.stSet.age_cap_ticks = (uint64_t)std::llround(dCap * astLink.dTicksPerS);
	c.stSet.e0 = n[OBS_N_E0];
	c.stSet.g = n[OBS_N_G];
	c.stSet.eps_v = n[OBS_N_EPS_V];
	c.stSet.a_max = n[OBS_N_A_MAX];
	c.stSet.eps_cal = n[OBS_N_EPS_CAL];
	c.dTolM = n[OBS_N_DELTA];
	c.nStateAge = (dAge >= 0.0 && dAge <= (double)CObsGoal::MAX_STATE_AGE) ? (int)dAge : -1;
	for (int k = 0; k < 6; k++)
		c.stFilter.adBox[k] = n[OBS_N_BOX + k];
	c.stFilter.dRMax = n[OBS_N_R_MAX];
	c.stFilter.dFloor = n[OBS_N_FLOOR];
	c.stFilter.dHover = n[OBS_N_HOVER];
	c.stFilter.dKappa = n[OBS_N_KAPPA];
	c.stFilter.dGapS = n[OBS_N_GAP];
	c.dLossS = n[OBS_N_LOSS];
	c.pTagAt = &CControlCycle::TagAt;
	c.pTagArg = this;
	// once here, so that a number that the receiver cannot use is known before any command
	if (!m_cObs.Init(astLink.pBox, c, &m_cObsSlot))
		return FALSE;
	m_stObs = astLink;
	m_stObs.pNumbers = NULL;				// copied
	obs_status_start(m_stObs.pStatus);
	obs_ring_start(m_stObs.pRing);
	m_uObsMethod = OBS_RING_OFF;
	m_bObsGoals = false;
	m_uObsEntries = 0;
	m_bObsLink = true;
	return TRUE;
}

// IPC_CMD_OBS: the receiver with the policy of the method, started again only when the method changes; M1 has
// none. Its objects are no goals until the TRACK of the application starts. Returns IPC_DONE_OK or IPC_REFUSED_OBS.
uint32_t
CControlCycle::ObsCommand(const ipc_command_t& astCommand)
{
	const uint32_t uMethod = astCommand.arg;
	m_bObsGoals = false;
	if (uMethod >= (uint32_t)obs_method::eMethods)
		return IPC_REFUSED_OBS;
	if (!obs_method::UsesMailbox((int)uMethod))		// M1: the receiver off, which it always can be
	{
		m_uObsMethod = OBS_RING_OFF;
		return IPC_DONE_OK;
	}
	// numbers that cannot be used, or not those of the configuration of the APU: the methods with the mailbox are
	// refused, and the receiver keeps what it had (the controller is not touched: its numbers are apart)
	if (!m_bObsLink || astCommand.check != m_stObs.uNumbersCrc)
		return IPC_REFUSED_OBS;
	if (uMethod != m_uObsMethod)
	{
		CObsGoal::Config c = m_stObsCfg;
		c.stSet.policy = obs_method::Policy((int)uMethod);
		if (!m_cObs.Init(m_stObs.pBox, c, &m_cObsSlot))
		{
			m_uObsMethod = OBS_RING_OFF;
			return IPC_REFUSED_OBS;
		}
		m_uObsMethod = uMethod;
	}
	return IPC_DONE_OK;
}

// A period with a state and a method that uses the mailbox: the receiver (CRobotIndy7::ObsStep), its status and
// its entries; in the TRACK of the application a new object goes to TRACK, the hover height above it, as the APU
// adds that height to the goals of the application (CControlClient::UseGoalSlot)
void
CControlCycle::ObsStep(const Input& astIn)
{
	m_cObs.Step(astIn.uPeriod, astIn.tRelease, m_adQ);
	obs_status_t stStatus = m_cObs.Status();
	stStatus.response_max = obs_recent_max(&m_stResponse);
	stStatus.contract_crc = m_stObs.uContractCrc;
	stStatus.method = (uint8_t)m_uObsMethod;
	obs_status_write(m_stObs.pStatus, &stStatus);
	for (unsigned i = 0; i < m_cObs.Entries(); i++)
	{
		const CObsGoal::Entry& stE = m_cObs.GetEntry(i);
		obs_entry_t stOut;
		memset(&stOut, 0, sizeof(stOut));
		stOut.rec = stE.stRec;
		stOut.workspace = stE.nWorkspace;
		stOut.folded = stE.nFolded;
		for (int k = 0; k < 3; k++)
			stOut.obj[k] = stE.adObj[k];
		obs_ring_put(m_stObs.pRing, m_uObsEntries++, &stOut);
	}
	// another writer of the mailbox (the application started again): its results are no goals of a TRACK that it
	// did not start
	if (m_bObsGoals && ipc_load32(&m_stObs.pBox->epoch) != m_uObsGoalsEpoch)
		m_bObsGoals = false;
	double adP[3];
	unsigned int uCount = 0;
	if (m_bObsGoals && m_cObsSlot.Read(adP, &uCount) && uCount != m_uObsSeen)
	{
		m_uObsSeen = uCount;
		adP[2] += m_adObsNumbers[OBS_N_HOVER];
		m_cGoal.Write(adP);
	}
}

// the stop of the motion (CRobotIndy7::StopMotion): TRACK or HOME ends, the law holds in gravity compensation
void
CControlCycle::StopMotion()
{
	if (m_cTraj.IsRunning())
		m_cTraj.End(CTrajectoryRT::eAborted);
	m_pCtl->SetControlMode(C::eGravityCompensation);
}

void
CControlCycle::ServoOff(Output* apOut)
{
	m_bServoAsked = false;
	m_bServoReached = false;
	apOut->nServo = IPC_CMD_SERVO_OFF;
}

// a start (IPC_CMD_START): the conditions of CRobotIndy7::CheckMotionStart, the numbers of the command, then
// those of CTrajectoryRT::Start. Returns IPC_DONE_OK or the refusal.
uint32_t
CControlCycle::Start(const ipc_command_t& astCommand, const Input& astIn)
{
	if (!astIn.bState)
		return IPC_REFUSED_NO_STATE;
	if (!m_pCtl->IsEnabled())
		return IPC_REFUSED_NOT_ENABLED;
	if (!astIn.bServoOn)
		return IPC_REFUSED_SERVO_OFF;
	// the numbers come from the other processor: none goes into the controller unchecked
	const CHighLevelRT::Params& stP = m_cTraj.HighLevel().GetParams();
	CTrajectoryRT::Refusal& stWhy = m_stRefusal;
	memset(&stWhy, 0, sizeof(stWhy));
	if (!(astCommand.delta >= 0.0 && astCommand.delta <= 1.0))
	{
		stWhy.nJoint = -1;
		stWhy.dValue = astCommand.delta;
		stWhy.dHi = 1.0;
		return IPC_REFUSED_ARGUMENT;
	}
	for (int i = 0; i < DOF; i++)
		if (!(astCommand.q_ready[i] >= stP.adLimLo[i] && astCommand.q_ready[i] <= stP.adLimHi[i]))
		{
			stWhy.nJoint = i;
			stWhy.dValue = astCommand.q_ready[i];
			stWhy.dLo = stP.adLimLo[i];
			stWhy.dHi = stP.adLimHi[i];
			return IPC_REFUSED_ARGUMENT;
		}
	const bool bHome = (astCommand.arg & IPC_START_HOME) != 0;
	if (m_cTraj.IsRunning())		// before the parameters of the running motion are touched
	{
		stWhy.eWhy = CTrajectoryRT::eMotionRuns;
		stWhy.nJoint = (int)m_cTraj.GetMotion();
		return IPC_REFUSED_START;
	}
	m_cTraj.HighLevel().SetDelta(astCommand.delta);
	m_cTraj.HighLevel().SetReady(astCommand.q_ready);
	if (FALSE == m_cTraj.Start(bHome, astIn.adQ, astIn.adQd, (astCommand.arg & IPC_START_FRICTION_FF) ? TRUE : FALSE,
	                           &stWhy, (bHome && (astCommand.arg & IPC_START_TARGET)) ? astCommand.target : NULL))
		return IPC_REFUSED_START;
	m_uMotionPeriods = 0;
	// the TRACK of the application: the objects of the receiver after this start are its goals, while the writer of
	// the mailbox is the one of now; every other motion has none of them
	m_bObsGoals = !bHome && (astCommand.arg & IPC_START_OBS_GOALS) != 0 && m_uObsMethod != OBS_RING_OFF;
	m_uObsSeen = m_cObsSlot.Count();
	m_uObsGoalsEpoch = m_bObsGoals ? ipc_load32(&m_stObs.pBox->epoch) : 0u;
	return IPC_DONE_OK;
}

void
CControlCycle::Command(const ipc_command_t& astCommand, const Input& astIn, bool abDrivesWentOff, Output* apOut)
{
	uint32_t uResult = IPC_DONE_OK;
	switch (astCommand.cmd)
	{
		case IPC_CMD_CTRL_ON:
			m_pCtl->Enable(TRUE);
			break;
		case IPC_CMD_CTRL_OFF:
			// a law that is off follows no reference; off clears a controller fault (CRobotIndy7::EnableController)
			if (m_cTraj.IsRunning())
				StopMotion();
			m_pCtl->Enable(FALSE);
			m_bFault = false;
			break;
		case IPC_CMD_MOTION_STOP:
			StopMotion();
			break;
		case IPC_CMD_HOLD_ON:
			m_pCtl->m_bStickyEnable = true;
			break;
		case IPC_CMD_HOLD_OFF:
			m_pCtl->m_bStickyEnable = false;
			break;
		case IPC_CMD_START:
			uResult = Start(astCommand, astIn);
			m_uStartSeq = astCommand.seq;
			m_uStartResult = uResult;
			break;
		case IPC_CMD_SERVO_ON:
			// the interlock of 'h' (CRobotIndy7::DoInput): the drives enable at zero torque, so the controller must
			// hold in gravity compensation before the brakes release. A servo on in the period in which the drives
			// went off was given before that: dropped
			if (m_bFault)
				uResult = IPC_REFUSED_FAULT;
			else if (!astIn.bState)
				uResult = IPC_REFUSED_NO_STATE;
			else if (astIn.uNotReady != 0 || abDrivesWentOff)
				uResult = IPC_REFUSED_NOT_READY;
			else if (!m_pCtl->IsEnabled())
				uResult = IPC_REFUSED_NOT_ENABLED;
			else if (m_pCtl->GetControlMode() != C::eGravityCompensation)
				uResult = IPC_REFUSED_MODE;
			else
			{
				m_bServoAsked = true;
				m_uServoWait = 0;
				m_uServoLimit = astCommand.arg != 0 ? astCommand.arg : (uint32_t)SERVO_ON_PERIODS;
				apOut->nServo = IPC_CMD_SERVO_ON;
			}
			break;
		case IPC_CMD_OBS:
			uResult = ObsCommand(astCommand);
			break;
		case IPC_CMD_SERVO_OFF:
		case IPC_CMD_STOP:				// shortcut: a servo off, as in the firmware without the control task
			StopMotion();				// a motion does not go on against the brakes
			ServoOff(apOut);
			break;
		default:
			uResult = IPC_REFUSED_UNKNOWN;
			break;
	}
	m_uCmdDone = astCommand.seq;
	m_uCmdResult = uResult;
}

void
CControlCycle::Step(const Input& astIn, Output* apOut, ipc_report_t* apReport)
{
	memset(apOut, 0, sizeof(*apOut));
	memset(apReport, 0, sizeof(*apReport));
	apReport->in.period = astIn.uPeriod;
	apReport->ctrl_crc = m_uCrc;
	if (m_pCtl == NULL)
		return;
	const bool bState = astIn.bState;
	if (bState)
		for (int i = 0; i < DOF; i++)
		{
			m_adQ[i] = astIn.adQ[i];
			m_adQd[i] = astIn.adQd[i];
		}
	else
		m_uNoState++;
	if (astIn.fResponse >= 0.0f)
		obs_recent_max_add(&m_stResponse, astIn.fResponse);

	/* the drives: a dropout, or a servo on that ran out */
	bool bDrivesWentOff = false;
	if (bState && m_bServoAsked)
	{
		if (astIn.bServoOn)
			m_bServoReached = true;
		else if (m_bServoReached || ++m_uServoWait >= m_uServoLimit)
		{
			bDrivesWentOff = true;
			m_uDriveStops++;
			StopMotion();
			ServoOff(apOut);
		}
	}

	/* the command and the goal of the APU */
	if (astIn.pCommand != NULL)
		Command(*astIn.pCommand, astIn, bDrivesWentOff, apOut);
	if (astIn.pGoal != NULL)
	{
		m_cGoal.Write(astIn.pGoal);
		m_uGoals++;
	}

	/* the camera results: the result this period may use, before the trajectory part reads the goal */
	if (bState && m_uObsMethod != OBS_RING_OFF)
		ObsStep(astIn);

	/* the trajectory part: the reference of this period */
	uint32_t uState = 0;
	bool bRef = false, bHome = false;
	if (bState && m_cTraj.IsRunning())
	{
		const RTTIME t0 = read_timer();
		bHome = (m_cTraj.GetMotion() == CTrajectoryRT::eMotionHome);
		bRef = (FALSE != m_cTraj.Update(m_adQ, m_adQd, m_stRef.adQ, m_stRef.adQd, m_stRef.adQdd));
		m_cWriter.TrajUpdated(m_cTraj);
		// HOME at rest, or a generator error: the motion ends with this period, and the law holds
		if (m_cTraj.Pending() != CTrajectoryRT::eRunning)
		{
			m_cTraj.End(m_cTraj.Pending());
			bRef = false;
		}
		apReport->traj_ns = (uint32_t)(read_timer() - t0);
		const CTrajectoryRT::Sample& s = m_cTraj.GetSample();
		uState |= IPC_CTL_SAMPLE;
		apReport->hl_ns = (uint32_t)(s.dHlUs * 1e3);
		apReport->hl_flags = s.uFlags;
		apReport->otg = s.nOtg;
		apReport->lead_reset = s.uLeadReset;
		apReport->ik_iter = s.nIkIter;
		apReport->goal_age = (int32_t)s.nGoalAge;
		apReport->n_cycle = m_uMotionPeriods++;
		const CHighLevelRT::Status& stHl = m_cTraj.HighLevel().GetStatus();
		apReport->n_accepted = (uint32_t)stHl.nAccepted;
		apReport->n_converged = (uint32_t)stHl.nConverged;
		apReport->n_rejected = (uint32_t)stHl.nRejected;
		apReport->n_lead_resets = (uint32_t)stHl.nLeadResets;
		apReport->ik_err = s.dIkErr;
		apReport->otg_duration = s.dOtgDuration;
		for (int k = 0; k < 3; k++)
		{
			apReport->accepted[k] = s.adAccepted[k];
			apReport->tool[k] = s.adTool[k];
		}
		for (int i = 0; i < DOF; i++)
		{
			apReport->target[i] = s.adTarget[i];
			apReport->q_ref[i] = s.adQRef[i];
			apReport->qd_ref[i] = s.adQdRef[i];
			apReport->qdd_ref[i] = s.adQddRef[i];
		}
	}

	/* the dynamics part: the mode follows the reference, then the law */
	bool bLaw = false, bFailed = false;
	if (bState)
	{
		const C::eControlMode eNow = m_pCtl->GetControlMode();
		if (bRef)
		{
			const C::eControlMode eWant = bHome ? C::eHome : C::eTrack;
			if (eNow != eWant)
				m_pCtl->SetControlMode(eWant);
			m_stRef.bFrictionFF = m_cTraj.GetFrictionFF();
			m_pCtl->SetReference(m_stRef);
		}
		else if (eNow == C::eTrack || eNow == C::eHome)
			m_pCtl->SetControlMode(C::eGravityCompensation);	// the reference is gone
		if (m_pCtl->IsEnabled())
		{
			for (int i = 0; i < DOF; i++)
			{
				m_vQ[i] = m_adQ[i];
				m_vQd[i] = m_adQd[i];
				m_vTor[i] = astIn.adTau[i];
			}
			const RTTIME t0 = read_timer();
			bFailed = (FALSE == m_pCtl->Update(m_vQ, m_vQd, m_vTor, m_vTau));
			apReport->ctrl_ns = (uint32_t)(read_timer() - t0);
			apReport->clamp = m_pCtl->GetLastClamp();
			bLaw = true;
			if (bFailed)
				for (int i = 0; i < DOF; i++)
					m_vTau[i] = 0.0;
		}
	}
	apOut->bTorque = bLaw;
	for (int i = 0; i < DOF; i++)
		apOut->adTau[i] = bLaw ? m_vTau[i] : 0.0;

	/* the period as a second controller is told it: with the mode and the hold that its law had */
	const ipc_shadow_in_t* const pIn =
		m_cWriter.Finish(astIn.uPeriod, m_adQ, m_adQd, bLaw ? m_adQ : NULL, bLaw ? m_adQd : NULL, m_cTraj, *m_pCtl,
		                 bRef, m_stRef.bFrictionFF != FALSE, m_vTau.data());
	if (pIn != NULL)
	{
		apReport->in = *pIn;
		uState |= IPC_CTL_PAYLOAD;
	}

	/* a law that failed: a controller fault, latched once; the motion ends and the drives go off */
	if (bFailed && !m_bFault)
	{
		m_bFault = true;
		m_uFaults++;
		StopMotion();
		ServoOff(apOut);
	}

	apReport->state = uState | (bState ? (uint32_t)IPC_CTL_STATE : 0u) |
	                  ((bState && astIn.bSimArm) ? (uint32_t)IPC_CTL_SIM_ARM : 0u) |
	                  (m_pCtl->IsEnabled() ? (uint32_t)IPC_CTL_ENABLED : 0u) | (m_bFault ? (uint32_t)IPC_CTL_FAULT : 0u) |
	                  (m_bServoAsked ? (uint32_t)IPC_CTL_SERVO_ASKED : 0u) |
	                  ((bState && astIn.bServoOn) ? (uint32_t)IPC_CTL_SERVO_ON : 0u);
	apReport->traj_state = (uint32_t)m_cTraj.GetState();
	apReport->motion = (uint32_t)m_cTraj.GetMotion();
	apReport->sim_step = astIn.uSimStep;
	apReport->cmd_done = m_uCmdDone;
	apReport->cmd_result = m_uCmdResult;
	apReport->start_seq = m_uStartSeq;
	apReport->start_result = m_uStartResult;
	apReport->refusal_why = m_stRefusal.eWhy;
	apReport->refusal_joint = m_stRefusal.nJoint;
	apReport->refusal_value = m_stRefusal.dValue;
	apReport->refusal_lo = m_stRefusal.dLo;
	apReport->refusal_hi = m_stRefusal.dHi;
	apReport->n_no_state = m_uNoState;
	apReport->n_ctrl_faults = m_uFaults;
	apReport->n_drive_stops = m_uDriveStops;
	apReport->n_goals = m_uGoals;
	/* what the planner has done so far; while the planner task is in the middle of writing that, what was read last */
	m_cTraj.GetPlannerStats(m_stPlanner);
	apReport->n_plans = m_stPlanner.uPlans;
	apReport->plan_us = (uint32_t)m_stPlanner.dPlanUs;
	apReport->plan_max_us = (uint32_t)m_stPlanner.dPlanMaxUs;
	apReport->n_iks = m_stPlanner.uIks;
	apReport->ik_us = (uint32_t)m_stPlanner.dIkUs;
	apReport->ik_max_us = (uint32_t)m_stPlanner.dIkMaxUs;
	apReport->n_plan_late = (uint32_t)m_cTraj.PlansLate();

	/* the counts of the receiver, for the APU (obs_ring.h) */
	if (m_bObsLink)
	{
		CObsGoal::Counts stC;
		m_cObs.GetCounts(stC);
		obs_counts_t stOut;
		memset(&stOut, 0, sizeof(stOut));
		stOut.taken = stC.uTaken;
		for (int v = 0; v < (int)OBS_VERDICTS; v++)
			stOut.verdict[v] = stC.auVerdict[v];
		for (int v = 0; v < (int)OBS_WORKSPACE_VERDICTS; v++)
			stOut.workspace[v] = (uint32_t)stC.anWorkspace[v];
		stOut.folded = (uint32_t)stC.nFolded;
		stOut.restarts = (uint32_t)stC.nRestarts;
		stOut.goals = (uint32_t)stC.nGoals;
		stOut.entries = m_uObsEntries;
		stOut.method = m_uObsMethod;
		stOut.flags = m_bObsGoals ? (uint32_t)OBS_RING_GOALS : 0u;
		obs_ring_counts(m_stObs.pRing, &stOut);
	}
}
