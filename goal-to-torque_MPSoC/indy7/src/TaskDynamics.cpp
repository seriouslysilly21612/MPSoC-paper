/*****************************************************************************
*	Name: TaskDynamics.cpp
*	Description: The dynamics part of the control task (TASK1, paper,
*	Sec. IV-A; the Dynamics task of the paper), and the control task itself
*	(proc_control_task, at the end). In every period, after the trajectory
*	part:
*	  read     the State slot of the EtherCAT task (the measurement of this
*	           period) and the Reference slot of the Trajectory task (the
*	           joint reference of this period, or none);
*	  mode     TRACK or HOME while the Reference slot carries a reference of
*	           this period or the one before, gravity compensation when it
*	           carries none, has gone stale, or after a stop until the
*	           Trajectory task has acknowledged it; a JOINT TEST keeps its own
*	           mode, and a reference that arrives during one is refused (the
*	           motion is stopped);
*	  law      if the state is valid and the controller is enabled: the
*	           control law (gravity compensation with the hold, CTC on the
*	           reference, eq. (5), or the JOINT TEST), the torque limit; the
*	           JOINT TEST log. A law that fails (a non-finite measured state
*	           or torque) gives zero torque and latches a controller fault:
*	           the motion ends, the drives go off in the next period, and they
*	           stay off until the operator switches the controller off and on
*	           ('r', 'r', 'h'); a drive dropout that the EtherCAT task found in
*	           this period, or a servo on that it withdrew (every joint off),
*	           ends the motion here, as 'e' does;
*	  publish  the Torque slot: the torque of this period, which the
*	           EtherCAT task sends in the next one, and the state of the
*	           controller;
*	  then     the operator commands (DoInput), off the path of the torque,
*	           and the Torque slot again with the state they changed.
*	It ends when the EtherCAT task has ended (after the stop).
*****************************************************************************/
#include "Indy7Ctrl.h"
#include "base/MallocCount.h"
#include "base/RtConsole.h"
#include "CallTiming.h"
#include "shadow/ShadowTask.h"

#include <cstring>
#include <limits>

namespace
{
	// what the dynamics part keeps from one period to the next
	// shortcut: one controller per process, as in TaskTrajectory.cpp
	struct ST_DYN_KEEP
	{
		unsigned int udof;
		CControllerFullDynamicsRT* pCtl;
		RTTIME tBudget;
		task_slots::ST_TORQUE_SLOT stTorque;
		CControllerFullDynamicsRT::Reference stReference;
		uint32_t uLastPeriod;
		BOOL bRefusedTold;
	} g_stDyn;
}

bool
dynamics_begin(CRobotIndy7* pRobot, CRtTask* pTask)
{
	using namespace task_slots;
	const unsigned int udof = (unsigned int)pRobot->GetTotalAxis();
	CControllerFullDynamicsRT* const pCtl = pRobot->GetController();
	DBG_LOG_INFO("(dynamics_begin) dynamics part of the control task (controller %s)",
	             pCtl != NULL ? "available; 'r' enables it" : "NOT available - no torque");
	if (pTask == NULL || udof == 0 || udof > (unsigned int)MAX_DOF)
	{
		DBG_LOG_ERROR("(dynamics_begin) %u joints", udof);
		return false;
	}
	ST_TASK_TIMING& st = pRobot->m_stDynTiming;
	st.tPeriod = pTask->Period();
	memset(&g_stDyn, 0, sizeof(g_stDyn));
	g_stDyn.udof = udof;
	g_stDyn.pCtl = pCtl;
	// released START_DELAY after the period: its torque must be out before
	// the next period, i.e. within T - START_DELAY of its own release
	g_stDyn.tBudget = st.tPeriod - pTask->GetSpec().tStartDelay;
	g_stDyn.uLastPeriod = 0xFFFFFFFF;
	return true;
}

void
dynamics_cycle(CRobotIndy7* pRobot, RTTIME tRelease, RTTIME tStart)
{
	using namespace task_slots;
	typedef CControllerFullDynamicsRT C;
	const unsigned int udof = g_stDyn.udof;
	C* const pCtl = g_stDyn.pCtl;
	ST_TASK_TIMING& st = pRobot->m_stDynTiming;
	const RTTIME tBudget = g_stDyn.tBudget;
	ST_STATE_SLOT&  stState = pRobot->m_stDynState;      // DoInput reads these copies
	ST_REF_SLOT&    stRef = pRobot->m_stDynRef;
	ST_TORQUE_SLOT& stTorque = g_stDyn.stTorque;
	C::Reference&   stReference = g_stDyn.stReference;
	uint32_t&       uLastPeriod = g_stDyn.uLastPeriod;
	BOOL&           bRefusedTold = g_stDyn.bRefusedTold;
	if (tStart - tRelease > st.tPeriod)
		st.nLate++;

	// malloc-count build: allocations of the last cycle of this task
	// (nothing is counted in the normal build)
	{
		static unsigned long s_uCycleStart = 0;
		const unsigned long uNow = malloc_count::Count();
		pRobot->m_uAllocCycle += uNow - s_uCycleStart;
		s_uCycleStart = uNow;
		malloc_count::Arm(true);
	}

	/* read */
	const bool bNewState = pRobot->m_stateSlot.Read(stState) && stState.uPeriod != uLastPeriod;
	pRobot->m_refSlot.Read(stRef);
	int32_t nDynNs = -1;
	BOOL bValid = FALSE, bFollowed = FALSE;		// the law was updated in this cycle; on the reference of the period
	unsigned int uClamp = 0;

	if (bNewState)
	{
		uLastPeriod = stState.uPeriod;
		// a drive dropout or a withdrawn servo on in this period
		// (TaskEtherCAT.cpp): the motion ends (a JOINT TEST here, TRACK or
		// HOME in the Trajectory task), as with 'e'; the drives are off
		// already
		if (pRobot->m_nDriveStop.load(std::memory_order_relaxed) != 0 &&
		    pRobot->m_nDriveStop.exchange(0, std::memory_order_acq_rel) != 0)
			pRobot->StopMotion();
		if (stState.bOp && !stState.bStop)
		{
			// the measurement of this period, in every cycle, also while
			// the controller is off: the tests and the hand-eye capture
			// use the measured posture
			for (unsigned int i = 0; i < udof; i++)
			{
				pRobot->m_vCurrentPos[i] = stState.adQ[i];
				pRobot->m_vCurrentVel[i] = stState.adQd[i];
				pRobot->m_vCurrentTor[i] = stState.adTau[i];
			}

			/* the mode: the reference of the Trajectory task, if any */
			if (pCtl != NULL)
			{
				// after a stop: nothing until the Trajectory task has acknowledged
				if (pRobot->m_bRefInhibit && stRef.eMode == eRefNone)
					pRobot->m_bRefInhibit = FALSE;
				const BOOL bHasRef = (stRef.eMode == eRefTrack || stRef.eMode == eRefHome) ? TRUE : FALSE;
				// of this period, or of the one before
				const BOOL bFresh = (stRef.uPeriod == stState.uPeriod || stRef.uPeriod + 1 == stState.uPeriod)
				                        ? TRUE : FALSE;
				const BOOL bJt = pCtl->IsJointTestRunning();
				BOOL bFollow = (bHasRef && bFresh && !pRobot->m_bRefInhibit && !bJt) ? TRUE : FALSE;
				if (bHasRef && !bFresh && !pRobot->m_bRefInhibit)
					pRobot->m_nRefStale++;
				if (bHasRef && bJt && !pRobot->m_bRefInhibit)
				{
					// a reference during a JOINT TEST: the test keeps the arm,
					// the motion of the Trajectory task is stopped
					pRobot->m_nRefRefused++;
					pRobot->m_nHlCmd.store(HL_CMD_STOP, std::memory_order_release);
					pRobot->m_bRefInhibit = TRUE;
					if (!bRefusedTold)
					{
						printf("[DYN] a reference arrived during a JOINT TEST: TRACK/HOME stopped\n");
						bRefusedTold = TRUE;
					}
				}
				const C::eControlMode eNow = pCtl->GetControlMode();
				if (bFollow)
				{
					const C::eControlMode eWant = (stRef.eMode == eRefHome) ? C::eHome : C::eTrack;
					if (eNow != eWant)
					{
						// from gravity compensation only (a JOINT TEST was excluded above)
						if (eNow == C::eGravityCompensation || eNow == C::eTrack || eNow == C::eHome)
							pRobot->SetControllerMode(eWant);
						else
							bFollow = FALSE;
					}
					if (bFollow)
					{
						for (unsigned int i = 0; i < udof; i++)
						{
							stReference.adQ[i]   = stRef.adQRef[i];
							stReference.adQd[i]  = stRef.adQdRef[i];
							stReference.adQdd[i] = stRef.adQddRef[i];
						}
						stReference.bFrictionFF = stRef.bFrictionFF;
						pCtl->SetReference(stReference);
					}
				}
				else if (eNow == C::eTrack || eNow == C::eHome)
				{
					// the reference is gone (the end of the motion, a stop,
					// or stale): gravity compensation
					pRobot->SetControllerMode(C::eGravityCompensation);
				}
				bFollowed = bFollow;
			}

			/* the law */
			const BOOL bCtrlOn = (pCtl != NULL && pCtl->IsEnabled()) ? TRUE : FALSE;
			if (bCtrlOn)
			{
				// fault injection ('n', SIM_FAULT_WHAT=state; simulation
				// only): the measured angle of the joint is not finite in
				// this cycle (the next copy of the state restores it)
				if (pRobot->m_nStateInject != 0 && pRobot->m_nSimFaultJoint >= 0 &&
				    pRobot->m_nSimFaultJoint < (int)udof)
				{
					pRobot->m_vCurrentPos[pRobot->m_nSimFaultJoint] = std::numeric_limits<double>::quiet_NaN();
					if (pRobot->m_nStateInject == 1)
						pRobot->m_nStateInject = 0;
				}
				// the malloc-count build counts the allocations of the
				// control law, which must stay at zero
				const unsigned long uAlloc0 = malloc_count::Count();
				const RTTIME tCt = call_timing::Now();
				const BOOL bUpdated = pCtl->Update(pRobot->m_vCurrentPos, pRobot->m_vCurrentVel,
				                                   pRobot->m_vCurrentTor, pRobot->m_vOutputTorque);
				call_timing::Lap(call_timing::eControl, tCt);
				const unsigned long uAlloc = malloc_count::Count() - uAlloc0;
				pRobot->m_uAllocUpdate += uAlloc;
				pRobot->m_uAllocCycles++;
				if (uAlloc > pRobot->m_uAllocUpdateMaxCycle)
					pRobot->m_uAllocUpdateMaxCycle = uAlloc;
				nDynNs = Sat32(pCtl->GetDynamicsNs());
				uClamp = pCtl->GetLastClamp();
				if (!bUpdated)
				{
					// The controller failed: zero torque. Update() fails
					// only on a non-finite measured state or torque
					// (FullDynControllerRT.cpp; its other FALSE cases
					// cannot occur here: the controller is initialized,
					// enabled, and the vectors have its size). A controller
					// fault, latched once: the motion ends and the drives go
					// off in the next period, as with 'e' (the brakes
					// engage); 'h' is refused until 'r', 'r'. While it stays
					// latched, a cycle that fails again sends zero torque
					// and repeats nothing (the controller logs one failed
					// cycle in 1000).
					for (unsigned int i = 0; i < udof; i++)
						pRobot->m_vOutputTorque[i] = 0.0;
					pRobot->m_nCtrlFaultCycles++;
					if (!pRobot->m_bCtrlFault.load(std::memory_order_relaxed))
					{
						pRobot->m_bCtrlFault.store(true, std::memory_order_release);
						if (pRobot->m_nCtrlFaults.fetch_add(1, std::memory_order_acq_rel) == 0)
							pRobot->m_uCtrlFaultFirst = stState.uPeriod;
						pRobot->m_uCtrlFaultLast = stState.uPeriod;
						pRobot->StopMotion();
						pRobot->m_nServoCmd.store(eServoCmdOff, std::memory_order_release);
						DBG_LOG_ERROR(">>> CONTROLLER FAULT (non-finite state or torque): motion stopped, "
						              "SERVO OFF (brakes) - clear with 'r', 'r', then 'h' (period %u)",
						              (unsigned)stState.uPeriod);
					}
				}
				bValid = TRUE;
			}

			// JOINT TEST log: the sample of every cycle of the test, then
			// the end of the test (at rest, timeout, or a stop) closes it
			if (pRobot->m_cJtLog.IsOpen() && pCtl != NULL)
			{
				if (bCtrlOn && pCtl->HasJointTestSample())
				{
					// with what the drives report in this period (0x6077)
					// and the target written in this period (0x6071, the
					// torque of the last cycle of this task); in the
					// simulation the applied torque
					C::JointTestSample stS = pCtl->GetJointTestSample();
					for (int i = 0; i < (int)udof && i < C::JT_MAX_DOF; ++i)
					{
						stS.adTauAct[i] = stState.adTau[i];
						stS.anTauActRaw[i] = stState.anTauActRaw[i];
						stS.anTauCmdRaw[i] = stState.anTauCmdRaw[i];
					}
					pRobot->m_cJtLog.Push(stS);
				}
				const C::eJointTestState eJt = pCtl->GetJointTestState();
				if (eJt != C::eJtRunning)
				{
					JointTestFormat::Result stR;
					memcpy(stR.adQ0, pCtl->GetJointTestQ0(), sizeof(stR.adQ0));
					memcpy(stR.adD, pCtl->GetJointTestD(), sizeof(stR.adD));
					pRobot->FillRtResult(stR, stState.stFaults, pRobot->m_stJtRt0, pRobot->m_nJtCtrlFaults0,
					                     pRobot->m_nJtDropouts0);
					stR.nAllocs = malloc_count::Enabled() ? (long)(pRobot->m_uAllocUpdate - pRobot->m_uJtAllocs0) : -1;
					pRobot->m_cJtLog.Close((int)eJt, stR);
					DBG_LOG_INFO("[JT] %s - gravity compensation",
					             eJt == C::eJtDoneAtRest ? "done, the arm is at rest" :
					             eJt == C::eJtDoneTimeout ? "done, NOT at rest after 5 s" :
					             "stopped");
				}
			}
		}
	}

	/* publish: the torque of this period (sent by the EtherCAT task in the
	 * next one) and the state of the controller. Without a new state
	 * nothing is written: the torque of the last state stays in the slot
	 * for the EtherCAT task (--arch S1: in the Torque slot of the R5, which
	 * counts a period without a new one). */
	const RTTIME tEnd0 = read_timer();
	if (bNewState)
	{
		stTorque.uPeriod = stState.uPeriod;
		stTorque.bValid = bValid;
		for (unsigned int i = 0; i < udof; i++)
			stTorque.adTau[i] = bValid ? pRobot->m_vOutputTorque[i] : 0.0;
		stTorque.uClamp = uClamp;
		stTorque.eCtrlMode = (pCtl != NULL) ? (int)pCtl->GetControlMode() : (int)C::eGravityCompensation;
		stTorque.bEnabled = pRobot->m_bRTControllerEnabled;
		stTorque.bJtRunning = (pCtl != NULL && pCtl->IsJointTestRunning()) ? TRUE : FALSE;
		stTorque.nDynNs = nDynNs;
		stTorque.nW = Sat32(tStart - tRelease);
		stTorque.nE = Sat32(tEnd0 - tStart);
		stTorque.uAllocUpdate = pRobot->m_uAllocUpdate;
		pRobot->m_torqueSlot.Write(stTorque);
		// --arch S1: also to the R5, now, which takes it at the start of its next period (r5link/R5Link.h)
		if (pRobot->m_pLink != NULL)
			pRobot->m_pLink->SendTorque(stTorque.uPeriod, stTorque.bValid, stTorque.bEnabled, stTorque.adTau, udof);
		// the response of this period, for the status of the next (obs_status.h): same thread as ObsStep()
		if (obs_method::UsesMailbox(pRobot->m_nMethod))
			obs_recent_max_add(&pRobot->m_stObsResponse, (float)((double)(tEnd0 - tRelease) * 1e-9));
	}

	/* --shadow: what this period did with the controller, for the second controller on the R5, and its last
	 * answer (shadow/ShadowTask.h). Before the commands: they may change the mode and the hold for the next period */
	if (pRobot->m_pShadow != NULL && pCtl != NULL)
		pRobot->m_pShadow->Finish(bNewState, stState.uPeriod, bValid ? pRobot->m_vCurrentPos.data() : NULL,
		                          bValid ? pRobot->m_vCurrentVel.data() : NULL, pRobot->m_cTraj, *pCtl,
		                          bFollowed != FALSE, stReference.bFrictionFF != FALSE, pRobot->m_vOutputTorque.data());

	/* then: the operator commands, after the torque is out */
	if (bNewState && stState.bOp && !stState.bStop && pRobot->m_cKeyPress.load() != ' ')
	{
		pRobot->DoInput();
		// the state of the controller as the command left it, for the
		// Trajectory task of the next period (the torque is unchanged)
		stTorque.eCtrlMode = (pCtl != NULL) ? (int)pCtl->GetControlMode() : (int)C::eGravityCompensation;
		stTorque.bEnabled = pRobot->m_bRTControllerEnabled;
		stTorque.bJtRunning = (pCtl != NULL && pCtl->IsJointTestRunning()) ? TRUE : FALSE;
		pRobot->m_torqueSlot.Write(stTorque);
		if (pRobot->m_pLink != NULL)
			pRobot->m_pLink->SendTorque(stTorque.uPeriod, stTorque.bValid, stTorque.bEnabled, stTorque.adTau, udof);
	}

	/* the timing of this task */
	const RTTIME tEnd = read_timer();
	const RTTIME tW = tStart - tRelease, tE = tEnd - tStart;
	st.nCycles++;
	st.dWSum += (double)tW;
	st.dESum += (double)tE;
	if (tW > st.tWMax) st.tWMax = tW;
	if (tE > st.tEMax) st.tEMax = tE;
	if (tW + tE > tBudget)
		st.nOverruns++;
}

void
dynamics_end(CRobotIndy7* pRobot)
{
	DBG_LOG_INFO("(dynamics_end) dynamics part ended");
	pRobot->m_bDynDone.store(true, std::memory_order_release);
}

bool trajectory_begin(CRobotIndy7* pRobot, CRtTask* pTask);
void trajectory_cycle(CRobotIndy7* pRobot, RTTIME tRelease, RTTIME tStart);
void trajectory_end(CRobotIndy7* pRobot);
// --arch S2: what stands in place of the dynamics part (TaskControlS2.cpp)
bool dynamics_begin_s2(CRobotIndy7* pRobot, CRtTask* pTask);
void dynamics_cycle_s2(CRobotIndy7* pRobot, RTTIME tRelease, RTTIME tStart);

// The control task (TASK1, D-05 of the T-ASE work): in every period the
// trajectory part (TaskTrajectory.cpp), then the dynamics part, in the order
// in which the Trajectory and the Dynamics task of the paper (Table 3: two
// tasks, 96 and 95, on one core) ran them; the Reference slot still passes
// between the two, so the data of every period are those of the two tasks.
// With --arch S2 the controller is on the R5, and the second part takes its reports instead.
void
proc_control_task(void* apRobot)
{
	rt_console::MarkRealTime();
	auto* pRobot = static_cast<CRobotIndy7*>(apRobot);
	CRtTask* const pTask = CRtTask::Self();
	const bool bS2 = (pRobot->m_pS2 != NULL);
	if (!trajectory_begin(pRobot, pTask) || !(bS2 ? dynamics_begin_s2(pRobot, pTask) : dynamics_begin(pRobot, pTask)))
	{
		pRobot->m_bTrajDone.store(true, std::memory_order_release);
		pRobot->m_bDynDone.store(true, std::memory_order_release);
		return;
	}
	while (TRUE)
	{
		const RTTIME tRelease = pTask->WaitNextRelease();
		const RTTIME tStart = read_timer();
		// the EtherCAT task has ended (1 s after the stop): so does this one
		if (pRobot->m_bEcatDone.load(std::memory_order_acquire))
			break;
		trajectory_cycle(pRobot, tRelease, tStart);
		// t_w of the dynamics part counts from the same release
		if (bS2)
			dynamics_cycle_s2(pRobot, tRelease, read_timer());
		else
			dynamics_cycle(pRobot, tRelease, read_timer());
	}
	trajectory_end(pRobot);
	dynamics_end(pRobot);
}
