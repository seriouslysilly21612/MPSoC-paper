/*****************************************************************************
*	Name: TaskControlS2.cpp
*	Description: The control task with --arch S2 (Arch.h): the control task
*	itself runs on the R5 (a firmware built with CONTROL_TASK;
*	rpu_work/r5_freertos, shadow/ControlCycle.h), and this is what TASK1
*	still does here.
*
*	The trajectory part (TaskTrajectory.cpp) keeps the operator motions, the
*	tests and their logs. Where it started, stepped and ended a motion of
*	its high-level controller, it now
*	  starts and ends  with a command for the R5 (CRobotIndy7::StartMotion,
*	           EndMotion). m_cTraj holds what this side expects of the
*	           command (a motion that it started runs, one that it stopped
*	           does not) until the R5 has answered every command; from then
*	           on the reports tell, and a motion that the R5 ended (HOME at
*	           rest, a generator error, a stop of the drives, a controller
*	           fault, a start that it refused) ends here;
*	  steps    nothing: s2_take_report() takes the report that the EtherCAT
*	           task brought, s2_trajectory_step() gives the reference of its
*	           period for the Reference slot, and the sample of the period
*	           for the log is the report's (CControlClient::Sample);
*	  goals    the goals of the slot that TRACK reads go to the R5, from the
*	           dynamics part below, after the command of the cycle;
*	  the arm  simulated here (-s, -b): the state that the EtherCAT task
*	           stepped it to goes to the R5 from the dynamics part, after
*	           the command and the goal. The R5 reads the three slots in the
*	           reverse order (state, goal, command): when it has a state it
*	           has what this cycle decided before it, and when it has a goal
*	           it has the start of its TRACK.
*
*	In place of the dynamics part (dynamics_cycle_s2): no law. The controller
*	object of the application keeps the state that the controller of the R5
*	reports (enabled, mode, hold) once every command is answered, so the
*	checks of the operator commands read it as before; the Torque slot here
*	gets the torque of the report, for the logs; the controller faults and
*	the stops of the drives that the R5 counts are told; then the operator
*	commands (DoInput), whose effects go to the R5 as commands: controller
*	on and off, the hold, servo on and off.
*
*	What the R5 decides alone: every condition of a start and of a servo on
*	(checked here first, for the operator, and there on its own state), the
*	drive dropout, the time limit of a servo on, the controller fault.
*
*	Not with S2: the JOINT TEST ('u'), the tool pose ('s', 'w'), and the
*	fault injection into the torque of the law ('n', SIM_FAULT_WHAT=torque):
*	they need the law here. The angle that is no number (SIM_FAULT_WHAT=state)
*	goes into the state of the simulated arm that the R5 gets
*	(TaskEtherCAT.cpp), and the controller fault is the R5's.
*
*	A report is new when the R5 had a period with a state of the arm (with
*	an arm simulated here), or in every period (the arm of its bus). A cycle
*	here without one changes nothing but the commands that wait.
*****************************************************************************/
#include "Indy7Ctrl.h"
#include "base/RtConsole.h"

#include <cstring>

namespace
{
	// what the control task keeps from one cycle to the next with --arch S2
	// shortcut: one controller per process, as in TaskTrajectory.cpp
	struct ST_S2_KEEP
	{
		unsigned int udof;
		RTTIME   tBudget;
		uint32_t uReportCount;		// of the report slot, at the last report taken
		uint32_t uSimOutCount;		// of the slot with the state of the arm simulated here, at the last one taken
		bool     bSimOut;			// ... which this cycle writes for the R5
		ipc_sim_state_t stSimOut;
		bool     bNew;				// this cycle took a report
		bool     bFirst;			// no report yet
		ipc_report_t stReport;		// the copy of the slot
		long     nFaults, nDriveStops;		// of this run (CControlClient::Counts), at the last report
		bool     bServoWait;		// a servo on was taken by the R5 and the drives are not on yet ...
		uint32_t uServoPeriod;		// ... the period of the R5 that took it
		uint32_t uServoOnPeriods;	// SERVO_ON_TIMEOUT_MS in periods
		bool     bStopSent;			// the servo off of the end of the application
		uint32_t uLastPeriod;
		task_slots::ST_TORQUE_SLOT stTorque;
	} g_stS2;
}

BOOL
CRobotIndy7::QueueControl(uint32_t auCmd, uint32_t auArg)
{
	if (m_pS2->Queue(auCmd, auArg))
		return TRUE;
	printf("[R5] command %u is dropped: the control task of the R5 has not answered the commands before it\n",
	       (unsigned)auCmd);
	return FALSE;
}

// The trajectory part, once per cycle after the commands of the operator: the report of the R5 that the EtherCAT
// task took, if it is new. What it says of the commands, of the controller and of the motion goes into the
// objects of the application.
bool
s2_take_report(CRobotIndy7* pRobot)
{
	typedef CControllerFullDynamicsRT C;
	typedef CTrajectoryRT T;
	CControlClient& c = *pRobot->m_pS2;
	// The state of the arm simulated here that the EtherCAT task has for the R5, before the report: that task
	// writes the report of a step, then the state after the step, so a state found here has its report in the
	// slot. This cycle takes that report, and writes the state for the R5 at its end (dynamics_cycle_s2): the R5
	// writes its next report on that state, when the one before is taken.
	uint32_t uOut = 0;
	if (pRobot->m_simOutSlot.Read(g_stS2.stSimOut, &uOut) && uOut != g_stS2.uSimOutCount)
	{
		g_stS2.uSimOutCount = uOut;
		g_stS2.bSimOut = true;
	}
	uint32_t uCount = 0;
	g_stS2.bNew = pRobot->m_reportSlot.Read(g_stS2.stReport, &uCount) && uCount != g_stS2.uReportCount;
	if (!g_stS2.bNew)
		return false;
	g_stS2.uReportCount = uCount;
	c.Take(g_stS2.stReport);
	if (pRobot->m_pFollow != NULL)		// --shadow: the period of this report a second time
		pRobot->m_pFollow->Take(g_stS2.stReport, c.StatePeriod());
	const ipc_report_t& r = c.Report();

	/* the answer of the R5 to a command */
	uint32_t uCmd = IPC_CMD_NONE, uResult = IPC_DONE_OK;
	if (c.TakeAnswer(&uCmd, &uResult))
	{
		char acWhy[192];
		c.ResultText(uResult, acWhy, sizeof(acWhy));
		if (uCmd == IPC_CMD_SERVO_ON && uResult == IPC_DONE_OK)
		{
			g_stS2.bServoWait = true;
			g_stS2.uServoPeriod = r.in.period;
		}
		else if (uCmd == IPC_CMD_START && uResult != IPC_DONE_OK)
		{
			// the motion that this side took for started never ran
			printf("[HL] the R5 refused the start - %s\n", acWhy);
			pRobot->m_cTraj.End(T::eAborted);
		}
		else if (uCmd == IPC_CMD_SERVO_ON)
		{
			pRobot->m_nServoRefused++;
			printf("[ARM] the R5 refused the servo on - %s\n", acWhy);
		}
		else if (uResult != IPC_DONE_OK)
			printf("[R5] command %u refused - %s\n", (unsigned)uCmd, acWhy);
	}

	/* the first report: the controller of the R5 is the one of this configuration, or 'h' is refused */
	if (g_stS2.bFirst)
	{
		g_stS2.bFirst = false;
		pRobot->m_bS2SameController = (r.ctrl_crc == pRobot->m_uS2Check);
		if (!pRobot->m_bS2SameController)
			DBG_LOG_ERROR("[R5] the controller of the R5 was built from other constants or another model than this "
			              "configuration (its check value 0x%08X, here 0x%08X): servo on is refused. Build the firmware "
			              "from this cfg (bin/ctrl_numbers_header, bin/rbdl_model_code)", (unsigned)r.ctrl_crc,
			              (unsigned)pRobot->m_uS2Check);
		else
			// the R5 goes on when the application starts again: what a run before left is told
			printf("[R5] the control task of the R5 reports (its period %u): controller %s, %s, %s; its check value "
			       "0x%08X is that of this configuration\n", (unsigned)r.in.period,
			       (r.state & IPC_CTL_ENABLED) ? "on" : "off",
			       (r.state & IPC_CTL_SERVO_ASKED) ? "the servo asked on" : "the servo not asked",
			       r.traj_state == (uint32_t)T::eRunning ? "a motion runs there ('x' stops it)" : "no motion",
			       (unsigned)r.ctrl_crc);
	}

	/* what the R5 did by itself: a controller fault, a stop of the drives */
	const CControlClient::Counts& stN = c.GetCounts();
	if (stN.nCtrlFaults != g_stS2.nFaults)
	{
		g_stS2.nFaults = stN.nCtrlFaults;
		if (pRobot->m_nCtrlFaults.exchange(stN.nCtrlFaults, std::memory_order_acq_rel) == 0)
			pRobot->m_uCtrlFaultFirst = r.in.period;
		pRobot->m_uCtrlFaultLast = r.in.period;
		DBG_LOG_ERROR(">>> CONTROLLER FAULT on the R5 (non-finite state or torque): motion stopped, SERVO OFF "
		              "(brakes) - clear with 'r', 'r', then 'h' (its period %u)", (unsigned)r.in.period);
	}
	if (stN.nDriveStops != g_stS2.nDriveStops)
	{
		const long nMore = stN.nDriveStops - g_stS2.nDriveStops;
		g_stS2.nDriveStops = stN.nDriveStops;
		if (g_stS2.bServoWait)
		{
			// the drives were not on yet: the servo on that this side waits for ran out on the R5
			g_stS2.bServoWait = false;
			pRobot->m_nServoExpired++;
			DBG_LOG_ERROR(">>> SERVO ON NOT COMPLETE after %d ms (period %u of the R5): the R5 has withdrawn the request, "
			              "SERVO OFF (all joints); clear the cause (E-stop, a fault of the drive), then 'h'",
			              pRobot->m_nServoOnTimeoutMs, (unsigned)r.in.period);
		}
		else
		{
			if (pRobot->m_nDriveDropouts.fetch_add(nMore, std::memory_order_acq_rel) == 0)
				pRobot->m_uDropoutFirst = r.in.period;
			pRobot->m_uDropoutLast = r.in.period;
			DBG_LOG_ERROR(">>> DRIVES OFF by the R5 (its period %u): a joint left Operation enabled during servo on (E-stop, "
			              "or a fault of the drive) - SERVO OFF (all joints, brakes), motion stopped; no drive enables by "
			              "itself: clear the cause, then 'h'", (unsigned)r.in.period);
		}
	}
	if (g_stS2.bServoWait && (r.state & IPC_CTL_SERVO_ON))
	{
		g_stS2.bServoWait = false;
		const RTTIME tOn = (RTTIME)(r.in.period - g_stS2.uServoPeriod) * pRobot->m_stTrajTiming.tPeriod;
		pRobot->m_nServoOnDone++;
		if (tOn > pRobot->m_tServoOnMax)
			pRobot->m_tServoOnMax = tOn;
		printf("[ARM] servo on: every joint in Operation enabled %.0f ms after the R5 took the command\n", (double)tOn * 1e-6);
	}

	/* the state that the commands of this side left: told by a report once the R5 has answered them all */
	if (!c.IsSettled())
		return true;
	C* const pCtl = pRobot->m_pController;
	const BOOL bEnabled = (r.state & IPC_CTL_ENABLED) ? TRUE : FALSE;
	if (bEnabled != pRobot->m_bRTControllerEnabled)
		pRobot->EnableController(bEnabled);
	pRobot->m_bCtrlFault.store((r.state & IPC_CTL_FAULT) != 0, std::memory_order_release);
	const uint32_t uMode = r.in.ctrl_mode;
	if ((uMode == (uint32_t)C::eGravityCompensation || uMode == (uint32_t)C::eTrack || uMode == (uint32_t)C::eHome) &&
	    pCtl->GetControlMode() != (C::eControlMode)uMode)
		pRobot->SetControllerMode((C::eControlMode)uMode);
	pCtl->m_bStickyEnable = (r.in.flags & IPC_SHADOW_HOLD) != 0;
	// a motion that the R5 has ended, or does not have (it started again)
	if (pRobot->m_cTraj.IsRunning() && r.traj_state != (uint32_t)T::eRunning)
		pRobot->m_cTraj.End((r.traj_state == (uint32_t)T::eDoneAtRest || r.traj_state == (uint32_t)T::eError) ?
		                    (T::eState)r.traj_state : T::eAborted);
	return true;
}

// The trajectory part, in place of the update of its high-level controller: the reference that the law of the R5
// followed in the period of the report, for the Reference slot. Returns task_slots::eRefMode.
int
s2_trajectory_step(CRobotIndy7* pRobot, double* adQRef, double* adQdRef, double* adQddRef, int32_t* apnHlNs)
{
	using namespace task_slots;
	const ipc_report_t& r = pRobot->m_pS2->Report();
	*apnHlNs = (r.state & IPC_CTL_SAMPLE) ? Sat32((RTTIME)r.hl_ns) : -1;
	for (int i = 0; i < CTrajectoryRT::MAX_DOF; i++)
	{
		adQRef[i] = r.q_ref[i];
		adQdRef[i] = r.qd_ref[i];
		adQddRef[i] = r.qdd_ref[i];
	}
	if (!(r.in.flags & IPC_SHADOW_REF))
		return eRefNone;
	return (r.in.ctrl_mode == (uint32_t)CControllerFullDynamicsRT::eHome) ? eRefHome : eRefTrack;
}

bool
dynamics_begin_s2(CRobotIndy7* pRobot, CRtTask* pTask)
{
	const unsigned int udof = (unsigned int)pRobot->GetTotalAxis();
	DBG_LOG_INFO("(dynamics_begin_s2) --arch S2: the control law runs on the R5; here its reports and the commands "
	             "of the operator");
	if (pTask == NULL || udof == 0 || udof > (unsigned int)task_slots::MAX_DOF || pRobot->GetController() == NULL)
	{
		DBG_LOG_ERROR("(dynamics_begin_s2) %u joints, or no controller object", udof);
		return false;
	}
	task_slots::ST_TASK_TIMING& st = pRobot->m_stDynTiming;
	st.tPeriod = pTask->Period();
	memset(&g_stS2, 0, sizeof(g_stS2));
	g_stS2.udof = udof;
	g_stS2.tBudget = st.tPeriod - pTask->GetSpec().tStartDelay;
	g_stS2.bFirst = true;
	g_stS2.uLastPeriod = 0xFFFFFFFF;
	// SERVO_ON_TIMEOUT_MS in periods, at least one: the R5 counts it
	const unsigned long long uTimeoutNs = (unsigned long long)(pRobot->m_nServoOnTimeoutMs > 0 ?
	                                                           pRobot->m_nServoOnTimeoutMs : 1) * 1000000ULL;
	const unsigned long long uPeriodNs = st.tPeriod > 0 ? (unsigned long long)st.tPeriod : 1000000ULL;
	g_stS2.uServoOnPeriods = (uint32_t)((uTimeoutNs + uPeriodNs - 1ULL) / uPeriodNs);
	return true;
}

void
dynamics_cycle_s2(CRobotIndy7* pRobot, RTTIME tRelease, RTTIME tStart)
{
	using namespace task_slots;
	typedef CControllerFullDynamicsRT C;
	CControlClient& c = *pRobot->m_pS2;
	C* const pCtl = pRobot->GetController();
	const unsigned int udof = g_stS2.udof;
	ST_TASK_TIMING& st = pRobot->m_stDynTiming;
	ST_STATE_SLOT&  stState = pRobot->m_stDynState;      // DoInput reads these copies
	ST_REF_SLOT&    stRef = pRobot->m_stDynRef;
	ST_TORQUE_SLOT& stTorque = g_stS2.stTorque;
	if (tStart - tRelease > st.tPeriod)
		st.nLate++;

	/* read */
	const bool bNewState = pRobot->m_stateSlot.Read(stState) && stState.uPeriod != g_stS2.uLastPeriod;
	pRobot->m_refSlot.Read(stRef);
	if (bNewState)
	{
		g_stS2.uLastPeriod = stState.uPeriod;
		if (stState.bOp && !stState.bStop)
			for (unsigned int i = 0; i < udof; i++)
			{
				pRobot->m_vCurrentPos[i] = stState.adQ[i];
				pRobot->m_vCurrentVel[i] = stState.adQd[i];
				pRobot->m_vCurrentTor[i] = stState.adTau[i];
			}
	}

	/* publish: the torque that the law of the R5 computed in the period of the report, and the state of the
	 * controller as this side has it. The trajectory part reads it in the next cycle, as it reads the torque of
	 * the period before from the dynamics part (the log of the tests), the EtherCAT task for its log. */
	const RTTIME tEnd0 = read_timer();
	bool bWrite = false;
	if (g_stS2.bNew)
	{
		const ipc_report_t& r = c.Report();
		const BOOL bLaw = (r.in.flags & IPC_SHADOW_CTRL) ? TRUE : FALSE;
		stTorque.uPeriod = stState.uPeriod;
		stTorque.bValid = bLaw;
		for (unsigned int i = 0; i < udof && i < (unsigned int)IPC_CTRL_DOF; i++)
			stTorque.adTau[i] = bLaw ? r.in.tau[i] : 0.0;
		stTorque.uClamp = bLaw ? r.clamp : 0u;
		stTorque.nDynNs = bLaw ? Sat32((RTTIME)r.ctrl_ns) : -1;		// the whole law on the R5, not its RBDL calls alone
		stTorque.nW = Sat32(tStart - tRelease);
		stTorque.nE = Sat32(tEnd0 - tStart);
		bWrite = true;
	}

	/* then: the operator commands; what they ask of the drives goes to the R5 as a command */
	if (bNewState && stState.bOp && !stState.bStop && pRobot->m_cKeyPress.load() != ' ')
	{
		pRobot->DoInput();
		bWrite = true;
	}
	switch (pRobot->m_nServoCmd.exchange(eServoCmdNone, std::memory_order_acq_rel))
	{
		case eServoCmdOn:   pRobot->QueueControl(IPC_CMD_SERVO_ON, g_stS2.uServoOnPeriods); break;
		case eServoCmdOff:  pRobot->QueueControl(IPC_CMD_SERVO_OFF); break;
		case eServoCmdStop: pRobot->QueueControl(IPC_CMD_STOP); break;
		default: break;
	}
	// the application ends ('q', a signal): the motion of the R5 stops and its drives go off, as they do with
	// the control task here; then its controller off, as a controller here ends with the application: the next
	// run finds the R5 as a new one has it, and 'r' switches the controller on there. An application that is
	// killed sends nothing, and the R5 goes on
	if (stState.bStop && !g_stS2.bStopSent)
	{
		g_stS2.bStopSent = true;
		pRobot->QueueControl(IPC_CMD_SERVO_OFF);
		pRobot->QueueControl(IPC_CMD_CTRL_OFF);
	}
	c.Send();
	// the goals of TRACK go to the R5, where its high-level controller reads them: after the command of the cycle,
	// so that no goal is there before the start of its TRACK (CControlClient::ForwardGoal)
	if (pRobot->m_cTraj.IsRunning() && pRobot->m_cTraj.GetMotion() == CTrajectoryRT::eMotionTrack)
		c.ForwardGoal();
	// The state of the arm simulated here (the trajectory part took it from the EtherCAT task), last: the R5 runs a
	// period on it, and reads the state first, so it has the command and the goal above when it has the state.
	// Which period of the R5 gets them then does not depend on how late this cycle is.
	if (g_stS2.bSimOut)
	{
		g_stS2.bSimOut = false;
		pRobot->m_pLink->SendSimState(g_stS2.stSimOut);
	}
	if (bWrite)
	{
		stTorque.eCtrlMode = (int)pCtl->GetControlMode();
		stTorque.bEnabled = pRobot->m_bRTControllerEnabled;
		stTorque.bJtRunning = FALSE;
		stTorque.uAllocUpdate = 0;
		pRobot->m_torqueSlot.Write(stTorque);
	}

	/* the timing of this task */
	const RTTIME tEnd = read_timer();
	const RTTIME tW = tStart - tRelease, tE = tEnd - tStart;
	st.nCycles++;
	st.dWSum += (double)tW;
	st.dESum += (double)tE;
	if (tW > st.tWMax) st.tWMax = tW;
	if (tE > st.tEMax) st.tEMax = tE;
	if (tW + tE > g_stS2.tBudget)
		st.nOverruns++;
}
