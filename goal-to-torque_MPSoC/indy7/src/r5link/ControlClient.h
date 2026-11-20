/*****************************************************************************
*	Name: ControlClient.h
*	Description: What the control task of the application keeps of the
*	control task when that runs on the R5 (--arch S2; a firmware built with
*	CONTROL_TASK): its commands, and what its reports say.
*
*	  commands  a queue of a few (the keys of an operator, the steps of a
*	            test). One is written at a time (CR5Link::SendControlCommand):
*	            the next when a report has the number of the last. The answer
*	            to each is told once (TakeAnswer).
*	  settled   every command written is answered and none waits. Only then
*	            does a report tell the state that the commands of this side
*	            left: until then the application keeps what it expects of them
*	            (a motion that it started runs, one that it stopped does not).
*	  report    Take(): the newest report, as the EtherCAT task of the
*	            application took it from the R5. Of a period of a motion it
*	            has the sample that the log of the tests takes
*	            (CTrajectoryRT::Sample), but for the torque: the log has the
*	            torque of the period before, which the caller keeps.
*	  goal      the goal slot that TRACK reads here (UseGoalSlot) goes to the
*	            R5 when it has a new goal (ForwardGoal), with the offset that
*	            the high-level controller would add (the hover height). Only
*	            goals written after UseGoalSlot leave, as only goals written
*	            after a start count.
*
*	A report that is not taken (the tasks here came late, with an arm on the
*	bus of the R5) is a period that the log misses; they are counted from the
*	period numbers (StateReportsMissed).
*
*	One thread: the control task. Nothing here allocates, locks or waits.
*****************************************************************************/
#ifndef G2T_CONTROL_CLIENT_H
#define G2T_CONTROL_CLIENT_H

#include <cstdio>
#include <cstring>
#include <limits>

#include "GoalSlot.h"
#include "TrajectoryRT.h"
#include "r5link/R5Link.h"

class CControlClient
{
public:
	enum { QUEUE = 8 };

	struct Counts
	{
		long	nReports;			// taken
		long	nStateReports;		// ... of a period with a state of the arm
		long	nStateMissed;		// reports of such a period that were not taken, by the period numbers
		long	nCommands;			// written
		long	nRefused;			// ... that the R5 refused
		long	nQueueFull;			// commands that found the queue full: not written
		long	nGoals;				// goals written
		long	nCtrlFaults;		// controller faults of the R5 since the first report of this run ...
		long	nDriveStops;		// ... and its stops of the drives (a dropout, a servo on that ran out) ...
		long	nNoState;			// ... its periods without a state of the arm ...
		long	nPlans, nIks;		// ... the plans and the IK of its planner ...
		long	nPlanLate;			// ... its plans that were not back in their period ...
		long	nGoalsTaken;		// ... and the goals it took
		uint32_t uPlanUsMax, uIkUsMax;	// the longest plan and IK among those of the reports taken [us]
		long	nTraj, nLaw;		// reports with the high-level step, with the law
		double	dTrajNsSum, dLawNsSum;
		uint32_t uTrajNsMax, uLawNsMax;
	};

	CControlClient() : m_pLink(NULL), m_dTs(1e-3), m_nQueued(0), m_uSent(0), m_uSentCmd(IPC_CMD_NONE), m_uAnswered(0),
	                   m_bAnswer(false), m_bReport(false), m_bSample(false), m_uMotion(0), m_uStates(0), m_bStates(false),
	                   m_bStatesSim(false), m_uStatePeriod(0), m_uFaults0(0), m_uStops0(0), m_uNoState0(0), m_uPlans0(0),
	                   m_uIks0(0), m_uPlanLate0(0), m_uGoals0(0),
	                   m_pGoalSlot(NULL), m_uGoalSeen(0)
	{
		memset(&m_stReport, 0, sizeof(m_stReport));
		memset(&m_stSample, 0, sizeof(m_stSample));
		memset(&m_stCounts, 0, sizeof(m_stCounts));
		memset(m_astQueue, 0, sizeof(m_astQueue));
		m_adGoalOffset[0] = m_adGoalOffset[1] = m_adGoalOffset[2] = 0.0;
		m_auHlCounts[0] = m_auHlCounts[1] = m_auHlCounts[2] = m_auHlCounts[3] = 0;
	}

	/* adTs: the control period [s] */
	void Init(CR5Link* apLink, double adTs)
	{
		m_pLink = apLink;
		m_dTs = adTs;
	}

	/* --- the report --- */
	void Take(const ipc_report_t& r)
	{
		m_stCounts.nReports++;
		if (r.state & IPC_CTL_STATE)
		{
			// The periods with a state so far: one more than at the report of such a period taken before this one.
			// With an arm simulated here they are its steps, which this side numbers; with the arm of the bus the
			// periods of the R5 less those without a state
			const bool bSimArm = (r.state & IPC_CTL_SIM_ARM) != 0;
			const uint32_t uStates = bSimArm ? r.sim_step : r.in.period - r.n_no_state;
			if (m_bStates && bSimArm == m_bStatesSim && uStates - m_uStates - 1u < 0x80000000u)	// not after a restart
			{
				m_stCounts.nStateMissed += (long)(uStates - m_uStates - 1u);
				m_uStatePeriod += uStates - m_uStates;
			}
			else
				m_uStatePeriod += 2u;		// the first, another arm, or numbers that begin again: not the next one
			m_uStates = uStates;
			m_bStates = true;
			m_bStatesSim = bSimArm;
			m_stCounts.nStateReports++;
		}
		if (r.in.flags & IPC_SHADOW_CTRL)
		{
			m_stCounts.nLaw++;
			m_stCounts.dLawNsSum += (double)r.ctrl_ns;
			if (r.ctrl_ns > m_stCounts.uLawNsMax)
				m_stCounts.uLawNsMax = r.ctrl_ns;
		}
		if (r.motion != (uint32_t)CTrajectoryRT::eMotionNone)
			m_uMotion = r.motion;			// the period in which a motion ends has none
		m_bSample = (r.state & IPC_CTL_SAMPLE) != 0;
		if (m_bSample)
		{
			m_stCounts.nTraj++;
			m_stCounts.dTrajNsSum += (double)r.traj_ns;
			if (r.traj_ns > m_stCounts.uTrajNsMax)
				m_stCounts.uTrajNsMax = r.traj_ns;
			CTrajectoryRT::Sample& s = m_stSample;
			s.dT = (double)r.n_cycle * m_dTs;
			s.nMode = (m_uMotion == (uint32_t)CTrajectoryRT::eMotionHome) ? 1 : 0;
			s.uFlags = r.hl_flags;
			s.nOtg = r.otg;
			s.uLeadReset = r.lead_reset;
			s.nIkIter = r.ik_iter;
			s.dIkErr = r.ik_err;
			s.dOtgDuration = r.otg_duration;
			s.dHlUs = (double)r.hl_ns * 1e-3;
			s.nGoalAge = r.goal_age;
			for (int k = 0; k < 3; k++)
			{
				s.adGoal[k] = (r.in.flags & IPC_SHADOW_GOAL) ? r.in.goal[k] : std::numeric_limits<double>::quiet_NaN();
				s.adAccepted[k] = r.accepted[k];
				s.adTool[k] = r.tool[k];
			}
			for (int i = 0; i < CTrajectoryRT::MAX_DOF; i++)
			{
				s.adTarget[i] = r.target[i];
				s.adQRef[i] = r.q_ref[i];
				s.adQdRef[i] = r.qd_ref[i];
				s.adQddRef[i] = r.qdd_ref[i];
				s.adQ[i] = r.in.q[i];
				s.adQd[i] = r.in.qd[i];
				s.adTau[i] = 0.0;			// the caller's: the torque of the period before
			}
			s.uClamp = 0;
			m_auHlCounts[0] = r.n_accepted;
			m_auHlCounts[1] = r.n_converged;
			m_auHlCounts[2] = r.n_rejected;
			m_auHlCounts[3] = r.n_lead_resets;
		}
		// the answer to the command written last, once
		if (m_uSent != 0 && m_uSent != m_uAnswered && r.cmd_done == m_uSent)
		{
			m_uAnswered = m_uSent;
			m_bAnswer = true;
			if (r.cmd_result != IPC_DONE_OK)
				m_stCounts.nRefused++;
		}
		// the controller faults and the stops of the drives of this run: the R5 goes on when the application starts
		// again, and what it had counted at the first report is not of this run
		if (!m_bReport)
		{
			m_uFaults0 = r.n_ctrl_faults;
			m_uStops0 = r.n_drive_stops;
			m_uNoState0 = r.n_no_state;
			m_uPlans0 = r.n_plans;
			m_uIks0 = r.n_iks;
			m_uPlanLate0 = r.n_plan_late;
			m_uGoals0 = r.n_goals;
		}
		else
		{
			// the time of the planner's last plan and of its last IK: one of this run when their count has gone on
			// since the report taken before
			if (r.n_plans != m_stReport.n_plans && r.plan_us > m_stCounts.uPlanUsMax)
				m_stCounts.uPlanUsMax = r.plan_us;
			if (r.n_iks != m_stReport.n_iks && r.ik_us > m_stCounts.uIkUsMax)
				m_stCounts.uIkUsMax = r.ik_us;
		}
		m_stCounts.nCtrlFaults = (long)(uint32_t)(r.n_ctrl_faults - m_uFaults0);
		m_stCounts.nDriveStops = (long)(uint32_t)(r.n_drive_stops - m_uStops0);
		m_stCounts.nNoState = (long)(uint32_t)(r.n_no_state - m_uNoState0);
		m_stCounts.nPlans = (long)(uint32_t)(r.n_plans - m_uPlans0);
		m_stCounts.nIks = (long)(uint32_t)(r.n_iks - m_uIks0);
		m_stCounts.nPlanLate = (long)(uint32_t)(r.n_plan_late - m_uPlanLate0);
		m_stCounts.nGoalsTaken = (long)(uint32_t)(r.n_goals - m_uGoals0);
		m_stReport = r;
		m_bReport = true;
	}
	bool HasReport() const { return m_bReport; }
	/* a number for the period of the last report with a state: one more than that of the report with a state before
	 * it when no such report is missing between the two, more otherwise */
	uint32_t StatePeriod() const { return m_uStatePeriod; }
	const ipc_report_t& Report() const { return m_stReport; }
	/* the report taken last was of a period of a motion: its sample */
	CTrajectoryRT::Sample* Sample() { return m_bSample ? &m_stSample : NULL; }
	/* what the high-level controller of the R5 counted in the motion, at its last sample: goals accepted, IK
	 * converged, IK given up, restarts at the lead bound */
	const uint32_t* HlCounts() const { return m_auHlCounts; }

	/* --- the commands --- */
	/* false: the queue is full, the command is dropped (counted) */
	bool Queue(uint32_t auCmd, uint32_t auArg = 0, const ipc_command_t* apNumbers = NULL)
	{
		if (m_nQueued >= (int)QUEUE)
		{
			m_stCounts.nQueueFull++;
			return false;
		}
		ipc_command_t& c = m_astQueue[m_nQueued++];
		memset(&c, 0, sizeof(c));
		if (apNumbers != NULL)
			c = *apNumbers;
		c.cmd = auCmd;
		c.arg = auArg;
		if (auCmd == IPC_CMD_START)			// the counts of the motion before are not those of this one
			m_auHlCounts[0] = m_auHlCounts[1] = m_auHlCounts[2] = m_auHlCounts[3] = 0;
		return true;
	}
	/* the next command of the queue, when the R5 has answered the one before it; the first after a report of the R5
	 * of this run: a firmware that starts takes what the Command slot holds for the command of an earlier run */
	void Send()
	{
		if (m_nQueued == 0 || m_uSent != m_uAnswered || !m_bReport)
			return;
		m_uSentCmd = m_astQueue[0].cmd;
		m_uSent = m_pLink->SendControlCommand(m_astQueue[0]);
		m_nQueued--;
		for (int n = 0; n < m_nQueued; n++)
			m_astQueue[n] = m_astQueue[n + 1];
		m_stCounts.nCommands++;
	}
	bool IsSettled() const { return m_nQueued == 0 && m_uSent == m_uAnswered; }
	/* once per command: TRUE with the command and what the R5 did with it (IPC_DONE_OK or IPC_REFUSED_*); the
	 * numbers of a start that was refused are in the report */
	bool TakeAnswer(uint32_t* apuCmd, uint32_t* apuResult)
	{
		if (!m_bAnswer)
			return false;
		m_bAnswer = false;
		*apuCmd = m_uSentCmd;
		*apuResult = m_stReport.cmd_result;
		return true;
	}
	/* why the R5 refused a command, as a line for the operator */
	void ResultText(uint32_t auResult, char* apcOut, size_t auLen) const
	{
		switch (auResult)
		{
			case IPC_REFUSED_NO_STATE:    snprintf(apcOut, auLen, "the control task of the R5 has no state of the arm"); break;
			case IPC_REFUSED_NOT_ENABLED: snprintf(apcOut, auLen, "controller not enabled ('r', then 'g')"); break;
			case IPC_REFUSED_SERVO_OFF:   snprintf(apcOut, auLen, "servo OFF ('h' first)"); break;
			case IPC_REFUSED_MODE:        snprintf(apcOut, auLen, "not in gravity compensation ('g')"); break;
			case IPC_REFUSED_FAULT:       snprintf(apcOut, auLen, "controller fault latched ('r' off, 'r' on, then 'h')"); break;
			case IPC_REFUSED_NOT_READY:   snprintf(apcOut, auLen, "a drive in Switch on disabled, or the drives went off in that period"); break;
			case IPC_REFUSED_UNKNOWN:     snprintf(apcOut, auLen, "the firmware does not know this command"); break;
			case IPC_REFUSED_OBS:         snprintf(apcOut, auLen, "the receiver of the camera results on the R5 has other "
			                                       "[CONTRACT] or APP_* values than this cfg, or cannot use them: build the "
			                                       "firmware from this cfg (bin/ctrl_numbers_header; the check values are in "
			                                       "the [S2] line here and the [control] line of the firmware)"); break;
			case IPC_REFUSED_ARGUMENT:
				if (m_stReport.refusal_joint < 0)
					snprintf(apcOut, auLen, "the goal threshold %.4g m is outside [0, %.4g]", m_stReport.refusal_value,
					         m_stReport.refusal_hi);
				else
					snprintf(apcOut, auLen, "the ready posture: J%d at %.3f rad, outside [%.3f, %.3f]",
					         (int)m_stReport.refusal_joint + 1, m_stReport.refusal_value, m_stReport.refusal_lo,
					         m_stReport.refusal_hi);
				break;
			case IPC_REFUSED_START:
			{
				CTrajectoryRT::Refusal stWhy;
				stWhy.eWhy = m_stReport.refusal_why;
				stWhy.nJoint = m_stReport.refusal_joint;
				stWhy.dValue = m_stReport.refusal_value;
				stWhy.dLo = m_stReport.refusal_lo;
				stWhy.dHi = m_stReport.refusal_hi;
				CTrajectoryRT::RefusalText(stWhy, apcOut, auLen);
				break;
			}
			default: snprintf(apcOut, auLen, "refused (%u)", (unsigned)auResult); break;
		}
	}

	/* --- the goal --- */
	/* the slot whose goals go to the R5 from here on (a goal that is in it now does not), and their offset */
	void UseGoalSlot(const CGoalSlot* apSlot, const double* adOffset)
	{
		m_pGoalSlot = apSlot;
		m_uGoalSeen = apSlot->Count();
		for (int k = 0; k < 3; k++)
			m_adGoalOffset[k] = adOffset[k];
	}
	/* A new goal of the slot, to the R5. Call it after Send(): TRACK on the R5 counts the goals that arrive from
	 * its start on, so a goal must not be there before the start of its TRACK. While a start waits in the queue
	 * the goal waits with it; a start that Send() has just written is in its slot before the goal, and the R5
	 * reads the Goal slot first. */
	void ForwardGoal()
	{
		for (int n = 0; n < m_nQueued; n++)
			if (m_astQueue[n].cmd == IPC_CMD_START)
				return;
		double adP[3];
		unsigned int uCount = 0;
		if (m_pGoalSlot == NULL || !m_pGoalSlot->Read(adP, &uCount) || uCount == m_uGoalSeen)
			return;
		m_uGoalSeen = uCount;
		for (int k = 0; k < 3; k++)
			adP[k] += m_adGoalOffset[k];
		m_pLink->SendGoal(adP);
		m_stCounts.nGoals++;
	}

	const Counts& GetCounts() const { return m_stCounts; }

private:
	CR5Link*		m_pLink;
	double			m_dTs;
	ipc_command_t	m_astQueue[QUEUE];
	int				m_nQueued;
	uint32_t		m_uSent;			// the number of the command written last (0: none yet) ...
	uint32_t		m_uSentCmd;			// ... and what it was
	uint32_t		m_uAnswered;		// the number of the last one that a report answered
	bool			m_bAnswer;			// ... and TakeAnswer has not told it yet
	ipc_report_t	m_stReport;
	bool			m_bReport;
	CTrajectoryRT::Sample m_stSample;
	bool			m_bSample;
	uint32_t		m_uMotion;			// CTrajectoryRT::eMotion of the motion that runs, or ran last
	uint32_t		m_auHlCounts[4];
	uint32_t		m_uStates;			// the periods with a state at the last report of one ...
	bool			m_bStates;
	bool			m_bStatesSim;		// ... counted as steps of the arm simulated here
	uint32_t		m_uStatePeriod;		// StatePeriod()
	uint32_t		m_uFaults0, m_uStops0;		// what the R5 had counted at the first report
	uint32_t		m_uNoState0, m_uPlans0, m_uIks0, m_uPlanLate0, m_uGoals0;
	const CGoalSlot* m_pGoalSlot;
	unsigned int	m_uGoalSeen;
	double			m_adGoalOffset[3];
	Counts			m_stCounts;
};

#endif // G2T_CONTROL_CLIENT_H
