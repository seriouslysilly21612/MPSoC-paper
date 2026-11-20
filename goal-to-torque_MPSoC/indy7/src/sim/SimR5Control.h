/*****************************************************************************
*	Name: SimR5Control.h
*	Description: The R5 of the simulation with the control task on it
*	(--arch S2): what the firmware built with CONTROL_TASK does in every
*	period (rpu_work/r5_freertos: ecat_task.c, control_link.c), on the slots
*	of SimR5Slots.h, without a thread, so that it can be checked one period
*	at a time (tools/s2_link_check.cpp); sim/SimR5.h runs it in its thread.
*
*	  bus      a bus that has not the joints of the arm, as the L7N bench:
*	           its drives are at 0, ready to switch on, and stay off
*	  State    of that bus, with IPC_STATE_CONTROL
*	  control  after the send: the command and the goal of the APU that are
*	           new, and the state of the arm that the APU simulates when it
*	           is new (Sim state slot); one period of the control task
*	           (shadow/ControlCycle.h, the class the firmware has); its
*	           report, then its torque back with the number of that state
*	           (Sim torque slot); a period without a new state of that arm
*	           reports nothing; then its planner, which the planner task of
*	           the firmware runs until the next period
*
*	The control task is the firmware's; what is modelled here is the cycle
*	around it. The arm of the bus (the Indy7) is not modelled: with this
*	R5 the arm is always the one the APU simulates.
*****************************************************************************/
#ifndef G2T_SIM_R5_CONTROL_H
#define G2T_SIM_R5_CONTROL_H

#include "../shadow/ControlCycle.h"
#include "SimR5Slots.h"
#include "../r5link/LinkClock.h"

class CSimR5Control
{
public:
	CSimR5Control() : m_pControl(NULL), m_nNoState(0), m_fResponse(-1.0f)
	{
		memset(&m_stDrives, 0, sizeof(m_stDrives));
		memset(&m_stSim, 0, sizeof(m_stSim));
		memset(&m_stReport, 0, sizeof(m_stReport));
	}

	/* the R5 side comes up: the control task (built, the caller's), the region, the check value of its State and
	 * the drives of its bus; its epoch */
	uint32_t Start(CControlCycle* apControl, volatile uint32_t* apRegion, uint32_t auCfgCrc, unsigned auDrives)
	{
		m_pControl = apControl;
		m_stDrives.uDof = auDrives;
		for (unsigned i = 0; i < auDrives && i < (unsigned)IPC_MAX_DOF; i++)
			m_stDrives.ausStatus[i] = 0x0231;		// Ready to switch on
		return m_cSlots.Start(apRegion, auCfgCrc, 1);
	}

	/* one period */
	void Cycle(uint64_t auReleaseTicks, uint32_t auPeriod, uint32_t auOverruns)
	{
		m_cSlots.PublishState(auReleaseTicks, auPeriod, m_stDrives, auOverruns, false, true);

		CControlCycle::Input stIn;
		memset(&stIn, 0, sizeof(stIn));
		stIn.uPeriod = auPeriod;
		stIn.tRelease = auReleaseTicks;
		stIn.fResponse = m_fResponse;		// of the period before that had a state, once (control_link.c)
		m_fResponse = -1.0f;
		ipc_command_t stCommand;
		ipc_goal_t stGoal;
		// the state, the goal, the command, as the firmware reads them (control_link.c): the reverse of the order
		// in which the APU writes them
		const int nSim = m_cSlots.TakeSimState(&m_stSim);
		stIn.pGoal = m_cSlots.TakeGoal(&stGoal) ? stGoal.p : NULL;
		stIn.pCommand = m_cSlots.TakeCommand(&stCommand) ? &stCommand : NULL;
		if (nSim >= 0)
		{
			stIn.bSimArm = true;
			stIn.bState = (nSim == 1);			// a period of the control task per state of the arm
			stIn.bServoOn = (m_stSim.flags & IPC_SIM_SERVO_ON) != 0;
			stIn.uNotReady = m_stSim.not_ready;
			stIn.uSimStep = m_stSim.step;
			for (int i = 0; i < IPC_CTRL_DOF; i++)
			{
				stIn.adQ[i] = m_stSim.q[i];
				stIn.adQd[i] = m_stSim.qd[i];
				stIn.adTau[i] = m_stSim.tau[i];
			}
		}
		CControlCycle::Output stOut;
		m_pControl->Step(stIn, &stOut, &m_stReport);
		// the report before the torque, and none without a new state of the simulated arm (control_link.c)
		if (!stIn.bSimArm || stIn.bState)
			m_cSlots.PublishReport(m_stReport);
		if (stIn.bState)				// of the simulated arm: this R5 has no other
		{
			ipc_sim_torque_t stTorque;
			stTorque.step = m_stSim.step;
			stTorque.flags = (stOut.bTorque ? (uint32_t)IPC_SIM_TORQUE_USABLE : 0u) |
			                 ((m_stReport.state & IPC_CTL_SERVO_ASKED) ? (uint32_t)IPC_SIM_SERVO_ASKED : 0u);
			for (int i = 0; i < IPC_CTRL_DOF; i++)
				stTorque.tau[i] = stOut.adTau[i];
			m_cSlots.PublishSimTorque(stTorque);
			m_fResponse = (float)((double)(link_clock::Ticks() - auReleaseTicks) / (double)link_clock::TicksPerSecond());
		}
		else
			m_nNoState++;
		while (m_pControl->ServePlanner())
			;
	}

	/* periods so far in which the control task had no state of the arm */
	long NoState() const { return m_nNoState; }
	const ipc_report_t& LastReport() const { return m_stReport; }

private:
	CControlCycle*		m_pControl;
	CSimR5Slots			m_cSlots;
	CSimR5Slots::Drives	m_stDrives;
	ipc_sim_state_t		m_stSim;		// the last state of the arm taken
	ipc_report_t		m_stReport;
	long				m_nNoState;
	float				m_fResponse;	// [s] release to torque of the last period with a state; < 0: given already
};

#endif // G2T_SIM_R5_CONTROL_H
