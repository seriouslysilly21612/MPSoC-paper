/*****************************************************************************
*	Name: SimR5Slots.h
*	Description: The R5's side of the slots in the simulation (sim/SimR5.h):
*	what the firmware does with them in every period (rpu_work/r5_freertos:
*	ocm_link.c, and the stale-torque rule of axis_control.c), without a bus
*	and without a thread, so that the rules can be checked one period at a
*	time (tools/r5_link_check.cpp).
*
*	  Start         the R5 half of the head; what the Command slot holds
*	                then was sent to an earlier run
*	  TakeApuSlots  after the receive of a period: a command counts once,
*	                when (epoch, seq) changes (servo on; servo off; a stop is
*	                a servo off, as in the firmware); a Torque slot written
*	                since the last period is new. STALE_LIMIT periods in a
*	                row without a new one while the servo is asked for switch
*	                the servo off, until the APU asks again (D-03, D-10)
*	  PublishState  the State slot of the period: the status and the
*	                measurement of the receive, and what TakeApuSlots left
*****************************************************************************/
#ifndef G2T_SIM_R5_SLOTS_H
#define G2T_SIM_R5_SLOTS_H

#include <string.h>

#include "../ipc/ipc_slot.h"

class CSimR5Slots
{
public:
	/* what the receive of a period brought from the drives */
	struct Drives
	{
		unsigned	uDof;
		uint16_t	ausStatus[IPC_MAX_DOF];		// 0x6041
		double		adQ[IPC_MAX_DOF], adQd[IPC_MAX_DOF], adTau[IPC_MAX_DOF];
	};

	CSimR5Slots() : m_pRegion(NULL), m_uCfgCrc(0), m_nStaleLimit(1), m_uEpoch(0), m_uCommandEpoch(0),
	                m_uCommandSeq(0), m_uCommandDone(0), m_uTorquePeriod(0), m_bTorqueUsable(false),
	                m_bServoAsked(false), m_nStaleRun(0), m_uStaleTotal(0)
	{
		m_auTorqueStamp[0] = m_auTorqueStamp[1] = 0;
		memset(m_adTorque, 0, sizeof(m_adTorque));
	}

	/* Operation enabled, as the firmware reads 0x6041 (cia402_drive.c) */
	static bool IsEnabled(uint16_t ausStatus) { return (ausStatus & 0x6F) == 0x27; }

	/* the R5 side comes up on a region with the layout of ipc_layout.h; its epoch */
	uint32_t Start(volatile uint32_t* apRegion, uint32_t auCfgCrc, int anStaleLimit)
	{
		m_pRegion = apRegion;
		m_uCfgCrc = auCfgCrc;
		m_nStaleLimit = anStaleLimit;
		volatile uint32_t* apSlots[1] = { Slot(IPC_OFF_STATE) };
		m_uEpoch = ipc_side_start(&Head()->r5, apSlots, 1);
		(void)TakeCommand();
		m_uCommandDone = 0;
		return m_uEpoch;
	}

	/* true: the stale-torque rule switched the servo off in this period */
	bool TakeApuSlots(const Drives& astDrives)
	{
		const uint32_t uCommand = TakeCommand();
		if (uCommand == IPC_CMD_SERVO_ON)
			m_bServoAsked = true;
		else if (uCommand != IPC_CMD_NONE)
			m_bServoAsked = false;
		const bool bFresh = TakeTorque(astDrives.uDof);

		bool bAllEnabled = true;
		for (unsigned i = 0; i < astDrives.uDof; i++)
			if (!IsEnabled(astDrives.ausStatus[i]))
				bAllEnabled = false;
		// the State slot counts as the application counted them; the stop looks at the request alone
		if (!bFresh && bAllEnabled)
			m_uStaleTotal++;
		if (bFresh || !m_bServoAsked)
		{
			m_nStaleRun = 0;
			return false;
		}
		if (++m_nStaleRun < m_nStaleLimit)
			return false;
		m_bServoAsked = false;
		return true;
	}
	bool			IsServoAsked	(	) const { return m_bServoAsked; }
	/* the APU marked the last new torque valid and enabled */
	bool			IsTorqueUsable	(	) const { return m_bTorqueUsable; }
	const double*	Torque			(	) const { return m_adTorque; }
	int				StaleRun		(	) const { return m_nStaleRun; }

	void PublishState(uint64_t auReleaseTicks, uint32_t auPeriod, const Drives& astDrives, uint32_t auOverruns)
	{
		ipc_state_t& s = m_stState;
		memset(&s, 0, sizeof(s));
		s.t_release = auReleaseTicks;
		s.period = auPeriod;
		s.dof = astDrives.uDof;
		s.flags = IPC_STATE_OP | IPC_STATE_SERVO_ON;
		for (unsigned i = 0; i < astDrives.uDof; i++)
		{
			const uint16_t usStatus = astDrives.ausStatus[i];
			s.status[i] = usStatus;
			if ((usStatus & 0x4F) == 0x40 || (usStatus & 0x4F) == 0x00)
				s.not_ready |= 1u << i;
			if (!IsEnabled(usStatus))
				s.flags &= ~(uint32_t)IPC_STATE_SERVO_ON;
			s.q[i] = astDrives.adQ[i];
			s.qd[i] = astDrives.adQd[i];
			s.tau[i] = astDrives.adTau[i];
		}
		s.cmd_done = m_uCommandDone;
		s.cfg_crc = m_uCfgCrc;
		s.torque_period = m_uTorquePeriod;
		s.n_torque_stale = m_uStaleTotal;
		s.n_overrun = auOverruns;
		IPC_WRITE(Slot(IPC_OFF_STATE), m_uEpoch, &s);
	}

private:
	volatile ipc_head_t*	Head	(	) const { return (volatile ipc_head_t*)m_pRegion; }
	volatile uint32_t*		Slot	(unsigned auOffset) const { return m_pRegion + auOffset / 4u; }

	// ocm_link_take_command of the firmware
	uint32_t TakeCommand()
	{
		ipc_command_t stCommand;
		uint32_t auStamp[2];
		if (IPC_READ(Slot(IPC_OFF_COMMAND), &Head()->apu, &stCommand, auStamp) != IPC_OK)
			return IPC_CMD_NONE;
		if (auStamp[0] == m_uCommandEpoch && stCommand.seq == m_uCommandSeq)
			return IPC_CMD_NONE;
		m_uCommandEpoch = auStamp[0];
		m_uCommandSeq = stCommand.seq;
		m_uCommandDone = stCommand.seq;
		return stCommand.cmd;
	}
	// ocm_link_take_torque of the firmware
	bool TakeTorque(unsigned auDof)
	{
		ipc_torque_t stTorque;
		uint32_t auStamp[2];
		if (IPC_READ(Slot(IPC_OFF_TORQUE), &Head()->apu, &stTorque, auStamp) != IPC_OK ||
		    (auStamp[0] == m_auTorqueStamp[0] && auStamp[1] == m_auTorqueStamp[1]))
			return false;
		m_auTorqueStamp[0] = auStamp[0];
		m_auTorqueStamp[1] = auStamp[1];
		m_uTorquePeriod = stTorque.period;
		m_bTorqueUsable = (stTorque.flags & IPC_TORQUE_VALID) && (stTorque.flags & IPC_TORQUE_ENABLED);
		for (unsigned i = 0; i < auDof && i < (unsigned)IPC_MAX_DOF; i++)
			m_adTorque[i] = stTorque.tau[i];
		return true;
	}

	volatile uint32_t*	m_pRegion;
	uint32_t	m_uCfgCrc;
	int			m_nStaleLimit;
	uint32_t	m_uEpoch;
	uint32_t	m_uCommandEpoch, m_uCommandSeq, m_uCommandDone;
	uint32_t	m_auTorqueStamp[2];
	uint32_t	m_uTorquePeriod;
	bool		m_bTorqueUsable;
	double		m_adTorque[IPC_MAX_DOF];
	bool		m_bServoAsked;
	int			m_nStaleRun;
	uint32_t	m_uStaleTotal;
	ipc_state_t	m_stState;			// the State slot of the period, before it is written
};

#endif // G2T_SIM_R5_SLOTS_H
