/*****************************************************************************
*	Name: R5Joint.h
*	Description: A joint of the arm whose drive is on the bus of the R5
*	(--arch S1). Its state is that of the State slot which the EtherCAT task
*	of the application took last (r5link/R5Link.h). The R5 walks every drive
*	at once, so servo on, servo off and stop of a joint are requests for the
*	Command slot, written once per cycle (CR5Link::SendCommand). The target
*	torque does not pass here: the control task writes the Torque slot
*	itself, right after the control law. Called by the EtherCAT task of the
*	application only.
*****************************************************************************/
#ifndef G2T_R5_JOINT_H
#define G2T_R5_JOINT_H

#include "Joint.h"
#include "../r5link/R5Link.h"

class CR5Joint : public CJoint
{
public:
	CR5Joint(const TSTRING& astrName, CR5Link* apLink, unsigned auIndex) : m_pLink(apLink), m_uIndex(auIndex)
	{
		m_strName = astrName;
	}

	virtual double	GetCurrentPos	(	) const { return m_pLink->State().q[m_uIndex]; }
	virtual double	GetCurrentVel	(	) const { return m_pLink->State().qd[m_uIndex]; }
	virtual double	GetCurrentTor	(	) const { return m_pLink->State().tau[m_uIndex]; }
	/* Operation enabled, as the firmware reads 0x6041 (cia402_drive.c) */
	virtual BOOL	IsServoOn		(	) const { return ((GetStatusWord() & 0x6F) == 0x27) ? TRUE : FALSE; }
	virtual UINT16	GetStatusWord	(	) const { return m_pLink->State().status[m_uIndex]; }
	virtual BOOL	ServoOn			(	) { m_pLink->RequestCommand(IPC_CMD_SERVO_ON); return TRUE; }
	virtual BOOL	ServoOff		(	) { m_pLink->RequestCommand(IPC_CMD_SERVO_OFF); return TRUE; }
	virtual BOOL	StopAxis		(	) { m_pLink->RequestCommand(IPC_CMD_STOP); return TRUE; }
	virtual BOOL	MoveTorque		(double) { return TRUE; }
	virtual INT32	GetActualTorRaw	(	) const { return m_pLink->State().tau_act_raw[m_uIndex]; }
	virtual INT32	GetTargetTorRaw	(	) const { return m_pLink->State().tau_cmd_raw[m_uIndex]; }

private:
	CR5Link* const	m_pLink;
	const unsigned	m_uIndex;
};

#endif // G2T_R5_JOINT_H
