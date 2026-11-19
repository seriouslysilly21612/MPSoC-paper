/*****************************************************************************
*	Name: ShadowWriter.h
*	Description: The control task's side of --shadow: what it did with its
*	controller in a period, as the payload of the Shadow in slot
*	(ipc/ipc_layout.h), for the controller on the R5 to do the same
*	(shadow/ShadowControl.h). Two calls in the period of the control task:
*
*	    TrajUpdated   right after CTrajectoryRT::Update(), in a period
*	                  with one: the goal and the parameters of that update,
*	                  whether a motion had started before it, and what it
*	                  had from its planner: the IK it took, if it converged,
*	                  and whether a plan came into use (the planner of the
*	                  R5 is another and has its own times)
*	    Finish        at the end of the period: the measured state, the
*	                  starts after the update, the mode and the torque of
*	                  the control law
*
*	One thread (the control task). The starts are read from the trajectory
*	object itself (CTrajectoryRT::LastStart), so every start is seen
*	whichever part of the application made it.
*
*	The payload has one measured state. The control task uses one in a
*	period, unless a State arrives between its two parts: then the update
*	or the start of the period had another state than its control law, and
*	Finish gives no payload. The second controller sees the gap in the
*	period numbers and stands apart until the next start.
*****************************************************************************/
#ifndef G2T_SHADOW_WRITER_H
#define G2T_SHADOW_WRITER_H

#include <cmath>
#include <cstring>

#include "FullDynControllerRT.h"
#include "TrajectoryRT.h"
#include "ipc/ipc_layout.h"

class CShadowWriter
{
public:
	CShadowWriter() : m_uStartsEnd(0), m_uStartsAtUpdate(0), m_uPlanner(0), m_bUpdated(false), m_bGoal(false)
	{
		memset(&m_stIn, 0, sizeof(m_stIn));
	}

	void TrajUpdated(const CTrajectoryRT& acTraj)
	{
		const CTrajectoryRT::Sample& s = acTraj.GetSample();
		const CHighLevelRT::Params& p = acTraj.HighLevel().GetParams();
		m_bUpdated = true;
		m_uStartsAtUpdate = acTraj.LastStart().uCount;
		m_bGoal = !std::isnan(s.adGoal[0]);			// NaN: the high-level controller got no goal (CTrajectoryRT::Update)
		for (int k = 0; k < 3; k++)
			m_stIn.goal[k] = m_bGoal ? s.adGoal[k] : 0.0;
		m_stIn.delta = p.dDelta;
		m_uPlanner = ((s.uFlags & CTrajectoryRT::HL_IK_CONVERGED) ? (uint32_t)IPC_SHADOW_IK : 0u) |
		             ((s.uFlags & CTrajectoryRT::HL_OTG_NEW) ? (uint32_t)IPC_SHADOW_PLAN : 0u);
		for (int i = 0; i < IPC_CTRL_DOF; i++)
		{
			m_stIn.q_ready[i] = p.adQr[i];
			m_stIn.ik_q[i] = s.adTarget[i];			// the solution of the IK that this update took, if it took one
		}
	}

	/* adTrajQ, adTrajQd: the measured state that the Trajectory part has (its starts and its update of this period
	 * were from it); adLawQ, adLawQd: the measured state the control law was updated with, NULL: it was not
	 * updated; abRef: on the reference of this period, abFrictionFF: with the friction feedforward of the motion;
	 * adTau: its torque after the limit.
	 * Returns the payload of the period (valid until the next call), or NULL: the period had more than one
	 * measured state (see above). */
	const ipc_shadow_in_t* Finish(uint32_t auPeriod, const double* adTrajQ, const double* adTrajQd,
	                              const double* adLawQ, const double* adLawQd, const CTrajectoryRT& acTraj,
	                              const CControllerFullDynamicsRT& acCtl, bool abRef, bool abFrictionFF,
	                              const double* adTau)
	{
		const CTrajectoryRT::StartCall& stStart = acTraj.LastStart();
		const unsigned int uBefore = m_bUpdated ? m_uStartsAtUpdate - m_uStartsEnd : stStart.uCount - m_uStartsEnd;
		const unsigned int uAfter = m_bUpdated ? stStart.uCount - m_uStartsAtUpdate : 0u;
		const bool bStart = (uBefore + uAfter > 0), bLaw = (adLawQ != NULL);
		const size_t uBytes = sizeof(double) * IPC_CTRL_DOF;
		const bool bOneState = !bLaw || !(m_bUpdated || bStart) ||
		                       (memcmp(adTrajQ, adLawQ, uBytes) == 0 && memcmp(adTrajQd, adLawQd, uBytes) == 0);
		uint32_t uFlags = 0;
		if (m_bUpdated)
			uFlags |= IPC_SHADOW_TRAJ | (m_bGoal ? (uint32_t)IPC_SHADOW_GOAL : 0u) | m_uPlanner;
		if (bStart)
		{
			// one start is told, the last: an earlier one of the same period matters only if the update was its own
			uFlags |= IPC_SHADOW_START | (uAfter > 0 ? (uint32_t)IPC_SHADOW_START_AFTER : 0u) |
			          (stStart.bHome ? (uint32_t)IPC_SHADOW_START_HOME : 0u) |
			          ((uAfter > 0 && uBefore > 0) ? (uint32_t)IPC_SHADOW_STARTS_LOST : 0u);
			for (int i = 0; i < IPC_CTRL_DOF; i++)
				m_stIn.start_target[i] = stStart.adTarget[i];
		}
		if (bLaw)
			uFlags |= IPC_SHADOW_CTRL | (abRef ? (uint32_t)IPC_SHADOW_REF : 0u) |
			          ((abRef && abFrictionFF) ? (uint32_t)IPC_SHADOW_FRICTION_FF : 0u);
		if (acCtl.m_bStickyEnable.load(std::memory_order_relaxed))
			uFlags |= IPC_SHADOW_HOLD;
		m_stIn.period = auPeriod;
		m_stIn.flags = uFlags;
		m_stIn.starts = stStart.uCount;
		m_stIn.ctrl_mode = (uint32_t)acCtl.GetControlMode();
		for (int i = 0; i < IPC_CTRL_DOF; i++)
		{
			m_stIn.q[i] = bLaw ? adLawQ[i] : adTrajQ[i];
			m_stIn.qd[i] = bLaw ? adLawQd[i] : adTrajQd[i];
			m_stIn.tau[i] = bLaw ? adTau[i] : 0.0;
		}
		m_uStartsEnd = stStart.uCount;
		m_bUpdated = false;
		return bOneState ? &m_stIn : NULL;
	}

private:
	ipc_shadow_in_t	m_stIn;
	unsigned int	m_uStartsEnd;			// starts at the end of the period before
	unsigned int	m_uStartsAtUpdate;
	uint32_t		m_uPlanner;				// IPC_SHADOW_IK, IPC_SHADOW_PLAN of the update
	bool			m_bUpdated, m_bGoal;
};

#endif // G2T_SHADOW_WRITER_H
