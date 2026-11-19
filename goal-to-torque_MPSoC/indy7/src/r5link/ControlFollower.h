/*****************************************************************************
*	Name: ControlFollower.h
*	Description: The control of the R5 a second time, here (--arch S2 with
*	--shadow): a controller of the application (shadow/ShadowControl.h, the
*	one that the R5 has beside the control task of S1) fed with the report
*	of every period of the R5's control task. A report begins with the
*	description of its period, in the form in which the control task of S1
*	tells the R5 (ipc_shadow_in_t): the measured state, the goal, the start,
*	the mode of the law, the IK that the update took, and the torque that
*	the law gave. The controller here repeats the period from it, and its
*	torque and its reference are compared with those of the report: what the
*	report says was done on the R5 is then what this code does on the APU.
*
*	Its planner is served here as well, after every period, so its plans
*	are back in their period; a plan that the planner of the R5 had late
*	leaves the two apart until the next start, as does a report that did
*	not arrive (ShadowControl.h). The periods in which they are apart are
*	counted, not compared.
*
*	Take() and what it calls: the control task. No allocation after Init().
*****************************************************************************/
#ifndef G2T_R5LINK_CONTROL_FOLLOWER_H
#define G2T_R5LINK_CONTROL_FOLLOWER_H

#include <cmath>
#include <cstring>

#include "shadow/ShadowControl.h"

class CControlFollower
{
public:
	/* a torque differs: by more than this [N m] (the two CPUs do not round alike); a reference: [rad] */
	static constexpr double TAU_TOL = 1e-6;
	static constexpr double REF_TOL = 1e-9;

	struct Counts
	{
		long	nPeriods;			// reports of a period with a state that this controller repeated
		long	nApart;				// of them: not where the R5's controller is (not compared)
		long	nLaw, nTraj;		// compared: the torque of the law, the reference
		long	nTauDiff, nRefDiff;	// of them: differ by more than TAU_TOL, REF_TOL
		double	dTauDiffMax;		// the largest difference of a torque [N m] ...
		double	dRefDiffMax;		// ... and of a reference [rad]
		double	dTrajNsSum, dLawNsSum;		// the time of the two parts here, in the periods compared
		uint32_t uTrajNsMax, uLawNsMax;
	};

	CControlFollower() { memset(&m_stCounts, 0, sizeof(m_stCounts)); }

	/* the controller of the numbers and of the model, as the R5 has it (heap: before the periods start) */
	BOOL Init(const char* astrURDF, const double* adNumbers, uint32_t auCrc) { return m_cCtl.Init(astrURDF, adNumbers, auCrc); }

	/* A report of a period with a state of the arm: that period a second time. auStatePeriod: a number that goes
	 * up by one from one such report to the next when none is missing between them (CControlClient::StatePeriod).
	 * The controller takes a gap as a period that it was not told. */
	void Take(const ipc_report_t& r, uint32_t auStatePeriod)
	{
		if (!(r.state & IPC_CTL_STATE))
			return;
		ipc_shadow_in_t stIn = r.in;
		stIn.period = auStatePeriod;
		ipc_shadow_out_t f;
		m_cCtl.Step(stIn, &f);
		while (m_cCtl.ServePlanner())
			;
		m_stCounts.nPeriods++;
		if (f.flags & IPC_SHADOW_OUT_APART)
		{
			m_stCounts.nApart++;
			return;
		}
		if (f.flags & IPC_SHADOW_OUT_CTRL)
		{
			m_stCounts.nLaw++;
			m_stCounts.dLawNsSum += (double)f.ctrl_ns;
			if (f.ctrl_ns > m_stCounts.uLawNsMax)
				m_stCounts.uLawNsMax = f.ctrl_ns;
			// a difference that is no number counts as one
			if (!(f.tau_diff <= TAU_TOL))
				m_stCounts.nTauDiff++;
			if (f.tau_diff > m_stCounts.dTauDiffMax)
				m_stCounts.dTauDiffMax = f.tau_diff;
		}
		if (f.flags & IPC_SHADOW_OUT_TRAJ)
		{
			double dDiff = 0.0;
			bool bNumber = true;
			for (int i = 0; i < IPC_CTRL_DOF; i++)
			{
				const double d = std::fabs(f.q_ref[i] - r.q_ref[i]);
				if (!(d == d))
					bNumber = false;
				else if (d > dDiff)
					dDiff = d;
			}
			m_stCounts.nTraj++;
			m_stCounts.dTrajNsSum += (double)f.traj_ns;
			if (f.traj_ns > m_stCounts.uTrajNsMax)
				m_stCounts.uTrajNsMax = f.traj_ns;
			if (!bNumber || dDiff > REF_TOL)
				m_stCounts.nRefDiff++;
			if (dDiff > m_stCounts.dRefDiffMax)
				m_stCounts.dRefDiffMax = dDiff;
		}
	}
	const Counts& GetCounts() const { return m_stCounts; }

private:
	CShadowControl	m_cCtl;
	Counts			m_stCounts;
};

#endif // G2T_R5LINK_CONTROL_FOLLOWER_H
