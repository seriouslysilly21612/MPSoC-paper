/*****************************************************************************
*	Name: ShadowTask.h
*	Description: --shadow in the control task (--arch S1): every period it
*	tells the R5 what it did with its controller (shadow/ShadowWriter.h, the
*	Shadow in slot), takes the answer of the second controller there (Shadow
*	out slot), and counts: does the R5 compute the same torque, and how long
*	does it take. The torque of the R5 is not used.
*
*	    TrajState     the Trajectory part, in a cycle with a new State
*	    TrajUpdated   right after CTrajectoryRT::Update()
*	    Finish        the Dynamics part, when the torque of the cycle is
*	                  out and before the commands of the operator
*	    PrintSummary  after the tasks
*
*	One thread (the control task). An answer belongs to an earlier period
*	than the cycle that takes it (the R5 computes after its next send), so
*	the comparison of the torque is the R5's (tau_diff of the answer: it has
*	the torque of the APU in the payload). An answer also has what the
*	planner of the R5's controller has done so far: its plans, the goals it
*	solved beside the APU's planner and how far its solution of the last
*	was from the APU's, and the plans that were late for the R5's
*	high-level controller.
*****************************************************************************/
#ifndef G2T_SHADOW_TASK_H
#define G2T_SHADOW_TASK_H

#include <cstdio>

#include "r5link/R5Link.h"
#include "shadow/ShadowWriter.h"

class CShadowTask
{
public:
	/* a torque difference above this is a difference [N m]: far above the rounding of two compilers (1e-11 seen
	 * on an emulated R5), far below what a drive resolves */
	static constexpr double TAU_SAME = 1e-6;
	/* the same for the solution of an IK [rad]: far above that rounding, far below what the tolerance of the IK
	 * leaves open (0.1 mm at the tool) */
	static constexpr double IK_SAME = 1e-9;

	struct Counts
	{
		long	nPayloads, nWithheld;		// periods told; periods with two measured states, not told
		long	nAnswers, nOtherCheck;		// answers taken; of them with another check value than auCheck
		long	nApart, nFault;				// answers of a controller apart from this one; that failed its period
		long	nCompared, nDiffer;			// answers with a torque to compare; of them above TAU_SAME or no number
		double	dDiffMax;					// largest difference of the compared [N m]
		uint32_t uMissed;					// payloads the R5 did not get since its first answer here (by its count)
		long	nTraj, nCtrl;				// answers with the time of the high-level controller, of the law
		double	dTrajSumNs, dCtrlSumNs, dBothMaxNs;
		uint32_t uTrajMaxNs, uCtrlMaxNs;
		/* the planner of the R5's controller since its first answer here (by its counts): the plans and the IK it
		 * did, the longest of each among those that an answer showed, its plans that were not back in their period */
		uint32_t uR5Plans, uR5Iks, uR5PlanLate, uR5PlanMaxUs, uR5IkMaxUs;
		/* the goals that the R5's planner solved as this one did, since its first answer here (by its count); of
		 * those that an answer showed, the ones above IK_SAME or no number, and the largest difference [rad] */
		uint32_t uIkCompared;
		long	nIkDiffer;
		double	dIkDiffMax;
	};

	/* auCheck: the check value of this controller (ShadowCheckValue), which the R5's must have */
	CShadowTask(CR5Link* apLink, uint32_t auCheck)
		: m_pLink(apLink), m_uCheck(auCheck), m_bTrajNew(false), m_uTrajPeriod(0), m_uMissed0(0), m_uPlans0(0),
		  m_uIks0(0), m_uLate0(0), m_uIkCompared0(0), m_uPlansSeen(0), m_uIksSeen(0), m_uIkComparedSeen(0)
	{
		memset(&m_stCounts, 0, sizeof(m_stCounts));
		memset(m_adTrajQ, 0, sizeof(m_adTrajQ));
		memset(m_adTrajQd, 0, sizeof(m_adTrajQd));
	}

	void TrajState(uint32_t auPeriod, const double* adQ, const double* adQd)
	{
		m_bTrajNew = true;
		m_uTrajPeriod = auPeriod;
		for (int i = 0; i < IPC_CTRL_DOF; i++)
		{
			m_adTrajQ[i] = adQ[i];
			m_adTrajQd[i] = adQd[i];
		}
	}

	void TrajUpdated(const CTrajectoryRT& acTraj) { m_cWriter.TrajUpdated(acTraj); }

	/* abNewState: the Dynamics part had a new State in this cycle, of period auPeriod; adLawQ, adLawQd: the
	 * measured state its control law was updated with, NULL: it was not updated; the rest as CShadowWriter::Finish */
	void Finish(bool abNewState, uint32_t auPeriod, const double* adLawQ, const double* adLawQd,
	            const CTrajectoryRT& acTraj, const CControllerFullDynamicsRT& acCtl, bool abRef, bool abFrictionFF,
	            const double* adTau)
	{
		if (abNewState || m_bTrajNew)		// otherwise nothing happened in this cycle
		{
			const ipc_shadow_in_t* const pIn = m_cWriter.Finish(abNewState ? auPeriod : m_uTrajPeriod, m_adTrajQ, m_adTrajQd,
			                                                    adLawQ, adLawQd, acTraj, acCtl, abRef, abFrictionFF, adTau);
			m_bTrajNew = false;
			if (pIn != NULL)
			{
				m_pLink->SendShadow(*pIn);
				m_stCounts.nPayloads++;
			}
			else
				m_stCounts.nWithheld++;
		}
		ipc_shadow_out_t stOut;
		if (m_pLink->TakeShadow(&stOut))
			Take(stOut);
	}

	/* an answer of the second controller */
	void Take(const ipc_shadow_out_t& astOut)
	{
		Counts& c = m_stCounts;
		// the count of the R5 goes on over the runs of this side, and holds the time between them: from the first
		// answer here (or from a controller of the R5 that started again)
		if (c.nAnswers == 0 || astOut.n_missed < m_uMissed0)
		{
			m_uMissed0 = astOut.n_missed;
			m_uPlans0 = m_uPlansSeen = astOut.n_plans;
			m_uIks0 = m_uIksSeen = astOut.n_iks;
			m_uLate0 = astOut.n_plan_late;
			m_uIkCompared0 = m_uIkComparedSeen = astOut.n_ik_compared;
		}
		c.nAnswers++;
		c.uMissed = astOut.n_missed - m_uMissed0;
		if (astOut.ctrl_crc != m_uCheck)
		{
			c.nOtherCheck++;				// another controller: its torque is not this one's to compare
			return;
		}
		// its planner: a time is that of the last plan or IK, new when the count is
		if (astOut.n_plans != m_uPlansSeen && astOut.plan_us > c.uR5PlanMaxUs) c.uR5PlanMaxUs = astOut.plan_us;
		if (astOut.n_iks != m_uIksSeen && astOut.ik_us > c.uR5IkMaxUs) c.uR5IkMaxUs = astOut.ik_us;
		m_uPlansSeen = astOut.n_plans;
		m_uIksSeen = astOut.n_iks;
		c.uR5Plans = astOut.n_plans - m_uPlans0;
		c.uR5Iks = astOut.n_iks - m_uIks0;
		c.uR5PlanLate = astOut.n_plan_late - m_uLate0;
		// its solution of a goal against this planner's: the difference is of the last goal, new when the count is
		if (astOut.n_ik_compared != m_uIkComparedSeen)
		{
			if (!(astOut.ik_diff <= IK_SAME))
				c.nIkDiffer++;
			if (astOut.ik_diff > c.dIkDiffMax || astOut.ik_diff != astOut.ik_diff)	// no number: it stays
				c.dIkDiffMax = astOut.ik_diff;
		}
		m_uIkComparedSeen = astOut.n_ik_compared;
		c.uIkCompared = astOut.n_ik_compared - m_uIkCompared0;
		const bool bApart = (astOut.flags & IPC_SHADOW_OUT_APART) != 0;
		if (bApart) c.nApart++;
		if (astOut.flags & IPC_SHADOW_OUT_FAULT) c.nFault++;
		if ((astOut.flags & IPC_SHADOW_OUT_CTRL) && !bApart)
		{
			c.nCompared++;
			if (!(astOut.tau_diff <= TAU_SAME))
				c.nDiffer++;
			if (astOut.tau_diff > c.dDiffMax || astOut.tau_diff != astOut.tau_diff)	// no number: it stays
				c.dDiffMax = astOut.tau_diff;
		}
		if (astOut.flags & IPC_SHADOW_OUT_TRAJ)
		{
			c.nTraj++;
			c.dTrajSumNs += (double)astOut.traj_ns;
			if (astOut.traj_ns > c.uTrajMaxNs) c.uTrajMaxNs = astOut.traj_ns;
		}
		if (astOut.flags & IPC_SHADOW_OUT_CTRL)
		{
			c.nCtrl++;
			c.dCtrlSumNs += (double)astOut.ctrl_ns;
			if (astOut.ctrl_ns > c.uCtrlMaxNs) c.uCtrlMaxNs = astOut.ctrl_ns;
		}
		const double dBoth = (double)astOut.traj_ns + (double)astOut.ctrl_ns;
		if (dBoth > c.dBothMaxNs) c.dBothMaxNs = dBoth;
	}

	const Counts& GetCounts() const { return m_stCounts; }

	void PrintSummary() const
	{
		const Counts& c = m_stCounts;
		printf("[SHADOW] the control task told the R5 %ld periods (%ld with two measured states not told); the R5 "
		       "answered %ld, did not get %u since its first answer\n", c.nPayloads, c.nWithheld, c.nAnswers,
		       (unsigned)c.uMissed);
		if (c.nOtherCheck > 0)
			printf("[SHADOW] WARNING: %ld answers have another check value than this controller's 0x%08X: the firmware "
			       "was built from other constants or another model (ctrl_numbers_indy7.h, ctrl_model_indy7.h); they "
			       "are not compared\n", c.nOtherCheck, (unsigned)m_uCheck);
		printf("[SHADOW] torque of the R5's controller against this one's: %ld periods compared, %ld differ by more than "
		       "%.0e N m, largest difference %.3g N m; apart (after a period not told or not got, until the next "
		       "start) %ld, failed %ld\n", c.nCompared, c.nDiffer, TAU_SAME, c.dDiffMax, c.nApart, c.nFault);
		printf("[SHADOW] time on the R5: high-level controller mean %.1f us, max %.1f us (%ld periods); control law "
		       "mean %.1f us, max %.1f us (%ld periods); both together max %.1f us\n",
		       c.nTraj > 0 ? c.dTrajSumNs / (double)c.nTraj * 1e-3 : 0.0, (double)c.uTrajMaxNs * 1e-3, c.nTraj,
		       c.nCtrl > 0 ? c.dCtrlSumNs / (double)c.nCtrl * 1e-3 : 0.0, (double)c.uCtrlMaxNs * 1e-3, c.nCtrl,
		       c.dBothMaxNs * 1e-3);
		printf("[SHADOW] the planner of the R5: %u plans, the longest %u us from start to end; %u IK, the longest %u us "
		       "from its request to its result; %u plans not back in their period (asked for again)\n",
		       (unsigned)c.uR5Plans, (unsigned)c.uR5PlanMaxUs, (unsigned)c.uR5Iks, (unsigned)c.uR5IkMaxUs,
		       (unsigned)c.uR5PlanLate);
		printf("[SHADOW] the IK of the R5's planner against this one's: %u goals compared, %ld differ by more than "
		       "%.0e rad, largest difference %.3g rad\n", (unsigned)c.uIkCompared, c.nIkDiffer, IK_SAME, c.dIkDiffMax);
	}

private:
	CR5Link*		m_pLink;
	uint32_t		m_uCheck;
	CShadowWriter	m_cWriter;
	bool			m_bTrajNew;				// the Trajectory part had a new State in this cycle
	uint32_t		m_uTrajPeriod;
	uint32_t		m_uMissed0;				// the count of the R5 at its first answer here
	uint32_t		m_uPlans0, m_uIks0, m_uLate0, m_uIkCompared0;		// those of its planner then
	uint32_t		m_uPlansSeen, m_uIksSeen, m_uIkComparedSeen;		// and in the last answer
	double			m_adTrajQ[IPC_CTRL_DOF], m_adTrajQd[IPC_CTRL_DOF];	// the measured state it has
	Counts			m_stCounts;
};

#endif // G2T_SHADOW_TASK_H
