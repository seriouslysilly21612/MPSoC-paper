/*****************************************************************************
*	Name: ObsGoal.h
*	Description: The control task's side of the camera application for the
*	methods with the mailbox (ObsMethod.h): in every period, from the results
*	in the mailbox to the object position in the goal slot of the
*	application, which the high-level controller reads as it reads that of
*	the bridge (M1).
*
*	  state     the joint angles of the period go into the record of the
*	            receiver, with the time they belong to: they are nStateAge
*	            periods old when the period is released.
*	  step      obs_receiver.h: the result that this period may use, by the
*	            policy of the method.
*	  object    a result used for the first time gives the object in the
*	            base frame; with a result relative to the tag, from the tag
*	            at the joint angles of the result. It must pass the workspace
*	            checks of GoalFilter.h: the bridge does them for M1, but a
*	            result relative to the tag has no place in the base frame
*	            before this point.
*	  estimate  ObsEstimate.h. A new object position goes into the goal slot.
*	  entries   what this period did with every result that left the
*	            receiver, for the log (ObsLog.h).
*	  status    what this period tells the camera side (obs_status.h): its
*	            tolerance and the result it used last.
*
*	One caller: the control task. No lock, no allocation, no system call.
*****************************************************************************/
#ifndef G2T_OBS_GOAL_H
#define G2T_OBS_GOAL_H

#include <cmath>
#include <cstdint>

#include "GoalFilter.h"
#include "GoalSlot.h"
#include "obs/ObsEstimate.h"
#include "obs/obs_receiver.h"
#include "obs/obs_status.h"

class CObsGoal
{
public:
	enum { MAX_STATE_AGE = 8 };		// [periods]

	struct Config
	{
		obs_settings_t		stSet;		// the policy, the period, the age cap and eq. (7)
		double				dTolM;		// the tolerance of every period: delta of the contract [m]
		int					nStateAge;	// the joint angles of a period are this old at its release [periods]
		CGoalFilter::Params	stFilter;	// the workspace checks; dKappa and dGapS: the estimate
		double				dLossS;		// the target is lost after this time without a folded result [s]
		/* the position adP[3] of the tool tag in the base frame at the joint angles afQ[OBS_DOF] */
		void				(*pTagAt)(void* apArg, const float* afQ, double* adP);
		void*				pTagArg;
	};

	/* what a period did with a result */
	struct Entry
	{
		obs_record_t	stRec;			// the result and the verdict of the receiver
		int				nWorkspace;		// a result used for the first time: CGoalFilter::eVerdict of its object; else -1
		int				nFolded;		// 1: the estimate took it; 2: and started again from it
		double			adObj[3];		// its object in the base frame [m] (nWorkspace >= 0)
	};

	struct Counts
	{
		uint32_t	uTaken;								// results out of the mailbox
		uint32_t	auVerdict[OBS_VERDICTS];
		long		anWorkspace[CGoalFilter::eVerdicts];	// the objects of the results used
		long		nFolded, nRestarts;					// the estimate
		long		nGoals;								// object positions written into the goal slot

		void Minus(const Counts& astAt0)				// the counts since astAt0
		{
			uTaken -= astAt0.uTaken;
			for (int v = 0; v < (int)OBS_VERDICTS; v++)
				auVerdict[v] -= astAt0.auVerdict[v];
			for (int v = 0; v < (int)CGoalFilter::eVerdicts; v++)
				anWorkspace[v] -= astAt0.anWorkspace[v];
			nFolded -= astAt0.nFolded;
			nRestarts -= astAt0.nRestarts;
			nGoals -= astAt0.nGoals;
		}
	};

	/* before the control task starts. false: a setting cannot be used (the tolerance and the terms of eq. (7) must
	 * be finite and not negative, the tolerance above 0, the age cap below the record of the states) */
	bool Init(volatile obs_mailbox_t* apBox, const Config& astCfg, CGoalSlot* apSlot)
	{
		const obs_settings_t& s = astCfg.stSet;
		const double adTerm[5] = { s.e0, s.g, s.eps_v, s.a_max, s.eps_cal };
		for (double dTerm : adTerm)
			if (!std::isfinite(dTerm) || dTerm < 0.0)
				return false;
		if (apBox == NULL || apSlot == NULL || astCfg.pTagAt == NULL || !std::isfinite(astCfg.dTolM) ||
		    !(astCfg.dTolM > 0.0) || astCfg.nStateAge < 0 || astCfg.nStateAge > (int)MAX_STATE_AGE ||
		    !obs_receiver_start(&m_stR, &s))
			return false;
		obs_receiver_set_clock(&m_stR, m_pNow);		// the start cleared it
		m_pBox = apBox;
		m_pSlot = apSlot;
		m_stCfg = astCfg;
		m_uStateAge = (uint64_t)astCfg.nStateAge * s.period_ticks;
		m_cCheck.SetParams(astCfg.stFilter);
		const CObsEstimate::Params stE = { astCfg.stFilter.dKappa, 0.0, astCfg.stFilter.dGapS, astCfg.dLossS,
		                                   s.ticks_per_s };		// beta 0: the target rests
		m_cEst.SetParams(stE);
		for (int v = 0; v < (int)CGoalFilter::eVerdicts; v++)
			m_anWorkspace[v] = 0;
		m_nEntries = 0;
		m_nGoals = 0;
		m_bLost = false;
		m_stStatus = obs_status_t();
		obs_reader_start(apBox);
		return true;
	}

	/* one period: its number, its release, and the joint angles that its state brought (NULL: none, the bus is
	 * not up) */
	void Step(uint32_t auPeriod, uint64_t atRelease, const double* adQ)
	{
		if (adQ != NULL && atRelease > m_uStateAge)
		{
			float afQ[OBS_DOF];
			for (int i = 0; i < (int)OBS_DOF; i++)
				afQ[i] = (float)adQ[i];
			obs_state_push(&m_stR, auPeriod, atRelease - m_uStateAge, afQ);
		}
		int nFresh = 0;
		const obs_held_t* const pUse = obs_step(&m_stR, m_pBox, atRelease, m_stCfg.dTolM, &nFresh);
		m_nEntries = m_stR.n_rec;
		for (unsigned i = 0; i < m_nEntries; i++)
		{
			Entry& stE = m_aEntry[i];
			stE.stRec = m_stR.rec[i];
			stE.nWorkspace = -1;
			stE.nFolded = 0;
			stE.adObj[0] = stE.adObj[1] = stE.adObj[2] = 0.0;
			if (stE.stRec.verdict != OBS_V_ACCEPTED)		// accepted: the result that this period uses for the first time
				continue;
			const obs_held_t& stH = stE.stRec.h;
			obs_object(&stH, m_stCfg.pTagAt, m_stCfg.pTagArg, stE.adObj);
			const CGoalFilter::eVerdict eV = m_cCheck.Check(true, stE.adObj);
			stE.nWorkspace = (int)eV;
			m_anWorkspace[eV]++;
			if (eV == CGoalFilter::eAccepted && m_cEst.Fold(stE.adObj, stH.d.t_e, stH.d.target_seq, atRelease))
				stE.nFolded = m_cEst.Restarted() ? 2 : 1;
		}
		double adObj[3];
		if (m_cEst.Object(atRelease, pUse != NULL, adObj))
		{
			m_pSlot->Write(adObj);
			m_nGoals++;
		}
		m_bLost = m_cEst.Lost(atRelease);

		m_stStatus.t_release = atRelease;
		m_stStatus.tolerance = (float)m_stCfg.dTolM;
		m_stStatus.control_period = (float)((double)m_stCfg.stSet.period_ticks / m_stCfg.stSet.ticks_per_s);
		if (pUse != NULL)		// else the fields of the result used last stay: the camera side reads its deadline
		{
			m_stStatus.result_t_e = pUse->d.t_e;
			m_stStatus.result_t_valid_until = pUse->d.t_valid_until;
			m_stStatus.result_n = pUse->d.n;
			m_stStatus.result_mode = pUse->d.mode;
			m_stStatus.target_seq = pUse->d.target_seq;
			m_stStatus.result_error_bound = pUse->ebar;
		}
		m_stStatus.flags = (uint8_t)((pUse != NULL ? OBS_STATUS_RESULT_IN_USE : 0u) |
		                             (m_bLost ? OBS_STATUS_TARGET_LOST : 0u));
	}

	/* the last Step() */
	unsigned		Entries() const { return m_nEntries; }
	const Entry&	GetEntry(unsigned auI) const { return m_aEntry[auI]; }
	bool			Lost() const { return m_bLost; }
	/* results so far that the receiver accepted and the workspace checks refused */
	long OutsideWorkspace() const
	{
		long nOutside = 0;
		for (int v = 0; v < (int)CGoalFilter::eVerdicts; v++)
			if (v != (int)CGoalFilter::eAccepted)
				nOutside += m_anWorkspace[v];
		return nOutside;
	}
	/* the clock of the reader's cost (eq. (6), obs_receiver.h) from now on and after every Init(); NULL: none */
	void SetClock(uint64_t (*apNow)(void))
	{
		m_pNow = apNow;
		obs_receiver_set_clock(&m_stR, apNow);
	}
	const obs_cost_t&	Cost() const { return m_stR.cost; }
	void				ClearCost() { obs_cost_clear(&m_stR); }
	/* what this class knows of the status: the caller adds response_max, contract_crc and method */
	const obs_status_t&	Status() const { return m_stStatus; }

	void GetCounts(Counts& astC) const
	{
		astC.uTaken = m_stR.taken;
		for (int v = 0; v < (int)OBS_VERDICTS; v++)
			astC.auVerdict[v] = m_stR.count[v];
		for (int v = 0; v < (int)CGoalFilter::eVerdicts; v++)
			astC.anWorkspace[v] = m_anWorkspace[v];
		astC.nFolded = m_cEst.Folded();
		astC.nRestarts = m_cEst.Restarts();
		astC.nGoals = m_nGoals;
	}
	static const char* VerdictName(int anVerdict)
	{
		static const char* const s_apc[OBS_VERDICTS] = { "accepted", "superseded", "expired", "state_unrecoverable",
		                                                 "seq_fail", "error_exceeded", "evicted" };
		return (anVerdict >= 0 && anVerdict < (int)OBS_VERDICTS) ? s_apc[anVerdict] : "?";
	}

private:
	volatile obs_mailbox_t*	m_pBox{NULL};
	CGoalSlot*				m_pSlot{NULL};
	Config					m_stCfg{};
	uint64_t				m_uStateAge{0};		// [ticks]
	obs_receiver_t			m_stR{};
	uint64_t				(*m_pNow)(void){NULL};	// of SetClock()
	CGoalFilter				m_cCheck;			// Check() only: the average is that of m_cEst
	CObsEstimate			m_cEst;
	Entry					m_aEntry[OBS_RECORDS]{};
	unsigned				m_nEntries{0};
	long					m_anWorkspace[CGoalFilter::eVerdicts]{};
	long					m_nGoals{0};
	bool					m_bLost{false};
	obs_status_t			m_stStatus{};
};

#endif // G2T_OBS_GOAL_H
