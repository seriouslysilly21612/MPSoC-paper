/*****************************************************************************
*	Name: HighLevelLink.h
*	Description: The high-level controller with its planner in another task
*	(HighLevelRT.h). What the 1 ms part asks for and what the planner has
*	finished pass through atomic slots (base/AtomicSlot.h), one writer each,
*	so neither task waits for the other.
*
*	    the task of the 1 ms part, every period:
*	        Before(hl);  hl.Update(...);  After(hl);
*	    the task of the planner, when it is released:
*	        while (Serve(planner)) { }
*
*	Serve() does one piece of work per call, a plan before the IK and the IK
*	one iteration at a time: a plan has H periods to be back in
*	(Params::nPlanAhead), an IK that ends at N_max takes longer than that on
*	the R5, and the IK of a goal that replaces another starts at once.
*	A result that the 1 ms part does not wait for any more has another
*	number than its request, and the 1 ms part does not take it.
*****************************************************************************/
#ifndef G2T_HIGH_LEVEL_LINK_H
#define G2T_HIGH_LEVEL_LINK_H

#include "HighLevelRT.h"
#include "base/AtomicSlot.h"

template <int N>
class CHlLinkN
{
public:
	typedef CHighLevelRTN<N> HL;

	/* what the planner has done so far (its task writes it, the summary reads it) */
	struct Stats
	{
		uint32_t	uPlans, uIks;
		double		dPlanUs, dPlanMaxUs;	// the calculation of a plan: the last, the longest
		double		dIkUs, dIkMaxUs;		// an IK from its request to its result: the last, the longest
	};

	CHlLinkN() : m_uIkSent(0), m_uPlanSent(0), m_uIkGot(0), m_uPlanGot(0), m_stIkIn(), m_stPlanIn(),
	             m_uIkSeen(0), m_uPlanSeen(0), m_stIkReq(), m_stPlanReq(), m_stIkOut(), m_stPlanOut(), m_stStats(),
	             m_tIkBegin(0) { }

	/* --- the task of the 1 ms part --- */
	/* before Update(): the results that came since the last call. With apIk the result of an IK goes there and
	 * not to the high-level controller (the second controller of --shadow has the IK of a goal from the
	 * controller it follows, and compares what its own planner found with it) */
	void Before(HL& acHl, typename HL::IkResult* apIk = NULL)
	{
		uint32_t uCount = 0;
		if (m_cIkRes.Count() != m_uIkGot && m_cIkRes.Read(m_stIkIn, &uCount))
		{
			m_uIkGot = uCount;
			if (apIk == NULL)
				acHl.TakeIk(m_stIkIn);
			else
				*apIk = m_stIkIn;
		}
		if (m_cPlanRes.Count() != m_uPlanGot && m_cPlanRes.Read(m_stPlanIn, &uCount))
		{
			m_uPlanGot = uCount;
			acHl.TakePlan(m_stPlanIn);
		}
	}
	/* after Update(): what it asked for in this period */
	void After(const HL& acHl)
	{
		if (acHl.GetIkRequest().uSeq != m_uIkSent)
		{
			m_uIkSent = acHl.GetIkRequest().uSeq;
			m_cIkReq.Write(acHl.GetIkRequest());
		}
		if (acHl.GetPlanRequest().uSeq != m_uPlanSent)
		{
			m_uPlanSent = acHl.GetPlanRequest().uSeq;
			m_cPlanReq.Write(acHl.GetPlanRequest());
		}
	}

	/* --- the task of the planner --- */
	/* true: it did something, call again; false: nothing is asked for */
	bool Serve(CHlPlannerN<N>& acPlanner)
	{
		uint32_t uCount = 0;
		if (m_cPlanReq.Count() != m_uPlanSeen && m_cPlanReq.Read(m_stPlanReq, &uCount))
		{
			m_uPlanSeen = uCount;
			acPlanner.Plan(m_stPlanReq, &m_stPlanOut);
			m_cPlanRes.Write(m_stPlanOut);
			m_stStats.uPlans++;
			m_stStats.dPlanUs = m_stPlanOut.dCalcUs;
			if (m_stPlanOut.dCalcUs > m_stStats.dPlanMaxUs)
				m_stStats.dPlanMaxUs = m_stPlanOut.dCalcUs;
			m_cStats.Write(m_stStats);
			return true;
		}
		if (m_cIkReq.Count() != m_uIkSeen && m_cIkReq.Read(m_stIkReq, &uCount))
		{
			m_uIkSeen = uCount;
			acPlanner.IkBegin(m_stIkReq);		// in place of the IK that runs, if one does
			m_tIkBegin = read_timer();
		}
		if (!acPlanner.IkActive())
			return false;
		if (acPlanner.IkStep(&m_stIkOut))
		{
			m_cIkRes.Write(m_stIkOut);
			const double dUs = (double)(read_timer() - m_tIkBegin) * 1e-3;
			m_stStats.uIks++;
			m_stStats.dIkUs = dUs;
			if (dUs > m_stStats.dIkMaxUs)
				m_stStats.dIkMaxUs = dUs;
			m_cStats.Write(m_stStats);
		}
		return true;
	}

	/* --- any task --- */
	/* false, and astStats as it was: the planner has done nothing yet, or its task is in the middle of writing them
	 * and cannot go on while the caller runs (the two on one core, as on the R5) */
	bool GetStats(Stats& astStats) const { return m_cStats.Read(astStats); }

private:
	CAtomicSlot<typename HL::IkRequest>		m_cIkReq;		// written by the task of the 1 ms part
	CAtomicSlot<typename HL::PlanRequest>	m_cPlanReq;
	CAtomicSlot<typename HL::IkResult>		m_cIkRes;		// written by the task of the planner
	CAtomicSlot<typename HL::PlanResult>	m_cPlanRes;
	CAtomicSlot<Stats>						m_cStats;

	/* of the task of the 1 ms part */
	uint32_t	m_uIkSent, m_uPlanSent;		// the numbers of the requests it wrote last
	uint32_t	m_uIkGot, m_uPlanGot;		// the counts of the result slots it read last
	typename HL::IkResult	m_stIkIn;
	typename HL::PlanResult	m_stPlanIn;

	/* of the task of the planner */
	uint32_t	m_uIkSeen, m_uPlanSeen;		// the counts of the request slots it read last
	typename HL::IkRequest		m_stIkReq;
	typename HL::PlanRequest	m_stPlanReq;
	typename HL::IkResult		m_stIkOut;
	typename HL::PlanResult		m_stPlanOut;
	Stats		m_stStats;
	RTTIME		m_tIkBegin;
};

typedef CHlLinkN<6> CHlLink;

#endif // G2T_HIGH_LEVEL_LINK_H
