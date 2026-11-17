/*****************************************************************************
*	Name: ObsEstimate.h
*	Description: The object of the camera application from the results that
*	the control task used (obs_receiver.h): the smoothing of GoalFilter.h on
*	the receiving side, with a velocity for an object that moves (D-31).
*
*	A result that a period uses for the first time is folded in, by its
*	capture time (dt since the capture of the result before):
*	    p- = p + v dt,   r = z - p-,   p = p- + alpha r,   v = v + (beta / dt) r
*	With beta = 0 the velocity stays 0, and this is the moving average of the
*	bridge, p = p + kappa (z - p). The estimate restarts from the result when
*	the result is of another target (the operator chose another object), or
*	after a gap of more than dGapS between two captures.
*	A period with a result in use gets the object at its release,
*	p + v (t_k - t); a period without one keeps the object it has. The target
*	is lost after dLossS without a folded result.
*	No allocation, no clock: the caller passes the times, in ticks.
*****************************************************************************/
#ifndef G2T_OBS_ESTIMATE_H
#define G2T_OBS_ESTIMATE_H

#include <cstdint>

class CObsEstimate
{
public:
	struct Params
	{
		double dAlpha;       // weight of the newest result on the position (0 < alpha <= 1)
		double dBeta;        // weight of the newest result on the velocity; 0: the object stands still
		double dGapS;        // restart after this gap between two captures [s]
		double dLossS;       // the target is lost after this time without a folded result [s]
		double dTicksPerS;   // ticks of the times in a second
	};

	CObsEstimate() { Reset(); }

	void SetParams(const Params& astP)
	{
		m_stP = astP;
		if (!(m_stP.dAlpha > 0.0) || m_stP.dAlpha > 1.0)
			m_stP.dAlpha = 1.0;   // as CGoalFilter: no smoothing rather than an estimate that never moves
		Reset();
	}

	void Reset()
	{
		m_bSeeded = m_bRestarted = m_bFolded = false;
		m_uTe = m_uFoldTk = 0;
		m_uTarget = 0;
		m_nFolded = m_nRestarts = 0;
		for (int k = 0; k < 3; k++)
			m_adP[k] = m_adV[k] = 0.0;
	}

	/* a result that the period of release auTk uses for the first time: the object [m, base frame], its capture
	 * time and its target. false: its capture is not after the last one folded; nothing changes. */
	bool Fold(const double* adObj, uint64_t auTe, uint32_t auTarget, uint64_t auTk)
	{
		if (m_bSeeded && auTe <= m_uTe)
			return false;
		const double dDt = (double)(auTe - m_uTe) / m_stP.dTicksPerS;
		m_bRestarted = !m_bSeeded || auTarget != m_uTarget || dDt > m_stP.dGapS;
		for (int k = 0; k < 3; k++)
		{
			const double dPrior = m_adP[k] + m_adV[k] * dDt, dR = adObj[k] - dPrior;
			m_adP[k] = m_bRestarted ? adObj[k] : dPrior + m_stP.dAlpha * dR;
			m_adV[k] = m_bRestarted ? 0.0 : m_adV[k] + m_stP.dBeta / dDt * dR;
		}
		if (m_bRestarted)
			m_nRestarts++;
		m_bSeeded = m_bFolded = true;
		m_uTe = auTe;
		m_uTarget = auTarget;
		m_uFoldTk = auTk;
		m_nFolded++;
		return true;
	}

	/* the object for the period of release auTk; abInUse: the period has a result in use. true: adObj is the
	 * object at auTk and differs from the last one given (a result was folded, or the object moves). false: the
	 * last one stands, adObj is not written. */
	bool Object(uint64_t auTk, bool abInUse, double* adObj)
	{
		if (!m_bFolded && !(abInUse && m_bSeeded && m_stP.dBeta != 0.0))
			return false;
		m_bFolded = false;
		const double dDt = (double)(auTk - m_uTe) / m_stP.dTicksPerS;
		for (int k = 0; k < 3; k++)
			adObj[k] = m_adP[k] + m_adV[k] * dDt;
		return true;
	}

	bool     Lost(uint64_t auTk) const { return m_bSeeded && (double)(auTk - m_uFoldTk) / m_stP.dTicksPerS > m_stP.dLossS; }
	bool     Restarted() const { return m_bRestarted; }     // the last folded result restarted the estimate
	bool     HasFolded() const { return m_bSeeded; }
	long     Folded() const { return m_nFolded; }
	long     Restarts() const { return m_nRestarts; }
	uint32_t Target() const { return m_uTarget; }

private:
	Params   m_stP{0.4, 0.0, 1.0, 1.0, 1e9};
	bool     m_bSeeded, m_bRestarted, m_bFolded;
	uint64_t m_uTe;         // capture of the last folded result
	uint64_t m_uFoldTk;     // release of the period that folded it
	uint32_t m_uTarget;
	double   m_adP[3], m_adV[3];
	long     m_nFolded, m_nRestarts;
};

#endif // G2T_OBS_ESTIMATE_H
