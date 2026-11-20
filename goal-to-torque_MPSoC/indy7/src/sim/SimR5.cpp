/*****************************************************************************
*	Name: SimR5.cpp
*	Description: The R5 of the simulation. See SimR5.h.
*****************************************************************************/
#include "SimR5.h"

#include <cerrno>

#include "../base/RtConsole.h"
#include "../r5link/LinkClock.h"
#include "../shadow/ShadowControl.h"

namespace {

void SleepUntil(RTTIME atNs)
{
	struct timespec stTs;
	stTs.tv_sec  = (time_t)(atNs / 1000000000ULL);
	stTs.tv_nsec = (long)(atNs % 1000000000ULL);
	while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &stTs, NULL) == EINTR)
		;
}

/* the first releases come this long after Start() */
const RTTIME START_LEAD_NS = 20000000ULL;

} // namespace

CSimR5::CSimR5() : m_pArm(NULL)
{
	for (size_t i = 0; i < sizeof(m_auRegion) / sizeof(m_auRegion[0]); i++)
		m_auRegion[i] = 0u;
	memset(&m_stSpec, 0, sizeof(m_stSpec));
	memset(&m_stDrives, 0, sizeof(m_stDrives));
	memset(&m_stCounts, 0, sizeof(m_stCounts));
}

BOOL
CSimR5::Start(const Spec& astSpec, CSimArm* apArm, const std::vector<CSimJoint*>& avJoints)
{
	const bool bControl = (m_pControl != NULL);
	if (astSpec.tPeriod == 0 || astSpec.nStaleLimit < 1 ||
	    (!bControl && (apArm == NULL || avJoints.empty() || avJoints.size() > (size_t)IPC_MAX_DOF)) ||
	    (bControl && (apArm != NULL || !avJoints.empty() || astSpec.uDrives == 0 || astSpec.uDrives > (unsigned)IPC_MAX_DOF)))
	{
		DBG_LOG_ERROR("(CSimR5) no arm, %u joints, or no period; with the control task: an arm, or no drives on its bus",
		              (unsigned)avJoints.size());
		return FALSE;
	}
	m_stSpec = astSpec;
	m_pArm = apArm;
	m_vJoints = avJoints;
	m_stDrives.uDof = (unsigned)avJoints.size();
	const uint32_t uEpoch = bControl ? m_cControlR5.Start(m_pControl, m_auRegion, astSpec.uCfgCrc, astSpec.uDrives) :
	                                   m_cSlots.Start(m_auRegion, astSpec.uCfgCrc, astSpec.nStaleLimit);

	CRtTask::Spec stTask;
	stTask.strName = "Sim R5";
	stTask.nPriority = astSpec.nPriority;
	stTask.nCpu = astSpec.nCpu;
	stTask.tPeriod = 0;			// its grid is its own (Run)
	stTask.tStartDelay = 0;
	m_bStop.store(false);
	m_bEnded.store(false);
	if (FALSE == m_cTask.Create(stTask, 0) || FALSE == m_cTask.Start(&CSimR5::Entry, this))
	{
		m_bEnded.store(true);
		return FALSE;
	}
	for (int nWait = 0; nWait < 1000 && m_cTask.GetCheck() == CRtTask::eCheckPending; nWait++)
		usleep(1000);
	if (m_cTask.GetCheck() != CRtTask::eCheckOk)
	{
		DBG_LOG_ERROR("(CSimR5) the thread of the simulated R5 does not run as requested");
		Stop();
		return FALSE;
	}
	if (bControl)
		printf("[SIM R5] the R5 of the simulation with the control task: epoch %u, a bus of %u drives that stay off, "
		       "period %.3f ms; the arm is the application's (Sim state slot)\n", (unsigned)uEpoch, astSpec.uDrives,
		       (double)astSpec.tPeriod * 1e-6);
	else
		printf("[SIM R5] the R5 of the simulation: epoch %u, %u joints, period %.3f ms, servo off after %d periods "
		       "without a new torque\n", (unsigned)uEpoch, (unsigned)m_vJoints.size(), (double)astSpec.tPeriod * 1e-6,
		       astSpec.nStaleLimit);
	return TRUE;
}

void
CSimR5::Stop()
{
	m_bStop.store(true, std::memory_order_release);
	for (int nWait = 0; nWait < 2000 && !m_bEnded.load(std::memory_order_acquire); nWait++)
		usleep(1000);
}

void
CSimR5::PowerCut(int anCutOrRestore, int anReports)
{
	m_nPowerReports.store(anReports, std::memory_order_relaxed);
	m_nPowerCmd.store(anCutOrRestore, std::memory_order_release);
}

void
CSimR5::Entry(void* apThis)
{
	rt_console::MarkRealTime();
	static_cast<CSimR5*>(apThis)->Run();
}

void
CSimR5::Run()
{
	const RTTIME tPeriod = m_stSpec.tPeriod;
	// the offset of the counter, before the first release needs it
	for (int nTry = 0; nTry < 100 && !m_cClock.Sample(); nTry++)
		;
	RTTIME tRelease = read_timer() + START_LEAD_NS;
	uint32_t uPeriod = 0;
	while (!m_bStop.load(std::memory_order_acquire))
	{
		SleepUntil(tRelease);
		uPeriod++;
		// the releases that passed meanwhile are left out, with their numbers
		const RTTIME tNow = read_timer();
		if (tNow >= tRelease + tPeriod)
		{
			const RTTIME tLeftOut = (tNow - tRelease) / tPeriod;
			uPeriod += (uint32_t)tLeftOut;
			tRelease += tLeftOut * tPeriod;
			m_stCounts.nLeftOut += (long)tLeftOut;
		}
		// the release on the counter: with an offset that could not be taken now, the last one serves
		(void)m_cClock.Sample();
		Cycle(m_cClock.TicksOf(tRelease), uPeriod);
		const RTTIME tEnd = read_timer();
		m_stCounts.nPeriods++;
		if (tEnd - tRelease > m_stCounts.tCycleMax)
			m_stCounts.tCycleMax = tEnd - tRelease;
		tRelease += tPeriod;
		if (tEnd > tRelease)
			m_stCounts.nOverruns++;
	}
	for (CSimJoint* p : m_vJoints)
		p->ServoOff();
	m_bEnded.store(true, std::memory_order_release);
}

void
CSimR5::Cycle(uint64_t auReleaseTicks, uint32_t auPeriod)
{
	if (m_pControl != NULL)		// --arch S2: the period of a firmware with the control task
	{
		m_cControlR5.Cycle(auReleaseTicks, auPeriod, (uint32_t)m_stCounts.nOverruns);
		return;
	}
	const unsigned uDof = m_stDrives.uDof;
	double adTau[CSimArm::MAX_DOF];
	BOOL   abReleased[CSimArm::MAX_DOF];

	/* receive: the measurement, the fault tests, and the status of the drives as this period finds them */
	m_pArm->GetMeasured(m_stDrives.adQ, m_stDrives.adQd, m_stDrives.adTau);
	for (unsigned i = 0; i < uDof; i++)
		m_vJoints[i]->SetSimState(m_stDrives.adQ[i], m_stDrives.adQd[i], m_stDrives.adTau[i]);
	if (m_nDropout.load(std::memory_order_relaxed) != 0)
	{
		const int nJoint = m_nDropout.exchange(0, std::memory_order_acq_rel);
		if (nJoint >= 1 && nJoint <= (int)uDof)
			m_vJoints[nJoint - 1]->SimDropout();
	}
	if (m_nPowerCmd.load(std::memory_order_relaxed) != 0)
	{
		const int nCmd = m_nPowerCmd.exchange(0, std::memory_order_acq_rel);
		for (unsigned i = 0; i < uDof; i++)
			m_vJoints[i]->SetPowerCut(nCmd == 1 ? TRUE : FALSE, m_nPowerReports.load(std::memory_order_relaxed));
	}
	for (unsigned i = 0; i < uDof; i++)
		m_stDrives.ausStatus[i] = (uint16_t)m_vJoints[i]->GetStatusWord();

	/* the slots of the APU and the stale-torque rule */
	if (m_cSlots.TakeApuSlots(m_stDrives))
	{
		m_stCounts.nStaleStops++;
		printf("[SIM R5] no new torque for %d periods (period %u): servo off\n", m_stSpec.nStaleLimit,
		       (unsigned)auPeriod);
	}

	/* the drives */
	for (unsigned i = 0; i < uDof; i++)
	{
		if (m_cSlots.IsServoAsked())
			m_vJoints[i]->ServoOn();
		else
			m_vJoints[i]->ServoOff();
		// a simulated drive is enabled at once and its brake with it: it has its torque in the same step
		const bool bGive = m_cSlots.IsServoAsked() && m_cSlots.IsTorqueUsable() && m_vJoints[i]->IsServoOn();
		m_vJoints[i]->MoveTorque(bGive ? m_cSlots.Torque()[i] : 0.0);
	}

	/* the State slot of this period */
	m_cSlots.PublishState(auReleaseTicks, auPeriod, m_stDrives, (uint32_t)m_stCounts.nOverruns);

	/* send: the arm moves */
	for (unsigned i = 0; i < uDof; i++)
	{
		adTau[i] = m_vJoints[i]->GetCommandTorque();
		abReleased[i] = m_vJoints[i]->IsBrakeReleased();
	}
	m_pArm->Step(adTau, abReleased);

	/* --shadow, after the send: the period that the control task told last, through the second controller */
	ipc_shadow_in_t stIn;
	if (m_pShadow != NULL && m_cSlots.TakeShadow(&stIn))
	{
		ipc_shadow_out_t stOut;
		m_pShadow->Step(stIn, &stOut);
		m_cSlots.PublishShadow(stOut);
		while (m_pShadow->ServePlanner())		// the planner task of the firmware, here: back for the next period
			;
	}
}

void
CSimR5::PrintSummary() const
{
	printf("[SIM R5] %ld periods of %.3f ms run, %ld left out (the host let their release pass); longest %.1f us; "
	       "%ld ended after the next release; servo switched off for want of a new torque: %ld\n",
	       m_stCounts.nPeriods, (double)m_stSpec.tPeriod * 1e-6, m_stCounts.nLeftOut,
	       (double)m_stCounts.tCycleMax * 1e-3, m_stCounts.nOverruns, m_stCounts.nStaleStops);
	if (m_pControl != NULL)
		printf("[SIM R5] its control task had no new state of the arm in %ld of those periods\n", m_cControlR5.NoState());
}
