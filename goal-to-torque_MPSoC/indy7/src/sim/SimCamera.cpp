/*****************************************************************************
*	Name: SimCamera.cpp
*	Description: The camera of the simulation. See SimCamera.h.
*****************************************************************************/
#include "SimCamera.h"

#include <chrono>
#include <cmath>
#include <pthread.h>

namespace {

/* xorshift64* and Box-Muller, as the goal noise of the high-level tests: the same sequence in every run */
double Uniform(unsigned long long& auS)
{
	auS ^= auS >> 12;
	auS ^= auS << 25;
	auS ^= auS >> 27;
	return (double)((auS * 2685821657736338717ULL) >> 11) * (1.0 / 9007199254740992.0);
}
double Normal(unsigned long long& auS)
{
	double dU1 = Uniform(auS);
	if (dU1 < 1e-300)
		dU1 = 1e-300;
	const double dU2 = Uniform(auS);
	return std::sqrt(-2.0 * std::log(dU1)) * std::cos(2.0 * M_PI * dU2);
}

} // namespace

BOOL
CSimCamera::Start(const Config& astCfg, const char* astrURDF, const CHighLevelRT::Params& astHl,
                  const CAtomicSlot<task_slots::ST_STATE_SLOT>* apState, CObsWriter* apObs, CGoalSlot* apSlot)
{
	const Config& c = astCfg;
	bool bOk = apState != NULL && apSlot != NULL && std::isfinite(c.dHz) && c.dHz > 0.0 && c.dHz <= 1000.0 &&
	           std::isfinite(c.dLatencyS) && c.dLatencyS >= 0.0 && c.dLatencyS * c.dHz <= (double)(PENDING - 2) &&
	           std::isfinite(c.dNoiseM) && c.dNoiseM >= 0.0 && c.nStateAge >= 0 && c.tPeriod > 0;
	for (int k = 0; k < 3; k++)
		bOk = bOk && std::isfinite(c.adObject[k]) && std::isfinite(c.adBias[k]) && std::isfinite(c.adTagOffset[k]);
	if (!bOk)
	{
		DBG_LOG_ERROR("(CSimCamera) SIM_CAM_HZ above 0 up to 1000; SIM_CAM_LATENCY_MS not negative and below %d "
		              "captures; SIM_CAM_NOISE_MM not negative; SIM_CAM_OBJECT_M and SIM_CAM_BIAS_MM three numbers",
		              (int)PENDING - 2);
		return FALSE;
	}
	if (FALSE == m_cFk.Init(astrURDF, (double)c.tPeriod * 1e-9, astHl))
		return FALSE;
	m_stCfg = c;
	m_pState = apState;
	m_pObs = apObs;
	m_pSlot = apSlot;
	m_cFilter.SetParams(c.stFilter);
	m_bStop.store(false);
	m_th = std::thread(&CSimCamera::Run, this);
	const bool bTag = (apObs != NULL && c.bTagFormat);
	printf("[SIMCAM] a camera in place of the bridge: an object at (%.3f, %.3f, %.3f) m, %.0f results a second, each "
	       "%.0f ms after its capture, noise %.1f mm; %s\n", c.adObject[0], c.adObject[1], c.adObject[2], c.dHz,
	       c.dLatencyS * 1e3, c.dNoiseM * 1e3,
	       apObs == NULL ? "in the base frame, through the checks and the average of the bridge into the goal slot" :
	       bTag ? "relative to the tool tag, into the mailbox" : "in the base frame, into the mailbox");
	if (!bTag && (c.adBias[0] != 0.0 || c.adBias[1] != 0.0 || c.adBias[2] != 0.0))
		printf("[SIMCAM] error of the calibration (%.1f, %.1f, %.1f) mm on every result\n", c.adBias[0] * 1e3,
		       c.adBias[1] * 1e3, c.adBias[2] * 1e3);
	return TRUE;
}

void
CSimCamera::Stop()
{
	if (!m_th.joinable())
		return;
	m_bStop.store(true);
	m_th.join();
	printf("[SIMCAM] %ld captures, %ld results", m_nCaptures, m_nResults);
	if (m_pObs != NULL)
	{
		long nNot = 0;
		for (int w = 1; w < (int)CObsWriter::eResults; w++)
			nNot += m_pObs->Count((CObsWriter::eResult)w);
		printf(": %ld written into the mailbox, %ld not written", m_pObs->Count(CObsWriter::eWritten), nNot);
	}
	else
		printf(": %ld goals into the slot", m_cFilter.Count(CGoalFilter::eAccepted));
	printf("; %ld captures without a new state, %ld dropped (too many on their way)\n", m_nNoState, m_nDropped);
}

void
CSimCamera::Run()
{
	pthread_setname_np(pthread_self(), "sim-camera");
	const uint64_t tStep = (uint64_t)(1e9 / m_stCfg.dHz);
	const uint64_t tLatency = (uint64_t)(m_stCfg.dLatencyS * 1e9);
	uint64_t tNext = CObsWriter::NowNs() + tStep;
	while (!m_bStop.load())
	{
		const uint64_t tNow = CObsWriter::NowNs();
		if (tNow >= tNext)
		{
			Capture();
			tNext += tStep;
			if (tNext <= tNow)			// the thread was held up: no burst of captures
				tNext = tNow + tStep;
		}
		while (m_uTail != m_uHead && m_aPending[m_uTail % PENDING].tCapture + tLatency <= tNow)
			Deliver(m_aPending[m_uTail++ % PENDING]);
		std::this_thread::sleep_for(std::chrono::microseconds(500));
	}
}

// one image: the object and the tool tag as they are at the capture
void
CSimCamera::Capture()
{
	task_slots::ST_STATE_SLOT stState;
	if (!m_pState->Read(stState) || stState.uPeriod == m_uLastPeriod)		// the tasks are not running
	{
		m_nNoState++;
		return;
	}
	m_uLastPeriod = stState.uPeriod;
	if (m_uHead - m_uTail >= (unsigned)PENDING)
	{
		m_nDropped++;
		return;
	}
	const bool bTag = (m_pObs != NULL && m_stCfg.bTagFormat);
	double adTag[3] = { 0.0, 0.0, 0.0 };
	if (bTag)
		m_cFk.ToolPoint(stState.adQ, m_stCfg.adTagOffset, adTag);
	Result stR;
	stR.tCapture = (uint64_t)stState.tRelease - (uint64_t)m_stCfg.nStateAge * (uint64_t)m_stCfg.tPeriod;
	for (int k = 0; k < 3; k++)
		stR.adZ[k] = m_stCfg.adObject[k] - adTag[k] + (bTag ? 0.0 : m_stCfg.adBias[k]) + m_stCfg.dNoiseM * Normal(m_uRng);
	m_aPending[m_uHead++ % PENDING] = stR;
	m_nCaptures++;
}

// a result is ready
void
CSimCamera::Deliver(const Result& astR)
{
	m_nResults++;
	if (m_pObs != NULL)
	{
		m_pObs->Put(true, astR.adZ, m_stCfg.bTagFormat ? OBS_Z_TAG : OBS_Z_BASE, (int64_t)astR.tCapture, 0, 0);
		return;
	}
	/* M1: as the bridge (ROS2PickBridge.cpp, OnTarget) */
	double adOut[3];
	if (m_cFilter.Push(true, astR.adZ, (double)CObsWriter::NowNs() * 1e-9, adOut) == CGoalFilter::eAccepted)
		m_pSlot->Write(adOut);
}
