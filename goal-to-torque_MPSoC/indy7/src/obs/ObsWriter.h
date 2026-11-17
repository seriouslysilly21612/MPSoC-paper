/*****************************************************************************
*	Name: ObsWriter.h
*	Description: The camera side of the mailbox in the control application
*	(the methods other than M1, ObsMethod.h): one result of the camera
*	becomes a descriptor in the mailbox (obs_mailbox.h, eq. (5)). Used by the
*	bridge (ROS2PickBridge.cpp) or by the camera of the simulation
*	(sim/SimCamera.h): one thread calls Put().
*
*	  time      the capture on the clock of the control tasks
*	            (CLOCK_MONOTONIC [ns]); MonoOfReal() brings a stamp of the
*	            system clock there. A result older than the cap, or from the
*	            future, is not written (D-21).
*	  deadline  the capture plus the cap ([CONTRACT] AGE_CAP_MS).
*	  sigma     the measurement error of the mode ([CONTRACT] EPS_M_MM).
*	  checks    valid and finite here. The workspace is checked by the
*	            control task (ObsGoal.h): with a result relative to the tag,
*	            the object is known in the base frame only there.
*
*	No lock, no allocation; Put() reads the clock (vDSO).
*****************************************************************************/
#ifndef G2T_OBS_WRITER_H
#define G2T_OBS_WRITER_H

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <ctime>

#include "obs_mailbox.h"

class CObsWriter
{
public:
	struct Config
	{
		double	dAgeCapS;		// longest age of a result [s]
		float	afEps[2];		// measurement error of the modes 0 (416) and 1 (320) [m]
	};
	enum eResult { eWritten = 0, eInvalid, eNotFinite, eAge, eFull, eExpired, eResults };

	/* before the first Put(), and before the control task reads the mailbox */
	void Init(volatile obs_mailbox_t* apBox, const Config& astCfg)
	{
		m_pBox = apBox;
		m_stCfg = astCfg;
		m_tCap = (int64_t)(astCfg.dAgeCapS * 1e9);
		for (int r = 0; r < eResults; r++)
			m_anCount[r].store(0, std::memory_order_relaxed);
		obs_writer_start(apBox);
	}

	/* one result. adZ [m]: the object in the base frame (auFormat OBS_Z_BASE)
	 * or from the tool tag, in the directions of the base (OBS_Z_TAG, with
	 * OBS_TAG_LOST if the image shows no tag); atCapture [ns, CLOCK_MONOTONIC] */
	eResult Put(bool abValid, const double* adZ, unsigned auFormat, int64_t atCapture, int anMode, int anClass)
	{
		const eResult eR = PutOne(abValid, adZ, auFormat, atCapture, anMode, anClass);
		m_anCount[eR].fetch_add(1, std::memory_order_relaxed);
		return eR;
	}

	/* the operator picked another object: the estimate of the control task starts again (any thread) */
	void NextTarget() { m_uTarget.fetch_add(1, std::memory_order_relaxed); }

	long Count(eResult aeR) const { return m_anCount[aeR].load(std::memory_order_relaxed); }
	static const char* Name(eResult aeR)
	{
		static const char* const s_apc[eResults] = { "written", "invalid", "not finite", "older than the cap or from the future",
		                                             "no free slot", "past its deadline" };
		return (aeR >= 0 && aeR < eResults) ? s_apc[aeR] : "?";
	}

	static uint64_t NowNs()
	{
		struct timespec stTs;
		clock_gettime(CLOCK_MONOTONIC, &stTs);
		return (uint64_t)stTs.tv_sec * 1000000000ULL + (uint64_t)stTs.tv_nsec;
	}
	/* a stamp of the system clock (CLOCK_REALTIME [ns]) on CLOCK_MONOTONIC: the two clocks are read one after the
	 * other (D-21). The time synchronization of the host is off during an experiment. */
	static int64_t MonoOfReal(int64_t atReal)
	{
		struct timespec stReal;
		clock_gettime(CLOCK_REALTIME, &stReal);
		const int64_t tMono = (int64_t)NowNs();
		return tMono - (((int64_t)stReal.tv_sec * 1000000000LL + (int64_t)stReal.tv_nsec) - atReal);
	}
	/* the other way: a time of CLOCK_MONOTONIC [ns] as a stamp of the system clock */
	static int64_t RealOfMono(int64_t atMono)
	{
		return atMono - MonoOfReal(0);
	}

private:
	eResult PutOne(bool abValid, const double* adZ, unsigned auFormat, int64_t atCapture, int anMode, int anClass)
	{
		if (!abValid)
			return eInvalid;
		if (!std::isfinite(adZ[0]) || !std::isfinite(adZ[1]) || !std::isfinite(adZ[2]))
			return eNotFinite;
		const int64_t tAge = (int64_t)NowNs() - atCapture;
		if (tAge < 0 || tAge > m_tCap)
			return eAge;
		obs_desc_t stD;
		memset(&stD, 0, sizeof(stD));
		stD.t_e = (uint64_t)atCapture;
		stD.t_valid_until = stD.t_e + (uint64_t)m_tCap;
		for (int k = 0; k < 3; k++)
			stD.z[k] = (float)adZ[k];
		stD.mode = (uint8_t)(anMode == 1 ? 1 : 0);
		stD.sigma = m_stCfg.afEps[stD.mode];
		stD.flags = (uint8_t)auFormat;
		stD.cls = (uint16_t)anClass;
		stD.target_seq = m_uTarget.load(std::memory_order_relaxed);
		const int nPut = obs_put(m_pBox, &stD, &NowNs);
		return nPut == OBS_OK ? eWritten : (nPut == OBS_FULL ? eFull : eExpired);
	}

	volatile obs_mailbox_t*	m_pBox{NULL};
	Config					m_stCfg{};
	int64_t					m_tCap{0};
	std::atomic<uint32_t>	m_uTarget{0};
	std::atomic<long>		m_anCount[eResults]{};
};

#endif // G2T_OBS_WRITER_H
