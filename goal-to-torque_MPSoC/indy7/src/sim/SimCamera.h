/*****************************************************************************
*	Name: SimCamera.h
*	Description: A camera for the simulation (-s, cfg SIM_CAM=1) in place of
*	the bridge of the camera application, so that every method (--method,
*	obs/ObsMethod.h) can be run and compared without the camera pipeline.
*
*	A thread that is not real-time. dHz times a second it captures: it takes
*	the newest state of the EtherCAT task, whose joint angles are those of
*	the simulated arm nStateAge periods before the release of that period
*	(sim/SimArm.h), and that instant is the capture time. It sees an object
*	that rests at adObject and the tool tag at those joint angles, with
*	noise. The result is ready dLatencyS after the capture:
*	  M1      the object in the base frame, with the error of the calibration
*	          adBias, through the checks and the average of the bridge
*	          (GoalFilter.h) into the goal slot of the application;
*	  others  into the mailbox (obs/ObsWriter.h), relative to the tag in the
*	          directions of the base (bTagFormat), or in the base frame with
*	          adBias, as the bridge sends it today.
*	The error of the calibration moves the object and the tag alike, so a
*	result relative to the tag does not have it.
*****************************************************************************/
#ifndef G2T_SIM_CAMERA_H
#define G2T_SIM_CAMERA_H

#include <atomic>
#include <thread>

#include "../base/Common.h"
#include "../base/AtomicSlot.h"
#include "../GoalFilter.h"
#include "../GoalSlot.h"
#include "../HighLevelRT.h"
#include "../TaskSlots.h"
#include "../obs/ObsWriter.h"

class CSimCamera
{
public:
	enum { PENDING = 64 };		// results on their way: latency x rate must stay below

	struct Config
	{
		double	dHz;				// captures per second
		double	dLatencyS;			// from the capture to the result [s]
		double	dNoiseM;			// standard deviation of the result, each axis [m]
		double	adObject[3];		// the object [m, base frame]
		bool	bTagFormat;			// the methods with the mailbox: the result relative to the tool tag
		double	adBias[3];			// error of the calibration on a result in the base frame [m]
		double	adTagOffset[3];		// the tool tag in the frame of the tool [m]
		int		nStateAge;			// age of the joint angles of a state at its release [periods]
		RTTIME	tPeriod;			// of the EtherCAT task [ns]
		CGoalFilter::Params stFilter;	// M1: the checks and the average of the bridge
	};

	~CSimCamera() { Stop(); }

	/* before the tasks. apObs NULL: M1, the goals go into apSlot. FALSE: a setting cannot be used, or the model
	 * of the arm cannot be read. */
	BOOL	Start	(const Config& astCfg, const char* astrURDF, const CHighLevelRT::Params& astHl,
	                 const CAtomicSlot<task_slots::ST_STATE_SLOT>* apState, CObsWriter* apObs, CGoalSlot* apSlot);
	/* after the tasks; prints what the camera did */
	void	Stop	(	);

private:
	struct Result { uint64_t tCapture; double adZ[3]; };

	void	Run		(	);
	void	Capture	(	);
	void	Deliver	(const Result& astR);

	Config			m_stCfg{};
	CHighLevelRT	m_cFk;			// a model of the arm of its own: the tool tag at joint angles
	const CAtomicSlot<task_slots::ST_STATE_SLOT>* m_pState{NULL};
	CObsWriter*		m_pObs{NULL};
	CGoalSlot*		m_pSlot{NULL};
	CGoalFilter		m_cFilter;
	std::thread		m_th;
	std::atomic<bool> m_bStop{false};
	Result			m_aPending[PENDING];
	unsigned		m_uHead{0}, m_uTail{0};		// [tail, head): on their way
	uint32_t		m_uLastPeriod{0};
	unsigned long long m_uRng{0x9E3779B97F4A7C15ULL};
	long			m_nCaptures{0}, m_nResults{0}, m_nNoState{0}, m_nDropped{0};
};

#endif // G2T_SIM_CAMERA_H
