/*****************************************************************************
*	Name: ObsLog.h
*	Description: The log of the camera results during a run of the camera
*	application with a method that uses the mailbox (ObsMethod.h), written by
*	CTestLog (TestLog.h): one CSV file per run with one row per result that
*	left the receiver (ObsGoal.h), and a .json file with the settings and the
*	counts. It carries the run number of the high-level log of the same run
*	(hl_app_..._rNN), and its times count from the same instant.
*
*	CSV columns: t [s] (the release of the period of the verdict); verdict
*	(accepted, superseded, expired, state_unrecoverable, seq_fail,
*	error_exceeded, evicted; obs_receiver.h); n, epoch (the number of the
*	result and the start of its writer); target (+1 with every object the
*	operator picks); cls; mode (0 = 416, 1 = 320); flags (1 relative to the
*	tag, 2 no tag in the image, 4 in the base frame); t_e (capture), f
*	(publication), t_valid (deadline), t_in (the release of the period that
*	took it) [s]; sigma, ebar (eq. (7) in the last period that checked it)
*	[m]; s (the period of the state used); z_x..z (the result) [m]. For a
*	result used for the first time: obj_x..z (its object in the base frame)
*	[m], ws (the workspace check: accepted, below the goal floor, outside
*	the workspace box, not finite) and folded (0, 1 the estimate took it,
*	2 and started again from it).
*****************************************************************************/
#ifndef G2T_OBS_LOG_H
#define G2T_OBS_LOG_H

#include "TestLog.h"
#include "obs/ObsGoal.h"
#include "obs/ObsWriter.h"

struct ObsFormat
{
	typedef CObsGoal::Entry Sample;

	struct Meta {
		char			acMethod[8];		// obs_method::Name
		int				nRun;				// the run of the high-level log
		BOOL			bSim;
		uint64_t		tStart;				// the release of the first period of TRACK [ticks]: t = 0
		obs_settings_t	stSet;
		double			dTolM;
		int				nStateAge;
		double			adTagOffset[3];
	};
	struct Result {							// during the run
		CObsGoal::Counts stCounts;			// the control task's side
		long			anWriter[CObsWriter::eResults];		// the camera side
	};

	void	Name	(const Meta& astM, const char* astrStamp, char* acOut, size_t auLen);
	void	Header	(FILE* fp, const Meta& astM);
	void	Begin	(	) {}
	void	Row		(FILE* fp, const Meta& astM, const Sample& astS);
	void	Finish	(FILE* apJson, const char* astrCsv, const Meta& astM, const Result& astR,
	                 const char* astrState, long anRows, unsigned anDrops);
};

/* at most 16 rows per period, about one per camera result */
typedef CTestLog<ObsFormat, 1024> CObsLog;

#endif // G2T_OBS_LOG_H
