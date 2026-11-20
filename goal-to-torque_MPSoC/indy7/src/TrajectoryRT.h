/*****************************************************************************
*	Name: TrajectoryRT.h
*	Description: The two motions of the high-level controller, TRACK and
*	HOME (paper, Sec. IV-C), as the Trajectory task runs them: in every
*	period, the latest goal of the goal slot and the measured state of the
*	State slot go into CHighLevelRT (goal admission, the reference of the
*	plan in use, lead bound), and the joint reference (q_ref, qd_ref,
*	qdd_ref) comes out, for the Reference slot that the Dynamics task reads.
*	The IK of a goal and the plans are calculated by the planner of the
*	controller, which the planner task runs (ServePlanner(), HighLevelLink.h).
*	The Dynamics task runs computed torque control on that reference; this
*	class never computes a torque.
*
*	TRACK and HOME are started and ended here, by the Trajectory task alone;
*	the Dynamics task follows the mode of the Reference slot (eRefTrack,
*	eRefHome, eRefNone).
*
*	Started with the arm at rest and every joint inside its range (the caller
*	checks that the controller is enabled and in gravity compensation, with
*	the servo on, from the Torque slot and the State slot). Only goals
*	written after the start count. HOME ends when the arm is at rest at q_r
*	(or at the target of the operator); TRACK runs until a stop. A generator
*	error ends either motion. Everything is allocated in Init(); Update()
*	does not allocate.
*****************************************************************************/
#ifndef G2T_TRAJECTORY_RT_H
#define G2T_TRAJECTORY_RT_H

#include <cstddef>

#include "base/Common.h"
#include "HighLevelRT.h"
#include "HighLevelLink.h"
#include "GoalSlot.h"

class CTrajectoryRT
{
public:
	enum { MAX_DOF = CHighLevelRT::DOF };

	/* the state of the motion (the end states of TestLog.h) */
	enum eState { eIdle = 0, eRunning = 1, eDoneAtRest = 2, eAborted = 4, eError = 6 };
	/* the motion (the values of task_slots::eRefMode) */
	enum eMotion { eMotionNone = 0, eMotionTrack = 1, eMotionHome = 2 };

	/* at rest: |qd| <= REST_QD for REST_CYC cycles (paper, Table 9) */
	static constexpr double REST_QD  = 0.05;	// [rad/s]
	static constexpr int    REST_CYC = 200;		// 200 ms at 1 kHz

	/* one cycle of TRACK or HOME, for the log (HighLevelLog.h). The torque
	 * (adTau, uClamp) is filled by the Trajectory task from the TorqueSlot:
	 * the command of the Dynamics task of the previous period, which acted
	 * on the previous reference. */
	struct Sample {
		double       dT;                        // time since the start [s]
		int          nMode;                     // 0 TRACK, 1 HOME
		unsigned int uFlags;                    // HL_* bits below
		int          nOtg;                      // ruckig::Result
		unsigned int uLeadReset;                // bit i: lead bound reset joint i
		int          nIkIter;                   // IK iterations for the current goal
		double       dIkErr;                    // |e| of the IK [m]
		double       dOtgDuration;              // duration of the current plan [s]
		double       dHlUs;                     // time of the high-level step [us]
		long         nGoalAge;                  // cycles since the goal slot changed
		double       adGoal[3];                 // goal slot (NaN: none)
		double       adAccepted[3];             // accepted goal
		double       adTool[3];                 // tool position p(q) of the measurement
		double       adTarget[MAX_DOF];         // target of the generator (IK solution)
		double       adQRef[MAX_DOF], adQdRef[MAX_DOF], adQddRef[MAX_DOF];
		double       adQ[MAX_DOF], adQd[MAX_DOF];
		double       adTau[MAX_DOF];            // commanded, after the limit (see above)
		unsigned int uClamp;
	};
	enum { HL_GOAL_ACCEPTED = 1, HL_IK_CONVERGED = 2, HL_IK_REJECTED = 4, HL_IK_ACTIVE = 8,
	       HL_OTG_NEW = 16, HL_AT_REST = 32, HL_HAS_GOAL = 64 };

	CTrajectoryRT();

	/* load the model of the IK from the URDF; Ts: control period [s] */
	BOOL	Init			(const char* astrURDF, double adTs, const CHighLevelRT::Params& astParams);
	BOOL	IsInit			(	) const { return m_hl.IsInit(); }
	CHighLevelRT&		HighLevel()		  { return m_hl; }
	const CHighLevelRT&	HighLevel() const { return m_hl; }

	/* the goal slot that TRACK reads, and the offset added to every position
	 * read from it [m]: the hover height of the camera application (paper,
	 * Sec. V-C), zero for the tests. Set before Start(). */
	void	SetGoalSlot		(const CGoalSlot* apSlot) { m_pGoalSlot = apSlot; }
	void	SetGoalOffset	(const double* adOff) { for (int k = 0; k < 3; k++) m_adGoalOffset[k] = adOff[k]; }

	/* why a start is refused, as numbers: a caller that cannot print (the control task on the R5) passes them
	 * on, and RefusalText() makes the line of them elsewhere */
	enum eRefusal { eStartOk = 0, eNotInit = 1, eMotionRuns = 2, eArmMoves = 3, eJointOutside = 4, eTargetOutside = 5 };
	struct Refusal {
		int    eWhy;                            // eRefusal
		int    nJoint;                          // eJointOutside, eTargetOutside: the joint (0-based); eMotionRuns: eMotion
		double dValue, dLo, dHi;                // eArmMoves: |qd|; the two with a joint: its angle and its range
	};
	static void	RefusalText	(const Refusal& astWhy, char* astrWhy, size_t auWhyLen);

	/* start TRACK (abHome false) or HOME from the measured state, HOME to
	 * q_r or to adHomeTarget; FALSE with the reason: not initialized, a
	 * motion is running, the arm is moving, a joint is outside its range, or
	 * the target of HOME is outside the joint limits minus the margin */
	BOOL	Start			(bool abHome, const double* adQ, const double* adQd, BOOL abFrictionFF,
	                         char* astrWhy, size_t auWhyLen, const double* adHomeTarget = NULL);
	BOOL	Start			(bool abHome, const double* adQ, const double* adQd, BOOL abFrictionFF,
	                         Refusal* apWhy, const double* adHomeTarget = NULL);
	/* end the motion (a stop, or the end of a test); nothing if none runs */
	void	End				(eState aeState);

	/* the last successful Start(): its number (1 the first), HOME or TRACK, and the target of HOME (of TRACK:
	 * the posture it started from). For a second controller that is to follow this one (shadow/ShadowWriter.h) */
	struct StartCall {
		unsigned int uCount;
		bool         bHome;
		double       adTarget[MAX_DOF];
	};
	const StartCall& LastStart	(	) const { return m_start; }

	/* one period: the measured state -> the reference of this period. TRUE
	 * if the reference is valid (the generator returned no error). After it,
	 * Pending() tells whether the motion ends with this period (HOME at
	 * rest, or a generator error): the caller then calls End(Pending()). */
	BOOL	Update			(const double* adQ, const double* adQd,
	                         double* adQRef, double* adQdRef, double* adQddRef);
	eState	Pending			(	) const { return m_ePending; }

	eState	GetState		(	) const { return m_eState; }
	BOOL	IsRunning		(	) const { return m_eState == eRunning; }
	eMotion	GetMotion		(	) const { return m_eMotion; }
	BOOL	GetFrictionFF	(	) const { return m_bFrictionFF; }
	/* TRUE if the last Update() was a cycle of a motion (its sample is valid) */
	BOOL	HasSample		(	) const { return m_bSample; }
	Sample&	GetSample		(	) { return m_sample; }
	const Sample& GetSample	(	) const { return m_sample; }

	/* the planner of the high-level controller (HighLevelLink.h), for the one task that runs it, the planner
	 * task; a tool without one calls it after Update(). TRUE: it did something, call again. */
	BOOL	ServePlanner	(	) { return m_link.Serve(m_planner) ? TRUE : FALSE; }
	/* for the summary, from any task: what the planner has done (false: nothing yet), and from the task of
	 * Update(): the plans that were not back in their period, over all motions */
	bool	GetPlannerStats	(CHlLink::Stats& astStats) const { return m_link.GetStats(astStats); }
	long	PlansLate		(	) const { return m_nPlansLate; }

private:
	CHighLevelRT		m_hl;
	CHlPlanner			m_planner;		// used by ServePlanner() only
	CHlLink				m_link;			// between m_hl and m_planner
	const CGoalSlot*	m_pGoalSlot;
	double				m_adGoalOffset[3];
	eState				m_eState;
	eState				m_ePending;		// the end requested by the last Update()
	eMotion				m_eMotion;
	BOOL				m_bFrictionFF;
	unsigned int		m_uGoalCountStart, m_uGoalCountSeen;
	long				m_nGoalAge;
	long				m_nCycle;
	double				m_dTs;
	Sample				m_sample;
	BOOL				m_bSample;
	StartCall			m_start;
	long				m_nPlansLate, m_nLateSeen;		// plans not back in their period: over all motions, of this one
};

#endif // G2T_TRAJECTORY_RT_H
