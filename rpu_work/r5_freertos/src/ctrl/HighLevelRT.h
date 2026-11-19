/*****************************************************************************
*	Name: HighLevelRT.h
*	Description: The high-level controller (paper, Sec. IV-C, eq. (9) and
*	Table 9). It turns the latest goal, a position of the tool, into a joint
*	reference that meets the reference contract. It has two parts, because on
*	the R5 one calculation of the generator takes up to 1.1 ms and one
*	iteration of the IK 70 us (measured 2026-11-19; decision D-14): neither
*	fits a cycle of 1 ms.
*
*	CHighLevelRTN runs in the 1 ms cycle:
*	  1. goal admission: a goal replaces the accepted one only if it differs
*	     from it by more than delta, and then asks for its IK;
*	  3. the reference: the plan in use, evaluated at the time of the period.
*	     A new target, or a joint at its lead bound, asks for a new plan that
*	     starts from the state the reference has H periods later (N_ahead),
*	     and the new plan is used from that period on: a new target changes
*	     the plan, not the state;
*	  4. lead bound: a joint whose reference leads the measurement by ell gets
*	     its measured state (q_i, qd_i, 0) as the start of that next plan.
*	CHlPlannerN runs beside it, in a task of lower priority:
*	  2. inverse kinematics: damped least-squares steps with the 3x6 position
*	     Jacobian, from the ready posture q_r for every accepted goal, at most
*	     N_max per goal, every iterate clipped to the joint limits minus a
*	     margin, on a model instance of its own;
*	  3. trajectory generation: Ruckig, position interface, from the state of
*	     the request to its target, within the limits on velocity,
*	     acceleration, and jerk.
*	The two exchange requests and results (IkRequest .. PlanResult). Who
*	carries them is not decided here: CHlInline below does it in the calling
*	task, one period later, for the checks and until the planner has a task.
*
*	Every motion starts its reference at the measured position, at rest, and
*	holds it there: TRACK until the plan of its first goal is in use, HOME
*	until its plan to q_r (or to a target of the operator, StartHomeTo) is,
*	H periods after the start. The arm is at rest when the plan in use has
*	reached its target, nothing is asked of the planner, and |qd| has stayed
*	<= 0.05 rad/s for 200 ms.
*	Everything is allocated in Init(); Update() does not allocate (checked by
*	the malloc-count build) and takes a bounded time: it evaluates one plan.
*	A template on the number of joints N of the IK model (the arm: N = 6,
*	CHighLevelRT; a hand of the humanoid: N = 7, paper, Sec. VI), with the
*	tool body as a parameter (Params::acTool, "tcp" for the arm).
*****************************************************************************/
#ifndef G2T_HIGH_LEVEL_RT_H
#define G2T_HIGH_LEVEL_RT_H

#include <rbdl/rbdl.h>
#include <ruckig/ruckig.hpp>
#include <Eigen/Dense>

#include "base/Common.h"

/* Step 2 on a model of its own: the error of the tool at the iterate, and one damped least-squares step. The
 * planner iterates with it; so does CHighLevelRTN::SolveIk(), outside a cycle. */
template <int N>
struct CHlIkN
{
	BOOL	Init		(const char* astrURDF, const char* astrTool);
	/* the four calls of one iteration at the iterate vQk, apart so that the planner can time each */
	void	Kinematics	(	) { RigidBodyDynamics::UpdateKinematics(model, vQk, vZero, vZero); }
	/* |p_goal - p(q_k)| [m]; *apvE: the difference */
	double	Error		(const double* adGoal, RigidBodyDynamics::Math::Vector3d* apvE);
	void	Jacobian	(	);
	/* q_k <- q_k + dq: the damped least-squares step, bounded by adStepMax, every joint kept inside its limits
	 * minus adMargin */
	void	Step		(const RigidBodyDynamics::Math::Vector3d& avE, double adLambda, double adStepMax,
	                     const double* adLimLo, const double* adLimHi, double adMargin);

	RigidBodyDynamics::Model			model;
	unsigned int						uTcp;
	RigidBodyDynamics::Math::Vector3d	vTcpPoint;
	RigidBodyDynamics::Math::VectorNd	vQk, vZero;
	RigidBodyDynamics::Math::MatrixNd	mG;			// 3 x n, filled by RBDL
	Eigen::Matrix<double, 3, N>			mJp;
	Eigen::LDLT<Eigen::Matrix3d>		ldlt;
};

template <int N>
class CHighLevelRTN
{
public:
	enum { DOF = N };

	struct Params {
		double	dDelta;					// goal threshold [m]
		double	dLambda;				// DLS damping [m]
		double	dStepMax;				// step bound Delta_max [rad]
		double	dTol;					// IK tolerance e_tol [m]
		int		nIterMax;				// N_max, iterations per goal
		int		nPlanAhead;				// N_ahead (H): a plan asked for in period k is used from period k + H on
		double	dVelMax, dAccMax, dJerkMax;		// joint limits of the generator
		double	dMargin;				// position-limit margin [rad]
		double	dLead;					// lead bound ell [rad]
		double	dRestQd;				// at rest: |qd| [rad/s] ...
		int		nRestCyc;				// ... for this many cycles
		double	adQr[DOF];				// ready posture [rad]
		double	adLimLo[DOF], adLimHi[DOF];		// joint limits [rad]
		bool	bStepRef;				// baseline, simulation only: the IK solution as a step,
										// without Step 3 (generator) and Step 4 (lead bound)
		char	acTool[48];				// body of the tool point in the URDF ("tcp")
	};
	static Params DefaultParams();

	/* What the two parts exchange. A request is new when its uSeq changed; a result answers the request with its
	 * uSeq, and one that answers an older request is not taken. Plain data: they pass through a slot as they are. */
	struct IkRequest {					// Step 1 -> Step 2
		uint32_t	uSeq;
		double		adGoal[3];			// the accepted goal [m]
		double		adQr[DOF];			// where its IK starts: the ready posture at the acceptance
	};
	struct IkResult {
		uint32_t	uSeq;
		uint32_t	bConverged;			// 0: gave up after N_max
		int32_t		nIter;
		double		dErr;				// |e| at the last iterate [m]
		double		adQ[DOF];			// the solution, when it converged
	};
	struct PlanRequest {				// Step 3, the calculation
		uint32_t	uSeq;
		uint32_t	uAdopt;				// the period from which the plan is used: its time 0
		double		adQ[DOF], adQd[DOF], adQdd[DOF];	// the state of the reference in that period
		double		adTarget[DOF];
	};
	struct PlanResult {
		uint32_t	uSeq;
		int32_t		nResult;			// ruckig::Result of the calculation: below 0 an error
		double		dCalcUs;			// its time [us]
		ruckig::Trajectory<DOF>	cPlan;
	};

	/* what the last Update() did, and the state */
	struct Status {
		bool	bGoalAccepted;			// a goal replaced the accepted one (this cycle)
		bool	bIkConverged;			// the IK of the accepted goal converged: new target (this cycle)
		bool	bIkRejected;			// the IK gave up after N_max (this cycle)
		bool	bIkActive;				// the IK of the accepted goal is asked for and not back yet
		int		nIkIter;				// iterations of the last IK that came back
		double	dIkErr;					// |e| at its last iterate [m]
		int		nOtg;					// ruckig::Result: Working while the plan in use runs, Finished at its target
		bool	bOtgNewCalc;			// a new plan is in use from this cycle on
		double	dOtgDuration;			// duration of the plan in use [s]
		double	dOtgCalcUs;				// time of its calculation [us]
		bool	bPlanAsked;				// a plan is asked for and not in use yet
		unsigned uLeadReset;			// bit i: joint i is at its lead bound, or was, and the plan that restarts its
										// reference at the measurement is not in use yet: the reference of the
										// joint jumps in the cycle after the last of these
		bool	bAtRest;
		bool	bHasGoal;				// a goal has been accepted
		double	adGoal[3];				// accepted goal [m]
		double	adTarget[DOF];			// target of the newest plan asked for or in use [rad]
		long	nAccepted, nConverged, nRejected, nLeadResets;	// since Start()
		long	nPlanLate;				// plans that were not back in their period (asked for again), since Start()
		double	dUs;					// duration of Update() [us]
	};

	CHighLevelRTN();

	/* load the model of the tool position from the URDF; Ts: control period [s] */
	BOOL	Init			(const char* astrURDF, double adTs, const Params& astParams);
	BOOL	IsInit			(	) const { return m_bInit; }
	void	SetDelta		(double adDelta) { m_p.dDelta = adDelta; }
	void	SetReady		(const double* adQr);
	const Params& GetParams	(	) const { return m_p; }

	/* every motion starts its reference at the measured position, at rest: TRACK holds it until the plan of a
	 * goal is in use; HOME moves to q_r from period H on. adQd is not used (the start was (q, qd, 0) while the
	 * generator ran in the cycle). */
	void	StartTrack		(const double* adQ, const double* adQd);
	void	StartHome		(const double* adQ, const double* adQd);
	/* HOME to adTarget instead of q_r (the joint move of the operator) */
	void	StartHomeTo		(const double* adQ, const double* adQd, const double* adTarget);

	/* one cycle: adGoal = latest goal [m] or NULL, measured state -> reference.
	 * FALSE if a plan came back with an error of the generator (the reference is
	 * then the last one, at rest is false). */
	BOOL	Update			(const double* adGoal, const double* adQ, const double* adQd,
	                         double* adQRef, double* adQdRef, double* adQddRef);
	const Status& GetStatus	(	) const { return m_st; }

	/* the requests as the last Update() left them (uSeq 0: none yet), and the results of the planner: to be
	 * given before the Update() of the period in which they are to count */
	const IkRequest&	GetIkRequest	(	) const { return m_stIkReq; }
	const PlanRequest&	GetPlanRequest	(	) const { return m_stPlanReq; }
	void	TakeIk			(const IkResult& astResult);
	void	TakePlan		(const PlanResult& astResult);

	/* position of the tool [m] at adQ, on the model of this part (RT-safe) */
	void	ToolPosition	(const double* adQ, double* adP);
	/* the same for a point adLocal [m] of the frame of the tool (the tool tag
	 * of the camera application, obs/ObsGoal.h) */
	void	ToolPoint		(const double* adQ, const double* adLocal, double* adP);

	/* Step 2 alone, as the planner runs it, from q_r until it converges or
	 * gives up after N_max iterations. Not for the control task: the goals
	 * of high-level tests 1 and 2 are drawn with it before the tasks start,
	 * and the keyboard task checks the reach of a task move ('t') with it,
	 * each on an instance of its own. TRUE: converged; adQ (may be NULL): the
	 * solution, *pnIter: iterations, *pdErr: |e| at the last iterate [m] */
	BOOL	SolveIk			(const double* adGoal, double* adQ, int* pnIter, double* pdErr);

private:
	void	StartFrom		(const double* adQ);
	/* the reference of period auK on the plan in use (or the hold); TRUE: the plan has reached its target */
	bool	Reference		(uint32_t auK, double* adQ, double* adQd, double* adQdd) const;

	bool	m_bInit;
	bool	m_bHome;
	Params	m_p;
	double	m_dTs;
	CHlIkN<N>	m_cIk;					// the model of ToolPoint() and SolveIk()
	RigidBodyDynamics::Math::VectorNd	m_vQ;

	uint32_t	m_uK;					// the period of the next Update(); it wraps, only differences are used
	ruckig::Trajectory<DOF>	m_acPlan[2];	// [m_nPlan]: in use; the other: the plan that came back
	int			m_nPlan;
	bool		m_bPlan;				// false: no plan yet or the plan has ended, the reference holds m_adHold
	uint32_t	m_uPlanStart;			// the period of time 0 of the plan in use
	double		m_adHold[DOF];
	double		m_adRef[DOF], m_adRefQd[DOF], m_adRefQdd[DOF];	// the last reference

	IkRequest	m_stIkReq;
	IkResult	m_stIkRes;
	bool		m_bIkBack;				// m_stIkRes answers m_stIkReq and Update() has not used it yet
	PlanRequest	m_stPlanReq;
	bool		m_bPlanBack;			// the plan of m_stPlanReq is in m_acPlan[1 - m_nPlan]
	int			m_nPlanResult;
	double		m_dPlanCalcUs;
	bool		m_bWantPlan;			// the target changed since the last request
	unsigned	m_uLeadWant;			// joints at their lead bound, for the next request ...
	unsigned	m_uLeadAsked;			// ... and those of the request that is out
	double		m_adLeadQ[DOF], m_adLeadQd[DOF];		// their measured state, at the bound
	int		m_nRest;
	Status	m_st;
};

/* Steps 2 and 3 as far as they are calculation: what CHighLevelRTN asks for. One instance, one caller at a time. */
template <int N>
class CHlPlannerN
{
public:
	typedef CHighLevelRTN<N> HL;

	CHlPlannerN();
	BOOL	Init			(const char* astrURDF, double adTs, const typename HL::Params& astParams);

	/* Step 2: IkBegin() takes a request (and drops the one before it); IkStep() then does one iteration at
	 * most and returns TRUE with the result when the IK has converged or given up. */
	void	IkBegin			(const typename HL::IkRequest& astRequest);
	bool	IkActive		(	) const { return m_bIkActive; }
	BOOL	IkStep			(typename HL::IkResult* apResult);

	/* Step 3: the plan of a request. The result is written in place: it is large. */
	void	Plan			(const typename HL::PlanRequest& astRequest, typename HL::PlanResult* apResult);

private:
	bool	m_bInit;
	typename HL::Params	m_p;
	CHlIkN<N>	m_cIk;
	typename HL::IkRequest	m_stReq;
	bool	m_bIkActive;
	int		m_nIkIter;
	ruckig::Ruckig<N>			m_otg;
	ruckig::InputParameter<N>	m_in;
};

/* The planner in the task of its caller, until it has a task of its own: after every Update() of the high-level
 * controller, After() gives the planner its new requests, lets it work, and hands the results back for the next
 * Update(). A plan is back one period after it was asked for (H - 1 periods before it is needed); the IK does
 * IK_STEPS iterations per period, as many as the cycle did while the IK ran in it. */
template <int N>
class CHlInlineN
{
public:
	typedef CHighLevelRTN<N> HL;

	BOOL	Init			(const char* astrURDF, double adTs, const typename HL::Params& astParams)
	{
		m_uIkSeq = 0;
		m_uPlanSeq = 0;
		return m_cPlanner.Init(astrURDF, adTs, astParams);
	}
	enum { IK_STEPS = 10 };
	void	After			(HL& acHl);
	CHlPlannerN<N>&	Planner	(	) { return m_cPlanner; }

private:
	CHlPlannerN<N>			m_cPlanner;
	uint32_t				m_uIkSeq, m_uPlanSeq;	// the requests it has taken
	typename HL::IkResult	m_stIk;
	typename HL::PlanResult	m_stPlan;
};

/* the arm (paper, Sec. IV-C); CHighLevelRTN<7>: a hand of the humanoid (Sec. VI) */
typedef CHighLevelRTN<6> CHighLevelRT;
typedef CHlPlannerN<6> CHlPlanner;
typedef CHlInlineN<6> CHlInline;

#endif // G2T_HIGH_LEVEL_RT_H
