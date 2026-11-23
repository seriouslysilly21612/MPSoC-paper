/*****************************************************************************
*	Name: ControlCycle.h
*	Description: The control task without its operating system: one call
*	per period, the trajectory part and then the dynamics part
*	(TaskTrajectory.cpp, TaskDynamics.cpp), for the control task on the R5
*	(S2). The R5 runs it after its EtherCAT send, on the measured state of
*	the period, and the torque it gives goes out with the send of the next
*	period, as the torque of the APU's control task does.
*
*	What the control task of the APU takes from its console and its tests
*	arrives here as commands (ipc/ipc_layout.h, IPC_CMD_*), one per period
*	at most, and the goal of the tool as a payload of the Goal slot:
*
*	  drives    a joint that leaves Operation enabled while the servo is
*	            asked for and was on, or a servo on that has not brought
*	            every joint there in its time: the motion ends and the
*	            servo is asked off (TaskEtherCAT.cpp has the two rules on
*	            the APU)
*	  command   controller on, off (off clears a controller fault); the
*	            stop of the motion; the hold of gravity compensation; a
*	            start of TRACK or HOME from the measured state, under the
*	            conditions of CRobotIndy7::CheckMotionStart; servo on under
*	            those of 'h', servo off. A command that is refused changes
*	            nothing, and the report says why
*	  goal      into the goal slot of the trajectory part (GoalSlot.h)
*	  camera    with a method that uses the mailbox (IPC_CMD_OBS; the
*	            region of SetObsLink): the receiver of the camera results
*	            (obs/ObsGoal.h), as the Trajectory task of the APU runs it
*	            with P0 and S1, before the trajectory part. Its status goes
*	            to the camera side (obs_status.h), its counts and one entry
*	            per result to the APU (obs_ring.h); in the TRACK of the
*	            application (IPC_START_OBS_GOALS) its objects, with the hover
*	            height, are the goals, until a new writer of the mailbox
*	            starts
*	  motion    CTrajectoryRT::Update on the measured state; HOME at rest
*	            or a generator error ends the motion with this period
*	  law       it follows the reference of this period (TRACK or HOME
*	            mode), without one it holds in gravity compensation; a law
*	            that fails (a non-finite state or torque) gives zero
*	            torque, latches a controller fault, ends the motion and
*	            asks the servo off
*	  report    what the period did, in the form in which the control task
*	            of the APU tells it to a second controller (--shadow,
*	            ShadowWriter.h), the state of the motion and the
*	            controller, the answer to the last command and the sample
*	            of the high-level controller for the logs
*
*	A period without a measured state (the bus not in OP, the frame lost,
*	the firmware stopping) runs neither part and gives no torque; its
*	command and its goal are still taken.
*
*	Not here: the JOINT TEST and the fault injection of the console.
*
*	The planner of the high-level controller runs where ServePlanner() is
*	called (HighLevelLink.h): on the R5 in the planner task. The constants
*	are the numbers of ShadowControl.h, so the controller is that of
*	CShadowControl and has its check value. No operating system, no
*	allocation after Init(): built for the firmware as it is
*	(rpu_work/r5_freertos/src/ctrl).
*****************************************************************************/
#ifndef G2T_CONTROL_CYCLE_H
#define G2T_CONTROL_CYCLE_H

#include <vector>

#include "FullDynControllerRT.h"
#include "GoalSlot.h"
#include "ShadowControl.h"
#include "ShadowWriter.h"
#include "TrajectoryRT.h"
#include "ipc/ipc_layout.h"
#include "obs/ObsGoal.h"
#include "obs/obs_ring.h"

class CControlCycle
{
public:
	enum { DOF = IPC_CTRL_DOF };
	/* periods a servo on has to bring every joint to Operation enabled, when the command names none
	 * (SERVO_ON_TIMEOUT_MS of the configuration at 1 kHz) */
	enum { SERVO_ON_PERIODS = 2000 };

	/* what a period starts from */
	struct Input
	{
		uint32_t	uPeriod;
		bool		bState;			// the measured state below is of this period: the two parts may run
		bool		bSimArm;		// ... of an arm that the APU simulates (step uSimStep), not of the bus
		bool		bServoOn;		// every joint in Operation enabled
		uint32_t	uNotReady;		// bit i: joint i cannot be switched on
		uint32_t	uSimStep;
		uint64_t	tRelease;		// the release of the period, ticks of the counter of SetObsLink
		float		fResponse;		// [s] release to torque of the period before; below 0: not known
		double		adQ[DOF], adQd[DOF];	// [rad], [rad/s]
		double		adTau[DOF];				// the torque the drives report [N m]
		const ipc_command_t*	pCommand;	// a command that is new in this period, or NULL
		const double*			pGoal;		// a goal that is new in this period [m], or NULL
	};
	/* what the drives get of it */
	struct Output
	{
		bool	bTorque;			// the law ran: adTau is the torque for the next send; without it 0
		double	adTau[DOF];			// [N m], after the torque limit
		int		nServo;				// IPC_CMD_SERVO_ON or IPC_CMD_SERVO_OFF: to be asked of the drives now; 0: nothing
	};

	CControlCycle();
	~CControlCycle();

	/* the receiver of the camera results: where its mailbox, its status and its record are in the region of the
	 * link, the ticks of the counter of the releases per second, its numbers (OBS_NUMBERS of ShadowControl.h) with
	 * their check value (ObsNumbersCrc), and the check value of [CONTRACT] (ObsContractCrc) for its status */
	struct ObsLink
	{
		volatile obs_mailbox_t*			pBox;
		volatile obs_status_region_t*	pStatus;
		volatile obs_ring_t*			pRing;
		double							dTicksPerS;
		const double*					pNumbers;
		uint32_t						uNumbersCrc;
		uint32_t						uContractCrc;
	};

	/* builds the controller (heap: before the periods start), as CShadowControl::Init. The controller is off. */
	BOOL	Init		(const char* astrURDF, const double* adNumbers, uint32_t auCrc);
	/* after Init(), before the periods. FALSE: a number of the receiver cannot be used, and IPC_CMD_OBS is
	 * refused, as it is without this call */
	BOOL	SetObsLink	(const ObsLink& astLink);
	/* the clock of the receiver's cost (eq. (6)), ticks of the counter of SetObsLink; NULL (the default): none.
	 * Before the periods. */
	void	SetObsClock	(uint64_t (*apNow)(void)) { m_cObs.SetClock(apNow); }
	/* what the receiver's work of a period took so far in its run (another task may read it: words, no lock) */
	const obs_cost_t&	ObsCost	(	) const { return m_cObs.Cost(); }
	BOOL	IsInit		(	) const { return m_pCtl != NULL; }

	/* one period. No allocation. */
	void	Step		(const Input& astIn, Output* apOut, ipc_report_t* apReport);
	/* the planner of the high-level controller, for the one task that runs it. Only once Init() has returned
	 * TRUE. TRUE: it did something, call again. No allocation. */
	BOOL	ServePlanner(	) { return m_cTraj.ServePlanner(); }

private:
	void		Command		(const ipc_command_t& astCommand, const Input& astIn, bool abDrivesWentOff, Output* apOut);
	uint32_t	Start		(const ipc_command_t& astCommand, const Input& astIn);
	void		StopMotion	(	);
	void		ServoOff	(Output* apOut);
	uint32_t	ObsCommand	(const ipc_command_t& astCommand);
	void		ObsStep		(const Input& astIn);
	static void	TagAt		(void* apArg, const float* afQ, double* adP);

	CControllerFullDynamicsRT*	m_pCtl;
	CTrajectoryRT				m_cTraj;
	CGoalSlot					m_cGoal;		// written and read by Step()
	CShadowWriter				m_cWriter;
	CControllerFullDynamicsRT::Reference m_stRef;
	CHlLink::Stats				m_stPlanner;	// what the planner had done when that could last be read
	CTrajectoryRT::Refusal		m_stRefusal;	// of the last start refused by CTrajectoryRT::Start or for a number
	std::vector<double>			m_vQ, m_vQd, m_vTor, m_vTau;
	double		m_adQ[DOF], m_adQd[DOF];		// the last measured state
	uint32_t	m_uCrc;
	bool		m_bFault;						// a controller fault is latched
	bool		m_bServoAsked, m_bServoReached;
	uint32_t	m_uServoWait, m_uServoLimit;	// periods since the servo on, and how many it has
	uint32_t	m_uCmdDone, m_uCmdResult, m_uStartSeq, m_uStartResult;
	uint32_t	m_uNoState, m_uFaults, m_uDriveStops, m_uGoals;
	uint32_t	m_uMotionPeriods;				// updates of the running motion so far
	/* the receiver of the camera results */
	double		m_dTs;							// the period [s] of Init()
	double		m_adObsNumbers[OBS_NUMBERS];	// of SetObsLink()
	ObsLink		m_stObs;
	bool		m_bObsLink;						// SetObsLink() took it
	CObsGoal::Config m_stObsCfg;				// ... and these settings; the policy comes with IPC_CMD_OBS
	CObsGoal	m_cObs;
	uint32_t	m_uObsMethod;					// obs_method::eMethod, or OBS_RING_OFF
	bool		m_bObsGoals;					// its objects are the goals of TRACK (IPC_START_OBS_GOALS) ...
	uint32_t	m_uObsGoalsEpoch;				// ... while the mailbox has the writer of that start
	CGoalSlot	m_cObsSlot;						// where it writes its objects
	unsigned	m_uObsSeen;						// ... the count of that slot at the last one taken
	uint32_t	m_uObsEntries;					// entries written into the record so far
	obs_recent_max_t m_stResponse;				// the response of the last periods (the status)
};

#endif // G2T_CONTROL_CYCLE_H
