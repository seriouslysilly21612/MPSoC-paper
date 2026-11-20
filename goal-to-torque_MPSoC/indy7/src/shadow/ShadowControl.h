/*****************************************************************************
*	Name: ShadowControl.h
*	Description: The controller of the control task a second time (--shadow;
*	plan stage 3, the verdict whether the control computation fits the 1 ms
*	of the R5). The R5 runs it after its EtherCAT send: the classes of the
*	control task (CHighLevelRT, CControllerFullDynamicsRT), fed in every
*	period with what the control task tells in the Shadow in slot
*	(ipc/ipc_layout.h; shadow/ShadowWriter.h writes it). Its torque is not
*	used: it goes back in the Shadow out slot with its difference to the
*	torque of the APU and the time the two parts took.
*
*	Following the control task:
*	  motion    a start is repeated from the measured state of the period
*	            (every start of the control task is from it), before or
*	            after the update of the period as on the APU; every update
*	            gets the measured state, the goal, the goal threshold and
*	            the ready posture of the APU's update
*	  planner   the IK of a goal is the APU's: an update takes it when the
*	            APU's did, so the two do not depend on how long each
*	            planner needs for it. The plans are this controller's own,
*	            from the requests of its own high-level controller, and its
*	            planner solves every goal as well: for the time that takes,
*	            and its solution is compared with the APU's (ik_diff of the
*	            answer), once per goal and only while this controller
*	            follows. The planner runs where ServePlanner() is called:
*	            on the R5 in a task below the EtherCAT task, so the time of
*	            Step() is that of the 1 ms part
*	  law       the mode, the hold switch and whether the reference is
*	            followed are those of the APU's period; the reference is
*	            this controller's own
*	  apart     a period without a payload (it did not arrive, or the APU
*	            gave none: shadow/ShadowWriter.h), or a start that is not
*	            told, leaves this controller's motion behind the APU's: the
*	            answers say so (IPC_SHADOW_OUT_APART) until the next start,
*	            which sets both to the same state again. So does a plan
*	            that comes into use in one of the two only (the planner of
*	            the other was late with it)
*	  not followed: a JOINT TEST (it starts with arguments of its own), a
*	            gain changed at the console, a torque fault injected there
*
*	The constants of the controller are numbers in a fixed order
*	(SHADOW_NUMBERS), so that the firmware gets them as a header and both
*	sides can tell that they are the same (their check value). No operating
*	system: this file and the classes it uses are built for the firmware as
*	they are (rpu_work/r5_freertos/src/ctrl).
*****************************************************************************/
#ifndef G2T_SHADOW_CONTROL_H
#define G2T_SHADOW_CONTROL_H

#include <vector>

#include "FullDynControllerRT.h"
#include "HighLevelRT.h"
#include "HighLevelLink.h"
#include "ipc/ipc_layout.h"

/* the constants of the controller, as CRobotIndy7::InitController and InitTrajectory take them from the
 * configuration: the period [s]; per joint Kp, Kd, Kf, the rotor inertia, the torque limit, the ready posture;
 * the gate of the friction feedforward (0 or 1); the parameters of the high-level controller */
enum
{
	SHADOW_N_TS = 0,
	SHADOW_N_KP,
	SHADOW_N_KD = SHADOW_N_KP + IPC_CTRL_DOF,
	SHADOW_N_KF = SHADOW_N_KD + IPC_CTRL_DOF,
	SHADOW_N_ROTOR = SHADOW_N_KF + IPC_CTRL_DOF,
	SHADOW_N_TAU_MAX = SHADOW_N_ROTOR + IPC_CTRL_DOF,
	SHADOW_N_READY = SHADOW_N_TAU_MAX + IPC_CTRL_DOF,
	SHADOW_N_FF_GATE = SHADOW_N_READY + IPC_CTRL_DOF,
	SHADOW_N_DELTA, SHADOW_N_LAMBDA, SHADOW_N_STEP_MAX, SHADOW_N_TOL, SHADOW_N_PLAN_AHEAD, SHADOW_N_ITER_MAX,
	SHADOW_N_VEL_MAX, SHADOW_N_ACC_MAX, SHADOW_N_JERK_MAX, SHADOW_N_MARGIN, SHADOW_N_LEAD, SHADOW_N_STEP_REF,
	SHADOW_NUMBERS
};

/* the constants of the receiver of the camera results (obs/ObsGoal.h) that the control task on the R5 (S2) runs, as
 * CRobotIndy7::InitObs takes them from the configuration: [CONTRACT] (delta, the age cap, eq. (7), the measurement
 * errors of the two modes, the offset of the tool tag), the goal checks and the smoothing of the application
 * (APP_*), and the age of the joint angles of a period (SIM_STATE_DELAY_STEPS). Apart from those of the controller:
 * a firmware built from other values of them refuses the methods with the mailbox (IPC_CMD_OBS), and only them */
enum
{
	OBS_N_DELTA = 0, OBS_N_AGE_CAP, OBS_N_E0, OBS_N_G, OBS_N_EPS_V, OBS_N_A_MAX, OBS_N_EPS_CAL, OBS_N_EPS_M,
	OBS_N_TAG = OBS_N_EPS_M + 2,
	OBS_N_BOX = OBS_N_TAG + 3,
	OBS_N_R_MAX = OBS_N_BOX + 6, OBS_N_FLOOR, OBS_N_HOVER, OBS_N_KAPPA, OBS_N_GAP, OBS_N_LOSS, OBS_N_STATE_AGE,
	OBS_NUMBERS
};

/* the check value of a controller: of its numbers (ShadowNumbersCrc of shadow/ShadowParams.h) and of its model
 * (the firmware: CTRL_NUMBERS_CRC and CTRL_MODEL_CRC of its two generated headers). Either alone changes it. */
inline uint32_t ShadowCheckValue(uint32_t auNumbersCrc, uint32_t auModelCrc)
{
	return auNumbersCrc ^ ((auModelCrc << 1) | (auModelCrc >> 31));
}

class CShadowControl
{
public:
	enum { DOF = IPC_CTRL_DOF };

	CShadowControl();
	~CShadowControl();

	/* a controller of the numbers: the parameters of its high-level controller, as CRobotIndy7::InitTrajectory
	 * takes them from the configuration, and its control law, as InitController makes it (NULL: its model could
	 * not be made). Also for the control task where it has no configuration file (control/ControlCycle.h) */
	static CHighLevelRT::Params			HlParams	(const double* adNumbers);
	static CControllerFullDynamicsRT*	NewLaw		(const char* astrURDF, const double* adNumbers);

	/* builds the two controllers (heap: before the periods start). astrURDF: the model, as for the control task
	 * (the firmware has its own, whatever the name); adNumbers: SHADOW_NUMBERS constants; auCrc: the check value
	 * that goes into every answer */
	BOOL	Init		(const char* astrURDF, const double* adNumbers, uint32_t auCrc);
	BOOL	IsInit		(	) const { return m_pCtl != NULL; }

	/* one period of the control task: its payload -> the answer. No allocation. */
	void	Step		(const ipc_shadow_in_t& astIn, ipc_shadow_out_t* apOut);
	/* the planner of the high-level controller (HighLevelLink.h), for the one task that runs it: the planner task
	 * of the firmware; on a PC whoever calls Step(), after it. Only once Init() has returned TRUE. TRUE: it did
	 * something, call again. No allocation. */
	BOOL	ServePlanner(	) { return m_cLink.Serve(m_cPlanner) ? TRUE : FALSE; }
	/* for a test that compares more than the answer has: the state of its high-level controller and the
	 * reference of its last update */
	const CHighLevelRT::Status&					HlStatus	(	) const { return m_cHl.GetStatus(); }
	const CControllerFullDynamicsRT::Reference&	Reference	(	) const { return m_stRef; }

private:
	void	StartMotion	(const ipc_shadow_in_t& astIn);

	CControllerFullDynamicsRT*	m_pCtl;
	CHighLevelRT				m_cHl;
	CHlPlanner					m_cPlanner;		// used by ServePlanner() only
	CHlLink						m_cLink;		// between m_cHl and m_cPlanner
	long						m_nPlansLate, m_nLateSeen;	// plans not back in their period: so far, of this motion
	CHlLink::Stats				m_stPlanner;	// what its planner had done when that could last be read
	CHighLevelRT::IkResult		m_stIkOwn;		// the last IK that its planner ran to its end
	double						m_adIkApu[DOF];	// the APU's solution of the goal of request m_uIkApu
	uint32_t					m_uIkApu;		// 0: none that waits to be compared
	uint32_t					m_uIkCompared;	// goals compared so far
	double						m_dIkDiff;		// the last of them: largest difference over the joints [rad]
	std::vector<double>			m_vQ, m_vQd, m_vTor, m_vTau;
	CControllerFullDynamicsRT::Reference m_stRef;	// of the last update of the high-level controller
	uint32_t	m_uCrc;
	uint32_t	m_uPeriod;			// of the last payload
	uint32_t	m_uStarts;			// starts of the APU that this controller has repeated
	uint32_t	m_uMissed;
	bool		m_bFirst;			// no payload yet
	bool		m_bApart;
};

#endif // G2T_SHADOW_CONTROL_H
