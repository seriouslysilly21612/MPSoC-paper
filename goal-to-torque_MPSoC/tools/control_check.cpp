/*****************************************************************************
*	Name: control_check.cpp
*	Description: The control task without its operating system
*	(shadow/ControlCycle.h; S2: the control task on the R5), on a PC.
*
*	    bin/control_check [-f FILE] [-o VECTORS]
*
*	  -f FILE     the configuration, found as by indy7_ctrl
*	  -o VECTORS  the periods of the run as a file (shadow/control_vectors.h)
*	              for the control cycle as the firmware of the R5 builds it
*
*	The control cycle built from the numbers of the configuration
*	(shadow/ShadowParams.h), on an arm that follows its reference with a
*	small error and drives that come on a few periods after they are asked.
*	It is given commands and goals as the APU gives them and goes through:
*	no measured state; the controller off and on; servo on under its
*	interlock, a servo on that runs out, a dropout; the hold; HOME to the
*	ready posture until it ends by itself; a joint move; TRACK with goals,
*	one of them given before the start; the stops; a start refused for
*	each of its reasons; numbers of a command that are not usable; a law
*	that fails; a period without a state in the middle of a motion; a
*	command that is none.
*
*	Checked: what each command does and what the report says of it; the
*	torque and the servo request that the drives get; and, in every period,
*	that a second controller (shadow/ShadowControl.h) fed with the period as
*	the report describes it gives the same torque and the same reference,
*	bit for bit: what the control cycle says it did is what it did.
*
*	Exit code 0 when no check failed.
*****************************************************************************/
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <vector>
#include <unistd.h>

#include "base/Config.h"
#include "shadow/ControlCycle.h"
#include "shadow/ShadowControl.h"
#include "shadow/ShadowParams.h"
#include "shadow/control_vectors.h"

namespace {

const int N = IPC_CTRL_DOF;
typedef CControllerFullDynamicsRT C;

int g_nFailed = 0;

void Expect(bool abOk, const char* apcWhat)
{
	if (abOk)
		return;
	g_nFailed++;
	printf("  FAILED: %s\n", apcWhat);
}

/* the control cycle with its arm, its drives and the second controller that follows its reports */
struct Bench
{
	CControlCycle	cCtl;
	CShadowControl	cShadow;
	double		adQ[N], adQd[N];
	bool		bState = true;			// the next period has a measured state
	bool		bServoOn = false;
	bool		bDrivesCome = true;		// the drives reach Operation enabled when asked
	uint32_t	uNotReady = 0;
	int			nServoIn = 0;			// periods until the drives are on; 0: no request under way
	uint32_t	uPeriod = 0, uSeq = 0;
	CControlCycle::Output	stOut;
	ipc_report_t			stRep;
	long		nPeriods = 0, nLaw = 0, nTraj = 0, nDiff = 0, nApart = 0, nFollowerFault = 0, nNoPayload = 0, nGoals = 0;
	std::vector<control_vector_t>	vVectors;	// every period, for -o

	/* one period with a command and a goal, or without */
	void Period(const ipc_command_t* apCommand = NULL, const double* apGoal = NULL)
	{
		CControlCycle::Input stIn;
		memset(&stIn, 0, sizeof(stIn));
		stIn.uPeriod = ++uPeriod;
		stIn.bState = bState;
		stIn.bServoOn = bState && bServoOn;		// without a frame nothing is known of the drives
		stIn.uNotReady = uNotReady;
		for (int i = 0; i < N; i++)
		{
			stIn.adQ[i] = adQ[i];
			stIn.adQd[i] = adQd[i];
		}
		stIn.pCommand = apCommand;
		stIn.pGoal = apGoal;
		cCtl.Step(stIn, &stOut, &stRep);
		while (cCtl.ServePlanner())		// the planner task, here: back for the next period
			;
		nPeriods++;
		if (apGoal != NULL)
			nGoals++;
		{
			control_vector_t stV;
			memset(&stV, 0, sizeof(stV));
			stV.period = stIn.uPeriod;
			stV.flags = (stIn.bState ? (uint32_t)CONTROL_V_STATE : 0u) | (stIn.bServoOn ? (uint32_t)CONTROL_V_SERVO_ON : 0u) |
			            (apCommand != NULL ? (uint32_t)CONTROL_V_COMMAND : 0u) | (apGoal != NULL ? (uint32_t)CONTROL_V_GOAL : 0u) |
			            (stOut.bTorque ? (uint32_t)CONTROL_V_TORQUE : 0u);
			stV.not_ready = stIn.uNotReady;
			stV.servo = stOut.nServo;
			for (int i = 0; i < N; i++)
			{
				stV.q[i] = stIn.adQ[i];
				stV.qd[i] = stIn.adQd[i];
				stV.tau[i] = stOut.adTau[i];
			}
			for (int k = 0; apGoal != NULL && k < 3; k++)
				stV.goal[k] = apGoal[k];
			if (apCommand != NULL)
				stV.command = *apCommand;
			stV.report = stRep;
			// the times of this run are not part of what the file says: the same file from every run
			stV.report.traj_ns = stRep.traj_ns != 0 ? 1u : 0u;
			stV.report.ctrl_ns = stRep.ctrl_ns != 0 ? 1u : 0u;
			stV.report.hl_ns = stV.report.plan_us = stV.report.plan_max_us = stV.report.ik_us = stV.report.ik_max_us = 0u;
			vVectors.push_back(stV);
		}
		// the report has the period and, with a state, that state; without the law there is no torque
		if (stRep.in.period != uPeriod ||
		    (bState && (memcmp(stRep.in.q, adQ, sizeof(adQ)) != 0 || memcmp(stRep.in.qd, adQd, sizeof(adQd)) != 0)))
			nDiff++;
		for (int i = 0; i < N; i++)
			if (!stOut.bTorque && stOut.adTau[i] != 0.0)
				nDiff++;

		/* the second controller, on what the report says of the period */
		if (!(stRep.state & IPC_CTL_PAYLOAD))
			nNoPayload++;
		else
		{
			ipc_shadow_out_t stFollow;
			cShadow.Step(stRep.in, &stFollow);
			while (cShadow.ServePlanner())
				;
			if (stFollow.flags & IPC_SHADOW_OUT_APART)
				nApart++;
			if (stFollow.flags & IPC_SHADOW_OUT_FAULT)
				nFollowerFault++;
			else
			{
				if (stFollow.flags & IPC_SHADOW_OUT_CTRL)
				{
					nLaw++;
					if (stFollow.tau_diff != 0.0 || memcmp(stFollow.tau, stRep.in.tau, sizeof(stFollow.tau)) != 0 ||
					    memcmp(stOut.adTau, stRep.in.tau, sizeof(stOut.adTau)) != 0)
						nDiff++;
				}
				if (stFollow.flags & IPC_SHADOW_OUT_TRAJ)
				{
					nTraj++;
					if (memcmp(stFollow.q_ref, stRep.q_ref, sizeof(stFollow.q_ref)) != 0)
						nDiff++;
				}
			}
			// the two say the same of what ran
			if (((stFollow.flags & IPC_SHADOW_OUT_CTRL) != 0) != stOut.bTorque ||
			    ((stFollow.flags & IPC_SHADOW_OUT_TRAJ) != 0) != ((stRep.state & IPC_CTL_SAMPLE) != 0))
				nDiff++;
		}

		/* the drives, then the arm: on its reference with a small error while the law follows one, else at rest */
		if (stOut.nServo == IPC_CMD_SERVO_ON)
			nServoIn = 3;
		else if (stOut.nServo == IPC_CMD_SERVO_OFF)
		{
			bServoOn = false;
			nServoIn = 0;
		}
		if (nServoIn > 0 && --nServoIn == 0 && bDrivesCome)
			bServoOn = true;
		const bool bFollowed = (stRep.in.flags & IPC_SHADOW_REF) != 0;
		for (int i = 0; i < N; i++)
		{
			if (bFollowed)
				adQ[i] = stRep.q_ref[i] + 2e-5 * std::sin(0.01 * (double)uPeriod + (double)i);
			adQd[i] = bFollowed ? stRep.qd_ref[i] : 0.0;
		}
	}
	void Periods(int anCount)
	{
		for (int k = 0; k < anCount; k++)
			Period();
	}
	/* a period with the command; its result */
	uint32_t Command(uint32_t auCmd, uint32_t auArg = 0, const ipc_command_t* apNumbers = NULL)
	{
		ipc_command_t stC;
		memset(&stC, 0, sizeof(stC));
		if (apNumbers != NULL)
			stC = *apNumbers;
		stC.seq = ++uSeq;
		stC.cmd = auCmd;
		stC.arg = auArg;
		Period(&stC);
		Expect(stRep.cmd_done == uSeq, "the report has the number of the command taken");
		return stRep.cmd_result;
	}
	bool Has(uint32_t auState) const { return (stRep.state & auState) == auState; }
	bool Moving() const { return stRep.traj_state == (uint32_t)CTrajectoryRT::eRunning; }
	/* periods until the motion has ended, anMost at most; the count, -1: it did not end */
	int UntilEnd(int anMost)
	{
		for (int k = 1; k <= anMost; k++)
		{
			Period();
			if (!Moving())
				return k;
		}
		return -1;
	}
};

} // namespace

int main(int argc, char** argv)
{
	TSTRING strCfg, strOut;
	int nOpt;
	while ((nOpt = getopt(argc, argv, "f:o:")) != -1)
	{
		if (nOpt == 'f') strCfg = optarg;
		else if (nOpt == 'o') strOut = optarg;
		else
		{
			fprintf(stderr, "usage: %s [-f FILE] [-o VECTORS]\n", argv[0]);
			return 2;
		}
	}
	CConfig cConfig;
	strCfg = CConfig::Find(strCfg, argv[0]);
	if (FALSE == cConfig.Load(strCfg, FALSE))
		return 2;
	const char* const pcURDF = cConfig.System().strURDFPath.c_str();
	double adNumbers[SHADOW_NUMBERS];
	Bench b;
	CHighLevelRT cHl;		// for the tool position of a posture
	if (!ShadowNumbers(cConfig, adNumbers) || !b.cCtl.Init(pcURDF, adNumbers, 0xC0DE0001u) ||
	    !b.cShadow.Init(pcURDF, adNumbers, 0xC0DE0001u) ||
	    !cHl.Init(pcURDF, adNumbers[SHADOW_N_TS], CShadowControl::HlParams(adNumbers)))
	{
		fprintf(stderr, "%s: the controller cannot be built from it\n", strCfg.c_str());
		return 2;
	}
	const CHighLevelRT::Params stP = CShadowControl::HlParams(adNumbers);
	const double* const adQr = &adNumbers[SHADOW_N_READY];
	ipc_command_t stStart;		// the numbers of a start: the goal threshold and the ready posture of the configuration
	memset(&stStart, 0, sizeof(stStart));
	stStart.delta = stP.dDelta;
	for (int i = 0; i < N; i++)
	{
		stStart.q_ready[i] = adQr[i];
		stStart.target[i] = adQr[i] + 0.2;
		b.adQ[i] = adQr[i] + 0.25 * ((i % 2) ? 1.0 : -1.0);		// away from the ready posture
		b.adQd[i] = 0.0;
	}
	double adP0[3], adGoal[3], adGoal2[3];
	cHl.ToolPosition(adQr, adP0);
	for (int k = 0; k < 3; k++)
	{
		adGoal[k] = adP0[k] + (k == 0 ? 0.06 : k == 1 ? -0.04 : 0.03);
		adGoal2[k] = adP0[k] + (k == 0 ? -0.05 : k == 1 ? 0.07 : -0.02);
	}
	const uint32_t HOME = IPC_START_HOME, FF = IPC_START_FRICTION_FF;

	/* --- not built --- */
	{
		CControlCycle cNone;
		CControlCycle::Input stIn;
		CControlCycle::Output stOut;
		ipc_report_t stRep;
		memset(&stIn, 0, sizeof(stIn));
		stIn.uPeriod = 7;
		stIn.bState = true;
		cNone.Step(stIn, &stOut, &stRep);
		Expect(!cNone.IsInit() && !stOut.bTorque && stOut.nServo == 0 && stRep.in.period == 7 && stRep.state == 0,
		       "a control cycle that is not built gives nothing, and its report has the period");
	}

	/* --- nothing without a state, nothing with the controller off --- */
	b.bState = false;
	b.Period();
	Expect(!b.stOut.bTorque && b.stOut.nServo == 0 && b.stRep.state == IPC_CTL_PAYLOAD && b.stRep.n_no_state == 1 &&
	       b.stRep.in.period == 1 && b.stRep.in.flags == IPC_SHADOW_HOLD && b.stRep.ctrl_crc == 0xC0DE0001u,
	       "a period without a measured state: no torque, and the report says so");
	Expect(b.Command(IPC_CMD_START, HOME, &stStart) == IPC_REFUSED_NO_STATE && b.stRep.start_seq == b.uSeq &&
	       b.stRep.start_result == IPC_REFUSED_NO_STATE, "no start without a measured state");
	b.bState = true;
	b.Periods(3);
	Expect(!b.stOut.bTorque && b.Has(IPC_CTL_STATE) && !b.Has(IPC_CTL_ENABLED) && b.stRep.ctrl_ns == 0 &&
	       !(b.stRep.in.flags & IPC_SHADOW_CTRL) && b.stRep.n_no_state == 2, "the controller is off after Init: no torque");
	Expect(b.Command(IPC_CMD_START, HOME, &stStart) == IPC_REFUSED_NOT_ENABLED, "no start with the controller off");
	Expect(b.Command(IPC_CMD_SERVO_ON) == IPC_REFUSED_NOT_ENABLED && b.stOut.nServo == 0 && !b.Has(IPC_CTL_SERVO_ASKED),
	       "no servo on with the controller off");

	/* --- the controller on: gravity compensation; the hold --- */
	Expect(b.Command(IPC_CMD_CTRL_ON) == IPC_DONE_OK && b.stOut.bTorque && b.Has(IPC_CTL_ENABLED) && b.stRep.ctrl_ns != 0 &&
	       (b.stRep.in.flags & IPC_SHADOW_CTRL) && b.stRep.in.ctrl_mode == (uint32_t)C::eGravityCompensation,
	       "controller on: the law runs in gravity compensation from the period of the command on");
	{
		double dTau = 0.0;
		for (int i = 0; i < N; i++)
			dTau += std::fabs(b.stOut.adTau[i]);
		Expect(dTau > 1.0, "the torque of gravity compensation is there");
	}
	const bool bHoldAtStart = (b.stRep.in.flags & IPC_SHADOW_HOLD) != 0;
	Expect(b.Command(IPC_CMD_HOLD_OFF) == IPC_DONE_OK && !(b.stRep.in.flags & IPC_SHADOW_HOLD) &&
	       b.Command(IPC_CMD_HOLD_ON) == IPC_DONE_OK && (b.stRep.in.flags & IPC_SHADOW_HOLD) && bHoldAtStart,
	       "the hold of gravity compensation: on after Init, off and on by command");
	b.Periods(20);
	Expect(b.Command(IPC_CMD_START, HOME, &stStart) == IPC_REFUSED_SERVO_OFF && !b.Moving(), "no start with the servo off");

	/* --- servo on: the interlock, a request that runs out, the drives coming on --- */
	b.uNotReady = 1u << 2;
	Expect(b.Command(IPC_CMD_SERVO_ON) == IPC_REFUSED_NOT_READY && b.stOut.nServo == 0, "no servo on while a drive cannot be switched on");
	b.uNotReady = 0;
	b.bState = false;
	Expect(b.Command(IPC_CMD_SERVO_ON) == IPC_REFUSED_NO_STATE && b.stOut.nServo == 0, "no servo on without a measured state");
	b.bState = true;
	b.bDrivesCome = false;
	Expect(b.Command(IPC_CMD_SERVO_ON, 5) == IPC_DONE_OK && b.stOut.nServo == IPC_CMD_SERVO_ON && b.Has(IPC_CTL_SERVO_ASKED),
	       "servo on: asked of the drives in the period of the command");
	b.Periods(4);
	Expect(b.Has(IPC_CTL_SERVO_ASKED) && b.stOut.nServo == 0 && b.stRep.n_drive_stops == 0, "the request stands for its 5 periods");
	b.Period();
	Expect(!b.Has(IPC_CTL_SERVO_ASKED) && b.stOut.nServo == IPC_CMD_SERVO_OFF && b.stRep.n_drive_stops == 1,
	       "a servo on that has not brought the drives on in its periods is withdrawn");
	b.Period();
	Expect(b.stOut.nServo == 0 && b.stRep.n_drive_stops == 1, "... once");
	b.bDrivesCome = true;
	Expect(b.Command(IPC_CMD_SERVO_ON, 5) == IPC_DONE_OK && b.stOut.nServo == IPC_CMD_SERVO_ON, "servo on again, with 5 periods");
	b.Periods(3);
	Expect(b.bServoOn && b.Has(IPC_CTL_SERVO_ASKED | IPC_CTL_SERVO_ON) && b.stRep.n_drive_stops == 1,
	       "the drives are on: the request had its own periods");
	b.Periods(10);
	Expect(b.Has(IPC_CTL_SERVO_ASKED) && b.stRep.n_drive_stops == 1, "... and stands once they are");

	/* --- the numbers of a start --- */
	{
		ipc_command_t stBad = stStart;
		stBad.delta = std::numeric_limits<double>::quiet_NaN();
		Expect(b.Command(IPC_CMD_START, HOME, &stBad) == IPC_REFUSED_ARGUMENT && b.stRep.refusal_joint == -1 && !b.Moving(),
		       "a start with a goal threshold that is no number is refused");
		stBad.delta = -0.001;
		Expect(b.Command(IPC_CMD_START, HOME, &stBad) == IPC_REFUSED_ARGUMENT, "... or a negative one");
		stBad.delta = 1.5;
		Expect(b.Command(IPC_CMD_START, HOME, &stBad) == IPC_REFUSED_ARGUMENT && b.stRep.refusal_value == 1.5, "... or one above 1 m");
		stBad = stStart;
		stBad.q_ready[3] = stP.adLimHi[3] + 0.01;
		Expect(b.Command(IPC_CMD_START, HOME, &stBad) == IPC_REFUSED_ARGUMENT && b.stRep.refusal_joint == 3 &&
		       b.stRep.refusal_value == stBad.q_ready[3] && b.stRep.refusal_hi == stP.adLimHi[3] && !b.Moving(),
		       "a start with a ready posture outside the range of a joint is refused, with the joint");
		stBad.q_ready[3] = std::numeric_limits<double>::quiet_NaN();
		Expect(b.Command(IPC_CMD_START, HOME, &stBad) == IPC_REFUSED_ARGUMENT && b.stRep.refusal_joint == 3, "... or one that is no number");
		stBad.q_ready[3] = adQr[3];
		stBad.q_ready[5] = stP.adLimLo[5] - 0.01;
		Expect(b.Command(IPC_CMD_START, HOME, &stBad) == IPC_REFUSED_ARGUMENT && b.stRep.refusal_joint == 5 &&
		       b.stRep.refusal_lo == stP.adLimLo[5], "... or below the range");
		stBad = stStart;
		stBad.target[1] = stP.adLimHi[1];		// inside the range, not inside the margin
		Expect(b.Command(IPC_CMD_START, HOME | IPC_START_TARGET, &stBad) == IPC_REFUSED_START &&
		       b.stRep.refusal_why == (int32_t)CTrajectoryRT::eTargetOutside && b.stRep.refusal_joint == 1 &&
		       b.stRep.start_result == IPC_REFUSED_START && !b.Moving(),
		       "a joint move to a target outside the limits less the margin is refused by the trajectory part");
		const double dWas = b.adQd[0];
		b.adQd[0] = 0.2;
		Expect(b.Command(IPC_CMD_START, HOME, &stStart) == IPC_REFUSED_START &&
		       b.stRep.refusal_why == (int32_t)CTrajectoryRT::eArmMoves, "no start while the arm moves");
		b.adQd[0] = dWas;
		const double dQ = b.adQ[4];
		b.adQ[4] = stP.adLimLo[4] - 0.1;
		Expect(b.Command(IPC_CMD_START, HOME, &stStart) == IPC_REFUSED_START &&
		       b.stRep.refusal_why == (int32_t)CTrajectoryRT::eJointOutside && b.stRep.refusal_joint == 4,
		       "no start with a joint outside its range");
		b.adQ[4] = dQ;
	}
	Expect(b.Command(77u) == IPC_REFUSED_UNKNOWN && b.stOut.bTorque && b.Has(IPC_CTL_SERVO_ASKED), "a command that is none changes nothing");

	/* --- HOME to the ready posture, to its end --- */
	const long nPlansBefore = b.stRep.n_plans;
	Expect(b.Command(IPC_CMD_START, HOME | FF, &stStart) == IPC_DONE_OK && b.stRep.start_result == IPC_DONE_OK && b.Moving() &&
	       b.stRep.motion == (uint32_t)CTrajectoryRT::eMotionHome && b.Has(IPC_CTL_SAMPLE) && b.stRep.n_cycle == 0 &&
	       (b.stRep.in.flags & (IPC_SHADOW_START | IPC_SHADOW_START_HOME | IPC_SHADOW_TRAJ)) ==
	       (IPC_SHADOW_START | IPC_SHADOW_START_HOME | IPC_SHADOW_TRAJ) && !(b.stRep.in.flags & IPC_SHADOW_START_AFTER) &&
	       b.stRep.in.starts == 1 && b.stRep.traj_ns != 0,
	       "HOME starts in the period of its command, which has its first update");
	{
		ipc_command_t stOther = stStart;		// with other numbers than the motion that runs
		stOther.delta = 0.5 * stP.dDelta;
		stOther.q_ready[0] += 0.1;
		Expect(b.Command(IPC_CMD_START, 0, &stOther) == IPC_REFUSED_START && b.stRep.refusal_why == (int32_t)CTrajectoryRT::eMotionRuns &&
		       b.stRep.refusal_joint == (int32_t)CTrajectoryRT::eMotionHome && b.Moving() && b.stRep.in.starts == 1 &&
		       b.stRep.in.delta == stP.dDelta && b.stRep.in.q_ready[0] == adQr[0],
		       "no start while a motion runs, and that motion goes on with its own numbers");
	}
	Expect(b.Command(IPC_CMD_SERVO_ON) == IPC_REFUSED_MODE, "no servo on while the law follows a reference");
	b.Period();
	Expect(b.stRep.in.ctrl_mode == (uint32_t)C::eHome && (b.stRep.in.flags & IPC_SHADOW_REF) &&
	       (b.stRep.in.flags & IPC_SHADOW_FRICTION_FF) && b.stRep.n_cycle == 3,
	       "the law follows the reference of HOME, with the friction feedforward of the start");
	b.Periods(300);
	{
		double dQd = 0.0;
		for (int i = 0; i < N; i++)
			dQd = std::fmax(dQd, std::fabs(b.stRep.qd_ref[i]));
		Expect(b.Moving() && dQd > 0.02 && b.stRep.n_cycle == 303, "the report has the velocity of the reference");
	}
	const int nHome = b.UntilEnd(20000);
	double dOff = 0.0;
	for (int i = 0; i < N; i++)
		dOff = std::fmax(dOff, std::fabs(b.adQ[i] - adQr[i]));
	Expect(nHome > 200 && b.stRep.traj_state == (uint32_t)CTrajectoryRT::eDoneAtRest && b.stRep.motion == 0 && dOff < 1e-3 &&
	       (b.stRep.in.flags & IPC_SHADOW_TRAJ) && !(b.stRep.in.flags & IPC_SHADOW_REF) &&
	       b.stRep.in.ctrl_mode == (uint32_t)C::eGravityCompensation && b.stRep.n_plans > (uint32_t)nPlansBefore,
	       "HOME ends by itself at rest at the ready posture, and the law holds in gravity compensation from that period on");
	b.Period();
	Expect(!b.Has(IPC_CTL_SAMPLE) && b.stRep.traj_ns == 0 && b.stOut.bTorque && !(b.stRep.in.flags & IPC_SHADOW_TRAJ),
	       "no update of the trajectory part without a motion");

	/* --- a joint move, with numbers of its own --- */
	ipc_command_t stOwn = stStart;
	stOwn.delta = 0.0;
	stOwn.q_ready[1] = adQr[1] - 0.15;
	Expect(b.Command(IPC_CMD_START, HOME | IPC_START_TARGET, &stOwn) == IPC_DONE_OK && b.Moving() &&
	       memcmp(b.stRep.in.start_target, stStart.target, sizeof(stStart.target)) == 0 && b.stRep.in.starts == 2 &&
	       !(b.stRep.in.flags & IPC_SHADOW_FRICTION_FF) && b.stRep.n_cycle == 0 && b.stRep.in.delta == 0.0 &&
	       b.stRep.in.q_ready[1] == stOwn.q_ready[1],
	       "a joint move starts to its target, with the goal threshold and the ready posture of its command");
	const int nMove = b.UntilEnd(20000);
	dOff = 0.0;
	for (int i = 0; i < N; i++)
		dOff = std::fmax(dOff, std::fabs(b.adQ[i] - stStart.target[i]));
	Expect(nMove > 200 && b.stRep.traj_state == (uint32_t)CTrajectoryRT::eDoneAtRest && dOff < 1e-3, "... and ends there");
	Expect(b.Command(IPC_CMD_START, HOME, &stOwn) == IPC_DONE_OK && b.Moving(), "HOME to the ready posture of the command");
	const int nHome2 = b.UntilEnd(20000);
	dOff = 0.0;
	for (int i = 0; i < N; i++)
		dOff = std::fmax(dOff, std::fabs(b.adQ[i] - stOwn.q_ready[i]));
	Expect(nHome2 > 200 && b.stRep.traj_state == (uint32_t)CTrajectoryRT::eDoneAtRest && dOff < 1e-3, "... ends at that posture");
	Expect(b.Command(IPC_CMD_START, HOME, &stStart) == IPC_DONE_OK && b.UntilEnd(20000) > 200, "and HOME back to that of the configuration");

	/* --- TRACK: a goal from before the start does not count; goals; a period without a state; the stop --- */
	b.Period(NULL, adGoal2);
	Expect(b.stRep.n_goals == 1, "a goal is taken without a motion");
	Expect(b.Command(IPC_CMD_START, 0, &stStart) == IPC_DONE_OK && b.Moving() && b.stRep.motion == (uint32_t)CTrajectoryRT::eMotionTrack &&
	       !(b.stRep.in.flags & IPC_SHADOW_START_HOME) && !(b.stRep.in.flags & IPC_SHADOW_GOAL) && b.stRep.in.delta == stP.dDelta,
	       "TRACK starts, without a goal");
	b.Periods(50);
	Expect(b.Moving() && !(b.stRep.in.flags & IPC_SHADOW_GOAL) && !(b.stRep.hl_flags & CTrajectoryRT::HL_HAS_GOAL),
	       "the goal from before the start is not followed");
	const uint32_t uIksBefore = b.stRep.n_iks, uPlansTrack = b.stRep.n_plans;
	b.Period(NULL, adGoal);
	Expect(b.stRep.n_goals == 2 && (b.stRep.in.flags & IPC_SHADOW_GOAL) && memcmp(b.stRep.in.goal, adGoal, sizeof(adGoal)) == 0 &&
	       b.stRep.goal_age == 0, "a goal given in a period of TRACK goes into its update");
	b.Periods(1500);
	Expect(b.stRep.goal_age == 1500, "the report has the age of the goal");
	b.bState = false;
	const uint32_t uCycleBefore = b.stRep.n_cycle;
	b.Period();
	Expect(!b.stOut.bTorque && !b.Has(IPC_CTL_STATE) && !b.Has(IPC_CTL_SAMPLE) && b.Moving() && b.stRep.in.flags == IPC_SHADOW_HOLD,
	       "a period without a state in a motion: neither part runs, no torque, the motion stays");
	b.bState = true;
	b.Period();
	Expect(b.Has(IPC_CTL_SAMPLE) && b.stRep.n_cycle == uCycleBefore + 1 && b.stOut.bTorque, "... and goes on with the next state");
	b.Periods(4000);
	double adTool[3], dFar = 0.0;
	cHl.ToolPosition(b.adQ, adTool);
	for (int k = 0; k < 3; k++)
		dFar = std::fmax(dFar, std::fabs(adTool[k] - adGoal[k]));
	Expect(b.Moving() && dFar < 2e-3 && b.stRep.n_iks > uIksBefore && b.stRep.n_plans > uPlansTrack &&
	       (b.stRep.hl_flags & CTrajectoryRT::HL_AT_REST) && std::fabs(b.stRep.accepted[0] - adGoal[0]) < 1e-12 &&
	       std::fabs(b.stRep.tool[0] - adTool[0]) < 1e-3 && b.stRep.n_plan_late == 0,
	       "TRACK brings the tool to the goal and stays, with the IK and the plans of its planner");
	b.Period(NULL, adGoal2);
	b.Periods(300);
	Expect(b.Moving() && (b.stRep.in.flags & IPC_SHADOW_REF) && b.stRep.in.ctrl_mode == (uint32_t)C::eTrack, "a second goal: on the way");
	Expect(b.Command(IPC_CMD_MOTION_STOP) == IPC_DONE_OK && b.stRep.traj_state == (uint32_t)CTrajectoryRT::eAborted &&
	       !b.Has(IPC_CTL_SAMPLE) && b.stOut.bTorque && b.stRep.in.ctrl_mode == (uint32_t)C::eGravityCompensation &&
	       b.Has(IPC_CTL_SERVO_ASKED) && b.stOut.nServo == 0,
	       "the stop ends the motion in its period: gravity compensation, the servo stays on");
	b.Periods(300);
	Expect(b.Command(IPC_CMD_START, 0, &stStart) == IPC_DONE_OK && b.Moving(), "TRACK");
	b.Period(NULL, adGoal);
	b.Periods(100);
	b.bState = false;
	Expect(b.Command(IPC_CMD_MOTION_STOP) == IPC_DONE_OK && !b.Moving() && b.stRep.in.ctrl_mode == (uint32_t)C::eGravityCompensation &&
	       b.stRep.n_drive_stops == 1, "a stop in a period without a state acts in it, and that period tells no dropout");
	b.bState = true;
	b.Periods(300);

	/* --- the drives go off in a motion; servo off; controller off --- */
	Expect(b.Command(IPC_CMD_START, 0, &stStart) == IPC_DONE_OK && b.Moving(), "TRACK again");
	b.Period(NULL, adGoal);
	b.Periods(200);
	b.bServoOn = false;			// a dropout
	{
		ipc_command_t stOn;
		memset(&stOn, 0, sizeof(stOn));
		stOn.seq = ++b.uSeq;
		stOn.cmd = IPC_CMD_SERVO_ON;
		b.Period(&stOn);
	}
	Expect(b.stRep.traj_state == (uint32_t)CTrajectoryRT::eAborted && b.stOut.nServo == IPC_CMD_SERVO_OFF &&
	       !b.Has(IPC_CTL_SERVO_ASKED) && b.stRep.n_drive_stops == 2 && b.stRep.cmd_result == IPC_REFUSED_NOT_READY && b.stOut.bTorque,
	       "a drive that leaves Operation enabled: the motion ends, the servo is asked off, and a servo on of that period is dropped");
	b.Periods(300);
	Expect(b.stRep.n_drive_stops == 2 && b.stOut.nServo == 0, "... once");
	Expect(b.Command(IPC_CMD_SERVO_ON) == IPC_DONE_OK, "servo on after the dropout");
	b.Periods(3);
	Expect(b.Command(IPC_CMD_START, HOME, &stStart) == IPC_DONE_OK && b.Moving(), "HOME");
	b.Periods(100);
	Expect(b.Command(IPC_CMD_SERVO_OFF) == IPC_DONE_OK && b.stRep.traj_state == (uint32_t)CTrajectoryRT::eAborted &&
	       b.stOut.nServo == IPC_CMD_SERVO_OFF && !b.Has(IPC_CTL_SERVO_ASKED) && b.stRep.n_drive_stops == 2,
	       "servo off in a motion ends it");
	b.Periods(300);
	Expect(b.Command(IPC_CMD_SERVO_ON) == IPC_DONE_OK, "servo on");
	b.Periods(3);
	Expect(b.Command(IPC_CMD_START, HOME, &stStart) == IPC_DONE_OK && b.Moving(), "HOME");
	b.Periods(100);
	Expect(b.Command(IPC_CMD_CTRL_OFF) == IPC_DONE_OK && b.stRep.traj_state == (uint32_t)CTrajectoryRT::eAborted && !b.stOut.bTorque &&
	       !b.Has(IPC_CTL_ENABLED) && b.stRep.ctrl_ns == 0, "controller off in a motion ends it: no torque");
	b.Periods(300);
	Expect(b.Command(IPC_CMD_STOP) == IPC_DONE_OK && b.stOut.nServo == IPC_CMD_SERVO_OFF, "the stop of the drives is a servo off");
	b.Periods(5);

	/* --- a law that fails --- */
	Expect(b.Command(IPC_CMD_CTRL_ON) == IPC_DONE_OK && b.Command(IPC_CMD_SERVO_ON) == IPC_DONE_OK, "controller and servo on");
	b.Periods(3);
	Expect(b.Command(IPC_CMD_START, HOME, &stStart) == IPC_DONE_OK && b.Moving(), "HOME");
	b.Periods(100);
	const double dQ2 = b.adQ[2];
	const long nFaultBefore = b.nFollowerFault;
	b.adQ[2] = std::numeric_limits<double>::quiet_NaN();
	b.Period();
	const CControlCycle::Output stAtFault = b.stOut;
	const ipc_report_t stRepAtFault = b.stRep;
	b.adQ[2] = std::numeric_limits<double>::quiet_NaN();		// the arm had gone to its reference: still no number
	b.Periods(2);
	Expect(b.stRep.n_ctrl_faults == 1 && b.stOut.nServo == 0 && b.stOut.bTorque && b.nFollowerFault == nFaultBefore + 3,
	       "a law that fails again while the fault is latched repeats nothing");
	b.stOut = stAtFault;
	b.stRep = stRepAtFault;
	double dSum = 0.0;
	for (int i = 0; i < N; i++)
		dSum += std::fabs(b.stOut.adTau[i]);
	Expect(b.stOut.bTorque && dSum == 0.0 && b.Has(IPC_CTL_FAULT) && b.stRep.n_ctrl_faults == 1 && !b.Moving() &&
	       b.stOut.nServo == IPC_CMD_SERVO_OFF && !b.Has(IPC_CTL_SERVO_ASKED),
	       "a measured angle that is no number: zero torque, a controller fault, the motion ends, the servo is asked off");
	b.adQ[2] = dQ2;
	b.Periods(20);
	Expect(b.stRep.n_ctrl_faults == 1 && b.Has(IPC_CTL_FAULT) && b.stOut.bTorque, "the fault stays latched, once");
	Expect(b.Command(IPC_CMD_SERVO_ON) == IPC_REFUSED_FAULT, "no servo on with a controller fault latched");
	Expect(b.Command(IPC_CMD_CTRL_OFF) == IPC_DONE_OK && !b.Has(IPC_CTL_FAULT) && b.Command(IPC_CMD_CTRL_ON) == IPC_DONE_OK &&
	       b.Command(IPC_CMD_SERVO_ON) == IPC_DONE_OK, "controller off clears it");

	/* --- the second controller on the reports --- */
	printf("%ld periods, %ld goals: a second controller on the reports had the law in %ld and the reference in %ld of "
	       "them, %ld differences; apart in %ld; %ld without a report for it; it failed in %ld (the measured angle that "
	       "was no number)\n", b.nPeriods, b.nGoals, b.nLaw, b.nTraj, b.nDiff, b.nApart, b.nNoPayload, b.nFollowerFault);
	Expect(b.nLaw > 9000 && b.nTraj > 8000 && b.nDiff == 0 && b.nApart == 0 && b.nNoPayload == 0 &&
	       b.nFollowerFault == nFaultBefore + 3,
	       "a second controller fed with the reports gives the torque and the reference of every period, bit for bit");

	if (!strOut.empty())
	{
		shadow_vectors_head_t stHead;
		memset(&stHead, 0, sizeof(stHead));
		stHead.magic = CONTROL_VECTORS_MAGIC;
		stHead.periods = (uint32_t)b.vVectors.size();
		stHead.crc = ShadowNumbersCrc(adNumbers);
		memcpy(stHead.numbers, adNumbers, sizeof(stHead.numbers));
		FILE* const fp = fopen(strOut.c_str(), "wb");
		const bool bWritten = fp != NULL && fwrite(&stHead, sizeof(stHead), 1, fp) == 1 &&
		                      fwrite(b.vVectors.data(), sizeof(control_vector_t), b.vVectors.size(), fp) == b.vVectors.size();
		if (fp != NULL)
			fclose(fp);
		printf("%s: %u periods%s\n", strOut.c_str(), (unsigned)stHead.periods, bWritten ? "" : " NOT written");
		Expect(bWritten, "the periods are written");
	}
	printf("%s: %d checks failed\n", g_nFailed == 0 ? "PASS" : "FAIL", g_nFailed);
	return g_nFailed == 0 ? 0 : 1;
}
