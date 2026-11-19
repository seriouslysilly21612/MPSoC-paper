/*****************************************************************************
*	Name: s2_link_check.cpp
*	Description: The exchange with an R5 that has the control task
*	(--arch S2), on a PC: the APU's side (indy7/src/r5link/R5Link.h: the
*	commands, the goal, the report, the state and the torque of an arm
*	simulated here) against the R5 of the simulation with the control task
*	(indy7/src/sim/SimR5Control.h, which runs shadow/ControlCycle.h, the
*	class the firmware has). One period at a time, on times that the test
*	makes up; no thread, no simulator, no board.
*
*	    bin/s2_link_check [-f FILE]
*
*	  -f FILE     the configuration, found as by indy7_ctrl
*
*	The application in small: an arm that follows its reference with a
*	small error and that it steps when the R5 has answered its last state
*	(the Sim torque slot has the number of that state); commands in a
*	queue, the next written when the report has the number of the last;
*	a second controller (shadow/ShadowControl.h) fed with every report of
*	a period that had a state.
*
*	Checked: the firmware is told by its State; the arm and the control
*	task advance one step per period when the application keeps up, and
*	neither is ahead of the other when it does not: the R5 then has periods
*	without a state, reports none of them, and the application finds the
*	report of every state it wrote, with that state in it; a state that
*	is being written when the R5 reads costs a period and no more; a
*	report and a torque are new once; commands arrive one by one with
*	their numbers and their answers, also in a period without a state; a
*	start that is refused says why; a goal arrives; HOME runs to its end
*	and TRACK brings the tool to its goal across the slots; in every
*	period with a state the second controller gives the torque and the
*	reference of the report, bit for bit; another run of the application
*	takes nothing that answered the run before, neither a report, a torque
*	nor the answer to a command, and takes up the arm from its own first
*	state; what its state says of the drives reaches the control task; a
*	run without an arm leaves the control task without a state, and
*	reporting every period.
*
*	Exit code 0 when no check failed.
*****************************************************************************/
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <unistd.h>

#include "base/Config.h"
#include "ipc/FwCfg.h"
#include "r5link/R5Link.h"
#include "shadow/ControlCycle.h"
#include "shadow/ShadowControl.h"
#include "shadow/ShadowParams.h"
#include "sim/SimR5Control.h"

namespace {

const int N = IPC_CTRL_DOF;
const int64_t PERIOD_NS = 1000000, DELAY_NS = 750000;
const uint32_t CFG_CRC = 0x5EED0002u, DRIVES = 3;
typedef CControllerFullDynamicsRT C;

int g_nFailed = 0;

void Expect(bool abOk, const char* apcWhat)
{
	if (abOk)
		return;
	g_nFailed++;
	printf("  FAILED: %s\n", apcWhat);
}

/* a run of the application: its side of the link, its arm, its queue of commands, its second controller */
struct Apu
{
	CR5Link			cLink;
	CShadowControl	cFollow;
	bool			bArm = true;		// this run simulates an arm
	double			adQ[N], adQd[N];
	uint32_t		uNotReady = 0;		// bit i: joint i cannot be switched on
	bool			bServoOn = false;
	int				nServoIn = 0;		// steps of the arm until its drives are on; 0: no request under way
	uint32_t		uSimStep = 0;		// the last state of the arm written; 0: none yet
	std::deque<ipc_command_t>	vQueue;
	uint32_t		uSent = 0;			// the number of the last command written
	ipc_report_t	stRep;				// the last report taken
	ipc_report_t	stStateRep;			// ... of a period with a state
	bool			bRep = false;
	uint32_t		uStates = 0;		// the periods with a state of the last report with one (its count on the R5)
	long	nReports = 0, nStateReports = 0, nSteps = 0, nLaw = 0, nTraj = 0, nDiff = 0, nApart = 0, nLost = 0, nCommands = 0;
	long	nOtherState = 0;			// reports of a step that had not the state written for it

	void WriteState()
	{
		ipc_sim_state_t stSim;
		memset(&stSim, 0, sizeof(stSim));
		stSim.step = ++uSimStep;
		stSim.flags = bServoOn ? (uint32_t)IPC_SIM_SERVO_ON : 0u;
		stSim.not_ready = uNotReady;
		for (int i = 0; i < N; i++)
		{
			stSim.q[i] = adQ[i];
			stSim.qd[i] = adQd[i];
		}
		cLink.SendSimState(stSim);
	}
	void Queue(uint32_t auCmd, uint32_t auArg = 0, const ipc_command_t* apNumbers = NULL)
	{
		ipc_command_t stC;
		memset(&stC, 0, sizeof(stC));
		if (apNumbers != NULL)
			stC = *apNumbers;
		stC.cmd = auCmd;
		stC.arg = auArg;
		vQueue.push_back(stC);
	}
	/* one cycle: abArmToo false: the task that steps the arm did not run in it */
	void Cycle(RTTIME atRelease, const double* apGoal = NULL, bool abArmToo = true)
	{
		(void)cLink.Receive(atRelease);
		/* the control task: the report */
		ipc_report_t r;
		if (cLink.TakeReport(&r))
		{
			nReports++;
			stRep = r;
			bRep = true;
			if (r.state & IPC_CTL_STATE)
			{
				nStateReports++;
				stStateRep = r;
				const uint32_t uCount = r.in.period - r.n_no_state;		// the periods with a state so far
				if (uStates != 0 && uCount != uStates + 1)
					nLost++;				// the report of a period with a state was not read
				uStates = uCount;
				ipc_shadow_in_t stIn = r.in;
				stIn.period = uCount;
				ipc_shadow_out_t f;
				cFollow.Step(stIn, &f);
				while (cFollow.ServePlanner())
					;
				if (f.flags & IPC_SHADOW_OUT_APART)
					nApart++;
				if (f.flags & IPC_SHADOW_OUT_CTRL)
				{
					nLaw++;
					if (f.tau_diff != 0.0 || memcmp(f.tau, r.in.tau, sizeof(f.tau)) != 0)
						nDiff++;
				}
				if (f.flags & IPC_SHADOW_OUT_TRAJ)
				{
					nTraj++;
					if (memcmp(f.q_ref, r.q_ref, sizeof(f.q_ref)) != 0)
						nDiff++;
				}
			}
		}
		/* the EtherCAT task: the arm, one step per answer of the R5 to its last state */
		ipc_sim_torque_t t;
		if (bArm && abArmToo)
		{
			if (uSimStep == 0)
				WriteState();
			else if (cLink.TakeSimTorque(&t) && t.step == uSimStep)
			{
				// the report of the period that answered has been read, and that period had the state written last
				if (nStateReports == 0 || stStateRep.sim_step != t.step)
					nLost++;
				else if (memcmp(stStateRep.in.q, adQ, sizeof(adQ)) != 0 || memcmp(stStateRep.in.qd, adQd, sizeof(adQd)) != 0)
					nOtherState++;
				if (t.flags & IPC_SIM_SERVO_ASKED)
				{
					if (!bServoOn && nServoIn == 0)
						nServoIn = 3;
				}
				else
				{
					bServoOn = false;
					nServoIn = 0;
				}
				if (nServoIn > 0 && --nServoIn == 0)
					bServoOn = true;
				const bool bFollowed = (stStateRep.in.flags & IPC_SHADOW_REF) != 0;
				for (int i = 0; i < N; i++)
				{
					if (bFollowed)
						adQ[i] = stStateRep.q_ref[i] + 2e-5 * std::sin(0.01 * (double)uSimStep + (double)i);
					adQd[i] = bFollowed ? stStateRep.qd_ref[i] : 0.0;
				}
				nSteps++;
				WriteState();
			}
		}
		/* the control task: the next command, when the R5 has taken the last */
		if (!vQueue.empty() && (uSent == 0 || (bRep && stRep.cmd_done == uSent)))
		{
			uSent = cLink.SendControlCommand(vQueue.front());
			vQueue.pop_front();
			nCommands++;
		}
		if (apGoal != NULL)
			cLink.SendGoal(apGoal);
	}
};

/* the two sides on one region */
struct Bench
{
	alignas(64) volatile uint32_t auRegion[IPC_OFF_END2 / 4];
	CControlCycle	cControl;		// of the R5
	CSimR5Control	cR5;
	link_clock::CTickClock cClock;
	RTTIME			tGrid;
	uint32_t		uPeriod = 0;

	Bench()
	{
		for (size_t i = 0; i < sizeof(auRegion) / sizeof(auRegion[0]); i++)
			auRegion[i] = 0u;
		for (int nTry = 0; nTry < 1000 && !cClock.Sample(); nTry++)
			;
		tGrid = read_timer() / (RTTIME)PERIOD_NS * (RTTIME)PERIOD_NS - 600000 * (RTTIME)PERIOD_NS;
	}
	RTTIME ReleaseOf(uint32_t auPeriod) const { return tGrid + (RTTIME)auPeriod * (RTTIME)PERIOD_NS; }
	bool StartApu(Apu& a)
	{
		a.cLink.UseRegion(auRegion, "the test");
		return a.cLink.Start(DRIVES, CFG_CRC, (RTTIME)PERIOD_NS, (RTTIME)DELAY_NS) ? true : false;
	}
	void R5Period()
	{
		uPeriod++;
		cR5.Cycle(cClock.TicksOf(ReleaseOf(uPeriod)), uPeriod, 0);
	}
	/* a period of the R5, then the cycle of the application behind it */
	void Period(Apu& a, const double* apGoal = NULL)
	{
		R5Period();
		a.Cycle(ReleaseOf(uPeriod) + (RTTIME)DELAY_NS, apGoal);
	}
	void Periods(Apu& a, int anCount)
	{
		for (int k = 0; k < anCount; k++)
			Period(a);
	}
	/* periods until the queue of the application is empty and its last command answered; false: not in anMost */
	bool UntilAnswered(Apu& a, int anMost = 20)
	{
		for (int k = 0; k < anMost; k++)
		{
			Period(a);
			if (a.vQueue.empty() && a.bRep && a.stRep.cmd_done == a.uSent)
				return true;
		}
		return false;
	}
};

} // namespace

int main(int argc, char** argv)
{
	TSTRING strCfg;
	int nOpt;
	while ((nOpt = getopt(argc, argv, "f:")) != -1)
	{
		if (nOpt == 'f') strCfg = optarg;
		else
		{
			fprintf(stderr, "usage: %s [-f FILE]\n", argv[0]);
			return 2;
		}
	}
	CConfig cConfig;
	strCfg = CConfig::Find(strCfg, argv[0]);
	if (FALSE == cConfig.Load(strCfg, FALSE))
		return 2;
	const char* const pcURDF = cConfig.System().strURDFPath.c_str();
	double adNumbers[SHADOW_NUMBERS];
	static Bench b;
	static Apu a;
	CHighLevelRT cHl;		// for the tool position of a posture
	if (!ShadowNumbers(cConfig, adNumbers) || !b.cControl.Init(pcURDF, adNumbers, 0xC0DE0002u) ||
	    !a.cFollow.Init(pcURDF, adNumbers, 0xC0DE0002u) ||
	    !cHl.Init(pcURDF, adNumbers[SHADOW_N_TS], CShadowControl::HlParams(adNumbers)))
	{
		fprintf(stderr, "%s: the controller cannot be built from it\n", strCfg.c_str());
		return 2;
	}
	const CHighLevelRT::Params stP = CShadowControl::HlParams(adNumbers);
	const double* const adQr = &adNumbers[SHADOW_N_READY];
	ipc_command_t stStart;
	memset(&stStart, 0, sizeof(stStart));
	stStart.delta = stP.dDelta;
	for (int i = 0; i < N; i++)
	{
		stStart.q_ready[i] = adQr[i];
		a.adQ[i] = adQr[i] + 0.2 * ((i % 2) ? 1.0 : -1.0);
		a.adQd[i] = 0.0;
	}
	double adP0[3], adGoal[3];
	cHl.ToolPosition(adQr, adP0);
	for (int k = 0; k < 3; k++)
		adGoal[k] = adP0[k] + (k == 0 ? 0.05 : k == 1 ? -0.04 : 0.03);

	/* --- the two sides come up; the firmware is told by its State --- */
	(void)b.cR5.Start(&b.cControl, b.auRegion, CFG_CRC, DRIVES);
	Expect(b.StartApu(a) && !a.cLink.HasControlTask(), "the application's side starts; nothing is known of the R5 before its first State");
	b.Period(a);
	Expect(a.cLink.HasControlTask() && a.cLink.State().period == 1 && a.cLink.State().dof == DRIVES &&
	       !(a.cLink.State().flags & IPC_STATE_SERVO_ON) && a.uSimStep == 1 && a.nReports == 1 && a.nStateReports == 0 &&
	       !(a.stRep.state & IPC_CTL_SIM_ARM) && b.cR5.NoState() == 1,
	       "the State of the first period says that this R5 has the control task; that period had no arm yet, and its report says so");

	/* --- one step of the arm per period --- */
	b.Periods(a, 50);
	Expect(a.nSteps == 50 && a.uSimStep == 51 && a.nStateReports == 50 && a.nReports == 51 && b.cR5.NoState() == 1 &&
	       a.stRep.sim_step == 50 && a.stRep.in.period == 51 && (a.stRep.state & IPC_CTL_SIM_ARM) && a.nLost == 0,
	       "with the application behind every period: a step of the arm and a period of the control task in each");
	Expect(!(a.stRep.state & (IPC_CTL_ENABLED | IPC_CTL_SERVO_ASKED | IPC_CTL_SERVO_ON)),
	       "the controller is off and the drives of the arm are off, as its state says");
	{
		ipc_report_t r;
		ipc_sim_torque_t t;
		Expect(!a.cLink.TakeReport(&r) && !a.cLink.TakeSimTorque(&t), "a report and a torque are new once");
	}
	/* the R5 reads the state of the arm while the application writes it */
	{
		const long nReports = a.nReports, nSteps = a.nSteps, nNoState = b.cR5.NoState();
		volatile uint32_t* const pGen = &b.auRegion[IPC_OFF_SIM_STATE / 4 + IPC_SLOT_GEN];
		*pGen = *pGen | 1u;					// the write began
		b.Period(a);
		Expect(b.cR5.NoState() == nNoState + 1 && a.nReports == nReports && a.nSteps == nSteps,
		       "a state that is being written: a period without a state, of the arm that is still the simulated one (no report)");
		*pGen = *pGen + 1u;					// ... and ends
		b.Period(a);
		Expect(a.nSteps == nSteps + 1 && a.nStateReports == nSteps + 1 && a.nLost == 0, "the period after it has that state");
	}

	/* --- commands, one by one --- */
	a.Queue(IPC_CMD_START, IPC_START_HOME, &stStart);		// refused: the controller is off
	a.Queue(IPC_CMD_CTRL_ON);
	a.Queue(IPC_CMD_HOLD_OFF);
	a.Queue(IPC_CMD_HOLD_ON);
	a.Queue(IPC_CMD_SERVO_ON);
	b.Period(a);
	const uint32_t uFirst = a.uSent;
	Expect(uFirst != 0 && a.stRep.cmd_done == 0 && a.vQueue.size() == 4, "the first command of the queue is written");
	b.Period(a);
	Expect(a.stRep.cmd_done == uFirst && a.stRep.cmd_result == IPC_REFUSED_NOT_ENABLED && a.stRep.start_seq == uFirst &&
	       a.stRep.start_result == IPC_REFUSED_NOT_ENABLED && a.uSent == uFirst + 1,
	       "the report of the next period has its number and why it was refused; the second is written then");
	Expect(b.UntilAnswered(a) && a.uSent == uFirst + 4 && a.stRep.cmd_result == IPC_DONE_OK && a.nCommands == 5 &&
	       (a.stRep.state & (IPC_CTL_ENABLED | IPC_CTL_SERVO_ASKED)) == (IPC_CTL_ENABLED | IPC_CTL_SERVO_ASKED) &&
	       (a.stRep.in.flags & IPC_SHADOW_HOLD),
	       "five commands, each answered before the next: the controller is on and the servo asked for");
	b.Periods(a, 5);
	Expect(a.bServoOn && (a.stRep.state & IPC_CTL_SERVO_ON) && a.stRep.n_drive_stops == 0 && a.cLink.State().cmd_done == a.uSent,
	       "the drives of the arm come on by the request in the Sim torque slot, and the control task sees them on");

	/* --- the application does not keep up --- */
	const long nNoState = b.cR5.NoState(), nReports = a.nReports, nSteps = a.nSteps;
	const uint32_t uPeriodBefore = a.stRep.in.period;
	b.R5Period();
	b.R5Period();
	b.R5Period();			// three periods of the R5 without a cycle of the application
	Expect(b.cR5.NoState() == nNoState + 2 && b.cR5.LastReport().n_no_state == (uint32_t)(nNoState + 2),
	       "the first of them still has the state written before; the two after it have none");
	a.Cycle(b.ReleaseOf(b.uPeriod) + (RTTIME)DELAY_NS);
	Expect(a.nReports == nReports + 1 && a.stRep.in.period == uPeriodBefore + 1 && (a.stRep.state & IPC_CTL_STATE) &&
	       a.nSteps == nSteps + 1 && a.nLost == 0,
	       "the application then finds the report of the period that had its state (the periods without one reported nothing), and steps once");
	b.Periods(a, 20);
	Expect(a.nSteps == nSteps + 21 && a.nLost == 0 && a.nApart == 0, "and the two go on step by step");
	/* a command while the arm is not stepped: answered in a period without a state, by a report of that period */
	a.Queue(IPC_CMD_HOLD_OFF);
	b.R5Period();
	a.Cycle(b.ReleaseOf(b.uPeriod) + (RTTIME)DELAY_NS, NULL, false);		// the command leaves; the arm is not stepped
	const long nStateReports = a.nStateReports, nStepsBefore = a.nSteps;
	b.R5Period();															// no new state: the command alone
	a.Cycle(b.ReleaseOf(b.uPeriod) + (RTTIME)DELAY_NS, NULL, false);
	Expect(a.stRep.cmd_done == a.uSent && a.stRep.cmd_result == IPC_DONE_OK && !(a.stRep.state & IPC_CTL_STATE) &&
	       a.nStateReports == nStateReports && !(a.stRep.in.flags & IPC_SHADOW_HOLD) && a.nSteps == nStepsBefore,
	       "a command is answered in a period without a state of the arm too, by a report of that period");
	a.Queue(IPC_CMD_HOLD_ON);
	b.Periods(a, 20);
	Expect(a.nLost == 0 && a.nApart == 0 && a.nSteps == nStepsBefore + 20 && (a.stRep.in.flags & IPC_SHADOW_HOLD),
	       "the arm is stepped again from the answer that waited: nothing was lost");

	/* --- HOME to its end, TRACK to a goal, across the slots --- */
	a.Queue(IPC_CMD_START, IPC_START_HOME | IPC_START_FRICTION_FF, &stStart);
	Expect(b.UntilAnswered(a) && a.stRep.start_result == IPC_DONE_OK && a.stRep.traj_state == (uint32_t)CTrajectoryRT::eRunning &&
	       a.stRep.motion == (uint32_t)CTrajectoryRT::eMotionHome, "HOME starts by a command");
	int nHome = 0;
	while (nHome < 20000 && a.stRep.traj_state == (uint32_t)CTrajectoryRT::eRunning)
	{
		b.Period(a);
		nHome++;
	}
	double dOff = 0.0;
	for (int i = 0; i < N; i++)
		dOff = std::fmax(dOff, std::fabs(a.adQ[i] - adQr[i]));
	Expect(nHome > 200 && nHome < 20000 && a.stRep.traj_state == (uint32_t)CTrajectoryRT::eDoneAtRest && dOff < 1e-3,
	       "... and ends at rest at the ready posture");
	a.Queue(IPC_CMD_START, 0, &stStart);
	Expect(b.UntilAnswered(a) && a.stRep.start_result == IPC_DONE_OK && a.stRep.motion == (uint32_t)CTrajectoryRT::eMotionTrack, "TRACK starts");
	b.Period(a, adGoal);
	b.Periods(a, 5);
	Expect(a.stRep.n_goals == 1 && std::fabs(a.stRep.in.goal[0] - adGoal[0]) == 0.0 && (a.stRep.in.flags & IPC_SHADOW_GOAL),
	       "a goal arrives through the Goal slot");
	b.Periods(a, 5000);
	double adTool[3], dFar = 0.0;
	cHl.ToolPosition(a.adQ, adTool);
	for (int k = 0; k < 3; k++)
		dFar = std::fmax(dFar, std::fabs(adTool[k] - adGoal[k]));
	Expect(dFar < 2e-3 && a.stRep.n_plan_late == 0 && a.stRep.n_iks > 0 && a.stRep.n_plans > 1, "TRACK brings the tool of the arm to it");
	a.Queue(IPC_CMD_MOTION_STOP);
	Expect(b.UntilAnswered(a) && a.stRep.traj_state == (uint32_t)CTrajectoryRT::eAborted, "the stop");
	b.Periods(a, 200);

	printf("%u periods of the R5, %ld steps of the arm, %ld reports (%ld of periods with a state), %ld commands: a second "
	       "controller on the reports had the law in %ld and the reference in %ld of them, %ld differences, apart in "
	       "%ld; %ld reports of a period with a state not read, %ld with another state than the one written\n",
	       (unsigned)b.uPeriod, a.nSteps, a.nReports, a.nStateReports, a.nCommands, a.nLaw, a.nTraj, a.nDiff, a.nApart,
	       a.nLost, a.nOtherState);
	Expect(a.nLaw > 5000 && a.nTraj > 5000 && a.nDiff == 0 && a.nApart == 0 && a.nLost == 0 && a.nOtherState == 0,
	       "the second controller on the reports gives the torque and the reference of every period with a state, bit for bit");

	/* --- another run of the application --- */
	static Apu a2, a3, a4;
	{
		memcpy(a2.adQ, a.adQ, sizeof(a.adQ));
		memset(a2.adQd, 0, sizeof(a2.adQd));
		a2.bServoOn = a.bServoOn;
		const uint32_t uStepOfFirst = a.uSimStep;
		Expect(b.StartApu(a2), "a second run of the application starts on the region");
		bool bBeingWritten = true;
		const unsigned auSlots[5] = { IPC_OFF_TORQUE, IPC_OFF_COMMAND, IPC_OFF_SHADOW_IN, IPC_OFF_GOAL, IPC_OFF_SIM_STATE };
		for (int k = 0; k < 5; k++)
			if (!(b.auRegion[auSlots[k] / 4 + IPC_SLOT_GEN] & 1u))
				bBeingWritten = false;
		ipc_report_t r;
		ipc_sim_torque_t t;
		Expect(bBeingWritten && !a2.cLink.TakeReport(&r) && !a2.cLink.TakeSimTorque(&t),
		       "what its five slots held is not readable any more, and the report and the torque that answered the run before are not new to it");
		b.Period(a2);			// its first cycle writes state 1
		Expect(a2.uSimStep == 1 && a2.nSteps == 0 && a2.nReports == 1 && !(a2.stRep.state & IPC_CTL_STATE),
		       "it writes its first state; the R5 had a period without an arm between the two runs");
		b.Periods(a2, 10);
		Expect(a2.nSteps == 10 && a2.stRep.sim_step == 10 && uStepOfFirst > 100 && (a2.stRep.state & IPC_CTL_STATE) &&
		       (a2.stRep.state & IPC_CTL_ENABLED) && a2.nLost == 0 && a2.nOtherState == 0,
		       "the control task, which went on, takes up its arm from that state");
		/* what else the state of the arm says: joints that cannot be switched on. (Its measured torque goes to the
		 * law as on the APU, and the law does not use it: nothing to see.) */
		a2.Queue(IPC_CMD_SERVO_OFF);
		Expect(b.UntilAnswered(a2) && a2.stRep.cmd_result == IPC_DONE_OK, "servo off");
		b.Periods(a2, 3);
		a2.uNotReady = 1u << 2;
		b.Periods(a2, 2);
		a2.Queue(IPC_CMD_SERVO_ON);
		Expect(!a2.bServoOn && b.UntilAnswered(a2) && a2.stRep.cmd_result == IPC_REFUSED_NOT_READY,
		       "a servo on while the state of the arm says that a joint is not ready is refused for that");
		a2.uNotReady = 0;
		b.Periods(a2, 2);
		a2.Queue(IPC_CMD_SERVO_ON);
		Expect(b.UntilAnswered(a2) && a2.stRep.cmd_result == IPC_DONE_OK, "... and taken when it does not");
		b.Periods(a2, 5);
		Expect(a2.bServoOn && (a2.stRep.state & IPC_CTL_SERVO_ON) && a2.stRep.n_drive_stops == 0, "the drives are on again");
	}
	/* --- a run without an arm: the control task has no state --- */
	{
		a3.bArm = false;
		Expect(b.StartApu(a3), "a third run starts, without an arm");
		a3.Queue(IPC_CMD_START, IPC_START_HOME, &stStart);
		Expect(b.UntilAnswered(a3) && a3.stRep.cmd_result == IPC_REFUSED_NO_STATE && !(a3.stRep.state & (IPC_CTL_STATE | IPC_CTL_SIM_ARM)) &&
		       a3.nStateReports == 0 && a3.nCommands == 1,
		       "with it the control task of this R5 has no state: a start is refused for that");
		const long nReports = a3.nReports;
		b.Periods(a3, 5);
		Expect(a3.nReports == nReports + 5, "the arm is no longer the simulated one: every period reports");
	}
	/* --- and one more: the report still has the answer to the one command of the run before --- */
	{
		a4.bArm = false;
		Expect(b.StartApu(a4), "a fourth run starts");
		a4.Queue(IPC_CMD_HOLD_OFF);
		b.Period(a4);			// its first cycle: a report that is new to it, and its first command leaves
		Expect(a4.nCommands == 1 && a4.bRep && a4.stRep.cmd_done == a3.uSent && a4.stRep.cmd_done != a4.uSent,
		       "the first command of a run has not the number of the first command of the run before, which the report still has");
		Expect(b.UntilAnswered(a4) && a4.stRep.cmd_result == IPC_DONE_OK && !(a4.stRep.in.flags & IPC_SHADOW_HOLD), "it is answered in its turn");
	}

	printf("%s: %d checks failed\n", g_nFailed == 0 ? "PASS" : "FAIL", g_nFailed);
	return g_nFailed == 0 ? 0 : 1;
}
