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
*	their numbers and their answers, one taken in a period without a
*	state is answered with the next state; a start that is refused says
*	why; a goal arrives; HOME runs to its end
*	and TRACK brings the tool to its goal across the slots; in every
*	period with a state the second controller gives the torque and the
*	reference of the report, bit for bit; another run of the application
*	takes nothing that answered the run before, neither a report, a torque
*	nor the answer to a command, and takes up the arm from its own first
*	state; what its state says of the drives reaches the control task; a
*	run without an arm leaves the control task without a state, and
*	reporting every period; and with the R5 in a thread of its own, the
*	report of a period is there whenever the torque of its state is.
*	Of the client (r5link/ControlClient.h) also: a report that is not the
*	answer to the command written last is not taken for it; the goal of a
*	TRACK whose start still waits behind another command waits with it; the
*	controller faults and the stops of the drives that the R5 counts are
*	counted per run; reports that did not arrive are counted from the
*	numbers of those that did. Of the second controller on the reports
*	(r5link/ControlFollower.h, --shadow with S2): the bounds beyond which a
*	torque or a reference counts as different; apart from the R5's while
*	the run has had no start, and after a report that did not arrive, until
*	the next start.
*
*	Exit code 0 when no check failed.
*****************************************************************************/
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <thread>
#include <unistd.h>

#include "base/Config.h"
#include "ipc/FwCfg.h"
#include "GoalSlot.h"
#include "r5link/ControlClient.h"
#include "r5link/ControlFollower.h"
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

/* a run of the application: its side of the link, what its control task keeps of the control task on the R5
 * (r5link/ControlClient.h: the commands, the report, the goals), its arm, and a second controller */
struct Apu
{
	CR5Link			cLink;
	CControlClient	cClient;
	CControlFollower cFollow;			// the control of the R5 a second time, on the reports (--shadow with S2)
	CControlFollower cFollowOff;		// the same on reports whose torque and reference are off by dTauOff, dRefOff
	double			dTauOff = 0.0, dRefOff = 0.0;
	CGoalSlot		cGoal;				// the slot that TRACK reads
	bool			bArm = true;		// this run simulates an arm
	double			adQ[N], adQd[N];
	double			adQSent[N], adQdSent[N];	// as the state written last has them
	int				nNoNumber = -1;		// the joint whose angle is no number in the next state written (once)
	uint32_t		uNotReady = 0;		// bit i: joint i cannot be switched on
	bool			bServoOn = false;
	int				nServoIn = 0;		// steps of the arm until its drives are on; 0: no request under way
	uint32_t		uSimStep = 0;		// the last state of the arm written; 0: none yet
	ipc_report_t	stRep;				// the last report taken
	ipc_report_t	stStateRep;			// ... of a period with a state
	bool			bStateRep = false;
	uint32_t		uLastCmd = 0, uLastResult = 0;		// the answer told last
	long	nAnswers = 0, nSteps = 0, nLost = 0;
	long	nOtherState = 0;			// reports of a step that had not the state written for it

	long Reports() const { return cClient.GetCounts().nReports; }
	long StateReports() const { return cClient.GetCounts().nStateReports; }
	long Commands() const { return cClient.GetCounts().nCommands; }
	const CControlFollower::Counts& F() const { return cFollow.GetCounts(); }
	/* every torque and every reference that the second controller compared were the bits of the report */
	bool SameBits() const { return F().nTauDiff == 0 && F().nRefDiff == 0 && F().dTauDiffMax == 0.0 && F().dRefDiffMax == 0.0; }
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
		if (nNoNumber >= 0)
			stSim.q[nNoNumber] = std::numeric_limits<double>::quiet_NaN();
		nNoNumber = -1;
		memcpy(adQSent, stSim.q, sizeof(adQSent));
		memcpy(adQdSent, stSim.qd, sizeof(adQdSent));
		cLink.SendSimState(stSim);
	}
	/* one cycle: abArmToo false: the task that steps the arm did not run in it */
	void Cycle(RTTIME atRelease, bool abArmToo = true)
	{
		(void)cLink.Receive(atRelease);
		/* the EtherCAT task: the torque that answers its last state; with it the report of that period is there */
		ipc_sim_torque_t t;
		const bool bAnswered = bArm && abArmToo && uSimStep != 0 && cLink.TakeSimTorque(&t) && t.step == uSimStep;
		ipc_report_t r;
		if (cLink.TakeReport(&r))
		{
			/* the control task: what the report says */
			cClient.Take(r);
			stRep = r;
			if (cClient.TakeAnswer(&uLastCmd, &uLastResult))
				nAnswers++;
			if (r.state & IPC_CTL_STATE)
			{
				stStateRep = r;
				bStateRep = true;
			}
			cFollow.Take(r, cClient.StatePeriod());
			static ipc_report_t s_stOff;
			s_stOff = r;
			s_stOff.in.tau[2] += dTauOff;
			s_stOff.q_ref[4] += dRefOff;
			cFollowOff.Take(s_stOff, cClient.StatePeriod());
		}
		/* ... and the arm, one step per answer */
		bool bState = false;
		if (bArm && abArmToo && uSimStep == 0)
			bState = true;
		else if (bAnswered)
		{
			// the report taken last is that of the period that answered, and that period had the state written last
			if (!bStateRep || stStateRep.sim_step != t.step)
				nLost++;
			else if (memcmp(stStateRep.in.q, adQSent, sizeof(adQSent)) != 0 || memcmp(stStateRep.in.qd, adQdSent, sizeof(adQdSent)) != 0)
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
			bState = true;
		}
		/* the control task: the next command when the R5 has answered the last, then a new goal of its slot, then
		 * the new state of the arm: the order in which the application writes the three */
		cClient.Send();
		cClient.ForwardGoal();
		if (bState)
			WriteState();
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
		const double adNone[3] = { 0.0, 0.0, 0.0 };
		a.cLink.UseRegion(auRegion, "the test");
		a.cClient.Init(&a.cLink, 1e-3);
		a.cClient.UseGoalSlot(&a.cGoal, adNone);
		return a.cLink.Start(DRIVES, CFG_CRC, (RTTIME)PERIOD_NS, (RTTIME)DELAY_NS) ? true : false;
	}
	void R5Period()
	{
		uPeriod++;
		cR5.Cycle(cClock.TicksOf(ReleaseOf(uPeriod)), uPeriod, 0);
	}
	/* a period of the R5, then the cycle of the application behind it */
	void Period(Apu& a, bool abArmToo = true)
	{
		R5Period();
		a.Cycle(ReleaseOf(uPeriod) + (RTTIME)DELAY_NS, abArmToo);
	}
	void Periods(Apu& a, int anCount)
	{
		for (int k = 0; k < anCount; k++)
			Period(a);
	}
	/* periods until the R5 has answered every command of the application; false: not in anMost */
	bool UntilAnswered(Apu& a, int anMost = 20)
	{
		for (int k = 0; k < anMost; k++)
		{
			Period(a);
			if (a.cClient.IsSettled())
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
	    !a.cFollow.Init(pcURDF, adNumbers, 0xC0DE0002u) || !a.cFollowOff.Init(pcURDF, adNumbers, 0xC0DE0002u) ||
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
	       !(a.cLink.State().flags & IPC_STATE_SERVO_ON) && a.uSimStep == 1 && a.Reports() == 1 && a.StateReports() == 0 &&
	       !(a.stRep.state & IPC_CTL_SIM_ARM) && b.cR5.NoState() == 1 && a.cClient.HasReport() && a.cClient.Sample() == NULL,
	       "the State of the first period says that this R5 has the control task; that period had no arm yet, and its report says so");

	/* --- one step of the arm per period --- */
	b.Periods(a, 50);
	Expect(a.nSteps == 50 && a.uSimStep == 51 && a.StateReports() == 50 && a.Reports() == 51 && b.cR5.NoState() == 1 &&
	       a.stRep.sim_step == 50 && a.stRep.in.period == 51 && (a.stRep.state & IPC_CTL_SIM_ARM) && a.nLost == 0 &&
	       a.cClient.GetCounts().nStateMissed == 0,
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
		const long nReports = a.Reports(), nSteps = a.nSteps, nNoState = b.cR5.NoState();
		volatile uint32_t* const pGen = &b.auRegion[IPC_OFF_SIM_STATE / 4 + IPC_SLOT_GEN];
		*pGen = *pGen | 1u;					// the write began
		b.Period(a);
		Expect(b.cR5.NoState() == nNoState + 1 && a.Reports() == nReports && a.nSteps == nSteps,
		       "a state that is being written: a period without a state, of the arm that is still the simulated one (no report)");
		*pGen = *pGen + 1u;					// ... and ends
		b.Period(a);
		Expect(a.nSteps == nSteps + 1 && a.StateReports() == nSteps + 1 && a.nLost == 0, "the period after it has that state");
	}

	/* --- commands, one by one --- */
	a.cClient.Queue(IPC_CMD_START, IPC_START_HOME, &stStart);		// refused: the controller is off
	a.cClient.Queue(IPC_CMD_CTRL_ON);
	a.cClient.Queue(IPC_CMD_HOLD_OFF);
	a.cClient.Queue(IPC_CMD_HOLD_ON);
	a.cClient.Queue(IPC_CMD_SERVO_ON);
	Expect(!a.cClient.IsSettled(), "commands that wait: not settled");
	b.Period(a);
	Expect(a.Commands() == 1 && a.stRep.cmd_done == 0 && a.nAnswers == 0 && !a.cClient.IsSettled(),
	       "the first command of the queue is written");
	b.Period(a);
	const uint32_t uFirst = a.stRep.cmd_done;
	char acWhy[192];
	a.cClient.ResultText(a.uLastResult, acWhy, sizeof(acWhy));
	Expect(uFirst != 0 && a.stRep.cmd_result == IPC_REFUSED_NOT_ENABLED && a.stRep.start_seq == uFirst &&
	       a.stRep.start_result == IPC_REFUSED_NOT_ENABLED && a.Commands() == 2 && a.nAnswers == 1 &&
	       a.uLastCmd == IPC_CMD_START && a.uLastResult == IPC_REFUSED_NOT_ENABLED && a.cClient.GetCounts().nRefused == 1 &&
	       strstr(acWhy, "controller not enabled") != NULL,
	       "the report of the next period has its number and why it was refused, told once; the second is written then");
	Expect(b.UntilAnswered(a) && a.stRep.cmd_done == uFirst + 4 && a.stRep.cmd_result == IPC_DONE_OK && a.Commands() == 5 &&
	       a.nAnswers == 5 && a.uLastCmd == IPC_CMD_SERVO_ON && a.uLastResult == IPC_DONE_OK &&
	       a.cClient.GetCounts().nRefused == 1 &&
	       (a.stRep.state & (IPC_CTL_ENABLED | IPC_CTL_SERVO_ASKED)) == (IPC_CTL_ENABLED | IPC_CTL_SERVO_ASKED) &&
	       (a.stRep.in.flags & IPC_SHADOW_HOLD),
	       "five commands, each answered before the next, with numbers that go up by one: the controller is on and the servo asked for");
	b.Periods(a, 5);
	Expect(a.bServoOn && (a.stRep.state & IPC_CTL_SERVO_ON) && a.stRep.n_drive_stops == 0 &&
	       a.cLink.State().cmd_done == a.stRep.cmd_done && a.nAnswers == 5,
	       "the drives of the arm come on by the request in the Sim torque slot, and the control task sees them on");

	/* --- the application does not keep up --- */
	const long nNoState = b.cR5.NoState(), nReports = a.Reports(), nSteps = a.nSteps;
	const uint32_t uPeriodBefore = a.stRep.in.period;
	b.R5Period();
	b.R5Period();
	b.R5Period();			// three periods of the R5 without a cycle of the application
	Expect(b.cR5.NoState() == nNoState + 2 && b.cR5.LastReport().n_no_state == (uint32_t)(nNoState + 2),
	       "the first of them still has the state written before; the two after it have none");
	a.Cycle(b.ReleaseOf(b.uPeriod) + (RTTIME)DELAY_NS);
	Expect(a.Reports() == nReports + 1 && a.stRep.in.period == uPeriodBefore + 1 && (a.stRep.state & IPC_CTL_STATE) &&
	       a.nSteps == nSteps + 1 && a.nLost == 0,
	       "the application then finds the report of the period that had its state (the periods without one reported nothing), and steps once");
	b.Periods(a, 20);
	Expect(a.nSteps == nSteps + 21 && a.nLost == 0 && a.F().nApart == 0 && a.cClient.GetCounts().nStateMissed == 0,
	       "and the two go on step by step");
	/* a command while the arm is not stepped: taken in a period without a state, which reports nothing; the report
	 * of the next period with a state has the answer */
	a.cClient.Queue(IPC_CMD_HOLD_OFF);
	b.Period(a, false);			// the command leaves; the arm is not stepped
	const long nReportsBefore = a.Reports(), nStepsBefore = a.nSteps, nCommandsBefore = a.Commands();
	const uint32_t uDoneBefore = a.stRep.cmd_done;
	b.Period(a, false);			// no new state: the command alone
	Expect(a.Reports() == nReportsBefore && !a.cClient.IsSettled() && a.nSteps == nStepsBefore &&
	       b.cR5.LastReport().cmd_done == uDoneBefore + 1 && !(b.cR5.LastReport().in.flags & IPC_SHADOW_HOLD),
	       "a command in a period without a state of the arm is carried out, and that period reports nothing");
	a.cClient.Queue(IPC_CMD_HOLD_ON);			// waits for the answer to the one before it
	b.Period(a);
	Expect(a.nSteps == nStepsBefore + 1 && a.Reports() == nReportsBefore && a.Commands() == nCommandsBefore,
	       "the arm is stepped again from the torque that waited");
	b.Period(a);
	Expect(a.Reports() == nReportsBefore + 1 && a.stRep.cmd_done == uDoneBefore + 1 && a.stRep.cmd_result == IPC_DONE_OK &&
	       (a.stRep.state & IPC_CTL_STATE) && a.nSteps == nStepsBefore + 2 && a.Commands() == nCommandsBefore + 1,
	       "the report of the next period with a state has the answer, and the next command leaves");
	b.Periods(a, 18);
	Expect(a.nLost == 0 && a.F().nApart == 0 && a.nSteps == nStepsBefore + 20 && (a.stRep.in.flags & IPC_SHADOW_HOLD) &&
	       a.cClient.IsSettled(), "nothing was lost");
	/* more commands than the queue has room for */
	{
		const long nBefore = a.Commands();
		bool bEight = true;
		for (int k = 0; k < (int)CControlClient::QUEUE; k++)
			bEight = a.cClient.Queue((k % 2) ? IPC_CMD_HOLD_ON : IPC_CMD_HOLD_OFF) && bEight;
		Expect(bEight && !a.cClient.Queue(IPC_CMD_CTRL_OFF) && a.cClient.GetCounts().nQueueFull == 1,
		       "the queue takes eight commands; the ninth is refused here and counted");
		Expect(b.UntilAnswered(a, 40) && a.Commands() == nBefore + (long)CControlClient::QUEUE && (a.stRep.state & IPC_CTL_ENABLED) &&
		       (a.stRep.in.flags & IPC_SHADOW_HOLD), "the eight are carried out in their order; the ninth never left");
	}

	/* the R5 reads the Command slot while the application writes it: that period goes on without the command, and
	 * its report is not the answer */
	{
		a.cClient.Queue(IPC_CMD_HOLD_OFF);
		b.Period(a);				// the command is written at the end of this cycle ...
		const long nAnswersBefore = a.nAnswers, nReportsHere = a.Reports();
		volatile uint32_t* const pGen = &b.auRegion[IPC_OFF_COMMAND / 4 + IPC_SLOT_GEN];
		*pGen = *pGen | 1u;			// ... and the write has not ended when the R5 reads
		b.Period(a);
		Expect(a.Reports() == nReportsHere + 1 && a.nAnswers == nAnswersBefore && !a.cClient.IsSettled() &&
		       (a.stRep.in.flags & IPC_SHADOW_HOLD),
		       "a command that is being written: the report of that period, which did not have it, is not its answer");
		*pGen = *pGen + 1u;
		Expect(b.UntilAnswered(a) && a.nAnswers == nAnswersBefore + 1 && a.uLastCmd == IPC_CMD_HOLD_OFF &&
		       a.uLastResult == IPC_DONE_OK && !(a.stRep.in.flags & IPC_SHADOW_HOLD),
		       "the answer is the report of the period that had it");
		a.cClient.Queue(IPC_CMD_HOLD_ON);
		Expect(b.UntilAnswered(a) && (a.stRep.in.flags & IPC_SHADOW_HOLD), "the hold is on again");
	}

	/* --- HOME to its end, TRACK to a goal, across the slots --- */
	a.cClient.Queue(IPC_CMD_START, IPC_START_HOME | IPC_START_FRICTION_FF, &stStart);
	Expect(b.UntilAnswered(a) && a.stRep.start_result == IPC_DONE_OK && a.stRep.traj_state == (uint32_t)CTrajectoryRT::eRunning &&
	       a.stRep.motion == (uint32_t)CTrajectoryRT::eMotionHome && a.uLastCmd == IPC_CMD_START && a.uLastResult == IPC_DONE_OK,
	       "HOME starts by a command");
	int nHome = 0, nMoving = 0;
	bool bHomeSamples = true, bHomeRef = true;
	while (nHome < 20000 && a.stRep.traj_state == (uint32_t)CTrajectoryRT::eRunning)
	{
		b.Period(a);
		nHome++;
		const CTrajectoryRT::Sample* const p = a.cClient.Sample();
		if (p == NULL || p->nMode != 1)
		{
			bHomeSamples = false;
			continue;
		}
		const size_t uJ = sizeof(double) * N;
		if (memcmp(p->adQRef, a.stRep.q_ref, uJ) != 0 || memcmp(p->adQdRef, a.stRep.qd_ref, uJ) != 0 ||
		    memcmp(p->adQddRef, a.stRep.qdd_ref, uJ) != 0 || memcmp(p->adQ, a.stRep.in.q, uJ) != 0 ||
		    memcmp(p->adQd, a.stRep.in.qd, uJ) != 0)
			bHomeRef = false;
		if (a.stRep.qd_ref[0] != 0.0 && a.stRep.qdd_ref[0] != 0.0 && a.stRep.in.qd[0] != 0.0)
			nMoving++;
	}
	double dOff = 0.0;
	for (int i = 0; i < N; i++)
		dOff = std::fmax(dOff, std::fabs(a.adQ[i] - adQr[i]));
	Expect(nHome > 200 && nHome < 20000 && a.stRep.traj_state == (uint32_t)CTrajectoryRT::eDoneAtRest && dOff < 1e-3,
	       "... and ends at rest at the ready posture");
	Expect(bHomeSamples && a.stRep.motion == (uint32_t)CTrajectoryRT::eMotionNone,
	       "every period of it has a sample of HOME, the one in which it ends too");
	Expect(bHomeRef && nMoving > 100,
	       "... with the reference and the measured state of its report, also while they move");
	b.Period(a);
	Expect(a.cClient.Sample() == NULL, "a period without a motion has no sample");
	/* TRACK reads a slot whose goals get an offset; a goal written before the slot was chosen does not leave */
	const double adOffset[3] = { 0.0, 0.0, 0.02 };
	const double adBelow[3] = { adGoal[0], adGoal[1], adGoal[2] - adOffset[2] };
	a.cGoal.Write(adP0);
	a.cClient.UseGoalSlot(&a.cGoal, adOffset);
	a.cClient.Queue(IPC_CMD_START, 0, &stStart);
	Expect(b.UntilAnswered(a) && a.stRep.start_result == IPC_DONE_OK && a.stRep.motion == (uint32_t)CTrajectoryRT::eMotionTrack, "TRACK starts");
	b.Periods(a, 5);
	Expect(a.cClient.GetCounts().nGoals == 0 && a.stRep.n_goals == 0 && a.cClient.Sample() != NULL && a.cClient.Sample()->nMode == 0 &&
	       std::isnan(a.cClient.Sample()->adGoal[0]) && !(a.stRep.hl_flags & CTrajectoryRT::HL_HAS_GOAL),
	       "the goal from before does not leave: TRACK has none, and its sample says so");
	a.cGoal.Write(adBelow);
	b.Periods(a, 6);
	Expect(a.cClient.GetCounts().nGoals == 1 && a.stRep.n_goals == 1 && (a.stRep.in.flags & IPC_SHADOW_GOAL) &&
	       a.stRep.in.goal[0] == adBelow[0] && a.stRep.in.goal[2] == adBelow[2] + adOffset[2],
	       "a goal written then arrives through the Goal slot, with the offset");
	/* a start in a motion is refused, and the client has the reason as a line */
	{
		ipc_command_t stBad = stStart;
		stBad.delta = 2.0;
		a.cClient.Queue(IPC_CMD_START, IPC_START_HOME, &stStart);
		Expect(b.UntilAnswered(a) && a.uLastResult == IPC_REFUSED_START, "a start while TRACK runs is refused");
		a.cClient.ResultText(a.uLastResult, acWhy, sizeof(acWhy));
		Expect(strstr(acWhy, "TRACK is running") != NULL, "... for that");
		a.cClient.Queue(IPC_CMD_START, 0, &stBad);
		Expect(b.UntilAnswered(a) && a.uLastResult == IPC_REFUSED_ARGUMENT, "a start with a goal threshold of 2 m is refused");
		a.cClient.ResultText(a.uLastResult, acWhy, sizeof(acWhy));
		Expect(strstr(acWhy, "goal threshold") != NULL && a.stRep.traj_state == (uint32_t)CTrajectoryRT::eRunning,
		       "... for that, and TRACK goes on");
	}
	a.dTauOff = 5e-7;			// inside the bounds: no difference
	a.dRefOff = 5e-10;
	b.Periods(a, 1000);
	{
		const CControlFollower::Counts& o = a.cFollowOff.GetCounts();
		Expect(o.nTauDiff == 0 && o.nRefDiff == 0 && o.dTauDiffMax > 4e-7 && o.dTauDiffMax < 6e-7 && o.dRefDiffMax > 4e-10 &&
		       o.dRefDiffMax < 6e-10, "a torque 5e-7 N m off and a reference 5e-10 rad off are not differences, and the largest is kept");
	}
	a.dTauOff = 2e-6;			// outside: 200 torques ...
	b.Periods(a, 200);
	a.dTauOff = 0.0;
	a.dRefOff = 2e-9;			// ... and 150 references
	b.Periods(a, 150);
	a.dRefOff = 0.0;
	a.dTauOff = std::numeric_limits<double>::quiet_NaN();		// and one of each that is no number
	a.dRefOff = std::numeric_limits<double>::quiet_NaN();
	b.Period(a);
	a.dTauOff = 0.0;
	a.dRefOff = 0.0;
	b.Periods(a, 3649);
	double adTool[3], dFar = 0.0;
	cHl.ToolPosition(a.adQ, adTool);
	for (int k = 0; k < 3; k++)
		dFar = std::fmax(dFar, std::fabs(adTool[k] - adGoal[k]));
	Expect(dFar < 2e-3 && a.stRep.n_plan_late == 0 && a.stRep.n_iks > 0 && a.stRep.n_plans > 1 && a.cClient.GetCounts().nGoals == 1,
	       "TRACK brings the tool of the arm to it; the goal left once");
	/* the sample that the log of the tests takes is the report's */
	{
		const CTrajectoryRT::Sample* const p = a.cClient.Sample();
		const ipc_report_t& r = a.stRep;
		const size_t uJ = sizeof(double) * N, uP = sizeof(double) * 3;
		Expect(p != NULL && p->nMode == 0 && p->dT == (double)r.n_cycle * 1e-3 && r.n_cycle > 5000 &&
		       p->uFlags == r.hl_flags && (p->uFlags & CTrajectoryRT::HL_AT_REST) && p->nOtg == r.otg && r.otg == 1 &&
		       p->uLeadReset == r.lead_reset && p->nIkIter == r.ik_iter && r.ik_iter >= 1 && p->dIkErr == r.ik_err &&
		       r.ik_err > 0.0 && p->dOtgDuration == r.otg_duration && r.otg_duration > 0.1 &&
		       p->dHlUs == (double)r.hl_ns * 1e-3 && p->nGoalAge == (long)r.goal_age && r.goal_age > 5000,
		       "the sample of a period of TRACK: its time, its flags and the numbers of the IK and of the plan are the report's");
		Expect(p != NULL && memcmp(p->adGoal, r.in.goal, uP) == 0 && memcmp(p->adAccepted, r.accepted, uP) == 0 &&
		       memcmp(p->adTool, r.tool, uP) == 0 && memcmp(p->adTarget, r.target, uJ) == 0 &&
		       memcmp(p->adQRef, r.q_ref, uJ) == 0 && memcmp(p->adQdRef, r.qd_ref, uJ) == 0 &&
		       memcmp(p->adQddRef, r.qdd_ref, uJ) == 0 && memcmp(p->adQ, r.in.q, uJ) == 0 && memcmp(p->adQd, r.in.qd, uJ) == 0 &&
		       std::fabs(p->adTool[2] - adGoal[2]) < 2e-3 && std::fabs(p->adAccepted[2] - adGoal[2]) < 1e-12,
		       "... and so are its goal, the accepted goal, the tool, the target, the reference and the measured state");
		Expect(a.cClient.HlCounts()[0] == 1 && a.cClient.HlCounts()[1] == 1 && a.cClient.HlCounts()[2] == 0 &&
		       a.cClient.HlCounts()[3] == 0 && r.n_accepted == 1 && r.n_converged == 1,
		       "the counts of the motion: one goal accepted, its IK converged");
		const CControlClient::Counts& c = a.cClient.GetCounts();
		Expect(c.nTraj > 5000 && c.nLaw > c.nTraj && c.dTrajNsSum > 0.0 && c.dLawNsSum > 0.0 && c.uTrajNsMax > 0 &&
		       c.uLawNsMax > 0 && (double)c.uTrajNsMax >= c.dTrajNsSum / (double)c.nTraj &&
		       (double)c.uLawNsMax >= c.dLawNsSum / (double)c.nLaw,
		       "the times of the two parts on the R5 are summed over the reports taken");
	}
	a.cClient.Queue(IPC_CMD_MOTION_STOP);
	Expect(b.UntilAnswered(a) && a.stRep.traj_state == (uint32_t)CTrajectoryRT::eAborted, "the stop");
	a.cClient.Queue(IPC_CMD_START, IPC_START_HOME, &stStart);
	Expect(a.cClient.HlCounts()[0] == 0 && a.cClient.HlCounts()[1] == 0, "the counts of a motion do not stay for the next");
	Expect(b.UntilAnswered(a) && a.stRep.start_result == IPC_DONE_OK, "HOME once more");
	a.cClient.Queue(IPC_CMD_MOTION_STOP);
	Expect(b.UntilAnswered(a), "... and stopped");
	b.Periods(a, 200);

	/* a start of TRACK that waits behind another command: the goal written with it waits too (TRACK on the R5 counts
	 * the goals that arrive from its start on), leaves when the start has left, and TRACK has it */
	{
		const double adNone[3] = { 0.0, 0.0, 0.0 };
		double adHere[3];
		cHl.ToolPosition(a.adQ, adHere);
		const double adNear[3] = { adHere[0] + 0.02, adHere[1], adHere[2] };
		const long nGoalsBefore = a.cClient.GetCounts().nGoals, nCommandsBefore2 = a.Commands();
		a.cClient.UseGoalSlot(&a.cGoal, adNone);
		a.cClient.Queue(IPC_CMD_HOLD_OFF);
		a.cClient.Queue(IPC_CMD_HOLD_ON);
		a.cClient.Queue(IPC_CMD_START, 0, &stStart);
		a.cGoal.Write(adNear);
		b.Period(a);
		Expect(a.Commands() == nCommandsBefore2 + 1 && a.cClient.GetCounts().nGoals == nGoalsBefore,
		       "a start of TRACK behind two other commands: its goal does not leave before it");
		Expect(b.UntilAnswered(a) && a.stRep.start_result == IPC_DONE_OK && a.stRep.motion == (uint32_t)CTrajectoryRT::eMotionTrack &&
		       a.cClient.GetCounts().nGoals == nGoalsBefore + 1, "the start is carried out, and the goal has left with it");
		b.Periods(a, 20);
		Expect((a.stRep.hl_flags & CTrajectoryRT::HL_HAS_GOAL) && a.stRep.n_accepted == 1 && a.stRep.in.goal[0] == adNear[0],
		       "TRACK has the goal");
		// a goal of the running TRACK does not wait for other commands
		const double adNext[3] = { adNear[0], adNear[1] + 0.02, adNear[2] };
		const long nCommandsBefore3 = a.Commands();
		a.cClient.Queue(IPC_CMD_HOLD_ON);
		a.cClient.Queue(IPC_CMD_HOLD_OFF);
		a.cGoal.Write(adNext);
		b.Period(a);
		Expect(a.Commands() == nCommandsBefore3 + 1 && !a.cClient.IsSettled() && a.cClient.GetCounts().nGoals == nGoalsBefore + 2,
		       "a goal of the TRACK that runs leaves at once, also while commands wait");
		Expect(b.UntilAnswered(a), "... which are carried out");
		b.Periods(a, 20);
		Expect(a.stRep.n_accepted == 2 && a.stRep.in.goal[1] == adNext[1], "TRACK has the second goal");
		a.cClient.Queue(IPC_CMD_MOTION_STOP);
		a.cClient.Queue(IPC_CMD_HOLD_ON);
		Expect(b.UntilAnswered(a) && a.stRep.traj_state == (uint32_t)CTrajectoryRT::eAborted, "stopped");
		b.Periods(a, 300);
	}

	/* what the R5 counts of its controller and of the drives, for this run: a law that fails on a state that is no
	 * number, then the drives of the arm going off by themselves */
	{
		a.nNoNumber = 2;
		b.Periods(a, 5);			// the first of them writes the state with it
		Expect(a.stRep.n_ctrl_faults == 1 && (a.stRep.state & IPC_CTL_FAULT) && !(a.stRep.state & IPC_CTL_SERVO_ASKED) &&
		       a.cClient.GetCounts().nCtrlFaults == 1 && a.cClient.GetCounts().nDriveStops == 0 && !a.bServoOn,
		       "a state that is no number: the controller fault of the R5, counted once; the drives go off by its request");
		a.cClient.Queue(IPC_CMD_CTRL_OFF);
		a.cClient.Queue(IPC_CMD_CTRL_ON);
		a.cClient.Queue(IPC_CMD_SERVO_ON);
		Expect(b.UntilAnswered(a) && a.stRep.cmd_result == IPC_DONE_OK && !(a.stRep.state & IPC_CTL_FAULT), "controller off clears it; servo on again");
		b.Periods(a, 6);
		Expect(a.bServoOn && (a.stRep.state & IPC_CTL_SERVO_ON), "the drives are on");
		a.bServoOn = false;			// the drives of the arm drop out
		b.Periods(a, 4);
		Expect(a.stRep.n_drive_stops == 1 && !(a.stRep.state & IPC_CTL_SERVO_ASKED) && a.cClient.GetCounts().nDriveStops == 1 &&
		       a.cClient.GetCounts().nCtrlFaults == 1 && !a.bServoOn,
		       "the drives of the arm go off by themselves: the R5 withdraws its request, a stop of the drives, counted once");
		b.Periods(a, 50);
	}

	printf("%u periods of the R5, %ld steps of the arm, %ld reports (%ld of periods with a state), %ld commands: a second "
	       "controller on the reports had the law in %ld and the reference in %ld of them, %ld differences, apart in "
	       "%ld; %ld reports of a period with a state not read, %ld with another state than the one written\n",
	       (unsigned)b.uPeriod, a.nSteps, a.Reports(), a.StateReports(), a.Commands(), a.F().nLaw, a.F().nTraj,
	       a.F().nTauDiff + a.F().nRefDiff, a.F().nApart, a.nLost + a.cClient.GetCounts().nStateMissed, a.nOtherState);
	Expect(a.F().nPeriods == a.StateReports() && a.F().nLaw > 5000 && a.F().nTraj > 5000 && a.SameBits() && a.F().nApart == 0 &&
	       a.nLost == 0 && a.cClient.GetCounts().nStateMissed == 0 && a.nOtherState == 0 && a.F().dLawNsSum > 0.0 &&
	       a.F().dTrajNsSum > 0.0 && (double)a.F().uLawNsMax >= a.F().dLawNsSum / (double)a.F().nLaw &&
	       (double)a.F().uTrajNsMax >= a.F().dTrajNsSum / (double)a.F().nTraj,
	       "the second controller on the reports (r5link/ControlFollower.h) gives the torque and the reference of every "
	       "period with a state, bit for bit, and has the time of its two parts");
	{
		// the same on reports that are off: 5e-7 N m and 5e-10 rad are inside what two CPUs may differ by, 2e-6 N m
		// and 2e-9 rad are not, and a number that is none is a difference
		const CControlFollower::Counts& o = a.cFollowOff.GetCounts();
		Expect(o.nLaw == a.F().nLaw && o.nTraj == a.F().nTraj && o.nApart == 0 && o.nTauDiff == 201 && o.nRefDiff == 151 &&
		       o.dTauDiffMax > 1.9e-6 && o.dTauDiffMax < 2.1e-6 && o.dRefDiffMax > 1.9e-9 && o.dRefDiffMax < 2.1e-9,
		       "a torque that is off by more than 1e-6 N m, a reference by more than 1e-9 rad, or one that is no number, "
		       "is counted as a difference; inside those bounds it is not");
	}

	/* --- another run of the application --- */
	static Apu a2, a3, a4;
	{
		memcpy(a2.adQ, a.adQ, sizeof(a.adQ));
		memset(a2.adQd, 0, sizeof(a2.adQd));
		a2.bServoOn = a.bServoOn;
		const uint32_t uStepOfFirst = a.uSimStep;
		Expect(a2.cFollow.Init(pcURDF, adNumbers, 0xC0DE0002u) && b.StartApu(a2), "a second run of the application starts on the region");
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
		Expect(a2.uSimStep == 1 && a2.nSteps == 0 && a2.Reports() == 1 && !(a2.stRep.state & IPC_CTL_STATE),
		       "it writes its first state; the R5 had a period without an arm between the two runs");
		b.Periods(a2, 10);
		Expect(a2.nSteps == 10 && a2.stRep.sim_step == 10 && uStepOfFirst > 100 && (a2.stRep.state & IPC_CTL_STATE) &&
		       (a2.stRep.state & IPC_CTL_ENABLED) && a2.nLost == 0 && a2.nOtherState == 0 &&
		       a2.cClient.GetCounts().nStateMissed == 0,
		       "the control task, which went on, takes up its arm from that state");
		Expect(a2.stRep.n_ctrl_faults == 1 && a2.stRep.n_drive_stops == 1 && a2.cClient.GetCounts().nCtrlFaults == 0 &&
		       a2.cClient.GetCounts().nDriveStops == 0,
		       "the controller fault and the stop of the drives that the R5 counted in the run before are not of this run");
		/* what else the state of the arm says: joints that cannot be switched on. (Its measured torque goes to the
		 * law as on the APU, and the law does not use it: nothing to see.) */
		a2.cClient.Queue(IPC_CMD_SERVO_OFF);
		Expect(b.UntilAnswered(a2) && a2.stRep.cmd_result == IPC_DONE_OK, "servo off");
		b.Periods(a2, 3);
		a2.uNotReady = 1u << 2;
		b.Periods(a2, 2);
		a2.cClient.Queue(IPC_CMD_SERVO_ON);
		Expect(!a2.bServoOn && b.UntilAnswered(a2) && a2.stRep.cmd_result == IPC_REFUSED_NOT_READY &&
		       a2.uLastCmd == IPC_CMD_SERVO_ON && a2.uLastResult == IPC_REFUSED_NOT_READY,
		       "a servo on while the state of the arm says that a joint is not ready is refused for that");
		a2.uNotReady = 0;
		b.Periods(a2, 2);
		a2.cClient.Queue(IPC_CMD_SERVO_ON);
		Expect(b.UntilAnswered(a2) && a2.stRep.cmd_result == IPC_DONE_OK, "... and taken when it does not");
		b.Periods(a2, 5);
		Expect(a2.bServoOn && (a2.stRep.state & IPC_CTL_SERVO_ON) && a2.stRep.n_drive_stops == 1, "the drives are on again");
		a2.bServoOn = false;
		b.Periods(a2, 4);
		Expect(a2.stRep.n_drive_stops == 2 && a2.cClient.GetCounts().nDriveStops == 1 && a2.cClient.GetCounts().nCtrlFaults == 0,
		       "a stop of the drives in this run is its first");
		a2.cClient.Queue(IPC_CMD_SERVO_ON);
		Expect(b.UntilAnswered(a2) && a2.stRep.cmd_result == IPC_DONE_OK, "servo on once more");
		b.Periods(a2, 5);
		Expect(a2.bServoOn && (a2.stRep.state & IPC_CTL_SERVO_ON), "... and on");
		/* the second controller of this run: the R5 had starts before it, which no report told it */
		Expect(a2.F().nPeriods == a2.StateReports() && a2.F().nApart == a2.F().nPeriods && a2.F().nLaw == 0 && a2.F().nTraj == 0,
		       "the second controller of a run that finds the R5 with motions before it: apart, nothing compared");
		a2.cClient.Queue(IPC_CMD_START, IPC_START_HOME, &stStart);
		Expect(b.UntilAnswered(a2) && a2.stRep.start_result == IPC_DONE_OK, "HOME starts in this run");
		const long nApartAtStart = a2.F().nApart;
		b.Periods(a2, 300);
		Expect(a2.F().nApart == nApartAtStart && a2.F().nLaw >= 300 && a2.F().nTraj > 100 && a2.SameBits(),
		       "from the first start of the run on it follows: the torque and the reference of the report, bit for bit");
		/* a report of a period with a state that the control task here does not get is counted */
		b.Period(a2);
		const long nMissedBefore = a2.cClient.GetCounts().nStateMissed, nLostBefore = a2.nLost;
		const long nLawAtGap = a2.F().nLaw, nPeriodsAtGap = a2.F().nPeriods;
		b.R5Period();
		{
			ipc_report_t stUnread;
			(void)a2.cLink.TakeReport(&stUnread);		// this report never reaches the client
		}
		a2.Cycle(b.ReleaseOf(b.uPeriod) + (RTTIME)DELAY_NS);		// the arm steps all the same
		b.Period(a2);
		Expect(a2.cClient.GetCounts().nStateMissed == nMissedBefore + 1 && a2.nLost == nLostBefore + 1,
		       "a report of a period with a state that did not arrive is counted, from the numbers of the steps");
		b.Periods(a2, 20);
		Expect(a2.F().nPeriods == nPeriodsAtGap + 21 && a2.F().nApart == nApartAtStart + 21 && a2.F().nLaw == nLawAtGap && a2.SameBits(),
		       "... and leaves the second controller apart: its periods are counted, not compared");
		a2.cClient.Queue(IPC_CMD_MOTION_STOP);
		a2.cClient.Queue(IPC_CMD_START, IPC_START_HOME, &stStart);
		Expect(b.UntilAnswered(a2) && a2.stRep.start_result == IPC_DONE_OK, "the next start");
		const long nApartAtStart2 = a2.F().nApart, nLawAtStart2 = a2.F().nLaw;
		b.Periods(a2, 100);
		Expect(a2.F().nApart == nApartAtStart2 && a2.F().nLaw == nLawAtStart2 + 100 && a2.SameBits(), "... brings the two together again");
		a2.cClient.Queue(IPC_CMD_MOTION_STOP);
		Expect(b.UntilAnswered(a2), "stopped");
	}
	/* --- a run without an arm: the control task has no state --- */
	uint32_t uNumberOfThird = 0;
	{
		a3.bArm = false;
		Expect(b.StartApu(a3), "a third run starts, without an arm");
		a3.cClient.Queue(IPC_CMD_START, IPC_START_HOME, &stStart);
		Expect(b.UntilAnswered(a3) && a3.stRep.cmd_result == IPC_REFUSED_NO_STATE && !(a3.stRep.state & (IPC_CTL_STATE | IPC_CTL_SIM_ARM)) &&
		       a3.StateReports() == 0 && a3.Commands() == 1,
		       "with it the control task of this R5 has no state: a start is refused for that");
		uNumberOfThird = a3.stRep.cmd_done;
		const long nReportsThird = a3.Reports();
		b.Periods(a3, 5);
		Expect(a3.Reports() == nReportsThird + 5 && a3.cClient.GetCounts().nStateMissed == 0,
		       "the arm is no longer the simulated one: every period reports");
	}
	/* --- and one more: the report still has the answer to the one command of the run before --- */
	{
		a4.bArm = false;
		Expect(b.StartApu(a4), "a fourth run starts");
		a4.cClient.Queue(IPC_CMD_HOLD_OFF);
		b.Period(a4);			// its first cycle: a report that is new to it, and its first command leaves
		Expect(a4.Commands() == 1 && a4.stRep.cmd_done == uNumberOfThird && !a4.cClient.IsSettled() && a4.nAnswers == 0,
		       "the report still has the number of the one command of the run before: not the answer to the first of this run");
		Expect(b.UntilAnswered(a4) && a4.stRep.cmd_result == IPC_DONE_OK && !(a4.stRep.in.flags & IPC_SHADOW_HOLD) &&
		       a4.nAnswers == 1 && a4.stRep.cmd_done != uNumberOfThird && (a4.stRep.cmd_done >> 16) == (uNumberOfThird >> 16) + 1 &&
		       (a4.stRep.cmd_done & 0xFFFFu) == 1u && (uNumberOfThird & 0xFFFFu) == 1u,
		       "it is answered in its turn; the first command of a run has not the number of the first of the run before");
	}

	/* --- a run with a new arm while the R5 still asks the servo on, for the arm of a run before --- */
	{
		static Apu a6;
		memcpy(a6.adQ, adQr, sizeof(a6.adQ));
		memset(a6.adQd, 0, sizeof(a6.adQd));
		const uint32_t uStopsBefore = b.cR5.LastReport().n_drive_stops;
		Expect((b.cR5.LastReport().state & IPC_CTL_SERVO_ASKED) && b.StartApu(a6),
		       "a run with a new arm starts; the R5 still asks the servo on, as the run with the arm before left it");
		b.Periods(a6, 6);
		Expect(!(a6.stRep.state & IPC_CTL_SERVO_ASKED) && a6.stRep.n_drive_stops == uStopsBefore + 1 &&
		       a6.cClient.GetCounts().nDriveStops == 1 && !a6.bServoOn && (a6.stRep.state & IPC_CTL_STATE),
		       "the drives of the new arm are off: the R5 withdraws its request, a stop of the drives that this run is told");
	}

	/* --- the reports that did not arrive, counted from the numbers in those that did --- */
	{
		CControlClient cCount;
		cCount.Init(&a4.cLink, 1e-3);
		ipc_report_t r;
		memset(&r, 0, sizeof(r));
		// nMissed: reports counted as not arrived so far; uOn: how far the number of the period with a state goes on
		// (1: the next one; 2: one is missing between, or nothing is known of what was between)
		struct { uint32_t uState, uPeriod, uNoState, uStep; long nMissed; uint32_t uOn; } const ast[] = {
			// the arm of the bus: the periods of the R5 less those without a state
			{ IPC_CTL_STATE, 10, 2, 7, 0, 2 },		// the first report says nothing of those before it
			{ IPC_CTL_STATE, 11, 2, 7, 0, 1 },
			{ IPC_CTL_STATE, 13, 2, 7, 1, 2 },		// that of period 12 did not arrive
			{ IPC_CTL_STATE, 15, 3, 7, 1, 1 },		// period 14 had no state: nothing to miss
			{ IPC_CTL_STATE, 16, 3, 7, 1, 1 },
			// then an arm simulated here: its steps, whatever period numbers the R5 leaves out
			{ IPC_CTL_STATE | IPC_CTL_SIM_ARM, 30, 3, 20, 1, 2 },		// the first of another arm
			{ IPC_CTL_STATE | IPC_CTL_SIM_ARM, 32, 3, 21, 1, 1 },
			{ IPC_CTL_STATE | IPC_CTL_SIM_ARM, 33, 3, 23, 2, 2 },		// that of step 22 did not arrive
			{ IPC_CTL_SIM_ARM, 34, 4, 23, 2, 0 },						// a period without a state: not counted
			{ IPC_CTL_STATE | IPC_CTL_SIM_ARM, 35, 4, 1, 2, 2 },		// the numbers begin again: nothing is known
			{ IPC_CTL_STATE | IPC_CTL_SIM_ARM, 36, 4, 2, 2, 1 },
			{ IPC_CTL_STATE | IPC_CTL_SIM_ARM, 37, 4, 5, 4, 3 },		// two did not arrive
		};
		bool bCounted = true;
		for (size_t k = 0; k < sizeof(ast) / sizeof(ast[0]); k++)
		{
			const uint32_t uBefore = cCount.StatePeriod();
			r.state = ast[k].uState;
			r.in.period = ast[k].uPeriod;
			r.n_no_state = ast[k].uNoState;
			r.sim_step = ast[k].uStep;
			cCount.Take(r);
			if (cCount.GetCounts().nStateMissed != ast[k].nMissed || cCount.StatePeriod() - uBefore != ast[k].uOn)
			{
				bCounted = false;
				printf("  report %u: %ld counted as not arrived, not %ld; the period number goes on by %u, not %u\n", (unsigned)k,
				       cCount.GetCounts().nStateMissed, ast[k].nMissed, (unsigned)(cCount.StatePeriod() - uBefore), (unsigned)ast[k].uOn);
			}
		}
		Expect(bCounted && cCount.GetCounts().nReports == 12 && cCount.GetCounts().nStateReports == 11,
		       "the reports of periods with a state that did not arrive: by the periods of the R5 with the arm of its bus, by "
		       "the steps with an arm simulated here; not across a change of the arm or numbers that begin again");
	}

	/* --- what the R5 did in this run: counted from the first report that the run took --- */
	{
		CControlClient cRun;
		cRun.Init(&a4.cLink, 1e-3);
		ipc_report_t r;
		memset(&r, 0, sizeof(r));
		// what the R5 has counted since its own start, and the time of its last plan and IK [us]; then what the
		// client has of this run after the report
		struct { uint32_t uNoState, uPlans, uPlanUs, uIks, uIkUs, uLate; long nNoState, nPlans, nIks, nLate;
		         uint32_t uPlanMax, uIkMax, uGoals; long nGoals; } const ast[] = {
			{ 5000, 40, 900, 12, 3000, 3,   0, 0, 0, 0,     0,   0,   574, 0 },	// the first report: of before this run, the times too
			{ 5001, 40, 900, 12, 3000, 3,   1, 0, 0, 0,     0,   0,   574, 0 },	// a period without a state
			{ 5001, 41, 300, 12, 3000, 3,   1, 1, 0, 0,   300,   0,   575, 1 },	// a plan of this run
			{ 5001, 41, 300, 13,  700, 3,   1, 1, 1, 0,   300, 700,   575, 1 },	// an IK
			{ 5001, 42, 250, 13,  700, 4,   1, 2, 1, 1,   300, 700,   577, 3 },	// a shorter plan, which was late
			{ 5003, 43, 450, 14,  600, 4,   3, 3, 2, 1,   450, 700,   577, 3 },	// a longer plan, a shorter IK
			{ 5003, 43, 990, 14,  990, 4,   3, 3, 2, 1,   450, 700,   578, 4 },	// no new plan and no new IK: no new times
		};
		bool bRun = true;
		for (size_t k = 0; k < sizeof(ast) / sizeof(ast[0]); k++)
		{
			r.n_no_state = ast[k].uNoState;
			r.n_plans = ast[k].uPlans;
			r.plan_us = ast[k].uPlanUs;
			r.n_iks = ast[k].uIks;
			r.ik_us = ast[k].uIkUs;
			r.n_plan_late = ast[k].uLate;
			r.n_goals = ast[k].uGoals;
			r.plan_max_us = 5000;		// the longest since the start of the R5: not those of the run
			r.ik_max_us = 9000;
			cRun.Take(r);
			const CControlClient::Counts& c = cRun.GetCounts();
			if (c.nNoState != ast[k].nNoState || c.nPlans != ast[k].nPlans || c.nIks != ast[k].nIks ||
			    c.nPlanLate != ast[k].nLate || c.uPlanUsMax != ast[k].uPlanMax || c.uIkUsMax != ast[k].uIkMax ||
			    c.nGoalsTaken != ast[k].nGoals)
			{
				bRun = false;
				printf("  report %u: %ld periods without a state, %ld plans (longest %u us), %ld IK (longest %u us), %ld "
				       "plans late, %ld goals taken\n", (unsigned)k, c.nNoState, c.nPlans, (unsigned)c.uPlanUsMax, c.nIks,
				       (unsigned)c.uIkUsMax, c.nPlanLate, c.nGoalsTaken);
			}
		}
		Expect(bRun, "the periods without a state, the plans, the IK, the late plans and the goals that the R5 counts are told from the "
		             "first report of the run on, and the longest plan and IK are those of the run");
	}

	/* --- the two sides in two threads: the report of a period is written before its torque --- */
	{
		static Apu a5;
		memcpy(a5.adQ, adQr, sizeof(a5.adQ));
		memset(a5.adQd, 0, sizeof(a5.adQd));
		Expect(b.StartApu(a5), "a fifth run starts, against the R5 in a thread of its own");
		std::atomic<bool> bStop{false};
		std::thread cR5([&bStop]() {
			while (!bStop.load(std::memory_order_relaxed))
				b.R5Period();
		});
		const long nWant = 20000;
		long nWith = 0, nWithout = 0;
		const RTTIME tGiveUp = read_timer() + 60000000000ull;		// the host may stop a thread for a while
		a5.WriteState();
		while (nWith + nWithout < nWant && read_timer() < tGiveUp)
		{
			ipc_sim_torque_t t;
			if (!a5.cLink.TakeSimTorque(&t) || t.step != a5.uSimStep)
				continue;
			// read at once, as soon after the torque as this thread can
			ipc_report_t r;
			if (a5.cLink.TakeReport(&r) && (r.state & IPC_CTL_STATE) && r.sim_step == t.step)
				nWith++;
			else
				nWithout++;
			a5.WriteState();
		}
		bStop.store(true, std::memory_order_relaxed);
		cR5.join();
		printf("between two threads: %ld torques that answered a state, %ld of them without the report of that period\n",
		       nWith + nWithout, nWithout);
		Expect(nWith == nWant && nWithout == 0,
		       "whoever finds the torque of its state finds the report of that period, whenever it looks");
	}

	printf("%s: %d checks failed\n", g_nFailed == 0 ? "PASS" : "FAIL", g_nFailed);
	return g_nFailed == 0 ? 0 : 1;
}
