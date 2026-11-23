/*****************************************************************************
*	Name: obs_command_check.cpp
*	Description: The command of a method with the mailbox (IPC_CMD_OBS) and
*	the cost of the receiver in the control task (shadow/ControlCycle.h), on
*	a PC (I-135 C1, I-137 D4).
*
*	    bin/obs_command_check [-f FILE]
*
*	  -f FILE     the configuration, found as by indy7_ctrl
*
*	One control cycle with the link of the camera results, as the firmware of
*	the R5 has it, and a clock that advances on every read. Checked: the cost
*	of eq. (6) counts the periods from the command of a method on; a period
*	without a state does not count; a second command of the same method (the
*	next run of the application) starts the cost anew, with the slots B of
*	the writer of that run; M1 switches the receiver off.
*
*	Exit code 0 when no check failed.
*****************************************************************************/
#include <cstdio>
#include <cstring>
#include <vector>
#include <unistd.h>

#include "base/Config.h"
#include "obs/ObsContract.h"
#include "obs/ObsMethod.h"
#include "obs/obs_ring.h"
#include "shadow/ControlCycle.h"
#include "shadow/ShadowParams.h"

static int g_nFailed = 0;

static void Expect(bool abOk, const char* apcWhat)
{
	printf("%s  %s\n", abOk ? "ok  " : "FAIL", apcWhat);
	if (!abOk)
		g_nFailed++;
}

static uint64_t g_uClock = 0;
static uint64_t Clock() { return g_uClock += 7; }		// every read is later

int main(int argc, char** argv)
{
	TSTRING strCfg;
	int nOpt;
	while ((nOpt = getopt(argc, argv, "f:")) != -1)
	{
		if (nOpt == 'f')
			strCfg = optarg;
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
	double adNumbers[SHADOW_NUMBERS], adObs[OBS_NUMBERS];
	static CControlCycle cCtl;
	if (!ShadowNumbers(cConfig, adNumbers) || !cCtl.Init(cConfig.System().strURDFPath.c_str(), adNumbers, 0xC0DE0001u))
	{
		fprintf(stderr, "%s: the controller cannot be built from it\n", strCfg.c_str());
		return 2;
	}
	std::vector<uint32_t> vRegion(IPC_REGION_BYTES / 4u, 0u);
	volatile uint32_t* const pRegion = vRegion.data();
	ObsNumbers(cConfig, adObs);
	CControlCycle::ObsLink stObs;
	stObs.pBox = (volatile obs_mailbox_t*)(pRegion + OBS_OFF_MAILBOX / 4u);
	stObs.pStatus = (volatile obs_status_region_t*)(pRegion + OBS_OFF_STATUS / 4u);
	stObs.pRing = (volatile obs_ring_t*)(pRegion + OBS_OFF_RING / 4u);
	stObs.dTicksPerS = 1e9;
	stObs.pNumbers = adObs;
	stObs.uNumbersCrc = ObsNumbersCrc(adObs);
	stObs.uContractCrc = ObsContractCrc(cConfig.Contract());
	Expect(cCtl.SetObsLink(stObs), "the receiver gets the link and the numbers of this configuration");
	cCtl.SetObsClock(Clock);

	uint32_t uPeriod = 0, uSeq = 0;
	CControlCycle::Output stOut;
	ipc_report_t stRep;
	auto Period = [&](const ipc_command_t* apCommand, bool abState) {
		CControlCycle::Input stIn;
		memset(&stIn, 0, sizeof(stIn));
		stIn.uPeriod = ++uPeriod;
		stIn.bState = abState;
		stIn.tRelease = (uint64_t)uPeriod * 1000000u;
		stIn.fResponse = -1.0f;
		for (int i = 0; i < 6; i++)
			stIn.adQ[i] = adNumbers[SHADOW_N_READY + i];
		stIn.pCommand = apCommand;
		cCtl.Step(stIn, &stOut, &stRep);
		while (cCtl.ServePlanner())
			;
	};
	auto Obs = [&](uint32_t auMethod) {
		ipc_command_t stC;
		memset(&stC, 0, sizeof(stC));
		stC.cmd = IPC_CMD_OBS;
		stC.arg = auMethod;
		stC.check = stObs.uNumbersCrc;
		stC.seq = ++uSeq;
		Period(&stC, true);
		return stRep.cmd_result;
	};
	auto Periods = [&](int anPeriods, bool abState) {
		for (int i = 0; i < anPeriods; i++)
			Period(NULL, abState);
	};

	obs_writer_start_slots(stObs.pBox, 8);				// the writer of a first run of the application
	Expect(Obs(obs_method::eFULL) == IPC_DONE_OK, "FULL is taken");
	Periods(100, true);
	Expect(cCtl.ObsCost().periods == 101 && cCtl.ObsCost().slots == 8, "run 1: its 101 periods, B = 8");
	Periods(30, false);									// the application is gone (the bench with -b)
	Expect(cCtl.ObsCost().periods == 101, "a period without a state is not a period of the receiver");
	obs_writer_start_slots(stObs.pBox, 2);				// the writer of the next run, before its command
	Periods(5, true);
	Expect(cCtl.ObsCost().periods == 106 && cCtl.ObsCost().slots == 2, "run 2 before its command: B follows its writer");
	Expect(Obs(obs_method::eFULL) == IPC_DONE_OK && cCtl.ObsCost().periods == 1 && cCtl.ObsCost().slots == 2,
	       "run 2: the command of the same method starts the cost anew (its period the first)");
	Periods(50, true);
	Expect(cCtl.ObsCost().periods == 51, "run 2: 50 periods later");
	Expect(Obs(obs_method::eM1) == IPC_DONE_OK, "M1 is taken");
	const uint32_t uAfterM1 = cCtl.ObsCost().periods;
	Periods(20, true);
	Expect(cCtl.ObsCost().periods == uAfterM1, "M1: the receiver is off, its cost stays");
	printf("%s: %d checks failed\n", g_nFailed == 0 ? "PASS" : "FAIL", g_nFailed);
	return g_nFailed == 0 ? 0 : 1;
}
