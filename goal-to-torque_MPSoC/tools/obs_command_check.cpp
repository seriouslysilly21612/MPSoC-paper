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
#include "obs/ObsCrc.h"
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

	// IPC_CMD_OBS_NUMBERS (B of 2026-11-28): the APU's [CONTRACT] and beta in place of those of the build
	auto Numbers = [&](const double* adFrom, uint32_t auCheck) {
		ipc_command_t stC;
		memset(&stC, 0, sizeof(stC));
		stC.cmd = IPC_CMD_OBS_NUMBERS;
		stC.check = auCheck;
		for (int i = 0; i < OBS_SENT; i++)
			ipc_command_set_number(&stC, i, adFrom[OBS_SENT_INDEX[i]]);
		stC.seq = ++uSeq;
		Period(&stC, true);
		return stRep.cmd_result;
	};
	auto ObsWith = [&](uint32_t auMethod, uint32_t auCheck) {
		ipc_command_t stC;
		memset(&stC, 0, sizeof(stC));
		stC.cmd = IPC_CMD_OBS;
		stC.arg = auMethod;
		stC.check = auCheck;
		stC.seq = ++uSeq;
		Period(&stC, true);
		return stRep.cmd_result;
	};
	auto StatusCrc = [&]() {
		obs_status_t stS;
		uint32_t auStamp[2];
		memset(&stS, 0, sizeof(stS));
		obs_status_read(stObs.pStatus, &stS, auStamp);
		return stS.contract_crc;
	};
	double adApu[OBS_NUMBERS];				// another configuration of the APU: eps_v, eps_m, eps_cal and beta
	memcpy(adApu, adObs, sizeof(adApu));
	adApu[OBS_N_EPS_V] = 0.03;
	adApu[OBS_N_EPS_M] = 0.0054;
	adApu[OBS_N_EPS_CAL] = 0.0;
	adApu[OBS_N_BETA] = 0.1;
	const uint32_t uApu = ObsNumbersCrc(adApu);
	const double adApuContract[OBS_CONTRACT_CONSTANTS] = { adApu[OBS_N_DELTA], adApu[OBS_N_AGE_CAP], adApu[OBS_N_E0],
	                                                       adApu[OBS_N_G], adApu[OBS_N_EPS_V], adApu[OBS_N_A_MAX],
	                                                       adApu[OBS_N_EPS_CAL], adApu[OBS_N_EPS_M], adApu[OBS_N_EPS_M + 1] };
	Expect(ObsWith(obs_method::eFULL, uApu) == IPC_REFUSED_OBS, "numbers: FULL with the APU's check value is refused before");
	Expect(Numbers(adApu, uApu) == IPC_DONE_OK, "numbers: taken with the receiver off");
	Expect(ObsWith(obs_method::eFULL, uApu) == IPC_DONE_OK, "numbers: FULL with the APU's check value is taken");
	Periods(3, true);
	Expect(StatusCrc() == ObsContractCrcOf(adApuContract) && StatusCrc() != ObsContractCrc(cConfig.Contract()),
	       "numbers: the status has the check value of the APU's [CONTRACT]");
	Expect(Numbers(adObs, stObs.uNumbersCrc) == IPC_REFUSED_OBS, "numbers: refused while FULL runs");
	Expect(ObsWith(obs_method::eM1, 0u) == IPC_DONE_OK, "numbers: M1");
	double adOther[OBS_NUMBERS];			// another APP_* (the goal floor): not sent, so another check value
	memcpy(adOther, adApu, sizeof(adOther));
	adOther[OBS_N_FLOOR] += 0.01;
	Expect(Numbers(adOther, ObsNumbersCrc(adOther)) == IPC_REFUSED_OBS, "numbers: another APP_* is refused");
	double adBad[OBS_NUMBERS];				// beta at which the alpha-beta filter is unstable (4 - 2 kappa)
	memcpy(adBad, adApu, sizeof(adBad));
	adBad[OBS_N_BETA] = 4.0 - 2.0 * adBad[OBS_N_KAPPA];
	Expect(Numbers(adBad, ObsNumbersCrc(adBad)) == IPC_REFUSED_OBS, "numbers: an unstable beta is refused");
	adBad[OBS_N_BETA] = 0.1;
	adBad[OBS_N_EPS_V] = -0.01;
	Expect(Numbers(adBad, ObsNumbersCrc(adBad)) == IPC_REFUSED_OBS, "numbers: a negative term of eq. (7) is refused");
	Expect(ObsWith(obs_method::eFULL, uApu) == IPC_DONE_OK, "numbers: after the refusals the receiver keeps the APU's");
	Expect(ObsWith(obs_method::eM1, 0u) == IPC_DONE_OK && Numbers(adObs, stObs.uNumbersCrc) == IPC_DONE_OK &&
	       ObsWith(obs_method::eFULL, stObs.uNumbersCrc) == IPC_DONE_OK, "numbers: those of the build again");
	Periods(3, true);
	Expect(StatusCrc() == ObsContractCrc(cConfig.Contract()), "numbers: the status has the build's check value again");
	printf("%s: %d checks failed\n", g_nFailed == 0 ? "PASS" : "FAIL", g_nFailed);
	return g_nFailed == 0 ? 0 : 1;
}
