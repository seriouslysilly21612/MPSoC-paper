/*****************************************************************************
*	Name: ShadowParams.h
*	Description: The constants of the controller as the numbers that the
*	second controller is built from (SHADOW_NUMBERS, shadow/ShadowControl.h),
*	read from the configuration as CRobotIndy7::InitController and
*	InitTrajectory read it, and their check value: the firmware of the R5
*	carries the numbers as a header, and a firmware built from another
*	configuration is told apart by the value.
*****************************************************************************/
#ifndef G2T_SHADOW_PARAMS_H
#define G2T_SHADOW_PARAMS_H

#include "base/Config.h"
#include "ipc/FwCfg.h"
#include "shadow/ShadowControl.h"

/* false: the arm of the configuration is not the arm of the controller (IPC_CTRL_DOF joints; the configuration
 * has the gains of ROBOT_DOF joints), or it has no control task */
inline bool ShadowNumbers(const CConfig& acConfig, double* adNumbers)
{
	const ST_CONFIG_SYSTEM& stS = acConfig.System();
	const int nDof = IPC_CTRL_DOF;
	if (stS.nRobotAxis != nDof || acConfig.Tasks().size() < 2)
		return false;
	double* const n = adNumbers;
	const CHighLevelRT::Params stDefault = CHighLevelRT::DefaultParams();
	std::vector<double> vTauMax;
	for (const ST_CONFIG_ECAT_SLAVE& s : acConfig.Slaves())
		if (s.nSlaveType == 0 && (int)vTauMax.size() < nDof)
			vTauMax.push_back(s.dTorLimitU);
	vTauMax.resize(nDof, 0.0);
	n[SHADOW_N_TS] = (double)acConfig.Tasks()[1].nPeriod * 1e-9;
	for (int i = 0; i < nDof; i++)
	{
		n[SHADOW_N_KP + i] = stS.vKp[i];
		n[SHADOW_N_KD + i] = stS.vKd[i];
		n[SHADOW_N_KF + i] = stS.vKf[i];
		n[SHADOW_N_ROTOR + i] = stS.vRotorB[i];
		n[SHADOW_N_TAU_MAX + i] = vTauMax[i];
		n[SHADOW_N_READY + i] = i < (int)stS.vHlReadyQ.size() ? stS.vHlReadyQ[i] : stDefault.adQr[i];
	}
	n[SHADOW_N_FF_GATE]    = (stS.nFfGate != 0) ? 1.0 : 0.0;
	n[SHADOW_N_DELTA]      = stS.dHlDelta;
	n[SHADOW_N_LAMBDA]     = stS.dHlLambda;
	n[SHADOW_N_STEP_MAX]   = stS.dHlStepMax;
	n[SHADOW_N_TOL]        = stS.dHlTol;
	n[SHADOW_N_PLAN_AHEAD] = (double)stS.nHlPlanAhead;
	n[SHADOW_N_ITER_MAX]   = (double)stS.nHlIterGoal;
	n[SHADOW_N_VEL_MAX]    = stS.dHlVelMax;
	n[SHADOW_N_ACC_MAX]    = stS.dHlAccMax;
	n[SHADOW_N_JERK_MAX]   = stS.dHlJerkMax;
	n[SHADOW_N_MARGIN]     = stS.dHlMargin;
	n[SHADOW_N_LEAD]       = stS.dHlLead;
	n[SHADOW_N_STEP_REF]   = (stS.nHlStepBaseline != 0) ? 1.0 : 0.0;
	return true;
}

/* the check value of the numbers: their count, then each as 8 bytes, least significant first */
inline uint32_t ShadowNumbersCrc(const double* adNumbers)
{
	const uint8_t auCount[4] = { (uint8_t)SHADOW_NUMBERS, (uint8_t)(SHADOW_NUMBERS >> 8), 0, 0 };
	uint32_t uCrc = FwCfgCrc32(0, auCount, sizeof(auCount));
	for (int i = 0; i < SHADOW_NUMBERS; i++)
	{
		uint64_t u;
		memcpy(&u, &adNumbers[i], sizeof(u));
		uint8_t a[8];
		for (int b = 0; b < 8; b++)
			a[b] = (uint8_t)(u >> (8 * b));
		uCrc = FwCfgCrc32(uCrc, a, sizeof(a));
	}
	return uCrc;
}

#endif // G2T_SHADOW_PARAMS_H
