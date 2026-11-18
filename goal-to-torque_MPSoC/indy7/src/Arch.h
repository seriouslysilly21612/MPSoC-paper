/*****************************************************************************
*	Name: Arch.h
*	Description: The architectures of the comparison, chosen by the run
*	option --arch: where the EtherCAT task and the control task run.
*
*	  P0  both here, on the APU, as before: the EtherCAT task with the IgH
*	      master (TaskEtherCAT.cpp), the control task after it on one core.
*	  S1  the EtherCAT task on the R5 (rpu_work/r5_freertos). The control
*	      task stays here and follows the period of the R5; the EtherCAT task
*	      of the application remains as the task that takes the State slot
*	      of the R5 in every period (r5link/R5Link.h).
*
*	The method of the camera application (--method, obs/ObsMethod.h) is the
*	other axis: every method runs on every architecture.
*****************************************************************************/
#ifndef G2T_ARCH_H
#define G2T_ARCH_H

#include <cstring>

namespace arch
{
	enum eArch { eP0 = 0, eS1, eArchs };

	inline const char* Name(int anArch)
	{
		static const char* const s_apc[eArchs] = { "P0", "S1" };
		return (anArch >= 0 && anArch < eArchs) ? s_apc[anArch] : "?";
	}
	/* -1: no such architecture */
	inline int Parse(const char* apcName)
	{
		for (int n = 0; n < eArchs; n++)
			if (0 == strcmp(apcName, Name(n)))
				return n;
		return -1;
	}
}

#endif // G2T_ARCH_H
