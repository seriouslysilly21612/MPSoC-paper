/*****************************************************************************
*	Name: ObsMethod.h
*	Description: The methods of the comparison (paper, Table III), chosen by
*	the run option --method: what a run does with the results of the camera.
*
*	  M1    as before: the bridge averages the object position, which the
*	        camera gives in the base frame, and writes the goal slot.
*	  M2    the results go through the mailbox (obs_mailbox.h) to the control
*	        task, which accepts them by their age and takes the joint angles
*	        of the period of arrival.
*	  A2    M2 with the acceptance rule (C2): the joint angles of the capture
*	        and the error bound of eq. (7).
*	  A3    M2 with the admission rule (C3) on the camera side.
*	  FULL  both rules.
*
*	The control application switches the acceptance rule only; the camera
*	pipeline is started with the same method and switches the admission rule.
*****************************************************************************/
#ifndef G2T_OBS_METHOD_H
#define G2T_OBS_METHOD_H

#include <cstring>

#include "obs_receiver.h"

namespace obs_method
{
	enum eMethod { eM1 = 0, eM2, eA2, eA3, eFULL, eMethods };

	inline const char* Name(int anMethod)
	{
		static const char* const s_apc[eMethods] = { "M1", "M2", "A2", "A3", "FULL" };
		return (anMethod >= 0 && anMethod < eMethods) ? s_apc[anMethod] : "?";
	}
	/* -1: no such method */
	inline int Parse(const char* apcName)
	{
		for (int n = 0; n < eMethods; n++)
			if (0 == strcmp(apcName, Name(n)))
				return n;
		return -1;
	}
	inline bool UsesMailbox(int anMethod) { return anMethod != eM1; }
	/* the policy of the receiver (obs_receiver.h) */
	inline int Policy(int anMethod) { return (anMethod == eA2 || anMethod == eFULL) ? OBS_CONTRACT : OBS_MAX_AGE; }
}

#endif // G2T_OBS_METHOD_H
