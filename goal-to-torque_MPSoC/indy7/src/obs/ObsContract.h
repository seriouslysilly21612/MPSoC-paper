/*****************************************************************************
*	Name: ObsContract.h
*	Description: The check value of the contract (paper, eq. (2), (7)): one
*	number for the constants of [CONTRACT] that the control task and the
*	camera side both compute with. The control task writes it into its
*	status (obs_status.h); the camera side compares it with its own
*	(contract.py of the camera pipeline) and runs no inference by the
*	admission rule while the two differ.
*
*	CRC-32 (zlib) of OBS_CONTRACT_LAYOUT as a little-endian 32-bit word,
*	then of each constant as a little-endian double, in the order of
*	ObsContractCrc. TAG_OFFSET_M is not part of it: only the control task
*	uses it.
*****************************************************************************/
#ifndef G2T_OBS_CONTRACT_H
#define G2T_OBS_CONTRACT_H

#include "ipc/FwCfg.h"
#include "obs/ObsCrc.h"

/* the list and its layout number are those of ObsCrc.h (OBS_CONTRACT_LAYOUT), which the R5 also has */
inline uint32_t ObsContractCrc(const ST_CONFIG_CONTRACT& astContract)
{
	const double adConstant[OBS_CONTRACT_CONSTANTS] = { astContract.dDeltaM, astContract.dAgeCapS, astContract.dE0M,
	                                                    astContract.dG, astContract.dEpsV, astContract.dAMax,
	                                                    astContract.dEpsCalM, astContract.vEpsM[0], astContract.vEpsM[1] };
	return ObsContractCrcOf(adConstant);
}

#endif // G2T_OBS_CONTRACT_H
