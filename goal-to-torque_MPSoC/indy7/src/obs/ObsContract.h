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

#define OBS_CONTRACT_LAYOUT	1u		/* + 1 with every change of the list below */

inline uint32_t ObsContractCrc(const ST_CONFIG_CONTRACT& astContract)
{
	const double adConstant[9] = { astContract.dDeltaM, astContract.dAgeCapS, astContract.dE0M, astContract.dG,
	                               astContract.dEpsV, astContract.dAMax, astContract.dEpsCalM,
	                               astContract.vEpsM[0], astContract.vEpsM[1] };
	const uint32_t uLayout = OBS_CONTRACT_LAYOUT;
	const uint8_t auLayout[4] = { (uint8_t)uLayout, (uint8_t)(uLayout >> 8), (uint8_t)(uLayout >> 16),
	                              (uint8_t)(uLayout >> 24) };
	uint32_t uCrc = FwCfgCrc32(0u, auLayout, sizeof(auLayout));
	for (double dConstant : adConstant)
	{
		uint64_t uBits;
		memcpy(&uBits, &dConstant, sizeof(uBits));
		uint8_t auBytes[8];
		for (int i = 0; i < 8; i++)
			auBytes[i] = (uint8_t)(uBits >> (8 * i));
		uCrc = FwCfgCrc32(uCrc, auBytes, sizeof(auBytes));
	}
	return uCrc;
}

#endif // G2T_OBS_CONTRACT_H
