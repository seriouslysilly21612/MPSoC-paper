/*****************************************************************************
*	Name: ObsCrc.h
*	Description: The check values of the receiver's numbers, from the numbers
*	alone, for both sides: the APU computes them from its configuration
*	(ShadowParams.h, ObsContract.h), the control task on the R5 from the
*	numbers that IPC_CMD_OBS_NUMBERS brought (shadow/ControlCycle.h).
*
*	CRC-32 (zlib) as FwCfgCrc32 of ipc/FwCfg.h, which the R5 has not:
*	    ObsNumbersCrcOf    the count as a little-endian 32-bit word, then each
*	                       number as a little-endian double (NumbersCrc)
*	    ObsContractCrcOf   OBS_CONTRACT_LAYOUT as a little-endian 32-bit word,
*	                       then the nine constants of [CONTRACT] as
*	                       little-endian doubles (ObsContractCrc)
*	One file for both sides (rpu_work/r5_freertos/src/obs/ObsCrc.h is a copy).
*****************************************************************************/
#ifndef G2T_OBS_CRC_H
#define G2T_OBS_CRC_H

#include <cstddef>
#include <cstdint>
#include <cstring>

#define OBS_CONTRACT_LAYOUT	1u		/* + 1 with every change of the constants of ObsContractCrcOf */
enum { OBS_CONTRACT_CONSTANTS = 9 };	/* delta, age cap, e0, g, eps_v, a_max, eps_cal, eps_m of the two modes */

inline uint32_t ObsCrc32(uint32_t auCrc, const uint8_t* apBytes, size_t auLen)
{
	auCrc = ~auCrc;
	for (size_t i = 0; i < auLen; i++)
	{
		auCrc ^= apBytes[i];
		for (int b = 0; b < 8; b++)
			auCrc = (auCrc >> 1) ^ (0xEDB88320u & (0u - (auCrc & 1u)));
	}
	return ~auCrc;
}

inline uint32_t ObsCrcWord(uint32_t auCrc, uint32_t auWord)
{
	const uint8_t au[4] = { (uint8_t)auWord, (uint8_t)(auWord >> 8), (uint8_t)(auWord >> 16), (uint8_t)(auWord >> 24) };
	return ObsCrc32(auCrc, au, sizeof(au));
}

inline uint32_t ObsCrcDoubles(uint32_t auCrc, const double* adValues, int anCount)
{
	for (int i = 0; i < anCount; i++)
	{
		uint64_t u;
		memcpy(&u, &adValues[i], sizeof(u));
		uint8_t a[8];
		for (int b = 0; b < 8; b++)
			a[b] = (uint8_t)(u >> (8 * b));
		auCrc = ObsCrc32(auCrc, a, sizeof(a));
	}
	return auCrc;
}

/* anCount numbers (the receiver's: OBS_NUMBERS of ShadowControl.h); the count below 2^16 */
inline uint32_t ObsNumbersCrcOf(const double* adNumbers, int anCount)
{
	return ObsCrcDoubles(ObsCrcWord(0u, (uint32_t)(anCount & 0xFFFF)), adNumbers, anCount);
}

/* the nine constants of [CONTRACT], in the order of OBS_CONTRACT_CONSTANTS */
inline uint32_t ObsContractCrcOf(const double* adConstants)
{
	return ObsCrcDoubles(ObsCrcWord(0u, OBS_CONTRACT_LAYOUT), adConstants, OBS_CONTRACT_CONSTANTS);
}

#endif // G2T_OBS_CRC_H
