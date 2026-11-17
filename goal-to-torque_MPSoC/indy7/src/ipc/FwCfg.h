/*****************************************************************************
*	Name: FwCfg.h
*	Description: The check value of the constants that the R5 firmware is
*	built with (D-15). The firmware gets [ECAT_MASTER] and the slaves of the
*	bus as a generated header (bin/fw_cfg_header) and writes the CRC-32 of
*	that header into the State slot (cfg_crc, ipc_layout.h). The controller
*	computes the same value from its own configuration and sends "servo on"
*	only when the two agree: a firmware built for another bus, or from
*	another INDY7.cfg, does not get the drives switched on.
*
*	The values in a fixed order, each 4 or 8 bytes, least significant byte
*	first: FW_CFG_LAYOUT, the number of slaves, CYCLE_TIME, SEND_DEADLINE,
*	SYNC_MODE, DC_SYNC0_SHIFT; then for every slave VENDOR_ID, PRODUCT_CODE,
*	SLAVE_TYPE, MOVE_CCW, DRIVE_MODE, ENCODER_RESOLUTION, GEAR_RATIO,
*	TRANSMISSION_RATIO, TORQUE_CONSTANT, CURRENT_RATIO (doubles),
*	HOME_POSITION_OFFSET, TORQUE_LIMIT_U (double). The names are left out.
*****************************************************************************/
#ifndef G2T_FW_CFG_H
#define G2T_FW_CFG_H

#include <cstdint>
#include <cstring>

#include "base/Config.h"

enum { FW_CFG_LAYOUT = 1 };		// + 1 with every change of the list above

/* CRC-32 (IEEE 802.3, as zlib): the CRC of more bytes after those of auCrc; start with 0 */
inline uint32_t FwCfgCrc32(uint32_t auCrc, const void* apBytes, size_t auLen)
{
	const uint8_t* pu = (const uint8_t*)apBytes;
	auCrc = ~auCrc;
	for (size_t i = 0; i < auLen; i++)
	{
		auCrc ^= pu[i];
		for (int b = 0; b < 8; b++)
			auCrc = (auCrc >> 1) ^ (0xEDB88320u & (0u - (auCrc & 1u)));
	}
	return ~auCrc;
}

/* the check value of a bus: [ECAT_MASTER] and the slaves on it (CConfig::Master(), CConfig::BusSlaves()) */
inline uint32_t FwCfgCrc(const ST_CONFIG_ECAT_MASTER& astMaster, const VEC_ECAT_SLAVE_CONF& avSlaves)
{
	uint32_t uCrc = 0;
	const auto U32 = [&uCrc](uint32_t au) {
		const uint8_t a[4] = { (uint8_t)au, (uint8_t)(au >> 8), (uint8_t)(au >> 16), (uint8_t)(au >> 24) };
		uCrc = FwCfgCrc32(uCrc, a, sizeof(a));
	};
	const auto F64 = [&U32](double ad) {
		uint64_t u;
		memcpy(&u, &ad, sizeof(u));
		U32((uint32_t)u);
		U32((uint32_t)(u >> 32));
	};
	U32(FW_CFG_LAYOUT);
	U32((uint32_t)avSlaves.size());
	U32((uint32_t)astMaster.nCycleTime);
	U32((uint32_t)astMaster.nSendDeadline);
	U32((uint32_t)astMaster.nSyncMode);
	U32((uint32_t)astMaster.nDcSync0Shift);
	for (const ST_CONFIG_ECAT_SLAVE& s : avSlaves)
	{
		U32(s.uVendorID);
		U32(s.uProductCode);
		U32((uint32_t)s.nSlaveType);
		U32(s.bCCW ? 1u : 0u);
		U32((uint32_t)s.nDriveMode);
		F64(s.dEncResolution);
		F64(s.dGearRatio);
		F64(s.dTransRatio);
		F64(s.dTorqueConstant);
		F64(s.dCurrentRatio);
		U32((uint32_t)s.nHomePositionOffset);
		F64(s.dTorLimitU);
	}
	return uCrc;
}

#endif // G2T_FW_CFG_H
