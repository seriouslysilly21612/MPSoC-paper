/*****************************************************************************
*	Name: fw_cfg_header.cpp
*	Description: The constants of the R5 firmware from the configuration of
*	the controller (D-15): [ECAT_MASTER] and the slaves of the bus as a C
*	header, with the check value of indy7/src/ipc/FwCfg.h. The configuration
*	is read by the parser of the controller (base/Config.cpp), so the
*	firmware and the controller cannot read a value differently.
*
*	    bin/fw_cfg_header [-f FILE] [-B FILE] [-o HEADER]
*	    bin/fw_cfg_header --check
*
*	  -f FILE    the configuration, found as by indy7_ctrl
*	  -B FILE    the slaves of the bus from FILE (bench/L7N4_bus.cfg)
*	  -o HEADER  the header; without it to the standard output
*	  --check    the test of FwCfg.h: the CRC-32 against its check value and
*	             against a record packed and summed by another program; every
*	             value of the record changes the check value, a name does
*	             not; the header carries the values. Passes if no check failed.
*
*	The firmware must be built again from a new header whenever
*	[ECAT_MASTER] or an [AXISn] of the bus changes; the header is not edited.
*****************************************************************************/
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <unistd.h>

#include "ipc/FwCfg.h"

namespace {

/* the header of a bus. No text of the configuration goes into it, only numbers. */
std::string Header(const ST_CONFIG_ECAT_MASTER& astM, const VEC_ECAT_SLAVE_CONF& avSlaves)
{
	char ac[512];
	std::string str;
	snprintf(ac, sizeof(ac),
	         "/* The constants of the firmware. Written by fw_cfg_header from the configuration of the controller:\n"
	         " * not to be edited. */\n"
	         "#ifndef FW_CFG_H\n#define FW_CFG_H\n\n#include <stdint.h>\n\n"
	         "#define FW_CFG_CRC               0x%08Xu\t/* of the values below; into cfg_crc of the State slot */\n"
	         "#define FW_CFG_SLAVES            %u\n"
	         "#define FW_CFG_CYCLE_NS          %d\n"
	         "#define FW_CFG_SEND_DEADLINE_NS  %d\n"
	         "#define FW_CFG_SYNC_MODE         %d\t/* 0 FREE_RUN, 1 SM, 2 DC */\n"
	         "#define FW_CFG_DC_SYNC0_SHIFT_NS %d\n\n",
	         (unsigned)FwCfgCrc(astM, avSlaves), (unsigned)avSlaves.size(), (int)astM.nCycleTime,
	         (int)astM.nSendDeadline, (int)astM.nSyncMode, (int)astM.nDcSync0Shift);
	str += ac;
	str += "typedef struct\n{\n"
	       "\tuint32_t vendor_id, product_code;\n"
	       "\tint32_t  slave_type;\t\t\t/* 0 joint drive, 1 end tool */\n"
	       "\tint32_t  move_ccw, drive_mode;\n"
	       "\tdouble   encoder_resolution, gear_ratio, transmission_ratio, torque_constant, current_ratio;\n"
	       "\tint32_t  home_position_offset;\n"
	       "\tdouble   torque_limit_u;\n"
	       "} fw_cfg_axis_t;\n\n"
	       "static const fw_cfg_axis_t FW_CFG_AXIS[FW_CFG_SLAVES] =\n{\n";
	for (size_t i = 0; i < avSlaves.size(); i++)
	{
		const ST_CONFIG_ECAT_SLAVE& s = avSlaves[i];
		snprintf(ac, sizeof(ac), "\t{ 0x%08Xu, 0x%08Xu, %d, %d, %d, %.17g, %.17g, %.17g, %.17g, %.17g, %d, %.17g },\t/* [AXIS%u] */\n",
		         (unsigned)s.uVendorID, (unsigned)s.uProductCode, (int)s.nSlaveType, s.bCCW ? 1 : 0, (int)s.nDriveMode,
		         s.dEncResolution, s.dGearRatio, s.dTransRatio, s.dTorqueConstant, s.dCurrentRatio,
		         (int)s.nHomePositionOffset, s.dTorLimitU, (unsigned)i);
		str += ac;
	}
	str += "};\n\n#endif /* FW_CFG_H */\n";
	return str;
}

int g_nFailed = 0;

void Expect(bool abOk, const char* apcWhat)
{
	if (abOk)
		return;
	g_nFailed++;
	printf("  FAILED: %s\n", apcWhat);
}

int Check()
{
	Expect(FwCfgCrc32(0, "123456789", 9) == 0xCBF43926u && FwCfgCrc32(FwCfgCrc32(0, "1234", 4), "56789", 5) == 0xCBF43926u,
	       "CRC-32: the check value of \"123456789\", in one piece and in two");

	// a bus of a joint drive and an end tool; 0x86BCF14C is zlib.crc32 of the same 168 bytes from struct.pack
	ST_CONFIG_ECAT_MASTER stM;
	stM.nNoOfSlaves = 2;
	stM.nCycleTime = 1000000;
	stM.nSendDeadline = 400000;
	stM.nSyncMode = eSyncDc;
	stM.nDcSync0Shift = 125000;
	VEC_ECAT_SLAVE_CONF vS(2);
	vS[0].uVendorID = 0x00007595;
	vS[0].bCCW = TRUE;
	vS[0].dEncResolution = 524288.0;
	vS[0].dGearRatio = 1.0;
	vS[0].dTransRatio = 1.0;
	vS[0].dTorqueConstant = 1.337;
	vS[0].dCurrentRatio = 1000.0;
	vS[0].nHomePositionOffset = -12345;
	vS[0].dTorLimitU = 0.5;
	vS[1].uVendorID = 0x0000089a;
	vS[1].uProductCode = 0x30000000;
	vS[1].nSlaveType = 1;
	const uint32_t uCrc = FwCfgCrc(stM, vS);
	Expect(uCrc == 0x86BCF14Cu, "the check value of a bus: as packed and summed by another program");

	// every value counts, a name does not
	bool bOk = true;
	int nValues = 0;
	const auto Differs = [&](const ST_CONFIG_ECAT_MASTER& astM, const VEC_ECAT_SLAVE_CONF& avS) {
		bOk = bOk && FwCfgCrc(astM, avS) != uCrc;
		nValues++;
	};
	{ ST_CONFIG_ECAT_MASTER m = stM; m.nCycleTime++; Differs(m, vS); }
	{ ST_CONFIG_ECAT_MASTER m = stM; m.nSendDeadline++; Differs(m, vS); }
	{ ST_CONFIG_ECAT_MASTER m = stM; m.nSyncMode = eSyncFreeRun; Differs(m, vS); }
	{ ST_CONFIG_ECAT_MASTER m = stM; m.nDcSync0Shift++; Differs(m, vS); }
	{ VEC_ECAT_SLAVE_CONF v = vS; v.pop_back(); Differs(stM, v); }
	{ VEC_ECAT_SLAVE_CONF v = vS; v[0].uVendorID++; Differs(stM, v); }
	{ VEC_ECAT_SLAVE_CONF v = vS; v[0].uProductCode++; Differs(stM, v); }
	{ VEC_ECAT_SLAVE_CONF v = vS; v[0].nSlaveType = 1; Differs(stM, v); }
	{ VEC_ECAT_SLAVE_CONF v = vS; v[0].bCCW = FALSE; Differs(stM, v); }
	{ VEC_ECAT_SLAVE_CONF v = vS; v[0].nDriveMode = 8; Differs(stM, v); }
	{ VEC_ECAT_SLAVE_CONF v = vS; v[0].dEncResolution = 524289.0; Differs(stM, v); }
	{ VEC_ECAT_SLAVE_CONF v = vS; v[0].dGearRatio = 1.0000000000000002; Differs(stM, v); }
	{ VEC_ECAT_SLAVE_CONF v = vS; v[0].dTransRatio = 2.0; Differs(stM, v); }
	{ VEC_ECAT_SLAVE_CONF v = vS; v[0].dTorqueConstant = 1.338; Differs(stM, v); }
	{ VEC_ECAT_SLAVE_CONF v = vS; v[0].dCurrentRatio = 999.0; Differs(stM, v); }
	{ VEC_ECAT_SLAVE_CONF v = vS; v[0].nHomePositionOffset++; Differs(stM, v); }
	{ VEC_ECAT_SLAVE_CONF v = vS; v[0].dTorLimitU = 0.6; Differs(stM, v); }
	{ VEC_ECAT_SLAVE_CONF v = vS; std::swap(v[0], v[1]); Differs(stM, v); }
	Expect(bOk && nValues == 18, "every value of the bus, the number of slaves and their order change the check value");
	{
		VEC_ECAT_SLAVE_CONF v = vS;
		v[0].strName = "another name";
		ST_CONFIG_ECAT_MASTER m = stM;
		m.nNoOfSlaves = 7;			// of the configuration; the slaves of the bus are counted
		Expect(FwCfgCrc(m, v) == uCrc, "a name, and SLAVENUMBER of the configuration beside a bus file, do not");
	}

	const std::string str = Header(stM, vS);
	Expect(str.find("#define FW_CFG_CRC               0x86BCF14Cu") != std::string::npos &&
	       str.find("#define FW_CFG_SLAVES            2\n") != std::string::npos &&
	       str.find("#define FW_CFG_SYNC_MODE         2\t") != std::string::npos &&
	       str.find("#define FW_CFG_DC_SYNC0_SHIFT_NS 125000\n") != std::string::npos &&
	       str.find("\t{ 0x00007595u, 0x00000000u, 0, 1, 10, 524288, 1, 1, 1.337, 1000, -12345, 0.5 },") != std::string::npos &&
	       str.find("\t{ 0x0000089Au, 0x30000000u, 1, 0, 10, 0, 0, 0, 0, 0, 0, 0 },") != std::string::npos,
	       "the header carries the check value and the values");
	printf("%s: %d checks failed\n", g_nFailed == 0 ? "PASS" : "FAIL", g_nFailed);
	return g_nFailed == 0 ? 0 : 1;
}

} // namespace

int main(int argc, char** argv)
{
	if (argc == 2 && !strcmp(argv[1], "--check"))
		return Check();

	TSTRING strCfg, strBus, strOut;
	int nOpt;
	while ((nOpt = getopt(argc, argv, "f:B:o:")) != -1)
	{
		if (nOpt == 'f') strCfg = optarg;
		else if (nOpt == 'B') strBus = optarg;
		else if (nOpt == 'o') strOut = optarg;
		else
		{
			fprintf(stderr, "usage: %s [-f FILE] [-B FILE] [-o HEADER]\n       %s --check\n", argv[0], argv[0]);
			return 2;
		}
	}
	CConfig cConfig;
	strCfg = CConfig::Find(strCfg, argv[0]);
	if (FALSE == cConfig.Load(strCfg, FALSE) || (!strBus.empty() && FALSE == cConfig.LoadBus(strBus)))
		return 1;
	if (cConfig.BusSlaves().empty())
	{
		fprintf(stderr, "%s: no slave on the bus\n", strCfg.c_str());
		return 1;
	}
	const std::string str = Header(cConfig.Master(), cConfig.BusSlaves());
	FILE* const pf = strOut.empty() ? stdout : fopen(strOut.c_str(), "w");
	if (pf == NULL)
	{
		perror(strOut.c_str());
		return 1;
	}
	const bool bOk = fputs(str.c_str(), pf) >= 0 && (pf == stdout ? fflush(pf) : fclose(pf)) == 0;
	if (!bOk)
		perror(strOut.empty() ? "the standard output" : strOut.c_str());
	else
		fprintf(stderr, "[fw_cfg] %s%s%s: check value 0x%08X, %u slaves\n", strCfg.c_str(), strBus.empty() ? "" : " and ",
		        strBus.c_str(), (unsigned)FwCfgCrc(cConfig.Master(), cConfig.BusSlaves()), (unsigned)cConfig.BusSlaves().size());
	return bOk ? 0 : 1;
}
