/*****************************************************************************
*	Name: ctrl_numbers_header.cpp
*	Description: The constants of the controller for the firmware of the R5
*	(--shadow): the numbers that the second controller is built from
*	(SHADOW_NUMBERS, shadow/ShadowParams.h), read from the configuration by
*	the parser of the controller, as a C header with their check value.
*
*	    bin/ctrl_numbers_header [-f FILE] -o HEADER
*
*	  -f FILE    the configuration, found as by indy7_ctrl
*	  -o HEADER  the header
*
*	The numbers are hexadecimal, so none is rounded; the comment of each has
*	its value and what it is. The firmware must be built again from a new
*	header whenever one of them changes in the configuration (a gain, a
*	torque limit, a parameter of the high-level controller); the header is
*	not edited. rpu_work/r5_freertos/test/shadow_replay.cpp checks that the
*	header in the firmware has the numbers the application computes.
*****************************************************************************/
#include <cstdio>
#include <string>
#include <unistd.h>

#include "base/Config.h"
#include "shadow/ShadowParams.h"

namespace {

/* what number i is */
std::string Name(int i)
{
	static const struct { int nFirst; const char* pcWhat; } s_ast[] = {
		{ SHADOW_N_KP, "Kp" }, { SHADOW_N_KD, "Kd" }, { SHADOW_N_KF, "friction feedforward [N m]" },
		{ SHADOW_N_ROTOR, "rotor inertia [kg m^2]" }, { SHADOW_N_TAU_MAX, "torque limit [N m]" },
		{ SHADOW_N_READY, "ready posture [rad]" } };
	static const char* const s_apcOne[] = {
		"gate of the friction feedforward (0, 1)", "goal threshold [m]", "IK damping", "IK step limit [rad]",
		"IK tolerance [m]", "periods from the request of a plan to its use", "IK iterations per goal", "speed limit [rad/s]",
		"acceleration limit [rad/s^2]", "jerk limit [rad/s^3]", "limit margin [rad]", "lead bound [rad]",
		"step baseline (0, 1)" };
	if (i == SHADOW_N_TS)
		return "the period [s]";
	if (i >= SHADOW_N_FF_GATE)
		return s_apcOne[i - SHADOW_N_FF_GATE];
	const int nGroup = (i - SHADOW_N_KP) / IPC_CTRL_DOF;
	return std::string(s_ast[nGroup].pcWhat) + ", joint " + std::to_string(i - s_ast[nGroup].nFirst + 1);
}

} // namespace

int main(int argc, char** argv)
{
	TSTRING strCfg, strOut;
	int nOpt;
	while ((nOpt = getopt(argc, argv, "f:o:")) != -1)
	{
		if (nOpt == 'f') strCfg = optarg;
		else if (nOpt == 'o') strOut = optarg;
		else
			strOut.clear();
	}
	if (strOut.empty())
	{
		fprintf(stderr, "usage: %s [-f FILE] -o HEADER\n", argv[0]);
		return 2;
	}
	CConfig cConfig;
	strCfg = CConfig::Find(strCfg, argv[0]);
	if (FALSE == cConfig.Load(strCfg, FALSE))
		return 2;
	double adNumbers[SHADOW_NUMBERS];
	if (!ShadowNumbers(cConfig, adNumbers))
	{
		fprintf(stderr, "%s: not the arm of the controller (%d joints), or no control task\n", strCfg.c_str(), IPC_CTRL_DOF);
		return 1;
	}
	FILE* const pf = fopen(strOut.c_str(), "w");
	if (pf == NULL)
	{
		perror(strOut.c_str());
		return 2;
	}
	fprintf(pf, "/* The numbers of the controller (SHADOW_NUMBERS of ShadowControl.h), from the configuration of the\n"
	            " * controller. Written by ctrl_numbers_header: not to be edited. */\n"
	            "#ifndef CTRL_NUMBERS_H\n#define CTRL_NUMBERS_H\n\n"
	            "#define CTRL_NUMBERS_CRC   0x%08Xu\t/* of the numbers below */\n"
	            "#define CTRL_NUMBERS_COUNT %d\n\n"
	            "static const double CTRL_NUMBERS[CTRL_NUMBERS_COUNT] =\n{\n",
	        (unsigned)ShadowNumbersCrc(adNumbers), (int)SHADOW_NUMBERS);
	for (int i = 0; i < SHADOW_NUMBERS; i++)
		fprintf(pf, "\t%a,\t/* %.10g: %s */\n", adNumbers[i], adNumbers[i], Name(i).c_str());
	fprintf(pf, "};\n\n#endif /* CTRL_NUMBERS_H */\n");
	if (fclose(pf) != 0)
	{
		perror(strOut.c_str());
		return 2;
	}
	return 0;
}
