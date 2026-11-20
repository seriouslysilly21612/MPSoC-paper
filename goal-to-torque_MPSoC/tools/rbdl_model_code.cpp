/*****************************************************************************
*	Name: rbdl_model_code.cpp
*	Description: The RBDL model of the controller as numbers in a header, for
*	a build of the controller without the URDF reader and without a file
*	system: the firmware of the R5 (rpu_work/r5_freertos/src/ctrl).
*
*	    bin/rbdl_model_code URDF [-o HEADER]
*
*	The URDF is read by the reader the controller uses (RBDL's
*	URDFReadFromFile, fixed base), and what the reader built is written out,
*	not what the file says: of every moving body its parent, joint frame,
*	joint axis and inertia (the fixed bodies merged into it included), of
*	every fixed body its moving parent and its frame. ctrl_model.cpp of the
*	firmware adds them to an empty model in this order and gets the model of
*	the reader (test/check_model.cpp compares the two). The numbers are
*	hexadecimal, so none is rounded.
*
*	Stops at a model it cannot write this way: a joint with more than one
*	degree of freedom, a body without a name.
*
*	The firmware must be built again from a new header whenever the URDF or
*	the reader changes; the header is not edited.
*****************************************************************************/
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>
#include <unistd.h>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "shadow/ModelCode.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

namespace {

/* the header of the model, or an empty string (the reason is printed) */
std::string Header(const Model& am, const char* apcUrdf)
{
	model_code::ModelCode stCode;
	if (!model_code::ModelCodeOf(am, &stCode))
		return "";
	const char* pcFile = strrchr(apcUrdf, '/') ? strrchr(apcUrdf, '/') + 1 : apcUrdf;
	char ac[640];
	snprintf(ac, sizeof(ac),
	         "/* The RBDL model of %s as the URDF reader of the controller builds it. Written by rbdl_model_code:\n"
	         " * not to be edited. Per body: parent, E and r of the joint frame, joint axis, mass, centre of mass,\n"
	         " * inertia at the centre of mass, name. Per fixed body: moving parent, E and r of its frame, name. */\n"
	         "#ifndef CTRL_MODEL_DATA_H\n#define CTRL_MODEL_DATA_H\n\n#include \"ctrl_model.h\"\n\n"
	         "#define CTRL_MODEL_CRC    0x%08Xu\t/* of the names below and of the numbers, each on a grid of 2^-40 */\n"
	         "#define CTRL_MODEL_BODIES %u\n"
	         "#define CTRL_MODEL_FIXED  %u\n\n"
	         "static const ctrl_model_body_t CTRL_MODEL_BODY[CTRL_MODEL_BODIES] =\n{\n",
	         pcFile, (unsigned)stCode.uCrc, stCode.uBodies, stCode.uFixed);
	return std::string(ac) + stCode.strBodies + "};\n\nstatic const ctrl_model_fixed_t CTRL_MODEL_FIXED_BODY[CTRL_MODEL_FIXED] =\n{\n" +
	       stCode.strFixed + "};\n\n#endif /* CTRL_MODEL_DATA_H */\n";
}

} // namespace

int main(int argc, char** argv)
{
	std::string strOut;
	int nOpt;
	while ((nOpt = getopt(argc, argv, "o:")) != -1)
	{
		if (nOpt == 'o')
			strOut = optarg;
		else
			optind = argc + 1;
	}
	if (optind + 1 != argc)
	{
		fprintf(stderr, "usage: %s URDF [-o HEADER]\n", argv[0]);
		return 2;
	}
	Model cModel;
	if (!Addons::URDFReadFromFile(argv[optind], &cModel, false, false))
	{
		fprintf(stderr, "%s: the URDF reader cannot read it\n", argv[optind]);
		return 1;
	}
	const std::string str = Header(cModel, argv[optind]);
	if (str.empty())
		return 1;
	FILE* const pf = strOut.empty() ? stdout : fopen(strOut.c_str(), "w");
	if (pf == NULL)
	{
		perror(strOut.c_str());
		return 1;
	}
	const bool bOk = fputs(str.c_str(), pf) >= 0 && (pf == stdout ? fflush(pf) : fclose(pf)) == 0;
	if (!bOk)
		perror(strOut.empty() ? "the standard output" : strOut.c_str());
	return bOk ? 0 : 1;
}
