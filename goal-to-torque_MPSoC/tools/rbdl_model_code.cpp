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

#include "ipc/FwCfg.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

namespace {

/* the text of the header and the check value of its numbers and names, in the order written */
class CHeader
{
public:
	void Unsigned(unsigned au)
	{
		const uint8_t a[4] = { (uint8_t)au, (uint8_t)(au >> 8), (uint8_t)(au >> 16), (uint8_t)(au >> 24) };
		m_uCrc = FwCfgCrc32(m_uCrc, a, sizeof(a));
		char ac[16];
		snprintf(ac, sizeof(ac), "%uu, ", au);
		m_str += ac;
	}
	void Doubles(const double* ad, int anCount)
	{
		m_str += "{ ";
		for (int i = 0; i < anCount; i++)
		{
			uint64_t u;
			memcpy(&u, &ad[i], sizeof(u));
			Bytes8(u);
			char ac[40];
			snprintf(ac, sizeof(ac), "%a%s", ad[i], i + 1 < anCount ? ", " : "");
			m_str += ac;
		}
		m_str += " }, ";
	}
	void Name(const std::string& astr)
	{
		m_uCrc = FwCfgCrc32(m_uCrc, astr.c_str(), astr.size() + 1);
		m_str += "\"" + astr + "\"";
	}
	void Text(const std::string& astr) { m_str += astr; }
	uint32_t Crc() const { return m_uCrc; }
	const std::string& Str() const { return m_str; }

private:
	void Bytes8(uint64_t au)
	{
		uint8_t a[8];
		for (int i = 0; i < 8; i++)
			a[i] = (uint8_t)(au >> (8 * i));
		m_uCrc = FwCfgCrc32(m_uCrc, a, sizeof(a));
	}
	uint32_t	m_uCrc = 0;
	std::string	m_str;
};

/* E (by rows) and r of a frame */
void Frame(CHeader& ac, const SpatialTransform& aX)
{
	double adE[9], adR[3];
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
			adE[3 * i + j] = aX.E(i, j);
		adR[i] = aX.r[i];
	}
	ac.Doubles(adE, 9);
	ac.Doubles(adR, 3);
}

/* the header of the model, or an empty string with the reason on the standard error */
std::string Header(const Model& am, const char* apcUrdf)
{
	std::map<unsigned int, std::string> mapName;
	for (const auto& kv : am.mBodyNameMap)
		mapName[kv.second] = kv.first;
	const unsigned uBodies = (unsigned)am.mBodies.size() - 1;		// body 0 is the base
	const unsigned uFixed = (unsigned)am.mFixedBodies.size();

	CHeader cBodies, cFixed;
	for (unsigned i = 1; i <= uBodies; i++)
	{
		const Joint& j = am.mJoints[i];
		if (j.mDoFCount != 1 || mapName.find(i) == mapName.end())
		{
			fprintf(stderr, "body %u: %u degrees of freedom or no name; only named bodies on one-axis joints are written\n",
			        i, j.mDoFCount);
			return "";
		}
		const Body& b = am.mBodies[i];
		double adAxis[6], adCom[3], adI[9];
		for (int k = 0; k < 6; k++)
			adAxis[k] = j.mJointAxes[0][k];
		for (int k = 0; k < 3; k++)
		{
			adCom[k] = b.mCenterOfMass[k];
			for (int l = 0; l < 3; l++)
				adI[3 * k + l] = b.mInertia(k, l);
		}
		cBodies.Text("\t{ ");
		cBodies.Unsigned(am.lambda[i]);
		Frame(cBodies, am.X_T[i]);
		cBodies.Doubles(adAxis, 6);
		cBodies.Doubles(&b.mMass, 1);
		cBodies.Doubles(adCom, 3);
		cBodies.Doubles(adI, 9);
		cBodies.Name(mapName[i]);
		cBodies.Text(" },\n");
	}
	for (unsigned i = 0; i < uFixed; i++)
	{
		const unsigned uId = am.fixed_body_discriminator + i;
		if (mapName.find(uId) == mapName.end())
		{
			fprintf(stderr, "fixed body %u has no name\n", i);
			return "";
		}
		cFixed.Text("\t{ ");
		cFixed.Unsigned(am.mFixedBodies[i].mMovableParent);
		Frame(cFixed, am.mFixedBodies[i].mParentTransform);
		cFixed.Name(mapName[uId]);
		cFixed.Text(" },\n");
	}
	const uint32_t auCrc[2] = { cBodies.Crc(), cFixed.Crc() };
	const char* pcFile = strrchr(apcUrdf, '/') ? strrchr(apcUrdf, '/') + 1 : apcUrdf;
	char ac[640];
	snprintf(ac, sizeof(ac),
	         "/* The RBDL model of %s as the URDF reader of the controller builds it. Written by rbdl_model_code:\n"
	         " * not to be edited. Per body: parent, E and r of the joint frame, joint axis, mass, centre of mass,\n"
	         " * inertia at the centre of mass, name. Per fixed body: moving parent, E and r of its frame, name. */\n"
	         "#ifndef CTRL_MODEL_DATA_H\n#define CTRL_MODEL_DATA_H\n\n#include \"ctrl_model.h\"\n\n"
	         "#define CTRL_MODEL_CRC    0x%08Xu\t/* of the numbers and names below */\n"
	         "#define CTRL_MODEL_BODIES %u\n"
	         "#define CTRL_MODEL_FIXED  %u\n\n"
	         "static const ctrl_model_body_t CTRL_MODEL_BODY[CTRL_MODEL_BODIES] =\n{\n",
	         pcFile, (unsigned)FwCfgCrc32(0, auCrc, sizeof(auCrc)), uBodies, uFixed);
	return std::string(ac) + cBodies.Str() + "};\n\nstatic const ctrl_model_fixed_t CTRL_MODEL_FIXED_BODY[CTRL_MODEL_FIXED] =\n{\n" +
	       cFixed.Str() + "};\n\n#endif /* CTRL_MODEL_DATA_H */\n";
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
