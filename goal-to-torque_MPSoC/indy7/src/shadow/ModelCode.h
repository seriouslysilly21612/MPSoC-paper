/*****************************************************************************
*	Name: ModelCode.h
*	Description: The RBDL model of the controller as numbers, in the order
*	and form of the firmware's header (rpu_work/r5_freertos/src/ctrl/
*	ctrl_model_indy7.h), with their check value. tools/rbdl_model_code.cpp
*	writes the header from it; the application takes the check value of its
*	own model from it, to tell a firmware that was built from another one
*	(--shadow).
*
*	What the URDF reader built is taken, not what the file says: of every
*	moving body its parent, joint frame, joint axis and inertia (the fixed
*	bodies merged into it included), of every fixed body its moving parent
*	and its frame.
*
*	The header has every number as the reader of the PC gave it. The check
*	value takes each on a grid of 2^-40 (9.1e-13): the reader of the board
*	(aarch64) gives four numbers of this arm, entries of joint frames that
*	would be 0 in exact arithmetic, 1.6e-23 away from those of the PC (x86),
*	and with the exact numbers the application on the board took every
*	firmware for one of another arm (2026-11-19).
*****************************************************************************/
#ifndef G2T_MODEL_CODE_H
#define G2T_MODEL_CODE_H

#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>

#include <rbdl/rbdl.h>

#include "ipc/FwCfg.h"

namespace model_code {

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

/* the text of the header and the check value of its numbers and names, in the order written */
class CModelText
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
			Bytes8(Grid(ad[i]));
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
	/* the grid point of a number, as the check value takes it */
	static uint64_t Grid(double ad) { return (uint64_t)std::llround(std::ldexp(ad, 40)); }
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
inline void ModelFrame(CModelText& ac, const SpatialTransform& aX)
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

/* the model as the firmware's header has it: the lines of its two tables, their counts, and the check value of the
 * numbers and names (CTRL_MODEL_CRC). false: a model that cannot be written this way; the reason is printed. */
struct ModelCode
{
	std::string	strBodies, strFixed;
	unsigned	uBodies, uFixed;
	uint32_t	uCrc;
};

inline bool ModelCodeOf(const Model& am, ModelCode* apCode)
{
	std::map<unsigned int, std::string> mapName;
	for (const auto& kv : am.mBodyNameMap)
		mapName[kv.second] = kv.first;
	const unsigned uBodies = (unsigned)am.mBodies.size() - 1;		// body 0 is the base
	const unsigned uFixed = (unsigned)am.mFixedBodies.size();

	CModelText cBodies, cFixed;
	for (unsigned i = 1; i <= uBodies; i++)
	{
		const Joint& j = am.mJoints[i];
		if (j.mDoFCount != 1 || mapName.find(i) == mapName.end())
		{
			fprintf(stderr, "body %u: %u degrees of freedom or no name; only named bodies on one-axis joints are written\n",
			        i, j.mDoFCount);
			return false;
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
		ModelFrame(cBodies, am.X_T[i]);
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
			return false;
		}
		cFixed.Text("\t{ ");
		cFixed.Unsigned(am.mFixedBodies[i].mMovableParent);
		ModelFrame(cFixed, am.mFixedBodies[i].mParentTransform);
		cFixed.Name(mapName[uId]);
		cFixed.Text(" },\n");
	}
	const uint32_t auCrc[2] = { cBodies.Crc(), cFixed.Crc() };
	apCode->strBodies = cBodies.Str();
	apCode->strFixed = cFixed.Str();
	apCode->uBodies = uBodies;
	apCode->uFixed = uFixed;
	apCode->uCrc = FwCfgCrc32(0, auCrc, sizeof(auCrc));
	return true;
}

} // namespace model_code

#endif // G2T_MODEL_CODE_H
