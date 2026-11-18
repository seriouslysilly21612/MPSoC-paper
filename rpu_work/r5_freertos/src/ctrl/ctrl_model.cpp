/*
 * ctrl_model.cpp  --  see ctrl_model.h.  The bodies go in with the calls the URDF reader makes (Model::AddBody),
 * in its order, with the numbers it used, so RBDL derives everything else as it did for the reader.  A fixed
 * body is added without mass: the reader has merged its mass into the moving body, and the numbers of that body
 * are the merged ones.
 */
#include "ctrl_model.h"

#include "ctrl_model_indy7.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

static SpatialTransform frame(const double *E, const double *r)
{
    return SpatialTransform(Matrix3d(E[0], E[1], E[2], E[3], E[4], E[5], E[6], E[7], E[8]), Vector3d(r[0], r[1], r[2]));
}

bool ctrl_model_build(Model *model)
{
    if (model->mBodies.size() != 1 || !model->mFixedBodies.empty())
        return false;
    for (unsigned i = 0; i < CTRL_MODEL_BODIES; i++)
    {
        const ctrl_model_body_t &b = CTRL_MODEL_BODY[i];
        const double *I = b.inertia;
        model->AddBody(b.parent, frame(b.E, b.r),
                       Joint(SpatialVector(b.axis[0], b.axis[1], b.axis[2], b.axis[3], b.axis[4], b.axis[5])),
                       Body(b.mass, Vector3d(b.com[0], b.com[1], b.com[2]),
                            Matrix3d(I[0], I[1], I[2], I[3], I[4], I[5], I[6], I[7], I[8])),
                       b.name);
    }
    for (unsigned i = 0; i < CTRL_MODEL_FIXED; i++)
    {
        const ctrl_model_fixed_t &f = CTRL_MODEL_FIXED_BODY[i];
        model->AddBody(f.parent, frame(f.E, f.r), Joint(JointTypeFixed), Body(), f.name);
    }
    return true;
}

unsigned long ctrl_model_check_value(void)
{
    return CTRL_MODEL_CRC;
}
