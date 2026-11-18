/*
 * rbdl/addons/urdfreader/urdfreader.h of the firmware  --  the controller classes (copies of the application's,
 * unchanged) load their model with RBDL's URDF reader.  On the R5 this header is found in place of RBDL's: the
 * same call gives the model of ctrl_model.h, whatever the file name says.
 */
#ifndef RBDL_URDFREADER_H
#define RBDL_URDFREADER_H

#include <rbdl/rbdl.h>

#include "ctrl_model.h"

namespace RigidBodyDynamics
{
namespace Addons
{
/* the numbers are of a fixed base: a floating base is refused */
inline bool URDFReadFromFile(const char * /* filename */, Model *model, bool floating_base, bool /* verbose */ = false)
{
    return !floating_base && ctrl_model_build(model);
}
} // namespace Addons
} // namespace RigidBodyDynamics

#endif /* RBDL_URDFREADER_H */
