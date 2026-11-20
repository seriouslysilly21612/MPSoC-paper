/*
 * ctrl_model.h  --  the RBDL model of the controller on the R5.  There is no file system and no URDF reader here:
 * the model is the one that the reader of the application built from models/indy7.urdf, kept as numbers
 * (ctrl_model_indy7.h, written by goal-to-torque_MPSoC/tools/rbdl_model_code) and added to an empty model body
 * by body.  test/check_model.cpp compares the result with the reader's on a PC.
 */
#ifndef CTRL_MODEL_H
#define CTRL_MODEL_H

/* a moving body on a joint of one axis; E by rows */
typedef struct
{
    unsigned    parent;
    double      E[9], r[3];         /* the joint frame in the parent */
    double      axis[6];
    double      mass, com[3], inertia[9];
    const char *name;
} ctrl_model_body_t;

/* a body fixed to a moving one (a frame: its mass is already in that body) */
typedef struct
{
    unsigned    parent;
    double      E[9], r[3];
    const char *name;
} ctrl_model_fixed_t;

#ifdef __cplusplus
#include <rbdl/rbdl.h>

/* adds the bodies to an empty model; false if it is not empty */
bool ctrl_model_build(RigidBodyDynamics::Model *model);

/* the check value of the numbers and names (CTRL_MODEL_CRC of ctrl_model_indy7.h) */
unsigned long ctrl_model_check_value(void);
#endif

#endif /* CTRL_MODEL_H */
