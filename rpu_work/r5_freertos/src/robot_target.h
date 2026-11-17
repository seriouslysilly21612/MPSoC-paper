/*
 * robot_target.h  --  the constants of the drives this build talks to, and the conversions between drive units
 * and SI units (eq. (2) of the paper, as in Indy7Joint.cpp of the APU controller).  Which table is used is
 * ROBOT_TARGET in app_config.h.  The values are those of INDY7.cfg [AXISn] and bench/L7N4_bus.cfg.
 */
#ifndef ROBOT_TARGET_H
#define ROBOT_TARGET_H

#include <math.h>
#include <stdint.h>
#include "app_config.h"

typedef struct {
    int32_t dir;               /* +1 (MOVE_CCW = 1) or -1 (MOVE_CCW = 0) */
    double  encoder_res;       /* ENCODER_RESOLUTION: counts per motor turn */
    double  gear;              /* GEAR_RATIO */
    double  kt;                /* TORQUE_CONSTANT: N m per A (L7N: the rated torque, N m) */
    double  kappa;             /* CURRENT_RATIO: drive units per A (L7N: 1000, units of 0.1 % of rated) */
    int32_t home;              /* HOME_POSITION_OFFSET: the encoder count at q = 0 */
    double  torque_limit_nm;   /* TORQUE_LIMIT_U */
} axis_params_t;

extern const axis_params_t axis_params[AXIS_COUNT];

/* 0x6064 position [counts] -> joint angle [rad] */
static inline double axis_q_rad(int axis, int32_t position)
{
    const axis_params_t *p = &axis_params[axis];
    return 6.283185307179586 * ((double)(position - p->home) / p->encoder_res / p->gear) * (double)p->dir;
}

/* 0x606C velocity [counts/s] -> joint velocity [rad/s] */
static inline double axis_qd_rads(int axis, int32_t velocity)
{
    const axis_params_t *p = &axis_params[axis];
    return 6.283185307179586 * ((double)velocity / p->encoder_res / p->gear) * (double)p->dir;
}

/* torque [N m] -> 0x6071 [drive units]: eq. (2) inverted, as Indy7Joint::MoveTorque, plus two limits that source does not
 * have: |tau| is cut at TORQUE_LIMIT_U, and the result is cut to the int16 of the object (the C++ cast would wrap).
 * A value that is not a number gives 0.  The fraction is dropped, as there. */
static inline int16_t axis_raw_torque(int axis, double tau_nm)
{
    const axis_params_t *p = &axis_params[axis];
    double raw;

    if (!isfinite(tau_nm)) return 0;
    if (tau_nm >  p->torque_limit_nm) tau_nm =  p->torque_limit_nm;
    if (tau_nm < -p->torque_limit_nm) tau_nm = -p->torque_limit_nm;
    raw = tau_nm / p->gear / p->kt * p->kappa * (double)p->dir;
    if (raw >  32767.0) raw =  32767.0;
    if (raw < -32768.0) raw = -32768.0;
    return (int16_t)raw;
}

/* 0x6077 [drive units] -> joint torque [N m] (eq. (2)) */
static inline double axis_tau_nm(int axis, int16_t raw)
{
    const axis_params_t *p = &axis_params[axis];
    return ((double)raw / p->kappa) * p->gear * p->kt * (double)p->dir;
}

#endif /* ROBOT_TARGET_H */
