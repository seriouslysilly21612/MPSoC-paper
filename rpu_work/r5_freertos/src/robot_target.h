/*
 * robot_target.h  --  the constants of the drives this build talks to, and the conversions between drive units
 * and SI units (eq. (2) of the paper, as in Indy7Joint.cpp of the APU controller).
 *
 * The constants are not typed here.  They come from the cfg of the APU controller, through the header that its
 * tool writes (goal-to-torque_MPSoC/indy7: bin/fw_cfg_header), one per target in cfg/:
 *     cfg/fw_cfg_l7n.h     bin/fw_cfg_header -B bench/L7N3_bus.cfg -o ...     ROBOT_TARGET 0
 *     cfg/fw_cfg_indy7.h   bin/fw_cfg_header -o ...                           ROBOT_TARGET 1
 * FW_CFG_CRC of that header goes into the State slot (ocm_link.c), where the APU compares it with the check value
 * of its own cfg before it asks for servo on.  When [ECAT_MASTER] or an [AXISn] of the cfg changes, write the
 * header again and build again; test/check_copies.sh tells when a header is not what the tool writes now.
 *
 * Not taken from the header: the sync mode.  The bench cfg has FREE_RUN, with which the bench's drive with firmware
 * 0.12 did not stay in OP (I-40; off the bus since 2026-11-19), so ECAT_USE_DC of app_config.h decides (to be settled
 * with D-13).
 */
#ifndef ROBOT_TARGET_H
#define ROBOT_TARGET_H

#include <math.h>
#include <stdint.h>
#include "app_config.h"

#if ROBOT_TARGET == TARGET_L7N_BENCH
#include "cfg/fw_cfg_l7n.h"
#else
#include "cfg/fw_cfg_indy7.h"
_Static_assert((FW_CFG_SYNC_MODE == 2) == (ECAT_USE_DC == 1), "ECAT_USE_DC and SYNC_MODE of the cfg disagree");
#endif

_Static_assert(FW_CFG_SLAVES == BUS_SLAVE_COUNT && AXIS_COUNT <= FW_CFG_SLAVES,
               "BUS_SLAVE_COUNT / AXIS_COUNT of app_config.h and the slaves of the cfg disagree");
_Static_assert(FW_CFG_CYCLE_NS == CYCLE_NS, "CYCLE_NS of app_config.h and CYCLE_TIME of the cfg disagree");

/* MOVE_CCW: +1 (1) or -1 (0) */
static inline double axis_dir(int axis)
{
    return FW_CFG_AXIS[axis].move_ccw ? 1.0 : -1.0;
}

/* 0x6064 position [counts] -> joint angle [rad] */
static inline double axis_q_rad(int axis, int32_t position)
{
    const fw_cfg_axis_t *p = &FW_CFG_AXIS[axis];
    return 6.283185307179586 * ((double)(position - p->home_position_offset) / p->encoder_resolution / p->gear_ratio /
                                p->transmission_ratio) * axis_dir(axis);
}

/* 0x606C velocity [counts/s] -> joint velocity [rad/s] */
static inline double axis_qd_rads(int axis, int32_t velocity)
{
    const fw_cfg_axis_t *p = &FW_CFG_AXIS[axis];
    return 6.283185307179586 * ((double)velocity / p->encoder_resolution / p->gear_ratio / p->transmission_ratio) *
           axis_dir(axis);
}

/* torque [N m] -> 0x6071 [drive units]: eq. (2) inverted, as Indy7Joint::MoveTorque, plus two limits that source does not
 * have: |tau| is cut at TORQUE_LIMIT_U, and the result is cut to the int16 of the object (the C++ cast would wrap).
 * A value that is not a number gives 0.  The fraction is dropped, as there.  (L7N: TORQUE_CONSTANT is the rated torque
 * [N m] and CURRENT_RATIO 1000, the units of 0.1 % of rated.) */
static inline int16_t axis_raw_torque(int axis, double tau_nm)
{
    const fw_cfg_axis_t *p = &FW_CFG_AXIS[axis];
    double raw;

    if (!isfinite(tau_nm)) return 0;
    if (tau_nm >  p->torque_limit_u) tau_nm =  p->torque_limit_u;
    if (tau_nm < -p->torque_limit_u) tau_nm = -p->torque_limit_u;
    raw = tau_nm / p->gear_ratio / p->torque_constant * p->current_ratio * axis_dir(axis);
    if (raw >  32767.0) raw =  32767.0;
    if (raw < -32768.0) raw = -32768.0;
    return (int16_t)raw;
}

/* 0x6077 [drive units] -> joint torque [N m] (eq. (2)) */
static inline double axis_tau_nm(int axis, int16_t raw)
{
    const fw_cfg_axis_t *p = &FW_CFG_AXIS[axis];
    return ((double)raw / p->current_ratio) * p->gear_ratio * p->torque_constant * axis_dir(axis);
}

#endif /* ROBOT_TARGET_H */
