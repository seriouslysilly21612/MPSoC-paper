/*
 * robot_target.c  --  the per-axis constants of the target selected by ROBOT_TARGET (see robot_target.h).
 */
#include "robot_target.h"

#if ROBOT_TARGET == TARGET_L7N_BENCH
/* four identical L7N drives, bench/L7N4_bus.cfg (the 19-bit encoder resolution was not read from the drive) */
#define L7N_AXIS { -1, 524288.0, 1.0, 1.337, 1000.0, 0, 0.4 }
const axis_params_t axis_params[AXIS_COUNT] = { L7N_AXIS, L7N_AXIS, L7N_AXIS, L7N_AXIS };

#elif ROBOT_TARGET == TARGET_INDY7
/* INDY7.cfg [AXIS0] .. [AXIS5] */
const axis_params_t axis_params[AXIS_COUNT] = {
    { -1, 65536.0, 121.0, 0.0884,  48.0, 23567184, 117.7 },
    { -1, 65536.0, 121.0, 0.0884,  48.0,    28218, 117.7 },
    { +1, 65536.0, 121.0, 0.087,   96.0, 15841818,  47.5 },
    { -1, 65536.0, 101.0, 0.058,   96.0,  6161747,  21.0 },
    { -1, 65536.0, 101.0, 0.058,   96.0, 39749892,  21.0 },
    { +1, 65536.0, 101.0, 0.058,   96.0,    45545,  21.0 },
};
#endif
