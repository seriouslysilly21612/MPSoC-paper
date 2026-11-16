/*
 * axis_control.c  --  one 1 kHz cycle's axis logic: CiA402 state machine + trajectory
 * (translation of the IgH cyclic_task() body; the CiA402 and sine logic are REUSED).
 *
 * - target_position follows position_actual until an axis is OPERATION_ENABLED (no step when it enables);
 *   the sine starts only after ALL axes were enabled for SINE_START_DELAY_MS (brake release).
 * - If an axis leaves OPERATION_ENABLED after the sine started (fault, quick stop, ...) the sine stops for
 *   good: restarting it after the fault reset, from a stale phase, would be a step of up to half a turn.
 *   Automatic fault resets are limited to MAX_FAULT_RESETS per axis (IgH reset forever).
 */
#include <stdint.h>
#include "app_config.h"
#include "axis_control.h"
#include "cia402_drive.h"
#include "event_log.h"

static int axis_state[AXIS_COUNT]      = {0};   /* CiA402 state (CIA402_*) of each axis in this cycle */
static int axis_state_prev[AXIS_COUNT] = {0};
/* (some of these are only used with RUN_LEVEL >= 1 / 2: "unused" keeps -Wall quiet at every level) */
static int hold_position[AXIS_COUNT]     __attribute__((unused));   /* position the axis had when it was enabled */
static int fault_reset_count[AXIS_COUNT] __attribute__((unused));
static uint32_t all_enabled_cycles       __attribute__((unused));   /* consecutive cycles with ALL axes enabled */
static uint32_t sine_step                __attribute__((unused));   /* sine phase, 0 .. SINE_PERIOD_CYCLES-1 */
int sine_started;
int sine_stopped;

/* ----------------------------------------------------------------------- */
/* sin() without libm.  k = 0 .. SINE_PERIOD_CYCLES-1 is one full period.   */
/* Octant reduction by symmetry, then a Taylor series on 0..pi/2 (error < 1e-9). */
/* ----------------------------------------------------------------------- */
static double sine_of_step(uint32_t k) __attribute__((unused));   /* only called with RUN_LEVEL >= 2 */
static double sine_of_step(uint32_t k)
{
    const uint32_t half    = SINE_PERIOD_CYCLES / 2u;
    const uint32_t quarter = SINE_PERIOD_CYCLES / 4u;
    double sign = 1.0, x, x2;

    if (k >= half)    { k -= half; sign = -1.0; }     /* sin(t + pi) = -sin(t) */
    if (k > quarter)  { k = half - k; }               /* sin(pi - t) =  sin(t) */
    x  = 3.14159265358979323846 * (double)k / (double)half;
    x2 = x * x;
    return sign * x * (1.0 + x2 * (-1.0 / 6.0 + x2 * (1.0 / 120.0 + x2 * (-1.0 / 5040.0
                 + x2 * (1.0 / 362880.0 + x2 * (-1.0 / 39916800.0 + x2 * (1.0 / 6227020800.0)))))));
}

void axis_control_update(void)
{
    int i, axes_enabled = 0;

    for (i = 0; i < AXIS_COUNT; i++) {
        axis_state[i] = cia402_state_from_statusword(drive_in[i]->statusword);
        if (axis_state[i] == CIA402_OPERATION_ENABLED) axes_enabled++;

        /* until the axis is enabled its target follows its actual position: no step when it enables */
        if (axis_state[i] != CIA402_OPERATION_ENABLED) {
            drive_out[i]->target_position = drive_in[i]->position_actual;
        }
        if (axis_state_prev[i] == axis_state[i]) continue;

        event_post(EVENT_AXIS_STATE_CHANGED, 1, i, drive_in[i]->statusword, axis_state[i]);
#if RUN_LEVEL >= RUN_SERVO_HOLD
        switch (axis_state[i]) {
        case CIA402_SWITCH_ON_DISABLED:
            drive_out[i]->controlword = CW_SHUTDOWN;           break;
        case CIA402_READY_TO_SWITCH_ON:
            drive_out[i]->controlword = CW_SWITCH_ON;          break;
        case CIA402_SWITCHED_ON:
            drive_out[i]->controlword = CW_ENABLE_OPERATION;   break;
        case CIA402_OPERATION_ENABLED:
            hold_position[i] = drive_in[i]->position_actual;   /* hold the position the axis has right now */
            drive_out[i]->target_position = hold_position[i];
            break;
        case CIA402_FAULT:
            if (fault_reset_count[i] < MAX_FAULT_RESETS) {
                fault_reset_count[i]++;
                drive_out[i]->controlword = CW_FAULT_RESET;
            } else {
                event_post(EVENT_FAULT_RESET_LIMIT, 0, i, 0, 0);
            }
            break;
        default:
            break;
        }
#endif
        axis_state_prev[i] = axis_state[i];
    }

#if RUN_LEVEL >= RUN_SINE
    if (axes_enabled == AXIS_COUNT) {
        if (all_enabled_cycles < 0xFFFFFFFFu) all_enabled_cycles++;
    } else {
        all_enabled_cycles = 0U;
    }

    /* an axis dropped out after the sine began (fault, quick stop, ...): stop for good */
    if (sine_started && !sine_stopped && axes_enabled < AXIS_COUNT) {
        int dropped = 0;
        sine_stopped = 1;
        while (dropped < AXIS_COUNT - 1 && axis_state[dropped] == CIA402_OPERATION_ENABLED) dropped++;
        event_post(EVENT_SINE_STOPPED, 0, dropped, drive_in[dropped]->statusword, 0);
    }
    /* all axes enabled for a while (brakes released): cyclic synchronous position = sine trajectory */
    if (!sine_started && !sine_stopped && all_enabled_cycles >= SINE_START_DELAY_MS) {
        sine_started = 1;
        sine_step = 0U;
        event_post(EVENT_SINE_STARTED, 0, 0, 0, 0);
    }
    if (sine_started && !sine_stopped) {
        int j;
        double sine = sine_of_step(sine_step);
        for (j = 0; j < AXIS_COUNT; j++) {
            drive_out[j]->modes_of_operation = MODE_CSP;
            drive_out[j]->target_position    = hold_position[j] + (int)(SINE_AMPLITUDE * sine);
        }
        sine_step = (sine_step + 1U) % SINE_PERIOD_CYCLES;      /* 1 ms step */
    }
#else
    (void)axes_enabled;
#endif
}
