/*
 * axis_control.c  --  one 1 kHz cycle's axis logic: the CiA402 state machine and the torque of the axes (CST).
 *
 * The controlword is worked out again in every cycle from (servo requested, state of the drive), so a request
 * that changes at any time is followed at once.
 * - Servo off (the state after the bring-up): CW_DISABLE_VOLTAGE, the power stages stay off.
 * - Servo on: SHUTDOWN -> SWITCH_ON -> ENABLE_OPERATION, each step when the drive reports the state before it.
 * - A drive in FAULT gets a fault reset, at most MAX_FAULT_RESETS times per servo-on (IgH reset forever).  When the
 *   reset worked and the servo is still requested, the drive is enabled again, as before.
 * - Torque: an axis gets the torque of the Torque slot only while it is OPERATION_ENABLED, the servo is requested and
 *   the APU marked the slot valid and enabled; otherwise 0.  A cycle without a new slot repeats the last torque (D-03);
 *   TORQUE_STALE_LIMIT such cycles in a row while the servo is requested switch the servo off: torque 0 and
 *   CW_DISABLE_VOLTAGE.  The servo stays off until the APU asks for servo on again (D-10).  The stop does not wait
 *   for every axis to be enabled: with one axis in FAULT, or one that does not come up, the others are enabled and
 *   would keep the last torque of an APU that is gone.  So the APU writes the Torque slot in every cycle from the
 *   servo-on command on, also while the drives are still on their way to OPERATION_ENABLED.
 * - target_position follows position_actual: CST ignores it, and a drive switched to CSP would not jump.
 */
#include <stdint.h>
#include "app_config.h"
#include "axis_control.h"
#include "cia402_drive.h"
#include "robot_target.h"
#include "event_log.h"

static int servo_requested;
static int axis_state_prev[AXIS_COUNT];     /* CiA402 state (CIA402_*) of each axis in the last cycle */
static int fault_resets[AXIS_COUNT];        /* resets used since the last servo-on */
static int fault_reset_allowed[AXIS_COUNT]; /* the fault the axis is in now may be reset */

static int    torque_fresh, torque_usable;  /* the Torque slot of this cycle */
static double torque_nm[AXIS_COUNT];
static uint32_t stale_run, stale_total;     /* stale cycles in a row / so far */

void axis_control_servo(int on)
{
    int i;

    if (on && !servo_requested) {
        for (i = 0; i < AXIS_COUNT; i++) fault_resets[i] = 0;
        stale_run = 0;
    }
    servo_requested = on;
}

int axis_control_servo_requested(void)
{
    return servo_requested;
}

void axis_control_torque_input(int fresh, int usable, const double tau_nm[AXIS_COUNT])
{
    int i;

    torque_fresh  = fresh;
    torque_usable = usable;
    for (i = 0; i < AXIS_COUNT; i++) torque_nm[i] = tau_nm[i];
}

void axis_control_torque_zero(void)
{
    int i;

    for (i = 0; i < AXIS_COUNT; i++) drive_out[i]->target_torque = 0;
}

uint32_t axis_control_torque_stale_total(void)
{
    return stale_total;
}

static unsigned short next_controlword(int axis, int state, unsigned short current)
{
    if (!servo_requested) return CW_DISABLE_VOLTAGE;
    switch (state) {
    case CIA402_SWITCH_ON_DISABLED:  return CW_SHUTDOWN;
    case CIA402_READY_TO_SWITCH_ON:  return CW_SWITCH_ON;
    case CIA402_SWITCHED_ON:
    case CIA402_OPERATION_ENABLED:   return CW_ENABLE_OPERATION;
    case CIA402_FAULT:               return fault_reset_allowed[axis] ? CW_FAULT_RESET : CW_DISABLE_VOLTAGE;
    default:                         return current;   /* not ready, quick stop, fault reaction: the drive moves on by itself */
    }
}

/* the stale-torque rule: called once per cycle, after the states of this cycle are known.  all_enabled only says
 * which cycles the State slot counts (as the APU application counts them); the stop itself looks at the request. */
static void check_torque_stale(int all_enabled)
{
    if (!torque_fresh && all_enabled) stale_total++;
    if (torque_fresh || !servo_requested) {
        stale_run = 0;
        return;
    }
    if (++stale_run == TORQUE_STALE_LIMIT) {
        servo_requested = 0;
        event_post(EVENT_TORQUE_STALE_STOP, 0, (int)stale_run, 0, 0);
    }
}

void axis_control_update(void)
{
    int state[AXIS_COUNT];
    int i, enabled = 0;

    for (i = 0; i < AXIS_COUNT; i++) {
        state[i] = cia402_state_from_statusword(drive_in[i]->statusword);
        if (state[i] == CIA402_OPERATION_ENABLED) enabled++;

        if (state[i] != axis_state_prev[i]) {
            event_post(EVENT_AXIS_STATE_CHANGED, 1, i, drive_in[i]->statusword, state[i]);
            if (state[i] == CIA402_FAULT) {
                fault_reset_allowed[i] = fault_resets[i] < MAX_FAULT_RESETS;
                if (fault_reset_allowed[i]) fault_resets[i]++;
                else event_post(EVENT_FAULT_RESET_LIMIT, 0, i, 0, 0);
            }
            axis_state_prev[i] = state[i];
        }
    }

    check_torque_stale(enabled == AXIS_COUNT);

    for (i = 0; i < AXIS_COUNT; i++) {
        int give_torque = servo_requested && torque_usable && state[i] == CIA402_OPERATION_ENABLED;

        drive_out[i]->modes_of_operation = MODE_CST;
        drive_out[i]->target_position    = drive_in[i]->position_actual;
        drive_out[i]->target_torque      = give_torque ? axis_raw_torque(i, torque_nm[i]) : 0;
        drive_out[i]->controlword        = next_controlword(i, state[i], drive_out[i]->controlword);
    }
}
