/*
 * axis_control.h  --  what one 1 kHz cycle does with the axes: the CiA402 state machine (enable, fault reset), the
 * torque the APU asked for, and the stop when that torque stops coming.  Nothing moves by itself: the axes are
 * enabled only after axis_control_servo(1), which the APU asks for with the Command slot (ocm_link.h), and they get
 * a torque only from the Torque slot.
 */
#ifndef AXIS_CONTROL_H
#define AXIS_CONTROL_H

#include <stdint.h>
#include "app_config.h"

void axis_control_servo(int on);          /* 1: bring every axis to OPERATION_ENABLED; 0: power stages off */
int  axis_control_servo_requested(void);  /* what was asked for last (not what the drives did) */

/* The torque of this cycle, from the Torque slot (ocm_link_take_torque).  fresh: the slot changed since the last
 * cycle.  usable: the APU marked it valid and enabled.  tau_nm: the last fresh values, also on a stale cycle. */
void axis_control_torque_input(int fresh, int usable, const double tau_nm[AXIS_COUNT]);

void axis_control_update(void);           /* call it only on a cycle with a complete working counter */

/* Torque 0 on every axis, now.  For the cycles in which update() does not run (a stop, a lost frame). */
void axis_control_torque_zero(void);

uint32_t axis_control_torque_stale_total(void);   /* cycles with the drives on and no fresh torque, so far */

#endif /* AXIS_CONTROL_H */
