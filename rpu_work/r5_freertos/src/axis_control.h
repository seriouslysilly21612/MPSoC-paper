/*
 * axis_control.h  --  what one 1 kHz cycle does with the axes: the CiA402 state machine (enable, fault reset),
 * hold the position, the sine trajectory.  How far it goes is RUN_LEVEL (app_config.h).
 */
#ifndef AXIS_CONTROL_H
#define AXIS_CONTROL_H

extern int sine_started;   /* the sine began */
extern int sine_stopped;   /* latched: an axis dropped out afterwards */

void axis_control_update(void);   /* call it only on a cycle with a complete working counter */

#endif /* AXIS_CONTROL_H */
