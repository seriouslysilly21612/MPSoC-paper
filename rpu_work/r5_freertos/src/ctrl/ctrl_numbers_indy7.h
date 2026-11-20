/* The numbers of the controller (SHADOW_NUMBERS of ShadowControl.h), from the configuration of the
 * controller. Written by ctrl_numbers_header: not to be edited. */
#ifndef CTRL_NUMBERS_H
#define CTRL_NUMBERS_H

#define CTRL_NUMBERS_CRC   0x087FA2D3u	/* of the numbers below */
#define CTRL_CONTRACT_CRC  0x2E7B0347u	/* of [CONTRACT] (obs/ObsContract.h): the camera side compares it */
#define CTRL_NUMBERS_COUNT 75

static const double CTRL_NUMBERS[CTRL_NUMBERS_COUNT] =
{
	0x1.0624dd2f1a9fcp-10,	/* 0.001: the period [s] */
	0x1.9p+9,	/* 800: Kp, joint 1 */
	0x1.2cp+9,	/* 600: Kp, joint 2 */
	0x1.9p+8,	/* 400: Kp, joint 3 */
	0x1.9p+7,	/* 200: Kp, joint 4 */
	0x1.2cp+7,	/* 150: Kp, joint 5 */
	0x1.9p+6,	/* 100: Kp, joint 6 */
	0x1.4p+6,	/* 80: Kd, joint 1 */
	0x1.ep+5,	/* 60: Kd, joint 2 */
	0x1.4p+5,	/* 40: Kd, joint 3 */
	0x1.cp+4,	/* 28: Kd, joint 4 */
	0x1.9p+4,	/* 25: Kd, joint 5 */
	0x1.4p+4,	/* 20: Kd, joint 6 */
	0x1.7dc28f5c28f5cp+2,	/* 5.965: friction feedforward [N m], joint 1 */
	0x1.571a9fbe76c8bp+2,	/* 5.361: friction feedforward [N m], joint 2 */
	0x1.76e978d4fdf3bp+1,	/* 2.929: friction feedforward [N m], joint 3 */
	0x1.dp+1,	/* 3.625: friction feedforward [N m], joint 4 */
	0x1.024dd2f1a9fbep+1,	/* 2.018: friction feedforward [N m], joint 5 */
	0x1.1ac083126e979p+2,	/* 4.418: friction feedforward [N m], joint 6 */
	0x1.85111f0c34c1bp+2,	/* 6.07917: rotor inertia [kg m^2], joint 1 */
	0x1.ac1dfb9389b52p+2,	/* 6.68933: rotor inertia [kg m^2], joint 2 */
	0x1.ff19f7f8ca81ap+0,	/* 1.99649: rotor inertia [kg m^2], joint 3 */
	0x1.bf01b866e43aap-1,	/* 0.87306: rotor inertia [kg m^2], joint 4 */
	0x1.ba5657fb69985p-1,	/* 0.86394: rotor inertia [kg m^2], joint 5 */
	0x1.b1e7967caea74p-1,	/* 0.84747: rotor inertia [kg m^2], joint 6 */
	0x1.d6ccccccccccdp+6,	/* 117.7: torque limit [N m], joint 1 */
	0x1.d6ccccccccccdp+6,	/* 117.7: torque limit [N m], joint 2 */
	0x1.7cp+5,	/* 47.5: torque limit [N m], joint 3 */
	0x1.5p+4,	/* 21: torque limit [N m], joint 4 */
	0x1.5p+4,	/* 21: torque limit [N m], joint 5 */
	0x1.5p+4,	/* 21: torque limit [N m], joint 6 */
	0x0p+0,	/* 0: ready posture [rad], joint 1 */
	0x0p+0,	/* 0: ready posture [rad], joint 2 */
	-0x1.921fb54442d18p+0,	/* -1.570796327: ready posture [rad], joint 3 */
	0x0p+0,	/* 0: ready posture [rad], joint 4 */
	-0x1.921fb54442d18p+0,	/* -1.570796327: ready posture [rad], joint 5 */
	0x0p+0,	/* 0: ready posture [rad], joint 6 */
	0x1p+0,	/* 1: gate of the friction feedforward (0, 1) */
	0x1.89374bc6a7efap-8,	/* 0.006: goal threshold [m] */
	0x1.47ae147ae147bp-7,	/* 0.01: IK damping */
	0x1.999999999999ap-3,	/* 0.2: IK step limit [rad] */
	0x1.a36e2eb1c432dp-14,	/* 0.0001: IK tolerance [m] */
	0x1.4p+2,	/* 5: periods from the request of a plan to its use */
	0x1.9p+6,	/* 100: IK iterations per goal */
	0x1.3333333333333p-1,	/* 0.6: speed limit [rad/s] */
	0x1p+1,	/* 2: acceleration limit [rad/s^2] */
	0x1.4p+3,	/* 10: jerk limit [rad/s^3] */
	0x1.0a3d70a3d70a4p-2,	/* 0.26: limit margin [rad] */
	0x1.999999999999ap-4,	/* 0.1: lead bound [rad] */
	0x0p+0,	/* 0: step baseline (0, 1) */
	0x1.47ae147ae147bp-7,	/* 0.01: contract: delta [m] */
	0x1p-2,	/* 0.25: contract: age cap [s] */
	0x1.0624dd2f1a9fcp-10,	/* 0.001: contract: e0 [m] */
	0x1p+0,	/* 1: contract: g */
	0x0p+0,	/* 0: contract: eps_v [m/s] */
	0x0p+0,	/* 0: contract: a_max [m/s^2] */
	0x1.0624dd2f1a9fcp-9,	/* 0.002: contract: eps_cal [m] */
	0x1.0624dd2f1a9fcp-9,	/* 0.002: contract: eps_m of mode 0 (416) [m] */
	0x1.89374bc6a7efap-9,	/* 0.003: contract: eps_m of mode 1 (320) [m] */
	-0x1.5c28f5c28f5c3p-4,	/* -0.085: tool tag offset x [m] */
	0x0p+0,	/* 0: tool tag offset y [m] */
	0x0p+0,	/* 0: tool tag offset z [m] */
	0x1.3333333333333p-2,	/* 0.3: goal box x0 [m] */
	0x1.e147ae147ae14p-1,	/* 0.94: goal box x1 [m] */
	-0x1.7ae147ae147aep-2,	/* -0.37: goal box y0 [m] */
	0x1.428f5c28f5c29p-1,	/* 0.63: goal box y1 [m] */
	0x1p-2,	/* 0.25: goal box z0 [m] */
	0x1p-1,	/* 0.5: goal box z1 [m] */
	0x1.d70a3d70a3d71p-1,	/* 0.92: goal r max [m] */
	0x1p-2,	/* 0.25: goal floor [m] */
	0x1.999999999999ap-3,	/* 0.2: hover height [m] */
	0x1.999999999999ap-2,	/* 0.4: smoothing kappa */
	0x1p+0,	/* 1: smoothing gap [s] */
	0x1p+0,	/* 1: target lost after [s] */
	0x1p+1,	/* 2: age of the joint angles of a period [periods] */
};

#endif /* CTRL_NUMBERS_H */
