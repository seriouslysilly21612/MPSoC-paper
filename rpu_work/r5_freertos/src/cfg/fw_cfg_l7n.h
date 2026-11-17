/* The constants of the firmware. Written by fw_cfg_header from the configuration of the controller:
 * not to be edited. */
#ifndef FW_CFG_H
#define FW_CFG_H

#include <stdint.h>

#define FW_CFG_CRC               0x07AE943Du	/* of the values below; into cfg_crc of the State slot */
#define FW_CFG_SLAVES            4
#define FW_CFG_CYCLE_NS          1000000
#define FW_CFG_SEND_DEADLINE_NS  400000
#define FW_CFG_SYNC_MODE         0	/* 0 FREE_RUN, 1 SM, 2 DC */
#define FW_CFG_DC_SYNC0_SHIFT_NS 0

typedef struct
{
	uint32_t vendor_id, product_code;
	int32_t  slave_type;			/* 0 joint drive, 1 end tool */
	int32_t  move_ccw, drive_mode;
	double   encoder_resolution, gear_ratio, transmission_ratio, torque_constant, current_ratio;
	int32_t  home_position_offset;
	double   torque_limit_u;
} fw_cfg_axis_t;

static const fw_cfg_axis_t FW_CFG_AXIS[FW_CFG_SLAVES] =
{
	{ 0x00007595u, 0x00000000u, 0, 0, 10, 524288, 1, 1, 1.337, 1000, 0, 0.40000000000000002 },	/* [AXIS0] */
	{ 0x00007595u, 0x00000000u, 0, 0, 10, 524288, 1, 1, 1.337, 1000, 0, 0.40000000000000002 },	/* [AXIS1] */
	{ 0x00007595u, 0x00000000u, 0, 0, 10, 524288, 1, 1, 1.337, 1000, 0, 0.40000000000000002 },	/* [AXIS2] */
	{ 0x00007595u, 0x00000000u, 0, 0, 10, 524288, 1, 1, 1.337, 1000, 0, 0.40000000000000002 },	/* [AXIS3] */
};

#endif /* FW_CFG_H */
