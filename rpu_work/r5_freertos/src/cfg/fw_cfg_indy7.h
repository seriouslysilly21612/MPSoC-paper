/* The constants of the firmware. Written by fw_cfg_header from the configuration of the controller:
 * not to be edited. */
#ifndef FW_CFG_H
#define FW_CFG_H

#include <stdint.h>

#define FW_CFG_CRC               0x3896D4ECu	/* of the values below; into cfg_crc of the State slot */
#define FW_CFG_SLAVES            7
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
	{ 0x0000089Au, 0x30000000u, 0, 0, 10, 65536, 121, 1, 0.088400000000000006, 48, 23567184, 117.7 },	/* [AXIS0] */
	{ 0x0000089Au, 0x30000000u, 0, 0, 10, 65536, 121, 1, 0.088400000000000006, 48, 28218, 117.7 },	/* [AXIS1] */
	{ 0x0000089Au, 0x30000000u, 0, 1, 10, 65536, 121, 1, 0.086999999999999994, 96, 15841818, 47.5 },	/* [AXIS2] */
	{ 0x0000089Au, 0x30000000u, 0, 0, 10, 65536, 101, 1, 0.058000000000000003, 96, 6161747, 21 },	/* [AXIS3] */
	{ 0x0000089Au, 0x30000000u, 0, 0, 10, 65536, 101, 1, 0.058000000000000003, 96, 39749892, 21 },	/* [AXIS4] */
	{ 0x0000089Au, 0x30000000u, 0, 1, 10, 65536, 101, 1, 0.058000000000000003, 96, 45545, 21 },	/* [AXIS5] */
	{ 0x0000089Au, 0x10000007u, 1, 0, 10, 0, 0, 0, 0, 0, 0, 0 },	/* [AXIS6] */
};

#endif /* FW_CFG_H */
