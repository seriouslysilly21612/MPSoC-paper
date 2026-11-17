/*
 * cia402_drive.h  --  a CiA402 servo drive as the master sees it: its process data, the PDO mapping that
 * produces that layout, and the drive state read from the statusword.
 */
#ifndef CIA402_DRIVE_H
#define CIA402_DRIVE_H

#include "ethercat.h"
#include "app_config.h"

/* ----------------------------------------------------------------------- */
/* Process-data layout (must match the PDO mapping of drive_map_pdos(), Table 5 of the paper). */
/* EtherCAT payload is little-endian, R5 is little-endian -> direct access.*/
/* The fields sit at odd offsets: plain unaligned loads/stores, fine on the R5F in normal memory  */
/* (SCTLR.A = 0 as set by the Xilinx BSP).  A data abort on one of these -> build -mno-unaligned-access. */
/* ----------------------------------------------------------------------- */
typedef struct __attribute__((__packed__)) {   /* RxPDO 0x1600 (master->slave) */
    uint16 controlword;          /* 0x6040 */
    int32  target_position;      /* 0x607A  (ignored in CST, kept equal to the actual position) */
    int32  target_velocity;      /* 0x60FF  (ignored in CST, 0) */
    int16  target_torque;        /* 0x6071  drive units: L7N 0.1 % of rated, Indy7 per KAPPA of INDY7.cfg */
    int8   modes_of_operation;   /* 0x6060 */
} drive_outputs_t;

typedef struct __attribute__((__packed__)) {   /* TxPDO 0x1A00 (slave->master) */
    uint16 statusword;           /* 0x6041 */
    int32  position_actual;      /* 0x6064 */
    int32  velocity_actual;      /* 0x606C */
    int16  torque_actual;        /* 0x6077  same unit as target_torque */
    int8   modes_display;        /* 0x6061 */
} drive_inputs_t;

_Static_assert(sizeof(drive_outputs_t) == 13 && sizeof(drive_inputs_t) == 13, "PDO structs must be 13 bytes each");

/* axis i's process data in the IOmap (ec_slave[i + 1].outputs / .inputs), bound by ecat_bring_up() */
extern drive_outputs_t *drive_out[AXIS_COUNT];
extern drive_inputs_t  *drive_in[AXIS_COUNT];

/* ----------------------------------------------------------------------- */
/* CiA402 drive states (the value is the statusword bit pattern of the     */
/* state) and controlword commands, from the IgH version                   */
/* ----------------------------------------------------------------------- */
#define CIA402_NOT_READY_TO_SWITCH_ON   0x0000
#define CIA402_SWITCH_ON_DISABLED       0x0040
#define CIA402_READY_TO_SWITCH_ON       0x0021
#define CIA402_SWITCHED_ON              0x0023
#define CIA402_OPERATION_ENABLED        0x0027
#define CIA402_QUICK_STOP_ACTIVE        0x0007
#define CIA402_FAULT_REACTION_ACTIVE    0x000F
#define CIA402_FAULT                    0x0008
#define CIA402_STATE_UNKNOWN            0xFFFF

#define CW_DISABLE_VOLTAGE      0x0000         /* any state -> SWITCH_ON_DISABLED, the power stage off */
#define CW_SHUTDOWN             0x0006         /* IgH: READY            */
#define CW_SWITCH_ON            0x0007
#define CW_ENABLE_OPERATION     0x000F
#define CW_FAULT_RESET          0x0080

#define MODE_CST                0x0A           /* Cyclic Synchronous Torque (DRIVE_MODE=10 of INDY7.cfg) */

unsigned short cia402_state_from_statusword(unsigned short statusword);   /* -> CIA402_* */
const char    *cia402_state_name(unsigned state);

/* SOEM's PO2SOconfig hook: PDO mapping by CoE SDO writes, called per slave inside ec_config_map (PRE-OP) */
int drive_map_pdos(uint16 slave);

/* PRE-OP: write the torque limit (0.1 % of rated) to 0x6072, 0x60E0 and 0x60E1, read the three back and print them.
 * 1 = all three read back as written. */
int drive_set_torque_limit(uint16 slave, uint16 permille);

#endif /* CIA402_DRIVE_H */
