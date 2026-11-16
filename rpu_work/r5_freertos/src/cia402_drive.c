/*
 * cia402_drive.c  --  the CiA402 drive interface: PDO mapping by CoE SDO (translation of IgH's
 * ecrt_slave_config_pdos; IgH maps the PDOs itself, SOEM leaves it to this hook), statusword -> state.
 */
#include "xil_printf.h"
#include "cia402_drive.h"
#include "ecat_diag.h"

#define PDO_MAPPING_SDO_WRITES  19    /* number of ec_SDOwrite() calls in drive_map_pdos() */

drive_outputs_t *drive_out[AXIS_COUNT];   /* -> ec_slave[i+1].outputs */
drive_inputs_t  *drive_in[AXIS_COUNT];    /* -> ec_slave[i+1].inputs  */

unsigned short cia402_state_from_statusword(unsigned short statusword)
{
    if      ((statusword & 0x4F) == 0x00) return CIA402_NOT_READY_TO_SWITCH_ON;
    else if ((statusword & 0x4F) == 0x08) return CIA402_FAULT;
    else if ((statusword & 0x4F) == 0x40) return CIA402_SWITCH_ON_DISABLED;
    else if ((statusword & 0x6F) == 0x27) return CIA402_OPERATION_ENABLED;
    else if ((statusword & 0x6F) == 0x23) return CIA402_SWITCHED_ON;
    else if ((statusword & 0x6F) == 0x21) return CIA402_READY_TO_SWITCH_ON;
    else if ((statusword & 0x6F) == 0x07) return CIA402_QUICK_STOP_ACTIVE;
    else if ((statusword & 0x4F) == 0x0F) return CIA402_FAULT_REACTION_ACTIVE;
    return CIA402_STATE_UNKNOWN;
}

const char *cia402_state_name(unsigned state)
{
    switch (state) {
    case CIA402_NOT_READY_TO_SWITCH_ON:  return "NOT_READY_TO_SWITCH_ON";
    case CIA402_SWITCH_ON_DISABLED:      return "SWITCH_ON_DISABLED";
    case CIA402_READY_TO_SWITCH_ON:      return "READY_TO_SWITCH_ON";
    case CIA402_SWITCHED_ON:             return "SWITCHED_ON";
    case CIA402_OPERATION_ENABLED:       return "OPERATION_ENABLED";
    case CIA402_QUICK_STOP_ACTIVE:       return "QUICK_STOP_ACTIVE";
    case CIA402_FAULT_REACTION_ACTIVE:   return "FAULT_REACTION_ACTIVE";
    case CIA402_FAULT:                   return "FAULT";
    default:                             return "unknown";
    }
}

/* ----------------------------------------------------------------------- */
/* PDO mapping via CoE SDO.  SOEM calls this per slave during ec_config_map */
/* (PRE-OP).  VERIFY against the drive's ESI/manual: some drives use FIXED  */
/* mapping and reject these writes -- then skip this and use the default    */
/* layout.                                                                  */
/* ----------------------------------------------------------------------- */
int drive_map_pdos(uint16 slave)
{
    int wkc = 0;
    uint8  u8;
    uint16 u16;
    uint32 u32;

    /* RxPDO 0x1600: controlword, target_pos, target_vel, mode */
    u8 = 0; wkc += ec_SDOwrite(slave, 0x1C12, 0x00, FALSE, sizeof(u8),  &u8,  EC_TIMEOUTRXM);
    u8 = 0; wkc += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u8),  &u8,  EC_TIMEOUTRXM);
    u32 = 0x60400010; wkc += ec_SDOwrite(slave, 0x1600, 0x01, FALSE, sizeof(u32), &u32, EC_TIMEOUTRXM);
    u32 = 0x607A0020; wkc += ec_SDOwrite(slave, 0x1600, 0x02, FALSE, sizeof(u32), &u32, EC_TIMEOUTRXM);
    u32 = 0x60FF0020; wkc += ec_SDOwrite(slave, 0x1600, 0x03, FALSE, sizeof(u32), &u32, EC_TIMEOUTRXM);
    u32 = 0x60600008; wkc += ec_SDOwrite(slave, 0x1600, 0x04, FALSE, sizeof(u32), &u32, EC_TIMEOUTRXM);
    u8 = 4;  wkc += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u8),  &u8,  EC_TIMEOUTRXM);
    u16 = 0x1600; wkc += ec_SDOwrite(slave, 0x1C12, 0x01, FALSE, sizeof(u16), &u16, EC_TIMEOUTRXM);
    u8 = 1;  wkc += ec_SDOwrite(slave, 0x1C12, 0x00, FALSE, sizeof(u8),  &u8,  EC_TIMEOUTRXM);

    /* TxPDO 0x1A00: statusword, pos_actual, vel_actual, mode_display */
    u8 = 0; wkc += ec_SDOwrite(slave, 0x1C13, 0x00, FALSE, sizeof(u8),  &u8,  EC_TIMEOUTRXM);
    u8 = 0; wkc += ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(u8),  &u8,  EC_TIMEOUTRXM);
    u32 = 0x60410010; wkc += ec_SDOwrite(slave, 0x1A00, 0x01, FALSE, sizeof(u32), &u32, EC_TIMEOUTRXM);
    u32 = 0x60640020; wkc += ec_SDOwrite(slave, 0x1A00, 0x02, FALSE, sizeof(u32), &u32, EC_TIMEOUTRXM);
    u32 = 0x606C0020; wkc += ec_SDOwrite(slave, 0x1A00, 0x03, FALSE, sizeof(u32), &u32, EC_TIMEOUTRXM);
    u32 = 0x60610008; wkc += ec_SDOwrite(slave, 0x1A00, 0x04, FALSE, sizeof(u32), &u32, EC_TIMEOUTRXM);
    u8 = 4;  wkc += ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(u8),  &u8,  EC_TIMEOUTRXM);
    u16 = 0x1A00; wkc += ec_SDOwrite(slave, 0x1C13, 0x01, FALSE, sizeof(u16), &u16, EC_TIMEOUTRXM);
    u8 = 1;  wkc += ec_SDOwrite(slave, 0x1C13, 0x00, FALSE, sizeof(u8),  &u8,  EC_TIMEOUTRXM);

    /* default mode = CSP so the drive is in position mode from the start */
    u8 = MODE_CSP; wkc += ec_SDOwrite(slave, 0x6060, 0x00, FALSE, sizeof(u8), &u8, EC_TIMEOUTRXM);

    if (wkc != PDO_MAPPING_SDO_WRITES) {
        xil_printf("[ecat] slave %u: only %d of %d PDO-mapping SDO writes were accepted"
                   " (fixed PDO mapping?)\r\n", (unsigned)slave, wkc, PDO_MAPPING_SDO_WRITES);
        print_soem_errors(3);                    /* the first aborts of this slave say which object the drive refused */
    }
    return 1;   /* SOEM ignores the value; return 1 by convention */
}

/* All three objects: 0x6072 caps both directions, but an LS drive limits with 0x60E0 (forward) and 0x60E1 (reverse)
 * when its torque limit function select 0x2110 is 0 -- so the line also prints 0x2110.  A value that cannot be
 * read prints as 65535, like the 0x1C32 line. */
int drive_set_torque_limit(uint16 slave, uint16 permille)
{
    static const uint16 index[3] = { 0x6072, 0x60E0, 0x60E1 };
    uint16 readback[3] = { 0xFFFF, 0xFFFF, 0xFFFF }, limit_select = 0xFFFF, value;
    int i, size, ok = 1;

    for (i = 0; i < 3; i++) {
        value = permille;
        size  = sizeof(readback[i]);
        /* the read-back decides, and it shows what the drive holds even when it refused the write */
        (void)ec_SDOwrite(slave, index[i], 0x00, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
        if (ec_SDOread(slave, index[i], 0x00, FALSE, &size, &readback[i], EC_TIMEOUTRXM) <= 0 ||
            readback[i] != permille) {
            ok = 0;
        }
    }
    size = sizeof(limit_select);
    (void)ec_SDOread(slave, 0x2110, 0x00, FALSE, &size, &limit_select, EC_TIMEOUTRXM);
    /* no "%%": xil_printf prints it only when the BSP defines STDOUT_BASEADDRESS */
    xil_printf("[ecat]   slave %u torque limit: 0x6072 %u, 0x60E0 %u, 0x60E1 %u (wrote %u; unit 0.1 percent of rated),"
               " 0x2110 %u\r\n", (unsigned)slave, (unsigned)readback[0], (unsigned)readback[1],
               (unsigned)readback[2], (unsigned)permille, (unsigned)limit_select);
    return ok;
}
