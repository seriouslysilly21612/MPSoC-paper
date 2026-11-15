/*
 * ecat_diag.c  --  EtherCAT diagnostics in words: AL states, AL status codes, CoE SDO abort codes,
 * SOEM's error list, the slave states.
 */
#include "xil_printf.h"
#include "ecat_diag.h"

/* EtherCAT AL status codes (ETG.1000) -- the ones a CiA402 drive usually answers with */
const char *al_status_code_text(uint16 code)
{
    switch (code) {
    case 0x0000: return "no error";
    case 0x0001: return "unspecified error";
    case 0x0011: return "invalid requested state change";
    case 0x0016: return "invalid mailbox configuration";
    case 0x0017: return "invalid sync manager configuration";
    case 0x0018: return "no valid inputs available";
    case 0x0019: return "no valid outputs";
    case 0x001A: return "synchronization error";
    case 0x001B: return "sync manager watchdog";
    case 0x001D: return "invalid output configuration";
    case 0x001E: return "invalid input configuration";
    case 0x001F: return "invalid watchdog configuration";
    case 0x0024: return "invalid input mapping";
    case 0x0025: return "invalid output mapping";
    case 0x0026: return "inconsistent settings";
    case 0x002C: return "fatal sync error";
    case 0x002D: return "no sync error";
    case 0x0030: return "invalid DC SYNC configuration";
    case 0x0032: return "PLL error";
    case 0x0033: return "DC sync IO error";
    case 0x0034: return "DC sync timeout error";
    case 0x0035: return "DC invalid sync cycle time";
    case 0x0036: return "DC invalid sync0 cycle time";
    default:     return "see the ETG.1000 AL status code table";
    }
}

const char *al_state_name(unsigned state)
{
    switch (state) {
    case EC_STATE_INIT:        return "INIT";
    case EC_STATE_PRE_OP:      return "PRE-OP";
    case EC_STATE_BOOT:        return "BOOT";
    case EC_STATE_SAFE_OP:     return "SAFE-OP";
    case EC_STATE_OPERATIONAL: return "OPERATIONAL";
    default:                   return "?";
    }
}

/* CoE SDO abort codes (ETG.1000.6) that a drive typically gives when it refuses a PDO mapping */
static const char *sdo_abort_code_text(uint32_t code)
{
    switch (code) {
    case 0x05040000u: return "SDO protocol timed out";
    case 0x06010000u: return "unsupported access to an object";
    case 0x06010002u: return "write to a read-only object (fixed PDO mapping?)";
    case 0x06010003u: return "subindex cannot be written, SI0 must be 0";
    case 0x06020000u: return "object does not exist";
    case 0x06040041u: return "object cannot be mapped into a PDO";
    case 0x06040042u: return "mapped objects exceed the PDO length";
    case 0x06070010u: return "data type / length of the parameter does not match";
    case 0x06090011u: return "subindex does not exist";
    case 0x06090030u: return "value range of the parameter exceeded";
    case 0x08000000u: return "general error";
    case 0x08000020u: return "data cannot be transferred or stored";
    case 0x08000022u: return "not possible in the present device state";
    default:          return "";
    }
}

/* Pop SOEM's internal error list (SDO aborts, mailbox errors, emergencies ...) and print the OLDEST `max`
 * entries -- the first error of a burst is the informative one (the list is circular and drops the oldest). */
void print_soem_errors(int max)
{
    ec_errort e;
    int cnt = 0, more = 0;

    while (ec_poperror(&e)) {
        if (cnt >= max) { more++; continue; }
        cnt++;
        switch (e.Etype) {
        case EC_ERR_TYPE_SDO_ERROR:
            xil_printf("[ecat]   SOEM: slave %u SDO %04x:%02x aborted, code 0x%08x %s\r\n", (unsigned)e.Slave,
                       (unsigned)e.Index, (unsigned)e.SubIdx, (unsigned)e.AbortCode, sdo_abort_code_text((uint32_t)e.AbortCode));
            break;
        case EC_ERR_TYPE_EMERGENCY:
            xil_printf("[ecat]   SOEM: slave %u EMERGENCY code 0x%04x register 0x%02x\r\n", (unsigned)e.Slave,
                       (unsigned)e.ErrorCode, (unsigned)e.ErrorReg);
            break;
        default:
            xil_printf("[ecat]   SOEM: slave %u error type %d, %04x:%02x, code 0x%08x\r\n", (unsigned)e.Slave,
                       (int)e.Etype, (unsigned)e.Index, (unsigned)e.SubIdx, (unsigned)e.AbortCode);
            break;
        }
    }
    if (more > 0) xil_printf("[ecat]   SOEM: (+%d more error(s))\r\n", more);
}

void print_slave_states(const char *why)
{
    int i;
    (void)ec_readstate();
    xil_printf("[ecat] %s\r\n", why);
    for (i = 1; i <= ec_slavecount; i++) {
        xil_printf("[ecat]   slave %d \"%s\": state 0x%02x  AL status 0x%04x (%s)\r\n", i, ec_slave[i].name,
                   (unsigned)ec_slave[i].state, (unsigned)ec_slave[i].ALstatuscode,
                   al_status_code_text(ec_slave[i].ALstatuscode));
    }
    print_soem_errors(8);
}

/* The SM2 synchronization parameters (object 0x1C32) as each slave reports them, read in PRE-OP:
 * sub 1 sync type in use (0 free run, 1 SM2 event, 2 DC SYNC0, 3 DC SYNC1), sub 2 cycle time [ns],
 * sub 4 sync types supported (bit 0 free run, bit 1 SM2 event, bits 2-3 DC), sub 5 minimum cycle time [ns].
 * A sub that cannot be read prints as 0xFFFF / 4294967295, never as 0 (= free run); SOEM's error list says why. */
void print_sync_parameters(void)
{
    int i;

    for (i = 1; i <= ec_slavecount; i++) {
        uint16 sync_type = 0xFFFF, supported = 0xFFFF;
        uint32 cycle_ns = 0xFFFFFFFFu, min_cycle_ns = 0xFFFFFFFFu;
        int size, got = 0;

        size = sizeof(sync_type);    got += (ec_SDOread((uint16)i, 0x1C32, 0x01, FALSE, &size, &sync_type, EC_TIMEOUTRXM) > 0);
        size = sizeof(cycle_ns);     got += (ec_SDOread((uint16)i, 0x1C32, 0x02, FALSE, &size, &cycle_ns, EC_TIMEOUTRXM) > 0);
        size = sizeof(supported);    got += (ec_SDOread((uint16)i, 0x1C32, 0x04, FALSE, &size, &supported, EC_TIMEOUTRXM) > 0);
        size = sizeof(min_cycle_ns); got += (ec_SDOread((uint16)i, 0x1C32, 0x05, FALSE, &size, &min_cycle_ns, EC_TIMEOUTRXM) > 0);
        xil_printf("[ecat]   slave %d 0x1C32: sync type 0x%04x, cycle %u ns, supported 0x%04x, minimum cycle %u ns"
                   " (%d of 4 read)\r\n", i, (unsigned)sync_type, (unsigned)cycle_ns, (unsigned)supported,
                   (unsigned)min_cycle_ns, got);
    }
    print_soem_errors(8);
}
