/*
 * ecat_bringup.c  --  ec_init -> config -> PDO map -> SAFE_OP -> OPERATIONAL
 * (translation of the ecrt_* setup block of the IgH main(), without its distributed clocks: the drives run
 * FREE_RUN, each on its own internal cycle).
 *
 * - Order:  PRE-OP -> SM2 watchdog trigger -> PDO mapping -> torque limit -> DC offsets (no SYNC0) -> SAFE-OP ->
 *   process data FLOWING AT THE CYCLE RATE for SAFEOP_FRAMES_MS -> OP.
 *   Asking for OP without sending process data while waiting does not work: drives with an SM watchdog never
 *   leave SAFE-OP that way.
 * - PDO size check: what the drive reports must equal sizeof(drive_outputs_t)/sizeof(drive_inputs_t), otherwise
 *   the packed structs would be written to the wrong offsets.  Init fails instead; nothing moves.
 *   Exactly AXIS_COUNT slaves must be on the bus.
 */
#include "FreeRTOS.h"
#include "task.h"
#include "xil_printf.h"
#include "ethercat.h"
#include "app_config.h"
#include "ecat_bringup.h"
#include "cia402_drive.h"
#include "ecat_diag.h"

/* gem3_nic.c (rev4): NWSR/TXSR/RXSR, frame counters and the PHY link bits - printed when the bus scan finds nobody. */
extern void gem3_debug_dump(void);

static char IOmap[4096] __attribute__((aligned(4)));

#define SM_CONTROL_WATCHDOG_TRIGGER 0x00000040u     /* bit 6 of the SM control byte, the low byte of SMflags */

int ecat_expected_wkc;

/* Outputs while the drives are brought to OP: servo OFF (controlword 0), CSP selected, target = actual position. */
static void set_standstill_outputs(void)
{
    int i;
    for (i = 0; i < AXIS_COUNT; i++) {
        drive_out[i]->modes_of_operation = MODE_CSP;
        drive_out[i]->target_position    = drive_in[i]->position_actual;
    }
}

int ecat_bring_up(void)
{
    int i, n, t, lowest = 0, limited;
    TickType_t next;

    xil_printf("[ecat] ec_init(GEM3) ...\r\n");
    if (ec_init("GEM3") <= 0) {                           /* ifname ignored by our nicdrv */
        xil_printf("[ecat] FAIL: ec_init -- the NIC did not open (see the [gem3] lines above)\r\n");
        return 0;
    }

    n = ec_config_init(FALSE);                            /* bus scan, slaves -> PRE-OP, fills ec_slave[] */
    xil_printf("[ecat] ec_config_init: %d slave(s)\r\n", n);
    if (n <= 0) {
        xil_printf("[ecat] FAIL: nobody answered. Cable on the FIRST drive's IN port? Drive power on?"
                   " ([gem3] link UP?)\r\n");
        gem3_debug_dump();       /* tx>0 rx=0: cable / IN port.  rxFCSerr>0: RGMII delay.  TXSR 0x100 / RXSR 0x8: DMA bus error */
        return 0;
    }
    for (i = 1; i <= ec_slavecount; i++) {
        xil_printf("[ecat]   slave %d: \"%s\"  vendor 0x%08x  product 0x%08x  DC %s\r\n", i, ec_slave[i].name,
                   (unsigned)ec_slave[i].eep_man, (unsigned)ec_slave[i].eep_id, ec_slave[i].hasdc ? "yes" : "NO");
    }
    if (ec_slavecount != AXIS_COUNT) {
        xil_printf("[ecat] FAIL: found %d slave(s) but this program drives exactly %d (AXIS_COUNT)."
                   " Nothing will be moved.\r\n", ec_slavecount, AXIS_COUNT);
        return 0;
    }

    /* register the PDO-mapping hook on each motor (called inside config_map) */
    for (i = 1; i <= AXIS_COUNT; i++) {
        ec_slave[i].PO2SOconfig = &drive_map_pdos;
    }

    /* SM2 watchdog trigger on, as IgH's EC_WD_ENABLE: the drives' SII has the SM2 control byte 0x24 (trigger off), and
     * SOEM writes it as it is.  The drive with firmware 0.12 then leaves OP 100 ms after OP (AL status 0x001B, AL-67 on
     * its display) although every frame reaches it; under IgH, which writes 0x64, the same drive stays in OP
     * (2026-11-16).  ec_config_map() writes the SM registers. */
    for (i = 1; i <= AXIS_COUNT; i++) {
        ec_slave[i].SM[2].SMflags = htoel(etohl(ec_slave[i].SM[2].SMflags) | SM_CONTROL_WATCHDOG_TRIGGER);
    }

    /* The drives stay in PRE-OP until the SAFE-OP request below, after the PDO mapping, the 0x1C32 read and the
     * torque limit.
     * SOEM would request SAFE-OP inside ec_config_map(); manualstatechange suppresses exactly that
     * (the slaves are already in PRE-OP, so nothing else changes).                                  */
    ecx_context.manualstatechange = 1;
    n = ec_config_map(&IOmap);                            /* calls drive_map_pdos() per slave */
    ecx_context.manualstatechange = 0;
    xil_printf("[ecat] ec_config_map: IOmap %d bytes (expected %d)\r\n", n,
               AXIS_COUNT * (int)(sizeof(drive_outputs_t) + sizeof(drive_inputs_t)));
    print_soem_errors(8);
    if (n <= 0) {
        xil_printf("[ecat] FAIL: ec_config_map produced no process data\r\n");
        return 0;
    }

    /* the packed structs are only valid for exactly this mapping; bind the process-data pointers */
    for (i = 0; i < AXIS_COUNT; i++) {
        const ec_slavet *s = &ec_slave[i + 1];
        if (s->outputs == NULL || s->inputs == NULL ||
            s->Obytes != sizeof(drive_outputs_t) || s->Ibytes != sizeof(drive_inputs_t)) {
            xil_printf("[ecat] FAIL: slave %d reports PDO sizes out %u / in %u bytes, this program needs %u / %u"
                       " (PDO-mapping SDOs rejected? fixed mapping?). Nothing will be moved.\r\n",
                       i + 1, (unsigned)s->Obytes, (unsigned)s->Ibytes,
                       (unsigned)sizeof(drive_outputs_t), (unsigned)sizeof(drive_inputs_t));
            return 0;
        }
        drive_out[i] = (drive_outputs_t *)s->outputs;
        drive_in[i]  = (drive_inputs_t *)s->inputs;
    }

    /* what each drive's SM2 now holds (read from the drive, not from ec_slave[]): the control byte should be 0x64 */
    for (i = 1; i <= AXIS_COUNT; i++) {
        ec_smt sm2;
        if (ec_FPRD(ec_slave[i].configadr, ECT_REG_SM2, sizeof(sm2), &sm2, EC_TIMEOUTRET) <= 0) {
            xil_printf("[ecat]   slave %d SM2: cannot be read\r\n", i);
        } else {
            xil_printf("[ecat]   slave %d SM2 @0x%04x, %u bytes, control 0x%02x\r\n", i, (unsigned)etohs(sm2.StartAddr),
                       (unsigned)etohs(sm2.SMlength), (unsigned)(etohl(sm2.SMflags) & 0xFFu));
        }
    }

    /* FREE_RUN: no ec_dcsync0() (ec_configdc() below sets the DC offsets only), and ec_config_init() has cleared
     * every slave's SYNC activation.
     * The drives report 0x1C32:01 = 1 (SM2-synchronous) all the same, under IgH as well (2026-11-16). */
    print_sync_parameters();

    /* A low torque limit before any servo-on (D-10 condition 1).  Set at every RUN_LEVEL, so that the bus-only run
     * shows the read-back values before a build that enables the drives runs at all; such a build stops here
     * without them. */
    for (i = 1, limited = 0; i <= AXIS_COUNT; i++) {
        limited += drive_set_torque_limit((uint16)i, MAX_TORQUE_PERMILLE);
    }
    print_soem_errors(8);
    if (limited != AXIS_COUNT) {
#if RUN_LEVEL >= RUN_SERVO_HOLD
        xil_printf("[ecat] FAIL: the torque limit did not read back on %d drive(s). Nothing will be enabled.\r\n",
                   AXIS_COUNT - limited);
        return 0;
#else
        xil_printf("[ecat] WARNING: the torque limit did not read back on %d drive(s);"
                   " a RUN_LEVEL 1 or 2 build stops at this point\r\n", AXIS_COUNT - limited);
#endif
    }

    /* DC system time offsets and propagation delays (0x0920, 0x0928) without SYNC0: SYNC activation (0x0981) stays 0,
     * so the drives still run FREE_RUN.  IgH writes them for every DC slave whatever the sync mode, and the
     * colleague's DC build ran ec_configdc() too: under both the firmware 0.12 drive stays in OP; without this and
     * with OP after one frame it left OP 100 ms after OP (2026-11-16). */
    if (!ec_configdc()) {
        xil_printf("[ecat] WARN: no DC-capable slave found\r\n");
    }

    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);
    (void)ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    if (ec_slave[0].state != EC_STATE_SAFE_OP) {
        print_slave_states("FAIL: not all drives reached SAFE-OP");
        return 0;
    }
    xil_printf("[ecat] all drives in SAFE-OP\r\n");

    /* OPERATIONAL: process data must keep flowing at the cycle rate while the drives change state (SM watchdog).
     * SAFEOP_FRAMES_MS of it in SAFE-OP before the request, as under IgH (1.4 s) and the DC build (205 ms), where
     * the firmware 0.12 drive stays in OP; OP after one frame (SOEM simple_test) is where it did not. */
    ecat_expected_wkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    xil_printf("[ecat] expected working counter %d; requesting OPERATIONAL after %d ms of process data\r\n",
               ecat_expected_wkc, SAFEOP_FRAMES_MS);

    next = xTaskGetTickCount();
    for (t = 0; t < OP_TIMEOUT_MS; t++) {
        ec_send_processdata();
        (void)ec_receive_processdata(RX_TIMEOUT_US);
        set_standstill_outputs();
        if (t == SAFEOP_FRAMES_MS) {
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            ec_writestate(0);
        } else if (t > SAFEOP_FRAMES_MS) {
            lowest = ec_readstate();
            for (i = 1; i <= ec_slavecount; i++) {        /* a drive that refuses OP raises the AL error flag */
                if (ec_slave[i].state & EC_STATE_ERROR) {
                    print_slave_states("FAIL: a drive refused OPERATIONAL");
                    return 0;
                }
            }
            if (lowest == EC_STATE_OPERATIONAL) break;
        }
        vTaskDelayUntil(&next, pdMS_TO_TICKS(1));
    }
    if (lowest != EC_STATE_OPERATIONAL) {
        print_slave_states("FAIL: not all drives reached OPERATIONAL in time");
        return 0;
    }
    xil_printf("[ecat] all drives OPERATIONAL %d ms after the request\r\n", t - SAFEOP_FRAMES_MS);

    ec_send_processdata();                                /* prime: the first cyclic receive finds a frame */
    return 1;
}
