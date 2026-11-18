/*
 * ecat_bringup.c  --  ec_init -> config -> PDO map -> DC -> SAFE_OP -> OPERATIONAL
 * (translation of the ecrt_* setup block of the IgH main()).
 *
 * - Order = IgH:  PRE-OP -> SM2 watchdog trigger -> PDO mapping -> torque limit -> [DC + SYNC0, ECAT_USE_DC] -> SAFE-OP ->
 *   process data FLOWING AT THE CYCLE RATE -> OP.
 *   Asking for OP without sending process data while waiting does not work: drives with DC / SM watchdog never
 *   leave SAFE-OP that way, and SOEM's DC drift compensation (the FRMW datagram inside every process-data frame)
 *   never runs.
 * - PDO size check: what the drive reports must equal sizeof(drive_outputs_t)/sizeof(drive_inputs_t), otherwise
 *   the packed structs would be written to the wrong offsets.  Init fails instead; nothing moves.
 *   Exactly BUS_SLAVE_COUNT slaves must be on the bus, and the first AXIS_COUNT must be the drives of ROBOT_TARGET.
 */
#include "FreeRTOS.h"
#include "task.h"
#include "xil_printf.h"
#include "ethercat.h"
#include "app_config.h"
#include "ecat_bringup.h"
#include "cia402_drive.h"
#include "robot_target.h"
#include "ecat_diag.h"

/* gem3_nic.c (rev4): NWSR/TXSR/RXSR, frame counters and the PHY link bits - printed when the bus scan finds nobody. */
extern void gem3_debug_dump(void);

static char IOmap[4096] __attribute__((aligned(4)));

#define SM_CONTROL_WATCHDOG_TRIGGER 0x00000040u     /* bit 6 of the SM control byte, the low byte of SMflags */

int ecat_expected_wkc;

/* Outputs while the drives are brought to OP: servo OFF (controlword 0), CST selected, torque 0, target = actual position. */
static void set_standstill_outputs(void)
{
    int i;
    for (i = 0; i < AXIS_COUNT; i++) {
        drive_out[i]->modes_of_operation = MODE_CST;
        drive_out[i]->target_position    = drive_in[i]->position_actual;
        drive_out[i]->target_torque      = 0;
    }
}

int ecat_bring_up(void)
{
    int i, n, t, lowest = 0;
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
    if (ec_slavecount != BUS_SLAVE_COUNT) {
        xil_printf("[ecat] FAIL: found %d slave(s) but this build (ROBOT_TARGET %d) expects exactly %d."
                   " Nothing will be moved.\r\n", ec_slavecount, ROBOT_TARGET, BUS_SLAVE_COUNT);
        return 0;
    }
    for (i = 1; i <= BUS_SLAVE_COUNT; i++) { /* the wrong target on the wrong hardware: the unit constants would be wrong */
        const fw_cfg_axis_t *cfg = &FW_CFG_AXIS[i - 1];

        if (ec_slave[i].eep_man != cfg->vendor_id || ec_slave[i].eep_id != cfg->product_code) {
            xil_printf("[ecat] FAIL: slave %d is vendor 0x%08x product 0x%08x, the cfg of this build (ROBOT_TARGET %d) has"
                       " 0x%08x 0x%08x. Nothing will be moved.\r\n", i, (unsigned)ec_slave[i].eep_man,
                       (unsigned)ec_slave[i].eep_id, ROBOT_TARGET, (unsigned)cfg->vendor_id, (unsigned)cfg->product_code);
            return 0;
        }
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

    /* Read before SYNC0 is set up below: without DC the drives report 0x1C32:01 = 1 (SM2-synchronous), under IgH as
     * well (2026-11-16).  With ECAT_USE_DC 0 this is also what the drives run with. */
    print_sync_parameters();

    /* A low torque limit before any servo-on (D-10 condition 1).  The objects 0x6072 / 0x60E0 / 0x60E1 are those of
     * the L7N manual; shortcut: for the Indy7 drives they are not checked, so nothing is written there (the torque
     * limit of the Indy7 is TORQUE_LIMIT_U of the cfg, applied by the APU) - look at it before the first
     * servo-on on the robot. */
#if ROBOT_TARGET == TARGET_L7N_BENCH
    {
        int limited = 0;
        for (i = 1; i <= AXIS_COUNT; i++) {
            limited += drive_set_torque_limit((uint16)i, MAX_TORQUE_PERMILLE);
        }
        print_soem_errors(8);
        if (limited != AXIS_COUNT) {
            xil_printf("[ecat] FAIL: the torque limit did not read back on %d drive(s). Nothing will be enabled.\r\n",
                       AXIS_COUNT - limited);
            return 0;
        }
    }
#else
    xil_printf("[ecat] torque limit objects not written for this target (see ecat_bringup.c)\r\n");
#endif

    /* Distributed clocks, as in the colleague's build: offsets + propagation delays, then SYNC0 at the cycle time on
     * every DC slave.  Without SYNC0 (FREE_RUN, D-13) the drive with firmware 0.12 left OP 100 ms after OP in every
     * run (AL status 0x001B, AL-67), with the SM2 watchdog trigger, the DC offsets and 200 ms of process data before
     * OP as well; with SYNC0 it reached OP and stayed (colleague, 11-11).  2-1 only checks the EtherCAT task on the
     * R5, so DC it is (user, 2026-11-16). */
#if ECAT_USE_DC
    if (!ec_configdc()) {
        xil_printf("[ecat] WARN: no DC-capable slave found\r\n");
    }
    for (i = 1; i <= AXIS_COUNT; i++) {
        if (ec_slave[i].hasdc) {
            ec_dcsync0((uint16)i, TRUE, CYCLE_NS, DC_SHIFT_NS);
        } else {
            xil_printf("[ecat] WARN: slave %d has no distributed clock, it runs without SYNC0\r\n", i);
        }
    }
#else
    /* Indy7 (user, 2026-11-17): no DC.  The old Indy7 build ran its joints without it: INDY7.cfg has no DC_SUPPORT key,
     * so EcatSlaveBase::EnableDC wrote activate word 0.  shortcut: that the Indy7 drives reach OP this way under
     * SOEM is not checked. */
    xil_printf("[ecat] distributed clocks not set up for this target (ECAT_USE_DC 0): the drives run free\r\n");
#endif

    /* SAFE-OP, now that the drives see their SYNC0 configuration */
    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);
    (void)ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    if (ec_slave[0].state != EC_STATE_SAFE_OP) {
        print_slave_states("FAIL: not all drives reached SAFE-OP");
        return 0;
    }
    xil_printf("[ecat] all drives in SAFE-OP\r\n");

    /* OPERATIONAL: process data must keep flowing at the cycle rate while the drives change state
     * (SM watchdog, DC drift compensation via the FRMW datagram, SYNC0 has to start first).        */
    ecat_expected_wkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    xil_printf("[ecat] expected working counter %d; requesting OPERATIONAL\r\n", ecat_expected_wkc);

    next = xTaskGetTickCount();
    for (t = 0; t < OP_TIMEOUT_MS; t++) {
        ec_send_processdata();
        (void)ec_receive_processdata(RX_TIMEOUT_US);
        set_standstill_outputs();
        if (t == 0) {                                     /* one valid frame first (SOEM simple_test), then ask for OP */
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            ec_writestate(0);
        } else {
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
    xil_printf("[ecat] all drives OPERATIONAL after %d ms\r\n", t);

    ec_send_processdata();                                /* prime: the first cyclic receive finds a frame */
    return 1;
}
