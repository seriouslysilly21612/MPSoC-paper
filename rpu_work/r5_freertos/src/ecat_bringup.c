/*
 * ecat_bringup.c  --  ec_init -> config -> PDO map -> DC -> SAFE_OP -> OPERATIONAL
 * (translation of the ecrt_* setup block of the IgH main()).
 *
 * - Order = IgH:  PRE-OP -> PDO mapping -> DC + SYNC0 -> SAFE-OP -> process data FLOWING AT THE CYCLE RATE -> OP.
 *   Asking for OP without sending process data while waiting does not work: drives with DC / SM watchdog never
 *   leave SAFE-OP that way, and SOEM's DC drift compensation (the FRMW datagram inside every process-data frame)
 *   never runs.
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
    if (ec_slavecount != AXIS_COUNT) {
        xil_printf("[ecat] FAIL: found %d slave(s) but this program drives exactly %d (AXIS_COUNT)."
                   " Nothing will be moved.\r\n", ec_slavecount, AXIS_COUNT);
        return 0;
    }

    /* register the PDO-mapping hook on each motor (called inside config_map) */
    for (i = 1; i <= AXIS_COUNT; i++) {
        ec_slave[i].PO2SOconfig = &drive_map_pdos;
    }

    /* IgH order: PDO mapping + DC/SYNC0 are in place BEFORE the drives are asked for SAFE-OP.
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

    /* distributed clocks: offsets + propagation delays, then SYNC0 at the cycle time on every DC slave */
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
