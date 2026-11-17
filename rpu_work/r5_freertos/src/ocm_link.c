/*
 * ocm_link.c  --  the R5's side of the shared region at the start of the OCM (see ocm_link.h).
 *
 * Reader and writer rules are those of ipc/ipc_slot.h: 32-bit words, one attempt, no waiting, a failed read leaves
 * the last copy as it was.  The R5 writes the State slot and its half of the head; it reads the Torque and Command
 * slots, which only the APU writes.
 */
#include <string.h>
#include "xil_mmu.h"
#include "xreg_cortexr5.h"
#include "xstatus.h"
#include "xil_printf.h"
#include "app_config.h"
#include "ocm_link.h"
#include "ipc/ipc_slot.h"
#include "cia402_drive.h"
#include "robot_target.h"
#include "axis_control.h"

#define LINK_HEAD       ((volatile ipc_head_t *)(OCM_LINK_BASE + IPC_OFF_HEAD))
#define SLOT_STATE      ((volatile uint32_t *)(OCM_LINK_BASE + IPC_OFF_STATE))
#define SLOT_TORQUE     ((volatile uint32_t *)(OCM_LINK_BASE + IPC_OFF_TORQUE))
#define SLOT_COMMAND    ((volatile uint32_t *)(OCM_LINK_BASE + IPC_OFF_COMMAND))

/* The system counter (D-08): the read-only window IOU_SCNTR, low word at 0x00, high word at 0x04.  The APU reads the
 * same counter as CNTVCT.  shortcut: that the R5 may read this window is not yet checked on the board (step 2-4 of
 * the plan); with LINK_TIMESTAMP 0 t_release stays 0 and nothing reads it. */
#ifndef LINK_TIMESTAMP
#define LINK_TIMESTAMP  1
#endif
#define SYSCNT_LOW      (*(volatile uint32_t *)0xFF250000UL)
#define SYSCNT_HIGH     (*(volatile uint32_t *)0xFF250004UL)

static uint32_t r5_epoch;
static uint32_t command_epoch, command_seq;          /* (epoch, seq) of the last command seen */
static uint32_t command_done;                        /* seq of the last command carried out */
static uint32_t torque_stamp[2];                     /* (epoch, gen) of the last torque taken */
static uint32_t torque_period;
static link_torque_t torque_last;                    /* the last new torque */

uint64_t ocm_link_counter(void)
{
#if LINK_TIMESTAMP
    uint32_t hi, lo, hi_again;

    do {                                             /* high, low, high: a carry between the two reads shows */
        hi = SYSCNT_HIGH;
        lo = SYSCNT_LOW;
        hi_again = SYSCNT_HIGH;
    } while (hi != hi_again);
    return ((uint64_t)hi << 32) | lo;
#else
    return 0u;
#endif
}

int ocm_link_start(void)
{
    volatile uint32_t *slots[1] = { SLOT_STATE };

    /* shortcut: nothing of this region has been touched before this call, so no cache line of it needs a flush */
    if (Xil_SetMPURegion((INTPTR)OCM_LINK_BASE, OCM_LINK_SIZE, NORM_SHARED_NCACHE | PRIV_RW_USER_RW) != (u32)XST_SUCCESS) {
        xil_printf("[link] FAIL: the MPU region for the OCM window could not be set\r\n");
        return 0;
    }
    r5_epoch = ipc_side_start(&LINK_HEAD->r5, slots, 1);
    (void)ocm_link_take_command();                   /* what is in the slot now was sent to an earlier run */
    command_done = 0u;
    xil_printf("[link] OCM 0x%08x: R5 epoch %u, APU side %s, system counter 0x%08x%08x\r\n", (unsigned)OCM_LINK_BASE,
               (unsigned)r5_epoch, ipc_load32(&LINK_HEAD->apu.magic) == IPC_MAGIC ? "up" : "not up yet",
               (unsigned)(ocm_link_counter() >> 32), (unsigned)ocm_link_counter());
    return 1;
}

static int drives_enabled(void)
{
    int i;
    for (i = 0; i < AXIS_COUNT; i++) {
        if (cia402_state_from_statusword(drive_in[i]->statusword) != CIA402_OPERATION_ENABLED) return 0;
    }
    return 1;
}

uint32_t ocm_link_take_command(void)
{
    ipc_command_t command;
    uint32_t stamp[2];

    if (IPC_READ(SLOT_COMMAND, &LINK_HEAD->apu, &command, stamp) != IPC_OK) return IPC_CMD_NONE;
    if (stamp[0] == command_epoch && command.seq == command_seq) return IPC_CMD_NONE;
    command_epoch = stamp[0];
    command_seq   = command.seq;
    command_done  = command.seq;
    return command.cmd;
}

int ocm_link_take_torque(link_torque_t *held)
{
    ipc_torque_t torque;
    uint32_t stamp[2];
    int i, fresh = 0;

    if (IPC_READ(SLOT_TORQUE, &LINK_HEAD->apu, &torque, stamp) == IPC_OK &&
        (stamp[0] != torque_stamp[0] || stamp[1] != torque_stamp[1])) {
        torque_stamp[0] = stamp[0];
        torque_stamp[1] = stamp[1];
        torque_period   = torque.period;
        torque_last.usable = (torque.flags & IPC_TORQUE_VALID) && (torque.flags & IPC_TORQUE_ENABLED);
        for (i = 0; i < AXIS_COUNT; i++) torque_last.tau[i] = torque.tau[i];
        fresh = 1;
    }
    *held = torque_last;
    return fresh;
}

void ocm_link_publish_state(uint64_t t_release, uint32_t period, uint32_t flags, uint32_t n_overrun, uint32_t n_wkc)
{
    static ipc_state_t state;                        /* in ordinary memory: one copy, then 32-bit stores into the OCM */
    int i;

    memset(&state, 0, sizeof(state));
    state.t_release = t_release;
    state.period    = period;
    state.dof       = AXIS_COUNT;
    state.flags     = flags | (drives_enabled() ? IPC_STATE_SERVO_ON : 0u);
    for (i = 0; i < AXIS_COUNT; i++) {
        unsigned short cia = cia402_state_from_statusword(drive_in[i]->statusword);
        if (cia == CIA402_SWITCH_ON_DISABLED || cia == CIA402_NOT_READY_TO_SWITCH_ON) state.not_ready |= 1u << i;
        state.status[i] = drive_in[i]->statusword;
        state.q[i]      = axis_q_rad(i, drive_in[i]->position_actual);
        state.qd[i]     = axis_qd_rads(i, drive_in[i]->velocity_actual);
        state.tau[i]         = axis_tau_nm(i, drive_in[i]->torque_actual);
        state.tau_act_raw[i] = drive_in[i]->torque_actual;
        state.tau_cmd_raw[i] = drive_out[i]->target_torque;      /* what this cycle's axis_control_update() wrote */
    }
    state.cmd_done       = command_done;
    state.cfg_crc        = 0u;                       /* shortcut: no [CONTRACT] section in INDY7.cfg to take a CRC of yet */
    state.torque_period  = torque_period;
    state.n_torque_stale = axis_control_torque_stale_total();
    state.n_overrun      = n_overrun;                /* n_late and n_missed stay 0: the R5 has no send deadline yet */
    state.n_wkc          = n_wkc;
    IPC_WRITE(SLOT_STATE, r5_epoch, &state);
}
