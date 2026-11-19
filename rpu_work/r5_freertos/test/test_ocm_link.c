/* host test of src/ocm_link.c: the R5's side of the slots, against an APU side written here with the same slot
 * protocol (ipc/ipc_slot.h).  The OCM window is memory of the test at its address.  From r5_freertos/:
 *   gcc -std=gnu11 -Wall -DHOST_TEST -Itest/stub -Isrc -Isrc/soem -Isrc/osal -Isrc/osal/freertos -Isrc/oshw/freertos \
 *       test/test_ocm_link.c src/ocm_link.c -lm -o /tmp/t_ocm && /tmp/t_ocm
 * It checks: a command left in the slot by an earlier run is not taken; a command is taken once, whole; a goal is
 * taken once; the State slot has the values of the drives and is what ocm_link_state() gives; the Sim torque and
 * the Report slot arrive at the APU; and which arm it is: that of the bus until the run of the APU that is up has
 * written a state of a simulated arm, the simulated one from then on, also while that APU writes or has stopped
 * writing, and that of the bus again when another run of the APU is up. */
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include "xil_mmu.h"
#include "xstatus.h"
#include "ocm_link.h"
#include "ipc/ipc_slot.h"
#include "cia402_drive.h"
#include "axis_control.h"

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

#define HEAD      ((volatile ipc_head_t *)OCM_LINK_BASE)
#define SLOT(off) ((volatile uint32_t *)(OCM_LINK_BASE + (off)))

/* ---- the BSP, the drives, the clock ---- */
u32 Xil_SetMPURegion(INTPTR addr, u32 size, u32 attrib) { (void)addr; (void)size; (void)attrib; return (u32)XST_SUCCESS; }
void xil_printf(const char *f, ...) { (void)f; }
static drive_outputs_t out[AXIS_COUNT]; static drive_inputs_t in[AXIS_COUNT];
drive_outputs_t *drive_out[AXIS_COUNT]; drive_inputs_t *drive_in[AXIS_COUNT];
unsigned short cia402_state_from_statusword(unsigned short s)
{
    if ((s & 0x6F) == 0x27) return CIA402_OPERATION_ENABLED;
    if ((s & 0x4F) == 0x40) return CIA402_SWITCH_ON_DISABLED;
    return CIA402_READY_TO_SWITCH_ON;
}
uint32_t axis_control_torque_stale_total(void) { return 5u; }
uint64_t host_test_counter(void) { return 0x100000000ull; }

/* ---- the APU ---- */
static uint32_t apu_epoch;
static void apu_start(void)
{
    volatile uint32_t *slots[5] = { SLOT(IPC_OFF_TORQUE), SLOT(IPC_OFF_SHADOW_IN), SLOT(IPC_OFF_COMMAND), SLOT(IPC_OFF_GOAL),
                                    SLOT(IPC_OFF_SIM_STATE) };
    apu_epoch = ipc_side_start(&HEAD->apu, slots, 5);
}

int main(void)
{
    ipc_command_t command, taken;
    ipc_goal_t goal, goal_taken;
    ipc_sim_state_t sim, sim_taken;
    ipc_sim_torque_t torque, torque_read;
    ipc_report_t report, report_read;
    ipc_state_t state_read;
    link_faults_t faults = { 1u, 2u, 3u, 4u };
    uint32_t stamp[2], gen;
    int i;

    if (mmap((void *)OCM_LINK_BASE, OCM_LINK_SIZE, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == MAP_FAILED) {
        printf("NOT RUN: no memory at 0x%lx\n", (unsigned long)OCM_LINK_BASE);
        return 2;
    }
    for (i = 0; i < AXIS_COUNT; i++) { drive_out[i] = &out[i]; drive_in[i] = &in[i]; in[i].statusword = 0x0027; }

    /* a command of an earlier run of this side is in the slot when it starts: not taken */
    apu_start();
    memset(&command, 0, sizeof(command));
    command.seq = 3; command.cmd = IPC_CMD_SERVO_ON;
    IPC_WRITE(SLOT(IPC_OFF_COMMAND), apu_epoch, &command);
    CHECK(ocm_link_start() == 1 && ipc_load32(&HEAD->r5.magic) == IPC_MAGIC && ipc_load32(&HEAD->r5.layout) == IPC_LAYOUT);
    /* what an earlier run of the R5 left in its slots reads as being written, in all four */
    CHECK((ipc_load32(SLOT(IPC_OFF_STATE)) & 1u) && (ipc_load32(SLOT(IPC_OFF_SHADOW_OUT)) & 1u) &&
          (ipc_load32(SLOT(IPC_OFF_SIM_TORQUE)) & 1u) && (ipc_load32(SLOT(IPC_OFF_REPORT)) & 1u));
    memset(&taken, 0x55, sizeof(taken));
    CHECK(ocm_link_take_command(&taken) == 0 && taken.seq == 0x55555555u);

    /* a command: once, whole */
    command.seq = 4; command.cmd = IPC_CMD_START; command.arg = IPC_START_HOME | IPC_START_TARGET; command.delta = 0.006;
    for (i = 0; i < IPC_CTRL_DOF; i++) { command.q_ready[i] = 0.1 * i; command.target[i] = -0.2 * i; }
    IPC_WRITE(SLOT(IPC_OFF_COMMAND), apu_epoch, &command);
    CHECK(ocm_link_take_command(&taken) == 1 && memcmp(&taken, &command, sizeof(command)) == 0);
    CHECK(ocm_link_take_command(&taken) == 0);

    /* a goal: once */
    CHECK(ocm_link_take_goal(&goal_taken) == 0);
    goal.p[0] = 0.4; goal.p[1] = -0.1; goal.p[2] = 0.35;
    IPC_WRITE(SLOT(IPC_OFF_GOAL), apu_epoch, &goal);
    CHECK(ocm_link_take_goal(&goal_taken) == 1 && memcmp(&goal_taken, &goal, sizeof(goal)) == 0);
    CHECK(ocm_link_take_goal(&goal_taken) == 0);
    IPC_WRITE(SLOT(IPC_OFF_GOAL), apu_epoch, &goal);           /* the same numbers again: a new goal all the same */
    CHECK(ocm_link_take_goal(&goal_taken) == 1);

    /* the State: the values of the drives, the command taken, and what ocm_link_state() gives */
    in[1].statusword = 0x0040; in[0].position_actual = 1000; in[0].torque_actual = 7;
    CHECK(ocm_link_publish_state(77ull, 12u, IPC_STATE_OP | IPC_STATE_CONTROL, &faults) == (IPC_STATE_OP | IPC_STATE_CONTROL));
    CHECK(IPC_READ(SLOT(IPC_OFF_STATE), &HEAD->r5, &state_read, stamp) == IPC_OK && state_read.period == 12u &&
          state_read.flags == (IPC_STATE_OP | IPC_STATE_CONTROL) && state_read.not_ready == 0x2u && state_read.cmd_done == 4u &&
          state_read.dof == AXIS_COUNT && state_read.n_late == 1u && state_read.n_wkc == 4u && state_read.tau_act_raw[0] == 7 &&
          state_read.n_torque_stale == 5u && state_read.q[0] != 0.0);
    CHECK(memcmp(ocm_link_state(), &state_read, sizeof(state_read)) == 0);
    in[1].statusword = 0x0027;
    CHECK(ocm_link_publish_state(78ull, 13u, IPC_STATE_OP, &faults) == (IPC_STATE_OP | IPC_STATE_SERVO_ON) &&
          (ocm_link_state()->flags & IPC_STATE_SERVO_ON) && ocm_link_state()->not_ready == 0u);

    /* the Sim torque and the Report */
    memset(&torque, 0, sizeof(torque)); torque.step = 9; torque.flags = IPC_SIM_TORQUE_USABLE; torque.tau[5] = 1.5;
    ocm_link_publish_sim_torque(&torque);
    CHECK(IPC_READ(SLOT(IPC_OFF_SIM_TORQUE), &HEAD->r5, &torque_read, stamp) == IPC_OK && memcmp(&torque_read, &torque, sizeof(torque)) == 0);
    memset(&report, 0, sizeof(report)); report.in.period = 13; report.state = IPC_CTL_STATE; report.qdd_ref[5] = -2.5;
    ocm_link_publish_report(&report);
    CHECK(IPC_READ(SLOT(IPC_OFF_REPORT), &HEAD->r5, &report_read, stamp) == IPC_OK && memcmp(&report_read, &report, sizeof(report)) == 0);

    /* which arm: that of the bus while this run of the APU has written no state of a simulated one */
    memset(&sim_taken, 0x55, sizeof(sim_taken));
    CHECK(ocm_link_take_sim_state(&sim_taken) == -1 && sim_taken.step == 0x55555555u);
    memset(&sim, 0, sizeof(sim)); sim.step = 1; sim.flags = IPC_SIM_SERVO_ON; sim.q[2] = 0.7;
    IPC_WRITE(SLOT(IPC_OFF_SIM_STATE), apu_epoch, &sim);
    CHECK(ocm_link_take_sim_state(&sim_taken) == 1 && memcmp(&sim_taken, &sim, sizeof(sim)) == 0);
    CHECK(ocm_link_take_sim_state(&sim_taken) == 0);           /* nothing new: still the simulated arm */
    /* the APU in the middle of a write: still the simulated arm, and nothing taken */
    gen = ipc_load32(SLOT(IPC_OFF_SIM_STATE));
    ipc_store32(SLOT(IPC_OFF_SIM_STATE), gen | 1u);
    sim_taken.step = 0x55555555u;
    CHECK(ocm_link_take_sim_state(&sim_taken) == 0 && sim_taken.step == 0x55555555u);
    ipc_store32(SLOT(IPC_OFF_SIM_STATE), gen);
    sim.step = 2;
    IPC_WRITE(SLOT(IPC_OFF_SIM_STATE), apu_epoch, &sim);
    CHECK(ocm_link_take_sim_state(&sim_taken) == 1 && sim_taken.step == 2u);
    /* the APU side goes down in a restart (its magic 0), and comes up as another run that has no simulated arm:
       the arm of the bus, and the state of the run before is not taken */
    ipc_store32(&HEAD->apu.magic, 0u);
    CHECK(ocm_link_take_sim_state(&sim_taken) == -1);
    apu_start();
    sim_taken.step = 0x55555555u;
    CHECK(ocm_link_take_sim_state(&sim_taken) == -1 && sim_taken.step == 0x55555555u);
    /* that run writes one: the simulated arm again */
    sim.step = 1;
    IPC_WRITE(SLOT(IPC_OFF_SIM_STATE), apu_epoch, &sim);
    CHECK(ocm_link_take_sim_state(&sim_taken) == 1 && sim_taken.step == 1u);
    CHECK(ocm_link_take_sim_state(&sim_taken) == 0);

    printf(fails ? "test_ocm_link: %d FAILED\n" : "test_ocm_link OK\n", fails);
    return fails != 0;
}
