/* host test of src/control_link.c: the control task of the R5 between the slots of the APU and the drives, with
 * stand-ins for the slots (ocm_link.c), the axes (axis_control.c) and the controller (ctrl/control_ctrl.cpp).  From
 * r5_freertos/, for the L7N bench and for the Indy7:
 *   gcc -std=gnu11 -Wall -DHOST_TEST -DCONTROL_TASK=1 -Itest/stub -Isrc test/test_control_link.c src/control_link.c -o /tmp/t_control && /tmp/t_control
 *   gcc -std=gnu11 -Wall -DHOST_TEST -DCONTROL_TASK=1 -DROBOT_TARGET=1 -Itest/stub -Isrc test/test_control_link.c src/control_link.c -o /tmp/t_control && /tmp/t_control
 * It checks: a controller that cannot be built never runs and the drives get nothing; a command and a goal reach
 * the step when they are new; with a state of the simulated arm the step has it and its torque goes back with the
 * step number, without a new one the step has no state and nothing goes back, and the drives of the bus stay off;
 * with the arm of the bus (the Indy7 only) the step has the State of the cycle, and the drives get its torque and
 * its servo request in the next cycle, once; on the bench without a simulated arm there is no state; the report. */
#include <stdio.h>
#include <string.h>
#include "control_link.h"
#include "ocm_link.h"
#include "axis_control.h"
#include "event_log.h"
#include "ctrl/control_ctrl.h"
#include "ctrl/ctrl_port.h"

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

/* ---- the controller ---- */
static int can_build, steps;
static control_in_t  stepped;
static ipc_command_t stepped_command;
static ipc_goal_t    stepped_goal;
static int           had_command, had_goal;
static control_out_t answer;
static ipc_report_t  report;
int      control_ctrl_build(void) { return can_build; }
uint32_t control_ctrl_check_value(void) { return 0xC0FFEE02u; }
void     control_ctrl_step(const control_in_t *in, control_out_t *out, ipc_report_t *r)
{
    steps++;
    stepped = *in;
    had_command = in->command != NULL;
    had_goal = in->goal != NULL;
    if (had_command) stepped_command = *in->command;
    if (had_goal) stepped_goal = *in->goal;
    *out = answer;
    *r = report;
    r->in.period = in->period;
}
void     ctrl_port_loop(void) { }
uint32_t ctrl_port_lines_held(void) { return 2u; }

/* ---- the slots ---- */
static int have_command, have_goal, sim_result = -1, sim_torques, reports;
static int writes, report_at, sim_torque_at;        /* the order in which the two slots were written */
static ipc_command_t    command;
static ipc_goal_t       goal;
static ipc_sim_state_t  sim;
static ipc_sim_torque_t sim_torque;
static ipc_report_t     published;
static ipc_state_t      bus;
static int reads, command_read_at, goal_read_at, sim_read_at;   /* the order in which the three slots were read */
int  ocm_link_take_command(ipc_command_t *c) { command_read_at = ++reads; if (!have_command) return 0; have_command = 0; *c = command; return 1; }
int  ocm_link_take_goal(ipc_goal_t *g) { goal_read_at = ++reads; if (!have_goal) return 0; have_goal = 0; *g = goal; return 1; }
int  ocm_link_take_sim_state(ipc_sim_state_t *s) { sim_read_at = ++reads; if (sim_result == 1) *s = sim; return sim_result; }
void ocm_link_publish_sim_torque(const ipc_sim_torque_t *t) { sim_torques++; sim_torque = *t; sim_torque_at = ++writes; }
void ocm_link_publish_report(const ipc_report_t *r) { reports++; published = *r; report_at = ++writes; }
const ipc_state_t *ocm_link_state(void) { return &bus; }

/* ---- the axes ---- */
static int servo_calls, servo_last = -1, inputs, input_fresh, input_usable;
static double input_tau[AXIS_COUNT];
void axis_control_servo(int on) { servo_calls++; servo_last = on; }
void axis_control_torque_input(int fresh, int usable, const double tau[AXIS_COUNT])
{
    inputs++; input_fresh = fresh; input_usable = usable; memcpy(input_tau, tau, sizeof(input_tau));
}

/* ---- events, console ---- */
static event_t an_event; static int events, fail_lines;
event_t *event_reserve(uint32_t id, int budgeted) { (void)budgeted; memset(&an_event, 0, sizeof(an_event)); an_event.id = id; return &an_event; }
void event_publish(void) { events++; }
void xil_printf(const char *f, ...) { if (strstr(f, "FAIL") != NULL) fail_lines++; }

int main(void)
{
    int32_t traj_ns = 7, ctrl_ns = 7;
    int i;

    /* not built: said so, never runs, the drives get no torque */
    CHECK(control_link_start() == 0 && fail_lines == 1);
    CHECK(control_link_step(1, 1, &traj_ns, &ctrl_ns) == 0 && steps == 0 && reports == 0 && traj_ns == 7);
    control_link_drives();
    CHECK(inputs == 1 && input_usable == 0 && input_fresh == 0 && input_tau[0] == 0.0 && servo_calls == 0);

    can_build = 1;
    CHECK(control_link_start() == 1 && fail_lines == 1);

    /* a simulated arm: its new state is the state of the step, and the torque goes back with its step number */
    sim_result = 1;
    sim.step = 41; sim.flags = IPC_SIM_SERVO_ON; sim.not_ready = 0x4u;
    for (i = 0; i < IPC_CTRL_DOF; i++) { sim.q[i] = 0.1 * (i + 1); sim.qd[i] = -0.2 * (i + 1); sim.tau[i] = 3.0 + i; }
    command.seq = 9; command.cmd = IPC_CMD_START; command.arg = IPC_START_HOME; command.delta = 0.006; have_command = 1;
    goal.p[0] = 0.4; goal.p[2] = 0.3; have_goal = 1;
    answer.torque = 1; answer.servo = IPC_CMD_SERVO_ON;
    for (i = 0; i < IPC_CTRL_DOF; i++) answer.tau[i] = 10.0 + i;
    memset(&report, 0, sizeof(report));
    report.state = IPC_CTL_STATE | IPC_CTL_SIM_ARM | IPC_CTL_SAMPLE | IPC_CTL_SERVO_ASKED;
    report.in.flags = IPC_SHADOW_CTRL; report.traj_ns = 60000; report.ctrl_ns = 180000;
    report.n_plans = 1; report.plan_us = 900; report.n_iks = 1; report.ik_us = 7000; report.n_goals = 1;
    CHECK(control_link_step(100, 0, &traj_ns, &ctrl_ns) == 1 && steps == 1 && traj_ns == 60000 && ctrl_ns == 180000);
    CHECK(stepped.period == 100 && stepped.state == 1 && stepped.sim_arm == 1 && stepped.servo_on == 1 &&
          stepped.not_ready == 0x4u && stepped.sim_step == 41 && stepped.q[5] == 0.1 * 6 && stepped.qd[0] == -0.2 &&
          stepped.tau[2] == 5.0);
    CHECK(had_command && stepped_command.seq == 9 && stepped_command.cmd == IPC_CMD_START && stepped_command.delta == 0.006);
    CHECK(had_goal && stepped_goal.p[0] == 0.4 && stepped_goal.p[2] == 0.3);
    CHECK(sim_torques == 1 && sim_torque.step == 41 && sim_torque.flags == (IPC_SIM_TORQUE_USABLE | IPC_SIM_SERVO_ASKED) &&
          sim_torque.tau[0] == 10.0 && sim_torque.tau[5] == 15.0);
    CHECK(reports == 1 && published.in.period == 100 && published.state == report.state);
    CHECK(report_at == 1 && sim_torque_at == 2);    /* the report first: whoever finds this torque finds its report */
    /* read in the reverse of the APU's order of writing: with a state every command and goal before it, with a goal
       the start of its TRACK */
    CHECK(sim_read_at == 1 && goal_read_at == 2 && command_read_at == 3);
    /* ... and the drives of the bus stay off: no torque for them, nothing asked but off, once */
    control_link_drives();
    CHECK(inputs == 2 && input_usable == 0 && input_tau[0] == 0.0 && servo_calls == 1 && servo_last == 0);

    /* no new state of it: the step has none, and no torque goes back; neither command nor goal is new */
    sim_result = 0;
    report.state = IPC_CTL_SIM_ARM; report.in.flags = 0; report.traj_ns = 0; report.ctrl_ns = 0;
    answer.torque = 0; answer.servo = 0;
    CHECK(control_link_step(101, 1, &traj_ns, &ctrl_ns) == 1 && steps == 2 && traj_ns == -1 && ctrl_ns == -1);
    CHECK(stepped.state == 0 && stepped.sim_arm == 1 && !had_command && !had_goal && sim_torques == 1);
    CHECK(reports == 1);                    /* nothing to report: the report of the period with a state stays to be read */
    command.seq = 20; command.cmd = IPC_CMD_HOLD_ON; have_command = 1;
    CHECK(control_link_step(101, 1, &traj_ns, &ctrl_ns) == 1 && stepped.state == 0 && had_command && sim_torques == 1);
    CHECK(reports == 1);                    /* nor with a command that the period took: the next report answers it */
    /* a torque that is not usable and no request: said so */
    sim_result = 1; sim.step = 42;
    report.state = IPC_CTL_STATE | IPC_CTL_SIM_ARM;
    CHECK(control_link_step(102, 1, &traj_ns, &ctrl_ns) == 1 && sim_torques == 2 && sim_torque.step == 42 && sim_torque.flags == 0u);
    CHECK(reports == 2 && published.in.period == 102 && report_at < sim_torque_at);
    control_link_drives();
    CHECK(servo_calls == 1 && input_usable == 0);

    /* the report: 4 periods, 2 without a state, 2 commands (none refused), the longest 240 us, the planner, the arm */
    control_link_report();
    CHECK(events == 1 && an_event.id == EVENT_CONTROL_REPORT && an_event.value[0] == 1 && an_event.value[1] == 4 &&
          an_event.value[2] == 2 && an_event.value[3] == 2 && an_event.value[4] == 0 && an_event.value[5] == 240 &&
          an_event.value[6] == 2);
    CHECK(an_event.value[7] == 1 && an_event.value[8] == 900 && an_event.value[9] == 1 && an_event.value[10] == 7000 &&
          an_event.value[11] == 0 && an_event.value[12] == 1 && an_event.value[15] == 1);

    /* no simulated arm (the run of the APU that is up has written none) */
    sim_result = -1;
    bus.flags = IPC_STATE_OP | IPC_STATE_SERVO_ON; bus.not_ready = 0x20u;
    for (i = 0; i < IPC_CTRL_DOF; i++) { bus.q[i] = 1.0 + i; bus.qd[i] = 0.5 * i; bus.tau[i] = -1.0 * i; }
    answer.torque = 1; answer.servo = IPC_CMD_SERVO_ON;
    report.state = IPC_CTL_STATE | IPC_CTL_ENABLED; report.in.flags = IPC_SHADOW_CTRL; report.ctrl_ns = 170000;
    report.cmd_result = IPC_REFUSED_FAULT; report.n_ctrl_faults = 2; report.n_drive_stops = 3;
    command.seq = 10; command.cmd = IPC_CMD_SERVO_ON; have_command = 1;
    CHECK(control_link_step(103, 1, &traj_ns, &ctrl_ns) == 1 && traj_ns == -1 && ctrl_ns == 170000 && sim_torques == 2);
    if (AXIS_COUNT == IPC_CTRL_DOF) {
    /* the arm of the bus: the State of the cycle, and the drives get the torque and the request in the next, once */
    CHECK(stepped.state == 1 && stepped.sim_arm == 0 && stepped.servo_on == 1 && stepped.not_ready == 0x20u &&
          stepped.q[3] == 4.0 && stepped.qd[2] == 1.0 && stepped.tau[5] == -5.0);
    control_link_drives();
    CHECK(servo_calls == 2 && servo_last == 1 && input_fresh == 1 && input_usable == 1 && input_tau[0] == 10.0 && input_tau[5] == 15.0);
    control_link_drives();                  /* a cycle without a step between (the firmware stopping) */
    CHECK(servo_calls == 2 && input_fresh == 0 && input_usable == 1 && input_tau[5] == 15.0);
    CHECK(control_link_step(104, 0, &traj_ns, &ctrl_ns) == 1 && stepped.state == 0);      /* the frame did not come back */
    CHECK(reports == 4);                    /* ... and the period reports all the same: no simulated arm waits for a report */
    answer.servo = IPC_CMD_SERVO_OFF; answer.torque = 0;
    CHECK(control_link_step(105, 1, &traj_ns, &ctrl_ns) == 1 && stepped.state == 1);
    control_link_drives();
    CHECK(servo_calls == 3 && servo_last == 0 && input_fresh == 1 && input_usable == 0);
    } else {
    /* the bench has not the joints of the controller: no state, and its drives get nothing */
    CHECK(stepped.state == 0 && stepped.sim_arm == 0);
    control_link_drives();
    CHECK(servo_calls == 1 && input_usable == 0 && input_tau[0] == 0.0);
    }
    control_link_report();
    CHECK(events == 2 && an_event.value[3] == 1 && an_event.value[4] == 1 && an_event.value[13] == 2 &&
          an_event.value[14] == 3 && an_event.value[15] == 0 && an_event.value[12] == 0);

    /* from the arm of the bus to a simulated one: the drives off, and nothing of the controller reaches them, neither
       what the last step with the bus decided nor what a step with the simulated arm does */
    sim_result = -1;
    answer.torque = 1; answer.servo = IPC_CMD_SERVO_ON;
    servo_calls = 0; servo_last = -1;
    i = reports;
    CHECK(control_link_step(110, 1, &traj_ns, &ctrl_ns) == 1);
    CHECK(reports == i + 1);                /* without a simulated arm every period reports, the bench's (no state) too */
    sim_result = 1; sim.step = 50;
    CHECK(control_link_step(111, 1, &traj_ns, &ctrl_ns) == 1 && servo_calls == 1 && servo_last == 0);
    control_link_drives();
    CHECK(servo_calls == 1 && input_usable == 0 && input_fresh == 0 && input_tau[0] == 0.0);
    sim.step = 51;
    CHECK(control_link_step(112, 1, &traj_ns, &ctrl_ns) == 1);
    control_link_drives();
    CHECK(servo_calls == 1 && input_usable == 0 && input_fresh == 0 && input_tau[0] == 0.0);

    printf(fails ? "test_control_link: %d FAILED\n" : "test_control_link OK\n", fails);
    return fails != 0;
}
