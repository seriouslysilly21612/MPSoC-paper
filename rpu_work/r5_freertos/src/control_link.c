/*
 * control_link.c  --  see control_link.h.
 */
#include <stdint.h>
#include <string.h>
#include "xil_printf.h"
#include "app_config.h"
#include "control_link.h"
#include "ocm_link.h"
#include "axis_control.h"
#include "event_log.h"
#include "ctrl/control_ctrl.h"
#include "ctrl/ctrl_port.h"

_Static_assert(EVENT_VALUES >= 16, "the report of control_link_report() has 16 values");

/* the bus has the joints of the controller (the Indy7).  An expression, not a condition of the preprocessor:
   IPC_CTRL_DOF is an enumerator, which #if takes for 0 */
#define BUS_IS_ARM  (AXIS_COUNT == IPC_CTRL_DOF)

static int built;
static control_out_t held;                           /* of the last step with the arm of the bus: for the drives.
                                                        Nothing with a simulated arm: their drives stay off */
static int      held_fresh;                          /* ... and not yet given to them */
static int      sim_arm;                             /* the arm of the last step was the simulated one */
/* of the window of a report */
static uint32_t n_steps, n_no_state, n_commands, n_refused, step_ns_max;
/* from the reports: counts so far, and of the window the counts at its begin and the longest plan and IK */
static uint32_t n_faults, n_drive_stops, n_goals, n_goals_before;
static uint32_t n_plans, n_iks, n_plan_late, n_plans_before, n_iks_before, plan_us_max, ik_us_max;

int control_link_start(void)
{
    built = control_ctrl_build();
    ctrl_port_loop();                                /* from here the classes print nothing */
    if (!built) {
        xil_printf("[control] FAIL: the controller could not be built (the C heap, _HEAP_SIZE of lscript.ld, or the"
                   " [ctrl] lines above): the cycle runs without it and the drives stay off\r\n");
        return 0;
    }
    xil_printf("[control] the control task runs here, after the send of every cycle: check value 0x%08x; the arm"
               " is %s\r\n", (unsigned)control_ctrl_check_value(),
               BUS_IS_ARM ? "that of the bus, or the one the APU simulates (Sim state slot)"
                                          : "the one the APU simulates (Sim state slot): this bus has not its joints");
    return 1;
}

void control_link_drives(void)
{
    double tau[AXIS_COUNT];
    int i, usable = 0;

    for (i = 0; i < AXIS_COUNT; i++) tau[i] = 0.0;
    if (BUS_IS_ARM) {
        if (held.servo == IPC_CMD_SERVO_ON)       axis_control_servo(1);
        else if (held.servo == IPC_CMD_SERVO_OFF) axis_control_servo(0);
        usable = held.torque;
        for (i = 0; i < AXIS_COUNT; i++) tau[i] = held.tau[i];
    }
    held.servo = 0;
    axis_control_torque_input(held_fresh, usable, tau);
    held_fresh = 0;
}

int control_link_step(uint32_t period, int bus_state, int32_t *traj_ns, int32_t *ctrl_ns)
{
    static control_in_t     in;
    static control_out_t    out;
    static ipc_report_t     report;
    static ipc_command_t    command;
    static ipc_goal_t       goal;
    static ipc_sim_state_t  sim;
    int i, sim_new;
    uint32_t ns;

    if (!built) return 0;
    memset(&in, 0, sizeof(in));
    in.period  = period;
    in.command = ocm_link_take_command(&command) ? &command : NULL;
    in.goal    = ocm_link_take_goal(&goal) ? &goal : NULL;
    sim_new = ocm_link_take_sim_state(&sim);
    if (sim_new >= 0) {
        if (!sim_arm) {                              /* the arm is no longer that of the bus: its drives off, and */
            axis_control_servo(0);                   /* nothing of the controller for them */
            memset(&held, 0, sizeof(held));
            held_fresh = 0;
        }
        sim_arm = 1;
        in.sim_arm   = 1;
        in.state     = (sim_new == 1);               /* a period of the control task per state of the arm */
        in.servo_on  = (sim.flags & IPC_SIM_SERVO_ON) != 0u;
        in.not_ready = sim.not_ready;
        in.sim_step  = sim.step;
        for (i = 0; i < IPC_CTRL_DOF; i++) {
            in.q[i]   = sim.q[i];
            in.qd[i]  = sim.qd[i];
            in.tau[i] = sim.tau[i];
        }
    } else {
        sim_arm = 0;
        if (BUS_IS_ARM) {                            /* on another bus there is no state */
            const ipc_state_t *state = ocm_link_state();     /* what this cycle's frame brought */

            in.state     = bus_state;
            in.servo_on  = (state->flags & IPC_STATE_SERVO_ON) != 0u;
            in.not_ready = state->not_ready;
            for (i = 0; i < IPC_CTRL_DOF; i++) {
                in.q[i]   = state->q[i];
                in.qd[i]  = state->qd[i];
                in.tau[i] = state->tau[i];
            }
        }
    }

    control_ctrl_step(&in, &out, &report);

    if (in.sim_arm) {
        if (in.state) {
            static ipc_sim_torque_t torque;

            torque.step  = sim.step;
            torque.flags = (out.torque ? IPC_SIM_TORQUE_USABLE : 0u) |
                           ((report.state & IPC_CTL_SERVO_ASKED) ? IPC_SIM_SERVO_ASKED : 0u);
            for (i = 0; i < IPC_CTRL_DOF; i++) torque.tau[i] = out.tau[i];
            ocm_link_publish_sim_torque(&torque);
        }
    } else {
        held = out;
        held_fresh = 1;
    }
    /* with a simulated arm, a period without a new state reports only a command that it took: the APU, which steps
       that arm, then finds the report of every period that had a state, however late it looks */
    if (!in.sim_arm || in.state || in.command != NULL) ocm_link_publish_report(&report);

    *traj_ns = (report.state & IPC_CTL_SAMPLE) ? (int32_t)report.traj_ns : -1;
    *ctrl_ns = (report.in.flags & IPC_SHADOW_CTRL) ? (int32_t)report.ctrl_ns : -1;
    n_steps++;
    if (!(report.state & IPC_CTL_STATE)) n_no_state++;
    if (in.command != NULL) {
        n_commands++;
        if (report.cmd_result != IPC_DONE_OK) n_refused++;
    }
    ns = report.traj_ns + report.ctrl_ns;
    if (ns > step_ns_max) step_ns_max = ns;
    if (report.n_plans != n_plans && report.plan_us > plan_us_max) plan_us_max = report.plan_us;   /* a new plan: its time */
    if (report.n_iks != n_iks && report.ik_us > ik_us_max) ik_us_max = report.ik_us;
    n_plans = report.n_plans;
    n_iks = report.n_iks;
    n_plan_late = report.n_plan_late;
    n_faults = report.n_ctrl_faults;
    n_drive_stops = report.n_drive_stops;
    n_goals = report.n_goals;
    return 1;
}

void control_link_report(void)
{
    event_t *e = event_reserve(EVENT_CONTROL_REPORT, 0);

    if (e != NULL) {
        e->value[0] = built;
        e->value[1] = (int32_t)n_steps;
        e->value[2] = (int32_t)n_no_state;
        e->value[3] = (int32_t)n_commands;
        e->value[4] = (int32_t)n_refused;
        e->value[5] = (int32_t)(step_ns_max / 1000u);
        e->value[6] = (int32_t)ctrl_port_lines_held();
        e->value[7] = (int32_t)(n_plans - n_plans_before);
        e->value[8] = (int32_t)plan_us_max;
        e->value[9] = (int32_t)(n_iks - n_iks_before);
        e->value[10] = (int32_t)ik_us_max;
        e->value[11] = (int32_t)n_plan_late;
        e->value[12] = (int32_t)(n_goals - n_goals_before);
        e->value[13] = (int32_t)n_faults;
        e->value[14] = (int32_t)n_drive_stops;
        e->value[15] = sim_arm;
        event_publish();
    }
    n_steps = 0u; n_no_state = 0u; n_commands = 0u; n_refused = 0u; step_ns_max = 0u;
    n_plans_before = n_plans; n_iks_before = n_iks; plan_us_max = 0u; ik_us_max = 0u; n_goals_before = n_goals;
}
