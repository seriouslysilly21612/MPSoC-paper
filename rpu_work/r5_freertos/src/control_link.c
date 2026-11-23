/*
 * control_link.c  --  see control_link.h.
 */
#include <stdint.h>
#include <string.h>
#include "xil_printf.h"
#include "app_config.h"
#include "control_link.h"
#include "ocm_link.h"
#include "sys_counter.h"
#include "axis_control.h"
#include "event_log.h"
#include "ctrl/control_ctrl.h"
#include "ctrl/ctrl_port.h"

#if defined(__arm__) && !defined(HOST_TEST)
#define MEMORY_BARRIER()    __asm__ volatile ("dmb" : : : "memory")
#else
#define MEMORY_BARRIER()    __asm__ volatile ("" : : : "memory")
#endif

_Static_assert(EVENT_VALUES >= 16, "the report of control_link_report() has 16 values");

/* the clock of the receiver's cost (eq. (6)): the low word, one read (the cost takes differences mod 2^32, a period
 * is far below its 42.9 s); each mark of the cost holds one read of it, obs_clock_ticks */
static uint64_t obs_clock(void) { return sys_counter_low(); }
static uint32_t obs_clock_ticks;

/* one read of the clock: the least of 16 differences of two reads in a row */
static uint32_t obs_clock_cost(void)
{
    uint32_t i, a, d, least = 0xFFFFFFFFu;

    for (i = 0u; i < 16u; i++) {
        a = sys_counter_low();
        d = sys_counter_low() - a;
        if (d < least)
            least = d;
    }
    return least;
}

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

/* Between the EtherCAT task and the control task (control_task.c), on one core, the EtherCAT task above it.  The
 * EtherCAT task runs only while the control task waits or was interrupted, so it hands a cycle over, and takes what
 * a step decided, only when the two generations are equal (the step it handed over is done): then the control task
 * is in none of hand, result, held and drives_off.  The control task writes them before gen_done. */
static volatile uint32_t gen_given, gen_done;
static struct { uint32_t period; int bus_state; uint64_t release; ipc_state_t state; } hand;
static struct { uint32_t period, end; int32_t traj_ns, ctrl_ns; } result;
static int      drives_off;                         /* the arm became the simulated one: the drives of the bus off */
static int      given_usable;                       /* what the drives got last: they keep it while a step is late */
static double   given_tau[AXIS_COUNT];
static uint32_t n_late, late_run;                   /* cycles that found the step of the cycle before not done */

int control_link_start(void)
{
    built = control_ctrl_build((volatile uint32_t *)OCM_LINK_BASE, (double)SYSCNT_HZ);
    if (built) {
        obs_clock_ticks = obs_clock_cost();
        control_ctrl_set_obs_clock(obs_clock);
    }
    ctrl_port_loop();                                /* from here the classes print nothing */
    if (built == 2) {
        xil_printf("[control] the receiver of the camera results cannot use its numbers of this build ([CONTRACT],"
                   " APP_*): the methods other than M1 are refused (IPC_CMD_OBS)\r\n");
        built = 1;
    }
    if (!built) {
        xil_printf("[control] FAIL: the controller could not be built (the C heap, _HEAP_SIZE of lscript.ld, or the"
                   " [ctrl] lines above): the cycle runs without it and the drives stay off\r\n");
        return 0;
    }
    xil_printf("[control] the control task runs here, after the send of every cycle: check value 0x%08x, of the"
               " receiver of the camera results 0x%08x; the arm is %s\r\n", (unsigned)control_ctrl_check_value(),
               (unsigned)control_ctrl_obs_check_value(),
               BUS_IS_ARM ? "that of the bus, or the one the APU simulates (Sim state slot)"
                                          : "the one the APU simulates (Sim state slot): this bus has not its joints");
    return 1;
}

int control_link_idle(void)
{
    return gen_done == gen_given;
}

uint32_t control_link_late_total(void)
{
    return n_late;
}

void control_link_drives(void)
{
    int i;

    if (!control_link_idle()) {                      /* the step of the cycle before ran past this release */
        n_late++;
        if (late_run++ == 0u) event_post(EVENT_CONTROL_LATE, 1, (int)hand.period, (int)n_late, 0);
        axis_control_torque_input(0, given_usable, given_tau);   /* not fresh: TORQUE_STALE_LIMIT in a row stop */
        return;
    }
    late_run = 0u;
    if (drives_off) {
        axis_control_servo(0);
        drives_off = 0;
    }
    given_usable = 0;
    for (i = 0; i < AXIS_COUNT; i++) given_tau[i] = 0.0;
    if (BUS_IS_ARM) {
        if (held.servo == IPC_CMD_SERVO_ON)       axis_control_servo(1);
        else if (held.servo == IPC_CMD_SERVO_OFF) axis_control_servo(0);
        given_usable = held.torque;
        for (i = 0; i < AXIS_COUNT; i++) given_tau[i] = held.tau[i];
    }
    held.servo = 0;
    axis_control_torque_input(held_fresh, given_usable, given_tau);
    held_fresh = 0;
}

int control_link_give(uint32_t period, int bus_state, uint64_t release)
{
    if (!built || !control_link_idle()) return 0;
    hand.period    = period;
    hand.bus_state = bus_state;
    hand.release   = release;
    hand.state     = *ocm_link_state();              /* a copy: the next cycle writes its own State there */
    MEMORY_BARRIER();
    gen_given = gen_given + 1u;
    return 1;
}

int control_link_result(uint32_t period, uint32_t *end, int32_t *traj_ns, int32_t *ctrl_ns)
{
    if (!control_link_idle() || result.period != period) return 0;
    *end     = result.end;
    *traj_ns = result.traj_ns;
    *ctrl_ns = result.ctrl_ns;
    return 1;
}

static int step(uint32_t period, int bus_state, uint64_t release, int32_t *traj_ns, int32_t *ctrl_ns);

int control_link_run(void)
{
    const uint32_t gen = gen_given;
    int32_t traj_ns = -1, ctrl_ns = -1;

    if (gen == gen_done) return 0;
    MEMORY_BARRIER();                                /* the hand-over before its generation was written */
    (void)step(hand.period, hand.bus_state, hand.release, &traj_ns, &ctrl_ns);
    result.end     = sys_counter_low();
    result.period  = hand.period;
    result.traj_ns = traj_ns;
    result.ctrl_ns = ctrl_ns;
    MEMORY_BARRIER();
    gen_done = gen;
    return 1;
}

static int step(uint32_t period, int bus_state, uint64_t release, int32_t *traj_ns, int32_t *ctrl_ns)
{
    static float response = -1.0f;                   /* [s] of the period before that had a state; < 0: none */
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
    in.period   = period;
    in.release  = release;
    in.response = response;
    response    = -1.0f;
    /* The state of a simulated arm, the goal, the command: the reverse of the order in which the APU writes them
       (command, goal, state, from one task).  So a state that is here has every command and goal that the APU wrote
       before it, and a run with a simulated arm does not depend on when the APU writes; and a goal that is here has
       the start of its TRACK, which counts the goals that arrive from its start on. */
    sim_new = ocm_link_take_sim_state(&sim);
    in.goal    = ocm_link_take_goal(&goal) ? &goal : NULL;
    in.command = ocm_link_take_command(&command) ? &command : NULL;
    if (sim_new >= 0) {
        if (!sim_arm) {                              /* the arm is no longer that of the bus: its drives off */
            drives_off = 1;                          /* (by the EtherCAT task, control_link_drives), and */
            memset(&held, 0, sizeof(held));          /* nothing of the controller for them */
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
            const ipc_state_t *state = &hand.state;          /* what the frame of that cycle brought */

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

    /* The report before the torque of a simulated arm: an APU that finds the torque of its state then finds the
       report of that period, and nothing overwrites it before the APU has written its next state.  So with a
       simulated arm a period without a new state reports nothing; what it did with a command is in the report of
       the next period that has one. */
    if (!in.sim_arm || in.state) ocm_link_publish_report(&report);
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
    if (in.state)                                    /* release to torque, as the APU's control task measures it */
        response = (float)((double)(sys_counter_now() - release) / (double)SYSCNT_HZ);

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

static int32_t ticks_ns(uint32_t ticks)
{
    return (int32_t)((uint64_t)ticks * 1000000000ull / SYSCNT_HZ);
}

/* the receiver's cost (eq. (6)) in this run of the application (cleared by each IPC_CMD_OBS of a method with the
 * mailbox), once it has measured periods: B, the periods, the longest period; the longest look at the slots by the
 * results taken (0, 1, 2, 3, 4 or more); the longest choice by the results held (0, 1, 2-3, 4-7, 8); the periods that
 * took a result; the most results taken in one period; one read of the clock, which each of the times holds.  [ns] */
static void obs_cost_report(void)
{
    obs_cost_t c;
    event_t *e;
    uint32_t i, more = 0u, held47 = 0u, taken_periods = 0u, most = 0u;

    if (!control_ctrl_obs_cost(&c) || c.periods == 0u)
        return;
    e = event_reserve(EVENT_OBS_COST_REPORT, 0);
    if (e == NULL)
        return;
    for (i = 4u; i <= (uint32_t)OBS_SLOTS; i++)
        if (c.drain_max[i] > more) more = c.drain_max[i];
    for (i = 4u; i < (uint32_t)OBS_HELD; i++)
        if (c.select_max[i] > held47) held47 = c.select_max[i];
    for (i = 1u; i <= (uint32_t)OBS_SLOTS; i++) {
        taken_periods += c.drain_periods[i];
        if (c.drain_periods[i] != 0u) most = i;
    }
    e->value[0] = (int32_t)c.slots;
    e->value[1] = (int32_t)c.periods;
    e->value[2] = ticks_ns(c.total_max);
    for (i = 0u; i < 4u; i++)
        e->value[3 + i] = ticks_ns(c.drain_max[i]);
    e->value[7] = ticks_ns(more);
    e->value[8] = ticks_ns(c.select_max[0]);
    e->value[9] = ticks_ns(c.select_max[1]);
    e->value[10] = ticks_ns(c.select_max[2] > c.select_max[3] ? c.select_max[2] : c.select_max[3]);
    e->value[11] = ticks_ns(held47);
    e->value[12] = ticks_ns(c.select_max[OBS_HELD]);
    e->value[13] = (int32_t)taken_periods;
    e->value[14] = (int32_t)most;
    e->value[15] = ticks_ns(obs_clock_ticks);
    event_publish();
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
    obs_cost_report();
    n_steps = 0u; n_no_state = 0u; n_commands = 0u; n_refused = 0u; step_ns_max = 0u;
    n_plans_before = n_plans; n_iks_before = n_iks; plan_us_max = 0u; ik_us_max = 0u; n_goals_before = n_goals;
}
