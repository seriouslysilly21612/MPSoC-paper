/*
 * control_replay.cpp  --  the control task as the firmware builds it (CONTROL_TASK; src/ctrl/control_ctrl.h), against
 * the control cycle of the application.  The firmware's build of it has another compiler and C library than the
 * application, the headers of src/ctrl/port in place of Linux, its model and its numbers from generated headers;
 * this test feeds it, through the calls the firmware makes, the periods that the application's control cycle went
 * through on a PC (goal-to-torque_MPSoC/tools/control_check -o FILE: the states, the commands and the goals) and
 * compares what it gives.
 *
 *   control_replay FILE [LIMIT [PERIODS]]    LIMIT: largest difference of a number allowed, default 0
 *                                            PERIODS: only the first so many
 *   test/control_replay.sh builds it for a PC and for the R5 (run by qemu-arm) and runs both.
 *
 * The numbers of the firmware (src/ctrl/ctrl_numbers_indy7.h) must be those the file was written with.  In every
 * period the torque and the servo request for the drives and the report must be the file's: the counts, flags and
 * states exactly, the numbers within LIMIT (the times of the report are this run's own).  After every period the
 * planner is served as the planner task of the firmware serves it.  A second controller (ctrl/shadow_ctrl.h) is fed
 * with every report and must give its torque and its reference, bit for bit: it is the same build.  On a PC with
 * the application's compiler the differences to the file are 0; the R5 has no fused multiply-add and another
 * library.
 * Neither a period nor the planner may allocate memory (the build puts counting wrappers in front of malloc, calloc
 * and realloc).  Exit 0 if no check failed.
 *
 * mark() is called before and after the step of every period: a trace of the instructions executed (qemu-arm
 * -d exec) is counted between the two (test/count_periods.py).
 *
 * Built with SLOT_PREEMPT_TEST (a PC only; every_place.h) it replays the file once for every place at which the
 * planner touches a slot after a period, with the step of the next period there, in the middle of what the planner
 * does.  The IK of a goal is then back a period later than in the file, so the motion of TRACK is another and the
 * file's torque is not this run's.  What must hold in every such replay: the second controller on the reports gives
 * the torque and the reference of every period and is never apart (the reports say what was done, whenever the
 * planner's results came); the commands end as in the file; the counts of the report never go back and end at the
 * file's; no plan is late; no memory is taken.
 *
 * With and without that step: the planner is served to its end before a period, so the high-level controller waits
 * for an IK in a period only if it asked for it in that period (or the period is the step in the middle of the
 * planner's work).  An IK that stays asked for is one whose request or whose result was lost between the two.
 */
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "ShadowControl.h"
#include "TrajectoryRT.h"
#include "control_ctrl.h"
#include "ctrl_numbers_indy7.h"
#include "shadow_ctrl.h"
#include "shadow/control_vectors.h"

#ifdef SLOT_PREEMPT_TEST
#include "every_place.h"
#endif

extern "C" {
/* the clock of the classes: here a count of its readings, so that a run is the same every time */
RTTIME read_timer(void)
{
    static RTTIME now = 0;
    return now += 1000;
}

void ctrl_port_log(int level, const char *format, ...)
{
    if (level != eLogWarn && level != eLogError)
        return;
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    printf("\n");
}

__attribute__((noinline, used)) void mark(void) { __asm__ volatile(""); }

static long allocations;
void *__real_malloc(size_t bytes);
void *__real_calloc(size_t count, size_t bytes);
void *__real_realloc(void *block, size_t bytes);
void *__wrap_malloc(size_t bytes) { allocations++; return __real_malloc(bytes); }
void *__wrap_calloc(size_t count, size_t bytes) { allocations++; return __real_calloc(count, bytes); }
void *__wrap_realloc(void *block, size_t bytes) { allocations++; return __real_realloc(block, bytes); }
}

/* what a replay found */
struct tally_t
{
    long report_differs, results_differ, over, back, law, traj, follower_differs, apart, follower_failed, interruptions,
         allocated, ik_owed;
    double largest;                 /* difference of a number to the file's */
    uint32_t largest_at;
};

static const control_vector_t *vectors;
static uint32_t periods, next_period;       /* next_period: the first whose step has not run */
static double limit;
static bool with_file;                      /* the torque and the report must be the file's (not with a step in the middle) */
static tally_t tally;
static ipc_report_t last;                   /* the report of the period before */
static bool in_the_middle;                  /* this step runs in the middle of the planner's work (SLOT_PREEMPT_TEST) */

/* two numbers: within the limit, or both no number */
static void number(double is, double must, uint32_t period)
{
    if (is != is && must != must)
        return;
    const double d = std::fabs(is - must);
    if (!(d <= limit))
        tally.over++;
    if (!(d <= tally.largest))
    {
        tally.largest = d;
        tally.largest_at = period;
    }
}

static void numbers(const double *is, const double *must, int count, uint32_t period)
{
    for (int i = 0; i < count; i++)
        number(is[i], must[i], period);
}

/* the step of the next period */
static void step(void)
{
    const control_vector_t &v = vectors[next_period++];
    control_in_t in;
    control_out_t out;
    ipc_report_t r;
    memset(&in, 0, sizeof(in));
    in.period = v.period;
    in.state = (v.flags & CONTROL_V_STATE) != 0;
    in.servo_on = (v.flags & CONTROL_V_SERVO_ON) != 0;
    in.not_ready = v.not_ready;
    for (int i = 0; i < IPC_CTRL_DOF; i++)
    {
        in.q[i] = v.q[i];
        in.qd[i] = v.qd[i];
    }
    ipc_goal_t goal;
    memcpy(goal.p, v.goal, sizeof(goal.p));
    in.command = (v.flags & CONTROL_V_COMMAND) ? &v.command : NULL;
    in.goal = (v.flags & CONTROL_V_GOAL) ? &goal : NULL;
    mark();
    control_ctrl_step(&in, &out, &r);
    mark();
    const ipc_report_t &m = v.report;

    /* whatever the planner's results did to the motion */
    if (r.in.period != v.period || r.ctrl_crc != control_ctrl_check_value() || r.cmd_done != m.cmd_done ||
        r.cmd_result != m.cmd_result || r.start_seq != m.start_seq || r.start_result != m.start_result ||
        r.refusal_why != m.refusal_why || r.refusal_joint != m.refusal_joint || r.n_no_state != m.n_no_state ||
        r.n_ctrl_faults != m.n_ctrl_faults || r.n_drive_stops != m.n_drive_stops || r.n_goals != m.n_goals ||
        out.servo != v.servo || (out.torque != 0) != ((v.flags & CONTROL_V_TORQUE) != 0))
        tally.results_differ++;
    if (r.n_plans < last.n_plans || r.n_iks < last.n_iks || r.n_plan_late < last.n_plan_late || r.n_goals < last.n_goals ||
        r.n_no_state < last.n_no_state)
        tally.back++;
    /* an IK asked for before this period is back, the planner having been served to its end since */
    if (!in_the_middle && (r.state & IPC_CTL_SAMPLE) && (r.hl_flags & CTrajectoryRT::HL_IK_ACTIVE) &&
        !(r.hl_flags & CTrajectoryRT::HL_GOAL_ACCEPTED))
        tally.ik_owed++;
    last = r;

    if (with_file)
    {
        numbers(out.tau, v.tau, IPC_CTRL_DOF, v.period);
        if (r.in.flags != m.in.flags || r.in.starts != m.in.starts || r.in.ctrl_mode != m.in.ctrl_mode || r.state != m.state ||
            r.traj_state != m.traj_state || r.motion != m.motion || r.clamp != m.clamp || r.sim_step != m.sim_step ||
            r.n_plans != m.n_plans || r.n_iks != m.n_iks || r.n_plan_late != m.n_plan_late || r.hl_flags != m.hl_flags ||
            r.otg != m.otg || r.lead_reset != m.lead_reset || r.ik_iter != m.ik_iter || r.goal_age != m.goal_age ||
            r.n_cycle != m.n_cycle || r.n_accepted != m.n_accepted || r.n_converged != m.n_converged ||
            r.n_rejected != m.n_rejected || r.n_lead_resets != m.n_lead_resets ||
            (r.traj_ns != 0) != (m.traj_ns != 0) || (r.ctrl_ns != 0) != (m.ctrl_ns != 0))
            tally.report_differs++;
        numbers(r.in.q, m.in.q, IPC_CTRL_DOF, v.period);
        numbers(r.in.qd, m.in.qd, IPC_CTRL_DOF, v.period);
        numbers(r.in.goal, m.in.goal, 3, v.period);
        number(r.in.delta, m.in.delta, v.period);
        numbers(r.in.q_ready, m.in.q_ready, IPC_CTRL_DOF, v.period);
        numbers(r.in.start_target, m.in.start_target, IPC_CTRL_DOF, v.period);
        numbers(r.in.tau, m.in.tau, IPC_CTRL_DOF, v.period);
        numbers(r.in.ik_q, m.in.ik_q, IPC_CTRL_DOF, v.period);
        number(r.refusal_value, m.refusal_value, v.period);
        number(r.refusal_lo, m.refusal_lo, v.period);
        number(r.refusal_hi, m.refusal_hi, v.period);
        number(r.ik_err, m.ik_err, v.period);
        number(r.otg_duration, m.otg_duration, v.period);
        numbers(r.accepted, m.accepted, 3, v.period);
        numbers(r.tool, m.tool, 3, v.period);
        numbers(r.target, m.target, IPC_CTRL_DOF, v.period);
        numbers(r.q_ref, m.q_ref, IPC_CTRL_DOF, v.period);
        numbers(r.qd_ref, m.qd_ref, IPC_CTRL_DOF, v.period);
        numbers(r.qdd_ref, m.qdd_ref, IPC_CTRL_DOF, v.period);
    }

    /* the second controller on the report: the same build, so the same numbers */
    if (!(r.state & IPC_CTL_PAYLOAD))
        tally.apart++;
    else
    {
        ipc_shadow_out_t f;
        shadow_ctrl_step(&r.in, &f);
        while (shadow_ctrl_serve())
        {
        }
        if (f.flags & IPC_SHADOW_OUT_APART)
            tally.apart++;
        if (f.flags & IPC_SHADOW_OUT_FAULT)
            tally.follower_failed++;
        else
        {
            if (f.flags & IPC_SHADOW_OUT_CTRL)
            {
                tally.law++;
                if (f.tau_diff != 0.0 || memcmp(f.tau, r.in.tau, sizeof(f.tau)) != 0 || memcmp(out.tau, r.in.tau, sizeof(out.tau)) != 0)
                    tally.follower_differs++;
            }
            if (f.flags & IPC_SHADOW_OUT_TRAJ)
            {
                tally.traj++;
                if (memcmp(f.q_ref, r.q_ref, sizeof(f.q_ref)) != 0)
                    tally.follower_differs++;
            }
        }
        if (((f.flags & IPC_SHADOW_OUT_CTRL) != 0) != (out.torque != 0) ||
            ((f.flags & IPC_SHADOW_OUT_TRAJ) != 0) != ((r.state & IPC_CTL_SAMPLE) != 0))
            tally.follower_differs++;
    }
}

#ifdef SLOT_PREEMPT_TEST
/* while the planner is served, at the place of this replay: the EtherCAT task of the firmware, the step of the next
 * period */
extern "C" void slot_preempt_point(void)
{
    if (next_period < periods && at_place())
    {
        in_the_middle = true;
        step();
        in_the_middle = false;
    }
}
#endif

static void replay(void)
{
    memset(&tally, 0, sizeof(tally));
    memset(&last, 0, sizeof(last));
    next_period = 0;
    const long allocations_before = allocations;
    while (next_period < periods)
    {
        step();
        /* the planner task of the firmware, here: back for the next period */
#ifdef SLOT_PREEMPT_TEST
        tally.interruptions += serve_planner(control_ctrl_serve) ? 1 : 0;
#else
        while (control_ctrl_serve())
        {
        }
#endif
    }
    tally.allocated = allocations - allocations_before;
}

/* the last report against the file's: the counts that do not depend on when the planner's results came */
static bool ends_as_file(void)
{
    const ipc_report_t &m = vectors[periods - 1].report;
    return last.n_goals == m.n_goals && last.n_drive_stops == m.n_drive_stops && last.n_ctrl_faults == m.n_ctrl_faults &&
           last.n_plan_late == 0 && m.n_plan_late == 0 && last.n_plans > 0 && last.n_iks > 0;
}

static bool passed(void)
{
    const long failed_in_file = vectors[periods - 1].report.n_ctrl_faults > 0 ? 1 : 0;      /* the follower fails with it */
    return tally.report_differs == 0 && tally.results_differ == 0 && tally.over == 0 &&
           tally.back == 0 && tally.follower_differs == 0 && tally.apart == 0 && tally.follower_failed >= failed_in_file &&
           tally.law > 0 && tally.traj > 0 && tally.allocated == 0 && tally.ik_owed == 0 && ends_as_file();
}

static void print(long allocations_before)
{
    printf("%u periods: %ld with another report than the file, %ld with another end of a command or another count, "
           "largest difference of a number %.3g (period %u), %ld above the limit %.3g; a second controller on the "
           "reports had the law in %ld and the reference in %ld: %ld differences, apart in %ld, failed in %ld; %ld with "
           "a count below that of the period before; plans %u, IK %u, late %u; %ld periods that still waited for an IK "
           "asked for before them; %ld allocations (%ld while the controllers were built)\n",
           (unsigned)periods, tally.report_differs, tally.results_differ, tally.largest, (unsigned)tally.largest_at,
           tally.over, limit, tally.law, tally.traj, tally.follower_differs, tally.apart, tally.follower_failed, tally.back,
           (unsigned)last.n_plans, (unsigned)last.n_iks, (unsigned)last.n_plan_late, tally.ik_owed, tally.allocated,
           allocations_before);
}

/* the two controllers, once in a process; 0: built */
static int build(void)
{
    if (control_ctrl_serve() != 0)      // the planner task may run before the controller is built
    {
        printf("FAIL: the planner has work before the controller is built\n");
        return 1;
    }
    static volatile uint32_t region[IPC_REGION_BYTES / 4];      // the link of the firmware: its receiver is not replayed
    if (!control_ctrl_build(region, 1e9) || !shadow_ctrl_build())
    {
        fprintf(stderr, "the controllers cannot be built\n");
        return 2;
    }
    return 0;
}

#ifdef SLOT_PREEMPT_TEST
/* the replay of one place (every_place.h) */
static int one_place(long *interruptions)
{
    const int code = build();
    if (code != 0)
        return code;
    const long allocations_before = allocations;
    replay();
    *interruptions = tally.interruptions;
    if (passed())
        return 0;
    printf("FAIL with the step of the next period at place %ld of the planner's work (%ld times): ", interrupt_at,
           tally.interruptions);
    print(allocations_before);
    return 1;
}
#endif

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        fprintf(stderr, "usage: %s FILE [LIMIT [PERIODS]]\n", argv[0]);
        return 2;
    }
    limit = argc > 2 ? atof(argv[2]) : 0.0;
    FILE *const fp = fopen(argv[1], "rb");
    shadow_vectors_head_t head;
    if (fp == NULL || fread(&head, sizeof(head), 1, fp) != 1 || head.magic != CONTROL_VECTORS_MAGIC)
    {
        fprintf(stderr, "%s: not a file of control_check -o\n", argv[1]);
        return 2;
    }
    periods = head.periods;
    if (argc > 3 && (uint32_t)atoi(argv[3]) < periods)
        periods = (uint32_t)atoi(argv[3]);
    control_vector_t *const read = (control_vector_t *)malloc((size_t)periods * sizeof(control_vector_t));
    if (periods == 0 || read == NULL || fread(read, sizeof(control_vector_t), periods, fp) != periods)
    {
        fprintf(stderr, "%s: %u periods cannot be read\n", argv[1], (unsigned)periods);
        return 2;
    }
    fclose(fp);
    vectors = read;

    if (CTRL_NUMBERS_CRC != head.crc || memcmp(CTRL_NUMBERS, head.numbers, sizeof(head.numbers)) != 0)
    {
        printf("FAIL: the numbers of src/ctrl/ctrl_numbers_indy7.h are not those of %s: write the header again "
               "(bin/ctrl_numbers_header of the application), or the file\n", argv[1]);
        return 1;
    }
#ifdef SLOT_PREEMPT_TEST
    with_file = false;
    return every_place(one_place);
#else
    with_file = true;
    const int code = build();
    if (code != 0)
        return code;
    const long allocations_before = allocations;
    replay();
    print(allocations_before);
    const bool pass = passed() && allocations_before > 0;
    printf("%s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
#endif
}
