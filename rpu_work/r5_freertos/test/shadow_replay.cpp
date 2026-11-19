/*
 * shadow_replay.cpp  --  the second controller as the firmware builds it, against the controller of the
 * application.  The firmware's build of it (src/ctrl) has another compiler and C library than the application,
 * the headers of src/ctrl/port in place of Linux, its model and its numbers from generated headers; this test
 * feeds it, through the calls the firmware makes (ctrl/shadow_ctrl.h), the periods that the application's
 * controller computed on a PC (goal-to-torque_MPSoC/tools/shadow_check -o FILE) and compares.
 * Guards the verdict of plan stage 3 against a controller on the R5 that computes something else.
 *
 *   shadow_replay FILE [LIMIT [PERIODS]]     LIMIT: largest torque difference allowed [N m], default 0
 *                                            PERIODS: only the first so many
 *   test/shadow_replay.sh builds it for a PC and for the R5 (run by qemu-arm) and runs both.
 *
 * The numbers of the firmware (src/ctrl/ctrl_numbers_indy7.h) must be those the file was written with.  Every
 * answer must have the flags of the file and the check value of the firmware; in the periods followed, the torque
 * and the reference must be within LIMIT of the file's, and so must the IK of every goal that the planner of
 * this controller solved as the file's did (ik_diff of the answer).  After every period the planner is served as
 * the planner task of the firmware serves it, unless the file says that it had no time then.  On a PC with the
 * application's compiler the difference is 0; the R5 has no fused multiply-add and another library, so there it
 * is small and not 0.  Neither a period nor the planner may allocate memory: the firmware must not after its
 * start (the build puts counting wrappers in front of malloc, calloc and realloc:
 * -Wl,--wrap=malloc,--wrap=calloc,--wrap=realloc).  Exit 0 if no check failed.
 *
 * mark() is called before and after the step of every period: a trace of the instructions executed (qemu-arm
 * -d exec) is counted between the two (test/count_periods.py).  The planner's work is not in that count.
 *
 * Built with SLOT_PREEMPT_TEST (a PC only) it replays the file once for every place at which the planner touches
 * a slot after a period (slot_preempt_point() of port/base/AtomicSlot.h): the step of the next period runs
 * there, in the middle of what the planner does, as the EtherCAT task of the firmware interrupts the planner
 * task on the core they share, and the planner goes on after it.  Every such replay must pass as the plain one:
 * a plan has more periods to be back in than the one it loses, and a result that the step finds half written
 * is taken a period later.  Guards the slots and their readers against a reader that takes a half-written
 * payload, loses one, or (the counts of the answer) puts nothing where it could not read.
 */
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "ShadowControl.h"
#include "ctrl_numbers_indy7.h"
#include "shadow_ctrl.h"
#include "shadow/shadow_vectors.h"

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
    long flags_differ, law, traj, told, goals, over, back, interruptions, allocated;
    double tau_max, ref_max, ik_max;
    uint32_t tau_at, ref_at;
};

/* a replay: the periods of the file in turn, each checked against the file when its step has run */
static const shadow_vector_t *vectors;
static uint32_t periods, next_period;       /* next_period: the first whose step has not run */
static double limit;
static tally_t tally;
static ipc_shadow_out_t last;               /* the answer of the period before */

/* the step of the next period; true: the planner has no time after it */
static bool step(void)
{
    const shadow_vector_t &v = vectors[next_period++];
    ipc_shadow_out_t out;
    mark();
    shadow_ctrl_step(&v.in, &out);
    mark();
    if (out.flags != v.out_flags || out.ctrl_crc != shadow_ctrl_check_value())
        tally.flags_differ++;
    /* what the answer counts "so far" never goes back: a period that cannot read the planner's counts keeps the
     * last it read */
    if (out.n_missed < last.n_missed || out.n_plans < last.n_plans || out.n_iks < last.n_iks ||
        out.n_plan_late < last.n_plan_late || out.n_ik_compared < last.n_ik_compared)
        tally.back++;
    if ((v.in.flags & IPC_SHADOW_IK) && !(v.out_flags & IPC_SHADOW_OUT_APART))
        tally.told++;           /* a goal whose IK the file has, in a period that is followed */
    if (out.n_ik_compared != last.n_ik_compared)    /* a goal that its planner solved as well: against the file's IK */
    {
        tally.goals++;
        if (!(out.ik_diff <= limit))
            tally.over++;
        if (!(out.ik_diff <= tally.ik_max))
            tally.ik_max = out.ik_diff;
    }
    last = out;
    if (!(out.flags & IPC_SHADOW_OUT_APART) && (out.flags & IPC_SHADOW_OUT_CTRL))
    {
        tally.law++;
        if (!(out.tau_diff <= limit))
            tally.over++;
        if (!(out.tau_diff <= tally.tau_max))
        {
            tally.tau_max = out.tau_diff;
            tally.tau_at = v.in.period;
        }
    }
    if (!(out.flags & IPC_SHADOW_OUT_APART) && (out.flags & IPC_SHADOW_OUT_TRAJ))
    {
        tally.traj++;
        for (int i = 0; i < IPC_CTRL_DOF; i++)
        {
            const double d = std::fabs(out.q_ref[i] - v.q_ref[i]);
            if (!(d <= limit))
                tally.over++;
            if (!(d <= tally.ref_max))
            {
                tally.ref_max = d;
                tally.ref_at = v.in.period;
            }
        }
    }
    return v.planner_stalled != 0;
}

#ifdef SLOT_PREEMPT_TEST
/* while the planner is served, at the place of this replay: the EtherCAT task of the firmware, which is the step of
 * the next period, and of those after it as long as the planner has no time after them */
extern "C" void slot_preempt_point(void)
{
    if (next_period >= periods || !at_place())
        return;
    while (next_period < periods && step())
    {
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
        if (step())
            continue;
        /* the planner task of the firmware, here: back for the next period */
#ifdef SLOT_PREEMPT_TEST
        tally.interruptions += serve_planner(shadow_ctrl_serve) ? 1 : 0;
#else
        while (shadow_ctrl_serve())
        {
        }
#endif
    }
    tally.allocated = allocations - allocations_before;
}

static bool passed(const tally_t &t)
{
    return t.flags_differ == 0 && t.over == 0 && t.back == 0 && t.law > 0 && t.traj > 0 && t.goals > 0 &&
           t.goals == t.told && t.allocated == 0;
}

static void print(const tally_t &t, long allocations_before)
{
    printf("%u periods: %ld with other flags than the file; the law followed in %ld, largest torque difference %.3g N m "
           "(period %u); the reference in %ld, largest difference %.3g rad (period %u); the IK of %ld of the %ld goals "
           "followed, largest difference %.3g rad; %ld above the limit %.3g; %ld with a count below that of the period "
           "before; %ld allocations (%ld while the controller was built)\n",
           (unsigned)periods, t.flags_differ, t.law, t.tau_max, (unsigned)t.tau_at, t.traj, t.ref_max, (unsigned)t.ref_at,
           t.goals, t.told, t.ik_max, t.over, limit, t.back, t.allocated, allocations_before);
}

/* the controller, once in a process; 0: built */
static int build(void)
{
    if (shadow_ctrl_serve() != 0)       // the planner task may run before the controller is built
    {
        printf("FAIL: the planner has work before the controller is built\n");
        return 1;
    }
    if (!shadow_ctrl_build())
    {
        fprintf(stderr, "the controller cannot be built\n");
        return 2;
    }
    return 0;
}

#ifdef SLOT_PREEMPT_TEST
/* the replay of one place (every_place.h) */
static int one_place(long *interruptions)
{
    memset(&tally, 0, sizeof(tally));
    const int code = build();
    if (code != 0)
        return code;
    const long allocations_before = allocations;
    replay();
    *interruptions = tally.interruptions;
    if (passed(tally))
        return 0;
    printf("FAIL with the step of the next period at place %ld of the planner's work (%ld times): ", interrupt_at,
           tally.interruptions);
    print(tally, allocations_before);
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
    if (fp == NULL || fread(&head, sizeof(head), 1, fp) != 1 || head.magic != SHADOW_VECTORS_MAGIC)
    {
        fprintf(stderr, "%s: not a file of shadow_check -o\n", argv[1]);
        return 2;
    }
    periods = head.periods;
    if (argc > 3 && (uint32_t)atoi(argv[3]) < periods)
        periods = (uint32_t)atoi(argv[3]);
    shadow_vector_t *const read = (shadow_vector_t *)malloc((size_t)periods * sizeof(shadow_vector_t));
    if (read == NULL || fread(read, sizeof(shadow_vector_t), periods, fp) != periods)
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
    return every_place(one_place);
#else
    const int code = build();
    if (code != 0)
        return code;
    const long allocations_before = allocations;
    replay();
    print(tally, allocations_before);
    const bool pass = passed(tally) && allocations_before > 0;
    printf("%s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
#endif
}
