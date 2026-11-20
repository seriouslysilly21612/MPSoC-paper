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
 * and the reference must be within LIMIT of the file's.  On a PC with the application's compiler the difference is 0; the R5 has no fused
 * multiply-add and another library, so there it is small and not 0.  No period may allocate memory: the 1 kHz
 * loop of the firmware must not (the build puts counting wrappers in front of malloc, calloc and realloc:
 * -Wl,--wrap=malloc,--wrap=calloc,--wrap=realloc).  Exit 0 if no check failed.
 *
 * mark() is called before and after every period: a trace of the instructions executed (qemu-arm -d exec) is
 * counted between the two (test/count_periods.py).
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

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        fprintf(stderr, "usage: %s FILE [LIMIT [PERIODS]]\n", argv[0]);
        return 2;
    }
    const double limit = argc > 2 ? atof(argv[2]) : 0.0;
    FILE *const fp = fopen(argv[1], "rb");
    shadow_vectors_head_t head;
    if (fp == NULL || fread(&head, sizeof(head), 1, fp) != 1 || head.magic != SHADOW_VECTORS_MAGIC)
    {
        fprintf(stderr, "%s: not a file of shadow_check -o\n", argv[1]);
        return 2;
    }
    uint32_t periods = head.periods;
    if (argc > 3 && (uint32_t)atoi(argv[3]) < periods)
        periods = (uint32_t)atoi(argv[3]);
    shadow_vector_t *const vectors = (shadow_vector_t *)malloc((size_t)periods * sizeof(shadow_vector_t));
    if (vectors == NULL || fread(vectors, sizeof(shadow_vector_t), periods, fp) != periods)
    {
        fprintf(stderr, "%s: %u periods cannot be read\n", argv[1], (unsigned)periods);
        return 2;
    }
    fclose(fp);

    if (CTRL_NUMBERS_CRC != head.crc || memcmp(CTRL_NUMBERS, head.numbers, sizeof(head.numbers)) != 0)
    {
        printf("FAIL: the numbers of src/ctrl/ctrl_numbers_indy7.h are not those of %s: write the header again "
               "(bin/ctrl_numbers_header of the application), or the file\n", argv[1]);
        return 1;
    }
    if (!shadow_ctrl_build())
    {
        fprintf(stderr, "the controller cannot be built\n");
        return 2;
    }

    long flags_differ = 0, law = 0, traj = 0, over = 0;
    double tau_max = 0.0, ref_max = 0.0;
    uint32_t tau_at = 0, ref_at = 0;
    const long allocations_before = allocations;
    for (uint32_t k = 0; k < periods; k++)
    {
        const shadow_vector_t &v = vectors[k];
        ipc_shadow_out_t out;
        mark();
        shadow_ctrl_step(&v.in, &out);
        mark();
        if (out.flags != v.out_flags || out.ctrl_crc != shadow_ctrl_check_value())
            flags_differ++;
        if (out.flags & IPC_SHADOW_OUT_APART)
            continue;
        if (out.flags & IPC_SHADOW_OUT_CTRL)
        {
            law++;
            if (!(out.tau_diff <= limit))
                over++;
            if (!(out.tau_diff <= tau_max))
            {
                tau_max = out.tau_diff;
                tau_at = v.in.period;
            }
        }
        if (out.flags & IPC_SHADOW_OUT_TRAJ)
        {
            traj++;
            for (int i = 0; i < IPC_CTRL_DOF; i++)
            {
                const double d = std::fabs(out.q_ref[i] - v.q_ref[i]);
                if (!(d <= limit))
                    over++;
                if (!(d <= ref_max))
                {
                    ref_max = d;
                    ref_at = v.in.period;
                }
            }
        }
    }
    const long allocated = allocations - allocations_before;
    printf("%u periods: %ld with other flags than the file; the law followed in %ld, largest torque difference %.3g N m "
           "(period %u); the reference in %ld, largest difference %.3g rad (period %u); %ld above the limit %.3g; "
           "%ld allocations (%ld while the controller was built)\n",
           (unsigned)periods, flags_differ, law, tau_max, (unsigned)tau_at, traj, ref_max, (unsigned)ref_at, over, limit,
           allocated, allocations_before);
    const bool pass = flags_differ == 0 && over == 0 && law > 0 && traj > 0 && allocated == 0 && allocations_before > 0;
    printf("%s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}
