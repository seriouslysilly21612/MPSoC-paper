/*
 * every_place.h  --  for a replay built with SLOT_PREEMPT_TEST (a PC): the task of the higher priority at every
 * place at which the planner touches a slot (slot_preempt_point() of src/ctrl/port/base/AtomicSlot.h), one place per
 * replay, each replay in a process of its own.  A test includes it once and has:
 *
 *     extern "C" void slot_preempt_point(void) { if (<a next period is there> && at_place()) <its steps>; }
 *     ... serve_planner(<the serve call>) where the planner task of the firmware would run ...
 *     main(): return every_place(<one replay>);
 */
#ifndef EVERY_PLACE_H
#define EVERY_PLACE_H

#include <cstdio>
#include <sys/wait.h>
#include <unistd.h>

static long interrupt_at, place;        /* the place of this process; the places passed since the planner was served */
static bool serving, interrupted;

/* in slot_preempt_point(): true once while the planner is served, at the place of this process */
static bool at_place(void)
{
    if (!serving || interrupted || place++ != interrupt_at)
        return false;
    interrupted = true;                 /* before the caller's steps: they touch slots as well */
    return true;
}

/* the planner, until it has nothing to do; true: steps ran in the middle of it */
template <class Serve>
static bool serve_planner(Serve serve)
{
    serving = true;
    place = 0;
    interrupted = false;
    while (serve())
    {
    }
    serving = false;
    return interrupted;
}

/* A replay for every place, as many at a time as there are processors, until one has no place left for its steps.
 * one(): the replay of this process (interrupt_at is set), with the controller built in it; it returns 0 (passed)
 * or 1 (failed, and it has printed why) and gives the number of times its steps ran in the middle.  Returns the
 * exit code of the test and prints its last two lines. */
static int every_place(int (*one)(long *interruptions))
{
    int counts[2];              /* every replay that got to its end writes how often its steps ran in the middle */
    if (pipe(counts) != 0)
    {
        perror("pipe");
        return 2;
    }
    const long processors = sysconf(_SC_NPROCESSORS_ONLN) > 0 ? sysconf(_SC_NPROCESSORS_ONLN) : 1;
    long started = 0, running = 0, places = 0, failed = 0, interruptions = 0;
    bool ended = false;         /* a replay had no place for its steps: nor have those after it */
    while (!ended || running > 0)
    {
        while (!ended && running < processors)
        {
            fflush(stdout);
            const pid_t pid = fork();
            if (pid < 0)
            {
                perror("fork");
                return 2;
            }
            if (pid == 0)
            {
                long n = 0;
                interrupt_at = started;
                int code = one(&n);
                fflush(stdout);
                if ((code == 0 || code == 1) && write(counts[1], &n, sizeof(n)) != (ssize_t)sizeof(n))
                    code = 2;
                _exit(code);
            }
            started++;
            running++;
        }
        int status = 0;
        if (wait(&status) < 0)
        {
            perror("wait");
            return 2;
        }
        running--;
        const int code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
        long n = 0;
        if (code != 0 && code != 1)     /* it did not get to its count */
        {
            printf("FAIL: a replay ended with status %d\n", status);
            failed++;
            ended = true;
            continue;
        }
        if (read(counts[0], &n, sizeof(n)) != (ssize_t)sizeof(n))
        {
            perror("read");
            return 2;
        }
        if (code == 1)
            failed++;
        if (n == 0)
            ended = true;
        else
        {
            places++;
            interruptions += n;
        }
    }
    printf("the step of the next period in the middle of the planner's work, at each of the %ld places where the "
           "planner touches a slot after a period: %ld steps in the middle in all, %ld replays failed\n",
           places, interruptions, failed);
    const bool pass = failed == 0 && places > 0;
    printf("%s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}

#endif /* EVERY_PLACE_H */
