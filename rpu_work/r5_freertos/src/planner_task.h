/*
 * planner_task.h  --  the planner of the second controller of --shadow, as a task of its own (decision D-14,
 * 2026-11-19): the inverse kinematics of a goal and the calculation of a plan do not fit the 1 kHz cycle of this
 * processor (one iteration of the IK 70 us, one plan up to 1.1 ms; measured 2026-11-19), so the cycle only asks
 * for them and uses the result when it is there.
 *
 * The task has a priority below the EtherCAT task and above the log task: it runs in what a period leaves, and
 * every cycle interrupts it.  It looks for work once per tick, after the cycle of that tick.
 *
 * With SHADOW_CONTROL 0 (app_config.h) it is not in the build.
 */
#ifndef PLANNER_TASK_H
#define PLANNER_TASK_H

/* its stack [words], 16 KB: one call of the planner used 5.0 KB at most (emulated R5, 2026-11-19) */
#define PLANNER_TASK_STACK_WORDS 4096

void planner_task(void *arg);       /* FreeRTOS task */

#endif /* PLANNER_TASK_H */
