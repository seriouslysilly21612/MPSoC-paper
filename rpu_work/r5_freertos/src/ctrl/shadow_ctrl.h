/*
 * shadow_ctrl.h  --  the second controller of --shadow (ShadowControl.h, C++) for the C side of the firmware: built
 * once from the constants of this build, then one step per payload of the Shadow in slot.
 *
 * The constants are two generated headers, each with a check value: the numbers of the controller from the
 * configuration (ctrl_numbers_indy7.h) and its RBDL model from the URDF (ctrl_model_indy7.h).  The check value of
 * the controller (ShadowCheckValue of ShadowControl.h) is of both, and goes back in every answer: the APU computes
 * it from its own configuration and model, and tells a firmware built from others.
 */
#ifndef SHADOW_CTRL_H
#define SHADOW_CTRL_H

#include <stdint.h>
#include "../ipc/ipc_layout.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Builds the controller.  It takes memory from the C heap (_HEAP_SIZE of lscript.ld), here and never after.
 * 0 = not built: the heap is too small, or the classes refused the constants (their lines say why). */
int      shadow_ctrl_build(void);

uint32_t shadow_ctrl_check_value(void);

/* One period of the control task: its payload -> the answer.  Only after shadow_ctrl_build() returned 1.  No
 * memory is taken.  Should a class throw, the answer is a fault (IPC_SHADOW_OUT_FAULT) and the cycle goes on. */
void     shadow_ctrl_step(const ipc_shadow_in_t *in, ipc_shadow_out_t *out);

/* The planner of the controller: one piece of what its high-level controller asked for, a plan or one iteration of
 * the inverse kinematics of a goal (HighLevelLink.h).  From one task only, another than that of
 * shadow_ctrl_step() (planner_task.c), at any time: 0 while the controller is not built.  1 = it did something,
 * call again; 0 = nothing is asked for.  No memory is taken. */
int      shadow_ctrl_serve(void);

#ifdef __cplusplus
}
#endif

#endif /* SHADOW_CTRL_H */
