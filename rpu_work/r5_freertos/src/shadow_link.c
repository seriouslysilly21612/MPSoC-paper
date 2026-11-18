/*
 * shadow_link.c  --  see shadow_link.h.
 */
#include <stdint.h>
#include "xil_printf.h"
#include "app_config.h"
#include "shadow_link.h"
#include "ocm_link.h"
#include "event_log.h"
#include "ctrl/shadow_ctrl.h"
#include "ctrl/ctrl_port.h"

static int      built;
static uint32_t n_steps, n_apart, n_fault;           /* of the window of a report */
static uint32_t n_missed;                            /* payloads the controller did not get, so far (its count) */
static uint32_t step_ns_max;                         /* the two parts together, largest of the window */

int shadow_link_start(void)
{
    built = shadow_ctrl_build();
    ctrl_port_loop();                                /* from here the classes print nothing */
    if (!built) {
        xil_printf("[shadow] FAIL: the controller could not be built (the C heap, _HEAP_SIZE of lscript.ld, or the"
                   " [ctrl] lines above): the cycle runs without it\r\n");
        return 0;
    }
    xil_printf("[shadow] the controller of the control task is built: check value 0x%08x; it runs after the send of"
               " every cycle that has a payload of the APU (--shadow)\r\n", (unsigned)shadow_ctrl_check_value());
    return 1;
}

int shadow_link_step(int32_t *traj_ns, int32_t *ctrl_ns)
{
    static ipc_shadow_in_t  in;
    static ipc_shadow_out_t out;
    uint32_t ns;

    if (!built || !ocm_link_take_shadow(&in)) return 0;
    shadow_ctrl_step(&in, &out);
    ocm_link_publish_shadow(&out);

    *traj_ns = (out.flags & IPC_SHADOW_OUT_TRAJ) ? (int32_t)out.traj_ns : -1;
    *ctrl_ns = (out.flags & IPC_SHADOW_OUT_CTRL) ? (int32_t)out.ctrl_ns : -1;
    n_steps++;
    if (out.flags & IPC_SHADOW_OUT_APART) n_apart++;
    if (out.flags & IPC_SHADOW_OUT_FAULT) n_fault++;
    n_missed = out.n_missed;
    ns = out.traj_ns + out.ctrl_ns;
    if (ns > step_ns_max) step_ns_max = ns;
    return 1;
}

void shadow_link_report(void)
{
    event_t *e = event_reserve(EVENT_SHADOW_REPORT, 0);

    if (e != NULL) {
        e->value[0] = built;
        e->value[1] = (int32_t)n_steps;
        e->value[2] = (int32_t)n_apart;
        e->value[3] = (int32_t)n_fault;
        e->value[4] = (int32_t)n_missed;
        e->value[5] = (int32_t)(step_ns_max / 1000u);
        e->value[6] = (int32_t)ctrl_port_lines_held();
        event_publish();
    }
    n_steps = 0u; n_apart = 0u; n_fault = 0u; step_ns_max = 0u;
}
