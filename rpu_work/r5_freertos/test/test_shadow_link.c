/* host test of src/shadow_link.c: the second controller in the cycle, with stand-ins for the slots (ocm_link.c)
 * and the controller (ctrl/shadow_ctrl.cpp).  From r5_freertos/:
 *   gcc -std=gnu11 -Wall -DHOST_TEST -Itest/stub -Isrc test/test_shadow_link.c src/shadow_link.c -o /tmp/t_shadow && /tmp/t_shadow
 * It checks: a controller that cannot be built is said so and never runs; a built one runs on a new payload only,
 * and its answer goes into the slot; the times are those of the parts that ran, -1 for the others; from the start
 * on the classes print nothing; the report has the counts of its window and starts a new one, and of the planner
 * the plans and IK of the window with the longest of each that an answer showed, and its late plans so far. */
#include <stdio.h>
#include <string.h>
#include "shadow_link.h"
#include "ocm_link.h"
#include "event_log.h"
#include "ctrl/shadow_ctrl.h"
#include "ctrl/ctrl_port.h"

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

/* ---- the controller ---- */
static int can_build = 0, builds, steps, quiet;
static ipc_shadow_in_t  stepped;
static ipc_shadow_out_t answer;
int      shadow_ctrl_build(void) { builds++; return can_build; }
uint32_t shadow_ctrl_check_value(void) { return 0xC0FFEE01u; }
void     shadow_ctrl_step(const ipc_shadow_in_t *in, ipc_shadow_out_t *out) { steps++; stepped = *in; *out = answer; out->period = in->period; }
void     ctrl_port_loop(void) { quiet = 1; }
uint32_t ctrl_port_lines_held(void) { return 3u; }

/* ---- the slots ---- */
static int have_payload, published;
static ipc_shadow_in_t  payload;
static ipc_shadow_out_t in_slot;
int  ocm_link_take_shadow(ipc_shadow_in_t *in) { if (!have_payload) return 0; have_payload = 0; *in = payload; return 1; }
void ocm_link_publish_shadow(const ipc_shadow_out_t *out) { published++; in_slot = *out; }

/* ---- events, console ---- */
static event_t an_event; static int events, fail_lines;
event_t *event_reserve(uint32_t id, int budgeted) { (void)budgeted; memset(&an_event, 0, sizeof(an_event)); an_event.id = id; return &an_event; }
void event_publish(void) { events++; }
void xil_printf(const char *f, ...) { if (strstr(f, "FAIL") != NULL) fail_lines++; }

int main(void)
{
    int32_t traj_ns = 77, ctrl_ns = 88;

    /* not built: said once, the loop goes on without it and takes no payload */
    CHECK(shadow_link_start() == 0 && builds == 1 && fail_lines == 1 && quiet == 1);
    have_payload = 1;
    CHECK(shadow_link_step(&traj_ns, &ctrl_ns) == 0 && steps == 0 && published == 0 && have_payload == 1);
    shadow_link_report();
    CHECK(events == 1 && an_event.id == EVENT_SHADOW_REPORT && an_event.value[0] == 0);

    /* built */
    can_build = 1; quiet = 0; have_payload = 0;
    CHECK(shadow_link_start() == 1 && builds == 2 && fail_lines == 1 && quiet == 1);
    CHECK(shadow_link_step(&traj_ns, &ctrl_ns) == 0 && steps == 0 && published == 0 && traj_ns == 77 && ctrl_ns == 88);

    /* a payload: the controller runs on it, the answer goes out; here only the law ran */
    payload.period = 500; payload.flags = IPC_SHADOW_CTRL; have_payload = 1;
    answer.flags = IPC_SHADOW_OUT_CTRL; answer.traj_ns = 0; answer.ctrl_ns = 41000; answer.n_missed = 0;
    answer.n_plans = 4; answer.plan_us = 1100; answer.n_iks = 1; answer.ik_us = 400;    /* its planner so far */
    CHECK(shadow_link_step(&traj_ns, &ctrl_ns) == 1 && steps == 1 && published == 1);
    CHECK(stepped.period == 500 && stepped.flags == IPC_SHADOW_CTRL && in_slot.period == 500 && in_slot.flags == IPC_SHADOW_OUT_CTRL);
    CHECK(traj_ns == -1 && ctrl_ns == 41000);
    CHECK(shadow_link_step(&traj_ns, &ctrl_ns) == 0 && steps == 1);         /* the same payload is not new */

    /* both parts, apart, a fault, two payloads missed */
    payload.period = 503; have_payload = 1;
    answer.flags = IPC_SHADOW_OUT_TRAJ | IPC_SHADOW_OUT_CTRL | IPC_SHADOW_OUT_APART | IPC_SHADOW_OUT_FAULT;
    answer.traj_ns = 250000; answer.ctrl_ns = 43000; answer.n_missed = 2;
    answer.n_plans = 5; answer.plan_us = 300; answer.n_iks = 2; answer.ik_us = 350; answer.n_plan_late = 1;
    CHECK(shadow_link_step(&traj_ns, &ctrl_ns) == 1 && traj_ns == 250000 && ctrl_ns == 43000 && published == 2);
    payload.period = 504; have_payload = 1;
    answer.flags = IPC_SHADOW_OUT_TRAJ; answer.traj_ns = 90000; answer.ctrl_ns = 0;
    answer.plan_us = 2000; answer.ik_us = 900;          /* no new plan and no new IK: these are not times of new ones */
    CHECK(shadow_link_step(&traj_ns, &ctrl_ns) == 1 && traj_ns == 90000 && ctrl_ns == -1);

    /* the report: built, 3 periods, 1 apart, 1 failed, 2 missed, longest 293 us, 3 lines held; then a new window */
    shadow_link_report();
    CHECK(events == 2 && an_event.value[0] == 1 && an_event.value[1] == 3 && an_event.value[2] == 1 && an_event.value[3] == 1);
    CHECK(an_event.value[4] == 2 && an_event.value[5] == 293 && an_event.value[6] == 3);
    /* the planner: 5 plans, the longest 1100 us; 2 IK, the longest 400 us; 1 plan late */
    CHECK(an_event.value[7] == 5 && an_event.value[8] == 1100 && an_event.value[9] == 2 && an_event.value[10] == 400);
    CHECK(an_event.value[11] == 1);
    shadow_link_report();
    CHECK(events == 3 && an_event.value[1] == 0 && an_event.value[2] == 0 && an_event.value[3] == 0 && an_event.value[5] == 0);
    CHECK(an_event.value[4] == 2);                                           /* the missed payloads: of the run */
    CHECK(an_event.value[7] == 0 && an_event.value[8] == 0 && an_event.value[9] == 0 && an_event.value[10] == 0);
    CHECK(an_event.value[11] == 1);                                          /* the late plans: of the run */

    printf(fails ? "test_shadow_link: %d FAILED\n" : "test_shadow_link OK\n", fails);
    return fails != 0;
}
