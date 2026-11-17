/*
 * obs_link.c  --  see obs_link.h.  The reader protocol is that of obs/obs_mailbox.h, copied unchanged from the APU build.
 */
#include <stdint.h>
#include "xil_printf.h"
#include "app_config.h"
#include "obs_link.h"
#include "event_log.h"

#ifdef HOST_TEST
extern volatile obs_mailbox_t *obs_link_mailbox;        /* the test's mailbox */
#define MAILBOX     obs_link_mailbox
#else
#define MAILBOX     ((volatile obs_mailbox_t *)(OCM_LINK_BASE + OBS_OFF_MAILBOX))
#endif

static obs_desc_t latest;                    /* newest capture among the results taken that had not expired */
static int        have_latest;
static uint32_t   n_taken, n_stale, n_expired;

static int writer_up(void)
{
    return ipc_load32(&MAILBOX->magic) == OBS_MAGIC && ipc_load32(&MAILBOX->layout) == OBS_LAYOUT;
}

void obs_link_start(void)
{
    obs_reader_start(MAILBOX);               /* a result this reader was reading when it stopped goes back to FREE */
    xil_printf("[obs] mailbox at OCM +0x%x, %u bytes: the writer is %s\r\n", (unsigned)OBS_OFF_MAILBOX,
               (unsigned)sizeof(obs_mailbox_t), writer_up() ? "up" : "not up yet");
}

void obs_link_poll(uint64_t now)
{
    unsigned b;

    if (have_latest && now > latest.t_valid_until) have_latest = 0;
    if (!writer_up()) return;

    for (b = 0; b < OBS_SLOTS; b++) {
        obs_desc_t d;
        int r = obs_take(MAILBOX, b, &d);

        if (r == OBS_STALE) n_stale++;       /* copied and freed, but not to be used: another epoch, a broken copy */
        if (r != OBS_OK) continue;
        n_taken++;
        if (now > d.t_valid_until) {         /* too late to be of use; the slot is free again all the same */
            n_expired++;
            continue;
        }
        if (!have_latest || d.t_e >= latest.t_e) {
            latest = d;
            have_latest = 1;
        }
    }
}

int obs_link_latest(obs_desc_t *out)
{
    if (!have_latest) return 0;
    *out = latest;
    return 1;
}

void obs_link_report(uint64_t now)
{
    event_t *e = event_reserve(EVENT_OBS_REPORT, 0);
    uint64_t age;

    if (e == NULL) return;
    e->value[0] = writer_up();
    e->value[1] = (int32_t)n_taken;
    e->value[2] = (int32_t)n_stale;
    e->value[3] = (int32_t)n_expired;
    e->value[4] = have_latest ? (int32_t)latest.n : -1;
    age = (have_latest && now > latest.t_e) ? now - latest.t_e : 0u;
    e->value[5] = age > (uint64_t)INT32_MAX ? INT32_MAX : (int32_t)age;     /* ticks: the counter's frequency is not assumed here */
    e->value[6] = (int32_t)ipc_load32(&MAILBOX->dropped_full);
    e->value[7] = (int32_t)ipc_load32(&MAILBOX->dropped_expired);
    event_publish();
}
