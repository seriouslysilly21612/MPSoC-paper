/* host test of obs_link.c, with the real writer of obs/obs_mailbox.h on the other side.  From r5_freertos/:
 *   gcc -std=gnu11 -DHOST_TEST -Itest/stub -Isrc test/test_obs_link.c src/obs_link.c -o /tmp/t_obs && /tmp/t_obs
 * It checks: no writer, results taken and freed, the newest one kept, expired and stale results, a full mailbox */
#include <stdio.h>
#include <string.h>
#include "app_config.h"
#include "obs_link.h"
#include "event_log.h"

static obs_mailbox_t mailbox;
volatile obs_mailbox_t *obs_link_mailbox = &mailbox;
static event_t last_event; static int n_events;
event_t *event_reserve(uint32_t id, int budgeted) { (void)budgeted; last_event.id = id; memset(last_event.value, 0, sizeof(last_event.value)); return &last_event; }
void event_publish(void) { n_events++; }
void xil_printf(const char *f, ...) { (void)f; }

static uint64_t clock_now;
static uint64_t now(void) { return clock_now; }
static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

static int put(uint64_t t_e, uint64_t valid_until)
{
    obs_desc_t d; memset(&d, 0, sizeof(d));
    d.t_e = t_e; d.t_valid_until = valid_until; d.z[0] = 0.5f;
    return obs_put(&mailbox, &d, now);
}
static int free_slots(void)
{
    int b, n = 0; for (b = 0; b < OBS_SLOTS; b++) if (mailbox.state[b][0] == OBS_FREE) n++; return n;
}

int main(void)
{
    obs_desc_t got;
    int i;

    memset(&mailbox, 0xA5, sizeof(mailbox));                  /* garbage, as the OCM is after a boot */
    obs_link_start();                                         /* the reader's start with no writer: nothing breaks */
    clock_now = 1000; obs_link_poll(clock_now);
    CHECK(!obs_link_latest(&got));
    obs_link_report(clock_now); CHECK(last_event.value[0] == 0);

    obs_writer_start(&mailbox);
    obs_link_poll(clock_now);
    CHECK(!obs_link_latest(&got));                            /* up, but empty */
    obs_link_report(clock_now); CHECK(last_event.value[0] == 1 && last_event.value[1] == 0 && last_event.value[4] == -1);

    CHECK(put(900, 5000) == OBS_OK); CHECK(put(950, 5000) == OBS_OK); CHECK(put(920, 5000) == OBS_OK);
    obs_link_poll(clock_now);
    CHECK(obs_link_latest(&got) && got.t_e == 950 && got.n == 2);   /* the newest capture, not the last written */
    CHECK(free_slots() == OBS_SLOTS);                         /* all taken, all freed */
    obs_link_report(clock_now + 10);
    CHECK(last_event.value[1] == 3 && last_event.value[4] == 2 && last_event.value[5] == 60);   /* taken 3, n 2, age 1010 - 950 */

    clock_now = 6000; obs_link_poll(clock_now);               /* the kept result is past its time */
    CHECK(!obs_link_latest(&got));

    clock_now = 1000; CHECK(put(990, 1100) == OBS_OK);        /* written in time, past its time when the reader looks */
    clock_now = 2000; obs_link_poll(clock_now);
    CHECK(!obs_link_latest(&got) && free_slots() == OBS_SLOTS);
    obs_link_report(clock_now); CHECK(last_event.value[3] == 1);

    clock_now = 2000; CHECK(put(1990, 9000) == OBS_OK);       /* a result of an earlier epoch: the writer restarts */
    obs_writer_start(&mailbox);
    obs_link_poll(clock_now);
    CHECK(!obs_link_latest(&got) && free_slots() == OBS_SLOTS);
    obs_link_report(clock_now); CHECK(last_event.value[2] == 1);   /* stale: copied, freed, refused */

    for (i = 0; i < OBS_SLOTS; i++) CHECK(put(2000 + i, 9000) == OBS_OK);   /* a full mailbox: the writer drops the 9th ... */
    CHECK(put(3000, 9000) == OBS_FULL);
    obs_link_poll(clock_now);
    CHECK(obs_link_latest(&got) && got.t_e == 2000 + OBS_SLOTS - 1);
    obs_link_report(clock_now); CHECK(last_event.value[6] == 1);   /* ... and the reader sees the writer's counter */
    CHECK(put(3000, 9000) == OBS_OK);                         /* the slots are free again */

    printf(fails ? "%d FAILED\n" : "all passed\n", fails);
    return fails != 0;
}
