/*
 * cycle_record.c  --  the ring of the records of the 1 kHz cycles (see cycle_record.h).
 */
#include <string.h>
#include "xil_cache.h"
#include "app_config.h"
#include "cycle_record.h"
#include "sys_counter.h"
#include "robot_target.h"
#include "gem3_nic.h"
#include "r5log.h"

#ifdef HOST_TEST
extern uint8_t host_test_ring[];                  /* the host test provides RECORD_RING_BYTES of it */
#define RING_BASE           ((uintptr_t)host_test_ring)
#else
#define RING_BASE           ((uintptr_t)RECORD_RING_ADDR)
#endif
#define RING_CAPACITY       ((uint32_t)((RECORD_RING_BYTES - RECORD_HEAD_BYTES) / sizeof(cycle_record_t)))

typedef struct {
    volatile uint32_t magic, version, record_bytes, capacity, count, boot_id, period_ns, deadline_ns, counter_hz,
                      cfg_crc, build, reserved[5];
} ring_head_t;
_Static_assert(sizeof(ring_head_t) == RECORD_HEAD_BYTES, "the head of the record ring is 64 bytes");

#define RING_HEAD           ((ring_head_t *)RING_BASE)

static uint32_t count;                            /* records written in this run; the copy of this side */

void cycle_record_start(void)
{
    ring_head_t *h = RING_HEAD;

    h->magic        = 0u;                         /* no ring while the head is rewritten */
    Xil_DCacheFlushRange((INTPTR)h, RECORD_HEAD_BYTES);
    h->version      = RECORD_VERSION;
    h->record_bytes = sizeof(cycle_record_t);
    h->capacity     = RING_CAPACITY;
    h->count        = 0u;
    h->boot_id      = r5log_boot_id();
    h->period_ns    = CYCLE_NS;
    h->deadline_ns  = FW_CFG_SEND_DEADLINE_NS;
    h->counter_hz   = SYSCNT_HZ;
    h->cfg_crc      = FW_CFG_CRC;
    h->build        = ((uint32_t)ROBOT_TARGET << 8) | (GEM3_BUF_IN_OCM ? RECORD_BUILD_GEM3_OCM : 0u) |
                      (ECAT_USE_DC ? RECORD_BUILD_DC : 0u)
#ifdef __OPTIMIZE__
                      | RECORD_BUILD_OPTIMIZED
#endif
                      ;
    memset((void *)h->reserved, 0, sizeof(h->reserved));
    Xil_DCacheFlushRange((INTPTR)h, RECORD_HEAD_BYTES);
    h->magic        = RECORD_MAGIC;               /* valid only after the rest is in the DDR */
    Xil_DCacheFlushRange((INTPTR)h, RECORD_HEAD_BYTES);
}

void cycle_record_init(cycle_record_t *r)
{
    memset(r, 0, sizeof(*r));
    r->dyn_ns = r->hl_ns = r->sim_ns = -1;
    r->traj_w_ns = r->traj_e_ns = r->dyn_w_ns = r->dyn_e_ns = -1;
    r->sync_ns = 0xFFFFFFFFu;
}

void cycle_record_push(const cycle_record_t *r)
{
    void *slot = (void *)(RING_BASE + RECORD_HEAD_BYTES + (uintptr_t)(count % RING_CAPACITY) * sizeof(*r));

    memcpy(slot, r, sizeof(*r));
    Xil_DCacheFlushRange((INTPTR)slot, sizeof(*r));          /* the record is in the DDR ... */
    count++;
    RING_HEAD->count = count;
    Xil_DCacheFlushRange((INTPTR)&RING_HEAD->count, 4u);     /* ... before the count that announces it */
}
