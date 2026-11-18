/* host test of cycle_record.c: the record ring as Linux sees it.  From r5_freertos/:
 *   gcc -std=gnu11 -DHOST_TEST -DRECORD_RING_BYTES=1024 -Itest/stub -Isrc -Isrc/oshw/freertos test/test_cycle_record.c src/cycle_record.c -o /tmp/t_rec && /tmp/t_rec [FILE]
 * The ring is small here (16 records) so that it wraps.  What Linux sees is modelled as a second copy of the ring
 * that only a cache flush updates, in whole cache lines of 32 bytes.  It checks: the head, which Linux sees as valid
 * only once it is complete, also over the head of an earlier run;
 * record k in slot k % capacity; a record is in the DDR before the count that announces it; the fields this side
 * does not measure.  With FILE, the ring as Linux sees it at the end is written there (test/test_r5rec.py reads it). */
#include <stdio.h>
#include <string.h>
#include "app_config.h"
#include "cycle_record.h"
#include "robot_target.h"
#include "sys_counter.h"
#include "gem3_nic.h"
#include "xil_cache.h"

#define CAPACITY    ((RECORD_RING_BYTES - RECORD_HEAD_BYTES) / 60u)
uint8_t host_test_ring[RECORD_RING_BYTES];          /* the R5's view: its cache */
static uint8_t ddr[RECORD_RING_BYTES];              /* Linux's view */
uint64_t host_test_counter(void) { return 0; }
uint32_t r5log_boot_id(void) { return 7u; }
static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

static uint32_t ddr_word(unsigned offset) { uint32_t v; memcpy(&v, ddr + offset, 4); return v; }
static cycle_record_t last_pushed;
static int count_before_record;                     /* a count reached the DDR before its record */
static int starting, head_incomplete;               /* cache lines flushed by cycle_record_start() after which Linux saw
                                                       a valid head that was not yet the whole new one */

static int head_is_new(void)
{
    return ddr_word(0x04) == 1u && ddr_word(0x08) == 60u && ddr_word(0x0C) == CAPACITY && ddr_word(0x10) == 0u &&
           ddr_word(0x14) == 7u && ddr_word(0x18) == CYCLE_NS && ddr_word(0x1C) == FW_CFG_SEND_DEADLINE_NS &&
           ddr_word(0x20) == SYSCNT_HZ && ddr_word(0x24) == FW_CFG_CRC && ddr_word(0x2C) == 0u && ddr_word(0x3C) == 0u;
}

void Xil_DCacheFlushRange(INTPTR adr, uint32_t len)
{
    uintptr_t line = ((uintptr_t)adr - (uintptr_t)host_test_ring) & ~(uintptr_t)31u;
    uintptr_t end = (((uintptr_t)adr - (uintptr_t)host_test_ring) + len + 31u) & ~(uintptr_t)31u;

    for (; line < end; line += 32u) {               /* one cache line at a time: Linux may look in between */
        memcpy(ddr + line, host_test_ring + line, 32u);
        if (starting && ddr_word(0x00) == RECORD_MAGIC && !head_is_new()) head_incomplete++;
        if (!starting && ddr_word(0x00) == RECORD_MAGIC && ddr_word(0x10) > 0u) {   /* Linux may read record count - 1 now */
            uint32_t k = ddr_word(0x10) - 1u;
            if (k == last_pushed.period && memcmp(ddr + RECORD_HEAD_BYTES + (k % CAPACITY) * 60u, &last_pushed, 60) != 0) {
                count_before_record++;
            }
        }
    }
}

int main(int argc, char **argv)
{
    cycle_record_t r;
    uint32_t k;

    CHECK(CAPACITY == 16u);
    memset(host_test_ring, 0xA5, sizeof(host_test_ring));       /* what an earlier run or the boot left */
    memset(ddr, 0xA5, sizeof(ddr));
    memset(&last_pushed, 0xFF, sizeof(last_pushed));

    starting = 1; cycle_record_start(); starting = 0;
    CHECK(head_incomplete == 0 && head_is_new());
    CHECK(ddr_word(0x00) == RECORD_MAGIC && ddr_word(0x04) == 1u && ddr_word(0x08) == 60u && ddr_word(0x0C) == CAPACITY);
    CHECK(ddr_word(0x10) == 0u && ddr_word(0x14) == 7u && ddr_word(0x18) == CYCLE_NS);
    CHECK(ddr_word(0x1C) == FW_CFG_SEND_DEADLINE_NS && ddr_word(0x20) == SYSCNT_HZ && ddr_word(0x24) == FW_CFG_CRC);
    CHECK(ddr_word(0x28) == (((uint32_t)ROBOT_TARGET << 8) | (GEM3_BUF_IN_OCM ? RECORD_BUILD_GEM3_OCM : 0u) |
                             (ECAT_USE_DC ? RECORD_BUILD_DC : 0u)
#ifdef __OPTIMIZE__
                             | RECORD_BUILD_OPTIMIZED
#endif
                             ));
    CHECK(ddr_word(0x2C) == 0u && ddr_word(0x3C) == 0u);

    cycle_record_init(&r);
    CHECK(r.period == 0u && r.w_ns == 0 && r.s_ns == 0 && r.e_ns == 0 && r.flags == 0u && r.reserved == 0u);
    CHECK(r.dyn_ns == -1 && r.hl_ns == -1 && r.sim_ns == -1 && r.sync_ns == 0xFFFFFFFFu);
    CHECK(r.traj_w_ns == -1 && r.traj_e_ns == -1 && r.dyn_w_ns == -1 && r.dyn_e_ns == -1 && r.recv_ns == 0 && r.send_ns == 0);

    for (k = 0; k < 2u * CAPACITY + 5u; k++) {                  /* twice round the ring and five more */
        cycle_record_init(&r);
        r.period = k; r.w_ns = (int32_t)(4000 + k); r.s_ns = (int32_t)(60000 + k); r.e_ns = r.s_ns;
        r.recv_ns = 15000; r.send_ns = 12000; r.flags = (uint16_t)(RECORD_BUS | RECORD_OP | (k % 7u == 3u ? RECORD_MISSED : 0u));
        last_pushed = r;
        cycle_record_push(&r);
        CHECK(ddr_word(0x10) == k + 1u);
        CHECK(memcmp(ddr + RECORD_HEAD_BYTES + (k % CAPACITY) * 60u, &r, 60) == 0);
    }
    CHECK(count_before_record == 0);
    CHECK(ddr_word(RECORD_HEAD_BYTES + 4u * 60u) == 2u * CAPACITY + 4u);     /* slot 4 holds the newest record */
    CHECK(ddr_word(RECORD_HEAD_BYTES + 5u * 60u) == CAPACITY + 5u);          /* slot 5 the oldest still there */
    CHECK(memcmp(ddr, host_test_ring, sizeof(ddr)) == 0);                    /* nothing left in the cache only */

    if (argc > 1) {
        FILE *f = fopen(argv[1], "wb");
        if (f == NULL || fwrite(ddr, 1, sizeof(ddr), f) != sizeof(ddr)) { printf("FAIL: cannot write %s\n", argv[1]); fails++; }
        if (f != NULL) fclose(f);
    }
    starting = 1; cycle_record_start(); starting = 0;           /* the next run finds the head of this one */
    CHECK(head_incomplete == 0 && ddr_word(0x00) == RECORD_MAGIC && head_is_new());

    printf(fails ? "test_cycle_record: %d FAILED\n" : "test_cycle_record OK\n", fails);
    return fails != 0;
}
