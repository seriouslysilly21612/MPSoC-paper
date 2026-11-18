/*
 * cycle_record.h  --  the record of every 1 kHz cycle, and the ring that hands the records to Linux (plan 2-11).
 *
 * The record is that of the timing log of the APU controller (goal-to-torque_MPSoC/indy7/src/TimingLog.h, format
 * g2t-timing-1: 60 bytes, the same fields and flag bits), so that one tool reads the cycles of both sides
 * (tools/timing_summary.py).  test/check_record.cpp fails to compile when the two differ (test/check_copies.sh).
 * The fields of the control task stay -1 here: it runs on the APU.  In a cycle in which the second controller of
 * --shadow ran (shadow_link.h), hl_ns and dyn_ns are the times of its two parts, and e_ns includes it.
 *
 * The ring is in the DDR that Linux reserves for the R5 (RECORD_RING_ADDR, outside what lscript.ld links), written
 * by the EtherCAT task only and read by Linux through /dev/mem (r5rec.py).  The R5 never waits for the reader: it
 * writes record number k into slot k % capacity and then the count k + 1, and the reader sees from the count what
 * it lost.  As in r5log.c, every write is flushed to the DDR before the count that announces it.
 *
 * Layout (little endian), 64 bytes of head, then the slots of 60 bytes:
 *   0x00 magic "R5RC"   0x04 version 1    0x08 record bytes (60)   0x0C capacity [records]
 *   0x10 count: records written since this start                   0x14 boot_id (that of the r5log ring)
 *   0x18 period [ns]    0x1C send deadline [ns]   0x20 ticks of the system counter per second
 *   0x24 check value of the constants (FW_CFG_CRC)   0x28 build: RECORD_BUILD_*, the target in bits 8..15
 *   0x2C..0x3F reserved
 */
#ifndef CYCLE_RECORD_H
#define CYCLE_RECORD_H

#include <stdint.h>

#ifndef RECORD_RING_ADDR
#define RECORD_RING_ADDR    0x36300000UL          /* up to the 64 KB that hold the stop page and the r5log ring */
#endif
#ifndef RECORD_RING_BYTES
#define RECORD_RING_BYTES   0x000F0000UL          /* 16382 records: 16 s */
#endif
#define RECORD_HEAD_BYTES   64u
#define RECORD_MAGIC        0x43523552UL          /* bytes 'R','5','R','C' */
#define RECORD_VERSION      1u

enum {                                            /* build word of the head */
    RECORD_BUILD_GEM3_OCM  = 1u << 0,             /* the GEM3 descriptors and frame buffers are in the OCM (gem3_nic.h) */
    RECORD_BUILD_OPTIMIZED = 1u << 1,             /* compiled with optimization */
    RECORD_BUILD_DC        = 1u << 2,             /* DC SYNC0 on the drives (ECAT_USE_DC) */
    RECORD_BUILD_SHADOW    = 1u << 3              /* with the second controller of --shadow (SHADOW_CONTROL) */
};

enum {                                            /* flags of a record: the bits of CTimingLog (TimingLog.h) */
    RECORD_OP      = 1u << 0,                     /* the working counter of the cycle is complete */
    RECORD_WKC     = 1u << 1,                     /* incomplete working counter (not during a stop) */
    RECORD_LATE    = 1u << 2,                     /* t_w > T: the cycle started more than a period after its release */
    RECORD_MISSED  = 1u << 3,                     /* t_w + t_s > t_b: the send returned after the send deadline */
    RECORD_OVERRUN = 1u << 4,                     /* t_w + t_e > T: the cycle ended after the next release */
    RECORD_STOP    = 1u << 8,                     /* a stop asked for by Linux is under way */
    RECORD_BUS     = 1u << 10,                    /* an EtherCAT exchange ran: always */
    RECORD_SERVO   = 1u << 11                     /* every axis is in Operation enabled */
};

#pragma pack(push, 1)
typedef struct {
    uint32_t period;                              /* release index of the cycle: period of the State slot */
    int32_t  w_ns;                                /* t_w: release -> start of the cycle */
    int32_t  s_ns;                                /* t_s: start -> return of the send */
    int32_t  e_ns;                                /* t_e: start -> end of the cycle */
    int32_t  dyn_ns, hl_ns, sim_ns;               /* -1: of the control task and the simulator, on the APU; dyn_ns
                                                     and hl_ns: of the second controller, in a cycle in which it ran */
    uint32_t sync_ns;                             /* 0xFFFFFFFF: not measured */
    uint16_t flags;                               /* RECORD_* */
    uint16_t reserved;                            /* 0 */
    int32_t  recv_ns;                             /* start -> the inputs of the cycle are in */
    int32_t  send_ns;                             /* the send call */
    int32_t  traj_w_ns, traj_e_ns, dyn_w_ns, dyn_e_ns;    /* -1 */
} cycle_record_t;
#pragma pack(pop)

#ifndef __cplusplus                               /* test/check_record.cpp includes this file for the layout only */
_Static_assert(sizeof(cycle_record_t) == 60, "the record is that of g2t-timing-1: 60 bytes");

void cycle_record_start(void);                    /* once, before the first cycle: a new head, count 0 */
void cycle_record_init(cycle_record_t *r);        /* every field that this side does not measure: -1, 0 or not measured */
void cycle_record_push(const cycle_record_t *r);
#endif

#endif /* CYCLE_RECORD_H */
