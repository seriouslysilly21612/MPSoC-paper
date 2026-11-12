/*
 * r5log.c -- R5 console (xil_printf / print / printf) -> ring buffer in DDR
 *
 * WHY
 *   The BSP's stdout is "psu_coresight_0" (JTAG debug channel, DCC).  Its outbyte() is
 *       XCoresightPs_DccSendByte():  while (TX register still full) ;  write byte;
 *   i.e. it waits until the *debugger host* has read the previous byte.  When the R5 is
 *   started by remoteproc there is no JTAG host, so the first xil_printf() that prints
 *   two or more characters hangs its task for ever.  (The Gate-1 LED test did not notice:
 *   only its print task hung, the LED task kept running.)  gem3_nic.c and the SOEM
 *   bring-up print a lot -> they MUST NOT go through the DCC when run by remoteproc.
 *
 * WHAT
 *   This file defines outbyte() itself.  The linker then never pulls the BSP's outbyte.o
 *   out of libxil.a, and every print goes into a 16 KB ring in the last 16 KB of the 4 MB
 *   R5 carveout:
 *
 *       physical 0x363FC000 .. 0x363FFFFF      (inside the top 64 KB that lscript.ld must leave
 *                                               free: psu_r5_ddr_0_MEM_0 LENGTH = 0x3F0000)
 *
 *   Linux reads it with r5log.py (/dev/mem).  The ring survives an R5 hang, so after a freeze
 *   the LAST lines are still there -- that is the point.
 *
 * RING LAYOUT (little endian)
 *   0x00 u32  magic      'R','5','L','G'  (bytes 52 35 4C 47)
 *   0x04 u32  version    1
 *   0x08 u32  data_bytes size of the data area (0x4000 - 32 = 16352)
 *   0x0C u32  head       total number of bytes written since this boot; byte k is at data[k % data_bytes]
 *   0x10 u32  boot_id    +1 on every start (lets the reader notice a restart)
 *   0x14..0x1F reserved
 *   0x20      data[16352]
 *
 * CACHES
 *   The carveout is normal cacheable memory for the R5 (the BSP MPU table), and the A53
 *   does not snoop the R5 caches.  So every byte is written, then its cache line is
 *   cleaned+invalidated (Xil_DCacheFlushRange, includes a DSB) BEFORE the header's head
 *   counter is advanced and flushed.  Linux therefore never sees head ahead of the data.
 *   (No MPU region is used, so this cannot collide with the strongly-ordered region that
 *   gem3_nic.c creates for the GEM DMA window.)
 *
 * COST / RULES
 *   ~2 cache-line flushes + IRQs masked for the duration of one byte (some 100 ns).  Fine
 *   for diagnostics; do not print from the 1 kHz control path.
 *   Safe from tasks, ISRs and exception handlers (interrupts are masked around the update).
 *
 * BUILD SWITCHES
 *   -DR5LOG_ADDR=0x363FC000UL   ring location (must be inside memory Linux does not use)
 *   -DR5LOG_BYTES=0x4000UL      ring size incl. 32-byte header
 */

#include "r5log.h"

#include "xil_types.h"
#include "xil_cache.h"

#ifndef R5LOG_ADDR
#define R5LOG_ADDR        0x363FC000UL
#endif
#ifndef R5LOG_BYTES
#define R5LOG_BYTES       0x4000UL
#endif

#define R5LOG_HDR_BYTES   32U
#define R5LOG_DATA_BYTES  ((u32)(R5LOG_BYTES - R5LOG_HDR_BYTES))
#define R5LOG_MAGIC       0x474C3552UL      /* bytes 'R','5','L','G' */
#define R5LOG_VERSION     1U

typedef struct {
    volatile u32 magic;        /* 0x00 */
    volatile u32 version;      /* 0x04 */
    volatile u32 data_bytes;   /* 0x08 */
    volatile u32 head;         /* 0x0C */
    volatile u32 boot_id;      /* 0x10 */
    volatile u32 reserved[3];  /* 0x14..0x1F */
} r5log_hdr_t;

typedef char r5log_hdr_size_check[(sizeof(r5log_hdr_t) == R5LOG_HDR_BYTES) ? 1 : -1];

#define R5LOG_HDR()   ((r5log_hdr_t *)(UINTPTR)(R5LOG_ADDR))
#define R5LOG_DATA()  ((volatile u8 *)(UINTPTR)(R5LOG_ADDR + R5LOG_HDR_BYTES))

static u32 s_head;     /* private copy of the byte counter (also stored in the header) */
static int s_ready;

/* ---- interrupt masking (task / ISR / exception safe) ------------------------------------- */
#if defined(__arm__) && !defined(R5LOG_HOST_TEST)
static inline u32 r5log_irq_save(void)
{
    u32 cpsr;
    __asm__ volatile ("mrs %0, cpsr\n\tcpsid i" : "=r"(cpsr) : : "memory", "cc");
    return cpsr;
}
static inline void r5log_irq_restore(u32 cpsr)
{
    if ((cpsr & 0x80U) == 0U) {                 /* I bit was clear before -> re-enable */
        __asm__ volatile ("cpsie i" : : : "memory", "cc");
    }
}
#else
u32  r5log_test_irq_save(void);                 /* host test hooks */
void r5log_test_irq_restore(u32 cpsr);
#define r5log_irq_save()        r5log_test_irq_save()
#define r5log_irq_restore(c)    r5log_test_irq_restore(c)
#endif

static void r5log_flush(volatile void *p, u32 len)
{
    Xil_DCacheFlushRange((INTPTR)(UINTPTR)p, len);
}

/* caller holds the IRQ mask */
static void r5log_init_locked(void)
{
    r5log_hdr_t *h = R5LOG_HDR();
    u32 boot = 0U;

    if (h->magic == (u32)R5LOG_MAGIC) {         /* previous run left a valid header */
        boot = h->boot_id;
    }
    h->magic      = 0U;                         /* invalid while it is rewritten */
    h->version    = R5LOG_VERSION;
    h->data_bytes = R5LOG_DATA_BYTES;
    h->head       = 0U;
    h->boot_id    = boot + 1U;
    h->reserved[0] = 0U;
    h->reserved[1] = 0U;
    h->reserved[2] = 0U;
    r5log_flush(h, R5LOG_HDR_BYTES);
    h->magic      = (u32)R5LOG_MAGIC;           /* valid only after the rest is in DDR */
    r5log_flush(h, R5LOG_HDR_BYTES);

    s_head  = 0U;
    s_ready = 1;
}

void r5log_init(void)
{
    u32 cpsr = r5log_irq_save();
    r5log_init_locked();
    r5log_irq_restore(cpsr);
}

/* Called by xil_printf(), print(), newlib's _write() ... instead of the BSP's DCC version. */
void outbyte(char c)
{
    u32 cpsr = r5log_irq_save();

    if (s_ready == 0) {
        r5log_init_locked();
    }

    {
        volatile u8 *p = &R5LOG_DATA()[s_head % R5LOG_DATA_BYTES];
        *p = (u8)c;
        r5log_flush(p, 1U);                     /* 1) data byte reaches DDR (flush ends with DSB) */
        s_head++;
        R5LOG_HDR()->head = s_head;
        r5log_flush(R5LOG_HDR(), R5LOG_HDR_BYTES);   /* 2) only now advance head */
    }

    r5log_irq_restore(cpsr);
}

void r5log_puts(const char *s)
{
    while (*s != '\0') {
        outbyte(*s);
        s++;
    }
}
