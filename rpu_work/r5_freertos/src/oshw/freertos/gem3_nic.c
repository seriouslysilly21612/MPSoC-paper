/*
 * gem3_nic.c  --  Raw Layer-2 EtherCAT NIC for SOEM on the Zynq UltraScale+ PS
 *                 GEM3 (KV260 on-board RJ45), bare-metal Cortex-R5F + FreeRTOS.
 *
 * Implements the 4 hooks that oshw/freertos/nicdrv.c calls:
 *     gem3_init / gem3_send / gem3_recv / gem3_close
 *
 * Style: POLLED (no interrupts) -- matches SOEM's non-blocking model
 * (ecx_recvpkt returns 0 when no frame is waiting).
 *
 * REV 2.  Re-written after cross-checking against the Xilinx embeddedsw
 * sources (drivers/emacps, xemacps_example_intr_dma.c, lwIP xemacpsif_dma.c).
 * What changed versus rev 1:
 *   - BD rings + "terminate" BDs live in a 64 KB *strongly-ordered* MPU window
 *     (the Xilinx recipe: BDs must not be cached on R5).
 *   - ZynqMP GEM has priority queues: RX = queue 0, TX = queue 1; the unused
 *     queues are parked on terminate BDs.  XEmacPs_SetQueuePtr() is required.
 *   - RX BD re-arm / TX BD reclaim follow lwIP: clear NEW/USED, restore WRAP.
 *   - One TX buffer per TX BD (SOEM sends several frames back-to-back).
 *   - DP83867 RGMII-delay set-up (same values as Linux DT / Xilinx lwIP) and
 *     auto-negotiation limited to 100BASE-TX full duplex.
 *   - GEM3 reference clock (CRL_APB) set for 100 Mbit/s.
 *   - Removed XEMACPS_FRAME_RECV_OPTION (does not exist).
 *
 * REV 3.  Added after reading the board's live device tree (K26 SOM):
 *   - MDIO read/write are now BOUNDED.  The Xilinx XEmacPs_PhyRead/Write spin
 *     forever when the MDIO engine never finishes (GEM3 clock/reset wrong) --
 *     that would be a silent R5 hang.  Ours time out and say why.
 *   - PHY hardware reset: the DT says  reset-gpios = <&gpio 38 ACTIVE_LOW>,
 *     reset-assert-us = 100, reset-deassert-us = 280.  Linux was the only one
 *     toggling it; with the GEM3 node disabled the R5 does it (MIO38 pulse).
 *   - Clock/reset prep now prints the CRL_APB values before/after and only
 *     writes when something has to change; write-back is verified.
 *   - Every stage prints before it touches hardware, so the last UART line
 *     tells where a hang happened.  PHY address 1 (DT) is tried first.
 *
 * REV 4.  Found by running the host test against the emacps v3.16 sources that ship with
 * Vitis 2022.1 (rev 3 had only been checked against v3.18 / the 2023.1 tag):
 *   - TX reclaim no longer calls XEmacPs_BdRingFromHwTx() with a big limit.  Driver <= v3.16
 *     keeps walking past the end of the work group while frames are still in flight, wraps
 *     around the ring and then reports BDs the MAC has NOT finished as "done"; we would
 *     recycle them under the DMA's feet (lost / corrupted frames, 12 sends accepted into an
 *     8-entry ring).  v3.18 (Vitis 2023.1) stops at HwTail.  Now BDs are reclaimed ONE at a time and
 *     only after this file has seen the BD's USED bit set -- identical result with either
 *     driver version.
 *   - If the TX ring is full (the MAC never completes frames: DMA/SMMU fault, no clock...)
 *     one warning with TXSR/NWSR is printed instead of failing silently.
 *
 * REV 5.  First run with a cable (stage 2) showed link UP at 100M FD but "-1 slave(s)", MAC counters
 *   rx=2 rxResErr=35714 (frames received but no RX buffer for them) and RXSR.BUFFNA set:
 *   - ORDER.  rev 4 let XEmacPs_SetOptions() switch RX/TX on right after the reset, i.e. BEFORE the BD rings
 *     and the queue pointers existed (the Xilinx flow switches them on inside XEmacPs_Start(), and the
 *     driver documents that the queue addresses must be written before the start).  Now:
 *       MAC set-up -> BD rings + queue pointers -> PHY reset/tune -> XEmacPs_Start() (RX/TX on)
 *       -> [self-test] -> auto-negotiation + link wait -> RX restarted on a fresh ring (RX off, ring
 *       re-armed, RX on) -> statistics cleared -> [listen probe].
 *     Queue base registers are written directly: XEmacPs_SetQueuePtr() does nothing once the device is started.
 *   - SELF-TEST (GEM3_SELFTEST).  Sends 4 frames through MAC -> RGMII -> PHY (BMCR loopback) -> RGMII -> MAC ->
 *     RX BD ring -> gem3_recv() and compares them.  PASS = the whole MAC/DMA/BD/cache path works, whatever is
 *     (not) plugged in.  FAIL = it says whether the MAC saw nothing or saw frames the DMA could not store.
 *   - LISTEN PROBE (GEM3_LISTEN_MS).  After link-up it listens WITHOUT transmitting and reports what the wire
 *     carries (frame count, headers of the first frames, MAC counters).
 *   - gem3_debug_dump() prints every MAC statistics counter, the live RX queue pointer with the first RX BDs, and
 *     the first frames the software received (destination / source / ethertype / payload).
 *   - The link wait needs link UP *and* auto-negotiation complete (BMSR bit 5): after the loopback test the
 *     link bit can still read "up" for a moment.
 *
 * REQUIREMENTS (see [[CHECK]] tags):
 *   1) emacps driver must be available: either from the BSP (GEM3 enabled in the XSA)
 *      or by copying its sources into the app (see "Two ways" below) -- both work.
 *   2) lscript.ld:  psu_r5_ddr_0_MEM_0  LENGTH = 0x3F0000   (NOT 0x400000)
 *      -> the top 64 KB of the 4 MB carveout (0x363F0000..0x363FFFFF) is the
 *         GEM DMA window and must never be used by the linker.
 *   3) Linux must NOT own GEM3 while this runs (DT status="disabled" for
 *      ethernet@ff0e0000, or at least the interface never brought up AND
 *      clocks/power left on) -- see the hand-over notes.
 *   4) Build flags you may want:  -DGEM3_PHY_RST_MIO=-1  (do not touch the PHY
 *      reset pin),  -DGEM3_PREPARE_CLK=0  (do not write CRL_APB).
 */

#include "gem3_nic.h"

#include <string.h>
#include "xparameters.h"
#include "xemacps.h"
#include "xil_cache.h"
#include "xil_mmu.h"
#include "xreg_cortexr5.h"     /* STRONG_ORDERD_SHARED / PRIV_RW_USER_RW */
#include "xil_io.h"
#include "xil_printf.h"
#include "xpseudo_asm.h"

#include "FreeRTOS.h"
#include "task.h"

/* Two ways to get the emacps driver:
 *   (1) GEM3 enabled in the XSA  -> BSP ships the driver, XEmacPs_LookupConfig() and the
 *       100 Mbit/s divisors (XPAR_XEMACPS_NUM_INSTANCES is defined).
 *   (2) driver sources copied into the app (xemacps.c/_bdring.c/_control.c/_hw.c/_intr.c +
 *       headers; NOT xemacps_g.c / xemacps_sinit.c) -> no XSA table; we build the config here
 *       and use the divisors below.  The PS itself is initialised by the Kria boot firmware,
 *       so the XSA does not need GEM3 for this to work.                                    */
#if defined(SDT) || defined(XPAR_XEMACPS_NUM_INSTANCES)
#define GEM3_CFG_FROM_BSP      1
#else
#define GEM3_CFG_FROM_BSP      0
#endif

/* GEM3_REF_CTRL divisors for the 25 MHz RGMII reference at 100 Mbit/s when the BSP gives none.
 * 25 MHz = IOPLL / (DIV0 * DIV1).  IOPLL = 1500 MHz on the K26 -> 12 * 5.
 * [[CHECK]] verify:  sudo grep -E "iopll|gem3" /sys/kernel/debug/clk/clk_summary          */
#ifndef GEM3_DIV0_100M
#define GEM3_DIV0_100M         12U
#endif
#ifndef GEM3_DIV1_100M
#define GEM3_DIV1_100M         5U
#endif

/* ---------------------------------------------------------------------- */
/* Build-time switches                                                    */
/* ---------------------------------------------------------------------- */
#ifndef GEM3_PREPARE_CLK
#define GEM3_PREPARE_CLK       1   /* poke CRL_APB: de-assert reset, 100M ref clock, CLKACT */
#endif
#ifndef GEM3_FORCE_100FD
#define GEM3_FORCE_100FD       0   /* 1: PHY forced 100FD (no autoneg). Debug only: causes
                                      duplex mismatch with autonegotiating slaves.          */
#endif
#ifndef GEM3_LINK_TIMEOUT_MS
#define GEM3_LINK_TIMEOUT_MS   5000
#endif
#ifndef GEM3_PHY_RST_MIO
#define GEM3_PHY_RST_MIO       38  /* PS-GPIO pin wired to the PHY nRESET (K26 DT: reset-gpios = <&gpio 38 LOW>).
                                      -1 = never touch it.  Valid: 0..77 (MIO only).                        */
#endif
#ifndef GEM3_PHY_ADDR_HINT
#define GEM3_PHY_ADDR_HINT     1   /* K26 DT: ethernet-phy@1.  The scan still covers all 32 addresses.     */
#endif
#ifndef GEM3_MDIO_SPIN
#define GEM3_MDIO_SPIN         300000U   /* register polls per MDIO transaction before giving up */
#endif
#ifndef GEM3_SELFTEST
#define GEM3_SELFTEST          1   /* PHY-loopback self-test of the MAC TX/RX/DMA path inside gem3_init() (about 0.1 s) */
#endif
#ifndef GEM3_LISTEN_MS
#define GEM3_LISTEN_MS         100 /* after link-up: listen this long WITHOUT transmitting and report what arrives (0 = off) */
#endif

#if GEM3_PHY_RST_MIO > 77
#error "GEM3_PHY_RST_MIO must be an MIO number (0..77) or -1"
#endif

#define GEM3_BASEADDR          0xFF0E0000U

/* ---------------------------------------------------------------------- */
/* DMA window = top 64 KB of the R5 carveout                              */
/*   0x363F0000  RX BD ring   (RXBD_CNT * 8  bytes)                       */
/*   0x363F1000  TX BD ring   (TXBD_CNT * 8  bytes)                       */
/*   0x363F2000  RX terminate BD   (64 B aligned)                         */
/*   0x363F2040  TX terminate BD                                          */
/* Frame buffers are ordinary cacheable .bss (explicit flush/invalidate). */
/* ---------------------------------------------------------------------- */
#define GEM3_DMA_BASE          0x363F0000UL      /* [[CHECK]] must match lscript.ld */
#define GEM3_DMA_SIZE          0x00010000UL      /* 64 KB, naturally aligned (MPU) */
#define RXBD_OFFSET            0x0000UL
#define TXBD_OFFSET            0x1000UL
#define RXTERM_OFFSET          0x2000UL
#define TXTERM_OFFSET          0x2040UL

#define RXBD_CNT               16U
#define TXBD_CNT               8U
#define GEM_BUF                1536U             /* multiple of 64; == XEMACPS_RX_BUF_SIZE */

static XEmacPs emac;
static u8      RxBuf[RXBD_CNT][GEM_BUF] __attribute__((aligned(64)));
static u8      TxBuf[TXBD_CNT][GEM_BUF] __attribute__((aligned(64)));
static int     g_mpu_done;
static u32     g_phy;
static u32     g_tx_full_warned;      /* "TX ring full" warning is printed once per gem3_init() */
static u32     g_link_up;             /* 1 once phy_link() saw link up + auto-negotiation complete */
static TickType_t g_t_stats0;         /* tick of the last gem3_stats_clear() */

/* What gem3_recv() handed to SOEM since the last gem3_stats_clear(): counters + the first few frames. */
#define RXLOG_N                6U
#define RXLOG_B                32U
static u32     g_rx_total, g_rx_ecat, g_rx_other, g_rx_logged;
static u32     g_rx_restarts;         /* automatic RX restarts after BUFFNA (the MAC ran out of RX buffers) since the last stats clear */
static TickType_t g_rx_restart_tick;  /* tick of the last automatic restart (at most one per tick) */
static u16     g_rxlog_len[RXLOG_N];
static u8      g_rxlog[RXLOG_N][RXLOG_B];

/* GEM statistics registers (UG1087; same offsets as xemacps_hw.h, copied so that this file does not depend on
 * which macros the BSP's emacps version defines).                                                           */
#define G3_TXCNT        0x108U
#define G3_TXBCCNT      0x10CU
#define G3_TXMCCNT      0x110U
#define G3_TXURUNCNT    0x134U
#define G3_TXSNGLCOL    0x138U
#define G3_TXMULTICOL   0x13CU
#define G3_TXEXCESSCOL  0x140U
#define G3_TXLATECOL    0x144U
#define G3_TXDEFER      0x148U
#define G3_TXCSENSE     0x14CU
#define G3_RXCNT        0x158U
#define G3_RXBCCNT      0x15CU
#define G3_RXMCCNT      0x160U
#define G3_RXPAUSECNT   0x164U
#define G3_RX64         0x168U
#define G3_RX65         0x16CU
#define G3_RX128        0x170U
#define G3_RX256        0x174U
#define G3_RX512        0x178U
#define G3_RX1024       0x17CU
#define G3_RX1519       0x180U
#define G3_RXUNDR       0x184U
#define G3_RXOVR        0x188U
#define G3_RXJAB        0x18CU
#define G3_RXFCS        0x190U
#define G3_RXLEN        0x194U
#define G3_RXSYMB       0x198U
#define G3_RXALIGN      0x19CU
#define G3_RXRESERR     0x1A0U
#define G3_RXOVRRUN     0x1A4U
#define G3_RD(off)      Xil_In32((u32)emac.Config.BaseAddress + (off))

/* arbitrary unicast source MAC (EtherCAT ignores it; promiscuous mode) */
static const u8 g_mac[6] = { 0x00, 0x0A, 0x35, 0x00, 0x01, 0x03 };

#define BD_INDEX(ring, bd) \
    ((u32)(((UINTPTR)(bd) - (UINTPTR)(ring)->BaseBdAddr) / (ring)->Separation))

/* ---------------------------------------------------------------------- */
/* small helpers                                                          */
/* ---------------------------------------------------------------------- */
static void delay_ms(u32 ms)
{
    TickType_t t = pdMS_TO_TICKS(ms);
    if (t == 0U) { t = 1U; }
    vTaskDelay(t);                 /* gem3_init() must run inside a task */
}

/* MDIO access.  Same register protocol as XEmacPs_PhyRead/Write, but BOUNDED:
 * the Xilinx versions loop forever if NWSR.MDIOIDLE never comes back (GEM3 in
 * reset / no clock), which would freeze the R5 without any message.         */
static int mdio_wait_idle(u32 base)
{
    u32 n;

    for (n = 0; n < GEM3_MDIO_SPIN; n++) {
        if ((Xil_In32(base + XEMACPS_NWSR_OFFSET) & XEMACPS_NWSR_MDIOIDLE_MASK) != 0U) { return 0; }
    }
    return -1;
}

static int mdio_rd(u32 phy, u32 reg, u16 *v)
{
    u32 base = (u32)emac.Config.BaseAddress;

    if (mdio_wait_idle(base) != 0) { return -1; }
    Xil_Out32(base + XEMACPS_PHYMNTNC_OFFSET,
              XEMACPS_PHYMNTNC_OP_MASK | XEMACPS_PHYMNTNC_OP_R_MASK |
              (phy << XEMACPS_PHYMNTNC_PHAD_SHFT_MSK) | (reg << XEMACPS_PHYMNTNC_PREG_SHFT_MSK));
    if (mdio_wait_idle(base) != 0) { return -1; }
    *v = (u16)Xil_In32(base + XEMACPS_PHYMNTNC_OFFSET);
    return 0;
}

static int mdio_wr(u32 phy, u32 reg, u16 v)
{
    u32 base = (u32)emac.Config.BaseAddress;

    if (mdio_wait_idle(base) != 0) { return -1; }
    Xil_Out32(base + XEMACPS_PHYMNTNC_OFFSET,
              XEMACPS_PHYMNTNC_OP_MASK | XEMACPS_PHYMNTNC_OP_W_MASK |
              (phy << XEMACPS_PHYMNTNC_PHAD_SHFT_MSK) | (reg << XEMACPS_PHYMNTNC_PREG_SHFT_MSK) | (u32)v);
    return mdio_wait_idle(base);
}

/* TI extended registers via REGCR(0x0D)/ADDAR(0x0E) (clause-45-style access) */
static void ti_ext_wr(u32 phy, u16 reg, u16 val)
{
    (void)mdio_wr(phy, 0x0D, 0x001F);
    (void)mdio_wr(phy, 0x0E, reg);
    (void)mdio_wr(phy, 0x0D, 0x401F);
    (void)mdio_wr(phy, 0x0E, val);
}

static u16 ti_ext_rd(u32 phy, u16 reg)
{
    u16 v = 0;
    (void)mdio_wr(phy, 0x0D, 0x001F);
    (void)mdio_wr(phy, 0x0E, reg);
    (void)mdio_wr(phy, 0x0D, 0x401F);
    (void)mdio_rd(phy, 0x0E, &v);
    return v;
}

/* ---------------------------------------------------------------------- */
/* GEM3 reference clock / reset (CRL_APB).                                */
/* [[CHECK]] Only matters once Linux no longer manages GEM3: Linux's clock */
/* framework may have gated it ("unused clock"), PMUFW may have it in     */
/* reset.  Values per UG1087: GEM3_REF_CTRL @0xFF5E005C                   */
/*   [2:0] SRCSEL  [13:8] DIVISOR0  [21:16] DIVISOR1  [25] CLKACT  [26] RX_CLKACT */
/* RST_LPD_IOU0 @0xFF5E0230 bit3 = GEM3_RESET.                            */
/* ---------------------------------------------------------------------- */
#define CRL_APB_GEM3_REF_CTRL  0xFF5E005CU
#define CRL_APB_RST_LPD_IOU0   0xFF5E0230U
#define GEM3_RESET_BIT         (1U << 3)
#define GEM_REF_DIV0_MASK      0x00003F00U
#define GEM_REF_DIV0_SHIFT     8U
#define GEM_REF_DIV1_MASK      0x003F0000U
#define GEM_REF_DIV1_SHIFT     16U
#define GEM_REF_CLKACT         (1U << 25)
#define GEM_REF_RX_CLKACT      (1U << 26)

#if GEM3_PREPARE_CLK
#if GEM3_CFG_FROM_BSP
static int cfg_has_div(const XEmacPs_Config *cfg)
{
    return (cfg->S100MDiv0 != 0U && cfg->S100MDiv1 != 0U) ? 1 : 0;
}
#else
#define cfg_has_div(c)  0
#endif

static void gem3_100m_div(const XEmacPs_Config *cfg, u32 *d0, u32 *d1)
{
    *d0 = GEM3_DIV0_100M;
    *d1 = GEM3_DIV1_100M;
#if GEM3_CFG_FROM_BSP
    if (cfg_has_div(cfg) != 0) { *d0 = (u32)cfg->S100MDiv0; *d1 = (u32)cfg->S100MDiv1; }
#else
    (void)cfg;
#endif
}

static void gem3_clk_prepare(const XEmacPs_Config *cfg)
{
    u32 rst = Xil_In32(CRL_APB_RST_LPD_IOU0);
    u32 ref = Xil_In32(CRL_APB_GEM3_REF_CTRL);
    u32 want = ref;
    u32 d0, d1;

    /* Read-only first: this is what the boot firmware / Linux left behind. */
    xil_printf("[gem3] CRL_APB before : GEM3_REF_CTRL=%08x  RST_LPD_IOU0=%08x (GEM3_RESET=%u)\r\n",
               (unsigned)ref, (unsigned)rst, (unsigned)((rst >> 3) & 1U));

    if ((rst & GEM3_RESET_BIT) != 0U) {                 /* never touch GEM3 registers while it is held in reset */
        Xil_Out32(CRL_APB_RST_LPD_IOU0, rst & ~GEM3_RESET_BIT);
        delay_ms(2);
        rst = Xil_In32(CRL_APB_RST_LPD_IOU0);
        xil_printf("[gem3] GEM3 was in reset -> released (RST_LPD_IOU0=%08x)\r\n", (unsigned)rst);
        if ((rst & GEM3_RESET_BIT) != 0U) {
            xil_printf("[gem3] WARN: GEM3_RESET still set -> write blocked (XPPU?)\r\n");
        }
    }

    gem3_100m_div(cfg, &d0, &d1);
    xil_printf("[gem3] 100 Mbit/s ref clock: DIV0=%u DIV1=%u (%s)\r\n", (unsigned)d0, (unsigned)d1,
               (GEM3_CFG_FROM_BSP && cfg_has_div(cfg)) ? "from XSA" : "built-in default, IOPLL=1500MHz assumed");
    want &= ~(GEM_REF_DIV0_MASK | GEM_REF_DIV1_MASK);
    want |= (d0 << GEM_REF_DIV0_SHIFT) | (d1 << GEM_REF_DIV1_SHIFT);
    want |= GEM_REF_CLKACT | GEM_REF_RX_CLKACT;

    if (want != ref) {
        Xil_Out32(CRL_APB_GEM3_REF_CTRL, want);
        delay_ms(1);
        ref = Xil_In32(CRL_APB_GEM3_REF_CTRL);
        xil_printf("[gem3] GEM3_REF_CTRL -> %08x (100 Mbit/s: DIV0=%u DIV1=%u)\r\n", (unsigned)ref,
                   (unsigned)((ref & GEM_REF_DIV0_MASK) >> GEM_REF_DIV0_SHIFT),
                   (unsigned)((ref & GEM_REF_DIV1_MASK) >> GEM_REF_DIV1_SHIFT));
        if (ref != want) {
            xil_printf("[gem3] WARN: GEM3_REF_CTRL read-back %08x != wanted %08x -> write blocked (XPPU?)\r\n",
                       (unsigned)ref, (unsigned)want);
        }
    } else {
        xil_printf("[gem3] GEM3_REF_CTRL already correct for 100 Mbit/s\r\n");
    }
}
#endif

/* ---------------------------------------------------------------------- */
/* PHY nRESET line on a PS GPIO (MIO).  K26 DT (live board):               */
/*   reset-gpios = <&gpio 38 GPIO_ACTIVE_LOW>; reset-assert-us = <100>;     */
/*   reset-deassert-us = <280>;                                            */
/* Banks: 0 = MIO0..25, 1 = MIO26..51, 2 = MIO52..77.                      */
/* MASK_DATA writes touch only the addressed pin; DIRM/OUTEN are RMW'd for  */
/* that one bit.  (Same register layout as the Xilinx gpiops driver.)      */
/* ---------------------------------------------------------------------- */
#if GEM3_PHY_RST_MIO >= 0
#define PSGPIO_BASE            0xFF0A0000U
#define PSGPIO_MASKDATA(b, hi) (PSGPIO_BASE + (b) * 0x08U + ((hi) ? 0x04U : 0x00U))
#define PSGPIO_DATA_RO(b)      (PSGPIO_BASE + 0x60U  + (b) * 0x04U)
#define PSGPIO_DIRM(b)         (PSGPIO_BASE + 0x204U + (b) * 0x40U)
#define PSGPIO_OUTEN(b)        (PSGPIO_BASE + 0x208U + (b) * 0x40U)

static void psgpio_locate(u32 mio, u32 *bank, u32 *bit)
{
    if (mio < 26U)      { *bank = 0U; *bit = mio; }
    else if (mio < 52U) { *bank = 1U; *bit = mio - 26U; }
    else                { *bank = 2U; *bit = mio - 52U; }
}

static void psgpio_write(u32 mio, u32 level)
{
    u32 bank, bit, b;

    psgpio_locate(mio, &bank, &bit);
    b = bit & 15U;                                   /* 16 data bits per LSW/MSW register */
    Xil_Out32(PSGPIO_MASKDATA(bank, bit > 15U),
              (~(1U << (b + 16U))) & (((level & 1U) << b) | 0xFFFF0000U));
}

static u32 psgpio_read(u32 mio)
{
    u32 bank, bit;

    psgpio_locate(mio, &bank, &bit);
    return (Xil_In32(PSGPIO_DATA_RO(bank)) >> bit) & 1U;
}

static void phy_hw_reset(void)
{
    u32 bank, bit, v;

    psgpio_locate((u32)GEM3_PHY_RST_MIO, &bank, &bit);
    xil_printf("[gem3] PHY nRESET = MIO%u: pin=%u dirm=%u oen=%u  -> pulsing low\r\n",
               (unsigned)GEM3_PHY_RST_MIO, (unsigned)psgpio_read((u32)GEM3_PHY_RST_MIO),
               (unsigned)((Xil_In32(PSGPIO_DIRM(bank)) >> bit) & 1U),
               (unsigned)((Xil_In32(PSGPIO_OUTEN(bank)) >> bit) & 1U));

    psgpio_write((u32)GEM3_PHY_RST_MIO, 1U);         /* data=1 (not in reset) BEFORE enabling the driver: no glitch */
    v = Xil_In32(PSGPIO_DIRM(bank));  Xil_Out32(PSGPIO_DIRM(bank),  v | (1U << bit));
    v = Xil_In32(PSGPIO_OUTEN(bank)); Xil_Out32(PSGPIO_OUTEN(bank), v | (1U << bit));
    delay_ms(2);
    psgpio_write((u32)GEM3_PHY_RST_MIO, 0U);         /* assert nRESET   (DT: >= 100 us) */
    delay_ms(5);
    psgpio_write((u32)GEM3_PHY_RST_MIO, 1U);         /* release nRESET  (DT: wait >= 280 us before MDIO) */
    delay_ms(30);
    xil_printf("[gem3] PHY nRESET released (pin reads %u)\r\n", (unsigned)psgpio_read((u32)GEM3_PHY_RST_MIO));
}
#endif

/* ---------------------------------------------------------------------- */
/* PHY (TI DP83867 on the K26 SOM): find, reset, RGMII delays, autoneg.    */
/* ---------------------------------------------------------------------- */
/* Statistics of the last scan -- used for the failure message. */
static u32 g_scan_timeouts, g_scan_allones, g_scan_other;
static u16 g_scan_other_val;

static int phy_find(u32 *addr, u16 *id1, u16 *id2)
{
    u32 i, a;
    u16 s = 0;

    g_scan_timeouts = 0U;
    g_scan_allones  = 0U;
    g_scan_other    = 0U;
    g_scan_other_val = 0U;

    for (i = 0; i < 32U; i++) {
        a = ((u32)GEM3_PHY_ADDR_HINT + i) & 31U;          /* DT address first, then wrap around */
        if (mdio_rd(a, 1, &s) != 0) {
            if (++g_scan_timeouts >= 2U) { return -1; }   /* MDIO engine is dead: do not wait 32 times */
            continue;
        }
        if (s == 0xFFFFU) { g_scan_allones++; continue; }
        /* same detect rule as the Xilinx lwIP port: BMSR caps 0x1808 */
        if ((s & 0x1808U) == 0x1808U) {
            *addr = a;
            (void)mdio_rd(a, 2, id1);
            (void)mdio_rd(a, 3, id2);
            return 0;
        }
        if (g_scan_other++ == 0U) { g_scan_other_val = s; }
    }
    return -1;
}

static void phy_scan_failed_msg(void)
{
    if (g_scan_timeouts >= 2U) {
        xil_printf("[gem3] MDIO engine never completes (NWSR.MDIOIDLE stays 0): GEM3 clock/reset not OK?\r\n");
    } else if (g_scan_allones >= 32U) {
#if GEM3_PHY_RST_MIO >= 0
        xil_printf("[gem3] MDIO works but no PHY answers (all 0xFFFF): PHY still in reset (MIO%d pulsed by us), "
                   "MIO76/77 not muxed to GEM3 MDIO, or PHY unpowered\r\n", (int)GEM3_PHY_RST_MIO);
#else
        xil_printf("[gem3] MDIO works but no PHY answers (all 0xFFFF): PHY may be held in reset (this build does "
                   "not drive the reset pin), MIO76/77 not muxed to GEM3 MDIO, or PHY unpowered\r\n");
#endif
    } else {
        xil_printf("[gem3] MDIO answers but no valid PHY: %u odd replies, first BMSR=%04x\r\n",
                   (unsigned)g_scan_other, (unsigned)g_scan_other_val);
    }
}

static int phy_reset(u32 phy)
{
    u16 v = 0x8000U;
    u32 i;

    (void)mdio_wr(phy, 0, 0x8000U);                 /* BMCR soft reset */
    for (i = 0; i < 100U; i++) {
        delay_ms(10);
        if (mdio_rd(phy, 0, &v) == 0 && (v & 0x8000U) == 0U) { break; }
    }
    delay_ms(300);                                  /* let the PHY settle before extended regs */
    return ((v & 0x8000U) != 0U) ? -1 : 0;
}

/* Same values the Linux driver applies for the K26 SOM DT:
 *   phy-mode = "rgmii-id", ti,rx-internal-delay = <0x8>, ti,tx-internal-delay = <0xa>,
 *   ti,fifo-depth = <1>, ti,dp83867-rxctrl-strap-quirk                           */
#define TI_PHYCR        0x10U
#define TI_CFG4         0x0031U
#define TI_RGMIICTL     0x0032U
#define TI_RGMIIDCTL    0x0086U
#define TI_RX_DELAY     0x8U
#define TI_TX_DELAY     0xAU

static void ti_dp83867_tune(u32 phy)
{
    u16 v;

    (void)mdio_rd(phy, TI_PHYCR, &v);
    v = (u16)((v & ~((1U << 11) | (3U << 14))) | (1U << 14));   /* FIFO depth = 1 */
    (void)mdio_wr(phy, TI_PHYCR, v);

    ti_ext_wr(phy, TI_RGMIIDCTL, (u16)((TI_TX_DELAY << 4) | TI_RX_DELAY));

    v = ti_ext_rd(phy, TI_RGMIICTL);
    v |= 0x0003U;                                  /* TX + RX internal clock delay enable */
    ti_ext_wr(phy, TI_RGMIICTL, v);

    v = ti_ext_rd(phy, TI_CFG4);
    v &= (u16)~0x0080U;                            /* rxctrl strap quirk */
    ti_ext_wr(phy, TI_CFG4, v);
}

/* Link up AND (unless the PHY is forced) auto-negotiation complete.  After the loopback self-test the BMSR link
 * bit can still read "up" for a moment, and a parallel-detect link (partner without auto-negotiation) never
 * completes AN, hence the fall-back to "link bit only" after 2 s.                                          */
static int phy_wait_link(u32 phy, u32 timeout_ms)
{
    u16 s = 0;
    u32 t;
#if GEM3_FORCE_100FD
    const u16 need = 0x0004U;
#else
    const u16 need = 0x0024U;
#endif

    for (t = 0; t < timeout_ms; t += 100U) {
        (void)mdio_rd(phy, 1, &s);                 /* link bit is latched-low: read twice */
        (void)mdio_rd(phy, 1, &s);
        if ((s & need) == need) { return 1; }
        if (t >= 2000U && (s & 0x0004U) != 0U) { return 1; }
        delay_ms(100);
    }
    return 0;
}

/* Phase A: PHY hardware reset, MDIO scan, soft reset, DP83867 RGMII delays, advertise 100BASE-TX FD only.
 * Returns the PHY address or -1.  The link is NOT started here (phy_link() does that), so the self-test can run
 * in between.                                                                                                */
static int phy_open(void)
{
    u16 id1 = 0, id2 = 0;
    u32 phy = 0;

#if GEM3_PHY_RST_MIO >= 0
    phy_hw_reset();
#endif

    xil_printf("[gem3] scanning MDIO (DT says PHY address %d)...\r\n", (int)GEM3_PHY_ADDR_HINT);
    if (phy_find(&phy, &id1, &id2) != 0) { phy_scan_failed_msg(); return -1; }
    g_phy = phy;
    xil_printf("[gem3] PHY addr %u  id %04x:%04x (DP83867 = 2000:a231)\r\n",
               (unsigned)phy, (unsigned)id1, (unsigned)id2);

    if (phy_reset(phy) != 0) { xil_printf("[gem3] WARN: PHY soft reset did not complete\r\n"); }

    if (id1 == 0x2000U) {                          /* TI PHY (DP83867) */
        u16 pc = 0;
        ti_dp83867_tune(phy);
        (void)mdio_rd(phy, TI_PHYCR, &pc);
        xil_printf("[gem3] DP83867 PHYCR=%04x  RGMIICTL=%04x (want xx03)  RGMIIDCTL=%04x (want 00a8)  CFG4=%04x\r\n",
                   (unsigned)pc, (unsigned)ti_ext_rd(phy, TI_RGMIICTL),
                   (unsigned)ti_ext_rd(phy, TI_RGMIIDCTL), (unsigned)ti_ext_rd(phy, TI_CFG4));
    } else {
        xil_printf("[gem3] WARN: non-TI PHY, RGMII delay left as strapped\r\n");
    }

#if !GEM3_FORCE_100FD
    (void)mdio_wr(phy, 4, 0x0101U);                /* advertise 100BASE-TX FD only (selector=1) */
    (void)mdio_wr(phy, 9, 0x0000U);                /* no 1000BASE-T advertisement */
#endif
    return (int)phy;
}

/* Phase B: (re)start auto-negotiation (this also leaves the self-test's loopback mode: BMCR bit 14 = 0) and wait. */
static void phy_link(void)
{
    u16 sts = 0;

#if GEM3_FORCE_100FD
    (void)mdio_wr(g_phy, 0, 0x2100U);              /* forced 100M full duplex, autoneg off */
#else
    (void)mdio_wr(g_phy, 0, 0x1200U);              /* autoneg enable + restart */
#endif
    delay_ms(20);                                  /* let the restart take effect before the first BMSR poll */

    g_link_up = 0U;
    if (phy_wait_link(g_phy, GEM3_LINK_TIMEOUT_MS) != 0) {
        g_link_up = 1U;
        (void)mdio_rd(g_phy, 0x11, &sts);           /* DP83867 PHYSTS: [15:14] speed (01=100M), [13] full duplex */
        xil_printf("[gem3] link UP  PHYSTS=%04x (speed bits %u: 1=100M, duplex %s)\r\n",
                   (unsigned)sts, (unsigned)((sts >> 14) & 3U), ((sts >> 13) & 1U) ? "FULL" : "HALF");
    } else {
        xil_printf("[gem3] link DOWN after %u ms (cable / slave power?)\r\n",
                   (unsigned)GEM3_LINK_TIMEOUT_MS);
    }
}

/* ---------------------------------------------------------------------- */
/* BD helpers (lwIP recipe)                                               */
/* ---------------------------------------------------------------------- */
/* Hand an RX BD (already Alloc'd + ToHw'd in the driver's books) to the MAC. */
static void rx_arm(XEmacPs_BdRing *r, XEmacPs_Bd *bd)
{
    u32 idx = BD_INDEX(r, bd);
    volatile u32 *w = (volatile u32 *)bd;

    w[1] = 0U;
    w[0] = (idx == (RXBD_CNT - 1U)) ? (u32)XEMACPS_RXBUF_WRAP_MASK : 0U;   /* NEW=0 */
    dsb();
    Xil_DCacheInvalidateRange((INTPTR)RxBuf[idx], GEM_BUF);
    XEmacPs_BdSetAddressRx(bd, (UINTPTR)RxBuf[idx]);    /* RMW: keeps WRAP, NEW stays 0 -> MAC owns it */
    dsb();
}

static XEmacPs_Config *gem3_lookup_cfg(void)
{
#if GEM3_CFG_FROM_BSP
#ifdef SDT
    return XEmacPs_LookupConfig(GEM3_BASEADDR);
#else
    u32 id;
    XEmacPs_Config *c;

    for (id = 0; id < (u32)XPAR_XEMACPS_NUM_INSTANCES; id++) {
        c = XEmacPs_LookupConfig((u16)id);
        if (c != NULL && c->BaseAddress == GEM3_BASEADDR) { return c; }   /* [[CHECK]] */
    }
    return NULL;
#endif
#else
    /* driver sources copied into the app: no XSA table, build the config by hand.
     * CfgInitialize() only needs DeviceId + BaseAddress; the rest stays zero.     */
    static XEmacPs_Config c;

    c.DeviceId    = 0U;
    c.BaseAddress = GEM3_BASEADDR;
    return &c;
#endif
}

/* ---------------------------------------------------------------------- */
/* Rings, RX restart, statistics, self-test, listen probe  (rev 5)          */
/* ---------------------------------------------------------------------- */

/* Queue base registers are written DIRECTLY: XEmacPs_SetQueuePtr() silently does nothing once the device is
 * "started" (xemacps.c), and gem3_rx_restart() has to re-program the RX pointer after XEmacPs_Start().       */
static void q_write(u32 off, u32 val)
{
    Xil_Out32((u32)emac.Config.BaseAddress + off, val);
}

/* RX ring: every BD armed with its buffer (the MAC owns them), RX queue 0 -> this ring, RX queue 1 -> terminate BD. */
static int gem3_rx_ring_setup(void)
{
    XEmacPs_BdRing *rx = &XEmacPs_GetRxRing(&emac);
    XEmacPs_Bd      tmpl, *bd, *term;
    u32             i;

    XEmacPs_BdClear(&tmpl);
    if (XEmacPs_BdRingCreate(rx, GEM3_DMA_BASE + RXBD_OFFSET, GEM3_DMA_BASE + RXBD_OFFSET,
                             XEMACPS_BD_ALIGNMENT, RXBD_CNT) != XST_SUCCESS) { return -5; }
    if (XEmacPs_BdRingClone(rx, &tmpl, XEMACPS_RECV) != XST_SUCCESS)           { return -5; }

    for (i = 0; i < RXBD_CNT; i++) {
        if (XEmacPs_BdRingAlloc(rx, 1, &bd) != XST_SUCCESS) { return -6; }
        if (XEmacPs_BdRingToHw(rx, 1, bd) != XST_SUCCESS)   { return -6; }
        rx_arm(rx, bd);
    }

    q_write(XEMACPS_RXQBASE_OFFSET, (u32)(UINTPTR)rx->BaseBdAddr);              /* RX = queue 0 */
    term = (XEmacPs_Bd *)(GEM3_DMA_BASE + RXTERM_OFFSET);                       /* RX Q1 -> "new+wrap" BD */
    XEmacPs_BdClear(term);
    XEmacPs_BdSetAddressRx(term, (XEMACPS_RXBUF_NEW_MASK | XEMACPS_RXBUF_WRAP_MASK));
    q_write(XEMACPS_RXQ1BASE_OFFSET, (u32)(UINTPTR)term);
    dsb();
    return 0;
}

/* TX ring (template "used"), TX queue 1 -> this ring (ZynqMP), TX queue 0 -> terminate BD. */
static int gem3_tx_ring_setup(void)
{
    XEmacPs_BdRing *tx = &XEmacPs_GetTxRing(&emac);
    XEmacPs_Bd      tmpl, *term;

    XEmacPs_BdClear(&tmpl);
    XEmacPs_BdSetStatus(&tmpl, XEMACPS_TXBUF_USED_MASK);
    if (XEmacPs_BdRingCreate(tx, GEM3_DMA_BASE + TXBD_OFFSET, GEM3_DMA_BASE + TXBD_OFFSET,
                             XEMACPS_BD_ALIGNMENT, TXBD_CNT) != XST_SUCCESS) { return -5; }
    if (XEmacPs_BdRingClone(tx, &tmpl, XEMACPS_SEND) != XST_SUCCESS)           { return -5; }

    q_write(XEMACPS_TXQ1BASE_OFFSET, (u32)(UINTPTR)tx->BaseBdAddr);             /* TX = queue 1 */
    term = (XEmacPs_Bd *)(GEM3_DMA_BASE + TXTERM_OFFSET);                       /* TX Q0 -> "used+wrap" BD */
    XEmacPs_BdClear(term);
    XEmacPs_BdSetStatus(term, (XEMACPS_TXBUF_USED_MASK | XEMACPS_TXBUF_WRAP_MASK));
    q_write(XEMACPS_TXQBASE_OFFSET, (u32)(UINTPTR)term);
    dsb();
    return 0;
}

/* RX off -> fresh BD ring (all BDs armed) + queue pointer -> RX on.  The GEM only takes a new queue pointer while
 * RX is off, and a ring that filled up while nobody was reading (BUFFNA) has to start clean (Linux's macb driver
 * toggles RX enable on "RX used bit read" for the same reason).  settle_ms = 0 when called from gem3_recv().       */
static int gem3_rx_restart(u32 settle_ms)
{
    u32 b = (u32)emac.Config.BaseAddress;
    int rc;

    Xil_Out32(b + XEMACPS_NWCTRL_OFFSET, Xil_In32(b + XEMACPS_NWCTRL_OFFSET) & ~XEMACPS_NWCTRL_RXEN_MASK);
    dsb();
    if (settle_ms != 0U) { delay_ms(settle_ms); }             /* a frame that is just being written to memory finishes */
    rc = gem3_rx_ring_setup();
    Xil_Out32(b + XEMACPS_RXSR_OFFSET, XEMACPS_SR_ALL_MASK);  /* write-1-to-clear: BUFFNA, FRAMERX, ... */
    Xil_Out32(b + XEMACPS_NWCTRL_OFFSET, Xil_In32(b + XEMACPS_NWCTRL_OFFSET) | XEMACPS_NWCTRL_RXEN_MASK);
    return rc;
}

/* All MAC statistics counters are read ONCE into g_stat[] (they may be clear-on-read) and printed from there. */
#define G3_STAT_FIRST   0x100U
#define G3_STAT_WORDS   (((0x1B4U - 0x100U) / 4U) + 1U)
static u32 g_stat[G3_STAT_WORDS];
#define G3_S(off)       (g_stat[((off) - G3_STAT_FIRST) / 4U])

static void gem3_stats_snapshot(void)
{
    u32 i;

    for (i = 0U; i < G3_STAT_WORDS; i++) { g_stat[i] = G3_RD(G3_STAT_FIRST + (4U * i)); }
}

/* Zero the MAC counters and the software-side frame log; remember when. */
static void gem3_stats_clear(void)
{
    u32 b = (u32)emac.Config.BaseAddress;

    Xil_Out32(b + XEMACPS_NWCTRL_OFFSET, Xil_In32(b + XEMACPS_NWCTRL_OFFSET) | XEMACPS_NWCTRL_STATCLR_MASK);
    dsb();
    Xil_Out32(b + XEMACPS_RXSR_OFFSET, XEMACPS_SR_ALL_MASK);
    Xil_Out32(b + XEMACPS_TXSR_OFFSET, XEMACPS_SR_ALL_MASK);
    g_rx_total = 0U; g_rx_ecat = 0U; g_rx_other = 0U; g_rx_logged = 0U; g_rx_restarts = 0U;
    g_t_stats0 = xTaskGetTickCount();
}

/* Called by gem3_recv() for every frame it hands out. */
static void rxlog_note(const u8 *p, u32 len)
{
    g_rx_total++;
    if (len >= 14U && p[12] == 0x88U && p[13] == 0xA4U) { g_rx_ecat++; } else { g_rx_other++; }
    if (g_rx_logged < RXLOG_N) {
        u32 n = (len < RXLOG_B) ? len : RXLOG_B;

        g_rxlog_len[g_rx_logged] = (u16)len;
        memcpy(g_rxlog[g_rx_logged], p, (size_t)n);
        g_rx_logged++;
    }
}

static void rxlog_print(void)
{
    u32 k, i, n;

    for (k = 0U; k < g_rx_logged; k++) {
        n = (g_rxlog_len[k] < RXLOG_B) ? (u32)g_rxlog_len[k] : RXLOG_B;
        xil_printf("[gem3]   #%u len=%u:", (unsigned)k, (unsigned)g_rxlog_len[k]);
        for (i = 0U; i < n; i++) { xil_printf(" %02x", (unsigned)g_rxlog[k][i]); }
        xil_printf("\r\n");
    }
}

#if GEM3_SELFTEST
#define ST_FRAMES   4U
#define ST_LEN      60U

static void st_make(u8 *f, u32 seq)
{
    u32 i;

    for (i = 0U; i < 6U; i++) { f[i] = 0xFFU; f[6U + i] = g_mac[i]; }       /* dst broadcast, src = our MAC */
    f[12] = 0x88U; f[13] = 0xB5U;                                           /* IEEE 802 local experimental ethertype 1 */
    f[14] = (u8)'G'; f[15] = (u8)'3'; f[16] = (u8)seq;
    for (i = 17U; i < ST_LEN; i++) { f[i] = (u8)((i * 7U) + (seq * 31U)); }
}

/* PHY digital loopback (BMCR bit 14): frames go MAC -> RGMII -> PHY -> RGMII -> MAC, through the real TX BD ring,
 * RX BD ring, DMA and cache handling.  PASS means the MAC side is fine whatever is (not) plugged in.             */
static void gem3_selftest(void)
{
    u8  tx[ST_LEN], ref[ST_LEN], rx[GEM_BUF];
    u32 sent = 0U, good = 0U, bad = 0U, other = 0U, seen = 0U, k, t;
    u16 s = 0;
    int n;

    if (mdio_wr(g_phy, 0, 0x6100U) != 0) {                   /* BMCR: loopback + 100M + full duplex, autoneg off */
        xil_printf("[gem3] selftest skipped: PHY write failed\r\n");
        return;
    }
    for (t = 0U; t < 30U; t++) {                             /* in loopback the link bit comes up within a few ms */
        (void)mdio_rd(g_phy, 1, &s);
        (void)mdio_rd(g_phy, 1, &s);
        if ((s & 0x0004U) != 0U) { break; }
        delay_ms(10);
    }
    delay_ms(20);
    gem3_stats_clear();

    for (k = 0U; k < ST_FRAMES; k++) {
        st_make(tx, k);
        if (gem3_send(tx, (int)ST_LEN) == (int)ST_LEN) { sent++; }
    }
    for (t = 0U; t < 100U && (good + bad + other) < sent; t++) {          /* up to 100 ms for them to come back */
        for (k = 0U; k < 8U; k++) {
            n = gem3_recv(rx, (int)sizeof rx);
            if (n <= 0) { break; }
            if ((u32)n == ST_LEN && rx[12] == 0x88U && rx[13] == 0xB5U && rx[14] == (u8)'G' && rx[15] == (u8)'3' &&
                rx[16] < ST_FRAMES && (seen & (1U << rx[16])) == 0U) {
                st_make(ref, rx[16]);
                if (memcmp(rx, ref, ST_LEN) == 0) { good++; seen |= 1U << rx[16]; } else { bad++; }
            } else {
                other++;
            }
        }
        if ((good + bad + other) < sent) { delay_ms(1); }
    }

    gem3_stats_snapshot();
    xil_printf("[gem3] selftest (PHY loopback): sent %u back %u corrupt %u other %u | MAC tx=%u rx=%u resErr=%u fcsErr=%u"
               " RXSR=%08x -> %s\r\n",
               (unsigned)sent, (unsigned)good, (unsigned)bad, (unsigned)other,
               (unsigned)G3_S(G3_TXCNT), (unsigned)G3_S(G3_RXCNT), (unsigned)G3_S(G3_RXRESERR),
               (unsigned)G3_S(G3_RXFCS), (unsigned)Xil_In32((u32)emac.Config.BaseAddress + XEMACPS_RXSR_OFFSET),
               (good == ST_FRAMES && sent == ST_FRAMES && bad == 0U) ? "PASS" : "FAIL");
    if (good != ST_FRAMES || sent != ST_FRAMES || bad != 0U) {
        if (sent < ST_FRAMES) {
            xil_printf("[gem3]   -> the TX ring refused frames: the MAC does not complete transmissions (see TXSR/NWSR)\r\n");
        } else if (G3_S(G3_RXCNT) == 0U && G3_S(G3_RXRESERR) == 0U && G3_S(G3_RXFCS) == 0U) {
            xil_printf("[gem3]   -> nothing reached the MAC receiver: PHY loopback ineffective, or the RGMII RX path"
                       " (RX clock / GEM3_REF_CTRL RX_CLKACT, DP83867 RX delay)\r\n");
        } else if (G3_S(G3_RXFCS) != 0U) {
            xil_printf("[gem3]   -> frames arrive with bad FCS: RGMII timing (DP83867 internal delays)\r\n");
        } else if (bad != 0U) {
            xil_printf("[gem3]   -> frames arrive but their content is wrong: RX buffer / cache handling\r\n");
        } else {
            xil_printf("[gem3]   -> the MAC received frames (or counted resource errors) but software got none:"
                       " RX DMA / BD ring (see the RX BD lines of gem3_debug_dump)\r\n");
        }
    }
}
#endif

#if GEM3_LISTEN_MS > 0
/* After link-up and with NOTHING transmitted: what does the wire carry?  A powered EtherCAT drive is silent. */
static void gem3_listen(u32 ms)
{
    u8  rx[GEM_BUF];
    u32 t, k;

    gem3_stats_clear();
    for (t = 0U; t < ms; t++) {
        delay_ms(1);
        for (k = 0U; k < 64U; k++) {
            if (gem3_recv(rx, (int)sizeof rx) <= 0) { break; }        /* counted + logged inside gem3_recv() */
        }
    }
    gem3_stats_snapshot();
    xil_printf("[gem3] listen %u ms, nothing transmitted: SW saw %u frame(s) (EtherCAT %u, other %u) | MAC rx=%u"
               " resErr=%u fcsErr=%u\r\n",
               (unsigned)ms, (unsigned)g_rx_total, (unsigned)g_rx_ecat, (unsigned)g_rx_other,
               (unsigned)G3_S(G3_RXCNT), (unsigned)G3_S(G3_RXRESERR), (unsigned)G3_S(G3_RXFCS));
    if (g_rx_logged != 0U) {
        xil_printf("[gem3]   first frames (dst 6B, src 6B, ethertype 2B, payload; 88 a4 = EtherCAT):\r\n");
        rxlog_print();
    } else {
        xil_printf("[gem3]   wire is quiet (normal for a powered EtherCAT drive: it only answers)\r\n");
    }
}
#endif

/* ---------------------------------------------------------------------- */
/* gem3_init: bring GEM3 up for raw EtherCAT traffic.                      */
/*   0 = OK ; <0 = error (see code)                                        */
/* ---------------------------------------------------------------------- */
int gem3_init(const uint8_t *mac)
{
    XEmacPs_Config *cfg;
    u32             base, v;
    int             rc;

    (void)mac;     /* we program our own unicast g_mac; SOEM's priMAC is only used inside frames */

    cfg = gem3_lookup_cfg();
    if (cfg == NULL) { xil_printf("[gem3] GEM3 (0xFF0E0000) not in BSP\r\n"); return -1; }
    g_tx_full_warned = 0U;
    g_link_up = 0U;
    xil_printf("[gem3] rev5 init: GEM3 @0x%08x, config from %s\r\n", (unsigned)GEM3_BASEADDR,
               GEM3_CFG_FROM_BSP ? "BSP/XSA" : "built-in (driver copied into app)");

#if GEM3_PREPARE_CLK
    gem3_clk_prepare(cfg);
#endif

    /* If the R5 freezes after this line, GEM3 itself does not answer on the bus
     * (clock gated / held in reset / access blocked).                          */
    xil_printf("[gem3] touching GEM3 registers (reset + MDIO enable)...\r\n");
    if (XEmacPs_CfgInitialize(&emac, cfg, cfg->BaseAddress) != XST_SUCCESS) { return -1; }
    base = (u32)emac.Config.BaseAddress;
    xil_printf("[gem3] GEM3 answers: NWCTRL=%08x NWCFG=%08x  (GEM version %u)\r\n",
               (unsigned)Xil_In32(base + XEMACPS_NWCTRL_OFFSET), (unsigned)Xil_In32(base + XEMACPS_NWCFG_OFFSET),
               (unsigned)emac.Version);
    if (emac.Version <= 2U) {                       /* Zynq-7000 style GEM: not supported here */
        xil_printf("[gem3] unexpected GEM version %u\r\n", (unsigned)emac.Version);
        return -3;
    }

    XEmacPs_SetMdioDivisor(&emac, MDC_DIV_224);
    (void)XEmacPs_SetMacAddress(&emac, (void *)g_mac, 1);

    /* promiscuous + broadcast; strip RX FCS (SOEM wants no FCS); TX FCS is appended by the MAC (BD NOCRC bit
     * stays 0).  rev 5: RX/TX stay OFF here -- XEmacPs_Start() switches them on once the rings exist
     * (XEmacPs_Reset() left the TX/RX-enable flags in the driver's option word).                              */
    (void)XEmacPs_SetOptions(&emac,
        XEMACPS_PROMISC_OPTION |
        XEMACPS_BROADCAST_OPTION |
        XEMACPS_FCS_STRIP_OPTION);
    (void)XEmacPs_ClearOptions(&emac,
        XEMACPS_FLOW_CONTROL_OPTION |
        XEMACPS_LENTYPE_ERR_OPTION |
        XEMACPS_RX_CHKSUM_ENABLE_OPTION |
        XEMACPS_TX_CHKSUM_ENABLE_OPTION |
        XEMACPS_JUMBO_ENABLE_OPTION);

    XEmacPs_SetOperatingSpeed(&emac, 100);          /* EtherCAT = 100 Mbit/s, full duplex (FDEN set by Reset) */

    /* ---- non-cacheable window for BDs (once) ---- */
    if (g_mpu_done == 0) {
        if (Xil_SetMPURegion((INTPTR)GEM3_DMA_BASE, GEM3_DMA_SIZE,
                             STRONG_ORDERD_SHARED | PRIV_RW_USER_RW) != (u32)XST_SUCCESS) {
            xil_printf("[gem3] MPU region for DMA window failed\r\n");
            return -4;
        }
        g_mpu_done = 1;
    }

    /* ---- BD rings + queue pointers, all BEFORE the MAC can receive or transmit ---- */
    rc = gem3_tx_ring_setup();
    if (rc != 0) { xil_printf("[gem3] TX BD ring set-up failed (%d)\r\n", rc); return rc; }
    rc = gem3_rx_ring_setup();
    if (rc != 0) { xil_printf("[gem3] RX BD ring set-up failed (%d)\r\n", rc); return rc; }

    if (phy_open() < 0) {
        xil_printf("[gem3] init failed: no PHY\r\n");
        return -2;
    }

    XEmacPs_Start(&emac);                           /* RX/TX on: rings and queue pointers are in place */

    /* polled operation: keep the GEM from raising interrupts */
    XEmacPs_IntDisable(&emac, XEMACPS_IXR_ALL_MASK);
    XEmacPs_IntQ1Disable(&emac, XEMACPS_INTQ1_IXR_ALL_MASK);

    v = Xil_In32(base + XEMACPS_NWCTRL_OFFSET);     /* belt and braces: both must be on now */
    if ((v & (XEMACPS_NWCTRL_TXEN_MASK | XEMACPS_NWCTRL_RXEN_MASK)) !=
        (XEMACPS_NWCTRL_TXEN_MASK | XEMACPS_NWCTRL_RXEN_MASK)) {
        xil_printf("[gem3] NWCTRL=%08x after start: forcing TX/RX enable\r\n", (unsigned)v);
        Xil_Out32(base + XEMACPS_NWCTRL_OFFSET, v | XEMACPS_NWCTRL_TXEN_MASK | XEMACPS_NWCTRL_RXEN_MASK);
    }

#if GEM3_SELFTEST
    gem3_selftest();
#endif

    phy_link();                                     /* auto-negotiation + link wait (also leaves the loopback mode) */

    rc = gem3_rx_restart(1U);                       /* whatever arrived while we waited: start on a clean ring */
    if (rc != 0) { xil_printf("[gem3] RX ring restart failed (%d)\r\n", rc); return rc; }
    gem3_stats_clear();

#if GEM3_LISTEN_MS > 0
    if (g_link_up != 0U) { gem3_listen((u32)GEM3_LISTEN_MS); }
#endif
    gem3_stats_clear();                             /* gem3_debug_dump() then shows what happened since init */

    xil_printf("[gem3] init OK  (GEM ver %u, 100M FD, polled)\r\n", (unsigned)emac.Version);
    return 0;      /* success: 0 (rtk/bfin convention) */
}

/* ---------------------------------------------------------------------- */
/* gem3_send: queue one raw frame and kick the DMA (non-blocking).         */
/* ---------------------------------------------------------------------- */
static void gem3_warn_tx_full(const XEmacPs_BdRing *tx)
{
    u32 b, oldest = 0U;

    if (g_tx_full_warned != 0U) { return; }          /* once per gem3_init(): never flood the 1 kHz path */
    g_tx_full_warned = 1U;
    b = (u32)emac.Config.BaseAddress;
    if (tx->HwHead != NULL) { oldest = XEmacPs_BdRead(tx->HwHead, XEMACPS_BD_STAT_OFFSET); }
    xil_printf("[gem3] WARN TX ring full: %u frames in flight, oldest BD status=%08x (bit31 USED, bit15 LAST),"
               " TXSR=%08x NWSR=%08x -> the MAC is not completing frames (link / clock / DMA-SMMU?)\r\n",
               (unsigned)tx->HwCnt, (unsigned)oldest,
               (unsigned)Xil_In32(b + XEMACPS_TXSR_OFFSET), (unsigned)Xil_In32(b + XEMACPS_NWSR_OFFSET));
}

int gem3_send(const void *frame, int len)
{
    XEmacPs_BdRing *tx = &XEmacPs_GetTxRing(&emac);
    XEmacPs_Bd *bd, *done;
    volatile u32 *w;
    u32 idx;

    if (len <= 0 || (u32)len > GEM_BUF) { return -1; }

    /* Reclaim finished BDs, oldest first, one at a time, and only when WE have seen USED set on
     * the oldest one (see "REV 4" above: with driver <= v3.16 a big limit reclaims BDs that the
     * MAC has not finished).  Reclaim = reset the BD to "used" (+wrap on the last one).        */
    while (tx->HwCnt > 0U && XEmacPs_BdIsTxUsed(tx->HwHead)) {
        done = NULL;
        if (XEmacPs_BdRingFromHwTx(tx, 1U, &done) != 1U || done == NULL) { break; }
        idx = BD_INDEX(tx, done);
        w = (volatile u32 *)done;
        w[0] = 0U;
        w[1] = (idx == (TXBD_CNT - 1U)) ? ((u32)XEMACPS_TXBUF_USED_MASK | (u32)XEMACPS_TXBUF_WRAP_MASK)
                                        : (u32)XEMACPS_TXBUF_USED_MASK;
        dsb();
        (void)XEmacPs_BdRingFree(tx, 1U, done);
    }

    if (XEmacPs_BdRingAlloc(tx, 1, &bd) != XST_SUCCESS) {            /* all TX BDs in flight */
        gem3_warn_tx_full(tx);
        return -1;
    }
    idx = BD_INDEX(tx, bd);

    memcpy(TxBuf[idx], frame, (size_t)len);
    Xil_DCacheFlushRange((INTPTR)TxBuf[idx], (u32)len);

    XEmacPs_BdSetAddressTx(bd, (UINTPTR)TxBuf[idx]);
    XEmacPs_BdSetLength(bd, (u32)len);
    XEmacPs_BdSetLast(bd);
    dsb();
    XEmacPs_BdClearTxUsed(bd);                       /* ownership -> MAC (must be last BD write) */
    dsb();

    (void)XEmacPs_BdRingToHw(tx, 1, bd);
    XEmacPs_Transmit(&emac);                         /* NWCTRL |= STARTTX */
    return len;
}

/* ---------------------------------------------------------------------- */
/* gem3_recv: return one received frame if any (non-blocking).             */
/* ---------------------------------------------------------------------- */
int gem3_recv(void *frame, int maxlen)
{
    XEmacPs_BdRing *rx = &XEmacPs_GetRxRing(&emac);
    XEmacPs_Bd *bd, *nbd;
    u32 idx;
    int len;

    if (XEmacPs_BdRingFromHwRx(rx, 1, &bd) == 0U) {                  /* nothing waiting */
        /* rev 5: "RX buffer not available" was flagged although we have drained the ring: the receiver may be
         * stalled (Xilinx treats BUFFNA as an error that needs a restart).  Start over on a clean ring, at most
         * once per tick, and count it (gem3_debug_dump prints the count).                                    */
        if ((Xil_In32((u32)emac.Config.BaseAddress + XEMACPS_RXSR_OFFSET) & XEMACPS_RXSR_BUFFNA_MASK) != 0U) {
            TickType_t now = xTaskGetTickCount();

            if (g_rx_restarts == 0U || now != g_rx_restart_tick) {
                g_rx_restart_tick = now;
                g_rx_restarts++;
                (void)gem3_rx_restart(0U);
            }
        }
        return 0;
    }

    idx = BD_INDEX(rx, bd);
    len = (int)XEmacPs_BdGetLength(bd);                              /* FCS already stripped */
    if (len > maxlen)          { len = maxlen; }
    if ((u32)len > GEM_BUF)    { len = (int)GEM_BUF; }

    Xil_DCacheInvalidateRange((INTPTR)RxBuf[idx], (u32)len);
    memcpy(frame, RxBuf[idx], (size_t)len);
    rxlog_note((const u8 *)frame, (u32)len);                         /* bring-up statistics (a few instructions) */

    /* recycle: Free -> Alloc (gives the same BD back) -> ToHw -> clear NEW + set buffer */
    (void)XEmacPs_BdRingFree(rx, 1, bd);
    if (XEmacPs_BdRingAlloc(rx, 1, &nbd) == XST_SUCCESS) {
        (void)XEmacPs_BdRingToHw(rx, 1, nbd);
        rx_arm(rx, nbd);
    }
    return len;
}

/* ---------------------------------------------------------------------- */
void gem3_close(void)
{
    XEmacPs_Stop(&emac);
}

/* ---------------------------------------------------------------------- */
/* Optional bring-up helper (declare "extern void gem3_debug_dump(void);") */
/* ---------------------------------------------------------------------- */
void gem3_debug_dump(void)
{
    u32 b = (u32)emac.Config.BaseAddress;
    u32 i, q, rxbase = GEM3_DMA_BASE + RXBD_OFFSET, n_new = 0U;
    u16 bmsr = 0, anlpar = 0, physts = 0, p15 = 0;
    volatile u32 *bd = (volatile u32 *)(UINTPTR)rxbase;

    (void)mdio_rd(g_phy, 1, &bmsr);
    (void)mdio_rd(g_phy, 1, &bmsr);
    (void)mdio_rd(g_phy, 5, &anlpar);
    (void)mdio_rd(g_phy, 0x11, &physts);
    (void)mdio_rd(g_phy, 0x15, &p15);
    gem3_stats_snapshot();                          /* every counter is read exactly once; all lines below print from this copy */

    xil_printf("[gem3] NWCTRL=%08x NWCFG=%08x NWSR=%08x TXSR=%08x RXSR=%08x\r\n",
               (unsigned)Xil_In32(b + XEMACPS_NWCTRL_OFFSET), (unsigned)Xil_In32(b + XEMACPS_NWCFG_OFFSET),
               (unsigned)Xil_In32(b + XEMACPS_NWSR_OFFSET),   (unsigned)Xil_In32(b + XEMACPS_TXSR_OFFSET),
               (unsigned)Xil_In32(b + XEMACPS_RXSR_OFFSET));
    xil_printf("[gem3] frames tx=%u rx=%u  rxFCSerr=%u rxResErr=%u   PHY bmsr=%04x anlpar=%04x\r\n",
               (unsigned)G3_S(G3_TXCNT), (unsigned)G3_S(G3_RXCNT),
               (unsigned)G3_S(G3_RXFCS), (unsigned)G3_S(G3_RXRESERR),
               (unsigned)bmsr, (unsigned)anlpar);

    /* ---- rev 5: everything below ---- */
    xil_printf("[gem3] counters/log cover the last %u ms (since gem3_init finished)   DMACR=%08x  PHY physts=%04x phy[0x15]=%04x\r\n",
               (unsigned)(xTaskGetTickCount() - g_t_stats0), (unsigned)Xil_In32(b + XEMACPS_DMACR_OFFSET),
               (unsigned)physts, (unsigned)p15);
    xil_printf("[gem3] TX frames=%u bcast=%u mcast=%u underrun=%u | collisions single=%u multi=%u excess=%u late=%u defer=%u crs=%u\r\n",
               (unsigned)G3_S(G3_TXCNT), (unsigned)G3_S(G3_TXBCCNT), (unsigned)G3_S(G3_TXMCCNT), (unsigned)G3_S(G3_TXURUNCNT),
               (unsigned)G3_S(G3_TXSNGLCOL), (unsigned)G3_S(G3_TXMULTICOL), (unsigned)G3_S(G3_TXEXCESSCOL),
               (unsigned)G3_S(G3_TXLATECOL), (unsigned)G3_S(G3_TXDEFER), (unsigned)G3_S(G3_TXCSENSE));
    xil_printf("[gem3] RX frames=%u bcast=%u mcast=%u pause=%u | size 64:%u 65-127:%u 128-255:%u 256-511:%u 512-1023:%u 1024-1518:%u 1519+:%u\r\n",
               (unsigned)G3_S(G3_RXCNT), (unsigned)G3_S(G3_RXBCCNT), (unsigned)G3_S(G3_RXMCCNT), (unsigned)G3_S(G3_RXPAUSECNT),
               (unsigned)G3_S(G3_RX64), (unsigned)G3_S(G3_RX65), (unsigned)G3_S(G3_RX128), (unsigned)G3_S(G3_RX256),
               (unsigned)G3_S(G3_RX512), (unsigned)G3_S(G3_RX1024), (unsigned)G3_S(G3_RX1519));
    xil_printf("[gem3] RX errors: undersize=%u oversize=%u jabber=%u fcs=%u length=%u symbol=%u align=%u resource=%u overrun=%u\r\n",
               (unsigned)G3_S(G3_RXUNDR), (unsigned)G3_S(G3_RXOVR), (unsigned)G3_S(G3_RXJAB), (unsigned)G3_S(G3_RXFCS),
               (unsigned)G3_S(G3_RXLEN), (unsigned)G3_S(G3_RXSYMB), (unsigned)G3_S(G3_RXALIGN),
               (unsigned)G3_S(G3_RXRESERR), (unsigned)G3_S(G3_RXOVRRUN));

    q = Xil_In32(b + XEMACPS_RXQBASE_OFFSET);
    if (q >= rxbase && q < (rxbase + (8U * RXBD_CNT))) {
        xil_printf("[gem3] RX queue pointer register = %08x (BD #%u of %u)   TX queue 1 = %08x\r\n", (unsigned)q,
                   (unsigned)((q - rxbase) / 8U), (unsigned)RXBD_CNT, (unsigned)Xil_In32(b + XEMACPS_TXQ1BASE_OFFSET));
    } else {
        xil_printf("[gem3] RX queue pointer register = %08x is OUTSIDE the RX ring %08x..%08x !!   TX queue 1 = %08x\r\n",
                   (unsigned)q, (unsigned)rxbase, (unsigned)(rxbase + (8U * RXBD_CNT) - 1U),
                   (unsigned)Xil_In32(b + XEMACPS_TXQ1BASE_OFFSET));
    }
    for (i = 0U; i < RXBD_CNT; i++) { if ((bd[2U * i] & XEMACPS_RXBUF_NEW_MASK) != 0U) { n_new++; } }
    xil_printf("[gem3] RX BDs holding a frame the software has not taken yet: %u of %u (all full = ring overflow)."
               " word0/word1 (word0: buffer | WRAP bit1 | NEW bit0; word1: length, bit14 SOF, bit15 EOF):\r\n",
               (unsigned)n_new, (unsigned)RXBD_CNT);
    for (i = 0U; i < RXBD_CNT; i += 4U) {
        xil_printf("[gem3]   BD%u %08x/%08x  BD%u %08x/%08x  BD%u %08x/%08x  BD%u %08x/%08x\r\n",
                   (unsigned)i,        (unsigned)bd[2U * i],        (unsigned)bd[(2U * i) + 1U],
                   (unsigned)(i + 1U), (unsigned)bd[2U * (i + 1U)], (unsigned)bd[(2U * (i + 1U)) + 1U],
                   (unsigned)(i + 2U), (unsigned)bd[2U * (i + 2U)], (unsigned)bd[(2U * (i + 2U)) + 1U],
                   (unsigned)(i + 3U), (unsigned)bd[2U * (i + 3U)], (unsigned)bd[(2U * (i + 3U)) + 1U]);
    }

    xil_printf("[gem3] automatic RX restarts after BUFFNA in this period: %u\r\n", (unsigned)g_rx_restarts);
    xil_printf("[gem3] software received %u frame(s) in this period: EtherCAT %u, other %u%s\r\n",
               (unsigned)g_rx_total, (unsigned)g_rx_ecat, (unsigned)g_rx_other,
               (g_rx_logged != 0U) ? "   first frames (dst 6B, src 6B, ethertype 2B, payload):" : "");
    rxlog_print();
}
