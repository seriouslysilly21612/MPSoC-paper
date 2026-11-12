/*
 * Raw Layer-2 NIC layer for SOEM on the Zynq UltraScale+ GEM3
 * (KV260 on-board RJ45).
 *
 * These four functions are the ONLY hardware touchpoints of the whole SOEM
 * port. nicdrv.c calls them; everything else in the port is OS-/HW-agnostic.
 *
 * Implement with the Xilinx bare-metal Ethernet driver (xemacps) in Phase 2,
 * after GEM3 is handed over from Linux/IgH to the R5. EtherCAT runs at
 * 100 Mbit/s full-duplex, raw frames (EtherType 0x88A4), promiscuous, no
 * IP/TCP stack.
 */
#ifndef GEM3_NIC_H
#define GEM3_NIC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Initialise GEM3 + PHY for raw EtherCAT traffic.
 *  @param mac  6-byte source MAC to program (EtherCAT ignores it, but the MAC
 *              block still needs one).
 *  @return 0 on success, non-zero on failure. */
int  gem3_init(const uint8_t *mac);

/** Send one raw Ethernet frame (non-blocking).
 *  @return number of bytes queued, or <=0 on error. */
int  gem3_send(const void *frame, int len);

/** Receive one raw Ethernet frame if available (non-blocking).
 *  @return number of bytes copied into frame, or 0 if nothing waiting. */
int  gem3_recv(void *frame, int maxlen);

/** Stop/close the NIC. */
void gem3_close(void);

#ifdef __cplusplus
}
#endif

#endif
