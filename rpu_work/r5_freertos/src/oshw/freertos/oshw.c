/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 *
 * SOEM oshw port for FreeRTOS / Xilinx Cortex-R5 (KV260 RPU).
 */
#include <stddef.h>
#include "oshw.h"

/*
 * EtherCAT payload is little-endian, but the Ethernet header (dst/src MAC,
 * ethertype) is big-endian like any Ethernet frame. Cortex-R5 runs
 * little-endian, so host<->network for 16-bit values is just a byte swap.
 */
uint16 oshw_htons(uint16 host)
{
   return (uint16)(((host & 0x00FFu) << 8) | ((host & 0xFF00u) >> 8));
}

uint16 oshw_ntohs(uint16 network)
{
   return (uint16)(((network & 0x00FFu) << 8) | ((network & 0xFF00u) >> 8));
}

/*
 * We drive a single fixed NIC (GEM3 -> on-board RJ45), so there is no
 * adapter list to enumerate. SOEM only needs these for the "list adapters"
 * helper, which we don't use.
 */
ec_adaptert * oshw_find_adapters(void)
{
   return NULL;
}

void oshw_free_adapters(ec_adaptert * adapter)
{
   (void)adapter;
}
