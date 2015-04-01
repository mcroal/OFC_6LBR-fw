#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

#include <stdint.h>
#define CCIF
#define CLIF

/* These names are deprecated, use C99 names. */
typedef uint8_t     u8_t;
typedef uint16_t    u16_t;
typedef uint32_t    u32_t;
typedef int8_t      s8_t;
typedef int16_t     s16_t;
typedef int32_t     s32_t;

#define RTIMER_CLOCK_LT(a,b)     ((signed short)((a)-(b)) < 0)

typedef unsigned int clock_time_t;
typedef unsigned int rtimer_clock_t;
typedef unsigned int uip_stats_t;

#ifndef BV
#define BV(x) (1<<(x))
#endif

#define AUTOSTART_ENABLE                1

/*CPUÊ±ÖÓ*/
#define F_CPU                           72000000ul
#define CLOCK_CONF_SECOND               128
// One tick: 1ms
#define RTIMER_ARCH_SECOND              4608/*32768*/

/* RF Config */
#define IEEE802154_CONF_PANID           0x5449 

#ifndef RF_CHANNEL      
#define RF_CHANNEL               25
#endif /*RF_CHANNEL*/

#define RIMESTATS_CONF_ENABLED   1

/* Network Stack */
#define NETSTACK_CONF_MAC       csma_driver
#define NETSTACK_CONF_RDC       nullrdc_driver
#define NETSTACK_CONF_NETWORK   sicslowpan_driver
#define NETSTACK_CONF_FRAMER    framer_802154
#define NETSTACK_CONF_RADIO     at86rf231_radio_driver//nullradio_driver

#define NULLRDC_802154_AUTOACK               1

#ifndef NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE
#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE 8
#endif

#if UIP_CONF_IPV6
/* Addresses, Sizes and Interfaces */
/* 8-byte addresses here, 2 otherwise */
#define LINKADDR_CONF_SIZE                   8
#define UIP_CONF_LL_802154                   1
#define UIP_CONF_LLH_LEN                     0
#define UIP_CONF_NETIF_MAX_ADDRESSES         3

/* TCP, UDP, ICMP */
#define UIP_CONF_TCP                         1
#define UIP_CONF_UDP                         1
#define UIP_CONF_UDP_CHECKSUMS               1
#define UIP_CONF_ICMP6                       1

/* ND and Routing */
#define UIP_CONF_ROUTER                      1
#define UIP_CONF_IPV6_RPL                    1

#define UIP_CONF_IP_FORWARD                  0
#define RPL_CONF_STATS                       1
#define RPL_CONF_MAX_DAG_ENTRIES             1

#ifndef RPL_CONF_OF
#define RPL_CONF_OF rpl_mrhof
#endif

#define UIP_CONF_ND6_REACHABLE_TIME          600000
#define UIP_CONF_ND6_RETRANS_TIMER           10000

#ifndef UIP_CONF_DS6_NBR_NBU
#define UIP_CONF_DS6_NBR_NBU                 30 /* Handle n Neighbors */
#endif
#ifndef UIP_CONF_DS6_ROUTE_NBU
#define UIP_CONF_DS6_ROUTE_NBU               30 /* Handle n Routes */
#endif

/* uIP */
#define UIP_CONF_IPV6_QUEUE_PKT              0
#define UIP_CONF_IPV6_CHECKS                 1
#define UIP_CONF_IPV6_REASSEMBLY             0

/* 6lowpan */
#define SICSLOWPAN_CONF_COMPRESSION          SICSLOWPAN_COMPRESSION_HC06
#ifndef SICSLOWPAN_CONF_FRAG
#define SICSLOWPAN_CONF_FRAG                 0 /* About 2KB of CODE if 1 */
#endif
#define SICSLOWPAN_CONF_MAXAGE               8

/* Define our IPv6 prefixes/contexts here */
#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS    2
#define SICSLOWPAN_CONF_ADDR_CONTEXT_0 { \
  addr_contexts[0].prefix[0] = 0xaa; \
  addr_contexts[0].prefix[1] = 0xaa; \
}

#define MAC_CONF_CHANNEL_CHECK_RATE          8
#define QUEUEBUF_CONF_NUM                    16
#define PACKETBUF_CONF_ATTRS_INLINE          1

#else /* UIP_CONF_IPV6 */
/* Network setup for non-IPv6 (rime). */
#define UIP_CONF_IP_FORWARD                  1
#define UIP_CONF_BUFFER_SIZE               108
#define RIME_CONF_NO_POLITE_ANNOUCEMENTS     0
#define QUEUEBUF_CONF_NUM                    8
#endif /* UIP_CONF_IPV6 */

#include "project-conf.h"

#define CC_BYTE_ALIGNED __attribute__ ((packed, aligned(1)))

#undef  RAND_MAX
#define RAND_MAX            0x7fff

/* REST CONFIG*/
#define REST_MAX_CHUNK_SIZE 32

#endif /* __CONTIKI_CONF_H__CDBB4VIH3I__ */
