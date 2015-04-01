/*
 * Copyright (c) 2013, CETIC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/**
 * \file
 *         Main 6LBR process and initialisation
 * \author
 *         6LBR Team <6lbr@cetic.be>
 */

#define LOG6LBR_MODULE "6LBR"
/* @mcroal:contiki inc */
#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-nd6.h"
#include "net/rpl/rpl.h"
#include "net/netstack.h"
#include "net/rpl/rpl.h"
#include "net/ip/uip-debug.h"
/* @mcroal:compiler inc */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
/* @mcroal:cetic inc */
#include "cetic-6lbr.h"
#include "platform-init.h"
#include "packet-filter.h"
#include "eth-drv.h"
#include "nvm-config.h"
#include "rio.h"
#include "sicslow-ethernet.h"

#define DEBUG 0
#if DEBUG
  #include <stdio.h>
  #include "bsp.h"
  #define PRINTF(...) printf(__VA_ARGS__)
  #define PRINTETHADDR(addr) printf(" %02x:%02x:%02x:%02x:%02x:%02x ",
                                    (*addr)[0], (*addr)[1], (*addr)[2], (*addr)[3], (*addr)[4], (*addr)[5])
#else
  #define PRINTF(...)
  #define PRINTETHADDR(addr)
#endif

//Initialisation flags
int ethernet_ready = 0;
int eth_mac_addr_ready = 0;

//WSN
uip_lladdr_t  wsn_mac_addr;
uip_ip6addr_t wsn_net_prefix;
uint8_t       wsn_net_prefix_len;
uip_ipaddr_t  wsn_ip_addr;
uip_ipaddr_t  wsn_ip_local_addr;
rpl_dag_t     *cetic_dag;

// Eth
ethaddr_t     eth_mac_addr;
uip_lladdr_t  eth_mac64_addr;
uip_ipaddr_t  eth_ip_addr;
uip_ipaddr_t  eth_net_prefix;
uip_ipaddr_t  eth_ip_local_addr;
uip_ipaddr_t  eth_dft_router;

//Misc
unsigned long cetic_6lbr_startup;

enum cetic_6lbr_restart_type_t cetic_6lbr_restart_type;
/*---------------------------------------------------------------------------*/
PROCESS_NAME(webserver_nogui_process);
PROCESS_NAME(udp_server_process);
PROCESS(cetic_6lbr_process, "CETIC Bridge process");
AUTOSTART_PROCESSES(&cetic_6lbr_process);
/*---------------------------------------------------------------------------*/
void
cetic_6lbr_set_prefix(uip_ipaddr_t * prefix, unsigned len,
                      uip_ipaddr_t * ipaddr)
{
#if CETIC_6LBR_SMARTBRIDGE
  int new_prefix = cetic_dag != NULL && !uip_ipaddr_prefixcmp(&cetic_dag->prefix_info.prefix, prefix, len);
  if((nvm_data.mode & CETIC_MODE_WAIT_RA_MASK) == 0) {
    PRINTF("Ignoring RA\n");
    return;
  }
  PRINTF("CETIC_BRIDGE : set_prefix\n");

  if(cetic_dag != NULL) {
    rpl_set_prefix(cetic_dag, prefix, len);
    uip_ipaddr_copy(&wsn_net_prefix, prefix);
    wsn_net_prefix_len = len;
    if(new_prefix) {
      PRINTF("Setting DAG prefix : ");
      PRINT6ADDR(prefix);
      PRINTF("\n");
      rpl_repair_root(RPL_DEFAULT_INSTANCE);
    }
  }
#endif
}
/*---------------------------------------------------------------------------*/
void
cetic_6lbr_init(void)
{
  uip_ds6_addr_t *local = uip_ds6_get_link_local(-1);

  uip_ipaddr_copy(&wsn_ip_local_addr, &local->ipaddr);

  PRINTF("Tentative local IPv6 address ");
  PRINT6ADDR(&wsn_ip_local_addr);
  PRINTF("\n");
#if CETIC_6LBR_SMARTBRIDGE
  if((nvm_data.mode & CETIC_MODE_WAIT_RA_MASK) == 0)    //Manual configuration
  {
    memcpy(wsn_net_prefix.u8, &nvm_data.wsn_net_prefix,
           sizeof(nvm_data.wsn_net_prefix));
    if((nvm_data.mode & CETIC_MODE_WSN_AUTOCONF) != 0)  //Address auto configuration
    {
      uip_ipaddr_copy(&wsn_ip_addr, &wsn_net_prefix);
      uip_ds6_set_addr_iid(&wsn_ip_addr, &uip_lladdr);
      uip_ds6_addr_add(&wsn_ip_addr, 0, ADDR_AUTOCONF);
    } else {
      memcpy(wsn_ip_addr.u8, &nvm_data.wsn_ip_addr,
             sizeof(nvm_data.wsn_ip_addr));
      uip_ds6_addr_add(&wsn_ip_addr, 0, ADDR_MANUAL);
    }
    PRINTF("Tentative global IPv6 address ");
    PRINT6ADDR(&wsn_ip_addr);
    PRINTF("\n");
    /* @mcroal:Confige a default router ,add it to list */
    memcpy(eth_dft_router.u8, &nvm_data.eth_dft_router,
           sizeof(nvm_data.eth_dft_router));
    if ( !uip_is_addr_unspecified(&eth_dft_router) ) {
      uip_ds6_defrt_add(&eth_dft_router, 0);
    }
  }                                                     //End manual configuration
#endif

#if CETIC_6LBR_ROUTER
  //WSN network configuration
  memcpy(wsn_net_prefix.u8, &nvm_data.wsn_net_prefix,
         sizeof(nvm_data.wsn_net_prefix));
  wsn_net_prefix_len = nvm_data.wsn_net_prefix_len;
  if((nvm_data.mode & CETIC_MODE_WSN_AUTOCONF) != 0)    //Address auto configuration
  {
    uip_ipaddr_copy(&wsn_ip_addr, &wsn_net_prefix);
    uip_ds6_set_addr_iid(&wsn_ip_addr, &uip_lladdr);
    uip_ds6_addr_add(&wsn_ip_addr, 0, ADDR_AUTOCONF);
  } else {
    memcpy(wsn_ip_addr.u8, &nvm_data.wsn_ip_addr,
           sizeof(nvm_data.wsn_ip_addr));
    uip_ds6_addr_add(&wsn_ip_addr, 0, ADDR_MANUAL);
  }
  PRINTF("Tentative global IPv6 address (WSN) ");
  PRINT6ADDR(&wsn_ip_addr);
  PRINTF("\n");

  //Ethernet network configuration
  memcpy(eth_net_prefix.u8, &nvm_data.eth_net_prefix,
         sizeof(nvm_data.eth_net_prefix));
  memcpy(eth_dft_router.u8, &nvm_data.eth_dft_router,
         sizeof(nvm_data.eth_dft_router));
  if ( !uip_is_addr_unspecified(&eth_dft_router) ) {
    /* @mcroal:Confige a default router ,add it to list */
    uip_ds6_defrt_add(&eth_dft_router, 0);
  }

  eth_mac64_addr.addr[0] = eth_mac_addr[0];
  eth_mac64_addr.addr[1] = eth_mac_addr[1];
  eth_mac64_addr.addr[2] = eth_mac_addr[2];
  eth_mac64_addr.addr[3] = CETIC_6LBR_ETH_EXT_A;
  eth_mac64_addr.addr[4] = CETIC_6LBR_ETH_EXT_B;
  eth_mac64_addr.addr[5] = eth_mac_addr[3];
  eth_mac64_addr.addr[6] = eth_mac_addr[4];
  eth_mac64_addr.addr[7] = eth_mac_addr[5];

  if((nvm_data.mode & CETIC_MODE_ETH_AUTOCONF) != 0)    //Address auto configuration
  {
    uip_ipaddr_copy(&eth_ip_addr, &eth_net_prefix);
    uip_ds6_set_addr_iid(&eth_ip_addr, &eth_mac64_addr);
    uip_ds6_addr_add(&eth_ip_addr, 0, ADDR_AUTOCONF);
  } else {
    memcpy(eth_ip_addr.u8, &nvm_data.eth_ip_addr,
           sizeof(nvm_data.eth_ip_addr));
    uip_ds6_addr_add(&eth_ip_addr, 0, ADDR_MANUAL);
  }
  PRINTF("Tentative global IPv6 address (ETH) ");
  PRINT6ADDR(&eth_ip_addr);
  PRINTF("\n");

  //Ugly hack : in order to set WSN local address as the default address
  //We must add it afterwards as uip_ds6_addr_add allocates addr from the end of the list
  uip_ds6_addr_rm(local);
  /* @mcroal:Confige a local ipv6 address (ETH) */
  uip_create_linklocal_prefix(&eth_ip_local_addr);
  uip_ds6_set_addr_iid(&eth_ip_local_addr, &eth_mac64_addr);
  uip_ds6_addr_add(&eth_ip_local_addr, 0, ADDR_AUTOCONF);

  /* @mcroal:Confige a local ipv6 address (WSN) */
  uip_ds6_addr_add(&wsn_ip_local_addr, 0, ADDR_AUTOCONF);

  //Prefix and RA configuration
#if UIP_CONF_IPV6_RPL
  /* @mcroal:RA PIO option is set or not */
  uint8_t publish = (nvm_data.ra_prefix_flags & CETIC_6LBR_MODE_SEND_PIO) != 0;
  /* @mcroal:We confige a ETH prefix */
  uip_ds6_prefix_add(&eth_net_prefix, nvm_data.eth_net_prefix_len, publish,
                     nvm_data.ra_prefix_flags,
                     nvm_data.ra_prefix_vtime, nvm_data.ra_prefix_ptime);
#else
  uip_ds6_prefix_add(&eth_net_prefix, nvm_data.eth_net_prefix_len, 0, 0, 0, 0);
  uint8_t publish = (nvm_data.ra_prefix_flags & CETIC_6LBR_MODE_SEND_PIO) != 0;
  uip_ds6_prefix_add(&wsn_net_prefix, nvm_data.wsn_net_prefix_len, publish,
		             nvm_data.ra_prefix_flags,
		             nvm_data.ra_prefix_vtime, nvm_data.ra_prefix_ptime);
#endif

#if UIP_CONF_IPV6_RPL
  /* @mcroal:RA RIO option is set or not */
  if ((nvm_data.ra_rio_flags & CETIC_6LBR_MODE_SEND_RIO) != 0 ) {
    /* @mcroal:6lbr to adding route information   */
    uip_ds6_route_info_add(&wsn_net_prefix, nvm_data.wsn_net_prefix_len, nvm_data.ra_rio_flags, nvm_data.ra_rio_lifetime);
  }
#endif
#endif

#if UIP_CONF_IPV6_RPL && CETIC_6LBR_DODAG_ROOT
  //DODAGID = link-local address used !
  cetic_dag = rpl_set_root(nvm_data.rpl_instance_id, &wsn_ip_local_addr);
#if CETIC_6LBR_SMARTBRIDGE
  if((nvm_data.mode & CETIC_MODE_WAIT_RA_MASK) == 0) {
    rpl_set_prefix(cetic_dag, &wsn_net_prefix, nvm_data.wsn_net_prefix_len);
  }
#else
  rpl_set_prefix(cetic_dag, &wsn_net_prefix, nvm_data.wsn_net_prefix_len);
#endif
  PRINTF("Configured as DODAG Root\n");
#endif

#if CETIC_6LBR_TRANSPARENTBRIDGE
  #if CETIC_6LBR_LEARN_RPL_MAC
    PRINTF("Starting as RPL Relay\n");
  #else
    PRINTF("Starting as Full TRANSPARENT-BRIDGE\n");
  #endif
#elif CETIC_6LBR_SMARTBRIDGE
  PRINTF("Starting as SMART-BRIDGE\n");
#elif CETIC_6LBR_ROUTER
  #if UIP_CONF_IPV6_RPL
    PRINTF("Starting as RPL ROUTER\n");
  #else
    PRINTF("Starting as NDP ROUTER\n");
  #endif
#elif CETIC_6LBR_6LR 
  PRINTF("Starting as 6LR\n");
#else
  PRINTF("Starting in UNKNOWN mode\n");
#endif
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cetic_6lbr_process, ev, data)
{
  PROCESS_BEGIN();

  cetic_6lbr_startup = clock_seconds();


  PRINTF("Starting 6LBR version %d ( %s \n)", CETIC_6LBR_VERSION, CONTIKI_VERSION_STRING);

  load_nvm_config();
  /*@mcroal:initialize eth mac */
  linkaddr_copy((linkaddr_t *) & wsn_mac_addr, &linkaddr_node_addr);
  mac_createEthernetAddr((uint8_t *) eth_mac_addr, &wsn_mac_addr);
  PRINTF("Eth MAC address : ");
  PRINTETHADDR(&eth_mac_addr);
  PRINTF("\r\n");
  eth_mac_addr_ready = 1;

  process_start(&eth_drv_process, NULL);

  while(!ethernet_ready) {
    PROCESS_PAUSE();
  }

  //clean up any early packet
  memset(uip_buf, 0, UIP_CONF_BUFFER_SIZE);
  uip_len = 0;
  
  process_start(&tcpip_process, NULL);

  PROCESS_PAUSE();
  
  packet_filter_init();
  cetic_6lbr_init();

#if WEBSERVER
  process_start(&webserver_nogui_process, NULL);
#endif
#if UDPSERVER
  process_start(&udp_server_process, NULL);
#endif

  PRINTF("CETIC 6LBR Started\n");

  PROCESS_END();
}
