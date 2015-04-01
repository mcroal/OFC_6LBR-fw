#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"

#include <string.h>

#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"
#include "dev/watchdog.h"
#include "net/rpl/rpl.h"
#include "ip64.h"
#include "ip64-addr.h"
#include "udp-socket.h"
//#include "simple-rpl.h"
#include "ip64-addrmap.h"
#include "bsp.h"

#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_BUF  ((struct uip_udp_hdr *)&uip_buf[uip_l2_l3_hdr_len])

#define MAX_PAYLOAD_LEN 10

static struct uip_udp_conn *server_conn;
static char buf[MAX_PAYLOAD_LEN];
static uint16_t len;
static uip_ipaddr_t ipaddr;

/*---------------------------------------------------------------------------*/
PROCESS(router_node_process, "Router node");
AUTOSTART_PROCESSES(&router_node_process);
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  memset(buf, 0, MAX_PAYLOAD_LEN);
  if(uip_newdata()) {
    BSP_LED_Toggle(LED2);
    len = uip_datalen();
    memcpy(buf, uip_appdata, len);
    if(buf[0] == 'A') {
      if(buf[1] == 'a') {
        /* @mcroal:check the ipv6 address */
        printf("give a ipv4 address for :");
        PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
        struct ip64_addrmap_entry *m;
        uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
        m = ip64_addrmap_lookup(&server_conn->ripaddr, NULL);
        if(m == NULL) {
          m = ip64_addrmap_create(&server_conn->ripaddr);          
          if(m == NULL) {               
            PRINTF("Could not create new map\n");  
            return;            
          } 
        }
        server_conn->rport = UIP_UDP_BUF->srcport;
        memcpy(&buf[2], m->ip4addr.u8, 4);
        uip_udp_packet_send(server_conn, buf, len);
        /* Restore server connection to allow data from any node */
        uip_create_unspecified(&server_conn->ripaddr);
        server_conn->rport = 0;
      }
    }
    memset(buf, 0, MAX_PAYLOAD_LEN);
  }
  return;
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("OFC-6LBR:\n");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused && (state == ADDR_TENTATIVE || state
        == ADDR_PREFERRED)) {
      PRINTF("  ");
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
      if(state == ADDR_TENTATIVE) {
        uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
void
create_dag()
{
  rpl_dag_t *dag;
  /* Assign a global address (RFC4193) */
  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

  print_local_addresses();

  dag = rpl_set_root(RPL_DEFAULT_INSTANCE,
                     &uip_ds6_get_global(ADDR_PREFERRED)->ipaddr);
  if(dag != NULL) {
    uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
    rpl_set_prefix(dag, &ipaddr, 64);
    PRINTF("Created a new RPL dag with ID: \n");
    PRINT6ADDR(&dag->dag_id);
    PRINTF("\n");
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(router_node_process, ev, data)
{
  PROCESS_BEGIN();

  /* Set us up as a RPL root node. */
//  simple_rpl_init_dag();
  create_dag();
  /* Initialize the IP64 module so we'll start translating packets */
  ip64_init();
 
  /* @mcroal:Listen this port and confige ipv4 address for a node. */
  server_conn = udp_new(NULL, UIP_HTONS(0), NULL);
  udp_bind(server_conn, UIP_HTONS(3000));
  PRINTF("Listen port: 3000, TTL=%u\n", server_conn->ttl);
  
  while(1) {
    PROCESS_WAIT_EVENT();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
