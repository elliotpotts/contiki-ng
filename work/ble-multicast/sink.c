#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ipv6/multicast/uip-mcast6.h"

#include <string.h>

#define DEBUG DEBUG_PRINT
#include "net/ipv6/uip-debug.h"

uip_ipaddr_t send_ip = {0};
enum { recv_port = 3001 };
enum { send_port = 3002 };

static struct uip_udp_conn *recv_conn = NULL;
static struct uip_udp_conn *send_conn = NULL;
static uint16_t count = 0;

PROCESS(mcast_sink_process, "Multicast Sink");
AUTOSTART_PROCESSES(&mcast_sink_process);

static void recv_handler(void) {
  if(uip_newdata()) {
    unsigned long net_echo_id = *((unsigned long*) uip_appdata);
    unsigned long host_echo_id = uip_ntohl(net_echo_id);
    //       (unsigned long)uip_ntohl((unsigned long) *((uint32_t *)(uip_appdata))),
    count++;
    PRINTF("In: [0x%08lx], TTL %u, total %x\n", host_echo_id, UIP_IP_BUF->ttl, count);
    // send ack
    uip_udp_packet_send(send_conn, &net_echo_id, sizeof(net_echo_id));
  }
  return;
}

static uip_ds6_maddr_t * join_mcast_group(void) {
  uip_ipaddr_t addr;
  uip_ds6_maddr_t *rv;
  const uip_ipaddr_t *default_prefix = uip_ds6_default_prefix();

  /* First, set our v6 global */
  uip_ip6addr_copy(&addr, default_prefix);
  uip_ds6_set_addr_iid(&addr, &uip_lladdr);
  uip_ds6_addr_add(&addr, 0, ADDR_AUTOCONF);

  /*
   * IPHC will use stateless multicast compression for this destination
   * (M=1, DAC=0), with 32 inline bits (1E 89 AB CD)
   */
  uip_ip6addr(&addr, 0xFF1E,0,0,0,0,0,0x89,0xABCD);
  rv = uip_ds6_maddr_add(&addr);

  if(rv) {
    PRINTF("Joined multicast group ");
    PRINT6ADDR(&uip_ds6_maddr_lookup(&addr)->ipaddr);
    PRINTF("\n");
  }
  return rv;
}

PROCESS_THREAD(mcast_sink_process, ev, data) {
  PROCESS_BEGIN();

  PRINTF("Multicast Engine: '%s'\n", UIP_MCAST6.name);

  if(join_mcast_group() == NULL) {
    PRINTF("Failed to join multicast group\n");
    PROCESS_EXIT();
  }

  recv_conn = udp_new(NULL, UIP_HTONS(0), NULL);
  udp_bind(recv_conn, UIP_HTONS(recv_port));

  if(uiplib_ipaddrconv("fe80::566c:eff:fe9b:6353", &send_ip) == 0) {
    PRINTF("bad ipaddr\n");
  }
  send_conn = udp_new(&send_ip, UIP_HTONS(send_port), NULL);
  //udp_bind(send_conn, UIP_HTONS(send_port));

  PRINTF("Listening: ");
  PRINT6ADDR(&recv_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
        UIP_HTONS(recv_conn->lport), UIP_HTONS(recv_conn->rport));

  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      recv_handler();
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
