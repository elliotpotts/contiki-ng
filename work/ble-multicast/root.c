#include "contiki.h"
#include "contiki-net.h"
#include "net/ipv6/multicast/uip-mcast6.h"
#include "sys/energest.h"
#include "sys/log.h"

#include <string.h>
#include <inttypes.h>

//#define DEBUG DEBUG_PRINT
#include "net/ipv6/uip-debug.h"

static char buf[] = "____:Lorem ipsum dolor sit amet, consectetur adipiscing elit. Duis accumsan fringilla ultrices. Cras nibh magna, tincidunt in libero nec, dapibus vulputate nisi. Praesent dapibus ipsum eu eros scelerisque congue. Aliquam pharetra eros eu est tincidunt cursus. Pellentesque commodo quam vel neque posuere, ornare finibus arcu sollicitudin. Maecenas orci dui, commodo nec tellus eu, tincidunt consequat ligula. Maecenas laoreet leo ligula, ac consectetur justo accumsan ac. In elementum feugiat felis. Maecenas metus dui, vulputate eget condimentum a, ullamcorper ut metus. Phasellus venenatis sem arcu, non finibus velit vehicula quis. Cras ac porta enim. Maecenas lacus quam, venenatis ut diam et, tincidunt pulvinar nibh. Ut et orci sit amet nulla feugiat dapibus nec varius tortor. In consequat efficitur viverra. Nullam cursus lectus eu sem pulvinar, vitae hendrerit nibh vehicula. Donec viverra magna arcu, a elementum massa iaculis sed. Nullam sed bibendum nisi. Interdum et malesuada fames ac ante ipsum primis in faucibus. Proin sit amet eleifend nulla. Donec facilisis sem elit, et aliquet nulla vestibulum nec. Interdum et malesuada fames ac ante ipsum primis in faucibus. Vestibulum in volutpat.";

enum { echo_interval = 5 * CLOCK_SECOND };
enum { echo_count = 8 };
enum { max_peers = 2 };
static unsigned long curr_echo_id;
static struct {
  rtimer_clock_t sent;
  rtimer_clock_t arrivals[max_peers];
} echos[echo_count] = {0}; 

enum { send_port = 3001 };
enum { recv_port = 3002 }; 
enum { start_delay = 60 * CLOCK_SECOND };

static struct uip_udp_conn* send_conn;
static struct uip_udp_conn* recv_conn;

/*---------------------------------------------------------------------------*/
PROCESS(rpl_root_process, "RPL ROOT, Multicast Sender");
AUTOSTART_PROCESSES(&rpl_root_process);
/*---------------------------------------------------------------------------*/
static void send_handler() {
  unsigned long n_id = uip_htonl(curr_echo_id);
  unsigned echo_size = sizeof(n_id) + curr_echo_id * 50;
  memcpy(buf, &n_id, sizeof(n_id));

  //PRINTF("Send %u bytes to: ", echo_size);
  //PRINT6ADDR(&send_conn->ripaddr);
  //PRINTF(":%u\n", uip_ntohs(send_conn->rport));

  uip_udp_packet_send(send_conn, buf, echo_size);
  echos[curr_echo_id].sent = RTIMER_NOW();
}

static void prepare_mcast() {
  /* IPHC will use stateless multicast compression for this destination
   * (M=1, DAC=0), with 32 inline bits (1E 89 AB CD) */
  uip_ipaddr_t ipaddr;
  uip_ip6addr(&ipaddr, 0xFF1E,0,0,0,0,0,0x89,0xABCD);
  send_conn = udp_new(&ipaddr, UIP_HTONS(send_port), NULL);
}

static void recv_handler() {
  if (uip_newdata()) {
    unsigned long echo_id = uip_ntohl(*((unsigned long*)(uip_appdata)));
    //PRINTF("Got ack for id %lu\n", echo_id);
    for (int i = 0; i < max_peers; i++) {
      if (echos[echo_id].arrivals[i] == 0) {
	echos[echo_id].arrivals[i] = RTIMER_NOW();
	return;
      }
    }
    //PRINTF("(no room for peer arrival time)\n");
  }
  return;
}

static void prepare_ucast() {
  recv_conn = udp_new(NULL, UIP_HTONS(0), NULL);
  udp_bind(recv_conn, UIP_HTONS(recv_port));
}

PROCESS_THREAD(rpl_root_process, ev, data) {
  static struct etimer echo_timer;
  static unsigned long base_radio_listen = 0;
  static unsigned long base_radio_transmit = 0;

  PROCESS_BEGIN();

  printf("Multicast Engine: '%s'\n", UIP_MCAST6.name);

  NETSTACK_ROUTING.root_start();

  prepare_mcast();
  prepare_ucast();

  etimer_set(&echo_timer, start_delay);
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&echo_timer));
    break;
  }
  
  energest_flush();
  base_radio_listen = energest_type_time(ENERGEST_TYPE_LISTEN);
  base_radio_transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT);
  printf("  Energest base listen time: %4lus\n", base_radio_listen);
  printf("Energest base transmit time: %4lus\n", base_radio_transmit);
  
  while(1) {
    if(etimer_expired(&echo_timer)) {
      curr_echo_id++;
      if(curr_echo_id > echo_count) {
        etimer_stop(&echo_timer);
	energest_flush();
	unsigned long final_radio_listen = energest_type_time(ENERGEST_TYPE_LISTEN);
	unsigned long final_radio_transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT);
	printf("  Energest total listen time: %lu\n", final_radio_listen - base_radio_listen);
	printf("Energest total transmit time: %lu\n", final_radio_transmit - base_radio_transmit);
	printf("Latencies: \n");
	for(int i = 0; i < echo_count; i++) {
	  for(int j = 0; j < max_peers; j++) {
	    if (echos[i].arrivals[j] != 0) {
	      printf("%d, %lu\n", i, echos[i].arrivals[j] - echos[i].sent);
	    }
	  }
	}
      } else {
        send_handler();
        etimer_set(&echo_timer, echo_interval);
      }
    }
    if(ev == tcpip_event) {
      recv_handler();
    }
    PROCESS_YIELD();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
