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

enum { echo_interval = 1 * CLOCK_SECOND };
enum { echo_size_start = 100 };
enum { echo_size_max = 1200 };
enum { echo_size_step = 100 };
enum { echo_repeats = 10 };
enum { max_peers = 3 };
static unsigned curr_echo_size = echo_size_start;
static int curr_echo_remain = echo_repeats;
static rtimer_clock_t last_sent = {0};
static rtimer_clock_t arrivals[max_peers] = {0};

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
  last_sent = RTIMER_NOW();
  uip_udp_packet_send(send_conn, buf, curr_echo_size);
}

static void recv_handler() {
  if (uip_newdata() && *(char*)uip_appdata == '!') {
    rtimer_clock_t arrival = RTIMER_NOW();
    for (int i = 0; i < max_peers; i++) {
      if (arrivals[i] == 0) {
	arrivals[i] = arrival;
	return;
      }
    }
  }
}

static void prepare_mcast() {
  /* IPHC will use stateless multicast compression for this destination
   * (M=1, DAC=0), with 32 inline bits (1E 89 AB CD) */
  uip_ipaddr_t ipaddr;
  uip_ip6addr(&ipaddr, 0xFF1E,0,0,0,0,0,0x89,0xABCD);
  send_conn = udp_new(&ipaddr, UIP_HTONS(send_port), NULL);
}

static void prepare_ucast() {
  recv_conn = udp_new(NULL, UIP_HTONS(0), NULL);
  udp_bind(recv_conn, UIP_HTONS(recv_port));
}

PROCESS_THREAD(rpl_root_process, ev, data) {
  static struct etimer echo_timer;
  static unsigned long base_radio_listen;
  static unsigned long base_radio_transmit;
  static bool started = false;
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
  printf("  Energest base listen time: %lu\n", base_radio_listen);
  printf("Energest base transmit time: %lu\n", base_radio_transmit);
  
  while(1) {
    if(etimer_expired(&echo_timer)) {
      if (started) {
	for (int i = 0; i < max_peers; i++) {
	  if (arrivals[i] != 0) {
	    printf("%u, %lu\n", curr_echo_size, arrivals[i] - last_sent);
	    arrivals[i] = 0;
	  }
	}
	if (curr_echo_remain == 0) {
	  curr_echo_remain = echo_repeats;
	  curr_echo_size += echo_size_step;
	}
      }
      if (curr_echo_size > echo_size_max) {
	etimer_stop(&echo_timer);
	unsigned long final_radio_listen = energest_type_time(ENERGEST_TYPE_LISTEN);
      	unsigned long final_radio_transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT);
      	printf("  Energest total listen time: %lu\n", final_radio_listen - base_radio_listen);
	printf("Energest total transmit time: %lu\n", final_radio_transmit - base_radio_transmit);
      } else {
	send_handler();
	etimer_set(&echo_timer, echo_interval);
	curr_echo_remain--;
	started = true;
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
