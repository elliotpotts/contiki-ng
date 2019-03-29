#include "contiki.h"
#include "net/ipv6/uip-icmp6.h"

#include "sys/log.h"
#define LOG_MODULE "MAIN"
#define LOG_LEVEL LOG_LEVEL_MAIN

static struct uip_icmp6_echo_reply_notification icmp_notification;
void icmp_reply_handler(uip_ipaddr_t *source, uint8_t ttl, uint8_t *data, uint16_t datalen) {
  LOG_INFO("Received ping.\n");
}

PROCESS(main_proc, "Main process");
AUTOSTART_PROCESSES(&main_proc);

PROCESS_THREAD(main_proc, ev, data)
{
  PROCESS_BEGIN();
  uip_icmp6_echo_reply_callback_add(&icmp_notification, icmp_reply_handler);
  PROCESS_END();
}
