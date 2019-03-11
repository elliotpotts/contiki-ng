#include "net/mac/ble-cl/ble-cl.h"
#include "net/netstack.h"
#include "net/ipv6/uip.h"
#include "net/ipv6/tcpip.h"
#include "net/packetbuf.h"
#include "net/netstack.h"

#include "dev/ble-hal.h"

#include "sys/log.h"
#define LOG_MODULE "BLE-CL"
#define LOG_LEVEL LOG_LEVEL_MAC

extern const struct ble_hal_driver ble_hal;

static void send_packet(mac_callback_t sent, void *ptr) {
  LOG_DBG("send_packet\n");
}

static void packet_input(void) {
  LOG_DBG("packet_input\n");
}

static int on() {
  LOG_DBG("on\n");
  ble_hal.set_scan_enable(1, 0);
  return 0;
}

static int off() {
  LOG_DBG("off\n");
  return 0;
}

static int max_payload() {
  LOG_DBG("max_payload\n");
  return 60;
}

static void init() {
  int result = ble_hal.reset();
  LOG_DBG("init: %s\n", result == BLE_RESULT_OK ? "succeeded" : "failed");
  NETSTACK_MAC.on();
}

const struct mac_driver ble_cl_driver = {
  "ble-cl",
  init,
  send_packet,
  packet_input,
  on,
  off,
  max_payload,
};

