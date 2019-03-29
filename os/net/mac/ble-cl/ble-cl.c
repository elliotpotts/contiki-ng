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

enum { ADV_DATA_MAX_LEN = 31 };
enum { GAP_ADV_OVERHEAD = 2 };
enum { ADV_TYPE = 0xff };

extern const struct ble_hal_driver ble_hal;

void eu64_to_ble_addr(uint8_t *dst, const uint8_t *src) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = src[5];
  dst[4] = src[6];
  dst[5] = src[7];
}

static void send_packet(mac_callback_t sent, void *ptr) {
  uint16_t data_len = packetbuf_datalen();
  if (data_len > 240) {
    LOG_ERR("data too long to send\n");
    mac_call_sent_callback(sent, ptr, MAC_TX_ERR, 0);
    return;
  }

  if (packetbuf_holds_broadcast()) {
    ble_hal.adv_ext(NULL, packetbuf_dataptr(), data_len);
    mac_call_sent_callback(sent, ptr, MAC_TX_OK, 1);
  } else {
    uint8_t ble_addr[BLE_ADDR_SIZE];
    eu64_to_ble_addr(ble_addr, (uint8_t*) packetbuf_addr(PACKETBUF_ADDR_RECEIVER));
    ble_hal.adv_ext(ble_addr, packetbuf_dataptr(), data_len);
    mac_call_sent_callback(sent, ptr, MAC_TX_OK, 1);
  }
}

static void packet_input(void) {
  LOG_DBG("packet_input\n");
  NETSTACK_NETWORK.input(); // and say a prayer
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
    return 240;
    //return ADV_DATA_MAX_LEN - GAP_ADV_OVERHEAD - 6 - 6;
}

static void init() {
  int result = ble_hal.reset();
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
