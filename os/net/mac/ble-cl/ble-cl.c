#include "net/mac/ble-cl/ble-cl.h"
#include "net/netstack.h"
#include "net/ipv6/uip.h"
#include "net/ipv6/tcpip.h"
#include "net/packetbuf.h"
#include "net/netstack.h"

#include "os/lib/random.h"
#include "dev/ble-hal.h"

#include <math.h>

#include "sys/log.h"
#define LOG_MODULE "BLE-CL"
#define LOG_LEVEL LOG_LEVEL_MAC

enum { GAP_ADV_OVERHEAD = 2 };
enum { ADV_TYPE = 0xff };

extern const struct ble_hal_driver ble_hal;

/* static void eui64_to_ble_addr(uint8_t *dst, const uint8_t *src) { */
/*   dst[0] = src[0]; */
/*   dst[1] = src[1]; */
/*   dst[2] = src[2]; */
/*   dst[3] = src[5]; */
/*   dst[4] = src[6]; */
/*   dst[5] = src[7]; */
/* } */

static int max_payload() {
  return BLE5_ADV_DATA_MAX_TOTAL_SIZE;
}

static void send_packet(mac_callback_t sent, void *ptr) {
  /* uint16_t data_left = packetbuf_datalen(); */
  /* if (data_left > max_payload()) { */
  /*   LOG_ERR("Packetbuf data length %u exceeds maximum %u; cannot send\n", data_left, max_payload()); */
  /*   mac_call_sent_callback(sent, ptr, MAC_TX_ERR, 0); */
  /* } */

  /* unsigned set_id = random_set_id(); */

  /* if (data_left < FIRST_FRAGMENT_MAX_SIZE) { */
  /*   // No need to fragment */
  /* } else { */
  /*   // Plan fragments */
  /* } */

  /*if (packetbuf_holds_broadcast()) {
    ble_hal.adv_ext(NULL, packetbuf_dataptr(), data_len);
    mac_call_sent_callback(sent, ptr, MAC_TX_OK, 1);
  } else {
    uint8_t ble_addr[BLE_ADDR_SIZE];
    eui64_to_ble_addr(ble_addr, (uint8_t*) packetbuf_addr(PACKETBUF_ADDR_RECEIVER));
    ble_hal.adv_ext(ble_addr, packetbuf_dataptr(), data_len);
    mac_call_sent_callback(sent, ptr, MAC_TX_OK, 1);
    }*/
}

static void packet_input(void) {
  LOG_DBG("packet_input\n");
  NETSTACK_NETWORK.input(); // and say a prayer
}

static int on() {
  return 0;
}

static int off() {
  return 0;
}

static void init() {
  ble_hal.reset();
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
