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

enum { gap_adv_overhead = 2 };
enum { gap_adv_type = 0xff };
enum { adv_chain_entries_max_num = 6 };

extern const struct ble_hal_driver ble_hal;

// A single advertising packet belonging to a chain of advertisements (a "link")
typedef struct adv_chain_entry_t adv_chain_entry_t;
struct adv_chain_entry_t {
  bool populated;
  bool is_head;
  ext_adv_pdu pdu;
  adv_chain_entry_t* next;
};

// A BLE Connectionless Link Layer
typedef struct {
  adv_chain_entry_t adv_links[adv_chain_entries_max_num];
} ble_cl_t;
static ble_cl_t g_ble_cl;

static void init_ble_cl(ble_cl_t* ble_cl) {
  memset(ble_cl, 0, sizeof(*ble_cl));
}

/* Start a new chain with the given pdu as it's first link */
static adv_chain_entry_t* ble_cl_chain_start(ble_cl_t* ble_cl, const ext_adv_pdu* pdu) {
  for (int i = 0; i < adv_chain_entries_max_num; i++) {
    adv_chain_entry_t* link = &ble_cl->adv_links[i];
    if (!link->populated) {
      link->populated = true;
      link->is_head = true;
      memcpy(&link->pdu, pdu, sizeof(*pdu));
      link->next = NULL;
      return link;
    }
  }
  return NULL;
}

/* Find the first link the given pdu belongs to, or NULL if none exists */
static adv_chain_entry_t* ble_cl_chain_get_head(ble_cl_t* ble_cl, const ext_adv_pdu* pdu) {
  for (int i = 0; i < adv_chain_entries_max_num; i++) {
    adv_chain_entry_t* link = &ble_cl->adv_links[i];
    if (link->populated && link->is_head && link->pdu.adi_present && pdu->adi_present && link->pdu.adi.set_id == pdu->adi.set_id) {
      return link;
    }
  }
  return NULL;
}

/* Append the given pdu to the end of the chain containing the given link */
static adv_chain_entry_t* ble_cl_chain_append(ble_cl_t* ble_cl, adv_chain_entry_t* head, const ext_adv_pdu* pdu) {
  adv_chain_entry_t* last = head;
  while (last->next != NULL) {
    // Don't append a duplicated
    if (last->pdu.adi.set_id == pdu->adi.set_id &&
	last->pdu.adi.data_id == pdu->adi.data_id) {
      LOG_DBG("We've seen %u.%u before; dropping", pdu->adi.set_id, pdu->adi.data_id);
      return NULL;
    }
    last = last->next;
  }
  // find free slot to put link
  for (int i = 0; i < adv_chain_entries_max_num; i++) {
    adv_chain_entry_t* link = &ble_cl->adv_links[i];
    if (!link->populated) {
      link->populated = true;
      link->is_head = false;
      memcpy(&link->pdu, pdu, sizeof(*pdu));
      last->next = link;
      return link;
    }
  }
  return NULL;
};

void ble_addr_to_eui64(uint8_t *dst, const uint8_t *src);
static uint8_t* append_pdu_to_packetbuf(uint8_t* out, const ext_adv_pdu* pdu) {
  if (pdu->adv_a_present) {
    linkaddr_t sender;
    ble_addr_to_eui64(sender.u8, pdu->adv_a);
    packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &sender);
  }
  if (pdu->tgt_a_present) {
    packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, &linkaddr_node_addr);
  }
  const unsigned adv_data_len = pdu->adv_data_len;
  if (packetbuf_remaininglen() < adv_data_len) {
    LOG_ERR("Not enough room in packetbuf!\n");
  }
  memcpy(out, pdu->adv_data, adv_data_len);
  packetbuf_set_datalen(packetbuf_datalen() + adv_data_len);
  return out + adv_data_len;
}

// Finish the chain using the final pdu.
// If head == NULL, this is both the first and last pdu
static void ble_cl_chain_finish(ble_cl_t* ble_cl, adv_chain_entry_t* head, const ext_adv_pdu* last_pdu) {
  // Start a new packet
  packetbuf_clear();
  // Where to put next chunk of data
  uint8_t *dataptr = packetbuf_dataptr();
  // Process packets earlier on in the chain
  for (adv_chain_entry_t* adv = head; adv != NULL; adv = adv->next) {
    dataptr = append_pdu_to_packetbuf(dataptr, &adv->pdu);
    adv = adv->next;
  }
  // Process last packet
  dataptr = append_pdu_to_packetbuf(dataptr, last_pdu);

  // Remove entries belonging to this chain
  for (adv_chain_entry_t* adv = head; adv != NULL; adv = adv->next) {
    adv->populated = false;
  }

  NETSTACK_NETWORK.input(); // and PRAY
}

static void eui64_to_ble_addr(uint8_t *dst, const uint8_t *src) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = src[5];
  dst[4] = src[6];
  dst[5] = src[7];
}

static int max_payload() {
  return BLE5_ADV_DATA_MAX_TOTAL_SIZE;
}

static unsigned random_set_id() {
  return random_rand() % 12;
}

static unsigned random_data_id() {
  return random_rand() % 4096;
}

static void send_packet(mac_callback_t sent, void *ptr) {
  uint16_t data_len = packetbuf_datalen();
  if (data_len > max_payload()) {
    LOG_ERR("Packetbuf data length %uB exceeds maximum %uB; cannot send\n", data_len, max_payload());
    mac_call_sent_callback(sent, ptr, MAC_TX_ERR, 0);
  }

  uint8_t tgt_ble_addr[BLE_ADDR_SIZE];
  uint8_t *maybe_tgt_ble_addr = NULL;
  if (!packetbuf_holds_broadcast()) {
    eui64_to_ble_addr(tgt_ble_addr, (uint8_t*) packetbuf_addr(PACKETBUF_ADDR_RECEIVER));
    LOG_DBG("AKA ^%.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n",
    	    tgt_ble_addr[0], tgt_ble_addr[1], tgt_ble_addr[2], tgt_ble_addr[3], tgt_ble_addr[4], tgt_ble_addr[5]);
    maybe_tgt_ble_addr = tgt_ble_addr;
  } else {
    LOG_DBG("Sending broadcast\n");
  }
  
  uint8_t* data = packetbuf_dataptr();
  uint8_t* data_end = data + data_len;
  adi_t adi = (adi_t) {
    .set_id = random_set_id(),
    .data_id = random_data_id()
  };
  aux_ptr_t aux_ptr = (aux_ptr_t) {
    .channel_ix = 20,
    .clock_accuracy = 0,
    .offset_units = 0,
    .aux_offset = 36,
    .aux_phy = 0
  };
  // pause scanning while we send
  ble_hal.set_scan_enable(0, 0);
  LOG_DBG("---> Starting chain of %u octets\n", data_len);
  ble5_ext_adv_result_t result = ble_hal.adv_ext(maybe_tgt_ble_addr, &adi, &aux_ptr, data, data_end);
  data += result.bytes_sent;
  while (data < data_end) {
    adi.data_id = random_data_id();
    result = ble_hal.aux_adv(&result, &adi, &aux_ptr, data, data_end);
    LOG_DBG("---> Continuing chain, %u octets left\n", data_end - data);
    data += result.bytes_sent;
  }
  LOG_DBG("---> Chain finished\n");

  mac_call_sent_callback(sent, ptr, MAC_TX_OK, 1);
  ble_hal.set_scan_enable(1, 0);
}

static void rmemcpy(void *restrict dst, const void *restrict src, size_t count) {
  unsigned char *dst_char = dst;
  const unsigned char *src_char = src;
  for (size_t i = 0; i < count; ++i) {
    dst_char[count - 1 - i] = src_char[i];
  }
}

//TODO terrible disgusting ugly hack - please replace!
extern uint8_t* g_payload;
extern uint8_t* g_payload_end;
static void packet_input(void) {
  /* LOG_DBG("receiving bluetooth adv...\n"); */
  uint8_t* payload = g_payload;
  uint8_t* payload_end = g_payload_end;
  ble_cl_t* ble_cl = &g_ble_cl;
  
  ext_adv_pdu pdu = { 0 };

  pdu.adv_mode = *g_payload & 0b11000000;
  const uint8_t ext_header_len = *payload++ & 0b00111111;
  uint8_t* const ext_header_end = payload + ext_header_len;
  
  if (ext_header_len > 0) {
    const uint8_t ext_header_flags = *payload++;
    
    if (ext_header_flags & ble5_adv_ext_hdr_flag_adv_a) {
      pdu.adv_a_present = true;
      rmemcpy(&pdu.adv_a, payload, BLE_ADDR_SIZE);
      payload += BLE_ADDR_SIZE;
      /* LOG_DBG("pdu.adv_a is present: %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n", */
      /* 	      pdu.adv_a[0], pdu.adv_a[1], pdu.adv_a[2], pdu.adv_a[3], pdu.adv_a[4], pdu.adv_a[5]); */
    }

    if (ext_header_flags & ble5_adv_ext_hdr_flag_tgt_a) {
      pdu.tgt_a_present = true;
      rmemcpy(&pdu.tgt_a, payload, BLE_ADDR_SIZE);
      payload += BLE_ADDR_SIZE;
      /* LOG_DBG("pdu.tgt_a is present: %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n", */
      /* 	      pdu.tgt_a[0], pdu.tgt_a[1], pdu.tgt_a[2], pdu.tgt_a[3], pdu.tgt_a[4], pdu.tgt_a[5]); */
    }

    if (ext_header_flags & ble5_adv_ext_hdr_flag_adi) {
      pdu.adi_present = true;
      memcpy(&pdu.adi, payload, sizeof(pdu.adi));
      payload += sizeof(pdu.adi);
      /* LOG_DBG("pdu.adi is present: { .data_id = %u, .set_id = %u } \n", pdu.adi.data_id, pdu.adi.set_id); */
    }

    if (ext_header_flags & ble5_adv_ext_hdr_flag_aux_ptr) {
      pdu.aux_ptr_present = true;
      memcpy(&pdu.aux_ptr, payload, sizeof(pdu.aux_ptr));
      payload += sizeof(pdu.aux_ptr);
      /* LOG_DBG("pdu.aux_ptr is present: {\n\t.channel_ix = %u,\n\t.clock_accuracy = %u,\n\t.offset_units = %u,\n\t.aux_offset = %u,\n\t.aux_phy = %u,\n}\n", */
      /* 	      pdu.aux_ptr.channel_ix, pdu.aux_ptr.clock_accuracy, pdu.aux_ptr.offset_units, pdu.aux_ptr.aux_offset, pdu.aux_ptr.aux_phy); */
    }

    if (ext_header_flags & ble5_adv_ext_hdr_flag_sync_info) {
      pdu.sync_info_present = true;
      memcpy(&pdu.sync_info, payload, sizeof(pdu.sync_info));
      payload += sizeof(pdu.sync_info);
      /* LOG_DBG("pdu.sync_info is present\n"); */
    }

    if (ext_header_flags & ble5_adv_ext_hdr_flag_tx_power) {
      pdu.tx_power_present = true;
      memcpy(&pdu.tx_power, payload, sizeof(pdu.tx_power));
      payload += sizeof(pdu.tx_power);
      /* LOG_DBG("pdu.tx_power is present\n"); */
    }
  }

  uint8_t *acad_end = ext_header_end;
  uint8_t *acad_begin = payload;
  pdu.acad_len = acad_end - acad_begin;
  memcpy(&pdu.acad, acad_begin, pdu.acad_len);

  uint8_t *adv_data_end = payload_end;
  uint8_t *adv_data_begin = ext_header_end;
  pdu.adv_data_len = adv_data_end - adv_data_begin;
  memcpy(&pdu.adv_data, adv_data_begin, pdu.adv_data_len);

  /* LOG_DBG("%u bytes of advData present\n", adv_data_end - adv_data_begin); */

  adv_chain_entry_t* head = ble_cl_chain_get_head(ble_cl, &pdu);
  unsigned conditions = ((adv_data_begin != adv_data_end) << 2)
                      | (pdu.adi_present << 1)
                      | (pdu.aux_ptr_present << 0);
  switch (conditions) {
  case 0b000:
  case 0b001:
  case 0b010:
  case 0b011:
  case 0b101:
    LOG_DBG("dropping %u%u%u\n", conditions & 0b100, conditions & 0b010, conditions & 0b001);
    return;
  case 0b111: {
    adv_chain_entry_t* head = ble_cl_chain_get_head(ble_cl, &pdu);
    if (head) {
      LOG_DBG("<--- Continueing chain\n");
      if (!ble_cl_chain_append(ble_cl, head, &pdu)) {
	LOG_ERR("Failed to append chain entry!\n");
      }
    } else {
      LOG_DBG("<--- Starting chain\n");
      if (!ble_cl_chain_start(ble_cl, &pdu)) {
	LOG_ERR("Failed to start chain!\n");
      }
    }
    return;
  }
  case 0b100:
  case 0b110: {
    LOG_DBG("<--- Finishing chain\n");
    ble_cl_chain_finish(ble_cl, head, &pdu);
    return;
  }
  }
  LOG_DBG("unaccounted for!\n");
}

static int on() {
  return 0;
}

static int off() {
  return 0;
}

static void init() {
  init_ble_cl(&g_ble_cl);
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
