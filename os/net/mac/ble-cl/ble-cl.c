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
enum { adv_links_max_num = 6 };

extern const struct ble_hal_driver ble_hal;

// A single advertising packet belonging to a chain of advertisements (a "link")
typedef struct adv_link_t adv_link_t;
struct adv_link_t {
  bool populated;
  bool is_head;
  ext_adv_pdu pdu;
  adv_link_t* next_in_chain;
};

// A BLE Connectionless Link Layer
typedef struct {
  adv_link_t adv_links[adv_links_max_num];
} ble_cl_t;
static ble_cl_t g_ble_cl;

static void init_ble_cl(ble_cl_t* ble_cl) {
  memset(ble_cl, 0, sizeof(*ble_cl));
}

/* Start a new chain with the given pdu as it's first link */
static adv_link_t* ble_cl_chain_start(ble_cl_t* ble_cl, const ext_adv_pdu* pdu) {
  for (int i = 0; i < adv_links_max_num; i++) {
    adv_link_t* link = &ble_cl->adv_links[i];
    if (!link->populated) {
      link->populated = true;
      link->is_head = true;
      memcpy(&link->pdu, pdu, sizeof(*pdu));
      link->next_in_chain = NULL;
      return link;
    }
  }
  return NULL;
}

/* Find the first link the given pdu belongs to, or NULL if none exists */
static adv_link_t* ble_cl_chain_get_head(ble_cl_t* ble_cl, const ext_adv_pdu* pdu) {
  for (int i = 0; i < adv_links_max_num; i++) {
    adv_link_t* link = &ble_cl->adv_links[i];
    if (link->populated && link->is_head && link->pdu.adi.set_id == pdu->adi.set_id) {
      return link;
    }
  }
  return NULL;
}

/* Append the given pdu to the end of the chain containing the given link */
static adv_link_t* ble_cl_chain_append(ble_cl_t* ble_cl, adv_link_t* head, const ext_adv_pdu* pdu) {
  adv_link_t* last = head;
  while (last->next_in_chain != NULL) {
    // Don't append a duplicated
    if (last->pdu.adi.set_id == pdu->adi.set_id &&
	last->pdu.adi.data_id == pdu->adi.data_id) {
      LOG_DBG("We've seen %u.%u before; dropping", pdu->adi.set_id, pdu->adi.data_id);
      return NULL;
    }
    last = last->next_in_chain;
  }
  // find free slot to put link
  for (int i = 0; i < adv_links_max_num; i++) {
    adv_link_t* link = &ble_cl->adv_links[i];
    if (!link->populated) {
      link->populated = true;
      link->is_head = false;
      memcpy(&link->pdu, pdu, sizeof(*pdu));
      last->next_in_chain = link;
      return link;
    }
  }
  return NULL;
};

static adv_link_t* ble_cl_chain_finish(ble_cl_t* ble_cl, adv_link_t* head, const ext_adv_pdu* last_pdu) {
  //TODO: actually finish
  LOG_DBG("FINISHING WITH DATA:\n");
  while (head != NULL) {
    LOG_DBG("    %s\n", head->pdu.adv_data);
    head = head->next_in_chain;
  }
  LOG_DBG("    %s\n", last_pdu->adv_data);
  return NULL;
}

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

enum { lorem_ipsum_len = 600 };
uint8_t lorem_ipsum[lorem_ipsum_len] = "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Morbi id mollis felis, at tristique lectus. Nulla facilisi. Ut fermentum odio a sapien vehicula, nec tristique leo luctus. Pellentesque iaculis viverra sem a facilisis. Fusce at dolor nulla. Donec sagittis, lorem a condimentum auctor, nunc lectus tempor nunc, in commodo ligula orci in neque. Donec vel egestas odio, vitae tincidunt felis. Suspendisse non turpis nisl. In nec magna id est auctor suscipit a et metus. Aliquam porta ut ex non consequat. Pellentesque vel lacinia libero. Duis nec ex nisl. Etiam pretium odio dolor, vel cras amet.";

static unsigned random_set_id() {
  return random_rand() % 12;
}

static unsigned random_data_id() {
  return random_rand() % 4096;
}

static void send_packet(mac_callback_t sent, void *ptr) {
  uint8_t* data = lorem_ipsum;
  uint8_t* data_end = lorem_ipsum + lorem_ipsum_len;
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
  ble5_ext_adv_result_t result = ble_hal.adv_ext(NULL, &adi, &aux_ptr, data, data_end);
  data += result.bytes_sent;
  while (data < data_end) {
    adi.data_id = random_data_id();
    result = ble_hal.aux_adv(&result, &adi, &aux_ptr, data, data_end);
    data += result.bytes_sent;
  }
  
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
      LOG_DBG("pdu.adv_a is present: %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n",
      	      pdu.adv_a[0], pdu.adv_a[1], pdu.adv_a[2], pdu.adv_a[3], pdu.adv_a[4], pdu.adv_a[5]);
    }

    if (ext_header_flags & ble5_adv_ext_hdr_flag_tgt_a) {
      pdu.tgt_a_present = true;
      rmemcpy(&pdu.tgt_a, payload, BLE_ADDR_SIZE);
      payload += BLE_ADDR_SIZE;
      LOG_DBG("pdu.tgt_a is present: %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n",
      	      pdu.tgt_a[0], pdu.tgt_a[1], pdu.tgt_a[2], pdu.tgt_a[3], pdu.tgt_a[4], pdu.tgt_a[5]);
    }

    if (ext_header_flags & ble5_adv_ext_hdr_flag_adi) {
      pdu.adi_present = true;
      memcpy(&pdu.adi, payload, sizeof(pdu.adi));
      payload += sizeof(pdu.adi);
      LOG_DBG("pdu.adi is present: { .data_id = %u, .set_id = %u } \n", pdu.adi.data_id, pdu.adi.set_id);
    }

    if (ext_header_flags & ble5_adv_ext_hdr_flag_aux_ptr) {
      pdu.aux_ptr_present = true;
      memcpy(&pdu.aux_ptr, payload, sizeof(pdu.aux_ptr));
      payload += sizeof(pdu.aux_ptr);
      LOG_DBG("pdu.aux_ptr is present: {\n\t.channel_ix = %u,\n\t.clock_accuracy = %u,\n\t.offset_units = %u,\n\t.aux_offset = %u,\n\t.aux_phy = %u,\n}\n",
      	      pdu.aux_ptr.channel_ix, pdu.aux_ptr.clock_accuracy, pdu.aux_ptr.offset_units, pdu.aux_ptr.aux_offset, pdu.aux_ptr.aux_phy);
    }

    if (ext_header_flags & ble5_adv_ext_hdr_flag_sync_info) {
      pdu.sync_info_present = true;
      memcpy(&pdu.sync_info, payload, sizeof(pdu.sync_info));
      payload += sizeof(pdu.sync_info);
      LOG_DBG("pdu.sync_info is present\n");
    }

    if (ext_header_flags & ble5_adv_ext_hdr_flag_tx_power) {
      pdu.tx_power_present = true;
      memcpy(&pdu.tx_power, payload, sizeof(pdu.tx_power));
      payload += sizeof(pdu.tx_power);
      LOG_DBG("pdu.tx_power is present\n");
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
  pdu.adv_data[pdu.adv_data_len] = '\0';

  /* Is there an adi with which to associate/start a chain?
   * All extended advertisements carrying Adv Data must have an ADI.
   */
  if (adv_data_begin == adv_data_end && !pdu.adi_present) {
    LOG_DBG("packet without adi or adv data is not meaningful\n");
    return;
  } else  if (!pdu.adi_present) {
    LOG_DBG("RECV ACTION: TODO: handle non-chained data; dropping\n");
    return;
  } else if (adv_data_begin == adv_data_end) {
    LOG_DBG("RECV ACTION (or lack thereof): no adv_data; chain will be started on first aux packet\n");
    return;
  } else {
    // Are we tracking this chain yet?
    adv_link_t* chain_head = ble_cl_chain_get_head(ble_cl, &pdu);
    if (chain_head) {
      // Yes, so continue or finish it
      if (pdu.aux_ptr_present) {
	// Continue chain 
	if (ble_cl_chain_append(ble_cl, chain_head, &pdu)) {
	  LOG_DBG("RECV ACTION: Appending did %u to sid %u\n", pdu.adi.data_id, pdu.adi.set_id);
	} else {
	  LOG_ERR("Could not append did %u to sid %u\n", pdu.adi.data_id, pdu.adi.set_id);
	}
	return;
      } else {
	// Finishing
	if (ble_cl_chain_finish(ble_cl, chain_head, &pdu)) {
	  LOG_DBG("RECV ACTION: Finished chain with sid %u\n", pdu.adi.set_id);
	} else {
	  LOG_ERR("Couldn't finish chain with sid %u\n", pdu.adi.set_id);
	}
	return;
      }
    } else {
      // No, will there be further aux packets?
      if (pdu.aux_ptr_present) {
	// yes, start a new chain
	if (ble_cl_chain_start(ble_cl, &pdu)) {
	  LOG_DBG("RECV ACTION: Started chain sid %u\n", pdu.adi.set_id);
	} else {
	  LOG_ERR("Couldn't start new chain!\n");
	}
	return;
      } else {
	// no, instead of starting then finishing, just report packet directly
	LOG_DBG("RECV_ACTION: TODO: send single adv_data to upper layer\n%s\n", pdu.adv_data);
	return;
      }
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
