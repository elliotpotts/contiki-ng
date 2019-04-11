#ifdef PROJECT_CONF_PATH
#include PROJECT_CONF_PATH
#endif /* PROJECT_CONF_PATH */

#if !RADIO_CONF_BLE5
#error Connectionless BLE requies BLE5 support
#endif

#include <string.h>

#include "os/sys/rtimer.h"
#include "os/sys/log.h"
#include "os/lib/random.h"
#include "os/dev/ble-hal.h"
#include "os/net/netstack.h"
#include "os/net/packetbuf.h"

#include "dev/oscillators.h"
#include "lpm.h"
#include "ble-addr.h"

#include "rf_data_entry.h"
#include "rf_ble_cmd.h"
#include "rf-core/rf-core.h"
#include "rf-ble-cmd.h"
#include "rf-core/ble-hal/rf-ble-cmd.h"
#include "rf_patches/rf_patch_cpe_bt5.h"

#include "ble5.h"

#define LOG_MODULE "BLE-HAL"
#define LOG_LEVEL LOG_LEVEL_RADIO

static uint8_t request(void) {
  if(rf_core_is_accessible()) {
    return LPM_MODE_SLEEP;
  }
  return LPM_MODE_MAX_SUPPORTED;
}
LPM_MODULE(cc26xx_ble_lpm_module, request, NULL, NULL, LPM_DOMAIN_NONE);

/* SCANNER data structures */
#define SCAN_RX_BUFFERS_DATA_LEN       260
#define SCAN_RX_BUFFERS_NUM            8
#define SCAN_PREPROCESSING_TIME_TICKS  65

typedef struct {
  rfc_dataEntry_t entry;
  uint8_t data[SCAN_RX_BUFFERS_DATA_LEN];
} __attribute__ ((packed)) scan_data_entry;

typedef struct adv_link_t adv_link_t;
struct adv_link_t {
  bool populated;
  ext_adv_pdu pdu;
  adv_link_t* next_in_chain;
};

typedef struct {
  /* state */
  bool scanning;
  rtimer_clock_t scan_start;
  struct rtimer timer;
  /* data */
  dataQueue_t rx_queue;
  scan_data_entry rx_buffers[SCAN_RX_BUFFERS_NUM];
  scan_data_entry *rx_queue_current;
  adv_link_t adv_links[SCAN_RX_BUFFERS_NUM];
  /* radio interface */
  uint8_t ble_addr[BLE_ADDR_SIZE];
  rfc_bleWhiteListEntry_t whitelist[5];
  rfc_ble5ScannerPar_t params;
  rfc_ble5ScanInitOutput_t output;
  rfc_CMD_BLE5_SCANNER_t cmd;
} ble_scanner_t;
static void init_scanner(ble_scanner_t* scanner);
ble_result_t set_scan_enable(unsigned short enable, unsigned short filter_duplicates);
static ble_scanner_t g_scanner;

/* Common */
#define TIME_UNIT_30US     33333
#define TIME_UNIT_300US    3333
#define TIME_UNIT_MS       1000          /* 1000 times per second */
#define TIME_UNIT_0_625_MS 1600          /* 1600 times per second */
#define TIME_UNIT_1_25_MS  800           /* 800 times per second */
#define TIME_UNIT_10_MS    100           /* 100 times per second */
#define TIME_UNIT_RF_CORE  4000000       /* runs at 4 MHz */
#define TIME_UNIT_RTIMER   RTIMER_SECOND
rtimer_clock_t ticks_from_unit(uint32_t value, uint32_t unit) {
  double temp = (((double)value) / unit) * RTIMER_SECOND;
  return (rtimer_clock_t)temp;
}
uint32_t ticks_to_unit(rtimer_clock_t value, uint32_t unit) {
  double temp = (((double)value) / RTIMER_SECOND) * unit;
  return (uint32_t)temp;
}

ble_result_t on(void) {
  oscillators_request_hf_xosc();
  if(!rf_core_is_accessible()) {
    /* boot the rf core */
    if(rf_core_power_up() != RF_CORE_CMD_OK) {
      LOG_ERR("rf_core_boot: rf_core_power_up() failed\n");
      rf_core_power_down();
      return RF_CORE_CMD_ERROR;
    }

    /*  Apply Bluetooth 5 patch */
    rf_patch_cpe_bt5();

    /* Start radio timer */
    if(rf_core_start_rat() != RF_CORE_CMD_OK) {
      LOG_ERR("rf_core_boot: rf_core_start_rat() failed\n");
      rf_core_power_down();
      return RF_CORE_CMD_ERROR;
    }

    /* Setup interrupts */
    rf_core_setup_interrupts();
    oscillators_switch_to_hf_xosc();

    if(rf_ble_cmd_setup_ble_mode() != RF_BLE_CMD_OK) {
      LOG_ERR("could not setup rf-core to BLE mode\n");
      return BLE_RESULT_ERROR;
    }
  }
  return BLE_RESULT_OK;
}

void off(void) {
  rf_core_power_down();
  oscillators_switch_to_hf_rc();
}

static ble_result_t reset(void) {
  lpm_register_module(&cc26xx_ble_lpm_module);
  rf_core_set_modesel();
  init_scanner(&g_scanner);
  if(on() != BLE_RESULT_OK) {
    return BLE_RESULT_ERROR;
  }
  off();
  return BLE_RESULT_OK;
}

static ble_result_t read_bd_addr(uint8_t *addr) {
  ble_addr_cpy_to(addr);
  return BLE_RESULT_OK;
}

void rmemcpy(void *restrict dst, const void *restrict src, size_t count) {
  unsigned char *dst_char = dst;
  const unsigned char *src_char = src;
  for (size_t i = 0; i < count; ++i) {
    dst_char[count - 1 - i] = src_char[i];
  }
}

void write_ext_adv_hdr(uint8_t *out,
		       const uint8_t *flags,
		       const uint8_t *adv_addr,
		       const uint8_t *tgt_addr,
		       const adi_t *adi,
		       const aux_ptr_t *aux_ptr,
		       const sync_info_t *sync_info,
		       const uint8_t *tx_power) {
  if (flags) {
    *out++ = *flags;
  }
  if (adv_addr) {
    rmemcpy(out, adv_addr, BLE_ADDR_SIZE);
    out += BLE_ADDR_SIZE;
  }
  if (tgt_addr) {
    rmemcpy(out, tgt_addr, BLE_ADDR_SIZE);
    out += BLE_ADDR_SIZE;
  }
  if (adi) {
    memcpy(out, adi, sizeof(*adi));
    out += sizeof(*adi);
  }
  if (aux_ptr) {
    memcpy(out, aux_ptr, sizeof(*aux_ptr));
    out += sizeof(*aux_ptr);
  }
  if (sync_info) {
    memcpy(out, sync_info, sizeof(*sync_info));
    out += sizeof(*sync_info);
  }
  if (tx_power) {
    memcpy(out, tx_power, sizeof(*tx_power));
    out += sizeof(*tx_power);
  }
}

uint8_t hello_world[] = "Hello, World!";

/* Advertising */
ble_result_t adv_ext(const uint8_t *tgt_bd_addr, const uint8_t *adv_data, unsigned adv_data_len) {
  // We can't send and scan at the same time, so disable scanning
  set_scan_enable(0, 0);

  /* In this function we create two commands like this:
   *
   * Timeline:
   * _____________________________________________________
   *   |                          |
   *   ^                          |
   *   "prev start"           "prev start" + 60'000 RAT ticks
   *
   *   +------------------+      +------------------------+
   *   | CMD_BLE5_ADV_EXT |      | CMD_BLE5_ADV_AUX       |
   *   | TRIGGER_NOW      |      | TRIGGER_REL_PREVSTART  |
   *   |          next op +----->|                next op +------> NULL
   *   +------------------+      | advData: "hello world" |
   *                             +------------------------+
   */

  /* Common */
  rfc_bleAdvOutput_t output = { 0 }; // clear all counters

  // aux_target_time is the number of RF Core ticks after which the Aux packet will be sent
  // RF Core timer has 4MHz resolution
  // 15ms in RF Core timer ticks (60'000 * 1/4'000'000 seconds = 15ms)
  long long unsigned aux_target_time = 60000; 

  // ADI is essentially two sequence numbers:
  // set_id is to distinguish between two separate chains of advertising data
  // set_id is 4 bits, so 16 possible values
  // data_id is to distinguish between packets within the same chain
  // data_id 12 bits, so 4096 possible values
  adi_t adi = {
    .set_id = random_rand() % 16,
    .data_id = random_rand() % 4096
  };
  LOG_DBG("Starting %u.%u\n", adi.set_id, adi.data_id);

  /* PACKET 2 */
  uint8_t aux_adv_hdr[64];
  write_ext_adv_hdr(aux_adv_hdr, NULL, NULL, NULL, &adi, NULL, NULL, NULL);
  rfc_ble5ExtAdvEntry_t aux_adv_entry = {
    .extHdrInfo = { .length = 1 + 2 }, // 1 = sizeof(flags), 2 = sizeof(adi)
    .extHdrFlags = ble5_adv_ext_hdr_flag_adi,
    .pExtHeader = aux_adv_hdr,
    .advDataLen = 13,
    .pAdvData = hello_world
  };
  rfc_ble5AdvAuxPar_t aux_adv_params = { .pAdvPkt = (uint8_t*) &aux_adv_entry };
  rfc_CMD_BLE5_ADV_AUX_t aux_adv_cmd = {
    .commandNo = CMD_BLE5_ADV_AUX,
    .startTime = aux_target_time - 1180,
    .startTrigger = { .triggerType = TRIG_REL_PREVSTART },
    .condition = { .rule = COND_NEVER },
    .channel = 20,
    .pParams = &aux_adv_params,
    .pOutput = &output
  };
  
  /* PACKET 1 */
  uint8_t adv_addr[BLE_ADDR_SIZE];
  ble_addr_cpy_to(adv_addr);

  aux_ptr_t aux_ptr = {
    .channel_ix = aux_adv_cmd.channel,
    /*.offset_units = <filled by radio cpu> */
    /*.aux_offset = <filled by radio cpu> */
  };
  uint8_t adv_ext_hdr[64];
  write_ext_adv_hdr(adv_ext_hdr, NULL, adv_addr, NULL, &adi, &aux_ptr, NULL, NULL);
  rfc_ble5ExtAdvEntry_t adv_ext_entry = {
    .extHdrInfo = { .length = 1 + 6 + 2 + 3 }, // 1 = sizeof(flags), 2 = sizeof(adv_addr), 2 = sizeof(adi), 3 = sizeof(aux_ptr)
    .extHdrFlags = ble5_adv_ext_hdr_flag_adv_a | ble5_adv_ext_hdr_flag_adi | ble5_adv_ext_hdr_flag_aux_ptr,
    .pExtHeader = adv_ext_hdr,
  };
  rfc_ble5AdvExtPar_t adv_ext_params = {
    .pAdvPkt = (uint8_t*) &adv_ext_entry,
    .auxPtrTargetType = TRIG_REL_START,
    .auxPtrTargetTime = aux_target_time
  };
  rfc_CMD_BLE5_ADV_EXT_t adv_ext_cmd = {
    .commandNo = CMD_BLE5_ADV_EXT,
    .startTrigger = { .triggerType = TRIG_NOW },
    .pNextOp = (rfc_radioOp_t*) &aux_adv_cmd,
    .condition = { .rule = COND_ALWAYS },
    .channel = 37,
    .pParams = &adv_ext_params,
    .pOutput = &output
  };

  // Submit command
  //if(on() != BLE_RESULT_OK) {
  //  LOG_DBG("could not enable rf core prior to ADV_EXT \n");
  //  return BLE_RESULT_ERROR;
  //}
  long unsigned before_send_adv_ext = RTIMER_NOW();
  rf_ble_cmd_send((uint8_t*) &adv_ext_cmd);
  long unsigned after_send_adv_ext = RTIMER_NOW();
  rf_ble_cmd_wait((uint8_t*) &adv_ext_cmd);
  long unsigned after_wait_adv_ext = RTIMER_NOW();
  rf_ble_cmd_wait((uint8_t*) &aux_adv_cmd);
  long unsigned after_wait_aux_adv = RTIMER_NOW();
  LOG_DBG("started sending adv_ext at %lu\n", before_send_adv_ext);
  LOG_DBG("finished sending & started waiting for adv_ext at %lu\n", after_send_adv_ext);
  LOG_DBG("finished waiting for adv_ext & started waiting for aux_adv at %lu\n", after_wait_adv_ext);
  LOG_DBG("finished waiting for aux_adv at %lu\n", after_wait_aux_adv);

  set_scan_enable(1,0);
  return BLE_RESULT_OK;
}

/* Scanning */
static void init_scanner(ble_scanner_t* scanner) {
  memset(scanner, 0, sizeof(ble_scanner_t));
  for (int i = 0; i < SCAN_RX_BUFFERS_NUM; i++) {
    rfc_dataEntry_t* e = &scanner->rx_buffers[i].entry;
    e->pNextEntry = (uint8_t*) &scanner->rx_buffers[(i + 1) % SCAN_RX_BUFFERS_NUM];
    e->status = DATA_ENTRY_PENDING;
    e->length = SCAN_RX_BUFFERS_DATA_LEN;
  }
  scanner->rx_queue.pCurrEntry = (uint8_t*) &scanner->rx_buffers[0].entry;
  scanner->rx_queue.pLastEntry = NULL;
  scanner->rx_queue_current = &scanner->rx_buffers[0];
  /*  TODO: figure out why this whitelist lets nothing through */
  scanner->whitelist[0] = (rfc_bleWhiteListEntry_t) {
    .size = 5,
    .conf = { .bEnable = 1 },
    .addressHi = (0xCC << 24) | (0x78 << 16) | (0xAB << 8) | 0x77,
    .address =   (0xA7 << 8)  | 0x82
  };
  scanner->whitelist[1] = (rfc_bleWhiteListEntry_t) {
    .conf = { .bEnable = 1 },
    .addressHi = (0xCC << 24) | (0x78 << 16) | (0xAB << 8) | 0x71,
    .address =   (0x40 << 8)  | 0x07
  };
  scanner->whitelist[2] = (rfc_bleWhiteListEntry_t) {
    .conf = { .bEnable = 1 },
    .addressHi = (0x54 << 24) | (0x6C << 16) | (0x0E << 8) | 0x83,
    .address =   (0x3F << 8)  | 0xE6
  };
  scanner->whitelist[3] = (rfc_bleWhiteListEntry_t) {
    .conf = { .bEnable = 1 },
    .addressHi = (0x54 << 24) | (0x6C << 16) | (0x0E << 8) | 0x9B,
    .address =   (0x63 << 8)  | 0x53
  };
  scanner->whitelist[4] = (rfc_bleWhiteListEntry_t) {
    .conf = { .bEnable = 1 },
    .addressHi = (0xB0 << 24) | (0x91 << 16) | (0x22 << 8) | 0x69,
    .address =   (0xFC << 8)  | 0x5A
  };
  read_bd_addr(scanner->ble_addr);
  scanner->params = (rfc_ble5ScannerPar_t) {
    .pRxQ = &scanner->rx_queue,
    .rxConfig = {
      .bAutoFlushIgnored = 1,
      .bAutoFlushCrcErr = 1,
      .bAutoFlushEmpty = 1,
      .bIncludeLenByte = 1,
      //.bAppendRssi = 1 TODO: elliot: maybe use rssi for TSCH-over-BLE5
    },
    .scanConfig = {
      .scanFilterPolicy = 1,
      .bStrictLenFilter = 0
    },
    .pDeviceAddress = (uint16_t*) scanner->ble_addr, // will be checked against incoming tgt addr
    .pWhiteList = scanner->whitelist,
    .maxWaitTimeForAuxCh = UINT16_MAX, // never follow aux pointer
    .timeoutTrigger = { .triggerType = TRIG_NEVER },
    .endTrigger = { .triggerType = TRIG_NEVER }
  };
  scanner->cmd = (rfc_CMD_BLE5_SCANNER_t) {
    .commandNo = CMD_BLE5_SCANNER,
    .startTrigger = { .triggerType = TRIG_NOW },
    .condition = { .rule = COND_NEVER }, // never execute 'next op'
    .channel = 37,
    .pParams = &scanner->params,
    .pOutput = &scanner->output
  };
}

static adv_link_t* scanner_make_link(ble_scanner_t* scanner) {
  for (int i = 0; i < SCAN_RX_BUFFERS_NUM; i++) {
    if (!scanner->adv_links[i].populated) {
      scanner->adv_links[i].populated = true;
      memset(&scanner->adv_links[i].pdu, 0, sizeof(scanner->adv_links[i].pdu));
      scanner->adv_links[i].next_in_chain = NULL;
      return &scanner->adv_links[i];
    }
  }
  return NULL;
}

static void free_adv_link(adv_link_t* link) {
  link->populated = false;
}

static void adv_chain_append(adv_link_t* chain, adv_link_t* x) {
  while (chain->next_in_chain) {
    chain = chain->next_in_chain;
  }
  chain->next_in_chain = x;
}

static adv_link_t* scanner_get_chain_head(ble_scanner_t* scanner, const ext_adv_pdu* pdu) {
  for (int i = 0; i < SCAN_RX_BUFFERS_NUM; i++) {
    if (scanner->adv_links[i].populated &&
	scanner->adv_links[i].pdu.adi.set_id == pdu->adi.set_id &&
	scanner->adv_links[i].pdu.type == ble_adv_ext_ind) {
      return &scanner->adv_links[i];
    }
  }
  return NULL;
}

static void scanner_free_chain(ble_scanner_t* scanner, adv_link_t* link) {
  for (adv_link_t* l = scanner_get_chain_head(scanner, &link->pdu); l->next_in_chain; l = l->next_in_chain) {
    free_adv_link(l);
  }
}

static bool scanner_remembers(ble_scanner_t* scanner, const ext_adv_pdu* pdu) {
  for (int i = 0; i < SCAN_RX_BUFFERS_NUM; i++) {
    adv_link_t* l = &scanner->adv_links[i];
    if (l->populated && l->pdu.adi_present &&
	l->pdu.adi.set_id == pdu->adi.set_id &&
	l->pdu.adi.data_id == pdu->adi.data_id) {
      return true;
    }
  }
  return false;
}

static void scanner_recv_ext_adv(ble_scanner_t* scanner, ble_adv_pdu_type_t type, uint8_t *payload, uint8_t *payload_end) { 
  ext_adv_pdu pdu = { .type = type };

  pdu.adv_mode = *payload & 0b11000000;
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

  uint8_t *adv_data_end = payload_end;
  uint8_t *adv_data_begin = ext_header_end;

  // have we already stored this pdu?
  if (scanner_remembers(scanner, &pdu)) {
    LOG_DBG("already seen; dropping\n");
    return; // yes, we don't need to see it gain.
  };
  // is there an adi with which to associate a chain?
  if (!pdu.adi_present) {
    // no indication of which chain it belongs to, drop it
    LOG_DBG("no adi with which to continue chain; dropping\n");
    return;
  }
  // is there an adv a?
  if (!pdu.adv_a_present) {
    LOG_DBG("no advertising address present; dropping\n");
    return; // we only care about packets we know the sender of
  }
  
  // are we starting a new chain or continuing a previous one?
  if (pdu.type == 7) {
    // starting a new chain
    if (!pdu.aux_ptr_present) {
      LOG_DBG("can't start a new chain without an aux ptr; dropping\n");
      return; // either no aux ptr therefore no data or no adi therefore no means of identification
    }
    if (adv_data_begin != adv_data_end) {
      LOG_DBG("advData not allowed on ADV_EXT; dropping\n");
      return;
    }
    // don't drop - start tracking this chain
    LOG_DBG("tracking new chain %u\n", pdu.adi.set_id);
    return;
  } else {
    // continuing or finishing an existing chain
    adv_link_t* head = scanner_get_chain_head(scanner, &pdu);
    if (head == NULL) {
      // we didn't see the start of the chain, drop it
      LOG_DBG("missed chain head; dropping\n");
      return;
    }
    // if we're starting to send just adv data, make sure there is an adv a somewhere in the chain
    uint8_t* adv_a = NULL;
    if (pdu.type == ble_aux_chain_ind) {
      for (adv_link_t* x = head; x != NULL; x = x->next_in_chain) {
	if (x->pdu.adv_a_present) {
	  adv_a = x->pdu.adv_a;
	}
      }
      if (!adv_a) {
	// they haven't sent an adv a yet and they've missed their chance
	LOG_DBG("chain never sent adv a; dropping\n");
	scanner_free_chain(scanner, head);
	return;
      }
    }
    // are we finishing or continuing?
    if (pdu.aux_ptr_present) {
      // continuing, so keep adv_data
      pdu.acad_len = acad_end - acad_begin;
      memcpy(&pdu.acad, acad_begin, pdu.acad_len);
      pdu.adv_data_len = adv_data_end - adv_data_begin;
      memcpy(&pdu.adv_data, adv_data_begin, pdu.adv_data_len);
      // point last link to our new link
      //adv_chain_append(head, new_link);
      LOG_DBG("continuing chain %u\n", pdu.adi.set_id);
      return;
    } else {
      // finishing
      // find the sender address
      LOG_DBG("finishing chain %u\n", pdu.adi.set_id);
      scanner_free_chain(scanner, head);
      return;
    }
  }
  LOG_DBG("unaccounted for!\n");
}

static void scan_rx(struct rtimer *t, void *userdata) {
  ble_scanner_t *scanner = (ble_scanner_t *)userdata;
  LOG_DBG("scanner->cmd.status => 0x%x\n", scanner->cmd.status);
  
  while (scanner->rx_queue_current->entry.status == DATA_ENTRY_FINISHED) {
    uint8_t *rx_data = scanner->rx_queue_current->data;
    uint8_t header_lo = *rx_data++;
    uint8_t pdu_type = header_lo & ble_adv_pdu_hdr_type;
    uint8_t payload_len = *rx_data++;
    uint8_t *payload = rx_data;
    uint8_t *payload_end = payload + payload_len;
    switch (pdu_type) {
    /* case ble_adv_ext_ind: */
    /* case ble_aux_adv_ind: */
    /* case ble_aux_chain_ind: */
    case 7:
      scanner_recv_ext_adv(scanner, pdu_type, payload, payload_end);
    default: break;
    }
    /* free current entry */
    scanner->rx_queue_current->entry.status = DATA_ENTRY_PENDING;
    scanner->rx_queue_current = (scan_data_entry*) scanner->rx_queue_current->entry.pNextEntry;
  }

  if (scanner->cmd.status == RF_CORE_RADIO_OP_STATUS_BLE_ERROR_RXBUF) {
    LOG_ERR("Scan rx buffer is out of space! TODO: restart scanner here\n");
    g_scanner.scanning = false;
  } else {
    rtimer_set(&scanner->timer, RTIMER_NOW() + ticks_from_unit(60, TIME_UNIT_MS), 0, scan_rx, scanner);
  }
}

ble_result_t set_scan_enable(unsigned short enable, unsigned short filter_duplicates) {
  if(enable && !g_scanner.scanning) {
    if(on() != BLE_RESULT_OK) {
     LOG_ERR("could not enable rf core prior to scanning\n");
     return BLE_RESULT_ERROR;
    }
    rf_ble_cmd_send((uint8_t*)&g_scanner.cmd);
    g_scanner.scanning = true;
    rtimer_set(&g_scanner.timer, RTIMER_NOW() + ticks_from_unit(100, TIME_UNIT_MS), 0, scan_rx, &g_scanner);
    return BLE_RESULT_OK;
  } else if (!enable) {
    rfc_CMD_STOP_t cmd = {
      .commandNo = CMD_STOP
    };
    rf_ble_cmd_send((uint8_t*)&cmd);
    rf_core_wait_cmd_done((uint8_t*)&cmd);
    g_scanner.scanning = false;
    return BLE_RESULT_OK;
  } else {
    /* already on */
    return BLE_RESULT_OK;
  }
}

const struct ble_hal_driver ble_hal =
{
  reset,
  read_bd_addr,
  NULL, //read_buffer_size,
  adv_ext,
  NULL, //set_adv_param,
  NULL, //read_adv_channel_tx_power,
  NULL, //set_adv_data,
  NULL, //set_scan_resp_data,
  NULL, //set_adv_enable,
  NULL, //set_scan_param,
  set_scan_enable,
  NULL, //create_connection,
  NULL, //create_connection_cancel,
  NULL, //connection_update,
  NULL, //disconnect,
  NULL, //send,
  NULL, //send_list,
  NULL, //read_connection_interval
};
