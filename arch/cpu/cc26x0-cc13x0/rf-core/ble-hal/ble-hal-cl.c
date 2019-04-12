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
  bool is_head;
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

typedef struct {
  unsigned length;
  unsigned flags;
} write_ext_adv_hdr_result_t;

/**
 * Write given fields to a given output pointer according to the common extended advertising format
 *
 *     flags - pointer to byte representing flags.
 *             NULL for no flag
 *  adv_addr - pointer to uint8_t[6] representing this devices address with most-significant-octet first
 *             NULL for no advertising address
 *  tgt_addr - pointer to uint8_t[6] representing the target devices address with most-significant-octet first
 *             NULL for no advertising address
 *       adi - pointer to an adi_t describing the AdvDataInfo
 *             NULL for no adi
 *   aux_ptr - pointer to an aux_ptr_t describing how the auxilliary packet can be received
 *             NULL for no auxilliary packet pointer
 * sync_info - pointer to a sync_info_t describing how to time synchronise to periodic advertisements
 *             NULL for no sync info
 *  tx_power - pointer to a byte stating the transmission power
 *             NULL for no tx power
 * 
 * Returns flags representing which fields were non-null, and the total length of non-null fields
 **/
write_ext_adv_hdr_result_t
write_ext_adv_hdr(uint8_t *out,
		  const uint8_t *flags,
		  const uint8_t *adv_addr,
		  const uint8_t *tgt_addr,
		  const adi_t *adi,
		  const aux_ptr_t *aux_ptr,
		  const sync_info_t *sync_info,
		  const uint8_t *tx_power) {
  unsigned flags_out = 0;
  unsigned length_out = 0;
  if (flags) {
    *out++ = *flags;
    length_out += 1;
  }
  if (adv_addr) {
    flags_out |= ble5_adv_ext_hdr_flag_adv_a;
    rmemcpy(out, adv_addr, BLE_ADDR_SIZE);
    out += BLE_ADDR_SIZE;
    length_out += BLE_ADDR_SIZE;
  }
  if (tgt_addr) {
    flags_out |= ble5_adv_ext_hdr_flag_tgt_a;
    rmemcpy(out, tgt_addr, BLE_ADDR_SIZE);
    out += BLE_ADDR_SIZE;
    length_out += BLE_ADDR_SIZE;
  }
  if (adi) {
    flags_out |= ble5_adv_ext_hdr_flag_adi;
    memcpy(out, adi, sizeof(*adi));
    out += sizeof(*adi);
    length_out += sizeof(*adi);
  }
  if (aux_ptr) {
    flags_out |= ble5_adv_ext_hdr_flag_aux_ptr;
    memcpy(out, aux_ptr, sizeof(*aux_ptr));
    out += sizeof(*aux_ptr);
    length_out += sizeof(*aux_ptr);
  }
  if (sync_info) {
    flags_out |= ble5_adv_ext_hdr_flag_sync_info;
    memcpy(out, sync_info, sizeof(*sync_info));
    out += sizeof(*sync_info);
    length_out += sizeof(*sync_info);
  }
  if (tx_power) {
    flags_out |= ble5_adv_ext_hdr_flag_tx_power;
    memcpy(out, tx_power, sizeof(*tx_power));
    out += sizeof(*tx_power);
    length_out += sizeof(*tx_power);
  }
  return (write_ext_adv_hdr_result_t) {
    .length = length_out,
    .flags = flags_out
  };
}

enum { hello_world_len = 13 };
uint8_t hello_world[hello_world_len] = "Hello, World!";

enum { lorem_ipsum_len = 255 };
uint8_t lorem_ipsum[lorem_ipsum_len] = "Lorem ipsum dolor sit amet, consectetur adipiscing elit. In sem ipsum, dapibus commodo risus eget, porta egestas nunc. Nam ultricies enim non quam accumsan vestibulum. Mauris venenatis consectetur diam, nec eleifend odio blandit ac. Duis ac interdum amet.";

/* Advertising */
ble_result_t adv_ext(const uint8_t *tgt_bd_addr, const uint8_t *adv_data, unsigned adv_data_len) {
  // We can't send and scan at the same time, so disable scanning
  set_scan_enable(0, 0);

  unsigned long base = ticks_to_unit(RTIMER_NOW(), TIME_UNIT_RF_CORE);
  unsigned long ext_start = base + ticks_to_unit(ticks_from_unit(50, TIME_UNIT_MS), TIME_UNIT_RF_CORE);
  unsigned long aux_tgt = ext_start + 5000;
  unsigned long aux_start = aux_tgt - 680;

  rfc_bleAdvOutput_t output = { 0 }; // clear all counters

  unsigned common_set_id = random_rand() % 16;

  /* PACKET 2 */
  adi_t aux_adv0_adi = {
    .set_id = common_set_id,
    .data_id = random_rand() % 4096
  };
  uint8_t aux_adv0_hdr[64];
  write_ext_adv_hdr_result_t aux_adv0_hdr_result = write_ext_adv_hdr(aux_adv0_hdr, NULL, NULL, NULL, &aux_adv0_adi, NULL, NULL, NULL);
  rfc_ble5ExtAdvEntry_t aux_adv0_entry = {
    .extHdrInfo = { .length = aux_adv0_hdr_result.length + 1 }, // +1 because radio cpu adds flags for us
    .extHdrFlags = aux_adv0_hdr_result.flags,
    .pExtHeader = aux_adv0_hdr,
    .advDataLen = 0,
    .pAdvData = NULL
  };
  rfc_ble5AdvAuxPar_t aux_adv0_params = {
    .pAdvPkt = (uint8_t*) &aux_adv0_entry
  };
  rfc_CMD_BLE5_ADV_AUX_t aux_adv0_cmd = {
    .commandNo = CMD_BLE5_ADV_AUX,
    .startTime = aux_start,
    .startTrigger = { .triggerType = TRIG_ABSTIME },
    .condition = { .rule = COND_NEVER },
    .channel = 20,
    .pParams = &aux_adv0_params,
    .pOutput = &output
  };
  
  /* PACKET 1 */
  // device address stored most-significant-octet first
  uint8_t adv_addr[BLE_ADDR_SIZE];
  ble_addr_cpy_to(adv_addr);
  adi_t adv_ext_adi = {
    .set_id = common_set_id,
    .data_id = random_rand() % 4096
  };
  aux_ptr_t adv_ext_aux_ptr = {
    .channel_ix = aux_adv0_cmd.channel,
    /*.offset_units = <filled by radio cpu> */
    /*.aux_offset = <filled by radio cpu> */
  };
  uint8_t adv_ext_hdr[64];
  write_ext_adv_hdr_result_t adv_ext_hdr_result = write_ext_adv_hdr(adv_ext_hdr, NULL, adv_addr, NULL, &adv_ext_adi, &adv_ext_aux_ptr, NULL, NULL);
  rfc_ble5ExtAdvEntry_t adv_ext_entry = {
    .extHdrInfo = { .length = adv_ext_hdr_result.length + 1 }, // +1 because radio cpu adds flags for us
    .extHdrFlags = adv_ext_hdr_result.flags,
    .pExtHeader = adv_ext_hdr,
  };
  rfc_ble5AdvExtPar_t adv_ext_params = {
    .pAdvPkt = (uint8_t*) &adv_ext_entry,
    .auxPtrTargetType = TRIG_ABSTIME,
    .auxPtrTargetTime = aux_tgt
  };
  rfc_CMD_BLE5_ADV_EXT_t adv_ext_cmd = {
    .commandNo = CMD_BLE5_ADV_EXT,
    .startTime = ext_start,
    .startTrigger = { .triggerType = TRIG_ABSTIME },
    .pNextOp = (rfc_radioOp_t*) &aux_adv0_cmd,
    .condition = { .rule = COND_ALWAYS },
    .channel = 37,
    .pParams = &adv_ext_params,
    .pOutput = &output
  };

  rf_ble_cmd_send((uint8_t*) &adv_ext_cmd);

  long unsigned ext_sent = RTIMER_NOW();
  long unsigned ext_started;
  long unsigned aux_started;

  while (ext_started = RTIMER_NOW(), adv_ext_cmd.status != 2) {
  }
  while (aux_started = RTIMER_NOW(), aux_adv0_cmd.status != 2) {
  }

  LOG_DBG("       base -> %lu\n", base);
  LOG_DBG("  ext_start -> %lu\n", ext_start);
  LOG_DBG("    aux_tgt -> %lu\n", aux_tgt);
  LOG_DBG("  aux_start -> %lu\n", aux_start);
  LOG_DBG("   ext_sent -> %lu\n", ext_sent);
  LOG_DBG("ext_started -> %lu\n", ext_started);
  LOG_DBG("aux_started -> %lu\n", aux_started);

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

/* Start a new chain with the given pdu as it's first link */
static adv_link_t* scanner_chain_start(ble_scanner_t* scanner, const ext_adv_pdu* pdu) {
  for (int i = 0; i < SCAN_RX_BUFFERS_NUM; i++) {
    adv_link_t* link = &scanner->adv_links[i];
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
static adv_link_t* scanner_chain_get_head(ble_scanner_t* scanner, const ext_adv_pdu* pdu) {
  for (int i = 0; i < SCAN_RX_BUFFERS_NUM; i++) {
    adv_link_t* link = &scanner->adv_links[i];
    if (link->populated && link->is_head && link->pdu.adi.set_id == pdu->adi.set_id) {
      return link;
    }
  }
  return NULL;
}

/* Append the given pdu to the end of the chain containing the given link */
static adv_link_t* scanner_chain_append(ble_scanner_t* scanner, adv_link_t* head, const ext_adv_pdu* pdu) {
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
  for (int i = 0; i < SCAN_RX_BUFFERS_NUM; i++) {
    adv_link_t* link = &scanner->adv_links[i];
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

static adv_link_t* scanner_chain_finish(ble_scanner_t* scanner, adv_link_t* head, const ext_adv_pdu* last_pdu) {
  LOG_DBG("FINISHING WITH DATA:\n");
  while (head != NULL) {
    LOG_DBG("    %s\n", head->pdu.adv_data);
    head = head->next_in_chain;
  }
  LOG_DBG("    %s\n", last_pdu->adv_data);
  return NULL;
}

static void scanner_recv_ext_adv(ble_scanner_t* scanner, uint8_t *payload, uint8_t *payload_end) { 
  ext_adv_pdu pdu = { 0 };

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
    LOG_DBG("TODO: handle non-chained data; dropping\n");
    return;
  } else if (adv_data_begin == adv_data_end) {
    LOG_DBG("no adv_data; chain will be started on first aux packet\n");
    return;
  } else {
    // Are we tracking this chain yet?
    adv_link_t* chain_head = scanner_chain_get_head(scanner, &pdu);
    if (chain_head) {
      // Yes, so continue or finish it
      if (pdu.aux_ptr_present) {
	// Continue chain 
	if (scanner_chain_append(scanner, chain_head, &pdu)) {
	  LOG_DBG("Appending did %u to sid %u\n", pdu.adi.data_id, pdu.adi.set_id);
	} else {
	  LOG_DBG("Could not append did %u to sid %u\n", pdu.adi.data_id, pdu.adi.set_id);
	}
	return;
      } else {
	// Finishing
	if (scanner_chain_finish(scanner, chain_head, &pdu)) {
	  LOG_DBG("Finished chain with sid %u\n", pdu.adi.set_id);
	} else {
	  LOG_ERR("Couldn't finish chain with sid %u\n", pdu.adi.set_id);
	}
	return;
      }
    } else {
      // No, will there be further aux packets?
      if (pdu.aux_ptr_present) {
	// yes, start a new chain
	if (scanner_chain_start(scanner, &pdu)) {
	  LOG_DBG("Started chain sid %u\n", pdu.adi.set_id);
	} else {
	  LOG_ERR("Couldn't start new chain!\n");
	}
	return;
      } else {
	// no, instead of starting then finishing, just report packet directly
	LOG_DBG("TODO: send single adv_data to upper layer\n");
	return;
      }
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
      scanner_recv_ext_adv(scanner, payload, payload_end);
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
