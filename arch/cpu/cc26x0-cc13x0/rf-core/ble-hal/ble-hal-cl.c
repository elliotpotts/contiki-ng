#ifdef PROJECT_CONF_PATH
#include PROJECT_CONF_PATH
#endif /* PROJECT_CONF_PATH */

#if !RADIO_CONF_BLE5
#error Connectionless BLE requies BLE5 support
#endif

#include <string.h>
#include <assert.h>

#include "os/sys/rtimer.h"
#include "os/sys/log.h"
#include "os/lib/random.h"
#include "os/dev/ble-hal.h"
#include "os/net/netstack.h"
#include "os/net/packetbuf.h"
#include "sys/energest.h"

#include "dev/oscillators.h"
#include "lpm.h"
#include "ble-addr.h"

#include "rf_data_entry.h"
#include "rf_ble_cmd.h"
#include "rf-core/rf-core.h"
#include "rf-ble-cmd.h"
#include "rf-core/ble-hal/rf-ble-cmd.h"
#include "rf_patches/rf_patch_cpe_bt5.h"

#include "os/dev/ble5.h"

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
#define SCAN_RX_INTERVAL               (ticks_from_unit(4, TIME_UNIT_MS))

typedef struct {
  rfc_dataEntry_t entry;
  uint8_t data[SCAN_RX_BUFFERS_DATA_LEN];
} __attribute__ ((packed)) scan_data_entry;

typedef struct {
  struct rtimer timer;
  /* data */
  dataQueue_t rx_queue;
  scan_data_entry rx_buffers[SCAN_RX_BUFFERS_NUM];
  scan_data_entry *rx_queue_current;
  /* radio interface */
  uint8_t ble_addr[BLE_ADDR_SIZE];
  rfc_bleWhiteListEntry_t whitelist[6];
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
  LOG_DBG("Turning off!\n");
  rf_core_power_down();
  oscillators_switch_to_hf_rc();
}

static ble_result_t read_bd_addr(uint8_t *addr) {
  ble_addr_cpy_to(addr);
  return BLE_RESULT_OK;
}

unsigned min(unsigned lhs, unsigned rhs) {
  if (lhs < rhs) {
    return lhs;
  } else {
    return rhs;
  }
}

enum { AUX_TGT_DELAY_TICKS = 60000 };
enum { ADV_PREPROCESSING_TICKS = 800 };

/* unsigned long base = ticks_to_unit(RTIMER_NOW(), TIME_UNIT_RF_CORE); */
/* unsigned long ext_start = base + ticks_to_unit(ticks_from_unit(50, TIME_UNIT_MS), TIME_UNIT_RF_CORE); */
/* unsigned long aux_tgt = ext_start + 60000; */
/* unsigned long aux_start = aux_tgt - 800; */

typedef struct {
  aux_ptr_t last_aux_ptr;
  long unsigned last_start_time;
} advertiser_t;
static advertiser_t g_advertiser;

ble5_ext_adv_result_t adv_ext(const uint8_t *tgt_addr,
			      const adi_t *adi,
			      const aux_ptr_t *aux_ptr,
			      const uint8_t *data_begin,
			      const uint8_t *data_end) {
  unsigned data_len = data_end - data_begin;
  if (data_len <= BLE5_ADV_DATA_MAX_SIZE - BLE_ADDR_SIZE - sizeof(*adi) - sizeof(*aux_ptr)) {
    // Total remaining data will fit in this IND.
    // We are the first and last packet therefore no need for an adi nor an aux_ptr
    adi = NULL;
    aux_ptr = NULL;
  }

  uint8_t adv_addr[BLE_ADDR_SIZE];
  ble_addr_cpy_to(adv_addr);

  uint8_t hdr[BLE5_EXT_HDR_MAX_SIZE];
  write_ext_adv_hdr_result_t hdr_result = write_ext_adv_hdr(hdr, NULL, adv_addr, tgt_addr, adi, aux_ptr, NULL, NULL);
  unsigned hdr_tot_len = hdr_result.length + 1; // + 1 because radio cpu adds flags for us
  unsigned max_adv_data_len = BLE5_ADV_DATA_MAX_SIZE - hdr_tot_len;
  unsigned adv_data_len = min(max_adv_data_len, data_len);
  
  unsigned long start_time = ticks_to_unit(RTIMER_NOW(), TIME_UNIT_RF_CORE) + AUX_TGT_DELAY_TICKS;
    
  rfc_ble5ExtAdvEntry_t entry = {
    .extHdrInfo = { .length = hdr_tot_len },
    .extHdrFlags = hdr_result.flags,
    .pExtHeader = hdr,
    .advDataLen = adv_data_len,
    .pAdvData = (uint8_t*) data_begin
  };
  rfc_ble5AdvExtPar_t params = {
    .pAdvPkt = (uint8_t*) &entry,
    .auxPtrTargetType = TRIG_ABSTIME,
    .auxPtrTargetTime = start_time + AUX_TGT_DELAY_TICKS
  };
  rfc_bleAdvOutput_t output = { 0 };
  rfc_CMD_BLE5_ADV_EXT_t cmd = {
    .commandNo = CMD_BLE5_ADV_EXT,
    .startTime = start_time,
    .startTrigger = { .triggerType = TRIG_ABSTIME },
    .condition = { .rule = COND_NEVER },
    .channel = 37,
    .pParams = &params,
    .pOutput = &output
  };  
  if (rf_ble_cmd_send((uint8_t*) &cmd) != RF_BLE_CMD_OK) {
    LOG_ERR("Fatal: failed to send CMD_BLE5_ADV_EXT!\n");
    return (ble5_ext_adv_result_t) {
      .bytes_sent = 0,
      .radio_data = &g_advertiser
    };
  };
  ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
  g_advertiser.last_start_time = start_time;
  memcpy(&g_advertiser.last_aux_ptr, aux_ptr, sizeof(*aux_ptr));
  if (rf_ble_cmd_wait((uint8_t*) &cmd) != RF_BLE_CMD_OK) {
    LOG_ERR("Fatal: failed to wait for CMD_BLE5_ADV_EXT!\n");
    return (ble5_ext_adv_result_t) {
      .bytes_sent = 0,
      .radio_data = &g_advertiser
    };
  }
  ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
  return (ble5_ext_adv_result_t) {
    .bytes_sent = adv_data_len,
    .radio_data = &g_advertiser
  };
}

ble5_ext_adv_result_t aux_adv(ble5_ext_adv_result_t *prev_result,
			      const adi_t *adi,
			      const aux_ptr_t *aux_ptr,
			      const uint8_t *data_begin,
			      const uint8_t *data_end) {
  advertiser_t* advertiser = prev_result->radio_data;

  unsigned data_len = data_end - data_begin;
  if (data_len <= BLE5_ADV_DATA_MAX_SIZE - sizeof(*adi)) {
    // Total remaining data will fit in this IND.
    // We are the last packet therefore no need for an aux_ptr
    aux_ptr = NULL;
  }

  uint8_t hdr[BLE5_EXT_HDR_MAX_SIZE];
  write_ext_adv_hdr_result_t hdr_result = write_ext_adv_hdr(hdr, NULL, NULL, NULL, adi, aux_ptr, NULL, NULL);
  unsigned hdr_tot_len = hdr_result.length + 1; // + 1 because radio cpu adds flags for us
  unsigned max_adv_data_len = BLE5_ADV_DATA_MAX_SIZE - hdr_tot_len;
  unsigned adv_data_len = min(max_adv_data_len, data_len);

  unsigned long start_time = advertiser->last_start_time + AUX_TGT_DELAY_TICKS;
  
  rfc_ble5ExtAdvEntry_t entry = {
    .extHdrInfo = { .length = hdr_tot_len },
    .extHdrFlags = hdr_result.flags,
    .pExtHeader = hdr,
    .advDataLen = adv_data_len,
    .pAdvData = (uint8_t*) data_begin
  };
  rfc_ble5AdvAuxPar_t params = {
    .pAdvPkt = (uint8_t*) &entry,
    .auxPtrTargetType = TRIG_ABSTIME,
    .auxPtrTargetTime = start_time + AUX_TGT_DELAY_TICKS
  };
  rfc_bleAdvOutput_t output = { 0 };
  rfc_CMD_BLE5_ADV_AUX_t cmd = {
    .commandNo = CMD_BLE5_ADV_AUX,
    .startTime = start_time - ADV_PREPROCESSING_TICKS,
    .startTrigger = { .triggerType = TRIG_ABSTIME },
    .condition = { .rule = COND_NEVER },
    .channel = advertiser->last_aux_ptr.channel_ix,
    .pParams = &params,
    .pOutput = &output
  };
  if (rf_ble_cmd_send((uint8_t*) &cmd) != RF_BLE_CMD_OK) {
    LOG_ERR("Fatal: failed to send CMD_BLE5_ADV_AUX!\n");
    return (ble5_ext_adv_result_t) {
      .bytes_sent = 0,
      .radio_data = &g_advertiser
    };
  };
  ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
  advertiser->last_start_time = start_time;
  memcpy(&advertiser->last_aux_ptr, aux_ptr, sizeof(*aux_ptr));
  if (rf_ble_cmd_wait((uint8_t*) &cmd) != RF_BLE_CMD_OK) {
    LOG_ERR("Fatal: failed to wait for CMD_BLE5_ADV_AUX!\n");
    if (cmd.status == 0x800) {
      LOG_ERR("Cause: scheduled to execute in the past. Time now: %lu, scheduled for: %lu\n", RTIMER_NOW(), cmd.startTime);
    }
    return (ble5_ext_adv_result_t) {
      .bytes_sent = 0,
      .radio_data = &g_advertiser
    };
  };
  ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);

  return (ble5_ext_adv_result_t) {
    .bytes_sent = adv_data_len,
    .radio_data = &g_advertiser
  };
}

/* Scanning */
static void init_scanner(ble_scanner_t* scanner) {
  memset(scanner, 0, sizeof(*scanner));
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
    .size = 6,
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
  scanner->whitelist[5] = (rfc_bleWhiteListEntry_t) {
    .conf = { .bEnable = 1 },
    .addressHi = (0xB0 << 24) | (0x91 << 16) | (0x22 << 8) | 0x69,
    .address =   (0xFB << 8)  | 0x8F
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

ble_result_t scanner_stop(ble_scanner_t* scanner) {
  rfc_CMD_STOP_t cmd = {
    .commandNo = CMD_STOP
  };
  if (rf_ble_cmd_send((uint8_t*) &cmd) != RF_BLE_CMD_OK) {
    LOG_ERR("Fatal: failed to send CMD_STOP!\n");
    return BLE_RESULT_ERROR;
  };
  uint_fast8_t cmd_status = rf_core_wait_cmd_done((uint8_t*)&cmd);
  assert(cmd_status = RF_CORE_CMD_OK);
  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
  return BLE_RESULT_OK;
}

static void scan_rx(struct rtimer *t, void *userdata);
ble_result_t scanner_start(ble_scanner_t* scanner) {
  if (scanner->cmd.status == RF_CORE_RADIO_OP_STATUS_ACTIVE) {
    // Scanner command already running
    //potential bug: need to set rtimer again?
    return BLE_RESULT_OK;
  } else {
    scanner->cmd.status = RF_CORE_RADIO_OP_STATUS_IDLE;
    if (rf_ble_cmd_send((uint8_t*) &scanner->cmd) != RF_BLE_CMD_OK) {
      LOG_ERR("Fatal: failed to send CMD_BLE5_SCANNER!\n");
      return BLE_RESULT_ERROR;
    };
    ENERGEST_ON(ENERGEST_TYPE_LISTEN);
    
    //and PRAY!
    rtimer_set(&scanner->timer, RTIMER_NOW() + SCAN_RX_INTERVAL, 0, scan_rx, scanner);
    return BLE_RESULT_OK;
  }
}

static ble_result_t reset(void) {
  lpm_register_module(&cc26xx_ble_lpm_module);
  rf_core_set_modesel();
  if(on() != BLE_RESULT_OK) {
    LOG_DBG("Error turning on!\n");
    return BLE_RESULT_ERROR;
  }
  init_scanner(&g_scanner);
  scanner_start(&g_scanner);
  return BLE_RESULT_OK;
}

//TODO terrible disgusting ugly hack - please replace!
uint8_t* g_payload;
uint8_t* g_payload_end;
static void scan_rx(struct rtimer *t, void *userdata) {
  ble_scanner_t *scanner = (ble_scanner_t *)userdata;
  while (scanner->rx_queue_current->entry.status == DATA_ENTRY_FINISHED) {
    uint8_t *rx_data = scanner->rx_queue_current->data;
    uint8_t header_lo = *rx_data++;
    uint8_t pdu_type = header_lo & ble_adv_pdu_hdr_type;
    uint8_t payload_len = *rx_data++;
    uint8_t *payload = rx_data;
    uint8_t *payload_end = payload + payload_len;
    if (pdu_type == ble5_adv_pdu_type_ext_adv) {
      g_payload = payload;
      g_payload_end = payload_end;
      NETSTACK_MAC.input();
    } else {
      LOG_DBG("Unknown adv type\n");
    }
    /* free current entry */
    scanner->rx_queue_current->entry.status = DATA_ENTRY_PENDING;
    scanner->rx_queue_current = (scan_data_entry*) scanner->rx_queue_current->entry.pNextEntry;
  }

  if (scanner->cmd.status == RF_CORE_RADIO_OP_STATUS_BLE_ERROR_RXBUF) {
    LOG_ERR("Scan rx buffer is out of space! TODO: restart scanner here\n");
  } else if (scanner->cmd.status == RF_CORE_RADIO_OP_STATUS_BLE_DONE_OK) {
    scanner_start(scanner);
  } else {
    rtimer_set(&scanner->timer, RTIMER_NOW() + SCAN_RX_INTERVAL, 0, scan_rx, scanner);
  }
}

ble_result_t set_scan_enable(unsigned short enable, unsigned short filter_duplicates) {
  if (enable) {
    return scanner_start(&g_scanner);
  } else {
    return scanner_stop(&g_scanner);
  }
}

const struct ble_hal_driver ble_hal =
{
  reset,
  read_bd_addr,
  NULL, //read_buffer_size,
  adv_ext,
  aux_adv,
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
