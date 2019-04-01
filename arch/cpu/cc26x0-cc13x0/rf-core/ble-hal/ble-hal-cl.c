#include "lpm.h"

#include "sys/rtimer.h"

#include "os/dev/ble-hal.h"
#include "dev/oscillators.h"

#include "ble-addr.h"

#include "net/netstack.h"
#include "net/packetbuf.h"

#include "rf_data_entry.h"
#include "rf_ble_cmd.h"

#include "rf-core/rf-core.h"
#include "rf-ble-cmd.h"

#define BLE5_ADV_EXT_HDR_FLAG_ADV_A     (1 << 0)
#define BLE5_ADV_EXT_HDR_FLAG_TGT_A     (1 << 1)
/* RFU */
#define BLE5_ADV_EXT_HDR_FLAG_ADI       (1 << 3)
#define BLE5_ADV_EXT_HDR_FLAG_AUX_PTR   (1 << 4)
#define BLE5_ADV_EXT_HDR_FLAG_SYNC_INFO (1 << 5)
#define BLE5_ADV_EXT_HDR_FLAG_TX_POWER  (1 << 6)
/* RFU... */

#include <string.h>

#include "rf-core/ble-hal/rf-ble-cmd.h"
#if RADIO_CONF_BLE5
#include "rf_patches/rf_patch_cpe_bt5.h"
#else
#error Connectionless BLE radio requies BLE5 support
#endif

#include "sys/log.h"
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

typedef struct {
  /* state */
  bool scanning;
  rtimer_clock_t scan_start;
  struct rtimer timer;
  /* data */
  dataQueue_t rx_queue;
  scan_data_entry rx_buffers[SCAN_RX_BUFFERS_NUM];
  scan_data_entry *rx_queue_current;
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

ble_result_t on(void) {
  oscillators_request_hf_xosc();
  if(!rf_core_is_accessible()) {
    /* boot the rf core */

    /*    boot and apply Bluetooth 5 Patch    */
    if(rf_core_power_up() != RF_CORE_CMD_OK) {
      LOG_ERR("rf_core_boot: rf_core_power_up() failed\n");
      rf_core_power_down();
      return RF_CORE_CMD_ERROR;
    }
    
#if RADIO_CONF_BLE5
    /*  Apply Bluetooth 5 patch, if applicable  */
    rf_patch_cpe_bt5();
#endif
    if(rf_core_start_rat() != RF_CORE_CMD_OK) {
      LOG_ERR("rf_core_boot: rf_core_start_rat() failed\n");
      rf_core_power_down();
      return RF_CORE_CMD_ERROR;
    }
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

/* Advertising */
ble_result_t adv_ext(const uint8_t *tgt_bd_addr, const uint8_t *adv_data, unsigned adv_data_len) {
  bool should_restart_scan = g_scanner.scanning;
  if (should_restart_scan) {
    set_scan_enable(0, 0);
  }
  // max pdu size = 255
  // ext header size = 1
  //   +- flags = 1
  //   +- AdvA  = 6
  //   +- TgtA  = 6
  int header_len = 1 + 6 + (tgt_bd_addr ? 6 : 0);
  int payload_len = header_len + adv_data_len;
  if (payload_len > 255) {
    LOG_ERR("attempt to send adv_ext_ind payload of %d, max is 255.\n", adv_data_len);
    return BLE_RESULT_ERROR;
  }

  rfc_ble5ExtAdvEntry_t adv_pkt = {
    .extHdrInfo = {
      .length = header_len,
      .advMode = 0,
    },
    .extHdrFlags = BLE5_ADV_EXT_HDR_FLAG_ADV_A
                 | (tgt_bd_addr ? BLE5_ADV_EXT_HDR_FLAG_TGT_A : 0),
    .extHdrConfig = {
      .bSkipAdvA = 1
    },
    .advDataLen = adv_data_len,
    .pExtHeader = tgt_bd_addr ? (uint8_t*) tgt_bd_addr : NULL,
    .pAdvData = (uint8_t*) adv_data //TODO: WARNING: double check that this is allowed (re: const cast). spec says only system CPU writes
  };
  
  uint8_t my_addr[BLE_ADDR_SIZE];
  ble_addr_cpy_to(my_addr);

  // Construct parameters and command
  rfc_ble5AdvExtPar_t params = {
    .pAdvPkt = (uint8_t*) &adv_pkt,
    .auxPtrTargetTime = TRIG_NEVER,
    .pDeviceAddress = (uint16_t*)&my_addr[0]
  };
  rfc_bleAdvOutput_t output = { 0 };
  rfc_CMD_BLE5_ADV_EXT_t cmd = {
    .commandNo = CMD_BLE5_ADV_EXT,
    .startTrigger = {
      .triggerType = TRIG_NOW,
    },
    .condition = {
      .rule = COND_NEVER
    },
    .channel = 37,
    .pParams = &params,
    .pOutput = &output
  };

  // Submit command
  if(on() != BLE_RESULT_OK) {
    LOG_DBG("could not enable rf core prior to ADV_EXT \n");
    return BLE_RESULT_ERROR;
  }
  rf_ble_cmd_send((uint8_t*) &cmd);
  rf_ble_cmd_wait((uint8_t*) &cmd);
  off();

  if (should_restart_scan) {
    set_scan_enable(1, 0);
  }
  
  return BLE_RESULT_OK;
}

/* Scanning */
static void init_scanner(ble_scanner_t* scanner) {
  memset(scanner, 0, sizeof(ble_scanner_t));
  for (int i = 0; i < SCAN_RX_BUFFERS_NUM; i++) {
    rfc_dataEntry_t* e = &scanner->rx_buffers[i].entry;
    e->pNextEntry = (uint8_t*) &scanner->rx_buffers[(i + 1) % SCAN_RX_BUFFERS_NUM];
    e->status = DATA_ENTRY_PENDING;
    e->config.type = 0;
    e->config.lenSz = 1;
    e->length = SCAN_RX_BUFFERS_DATA_LEN;
  }
  scanner->rx_queue.pCurrEntry = (uint8_t*) &scanner->rx_buffers[0].entry;
  scanner->rx_queue.pLastEntry = NULL;
  scanner->rx_queue_current = &scanner->rx_buffers[0];
  /*  TODO: figure out why this whitelist let's nothing through */
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
      .bAppendStatus = 1
      //.bAppendRssi = 1 TODO: elliot: maybe use rssi for TSCH-over-BLE5
    },
    .scanConfig = {
      .scanFilterPolicy = 0, /* TODO: set to 1 when whitelist is fixed */
      .bStrictLenFilter = 1
    },
    .pDeviceAddress = (uint16_t*) scanner->ble_addr, // will be checked against incomding tgt addr
    .pWhiteList = scanner->whitelist,
    .timeoutTrigger = {
      .triggerType = TRIG_NEVER
    },
    .endTrigger = {
      .triggerType = TRIG_NEVER
    }
  };
  scanner->cmd = (rfc_CMD_BLE5_SCANNER_t) {
    .commandNo = CMD_BLE5_SCANNER,
    .startTrigger = {
      .triggerType = TRIG_NOW
    },
    .condition = {
      .rule = COND_NEVER // never execute 'next op'
    },
    .channel = 37,
    .pParams = &scanner->params,
    .pOutput = &scanner->output
  };
}

#define BLE5_ADV_DATA_SIZE_MAX 255

static void scan_rx(struct rtimer *t, void *userdata) {
  ble_scanner_t *param = (ble_scanner_t *)userdata;
  
  while (param->rx_queue_current->entry.status == DATA_ENTRY_FINISHED) {
    uint8_t *rx_data = param->rx_queue_current->data;
    uint8_t payload_len = *rx_data++ - 1 /* lenSz = 1 -> 8 bytes for len */
                                     - 2 /* bAppendStatus = 2 (was 1 octec for legacy BLE) */;
    uint8_t header = *rx_data++;
    uint8_t *payload = rx_data;
    uint8_t *payload_end = payload + payload_len;
    rfc_bleRxStatus_t status;
    memcpy(&status, payload_end, sizeof(rfc_bleRxStatus_t));

    uint8_t pdu_type = header & 0b00001111;
    if (pdu_type == 7 && !status.status.bIgnore && !status.status.bCrcErr) {
      uint8_t ext_header_len = *payload++;
      uint8_t* ext_header_end = payload + ext_header_len;
      bool adv_a_present = 0;
      uint8_t adv_a[BLE_ADDR_SIZE];
      linkaddr_t sender_addr;
	
      bool tgt_a_present = 0;
      //uint8_t tgt_a[BLE_ADDR_SIZE];
	
      bool adi_present = 0;
      bool aux_ptr_present = 0;
      bool sync_info_present = 0;
      bool tx_power_present = 0;
      if (ext_header_len > 0) {
	uint8_t ext_header_flags = *payload++;
	  
	adv_a_present = ext_header_flags & BLE5_ADV_EXT_HDR_FLAG_ADV_A;
	if (adv_a_present) {
	  for (int i = 0; i < BLE_ADDR_SIZE; i++) adv_a[i] = *payload++;
	  ble_addr_to_eui64(sender_addr.u8, adv_a);
	}

	tgt_a_present = ext_header_flags & BLE5_ADV_EXT_HDR_FLAG_TGT_A;
	if (tgt_a_present) {
	  for (int i = 0; i < BLE_ADDR_SIZE; i++) {
	    //tgt_a[i] = *payload++; we're not using tgt_a so just move payload forward:
	    payload++;
	  }
	}
	  
	adi_present = ext_header_flags & BLE5_ADV_EXT_HDR_FLAG_ADI;
	if (adi_present) payload += 16;
	  
	aux_ptr_present = ext_header_flags & BLE5_ADV_EXT_HDR_FLAG_AUX_PTR;
	if (aux_ptr_present) payload += 24;
	  
	sync_info_present = ext_header_flags & BLE5_ADV_EXT_HDR_FLAG_SYNC_INFO;
	if (sync_info_present) payload += (13 + 1 + 2 + 2 + 37 + 3 + 4 + 3 + 2);
	    
	tx_power_present = ext_header_flags & BLE5_ADV_EXT_HDR_FLAG_TX_POWER;
	if (tx_power_present) payload += 1;

	payload = ext_header_end;
      }

      packetbuf_clear();
      memcpy(packetbuf_dataptr(), payload, payload_end - payload);
      packetbuf_set_datalen(payload_end - payload);
      packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, &linkaddr_node_addr);
      packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &sender_addr);
      NETSTACK_MAC.input();
    }

    /* free current entry (clear BLE data length & reset status) */
    param->rx_queue_current->entry.status = DATA_ENTRY_PENDING;
    param->rx_queue_current = (scan_data_entry*) param->rx_queue_current->entry.pNextEntry;
  }

  if (param->cmd.status == RF_CORE_RADIO_OP_STATUS_BLE_ERROR_RXBUF) {
    LOG_ERR("Scan rx buffer is out of space!\n");
    g_scanner.scanning = false;
  } else {
    rtimer_set(&param->timer, RTIMER_NOW() + ticks_from_unit(60, TIME_UNIT_MS), 0, scan_rx, param);
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
