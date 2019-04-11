#ifndef BLE5_H_
#define BLE5_H_

typedef enum {
  ble_adv_ext_ind,
  ble_aux_adv_ind,
  ble_aux_chain_ind
} ble_adv_pdu_type_t;

typedef enum {
  ble5_adv_ext_hdr_flag_adv_a = 1 << 0,
  ble5_adv_ext_hdr_flag_tgt_a = 1 << 1,
  ble5_adv_ext_hdr_flag_adi = 1 << 3,
  ble5_adv_ext_hdr_flag_aux_ptr = 1 << 4,
  ble5_adv_ext_hdr_flag_sync_info = 1 << 5,
  ble5_adv_ext_hdr_flag_tx_power = 1 << 6,
} ble5_adv_ext_hdr_flag_t;

enum { BLE5_ADV_PDU_PAYLOAD_MAX_SIZE = 255 };

enum {
  ble_adv_pdu_hdr_type =  0b00001111,
  ble_adv_pdu_hdr_chsel = 0b00100000,
  ble_adv_pdu_hdr_txadd = 0b01000000,
  ble_adv_pdu_hdr_rxadd = 0b10000000
};

/* Accuracy of a clock */
typedef enum {
  /* between 51 ppm and 500 ppm */
  ble5_clock_accuracy_low = 0,
  /* between 0 ppm and 50 ppm */
  ble5_clock_accuracy_high = 1
} ble5_clock_accuracy_t;

/* Unit in time of a quantity */
typedef enum {
  /* aux_offset in seconds = aux_offset * 30,000 */
  ble5_offset_units_30us = 0,
  /* aux_offset in seconds = aux_offset * 3,000 */
  ble5_offset_units_300us = 1
} ble5_offset_units_t;

typedef enum {
  ble5_phy_le1m,
  ble5_phy_le2m,
  ble5_phy_coded
} ble5_phy_t;

/* Advertising Data Info
 * Sequence numbers used to distinguish advertising packets from one another
 * see Bluetooth 5.0 Core Spec Vol. 6, Pt. B, 2.3.4.4 */
typedef struct {
  /* A number unique to an advertising packet within a set of chained advertisements */
  uint16_t data_id:12;
  /* A number unique to a chain of advertisements */
  uint8_t set_id:4;
} __attribute__ ((packed)) adi_t;

/* Auxilliary Pointer
 * Describes how to scan for the next packet which will arrive in the chain
 * see Bluetooth 5.0 Core Spec Vol. 6, Pt. B, 2.3.4.5 */
typedef struct {
  /* Channel on which the packet will arrive */
  uint8_t channel_ix:6;
  /* Accuracy of the advertiser's clock */
  ble5_clock_accuracy_t clock_accuracy:1;
  /* Units aux_offset are specified in */
  ble5_offset_units_t offset_units:1;
  /* Time from start of packet containing the aux. ptr. to the approximate start of the auxilliary packet */
  uint16_t aux_offset:13;
  /* PHY used to transmit the auxilliary packet */
  ble5_phy_t aux_phy:3;
} __attribute__ ((packed)) aux_ptr_t;

// Bluetooth 5.0 Core Spec Vol. 6, Pt. B, 2.3.4.6
typedef struct {
  uint16_t sync_pkt_offset:13;
  uint8_t offset_units:1;
  uint8_t ___rfu0:2;
  uint8_t interval:2;
  uint64_t channel_map:37;
  uint8_t sleep_clock_accuracy:3;
  uint8_t access_addr:4;
  uint8_t crc_init:3;
  uint8_t event_cnt:2;
} __attribute__ ((packed)) sync_info_t;

typedef struct {
  ble_adv_pdu_type_t type;
  uint8_t adv_mode;
  bool adv_a_present;
  uint8_t adv_a[BLE_ADDR_SIZE];
  bool tgt_a_present;
  uint8_t tgt_a[BLE_ADDR_SIZE];
  bool adi_present;
  adi_t adi;
  bool aux_ptr_present;
  aux_ptr_t aux_ptr;
  bool sync_info_present;
  sync_info_t sync_info;
  bool tx_power_present;
  uint8_t tx_power;
  uint8_t acad_len;
  uint8_t acad[63];
  uint8_t adv_data_len;
  uint8_t adv_data[255];
} ext_adv_pdu;

#endif /* BLE5_H_ */
