#include "ble5.h"

#include <string.h>

static void rmemcpy(void *restrict dst, const void *restrict src, unsigned count) {
  unsigned char *dst_char = dst;
  const unsigned char *src_char = src;
  for (size_t i = 0; i < count; ++i) {
    dst_char[count - 1 - i] = src_char[i];
  }
}

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
