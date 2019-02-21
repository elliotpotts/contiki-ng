/**
 * \file
 *    bluetooth assigned number constants
 *
 * \author
 *    Elliot Potts <ep15449@my.bristol.ac.uk>
 */
/*---------------------------------------------------------------------------*/

#ifndef BLE_AN_H_
#define BLE_AN_H_

#define BLE_BDT_FLAGS_LIMITED_DISCOVERABLE_MODE (1 << 0)
#define BLE_BDT_FLAGS_GENERAL_DISCOVERABLE_MODE (1 << 1)
#define BLE_BDT_FLAGS_BR_EDR_NOT_SUPPORTED (1 << 2)
#define BLE_BDT_FLAGS_CONTROLLER_SIMULTANAEITY (1 << 3)
#define BLE_BDT_FLAGS_HOST_SIMULTANEITY (1 << 4)


/* Generic Access Profile assigned numbers
 * https://www.bluetooth.com/specifications/assigned-numbers/generic-access-profile */
#define BLE_AN_GAP_FLAGS                0x01
#define BLE_AN_GAP_SOME16_SERVICE_UUIDS 0x03
#define BLE_AN_GAP_ALL32_SERVICE_UUIDS  0x05
#define BLE_AN_GAP_ALL128_SERVICE_UUIDS 0x07
#define BLE_AN_GAP_TX_POWER_LEVEL       0x0A

#endif /* BLE_AN_H_ */