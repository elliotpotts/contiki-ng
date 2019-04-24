#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

#define ROM_BOOTLOADER_ENABLE                   1

#define LOG_CONF_LEVEL_IPV6                     LOG_LEVEL_ERR
#define LOG_CONF_LEVEL_RPL                      LOG_LEVEL_ERR
#define LOG_CONF_LEVEL_SMRF                     LOG_LEVEL_ERR
#define LOG_CONF_LEVEL_6LOWPAN                  LOG_LEVEL_ERR
#define LOG_CONF_LEVEL_TCPIP                    LOG_LEVEL_ERR
#define LOG_CONF_LEVEL_MAC                      LOG_LEVEL_DBG
#define LOG_CONF_LEVEL_FRAMER                   LOG_LEVEL_ERR
#define LOG_CONF_LEVEL_RADIO                    LOG_LEVEL_DBG
#define LOG_CONF_LEVEL_MAIN                     LOG_LEVEL_DBG

#define RTIMER_CONF_MULTIPLE_ACCESS             1

#define ENERGEST_CONF_ON                        0


// Use L2CAP
/* */
#define RADIO_CONF_BLE5                         0
#define BLE_MODE_CONF_MAX_CONNECTIONS           1
#define PACKETBUF_CONF_SIZE                     400
#define QUEUEBUF_CONF_NUM                       1
#define UIP_CONF_BUFFER_SIZE                    400
#define NETSTACK_CONF_RADIO                     ble_cc2650_driver
#define NETSTACK_CONF_MAC                       ble_l2cap_driver

// Use CL
/*
#define RADIO_CONF_BLE5                         1
#define PACKETBUF_CONF_SIZE                     1300
#define QUEUEBUF_CONF_NUM                       1
#define UIP_CONF_BUFFER_SIZE                    1100
#define NETSTACK_CONF_RADIO                     nullradio_driver
#define NETSTACK_CONF_MAC                       ble_cl_driver
*/

#define RPL_CONF_WITH_PROBING                   0

/* network layer settings */

//#define UIP_MCAST6_CONF_ENGINE                  UIP_MCAST6_ENGINE_ROLL_TM
#define UIP_MCAST6_CONF_ENGINE                  UIP_MCAST6_ENGINE_SMRF
#define UIP_CONF_ROUTER                         0
#define UIP_CONF_ND6_SEND_RA                    0
#define UIP_CONF_ND6_SEND_NA                    1
#define UIP_CONF_ND6_SEND_NS                    1

/* 6LoWPAN settings */
#define SICSLOWPAN_CONF_MAC_MAX_PAYLOAD         1600
#define SICSLOWPAN_CONF_COMPRESSION             SICSLOWPAN_COMPRESSION_6LORH
#define SICSLOWPAN_CONF_COMPRESSION_THRESHOLD   0  /* always use compression */
#define SICSLOWPAN_CONF_FRAG                    0
#define SICSLOWPAN_FRAMER_HDRLEN                0

#endif /* PROJECT_CONF_H_ */
