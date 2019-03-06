/*
 * Copyright (c) 2019, University of Bristol
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 *    MAC layer that implements BLE using connectionless advertising
 *    packets to suppoer IPv6 over BLE (???)
 *
 * \author
 *    Elliot Potts <ep15449@my.bristol.ac.uk>
 */
/*---------------------------------------------------------------------------*/
#ifndef BLE_CL_H_
#define BLE_CL_H_

#include "contiki.h"
#include "net/mac/mac.h"
#include "dev/radio.h"
/*---------------------------------------------------------------------------*/
/* device name used for BLE advertisement */
#ifdef BLE_CONF_DEVICE_NAME
#define BLE_DEVICE_NAME BLE_CONF_DEVICE_NAME
#else
#define BLE_DEVICE_NAME "BLE device name"
#endif

/* BLE advertisement in milliseconds */
#ifdef BLE_CONF_ADV_INTERVAL
#define BLE_ADV_INTERVAL BLE_CONF_ADV_INTERVAL
#else
#define BLE_ADV_INTERVAL 50
#endif


extern const struct mac_driver ble_cl_driver;

#endif /* BLE_CL_H_ */
