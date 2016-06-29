/*
 * Copyright (c) 2015, Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __BLE_APP_H__
#define __BLE_APP_H__

// For svc_client_handle_t
#include <stdbool.h>
#include "cfw/cfw.h"
//#include "ble_service_lap_count.h"

//typedef enum{False, True} boolean;
// Forward declarations
struct cfw_message;



//#define ACCEL_BUFFER_PACKAGE_COUNT 20;


/**
 * Start BLE application.
 *
 * Register and enable BLE and starts advertising depending on BLE service
 * internal state.
 *
 * @param p_svc_handle client svc handle allocated by CFW on open
 *
 */
void ble_start_app(cfw_handle_t handle);

/** Checks the message id and handle it if it is a BLE APP message.
 * returns true if it really was a BLE APP related message. */
bool handle_ble_messages(struct cfw_message * msg);

bool ble_connected;

void enable_update_lap_count(bool flag);

/**
        * Split data into PACKAGE_COUNT size data and send it to mobile
        * @param data Data from sensor to send
**/
int handle_service_sport_data(uint8_t* data, uint8_t packageCount);

int upload_data_to_device(uint16_t pageIndex, uint16_t offset, uint8_t bufferNum);

#if defined(CONFIG_SERVICES_BLE_ISPP)
/**
 * Send data via ispp.
 *
 * @param buf buffer to send
 * @param len length of buffer
 *
 * @return result of cfw layer sending data to ispp layer
 */
int send_data_to_ispp(uint8_t *buf, uint16_t len);
#endif
#endif /* __BLE_APP_H__ */
