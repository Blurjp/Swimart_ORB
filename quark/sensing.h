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


#ifndef __SENSING_H__
#define __SENSING_H__

#include <stdbool.h>
#include "cfw/cfw.h"
#include "services/sensor_service/sensor_svc_api.h"

#define ACCEL_BUFFER_DATA_LENGTH 4
#define PACKAGE_COUNT 20

bool handle_double_tap();
bool peakDetection();
void handle_sensor_subscribe_data(struct cfw_message *msg);
void handle_start_scanning_evt(struct cfw_message *msg);
void UpdateStartCountTime(uint32_t time);
void UpdateLapCount(uint8_t count);


/** Sensing init.
 * This will start the required services and start sensor scanning. */
void sensing_init(cfw_handle_t *handle);

extern uint8_t lapCount;
extern uint8_t accel_buffer_data_buf[ACCEL_BUFFER_DATA_LENGTH];
extern bool peakDetected;
extern bool double_tap_detected;
extern bool startCounter;

#endif
