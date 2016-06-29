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

#include <stdint.h>
#include "os/os.h"
#include "infra/log.h"
#include "sensing.h"
#include "infra/time.h"
#include "cfw/cfw.h"
/* CFW messages definition, to get connection messages definitions */
#include "cfw/cfw_messages.h"
/* Get services queue */
#include "services/service_queue.h"
#include "ssd1360.h"
/* Main sensors API */
#include "services/sensor_service/sensor_svc_api.h"
/* Utility API */
#include "services/sensor_service/sensor_svc_utils.h"
/* Properties Service will allow to read/store sensor data to flash */
#include "services/properties_service_api.h"
#include "services/ble/ble_service_lap_count.h"

#include "ble_app.h"

/* Mmodes macros */
#define MODE_GEST 0
#define MODE_STEP 1
/* Sensors properties */
#define ACCEL_CALI_DATA_NO 2
#define GYRO_CALI_DATA_NO 3

bool startCounter = false;
bool peakDetected = false;
bool double_tap_detected = false;

svc_client_handle_t *sensor_handle = NULL;
svc_client_handle_t *props_service_handle = NULL;
static cfw_handle_t props_handle;
static storage_class_t non_persist_storage_class = { false, false };

/* Variables used to keep track of sensing activities */
/* TODO: these variables cannot be made static because they are used by a test
 * command. This should not depend on this file though! */

int gyro_cali_data_buf[3];
int accel_cali_data_buf[3];
static uint8_t set_accel_cali_data_flag;
static uint8_t set_gyro_cali_data_flag;

uint8_t accel_buffer_data_buf[ACCEL_BUFFER_DATA_LENGTH];

typedef enum{False, True} boolean;

uint32_t startCountTime = 0;
uint32_t turnDetectionTime = 0;
uint8_t lapCount = 0;
float swimSpeed;

int counterTime;
int ret;
short minNum = 0;
short maxNum = 0;

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

void handle_tap_count(s16 tapCount)
{
	if(tapCount >= 2)
	{

		double_tap_detected = True;
	}
	/* reset tap detected in main loop ? */
	//else
	//	double_tap_detected = False;
}

bool handle_double_tap()
{
			if(double_tap_detected)
			{
				double_tap_detected = false;
		        startCounter = ~startCounter;

			}
			return startCounter;

}


bool peakDetection()
{
	if(startCounter)
	{
				if ((startCountTime > turnDetectionTime) && (maxNum - minNum > 100) && peakDetected) //check next peak after 10 second
				{
				      lapCount+=1;

				      counterTime = (get_uptime_ms() - startCountTime) / 1000;

				      swimSpeed = lapCount * 25 / counterTime;

				      //accelData.clean();
				      minNum = 0;
				      maxNum = 0;
				      peakDetected = false;
				      //peakDetected = False;
				      //sendData = String(lapCount)+";"+String(counterTime)+";"+String(swimSpeed);
				      //Serial.println(sendData);
				      //CHECK_STATUS(lapCountChar.setValue(lapCount));
				      //CHECK_STATUS(timeDataChar.setValue(counterTime));

				      displayInfo(swimSpeed, lapCount, counterTime);
				      accel_buffer_data_buf[3] = lapCount;
				      //if((ret = handle_service_sport_data(accel_cali_data_buf))!=0)
				      //			pr_error(LOG_MODULE_BLE, "ble_update_service_lap_count failed: %d.", ret);
				      turnDetectionTime = startCountTime + 10000;        //check next peak in 0.2 seconds
				}
				else
				{
				      minNum = min(minNum, (short)accel_buffer_data_buf[0]);
				      maxNum = max(maxNum, (short)accel_buffer_data_buf[0]);
				      //IdentifyStroke();
				}
				/*  write the data into memory chip							 */
			    //WriteIntoDevice(accel_cali_data_buf, 0, tSize,  pageIndex, offset);
				return 1;
	}
	else
	{
				return 0;
	}
}

/*
 * handle for incoming sensor data
 *
 *  */
void handle_sensor_subscribe_data(struct cfw_message *msg)
{
	ss_sensor_subscribe_data_event_t *p_evt =
		(ss_sensor_subscribe_data_event_t *)msg;
	ss_sensor_data_header_t *p_data_header = &p_evt->sensor_data_header;
	uint8_t sensor_type = GET_SENSOR_TYPE(p_evt->handle);

	switch (sensor_type) {
	/* TODO: use a header from sensor service defining these structs:
	 * redefining message structure is not a good practice. */
	case SENSOR_PATTERN_MATCHING_GESTURE: {
		struct gs_personalize *p =
			(struct gs_personalize *)p_data_header->data;
		pr_info(LOG_MODULE_MAIN, "GESTURE=%d,size=%d",
			p->nClassLabel, p->size);
	}
	break;
	case SENSOR_ABS_TAPPING: {
		struct tapping_result *p =
			(struct tapping_result *)p_data_header->data;
		pr_info(LOG_MODULE_MAIN, "TAPPING=%d", p->tapping_cnt);
		handle_tap_count(p->tapping_cnt);
	}
	break;
	case SENSOR_ABS_STEPCOUNTER: {
		struct stepcounter_result *p =
			(struct stepcounter_result *)p_data_header->data;
		pr_info(LOG_MODULE_MAIN, "steps:%d  activity:%d",
			p->steps, p->activity);
	}
	break;
	case SENSOR_ACCELEROMETER:
	{
		int i = 0;
		static int accel_read_count = 0;
		int data_len = p_data_header->data_length;
		int data_num = data_len / sizeof(struct accel_phy_data);
		uint8_t data_type = ACCEL_DATA;
		if (accel_read_count++ >= 3) {
			ss_sensor_unsubscribe_data(sensor_handle, NULL,
						   p_evt->handle, &data_type, 1);
			accel_read_count = 0;
			break;
		}

		for (i = 0; i < data_num; i++) {
			struct accel_phy_data *p_data =
				(struct accel_phy_data *)p_data_header->data +
				i;
			pr_debug(
				LOG_MODULE_SS_SVC,
				"[APP][NUM=%d]stamp(ms):%d, x(mg):%d, y(mg):%d, z(mg):%d",
				i, p_data_header->timestamp, (p_data->ax),
				(p_data->ay), (p_data->az));
		}
	} break;
	case SENSOR_ALGO_DEMO: {
		struct demo_algo_result *p =
			(struct demo_algo_result *)p_data_header->data;
		pr_info(LOG_MODULE_MAIN, "Demo Algo Accelerometer data (mg): x:%d, y:%d, z:%d",
			p->ax, p->ay, p->az);
		pr_info(LOG_MODULE_MAIN, "Demo Algo Gyroscope data (m_degree/s): x:%d, y:%d, z:%d",
			p->gx, p->gy, p->gz);

#if defined(CONFIG_SERVICES_BLE_ISPP)
		uint8_t data_2_send[100] = {0};
		int ret = 0;
		int data_len = snprintf(data_2_send, 100, "Demo Algo Accelerometer data (mg): x:%d, y:%d, z:%d",
				p->ax, p->ay, p->az);
		if ((ret = send_data_to_ispp(data_2_send, data_len + 1)) != 0)
			pr_error(LOG_MODULE_BLE, "ble_service_ispp_write failed: %d.", ret);
		data_len = snprintf(data_2_send, 100, "Demo Algo Gyroscope data (m_degree/s): x:%d, y:%d, z:%d",
				p->gx, p->gy, p->gz);
		if ((ret = send_data_to_ispp(data_2_send, data_len + 1)) != 0)
			pr_error(LOG_MODULE_BLE, "ble_service_ispp_write failed: %d.", ret);
#endif
	}
	break;

	case SENSOR_ALGO_LAP_COUNT: {
			struct lap_count_algo_result *p =
				(struct lap_count_algo_result *)p_data_header->data;
			pr_info(LOG_MODULE_MAIN, "lap_count Algo Accelerometer data (mg): x:%d, y:%d, z:%d",
				p->ax, p->ay, p->az);
			pr_info(LOG_MODULE_MAIN, "lap_count Algo Gyroscope data (m_degree/s): x:%d, y:%d, z:%d",
				p->gx, p->gy, p->gz);


			//uint8_t data_2_send[100] = {0};
			if(p->peakDetected)
			{
				peakDetected = True;
			}
			//else
			//{
			//	peakDetected = False;
			//}
			uint8_t data_2_send[4] = {(uint8_t)p->ax, (uint8_t)p->ay, (uint8_t)p->az, p->lap_count};
			memcpy(accel_buffer_data_buf, data_2_send, sizeof(accel_buffer_data_buf));
			//accel_buffer_data_buf = {p->ax, p->ay, p->az};
			//int ret = 0;
			//int data_len = snprintf(data_2_send, 100, "lap_count Algo Accelerometer data (mg): x:%d, y:%d, z:%d",
			//		p->ax, p->ay, p->az);
			//if ((ret = send_data_to_ispp(data_2_send, data_len + 1)) != 0)
			//if ((ret = ble_set_remote_data(data_2_send, data_len + 1)) != 0)
			//if((ret = handle_ble_update_service_lap_count(p->lap_count))!=0)
			//if((ret = handle_ble_update_service_sport_data(data_2_send))!=0)
			//	pr_error(LOG_MODULE_BLE, "ble_update_service_lap_count failed: %d.", ret);
			//data_len = snprintf(data_2_send, 100, "lap_count Algo Gyroscope data (m_degree/s): x:%d, y:%d, z:%d",
					//p->gx, p->gy, p->gz);
			//if ((ret = send_data_to_ispp(data_2_send, data_len + 1)) != 0)
			//if ((ret = ble_set_remote_data(data_2_send, data_len + 1)) != 0)
			//if((ret = ble_update_lap_count()))!=0)
				//pr_error(LOG_MODULE_BLE, "ble_service_write failed: %d.", ret);

		}
		break;

	}
}

void handle_start_scanning_evt(struct cfw_message *msg)
{
	ss_sensor_scan_event_t *p_evt = (ss_sensor_scan_event_t *)msg;
	ss_on_board_scan_data_t on_board_data = p_evt->on_board_data;
	uint8_t sensor_type = p_evt->sensor_type;

	p_evt->handle = GET_SENSOR_HANDLE(sensor_type, on_board_data.ch_id);
	uint8_t data_type = ACCEL_DATA;
	switch (sensor_type) {
	//case SENSOR_PATTERN_MATCHING_GESTURE:
	//	ss_sensor_subscribe_data(sensor_handle, NULL, p_evt->handle,
	//				 &data_type, 1, 100, 10);

	//	break;
	case SENSOR_ABS_TAPPING:
		ss_sensor_subscribe_data(sensor_handle, NULL, p_evt->handle,
					 &data_type, 1, 100, 0);
		break;
	//case SENSOR_ABS_STEPCOUNTER:
	//	ss_sensor_subscribe_data(sensor_handle, NULL, p_evt->handle,
	//				 &data_type, 1, 33, 0);
	//	break;
	case SENSOR_ACCELEROMETER:
		if (set_accel_cali_data_flag == 1) {
			ss_sensor_set_calibration(
				sensor_handle, NULL, p_evt->handle,
				0, sizeof(accel_cali_data_buf),
				(uint8_t *)&accel_cali_data_buf[0]);
			set_accel_cali_data_flag = 0;
			break;
		}
		ss_sensor_subscribe_data(sensor_handle, NULL, p_evt->handle,
					 &data_type, 1, 1, 1000);
		break;
	case SENSOR_GYROSCOPE:
		if (set_gyro_cali_data_flag == 1) {
			ss_sensor_set_calibration(
				sensor_handle, NULL, p_evt->handle,
				0, sizeof(gyro_cali_data_buf),
				(uint8_t *)&gyro_cali_data_buf[0]);
			set_gyro_cali_data_flag = 0;
			break;
		}
		ss_sensor_subscribe_data(sensor_handle, NULL, p_evt->handle,
					 &data_type, 1, 1, 1000);
		break;
	//case SENSOR_ALGO_DEMO:
	//	ss_sensor_subscribe_data(sensor_handle, NULL, p_evt->handle,
	//				 &data_type, 1, 33, 0);
	//	break;
	case SENSOR_ALGO_LAP_COUNT:
		ss_sensor_subscribe_data(sensor_handle, NULL, p_evt->handle,
					 &data_type, 1, 33, 0);
		break;
	}
}

static void props_handle_connect_cb(svc_client_handle_t *handle, void *param)
{
	uint8_t accel_value[6] = { 0 };
	props_service_handle = handle;
	storage_service_properties_add(props_service_handle, 1,
				       ACCEL_CALI_DATA_NO,
				       &non_persist_storage_class,
				       accel_value,
				       sizeof(accel_value),
				       NULL);
}

void props_handle_msg(struct cfw_message *msg, void *data)
{
	uint8_t gyro_value[12] = { 0 };
	static uint8_t cnt;

	switch (CFW_MESSAGE_ID(msg)) {
	case MSG_ID_PROP_SERVICE_ADD_PROP_RSP: {
		if (cnt == 0)
			storage_service_properties_read(props_service_handle, 1,
							ACCEL_CALI_DATA_NO,
							NULL);
		else if (cnt == 1)
			storage_service_properties_read(props_service_handle, 1,
							GYRO_CALI_DATA_NO,
							NULL);
	}
	break;
	case MSG_ID_PROP_SERVICE_READ_PROP_RSP: {
		int read_prop =
			((read_property_rsp_msg_t *)msg)->rsp_header.status;
		if (read_prop == DRV_RC_OK) {
			int read_size_1 =
				((read_property_rsp_msg_t *)msg)->property_size;
			if (cnt == 0) {
				memcpy(
					accel_cali_data_buf,
					&((read_property_rsp_msg_t *)msg)->
					start_of_values, read_size_1);
				if (sensor_handle != NULL) {
					storage_service_properties_add(
						props_service_handle, 1,
						GYRO_CALI_DATA_NO,
						&non_persist_storage_class,
						gyro_value,
						sizeof(gyro_value), NULL);
					cnt++;
				} else {
					storage_service_properties_read(
						props_service_handle, 1,
						ACCEL_CALI_DATA_NO, NULL);
				}
			} else if (cnt == 1) {
				memcpy(
					gyro_cali_data_buf,
					&((read_property_rsp_msg_t *)msg)->
					start_of_values, read_size_1);
				set_gyro_cali_data_flag = 1;
				set_accel_cali_data_flag = 1;
				ss_start_sensor_scanning(sensor_handle, NULL,
							 GYRO_TYPE_MASK
							 | ACCEL_TYPE_MASK
							 );
			}
		}
	}
	break;
	case MSG_ID_PROP_SERVICE_WRITE_PROP_RSP:
	case MSG_ID_PROP_SERVICE_REMOVE_PROP_RSP:
	default:
		break;
	}
	cfw_msg_free(msg);
}

void sensing_start_sensor_scanning(svc_client_handle_t *handle, void *param)
{
	sensor_handle = handle;
	/*
	ss_start_sensor_scanning(handle, NULL,
				 PATTERN_MATCHING_TYPE_MASK
				 | TAPPING_TYPE_MASK
				 | STEPCOUNTER_TYPE_MASK
				 | ALGO_DEMO_MASK
				 | ALGO_LAP_COUNT_MASK
				 );
				 */
	ss_start_sensor_scanning(handle, NULL,
					 TAPPING_TYPE_MASK
					 | ALGO_LAP_COUNT_MASK
					 );
}

void sensing_init(cfw_handle_t *handle)
{
	T_QUEUE queue = get_service_queue();
	props_handle = cfw_init(queue, props_handle_msg, NULL);

	/* Open properties service when available */
	cfw_open_service_helper(props_handle, PROPERTIES_SERVICE_ID,
			props_handle_connect_cb, NULL);
	/* Open sensor service when available and start sensor scanning */
	cfw_open_service_helper(handle, ARC_SC_SVC_ID,
			sensing_start_sensor_scanning, NULL);
}

void UpdateLapCount(uint8_t count)
{
	lapCount = count;
}

void UpdateStartCountTime(uint32_t time)
{
	startCountTime = time;
}
