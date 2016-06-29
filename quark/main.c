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
#include <stdbool.h>
/* OS include, needed for task handling */
#include <zephyr.h>

/* Infra */
#include "infra/log.h"
#include "infra/ipc.h"
#include "infra/time.h"
#include "infra/pm.h"
#include "util/misc.h"

/* Service setup and general headers for this app */
#include "system_setup.h"
#include "cfw/cfw_messages.h"

/* Generic services API */
#include "services/service_queue.h"

/* GPIO service */
#include "services/gpio_service.h"

/* Battery service */
#include "services/battery_service_api.h"

/* UI Services configruation and helper */
#include "ui_config.h"

/* BLE services helper */
#include "ble_app.h"

/* Sensing demo helper */
#include "sensing.h"

/* Service counter helper */
#include "service_counter.h"

/* Watchdog helper */
#include "wdt_helper.h"

#include "agm_app.h"

/* OLED helper */
#include "ssd1360.h"

/* memory helper */
#include "DataFlash.h"

#include "mraa/spi.h"

static const uint8_t CHIP_SELECT =   5;
//static const uint8_t RESET =         6;
//static const uint8_t WRITE_PROTECT = 7;

#define CONFIG_SERVICES_SENSOR 1


/* List of services ids that are available in the application */
static uint16_t ref_app_services[] = {
	BLE_SERVICE_ID,
	BLE_CORE_SERVICE_ID,
	SS_GPIO_SERVICE_ID,
	SOC_GPIO_SERVICE_ID,
	SS_ADC_SERVICE_ID,
	LL_STOR_SERVICE_ID,
	BATTERY_SERVICE_ID,
#ifdef CONFIG_UI_SERVICE_IMPL
	UI_SVC_SERVICE_ID,
#endif
	PROPERTIES_SERVICE_ID,
#ifdef CONFIG_SERVICES_SENSOR
	ARC_SC_SVC_ID,
#endif
	AON_GPIO_SERVICE_ID,
};

/* System main queue it will be used on the component framework to add messages
 * on it. */
static T_QUEUE queue;
/* Main application handle for the component framework, it will be used to
 * reference the queue processing. */
static cfw_handle_t handle;

/** Client message handler for main application */
static void client_message_handler(struct cfw_message * msg, void * param)
{
	/* handle all ble messages : register service -> start advertisement -> */
	if (handle_ble_messages(msg))
		goto msg_free;
	switch (CFW_MESSAGE_ID(msg)) {
	case MSG_ID_CFW_SVC_AVAIL_EVT:
		/* Service counter handler will mark a service has started and
		 * print a message once all service have started. */
		service_counter_handler(
				((cfw_svc_available_evt_msg_t*) msg)->service_id,
				 ref_app_services, ARRAY_SIZE(ref_app_services));
		break;
#ifdef CONFIG_SERVICES_SENSOR
	case MSG_ID_SS_START_SENSOR_SCANNING_EVT:
		handle_start_scanning_evt(msg);
		break;
	case MSG_ID_SS_SENSOR_SUBSCRIBE_DATA_EVT:
		handle_sensor_subscribe_data(msg);
		break;
#endif
#ifdef CONFIG_UI_SERVICE_IMPL
	case MSG_ID_UI_BTN_SINGLE_EVT: {
		pm_shutdown(0);
	}
		break;
#endif
	}
msg_free:
	/* Free message once processed */
	cfw_msg_free(msg);
}

/* Services startup function */
static void services_setup(void)
{
	/* Init OLED display */
	init();

	 /* Initialize SPI for AT45DB641E*/
	//setup(CHIP_SELECT, AT45_RESET_PIN, AT45_WP_PIN);
	setup(CHIP_SELECT, -1, -1);
	mraa_spi_context spi = mraa_spi_init(0);
	mraa_spi_frequency(spi, 100);

#ifdef CONFIG_UI_SERVICE_IMPL
	/* UI service startup*/
	ui_service_start_helper(handle);
	pr_info(LOG_MODULE_MAIN, "ui service init in progress...");
#endif

	/* Gpio services startup */
	gpio_service_init(get_service_queue(), SOC_GPIO_SERVICE_ID);
	pr_info(LOG_MODULE_MAIN, "SOC GPIO service init in progress...");

	/* The task storage will init low level storage service and property
	 * service */
	//task_start(TASK_STORAGE);
	//pr_info(LOG_MODULE_MAIN, "Storage service init in progress...");

#ifdef CONFIG_SERVICES_QUARK_SE_BATTERY_IMPL
	/* Battery service initialization */
	bs_init(queue, BATTERY_SERVICE_ID);
	pr_info(LOG_MODULE_MAIN, "Battery service init in progress...");
#endif

#ifdef CONFIG_SERVICES_SENSOR
	/* Initialize sensing on active queue */
	sensing_init(handle);
	pr_info(LOG_MODULE_MAIN, "Sensing init in progress...");
#endif

	/* Reset BLE core, start registering BLE service with CFW */
	ble_start_app(handle);
	pr_info(LOG_MODULE_MAIN, "BLE service init in progress...");
        
    //led_start_app(handle);
    //pr_info(LOG_MODULE_MAIN, "LED App in progress...");

	/* Initialize service counter, to be able to notify when all services
	 * have started. */
	service_counter_start(handle, ref_app_services);
}

/* Application main entry point */
void main_task(void *param)
{
	printString("Initialization", 4, 3);

	uint8_t bufferNum = 0;
	uint8_t tSize = sizeof(accel_buffer_data_buf);
	uint8_t txData[DF_45_BUFFER_SIZE];
	int ret = 0;
	int read_result= 0;

	int readReTry = 0;
	int bleSendReTry = 0;
	/* count of wrote pages */
	uint16_t pageCount = 0;

	/* iterator for page read */
	uint16_t pageIterator =0;

	/* offset in latest wrote page  */
	uint16_t pageOffSet = 0;

	/* Hardware and OS setup */
	handle = system_setup(&queue, client_message_handler, NULL);
	/* System setvices startup */
	services_setup();
	pr_info(LOG_MODULE_MAIN, "Quark SE go to main loop");

	//time_last_ping = get_uptime_ms();
	//turnDetectionTime = get_uptime_ms();
	memset(txData, '\0', sizeof(txData));
	/* Loop to process message queue */
	while (1) {
		OS_ERR_TYPE err = E_OS_OK;
		//startCountTime = get_uptime_ms();
		/* Process message with a given timeout */
		queue_process_message_wait(queue, 5000, &err);

		/* print on screen when get double tap or a new lap count*/
		if(handle_double_tap())
		{
			DisplayActive();
			UpdateStartCountTime(get_uptime_ms());
			enable_update_lap_count(true);
		}
		else
		{
			DisplayIdle();
			UpdateLapCount(0);
			enable_update_lap_count(false);
		}

		/* Once peak detected, write sensor data into flash with continued pageCount and pageOffSet
		*/
		if(peakDetection())
		{
			handle_sensor_data_record(accel_buffer_data_buf ,&pageCount, &pageOffSet, tSize);
		}

		/* Update the sport data to mobile device once the BLE got connected
		 * and the readable data exists in flash
		*/
		if(ble_connected && (pageOffSet>0||pageCount>0))
		{
				if(bleSendReTry>=3)
				{
					pr_error(LOG_MODULE_BLE, "BLE send retry exceed three times");
				}
				if(readReTry>=3)
				{
					pr_error(LOG_MODULE_BLE, "ReadFromDevice retry exceed three times");
				}
				while(pageIterator<=pageCount )
				{
					if(*txData != '\0' && read_result==0)
					{
						/*send txData(256) bytes data to mobile device*/
						if((ret = handle_service_sport_data(txData, PACKAGE_COUNT))!=0)
						{
							pr_error(LOG_MODULE_BLE, "handle_service_sport_data failed: %d.", ret);
							bleSendReTry++;
						}
						else
						{
							bleSendReTry = 0;
						    memset(txData, '\0', sizeof(txData));
						}
					}
					else
						break;

					/* only read from device when the last send to mobile success */
					if(ret==0)
					{
						//read DF_45_BUFFER_SIZE(256) bytes data from data flash

						if((read_result=ReadFromDevice(txData, pageIterator, bufferNum))!=0)
						{
							pr_error(LOG_MODULE_BLE, "ReadFromDevice failed: %d.", read_result);
							readReTry++;
						}
						else
						{
							/* move to next page if the read success */
							pageIterator++;
							readReTry = 0;
						}

					}

				}
				pageCount = 0;
				pageOffSet = 0;
			}
		}


			/* Print a keep alive message every 30 seconds
			if (get_uptime_ms()-time_last_ping>30000)
			{
				pr_info(LOG_MODULE_MAIN, "Keep alive notification");
				time_last_ping = get_uptime_ms();
	#ifdef CONFIG_UI_SERVICE_IMPL
				led_blink();
	#endif
			}
	        */

		/* Acknowledge the system watchdog to prevent panic and reset */
		wdt_keepalive();


}



