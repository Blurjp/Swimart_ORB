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

#include "ui_config.h"
#include "services/service_queue.h"
#include "util/misc.h"
#include "lib/button/button_gpio_svc.h"
#ifdef CONFIG_LED
#include "drivers/led/led.h"
#include "lib/led/led_gpio_svc.h"
#endif

struct ui_config ui_config_ref_app = {
#ifdef CONFIG_BUTTON
	.btns = (struct button[]) {
		[PWR_BTN_ID] = {
			.init = button_gpio_svc_init,
			.shutdown = button_gpio_svc_shutdown,
			.priv = &(struct button_gpio_svc_priv) {
				.gpio_service_id = AON_GPIO_SERVICE_ID,
				.pin = 0,
				.active_low = true,
			},
			.timing	= {
				.double_press_time = 250,
				.max_time = 3000,
			},
			.press_mask = SINGLE_PRESS
		}
	},
	.btn_count = UI_BUTTON_COUNT,
#endif
#ifdef CONFIG_LED
	.led_count = 1,
#ifdef CONFIG_GPIO_LED
	.leds = (struct led[]) {
		[0] = {
			.id = 0,
			.priv = &(struct led_gpio_svc_priv) {
				.gpio_service_id = SOC_GPIO_SERVICE_ID,
				.pin = 8,
				.active_high = true,
			},
		},
	}
#else
	.leds = NULL,
#endif
#endif
};

#ifdef CONFIG_LED
static led_s led_pattern = {
	.id = 0,
	.repetition_count = 0,
	.intensity = 255,
	.duration = {
		{1000,1000},
		{0,0},
		{0,0}
	},
#ifdef CONFIG_LED_MULTICOLOR
	.rgb = {
		{0, 255, 0},
		{0, 0, 0},
		{0, 0, 0},
	}
#endif
};
#endif

static svc_client_handle_t * ui_handle = NULL;

/* We don't do anything in the callback... */
static inline void ui_connect_cb(svc_client_handle_t * handle, void * param)
{
	ui_handle = handle;
}

void ui_service_start_helper(cfw_handle_t handle)
{
	int client_events[1] = {MSG_ID_UI_BTN_SINGLE_EVT};
	ui_service_init(get_service_queue(), &ui_config_ref_app);

	cfw_open_service_helper_evt(handle, UI_SVC_SERVICE_ID, client_events,
			ARRAY_SIZE(client_events),
			ui_connect_cb, NULL);
}

void led_blink(void) {
#ifdef CONFIG_LED
	if(ui_handle)
		ui_play_led_pattern(ui_handle, 0, LED_FIXED, &led_pattern, NULL);
#endif
}
