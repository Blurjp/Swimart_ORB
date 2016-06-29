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

#include <string.h>

#include "infra/log.h"
#include "infra/factory_data.h"
#include "infra/version.h"
#include "curie_factory_data.h"
#include "services/service_queue.h"
#include "cfw/cfw.h"
#include "services/services_ids.h"
#include "cfw/cfw_client.h"
#include "agm_app.h"
#include "infra/tcmd/handler.h"
#include "machine.h"
#include "scss_registers.h"
#include "drivers/io_config.h"
#include "services/gpio_service.h"
#include "cfw/cfw_messages.h"
#include "os/os.h"

#ifdef CONFIG_SS_GPIO
#include "drivers/ss_gpio_iface.h"
#endif
#ifdef CONFIG_SOC_GPIO
#include "drivers/soc_gpio.h"
#endif

#define SOC_GPIO_OUTPUT_PIN_DIG13	2	// This GPIO is connected to LED and so this is OUTPUT Pin
#define SOC_GPIO_OUTPUT_PIN_DIG12		1	// This GPIO is connected to Button and so this is INPUT Pin

struct device *soc_dev_dig13;

gpio_cfg_data_t out_pin_cfg;
enum led_state {
    LED_ON,
    LED_OFF
} led_state = LED_OFF;

static void led_toggle_callback(bool state, void *arg)
{
    DRIVER_API_RC ret;

    if(LED_OFF == led_state) {
	ret =  soc_gpio_write(soc_dev_dig13, SOC_GPIO_OUTPUT_PIN_DIG13, 1);
        if(ret != DRV_RC_OK) {
             pr_error(LOG_MODULE_MAIN, "Error pin %d write return error (%d)", SOC_GPIO_OUTPUT_PIN_DIG13, ret);
    	}
        led_state = LED_ON;     
    }
    else {
        ret =  soc_gpio_write(soc_dev_dig13, SOC_GPIO_OUTPUT_PIN_DIG13, 0);
        if(ret != DRV_RC_OK) {
             pr_error(LOG_MODULE_MAIN, "Error pin %d write return error (%d)", SOC_GPIO_OUTPUT_PIN_DIG13, ret);
        }
        led_state = LED_OFF;
    }
}

static DRIVER_API_RC soc_gpio_output_pin_config(struct device *dev, unsigned int output_pin)
{
    DRIVER_API_RC ret;
 
    // Set the output pin configurations 
    out_pin_cfg.gpio_type = GPIO_OUTPUT;
    out_pin_cfg.int_type = LEVEL;
    out_pin_cfg.int_polarity = ACTIVE_LOW;
    out_pin_cfg.int_debounce = DEBOUNCE_OFF;
    out_pin_cfg.int_ls_sync = LS_SYNC_OFF;
    out_pin_cfg.gpio_cb = NULL;

    // Write the output pin configurations
    ret = soc_gpio_set_config(dev, output_pin, &out_pin_cfg);
    if(ret != DRV_RC_OK) {
        pr_error(LOG_MODULE_MAIN, "Error pin for LED %d config (%d)\n", output_pin, ret);
        soc_gpio_deconfig(dev, output_pin);
	return ret;
    }
    return DRV_RC_OK;
}

static DRIVER_API_RC soc_gpio_input_pin_config(struct device *dev, unsigned int input_pin)
{
    gpio_cfg_data_t in_pin_cfg;
    DRIVER_API_RC ret;
 
    // Set the input pin configurations 
    in_pin_cfg.gpio_type = GPIO_INTERRUPT;
    in_pin_cfg.int_type = DOUBLE_EDGE;
    in_pin_cfg.int_polarity = ACTIVE_HIGH;
    in_pin_cfg.int_debounce = DEBOUNCE_ON;
    in_pin_cfg.int_ls_sync = LS_SYNC_OFF;
    in_pin_cfg.gpio_cb = led_toggle_callback;

    // Write the input pin configurations
    ret = soc_gpio_set_config(dev, input_pin, &in_pin_cfg);
    if(ret != DRV_RC_OK) {
        pr_error(LOG_MODULE_MAIN, "Error pin for Button %d config (%d)", input_pin, ret);
	return ret;
    }

    return DRV_RC_OK;
}

void config_gpio_handler(svc_client_handle_t * p_svc_handle, void * param)
{
    DRIVER_API_RC ret = DRV_RC_FAIL;

    soc_dev_dig13 = get_device(SOC_GPIO_32_ID);
    if(soc_dev_dig13 == NULL)
    	pr_error(LOG_MODULE_MAIN, "LED GPIO get_device failed");

    /* Change Pin Mode of pin DIG13/GPIO2 to be as GPIO */
    SET_PIN_MODE(SOC_GPIO_OUTPUT_PIN_DIG13, QRK_PMUX_SEL_MODEA);

    ret = soc_gpio_output_pin_config(soc_dev_dig13, SOC_GPIO_OUTPUT_PIN_DIG13);
    if(ret != DRV_RC_OK)
        pr_error(LOG_MODULE_MAIN, "Test for LED gpio pin interrupt failed failed: %d", ret);

    struct device *soc_dev_dig12 = get_device(SOC_GPIO_32_ID);
    if(soc_dev_dig12 == NULL)
    	pr_error(LOG_MODULE_MAIN, "Button GPIO get_device failed");

    /* Change Pin Mode of pin DIG12/GPIO1 to be as GPIO */
    SET_PIN_MODE(SOC_GPIO_OUTPUT_PIN_DIG12, QRK_PMUX_SEL_MODEA);

    ret = soc_gpio_input_pin_config(soc_dev_dig12, SOC_GPIO_OUTPUT_PIN_DIG12);
    if(ret != DRV_RC_OK)
        pr_error(LOG_MODULE_MAIN, "Test for BUTTON gpio pin interrupt failed failed: %d", ret);

}

void led_start_app(cfw_handle_t handle)
{
        /* start LED app when GPIO_SERVICE is available */
        cfw_open_service_helper(handle, SOC_GPIO_SERVICE_ID,
                        config_gpio_handler, NULL);
}

