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

#include "system_setup.h"

#include "infra/ipc.h"
#include "infra/log.h"

#include "machine/soc/intel/quark_se/quark/soc_setup.h"
#include "machine.h"

/* USB application setup helper */
#include "usb_app.h"

/* Watchdog helper */
#include "wdt_helper.h"

/* Loggging support helper */
#include "log_helper.h"
#include "machine/soc/intel/quark_se/quark/log_backend_uart.h"

/* IPC setup helper */
#include "ipc_helper.h"

/* CFW setup helper */
#include "cfw_helper.h"
#include "cfw/cfw_service.h"

/* Test command client setup */
#include "machine/soc/intel/quark_se/quark/uart_tcmd_client.h"
#include "infra/tcmd/engine.h"

cfw_handle_t system_setup(T_QUEUE *q, handle_msg_cb_t cb, void *cb_data)
{
	cfw_handle_t handle;

	/* Initialize OS abstraction */
	os_init();

	/* Setup IPC and main queue */
	*q = ipc_setup();

	/* General hardware setup function, pass the ipc message handler if you
	 * want to use this feature. */
	soc_setup(ipc_handle_message);

	/* start Quark watchdog */
	wdt_start(WDT_MAX_TIMEOUT_MS);

	/* USB setup */
	usb_app_setup();

	/* Start log infrastructure */
	log_start(log_backend_uart);

	/* Enable test command engine async support through the main queue */
	tcmd_async_init(*q);
	/* Test commands will use the same port as the log system */
	set_tcmd_uart_port(CONFIG_UART_CONSOLE_INDEX);

	/* Component framework and services setup */
	handle = cfw_setup(*q, cb, NULL);
	pr_info(LOG_MODULE_MAIN, "cfw init done");

	/* Initialize ARC shared structure and start it. */
	shared_data->ports = port_get_port_table();
	shared_data->services = services;
	shared_data->service_mgr_port_id = cfw_get_service_mgr_port_id();
	start_arc(0);

	return handle;
}
