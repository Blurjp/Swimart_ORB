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

#include "ble_app.h"

#include <string.h>
#include <stdbool.h>

#include "services/ble/ble_service_api.h"
#include "services/ble/ble_service_utils.h"
#include "infra/log.h"
#include "services/ble/ble_protocol.h"
#include "infra/factory_data.h"
#include "infra/version.h"
#include "curie_factory_data.h"
#include "services/ble/ble_service_api.h"
#include "services/service_queue.h"
#include "ble_core_interface.h"
#include "services/ble/ble_service_utils.h"
#include "services/ble/ble_service_dis.h"
#include "services/ble/ble_service_bas.h"
#include "services/ble/ble_service_nservice.h"
#include "services/ble/ble_service_lap_count.h"
#if defined(CONFIG_SERVICES_BLE_ISPP)
#include "services/ble/ble_service_ispp.h"
#include "services/ble/ble_ispp.h"
#endif

//#define ISPP_TEST /* ruggedize ispp test */

/* Default device name */
#define BLE_DEV_NAME "Swimart"

/* Connection parameters used for Peripheral Preferred Connection Parameters (PPCP) and update request */
#define MIN_CONN_INTERVAL MSEC_TO_1_25_MS_UNITS(80)
#define MAX_CONN_INTERVAL MSEC_TO_1_25_MS_UNITS(151)
#define SLAVE_LATENCY 0
#define CONN_SUP_TIMEOUT MSEC_TO_10_MS_UNITS(6000)


#define DIS_SVC 0x0
#define BAS_SVC 0x1
#define ISPP_SVC 0x2
#define NAS_SVC 0x3
#define SVC_LAST 0x4
#define LAP_SVC 0x5

#define BLE_DATA_PACAKGE_LENGTH 4

#define MANUFACTURER_NAME "IntelCorp"
#define MODEL_NUM         "Curie"
#define SERIAL_NUM         "0123456789abcdef0123"
#define HARDWARE_REV       "1.0"

struct _ble_register_svc {
	void (*func_cback)(void *); /**< callback function to execute on MSG_ID_xxx_RSP */
	int cback_param; /**< parameter to pass to above function */
	void (*start_adv)(uint8_t adv_type); /**< Start advertisement */
};

static struct _ble_app_cb {
	svc_client_handle_t *p_svc_h;
	uint16_t conn_handle;
#if !defined(CONFIG_BLE_DEVICE_MANAGER)
	ble_addr_t bd_addr;
#endif
} _ble_app_cb = { 0 };

#ifdef ISPP_TEST
struct ispp_test_inout {
	uint16_t msg_no;     /* asked messages to be sent from host */
	uint16_t msg_len;    /* len of tx/rx messages */
	uint32_t delay;      /* delay between tx messages */
	int msg_i;           /* index of msg which is tried to send */
	T_TIMER ble_app_send_data_timer0;
	T_TIMER ble_app_send_data_timer;
	uint8_t data[135];
};
struct ispp_test_inout inout = {};
#endif

static void fill_dis_information(struct ble_service_dis_param *params)
{
	params->manufacter_name.p_string = (uint8_t *)MANUFACTURER_NAME;
	params->manufacter_name.len = strlen(MANUFACTURER_NAME);

	params->model_number.p_string = (uint8_t *)MODEL_NUM;
	params->model_number.len = strlen(MODEL_NUM);

	params->hw_revision.p_string = (uint8_t *)HARDWARE_REV;
	params->hw_revision.len = strlen(HARDWARE_REV);

	params->serial_number.p_string = (uint8_t *)SERIAL_NUM;
	params->serial_number.len = strlen(SERIAL_NUM);

	if (NULL != params->fw_revision.p_string) {
		snprintf((char *)params->fw_revision.p_string, BLE_DIS_MAX_CHAR_STR_LEN + 1,
					"%.20s", version_header.version_string);
		params->fw_revision.len = strlen(params->fw_revision.p_string);
	}

	if (NULL != params->sw_revision.p_string) {
		snprintf((char *)params->sw_revision.p_string, BLE_DIS_MAX_CHAR_STR_LEN,
				"%d.%d.%d-%02x%02x%02x%02x", version_header.major,
				version_header.minor, version_header.patch,
				version_header.hash[0], version_header.hash[1],
				version_header.hash[2], version_header.hash[3]);
		params->sw_revision.p_string[BLE_DIS_MAX_CHAR_STR_LEN-1] = '\0';
		params->sw_revision.len = strlen(params->sw_revision.p_string);
	}

#ifdef BLE_DIS_SYSTEM_ID
	if (NULL != params->p_system_id) {
		params->p_system_id->manufact_id = MANUFACTURER_ID;
		params->p_system_id->org_unique_id = ORG_UNIQUE_ID;
	}
#endif
}

void ble_start_app_handler(svc_client_handle_t * p_svc_handle, void * param)
{
	struct ble_enable_config en_config = { 0, };
	const struct ble_gap_connection_params conn_params =
	    {MIN_CONN_INTERVAL, MAX_CONN_INTERVAL, SLAVE_LATENCY, CONN_SUP_TIMEOUT};
	ble_addr_t bda;

	if (NULL != _ble_app_cb.p_svc_h) {
		pr_warning(LOG_MODULE_MAIN, "ble_start_app() "
				"failed with: ret: %d", E_OS_ERR);
		return;
	}
	_ble_app_cb.p_svc_h = p_svc_handle;
	pr_info(LOG_MODULE_MAIN, "[NAGESH] ........................BLE_Handle is %ul",p_svc_handle);

	en_config.p_name = (uint8_t *) BLE_DEV_NAME;
	en_config.appearance = BLE_GAP_APPEARANCE_TYPE_GENERIC_WATCH;
	en_config.central_conn_params = en_config.peripheral_conn_params = conn_params;

	if (!strncmp((char*)global_factory_data->oem_data.magic, FACTORY_DATA_MAGIC, 4)) {
		struct curie_oem_data *p_oem =
				(struct curie_oem_data *) &global_factory_data->oem_data.project_data;
		if (p_oem->bt_mac_address_type < 2) {
			bda.type = p_oem->bt_mac_address_type;
			memcpy(bda.addr, &p_oem->bt_address[0], BLE_ADDR_LEN);
			en_config.p_bda = &bda;
		}
		/* Sanity check in case only bd address is set. only use
		 * printable chars */
		if (' ' <= p_oem->ble_name[0] && 0xff > p_oem->ble_name[0]) {
			en_config.p_name = p_oem->ble_name;
		}
	}

	if (E_OS_OK != ble_enable(_ble_app_cb.p_svc_h, 1, &en_config,
			&_ble_app_cb)) {
		pr_error(LOG_MODULE_MAIN, "BLE device name too long! BLE ENABLE FAILED");
		return;
	}

	/* List of events to receive */
	int client_events[] = { MSG_ID_BLE_CONNECT_EVT,
			MSG_ID_BLE_DISCONNECT_EVT,
			MSG_ID_BLE_SECURITY_EVT,
			MSG_ID_BLE_ADV_TO_EVT,
			MSG_ID_BLE_PROTOCOL_EVT
	};
	cfw_register_events(_ble_app_cb.p_svc_h, client_events,
			sizeof(client_events) / sizeof(int), &_ble_app_cb);
}

static void _ble_register_services(void *p_priv)
{
	struct _ble_register_svc *p_reg = (struct _ble_register_svc *)p_priv;
	struct ble_service_dis_param dis_info = {{0}};
	uint8_t fw_version_string[BLE_DIS_MAX_CHAR_STR_LEN + 1];
	uint8_t sw_version_string[BLE_DIS_MAX_CHAR_STR_LEN];
	uint8_t reg_param;
#ifdef BLE_DIS_SYSTEM_ID
	struct ble_dis_system_id system_id;
#endif

	reg_param = p_reg->cback_param++;

	switch (reg_param) {
	case DIS_SVC: {
		dis_info.fw_revision.p_string = fw_version_string;
		dis_info.sw_revision.p_string = sw_version_string;
#ifdef BLE_DIS_SYSTEM_ID
		dis_info.p_system_id = &system_id;
#endif
		fill_dis_information(&dis_info);

		ble_init_service_dis(_ble_app_cb.p_svc_h, &dis_info, p_reg);
		pr_info(LOG_MODULE_MAIN, "ble registering DIS");
		}
		break;
	case BAS_SVC:
		ble_init_service_bas(_ble_app_cb.p_svc_h, NULL, p_reg);
		pr_info(LOG_MODULE_MAIN, "ble registering BAS");
		break;
	case ISPP_SVC:
#if defined(CONFIG_SERVICES_BLE_ISPP)
	{
		ble_init_service_ispp(_ble_app_cb.p_svc_h, NULL, p_reg);
		pr_info(LOG_MODULE_MAIN, "ble registering ISPP");
	}
		break;
#endif
	case NAS_SVC:{
			ble_init_service_nas(_ble_app_cb.p_svc_h, NULL, p_reg);
			pr_info(LOG_MODULE_MAIN, "ble registering NAS");
			}
			break;
	case LAP_SVC:{
		    // implement registration in ble_service_lap_count.c
			ble_init_service_lap_count(_ble_app_cb.p_svc_h, NULL, p_reg);
			pr_info(LOG_MODULE_MAIN, "ble registering LAP");
			}
			break;
	case SVC_LAST:{
			ble_get_security_status(_ble_app_cb.p_svc_h,
				BLE_SEC_BONDING_DB_STATE, NULL, p_priv);
			break;
		}
	}
}

static void _ble_start_advertisement(uint8_t adv_type)
{
	pr_info(LOG_MODULE_MAIN, "_ble_start_advertisement: adv_type:0x%x",
			adv_type);
	struct ble_adv_data_params params;
	uint8_t ad[BLE_MAX_ADV_SIZE];
	uint8_t *p = ad;

	params.adv_type = BLE_GAP_ADV_TYPE_ADV_IND;
	params.sd_len = 0;
	params.ad_len = 0;
	params.p_le_addr = NULL;
	params.p_sd = NULL;
	params.p_ad = ad;

	uint8_t flag = (adv_type & BLE_NON_DISC_ADV) ?
			BLE_SVC_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED :
			BLE_SVC_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	p += ble_enc_adv_flags(p, flag);

	p += ble_enc_adv_appearance(p, BLE_GAP_APPEARANCE_TYPE_GENERIC_WATCH);

	p += ble_enc_adv_manuf(p, INTEL_MANUFACTURER, NULL, 0);

	params.ad_len = p - params.p_ad;

	/* The name of the device will be added later */
	ble_start_advertisement(_ble_app_cb.p_svc_h, adv_type, &params, NULL);
	return;
}

/** Handles BLE enable message */
static void handle_msg_id_ble_enable_rsp(struct cfw_message *msg)
{
	struct ble_enable_rsp *rsp = (struct ble_enable_rsp *)msg;

	if (BLE_STATUS_SUCCESS == rsp->status) {
		struct _ble_register_svc *p_reg = balloc(sizeof(*p_reg), NULL);

		p_reg->func_cback = _ble_register_services;
		p_reg->cback_param = DIS_SVC;
		p_reg->start_adv = _ble_start_advertisement;

		/* start registering of the first BLE service (DIS) */
		_ble_register_services(p_reg);

		pr_info(LOG_MODULE_MAIN, "ble_enable_rsp: addr type: %d, "
				"BLE addr: %.2x:%.2x:%.2x:%.2x:%.2x:%.2x",
				rsp->bd_addr.type,
				rsp->bd_addr.addr[5], rsp->bd_addr.addr[4],
				rsp->bd_addr.addr[3], rsp->bd_addr.addr[2],
				rsp->bd_addr.addr[1], rsp->bd_addr.addr[0]);
	} else {
		pr_error(LOG_MODULE_MAIN, "ble_enable_rsp: Failed status: %d",
				rsp->status);
	}
}

/** Handles BLE init service message */
static void handle_msg_id_ble_init_svc_rsp(struct cfw_message *msg)
{
	struct _ble_register_svc *p_reg =
			(struct _ble_register_svc *)CFW_MESSAGE_PRIV(msg);
	struct ble_init_service_rsp * rsp = (struct ble_init_service_rsp *)msg;
	if (BLE_STATUS_SUCCESS == rsp->status)
		p_reg->func_cback(p_reg);
	else
		pr_error(LOG_MODULE_MAIN, "ble_init_service() failed: %d",
				rsp->status);
}

/** Handles BLE start advertisement message */
static void handle_msg_id_ble_start_adv_rsp(struct cfw_message *msg)
{
	struct ble_rsp * rsp = (struct ble_rsp *)msg;
	if (BLE_STATUS_SUCCESS != rsp->status)
		pr_error(LOG_MODULE_MAIN, "ble_start_adv() failed: %d",
				rsp->status);
}

/**
static void send_data_handler(void *priv)
{
	int err_code = 0;
	uint8_t response[2], *p = response;

	UINT8_TO_LESTREAM(p,  0x4E);    // 'N'
    UINT8_TO_LESTREAM(p, 0x45);    // 'E'

	pr_debug(LOG_MODULE_BLE, "inside send_data_handler");
	//err_code = ble_service_nas_write(_ble_app_cb.p_svc_h,
	//			 sizeof(pkt1),pkt1);

	err_code = ble_service_nas_write(_ble_app_cb.p_svc_h,
			 sizeof(response),response);

	if (err_code != BLE_SVC_GATT_STATUS_SUCCESS) {
		pr_info(LOG_MODULE_BLE, "QWERTY Failed sending back data: %u", err_code);
	}

}
*/

#ifdef ISPP_TEST
#define BUFFER_LEN 20
static void send_data_handler(void *priv)
{
	int err_code;

	pr_debug(LOG_MODULE_BLE, "msg no %d", inout.msg_i);
	err_code = ble_service_ispp_write(_ble_app_cb.p_svc_h,
			_ble_app_cb.conn_handle, inout.msg_len, inout.data);

	if (err_code != BLE_SVC_GATT_STATUS_SUCCESS) {
		pr_info(LOG_MODULE_BLE, "QWERTY Failed sending back data: %u", err_code);
	}

	inout.msg_i++;

	if (!(inout.msg_i % inout.msg_no)) {
		timer_delete(inout.ble_app_send_data_timer0, NULL);
		timer_delete(inout.ble_app_send_data_timer, NULL);
	}
}

static void send_data_handler0(void *priv)
{
	inout.ble_app_send_data_timer = timer_create(send_data_handler, NULL, inout.delay, true, true, NULL);
}
#endif

/** Handles BLE disconnect message */
static void handle_msg_id_ble_connect_evt(struct cfw_message *msg)
{
#if !defined(CONFIG_BLE_DEVICE_MANAGER)
	/* nothing to do. Store BD Addr */
	struct ble_connect_evt *evt = (struct ble_connect_evt *)msg;
	memcpy(&_ble_app_cb.bd_addr, &evt->ble_addr, sizeof(ble_addr_t));
#endif
	union ble_set_sec_params params;
	params.dev_status = 0; /* disable pairing state in case it was on */
	ble_set_security_status(_ble_app_cb.p_svc_h, BLE_SEC_DEVICE_STATUS,
			&params, NULL);
	ble_connected = true;
	pr_info(LOG_MODULE_MAIN, "BLE connected(conn_h: %d, role: %d)",
			evt->conn_handle, evt->role);
}

/** Handles BLE disconnect message */
static void handle_msg_id_ble_disconnect_evt(struct cfw_message *msg)
{
	struct _ble_register_svc *p_reg = balloc(sizeof(*p_reg), NULL);

	p_reg->func_cback = NULL;
	p_reg->cback_param = 0;
	p_reg->start_adv = _ble_start_advertisement;

	/* restart advertisement on disconnect depending if bonded device are there or not */
	ble_get_security_status(_ble_app_cb.p_svc_h,
			BLE_SEC_BONDING_DB_STATE, NULL, p_reg);
	ble_connected = false;
	pr_info(LOG_MODULE_MAIN, "BLE disconnected(conn_h: %d, hci_reason: 0x%x)",
			((struct ble_disconnect_evt *)msg)->conn_handle,
			((struct ble_disconnect_evt *)msg)->reason);
}

/** Handles BLE get security message */
static void handle_msg_id_ble_get_security_rsp(struct cfw_message *msg)
{
	struct ble_get_security_rsp *rsp = (struct ble_get_security_rsp *) msg;
	struct _ble_register_svc *p_reg =
			(struct _ble_register_svc *)CFW_MESSAGE_PRIV(msg);

	if (BLE_SEC_BONDING_DB_STATE == rsp->op_code) {
		if (BLE_SEC_ST_BONDED_DEVICES_AVAIL == (rsp->dev_status &
				BLE_SEC_ST_BONDED_DEVICES_AVAIL)) {
			/* already bonded */
			p_reg->start_adv(BLE_NON_DISC_ADV);
			bfree(p_reg);
		} else if (BLE_SEC_ST_NO_BONDED_DEVICES == (rsp->dev_status &
				BLE_SEC_ST_NO_BONDED_DEVICES)) {
			union ble_set_sec_params params;
			params.dev_status = BLE_SEC_ST_PAIRABLE;
			ble_set_security_status(_ble_app_cb.p_svc_h,
					BLE_SEC_DEVICE_STATUS, &params,
					p_reg);
		}
	} else {
		/* TODO: handle other op_code's */
		pr_error(LOG_MODULE_MAIN, "Wrong BLE security op: %d, state %d",
				rsp->dev_status);
	}
}

/** Handles BLE set security message */
static void handle_msg_id_ble_set_security_rsp(struct cfw_message *msg)
{
	struct ble_set_security_rsp * rsp = (struct ble_set_security_rsp *) msg;
	struct _ble_register_svc *p_reg =
				(struct _ble_register_svc *)CFW_MESSAGE_PRIV(msg);
	if (BLE_SEC_DEVICE_STATUS == rsp->op_code) {
		if (BLE_SEC_ST_PAIRABLE ==
				(rsp->dev_status & BLE_SEC_ST_PAIRABLE)) {
			/* start advertise all services at boot */
			p_reg->start_adv(BLE_NO_ADV_OPT);
			/* TODO: call UI function */
			bfree(p_reg);
			pr_info(LOG_MODULE_MAIN, "Device in PAIRABLE state");
		} else {
			pr_info(LOG_MODULE_MAIN, "Device in NON pairable state");
			/* TODO: call UI function, eg. LED pattern */
		}
	} else {
		/* TODO: handle other op_code's */
		pr_error(LOG_MODULE_MAIN, "Wrong BLE security op: %d, state %d",
				rsp->op_code, rsp->dev_status);
	}
}

/** Handles BLE security event */
static void handle_msg_id_ble_security_evt(struct cfw_message *msg)
{
	/* TODO: handle pairing/security */
}

/** Handles BLE stop advertisement event */
static void handle_msg_id_ble_adv_to_evt(struct cfw_message *msg)
{
	/* when advertisement timedout, start slow advertising without timeout */
	_ble_start_advertisement(BLE_SLOW_ADV | BLE_NO_ADV_TO);
}

#ifdef CONFIG_SERVICES_BLE_ISPP

#define LOOPBACK_CHANNEL 0x1F
#define REF_APP_ISPP_HEADER_SIZE 3

static int _check_status(int32_t status)
{
	switch (status) {
	case 0:
		break;
	case BLE_SVC_GATT_STATUS_WRONG_STATE:
		pr_debug(LOG_MODULE_BLE, "ISPP wrong state. ");
		break;
	case BLE_SVC_GATT_STATUS_INVALID_ATTR_LEN:
		pr_debug(LOG_MODULE_BLE, "ISPP Exceeding max MTU (133 bytes). ");
		break;
	case BLE_SVC_GATT_STATUS_INSUF_RESOURCE:
		pr_debug(LOG_MODULE_BLE, "ISPP Not enough memory to store the message. ");
		break;
	case BLE_SVC_GATT_STATUS_BUSY:
		pr_debug(LOG_MODULE_BLE, "ISPP Max pending number of messages has been reached. ");
		break;
	case BLE_SVC_GATT_STATUS_NOT_FOUND:
		pr_debug(LOG_MODULE_BLE, "ISPP Read: empty queue. ");
		break;
	default:
		pr_debug(LOG_MODULE_BLE, "ISPP: ERROR: %x ", status);
	}
	return status;
}

void ble_protocol_ispp_read_write_back(struct cfw_message *msg)
{
	struct ble_protocol_rsp *prot_rsp = (__typeof__(prot_rsp))msg;

	pr_debug(LOG_MODULE_BLE, "%s: BLE_ISPP_READ_REQ message was received: len = %d",
			__func__,
			prot_rsp->len);
	if (_check_status(prot_rsp->status) == BLE_STATUS_SUCCESS) {
		uint32_t err_code = BLE_STATUS_SUCCESS;
#ifndef ISPP_TEST
		err_code = ble_service_ispp_write(_ble_app_cb.p_svc_h,
				_ble_app_cb.conn_handle,
				prot_rsp->len,
				prot_rsp->data);
#else
		uint8_t *p, channel;
		p = prot_rsp->data;
		LESTREAM_TO_UINT8(p, channel);
		switch (channel) {
		case LOOPBACK_CHANNEL:
			err_code = ble_service_ispp_write(_ble_app_cb.p_svc_h,
					_ble_app_cb.conn_handle,
					prot_rsp->len,
					prot_rsp->data);
			break;
		case 0x2F: /* rx only */
			break;
		case 0x3F: { /* tx only start test */
			LESTREAM_TO_UINT16(p, inout.msg_no);
			LESTREAM_TO_UINT16(p, inout.msg_len);
			LESTREAM_TO_UINT32(p, inout.delay);
			inout.msg_i = 0;
			inout.ble_app_send_data_timer0	= timer_create(send_data_handler0, NULL, 2000, false, true, NULL);
			}
			break;
		default:
			pr_error(LOG_MODULE_BLE, "Ispp wr_back default ch: %x",
					channel);
		}

#endif
		if (err_code != BLE_SVC_GATT_STATUS_SUCCESS) {
			pr_error(LOG_MODULE_BLE, "Failed sending back data: %u",
					err_code);
		}
	}
}
#endif

/** Handles BLE protocol message */
static void handle_msg_id_ble_protocol_rsp(struct cfw_message *msg)
{
	struct ble_protocol_rsp *prot_rsp = (__typeof__(prot_rsp))msg;

	switch (CFW_MESSAGE_ID(msg)) {
	case MSG_ID_BLE_PROTOCOL_RSP:
		switch (prot_rsp->protocol) {
#ifdef CONFIG_SERVICES_BLE_ISPP
			case BLE_PROTOCOL_ISPP: {
				switch (prot_rsp->req) {
				case BLE_ISPP_WRITE_REQ:
					pr_debug(LOG_MODULE_BLE, "BLE_ISPP_WRITE_REQ received");
					_check_status(prot_rsp->status);
					break;
				case BLE_ISPP_READ_REQ:
					pr_debug(LOG_MODULE_BLE, "BLE_ISPP_READ_REQ received");
					ble_protocol_ispp_read_write_back(msg);
					break;
				default:
					pr_info(LOG_MODULE_BLE,"%s: DEFAULT CASE prot_rsp->ispp.req  = %d",
							__func__, prot_rsp->req);
					break;
				}
			}
#endif
		}
	}
}

/** Handles BLE protocol event */
static void handle_msg_id_ble_protocol_evt(struct cfw_message *msg)
{
	struct ble_protocol_evt *prot_evt = (struct ble_protocol_evt *)msg;

	switch (CFW_MESSAGE_ID(msg)) {
	case MSG_ID_BLE_PROTOCOL_EVT:
		switch (prot_evt->protocol) {
#ifdef CONFIG_SERVICES_BLE_ISPP
		case BLE_PROTOCOL_ISPP:
			prot_evt = (struct ble_protocol_evt *)msg;
			pr_debug(LOG_MODULE_BLE, "%s: MSG_ID_BLE_PROTOCOL_EVT received", __func__);
			pr_debug(LOG_MODULE_BLE, "Ispp Event: %d", prot_evt->event);
			switch (prot_evt->event) {
			case BLE_ISPP_RX_COMPLETE:
				pr_debug(LOG_MODULE_BLE, "%s: ISPP RX_COMPLETE: data %d", __func__, prot_evt->len);
				ble_service_ispp_read(_ble_app_cb.p_svc_h, prot_evt->len);
				break;
			case BLE_ISPP_OPEN:
				pr_debug(LOG_MODULE_BLE, "BLE_ISPP_OPEN event");
				break;
			case BLE_ISPP_CLOSE:
				pr_debug(LOG_MODULE_BLE, "BLE_ISPP_CLOSE event");
				break;
			case BLE_ISPP_TX_COMPLETE:
				cfw_send_message(prot_evt->context);
				pr_debug(LOG_MODULE_BLE, "BLE_ISPP_TX_COMPLETE event");
				break;
			}
#endif
			break;
		}
	}
}


bool handle_ble_messages(struct cfw_message * msg)
{
	switch (CFW_MESSAGE_ID(msg)) {
	case MSG_ID_BLE_ENABLE_RSP:
		handle_msg_id_ble_enable_rsp(msg);
		break;
	case MSG_ID_BLE_INIT_SVC_RSP:
		handle_msg_id_ble_init_svc_rsp(msg);
		break;
	case MSG_ID_BLE_START_ADV_RSP:
		handle_msg_id_ble_start_adv_rsp(msg);
		break;
	case MSG_ID_BLE_GET_SECURITY_RSP:
		handle_msg_id_ble_get_security_rsp(msg);
		break;
	case MSG_ID_BLE_SET_SECURITY_RSP:
		handle_msg_id_ble_set_security_rsp(msg);
		break;
	case MSG_ID_BLE_PROTOCOL_RSP:
		handle_msg_id_ble_protocol_rsp(msg);
		break;
	case MSG_ID_BLE_CONNECT_EVT:
		handle_msg_id_ble_connect_evt(msg);
		//		pr_debug(LOG_MODULE_BLE, "[NAGESH]..................Before starting the timer");
		//		timer_create(send_data_handler, NULL, 5000, false, true, NULL);
		break;
	case MSG_ID_BLE_DISCONNECT_EVT:
		handle_msg_id_ble_disconnect_evt(msg);
		break;
	case MSG_ID_BLE_SECURITY_EVT:
		handle_msg_id_ble_security_evt(msg);
		break;
	case MSG_ID_BLE_ADV_TO_EVT:
		handle_msg_id_ble_adv_to_evt(msg);
		break;
	case MSG_ID_BLE_PROTOCOL_EVT:
		handle_msg_id_ble_protocol_evt(msg);
		break;
	default:
		return false;
	}
	return true;
}

void ble_start_app(cfw_handle_t handle)
{
	/* Reset the nordic to force sync - Warning: JTAG debugger may prevent reset! */
	ble_core_reset();
	uart_ipc_set_channel(uart_ipc_channel_open(SYNC_CHANNEL, uart_ipc_message_cback));

	ble_cfw_service_init(get_service_queue());

	/* start BLE app when BLE_SERVICE is available */
	cfw_open_service_helper(handle, BLE_SERVICE_ID,
			ble_start_app_handler, NULL);
	//timer_create(send_data_handler, NULL, 30000, true, true, NULL);
}

void enable_update_lap_count(bool flag)
{
	ble_service_set_lap_count_updates(flag);
}


/**
        * Split data into packageCount size data and send it to mobile
        * @param data Data from sensor to send
**/
int handle_service_sport_data(uint8_t* data, uint8_t packageCount)
{
	uint8_t index = 0;
	uint8_t size = sizeof(data);
	uint8_t txData[packageCount];
	int ret = 0;
	while(index < size)
	{
		if(index+packageCount<size)
		{
			memcpy(txData, data, packageCount);
		}
		else
		{
			memcpy(txData, data, size-index);
		}

		if((ret = handle_ble_update_service_sport_data(txData))!=0)
		{
			pr_error(LOG_MODULE_BLE, "handle_service_sport_data failed: %d.", ret);
			break;
		}
		data+=packageCount;
		index+=packageCount;
	}
	return ret;
	//return (handle_ble_update_service_sport_data(accel_buffer_data_buf));
}


#if defined(CONFIG_SERVICES_BLE_ISPP)
int send_data_to_ispp(uint8_t *buf, uint16_t len)
{
	return ble_service_ispp_write(_ble_app_cb.p_svc_h, _ble_app_cb.conn_handle,
		len, buf);
}
#endif
