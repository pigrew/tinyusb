/*
 * usbtmc_device.h
 *
 *  Created on: Sep 10, 2019
 *      Author: nconrad
 */
/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 N Conrad
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */


#ifndef CLASS_USBTMC_USBTMC_DEVICE_H_
#define CLASS_USBTMC_USBTMC_DEVICE_H_

#include "usbtmc.h"

// Enable 488 mode by default
#if !defined(CFG_USBTMC_CFG_ENABLE_488)
#define CFG_CFG_USBTMC_CFG_ENABLE_488 (1)
#endif

// USB spec says that full-speed must be 8,16,32, or 64.
// However, this driver implementation requires it to be >=32
#define USBTMCD_MAX_PACKET_SIZE (64u)

/***********************************************
 *  Functions to be implemeted by the class implementation
 */

#if (CFG_USBTMC_CFG_ENABLE_488)
extern usbtmc_response_capabilities_488_t const tud_usbtmc_app_capabilities;
#else
extern usbtmc_response_capabilities_t const tud_usbtmc_app_capabilities;
#endif

// In order to proceed, app must call call usbtmcd_start_bus_read(rhport) during or soon after:
// * tud_usbtmc_app_open_cb
// * tud_usbtmc_app_msg_data_cb
// * tud_usbtmc_app_msgBulkIn_complete_cb
// * tud_usbtmc_app_msg_trigger_cb
// * (successful) tud_usbtmc_app_check_abort_bulk_out_cb
// * (successful) tud_usbtmc_app_check_abort_bulk_in_cb
// * (successful) usmtmcd_app_bulkOut_clearFeature_cb

void tud_usbtmc_app_open_cb(uint8_t rhport, uint8_t interface_id);

bool tud_usbtmc_app_msgBulkOut_start_cb(uint8_t rhport, usbtmc_msg_request_dev_dep_out const * msgHeader);
// transfer_complete does not imply that a message is complete.
bool tud_usbtmc_app_msg_data_cb(uint8_t rhport, void *data, size_t len, bool transfer_complete);
void usmtmcd_app_bulkOut_clearFeature_cb(uint8_t rhport); // Notice to clear and abort the pending BULK out transfer

bool tud_usbtmc_app_msgBulkIn_request_cb(uint8_t rhport, usbtmc_msg_request_dev_dep_in const * request);
bool tud_usbtmc_app_msgBulkIn_complete_cb(uint8_t rhport);
void usmtmcd_app_bulkIn_clearFeature_cb(uint8_t rhport); // Notice to clear and abort the pending BULK out transfer

bool tud_usbtmc_app_initiate_abort_bulk_in_cb(uint8_t rhport, uint8_t *tmcResult);
bool tud_usbtmc_app_initiate_abort_bulk_out_cb(uint8_t rhport, uint8_t *tmcResult);
bool tud_usbtmc_app_initiate_clear_cb(uint8_t rhport, uint8_t *tmcResult);

bool tud_usbtmc_app_check_abort_bulk_in_cb(uint8_t rhport, usbtmc_check_abort_bulk_rsp_t *rsp);
bool tud_usbtmc_app_check_abort_bulk_out_cb(uint8_t rhport, usbtmc_check_abort_bulk_rsp_t *rsp);
bool tud_usbtmc_app_check_clear_cb(uint8_t rhport, usbtmc_get_clear_status_rsp_t *rsp);

// Indicator pulse should be 0.5 to 1.0 seconds long
TU_ATTR_WEAK bool tud_usbtmc_app_indicator_pulse_cb(uint8_t rhport, tusb_control_request_t const * msg, uint8_t *tmcResult);

#if (CFG_USBTMC_CFG_ENABLE_488)
uint8_t tud_usbtmc_app_get_stb_cb(uint8_t rhport, uint8_t *tmcResult);
TU_ATTR_WEAK bool tud_usbtmc_app_msg_trigger_cb(uint8_t rhport, usbtmc_msg_generic_t* msg);
//TU_ATTR_WEAK bool tud_usbtmc_app_go_to_local_cb(uint8_t rhport);
#endif

/*******************************************
 * Called from app
 *
 * We keep a reference to the buffer, so it MUST not change until the app is
 * notified that the transfer is complete.
 ******************************************/
bool usbtmcd_transmit_dev_msg_data(
    uint8_t rhport,
    const void * data, size_t len,
    bool endOfMessage, bool usingTermChar);

bool usbtmcd_start_bus_read(uint8_t rhport);


/* "callbacks" from USB device core */

bool usbtmcd_open_cb(uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t *p_length);
void usbtmcd_reset_cb(uint8_t rhport);
bool usbtmcd_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes);
bool usbtmcd_control_request_cb(uint8_t rhport, tusb_control_request_t const * request);
bool usbtmcd_control_complete_cb(uint8_t rhport, tusb_control_request_t const * request);
void usbtmcd_init_cb(void);

/************************************************************
 * USBTMC Descriptor Templates
 *************************************************************/


#endif /* CLASS_USBTMC_USBTMC_DEVICE_H_ */
