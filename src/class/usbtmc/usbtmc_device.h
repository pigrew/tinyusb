/*
 * usbtmc_device.h
 *
 *  Created on: Sep 10, 2019
 *      Author: nconrad
 */

#ifndef CLASS_USBTMC_USBTMC_DEVICE_H_
#define CLASS_USBTMC_USBTMC_DEVICE_H_

bool usbtmcd_open(uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t *p_length);
void usbtmcd_reset(uint8_t rhport);
bool usbtmcd_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes);
bool usbtmcd_control_request(uint8_t rhport, tusb_control_request_t const * request);
bool usbtmcd_control_complete(uint8_t rhport, tusb_control_request_t const * request);
void usbtmcd_init(void);
#endif /* CLASS_USBTMC_USBTMC_DEVICE_H_ */
