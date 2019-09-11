/*
 * usbtmc_device.h
 *
 *  Created on: Sep 10, 2019
 *      Author: nconrad
 */

#ifndef CLASS_USBTMC_USBTMC_DEVICE_H_
#define CLASS_USBTMC_USBTMC_DEVICE_H_

#include "usbtmc.h"

#if !defined(USBTMC_CFG_ENABLE_488)
#define USBTMC_CFG_ENABLE_488 (1)
#endif

/***********************************************
 *  Functions to be implemeted by the class implementation
 */

#if (USBTMC_CFG_ENABLE_488)
extern usbtmc_response_capabilities_488 const usbtmcd_app_capabilities;
#else
extern usbtmc_response_capabilities const usbtmcd_app_capabilities;
#endif


/* "callbacks" from USB device core */

bool usbtmcd_open(uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t *p_length);
void usbtmcd_reset(uint8_t rhport);
bool usbtmcd_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes);
bool usbtmcd_control_request(uint8_t rhport, tusb_control_request_t const * request);
bool usbtmcd_control_complete(uint8_t rhport, tusb_control_request_t const * request);
void usbtmcd_init(void);

/************************************************************
 * USBTMC Descriptor Templates
 *************************************************************/

#define USBTMC_APP_CLASS    TUSB_CLASS_APPLICATION_SPECIFIC
#define USBTMC_APP_SUBCLASS 0x03

#define USBTMC_PROTOCOL_STD    0x00
#define USBTMC_PROTOCOL_USB488 0x01

//   Interface number, number of endpoints, EP string index, USB_TMC_PROTOCOL*, bulk-out endpoint ID,
//   bulk-in endpoint ID
#define USBTMC_IF_DESCRIPTOR(_itfnum, _bNumEndpoints, _stridx, _itfProtocol) \
/* Interface */ \
  0x09, TUSB_DESC_INTERFACE, _itfnum, 0x00, _bNumEndpoints, USBTMC_APP_CLASS, USBTMC_APP_SUBCLASS, _itfProtocol, _stridx

#define USBTMC_IF_DESCRIPTOR_LEN 9u

// bulk-out Size must be a multiple of 4 bytes
#define USBTMC_BULK_DESCRIPTORS(_epout,_epoutsize, _epin, _epinsize) \
/* Endpoint Out */ \
7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epoutsize), 0u, \
/* Endpoint In */ \
7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epinsize), 0u

#define USBTMC_BULK_DESCRIPTORS_LEN (7u+7u)

/* optional interrupt endpoint */ \
// _int_pollingInterval : for LS/FS, expressed in frames (1ms each). 16 may be a good number?
#define USBTMC_INT_DESCRIPTOR(_ep_interrupt, _ep_interrupt_size, _int_pollingInterval ) \
7, TUSB_DESC_ENDPOINT, _ep_interrupt, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_ep_interrupt_size), 0x16

#define USBTMC_INT_DESCRIPTOR_LEN (7u)


#endif /* CLASS_USBTMC_USBTMC_DEVICE_H_ */
