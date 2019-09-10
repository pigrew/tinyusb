/*
 * usbtmc.c
 *
 *  Created on: Sep 9, 2019
 *      Author: nconrad
 */

#include "tusb_option.h"

#if (TUSB_OPT_DEVICE_ENABLED && CFG_TUD_USBTMC)

#include "usbtmc.h"
#include "usbtmc_device.h"
void usbtmcd_init(void)
{

}
bool usbtmcd_open(uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t *p_length)
{
#if defined(CFG_TUD_USBTMC_ENABLE_INT_EP)
   *p_length = USBTMC_DESC_LEN (USBTMC_IF_DESCRIPTOR_LEN + USBTMC_BULK_DESCRIPTORS_LEN + USBTMC_INT_DESCRIPTOR_LEN);

#else
  *p_length = (USBTMC_IF_DESCRIPTOR_LEN + USBTMC_BULK_DESCRIPTORS_LEN);

#endif /* CFG_TUD_USBTMC_ENABLE_INT_EP */
  return true;
}
void usbtmcd_reset(uint8_t rhport)
{

}
bool usbtmcd_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
  return true;
}
bool usbtmcd_control_request(uint8_t rhport, tusb_control_request_t const * request) {
  return true;
}
bool usbtmcd_control_complete(uint8_t rhport, tusb_control_request_t const * request) {
  return true;
}

#endif /* CFG_TUD_TSMC */
