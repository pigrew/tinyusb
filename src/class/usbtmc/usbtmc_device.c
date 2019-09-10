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
#include "device/dcd.h"

typedef struct {
  uint8_t itf_id;
  uint8_t ep_bulk_in;
  uint8_t ep_bulk_out;
  uint8_t ep_int_in;
} usbtmc_interface_state_t;

static usbtmc_interface_state_t usbtmc_state = {
    .itf_id = 0xFF,
    .ep_bulk_in = 0,
    .ep_bulk_out = 0,
    .ep_int_in = 0
};

void usbtmcd_init(void)
{

}
bool usbtmcd_open(uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t *p_length)
{
  (void)rhport;
  uint8_t const * p_desc;
  uint8_t found_endpoints = 0;

  // Perhaps there are other application specific class drivers, so don't assert here.
  if( itf_desc->bInterfaceClass != USBTMC_APP_CLASS)
    return false;
  if( itf_desc->bInterfaceSubClass != USBTMC_APP_SUBCLASS)
    return false;

  // Only 2 or 3 endpoints are allowed for USBTMC.
  TU_ASSERT((itf_desc->bNumEndpoints == 2) || (itf_desc->bNumEndpoints ==3));

  // Interface
  (*p_length) = 0u;
  p_desc = (uint8_t const *) itf_desc;

  usbtmc_state.itf_id = itf_desc->bInterfaceNumber;

  while (found_endpoints < itf_desc->bNumEndpoints)
  {
    if ( TUSB_DESC_ENDPOINT == p_desc[DESC_OFFSET_TYPE])
    {
      tusb_desc_endpoint_t const *ep_desc = (tusb_desc_endpoint_t const *)p_desc;
      switch(ep_desc->bmAttributes.xfer) {
        case TUSB_XFER_BULK:
          if (tu_edpt_dir(ep_desc->bEndpointAddress) == TUSB_DIR_IN) {
            usbtmc_state.ep_bulk_in = ep_desc->bEndpointAddress;
          } else {
            usbtmc_state.ep_bulk_out = ep_desc->bEndpointAddress;
          }

          break;
        case TUSB_XFER_INTERRUPT:
          TU_ASSERT(tu_edpt_dir(ep_desc->bEndpointAddress) == TUSB_DIR_IN);
          TU_ASSERT(usbtmc_state.ep_int_in == 0);
          usbtmc_state.ep_int_in = ep_desc->bEndpointAddress;
          break;
        default:
          TU_ASSERT(false);
      }
      TU_VERIFY( dcd_edpt_open(rhport, ep_desc));
      found_endpoints++;
    }
    (*p_length) = (uint8_t)((*p_length) + p_desc[DESC_OFFSET_LEN]);
    p_desc = tu_desc_next(p_desc);
  }

  // bulk endpoints are required, but interrupt IN is optional
  TU_ASSERT(usbtmc_state.ep_bulk_in != 0);
  TU_ASSERT(usbtmc_state.ep_bulk_out != 0);
  if (itf_desc->bNumEndpoints == 2) {
    TU_ASSERT(usbtmc_state.ep_int_in == 0);
  }
  else if (itf_desc->bNumEndpoints == 2)
  {
    TU_ASSERT(usbtmc_state.ep_int_in != 0);
  }


/*

  // Prepare for incoming data
  TU_ASSERT( usbd_edpt_xfer(rhport, p_midi->ep_out, p_midi->epout_buf, CFG_TUD_MIDI_EPSIZE), false);

  return true;
*/

  return true;
}
void usbtmcd_reset(uint8_t rhport)
{
  // FIXME: Do endpoints need to be closed here?
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
