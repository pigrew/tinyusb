/*
 * usbtmc.c
 *
 *  Created on: Sep 9, 2019
 *      Author: nconrad
 */

#include "tusb_option.h"

// LIMITATIONS:
// "vendor-specific" commands are not handled



#if (TUSB_OPT_DEVICE_ENABLED && CFG_TUD_USBTMC)

#include "usbtmc.h"
#include "usbtmc_device.h"
#include "device/dcd.h"
#include "device/usbd.h"

// FIXME: I shouldn't need to include _pvt headers.
#include "device/usbd_pvt.h"

typedef enum {
  STATE_IDLE,
  STATE_RCV,
} usbtmcd_state_enum;

typedef struct {
  usbtmcd_state_enum state;
  uint8_t itf_id;
  uint8_t ep_bulk_in;
  uint8_t ep_bulk_out;
  uint8_t ep_int_in;
  uint8_t ep_bulk_in_buf[64];
  uint8_t ep_bulk_out_buf[64];

  uint32_t transfer_size_remaining;
} usbtmc_interface_state_t;

static usbtmc_interface_state_t usbtmc_state = {
    .state = STATE_IDLE,
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
  //TU_VERIFY( usbd_edpt_xfer(rhport, usbtmc_state.ep_bulk_in, usbtmc_state.ep_bulk_in_buf,64));
  TU_VERIFY( usbd_edpt_xfer(rhport, usbtmc_state.ep_bulk_out, usbtmc_state.ep_bulk_out_buf, 64));

/*

  // Prepare for incoming data

  return true;
*/

  return true;
}
void usbtmcd_reset(uint8_t rhport)
{
  // FIXME: Do endpoints need to be closed here?
  (void)rhport;
}
static bool handle_devMsgOut(void *data, size_t len) {
  bool shortPacket = (len < USBTMCD_MAX_PACKET_SIZE);
  if(usbtmc_state.state == STATE_IDLE) {
    // must be a header, should have been confirmed before calling here.
    usbtmc_msg_request_dev_dep_out *msg = (usbtmc_msg_request_dev_dep_out*)data;
    usbtmc_state.transfer_size_remaining = msg->TransferSize;
    TU_VERIFY(usbtmcd_app_msgBulkOut_start(msg));
    len -= sizeof(*msg);
    data = (uint8_t*)data + sizeof(*msg);
  }
  // Packet is to be considered complete when we get enough data or at a short packet.
  bool atEnd = false;
  if(len >= usbtmc_state.transfer_size_remaining || shortPacket)
    atEnd = true;
  if(len > usbtmc_state.transfer_size_remaining)
    len = usbtmc_state.transfer_size_remaining;
  usbtmcd_app_msg_data(data, len, atEnd);
  if(atEnd)
    usbtmc_state.state = STATE_IDLE;
  else
    usbtmc_state.state = STATE_RCV;
  return true;
}

bool usbtmcd_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
  (void)rhport;
  TU_VERIFY(result == XFER_RESULT_SUCCESS);
  if(ep_addr == usbtmc_state.ep_bulk_out)
  {
    switch(usbtmc_state.state)
    {
    case STATE_IDLE:
      TU_VERIFY(xferred_bytes >= sizeof(usbtmc_msg_generic_t));
      usbtmc_msg_generic_t *msg = (usbtmc_msg_generic_t*)(usbtmc_state.ep_bulk_out_buf);
      uint8_t invInvTag = (uint8_t)~(msg->header.bTagInverse);
      TU_VERIFY(msg->header.bTag == invInvTag);
      TU_VERIFY(msg->header.bTag != 0x00);
      switch(msg->header.MsgID) {
      case USBTMC_MSGID_DEV_DEP_MSG_OUT:
        TU_VERIFY(handle_devMsgOut(msg, xferred_bytes));
        TU_VERIFY( usbd_edpt_xfer(rhport, usbtmc_state.ep_bulk_out, usbtmc_state.ep_bulk_out_buf, 64));
        break;
      case USBTMC_MSGID_DEV_DEP_MSG_IN:
      case USBTMC_MSGID_VENDOR_SPECIFIC_MSG_OUT:
      case USBTMC_MSGID_VENDOR_SPECIFIC_IN:
      case USBTMC_MSGID_USB488_TRIGGER:
      default:
        TU_VERIFY(false);
      }
      return true;

    case STATE_RCV:
      TU_VERIFY(handle_devMsgOut(usbtmc_state.ep_bulk_out_buf, xferred_bytes));
      TU_VERIFY( usbd_edpt_xfer(rhport, usbtmc_state.ep_bulk_out, usbtmc_state.ep_bulk_out_buf, 64));
      return true;
      break;

    default:
      TU_VERIFY(false);
    }
  }
  else if(ep_addr == usbtmc_state.ep_bulk_in)
  {
    TU_VERIFY(false);
  }
  return false;
}

bool usbtmcd_control_request(uint8_t rhport, tusb_control_request_t const * request) {

  // We only handle class requests.
  if(request->bmRequestType_bit.type != TUSB_REQ_TYPE_CLASS)
    return false;

  switch(request->bRequest)
  {
  // USBTMC required requests
  case USBTMC_bREQUEST_INITIATE_ABORT_BULK_OUT:
  case USBTMC_bREQUEST_CHECK_ABORT_BULK_OUT_STATUS:
  case USBTMC_bREQUEST_INITIATE_ABORT_BULK_IN:
  case USBTMC_bREQUEST_CHECK_ABORT_BULK_IN_STATUS:
  case USBTMC_bREQUEST_INITIATE_CLEAR:
  case USBTMC_bREQUEST_CHECK_CLEAR_STATUS:
    TU_VERIFY(false);
    break;

  case USBTMC_bREQUEST_GET_CAPABILITIES:
    TU_VERIFY(request->bmRequestType == 0xA1);
    TU_VERIFY(request->wValue == 0x0000);
    TU_VERIFY(request->wIndex == usbtmc_state.itf_id);
    TU_VERIFY(request->wLength == sizeof(usbtmcd_app_capabilities));
    return tud_control_xfer(rhport, request, (void*)&usbtmcd_app_capabilities, sizeof(usbtmcd_app_capabilities));
    break;

  // USBTMC Optional Requests
  case USBTMC_bREQUEST_INDICATOR_PULSE: // Optional
    TU_VERIFY(false);
    return false;
    break;

    // USB488 required requests
  case USBTMC488_bREQUEST_READ_STATUS_BYTE:
    TU_VERIFY(false);
    break;
    // USB488 optional requests
  case USBTMC488_bREQUEST_REN_CONTROL:
  case USBTMC488_bREQUEST_GO_TO_LOCAL:
  case USBTMC488_bREQUEST_LOCAL_LOCKOUT:
    TU_VERIFY(false);
    return false;
    break;
  default:
    TU_VERIFY(false);
  }
  TU_VERIFY(false);
  return false;
}

bool usbtmcd_control_complete(uint8_t rhport, tusb_control_request_t const * request)
{
  (void)rhport;
  //------------- Class Specific Request -------------//
  TU_VERIFY (request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS);

  return true;
}

#endif /* CFG_TUD_TSMC */
