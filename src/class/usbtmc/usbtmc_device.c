/*
 * usbtmc.c
 *
 *  Created on: Sep 9, 2019
 *      Author: nconrad
 */

/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Nathan Conrad
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

/*
 * This library is not fully reentrant, though it is reentrant from the view
 * of either the application layer or the USB stack. Due to its locking,
 * it is not safe to call its functions from interrupts.
 *
 * The one exception is that its functions may not be called from the application
 * until the USB stack is initialized. This should not be a problem since the
 * device shouldn't be sending messages until it receives a request from the
 * host.
 */


/*
 * In the case of single-CPU "no OS", this task is never preempted other than by
 * interrupts, and the USBTMC code isn't called by interrupts, so all is OK. For "no OS",
 * the mutex structure's main effect is to disable the USB interrupts.
 * With an OS, this class driver uses the OSAL to perform locking. The code uses a single lock
 * and does not call outside of this class with a lock held, so deadlocks won't happen.
 */

//Limitations:
// "vendor-specific" commands are not handled.
// Dealing with "termchar" must be handled by the application layer,
//    though additional error checking is does in this module.
// talkOnly and listenOnly are NOT supported. They're not permitted
// in USB488, anyway.

/* Supported:
 *
 * Notification pulse
 * Trigger
 * Read status byte (both by interrupt endpoint and control message)
 *
 */


// TODO:
// USBTMC 3.2.2 error conditions not strictly followed
// No local lock-out, REN, or GTL.
// Clear message available status byte at the correct time? (488 4.3.1.3)


#include "tusb_option.h"

#if (TUSB_OPT_DEVICE_ENABLED && CFG_TUD_USBTMC)

#include <string.h>
#include "usbtmc.h"
#include "usbtmc_device.h"
#include "device/dcd.h"
#include "device/usbd.h"
#include "osal/osal.h"

// FIXME: I shouldn't need to include _pvt headers, but it is necessary for usbd_edpt_xfer, _stall, and _busy
#include "device/usbd_pvt.h"

#ifdef xDEBUG
#include "uart_util.h"
static char logMsg[150];
#endif

/*
 * The state machine does not allow simultaneous reading and writing. This is
 * consistent with USBTMC.
 */

typedef enum
{
  STATE_CLOSED,
  STATE_IDLE,
  STATE_RCV,
  STATE_TX_REQUESTED,
  STATE_TX_INITIATED,
  STATE_TX_SHORTED,
  STATE_CLEARING,
  STATE_ABORTING_BULK_IN,
  STATE_ABORTING_BULK_IN_SHORTED, // aborting, and short packet has been queued for transmission
  STATE_ABORTING_BULK_IN_ABORTED, // aborting, and short packet has been transmitted
  STATE_ABORTING_BULK_OUT,
  STATE_NUM_STATES
} usbtmcd_state_enum;

typedef struct
{
  volatile usbtmcd_state_enum state;

  uint8_t itf_id;
  uint8_t ep_bulk_in;
  uint8_t ep_bulk_out;
  uint8_t ep_int_in;
  // IN buffer is only used for first packet, not the remainder
  // in order to deal with prepending header
  uint8_t ep_bulk_in_buf[USBTMCD_MAX_PACKET_SIZE];
  // OUT buffer receives one packet at a time
  uint8_t ep_bulk_out_buf[USBTMCD_MAX_PACKET_SIZE];
  uint32_t transfer_size_remaining; // also used for requested length for bulk IN.
  uint32_t transfer_size_sent;      // To keep track of data bytes that have been queued in FIFO (not header bytes)

  uint8_t lastBulkOutTag; // used for aborts (mostly)
  uint8_t lastBulkInTag; // used for aborts (mostly)

  uint8_t const * devInBuffer; // pointer to application-layer used for transmissions
} usbtmc_interface_state_t;

static usbtmc_interface_state_t usbtmc_state =
{
    .itf_id = 0xFF,
};

// We need all headers to fit in a single packet in this implementation.
TU_VERIFY_STATIC(USBTMCD_MAX_PACKET_SIZE >= 32u,"USBTMC dev EP packet size too small");
TU_VERIFY_STATIC(
    (sizeof(usbtmc_state.ep_bulk_in_buf) % USBTMCD_MAX_PACKET_SIZE) == 0,
    "packet buffer must be a multiple of the packet size");

static bool handle_devMsgOutStart(uint8_t rhport, void *data, size_t len);
static bool handle_devMsgOut(uint8_t rhport, void *data, size_t len, size_t packetLen);

static uint8_t termChar;
static uint8_t termCharRequested = false;


osal_mutex_def_t usbtmcLockBuffer;
static osal_mutex_t usbtmcLock;

// Our own private lock, mostly for the state variable.
#define criticalEnter() do {osal_mutex_lock(usbtmcLock,OSAL_TIMEOUT_WAIT_FOREVER); } while (0)
#define criticalLeave() do {osal_mutex_unlock(usbtmcLock); } while (0)

// called from app
// We keep a reference to the buffer, so it MUST not change until the app is
// notified that the transfer is complete.
// length of data is specified in the hdr.

// We can't just send the whole thing at once because we need to concatanate the
// header with the data.
bool usbtmcd_transmit_dev_msg_data(
    uint8_t rhport,
    const void * data, size_t len,
    bool endOfMessage,
    bool usingTermChar)
{
  const unsigned int txBufLen = sizeof(usbtmc_state.ep_bulk_in_buf);

#ifndef NDEBUG
  TU_ASSERT(len > 0u);
  TU_ASSERT(len <= usbtmc_state.transfer_size_remaining);
  TU_ASSERT(usbtmc_state.transfer_size_sent == 0u);
  if(usingTermChar)
  {
    TU_ASSERT(tud_usbtmc_app_capabilities.bmDevCapabilities.canEndBulkInOnTermChar);
    TU_ASSERT(termCharRequested);
    TU_ASSERT(((uint8_t*)data)[len-1u] == termChar);
  }
#endif

  TU_VERIFY(usbtmc_state.state == STATE_TX_REQUESTED);
  usbtmc_msg_dev_dep_msg_in_header_t *hdr = (usbtmc_msg_dev_dep_msg_in_header_t*)usbtmc_state.ep_bulk_in_buf;
  tu_varclr(&hdr);
  hdr->header.MsgID = USBTMC_MSGID_DEV_DEP_MSG_IN;
  hdr->header.bTag = usbtmc_state.lastBulkInTag;
  hdr->header.bTagInverse = (uint8_t)~(usbtmc_state.lastBulkInTag);
  hdr->TransferSize = len;
  hdr->bmTransferAttributes.EOM = endOfMessage;
  hdr->bmTransferAttributes.UsingTermChar = usingTermChar;

  // Copy in the header
  size_t packetLen = sizeof(*hdr);

  // If it fits in a single transmission:
  if((packetLen + hdr->TransferSize) <= txBufLen)
  {
    memcpy((uint8_t*)(usbtmc_state.ep_bulk_in_buf) + packetLen, data, hdr->TransferSize);
    packetLen = (uint16_t)(packetLen + hdr->TransferSize);
    usbtmc_state.transfer_size_remaining = 0;
    usbtmc_state.transfer_size_sent = len;
    usbtmc_state.devInBuffer = NULL;
  }
  else /* partial packet */
  {
    memcpy((uint8_t*)(usbtmc_state.ep_bulk_in_buf) + packetLen, data, txBufLen - packetLen);
    usbtmc_state.devInBuffer = (uint8_t*)data + (txBufLen - packetLen);
    usbtmc_state.transfer_size_remaining = len - (txBufLen - packetLen);
    usbtmc_state.transfer_size_sent = txBufLen - packetLen;
    packetLen = txBufLen;
  }


  criticalEnter();
  {
    TU_VERIFY(usbtmc_state.state == STATE_TX_REQUESTED);
    // We used packetlen as a max, not the buffer size, so this is OK here, no need for modulus
    usbtmc_state.state  = (packetLen >= txBufLen) ? STATE_TX_INITIATED : STATE_TX_SHORTED;
  }
  criticalLeave();

  TU_VERIFY( usbd_edpt_xfer(rhport, usbtmc_state.ep_bulk_in, usbtmc_state.ep_bulk_in_buf, (uint16_t)packetLen));
  return true;
}

void usbtmcd_init_cb(void)
{
#ifndef NDEBUG
# if CFG_USBTMC_CFG_ENABLE_488
    if(tud_usbtmc_app_capabilities.bmIntfcCapabilities488.supportsTrigger)
      TU_ASSERT(&tud_usbtmc_app_msg_trigger_cb != NULL,);
      // Per USB488 spec: table 8
      TU_ASSERT(!tud_usbtmc_app_capabilities.bmIntfcCapabilities.listenOnly,);
      TU_ASSERT(!tud_usbtmc_app_capabilities.bmIntfcCapabilities.talkOnly,);
# endif
    if(tud_usbtmc_app_capabilities.bmIntfcCapabilities.supportsIndicatorPulse)
      TU_ASSERT(&tud_usbtmc_app_indicator_pluse_cb != NULL,);
#endif

    usbtmcLock = osal_mutex_create(&usbtmcLockBuffer);
}

bool usbtmcd_open_cb(uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t *p_length)
{
  uart_tx_str_sync("usbtmcd_open_cb\r\n");
  (void)rhport;
  TU_ASSERT(usbtmc_state.state == STATE_CLOSED);
  uint8_t const * p_desc;
  uint8_t found_endpoints = 0;

  // Perhaps there are other application specific class drivers, so don't assert here.
  if( itf_desc->bInterfaceClass != TUD_USBTMC_APP_CLASS)
    return false;
  if( itf_desc->bInterfaceSubClass != TUD_USBTMC_APP_SUBCLASS)
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
          TU_ASSERT(ep_desc->wMaxPacketSize.size == USBTMCD_MAX_PACKET_SIZE);
          if (tu_edpt_dir(ep_desc->bEndpointAddress) == TUSB_DIR_IN)
          {
            usbtmc_state.ep_bulk_in = ep_desc->bEndpointAddress;
          } else {
            usbtmc_state.ep_bulk_out = ep_desc->bEndpointAddress;
          }

          break;
        case TUSB_XFER_INTERRUPT:
#ifndef NDEBUG
          TU_ASSERT(tu_edpt_dir(ep_desc->bEndpointAddress) == TUSB_DIR_IN);
          TU_ASSERT(usbtmc_state.ep_int_in == 0);
#endif
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
#ifndef NDEBUG
  TU_ASSERT(usbtmc_state.ep_bulk_in != 0);
  TU_ASSERT(usbtmc_state.ep_bulk_out != 0);
  if (itf_desc->bNumEndpoints == 2)
  {
    TU_ASSERT(usbtmc_state.ep_int_in == 0);
  }
  else if (itf_desc->bNumEndpoints == 3)
  {
    TU_ASSERT(usbtmc_state.ep_int_in != 0);
  }
#if (CFG_USBTMC_CFG_ENABLE_488)
  if(tud_usbtmc_app_capabilities.bmIntfcCapabilities488.is488_2 ||
      tud_usbtmc_app_capabilities.bmDevCapabilities488.SR1)
  {
    TU_ASSERT(usbtmc_state.ep_int_in != 0);
  }
#endif
#endif
  usbtmc_state.state = STATE_IDLE;
  TU_VERIFY( usbd_edpt_xfer(rhport, usbtmc_state.ep_bulk_out, usbtmc_state.ep_bulk_out_buf, 64));

  return true;
}
void usbtmcd_reset_cb(uint8_t rhport)
{
  (void)rhport;
  uart_tx_str_sync("usbtmcd_reset_cb\r\n");
  // FIXME: Do endpoints need to be closed here?
  criticalEnter();
  tu_varclr(&usbtmc_state);
  usbtmc_state.itf_id = 0xFFu;
  criticalLeave();
}

static bool handle_devMsgOutStart(uint8_t rhport, void *data, size_t len)
{
  (void)rhport;
  TU_VERIFY(usbtmc_state.state == STATE_IDLE);
  // must be a header, should have been confirmed before calling here.
  usbtmc_msg_request_dev_dep_out *msg = (usbtmc_msg_request_dev_dep_out*)data;
  usbtmc_state.transfer_size_remaining = msg->TransferSize;
  TU_VERIFY(tud_usbtmc_app_msgBulkOut_start_cb(rhport,msg));

  TU_VERIFY(handle_devMsgOut(rhport, (uint8_t*)data + sizeof(*msg), len - sizeof(*msg), len));
  return true;
}

static bool handle_devMsgOut(uint8_t rhport, void *data, size_t len, size_t packetLen)
{
  (void)rhport;
  bool shortPacket = (packetLen < USBTMCD_MAX_PACKET_SIZE);

  // Packet is to be considered complete when we get enough data or at a short packet.
  bool atEnd = false;
  if(len >= usbtmc_state.transfer_size_remaining || shortPacket)
    atEnd = true;
  if(len > usbtmc_state.transfer_size_remaining)
    len = usbtmc_state.transfer_size_remaining;
  tud_usbtmc_app_msg_data_cb(rhport,data, len, atEnd);

  usbtmc_state.transfer_size_remaining -= len;
  usbtmc_state.transfer_size_sent += len;
  if(atEnd)
  {
    usbtmc_state.state = STATE_IDLE;
  }
  else
  {
    usbtmc_state.state = STATE_RCV;
  }
  return true;
}

static bool handle_devMsgIn(uint8_t rhport, void *data, size_t len)
{
  TU_VERIFY(len == sizeof(usbtmc_msg_request_dev_dep_in));
  usbtmc_msg_request_dev_dep_in *msg = (usbtmc_msg_request_dev_dep_in*)data;

  criticalEnter();
  {
    TU_VERIFY(usbtmc_state.state == STATE_IDLE);
    usbtmc_state.state = STATE_TX_REQUESTED;
    usbtmc_state.lastBulkInTag = msg->header.bTag;
    usbtmc_state.transfer_size_remaining = msg->TransferSize;
    usbtmc_state.transfer_size_sent = 0u;
  }
  criticalLeave();

  termCharRequested = msg->bmTransferAttributes.TermCharEnabled;
  termChar = msg->TermChar;

  if(termCharRequested)
    TU_VERIFY(tud_usbtmc_app_capabilities.bmDevCapabilities.canEndBulkInOnTermChar);

  TU_VERIFY(tud_usbtmc_app_msgBulkIn_request_cb(rhport, msg));
  return true;
}

bool usbtmcd_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
  TU_VERIFY(result == XFER_RESULT_SUCCESS);
  uart_tx_str_sync("TMC XFER CB\r\n");
  if(usbtmc_state.state == STATE_CLEARING) {
    return true; /* I think we can ignore everything here */
  }

  if(ep_addr == usbtmc_state.ep_bulk_out)
  {
    usbtmc_msg_generic_t *msg = NULL;

    switch(usbtmc_state.state)
    {
    case STATE_IDLE:
      TU_VERIFY(xferred_bytes >= sizeof(usbtmc_msg_generic_t));
      msg = (usbtmc_msg_generic_t*)(usbtmc_state.ep_bulk_out_buf);
      uint8_t invInvTag = (uint8_t)~(msg->header.bTagInverse);
      TU_VERIFY(msg->header.bTag == invInvTag);
      TU_VERIFY(msg->header.bTag != 0x00);

      switch(msg->header.MsgID) {
      case USBTMC_MSGID_DEV_DEP_MSG_OUT:
        usbtmc_state.transfer_size_sent = 0u;
        TU_VERIFY(handle_devMsgOutStart(rhport, msg, xferred_bytes));
        usbtmc_state.lastBulkOutTag = msg->header.bTag;
        break;

      case USBTMC_MSGID_DEV_DEP_MSG_IN:
        TU_VERIFY(handle_devMsgIn(rhport, msg, xferred_bytes));
        break;

#if (CFG_USBTMC_CFG_ENABLE_488)
      case USBTMC_MSGID_USB488_TRIGGER:
        // Spec says we halt the EP if we didn't declare we support it.
        TU_VERIFY(tud_usbtmc_app_capabilities.bmIntfcCapabilities488.supportsTrigger);
        TU_VERIFY(tud_usbtmc_app_msg_trigger_cb(rhport, msg));

        break;
#endif
      case USBTMC_MSGID_VENDOR_SPECIFIC_MSG_OUT:
      case USBTMC_MSGID_VENDOR_SPECIFIC_IN:
      default:

        TU_VERIFY(false);
        return false;
      }
      TU_VERIFY(usbd_edpt_xfer(rhport, usbtmc_state.ep_bulk_out, usbtmc_state.ep_bulk_out_buf, USBTMCD_MAX_PACKET_SIZE));
      return true;

    case STATE_RCV:
      TU_VERIFY(handle_devMsgOut(rhport, usbtmc_state.ep_bulk_out_buf, xferred_bytes, xferred_bytes));
      TU_VERIFY(usbd_edpt_xfer(rhport, usbtmc_state.ep_bulk_out, usbtmc_state.ep_bulk_out_buf, USBTMCD_MAX_PACKET_SIZE));
      return true;

    case STATE_ABORTING_BULK_OUT:
      TU_VERIFY(false);
      return false; // Should be stalled by now...
    case STATE_TX_REQUESTED:
    case STATE_TX_INITIATED:
    case STATE_ABORTING_BULK_IN:
    case STATE_ABORTING_BULK_IN_SHORTED:
    case STATE_ABORTING_BULK_IN_ABORTED:
    default:

      TU_VERIFY(false);
    }
  }
  else if(ep_addr == usbtmc_state.ep_bulk_in)
  {
    switch(usbtmc_state.state) {
    case STATE_TX_SHORTED:

      usbtmc_state.state = STATE_IDLE;
      TU_VERIFY(tud_usbtmc_app_msgBulkIn_complete_cb(rhport));
      break;

    case STATE_TX_INITIATED:
      if(usbtmc_state.transfer_size_remaining >=sizeof(usbtmc_state.ep_bulk_in_buf))
    {
        // FIXME! This removes const below!
        TU_VERIFY( usbd_edpt_xfer(rhport, usbtmc_state.ep_bulk_in,
            (void*)usbtmc_state.devInBuffer,sizeof(usbtmc_state.ep_bulk_in_buf)));
        usbtmc_state.devInBuffer += sizeof(usbtmc_state.ep_bulk_in_buf);
        usbtmc_state.transfer_size_remaining -= sizeof(usbtmc_state.ep_bulk_in_buf);
        usbtmc_state.transfer_size_sent += sizeof(usbtmc_state.ep_bulk_in_buf);
    }
    else // last packet
    {
      size_t packetLen = usbtmc_state.transfer_size_remaining;
      memcpy(usbtmc_state.ep_bulk_in_buf, usbtmc_state.devInBuffer, usbtmc_state.transfer_size_remaining);
        usbtmc_state.transfer_size_sent += sizeof(usbtmc_state.transfer_size_remaining);
      usbtmc_state.transfer_size_remaining = 0;
      usbtmc_state.devInBuffer = NULL;
      TU_VERIFY( usbd_edpt_xfer(rhport, usbtmc_state.ep_bulk_in, usbtmc_state.ep_bulk_in_buf,(uint16_t)packetLen));
        if(((packetLen % USBTMCD_MAX_PACKET_SIZE) != 0) || (packetLen == 0 ))
        {
          usbtmc_state.state = STATE_TX_SHORTED;
    }
      }
      return true;
    case STATE_ABORTING_BULK_IN:
      // need to send short packet  (ZLP?)
      TU_VERIFY( usbd_edpt_xfer(rhport, usbtmc_state.ep_bulk_in, usbtmc_state.ep_bulk_in_buf,(uint16_t)0u));
      usbtmc_state.state = STATE_ABORTING_BULK_IN_SHORTED;
      return true;
    case STATE_ABORTING_BULK_IN_SHORTED:
      /* Done. :)*/
      usbtmc_state.state = STATE_ABORTING_BULK_IN_ABORTED;
    return true;
    default:
      TU_ASSERT(false);
      return false;
    }
  }
  else if (ep_addr == usbtmc_state.ep_int_in) {
    // Good?
    return true;
  }
  return false;
}

bool usbtmcd_control_request_cb(uint8_t rhport, tusb_control_request_t const * request) {

  uint8_t tmcStatusCode = USBTMC_STATUS_FAILED;
#if (CFG_USBTMC_CFG_ENABLE_488)
  uint8_t bTag;
#endif

  if((request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD) &&
      (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_ENDPOINT) &&
      (request->bRequest == TUSB_REQ_CLEAR_FEATURE) &&
      (request->wValue == TUSB_REQ_FEATURE_EDPT_HALT))
  {
    uint32_t ep_addr = (request->wIndex);
    sprintf(bigMsg,"clearing usbtmc stall %lu", (uint32_t)ep_addr);
    uart_tx_str_sync(bigMsg);
    if(ep_addr == usbtmc_state.ep_bulk_out)
    {
      usmtmcd_app_bulkOut_clearFeature_cb(rhport);
      // And start a new OUT xfer request now that things are clear
      TU_ASSERT( usbd_edpt_xfer(rhport, usbtmc_state.ep_bulk_out, usbtmc_state.ep_bulk_out_buf, 64));
    }
    else if ((request->wIndex) == usbtmc_state.ep_bulk_in)
    {
      usmtmcd_app_bulkIn_clearFeature_cb(rhport);
    }
    return true;
  }

  // We only handle class requests, IN direction.
  // (for now)
  if(request->bmRequestType_bit.type != TUSB_REQ_TYPE_CLASS)
  {
    return false;
  }

  // Verification that we own the interface is unneeded since it's been routed to us specifically.

  switch(request->bRequest)
  {
  // USBTMC required requests
  case USBTMC_bREQUEST_INITIATE_ABORT_BULK_OUT:
  {
    usbtmc_initiate_abort_rsp_t rsp = {
        .bTag = usbtmc_state.lastBulkOutTag,
    };
    TU_VERIFY(request->bmRequestType == 0xA2); // in,class,interface
    TU_VERIFY(request->wLength == sizeof(rsp));
    TU_VERIFY(request->wIndex == usbtmc_state.ep_bulk_out);

    // wValue is the requested bTag to abort
    if(usbtmc_state.state != STATE_RCV)
    {
      rsp.USBTMC_status = USBTMC_STATUS_FAILED;
    }
    else if(usbtmc_state.lastBulkOutTag == (request->wValue & 0xf7u))
    {
      rsp.USBTMC_status = USBTMC_STATUS_TRANSFER_NOT_IN_PROGRESS;
    }
    else
    {
      rsp.USBTMC_status = USBTMC_STATUS_SUCCESS;
      // Check if we've queued a short packet
      usbtmc_state.state = STATE_ABORTING_BULK_OUT;
      TU_VERIFY(tud_usbtmc_app_initiate_abort_bulk_out_cb(rhport, &(rsp.USBTMC_status)));
      usbd_edpt_stall(rhport, usbtmc_state.ep_bulk_out);
    }
    TU_VERIFY(tud_control_xfer(rhport, request, (void*)&rsp,sizeof(rsp)));
    return true;
  }
  case USBTMC_bREQUEST_CHECK_ABORT_BULK_OUT_STATUS:
  {
    usbtmc_check_abort_bulk_rsp_t rsp = {
        .USBTMC_status = USBTMC_STATUS_SUCCESS,
        .NBYTES_RXD_TXD = usbtmc_state.transfer_size_sent
    };
    TU_VERIFY(request->bmRequestType == 0xA2); // in,class,EP
    TU_VERIFY(request->wLength == sizeof(rsp));
    TU_VERIFY(request->wIndex == usbtmc_state.ep_bulk_out);
    TU_VERIFY(tud_usbtmc_app_check_abort_bulk_out_cb(rhport, &rsp));
    TU_VERIFY(usbd_edpt_xfer(rhport, 0u, (void*)&rsp,sizeof(rsp)));
    return true;
  }

  case USBTMC_bREQUEST_INITIATE_ABORT_BULK_IN:
  {
    usbtmc_initiate_abort_rsp_t rsp = {
        .bTag = usbtmc_state.lastBulkInTag,
    };
    TU_VERIFY(request->bmRequestType == 0xA2); // in,class,interface
    TU_VERIFY(request->wLength == sizeof(rsp));
    TU_VERIFY(request->wIndex == usbtmc_state.ep_bulk_in);
    // wValue is the requested bTag to abort
    if((usbtmc_state.state == STATE_TX_REQUESTED || usbtmc_state.state == STATE_TX_INITIATED) &&
        usbtmc_state.lastBulkInTag == (request->wValue & 0xf7u))
    {
      rsp.USBTMC_status = USBTMC_STATUS_SUCCESS;
    usbtmc_state.transfer_size_remaining = 0u;
      // Check if we've queued a short packet
      usbtmc_state.state = ((usbtmc_state.transfer_size_sent % USBTMCD_MAX_PACKET_SIZE) == 0) ?
              STATE_ABORTING_BULK_IN : STATE_ABORTING_BULK_IN_SHORTED;
      if(usbtmc_state.transfer_size_sent  == 0)
      {
        // Send short packet, nothing is in the buffer yet
        TU_VERIFY( usbd_edpt_xfer(rhport, usbtmc_state.ep_bulk_in, usbtmc_state.ep_bulk_in_buf,(uint16_t)0u));
        usbtmc_state.state = STATE_ABORTING_BULK_IN_SHORTED;
      }
      TU_VERIFY(tud_usbtmc_app_initiate_abort_bulk_in_cb(rhport, &(rsp.USBTMC_status)));
    }
    else if((usbtmc_state.state == STATE_TX_REQUESTED || usbtmc_state.state == STATE_TX_INITIATED))
    { // FIXME: Unsure how to check  if the OUT endpoint fifo is non-empty....
      rsp.USBTMC_status = USBTMC_STATUS_TRANSFER_NOT_IN_PROGRESS;
    }
    else
    {
      rsp.USBTMC_status = USBTMC_STATUS_FAILED;
    }
    TU_VERIFY(tud_control_xfer(rhport, request, (void*)&rsp,sizeof(rsp)));
    return true;
  }

  case USBTMC_bREQUEST_CHECK_ABORT_BULK_IN_STATUS:
  {
    TU_VERIFY(request->bmRequestType == 0xA2); // in,class,EP
    TU_VERIFY(request->wLength == 8u);

    usbtmc_check_abort_bulk_rsp_t rsp =
    {
        .USBTMC_status = USBTMC_STATUS_FAILED,
        .bmAbortBulkIn =
        {
            .BulkInFifoBytes = (usbtmc_state.state != STATE_ABORTING_BULK_IN_ABORTED)
        },
        .NBYTES_RXD_TXD = usbtmc_state.transfer_size_sent,
    };
    TU_VERIFY(tud_usbtmc_app_check_abort_bulk_in_cb(rhport, &rsp));
    switch(usbtmc_state.state)
    {
    case STATE_ABORTING_BULK_IN_ABORTED:
      rsp.USBTMC_status = USBTMC_STATUS_SUCCESS;
      usbtmc_state.state = STATE_IDLE;
      break;
    case STATE_ABORTING_BULK_IN:
    case STATE_ABORTING_BULK_OUT:
      rsp.USBTMC_status = USBTMC_STATUS_PENDING;
      break;
    default:
      break;
    }
    TU_VERIFY(tud_control_xfer(rhport, request, (void*)&rsp,sizeof(rsp)));

    return true;
  }

  case USBTMC_bREQUEST_INITIATE_CLEAR:
    {
      TU_VERIFY(request->bmRequestType == 0xA1); // in,class,interface
      TU_VERIFY(request->wLength == sizeof(tmcStatusCode));
      // After receiving an INITIATE_CLEAR request, the device must Halt the Bulk-OUT endpoint, queue the
      // control endpoint response shown in Table 31, and clear all input buffers and output buffers.
      usbd_edpt_stall(rhport, usbtmc_state.ep_bulk_out);
      usbtmc_state.transfer_size_remaining = 0;
      usbtmc_state.state = STATE_CLEARING;
      TU_VERIFY(tud_usbtmc_app_initiate_clear_cb(rhport, &tmcStatusCode));
      TU_VERIFY(tud_control_xfer(rhport, request, (void*)&tmcStatusCode,sizeof(tmcStatusCode)));
      return true;
    }

  case USBTMC_bREQUEST_CHECK_CLEAR_STATUS:
    {
      TU_VERIFY(request->bmRequestType == 0xA1); // in,class,interface
      usbtmc_get_clear_status_rsp_t clearStatusRsp = {0};
      TU_VERIFY(request->wLength == sizeof(clearStatusRsp));

      if(usbd_edpt_busy(rhport, usbtmc_state.ep_bulk_in))
      {
        // Stuff stuck in TX buffer?
        clearStatusRsp.bmClear.BulkInFifoBytes = 1;
        clearStatusRsp.USBTMC_status = USBTMC_STATUS_PENDING;
      }
      else
      {
        // Let app check if it's clear
        TU_VERIFY(tud_usbtmc_app_check_clear_cb(rhport, &clearStatusRsp));
      }
      if(clearStatusRsp.USBTMC_status == USBTMC_STATUS_SUCCESS)
        usbtmc_state.state = STATE_IDLE;
      TU_VERIFY(tud_control_xfer(rhport, request, (void*)&clearStatusRsp,sizeof(clearStatusRsp)));
      return true;
    }

  case USBTMC_bREQUEST_GET_CAPABILITIES:
    {
      TU_VERIFY(request->bmRequestType == 0xA1); // in,class,interface
      TU_VERIFY(request->wLength == sizeof(tud_usbtmc_app_capabilities));
      TU_VERIFY(tud_control_xfer(rhport, request, (void*)&tud_usbtmc_app_capabilities, sizeof(tud_usbtmc_app_capabilities)));
      return true;
    }
  // USBTMC Optional Requests

  case USBTMC_bREQUEST_INDICATOR_PULSE: // Optional
    {
      TU_VERIFY(request->bmRequestType == 0xA1); // in,class,interface
      TU_VERIFY(request->wLength == sizeof(tmcStatusCode));
      TU_VERIFY(tud_usbtmc_app_capabilities.bmIntfcCapabilities.supportsIndicatorPulse);
      TU_VERIFY(tud_usbtmc_app_indicator_pluse_cb(rhport, request, &tmcStatusCode));
      TU_VERIFY(tud_control_xfer(rhport, request, (void*)&tmcStatusCode, sizeof(tmcStatusCode)));
      return true;
    }
#if (CFG_USBTMC_CFG_ENABLE_488)

    // USB488 required requests
  case USB488_bREQUEST_READ_STATUS_BYTE:
    {
      usbtmc_read_stb_rsp_488_t rsp;
      TU_VERIFY(request->bmRequestType == 0xA1); // in,class,interface
      TU_VERIFY(request->wLength == sizeof(rsp)); // in,class,interface

      bTag = request->wValue & 0x7F;
      TU_VERIFY(request->bmRequestType == 0xA1);
      TU_VERIFY((request->wValue & (~0x7F)) == 0u); // Other bits are required to be zero
      TU_VERIFY(bTag >= 0x02 && bTag <= 127);
      TU_VERIFY(request->wIndex == usbtmc_state.itf_id);
      TU_VERIFY(request->wLength == 0x0003);
      rsp.bTag = (uint8_t)bTag;
      if(usbtmc_state.ep_int_in != 0)
      {
        rsp.USBTMC_status = USBTMC_STATUS_SUCCESS;
        rsp.statusByte = 0x00; // Use interrupt endpoint, instead.

        usbtmc_read_stb_interrupt_488_t intMsg =
        {
          .bNotify1 = {
              .one = 1,
              .bTag = bTag & 0x7Fu,
          },
          .StatusByte = tud_usbtmc_app_get_stb_cb(rhport, &(rsp.USBTMC_status))
        };
        usbd_edpt_xfer(rhport, usbtmc_state.ep_int_in, (void*)&intMsg, sizeof(intMsg));
      }
      else
      {
        rsp.statusByte = tud_usbtmc_app_get_stb_cb(rhport, &(rsp.USBTMC_status));
      }
      TU_VERIFY(tud_control_xfer(rhport, request, (void*)&rsp, sizeof(rsp)));
      return true;
    }
    // USB488 optional requests
  case USB488_bREQUEST_REN_CONTROL:
  case USB488_bREQUEST_GO_TO_LOCAL:
  case USB488_bREQUEST_LOCAL_LOCKOUT:
    {
      TU_VERIFY(request->bmRequestType == 0xA1); // in,class,interface
      TU_VERIFY(false);
      return false;
    }
#endif

  default:
    TU_VERIFY(false);
    return false;
  }
  TU_VERIFY(false);
}

bool usbtmcd_control_complete_cb(uint8_t rhport, tusb_control_request_t const * request)
{
  (void)rhport;
  //------------- Class Specific Request -------------//
  TU_VERIFY (request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS);

  return true;
}

#endif /* CFG_TUD_TSMC */
