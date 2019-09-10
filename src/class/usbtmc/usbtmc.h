#ifndef _TUSB_USBTMC_H__
#define _TUSB_USBTMC_H__

#include "common/tusb_common.h"


/* Implements USBTMC Revision 1.0, April 14, 2003

 String descriptors must have a "LANGID=0x409"/US English string.
 Characters must be 0x20 (' ') to 0x7E ('~') ASCII,
   But MUST not contain: "/:?\*
   Also must not have leading or trailing space (' ')
 Device descriptor must state USB version 0x0200 or greater

 If USB488DeviceCapabilites.D2 = 1 (SR1), then there must be a INT endpoint.
*/
#define USBTMC_MSGID_DEV_DEP_MSG_OUT 1
#define USBTMC_MSGID_REQUEST_DEV_DEP_MSG_IN 2
#define USBTMC_MSGID_DEV_DEP_MSG_IN 2
#define USBTMC_MSGID_VENDOR_SPECIFIC_OUT 126
#define USBTMC_MSGID_REQUEST_VENDOR_SPECIFIC_IN 127
#define USBTMC_MSGID_VENDOR_SPECIFIC_IN 127
#define USBTMC_MSGID_USB488_TRIGGER 128

/// \brief Message header (For BULK OUT); 4 bytes
typedef struct TU_ATTR_PACKED
{
  uint8_t MsgID              ; ///< Message ID
  uint8_t bTag    			 ; ///< Transfer ID 1<=bTag<=255
  uint8_t bTagInverse        ; ///< Complement of the tag
  uint8_t Reserved           ; ///< Must be 0x00
} usbtmc_msg_header_t;

typedef struct TU_ATTR_PACKED
{
  usbtmc_msg_header_t header;
  uint8_t data[8];
} usbtmc_msg_generic_t;

// Next 8 bytes are message-specific
typedef struct TU_ATTR_PACKED {
	usbtmc_msg_header_t header ; ///< Header
	uint32_t TransferSize      ; ///< Transfer size; LSB first
	struct {
      uint8_t EOM  : 1         ; ///< EOM set on last byte
      uint8_t : 0;
  } bmTransferAttributes;
} usbtmc_msg_request_dev_dep_msg_out;

#define USBTMC_APP_CLASS    0xFE
#define USBTMC_APP_SUBCLASS 0x03

#define USBTMC_PROTOCOL_STD    0x00
#define SUBTMC_PROTOCOL_USB488 0x01
// Other pro

// CDC Descriptor Template
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

#endif

