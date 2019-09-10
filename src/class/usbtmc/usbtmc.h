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
typedef enum {
  USBTMC_MSGID_DEV_DEP_MSG_OUT = 1u
  #define USBTMC_MSGID_DEV_DEP_MSG_IN = 2u,
  #define USBTMC_MSGID_VENDOR_SPECIFIC_MSG_OUT = 126u,
  #define USBTMC_MSGID_VENDOR_SPECIFIC_IN = 127u,
  #define USBTMC_MSGID_USB488_TRIGGER = 128u
} usbtmc_msgid_enum;

/// \brief Message header (For BULK OUT); 4 bytes
typedef struct TU_ATTR_PACKED
{
  uint8_t MsgID              ; ///< Message type ID (usbtmc_msgid_enum)
  uint8_t bTag    		       ; ///< Transfer ID 1<=bTag<=255
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
  uint8_t _reserved[3];
} usbtmc_msg_request_dev_dep_out;

// Next 8 bytes are message-specific
typedef struct TU_ATTR_PACKED {
  usbtmc_msg_header_t header ; ///< Header
  uint32_t TransferSize      ; ///< Transfer size; LSB first
  struct {
      uint8_t : 0;
      uint8_t TermCharEnabled  : 1 ; ///< "The Bulk-IN transfer must terminate on the specified TermChar."; CAPABILITIES must list TermChar
  } bmTransferAttributes;
  uint8_t TermChar;
  uint8_t _reserved[2];
} usbtmc_msg_request_dev_dep_in;



typedef struct TU_ATTR_PACKED {
  usbtmc_msg_header_t header ; ///< Header
  uint32_t TransferSize      ; ///< Transfer size; LSB first
  uint8_t _reserved[4];
} usbtmc_msg_request_vendor_specific_out;


typedef struct TU_ATTR_PACKED {
  usbtmc_msg_header_t header ; ///< Header
  uint32_t TransferSize      ; ///< Transfer size; LSB first
  uint8_t _reserved[4];
} usbtmc_msg_request_vendor_specific_in;

// Control request type should use tusb_control_request_t

/*
typedef struct TU_ATTR_PACKED {
  struct {
    uint8_t Recipient  : 5         ; ///< EOM set on last byte
    uint8_t Type       : 2         ; ///< EOM set on last byte
    uint8_t DirectionToHost  : 1   ; ///< 0 is OUT, 1 is IN
  } bmRequestType;
  uint8_t bRequest                 ; ///< If bmRequestType.Type = Class, see usmtmc_request_type_enum
  uint16_t wValue                  ;
  uint16_t wIndex                  ;
  uint16_t wLength                 ; // Number of bytes in data stage
} usbtmc_class_specific_control_req;

*/
// bulk-in protocol errors
enum {
  USBTMC_BULK_IN_ERR_INCOMPLETE_HEADER = 1u,
  USBTMC_BULK_IN_ERR_UNSUPPORTED = 2u,
  USBTMC_BULK_IN_ERR_BAD_PARAMETER = 3u,
  USBTMC_BULK_IN_ERR_DATA_TOO_SHORT = 4u,
  USBTMC_BULK_IN_ERR_DATA_TOO_LONG = 5u,
};
// bult-in halt errors
enum {
  USBTMC_BULK_IN_ERR = 1u, ///< receives a USBTMC command message that expects a response while a
                           /// Bulk-IN transfer is in progress
};

typedef enum {
  USBTMC_bREQUEST_INITIATE_ABORT_BULK_OUT      = 1u,
  USBTMC_bREQUEST_CHECK_ABORT_BULK_OUT_STATUS  = 2u,
  USBTMC_bREQUEST_INITIATE_ABORT_BULK_IN       = 3u,
  USBTMC_bREQUEST_CHECK_ABORT_BULK_IN_STATUS   = 4u,
  USBTMC_bREQUEST_INITIATE_CLEAR               = 5u,
  USBTMC_bREQUEST_CHECK_CLEAR_STATUS           = 6u,
  USBTMC_bREQUEST_GET_CAPABILITIES             = 7u,

  USBTMC_bREQUEST_INDICATOR_PULSE               = 64u, // Optional
} usmtmc_request_type_enum;

typedef enum {
  USBTMC488_bREQUEST_READ_STATUS_BYTE  = 128u,
  USBTMC488_bREQUEST_REN_CONTROL       = 160u,
  USBTMC488_bREQUEST_GO_TO_LOCAL       = 161u,
  USBTMC488_bREQUEST_LOCAL_LOCKOUT     = 162u,
};

enum {
  USBTMC_STATUS_SUCCESS = 0x01,
  USBTMC_STATUS_PENDING = 0x02,
  USBTMC_STATUS_FAILED = 0x80,
  USBTMC_STATUS_TRANSFER_NOT_IN_PROGRESS = 0x81,
  USBTMC_STATUS_SPLIT_NOT_IN_PROGRESS = 0x82,
  USBTMC_STATUS_SPLIT_IN_PROGRESS  = 0x83
};

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

#endif

