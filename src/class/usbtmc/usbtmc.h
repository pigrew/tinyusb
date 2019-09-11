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

#define USBTMC_VERSION 0x0100
#define USBTMC_488_VERSION 0x0100

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
} usbtmc_request_type_488_enum;

typedef enum {
  USBTMC_STATUS_SUCCESS = 0x01,
  USBTMC_STATUS_PENDING = 0x02,
  USBTMC_STATUS_FAILED = 0x80,
  USBTMC_STATUS_TRANSFER_NOT_IN_PROGRESS = 0x81,
  USBTMC_STATUS_SPLIT_NOT_IN_PROGRESS = 0x82,
  USBTMC_STATUS_SPLIT_IN_PROGRESS  = 0x83
} usbtmc_status_enum;

/************************************************************
 * Control Responses
 */

typedef struct TU_ATTR_PACKED {
  uint8_t USBTMC_status;                 ///< usbtmc_status_enum
  uint8_t _reserved;
  uint16_t bcdUSBTMC;                    ///< USBTMC_VERSION

  struct {
    uint8_t listenOnly :1;
    uint8_t talkOnly :1;
    uint8_t supportsIndicatorPulse :1;
  } bmIntfcCapabilities;
  struct {
    uint8_t canEndBulkInOnTermChar :1;
  } bmDevCapabilities;
  uint8_t _reserved2[6];
  uint8_t _reserved3[12];
} usbtmc_response_capabilities;

TU_VERIFY_STATIC(sizeof(usbtmc_response_capabilities) == 0x18, "struct wrong length");

typedef struct TU_ATTR_PACKED {
  uint8_t USBTMC_status;                 ///< usbtmc_status_enum
  uint8_t _reserved;
  uint16_t bcdUSBTMC;                    ///< USBTMC_VERSION

  struct {
    uint8_t listenOnly :1;
    uint8_t talkOnly :1;
    uint8_t supportsIndicatorPulse :1;
  } bmIntfcCapabilities;

  struct {
    uint8_t canEndBulkInOnTermChar :1;
  } bmDevCapabilities;

  uint8_t _reserved2[6];
  uint16_t bcdUSB488;

  struct {
    uint8_t is488_2 :1;
    uint8_t supportsREN_GTL_LLO :1;
    uint8_t supportsTrigger :1;
  } bmIntfcCapabilities488;
  struct {
    uint8_t SCPI :1;
    uint8_t SR1 :1;
    uint8_t RL1 :1;
    uint8_t DT1 :1;
  } bmDevCapabilities488;
  uint8_t _reserved3[8];
} usbtmc_response_capabilities_488;

TU_VERIFY_STATIC(sizeof(usbtmc_response_capabilities) == 0x18, "struct wrong length");
#endif

