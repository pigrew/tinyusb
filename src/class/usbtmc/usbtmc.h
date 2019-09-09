#ifndef _TUSB_USBTMC_H__
#define _TUSB_USBTMC_H__

#include "common/tusb_common.h"

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
  // Next 8 bytes are message-specific
} usbtmc_msg_header_t;

typedef struct TU_ATTR_PACKED {
	usbtmc_msg_header_t header ; ///< Header
	uint32_t TransferSize      ; ///< Transfer size; LSB first
	struct {
      uint8_t EOM  : 1         ; ///< EOM set on last byte
      uint8_t : 0;
  } bmTransferAttributes;
} usbtmc_msg_request_dev_dep_msg_out;







#endif