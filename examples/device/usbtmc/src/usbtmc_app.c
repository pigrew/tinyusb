#include "class/usbtmc/usbtmc_device.h"

#include "uart_util.h"

usbtmc_response_capabilities_488_t const usbtmcd_app_capabilities  =
{
    .USBTMC_status = USBTMC_STATUS_SUCCESS,
    .bcdUSBTMC = USBTMC_VERSION,
    .bmIntfcCapabilities =
    {
        .listenOnly = 0,
        .talkOnly = 0,
        .supportsIndicatorPulse = 0
    },
    .bmDevCapabilities = {
        .canEndBulkInOnTermChar = 0
    },
    .bcdUSB488 = USBTMC_488_VERSION,
    .bmIntfcCapabilities488 =
    {
        .supportsTrigger = 0,
        .supportsREN_GTL_LLO = 0,
        .is488_2 = 1
    },
    .bmDevCapabilities488 =
    {
      .SCPI = 1,
      .SR1 = 0,
      .RL1 = 0,
      .DT1 =0,
    }
};

static const char idn[] = "TinyUSB,ModelNumber,SerialNumber,FirmwareVer";


bool usbtmcd_app_msgBulkOut_start(usbtmc_msg_request_dev_dep_out const * msgHeader) {
  (void)msgHeader;
  return true;
}


bool usbtmcd_app_msg_data(void *data, size_t len, bool transfer_complete) {
  (void)transfer_complete;
  uart_tx_sync(data,len);
  return true;
}

bool usbtmcd_app_msgBulkIn_complete(uint8_t rhport) {
  (void)rhport;
  return true;
}

bool usbtmcd_app_msgBulkIn_request(uint8_t rhport, usbtmc_msg_request_dev_dep_in const * request) {
  usbtmc_msg_dev_dep_msg_in_header_t hdr = {
      .header =
      {
          .MsgID = request->header.MsgID,
          .bTag = request->header.bTag,
          .bTagInverse = request->header.bTagInverse
      },
      .TransferSize = sizeof(idn),
      .bmTransferAttributes =
      {
        .EOM = 1,
        .UsingTermChar = 0
      }
  };

  usbtmcd_transmit_dev_msg_data(rhport, &hdr, idn);

  return true;
}


bool usbtmcd_app_get_stb_rsp(uint8_t rhport, usbtmc_read_stb_rsp_488_t *rsp) {
  (void)rhport;
  rsp->USBTMC_status = USBTMC_STATUS_SUCCESS;
  rsp->statusByte = 0xAB;
  return true;
}
