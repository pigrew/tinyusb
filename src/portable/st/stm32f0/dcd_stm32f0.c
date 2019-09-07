/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
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

// TODO: Implement double-buffering
// TODO: Priority is currently based on the EP#. Make it configurable.
// TODO: Packet sizes must be even. Verify it when opening endpoint?
// TODO: STM32F0 uses special buffer memory. Currently memcpy performed, instead of having the class directly write to buffer.

#include "tusb_option.h"

#if TUSB_OPT_DEVICE_ENABLED && CFG_TUSB_MCU == OPT_MCU_STM32F0

#include "device/dcd.h"
#include "stm32f0xx.h"
#include "uart_util.h"
char msg[100];

#define USB_ISTR_ALL_EVENTS (USB_ISTR_PMAOVR | USB_ISTR_ERR | USB_ISTR_WKUP | USB_ISTR_SUSP | \
     USB_ISTR_RESET | USB_ISTR_SOF | USB_ISTR_ESOF | USB_ISTR_L1REQ )

#define EPREG(n) (((__IO uint16_t*)USB_BASE)[n*2])

/* SetENDPOINT */
#define PCD_SET_ENDPOINT(USBx, bEpNum,wRegValue)  (*((__IO uint16_t *)(((uint32_t)(&(USBx)->EP0R + (bEpNum) * 2U))))= (uint16_t)(wRegValue))
/* GetENDPOINT */
#define PCD_GET_ENDPOINT(USBx, bEpNum)            (*((__IO uint16_t *)(((uint32_t)(&(USBx)->EP0R + (bEpNum) * 2U)))))
#define PCD_SET_EPTYPE(USBx, bEpNum,wType) (PCD_SET_ENDPOINT((USBx), (bEpNum),\
                                  (((((uint32_t)(PCD_GET_ENDPOINT((USBx), (bEpNum)))) & ((uint32_t)(USB_EP_T_MASK))) | ((uint32_t)(wType))) | USB_EP_CTR_RX | USB_EP_CTR_TX)))
#define PCD_GET_EPTYPE(USBx, bEpNum) (((uint16_t)(PCD_GET_ENDPOINT((USBx), (bEpNum)))) & USB_EP_T_FIELD)



#define PCD_EP_TX_ADDRESS(USBx, bEpNum) ((uint16_t *)((uint32_t)((((USBx)->BTABLE+(bEpNum)*8)+     ((uint32_t)(USBx) + 0x400U)))))
#define PCD_EP_TX_CNT(USBx, bEpNum) ((uint16_t *)((uint32_t)((((USBx)->BTABLE+(bEpNum)*8+2)+  ((uint32_t)(USBx) + 0x400U)))))

#define PCD_EP_RX_ADDRESS(USBx, bEpNum) ((uint16_t *)((uint32_t)((((USBx)->BTABLE+(bEpNum)*8+4)+ ((uint32_t)(USBx) + 0x400U)))))
#define PCD_EP_RX_CNT(USBx, bEpNum) ((uint16_t *)((uint32_t)((((USBx)->BTABLE+(bEpNum)*8+6)+  ((uint32_t)(USBx) + 0x400U)))))



/**
  * @brief  sets the status for tx transfer (bits STAT_TX[1:0]).
  * @param  USBx USB peripheral instance register address.
  * @param  bEpNum Endpoint Number.
  * @param  wState new state
  * @retval None
  */
#define PCD_SET_EP_TX_STATUS(USBx, bEpNum, wState) { register uint16_t _wRegVal;\
   \
    _wRegVal = (uint32_t) (((uint32_t)(PCD_GET_ENDPOINT((USBx), (bEpNum)))) & USB_EPTX_DTOGMASK);\
   /* toggle first bit ? */     \
   if((USB_EPTX_DTOG1 & (wState))!= 0U)\
   {                                                                            \
     _wRegVal ^=(uint16_t) USB_EPTX_DTOG1;        \
   }                                                                            \
   /* toggle second bit ?  */         \
   if((USB_EPTX_DTOG2 & ((uint32_t)(wState)))!= 0U)      \
   {                                                                            \
     _wRegVal ^=(uint16_t) USB_EPTX_DTOG2;        \
   }                                                                            \
   PCD_SET_ENDPOINT((USBx), (bEpNum), (((uint32_t)(_wRegVal)) | USB_EP_CTR_RX|USB_EP_CTR_TX));\
  } /* PCD_SET_EP_TX_STATUS */

/**
  * @brief  sets the status for rx transfer (bits STAT_TX[1:0])
  * @param  USBx USB peripheral instance register address.
  * @param  bEpNum Endpoint Number.
  * @param  wState new state
  * @retval None
  */
#define PCD_SET_EP_RX_STATUS(USBx, bEpNum,wState) {\
    register uint16_t _wRegVal;   \
    \
    _wRegVal = (uint32_t) (((uint32_t)(PCD_GET_ENDPOINT((USBx), (bEpNum)))) & USB_EPRX_DTOGMASK);\
    /* toggle first bit ? */  \
    if((USB_EPRX_DTOG1 & (wState))!= 0U) \
    {                                                                             \
      _wRegVal ^= (uint16_t) USB_EPRX_DTOG1;  \
    }                                                                             \
    /* toggle second bit ? */  \
    if((USB_EPRX_DTOG2 & ((uint32_t)(wState)))!= 0U) \
    {                                                                             \
      _wRegVal ^= (uint16_t) USB_EPRX_DTOG2;  \
    }                                                                             \
    PCD_SET_ENDPOINT((USBx), (bEpNum), (((uint32_t)(_wRegVal)) | USB_EP_CTR_RX|USB_EP_CTR_TX)); \
  } /* PCD_SET_EP_RX_STATUS */


// WARNING: APB clock must be >=10 MHz

// EP Buffers assigned from end of memory location, to minimize their chance of crashing
// into the stack.
static uint16_t ep_buf_ptr;
static void dcd_handle_bus_reset();


//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

void dcd_init (uint8_t rhport)
{
  strcpy(msg,"\n\n\n\n\ndcd_init\n");
  uart_tx_sync(msg,strlen(msg));
  /* Clocks should already be enabled */
  /* Use __HAL_RCC_USB_CLK_ENABLE(); to enable the clocks before calling this function */

  /* The RM mentions to use a special ordering of PDWN and FRES, but this isn't done in HAL.
   * Here, the RM is followed. */

  for(uint32_t i = 0; i<200; i++) { // should be a few us
    asm("NOP");
  }
	// Perform USB peripheral reset
  USB->CNTR = USB_CNTR_FRES | USB_CNTR_PDWN;
  USB->CNTR &= ~(USB_CNTR_PDWN);// Remove powerdown
  // Wait startup time, for F042 and F070, this is <= 1 us.
  for(uint32_t i = 0; i<200; i++) { // should be a few us
    asm("NOP");
  }
  USB->CNTR = 0; // Enable USB

  USB->BTABLE = 0; // Remind it that BTABLE should start at offset 0 (which it should anyway after reset)

  USB->ISTR &= ~(USB_ISTR_ALL_EVENTS); // Clear pending interrupts

  // Clear all EP
  for(int i=0; i<8; i++) {
    EPREG(0) = 0u;
  }
  // Need to initialize the BTABLE for EP0 at this point (though setting up the EP0R is unneeded)
  //dcd_handle_bus_reset();

  USB->CNTR |= USB_CNTR_RESETM | USB_CNTR_SOFM | USB_CNTR_CTRM | USB_CNTR_SUSPM | USB_CNTR_WKUPM |
      USB_CNTR_ESOFM;
  dcd_handle_bus_reset();
  // And finally enable pull-up, which may trigger the RESET IRQ if the host is connected.
  USB->BCDR |= USB_BCDR_DPPU;
}

// Enable device interrupt
void dcd_int_enable (uint8_t rhport)
{

  NVIC_SetPriority(USB_IRQn, 0);
  NVIC_EnableIRQ(USB_IRQn);
}

// Disable device interrupt
void dcd_int_disable(uint8_t rhport)
{

  //sprintf(msg,"dcd_int_disable EP0REG=%04hx\n", USB->EP0R);
  //uart_tx_sync(msg, strlen(msg));
  NVIC_DisableIRQ(USB_IRQn);
}

// Receive Set Address request, mcu port must also include status IN response
void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{

  sprintf(msg,"dcd_set_address EP0REG=%04hx,addr=%02hx\n", USB->EP0R,(uint16_t)dev_addr);
  uart_tx_sync(msg, strlen(msg));
  USB->DADDR &= ~USB_DADDR_ADD;
  USB->DADDR |= dev_addr;
}

// Receive Set Config request
void dcd_set_config (uint8_t rhport, uint8_t config_num)
{

  sprintf(msg,"dcd_set_config config=%02hx, EP0REG=%04hx\n",(uint16_t)config_num, USB->EP0R);
  uart_tx_sync(msg, strlen(msg));
}

void dcd_remote_wakeup(uint8_t rhport)
{

  sprintf(msg,"dcd_remote_wakeup EP0REG=%04hx\n", USB->EP0R);
  uart_tx_sync(msg, strlen(msg));
  (void) rhport;
}

static const tusb_desc_endpoint_t ep0OUT_desc = {
    .wMaxPacketSize = CFG_TUD_ENDPOINT0_SIZE,
    .bDescriptorType = TUSB_XFER_CONTROL,
    .bEndpointAddress = 0x00
};

static const tusb_desc_endpoint_t ep0IN_desc = {
    .wMaxPacketSize = CFG_TUD_ENDPOINT0_SIZE,
    .bDescriptorType = TUSB_XFER_CONTROL,
    .bEndpointAddress = 0x80
};

static void dcd_handle_bus_reset() {
  //__IO uint16_t * const epreg = &(EPREG(0));
  ep_buf_ptr = 1024;
  dcd_edpt_open (0, &ep0OUT_desc);
  dcd_edpt_open (0, &ep0IN_desc);
  USB->DADDR |= USB_DADDR_EF; // Set enable flag
}

void dcd_fs_irqHandler(void) {
  //USB_OTG_OUTEndpointTypeDef * out_ep = OUT_EP_BASE;
  //USB_OTG_INEndpointTypeDef * in_ep = IN_EP_BASE;
  //uint32_t wInterrupt_Mask = 0U;
  sprintf(msg,"IRQ ISTR=%04hx;CNTR=%04hx; \n",USB->ISTR, USB->CNTR);
  uart_tx_sync(msg, strlen(msg));

  uint16_t int_status = USB->ISTR;
 // unused IRQs: (USB_ISTR_PMAOVR | USB_ISTR_ERR | USB_ISTR_WKUP | USB_ISTR_SUSP | USB_ISTR_ESOF | USB_ISTR_L1REQ )

  if (int_status & USB_ISTR_CTR)
  {
    sprintf(msg,"IRQ-CTR\n");
    uart_tx_sync(msg,strlen(msg));
    /* servicing of the endpoint correct transfer interrupt */
    /* clear of the CTR flag into the sub */
    //PCD_EP_ISR_Handler(hpcd);
    USB->ISTR &= ~USB_ISTR_CTR;
  }
  if(int_status & USB_ISTR_RESET) {
    sprintf(msg,"IRQ-RST\n");
    uart_tx_sync(msg,strlen(msg));
    // USBRST is start of reset.
    USB->ISTR &= ~USB_ISTR_RESET;
    dcd_handle_bus_reset();
    dcd_event_bus_signal(0, DCD_EVENT_BUS_RESET, true);
  }
  if (int_status & USB_ISTR_WKUP)
  {
    sprintf(msg,"IRQ-WKUP\n");
    uart_tx_sync(msg,strlen(msg));

    //USB->CNTR &= (uint16_t)(~(USB_CNTR_LPMODE));
    USB->CNTR &= ~USB_CNTR_FSUSP;


    USB->ISTR &= ~USB_ISTR_WKUP;
  }

  if (int_status & USB_ISTR_SUSP)
  {

    sprintf(msg,"IRQ-SUSP\n");
    uart_tx_sync(msg,strlen(msg));
    /* Force low-power mode in the macrocell */
    USB->CNTR |= USB_CNTR_FSUSP;
    //USB->CNTR |= USB_CNTR_LPMODE;

    /* clear of the ISTR bit must be done after setting of CNTR_FSUSP */
    USB->ISTR &= ~USB_ISTR_SUSP;


    if(int_status & USB_ISTR_SOF) {
      sprintf(msg,"IRQ-SOF\n");
      uart_tx_sync(msg,strlen(msg));
      USB->ISTR &= ~USB_ISTR_SOF;
      dcd_event_bus_signal(0, DCD_EVENT_SOF, true);
    }

    if(int_status & USB_ISTR_ESOF) {
      sprintf(msg,"IRQ-ESOF\n");
      uart_tx_sync(msg,strlen(msg));
      USB->ISTR &= ~USB_ISTR_ESOF;
    }

  }
  sprintf(msg,"IRQ-exit ISTR=%04hx; CNTR=%04hx; EP0R=%04hx\n",USB->ISTR, USB->CNTR, USB->EP0R);
  uart_tx_sync(msg,strlen(msg));
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

// The STM32F0 doesn't seem to like |= or &= to manipulate the EP#R registers,
// so I'm using the #define from HAL here, instead.

bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const * p_endpoint_desc)
{

  uint8_t const epnum = tu_edpt_number(p_endpoint_desc->bEndpointAddress);
  uint8_t const dir   = tu_edpt_dir(p_endpoint_desc->bEndpointAddress);

  sprintf(msg,"dcd_edpt_open epnum=%x, dir=%x,EP0REG=%04hx\n",epnum, dir, USB->EP0R);
  uart_tx_sync(msg, strlen(msg));
  // Unsupported endpoint numbers/size.
  if((p_endpoint_desc->wMaxPacketSize.size > 64) || (epnum >= 8)) {
    return false;
  }

 // __IO uint16_t * const epreg = &(EPREG(epnum));

  // Set type
  //*epreg &= ~(USB_EP_T_MASK | USB_EPKIND_MASK | USB_EPADDR_FIELD);
  switch(p_endpoint_desc->bDescriptorType) {
  case TUSB_XFER_CONTROL:
    PCD_SET_EPTYPE(USB, epnum, USB_EP_CONTROL); break;
  case TUSB_XFER_ISOCHRONOUS:
    PCD_SET_EPTYPE(USB, epnum, USB_EP_ISOCHRONOUS); break;
  case TUSB_XFER_BULK:
    PCD_SET_EPTYPE(USB, epnum, USB_EP_BULK); break;
  case TUSB_XFER_INTERRUPT:
    PCD_SET_EPTYPE(USB, epnum, USB_EP_INTERRUPT); break;
  }

  PCD_SET_EP_ADDRESS(USB, epnum, epnum);

  ep_buf_ptr -= p_endpoint_desc->wMaxPacketSize.size; // decrement buffer pointer

  if(dir == TUSB_DIR_IN) {
    *PCD_EP_TX_ADDRESS(USB,epnum) = ep_buf_ptr;
    *PCD_EP_TX_CNT(USB,epnum) = p_endpoint_desc->wMaxPacketSize.size;
    PCD_SET_EP_TX_STATUS(USB,epnum,USB_EP_TX_NAK);
  } else {
    *PCD_EP_RX_ADDRESS(USB,epnum) = ep_buf_ptr;
    *PCD_EP_RX_CNT(USB,epnum) = p_endpoint_desc->wMaxPacketSize.size;
    PCD_SET_EP_RX_STATUS(USB,epnum,USB_EP_RX_VALID);
  }
  sprintf(msg,"dcd_edpt_open-exit epnum=%x, dir=%x,EP0REG=%04hx\n",epnum, dir, USB->EP0R);
  uart_tx_sync(msg, strlen(msg));
  return true;
}

bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  sprintf(msg,"dcd_edpt_xfer ep_addr=%x, bytes=%x,EP0REG=%04hx\n",ep_addr, total_bytes, USB->EP0R);
  uart_tx_sync(msg, strlen(msg));
  return false;
}

void dcd_edpt_stall (uint8_t rhport, uint8_t ep_addr)
{
  sprintf(msg,"dcd_edpt_stall epaddr=%x, EP0REG=%04hx\n",ep_addr, USB->EP0R);
  uart_tx_sync(msg, strlen(msg));

}

void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr)
{
  sprintf(msg,"dcd_edpt_stall epaddr=%x, EP0REG=%04hx\n",ep_addr, USB->EP0R);
  uart_tx_sync(msg, strlen(msg));
}

#endif

