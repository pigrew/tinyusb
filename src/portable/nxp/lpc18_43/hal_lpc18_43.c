/**************************************************************************/
/*!
    @file     hal_lpc43xx.c
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, hathach (tinyusb.org)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    INCLUDING NEGLIGENCE OR OTHERWISE ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    This file is part of the tinyusb stack.
*/
/**************************************************************************/

#include "tusb.h"

#if (CFG_TUSB_MCU == OPT_MCU_LPC18XX || CFG_TUSB_MCU == OPT_MCU_LPC43XX)

#include "chip.h"

void tusb_hal_int_enable(uint8_t rhport)
{
  NVIC_EnableIRQ(rhport ? USB1_IRQn : USB0_IRQn);
}

void tusb_hal_int_disable(uint8_t rhport)
{
  NVIC_DisableIRQ(rhport ? USB1_IRQn : USB0_IRQn);
}

bool tusb_hal_init(void)
{
  return true;
}

void hal_dcd_isr(uint8_t rhport);

#if CFG_TUSB_RHPORT0_MODE
void USB0_IRQHandler(void)
{
  #if MODE_HOST_SUPPORTED
    hal_hcd_isr(0);
  #endif

  #if TUSB_OPT_DEVICE_ENABLED
    hal_dcd_isr(0);
  #endif
}
#endif

#if CFG_TUSB_RHPORT1_MODE
void USB1_IRQHandler(void)
{
  #if MODE_HOST_SUPPORTED
    hal_hcd_isr(1);
  #endif

  #if TUSB_OPT_DEVICE_ENABLED
    hal_dcd_isr(1);
  #endif
}
#endif

#endif
