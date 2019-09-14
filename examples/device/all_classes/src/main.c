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
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board.h"
#include "tusb.h"
#include "cdc_app.h"
#include "midi_app.h"
#include "hid_app.h"
#include "webusb_app.h"
#include "main.h"
#include "usb_descriptors.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

#define STM32L4_SYNOPSYS    (                                                  \
    defined (STM32L475xx) || defined (STM32L476xx) ||                          \
    defined (STM32L485xx) || defined (STM32L486xx) || defined (STM32L496xx) || \
    defined (STM32L4R5xx) || defined (STM32L4R7xx) || defined (STM32L4R9xx) || \
    defined (STM32L4S5xx) || defined (STM32L4S7xx) || defined (STM32L4S9xx)    \
)

#if TUSB_OPT_DEVICE_ENABLED && \
    ( CFG_TUSB_MCU == OPT_MCU_STM32F2 || \
      CFG_TUSB_MCU == OPT_MCU_STM32F4 || \
      CFG_TUSB_MCU == OPT_MCU_STM32F7 || \
      CFG_TUSB_MCU == OPT_MCU_STM32H7 || \
      (CFG_TUSB_MCU == OPT_MCU_STM32L4 && STM32L4_SYNOPSYS) \
    )
#define DCD_SYN
#endif

#if (TUSB_OPT_DEVICE_ENABLED) && ( \
      ((CFG_TUSB_MCU) == OPT_MCU_STM32F0) || \
      (((CFG_TUSB_MCU) == OPT_MCU_STM32F1) && ( \
          defined(stm32f102x6) || defined(stm32f102xb) || \
          defined(stm32f103x6) || defined(stm32f103xb) || \
          defined(stm32f103xe) || defined(stm32f103xg) \
      )) || \
      ((CFG_TUSB_MCU) == OPT_MCU_STM32F3) \
    )
#define DCD_ST_FSDEV
#endif

void OTG_FS_IRQHandler(void);


uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);

/*------------- MAIN -------------*/
int main(void)
{
  board_init();

  tusb_init();

  while (1)
  {
    tud_task(); // tinyusb device task
    led_blinking_task();

    hid_task();
	midi_task();
	cdc_task();
    webserial_task();
	// Trigger a fake interrupt; not sure if the static analyzer needs this to
	// know that interrupt handler is not dead code?

#if ((CFG_TUSB_MCU) == (OPT_MCU_STM32F3)) && defined(DCD_ST_FSDEV)
	USB_HP_CAN_TX_IRQHandler();
#elif defined(DCD_SYN)
	OTG_FS_IRQHandler();
#else
	#error need to determine the interrupt handler....
#endif
  }

  return 0;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}

