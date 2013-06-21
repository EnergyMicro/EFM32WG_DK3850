/**************************************************************************//**
 * @file main.c
 * @brief Vendor unique USB device example.
 * @author Energy Micro AS
 * @version 3.20.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012 Energy Micro AS, http://www.energymicro.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 4. The source and compiled code may only be used on Energy Micro "EFM32"
 *    microcontrollers and "EFR4" radios.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 *****************************************************************************/
#include "em_device.h"
#include "em_cmu.h"
#include "em_usb.h"
#include "bsp.h"
#include "bsp_trace.h"
#include "retargetserial.h"
#include <stdio.h>

/**************************************************************************//**
 *
 * This example shows how a vendor unique device can be implemented.
 * A vendor unique device is a device which does not belong to any
 * USB class.
 *
 * Use the file EFM32_Vendor_Unique_Device.inf to install libusb device driver
 * on the host PC. This file reside in example subdirectory:
 * ./host/libusb/efm32-vendor-unique-device-1.2.5.0
 *
 *****************************************************************************/

/*** Typedef's and defines. ***/

#define LED0            1
#define LED1            2
#define LED2            4
#define LED3            8
#define LED4            16

#define VND_GET_LEDS    0x10
#define VND_SET_LED     0x11

/*** Function prototypes. ***/

static void ConsoleDebugInit(void);
static int  SetupCmd(const USB_Setup_TypeDef *setup);

/*** Include device descriptor definitions. ***/

#include "descriptors.h"

/**************************************************************************//**
 * @brief main - the entrypoint after reset.
 *****************************************************************************/
int main(void)
{
  BSP_Init(BSP_INIT_DEFAULT);   /* Initialize DK board register access     */

  /* If first word of user data page is non-zero, enable eA Profiler trace */
  BSP_TraceProfilerSetup();

  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

  BSP_LedsSet(0);

  ConsoleDebugInit();           /* Initialize DK UART port. */

  printf("\nEFM32 USB LED Vendor Unique Device example\n");

  USBD_Init(&initstruct);

  /*
   * When using a debugger it is pratical to uncomment the following three
   * lines to force host to re-enumerate the device.
   */
  /* USBD_Disconnect(); */
  /* USBTIMER_DelayMs( 1000 ); */
  /* USBD_Connect(); */

  for (;;)
  {
  }
}

/**************************************************************************//**
 * @brief
 *   Handle USB setup commands.
 *
 * @param[in] setup Pointer to the setup packet received.
 *
 * @return USB_STATUS_OK if command accepted.
 *         USB_STATUS_REQ_UNHANDLED when command is unknown, the USB device
 *         stack will handle the request.
 *****************************************************************************/
static int SetupCmd(const USB_Setup_TypeDef *setup)
{
  int             retVal;
  uint16_t        leds;
  static uint32_t buffer;
  uint8_t         *pBuffer = (uint8_t*) &buffer;

  retVal = USB_STATUS_REQ_UNHANDLED;

  if (setup->Type == USB_SETUP_TYPE_VENDOR)
  {
    switch (setup->bRequest)
    {
    case VND_GET_LEDS:
      /********************/
      *pBuffer = BSP_LedsGet() & 0x1F;
      retVal   = USBD_Write(0, pBuffer, setup->wLength, NULL);
      break;

    case VND_SET_LED:
      /********************/
      leds = BSP_LedsGet() & 0x1F;
      if (setup->wValue)
      {
        leds |= LED0 << setup->wIndex;
      }
      else
      {
        leds &= ~(LED0 << setup->wIndex);
      }
      BSP_LedsSet(leds);
      putchar('.');
      retVal = USB_STATUS_OK;
      break;
    }
  }

  return retVal;
}

/**************************************************************************//**
 * @brief Initialize console I/O redirection.
 *****************************************************************************/
void ConsoleDebugInit(void)
{
  RETARGET_SerialInit();                        /* Initialize USART   */
  RETARGET_SerialCrLf(1);                       /* Map LF to CRLF     */
}
