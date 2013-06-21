/**************************************************************************//**
 * @file main.c
 * @brief USB HID keyboard device example.
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
#include "em_usb.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "bsp.h"
#include "bsp_trace.h"
#include "retargetserial.h"
#include <stdio.h>

/**************************************************************************//**
 *
 * This example shows how a HID keyboard can be implemented.
 *
 *****************************************************************************/

/*** Typedef's and defines. ***/

#define POLL_TIMER              0
#define DEFAULT_POLL_TIMEOUT    24
#define HEARTBEAT_MASK          0xF
#define KEYLED_MASK             0x8000
#define KBDLED_MASK             0xF00
#define INTR_IN_EP_ADDR         0x81
#define ACTIVITY_LED            gpioPortE,1 /* The blue led labeled STATUS. */
#define BUTTON                  gpioPortE,0


/*** Function prototypes. ***/

static int  OutputReportReceived(USB_Status_TypeDef status,
                                 uint32_t xferred,
                                 uint32_t remaining);
static int  SetupCmd(const USB_Setup_TypeDef *setup);
static void StateChange(USBD_State_TypeDef oldState,
                        USBD_State_TypeDef newState);


/*** Include device descriptor definitions. ***/

#include "descriptors.h"

/*** Variables ***/

static int      keySeqNo;           /* Current position in report table */
static bool     keyPushed;          /* Current pushbutton status  */
static int      pollTimeout;        /* Key poll rate, unit is ms. */
static uint8_t  idleRate;
static uint16_t leds;
static uint32_t tmpBuffer;

/**************************************************************************//**
 * @brief main - the entrypoint after reset.
 *****************************************************************************/
int main(void)
{
#if defined(BUSPOWERED)
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(BUTTON, gpioModeInputPull, 1);
  GPIO_PinModeSet(ACTIVITY_LED, gpioModePushPull,  0);
#else
  BSP_Init(BSP_INIT_DEFAULT);   /* Initialize DK board register access     */

  /* If first word of user data page is non-zero, enable eA Profiler trace */
  BSP_TraceProfilerSetup();
#endif

  CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFXO );

  leds = 0;
#if !defined(BUSPOWERED)
  BSP_LedsSet(leds);

  /* Initialize console I/O redirection. */
  RETARGET_SerialInit();        /* Initialize DK UART port */
  RETARGET_SerialCrLf( 1 );     /* Map LF to CRLF          */

  printf("\nEFM32 USB HID Keyboard device example\n");
#endif

  USBD_Init(&initstruct);

  /*
   * When using a debugger it is practical to uncomment the following three
   * lines to force host to re-enumerate the device.
   */
  /* USBD_Disconnect();      */
  /* USBTIMER_DelayMs(1000); */
  /* USBD_Connect();         */

  for (;;)
  {
  }
}

/**************************************************************************//**
 * @brief
 *   Called on timer elapsed event. This function is called at a rate set
 *   by the host driver with the SET_IDLE setup command.
 *****************************************************************************/
static void PollTimeout(void)
{
  bool pushed;

  /* Check pushbutton */
#if defined(BUSPOWERED)
  pushed = GPIO_PinInGet( BUTTON ) == 0;
#else
  pushed = BSP_PushButtonsGet() & 1;
#endif

  /* Update LED's */
  leds = (leds & ~(HEARTBEAT_MASK | KEYLED_MASK)) |
         (((leds & HEARTBEAT_MASK) + 1) & HEARTBEAT_MASK) |
         (pushed ? KEYLED_MASK : 0);

#if defined(BUSPOWERED)
  if (!keyPushed)
    GPIO_PinOutToggle(ACTIVITY_LED);
#else
  BSP_LedsSet(leds);
#endif

  if (pollTimeout != 0)     /* Send report with current key state */
  {
    if (pushed)
    {
      /* Send a key pushed report */
      USBD_Write(INTR_IN_EP_ADDR, (void*) &reportTable[ keySeqNo ],
                 sizeof(KeyReport_TypeDef), NULL);
    }
    else
    {
      /* Send an empty (key released) report */
      USBD_Write(INTR_IN_EP_ADDR,
                 (void*) &noKeyReport, sizeof(KeyReport_TypeDef), NULL);
    }
  }
  else        /* pollTimeout == 0, only send report on key status change */
  {
    if (pushed != keyPushed)        /* Any change ? */
    {
      if (pushed)
      {
        /* Send a key pushed report */
        USBD_Write(INTR_IN_EP_ADDR, (void*) &reportTable[ keySeqNo ],
                   sizeof(KeyReport_TypeDef), NULL);
      }
      else
      {
        /* Send an empty (key released) report */
        USBD_Write(INTR_IN_EP_ADDR,
                   (void*) &noKeyReport, sizeof(KeyReport_TypeDef), NULL);
      }
    }
  }

  /* Keep track of the new keypush event (if any) */
  if (pushed && !keyPushed)
  {
    /* Advance to next position in report table */
    keySeqNo++;
    if (keySeqNo == (sizeof(reportTable) / sizeof(KeyReport_TypeDef)))
    {
      keySeqNo = 0;
    }
#if defined(BUSPOWERED)
    GPIO_PinOutSet(ACTIVITY_LED);
#else
    putchar('.');
#endif
  }
  keyPushed = pushed;

  /* Restart HID poll timer */
  if (pollTimeout)
  {
    USBTIMER_Start(POLL_TIMER, pollTimeout, PollTimeout);
  }
  else
  {
    USBTIMER_Start(POLL_TIMER, DEFAULT_POLL_TIMEOUT, PollTimeout);
  }
}

/**************************************************************************//**
 * @brief
 *   Callback function called each time the USB device state is changed.
 *   Starts HID operation when device has been configured by USB host.
 *
 * @param[in] oldState The device state the device has just left.
 * @param[in] newState The new device state.
 *****************************************************************************/
static void StateChange(USBD_State_TypeDef oldState,
                        USBD_State_TypeDef newState)
{
  if (newState == USBD_STATE_CONFIGURED)
  {
    /* We have been configured, start HID functionality ! */
    if (oldState != USBD_STATE_SUSPENDED)   /* Resume ?   */
    {
      leds        = 0;
      keySeqNo    = 0;
      keyPushed   = false;
      idleRate    = DEFAULT_POLL_TIMEOUT / 4;
      pollTimeout = DEFAULT_POLL_TIMEOUT;
#if defined(BUSPOWERED)
      GPIO_PinOutSet(ACTIVITY_LED);
#else
      BSP_LedsSet(leds);
#endif
    }
    USBTIMER_Start(POLL_TIMER, DEFAULT_POLL_TIMEOUT, PollTimeout);
  }

  else if ((oldState == USBD_STATE_CONFIGURED) &&
           (newState != USBD_STATE_SUSPENDED))
  {
    /* We have been de-configured, stop HID functionality */
    USBTIMER_Stop(POLL_TIMER);
#if defined(BUSPOWERED)
    GPIO_PinOutClear(ACTIVITY_LED);
#endif
  }

  else if (newState == USBD_STATE_SUSPENDED)
  {
    /* We have been suspended, stop HID functionality */
    /* Reduce current consumption to below 2.5 mA.    */
#if defined(BUSPOWERED)
    GPIO_PinOutClear(ACTIVITY_LED);
#endif
    USBTIMER_Stop(POLL_TIMER);
  }

#if !defined(BUSPOWERED)
  putchar('\n');
  printf(USBD_GetUsbStateName(oldState));
  printf(" -> ");
  printf(USBD_GetUsbStateName(newState));
#endif
}

/**************************************************************************//**
 * @brief
 *   Handle USB setup commands. Implements HID class specific commands.
 *
 * @param[in] setup Pointer to the setup packet received.
 *
 * @return USB_STATUS_OK if command accepted.
 *         USB_STATUS_REQ_UNHANDLED when command is unknown, the USB device
 *         stack will handle the request.
 *****************************************************************************/
static int SetupCmd(const USB_Setup_TypeDef *setup)
{
  STATIC_UBUF( hidDesc, USB_HID_DESCSIZE );

  int retVal = USB_STATUS_REQ_UNHANDLED;

  if ((setup->Type == USB_SETUP_TYPE_STANDARD) &&
      (setup->Direction == USB_SETUP_DIR_IN) &&
      (setup->Recipient == USB_SETUP_RECIPIENT_INTERFACE))
  {
    /* A HID device must extend the standard GET_DESCRIPTOR command   */
    /* with support for HID descriptors.                              */
    switch (setup->bRequest)
    {
    case GET_DESCRIPTOR:
      /********************/
      if ((setup->wValue >> 8) == USB_HID_REPORT_DESCRIPTOR)
      {
        USBD_Write(0, (void*) ReportDescriptor,
                   EFM32_MIN(sizeof(ReportDescriptor), setup->wLength),
                   NULL);
        retVal = USB_STATUS_OK;
      }
      else if ((setup->wValue >> 8) == USB_HID_DESCRIPTOR)
      {
        /* The HID descriptor might be misaligned ! */
        memcpy( hidDesc,
                &configDesc[ USB_CONFIG_DESCSIZE + USB_INTERFACE_DESCSIZE ],
                USB_HID_DESCSIZE );
        USBD_Write(0, hidDesc, EFM32_MIN(USB_HID_DESCSIZE, setup->wLength),
                   NULL);
        retVal = USB_STATUS_OK;
      }
      break;
    }
  }

  else if ((setup->Type == USB_SETUP_TYPE_CLASS) &&
           (setup->Recipient == USB_SETUP_RECIPIENT_INTERFACE))
  {
    /* Implement the necessary HID class specific commands.           */
    switch (setup->bRequest)
    {
    case USB_HID_SET_REPORT:
      /********************/
      if (((setup->wValue >> 8) == 2) &&            /* Output report */
          ((setup->wValue & 0xFF) == 0) &&          /* Report ID     */
          (setup->wIndex == 0) &&                   /* Interface no. */
          (setup->wLength == 1) &&                  /* Report length */
          (setup->Direction != USB_SETUP_DIR_IN))
      {
        USBD_Read(0, (void*) &tmpBuffer, 1, OutputReportReceived);
        retVal = USB_STATUS_OK;
      }
      break;

    case USB_HID_GET_REPORT:
      /********************/
      if (((setup->wValue >> 8) == 1) &&            /* Input report  */
          ((setup->wValue & 0xFF) == 0) &&          /* Report ID     */
          (setup->wIndex == 0) &&                   /* Interface no. */
          (setup->wLength == 8) &&                  /* Report length */
          (setup->Direction == USB_SETUP_DIR_IN))
      {
        if (keyPushed)
        {
          /* Send a key pushed report */
          USBD_Write(0, (void*) &reportTable[ keySeqNo ],
                     sizeof(KeyReport_TypeDef), NULL);
        }
        else
        {
          /* Send an empty (key released) report */
          USBD_Write(0, (void*) &noKeyReport,
                     sizeof(KeyReport_TypeDef), NULL);
        }
        retVal = USB_STATUS_OK;
      }
      break;

    case USB_HID_SET_IDLE:
      /********************/
      if (((setup->wValue & 0xFF) == 0) &&          /* Report ID     */
          (setup->wIndex == 0) &&                   /* Interface no. */
          (setup->wLength == 0) &&
          (setup->Direction != USB_SETUP_DIR_IN))
      {
        idleRate    = setup->wValue >> 8;
        pollTimeout = 4 * idleRate;
        if (pollTimeout > DEFAULT_POLL_TIMEOUT)
        {
          pollTimeout = DEFAULT_POLL_TIMEOUT;
        }
        retVal = USB_STATUS_OK;
      }
      break;

    case USB_HID_GET_IDLE:
      /********************/
      if ((setup->wValue == 0) &&                   /* Report ID     */
          (setup->wIndex == 0) &&                   /* Interface no. */
          (setup->wLength == 1) &&
          (setup->Direction == USB_SETUP_DIR_IN))
      {
        *(uint8_t*)&tmpBuffer = idleRate;
        USBD_Write(0, (void*) &tmpBuffer, 1, NULL);
        retVal = USB_STATUS_OK;
      }
      break;
    }
  }

  return retVal;
}

/**************************************************************************//**
 * @brief
 *   Callback function called when the data stage of a USB_HID_SET_REPORT
 *   setup command has completed.
 *
 * @param[in] status    Transfer status code.
 * @param[in] xferred   Number of bytes transferred.
 * @param[in] remaining Number of bytes not transferred.
 *
 * @return USB_STATUS_OK.
 *****************************************************************************/
static int OutputReportReceived(USB_Status_TypeDef status,
                                uint32_t xferred,
                                uint32_t remaining)
{
  uint8_t *p = (uint8_t*) &tmpBuffer;
  (void) remaining;

  /* We have received new data for NumLock, CapsLock and ScrollLock LED's */
  if ((status == USB_STATUS_OK) && (xferred == 1))
  {
    leds = (leds & ~KBDLED_MASK) | (*p << 8);
#if !defined(BUSPOWERED)
    BSP_LedsSet(leds);
    putchar('.');
#endif
  }
  return USB_STATUS_OK;
}
