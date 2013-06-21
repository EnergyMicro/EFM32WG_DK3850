/***************************************************************************//**
 * @file
 * @brief Provide stdio retargeting to USART/UART or LEUART.
 * @author Energy Micro AS
 * @version 3.20.0
 *******************************************************************************
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
#include <stdio.h>
#include "em_device.h"
#include "em_cmu.h"
#include "em_int.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "bsp.h"
#include "bsp-uart.h"

/***************************************************************************//**
 * @addtogroup Drivers
 * @{
 ******************************************************************************/


/* Receive buffer */
#define RXBUFSIZE    8
static volatile int     rxReadIndex  = 0;
static volatile int     rxWriteIndex = 0;
static volatile int     rxCount      = 0;
static volatile uint8_t rxBuffer[RXBUFSIZE];
static uint8_t          LFtoCRLF    = 0;
static bool             initialized = false;

/**************************************************************************//**
 * @brief UART/LEUART IRQ Handler
 *****************************************************************************/
void UART1_RX_IRQHandler(void)
{
  if (UART1->STATUS & USART_STATUS_RXDATAV)
  {
    /* Store Data */
    rxBuffer[rxWriteIndex] = USART_Rx(UART1);
    rxWriteIndex++;
    rxCount++;
    if (rxWriteIndex == RXBUFSIZE)
    {
      rxWriteIndex = 0;
    }
    /* Check for overflow - flush buffer */
    if (rxCount > RXBUFSIZE)
    {
      rxWriteIndex = 0;
      rxCount      = 0;
      rxReadIndex  = 0;
    }
  }
}


/**************************************************************************//**
 * @brief UART/LEUART toggle LF to CRLF conversion
 * @param on If non-zero, automatic LF to CRLF conversion will be enabled
 *****************************************************************************/
void UART1_SerialCrLf(int on)
{
  if (on)
    LFtoCRLF = 1;
  else
    LFtoCRLF = 0;
}


/**************************************************************************//**
 * @brief Intializes UART/LEUART
 *****************************************************************************/
void UART1_SerialInit(void)
{
  /* Configure GPIO pins */
  CMU_ClockEnable(cmuClock_GPIO, true);
  /* To avoid false start, configure output as high */
  GPIO_PinModeSet(gpioPortB, 9, gpioModePushPull, 1);
  GPIO_PinModeSet(gpioPortB, 10, gpioModeInput, 0);

  USART_TypeDef           *usart = UART1;
  USART_InitAsync_TypeDef init   = USART_INITASYNC_DEFAULT;

  /* Enable EFM32WG_DK3850 RS232/UART switch */
  BSP_PeripheralAccess(BSP_RS232_UART, true);

  /* Enable peripheral clocks */
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_UART1, true);

  /* Configure USART for basic async operation */
  init.enable = usartDisable;
  USART_InitAsync(usart, &init);

  /* Enable pins at UART1 location #2 */
  usart->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | USART_ROUTE_LOCATION_LOC2;

  /* Clear previous RX interrupts */
  USART_IntClear(UART1, USART_IF_RXDATAV);
  NVIC_ClearPendingIRQ(UART1_RX_IRQn);

  /* Enable RX interrupts */
  USART_IntEnable(UART1, USART_IF_RXDATAV);
  NVIC_EnableIRQ(UART1_RX_IRQn);

  /* Finally enable it */
  USART_Enable(usart, usartEnable);

#if !defined(__CROSSWORKS_ARM) && defined(__GNUC__)
  setvbuf(stdout, NULL, _IONBF, 0);   /*Set unbuffered mode for stdout (newlib)*/
#endif

  initialized = true;
}


/**************************************************************************//**
 * @brief Receive a byte from USART/LEUART and put into global buffer
 * @return -1 on failure, or positive character integer on sucesss
 *****************************************************************************/
int UART1_ReadChar(void)
{
  int c = -1;

  INT_Disable();
  if (rxCount > 0)
  {
    c = rxBuffer[rxReadIndex];
    rxReadIndex++;
    if (rxReadIndex == RXBUFSIZE)
    {
      rxReadIndex = 0;
    }
    rxCount--;
  }
  INT_Enable();

  return c;
}

/**************************************************************************//**
 * @brief Transmit single byte to USART/LEUART
 * @param data Character to transmit
 *****************************************************************************/
int UART1WriteChar(char c)
{
  if (initialized == false)
  {
    UART1_SerialInit();
  }

  /* Add CR or LF to CRLF if enabled */
  if (LFtoCRLF && (c == '\n'))
  {
    USART_Tx(UART1, '\r');
  }
  USART_Tx(UART1, c);

  if (LFtoCRLF && (c == '\r'))
  {
    USART_Tx(UART1, '\n');
  }

  return c;
}

/** @} (end group Drivers) */
