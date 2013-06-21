/***************************************************************************//**
 * @file
 * @brief Provide stdio retargeting configuration parameters.
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
#ifndef __RETARGETSERIALCONFIG_H
#define __RETARGETSERIALCONFIG_H

#include "bsp.h"

/* Override if needed with commandline parameter -DRETARGET_xxx */

#if !defined(RETARGET_UART1) && !defined(RETARGET_LEUART1)
#define RETARGET_UART1    /* Use UART1 by default. */
#endif

#if defined(RETARGET_UART1)
  #define RETARGET_IRQ_NAME    UART1_RX_IRQHandler          /* UART IRQ Handler */
  #define RETARGET_CLK         cmuClock_UART1               /* HFPER Clock */
  #define RETARGET_IRQn        UART1_RX_IRQn                /* IRQ number */
  #define RETARGET_UART        UART1                        /* UART instance */
  #define RETARGET_TX          USART_Tx                     /* Set TX to USART_Tx */
  #define RETARGET_RX          USART_Rx                     /* Set RX to USART_Rx */
  #define RETARGET_LOCATION    USART_ROUTE_LOCATION_LOC2    /* Location of of the USART I/O pins */
  #define RETARGET_TXPORT      gpioPortB                    /* UART transmission port */
  #define RETARGET_TXPIN       9                            /* UART transmission pin */
  #define RETARGET_RXPORT      gpioPortB                    /* UART reception port */
  #define RETARGET_RXPIN       10                           /* UART reception pin */
  #define RETARGET_USART       1                            /* Includes em_usart.h */
  #define RETARGET_PERIPHERAL_ENABLE()  BSP_PeripheralAccess(BSP_RS232_UART, true)

#elif defined(RETARGET_LEUART1)
  #define RETARGET_IRQ_NAME    LEUART1_IRQHandler           /* LEUART IRQ Handler */
  #define RETARGET_CLK         cmuClock_LEUART1             /* LFB Clock */
  #define RETARGET_IRQn        LEUART1_IRQn                 /* IRQ number */
  #define RETARGET_UART        LEUART1                      /* LEUART instance */
  #define RETARGET_TX          LEUART_Tx                    /* Set TX to LEUART_Tx */
  #define RETARGET_RX          LEUART_Rx                    /* Set RX to LEUART_Rx */
  #define RETARGET_TXPORT      gpioPortC                    /* LEUART transmission port */
  #define RETARGET_TXPIN       6                            /* LEUART transmission pin */
  #define RETARGET_RXPORT      gpioPortC                    /* LEUART reception port */
  #define RETARGET_RXPIN       7                            /* LEUART reception pin */
  #define RETARGET_LOCATION    LEUART_ROUTE_LOCATION_LOC0   /* Location of of the LEUART I/O pins */
  #define RETARGET_LEUART      1                            /* Includes em_leuart.h */
  #define RETARGET_PERIPHERAL_ENABLE()  BSP_PeripheralAccess(BSP_RS232_LEUART, true)

#else
#error "Illegal UART/LEUART selection."
#endif

#endif
