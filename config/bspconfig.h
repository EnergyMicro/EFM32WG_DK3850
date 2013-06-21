/***************************************************************************//**
 * @file
 * @brief Provide BSP (board support package) configuration parameters.
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
#ifndef __BSPCONFIG_H
#define __BSPCONFIG_H

#define BSP_DK
#define BSP_DK_3201
#define BSP_MCUBOARD_3800

#define BSP_MCUBOARD_USB
#define BSP_USB_STATUSLED_PORT  gpioPortE
#define BSP_USB_STATUSLED_PIN   1
#define BSP_USB_OCFLAG_PORT     gpioPortE
#define BSP_USB_OCFLAG_PIN      2
#define BSP_USB_VBUSEN_PORT     gpioPortF
#define BSP_USB_VBUSEN_PIN      5

#include "bsp_dk_bcreg_3201.h"

#define BSP_DK_LEDS
#define BSP_NO_OF_LEDS  16
#define BSP_LED_MASK    0xFFFF
#define BSP_LED_PORT    (&BC_REGISTER->UIF_LEDS)

#define BSP_INIT_DEFAULT  BSP_INIT_DK_EBI

#endif
