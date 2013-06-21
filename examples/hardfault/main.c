/**************************************************************************//**
 * @file
 * @brief HARDFAULT exception handler example for EFM32WG_DK3850
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
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "em_device.h"
#include "bsp.h"
#include "bsp_trace.h"
#include "retargetserial.h"
#include "hardfault.h"

/* Note! You can set compile time define -DRETARGET_LEUART1 to build this
 * example to use LEUART instead of default UART1. See retargetserial.h for
 * details. */

/**************************************************************************//**
 * @brief  Dummy function to trigger mem access hardfault
 * @return Return value is just a dummy
 *****************************************************************************/
int BadFunctionAccess(void)
{
  int data = 0x00020100;

  memset(&data-4, 0xff, 20);

  return data;
}

/**************************************************************************//**
 * @brief  Dummy function to trigger div by zero
 * @return Return value is just a dummy
 *****************************************************************************/
int BadFunctionDiv(void)
{
  uint32_t data = DEVINFO->UNIQUEL;
  volatile int x = 0;

  return data / x;
}

/**************************************************************************//**
 * @brief  Dummy function to trigger unaligned access
 * @return Return value is just a dummy
 *****************************************************************************/
uint32_t BadFunctionUnaligned(void)
{
  uint32_t *data = (uint32_t *)((uint8_t *)&DEVINFO->UNIQUEL + 1);

  return *data;
}


/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Initialize DK board register access */
  BSP_Init(BSP_INIT_DEFAULT);

  /* If first word of user data page is non-zero, enable eA Profiler trace */
  BSP_TraceProfilerSetup();

  /* Initialize USART and map LF to CRLF */
  RETARGET_SerialInit();
  RETARGET_SerialCrLf(1);

  printf("\nEFM32WG_DK3850 HardFault handler example\n");

  HardFault_TrapDivByZero();
  HardFault_TrapUnaligned();

  /* Enable one of these function calls */
  // BadFunctionAccess();
  // BadFunctionDiv();
  BadFunctionUnaligned();

  printf("Did we ever get here?");

  while(1);
}
