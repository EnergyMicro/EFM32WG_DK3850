/**************************************************************************//**
 * @file
 * @brief UART/LEUART/RS232 example for EFM32WG_DK3850 development kit
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
#include "em_device.h"
#include "em_emu.h"
#include "bsp.h"
#include "bsp_trace.h"
#include "retargetserial.h"

/* Note! You can set compile time define -DRETARGET_LEUART1 to build this
 * example to use LEUART instead of default UART1. See retargetserial.h for
 * details. */

/** RS232 input buffer size */
#define ECHOBUFSIZE    80
/** RS232 Input buffer */
char echoBuffer[ECHOBUFSIZE];

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  int c;
  int index;

  /* Initialize DK board register access */
  BSP_Init(BSP_INIT_DEFAULT);

  /* If first word of user data page is non-zero, enable eA Profiler trace */
  BSP_TraceProfilerSetup();

  /* Initialize USART and map LF to CRLF */
  RETARGET_SerialInit();
  RETARGET_SerialCrLf(1);

#ifdef RETARGET_LEUART1
  printf("\nEFM32WG_DK3850 LEUART1 example\n");
#else
  printf("\nEFM32WG_DK3850 UART1 example\n");
#endif
  for (index = 0; index < ECHOBUFSIZE; index++)
  {
    echoBuffer[index] = (char) 'a' + index;
  }

  /* Retrieve characters, print local echo and full line back */
  index = 0;
  while (1)
  {
    /* Retrieve new character */
    c = getchar();
    if (c > 0)
    {
      /* Output character - most terminals use CRLF */
      if (c == '\r')
      {
        echoBuffer[index] = '\0';
        /* Output entire line */
        printf("\n%s\n", echoBuffer);
        index = 0;
      }
      else
      {
        /* Filter non-printable characters */
        if ((c < ' ') || (c > '~'))
          continue;

        /* Enter into buffer */
        echoBuffer[index] = c;
        index++;
        if (index == ECHOBUFSIZE)
        {
          /* Flush buffer */
          index = 0;
        }
        /* Local echo */
        putchar(c);
      }
    }
    else
    {
      #ifdef RETARGET_LEUART1
        /* Enter EM2 when idle */
        EMU_EnterEM2(true);
      #else
        /* Enter EM1 when idle */
        EMU_EnterEM1();
      #endif
    }
  }
}
