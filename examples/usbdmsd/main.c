/**************************************************************************//**
 * @file main.c
 * @brief Mass Storage Device example.
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
#include "em_usb.h"
#include "em_assert.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "bsp.h"
#include "bsp_trace.h"
#include "retargetserial.h"
#include "msdd.h"
#include "msddmedia.h"
#include "usbconfig.h"

/**************************************************************************//**
 *
 * This example shows how a Mass Storage Device (MSD) can be implemented.
 *
 * Different kinds of media can be used for data storage. Modify the
 * MSD_MEDIA #define macro in msdmedia.h to select between the different ones.
 *
 *****************************************************************************/

#if defined( BUSPOWERED )
#if ( ( MSD_MEDIA==MSD_PSRAM_MEDIA ) || ( MSD_MEDIA==MSD_SDCARD_MEDIA ) )
#error "Illegal combination of BUSPOWERED and MSD_MEDIA type."
#endif
#endif

/**************************************************************************//**
 * @brief main - the entrypoint after reset.
 *****************************************************************************/
int main( void )
{
#if !defined(BUSPOWERED)
  BSP_Init(BSP_INIT_DEFAULT);   /* Initialize DK board register access */

  /* If first word of user data page is non-zero, enable eA Profiler trace */
  BSP_TraceProfilerSetup();
#endif

  CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFXO );
  CMU_OscillatorEnable(cmuOsc_LFXO, true, false);

#if !defined(BUSPOWERED)
  RETARGET_SerialInit();        /* Initialize DK UART port */
  RETARGET_SerialCrLf( 1 );     /* Map LF to CRLF          */
  printf( "\nEFM32 Mass Storage Device example\n" );
#endif

  if ( !MSDDMEDIA_Init() )
  {
#if !defined(BUSPOWERED)
    printf( "\nMedia error !\n" );
#endif
    EFM_ASSERT( false );
    for( ;; ){}
  }

  MSDD_Init(gpioPortE, 1);

  for (;;)
  {
    if ( MSDD_Handler() )
    {
      /* There is no pending activity in the MSDD handler.  */
      /* Enter sleep mode to conserve energy.               */

      if ( USBD_SafeToEnterEM2() )
        EMU_EnterEM2(true);
      else
        EMU_EnterEM1();
    }
  }
}
