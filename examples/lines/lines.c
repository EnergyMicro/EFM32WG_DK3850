/**************************************************************************//**
 * @file
 * @brief Basic TFT Direct drive example for EFM32WG990F256/EFM32WG_DK3850
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
#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_cmu.h"
#include "em_ebi.h"
#include "em_gpio.h"
#include "bsp.h"
#include "bsp_trace.h"
#include "tftdirect.h"

/* Color component shifts for RGB-565 */
#define RED_SHIFT      11
#define GREEN_SHIFT    5
#define BLUE_SHIFT     0

/* Display size */
#define WIDTH          320
#define HEIGHT         240

/* Configure TFT direct drive from EBI BANK2 */
const EBI_TFTInit_TypeDef tftInit =
{ ebiTFTBank2,                  /* Select EBI Bank 2 */
  ebiTFTWidthHalfWord,          /* Select 2-byte (16-bit RGB565) increments */
  ebiTFTColorSrcMem,            /* Use memory as source for mask/blending */
  ebiTFTInterleaveUnlimited,    /* Unlimited interleaved accesses */
  ebiTFTFrameBufTriggerVSync,   /* VSYNC as frame buffer update trigger */
  false,                        /* Drive DCLK from negative edge of internal clock */
  ebiTFTMBDisabled,             /* No masking and alpha blending enabled */
  ebiTFTDDModeExternal,         /* Drive from external memory */
  ebiActiveLow,                 /* CS Active Low polarity */
  ebiActiveHigh,                /* DCLK Active High polarity */
  ebiActiveLow,                 /* DATAEN Active Low polarity */
  ebiActiveLow,                 /* HSYNC Active Low polarity */
  ebiActiveLow,                 /* VSYNC Active Low polarity */
  320,                          /* Horizontal size in pixels */
  1,                            /* Horizontal Front Porch */
  30,                           /* Horizontal Back Porch */
  2,                            /* Horizontal Synchronization Pulse Width */
  240,                          /* Vertical size in pixels */
  1,                            /* Vertical Front Porch */
  4,                            /* Vertical Back Porch */
  2,                            /* Vertical Synchronization Pulse Width */
  0x0000,                       /* Frame Address pointer offset to EBI memory base */
  8,                            /* DCLK Period */
  0,                            /* DCLK Start cycles */
  0,                            /* DCLK Setup cycles */
  0,                            /* DCLK Hold cycles */
};

/** Counts 1 ms timeTicks */
static volatile uint32_t msTicks;

/**************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 *****************************************************************************/
void SysTick_Handler(void)
{
  msTicks++;       /* increment counter necessary in Delay()*/
}


/**************************************************************************//**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 *****************************************************************************/
void Delay(uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) ;
}


/**************************************************************************//**
 * @brief  Draw line pattern into frame buffer
 * @param[in] frameBuffer Start of memory address to draw lines into
 *****************************************************************************/
void TFT_DrawScreen(uint16_t *frameBuffer)
{
  int        x, y;
  uint16_t   color;
  static int red    = 0, green = 0, blue = 0;
  static int redInc = 2, greenInc = 1, blueInc = 1;

  /* Draw blended horizontal lines */
  for (y = 0; y < HEIGHT; y++)
  {
    /* Blend colors over horizontal lines */
    color = (((blue * (HEIGHT - y) + red * y) / HEIGHT) << RED_SHIFT);
    if (y & 0x40)
    {
      color |= (green << GREEN_SHIFT) / (64 - (y % 64));
    }
    else
    {
      color |= (green << GREEN_SHIFT) / (y % 64);
    }
    color |= (((red * (HEIGHT - y) + blue * y) / HEIGHT) << BLUE_SHIFT);

    /* Draw horizontal line at y position */
    for (x = 0; x < WIDTH; x++)
    {
      *frameBuffer++ = color;
    }
  }
  /* Change colors slightly */
  red   = (red + redInc) % 32;
  green = (green + greenInc) % 64;
  blue  = (blue + blueInc) % 32;
}


/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  uint16_t *frameBuffer;
  bool     redraw;

  /* Configure for 48MHz HFXO operation of core clock */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))
  {
    while (1) ;
  }

  /* Initialize EBI banks (Board Controller, external PSRAM, ..) */
  BSP_Init(BSP_INIT_DEFAULT);

  /* If first word of user data page is non-zero, enable eA Profiler trace */
  BSP_TraceProfilerSetup();

  /* Indicate we are waiting for AEM button state enable EFM32WG */
  BSP_LedsSet(0x8001);
  while (BSP_RegisterRead(&BC_REGISTER->UIF_AEM) != BC_UIF_AEM_EFM)
  {
    /* Show a short "strobe light" on DK LEDs, indicating wait */
    BSP_LedsSet(0x8001);
    Delay(200);
    BSP_LedsSet(0x4002);
    Delay(50);
  }

  /* Set frame buffer start address */
  frameBuffer = (uint16_t *) EBI_BankAddress(EBI_BANK2);

  /* Loop demo forever */
  while (1)
  {
    redraw = TFT_DirectInit(&tftInit);
    if (redraw)
    {
      /* Inidicate that we are in active drawing mode */
      BSP_LedsSet(0x0000);

      /* Update frame buffer */
      TFT_DrawScreen(frameBuffer);

      /* Wait slightly before next update */
      Delay(100);
    }
    else
    {
      /* Sleep - no need to update display */
      BSP_LedsSet(0x8001);
      Delay(200);
    }
  }
}
