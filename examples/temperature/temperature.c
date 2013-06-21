/**************************************************************************//**
 * @file
 * @brief Temperature example for EFM32WG_DK3850
 * @details
 *   Show temperature using I2C sensor on DK.
 *
 * @par Usage
 * @li Joystick Push toggles Celsius/Fahrenheit display mode.
 *
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
#include <stdlib.h>
#include <stdio.h>
#include "em_device.h"
#include "em_emu.h"
#include "em_dbg.h"
#include "em_gpio.h"
#include "bsp.h"
#include "bsp_trace.h"
#include "rtcdrv.h"
#include "i2cdrv.h"
#include "tempsens.h"
#include "retargetserial.h"

/** Interrupt pin used to detect joystick activity */
#define GPIO_INT_PIN    0

/** Flag used to indicate if displaying in Celsius or Fahrenheit */
static int showFahrenheit;

/* Local prototypes */
void temperatureIRQInit(void);
void temperatureUpdate(TEMPSENS_Temp_TypeDef *temp);

/**************************************************************************//**
 * @brief GPIO Interrupt handler
 * This interrupt handler is not an example of good design, as it will do
 * a lot of operations inside the interrupt handler.
 *****************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  uint16_t joystick;

  /* Clear interrupt */
  BSP_InterruptFlagsClear(BC_INTEN_JOYSTICK);
  GPIO_IntClear(1 << GPIO_INT_PIN);

  /* LEDs on to indicate joystick used */
  BSP_LedsSet(0xffff);

  /* Read and store joystick activity - wait for key release */
  joystick = BSP_JoystickGet();
  while (BSP_JoystickGet()) ;

  /* LEDs off to indicate joystick release */
  BSP_LedsSet(0x0000);

  /* Push toggles celsius/fahrenheit */
  if (joystick & BC_UIF_JOYSTICK_CENTER)
  {
    showFahrenheit ^= 1;
  }
}


/**************************************************************************//**
 * @brief Initialize GPIO interrupt for joystick (ie FPGA signal)
 *****************************************************************************/
void temperatureIRQInit(void)
{
  /* Configure interrupt pin as input with pull-up */
  GPIO_PinModeSet(gpioPortE, GPIO_INT_PIN, gpioModeInputPull, 1);

  /* Set falling edge interrupt and clear/enable it */
  GPIO_IntConfig(gpioPortE, GPIO_INT_PIN, false, true, true);

  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}


/**************************************************************************//**
 * @brief Update TFT with temperature
 * @param[in] temp Temperature to display.
 *****************************************************************************/
void temperatureUpdate(TEMPSENS_Temp_TypeDef *temp)
{
  TEMPSENS_Temp_TypeDef dtemp;

  /* Work with local copy in case conversion to Fahrenheit is required */
  dtemp = *temp;

  if (showFahrenheit)
  {
    TEMPSENS_Celsius2Fahrenheit(&dtemp);
  }

  /* Round temperature to nearest 0.5 */
  if (dtemp.f >= 0)
  {
    dtemp.i += (dtemp.f + 2500) / 10000;
    dtemp.f  = (((dtemp.f + 2500) % 10000) / 5000) * 5000;
  }
  else
  {
    dtemp.i += (dtemp.f - 2500) / 10000;
    dtemp.f  = (((dtemp.f - 2500) % 10000) / 5000) * 5000;
  }

  if ((dtemp.i < 0) || (dtemp.f < 0))
  {
    putchar('-');
  }
  else
  {
    putchar('+');
  }
  printf("%d.%d %c\n", abs(dtemp.i), (int)(abs(dtemp.f) / 1000),
         showFahrenheit ? 'F' : 'C');
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  I2C_Init_TypeDef      i2cInit = I2C_INIT_DEFAULT;
  TEMPSENS_Temp_TypeDef temp;
  /* Define previous temp to invalid, just to ensure update first time */
  TEMPSENS_Temp_TypeDef prevTemp = {1000, 0};
  int                   prevShowFahrenheit = showFahrenheit;

  /* Initialize DK board register access */
  BSP_Init(BSP_INIT_DEFAULT);

  /* If first word of user data page is non-zero, enable eA Profiler trace */
  BSP_TraceProfilerSetup();

  /* Initialize TFT */
  RETARGET_SerialInit();
  printf("\nEFM32 I2C temperature sensor example\n\n");

  /* Enable board control interrupts */
  BSP_InterruptDisable(0xffff);
  BSP_InterruptFlagsClear(0xffff);
  BSP_InterruptEnable(BC_INTEN_JOYSTICK);
  temperatureIRQInit();

  /* Initialize I2C driver, using standard rate. Devices on DK itself */
  /* supports fast mode, but in case some slower devices are added on */
  /* prototype board, we use standard mode. */
  I2CDRV_Init(&i2cInit);

  /* Main loop - just read temperature and update LCD */
  while (1)
  {
    if (TEMPSENS_TemperatureGet(I2C0,
                                TEMPSENS_DK_ADDR,
                                &temp) < 0)
    {
      printf("ERROR\n");
      /* Enter EM2, no wakeup scheduled */
      EMU_EnterEM2(true);
    }

    /* Update LCD display if any change. This is just an example of how */
    /* to save some energy, since the temperature normally is quite static. */
    /* The compare overhead is much smaller than actually updating the display. */
    if ((prevTemp.i != temp.i) ||
        (prevTemp.f != temp.f) ||
        (prevShowFahrenheit != showFahrenheit))
    {
      temperatureUpdate(&temp);
    }
    prevTemp           = temp;
    prevShowFahrenheit = showFahrenheit;

    /* Read every 2 seconds which is more than it takes worstcase to */
    /* finish measurement inside sensor. */
    RTCDRV_Trigger(2000, NULL);
    EMU_EnterEM2(true);
  }
}
