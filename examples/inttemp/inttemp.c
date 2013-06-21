/**************************************************************************//**
 * @file
 * @brief Internal temperature sensor example for EFM32WG_DK3850
 * @details
 *   Show temperature using internal sensor on the EFM32.
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
#include <stdio.h>
#include "em_device.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_adc.h"
#include "em_cmu.h"
#include "bsp.h"
#include "bsp_trace.h"
#include "rtcdrv.h"
#include "retargetserial.h"

/** Interrupt pin used to detect joystick activity */
#define GPIO_INT_PIN    0

/** Flag used to indicate if displaying in Celsius or Fahrenheit */
static int showFahrenheit;

/* Local Prototypes */
void temperatureIRQInit(void);
void gpioSetup(void);
void setupSensor(void);
float convertToCelsius(uint32_t adcSample);
float convertToFahrenheit(uint32_t adcSample);

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
 * @brief ADC0 interrupt handler. Simply clears interrupt flag.
 *****************************************************************************/
void ADC0_IRQHandler(void)
{
  ADC_IntClear(ADC0, ADC_IF_SINGLE);
}

/**************************************************************************//**
 * @brief Initialize ADC for temperature sensor readings in single poin
 *****************************************************************************/
void setupSensor(void)
{
  /* Base the ADC configuration on the default setup. */
  ADC_Init_TypeDef       init  = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef sInit = ADC_INITSINGLE_DEFAULT;

  /* Initialize timebases */
  init.timebase = ADC_TimebaseCalc(0);
  init.prescale = ADC_PrescaleCalc(400000, 0);
  ADC_Init(ADC0, &init);

  /* Set input to temperature sensor. Reference must be 1.25V */
  sInit.reference = adcRef1V25;
  sInit.input     = adcSingleInpTemp;
  ADC_InitSingle(ADC0, &sInit);

  /* Setup interrupt generation on completed conversion. */
  ADC_IntEnable(ADC0, ADC_IF_SINGLE);
  NVIC_EnableIRQ(ADC0_IRQn);
}

/**************************************************************************//**
 * @brief Convert ADC sample values to celsius.
 * @note See section 2.3.4 in the reference manual for details on this
 *       calculatoin
 * @param adcSample Raw value from ADC to be converted to celsius
 * @return The temperature in degrees Celsius.
 *****************************************************************************/
float convertToCelsius(uint32_t adcSample)
{
  float temp;
  /* Factory calibration temperature from device information page. */
  float cal_temp_0 = (float)((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK)
                             >> _DEVINFO_CAL_TEMP_SHIFT);

  float cal_value_0 = (float)((DEVINFO->ADC0CAL2
                               & _DEVINFO_ADC0CAL2_TEMP1V25_MASK)
                              >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);

  /* Temperature gradient (from datasheet) */
  float t_grad = -6.27;

  temp = cal_temp_0 - ((cal_value_0 - adcSample) / t_grad);

  return temp;
}

/**************************************************************************//**
 * @brief Convert ADC sample values to fahrenheit
 * @param adcSample Raw value from ADC to be converted to fahrenheit
 * @return The temperature in degrees Fahrenheit
 *****************************************************************************/
float convertToFahrenheit(uint32_t adcSample)
{
  float celsius;
  float fahrenheit;
  celsius = convertToCelsius(adcSample);

  fahrenheit = (celsius * (9.0 / 5.0)) + 32.0;

  return fahrenheit;
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  uint8_t prod_rev;
  uint32_t temp;
  uint32_t temp_offset;

  /* Initialize DK board register access */
  BSP_Init(BSP_INIT_DEFAULT);

  /* If first word of user data page is non-zero, enable eA Profiler trace */
  BSP_TraceProfilerSetup();

  /* Initialize TFT */
  RETARGET_SerialInit();
  printf("\nEFM32 onchip temperature sensor example\n\n");

  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_ADC0, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Enable board control interrupts */
  BSP_InterruptDisable(0xffff);
  BSP_InterruptFlagsClear(0xffff);
  BSP_InterruptEnable(BC_INTEN_JOYSTICK);
  temperatureIRQInit();

  /* This is a work around for Chip Rev.D Errata, Revision 0.6. */
  /* Check for product revision 16 and 17 and set the offset */
  /* for ADC0_TEMP_0_READ_1V25. */
  prod_rev = (DEVINFO->PART & _DEVINFO_PART_PROD_REV_MASK) >> _DEVINFO_PART_PROD_REV_SHIFT;
  if( (prod_rev == 16) || (prod_rev == 17) )
  {
    temp_offset = 112;
  }
  else
  {
    temp_offset = 0;
  }

  /* Setup ADC for sampling internal temperature sensor. */
  setupSensor();

  /* Main loop - just read temperature and update LCD */
  while (1)
  {
    /* Start one ADC sample */
    ADC_Start(ADC0, adcStartSingle);

    /* Wait in EM1 for ADC to complete */
    EMU_EnterEM1();

    /* Read sensor value */
    /* According to rev. D errata ADC0_TEMP_0_READ_1V25 should be decreased */
    /* by the offset  but it is the same if ADC reading is increased - */
    /* reference manual 28.3.4.2. */
    temp = ADC_DataSingleGet(ADC0) + temp_offset;

    /* Convert ADC sample to Fahrenheit / Celsius and print string to display */
    if (showFahrenheit)
    {
      printf("%1.1f F\n", convertToFahrenheit(temp));
    }
    else
    {
      printf("%1.1f C\n", convertToCelsius(temp));
    }

    /* Sleep for 2 seconds in EM 2 */
    RTCDRV_Trigger(2000, NULL);
    EMU_EnterEM2(true);
  }
}
