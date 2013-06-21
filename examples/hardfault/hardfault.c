/**************************************************************************//**
 * @file
 * @brief Hardfault handler for Cortex-M3
 * @author Joseph Yiu, Frank Van Hooft, Energy Micro AS
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
#include "hardfault.h"
#include "em_device.h"

/**************************************************************************//**
 * @brief Enable trapping of divison by zero
 *****************************************************************************/
void HardFault_TrapDivByZero(void)
{
  volatile uint32_t *confctrl = (uint32_t *) 0xE000ED14;

  *confctrl |= (1<<4);
}


/**************************************************************************//**
 * @brief  Enable trapping of unaligned memory accesses
 *****************************************************************************/
void HardFault_TrapUnaligned(void)
{
  volatile uint32_t *confctrl = (uint32_t *) 0xE000ED14;

  *confctrl |= (1<<3);
}

/**************************************************************************//**
 * @brief  Exception handler for Cortex-M3 hard faults
 * @note This code is from http://blog.frankvh.com/, based on Joseph Yiu's
 *       hardfault code. For Keil MDKARM, this assembly code needs to be added
 *       as a separate assembly function, as the RVDS compiler cannot do inline
 *       assembly of thumb instructions required for Cortex-M3.
 *****************************************************************************/
void HardFault_Handler(void)
{
  /*
   * Get the appropriate stack pointer, depending on our mode,
   * and use it as the parameter to the C handler. This function
   * will never return
   */
  __asm("TST   LR, #4");
  __asm("ITE   EQ");
  __asm("MRSEQ R0, MSP");
  __asm("MRSNE R0, PSP");
  __asm("B HardFault_HandlerC");
}

/**************************************************************************//**
 * @brief Exception handler for Cortex-M3 hard faults
 * @param[in] hardfault_args Stack frame location
 * @note  From Joseph Yiu, minor edits by FVH and Energy Micro AS.
 *        Hard fault handler in C
 *****************************************************************************/
void HardFault_HandlerC(uint32_t *stack_pointer)
{
  uint32_t stacked_r0;
  uint32_t stacked_r1;
  uint32_t stacked_r2;
  uint32_t stacked_r3;
  uint32_t stacked_r12;
  uint32_t stacked_lr;
  uint32_t stacked_pc;
  uint32_t stacked_psr;

  stacked_r0 = ((uint32_t) stack_pointer[0]);
  stacked_r1 = ((uint32_t) stack_pointer[1]);
  stacked_r2 = ((uint32_t) stack_pointer[2]);
  stacked_r3 = ((uint32_t) stack_pointer[3]);

  stacked_r12 = ((uint32_t) stack_pointer[4]);
  stacked_lr =  ((uint32_t) stack_pointer[5]);
  stacked_pc =  ((uint32_t) stack_pointer[6]);
  stacked_psr = ((uint32_t) stack_pointer[7]);

  printf("\n\n[HardFault]\n");
  printf("R0        = %08x\n", (unsigned int)stacked_r0);
  printf("R1        = %08x\n", (unsigned int)stacked_r1);
  printf("R2        = %08x\n", (unsigned int)stacked_r2);
  printf("R3        = %08x\n", (unsigned int)stacked_r3);
  printf("R12       = %08x\n", (unsigned int)stacked_r12);
  printf("LR [R14]  = %08x - Subroutine Call return address\n", (unsigned int)stacked_lr);
  printf("PC [R15]  = %08x - Program Counter\n", (unsigned int)stacked_pc);
  printf("PSR       = %08x\n", (unsigned int)stacked_psr);
  printf("BFAR      = %08x - Bus Fault SR/Address causing bus fault\n",
         (unsigned int) (*((volatile uint32_t *)(0xE000ED38))));
  printf("CFSR      = %08x - Config. Fault SR\n",
         (unsigned int) (*((volatile uint32_t *)(0xE000ED28))));
  if((*((volatile uint32_t *)(0xE000ED28)))&(1<<25))
  {
    printf("  :UsageFault->DivByZero\n");
  }
  if((*((volatile uint32_t *)(0xE000ED28)))&(1<<24))
  {
    printf("  :UsageFault->Unaligned access\n");
  }
  if((*((volatile uint32_t *)(0xE000ED28)))&(1<<18))
  {
    printf("  :UsageFault->Integrity check error\n");
  }
  if((*((volatile uint32_t *)(0xE000ED28)))&(1<<0))
  {
    printf("  :MemFault->Data access violation\n");
  }
  if((*((volatile uint32_t *)(0xE000ED28)))&(1<<0))
  {
    printf("  :MemFault->Instruction access violation\n");
  }
  printf("HFSR      = %08x - Hard Fault SR\n",
         (unsigned int)(*((volatile uint32_t *)(0xE000ED2C))));
  if((*((volatile uint32_t *)(0xE000ED2C)))&(1UL<<1))
  {
    printf("  :VECTBL, Failed vector fetch\n");
  }
  if((*((volatile uint32_t *)(0xE000ED2C)))&(1UL<<30))
  {
    printf("  :FORCED, Bus fault/Memory management fault/usage fault\n");
  }
  if((*((volatile uint32_t *)(0xE000ED2C)))&(1UL<<31))
  {
    printf("  :DEBUGEVT, Hard fault triggered by debug event\n");
  }
  printf("DFSR      = %08x - Debug Fault SR\n", (unsigned int)(*((volatile uint32_t *)(0xE000ED30))));
  printf("MMAR      = %08x - Memory Manage Address R\n", (unsigned int)(*((volatile uint32_t *)(0xE000ED34))));
  printf("AFSR      = %08x - Auxilirary Fault SR\n", (unsigned int)(*((volatile uint32_t *)(0xE000ED3C))));
  printf("SCB->SHCSR= %08x - System Handler Control and State R (exception)\n", (unsigned int)SCB->SHCSR);

  while(1);
}
