/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                          (c) Copyright 2003-2010; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                         uC/OS-II example code
*                                       Main application function
*
*                                   Energy Micro EFM32 (EFM32WG990F256)
*                                              with the
*                                   Energy Micro DK3850 Development Kit
*
* @file   app.c           
* @brief
* @author Energy Micro AS
* @version 3.20.0
******************************************************************************
* @section License
* <b>(C) Copyright 2013 Energy Micro AS, http://www.energymicro.com</b>
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
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include <includes.h>


/*
*********************************************************************************************************
*                                      EXTERNAL GLOBAL VARIABLES
*********************************************************************************************************
*/

/* definition of global mailbox object for inter-task communication
 * extern declaration in includes.h */
OS_EVENT *pSerialMsgObj;


/*
*********************************************************************************************************
*                                         LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
static OS_STK App_TaskStartStk[APP_CFG_TASK_START_STK_SIZE];
static OS_STK App_TaskOneStk[APP_CFG_TASK_ONE_STK_SIZE];
static OS_STK App_TaskTwoStk[APP_CFG_TASK_TWO_STK_SIZE];
static OS_STK App_TaskThreeStk[APP_CFG_TASK_THREE_STK_SIZE];


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/
static void App_TaskStart (void *p_arg);
static void App_TaskCreate (void);
static void App_MailboxCreate(void);

/* static function for energyAware Profiler */
static void setupSWO(void);


/*
*********************************************************************************************************
*                                         FUNCTION DEFINITIONS
*********************************************************************************************************
*/
/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Argument(s) : none.
*
* Return(s)   : none.
*********************************************************************************************************
*/
int main(void)
{
#if (OS_TASK_NAME_EN > 0)
  CPU_INT08U  err;
#endif


  /* Disable all interrupts until we are ready to accept
   * them.                                                */
  CPU_IntDis();

  /* Chip errata */
  CHIP_Init();

  /* setup SW0 for energyAware Profiler */
  setupSWO();

  /* Initialize "uC/OS-II, The Real-Time Kernel".         */
  OSInit();

  /* Create the start task                                */
  OSTaskCreateExt((void (*)(void *)) App_TaskStart,
                  (void           *) 0,
                  (OS_STK         *)&App_TaskStartStk[APP_CFG_TASK_START_STK_SIZE - 1],
                  (INT8U           ) APP_CFG_TASK_START_PRIO,
                  (INT16U          ) APP_CFG_TASK_START_PRIO,
                  (OS_STK         *)&App_TaskStartStk[0],
                  (INT32U          ) APP_CFG_TASK_START_STK_SIZE,
                  (void           *) 0,
#ifdef __ICCARM__ /* IAR port */
                  (INT16U          )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR | OS_TASK_OPT_SAVE_FP));
#else
                  (INT16U          )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
#endif

#if (OS_TASK_NAME_EN > 0)
  OSTaskNameSet(APP_CFG_TASK_START_PRIO, "Start", &err);
#endif

  /* Start multitasking (i.e. give control to uC/OS-II).  */
  OSStart();

  /* OSStart() never returns, serious error had occured if
   * code execution reached this point                    */
  while(1) ;
}


/*
*********************************************************************************************************
*                                          AppTaskStart()
*
* Description : The startup task.  The uC/OS-II ticker should only be initialize once multitasking starts.
*
* Argument(s) : p_arg       Argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Note(s)     : (1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                   used.  The compiler should not generate any code for this statement.
*
*               (2) Interrupts are enabled once the task starts because the I-bit of the CCR register was
*                   set to 0 by 'OSTaskCreate()'.
*********************************************************************************************************
*/
static void App_TaskStart(void *p_arg)
{
  (void)p_arg; /* Note(1) */
  uint16_t osVersion1, osVersion2, osVersion3;

  /* Use 48MHZ HFXO as core clock frequency */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

  /* Initialize DK board register access */
  /* This demo currently only works in EBI mode */
  BSP_Init(BSP_INIT_DEFAULT);

  /* If first word of user data page is non-zero, enable eA Profiler trace */
  BSP_TraceProfilerSetup();

  /* Setup SysTick Timer for 10 msec interrupts  */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))
  {
    while (1) ;
  }

  /* Initialize the uC/OS-II ticker                       */
  OS_CPU_SysTickInit(CMU_ClockFreqGet(cmuClock_HFPER)/OS_TICKS_PER_SEC);

#if (OS_TASK_STAT_EN > 0)
  /* Determine CPU capacity                               */
  OSStatInit();
#endif

  /* Create application tasks                             */
  App_TaskCreate();

  /* Create application mailboxes                         */
  App_MailboxCreate();

  /* Enable EFM32WG_DK3850 RS232/UART switch */
  BSP_PeripheralAccess(BSP_RS232_UART, true);

  UART1_SerialInit();
  UART1_SerialCrLf(1);

  /* Initialize TFT serial                                 */
  RETARGET_SerialInit();
  RETARGET_SerialCrLf(0);

  osVersion3 = OSVersion();
  osVersion1 = osVersion3 / 10000;
  osVersion3 -= osVersion1 * 10000;
  osVersion2 = osVersion3 / 100;
  osVersion3 -= osVersion2 * 100;
  osVersion3 %= 100;

  printf("\n*************************************");
  printf("\n    uC/OS-II v%d.%02d.%02d on EFM32 DK    ",
         osVersion1, osVersion2, osVersion3 );
  printf("\n           Demo Application   \n");
  printf("\n              uC/OS-II        ");
  printf("\n      \"The real time kernel\" ");
  printf("\n           www.micrium.com      ");
  printf("\n            is running on      ");
  printf("\n          Energy Micro EFM32   ");
  printf("\n         www.energymicro.com   \n");
  printf("\nDescription:");
  printf("\n\nTask1: LED blink task");
  printf("\n\nTask2: Receives characters from serial and posts message to Task3");
  printf("\n\nTask3: Receives message from Task2 and writes it on screen");
  printf("\n\n**************************************\n");
  printf("\nStart typing...\n");

  /* Suspend this task as it is only used once in one Reset cycle */
  OSTaskSuspend(APP_CFG_TASK_START_PRIO);

  while (1)
  {/* endless loop of Start task                          */
    ;
  }
}


/*
*********************************************************************************************************
*                                      App_MailboxCreate()
*
* Description : Create the application Mailboxes
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/
static void App_MailboxCreate (void)
{
  /* Create mailbox object for messaging received serial data between tasks */
  pSerialMsgObj = OSMboxCreate((void *)0);

}


/*
*********************************************************************************************************
*                                      App_TaskCreate()
*
* Description : Create the application tasks.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/
static void App_TaskCreate (void)
{
#if (OS_TASK_NAME_EN > 0)
  CPU_INT08U  err;
#endif

  /* Create the One task                         */
  OSTaskCreateExt((void (*)(void *)) APP_TaskOne,
                  (void           *) 0,
                  (OS_STK         *)&App_TaskOneStk[APP_CFG_TASK_ONE_STK_SIZE - 1],
                  (INT8U           ) APP_CFG_TASK_ONE_PRIO,
                  (INT16U          ) APP_CFG_TASK_ONE_PRIO,
                  (OS_STK         *)&App_TaskOneStk[0],
                  (INT32U          ) APP_CFG_TASK_ONE_STK_SIZE,
                  (void           *) 0,
#ifdef __ICCARM__ /* IAR port */
                  (INT16U          )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR | OS_TASK_OPT_SAVE_FP));
#else
                  (INT16U          )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
#endif

  /* Create the Two task                         */
  OSTaskCreateExt((void (*)(void *)) APP_TaskTwo,
                  (void           *) 0,
                  (OS_STK         *)&App_TaskTwoStk[APP_CFG_TASK_TWO_STK_SIZE - 1],
                  (INT8U           ) APP_CFG_TASK_TWO_PRIO,
                  (INT16U          ) APP_CFG_TASK_TWO_PRIO,
                  (OS_STK         *)&App_TaskTwoStk[0],
                  (INT32U          ) APP_CFG_TASK_TWO_STK_SIZE,
                  (void           *) 0,
#ifdef __ICCARM__ /* IAR port */
                  (INT16U          )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR | OS_TASK_OPT_SAVE_FP));
#else
                  (INT16U          )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
#endif

    /* Create the Three task                     */
  OSTaskCreateExt((void (*)(void *)) APP_TaskThree,
                  (void           *) 0,
                  (OS_STK         *)&App_TaskThreeStk[APP_CFG_TASK_TWO_STK_SIZE - 1],
                  (INT8U           ) APP_CFG_TASK_THREE_PRIO,
                  (INT16U          ) APP_CFG_TASK_THREE_PRIO,
                  (OS_STK         *)&App_TaskThreeStk[0],
                  (INT32U          ) APP_CFG_TASK_THREE_STK_SIZE,
                  (void           *) 0,
#ifdef __ICCARM__ /* IAR port */
                  (INT16U          )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR | OS_TASK_OPT_SAVE_FP));
#else
                  (INT16U          )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
#endif

#if (OS_TASK_NAME_EN > 0)
  OSTaskNameSet(APP_CFG_TASK_ONE_PRIO, "One", &err);
  OSTaskNameSet(APP_CFG_TASK_TWO_PRIO, "Two", &err);
  OSTaskNameSet(APP_CFG_TASK_TWO_PRIO, "Three", &err);
#endif
}


/*
*********************************************************************************************************
*                                              setupSW0()
*
* Description : Setup SW0 for energyAware Profiler.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/
static void setupSWO(void)
{
  uint32_t *dwt_ctrl = (uint32_t *) 0xE0001000;
  uint32_t *tpiu_prescaler = (uint32_t *) 0xE0040010;
  uint32_t *tpiu_protocol = (uint32_t *) 0xE00400F0;

  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;
  /* Enable Serial wire output pin */
  GPIO->ROUTE |= GPIO_ROUTE_SWOPEN;
  /* Set location 1 */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC1;
  /* Enable output on pin */
  GPIO->P[2].MODEH &= ~(_GPIO_P_MODEH_MODE15_MASK);
  GPIO->P[2].MODEH |= GPIO_P_MODEH_MODE15_PUSHPULL;
  /* Enable debug clock AUXHFRCO */
  CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;

  while(!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY));

  /* Enable trace in core debug */
  CoreDebug->DHCSR |= 1;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* Enable PC and IRQ sampling output */
  *dwt_ctrl = 0x400113FF;
  /* Set TPIU prescaler to 16. */
  *tpiu_prescaler = 0xf;
  /* Set protocol to NRZ */
  *tpiu_protocol = 2;
  /* Unlock ITM and output data */
  ITM->LAR = 0xC5ACCE55;
  ITM->TCR = 0x10009;
}
