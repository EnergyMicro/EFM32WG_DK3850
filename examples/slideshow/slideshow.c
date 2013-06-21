/**************************************************************************//**
 * @file
 * @brief FAT example using FatFS for access to the MicroSD card on the DK.
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
#include <string.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_cmu.h"
#include "bsp.h"
#include "bsp_trace.h"
#include "ff.h"
#include "microsd.h"
#include "diskio.h"
#include "tftamapped.h"
#include "slides.h"

/* counts 1ms timeTicks */
volatile uint32_t msTicks;

/* File system */
FATFS Fatfs;
char path[ 100 ];

/* Manifest file */
FIL manifest;

/* Local prototypes */
int initFatFS(void);
DWORD get_fattime(void);
void Delay(uint32_t dlyTicks);

/***************************************************************************//**
 * @brief
 *   Initialize MicroSD driver.
 * @return
 *   Returns 0 if initialization succeded, non-zero otherwise.
 ******************************************************************************/
int initFatFS(void)
{
  MICROSD_Init();
  if (f_mount(0, &Fatfs) != FR_OK)
    return -1;
  return 0;
}

/***************************************************************************//**
 * @brief
 *   This function is required by the FAT file system in order to provide
 *   timestamps for created files. Since we do not have a reliable clock we
 *   hardcode a value here.
 *
 *   Refer to drivers/fatfs/doc/en/fattime.html for the format of this DWORD.
 * @return
 *    A DWORD containing the current time and date as a packed datastructure.
 ******************************************************************************/
DWORD get_fattime(void)
{
  return (28 << 25) | (2 << 21) | (1 << 16);
}

/**************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter.
 *****************************************************************************/
void SysTick_Handler(void)
{
  /* Increment counter necessary in Delay()*/
  msTicks++;
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
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  bool     redraw, firstRun;
  FRESULT  res;
  int      mountStatus;
  /* Used when iterating through directory */
  FILINFO Finfo;
  DIR     dir;
  FRESULT listDirStatus = FR_OK;
  /* Used for manifest operation */
  bool useManifest = false;
  char buffer[20];

  /* Use 48MHZ HFXO as core clock frequency */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

  /* Initialize DK board register access */
  BSP_Init(BSP_INIT_DEFAULT);

  /* If first word of user data page is non-zero, enable eA Profiler trace */
  BSP_TraceProfilerSetup();

  /* Setup SysTick Timer for 10 msec interrupts  */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 100))
  {
    while (1) ;
  }

  /* Enable SPI access to MicroSD card */
  BSP_PeripheralAccess(BSP_MICROSD, true);

  /* Initialize filesystem */
  mountStatus = initFatFS();

  /* Open Manifest file. If a manifest file is not present, iterate through the
   * File system in filesystem order. */
  if (f_open(&manifest, "files.txt", FA_READ) == FR_OK)
    useManifest = true;

  firstRun = true;
  /* Update TFT display forever */
  while (1)
  {
    if (!useManifest)
    {
      /* Open root directory */
      strcpy(path, "");
      listDirStatus = f_opendir(&dir, path);
    }
    /* Iterate through files */
    while (1)
    {
      /* Check if we should control TFT display instead of
       * AEM/board control application. Read state of AEM pushbutton */
      redraw = TFT_AddressMappedInit();
      if (redraw)
      {
        if ( firstRun )
        {
          firstRun = false;
          SLIDES_init();
        }

        /* Check disk status */
        if (disk_status(0) != 0)
        {
          /* Filesystem not mounted, show fatal error. */
          SLIDES_showError(true, "Fatal:\n  Filesystem is not ready.\n  (%d)", disk_status(0));
        }

        /* Check if filesystem was successfully mounted. */
        if (mountStatus != 0)
        {
          /* Filesystem not mounted, show fatal error. */
          SLIDES_showError(true, "Fatal:\n  Filesystem could not be mounted.\n  (%d)", mountStatus);
        }

        /* Is there a manifest file present? */
        if (useManifest)
        {
          /* If we are at the end of the file, reset filepointer */
          if (f_eof(&manifest))
            f_lseek(&manifest, 0);
          /* Read next file from manifest file */
          f_gets(buffer, 20, &manifest);
          /* Display Bitmap */
          SLIDES_showBMP(buffer);
        }
        else
        {
          /* Check to see if the root directory was correctly opened. */
          if (listDirStatus != FR_OK)
          {
            SLIDES_showError(true, "Fatal:\n  Could not read root directory.\n  (%d)", listDirStatus);
          }

          /* Open the next entry in directory */
          res = f_readdir(&dir, &Finfo);
          if ((res != FR_OK) || !Finfo.fname[0])
          {
            /* End of directory listing. Break out of inner loop and reopen
             * directory entry */
            break;
          }
          /* Update display */
          SLIDES_showBMP(Finfo.fname);
        }

        /* Delay to allow the user to see the BMP file. */
        Delay(200);
      }
    }
  }
}
