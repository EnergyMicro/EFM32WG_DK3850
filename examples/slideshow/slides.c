/**************************************************************************//**
 * @file
 * @brief Graphics routines for reading a single BMP image from the filesystem
 *        and displaying it on the TFT.
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

#include "glib/glib.h"
#include "glib/glib_color.h"
#include "glib/glib_font.h"
#include "glib/bmp.h"
#include "dmd/ssd2119/dmd_ssd2119.h"

#include "slides.h"

#include "diskio.h"
#include "ff.h"
#include "microsd.h"

/** Graphics context */
GLIB_Context gc;

/* Palette */
uint8_t palette[PALETTE_SIZE];
uint8_t rgbBuffer[RGB_BUFFER_SIZE];

/* File to read bmp data from */
FIL BMPfile;

/* Local prototypes */
EMSTATUS SLIDES_readData(uint8_t buffer[], uint32_t bufLength, uint32_t bytesToRead);

/***************************************************************************//**
 * @brief
 *   This function draws an error message and if it is fatal, loops forever.
 *   This function assumes that TFT_init has been run prior to calling this
 *   function.
 * @param fatal
 *   If true, the display will show the error message and loop forever.
 * @param fmt
 *   Format string to display.
 ******************************************************************************/
void SLIDES_showError(bool fatal, const char* fmt, ...)
{
  va_list argp;
  char    buffer[100];

  va_start(argp, fmt);
  vsnprintf(buffer, 100, fmt, argp);
  va_end(argp);

  /* Clear screen */
  GLIB_clear(&gc);
  /* Draw error string */
  GLIB_drawString(&gc, buffer, sizeof(buffer), 10, 50, 1);

  /* If it is fatal, loop forever here. */
  if (fatal)
    while (1) ;
}

/***************************************************************************//**
 * @brief
 *   Callback used by the BMP decompression library for reading data from
 *   the filesystem. Uses the global variable BMPfile to read from.
 * @param[out] buffer
 *   The buffer to write data to.
 * @param[in] bufLength
 *   Size of the buffer.
 * @param[in] bytesToRead
 *   Number of bytes to read from file.
 * @return
 *   An EMSTATUS error code.
 ******************************************************************************/
EMSTATUS SLIDES_readData(uint8_t buffer[], uint32_t bufLength, uint32_t bytesToRead)
{
  UINT bytes_read;
  (void) bufLength;      /* Unused parameter */

  if (f_read(&BMPfile, buffer, bytesToRead, &bytes_read) != FR_OK)
    return BMP_ERROR_IO;
  return BMP_OK;
}

/**************************************************************************//**
 * @brief Clears/updates entire background ready to be drawn
 *****************************************************************************/
void SLIDES_showBMP(char *fileName)
{
  int32_t  xCursor;
  int32_t  yCursor;
  uint32_t pixelsRead;
  uint32_t nPixelsPerRow;
  uint32_t nRows;

  EMSTATUS status;

  /* Open file */
  if (f_open(&BMPfile, fileName, FA_READ) != FR_OK)
  {
    SLIDES_showError(true, "Fatal:\n  Failed to open file:\n  %s", fileName);
  }

  /* Initialize BMP decoder */
  if (BMP_init(palette, 1024, &SLIDES_readData) != BMP_OK)
  {
    SLIDES_showError(true, "Fatal:\n  Failed to init BMP library.");
  }

  /* Read headers */
  if ((status = BMP_reset()) != BMP_OK)
  {
    SLIDES_showError(false, "Info:\n  %s is not a BMP file", fileName);
    goto cleanup;
  }

  /* Get important BMP data */
  nPixelsPerRow = BMP_getWidth();
  nRows         = BMP_getHeight();
  yCursor       = nRows - 1;
  xCursor       = 0;

  /* Check size of BMP */
  if ((nPixelsPerRow > 320) || (nRows > 240))
  {
    SLIDES_showError(false, "Info:\n  %s is larger than 320x240.", fileName);
  }

  /* Set clipping region */
  DMD_setClippingArea(0, 0, nPixelsPerRow, nRows);

  /* Read in and draw row for row */
  while (yCursor >= 0)
  {
    /* Read in row buffer */
    status = BMP_readRgbData(rgbBuffer, RGB_BUFFER_SIZE, &pixelsRead);
    if (status != BMP_OK || pixelsRead == 0)
      break;

    /* Draw row buffer. Remember, BMP is stored bottom-up */
    status = DMD_writeData(xCursor, yCursor, rgbBuffer, pixelsRead);
    if (status != DMD_OK)
      break;

    /* Update cursor */
    xCursor += pixelsRead;
    if ((uint32_t) xCursor >= nPixelsPerRow)
    {
      yCursor -= xCursor / nPixelsPerRow;
      xCursor  = xCursor % nPixelsPerRow;
    }
  }
  /* Reset clipping area in DMD driver */
  status = GLIB_resetDisplayClippingArea(&gc);
  if (status != 0)
    return;

 cleanup:
  /* Close the file */
  f_close(&BMPfile);
}

/**************************************************************************//**
 * @brief Initialize viewer
 *****************************************************************************/
void SLIDES_init(void)
{
  EMSTATUS status;
  GLIB_Rectangle rect = {
    .xMin =   0,
    .yMin =   0,
    .xMax = 319,
    .yMax = 239,
  };

  /* Init graphics context - abort on failure */
  status = GLIB_contextInit(&gc);
  if (status != GLIB_OK) while (1) ;

  /* Clear framebuffer */
  gc.foregroundColor = GLIB_rgbColor(20, 40, 20);
  GLIB_drawRectFilled(&gc, &rect);

  /* Update drawing regions of picture  */
  gc.foregroundColor = GLIB_rgbColor(200, 200, 200);
}
