/**************************************************************************//**
 * @file   plot.c
 * @brief  Simple wrapper for some emWin functions.
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
 *****************************************************************************/
#include "plot.h"
#include "em_device.h"
#include "em_common.h"
#include "arm_math.h"
#include "logo.h"
#include "GUIResource.h"

/* The size of the plotting area. */
#define GRAPH_WIDTH  265
#define GRAPH_HEIGHT 122
#define MAX_POINTS   256

static int      xPosGraph, yPosGraph, numPoints;
static int16_t  pData[MAX_POINTS];

/***************************************************************************//**
 * @brief Draw Y axis labels just left of the plotting area.
 * @param[in] xPos X position of plotting area.
 * @param[in] yPos Y position of plotting area.
 ******************************************************************************/
static void DrawLabel(int xPos, int yPos)
{
  int FontSizeY, i, x0, y0;

  GUI_SetFont(&GUI_FontD9_AA4);
  GUI_SetTextMode(GUI_TM_TRANS);
  FontSizeY = GUI_GetFontSizeY();
  for (i = 0; i < 5; i++) {
    x0 = xPos - 7;
    y0 = yPos + 100 - i * 20 - 7;
    GUI_GotoXY(x0 + 7, y0 + 7 - FontSizeY / 2);
    GUI_SetTextAlign(GUI_TA_HCENTER);
    GUI_SetColor(GUI_WHITE);
    GUI_DispDecMin((i + 1) * 2);
  }
}

/***************************************************************************//**
 * @brief Draw a graph inside the plotting area.
 * @param[in] xPos X position of plotting area.
 * @param[in] yPos Y position of plotting area.
 * @param[in] py Graph data array.
 ******************************************************************************/
static void DrawGraphAt(int xPos, int yPos, float* py) {
  int i;
  float f;
  int16_t val;

  /* Draw dark grey background. */
  GUI_SetColor(GUI_DARKGRAY);
  GUI_FillRoundedRect(xPos, yPos, xPos + GRAPH_WIDTH, yPos + GRAPH_HEIGHT, 4);

  /* Draw gray grid lines. */
  GUI_SetColor(GUI_GRAY);
  for (i = 0; i < 12; i++)
  {
    GUI_DrawHLine(yPos + 10 + i * 10, xPos + 2, xPos + GRAPH_WIDTH - 2 );
  }

  for (i = 0; i < numPoints; i++)
  {
    /* Scale plot data. */
    f = py[i] / 175;

    val = (int16_t)f;

    /* Clip large values. */
    pData[i] = GRAPH_HEIGHT - EFM32_MIN( val, GRAPH_HEIGHT - 5 );
  }

  /* Draw the actual graph. */
  GUI_SetColor(GUI_WHITE);
  GUI_DrawGraph( pData, numPoints, xPos+3, yPos-2);
}

/***************************************************************************//**
 * @brief Draw a graph.
 * @param[in] plotData Graph data array.
 ******************************************************************************/
void PLOT_Plot( float *plotData )
{
  DrawGraphAt(xPosGraph, yPosGraph, plotData );
}

/***************************************************************************//**
 * @brief Initialize PLOT module.
 * @param[in] points Number of graph data points along X axis.
 ******************************************************************************/
void PLOT_Init( int points )
{
  int i;

  numPoints = EFM32_MIN( points, MAX_POINTS );

  /* Calculate positions. */
  xPosGraph = (LCD_GetXSize() - GRAPH_WIDTH)  / 2;
  yPosGraph = (LCD_GetYSize() - GRAPH_HEIGHT) / 2;

  for (i=0; i<GUI_MULTIBUF_GetNumBuffers(); i++)
  {
    /* Make sure that all buffers are initialized. */
    GUI_MULTIBUF_Begin();

    GUI_SetBkColor(GUI_BLACK);
    GUI_Clear();
    GUI_DrawBitmap(&bmlogo, (LCD_GetXSize() - bmlogo.XSize)/2, 188);

    /* Y axis label. */
    DrawLabel(xPosGraph-10, yPosGraph);

    GUI_MULTIBUF_End();         /* Drawing finished. Buffers will be flipped. */
  }
}

/***************************************************************************//**
 * @brief Write a string centered at specified position.
 * @param[in] str String.
 * @param[in] xpos X position.
 * @param[in] ypos Y position.
 ******************************************************************************/
void PLOT_Puts(const char *str, int xpos, int ypos)
{
  GUI_SetTextMode(GUI_TM_NORMAL);
  GUI_SetColor(GUI_WHITE);
  GUI_SetFont(&GUI_Font8x13_1);
  GUI_DispStringHCenterAt(str, xpos, ypos);
}
