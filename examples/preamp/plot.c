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
#include <stdio.h>
#include "plot.h"
#include "em_device.h"
#include "em_common.h"
#include "arm_math.h"
#include "logo.h"
#include "preamp.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

#ifdef USE_GUI
#define GUI_VOL_XPOS              178
#define GUI_VOL_YPOS              60
#define GUI_BAL_XPOS              178
#define GUI_BAL_YPOS              80
#define GUI_BARL_LOWERRIGHT_XPOS  280
#define GUI_BARR_LOWERRIGHT_XPOS  300
#define GUI_BAR_YPOS              160
#define GUI_BAR_WIDTH             10
#define GUI_BAR_MAX_HEIGHT        100
#define GUI_BAR_RANGE_dB          33.0f

#define GUI_BASS_XPOS             170
#define GUI_BASS_YPOS             130
#define GUI_TREBLE_XPOS           170
#define GUI_TREBLE_YPOS           150
#define GUI_BASS_LOWERRIGHT_XPOS   30
#define GUI_TREBLE_LOWERRIGHT_XPOS 50
#endif

/***************************************************************************//**
 * @brief     Draw a vertical bar without solid fill.
 * @param[in] height          Bar watermark level.
 * @param[in] lowerRightXpos  Lower right x position of bar.
 * @param[in] lowerYpos       Lower y position of bar.
 * @param[in] width           Width of bar.
 * @param[in] maxHeight       Height of bar.
 ******************************************************************************/
void PLOT_Bar( int height,
               int lowerRightXpos, int lowerYpos, int width, int maxHeight )
{
  GUI_ClearRect( lowerRightXpos - width,      /* Upper left X  */
                 lowerYpos - maxHeight,       /* Upper left Y  */
                 lowerRightXpos,              /* Lower right X */
                 lowerYpos );                 /* Lower right Y */
  GUI_SetColor(  GUI_GRAY );
  GUI_DrawRect(  lowerRightXpos - width,      /* Upper left X  */
                 lowerYpos - maxHeight,       /* Upper left Y  */
                 lowerRightXpos,              /* Lower right X */
                 lowerYpos );                 /* Lower right Y */

  GUI_FillRect(  lowerRightXpos - width,      /* Upper left X  */
                 lowerYpos - height - 2,      /* Upper left Y  */
                 lowerRightXpos,              /* Lower right X */
                 lowerYpos - height + 2 );    /* Upper left Y  */
}

/***************************************************************************//**
 * @brief     Draw a vertical bar with solid fill.
 * @param[in] height          Bar fill level.
 * @param[in] lowerRightXpos  Lower right x position of bar.
 * @param[in] lowerYpos       Lower y position of bar.
 * @param[in] width           Width of bar.
 * @param[in] maxHeight       Height of bar.
 ******************************************************************************/
void PLOT_BarSolid( int height,
                    int lowerRightXpos, int lowerYpos,
                    int width, int maxHeight )
{
  GUI_ClearRect( lowerRightXpos - width,      /* Upper left X  */
                 lowerYpos - maxHeight,       /* Upper left Y  */
                 lowerRightXpos,              /* Lower right X */
                 lowerYpos );                 /* Lower right Y */
  GUI_SetColor(  GUI_GRAY );
  GUI_DrawRect(  lowerRightXpos - width,      /* Upper left X  */
                 lowerYpos - maxHeight,       /* Upper left Y  */
                 lowerRightXpos,              /* Lower right X */
                 lowerYpos );                 /* Lower right Y */
  GUI_FillRect(  lowerRightXpos - width,      /* Upper left X  */
                 lowerYpos - height,          /* Upper left Y  */
                 lowerRightXpos,              /* Lower right X */
                 lowerYpos );                 /* Lower right Y */
}

/**************************************************************************//**
 * @brief Draw initial text/graphics on TFT screen.
 *****************************************************************************/
void PLOT_DisplayInit( void )
{
#ifdef USE_GUI
  /* Initialize emWin Library. */
  GUI_Init();
  GUI_Clear();

  /* Initialize PLOT module. */
  PLOT_Init();

  PLOT_PutsCentered( "Wonder Gecko Preamplifier", 160, 5 );
  PLOT_Puts( "Volume  :"         , GUI_VOL_XPOS - 88   , GUI_VOL_YPOS );
  PLOT_Puts( "Balance :"         , GUI_BAL_XPOS - 88   , GUI_BAL_YPOS );
  PLOT_Puts( "Bass    :       dB", GUI_BASS_XPOS - 80  , GUI_BASS_YPOS );
  PLOT_Puts( "Treble  :       dB", GUI_TREBLE_XPOS - 80, GUI_TREBLE_YPOS );

  PLOT_DisplayUpdate();
#endif
}

/**************************************************************************//**
 * @brief Do a complete TFT display update.
 *****************************************************************************/
void PLOT_DisplayUpdate( void )
{
#ifdef USE_GUI
  float f;
  char bufL[10], bufR[10];

  /* Volume (text). */
  snprintf(bufL, 3, "%2d", volume);
  PLOT_Puts( bufL, GUI_VOL_XPOS, GUI_VOL_YPOS );

  /* Balance (text). */
  if ( balance == BALANCE_CENTER )
    snprintf(bufL, 3, "%2d", balance - BALANCE_CENTER );
  else
    snprintf(bufL, 3, "%+2d", balance - BALANCE_CENTER );
  PLOT_Puts( bufL, GUI_BAL_XPOS, GUI_BAL_YPOS );

  /* Bass (text). */
  f = (float)( (bass - TONE_CENTER) * 2 );
  if ( ( f < 0.5f ) && ( f > -0.5f ) )
    snprintf(bufL, 6, "%5.1f", f );
  else
    snprintf(bufL, 6, "%+5.1f", f );
  PLOT_Puts( bufL, GUI_BASS_XPOS, GUI_BASS_YPOS );

  /* Treble (text). */
  f = (float)( (treble - TONE_CENTER) * 2 );
  if ( ( f < 0.5f ) && ( f > -0.5f ) )
    snprintf(bufL, 6, "%5.1f", f );
  else
    snprintf(bufL, 6, "%+5.1f", f );
  PLOT_Puts( bufL, GUI_TREBLE_XPOS, GUI_TREBLE_YPOS );

  /* Volume & Balance (graphical). */
  if ( volume == 0 )
  {
    sprintf(bufL, " --- " );
    sprintf(bufR, " --- " );
    PLOT_VolumeBar( -100.0f, GUI_BARL_LOWERRIGHT_XPOS );
    PLOT_VolumeBar( -100.0f, GUI_BARR_LOWERRIGHT_XPOS );
  }
  else
  {
    f = volumeTable_dB[ volume ];
    f += balanceTable_dB[ BALANCE_MAX - balance ];

    if ( ( f < 0.5f ) && ( f > -0.5f ) )
      snprintf(bufL, 6, "%5.1f", f );
    else
      snprintf(bufL, 6, "%+5.1f", f );

    PLOT_VolumeBar( f, GUI_BARL_LOWERRIGHT_XPOS );

    f = volumeTable_dB[ volume ];
    f += balanceTable_dB[ balance ];

    if ( ( f < 0.5f ) && ( f > -0.5f ) )
      snprintf(bufR, 6, "%5.1f", f );
    else
      snprintf(bufR, 6, "%+5.1f", f );

    PLOT_VolumeBar( f, GUI_BARR_LOWERRIGHT_XPOS );
  }

  /* Bass & treble (graphical). */
  PLOT_ToneBar( bass, GUI_BASS_LOWERRIGHT_XPOS );
  PLOT_ToneBar( treble, GUI_TREBLE_LOWERRIGHT_XPOS );
#endif
}


/***************************************************************************//**
 * @brief Initialize PLOT module.
 ******************************************************************************/
void PLOT_Init( void )
{
  GUI_SetBkColor(GUI_BLACK);
  GUI_Clear();
  GUI_DrawBitmap(&bmlogo, (LCD_GetXSize() - bmlogo.XSize)/2, 188);
}

/***************************************************************************//**
 * @brief Write a string at specified position.
 * @param[in] str String.
 * @param[in] xpos X position.
 * @param[in] ypos Y position.
 ******************************************************************************/
void PLOT_Puts(const char *str, int xpos, int ypos)
{
  GUI_SetTextMode(GUI_TM_NORMAL);
  GUI_SetColor(GUI_WHITE);
  GUI_SetFont(&GUI_Font8x13_1);
  GUI_DispStringAt(str, xpos, ypos);
}

/***************************************************************************//**
 * @brief Write a string centered at specified position.
 * @param[in] str String.
 * @param[in] xpos X position.
 * @param[in] ypos Y position.
 ******************************************************************************/
void PLOT_PutsCentered(const char *str, int xpos, int ypos)
{
  GUI_SetTextMode(GUI_TM_NORMAL);
  GUI_SetColor(GUI_WHITE);
  GUI_SetFont(&GUI_Font8x13_1);
  GUI_DispStringHCenterAt(str, xpos, ypos);
}

#ifdef USE_GUI
/**************************************************************************//**
 * @brief Draw a tone control bar on TFT display.
 *****************************************************************************/
void PLOT_ToneBar( int level, int lowerRightXpos )
{
  int i;
  float f;

  f = (( level * (GUI_BAR_MAX_HEIGHT - 5) ) / TONE_MAX ) + 2;
  i = (int)f;

  PLOT_Bar( i, lowerRightXpos, GUI_BAR_YPOS, GUI_BAR_WIDTH, GUI_BAR_MAX_HEIGHT );
}
#endif

#ifdef USE_GUI
/**************************************************************************//**
 * @brief Draw a output volume bar on TFT display.
 *****************************************************************************/
void PLOT_VolumeBar( float level, int lowerRightXpos )
{
  int i;
  float f;

  f = fabsf( level );
  f = EFM32_MIN( f, GUI_BAR_RANGE_dB );
  f = GUI_BAR_MAX_HEIGHT - (( f * GUI_BAR_MAX_HEIGHT ) / GUI_BAR_RANGE_dB );
  i = (int)f;

  PLOT_BarSolid( i, lowerRightXpos,
                 GUI_BAR_YPOS, GUI_BAR_WIDTH, GUI_BAR_MAX_HEIGHT );
}
#endif

