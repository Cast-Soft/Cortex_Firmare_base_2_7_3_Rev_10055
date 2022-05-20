/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
* File Name          : blkt_gfx.h
* Author             : Matthew Mak			
* Version            : V1.0
* Date               : 4/18/2019
* Description        : This header file is used to interface with the INA219
					   exclusively.
*******************************************************************************/

/* DEFINE TO PREVENT RECURSIVE INCLUSION -------------------------------------*/
#ifndef __BLKT_GFX_H
#define __BLKT_GFX_H

/* INCLUDES ------------------------------------------------------------------*/
#include <stdint.h>
/* EXPORTED TYPES ------------------------------------------------------------*/
typedef enum{
	black 	= 0,
	gray_d 	= 1,
	gray_l 	= 2,
	white 	= 3
}EPD_grayscale;

typedef enum{
	smallfont = 0,
	largefont = 1
}EPD_textsize;

typedef enum{
	leftalign =0,
	centeralign = 1,
	//rightalign = 2	//don't have this supported
}GFX_TextAlignment;

typedef struct {
	const uint8_t	*bitmap;    // Glyph bitmaps, concatenated
	uint8_t		first;      // ASCII extents (first char)
    uint8_t		last;       // ASCII extents (last char)
	uint8_t		yAdvance;   // Newline distance (y axis)
	uint16_t		bitmapOffset;
	uint8_t		width;
	uint8_t		height;
	uint16_t	numofpixels;// total dimension of pixels
} GFXfont;

/* EXPORTED CONSTANTS --------------------------------------------------------*/

/* EXPORTED MACROS -----------------------------------------------------------*/

/* EXPORTED DEFINES ----------------------------------------------------------*/

/* EXPORTED FUNCTIONS --------------------------------------------------------*/
void GFX_DrawBitMap8Bit(uint8_t x, uint8_t y, const uint8_t bitmap[], uint8_t w, uint8_t h);
void GFX_DrawBitMap(uint8_t x, uint8_t y, const uint8_t bitmap[], uint8_t w, uint8_t h);
void GFX_FillBuffer(EPD_grayscale color);
EPD_grayscale getPixel8bit(const uint8_t x);

void setFont(EPD_textsize size);
void setCursor(uint8_t x, uint8_t y);
void GFX_Print(uint8_t * str, uint8_t size, GFX_TextAlignment alignment, EPD_grayscale bg, EPD_grayscale fg);
void GFX_drawChar(uint8_t x, uint8_t y, unsigned char c, EPD_grayscale bg, EPD_grayscale fg);

void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, EPD_grayscale color);
void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, EPD_grayscale color);
void drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, EPD_grayscale color);
void fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, EPD_grayscale color);
void drawCircle(int16_t x0, int16_t y0, int16_t r, EPD_grayscale color);
void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
/* EXTERNAL VARIABLES --------------------------------------------------------*/


#endif /*__BLKT_GFX_H*/
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
