/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
* File Name          : blkt_gfx.c
* Author             : ?
* Version            : V1.0
* Date               : 6/2/2011
* Description        : Custom graphics library based off adafruit gfx library
					   Graphics library designed specifically for PlasticLogic 2.1"
					   DrawPixel functionality to only work when Reg0x0F DEM bits assigned to 0x100b
*******************************************************************************/

/* INCLUDES ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "blkt_gfx.h"
#include "eink.h"

/* PRIVATE TYPEDEF -----------------------------------------------------------*/
typedef struct{
	const GFXfont* fontselect;
	uint8_t x_pos;
	uint8_t y_pos;
}GUI_Controller;
/* PRIVATE DEFINES -----------------------------------------------------------*/

/* PRIVATE MACROS ------------------------------------------------------------*/
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }

/* EXTERN VARIABLES ----------------------------------------------------------*/
extern const GFXfont Font9pt7b;
extern const GFXfont Font16pt12b;
/* PRIVATE VARIABLES ---------------------------------------------------------*/
//GUI_Controller GFX_TextDriver = {&Font9pt7b,0,0};
GUI_Controller GFX_TextDriver = {&Font16pt12b,0,0};

/* PUBLIC VARIABLES ----------------------------------------------------------*/

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/
static void GFX_drawPixel(int16_t x, int16_t y, EPD_grayscale color);
static EPD_grayscale getPixel(const uint8_t x, uint8_t pos);
static void drawFastVLine(int16_t x, int16_t y, int16_t h, EPD_grayscale color);
static void drawFastHLine(int16_t x, int16_t y, int16_t w, EPD_grayscale color);
static void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, EPD_grayscale color);
static void fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, EPD_grayscale color);
static void drawCircleHelper( int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color);
/* PRIVATE FUNCTIONS ---------------------------------------------------------*/

/*******************************************************************************
* Description : Inserts 2-bit grayscale into appropriate byte position of the frame buffer
* Input       : uint8_t x: targeted x row of internal framebuffer
				uint8_t y: targeted y column of internal framebuffer
* Return      :
*******************************************************************************/
static void GFX_drawPixel(int16_t x, int16_t y, EPD_grayscale color){
	uint16_t val;
	uint16_t mod;

	if ((x<0)||(y<0)||(x >= EPD_WIDTH) || (y >= EPD_HEIGHT) || (color>white )) return;
	y = (y<120)? 119-y : y;

	val = (x+(EPD_WIDTH*y));
	mod = val%4;
	val = val/4;
    uint8_t pixels = spiInk_ImgBuf.buffer[val];

	switch (mod) {					            //2-bit grayscale dot
	   	case 0: spiInk_ImgBuf.buffer[val] = (pixels & 0x3F) | ((uint8_t)color << 6); break;
	   	case 1: spiInk_ImgBuf.buffer[val] = (pixels & 0xCF) | ((uint8_t)color << 4); break;
	   	case 2: spiInk_ImgBuf.buffer[val] = (pixels & 0xF3) | ((uint8_t)color << 2); break;
	   	case 3: spiInk_ImgBuf.buffer[val] = (pixels & 0xFC) | (uint8_t)color; break;
	}
}

/*******************************************************************************
* Description : Draws the selected bitmap at the specified (x,y) coordinate
* Input       : uint8_t x: top left x coordinate
				uint8_t y: top left y coordinate
				const uint8_t bitmap[]: selected bitmap
				uint8_t w: width of bitmap
				uint8_t h: height of bitmap
* Return      :
*******************************************************************************/
void GFX_DrawBitMap8Bit(uint8_t x, uint8_t y, const uint8_t bitmap[], uint8_t w, uint8_t h){
	uint8_t i,j;
	for(j=0; j<h; j++, y++){
		for(i=0; i<w; i++){
			GFX_drawPixel(x+i,y,getPixel8bit(*(const uint8_t*) &bitmap[j*w+i]));
		}
	}
}

/*******************************************************************************
* Description : Retrieves selected pixel data from bitmap in flash
* Input       : uint8_t x: top left x coordinate
* Return      :
*******************************************************************************/
EPD_grayscale getPixel8bit(const uint8_t x) {
    if(x<0x40) return black;
    else if(x>=0x40 && x<0x53) return gray_l;
    else if(x>=0x53 && x<0xC0) return gray_d;
	else return white;
}

/*******************************************************************************
* Description : Retrieves selected pixel data from bitmap in flash
* Input       : uint8_t x: top left x coordinate
* Return      :
*******************************************************************************/
void GFX_DrawBitMap(uint8_t x, uint8_t y, const uint8_t bitmap[], uint8_t w, uint8_t h){
	uint8_t i=0;
	uint8_t bit=0;
	uint16_t pos=0;
	uint16_t pixelcnt = w*h;

	while(pixelcnt--){
		GFX_drawPixel(x+i,y,getPixel(*(const uint8_t*) &bitmap[pos],bit));
		if(bit++==3){bit=0; pos++;}
		if(++i == w){i=0; y++;}
	}
}

/*******************************************************************************
* Description : Retrieves selected pixel data from bitmap in flash
* Input       : uint8_t x: top left x coordinate
* Return      :
*******************************************************************************/
static EPD_grayscale getPixel(const uint8_t x, uint8_t pos){
	switch(pos){
	case 0: return (x &0x03);
	case 1: return ((x>>2) & 0x03);
	case 2: return ((x>>4) & 0x03);
	case 3:
	default:
	return ((x>>6) & 0x03);
	}
}

/**************************************************************************/
/*!
    @brief  Quarter-circle drawer with fill, used for circles and roundrects
    @param  x0       Center-point x coordinate
    @param  y0       Center-point y coordinate
    @param  r        Radius of circle
    @param  corners  Mask bits indicating which quarters we're doing
    @param  delta    Offset from center-point, used for round-rects
    @param  color    16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
static void fillCircleHelper(int16_t x0, int16_t y0, int16_t r,
  uint8_t corners, int16_t delta, EPD_grayscale color) {

    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;
    int16_t px    = x;
    int16_t py    = y;

    delta++; // Avoid some +1's in the loop

    while(x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
        // These checks avoid double-drawing certain lines, important
        // for the SSD1306 library which has an INVERT drawing mode.
        if(x < (y + 1)) {
            if(corners & 1) drawFastVLine(x0+x, y0-y, 2*y+delta, color);
            if(corners & 2) drawFastVLine(x0-x, y0-y, 2*y+delta, color);
        }
        if(y != py) {
            if(corners & 1) drawFastVLine(x0+py, y0-px, 2*px+delta, color);
            if(corners & 2) drawFastVLine(x0-py, y0-px, 2*px+delta, color);
            py = y;
        }
        px = x;
    }
}

/**************************************************************************/
/*!
    @brief    Quarter-circle drawer, used to do circles and roundrects
    @param    x0   Center-point x coordinate
    @param    y0   Center-point y coordinate
    @param    r   Radius of circle
    @param    cornername  Mask bit #1 or bit #2 to indicate which quarters of the circle we're doing
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
static void drawCircleHelper( int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color) {
    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;

    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
        if (cornername & 0x4) {
            GFX_drawPixel(x0 + x, y0 + y, color);
            GFX_drawPixel(x0 + y, y0 + x, color);
        }
        if (cornername & 0x2) {
            GFX_drawPixel(x0 + x, y0 - y, color);
            GFX_drawPixel(x0 + y, y0 - x, color);
        }
        if (cornername & 0x8) {
            GFX_drawPixel(x0 - y, y0 + x, color);
            GFX_drawPixel(x0 - x, y0 + y, color);
        }
        if (cornername & 0x1) {
            GFX_drawPixel(x0 - y, y0 - x, color);
            GFX_drawPixel(x0 - x, y0 - y, color);
        }
    }
}


/*******************************************************************************
* Description : Draws a vertical line
* Input       : uint8_t x: top left x coordinate
* 				uint8_t y: top left y coordinate
* 				uint8_t h: height in pixels
* 				EPD_grayscale color: four level grayscale color
* Return      :
*******************************************************************************/
static void drawFastVLine(int16_t x, int16_t y, int16_t h, EPD_grayscale color) {
    writeLine(x, y, x, y+h-1, color);
}

/*******************************************************************************
* Description : Draws a horizontal line
* Input       : uint8_t x: top left x coordinate
* 				uint8_t y: top left y coordinate
* 				uint8_t w: width in pixels
* 				EPD_grayscale color: four level grayscale color
* Return      :
*******************************************************************************/
static void drawFastHLine(int16_t x, int16_t y, int16_t w, EPD_grayscale color){
    writeLine(x, y, x+w-1, y, color);
}

/*******************************************************************************
* Description : Draws a line according to Bresenham's algorithm
* Input       : int16_t x0: top left x coordinate
* 				int16_t y0: top left y coordinate
* 				int16_t x1: end point x coordinate
* 				int16_t y1: end point y coordinate
* 				EPD_grayscale color: four level grayscale color
* Return      :
*******************************************************************************/
static void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, EPD_grayscale color) {
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        _swap_int16_t(x0, y0);
        _swap_int16_t(x1, y1);
    }
    if (x0 > x1) {
        _swap_int16_t(x0, x1);
        _swap_int16_t(y0, y1);
    }
    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    for (; x0<=x1; x0++) {
        if (steep) {
        	GFX_drawPixel(y0, x0, color);
        } else {
        	GFX_drawPixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}

/*******************************************************************************
* Description : Fills entire memory buffer with a selected color
* Input       :
* Return      :
*******************************************************************************/
void GFX_FillBuffer(EPD_grayscale color) {
    memset((uint8_t*)&spiInk_ImgBuf.buffer[0], color, EPD_BUFFERSIZE);
}


void setCursor(uint8_t x, uint8_t y){
	if(x>=EPD_WIDTH || y>=EPD_HEIGHT) return;
	GFX_TextDriver.x_pos = x; GFX_TextDriver.y_pos = y;
}

void setFont(EPD_textsize size){
	GFX_TextDriver.fontselect = (size==smallfont)? &Font9pt7b : &Font16pt12b;
}

/*******************************************************************************
* Description : draws string characters into the memory buffer
* Input       : 
* Return      :
*******************************************************************************/
void GFX_Print(uint8_t * str, uint8_t size, GFX_TextAlignment alignment, EPD_grayscale bg, EPD_grayscale fg){
	int16_t w,h;
	uint8_t c;
	h= *(const uint8_t*) &GFX_TextDriver.fontselect->height;

	if(size==0 || h+GFX_TextDriver.y_pos > EPD_HEIGHT)	return;

	w= *(const uint8_t*) &GFX_TextDriver.fontselect->width;
	//make sure cursor can accomodate spacing.
	if(alignment == centeralign && GFX_TextDriver.x_pos>=(size*w)/2){
		GFX_TextDriver.x_pos -= (size*w)/2;
	}
	if((w*size)+GFX_TextDriver.x_pos > EPD_WIDTH) return;

	while(size--){
		c = *(str++);
		if(c=='\n' || c=='\r') continue;
		c = (c>= 0x61 && c<=0x7A)? c-0x20:c;	//Remove if lower case graphics are included
		if(c >= *(const uint8_t*)&GFX_TextDriver.fontselect->first &&
			c<=*(const uint8_t*) &GFX_TextDriver.fontselect->last){
			GFX_drawChar(GFX_TextDriver.x_pos, GFX_TextDriver.y_pos, c, bg, fg);
			GFX_TextDriver.x_pos+= (uint8_t) w;//*(const uint8_t*) &GFX_TextDriver.fontselect->xAdvance;
		}
	}
}

/*******************************************************************************
* Description : Draws a single character
* Input       : uint8_t x: top left x coordinate
				uint8_t y: top left y coordinate
				unsigned char c: character to be printed
				EPD_grayscale bg: background color
				EPD_grayscale fg: foreground color
* Return      :
*******************************************************************************/
void GFX_drawChar(uint8_t x, uint8_t y, unsigned char c, EPD_grayscale bg, EPD_grayscale fg){
	uint16_t bo; uint16_t pixelcount;
	uint8_t w, i;
	uint8_t xx , bit=0;
	uint8_t val;
	const uint8_t * ptr;
	uint8_t color;

	c -= *(const uint8_t*) &GFX_TextDriver.fontselect->first;
	bo = c * (*(const uint16_t*) &GFX_TextDriver.fontselect->bitmapOffset);
    w = x+(*(const uint8_t*) &GFX_TextDriver.fontselect->width);
    pixelcount = *(const uint16_t*) &GFX_TextDriver.fontselect->numofpixels;
    ptr = (GFX_TextDriver.fontselect->bitmap)+bo;
    val = *ptr;
    xx=x;
    for(i=0;i<pixelcount;i++){
    	if(bit==8){
    		val = *(++ptr);
    		bit=0;
    	}
    	color = ((val>>bit)&0x01)? fg:bg;
   		GFX_drawPixel(xx++, y, color);
   		if(xx>=w){ xx=x; y++;}
   		bit++;
    }
}

/*******************************************************************************
* Description : Draws a circle
* Input       : int16_t x0: center x coordinate
				int16_t y0: center y coordinate
				int16_t r: radius of circle
				EPD_grayscale color: color of line
* Return      :
*******************************************************************************/
void drawCircle(int16_t x0, int16_t y0, int16_t r, EPD_grayscale color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    GFX_drawPixel(x0  , y0+r, color);
    GFX_drawPixel(x0  , y0-r, color);
    GFX_drawPixel(x0+r, y0  , color);
    GFX_drawPixel(x0-r, y0  , color);

    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        GFX_drawPixel(x0 + x, y0 + y, color);
        GFX_drawPixel(x0 - x, y0 + y, color);
        GFX_drawPixel(x0 + x, y0 - y, color);
        GFX_drawPixel(x0 - x, y0 - y, color);
        GFX_drawPixel(x0 + y, y0 + x, color);
        GFX_drawPixel(x0 - y, y0 + x, color);
        GFX_drawPixel(x0 + y, y0 - x, color);
        GFX_drawPixel(x0 - y, y0 - x, color);
    }
}

/*******************************************************************************
* Description : Draws a circle
* Input       : int16_t x0: center x coordinate
				int16_t y0: center y coordinate
				int16_t r: radius of circle
				EPD_grayscale color: color of circle
* Return      :
*******************************************************************************/
void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    drawFastVLine(x0, y0-r, 2*r+1, color);
    fillCircleHelper(x0, y0, r, 3, 0, color);
}

/*******************************************************************************
* Description : Fills a rectangle with one color
* Input       : int16_t x: Top left corner x coordinate
				int16_t y: Top left corner y coordinate
				int16_t w: width of rectangle
				int16_t h: height of rectangle
				EPD_grayscale color: color for rectangle
* Return      :
*******************************************************************************/
void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, EPD_grayscale color) {
    drawFastHLine(x, y, w, color);
    drawFastHLine(x, y+h-1, w, color);
    drawFastVLine(x, y, h, color);
    drawFastVLine(x+w-1, y, h, color);
}

/*******************************************************************************
* Description : Fills a rectangle with one color
* Input       : int16_t x: Top left corner x coordinate
				int16_t y: Top left corner y coordinate
				int16_t w: width of rectangle
				int16_t h: height of rectangle
				EPD_grayscale color: color for rectangle
* Return      :
*******************************************************************************/
void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, EPD_grayscale color) {
    for (int16_t i=x; i<x+w; i++) {
        drawFastVLine(i, y, h, color);
    }
}

/*******************************************************************************
* Description : Fills a rectangle with one color
* Input       : int16_t x: Top left corner x coordinate
				int16_t y: Top left corner y coordinate
				int16_t w: width of rectangle
				int16_t h: height of rectangle
				int16_t r: radius of corner rounding
				EPD_grayscale color: color for rectangle
* Return      :
*******************************************************************************/

void drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, EPD_grayscale color) {
    int16_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis
    if(r > max_radius) r = max_radius;
    // smarter version
    drawFastHLine(x+r  , y    , w-2*r, color); // Top
    drawFastHLine(x+r  , y+h-1, w-2*r, color); // Bottom
    drawFastVLine(x    , y+r  , h-2*r, color); // Left
    drawFastVLine(x+w-1, y+r  , h-2*r, color); // Right
    // draw four corners
    drawCircleHelper(x+r    , y+r    , r, 1, color);
    drawCircleHelper(x+w-r-1, y+r    , r, 2, color);
    drawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
    drawCircleHelper(x+r    , y+h-r-1, r, 8, color);
}

void fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, EPD_grayscale color) {
    int16_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis
    if(r > max_radius) r = max_radius;
    // smarter version
    fillRect(x+r, y, w-2*r, h, color);
    // draw four corners
    fillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
    fillCircleHelper(x+r    , y+r, r, 2, h-2*r-1, color);
}

/************************END OF FILE*******************************************/
