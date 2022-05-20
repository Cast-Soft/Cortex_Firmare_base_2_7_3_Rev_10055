/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
* File Name          : eink.h
* Author             : Matthew Mak			
* Version            : V1.0
* Date               : 5/17/2019
* Description        : This header file is used to interface with the Plastic Logic 700756
					   display exclusively.
*******************************************************************************/

/* DEFINE TO PREVENT RECURSIVE INCLUSION -------------------------------------*/
#ifndef __EINK_H
#define __EINK_H

/* INCLUDES ------------------------------------------------------------------*/
#include <stdint.h>

/* EXPORTED DEFINES ----------------------------------------------------------*/
#define EPD_WIDTH	146
#define EPD_HEIGHT	240
#define EPD_BUFFERSIZE (EPD_WIDTH*EPD_HEIGHT)/4
/* EXPORTED TYPES ------------------------------------------------------------*/
typedef struct{
	uint8_t regval;
	uint8_t buffer[EPD_BUFFERSIZE]; 
}EPD_buffer;

typedef enum{
	fullrefresh = 0,
	partialrefresh = 1
}EPD_refresh;

/* EXPORTED CONSTANTS --------------------------------------------------------*/

/* EXPORTED MACROS -----------------------------------------------------------*/

/* EXPORTED FUNCTIONS --------------------------------------------------------*/
uint8_t Eink_Init(void);
void init_coos_eink(void);
void WaitGrabEInk(void);
void ReleaseEInk(void);
void whiteerase(void);
void update(EPD_refresh type);
/* EXTERNAL VARIABLES --------------------------------------------------------*/
extern volatile EPD_buffer spiInk_ImgBuf;
extern volatile uint8_t spiInk_RxBuf[5];
extern uint8_t EInk_Present;
#endif /*__EINK_H*/
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
