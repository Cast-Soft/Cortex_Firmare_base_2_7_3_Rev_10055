/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
* File Name          : eink_comms.h
* Author             : Matthew Mak			
* Version            : V1.0
* Date               : 5/17/2019
* Description        : This header file is used to interface with the Plastic Logic 700756
					   display exclusively.
*******************************************************************************/

/* DEFINE TO PREVENT RECURSIVE INCLUSION -------------------------------------*/
#ifndef __EINK_COMMS_H
#define __EINK_COMMS_H

/* INCLUDES ------------------------------------------------------------------*/
#include <stdint.h>
/* EXPORTED TYPES ------------------------------------------------------------*/

/* EXPORTED CONSTANTS --------------------------------------------------------*/

/* EXPORTED MACROS -----------------------------------------------------------*/

/* EXPORTED DEFINES ----------------------------------------------------------*/

/* EXPORTED FUNCTIONS --------------------------------------------------------*/
uint8_t EINK_RegisterRead(uint8_t addr);
void EINK_RegisterReadBytes(uint8_t addr, uint8_t * buf, uint8_t cnt);
uint8_t EINK_MTP_RAM_Read(uint8_t addr);
void EINK_RegisterWrite(uint8_t reg, uint8_t val1, uint8_t val2, uint8_t val3, uint8_t val4, uint8_t count);
void EINK_write2ram(void);
void EINK_readram(void);
/* EXTERNAL VARIABLES --------------------------------------------------------*/


#endif /*__EINK_COMMS_H*/
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
