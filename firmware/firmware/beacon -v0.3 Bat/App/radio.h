/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
* File Name          : radio.h
* Author             : ?
* Version            : V1.0
* Date               : 6/2/2011
* Description        : ?
*******************************************************************************/

/* DEFINE TO PREVENT RECURSIVE INCLUSION -------------------------------------*/
#ifndef __RADIO_H
#define __RADIO_H

/* INCLUDES ------------------------------------------------------------------*/

#include <stdint.h>

/* EXPORTED TYPES ------------------------------------------------------------*/

/* EXPORTED CONSTANTS --------------------------------------------------------*/

/* EXPORTED MACROS -----------------------------------------------------------*/

#define HI_UINT16(a) (((uint16_t)(a) >> 8) & 0xFF)
#define LO_UINT16(a) ((uint16_t)(a) & 0xFF)

/* EXPORTED DEFINES ----------------------------------------------------------*/

/* EXPORTED FUNCTIONS --------------------------------------------------------*/

void                Send_SPI_2byte(uint16_t command, uint16_t s_data);
uint8_t             Send_SPI_byte(uint16_t s_data);
void                TK_BK_MEMWR8(uint16_t addr, uint8_t value);
void                TK_BK_MEMWR16(uint16_t addr, uint16_t value);
uint8_t             TK_BK_MEMRD8(uint16_t addr);
uint8_t             TK_BK_REGRD8(uint8_t addr);
void                TK_BK_REGWR8(uint8_t addr, uint8_t value);

#ifdef BC_HW_REVJ
uint8_t 				IMU_ReadOneByte(uint8_t reg);
void 				IMU_WriteOneByte(uint8_t reg, uint8_t Data);
void 				IMU_SelectBank(uint8_t bank);

#endif
/* EXTERNAL VARIABLES --------------------------------------------------------*/

#endif  /*__RADIO_H*/
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/