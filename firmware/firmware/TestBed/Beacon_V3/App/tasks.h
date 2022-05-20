/******************** (C) COPYRIGHT 2011 NaturalPoint, Inc. ********************
* File Name          : tasks.h
* Author             : ?
* Version            : V1.0
* Date               : 6/2/2011
* Description        : ?
*******************************************************************************/

/* DEFINE TO PREVENT RECURSIVE INCLUSION -------------------------------------*/
#ifndef __TASKS_H
#define __TASKS_H

/* INCLUDES ------------------------------------------------------------------*/

#include <stdint.h>

/* EXPORTED TYPES ------------------------------------------------------------*/

typedef struct {
    uint16_t    productID;
    uint16_t    serialNum;
    uint16_t    panId;
    uint16_t    mySrcAddr;
    uint16_t    tkDstAddr;
    uint16_t    ledOnOffs;
    uint16_t    ledOffOffs;
    uint16_t    ledDAC;
    uint8_t     rfChan;
    uint8_t     rfChanMin;
    uint8_t     rfChanMax;
    uint8_t     led0Id;
    uint8_t     led1Id;
    uint8_t     led2Id;
    uint8_t     TestMode;
    uint8_t     TxLevel;
    uint8_t     RxPrint;
    uint8_t     AtoDon_off;
    /* !! ABOVE MUST HAVE EVEN # BYTES!! */
    uint16_t    checksum;   // MUST BE LAST!!
} config_t;


typedef struct{
     uint32_t a;
     uint32_t b;
     uint32_t c;
} DeviceSerialNumber_t;

/* EXPORTED CONSTANTS --------------------------------------------------------*/

/* EXPORTED MACROS -----------------------------------------------------------*/

/* EXPORTED DEFINES ----------------------------------------------------------*/

#define FLASH_CONFIG_PAGE   0x0803F800   // Page#127 (last page of main memory)

/* EXPORTED FUNCTIONS --------------------------------------------------------*/

void Task1(void* pdata);
void Task2(void* pdata);
void Task3(void* pdata);
void Task4(void* pdata);
void Task5(void* pdata);
void Task6(void* pdata);
void Task7(void* pdata);
void Task8(void* pdata);

void GetARM_UUID(void);
/* EXTERNAL VARIABLES --------------------------------------------------------*/

extern config_t config;

#endif  /*__TASKS_H*/
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
