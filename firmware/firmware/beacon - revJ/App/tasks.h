/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
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
#include "config.h"
#include "hardware.h"
/* EXPORTED TYPES ------------------------------------------------------------*/
struct realTime{
  uint32_t sec;
  uint16_t uSec;
};

#ifdef CIRCULAR_LOG
typedef struct {
  uint32_t      timestamp;
  uint32_t      frameId;
  uint8_t       type;
}tLogStruct;
#define LOG_SIZE        0x4000
extern tLogStruct log[]; //LOG_SIZE/sizeof(tLogStruct)
extern uint16_t  log_index_in;

#define LOG_TYPE_RXPACKET       1
#define LOG_TYPE_FRAMEADJUST    2
#define LOG_TYPE_TIMER          3
#define LOG_TYPE_FRAMEDIFF      4
#define LOG_TYPE_CAPTURE        5
#define LOG_TYPE_ARR            6


#define W_LOG(time, tp, frame)                  {log[log_index_in].timestamp = time; \
                                                 log[log_index_in].type = tp; \
                                                 log[log_index_in].frameId = frame; \
                                                   log_index_in++; log_index_in %= (LOG_SIZE)/(sizeof(tLogStruct));}

#define WRITE_LOG(ts, t, f)                     {if (!radio_off) { \
                                                 W_LOG(ts, t, f); \
                                                 } };

#define WRITE_LOG_SAFE(ts, t, f) {if (!radio_off) { \
                                                 __disable_interrupt(); \
                                                 W_LOG(ts, t, f); \
                                                 __enable_interrupt();}}

#endif
/* EXPORTED CONSTANTS --------------------------------------------------------*/

/* EXPORTED MACROS -----------------------------------------------------------*/

/* EXPORTED DEFINES ----------------------------------------------------------*/

#define FLASH_CONFIG_PAGE   0x0803F800   // Page#127 (last page of main memory)

/* EXPORTED FUNCTIONS --------------------------------------------------------*/

void Task1(void* pdata);
void TaskRadioRx(void* pdata);
void Task3(void* pdata);
void TaskConfig(void* pdata);
void Task5(void* pdata);
void TaskRadioTx(void* pdata);
void Task8(void* pdata);
void TaskIMU_G(void* pdata);
void TaskLED(void* pdata);
void TaskEInk(void* pdata);
void TaskUI(void* pdata);
int TRACE(char* fmt, ...);

/* EXTERNAL VARIABLES --------------------------------------------------------*/

extern config_t backup_config, config;
extern uint8_t test_imu_pkt_ctr;
extern volatile int32_t remainOutOfSyncTime;

extern volatile ledcheck led_select;
extern IRled_status IRled_flags;

extern volatile uint8_t frameIdFlag;
extern uint32_t         errorFrameId;

#define SAVE_POINT      func = __func__; line = __LINE__;
#define SAVE_FUNC       func = __func__;
#define SAVE_LINE       line = __LINE__;

extern uint8_t  watchdog_active;
#ifdef WDT_ENABLE
#define RELOAD_WATCHDOG if (watchdog_active) {IWDG_ReloadCounter();}
#else
#define RELOAD_WATCHDOG
#endif

#endif  /*__TASKS_H*/
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
