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

/* EXPORTED TYPES ------------------------------------------------------------*/
#if 0
typedef __packed struct {
    uint16_t    productID;
    uint16_t    serialNum;
    uint16_t    panId;
    uint16_t    mySrcAddr;
    uint16_t    tkDstAddr;
    uint16_t    ledOnOffs;
    uint16_t    ledOffOffs;
    uint16_t    ledDAC;
    uint8_t     rfChan;
    uint8_t     rfTimeSlot;
    uint8_t     led0Id;
    uint8_t     led1Id;
    uint8_t     led2Id;
    uint8_t     TestMode;
    uint8_t     TxLevel;
    uint8_t     radioPacketFlags;
    uint32_t    u32IwdgResetEvents;
    /* !! ABOVE MUST HAVE EVEN # BYTES!! */
    uint16_t    checksum;   // MUST BE LAST!!
} config_t;

typedef __packed struct {
  uint16_t      size;
  uint16_t      checksum;
//  from config_t without checksum and u32IwdgResetEvents;
  uint16_t    productID;
  uint16_t    serialNum;
  uint16_t    panId;
  uint16_t    mySrcAddr;
  uint16_t    routerDstAddr;
  uint16_t    ledOnOffs;
  uint16_t    ledOffOffs;
  uint16_t    ledDAC;
  uint8_t     rfChan;
  uint8_t     rfTimeSlot;
  uint8_t     led0Id;
  uint8_t     led1Id;
  uint8_t     led2Id;
  uint8_t     TestMode;
  uint8_t     TxLevel;
  uint8_t     radioPacketFlags;

  uint32_t      led0IdPattern;
  uint32_t      led1IdPattern;
  uint32_t      led2IdPattern;
  uint32_t      led0Index;
  uint32_t      led1Index;
  uint32_t      led2Index;
  uint8_t       frameBits;
#ifndef OLD_CONFIG
  uint16_t      imuSlots; // by default 0xFFFF, which means
                          // all slots are for IMU. Any cleared
                          // bit is timeslot dedicated for random sending
                          // battery status. Corresponds to config.rfTimeSlot
  uint8_t       debounce_time; // *10 to get millisec to debounce button
                               // range 1- 9; default 4 (40 mS)
  uint8_t       doubleclick_time; // *100 time to wait for double click
                                  // range 1- 9, default 5 (500 mS)
#endif

}beacon_config_t;
#endif

struct realTime{
  uint32_t sec;
  uint16_t uSec;
};

typedef struct {
  uint8_t actual_debounce;
  uint8_t last_state;
  uint8_t pending_state;
  uint8_t actual_dblclick_time;
  uint8_t events;
}tButton;

typedef enum{
  BTNSTATE_OFF          =0,
  BTNSTATE_ON           =1,
  BTNSTATE_HOLD         =2,
  BTNSTATE_PRESSDETECT  =3,
  BTNSTATE_UIPWRMGMT    =4,
  BTNSTATE_UICTRL       =5,
}btnstate;

typedef struct{
  uint8_t state;
  uint8_t hcount;
  uint8_t prevstate;
  uint8_t message;
  uint8_t mcount;
  uint8_t sequence;
}tBtn_State;

typedef enum{
  BTNMSG_OFF      =0x00,
  BTNMSG_PRESS    =0x01,
  BTNMSG_DBLCLK   =0x02,
  BTNMSG_HOLD     =0x04,
  BTNMSG_RELEASE  =0x08
}Btn_TXMSG;

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
void Task7(void* pdata);
void Task8(void* pdata);
void TaskIMU_A(void* pdata);
void TaskIMU_G(void* pdata);
int TRACE(char* fmt, ...);

/* EXTERNAL VARIABLES --------------------------------------------------------*/

extern config_t backup_config, config;
extern uint8_t test_imu_pkt_ctr;
extern volatile int32_t remainOutOfSyncTime;
extern uint8_t button_state;

extern tButton buttonA;
extern tButton buttonB;


extern uint8_t          frameIdFlag;
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
