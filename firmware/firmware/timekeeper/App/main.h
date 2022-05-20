/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
******************************************************************************/
// main.h
//
//
//
//
/**********************************************************************/
/* DEFINE TO PREVENT RECURSIVE INCLUSION -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#include "CoOS.h"
#include <ysizet.h>
#include "config.h"
#include "packets.h"

#define DIAGNOSTIC_VERSION      1

#pragma pack(push, 1)

// command msg queue item header format. For inter-task communication only.
typedef struct {
    uint8_t    msgType;
    uint16_t   msgLength;       // body length only. No header
} MsgHeader;

typedef struct {
  uint16_t      event_count;
  uint32_t      version;
  uint32_t      size;
  uint32_t      rfTxErrors;
  uint32_t      ethRxRcvdOk;
  uint32_t      ethRxDropped;
  uint8_t       stopped_task;
  uint16_t      taskRadioTx_line;
  uint16_t      taskEther_line;
  uint16_t      taskConfig_line;
  uint32_t      taskRadioCycles;
  uint32_t      taskEtherCycles;
  uint32_t      taskConfigCycles;
  const char *  taskRadioTx_func;
  const char *  taskEther_func;
  const char *  taskConfig_func;
  StatusType    setLoadFrame;
  StatusType    waitLoadFrame;
  StatusType    setRFBeaconSent;
  StatusType    waitRFBeaconSent;
  StatusType    setRadioTxDone;
  StatusType    waitRadioTxDone;
  StatusType    setSPIMachineDone;
  StatusType    waitSPIMachineDone;
  StatusType    setEtherTxDone;
  StatusType    acceptEtherTxDone;
  StatusType    postEthernetRcvd;


} tDiagnostics;


#pragma pack(pop)


/**************************************************************************
 ** Variables
 *************************************************************************/


// command msg queue item header format. For inter-task communication only.
extern OS_EventID       ethTaskMsgQueue;
extern OS_EventID       ethTxMsgQueue;
extern OS_FlagID        flagRFBeaconSent;
extern OS_FlagID        flagLoadFrameId;
extern OS_FlagID        flagEtherTxDone;

// pre-defined allocated messages to save heap. Read only; DO NOT free them
extern MsgHeader*       wakeupMsg;
extern tDiagnostics     diagnostic;
extern config_t         config;
extern tDataSend        dataFramesTx[];
extern uint32_t         magicNumber;
extern uint32_t         dma_done;
extern volatile uint32_t rfBeaconFrameId;
extern volatile uint16_t rfBeaconLoaded;
extern uint8_t          tick;
extern int16_t          whole_time_adjust;
extern int8_t           part_time_adjust;
extern uint8_t          rfChannel;

/*
extern StatusType       setLoadFrame;
extern StatusType       setRFBeaconSent;
extern StatusType       setRadioTxDone;
extern StatusType       setSPIMachineDone;
extern StatusType       setEtherTxDone;
extern StatusType       acceptEtherTxDone;
extern StatusType       waitLoadFrame;
extern StatusType       waitRFBeaconSent;
extern StatusType       postEthernetRcvd;

*/

/**************************************************************************
 ** Defines
 *************************************************************************/

#define DO_NOT_FREE             0x80
#define FLAG_BUSY               0x40

#define FRAME_OFFSET_DEFAULT    8

#define MSG_TYPE_ETH_RX_FRAME   0       // A received Ethernet frame
#define MSG_TYPE_ETH_TX_PACKET  1       // A packet (not a ethernet frame) to send out
#define MSG_TYPE_WAKEUP         2       // wakeup the queue to check anything is happened

#define IS_RX_FRAME(A) ((A & 0x03) == MSG_TYPE_ETH_RX_FRAME)
#define IS_TX_PACKET(A) ((A & 0x03) == MSG_TYPE_ETH_TX_PACKET)
#define IS_WAKEUP(A) ((A & 0x03) == MSG_TYPE_WAKEUP)
#define IS_BUSY(A) (A & FLAG_BUSY)

#define SET_RX_FRAME(A) (A = ((A & 0xF6) | MSG_TYPE_ETH_RX_FRAME))
#define SET_TX_PACKET(A) (A = ((A & 0xF6) | MSG_TYPE_ETH_TX_PACKET))
#define SET_WAKEUP_FRAME(A) (A = ((A & 0xF6) | MSG_TYPE_WAKEUP))
#define SET_FLAG_BUSY(A) (A |= FLAG_BUSY)
#define CLEAR_FLAG_BUSY(A) (A &= ~FLAG_BUSY)

#define IS_ON_HEAP(A) ((A & DO_NOT_FREE) == 0x00)

// Another message queue. As outgoing ethernet data can be blocked / failed, a queue is introduced to prevent data from losing
#define MSG_TYPE_ETH_TX_FRAME  0x04       // for Tx message queue only
#define IS_TX_FRAME(A) ((A & MSG_TYPE_ETH_TX_FRAME) == MSG_TYPE_ETH_TX_FRAME)
#define SET_TX_FRAME(A) (A |= MSG_TYPE_ETH_TX_FRAME)

extern config_t config;

/**************************************************************************
 ** Functions
 *************************************************************************/

extern void*            MallocMsg(uint16_t size);          // malloc a block for message with size
extern void*            GetMsgBody(void* msg);             // skip the header
extern MsgHeader*       GetMsgHeader(void* msg);      // the header
extern int              b64_ntop(unsigned char const *src,
                                 size_t srclength,
                                 char *target,
                                 size_t targsize);
extern int              b64_pton(char const *src,
                                 unsigned char *target,
                                 size_t targsize);

extern int              TRACE(char* fmt, ...);
extern void             RadioWaitGrabSPI();
extern void             RadioReleaseSPI();
extern size_t           __writeCmdLineRespPacket(const unsigned char *buffer,
                                                 size_t size,
                                                 uint8_t contentType);

void GetARM_UUID(void);

extern void             RX_Pkt_Handler(void);
extern uint8_t          task_ticks[];
extern OS_TID          taskRadioTxId;
extern OS_TID          taskConfigId;
extern OS_TID          taskEtherId;

// Packet header version
#define TK_VERSION_MAJOR        2
#define TK_VERSION_MINOR        7
#define TK_VERSION_REVISION     0
#define TK_VERSION_BUILD        0

#define SAVE_POINT      func = __func__; line = __LINE__;
#define SAVE_FUNC       func = __func__;
#define SAVE_LINE       line = __LINE__;

//#ifdef WDT_ENABLE
extern uint8_t  watchdog_active;
#define RELOAD_WATCHDOG if (watchdog_active) {IWDG_ReloadCounter();}
//#else
//#define RELOAD_WATCHDOG
//#endif

extern BOOL usePreciseClock;

#endif