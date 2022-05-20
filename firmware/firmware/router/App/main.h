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
#include "util.h"
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
  const char *  taskRadioTx_func;
  const char *  taskEther_func;
  const char *  taskConfig_func;
  StatusType            enterFlagSPIIODone;
  StatusType            leaveFlagSPIIODone;
  StatusType            pendSemRFRxFrames;
  StatusType            clrSPIDone;
  StatusType            waitSPIDone;
  StatusType    setEtherTxDone;
  StatusType    acceptEtherTxDone;
  StatusType            postEthernetRcvd;
  StatusType            setSemRFRxFrames;
  StatusType            setFlagSPIDone1;
  StatusType            setFlagSPIDone2;
  StatusType            setFlagSPIDone3;
  StatusType            postEthTaskQueue;
  // 54 bytes
  uint8_t               flags;
} tDiagnostics;

// When hard fault occurs it clears this bit in flags {tDiagnostics} : 0x7F
#define DIAGNOSTICS_NOTFLAG_HARDFAULT   0x80
// By default last event would be a hard fault happened, if not, like
// CoOS lock up, it will be cleared in main
#define DIAGNOSTICS_LASTEVENT_HARDFAULT 0x40

#pragma pack(pop)

extern OS_EventID ethTaskMsgQueue;


#define DO_NOT_FREE             0x80
#define FLAG_BUSY               0x40

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

extern void* MallocMsg(uint16_t size);          // malloc a block for message with size
extern void* GetMsgBody(void* msg);             // skip the header
extern MsgHeader* GetMsgHeader(void* msg);      // the header
int TRACE(char* fmt, ...);
void RadioWaitGrabSPI();
void RadioReleaseSPI();
size_t __writeCmdLineRespPacket(const unsigned char *buffer, size_t size, uint8_t contentType);

// pre-defined allocated messages to save heap. Read only; DO NOT free them
extern MsgHeader* wakeupMsg;
extern tDiagnostics diagnostic;

// Another message queue. As outgoing ethernet data can be blocked / failed, a queue is introduced to prevent data from losing
#define MSG_TYPE_ETH_TX_FRAME  0x04       // for Tx message queue only
#define IS_TX_FRAME(A) ((A & MSG_TYPE_ETH_TX_FRAME) == MSG_TYPE_ETH_TX_FRAME)
#define SET_TX_FRAME(A) (A = ((A & 0xF6) | MSG_TYPE_ETH_TX_FRAME))


extern OS_EventID ethTxMsgQueue;

extern OS_EventID semRFRxFrames;    // Number of RF frames received in Rx-FIFO. It is notified by RX_FRM_DONE interrupt. CC2520 allows for multiple frames in Rx-FIFO

extern config_t config;
extern uint16_t rfBeaconEn ;
extern uint8_t task_ticks[];

// Communication header version
#define RT_VERSION_MAJOR        2
#define RT_VERSION_MINOR        7
#define RT_VERSION_REVISION     0
#define RT_VERSION_BUILD        0

#define SAVE_POINT      func = __func__; line = __LINE__;
#define SAVE_FUNC       func = __func__;
#define SAVE_LINE       line = __LINE__;

#ifdef WDT_ENABLE
extern uint8_t  watchdog_active;
#define RELOAD_WATCHDOG if (watchdog_active) {IWDG_ReloadCounter();}
#else
#define RELOAD_WATCHDOG
#endif

#endif


