/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************/
// main.h
//
//
//
//
/**********************************************************************/
/* DEFINE TO PREVENT RECURSIVE INCLUSION -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H


#include <ysizet.h>
//#include "build.h"

typedef __packed struct {
    uint16_t    productID;
    uint16_t    serialNum;
    uint16_t    panId;
    uint16_t    mySrcAddr;
    uint8_t     rfChan;
    uint8_t     TxPower;
    uint8_t     TestMode;
    uint8_t     SyncOutEn;
    uint32_t    u32IwdgResetEvents;
    //uint16_t     Dummy1;

    /* !! ABOVE MUST HAVE EVEN # BYTES!! */
    uint16_t    checksum;   // MUST BE LAST!!
} config_t;

// command msg queue item header format. For inter-task communication only.
typedef __packed struct {
    uint8_t    msgType;
    uint16_t   msgLength;       // body length only. No header
} MsgHeader;

//extern OS_EventID ethTaskMsgQueue;


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


// pre-defined allocated messages to save heap. Read only; DO NOT free them
extern MsgHeader* wakeupMsg;

// Another message queue. As outgoing ethernet data can be blocked / failed, a queue is introduced to prevent data from losing
#define MSG_TYPE_ETH_TX_FRAME  0x04       // for Tx message queue only
#define IS_TX_FRAME(A) ((A & MSG_TYPE_ETH_TX_FRAME) == MSG_TYPE_ETH_TX_FRAME)
#define SET_TX_FRAME(A) (A = ((A & 0xF6) | MSG_TYPE_ETH_TX_FRAME))


//extern OS_EventID ethTxMsgQueue;

//extern OS_EventID semRFRxFrames;    // Number of RF frames received in Rx-FIFO. It is notified by RX_FRM_DONE interrupt. CC2520 allows for multiple frames in Rx-FIFO

extern config_t config;
extern uint16_t rfBeaconEn ;

void DelayMs(uint32_t ms);
size_t __writeCmdLineRespPacket(const unsigned char *buffer, size_t size, uint8_t contentType);
size_t __writeNetworkResponse(uint8_t content_type, const unsigned char *buffer, size_t size);

uint32_t doPackets(const char *encoded, uint8_t peer);

extern const char *BOOTLOADER_VERSION;
#define FLAG_ETHER_RX_PACKET       0x10
#define FLAG_ETHER_TX_PACKET       0x20
#define FLAG_LOADING               0x01
#define FLAG_LOADED                0x02
#define FLAG_UDP_CONNECT           0x40
#define FLAG_USB_CONNECTED         0x100
#if defined(TIMEKEEPER) || defined(ROUTER)
  #define FLAG_BOOTLOADER_CONNECTED  0x1000
#endif

#define Bootloader_Listen_Port  24000
#define Bootloader_Send_Port    24001

#define PEER_USB        0x01
#define PEER_UDP        0x02

extern uint32_t flags;
#endif
