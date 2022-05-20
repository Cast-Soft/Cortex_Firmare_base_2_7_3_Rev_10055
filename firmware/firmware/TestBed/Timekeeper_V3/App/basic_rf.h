/******************** (C) COPYRIGHT 2011 NaturalPoint, Inc. ********************
* File Name          : basic_rf.h
* Author             : ?
* Version            : V1.0
* Date               : 6/2/2011
* Description        : ?
*******************************************************************************/

/* DEFINE TO PREVENT RECURSIVE INCLUSION -------------------------------------*/
#ifndef __BASIC_RF_H
#define __BASIC_RF_H

/* INCLUDES ------------------------------------------------------------------*/

#include <stdint.h>
#ifdef COOS
#include "CoOS.h"
#endif

/* EXPORTED TYPES ------------------------------------------------------------*/

 typedef struct {
    uint8_t     scratch[4];     // dummy bytes - do not remove!
    uint8_t     frameLength;    // PHR
    uint8_t     fcf0;           // MHR: Frame Control Field LSB
    uint8_t     fcf1;           // MHR: Frame Control Field MSB
    uint8_t     seqNumber;      // MHR
    uint16_t    panId;          // MHR
    uint16_t    destAddr;       // MHR
    uint16_t    srcAddr;        // MHR
    uint8_t     payload[128];   // MAC Payload
    uint8_t     payloadSize;    // MAC Payload Size
    uint8_t     rssi;           // radio's RSSI stamped value
    uint8_t     lqi;            // radio's LQI stamped value
} rxPkt_t;

/* EXPORTED CONSTANTS --------------------------------------------------------*/

#define BASIC_RF_BROADCAST_ADDR     0xFFFF

#define FRAME_LENGTH_MAX            127

// FCF(2) + SeqNum(1) + PanID(2) + DstAddr(2) + SrcAddr(2)
#define MHR_SIZE                    (2 + 1 + 2 + 2 + 2)
// FCS(2)
#define MFR_SIZE                    2

/* EXPORTED MACROS -----------------------------------------------------------*/

/* EXPORTED DEFINES ----------------------------------------------------------*/

/* EXPORTED FUNCTIONS --------------------------------------------------------*/

void                RadioInit(uint16_t panId, uint16_t shortAddr, uint8_t chan);
rxPkt_t*            RadioRxPkt(uint16_t wait);
void                RadioTxPkt(uint16_t dstAddr, uint8_t beacon, uint8_t payloadSize, uint8_t *payload, uint8_t trigger);
void                RadioSTXONBugFlush(void);
void                RadioSetPanIdShortAddr(uint16_t panId, uint16_t shortAddr);
void                RadioSetRFChan(uint8_t chan);
void                RadioSetRFLevel(uint8_t Lev);
void                Print_2520_reg(void);

/* EXTERNAL VARIABLES --------------------------------------------------------*/
extern const uint8_t TxAmpValues[];
extern const uint8_t RFChanValues[];


/* --- FOR ISR-USE ONLY --- */

extern volatile uint16_t    spiTxRxByteCount;
extern uint8_t*             pSpiTxBuf;
extern uint8_t*             pSpiRxBuf;
extern rxPkt_t              rxPkt;
extern uint8_t              scratchBuf[];

#ifdef COOS
extern OS_FlagID            flagRadioTxDone;
extern OS_FlagID            flagRadioRxFrame;
#else
extern volatile uint16_t    flagRadioTxDone;
extern volatile uint16_t    flagRadioRxFrame;
#endif

/* --- */

#ifdef COOS
extern OS_FlagID            flagRadioTxDoneUser;
#else
extern volatile uint16_t    flagRadioTxDoneUser;
#endif

#endif  /*__BASIC_RF_H*/
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/