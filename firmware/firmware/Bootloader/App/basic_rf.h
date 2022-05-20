/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
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


#define             RF_CHANNEL_MAX          21
#define             RF_CHANNEL_MIN          9

/* EXPORTED TYPES ------------------------------------------------------------*/

extern const uint8_t RF_SPI_INIT_STATE;                 // Can start an new RF_SPI operation from this state only
extern const uint8_t RF_SPI_UPLOAD_ONLY_STATE;          // Upload only. Does not care download content. For upload frame to CC2520
extern const uint8_t RF_SPI_RX1_UPLOADCMD_STATE;        // RX frame step 1:  upload command RXBUF
extern const uint8_t RF_SPI_RX2_DOWNLOAD_HEADER_STATE;  // RX frame step 2:  download frame header
extern const uint8_t RF_SPI_RX3_DOWNLOAD_BODY_STATE;    // RX frame step 3:  download frame body

// these are not verified yet
extern const uint8_t RF_SPI_CCA_CMD_STATE;       // CCA upload read register FSMSTAT1 command
extern const uint8_t RF_SPI_STXONCCA_CMD_STATE;

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
rxPkt_t*            RadioRxPkt();
void                RadioSTXONBugFlush(void);
void                RadioSetPanIdShortAddr(uint16_t panId, uint16_t shortAddr);
void                RadioSetRFChan(uint8_t chan);
void                RadioSetRFLevel(uint8_t Lev);
void                Print_2520_reg(void);

/* EXTERNAL VARIABLES --------------------------------------------------------*/
extern const uint8_t TxAmpValues[];



/* --- FOR ISR-USE ONLY --- */

extern volatile uint8_t spiTxRxByteCount;
extern volatile uint8_t spiTxRxByteState;

extern uint8_t*             pSpiTxBuf;
extern uint8_t*             pSpiRxBuf;
extern rxPkt_t              rxPkt;
extern uint8_t              scratchBuf[];

extern volatile uint16_t rxDoneType; // 0: successful, 1: failed
extern volatile uint16_t rxFIFOError; // 0: no error, 1: with error

// acquiring SPI lock is not enough. 
// This is for SPI3_IRQHandler to notify an SPI state machine is done (goes back to init state) so that SPI user can start another action 
// When SPI state machine is rolling, tasks should wait it finish to get control
//extern OS_FlagID flagSPIMachineDone;

#endif  /*__BASIC_RF_H*/
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/