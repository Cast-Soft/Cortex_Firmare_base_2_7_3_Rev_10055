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

#include "CoOS.h"

// Notet that this defined in packets.h in timekeeper project
#define NUM_OF_IMU_PKTS_IN_RF_PKT           5

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


__packed struct Beacon_Preamble{
    uint8_t   Seq_Num;
    uint8_t   type;
    uint8_t   BeaconRSSI;
    uint8_t   SyncFrame;
    uint8_t   MsTimerIMU;
    uint8_t   IMUPktNum;
};
__packed struct BK_IMUData{
    uint8_t  Timestamp;
  	uint16_t gyroscopeX;
	uint16_t gyroscopeY;
	uint16_t gyroscopeZ;
	uint16_t accelerationX;
	uint16_t accelerationY;
	uint16_t accelerationZ;
};
__packed struct BK_BTNData{
	uint8_t buttonA_events;
	uint8_t	buttonB_events;
	uint8_t buttonA_tick;
	uint8_t	buttonB_tick;    
};
__packed struct BK_BATData{
	uint8_t Battery_lev;
	uint8_t charge_cycle;
};
__packed struct BK_LEDData{
	uint8_t led1ID;
	uint8_t led2ID;
	uint8_t led3ID;
	uint8_t led1stat;
	uint8_t led2stat;
	uint8_t led3stat;
};

//unique 15 types of messages pertaining the above 4 main messages. 
__packed struct IMU_Data_pkt{
 struct Beacon_Preamble BK_Preamble;
 struct BK_IMUData BeaconIMUData[NUM_OF_IMU_PKTS_IN_RF_PKT];
};
__packed struct BTN_Data_pkt{
	struct Beacon_Preamble BK_Preamble;
    struct BK_BTNData BeaconBTNData;	
};
__packed struct BTN_IMU_Data_pkt{
	struct Beacon_Preamble BK_Preamble;
    struct BK_BTNData BeaconBTNData;
    struct BK_IMUData BeaconIMUData[NUM_OF_IMU_PKTS_IN_RF_PKT];
};
__packed struct BAT_Data_pkt{
    struct Beacon_Preamble BK_Preamble;
    struct BK_BATData BeaconBATData;
};
__packed struct BAT_IMU_Data_pkt{
	struct Beacon_Preamble BK_Preamble;
    struct BK_BATData BeaconBATData;
    struct BK_IMUData BeaconIMUData[NUM_OF_IMU_PKTS_IN_RF_PKT];
};
__packed struct BAT_BTN_Data_pkt{
    struct Beacon_Preamble BK_Preamble;
    struct BK_BATData BeaconBATData;
    struct BK_BTNData BeaconBTNData;
};
__packed struct BAT_BTN_IMU_Data_pkt{
    struct Beacon_Preamble BK_Preamble;
    struct BK_BATData BeaconBATData;
    struct BK_BTNData BeaconBTNData;
    struct BK_IMUData BeaconIMUData[NUM_OF_IMU_PKTS_IN_RF_PKT];
};
__packed struct LED_Data_pkt{
    struct Beacon_Preamble BK_Preamble;
    struct BK_LEDData BeaconLEDData;
};
__packed struct LED_IMU_Data_pkt{
    struct Beacon_Preamble BK_Preamble;
    struct BK_LEDData BeaconLEDData;
    struct BK_IMUData BeaconIMUData[NUM_OF_IMU_PKTS_IN_RF_PKT];
};
__packed struct LED_BTN_Data_pkt{
    struct Beacon_Preamble BK_Preamble;
    struct BK_LEDData BeaconLEDData;
    struct BK_BTNData BeaconBTNData;
};
__packed struct LED_BTN_IMU_Data_pkt{
    struct Beacon_Preamble BK_Preamble;
    struct BK_LEDData BeaconLEDData;
    struct BK_BTNData BeaconBTNData;
    struct BK_IMUData BeaconIMUData[NUM_OF_IMU_PKTS_IN_RF_PKT];
};
__packed struct LED_BAT_Data_pkt{
    struct Beacon_Preamble BK_Preamble;
    struct BK_LEDData BeaconLEDData;
    struct BK_BATData BeaconBATData;
};
__packed struct LED_BAT_IMU_Data_pkt{
    struct Beacon_Preamble BK_Preamble;
    struct BK_LEDData BeaconLEDData;
    struct BK_BATData BeaconBATData;
    struct BK_IMUData BeaconIMUData[NUM_OF_IMU_PKTS_IN_RF_PKT];
};
__packed struct LED_BAT_BTN_Data_pkt{
    struct Beacon_Preamble BK_Preamble;
    struct BK_LEDData BeaconLEDData;
    struct BK_BATData BeaconBATData;
    struct BK_BTNData BeaconBTNData;
};
__packed struct LED_BAT_BTN_IMU_Data_pkt{
    struct Beacon_Preamble BK_Preamble;
    struct BK_LEDData BeaconLEDData;
    struct BK_BATData BeaconBATData;
    struct BK_BTNData BeaconBTNData;
    struct BK_IMUData BeaconIMUData[NUM_OF_IMU_PKTS_IN_RF_PKT];
};

// structure of array of IMU data from Beacon
//struct BeaconIMUDStruct
//{
//	struct BK_IMUData BeaconIMUData[8]; //
//};

//}

/* EXPORTED CONSTANTS --------------------------------------------------------*/

#define BASIC_RF_BROADCAST_ADDR     0xFFFF

#define FRAME_LENGTH_MAX            127

// FCF(2) + SeqNum(1) + PanID(2) + DstAddr(2) + SrcAddr(2)
#define MHR_SIZE                    (2 + 1 + 2 + 2 + 2)
// FCS(2)
#define MFR_SIZE                    2

#define IMU_SPI_INIT_STATE  0
#define IMU_SPI_BODY_STATE  1
/* EXPORTED MACROS -----------------------------------------------------------*/

/* EXPORTED DEFINES ----------------------------------------------------------*/

/* EXPORTED FUNCTIONS --------------------------------------------------------*/

//radio exported functions
void                RadioInit(uint16_t panId, uint16_t shortAddr, uint8_t chan,uint8_t test);
rxPkt_t*            RadioRxPkt();
void                RadioTxPkt(uint16_t dstAddr, uint8_t beacon, uint8_t payloadSize, uint8_t *payload, uint8_t trigger);
void                RadioSTXONBugFlush(void);
void                RadioSetPanIdShortAddr(uint16_t panId, uint16_t shortAddr);
void                RadioSetRFChan(uint8_t chan);
void                RadioSetRFLevel(uint8_t TxLev);
uint8_t             RadioGetFLevel(void);
void                RadioPrint2520Registers(uint8_t flag);
uint32_t            RadioGetRandom();

//imu exported functions
void 				IMUInit(void);
void				IMUProcess(void);

/* EXTERNAL VARIABLES --------------------------------------------------------*/
extern const uint8_t TxAmpValues[];

/* --- FOR ISR-USE ONLY --- */

extern volatile uint8_t spiTxRxByteCount;
extern volatile uint8_t spiTxRxByteState;

extern uint8_t*             pSpiTxBuf;
extern uint8_t*             pSpiRxBuf;
extern rxPkt_t              rxPkt;
extern uint8_t              scratchBuf[];

extern OS_FlagID            flagRadioTxDone;
extern OS_FlagID            flagRadioTxAllow;

extern volatile uint16_t txDoneType; // 0: successful, 1: failed
extern volatile uint16_t rxDoneType; // 0: successful, 1: failed
extern volatile uint16_t rxFIFOError; // 0: no error, 1: with rror
extern volatile uint16_t txFIFOError; // 0: no error, 1: with rror

extern volatile uint8_t spiIMUCount;
extern volatile uint8_t spiIMUByteState;
	
extern uint8_t*		pSpiRxBuf_IMU;
extern uint8_t*		pSpiTxBuf_IMU;
extern uint8_t	    IMU_RawData[];
// acquiring SPI lock is not enough.
// This is for SPI3_IRQHandler to notify an SPI state machine is done (goes back to init state) so that SPI user can start another action
// When SPI state machine is rolling, tasks should wait it finish to get control
extern OS_FlagID flagSPIMachineDone;


#endif  /*__BASIC_RF_H*/
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/