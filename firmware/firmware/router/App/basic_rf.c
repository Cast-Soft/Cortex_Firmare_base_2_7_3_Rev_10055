/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
* File Name          : basic_rf.c
* Author             : ?
* Version            : V1.0
* Date               : 6/2/2011
* Description        : Basic RF library
*******************************************************************************/

/*
    FRAME FORMATS:
    Data packets (without security):
    [Preambles (4)][SFD (1)][Length (1)][Frame control field (2)]
    [Sequence number (1)][PAN ID (2)][Dest. address (2)][Source address (2)]
    [Payload (Length - 2+1+2+2+2)][Frame check sequence (2)]
*/

/* INCLUDES ------------------------------------------------------------------*/

#include "hardware.h"
#include "basic_rf.h"
#include "radio.h"
#include "radio_defs.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "CoOS.h"

/* PRIVATE TYPEDEF -----------------------------------------------------------*/

// Basic RF packet header (IEEE 802.15.4)
typedef struct {
    uint8_t     frameLength;    // PHR
    uint8_t     fcf0;           // MHR: Frame Control Field LSB
    uint8_t     fcf1;           // MHR: Frame Control Field MSB
    uint8_t     seqNumber;      // MHR
    uint16_t    panId;          // MHR
    uint16_t    destAddr;       // MHR
    uint16_t    srcAddr;        // MHR
} txPktHdr_t;

typedef struct {
    uint16_t    myAddr;
    uint16_t    myPanId;
    uint8_t     channel;
    uint8_t     level;
} radioConfig_t;

typedef struct {
    uint8_t reg;
    uint8_t val;
} regVal_t;

//const uint8_t TxAmpValues[9]={0xf7,0xF2,0xAB,0x13,0x32,0x81,0x88,0x2C,0x03};
const uint8_t TxAmpValues[9]={0x03,0x2C,0x88,0x81,0x32,0x13,0xAB,0xF2,0xF7};

const uint8_t RF_SPI_INIT_STATE = 0xFF;                 // Can start an new RF_SPI operation from this state only
const uint8_t RF_SPI_UPLOAD_ONLY_STATE = 0x00;          // Upload only. Does not care download content. For upload frame to CC2520
const uint8_t RF_SPI_RX1_UPLOADCMD_STATE = 0x40;        // RX frame step 1:  upload command RXBUF
const uint8_t RF_SPI_RX2_DOWNLOAD_HEADER_STATE = 0x20;  // RX frame step 2:  download frame header
const uint8_t RF_SPI_RX3_DOWNLOAD_BODY_STATE = 0x10;    // RX frame step 3:  download frame body

// these are not verified yet
const uint8_t RF_SPI_CCA_CMD_STATE = 0x08;       // CCA upload read register FSMSTAT1 command
const uint8_t RF_SPI_STXONCCA_CMD_STATE = 0x04;
uint32_t rxErrors;
extern uint32_t sec;

/* PRIVATE DEFINES -----------------------------------------------------------*/

// Frame Control Field
#define BASIC_RF_FCF_DATA               0x8841
#define BASIC_RF_FCF_BEACON             0x8040

// FrameLength(1)
#define PHR_SIZE                        1

/* IEEE 802.15.4 (2.4 GHz logical channels) */
#define MIN_CHANNEL                     11 // 2405 MHz
#define CHANNEL_SPACING                 5  // 5 MHz

/* CC2590 LNA Gain */
#define CC2590_HGM                      CC2520_GPIO_LOW
//#define CC2590_HGM                    CC2520_GPIO_HIGH

/* PRIVATE MACROS ------------------------------------------------------------*/

/* EXTERN VARIABLES ----------------------------------------------------------*/

/* PRIVATE VARIABLES ---------------------------------------------------------*/

static radioConfig_t    radioConfig;
static rxPkt_t          rxPktCopy;
// INS_SFLUSHTX(1 Byte) + INS_STXON(1 Byte) + 2*INS_SFLUSHRX(1 Byte) + INS_TXBUF(1 Byte) + PHR(1 Byte)

static regVal_t         regVal[] = {
    CC2520_CCACTRL0,    0xF8,               // Table21 (Required Updates)
    CC2520_MDMCTRL0,    0x85,               // Table21 (Required Updates)
    CC2520_MDMCTRL1,    0x14,               // Table21 (Required Updates)
    CC2520_RXCTRL,      0x3F,               // Table21 (Required Updates)
    CC2520_FSCTRL,      0x5A,               // Table21 (Required Updates)
    CC2520_FSCAL1,      0x2B,               // Table21 (Required Updates)
    CC2520_ADCTEST0,    0x10,               // Table21 (Required Updates)
    CC2520_ADCTEST1,    0x0E,               // Table21 (Required Updates)
    CC2520_ADCTEST2,    0x03,               // Table21 (Required Updates)

#ifdef INCLUDE_PA
    CC2520_TXPOWER,     0x32,               // Max TX output power
    CC2520_AGCCTRL1,    0x16,
    CC2520_TXCTRL,      0xC1,
#else
    CC2520_TXPOWER,     0xF7,               // Max TX output power
    CC2520_AGCCTRL1,    0x11,               // Table21 (Required Updates)
#endif

#ifdef INCLUDE_PA

    CC2520_GPIOCTRL3,   0x47,               // CC2590 HGM low gain mode for startup
    CC2520_GPIOCTRL4,   0x46,               // EN set to on for rx
    CC2520_GPIOCTRL5,   CC2590_HGM,         // GPIO5 to HGM pin
    CC2520_GPIOPOLARITY,0x27,               // Invert GPIO4 and GPIO5

#endif /* INCLUDE_PA */

    CC2520_FRMCTRL0,    0x40,               // APPEND_DATA_MODE=0, AUTOCRC=1
    CC2520_EXTCLOCK,    0x00,

    CC2520_EXCFLAG0,    0x00,               // Clear any Exceptions
    CC2520_EXCFLAG1,    0x00,               // Clear any Exceptions
    CC2520_EXCFLAG2,    0x00,               // Clear any Exceptions

    // outputs RX exception in channel A to GPIO1 (TP36)
    CC2520_GPIOCTRL1, 0x21,
    CC2520_EXCMASKA0, CC2520_EXC0_RX_UNDERFLOW_BM |  CC2520_EXC0_RX_OVERFLOW_BM,
    CC2520_EXCMASKA1, 0x00,
    CC2520_EXCMASKA2, 0x00,     // for debugging: CC2520_EXC2_MEMADDR_ERROR_BM | CC2520_EXC2_USAGE_ERROR_BM | CC2520_EXC2_OPERAND_ERROR_BM | CC2520_EXC2_SPI_ERROR_BM

    CC2520_FRMFILT1,  0x10,     //only data frames are accepted
    // Terminate array
    0, 0x00
};

/* PUBLIC VARIABLES ----------------------------------------------------------*/

/* --- FOR ISR-USE ONLY --- */
volatile uint8_t spiTxRxByteCount = 0;
volatile uint8_t spiTxRxByteState = 0xFF;   // RF_SPI_INIT_STATE;

uint8_t*            pSpiTxBuf;
uint8_t*            pSpiRxBuf;
rxPkt_t             rxPkt;
uint8_t             scratchBuf[FRAME_LENGTH_MAX + 5];

OS_MutexID flagSPIIODone;
volatile uint16_t rxDoneType = 1;        // 0: successful, 1: failed
volatile uint16_t rxFIFOError = 0;       // 0: no error, 1: with error

OS_FlagID flagSPIMachineDone = 0xFF;

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/


void                     Enable_Osc(void);
void                    Load_2520_Defaults(void);



/* PRIVATE FUNCTIONS ---------------------------------------------------------*/

/*******************************************************************************
* Description : Tasks access SPI exclusively
*               then Grab
* Input       : -
* Return      : -
*******************************************************************************/
void RadioWaitGrabSPI(void) {
    diagnostic.enterFlagSPIIODone = CoEnterMutexSection(flagSPIIODone);
}

/*******************************************************************************
* Description : Release SPI for other tasks
* Input       : -
* Return      : -
*******************************************************************************/
void RadioReleaseSPI(void) {
    diagnostic.leaveFlagSPIIODone = CoLeaveMutexSection(flagSPIIODone);
}

/* PUBLIC FUNCTIONS ----------------------------------------------------------*/

/*******************************************************************************
* Description : [API] Initialize and Configure Radio with default settings
* Input       : -
* Return      : -
*******************************************************************************/
void RadioInit(uint16_t panId, uint16_t shortAddr, uint8_t chan) {


            HwGPOHigh(GPO_RF_EN);  // turn on power
            HwWait(10);
            HwGPOHigh(GPO_2520_RST); // relese the reset
            HwWait(10);
            HwGPOHigh(GPO_TK_RAD_HGM); // turn on the HGM

    HwWait(10);
    assert( Send_SPI_byte(CC2520_INS_SNOP) & CC2520_STB_XOSC_STABLE_BM );

    flagSPIIODone = CoCreateMutex();
    flagSPIMachineDone = CoCreateFlag(1, 0); // auto-reset, flag clear

       Enable_Osc()     ;
/* Load defalt Radio enable values   */
      Load_2520_Defaults();


    // Initialize radioConfig struct
    radioConfig.myAddr = shortAddr;
    radioConfig.myPanId = panId;
    radioConfig.channel = chan;

    // Set channel
    TK_BK_REGWR8(CC2520_FREQCTRL, MIN_CHANNEL + (((radioConfig.channel)-MIN_CHANNEL) * CHANNEL_SPACING));

//    TK_BK_REGWR8(CC2520_TXPOWER,TxAmpValues[config.TxPower]);   // Max TX output power


    // Write the short address and the PAN ID to the CC2520 RAM
    TK_BK_MEMWR16(CC2520_RAM_SHORTADDR, radioConfig.myAddr);
    TK_BK_MEMWR16(CC2520_RAM_PANID, radioConfig.myPanId);

//

    Send_SPI_byte(CC2520_INS_SFLUSHRX);     // flush the rx fifo to clear any errors
    Send_SPI_byte(CC2520_INS_SFLUSHTX);     // flush the tx FIFO

    // Set up 2520 GPIO 0 (TP35) as RX_FRM_DONE Output
    TK_BK_REGWR8(CC2520_GPIOCTRL0, CC2520_GPIO_RX_FRM_DONE);

    // TODO!!! 2520 GPIO 2 (TP37) available. Maybe serves as a SFLUSHRX strobe, SFD or ...
    // TK_BK_REGWR8(CC2520_GPIOCTRL2, 0x80 | CC2520_STR_SFLUSHRX);
//---------------------------------------------------------------------
#ifndef TK_V2_hdwr

    // Set up 2520 GPIO 5 (TP40) as start of frame flag
 //  TK_BK_REGWR8(CC2520_GPIOCTRL5, CC2520_GPIO_SFD);
    TK_BK_REGWR8(CC2520_GPIOCTRL5,CC2520_GPIO_FIFO);

#endif

    // now enable RADIO_GPIO0 and RADIO_GPIO1 EXTIs
    HwRadioEXTIInit();

    // And...Turn on Rx ,mode as default
    Send_SPI_byte(CC2520_INS_SRXON);        // enable RX

 }

/*******************************************************************************
* Description : SOXON  turns on internal oscilator for analog potion of chip
* Input       :
* Return      :
*******************************************************************************/
void Enable_Osc(void){
    Send_SPI_2byte(CC2520_INS_SXOSCON, CC2520_INS_SNOP); // turn on the oscillator


    HwWait(10);
    assert( Send_SPI_byte(CC2520_INS_SNOP) & CC2520_STB_XOSC_STABLE_BM );

}
/*******************************************************************************
* Description : Loads 2520 registers
* Input       :
* Return      :
*******************************************************************************/
void Load_2520_Defaults(void){
    regVal_t *p = regVal;
    while (p->reg != 0) {
        TK_BK_MEMWR8(p->reg, p->val);
        p++;
    }

}

// handle rx overflow & underflow errors
void ProcessRXError() {
      func = __func__;

    assert(spiTxRxByteState == RF_SPI_INIT_STATE);      // State machine must be stopped first
    while (CoAcceptSem(semRFRxFrames) != E_SEM_EMPTY);        // remove all pending singnals

    rxErrors++;
    Send_SPI_byte(CC2520_INS_SFLUSHRX);     // double flush ...
    Send_SPI_byte(CC2520_INS_SFLUSHRX);     // ... [CC2520 Bug#1]

    // State machine must be stopped and SPI must be grabbed to run these
    TK_BK_REGWR8(CC2520_EXCFLAG0, 0);
    TK_BK_REGWR8(CC2520_EXCFLAG1, 0);
    TK_BK_REGWR8(CC2520_EXCFLAG2, 0);

    HwWait(1);      // turn around time
}

uint8_t rxCount;
uint8_t rxFirst;
uint8_t rxPayload;
uint8_t exc0;
uint8_t exc1;
uint8_t exc2;
uint16_t rxLine;

realTime rxWaitTime;
realTime rxInTime;

/*******************************************************************************
* Description : [API] wait and gets a new Received Packet
*******************************************************************************/
rxPkt_t* RadioRxPkt() {
    func = __func__;
    rxLine = line = __LINE__;
    if (rxFIFOError) {          // TODO!!! Question: when SPI downloading a frame a RX-FIFO takes place, will the behaviour get affected???  Now assuming No
        rxFIFOError = 0;
        ProcessRXError();
        return NULL;
    }

    rxWaitTime.sec = sec;
    rxWaitTime.uSec = TIM1->CNT;
    rxLine = line = __LINE__;
    diagnostic.pendSemRFRxFrames = CoPendSem(semRFRxFrames, 50);
    rxLine = line = __LINE__;
    if (diagnostic.pendSemRFRxFrames != E_OK) {   // yield to other tasks (like commandline interface)
        HwWait(1);
        return NULL;
    }
    rxLine = line = __LINE__;
    rxInTime.sec = sec;
    rxInTime.uSec = TIM1->CNT;

    /* kick off ISR-driven SPI download */
    exc0 = TK_BK_REGRD8(CC2520_EXCFLAG0);
    exc1 = TK_BK_REGRD8(CC2520_EXCFLAG1);
    exc2 = TK_BK_REGRD8(CC2520_EXCFLAG2);

    rxCount = TK_BK_REGRD8(CC2520_RXFIFOCNT);
    rxFirst = TK_BK_REGRD8(CC2520_RXFIRST);

    scratchBuf[0] = CC2520_INS_BCLR;    // reset RX_FRM_DONE signal/exception
    scratchBuf[1] = (CC2520_EXCFLAG1 << 3) | (CC2520_EXC_RX_FRM_DONE - 8);
    scratchBuf[2] = CC2520_INS_SNOP;    // pad to 16-bit word align
    scratchBuf[3] = CC2520_INS_RXBUF;
    scratchBuf[4] = CC2520_INS_SNOP;

    spiTxRxByteCount = 5;  // go to RXBUF_Part2
    assert(spiTxRxByteState == RF_SPI_INIT_STATE);
    spiTxRxByteState = RF_SPI_RX1_UPLOADCMD_STATE;

    pSpiTxBuf = scratchBuf;
    pSpiRxBuf = (uint8_t*)&rxPkt;

    rxDoneType = 1;     // Assuming not successful State Machine will change it to 0 if successful
    diagnostic.clrSPIDone = CoClearFlag(flagSPIMachineDone);        // can be removed
    // roll state machine
    HwSPISSAssert(SPI_RADIO);
    SPI_I2S_ITConfig(SPI_RADIO_SPI, SPI_I2S_IT_RXNE, ENABLE);
    SPI_I2S_SendData(SPI_RADIO_SPI, *pSpiTxBuf++);

    rxLine = line = __LINE__;
    diagnostic.waitSPIDone = CoWaitForSingleFlag(flagSPIMachineDone, 0);
    func = __func__;
    rxLine = line = __LINE__;
    rxPayload = rxPkt.payload[0];
    if (rxDoneType == 1) {      //overflow or underflow, which can stop SPI actions
        TRACE("RX error\n\r");
        return NULL;
    }
    rxLine = line = __LINE__;

    rxPkt.payloadSize = rxPkt.frameLength - MHR_SIZE - MFR_SIZE;
    rxPkt.rssi = rxPkt.payload[rxPkt.payloadSize];

    // TRACE("*RSSI*: %x \n\r",rxPkt.rssi);
    // check if received CRC good
    rxPkt.lqi = rxPkt.payload[rxPkt.payloadSize + 1];
    if (rxPkt.lqi & 0x80) {
        rxPkt.lqi &= 0x7F;
    } else {
      if (config.flags & FLAG_TRACE_CRC) {
        TRACE("Bad CRC. Abort rxCount=%d\n\r", rxCount);
      }
      rxLine = line = __LINE__;
      return NULL;    // bad CRC => reject packet
    }
    rxLine = line = __LINE__;

    // TODO!!! post to receive task; or should not happen if the first packet is not finished processing yet
    // give user a copy of receive packet (avoid clobbering by next ISR-received pkt)
    memcpy((void*)&rxPktCopy, (void*)&rxPkt, sizeof(rxPkt_t));
    rxLine = line = __LINE__;
    return &rxPktCopy;
}

/*******************************************************************************
* Description : [API] Sets new PAN ID and Short Source Address
* Input       :
* Return      : -
*******************************************************************************/
void RadioSetPanIdShortAddr(uint16_t panId, uint16_t shortAddr) {
    radioConfig.myPanId = panId;
    radioConfig.myAddr = shortAddr;
    // Write the short address and the PAN ID to the CC2520 RAM
    SAVE_POINT
    RadioWaitGrabSPI();
    SAVE_POINT
    TK_BK_MEMWR16(CC2520_RAM_PANID, panId);
    TK_BK_MEMWR16(CC2520_RAM_SHORTADDR, shortAddr);
    RadioReleaseSPI();
}

/*******************************************************************************
* Description : [API] Sets new RF Channel
* Input       : chan => {11 - 26}
* Return      : -
*******************************************************************************/
void RadioSetRFChan(uint8_t chan) {
uint8_t status;

  radioConfig.channel = chan;
  SAVE_POINT
  RadioWaitGrabSPI();
  SAVE_POINT
    status = Send_SPI_byte(CC2520_INS_SNOP);
    Send_SPI_byte(CC2520_INS_SRFOFF);
    TK_BK_REGWR8(CC2520_FREQCTRL, MIN_CHANNEL + ((chan - MIN_CHANNEL) * CHANNEL_SPACING) );
    if(status & 0x01){                // RX active
    Send_SPI_byte(CC2520_INS_SRXON);
    Send_SPI_byte(CC2520_INS_SFLUSHRX);     // double flush ...
    Send_SPI_byte(CC2520_INS_SFLUSHRX);     // ... [CC2520 Bug#1]
    }else if(status &0x02){               // TX active
    Send_SPI_byte(CC2520_INS_STXON);
        Send_SPI_byte(CC2520_INS_SFLUSHRX);     // double flush ...
    Send_SPI_byte(CC2520_INS_SFLUSHRX);     // ... [CC2520 Bug#1]
    }
    status = Send_SPI_byte(CC2520_INS_SNOP);
    RadioReleaseSPI();
}
/*******************************************************************************
* Description : [API] Sets new RF Transmit Level
* Input       :
* Return      : -
*******************************************************************************/
void RadioSetRFLevel(uint8_t Lev) {
volatile uint8_t status, output;
  if(Lev <= (sizeof TxAmpValues)){
    radioConfig.level = Lev;
    SAVE_POINT
    RadioWaitGrabSPI();
    SAVE_POINT
    status = Send_SPI_byte(CC2520_INS_SNOP);

    Send_SPI_byte(CC2520_INS_SRFOFF);

 //       status = Send_SPI_byte(CC2520_INS_SNOP);

    TK_BK_REGWR8(CC2520_TXPOWER,TxAmpValues[Lev]);
    if(status & 0x01){                // RX active
      Send_SPI_byte(CC2520_INS_SRXON);
      Send_SPI_byte(CC2520_INS_SFLUSHRX);     // double flush ...
      Send_SPI_byte(CC2520_INS_SFLUSHRX);     // ... [CC2520 Bug#1]
    }
    else if(status &0x02){               // TX active
      Send_SPI_byte(CC2520_INS_STXON);
      Send_SPI_byte(CC2520_INS_SFLUSHRX);     // double flush ...
      Send_SPI_byte(CC2520_INS_SFLUSHRX);     // ... [CC2520 Bug#1]
    }
    output = Send_SPI_byte(CC2520_INS_SNOP);
    RadioReleaseSPI();
  }
}

/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
