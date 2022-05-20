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

#include "util.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_usart.h"

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

const uint8_t TxAmpValues[9]={0x03,0x2C,0x88,0x81,0x32,0x13,0xAB,0xF2,0xF7};

const uint8_t RF_SPI_INIT_STATE = 0xFF;                 // Can start an new RF_SPI operation from this state only
const uint8_t RF_SPI_UPLOAD_ONLY_STATE = 0x00;          // Upload only. Does not care download content. For upload frame to CC2520
const uint8_t RF_SPI_RX1_UPLOADCMD_STATE = 0x40;        // RX frame step 1:  upload command RXBUF
const uint8_t RF_SPI_RX2_DOWNLOAD_HEADER_STATE = 0x20;  // RX frame step 2:  download frame header
const uint8_t RF_SPI_RX3_DOWNLOAD_BODY_STATE = 0x10;    // RX frame step 3:  download frame body

// these are not verified yet
const uint8_t RF_SPI_CCA_CMD_STATE = 0x08;       // CCA upload read register FSMSTAT1 command
const uint8_t RF_SPI_STXONCCA_CMD_STATE = 0x04;

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

/* PRIVATE MACROS ------------------------------------------------------------*/

/* EXTERN VARIABLES ----------------------------------------------------------*/

/* PRIVATE VARIABLES ---------------------------------------------------------*/

static radioConfig_t    radioConfig;
static rxPkt_t          rxPktCopy;
static uint8_t          txBuf[FRAME_LENGTH_MAX - MFR_SIZE + 6];

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
    CC2520_EXCMASKA0, CC2520_EXC0_RX_UNDERFLOW_BM | CC2520_EXC0_RX_OVERFLOW_BM,
    CC2520_EXCMASKA1, 0x00,
    CC2520_EXCMASKA2, 0x00,     // for debugging: CC2520_EXC2_MEMADDR_ERROR_BM | CC2520_EXC2_USAGE_ERROR_BM | CC2520_EXC2_OPERAND_ERROR_BM | CC2520_EXC2_SPI_ERROR_BM
    
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

//OS_MutexID flagSPIIODone;
volatile uint16_t rxDoneType = 1;        // 0: successful, 1: failed
volatile uint16_t rxFIFOError = 0;       // 0: no error, 1: with error
volatile uint16_t txFIFOError = 0;       // 0: no error, 1: with rror

//OS_FlagID flagSPIMachineDone = 0xFF;

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/

//static void             RadioCheckException(void);
//static uint8_t          RadioBuildHdr(txPktHdr_t *pktHdr, uint16_t destAddr, uint8_t beacon, uint8_t payloadLength);

void RadioWaitGrabSPI(void);
void RadioReleaseSPI(void);

/* PRIVATE FUNCTIONS ---------------------------------------------------------*/

/*******************************************************************************
* Description : Builds packet header according to IEEE 802.15.4 frame format
* Input       :
* Return      : size of header (in bytes)
*******************************************************************************/

static uint8_t RadioBuildHdr(txPktHdr_t *pktHdr, uint16_t destAddr, uint8_t beacon, uint8_t payloadLength) {
    static uint8_t txSeqNumber = 0;

    // 802.15.4 Frame Format with Short Addressing
    pktHdr->frameLength = payloadLength + MHR_SIZE + MFR_SIZE;
    if (beacon) {
        pktHdr->fcf0 = LO_UINT16(BASIC_RF_FCF_BEACON);
        pktHdr->fcf1 = HI_UINT16(BASIC_RF_FCF_BEACON);
    } else {
        pktHdr->fcf0 = LO_UINT16(BASIC_RF_FCF_DATA);
        pktHdr->fcf1 = HI_UINT16(BASIC_RF_FCF_DATA);
    }
    pktHdr->seqNumber = txSeqNumber++;
    pktHdr->panId = radioConfig.myPanId;
    pktHdr->destAddr = destAddr;
    pktHdr->srcAddr = radioConfig.myAddr;

    return (MHR_SIZE + PHR_SIZE);
}

/*******************************************************************************
* Description : Tasks access SPI exclusively
*               then Grab
* Input       : -
* Return      : -
*******************************************************************************/
void RadioWaitGrabSPI(void) {
 //   CoEnterMutexSection(flagSPIIODone);
}

/*******************************************************************************
* Description : Release SPI for other tasks
* Input       : -
* Return      : -
*******************************************************************************/
void RadioReleaseSPI(void) {
 //   CoLeaveMutexSection(flagSPIIODone);
}

/*******************************************************************************
* Description : SOXON  turns on internal oscilator for analog potion of chip
* Input       :
* Return      :
*******************************************************************************/
static void Enable_Osc(void){
    Send_SPI_2byte(CC2520_INS_SXOSCON, CC2520_INS_SNOP); // turn on the oscillator

    DelayMs(10);
    assert( Send_SPI_byte(CC2520_INS_SNOP) & CC2520_STB_XOSC_STABLE_BM );
}
/*******************************************************************************
* Description : Loads 2520 registers
* Input       :
* Return      :
*******************************************************************************/
static void Load_2520_Defaults(void){
    regVal_t *p = regVal;
    while (p->reg != 0) {
        TK_BK_MEMWR8(p->reg, p->val);
        p++;
    }

}


/* PUBLIC FUNCTIONS ----------------------------------------------------------*/

/*******************************************************************************
* Description : [API] Initialize and Configure Radio with default settings
* Input       : -
* Return      : -
*******************************************************************************/
void RadioInit(uint16_t panId, uint16_t shortAddr, uint8_t chan) {

      
  HwGPOHigh(GPO_RF_EN);  // turn on power
  DelayMs(100);
  HwGPOHigh(GPO_2520_RST); // relese the reset
  DelayMs(100);
  HwGPOHigh(GPO_TK_RAD_HGM); // turn on the HGM

  DelayMs(100);
  
  assert( Send_SPI_byte(CC2520_INS_SNOP) & CC2520_STB_XOSC_STABLE_BM );

  Enable_Osc()     ;

  /* Load defalt Radio enable values   */
  Load_2520_Defaults();


  // Initialize radioConfig struct
  radioConfig.myAddr = shortAddr;
  radioConfig.myPanId = panId;
  radioConfig.channel = chan;

  // Set channel
  TK_BK_REGWR8(CC2520_FREQCTRL, MIN_CHANNEL + (((radioConfig.channel)-MIN_CHANNEL) * CHANNEL_SPACING));

  TK_BK_REGWR8(CC2520_TXPOWER,TxAmpValues[config.TxPower]);   // Max TX output power


  // Write the short address and the PAN ID to the CC2520 RAM
  TK_BK_MEMWR16(CC2520_RAM_SHORTADDR, radioConfig.myAddr);
  TK_BK_MEMWR16(CC2520_RAM_PANID, radioConfig.myPanId);

  if (config.TestMode){
    rfBeaconEn = 0;    // turn off TK RF sync transmit
  }

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


// handle rx overflow & underflow errors
void ProcessRXError() {
    assert(spiTxRxByteState == RF_SPI_INIT_STATE);      // State machine must be stopped first        
//    while (CoAcceptSem(semRFRxFrames) != E_SEM_EMPTY);        // remove all pending singnals
    
    Send_SPI_byte(CC2520_INS_SFLUSHRX);     // double flush ...
    Send_SPI_byte(CC2520_INS_SFLUSHRX);     // ... [CC2520 Bug#1]
    
    // State machine must be stopped and SPI must be grabbed to run these
    TK_BK_REGWR8(CC2520_EXCFLAG0, 0);
    TK_BK_REGWR8(CC2520_EXCFLAG1, 0);
    TK_BK_REGWR8(CC2520_EXCFLAG2, 0);

    DelayMs(10);      // turn around time        
}

/*******************************************************************************
* Description : [API] wait and gets a new Received Packet
*******************************************************************************/
rxPkt_t* RadioRxPkt() {   
    if (rxFIFOError) {          // TODO!!! Question: when SPI downloading a frame a RX-FIFO takes place, will the behaviour get affected???  Now assuming No
        rxFIFOError = 0;
        ProcessRXError();
        return NULL;
    }
  /*  StatusType statusType = CoPendSem(semRFRxFrames, 50);
    if (statusType != E_OK) {   // yield to other tasks (like commandline interface)
        DelayMs(1);     
        return NULL;
    }
*/
    /* kick off ISR-driven SPI download */
    
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
  //  CoClearFlag(flagSPIMachineDone);        // can be removed
    // roll state machine    
    HwSPISSAssert(SPI_RADIO);
    SPI_I2S_ITConfig(SPI_RADIO_SPI, SPI_I2S_IT_RXNE, ENABLE);
    SPI_I2S_SendData(SPI_RADIO_SPI, *pSpiTxBuf++);
    
 /*   CoWaitForSingleFlag(flagSPIMachineDone, 0);
    if (rxDoneType == 1) {      //overflow or underflow, which can stop SPI actions
        printf("RX error\n\r");
        return NULL;
    }
*/
    rxPkt.payloadSize = rxPkt.frameLength - MHR_SIZE - MFR_SIZE;
    rxPkt.rssi = rxPkt.payload[rxPkt.payloadSize];
    
    // printf("*RSSI*: %x \n\r",rxPkt.rssi);    
    // check if received CRC good
    rxPkt.lqi = rxPkt.payload[rxPkt.payloadSize + 1];
    if (rxPkt.lqi & 0x80) {
        rxPkt.lqi &= 0x7F;
    } else {
        printf("Bad CRC. Abort\n\r"); 
        return NULL;    // bad CRC => reject packet
    }

    // TODO!!! post to receive task; or should not happen if the first packet is not finished processing yet  
    // give user a copy of receive packet (avoid clobbering by next ISR-received pkt)
    memcpy((void*)&rxPktCopy, (void*)&rxPkt, sizeof(rxPkt_t));
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
    RadioWaitGrabSPI();
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
    RadioWaitGrabSPI();
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
    RadioWaitGrabSPI();

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

/*******************************************************************************
* Description : [API] Transmit a Packet Immediately
* Input       : trigger => 0 = on GPIO2 rising-edge; 1 = right away
* Return      : -
*******************************************************************************/
void RadioTxPkt(uint16_t dstAddr, uint8_t beacon, uint8_t payloadSize, uint8_t *payload, uint8_t trigger) {
    if (txFIFOError) {          // TODO!!! Question: when SPI uploading a frame a TX-FIFO takes place, will the behaviour get affected???  Now assuming No
        txFIFOError = 0;
        assert(spiTxRxByteState == RF_SPI_INIT_STATE);      // State machine must be stopped first 
        // Clear TX-FIFO error
        Send_SPI_byte(CC2520_INS_SFLUSHTX);
        Send_SPI_byte(CC2520_INS_SFLUSHTX);
                    
        TK_BK_REGWR8(CC2520_EXCFLAG0, 0);
        TK_BK_REGWR8(CC2520_EXCFLAG1, 0);
        TK_BK_REGWR8(CC2520_EXCFLAG2, 0); 
        
        DelayMs(1);      // turn around time  
        return;
    }    
    
    uint8_t hdrBytes;
    
    // flush Tx-FIFO to be safe
    txBuf[0] = CC2520_INS_SFLUSHTX;
    // insert CC2520 STXON, or not
    if (trigger)
    {
#ifdef CCA_EN
        txBuf[1] = CC2520_INS_STXONCCA; // 192us Tx Turnaround Time + 160us Preamble+SFD Tx Time
#else
        txBuf[1] = CC2520_INS_STXON; // [[DEBUG]]
#endif
    }
    else
    {
        txBuf[1] = CC2520_INS_SNOP;
    }
    // double-flush Rx-FIFO [CC2520 Bug#1]
    txBuf[2] = CC2520_INS_SNOP;
    txBuf[3] = CC2520_INS_SNOP;
    // insert TXBUF instruction to write data to Tx-FIFO
    txBuf[4] = CC2520_INS_TXBUF;
    // create the packet header (PHR+MHR) in the transmit buffer
    hdrBytes = RadioBuildHdr((txPktHdr_t*)(txBuf + 5), dstAddr, beacon, payloadSize);
    // copy over the payload (to avoid clobbering by user upon function exit)
    memcpy((void*)(txBuf + hdrBytes + 5), (void*)payload, payloadSize);
  
    // prev TX is done, so reset TX_FRM_DONE signal/exception
    TK_BK_REGWR8(CC2520_EXCFLAG0, ~CC2520_EXC0_TX_FRM_DONE_BM);    
    
    // kick off ISR-driven SPI upload
    pSpiTxBuf = txBuf;
    pSpiRxBuf = scratchBuf;
    
    spiTxRxByteCount = 5 + hdrBytes + payloadSize;
    assert(spiTxRxByteState == RF_SPI_INIT_STATE);
    spiTxRxByteState = RF_SPI_UPLOAD_ONLY_STATE;
    
     
    //CoClearFlag(flagSPIMachineDone);        // can be removed
    HwSPISSAssert(SPI_RADIO);
    SPI_I2S_ITConfig(SPI_RADIO_SPI, SPI_I2S_IT_RXNE, ENABLE);
    SPI_I2S_SendData(SPI_RADIO_SPI, *pSpiTxBuf++);      // send first byte. Other will be handled by SPI3 interrupt

    //CoWaitForSingleFlag(flagSPIMachineDone, 0);
     // Upload to CC2520 does not mean that the frame has been sent out into the air
#if TODO    
    if (CoWaitForSingleFlag(flagRadioTxDone, 10) != E_OK) {  // TODO!!! verify if it can happen. If not, use infinit wait
        //assert(0);      // should not happen. Happened when unluging ST-Link in debug mode
        TRACE("TX timout error\n\r");
        // put radio back into known state
        Send_SPI_byte(CC2520_INS_SFLUSHTX);
        Send_SPI_byte(CC2520_INS_SFLUSHTX);
               
        TK_BK_REGWR8(CC2520_EXCFLAG0, 0);
        TK_BK_REGWR8(CC2520_EXCFLAG1, 0);
        TK_BK_REGWR8(CC2520_EXCFLAG2, 0);
        DelayMs(1);      // turn around time        
    }   
#endif
    
}


/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
