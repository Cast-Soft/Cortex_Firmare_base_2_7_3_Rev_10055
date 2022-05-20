/******************** (C) COPYRIGHT 2011 NaturalPoint, Inc. ********************
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
#ifdef COOS
#include "CoOS.h"
#endif

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
const uint8_t RFChanValues[16]={11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26};
uint8_t TxRfLevel = 0xF7;


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
//#define CC2590_HGM                      CC2520_GPIO_HIGH

/* PRIVATE MACROS ------------------------------------------------------------*/

/* EXTERN VARIABLES ----------------------------------------------------------*/

/* PRIVATE VARIABLES ---------------------------------------------------------*/

static radioConfig_t    radioConfig;
static rxPkt_t          rxPktCopy;
// INS_SFLUSHTX(1 Byte) + INS_STXON(1 Byte) + 2*INS_SFLUSHRX(1 Byte) + INS_TXBUF(1 Byte) + PHR(1 Byte)
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
  #ifdef OLIMEX
    CC2520_GPIOCTRL3,   CC2590_HGM,         // CC2590 HGM
    CC2520_GPIOCTRL4,   0x46,               // EN set to lna_pd[1] inverted
    CC2520_GPIOCTRL5,   0x47,               // PAEN set to pa_pd inverted
    CC2520_GPIOPOLARITY,0x0F,               // Invert GPIO4 and GPIO5
  #else                                     // if Timekeeper and Beacon
    CC2520_GPIOCTRL3,   0x47,               // CC2590 HGM low gain mode for startup
    CC2520_GPIOCTRL4,   0x46,               // EN set to on for rx
    CC2520_GPIOCTRL5,   CC2590_HGM,         // GPIO5 to HGM pin
    CC2520_GPIOPOLARITY,0x27,               // Invert GPIO4 and GPIO5
  #endif /* OLIMEX */
#endif /* INCLUDE_PA */

    CC2520_FRMCTRL0,    0x40,               // APPEND_DATA_MODE=0, AUTOCRC=1
    CC2520_EXTCLOCK,    0x00,

    CC2520_EXCFLAG0,    0x00,               // Clear any Exceptions
    CC2520_EXCFLAG1,    0x00,               // Clear any Exceptions
    CC2520_EXCFLAG2,    0x00,               // Clear any Exceptions

    // Terminate array
    0, 0x00
};

/* PUBLIC VARIABLES ----------------------------------------------------------*/

/* --- FOR ISR-USE ONLY --- */

volatile uint16_t   spiTxRxByteCount = 0;
uint8_t*            pSpiTxBuf;
uint8_t*            pSpiRxBuf;
rxPkt_t             rxPkt;
uint8_t             scratchBuf[FRAME_LENGTH_MAX + 5];
#ifdef COOS
OS_FlagID           flagRadioTxDone = 0xFF;
OS_FlagID           flagRadioRxFrame = 0xFF;
#else
volatile uint16_t   flagRadioTxDone = 1;
volatile uint16_t   flagRadioRxFrame = 0;
#endif

/* --- */

#ifdef COOS
OS_FlagID           flagRadioTxDoneUser = 0xFF;
#else
volatile uint16_t   flagRadioTxDoneUser = 0;
#endif

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/

//static void             RadioCheckException(void);
static void             RadioErrorRecover(void);
static uint8_t          RadioBuildHdr(txPktHdr_t *pktHdr, uint16_t destAddr, uint8_t beacon, uint8_t payloadLength);
static void             RadioWaitGrabSPI(void);
static void             RadioReleaseSPI(void);
void                     Enable_Osc(void);
void                    Load_2520_Defaults(void);  
void                    RadioTxTest(void);
    

/* PRIVATE FUNCTIONS ---------------------------------------------------------*/

/*******************************************************************************
* Description : Check for Radio Exceptions
* Input       : -
* Return      : -
*******************************************************************************/
/*static void RadioCheckException(void) {
    static uint8_t result[3];

    RadioWaitGrabSPI();
    if (result[2] = TK_BK_REGRD8(CC2520_EXCFLAG2) )
        printf("\n\rEXC2: %02X", result[2]);
    if (result[1] = TK_BK_REGRD8(CC2520_EXCFLAG1) & ~(CC2520_EXC1_RX_FRM_DONE_BM | CC2520_EXC1_RX_FRM_ACCEPTED_BM | CC2520_EXC1_SRC_MATCH_DONE_BM | CC2520_EXC1_FIFOP_BM | CC2520_EXC1_SFD_BM) )
        printf("\n\rEXC1: %02X", result[1]);
    if (result[0] = TK_BK_REGRD8(CC2520_EXCFLAG0) & ~(CC2520_EXC0_TX_FRM_DONE_BM) )
        printf("\n\rEXC0: %02X", result[0]);
    while (result[2] | result[1] | result[0]) {
        assert(0);
    };
    RadioReleaseSPI();
}*/

/*******************************************************************************
* Description : Recover from SERIOUS Radio Operational Errors
* Input       : -
* Return      : -
*******************************************************************************/
void RadioErrorRecover(void) {
#ifdef COOS
    CoSchedLock();
#endif
    printf("\n\r***RADIO ERROR*** EXC0:%2X EXC1:%2X EXC2:%2X\n\r",
           TK_BK_REGRD8(CC2520_EXCFLAG0),
           TK_BK_REGRD8(CC2520_EXCFLAG1),
           TK_BK_REGRD8(CC2520_EXCFLAG2)); // [[DEBUG]]

    // abort any ongoing SPI transfers
    __disable_interrupt();
    SPI_I2S_ITConfig(SPI_RADIO_SPI, SPI_I2S_IT_RXNE, DISABLE);
    HwSPISSDeAssert(SPI_RADIO);
    spiTxRxByteCount = 0;
    __enable_interrupt();

    // put radio back into known state
    Send_SPI_byte(CC2520_INS_SRFOFF);
    Send_SPI_byte(CC2520_INS_SFLUSHTX);
    Send_SPI_byte(CC2520_INS_SRXON);
    Send_SPI_byte(CC2520_INS_SFLUSHRX);     // double flush ...
    Send_SPI_byte(CC2520_INS_SFLUSHRX);     // ... [CC2520 Bug#1]
    TK_BK_REGWR8(CC2520_EXCFLAG0, 0);
    TK_BK_REGWR8(CC2520_EXCFLAG1, 0);
    TK_BK_REGWR8(CC2520_EXCFLAG2, 0);

    // perform internal housekeeping
#ifdef COOS
    CoSetFlag(flagRadioTxDone);
    CoClearFlag(flagRadioTxDoneUser);
    CoClearFlag(flagRadioRxFrame);
#else
    flagRadioTxDone     = 1; // flag set
    flagRadioTxDoneUser = 0; // flag clear
    flagRadioRxFrame    = 0; // flag clear
#endif

#ifdef COOS
    CoSchedUnlock();
#endif
}

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
* Description : Wait if SPI is in used by ISR (e.g. uploading/downloading FIFOs),
*               then Grab
* Input       : -
* Return      : -
*******************************************************************************/
static void RadioWaitGrabSPI(void) {
    while (spiTxRxByteCount != 0x0100) {
        // wait if SPI is busy (e.g. downloading / uploading)
        while (spiTxRxByteCount);
        /* try and grab SPI resource */
        __disable_interrupt();
        if (spiTxRxByteCount) {
            __enable_interrupt(); // too late - try again later
            continue;
        }
        spiTxRxByteCount = 0x0100;
        __enable_interrupt();
    }
}

/*******************************************************************************
* Description : Release SPI to be used again by ISR
* Input       : -
* Return      : -
*******************************************************************************/
static void RadioReleaseSPI(void) {
    __disable_interrupt();
    spiTxRxByteCount &= ~0x0100;
    __enable_interrupt();
}

/* PUBLIC FUNCTIONS ----------------------------------------------------------*/

/*******************************************************************************
* Description : [API] Initialize and Configure Radio with default settings
* Input       : -
* Return      : -
*******************************************************************************/
void RadioInit(uint16_t panId, uint16_t shortAddr, uint8_t chan) {
#ifdef OLIMEX
    GPIO_WriteBit(GPIOD, GPIO_Pin_3, Bit_RESET);       // reset radio...
    HwWait(10);
    GPIO_WriteBit(GPIOD, GPIO_Pin_3, Bit_SET);
#else
#ifdef  TK_V2_hdwr   
    Send_SPI_2byte(CC2520_INS_SRES, CC2520_INS_SRES);
    Send_SPI_2byte(CC2520_INS_SXOSCON, CC2520_INS_SNOP);
#endif
#ifdef TK_V3_hdwr
            HwGPOHigh(GPO_RF_EN);  // turn on power
            HwWait(10);
            HwGPOHigh(GPO_2520_RST); // relese the reset
            HwWait(10);
            HwGPOHigh(GPO_TK_RAD_HGM);
#endif    
    
    
#endif /* OLIMEX */

    HwWait(10);
    assert( Send_SPI_byte(CC2520_INS_SNOP) & CC2520_STB_XOSC_STABLE_BM );

#ifdef COOS
    flagRadioTxDone     = CoCreateFlag(1, 1); // auto-reset, flag set
    flagRadioTxDoneUser = CoCreateFlag(1, 0); // auto-reset, flag clear
    flagRadioRxFrame    = CoCreateFlag(1, 0); // auto-reset, flag clear
#else
    flagRadioTxDone     = 1; // flag set
    flagRadioTxDoneUser = 0; // flag clear
    flagRadioRxFrame    = 0; // flag clear
#endif

       Enable_Osc()     ;
/* Load defalt Radio enable values   */
      Load_2520_Defaults();       

     
    // Initialize radioConfig struct
    radioConfig.myAddr = shortAddr;
    radioConfig.myPanId = panId;
    radioConfig.channel = chan;

    // Set channel
    TK_BK_REGWR8(CC2520_FREQCTRL, MIN_CHANNEL + (((RFChanValues[radioConfig.channel])-MIN_CHANNEL) * CHANNEL_SPACING));

    TK_BK_REGWR8(CC2520_TXPOWER,TxAmpValues[config.TxPower]);   // Max TX output power

    
    // Write the short address and the PAN ID to the CC2520 RAM
    TK_BK_MEMWR16(CC2520_RAM_SHORTADDR, radioConfig.myAddr);
    TK_BK_MEMWR16(CC2520_RAM_PANID, radioConfig.myPanId);

      
    if(config.TestMode) {
      
      RadioTxTest();
    }else{ 
   

    Send_SPI_byte(CC2520_INS_SFLUSHRX);     // flush the rx fifo to clear any errors
    Send_SPI_byte(CC2520_INS_SFLUSHTX);     // flush the tx FIFO

    // Set up 2520 GPIO 0 (TP35) as RX_FRM_DONE Output
    TK_BK_REGWR8(CC2520_GPIOCTRL0, CC2520_GPIO_RX_FRM_DONE);

    // Set up 2520 GPIO 1 (TP36) as TX_FRM_DONE Output
    TK_BK_REGWR8(CC2520_GPIOCTRL1, CC2520_GPIO_TX_FRM_DONE);

    // Set up 2520 GPIO 2 (TP37) as STXON Command Strobe Input
    TK_BK_REGWR8(CC2520_GPIOCTRL2, 0x80 | CC2520_STR_STXON);

    // now enable RADIO_GPIO0 and RADIO_GPIO1 EXTIs
    HwRadioEXTIInit();

    // And...Turn on Rx ,mode as default
    Send_SPI_byte(CC2520_INS_SRXON);        // enable RX
    }
 }

/***********************************************************************/
// Load Tx packet to radio for testing
//
//
/***********************************************************************/
void RadioTxTest(void){
uint8_t status , i;  
uint8_t Len_Pkt;

        Send_SPI_byte(CC2520_INS_SFLUSHTX);     // flush the tx FIFO
        // Set up 2520 GPIO 1 (TP36) as TX_FRM_DONE Output
        TK_BK_REGWR8(CC2520_GPIOCTRL1, CC2520_GPIO_TX_FRM_DONE);
    
        TK_BK_REGWR8(CC2520_FRMCTRL0,0x42);      // APPEND_DATA_MODE=0, AUTOCRC=1 TX looping

     
        TK_BK_REGWR8(CC2520_EXCFLAG0, 0);
        TK_BK_REGWR8(CC2520_EXCFLAG1, 0);
        TK_BK_REGWR8(CC2520_EXCFLAG2, 0);
    
    Len_Pkt= RadioBuildHdr((txPktHdr_t*)txBuf, 0xffff , 0, 100);
      
      for(i=0; i<Len_Pkt; i++){
        Send_SPI_2byte(CC2520_INS_TXBUF,txBuf[i]);
      }
      
      
//        Send_SPI_2byte(CC2520_INS_TXBUF,0x60); // send length of 100 bytes
    
//        status = TK_BK_MEMRDR8(0x100);
//        printf("Tx buffer:%x \n\r",status);
    
        Send_SPI_byte(CC2520_INS_SNOP);
        Send_SPI_byte(CC2520_INS_SNOP);
    
    
       Send_SPI_byte(CC2520_INS_STXON);     // turn on the radio
          HwWait(10);


        
        status=Send_SPI_byte(CC2520_INS_SNOP);
        printf("RadioStatus:%x \n\r",status);
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

/*******************************************************************************
* Description : [API] Gets a new Received Packet
* Input       : wait => 0 = Don't Wait; 1 = Wait
* Return      : Pointer to received packet, otherwise NULL
*******************************************************************************/
rxPkt_t* RadioRxPkt(uint16_t wait) {
    if (wait) {
#ifdef COOS
        CoWaitForSingleFlag(flagRadioRxFrame, 0);
    } else if (CoAcceptSingleFlag(flagRadioRxFrame) != E_OK) {
        return NULL;    // no packet available
    }
#else
        while (!(flagRadioRxFrame));
    } else if (!flagRadioRxFrame) {
        return NULL;    // no packet available
    }
    flagRadioRxFrame = 0;
#endif

    // check for RX_UNDERFLOW or RX_OVERFLOW
    RadioWaitGrabSPI();
    if ( TK_BK_REGRD8(CC2520_EXCFLAG0) &
         (CC2520_EXC0_RX_UNDERFLOW_BM | CC2520_EXC0_RX_OVERFLOW_BM) ) {
        // could mean Rx-FIFO packet-framing is no longer valid
        RadioErrorRecover();
        return NULL;
    }
    RadioReleaseSPI();
    //RadioCheckException(); // [[DEBUG]]

    rxPkt.payloadSize = rxPkt.frameLength - MHR_SIZE - MFR_SIZE;
    rxPkt.rssi = rxPkt.payload[rxPkt.payloadSize];
    
 //   printf("*RSSI*: %x \n\r",rxPkt.rssi);
    
    
    // check if received CRC good
    rxPkt.lqi = rxPkt.payload[rxPkt.payloadSize + 1];
    if (rxPkt.lqi & 0x80) {
        rxPkt.lqi &= 0x7F;
    } else {
        return NULL;    // bad CRC => reject packet
    }

    // give user a copy of receive packet (avoid clobbering by next ISR-received pkt)
    memcpy((void*)&rxPktCopy, (void*)&rxPkt, sizeof(rxPkt_t));
    return &rxPktCopy;
}

/*******************************************************************************
* Description : [API] Transmit a Packet Immediately
* Input       : trigger => 0 = on GPIO2 rising-edge; 1 = right away
* Return      : -
*******************************************************************************/
void RadioTxPkt(uint16_t dstAddr, uint8_t beacon, uint8_t payloadSize, uint8_t *payload, uint8_t trigger) {
    uint8_t hdrBytes;

#ifdef COOS
   static uint16_t entrance_count = 0;
    assert(!entrance_count++); // [[DEBUG]]
#endif

    // can't initiate transmit if SPI is busy
    RadioWaitGrabSPI();

    // check for TX_UNDERFLOW or TX_OVERFLOW
    if ( TK_BK_REGRD8(CC2520_EXCFLAG0) &
         (CC2520_EXC0_TX_UNDERFLOW_BM | CC2520_EXC0_TX_OVERFLOW_BM) ) {
        // could mean last packet was aborted, hence flagRadioTxDone will never get set
        RadioErrorRecover();
    }
    //RadioCheckException(); // [[DEBUG]]

    // flush Tx-FIFO to be safe
    txBuf[0] = CC2520_INS_SFLUSHTX;
    // insert CC2520 STXON, or not
    if (trigger)
        txBuf[1] = CC2520_INS_STXON;    // 192us Tx Turnaround Time + 160us Preamble+SFD Tx Time
    else
        txBuf[1] = CC2520_INS_SNOP;
    // double-flush Rx-FIFO [CC2520 Bug#1]
    txBuf[2] = CC2520_INS_SFLUSHRX;
    txBuf[3] = CC2520_INS_SFLUSHRX;
    // insert TXBUF instruction to write data to Tx-FIFO
    txBuf[4] = CC2520_INS_TXBUF;
    // create the packet header (PHR+MHR) in the transmit buffer
    hdrBytes = RadioBuildHdr((txPktHdr_t*)(txBuf + 5), dstAddr, beacon, payloadSize);
    // copy over the payload (to avoid clobbering by user upon function exit)
    memcpy((void*)(txBuf + hdrBytes + 5), (void*)payload, payloadSize);

    // wait for previous TX to complete if necessary (can't clobber Tx-FIFO)
#ifdef COOS
    if (CoWaitForSingleFlag(flagRadioTxDone, 5) != E_OK) {
        RadioErrorRecover();
    }
#else
    HwTimerSet(5);
    while (!flagRadioTxDone && !HwTimerExpired());
    if (flagRadioTxDone) {
        flagRadioTxDone = 0;
    } else {
        RadioErrorRecover();
    }
#endif

    // prev TX is done, so reset TX_FRM_DONE signal/exception
    TK_BK_REGWR8(CC2520_EXCFLAG0, ~CC2520_EXC0_TX_FRM_DONE_BM);

    // kick off ISR-driven SPI upload
    pSpiTxBuf = txBuf;
    pSpiRxBuf = scratchBuf;
    spiTxRxByteCount = (spiTxRxByteCount & 0xFF00) | (5 + hdrBytes + payloadSize);
    RadioReleaseSPI();
    HwSPISSAssert(SPI_RADIO);
    SPI_I2S_ITConfig(SPI_RADIO_SPI, SPI_I2S_IT_RXNE, ENABLE);
    SPI_I2S_SendData(SPI_RADIO_SPI, *pSpiTxBuf++);

#ifdef COOS
    entrance_count--; // [[DEBUG]]
#endif
}

/*******************************************************************************
* Description : [API] Call this RIGHT AFTER asserting GPIO2 Trigger (STXON)
*               [CC2520 Bug #1] => STXON may cause Rx-FIFO corruption
*               !NOTE!: To avoid flushing a good RX packet in the Rx-FIFO, call
*                       this function only right after STXON trigger
* Input       : -
* Return      : -
*******************************************************************************/
void RadioSTXONBugFlush(void) {
    RadioWaitGrabSPI();
    Send_SPI_byte(CC2520_INS_SFLUSHRX);     // double flush ...
    Send_SPI_byte(CC2520_INS_SFLUSHRX);     // ... [CC2520 Bug#1]
    RadioReleaseSPI();
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
    }else if(status &0x02){               // TX active
    Send_SPI_byte(CC2520_INS_STXON);
        Send_SPI_byte(CC2520_INS_SFLUSHRX);     // double flush ...
    Send_SPI_byte(CC2520_INS_SFLUSHRX);     // ... [CC2520 Bug#1]
    }
       output = Send_SPI_byte(CC2520_INS_SNOP);
    RadioReleaseSPI();
  }
}

/*******************************************************************************
* Description : Temp_2520
* Input       : Sets up the 2520 to deliver internal temperature
* Return      : -
*******************************************************************************/
void Temp_2520(void){
uint16_t temp_a2d =0 ;
  
              TK_BK_REGWR8(CC2520_GPIOCTRL0,0x80); // set GPIO_0 as input                
              TK_BK_REGWR8(CC2520_GPIOCTRL1, 0x80); //set GPIO_1 as input              
//              TK_BK_REGWR8(CC2520_INS_BSET ,(CC2520_GPIOCTRL | 0x06h));
              TK_BK_REGWR8(CC2520_ATEST,0x01); // enable 
              ADC_SoftwareStartConvCmd(ADC1, ENABLE);
              while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)!= SET);
              temp_a2d = (ADC_GetConversionValue(ADC1));
              TK_BK_REGWR8(CC2520_ATEST,0x00); // Disable 
                  // Set up 2520 GPIO 0 (TP35) as RX_FRM_DONE Output
            TK_BK_REGWR8(CC2520_GPIOCTRL0, CC2520_GPIO_RX_FRM_DONE);
               // Set up 2520 GPIO 1 (TP36) as TX_FRM_DONE Output
            TK_BK_REGWR8(CC2520_GPIOCTRL1, CC2520_GPIO_TX_FRM_DONE);
}





/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
