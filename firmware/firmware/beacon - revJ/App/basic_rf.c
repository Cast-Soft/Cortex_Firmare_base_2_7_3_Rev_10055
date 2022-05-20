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
#include "imu_defs.h"

#include "stm32f10x_gpio.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "tasks.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "config.h"

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
//#define CC2590_HGM                      CC2520_GPIO_HIGH

/* PRIVATE MACROS ------------------------------------------------------------*/

/* EXTERN VARIABLES ----------------------------------------------------------*/
extern volatile uint16_t tasksWDT;
extern OS_EventID semRFRxFrames;
extern uint32_t random;

extern OS_FlagID flagIMUNewData;
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

    // outputs TX exception in channel A to 2520 GPIO 2 (TP37)
    CC2520_GPIOCTRL2, 0x21,
    CC2520_EXCMASKA0, CC2520_EXC0_TX_UNDERFLOW_BM | CC2520_EXC0_TX_OVERFLOW_BM,
    CC2520_EXCMASKA1, 0x00,
    CC2520_EXCMASKA2, 0x00,     // for debugging: CC2520_EXC2_MEMADDR_ERROR_BM | CC2520_EXC2_USAGE_ERROR_BM | CC2520_EXC2_OPERAND_ERROR_BM | CC2520_EXC2_SPI_ERROR_BM

    // outputs RX exception in channel B to GPIO5 (TP11)
    CC2520_GPIOCTRL5, 0x22,
    CC2520_EXCMASKB0, /*CC2520_EXC0_RX_UNDERFLOW_BM |*/ CC2520_EXC0_RX_OVERFLOW_BM,
    CC2520_EXCMASKB1, 0x00,
    CC2520_EXCMASKB2, 0x00,     // for debugging: CC2520_EXC2_MEMADDR_ERROR_BM | CC2520_EXC2_USAGE_ERROR_BM | CC2520_EXC2_OPERAND_ERROR_BM | CC2520_EXC2_SPI_ERROR_BM

    CC2520_FRMFILT1, 0x08,      //accepts only beacons
    // Terminate array
    0, 0x00
};

static regVal_t         regVal_IMU[] = {
    IMU_USER_CTRL,	    	0x78,				//disable I2C for SPI communication              
    IMU_PWR_MGMT_1, 		0x01,               //set auto select best clock source  
    IMU_PWR_MGMT_2,   		(0x38|0x07),		//disable both accelerometer and gyrometer        
    IMU_PWR_MGMT_2,     	(0x00|0x00),		//enable accelerometer, enable gyrometer

	IMU_REG_BANK_SEL,		BANK2,
	IMU_GYRO_CONFIG_1,		(0x00|0x29),		//gyro rate 250, gyro lpf 17hz
	IMU_GYRO_SMPLRT_DIV,	0x13,				//set gyroscope ODR to 190Hz (same val as REVH)
	
	IMU_ACCEL_CONFIG,		(0x00|0x11),		//accel rate 2g, accel lpf 136hz: 
	IMU_ACCEL_SMPLRT_DIV_1, 0x00,
	IMU_ACCEL_SMPLRT_DIV_2, 0x26, 				//set acceleration ODR to 190Hz 
	
	IMU_REG_BANK_SEL,		BANK0,
	IMU_ENABLE_1,			0x01,				//set raw data ready interrupt.
    0xFF, 0x0									// Terminate array
};
/* PUBLIC VARIABLES ----------------------------------------------------------*/

/* --- FOR ISR-USE ONLY --- */
volatile uint8_t spiTxRxByteCount = 0;
volatile uint8_t spiTxRxByteState = 0xFF;   // RF_SPI_INIT_STATE;

uint32_t            rxErrors;
uint8_t*            pSpiTxBuf;
uint8_t*            pSpiRxBuf;
rxPkt_t             rxPkt;
uint8_t             scratchBuf[FRAME_LENGTH_MAX + 5 + 20];      // 20 more bytes for sending FIFO clear command

OS_FlagID           flagRadioTxDone = 0xFF;
OS_FlagID           flagRadioTxAllow = 0xFF;
OS_FlagID           flagRadioTxDoneUser = 0xFF;
OS_EventID          semIMUAllow;
OS_MutexID 			flagSPIIODone;       // For exclusive accessing radio SPI
volatile uint16_t txDoneType = 1;        // 0: successful, 1: failed
volatile uint16_t rxDoneType = 1;        // 0: successful, 1: failed
volatile uint16_t rxFIFOError = 0;       // 0: no error, 1: with rror
volatile uint16_t txFIFOError = 0;       // 0: no error, 1: with rror

OS_FlagID flagSPIMachineDone = 0xFF;

uint8_t         radio_off = 0;
uint8_t         rxCount;
uint8_t         cc2520_flags0;
uint8_t         cc2520_flags1;
uint8_t         cc2520_flags2;
StatusType      txError;


//Developing secondary SPI buffer primarily for IMU. Don't want to edit currently working functionality of radio
volatile uint8_t spiIMUCount = 0;
volatile uint8_t spiIMUByteState = IMU_SPI_INIT_STATE;
	
volatile uint8_t*		pSpiRxBuf_IMU;
uint8_t*		pSpiTxBuf_IMU;

uint8_t	IMU_TXBuffer[]={IMU_READBIT|IMU_ACCEL_XOUT_H,	//start address to read	
								0,0,0,0,0,0,			//6 empty tx bytes to acquire 6 bytes of accel data	
								0,0,0,0,0,0};			//6 empty tx bytes to acquire 6 bytes of gyro data	
volatile uint8_t	IMU_RawData[12];
uint8_t IMUPresent;
//both radio and IMU 
//SPI3_CS_TypeDef SPI3_CS;	//indicate chip selection on the SPI line.

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/

//static void             RadioCheckException(void);
static uint8_t          RadioBuildHdr(txPktHdr_t *pktHdr, uint16_t destAddr, uint8_t beacon, uint8_t payloadLength);

void RadioWaitGrabSPI();
void RadioReleaseSPI(void);
void Enable_Osc(void);
void Load_2520_Defaults(void);
void RxTx_Setup(void);
void RadioTxTest(void);

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
* Input       : -
* Return      : -
*******************************************************************************/
void RadioIMU_WaitGrabSPI() {
  CoEnterMutexSection(flagSPIIODone);
}

/*******************************************************************************
* Description : Release SPI for other tasks
* Input       : -
* Return      : -
*******************************************************************************/
void RadioIMU_ReleaseSPI(void) {
    CoLeaveMutexSection(flagSPIIODone);
}

/*******************************************************************************
* Description : [API] Initialize and Configure Radio with default settings
* Input       : -
* Return      : -
*******************************************************************************/
void RadioInit(uint16_t panId, uint16_t shortAddr, uint8_t chan, uint8_t test) {
#ifdef OLIMEX
    GPIO_WriteBit(GPIOD, GPIO_Pin_3, Bit_RESET);       // reset radio...
    HwWait(10);
    GPIO_WriteBit(GPIOD, GPIO_Pin_3, Bit_SET);
#endif /* OLIMEX */
	HwGPOHigh(GPO_RF_EN);
	HwWait(10);
	HwGPOLow(GPO_2520_RST);
	HwWait(10);
	HwGPOHigh(GPO_2520_RST);

    HwWait(10);  // wait a bit
    // see if the SPI bus comes ready
    assert( Send_SPI_byte(CC2520_INS_SNOP) & CC2520_STB_XOSC_STABLE_BM );

    flagSPIIODone     = CoCreateMutex();
    flagSPIMachineDone = CoCreateFlag(0, 0); // manual-reset, flag clear


    flagRadioTxDone      = CoCreateFlag(1, 1); // auto-reset, flag set
    flagRadioTxAllow     = CoCreateFlag(1, 0); // auto-reset, flag clear
    semIMUAllow         = CoCreateSem(1, 1, EVENT_SORT_TYPE_FIFO);
    Send_SPI_byte(CC2520_INS_SRFOFF);

    //Enable_Osc();   //Turn on the oscillator
    /* Load defalt Radio enable values   */
    Load_2520_Defaults();  // load the defaults

    // Initialize radioConfig struct
    radioConfig.myAddr = shortAddr;
    radioConfig.myPanId = panId;
    radioConfig.channel = chan;

    // Set channel
    TK_BK_REGWR8(CC2520_FREQCTRL, MIN_CHANNEL + (((radioConfig.channel)-MIN_CHANNEL) * CHANNEL_SPACING));

    TK_BK_REGWR8(CC2520_TXPOWER,TxAmpValues[config.TxLevel]);   // Max TX output power

//      TK_BK_REGWR8(CC2520_TXPOWER,0xF7);

    // Write the short address and the PAN ID to the CC2520 RAM
    TK_BK_MEMWR16(CC2520_RAM_SHORTADDR, radioConfig.myAddr);
    TK_BK_MEMWR16(CC2520_RAM_PANID, radioConfig.myPanId);

    Send_SPI_byte(CC2520_INS_SFLUSHRX);     // flush the rx fifo to clear any errors
    Send_SPI_byte(CC2520_INS_SFLUSHRX);     // ... [CC2520 Bug#1]

    Send_SPI_byte(CC2520_INS_SFLUSHTX);     // flush the tx FIFO
    Send_SPI_byte(CC2520_INS_SFLUSHTX);     // again just in case an exception in previous SFLUSHTX

    // Set up 2520 GPIO 0 (TP35) as RX_FRM_DONE Output
    TK_BK_REGWR8(CC2520_GPIOCTRL0, CC2520_GPIO_RX_FRM_DONE);

    // Set up 2520 GPIO 1 (TP36) as TX_FRM_DONE Output
    TK_BK_REGWR8(CC2520_GPIOCTRL1, CC2520_GPIO_TX_FRM_DONE);

    // now enable RADIO_GPIO0 and RADIO_GPIO1 EXTIs
    HwRadioEXTIInit();

#if 0  //check radio - does it work?
    TK_BK_MEMWR8(0x17F, 0xC1);
    rxCount = TK_BK_MEMRD8(0x17F);
#endif

    // And...Turn on Rx ,mode as default
    Send_SPI_byte(CC2520_INS_SRXON);        // enable RX
    assert( Send_SPI_byte(CC2520_INS_SNOP) & CC2520_STB_RX_ACTIVE_BM  );
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

void RadioPrint2520Registers(uint8_t flag){
  regVal_t *p = regVal;
  uint8_t val;
  SAVE_POINT
  RadioIMU_WaitGrabSPI();
  SAVE_POINT
      while (p->reg != 0) {
        val = TK_BK_MEMRD8(p->reg);
        if(val != p->val)
        {
          TRACE("reg(0x%X) = 0x%X\n\r", p->reg, val);
        }
        p++;
      }

      {
        val = TK_BK_MEMRD8(CC2520_EXCFLAG2);
        TRACE("EXCFLAG2 = 0x%X\n\r", val);
      }

      {
        val = TK_BK_MEMRD8(CC2520_FSMSTAT0);
        TRACE("FSMSTAT0 = 0x%X\n\r", val);
      }

      {
        val = TK_BK_MEMRD8(CC2520_FSMSTAT1);
        TRACE("FSMSTAT1 = 0x%X\n\r", val);
      }

      RadioIMU_ReleaseSPI();

}

//CC2520 when raises RX_OVERFLOW (happens async) stops receiving until RX_OVERFLOW
//is cleared
// handle rx overflow & underflow errors
void ProcessRXError() {
    assert(spiTxRxByteState == RF_SPI_INIT_STATE);      // State machine must be stopped first
    while (CoAcceptSem(semRFRxFrames) != E_SEM_EMPTY);        // remove all pending singnals
    rxErrors++;
    Send_SPI_byte(CC2520_INS_SFLUSHRX);     // double flush ...
    Send_SPI_byte(CC2520_INS_SFLUSHRX);     // ... [CC2520 Bug#1]
    Send_SPI_byte(CC2520_INS_ABORT);
    Send_SPI_byte(CC2520_INS_SRXON);

    TK_BK_REGWR8(CC2520_EXCFLAG0, 0);
    TK_BK_REGWR8(CC2520_EXCFLAG1, 0);
    TK_BK_REGWR8(CC2520_EXCFLAG2, 0);
    Send_SPI_byte(CC2520_INS_SRXON);

    // there are cases that major body of packet received and even valid but CC2520 resports error. In this case all content must be cleared otherwise once CC2520 error cleared. Task2() will process it
    memset(scratchBuf, 0, sizeof(scratchBuf));
    HwWait(1);      // turn around time
}



/*******************************************************************************
* Description : [API] wait and gets a new Received Packet
*******************************************************************************/
rxPkt_t* RadioRxPkt() {

   SAVE_POINT
    //read count and first byte;
    rxCount 		= TK_BK_REGRD8(CC2520_RXFIFOCNT);
    cc2520_flags0	= TK_BK_REGRD8(CC2520_EXCFLAG0);
    cc2520_flags1 	= TK_BK_REGRD8(CC2520_EXCFLAG1);
    cc2520_flags2 	= TK_BK_REGRD8(CC2520_EXCFLAG2);

//    assert((cc2520_flags0 & (1 << CC2520_EXC_RX_OVERFLOW)) == 0);
    scratchBuf[0] 	= CC2520_INS_BCLR;    // reset RX_FRM_DONE signal/exception
    scratchBuf[1] 	= (CC2520_EXCFLAG1 << 3) | (CC2520_EXC_RX_FRM_DONE - 8);
    scratchBuf[2] 	= CC2520_INS_SNOP;    // pad to 16-bit word align
   // scratchBuf[3] = CC2520_INS_REGRD | CC2520_RXFIFOCNT;
   // scratchBuf[4] = CC2520_INS_SNOP;
    scratchBuf[3] 	= CC2520_INS_RXBUF;
    scratchBuf[4] 	= CC2520_INS_SNOP;

    spiTxRxByteCount = 5;  // go to RXBUF_Part2
    assert(spiTxRxByteState == RF_SPI_INIT_STATE);
    spiTxRxByteState = RF_SPI_RX1_UPLOADCMD_STATE;

    pSpiTxBuf = scratchBuf;
    pSpiRxBuf = (uint8_t*)&rxPkt;

    rxDoneType = 1;     // Assuming not successful State Machine will change it to 0 if successful
    CoClearFlag(flagSPIMachineDone);        // Must set before enabling SPI IO as SPI might happen at once (rear but possible)
    // roll state machine
	//CS_SPI3 = RADIO_CS;
    HwSPISSAssert(SPI_RADIO);
    SPI_I2S_ITConfig(SPI_RADIO_IMU_SPI, SPI_I2S_IT_RXNE, ENABLE);
    SPI_I2S_SendData(SPI_RADIO_IMU_SPI, *pSpiTxBuf++);

    CoWaitForSingleFlag(flagSPIMachineDone, 0);
    SAVE_POINT
    if (rxDoneType == 1) {      //overflow or underflow, which can stop SPI actions
      if (rxCount) {
        //TODO - should I flash? Sets rxFIFOError to flash
        TRACE("RX error rxCount=%d\n\r", rxCount);
      } // else false IRQ - ignore
        return NULL;
    }

    rxPkt.payloadSize = rxPkt.frameLength - MHR_SIZE - MFR_SIZE;
    rxPkt.rssi = rxPkt.payload[rxPkt.payloadSize];

    // printf("*RSSI*: %x \n\r",rxPkt.rssi);
    // check if received CRC good
    rxPkt.lqi = rxPkt.payload[rxPkt.payloadSize + 1];
    if (radio_off) {
      return NULL;
    }
    if (rxPkt.lqi & 0x80) {
        rxPkt.lqi &= 0x7F;
        random =  RadioGetRandom();
        if (config.flags & FLAG_TRACE_BEACON) {
          TRACE("OK srcAddr=0x%04X panId=%d rxCount=%d sec=%d flags=0x%02X\n\r",
              rxPkt.srcAddr, rxPkt.panId, rxCount, sec, cc2520_flags0);
        }
        //while(rxPkt.destAddr != 0xFFFF || rxPkt.srcAddr != 0xABCD);
    } else {
      if ( config.flags & FLAG_TRACE_CRC) {
        TRACE("Bad CRC. Abort rxCount=%d sec=%d.%d flags=0x%02X\n\r", rxCount, sec, TIM1->CNT, cc2520_flags0);
      }
      return NULL;    // bad CRC => reject packet
    }

    // TODO!!! post to receive task; or should not happen if the first packet is not finished processing yet
    // give user a copy of receive packet (avoid clobbering by next ISR-received pkt)
    memcpy((void*)&rxPktCopy, (void*)&rxPkt, sizeof(rxPkt_t));
    return &rxPktCopy;
}

uint8_t readCCA;
uint32_t txErrors;
uint32_t sem;
StatusType semAllowPost;
extern uint8_t semAllow;
uint16_t txLine;

/*******************************************************************************
* Description : [API] Transmit a Packet Immediately
* Input       : trigger => 0 = on GPIO2 rising-edge; 1 = right away
* Return      : -
*******************************************************************************/
void RadioTxPkt(uint16_t dstAddr, uint8_t beacon, uint8_t payloadSize, uint8_t *payload, uint8_t trigger) {

  SAVE_POINT
  txLine = __LINE__;
  if (radio_off) {
    // radio off
    return;
  }
  SAVE_POINT
  txLine = __LINE__;
    if (txFIFOError) {          // TODO!!! Question: when SPI uploading a frame a TX-FIFO takes place, will the behaviour get affected???  Now assuming No
        txFIFOError = 0;
        assert(spiTxRxByteState == RF_SPI_INIT_STATE);      // State machine must be stopped first
        // Clear TX-FIFO error
        Send_SPI_byte(CC2520_INS_SFLUSHTX);
        Send_SPI_byte(CC2520_INS_SFLUSHTX);

        TK_BK_REGWR8(CC2520_EXCFLAG0, 0);
        TK_BK_REGWR8(CC2520_EXCFLAG1, 0);
        TK_BK_REGWR8(CC2520_EXCFLAG2, 0);
        txErrors++;
        txLine = line = __LINE__;
        HwWait(1);      // turn around time
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
    txLine = line = __LINE__;

    // prev TX is done, so reset TX_FRM_DONE signal/exception
    TK_BK_REGWR8(CC2520_EXCFLAG0, ~CC2520_EXC0_TX_FRM_DONE_BM);

    // kick off ISR-driven SPI upload
    pSpiTxBuf = txBuf;
    pSpiRxBuf = scratchBuf;

    spiTxRxByteCount = 5 + hdrBytes + payloadSize;
    assert(spiTxRxByteState == RF_SPI_INIT_STATE);
    spiTxRxByteState = RF_SPI_UPLOAD_ONLY_STATE;
    readCCA = 2;
    txDoneType = 1;     // Assuming not successful. State Machine will change it to 0 if successful
    // Roll state machine when time slot arrives

    // TODO!!! change logic here:
    // wait till we are allowed to TX (i.e. not within TX Inhibit guard-band)
    txLine = line = __LINE__;

#ifndef CCA_EN
    if (config.flags & FLAG_TRACE_USE_TIMESLOT) {
        // imitate "event" synchornization. could be slower
       // CoClearFlag(flagRadioTxAllow);
      if (CoAcceptSingleFlag(flagRadioTxAllow) == E_OK) {
        // that means flag already was raised, no worry,
        // the flag was cleared, will deliver in the next
        // slot
      }
      SAVE_POINT
      txLine = __LINE__;
      CoWaitForSingleFlag(flagRadioTxAllow, 0);
      SAVE_POINT
      txLine = __LINE__;

    }
#endif

    txLine = __LINE__;

    if (config.flags & FLAG_TRACE_TIMESLOT) {
        // time slot debug
        extern uint16_t txTimeSlot;
        extern uint32_t secs;
        uint16_t checkPoint = TIM_GetCounter(TIM2);
        uint16_t slotDelta = 2100;
        uint16_t slot2Offset = 60000 / 2 +  + txTimeSlot;
        BOOL error = 0;
        if (   (checkPoint >= txTimeSlot && checkPoint < txTimeSlot + slotDelta)
            || (checkPoint >= slot2Offset && checkPoint < slot2Offset + slotDelta)
                ) {
                    error = 0;
                }
        else {
            error = 1;
        }
        TRACE("[%d-%d %d-%d]=>(%d.%d %c)-", txTimeSlot, txTimeSlot + slotDelta, slot2Offset, slot2Offset + slotDelta, secs, checkPoint, error == 0 ? ' ' : 'E');
    }

    CoClearFlag(flagSPIMachineDone);         // Must set before enabling SPI IO as SPI might happen at once (rear but possible)
    
	//CS_SPI3 = RADIO_CS;
	HwSPISSAssert(SPI_RADIO);
    SPI_I2S_ITConfig(SPI_RADIO_IMU_SPI, SPI_I2S_IT_RXNE, ENABLE);
    SPI_I2S_SendData(SPI_RADIO_IMU_SPI, *pSpiTxBuf++);      // send first byte. Other will be handled by SPI3 interrupt
    txLine =  __LINE__;
    SAVE_POINT
    CoWaitForSingleFlag(flagSPIMachineDone, 0);
    SAVE_POINT
    txLine = __LINE__;
    if (txDoneType == 1) {      //overflow or underflow, which can stop SPI actions
        TRACE("TX error\n\r");
    }

     // Upload to CC2520 does not mean that the frame has been sent out into the air
    if ((txError = CoWaitForSingleFlag(flagRadioTxDone, 50)) != E_OK) {  // TODO!!! verify if it can happen. If not, use infinit wait
        //assert(0);      // should not happen. Happened when unluging ST-Link in debug mode
      SAVE_POINT
      txLine = __LINE__;
      TRACE("TX timout error =%d sec=%d.%d\n\r", txError, sec, TIM1->CNT);
        // put radio back into known state
    //}
        Send_SPI_byte(CC2520_INS_SFLUSHTX);
        Send_SPI_byte(CC2520_INS_SFLUSHTX);

        TK_BK_REGWR8(CC2520_EXCFLAG0, 0);
        TK_BK_REGWR8(CC2520_EXCFLAG1, 0);
        TK_BK_REGWR8(CC2520_EXCFLAG2, 0);
        HwWait(1);      // turn around time
    }
    SAVE_POINT
    txLine = __LINE__;
    if (config.flags & FLAG_TRACE_TIMESLOT) {
      TRACE("radio done at %d.%d\r\n", secs, TIM2->CNT);
    }

    semAllowPost = CoPostSem(semIMUAllow);
    semAllow = 1;
    sem++;
    txLine = line = __LINE__;

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
    SAVE_POINT
    RadioIMU_WaitGrabSPI();
    SAVE_POINT
    Send_SPI_byte(CC2520_INS_SFLUSHRX);     // double flush ...
    Send_SPI_byte(CC2520_INS_SFLUSHRX);     // ... [CC2520 Bug#1]
    RadioIMU_ReleaseSPI();
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
    RadioIMU_WaitGrabSPI();
    SAVE_POINT
    TK_BK_MEMWR16(CC2520_RAM_PANID, panId);
    TK_BK_MEMWR16(CC2520_RAM_SHORTADDR, shortAddr);
    RadioIMU_ReleaseSPI();
}

/*******************************************************************************
* Description : [API] Sets new RF Channel
* Input       : chan => {11 - 26}
* Return      : -
*******************************************************************************/
void RadioSetRFChan(uint8_t chan) {
    radioConfig.channel = chan;
    SAVE_POINT
    RadioIMU_WaitGrabSPI();
    SAVE_POINT
    Send_SPI_byte(CC2520_INS_SRFOFF);
    TK_BK_REGWR8(CC2520_FREQCTRL, MIN_CHANNEL + ((chan - MIN_CHANNEL) * CHANNEL_SPACING) );
    Send_SPI_byte(CC2520_INS_SRXON);
    Send_SPI_byte(CC2520_INS_SFLUSHRX);     // double flush ...
    Send_SPI_byte(CC2520_INS_SFLUSHRX);     // ... [CC2520 Bug#1]

    TK_BK_REGWR8(CC2520_EXCFLAG0, 0);
    TK_BK_REGWR8(CC2520_EXCFLAG1, 0);
    TK_BK_REGWR8(CC2520_EXCFLAG2, 0);
    Send_SPI_byte(CC2520_INS_SRXON);

    RadioIMU_ReleaseSPI();
}


/*******************************************************************************
* Description : [API] Sets new RF Tx Level
* Input       : chan => {11 - 26}
* Return      : -
*******************************************************************************/
void RadioSetRFLevel(uint8_t TxLev) {
//    radioConfig.channel = chan;
int8_t status;
 //NO RadioWaitGrabSPI??? Potential bug!
    SAVE_POINT
    RadioIMU_WaitGrabSPI();
    status = Send_SPI_byte(CC2520_INS_SNOP); // save curent status
    Send_SPI_byte(CC2520_INS_SRFOFF);
	TK_BK_MEMWR8(CC2520_TXPOWER, TxAmpValues[TxLev]);
    Send_SPI_byte(CC2520_INS_SNOP);
    Send_SPI_byte(CC2520_INS_SNOP);
    if( status & CC2520_STB_TX_ACTIVE_BM){
        Send_SPI_byte(CC2520_INS_STXON);  // turn on the radio
    }else{
        Send_SPI_byte(CC2520_INS_SRXON);
        Send_SPI_byte(CC2520_INS_SFLUSHRX);     // double flush ...
        Send_SPI_byte(CC2520_INS_SFLUSHRX);
    }
    RadioIMU_ReleaseSPI();
    HwWait(10);
}
/*******************************************************************************
* Description : [API] Gets RF Tx Level from 2520 X Level register
* Input       :
* Return      : Level
*******************************************************************************/
uint8_t RadioGetFLevel(void){
uint8_t tx_power;

      SAVE_POINT
      RadioIMU_WaitGrabSPI();
      SAVE_POINT
      tx_power= TK_BK_REGRD8(CC2520_TXPOWER);
      RadioIMU_ReleaseSPI();
      return tx_power;
}

/*******************************************************************************
* Description : [API] sets 2520 radio GPIO 0 as analog temperature out
* Input       :
* Return      :
*******************************************************************************/

/*******************************************************************************
* Description : [API] REVJ. Sets up the IMU to default settings
* Input       :
* Return      :
*******************************************************************************/
void IMUInit(void){	
	const uint8_t whoami = 0xEA;
    regVal_t *p = regVal_IMU;

	HwWait(10);
	IMU_SelectBank(BANK0);
	IMU_WriteOneByte(IMU_PWR_MGMT_1, 0x01);
	HwWait(10);
	IMU_SelectBank(BANK0);
	IMU_WriteOneByte(IMU_USER_CTRL,0x78);
	
	while (p->reg != 0xFF) {
        IMU_WriteOneByte(p->reg, p->val);
        p++;
    }

    IMU_SelectBank(BANK0);
	if(IMU_ReadOneByte(IMU_WHO_AM_I)!=whoami){
		IMUPresent=0;
		assert(0);
	}
	else IMUPresent=1;

	//setup external gpio interrupts if required on the stm.
	HwIMUEXTIInit();
}

/*******************************************************************************
* Description : [API] REVJ. Sets up IMU_TX buffer to gather IMU raw data.
* Input       :
* Return      :
*******************************************************************************/
void IMUProcess(void){
	pSpiRxBuf_IMU = IMU_RawData;
	pSpiTxBuf_IMU = IMU_TXBuffer;
	spiIMUCount = 12;
	spiIMUByteState = IMU_SPI_INIT_STATE;
	
	SAVE_POINT
	//CS_SPI3 = IMU_CS;
	HwSPISSAssert(SPI_IMU);
	SAVE_POINT
	//I want to send: address read at address 0x33 (1 byte) + 12 bytes to ready gyro data (uint16_t) x,y,z.
	SPI_I2S_ITConfig(SPI_RADIO_IMU_SPI, SPI_I2S_IT_RXNE, ENABLE);
    SPI_I2S_SendData(SPI_RADIO_IMU_SPI, *pSpiTxBuf_IMU++);
	
	SAVE_POINT	
	CoWaitForSingleFlag(flagIMUNewData, 0);
	SAVE_POINT	
	//HwSPISSDeAssert(SPI_IMU); 		//Deassert within ISR when received all data.
}
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
