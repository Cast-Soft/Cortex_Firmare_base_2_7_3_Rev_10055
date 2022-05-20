/******************** (C) COPYRIGHT 2011 NaturalPoint, Inc. ********************
* File Name          : tasks.c
* Author             : ?
* Version            : V1.0
* Date               : 6/2/2011
* Description        : All the various System Tasks
*******************************************************************************/

/* INCLUDES ------------------------------------------------------------------*/

#include "tasks.h"
#include "CoOS.h"
#include "hardware.h"
#include "basic_rf.h"
#include "stm32f10x_it.h"
#include "stm32f10x_iwdg.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_adc.h"
#include "tk_version.h"
#include <stdio.h>
#include <string.h>

#include "radio.h"
#include "radio_defs.h"

/* PRIVATE TYPEDEF -----------------------------------------------------------*/

typedef struct {
    uint16_t    dstAddr;
    uint8_t     payloadSize;
    uint8_t     payload[FRAME_LENGTH_MAX - MHR_SIZE - MFR_SIZE];
} txPktRec_t;

/* PRIVATE DEFINES -----------------------------------------------------------*/

#ifdef STDIO_TO_USART
#define DISABLE_PWR_SW
#endif

#define PINGER_ADDR         0x1234
#define TX_PKT_QUEUE_SIZE   8

/*
 Beacon Pkt Duration = 0.70ms
 Max TX Pkt Duration = 3.5ms
 Inhibit Guard-Band Pre = 4.0ms
 Inhibit Guard-Band Post = 0.5ms
*/
#define RF_TX_INH_BEGIN     57200
#define RF_TX_INH_END       300

/* PRIVATE MACROS ------------------------------------------------------------*/

/* EXTERN VARIABLES ----------------------------------------------------------*/

/* Variables Defined in main.c */

extern char firmwareVersion[];

extern OS_TID task1Id;
extern OS_TID task2Id;
extern OS_TID task5Id;
extern OS_TID task8Id;
//extern OS_TID AtoDtaskID;

extern OS_FlagID flagIMUNewData;
extern OS_FlagID flagRadioTxReq;

extern  volatile uint32_t IMUSampleTime;
extern volatile uint16_t MsTimerAtSync;

extern Batt_Union_t BattUnion, *pBattUnion;
extern const uint8_t date_str[] = {__DATE__};
extern const uint8_t time_str[]={__TIME__};
/* PRIVATE VARIABLES ---------------------------------------------------------*/

static volatile uint16_t tasksWDT = 0;
static volatile uint16_t halted = 0;

static uint16_t beaconTxEn = 0; // beacon pkt rx = permission to transmit

static uint16_t txCount;        // ping count
static uint8_t txTestStr[] = {0xAA, 0x55, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};

static uint8_t TxmultB[128];

static uint8_t txBuf[FRAME_LENGTH_MAX - MHR_SIZE - MFR_SIZE];  // tx-payload buffer

static uint16_t txPktCount = 0;
static uint16_t rxPktCount = 0;

static uint8_t rfChan;
static uint8_t beaconRSSI;
static uint16_t tkAddr = 0;

static txPktRec_t txPktQueue[TX_PKT_QUEUE_SIZE];
static uint16_t inIdx = 0;
static uint16_t outIdx = 0;
uint16_t RfTxLevel;

uint16_t IMURegRd(uint8_t addr);
uint32_t IMUdbgPrt = 0;

uint8_t test_imu_pkt_ctr = 0;

DeviceSerialNumber_t ARM_proc_SN , *pARM_proc_SN ;

/* PUBLIC VARIABLES ----------------------------------------------------------*/

config_t config = {
    .productID      = 0xBC10,
    .serialNum      = 0x444,
    .panId          = 0x0001,
    .mySrcAddr      = 0x5678,
    .tkDstAddr      = 0xABCD,
    .ledOnOffs      = 52000,
    .ledOffOffs     = 4000,
    .ledDAC         = 3840,
    .rfChan         = 0x00,
    .rfChanMin      = 9,
    .rfChanMax      = 21,
    .led0Id         = 0xFE,
    .led1Id         = 0xFD,
    .led2Id         = 0xFB,
    .TestMode       = 0x00,
    .TxLevel         = 0x00,
    .RxPrint        =0x00,
    .AtoDon_off     =0x01
};




/* EXTERNAL FUNCTION PROTOTYPES ---------------------------------------------*/
void WriteConfig(void);
/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/

static void mygets(char *str);
static void IMURegWr(uint8_t addr, uint16_t val);
//static uint16_t IMURegRd(uint8_t addr);
static uint16_t CalcConfigChecksum(void);
static void SetConfig(uint16_t idx, uint16_t val, uint16_t val2);
static void PrintConfig(void);

static void RadioTxPktQueue(uint16_t dstAddr, uint8_t payloadSize, uint8_t *payload);

/* PRIVATE FUNCTIONS ---------------------------------------------------------*/

/*******************************************************************************
* Description : Get a line from STDIN
* Input       :
* Return      : -
*******************************************************************************/
static void mygets(char *str) {
    char *tmp = str;
    int c;

    do {
        while( (c = getchar()) == EOF ) {
            // 100ms => humans don't type >10 char/sec
            CoTickDelay(10);
            // reset tasksWDT
            tasksWDT |= 0x0002;
        }
        putchar(c);
        *tmp++ = (char) c;
    } while (c != '\r');

    putchar('\n');
    *(--tmp) = '\0';
}

/*******************************************************************************
* Description : Write to an IMU Register
* Input       :
* Return      : -
*******************************************************************************/
static void IMURegWr(uint8_t addr, uint16_t val) {
    SPI_I2S_ReceiveData(SPI_IMU_SPI); // flush RXNE

    // write LSB
    HwSPISSAssert(SPI_IMU);
    SPI_I2S_SendData(SPI_IMU_SPI, (((addr | 0x80) << 8) | (val & 0x00FF)));
    while (SPI_I2S_GetFlagStatus(SPI_IMU_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    SPI_I2S_ReceiveData(SPI_IMU_SPI); // flush RXNE
    HwSPISSDeAssert(SPI_IMU);

    for (volatile uint16_t i = 0; i < 30; i++); // t_stall(min) = 9us

    // write MSB
    HwSPISSAssert(SPI_IMU);
    SPI_I2S_SendData(SPI_IMU_SPI, ((((addr+1) | 0x80) << 8) | (val >> 8)));
    while (SPI_I2S_GetFlagStatus(SPI_IMU_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    SPI_I2S_ReceiveData(SPI_IMU_SPI); // flush RXNE
    HwSPISSDeAssert(SPI_IMU);
}

/*******************************************************************************
* Description : Read from an IMU Register
* Input       :
* Return      : -
*******************************************************************************/
//static uint16_t IMURegRd(uint8_t addr) {
uint16_t IMURegRd(uint8_t addr) {

    uint16_t value;

    SPI_I2S_ReceiveData(SPI_IMU_SPI); // flush RXNE

    // send register addr
    HwSPISSAssert(SPI_IMU);
    SPI_I2S_SendData(SPI_IMU_SPI, (addr << 8));
    while (SPI_I2S_GetFlagStatus(SPI_IMU_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    SPI_I2S_ReceiveData(SPI_IMU_SPI); // ignore received data
    HwSPISSDeAssert(SPI_IMU);

    for (volatile uint16_t i = 0; i < 30; i++); // t_stall(min) = 9us

    // read register value
    HwSPISSAssert(SPI_IMU);
    SPI_I2S_SendData(SPI_IMU_SPI, 0);
    while (SPI_I2S_GetFlagStatus(SPI_IMU_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    value = SPI_I2S_ReceiveData(SPI_IMU_SPI);
    HwSPISSDeAssert(SPI_IMU);

    return value;
}

/*******************************************************************************
* Description : Calculate Configuration Checksum
* Input       :
* Return      : checksum
*******************************************************************************/
static uint16_t CalcConfigChecksum(void) {
    uint16_t checksum = 0xCCCC; // checksum initialization
    uint16_t *pConfig = (uint16_t*)&config;

    // checksum is last half-word in struct
    // checksum field not included in checksum
    for (uint16_t i = 0; i < (sizeof(config)/2 - 1); i++) {
        checksum ^= (i ^ *pConfig++);
    }
    return checksum;
}

/*******************************************************************************
* Description : Set Led on/off
* Input       :
* Return      : -
*******************************************************************************/
void LedSet(uint16_t val){
uint16_t i, mask;

        val &= 0x1f;
        mask = 0x01;
      for (i=0;i<5;i++){
        if(val & mask){
          HwLEDOn(i);
        }else{
          HwLEDOff(i);
        }
        mask= mask <<1;
      }
      
  
}  

/*******************************************************************************
* Description : Set Led on/off
* Input       :
* Return      : -
*******************************************************************************/
void  LedSetIR(uint16_t val){
uint16_t i, mask;  
        val &= 0x1f;
        mask = 0x01;
      for (i=0;i<3;i++){
        if(val & mask){
          HwLEDOn(i);
        }else{
          HwLEDOff(i);
        }
        mask= mask <<1;
      }  
  
  
}  
/*-----------------------------------------------------------*/
// ARM processor unique ID
// set at manufacturing time
// 96 bits UUID
/*-----------------------------------------------------------*/
void GetARM_UUID(void){
   

    ARM_proc_SN.a = *(__IO uint32_t *)(0x1FFFF7E8);
    ARM_proc_SN.b = *(__IO uint32_t *)(0x1FFFF7EC);
    ARM_proc_SN.c = *(__IO uint32_t *)(0x1FFFF7F0);
    
}    
    

/*******************************************************************************
* Description : Set Configuration
* Input       :
* Return      : -
*******************************************************************************/
static void SetConfig(uint16_t idx, uint16_t val, uint16_t val2) {
    switch (idx) {
        /*
      
            8 : set RF channel
            a : set led vals
            b : set IR led state
            c : test IMU
            d : test I2CNV ram
            e : enable/disable Tx continuous
            f : stream Rx'd packets
            g : Tx test packet...
            h : read ADC battry voltage
            i : get ARM serial number
       
        */
        case '8': // set RF channel
            RadioSetRFChan( RFChanValues[val]);
            break;
         case 'a': // set led vals
            LedSet(val);
            break;
        case 'b':  // set IR led state
           LedSetIR(val);
            break;
        case 'c': // test IMU
            
            break;
        case 'd': // test I2CNV ram
            
            break;
        case 'e': // enable/disable Tx continuous 
          if(config.TestMode){
            config.TestMode =0;
          }else{
            config.TestMode=1;
          }
            RadioInit( config.panId, config.mySrcAddr, RFChanValues[config.rfChan]);
            break;
        case 'f': // stream Rx'd packets
          if(config.RxPrint){
            config.RxPrint=0;
          }else{
            config.RxPrint=1;
          }
            break;
            case 'g': // Tx test packet...data len, character to fil buffer  x len
              
              memset(TxmultB, val2 ,val);  // init the memory
              
            RadioTxPkt(BASIC_RF_BROADCAST_ADDR, 0, val , TxmultB, 1);  // send packet to address 0xFFFF, not a Sync beacon, len of data 115,dta buffer, send NOW
            
              
              
            break;
          case 'h': // read ADC battery voltage
            if(config.AtoDon_off){
              config.AtoDon_off=0;
            }else{
              config.AtoDon_off=1;
            CoAwakeTask(task8Id);
            }
            break;
          case 'i': // get ARM serial number
            printf("%x:%x:%x\r\n", pARM_proc_SN->a,pARM_proc_SN->b, pARM_proc_SN->c);
            
            case 'j': // Set TX transmit power
             RadioSetRFLevel(val);
            break;  
            
            
        default:
            printf("**ERROR** Unrecognized Configuration Setting\n\r");
            break;
    }
}



/*******************************************************************************
* Description : Set Configuration
* Input       :
* Return      : -
*******************************************************************************/
static void GetConfig(uint16_t idx, uint16_t val) {
    switch (idx) {
        /*
              h : read ADC battry voltage
            i : get ARM serial number
        */
          case 'h': // read ADC battry voltage
            
            break;
          case 'i': // get ARM serial number
            printf("%x:%x:%x\r\n", pARM_proc_SN->a,pARM_proc_SN->b, pARM_proc_SN->c);
              break;

            
        default:
            printf("**ERROR** Unrecognized Configuration Setting\n\r");
            break;
    }
}
/*******************************************************************************
* Description : Print Configuration
* Input       : -
* Return      : -
*******************************************************************************/
static void PrintConfig(void) {
    config.checksum = CalcConfigChecksum();

    printf("FIRMWARE VERSION : %d.%d.%d\n\r\n\r", BC_VERSION_MAJOR,BC_VERSION_MINOR,BC_VERSION_REVISION);
    printf(" %s : %s \n\n\r",date_str,time_str);
    printf(
"\
[-] checksum    : %04X\n\r\
[0] productID   : %X\n\r\
[1] serialNum   : %X\n\r\
[2] panId       : %04X\n\r\
[3] mySrcAddr   : %04X\n\r\
[4] tkDstAddr   : %04X\n\r\
[5] ledOnOffs   : %X\n\r\
[6] ledOffOffs  : %X\n\r\
[7] ledDAC      : %X\n\r\
[8] rfChan      : %02X\n\r\
[9] rfChanMin   : %02X\n\r\
[A] rfChanMax   : %02X\n\r\
[B] led0Id      : %02X\n\r\
[C] led1Id      : %02X\n\r\
[D] led2Id      : %02X\n\r\
[E] TestMode    : %d\n\r\
[Z] Tx RF Level : %X\n\r"
 ,
        config.checksum,
        config.productID,
        config.serialNum,
        config.panId,
        config.mySrcAddr,
        config.tkDstAddr,
        config.ledOnOffs,
        config.ledOffOffs,
        config.ledDAC,
        config.rfChan,
        config.rfChanMin,
        config.rfChanMax,
        config.led0Id,
        config.led1Id,
        config.led2Id,
        config.TestMode,  
        config.TxLevel  
        );
}

/*******************************************************************************
* Description : Write Configuration to FLASH
* Input       : -
* Return      : -
*******************************************************************************/
void WriteConfig(void) {
    uint16_t *pConfig = (uint16_t*)&config;

    config.checksum = CalcConfigChecksum();

    FLASH_Unlock();

    if (FLASH_ErasePage(FLASH_CONFIG_PAGE) != FLASH_COMPLETE) {
        printf("**ERROR** Config Page Erase FAILED!!");
        return;
    }

    for (uint16_t i = 0; i < (sizeof(config)/2); i++) {
        if (FLASH_ProgramHalfWord(FLASH_CONFIG_PAGE + (i*2), *pConfig++) != FLASH_COMPLETE) {
            printf("**ERROR** Config Page Write FAILED!!");
        }
    }
    printf("**Cofiguration Written Successfully**\n\r");

    FLASH_Lock();
}

/*******************************************************************************
* Description : Add Packet to Radio Transmit Queue
* Input       : -
* Return      : -
*******************************************************************************/
static void RadioTxPktQueue(uint16_t dstAddr, uint8_t payloadSize, uint8_t *payload) {
    if ( ((inIdx + 1) % TX_PKT_QUEUE_SIZE) == outIdx ) return; // queue is full => ignore request

    CoSchedLock();
    txPktQueue[inIdx].dstAddr = dstAddr;
    txPktQueue[inIdx].payloadSize = payloadSize;
    memcpy((void *)txPktQueue[inIdx].payload, (void *)payload, payloadSize);
    inIdx = (inIdx + 1) % TX_PKT_QUEUE_SIZE;

    CoSchedUnlock();

    CoSetFlag(flagRadioTxReq);
}

/* PUBLIC FUNCTIONS ----------------------------------------------------------*/

/*******************************************************************************
* Description    : [Task] Process and Send each new IMU data sample
* Input          :
* Return         :
*******************************************************************************/
void Task1(void* pdata){
    static uint8_t seqNum = 0;
    static uint16_t *pBuf;
    static uint16_t udata;
    static uint16_t i = 0;
    static uint16_t j = 0;
    static struct Beacon_Data_pkt *pBeacon_Data_pkt;
//  static uint16_t TestTimer = 0;
  
  
#define Pointer_test    


    while (1) {

#ifdef Pointer_test
      pBeacon_Data_pkt = (struct Beacon_Data_pkt*)txBuf;
      
//      TestTimer = TIM_GetCounter(TIM1);


      pBeacon_Data_pkt->BK_Preamble.button_pr = (HwButtonPressed(BUTTON2) ? 0x02 : 0x00) |
                  (HwButtonPressed(BUTTON1) ? 0x01 : 0x00) ;
         pBeacon_Data_pkt->BK_Preamble.Seq_Num=  seqNum++ ;
        pBeacon_Data_pkt->BK_Preamble.BeaconRSSI= beaconRSSI;
        pBeacon_Data_pkt->BK_Preamble.IRLed0= config.led0Id;
        pBeacon_Data_pkt->BK_Preamble.IRLed1= config.led1Id;
        pBeacon_Data_pkt->BK_Preamble.IRLed2= config.led2Id;
        pBeacon_Data_pkt->BK_Preamble.Battery_lev = BattUnion.BatteryLevel[1];
        pBeacon_Data_pkt->BK_Preamble.SyncFrameIMU=frameIdAtSync;
        pBeacon_Data_pkt->BK_Preamble.MsTimerIMU=MsTimerAtSync;
        
                for (i = 0; i < 8; i++) {
            CoWaitForSingleFlag(flagIMUNewData, 0);
            //CoTickDelay(2); // [[DEBUG]]
//  insert 8 bit LSB of 32 bit timstamp ahead of IMU data
// pBuff+ = 
#ifdef BC_HW_REVC            
       HwGPOHigh(GPO_TP50);
#endif       
         if(!(IMUdbgPrt)){
    //        pBeacon_Data_pkt->BeaconIMUData[i].Timestamp = IMUSampleTime;
              pBeacon_Data_pkt->BeaconIMUData[i].Timestamp = test_imu_pkt_ctr++;
              
             pBuf = (uint16_t*) &pBeacon_Data_pkt->BeaconIMUData[i].gyroscopeX;
            for (j = 1; j < 7; j++) {
                udata = spiIMURxBuf[j] & 0x3FFF; // IMU data is 14-bits
              *pBuf++ = (udata & 0x2000) ? (udata | 0xC000) : udata;  // sign-extension
            }
         }else{
           
        //    pBeacon_Data_pkt->BeaconIMUData[i].Timestamp = IMUSampleTime;
        //    printf("%d, ",IMUSampleTime);
            pBeacon_Data_pkt->BeaconIMUData[i].Timestamp = test_imu_pkt_ctr++;
            printf("%d, ",test_imu_pkt_ctr);
            
             pBuf = (uint16_t*) &pBeacon_Data_pkt->BeaconIMUData[i].gyroscopeX;
            for (j = 1; j < 7; j++) {
                udata = spiIMURxBuf[j] & 0x3FFF; // IMU data is 14-bits
                printf("%2X, ",udata);
              *pBuf++ = (udata & 0x2000) ? (udata | 0xC000) : udata;  // sign-extension
            }
           printf("\n\r");
         }
#ifdef BC_HW_REVC           
             HwGPOLow(GPO_TP50);
#endif         
        }
        
#else

       pBuf = (uint16_t*)txBuf;

        // packet first 2-bytes : {seq#, button_state}
        *pBuf++ = (HwButtonPressed(BUTTON2) ? 0x200 : 0x000) |
                  (HwButtonPressed(BUTTON1) ? 0x100 : 0x000) |
                  seqNum++ ;

        // packet next 4-bytes : {beaconRSSI, led0Id, led1Id, led2Id}
        *pBuf++ = (config.led0Id << 8) | beaconRSSI;
        *pBuf++ = (config.led2Id << 8) | config.led1Id;

        for (i = 0; i < 8; i++) {
            CoWaitForSingleFlag(flagIMUNewData, 0);
            //CoTickDelay(2); // [[DEBUG]]
//  insert 8 bit LSB of 32 bit timstamp ahead of IMU data
// pBuff+ =             
            
            for (j = 1; j < 7; j++) {
                udata = spiIMURxBuf[j] & 0x3FFF; // IMU data is 14-bits
                *pBuf++ = (udata & 0x2000) ? (udata | 0xC000) : udata;  // sign-extension
            }
        }



#endif      

       

        // disallow transmits if Beacon is not hearing TimeKeeper
        if (beaconTxEn > 0) {
            beaconTxEn--;
            /* (12 bytes/sample * 8 samples) + (2+4 bytes status) */
 //           RadioTxPktQueue(tkAddr, 96 + 6, txBuf);
            
             
             
             RadioTxPktQueue(tkAddr, sizeof(struct Beacon_Data_pkt) , txBuf);
            //RadioTxPkt(tkAddr, 0, 96 + 6, txBuf, 1); // [[DEBUG]]

            HwLEDToggle(LED1);
            if ((txPktCount++ % 50) == 0) {
                HwLEDToggle(LED2);
            }
        }

        // reset tasksWDT
        tasksWDT |= 0x0001;
    }
}

/*******************************************************************************
* Description    : [Task] Process Incoming Radio Packets
* Input          :
* Return         :
*******************************************************************************/
void Task2(void* pdata) {
    static rxPkt_t *pRxPkt;

    while (1) {
        if (pRxPkt = RadioRxPkt(1)) {   // will wait
            HwLEDToggle(LED3);
            if ((rxPktCount++ % 50) == 0) {
                HwLEDToggle(LED4);
            }

            if ((pRxPkt->fcf0 & 0x07) == 0) { // FCF[2:0] = 802.15.4 Beacon Frame Type
                /* RF Sync (Beacon) Packet */

                // synchronize local to remote FrameIDs
                if (*(uint32_t*)pRxPkt->payload) { // ignore stale FrameIDs
                    frameIdCorrection = *(uint32_t*)pRxPkt->payload - frameIdAtSync + 2;
                }

                // update the frequency of TIM3 & TIM2
                uint16_t tim4MovAvg = Tim4GetMovAvg();
                TIM_SetAutoreload(TIM3, tim4MovAvg - 1);
                TIM_SetAutoreload(TIM2, tim4MovAvg - 1);

                // update the phase of TIM3 (alignment of IRLED on/off)
                uint16_t tim3MovAvg = Tim3GetMovAvg();
                //printf(" %5d %5d\n\r", TIM3->CCR1, tim3MovAvg); // [[DEBUG]]
                TIM_SetCompare2(TIM3, (tim3MovAvg + config.ledOnOffs) % tim4MovAvg);
                TIM_SetCompare3(TIM3, (tim3MovAvg + config.ledOffOffs) % tim4MovAvg);

                // update the phase of TIM3 (alignment of Radio TX Inhibit)
                uint16_t tim2MovAvg = Tim2GetMovAvg();
                TIM_SetCompare2(TIM2, (tim2MovAvg + RF_TX_INH_BEGIN) % tim4MovAvg);
                TIM_SetCompare3(TIM2, (tim2MovAvg + RF_TX_INH_END) % tim4MovAvg);

                // allow up to 3 tx pkts per sync pkt
                beaconTxEn += 3;
                if (beaconTxEn > 3) beaconTxEn = 3;

                // record RSSI of Sync Packet to relay back to TK
                beaconRSSI =  pRxPkt->rssi;

            } else if (pRxPkt->srcAddr == PINGER_ADDR) {
                /* Ping Request Packet */
                pRxPkt->payload[pRxPkt->payloadSize] = pRxPkt->rssi;
               
//          printf("RSSI:%x \n\r", pRxPkt->rssi);
               
                pRxPkt->payload[pRxPkt->payloadSize + 1] = pRxPkt->lqi;
                RadioTxPktQueue(pRxPkt->srcAddr, pRxPkt->payloadSize + 2, pRxPkt->payload);
                //RadioTxPkt(pRxPkt->srcAddr, 0, pRxPkt->payloadSize + 2, pRxPkt->payload, 1); // [[DEBUG]]
                HwLEDToggle(LED1);
                if ((txPktCount++ % 50) == 0) {
                    HwLEDToggle(LED2);
                }
                continue;
            } else if (pRxPkt->payloadSize == (sizeof(txTestStr) + 2)) {
                /* Ping Reply Packet */
                printf("PING ECHO: ");
                for (int i = 0; i < pRxPkt->payloadSize; i++) {
                    printf("%02X ", pRxPkt->payload[i]);
                }
                printf("(%02X %02X)\n\r", pRxPkt->rssi, pRxPkt->lqi);
                continue;
            }
        }
    }
}

/*******************************************************************************
* Description    : [Task] RF Chan Scan, then Monitor GPO_PWRON Power Switch
* Input          :
* Return         :
*******************************************************************************/
void Task3(void* pdata){
    static uint16_t holdCount = 0;

    rfChan = config.rfChan;

#if 1
//#if 0    
    /* search for RF Chan on Power-Up */
    static rxPkt_t *pRxPkt = 0;
    static int i;

    HwLEDOn(LED1); HwLEDOn(LED2); HwLEDOn(LED3); HwLEDOn(LED4);
    while (1) {
        if (HwGPIState(GPI_SW_PWR)) {
#ifdef DISABLE_PWR_SW
          holdCount = 0;
#endif          
          if (holdCount++ > 2) {
                CoSchedLock();
                HwLEDOff(LED1); HwLEDOff(LED2); HwLEDOff(LED3); HwLEDOff(LED4);
                HwGPOLow(GPO_PWRON) ; // turn board OFF
                while (1) IWDG_ReloadCounter();

            }
        } else {
            holdCount = 0;
        }

        for (i = 0; i < 10; i++) {
            CoTickDelay(10);
            IWDG_ReloadCounter(); // reset IWDT
            if (pRxPkt = RadioRxPkt(0)) {
                if ((pRxPkt->fcf0 & 0x07) == 0) break; // got Beacon Frame
            }
        }
        if (pRxPkt) {
            if ((pRxPkt->fcf0 & 0x07) == 0) break;
        }
       if(config.TestMode) break;
        
        if (++rfChan > config.rfChanMax) rfChan = config.rfChanMin;
        RadioSetRFChan(rfChan);
        HwLEDToggle(LED2); HwLEDToggle(LED4);
    }
    // RF Scan => send IMU data to discovered address (hopefully that of the TK!)
    tkAddr = pRxPkt->srcAddr;

      
#else
    // NO RF Scan => send IMU data to TK address stored in FLASH Config
    tkAddr = config.tkDstAddr;
#endif

    /* we're now in business */
    printf("RF CHAN: %d\n\r", rfChan);
    HwLEDOff(LED1); HwLEDOff(LED2); HwLEDOff(LED3); HwLEDOff(LED4);
    CoAwakeTask(task1Id);
    CoAwakeTask(task2Id);
//    CoAwakeTask(AtoDtaskID);
    while (1) {
        CoTickDelay(10);

#ifdef WDT_ENABLE
        /* Reset IWDT if all is kosher */
        if (halted) tasksWDT |= 0x0001;
        if ((tasksWDT & 0x0003) == 0x0003) {
            tasksWDT = 0;
            IWDG_ReloadCounter();
        }
#endif

#ifndef DISABLE_PWR_SW
        if (HwGPIState(GPI_SW_PWR)) {
            if (holdCount++ > 20) {
                CoSchedLock();
                HwLEDOff(LED1); HwLEDOff(LED2); HwLEDOff(LED3); HwLEDOff(LED4);
                HwGPOLow(GPO_PWRON); // turn board OFF
                while (1) IWDG_ReloadCounter();
            }
        } else {
            holdCount = 0;
        }
#endif
    }
}

/*******************************************************************************
* Description    : [Task] Implements Simple Text Command Console
* Input          :
* Return         :
*******************************************************************************/
void Task4(void* pdata) {
    static char s[40] = "";
    static int i;
    static char cmd, addr,value2;
    static int  value;

    while (1) {
        // get a line of values from stdin
        printf("> ");
        mygets(s);

        // parse the line of hexadecimal values
        i = sscanf(s, "%c %c %x %c", &cmd, &addr, &value,&value2);
        if (i < 1) continue;

        switch (cmd) {
            case 'd': // Display configuration
                if (i != 1) continue;
                PrintConfig();
                break;
            case 's': // Set configuration
 //               if (i != 3) continue;
                SetConfig(addr, value, value2);
                break;
            case 'r': // Get config
             if (i != 3) continue;
                GetConfig(addr, value);
                break;
        
 
            case 'h': // Halt IMU Tx
                if (i != 1) continue;
                EXTI->IMR &= ~GPI_IMU_DIO1_EXTI_LINE;
                halted = 1;
                break;
            case 'c': // Continue IMU Tx
                if (i != 1) continue;
                EXTI->IMR |= GPI_IMU_DIO1_EXTI_LINE;
                halted = 0;
                break;
            case 'w': // Write IMU-Reg
                if (i != 3) continue;
                if (!halted) EXTI->IMR &= ~GPI_IMU_DIO1_EXTI_LINE;
                CoTickDelay(2);
                IMURegWr(addr, value);
                if (!halted) EXTI->IMR |= GPI_IMU_DIO1_EXTI_LINE;
                break;
  //          case 'r': // Read IMU-Reg
  //              if (i != 2) continue;
  //              if (!halted) EXTI->IMR &= ~GPI_IMU_DIO1_EXTI_LINE;
  //              CoTickDelay(2);
  //              printf("Reg%02X = %04X\n\r", addr, (int)IMURegRd(addr));
  //              if (!halted) EXTI->IMR |= GPI_IMU_DIO1_EXTI_LINE;
  //              break;
            case 'p': // Ping TK 1x
                if (i != 2) continue;
                printf("PING REQUESTED : %d\n\r", addr);
                txCount = addr;
                CoAwakeTask(task5Id);
                break;
                
            case 'z': // RF Tx Level set
                 if (i != 2) continue;
                 printf("RF Tx Level : %02x \n\r",addr);
                 RfTxLevel= addr;
                 RadioSetRFLevel(RfTxLevel);
                 config.TxLevel=RfTxLevel;
                 printf("RF Tx Level Set : %02x \n\r",RadioGetFLevel());
                 break;

        case '.': // IMU debug print
                 if (i != 1) continue;
                 printf("IMU Debug print:\n\r");
                 IMUdbgPrt = !IMUdbgPrt;
                  break;    
 
        
        
        default:
                printf("**ERROR** Unrecognized Command '%c'\n\r", cmd);
                break;
        }
    }
}

/*******************************************************************************
* Description    : [Task] Pinger
* Input          :
* Return         :
*******************************************************************************/
void Task5(void* pdata){
    while (1) {
        RadioSetPanIdShortAddr(config.panId, PINGER_ADDR); // change addr
        memcpy((void *)txBuf, (void *)txTestStr, sizeof(txTestStr));

        while (txCount) {
            RadioTxPktQueue(BASIC_RF_BROADCAST_ADDR, sizeof(txTestStr), txBuf);
            //RadioTxPkt(BASIC_RF_BROADCAST_ADDR, 0, sizeof(txTestStr), txBuf, 1); // [[DEBUG]]
            HwLEDToggle(LED1);
            if ((txPktCount++ % 50) == 0) {
                HwLEDToggle(LED2);
            }
            txCount--;
            CoTickDelay(10);
        }

        RadioSetPanIdShortAddr(config.panId, config.mySrcAddr); // change back addr

        CoSuspendTask(task5Id);
    }
}

/*******************************************************************************
* Description    : [Task] Manage Outgoing (TX) Packet Queue
* Input          :
* Return         :
*******************************************************************************/
void Task6(void* pdata){
    while (1) {
        // wait for incoming transmit packet request
        CoWaitForSingleFlag(flagRadioTxReq, 0);

        while (inIdx != outIdx) { // queue is NOT empty
            // RadioTxPkt() will block if necessary
            RadioTxPkt(txPktQueue[outIdx].dstAddr,
                       0,   // not a beacon frame
                       txPktQueue[outIdx].payloadSize,
                       txPktQueue[outIdx].payload,
                       1);  // transmit immediately

            outIdx = (outIdx + 1) % TX_PKT_QUEUE_SIZE;
            numTxRetries = 0; // reset counter as we initiate tx
            txRetryState = 1; // indicate STXONCCA has been issued

            /* fire up TIM5 to ring check CCA after 1ms */
            TIM5->EGR |= TIM_EGR_UG;
            TIM5->CR1 |= TIM_CR1_CEN;
        }
    }
}
/*******************************************************************************
* Description    : [Task] Deal with incomming USB command data from BT server
* Input          :
* Return         :
*******************************************************************************/
void Task7(void* pdata){
 while (1) { 
 
   
   
 }
  
} 

/*******************************************************************************
* Description    : [Task]A to D conversion for battery voltage 
* Input          :
* Return         :
*  Data for V#3 hardware.. 4.2v= E1, 4.0v=D7, 3.9v=D0, 3.8v=CA, 3.7v=C6, 3.6v=C1
*                         3.5v=BB, 3.4v=B6, 3.3v=B0,3.2v=AB, 3.1v= A5, 3.0v= A0
*                         2.9v= 9A, 2.8v= 96, 2.7v= 90, 2.6v= 8B             
*
*******************************************************************************/
void Task8(void* pdata){
 while (1) { 
#ifdef BC_HW_REVB   
    if(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)== SET){
            BattUnion.Battery_AtoD = (ADC_GetConversionValue(ADC1));
           // printf("A2D Value= %x \n\r",Temp_A2D);
             /* Start ADC1 Software Conversion */ 
            ADC_SoftwareStartConvCmd(ADC1, ENABLE);
            CoTickDelay(100);
    }else{
      CoTickDelay(5);
    } 
   
#endif   
#ifdef BC_HW_REVC
        // enable a to d battery input
    HwGPOHigh(GPO_VBATT_ADC_EN);
        // wait 1 for input to stabilise
    CoTickDelay(1);
        // trigger a to d
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        // wait 1 for data sample
    CoTickDelay(2);
        // get sample data, disable input
    BattUnion.Battery_AtoD = (ADC_GetConversionValue(ADC1));
    HwGPOHigh(GPO_VBATT_ADC_EN);
    
    printf("Bv Ato D: %x \n\r", BattUnion.BatteryLevel[1]);
    if(!(config.AtoDon_off)){
       CoSuspendTask(task8Id);  
 }else{
        // wait 100
    CoTickDelay(100);
 }
#endif    
    
 }
  
} 


/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
