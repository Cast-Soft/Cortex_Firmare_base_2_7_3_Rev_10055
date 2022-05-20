/******************** (C) COPYRIGHT 2011 NaturalPoint, Inc. ********************
* File Name          : main.c
* Author             : ?
* Version            : V1.0
* Date               : 6/2/2011
* Description        : Main Application for BlackTrax:TimeKeeper
*******************************************************************************/

/*
    LED1 : Toggle on RF Packet TX
    LED2 : Toggle on 50x RF Packet TX'es
    LED3 : Toggle on RF Packet RX
    LED4 : Toggle on 50x RF Packet RX'es
    LED5 : Toggle on 50x OH-Sync Packet TX'es
*/

/* INCLUDES ------------------------------------------------------------------*/

#include "tk_version.h"
#include "hardware.h"
#include "CoOS.h"
#include "basic_rf.h"
#include "stm32f10x_it.h"
#include <stdlib.h>
#ifndef STDIO_TO_USART
#include "usb_lib.h"
#include "usb_pwr.h"
#include "usb_hw_config.h"
#endif

#include "ether_1.h"
#include "ethernet.h"
#include "tst_dbg.h"

#include <stdio.h>
#include <string.h>
#include <yfuns.h>
#include "radio.h"
#include "radio_defs.h"
#include "packets.h"
#include "beacon.h"
#include "FlashUpdate.h"
#include "stm32f10x_flash.h"
#include "main.h"


/* PRIVATE TYPEDEF -----------------------------------------------------------*/

typedef enum {
    DEC, HEX, BIN
} OutputMode_t;



/* PRIVATE DEFINES -----------------------------------------------------------*/

#define OS_STACK_SIZE   128
#define MY_PAN_ID       0x0001
#define MY_ADDR         0xABCD
#define MY_RF_CHAN      15
#define PINGER_ADDR     0x1234
#define TESTBEACONADDR  0x001

#define FLASH_CONFIG_PAGE   0x0803F800   // Page#127 (last page of main memory)

#ifdef TK_V3_hdwr
#define HW_Version 'C'
#endif
#ifdef TK_V2_hdwr
#define HW_Version 'B'
#endif

/* PRIVATE MACROS ------------------------------------------------------------*/

/* EXTERN VARIABLES ----------------------------------------------------------*/

/* PRIVATE VARIABLES ---------------------------------------------------------*/

static OS_TID task1Id;
static OS_TID task2Id;
static OS_TID task3Id;
static OS_TID task4Id;
static OS_TID EthertaskId;
static OS_TID StarttskId;
static OS_TID TxTempTaskId;


static OS_STK task1Stack[OS_STACK_SIZE];
static OS_STK task2Stack[OS_STACK_SIZE];
static OS_STK task3Stack[OS_STACK_SIZE];
static OS_STK task4Stack[OS_STACK_SIZE];
static OS_STK EthertaskStack[OS_STACK_SIZE];
static OS_STK StarttskStack[OS_STACK_SIZE];
static OS_STK TxTempTaskStack[OS_STACK_SIZE];

const uint8_t date_str[] = {__DATE__};
const uint8_t time_str[]={__TIME__};
static uint16_t txCount;
static uint8_t txTestStr[] = {0xAA, 0x55, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};

static uint8_t TxmultB[128];

static OutputMode_t outputMode = DEC;
static uint16_t decimMask = 0x0007; // 1:8 decimation

static uint8_t txBuf[FRAME_LENGTH_MAX - MHR_SIZE - MFR_SIZE];   // tx-payload buffer

static uint16_t txPktCount = 0;
static uint16_t rxPktCount = 0;
             
//static uint16_t rfBeaconEn = 1;
uint16_t rfBeaconEn = 1;
static uint8_t rfChannel = MY_RF_CHAN;

extern struct Disc_Packet D_Packet;

extern struct Ether_status *pEther_status;
extern struct GenericSendHddr *pGenericSendHddr;
extern uint32_t BTHostTimeOut;
void *Memtest,*LastMem,*FirstMem;
extern uint32_t IMU_Dbg_Prt;

RadioTemp_Union_t  RadioTemperature, *pRadioTemperature;

/*PUBLIC VARIABLES ----------------------------------------------------------*/

OS_FlagID flagRFBeaconSent;



config_t config = {
    .productID      = 0xCD10,
    .serialNum      = 0x555,
    .panId          = 0x0001,
    .mySrcAddr      = 0xABCD,
    .rfChan         = 0,    // index of RF channel
    .rfChanMin      = 11,
    .rfChanMax      = 26,
    .TxPower        = 0,    // index of tx power
      .TestMode       = 0x00,
    .RxPrint         = 0x00

};



/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/

//static void OsStackPaint(OS_STK *stk, unsigned short size);
//static unsigned short OsStackCheck(OS_STK *stk);
static void Task1 (void* pdata);
static void Task2 (void* pdata);
static void Task3 (void* pdata);
static void Task4 (void* pdata);
static void Ethertask (void* pdata);
static void StartTask (void* pdata);
static void TxTemp (void* pdata);

static void OutputHex (uint16_t *data);
static void OutputDec (uint16_t *data);
static void OutputBin (uint16_t *data);

static void mygets(char *str);
static uint16_t CheckConfigChecksum(void);
extern void WriteConfig(void);
void LedSet(uint16_t);


/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/
void WriteConfig(void);
static uint16_t CalcConfigChecksum(void);
#if 0
/*******************************************************************************
* Description    :
* Input          :
* Return         :
*******************************************************************************/
static void OsStackPaint(OS_STK *stk, unsigned short size) {
    while (size--) {
        *stk++ = 0xABCD;
    }
}

/*******************************************************************************
* Description    :
* Input          :
* Return         :
*******************************************************************************/
static unsigned short OsStackCheck(OS_STK *stk) {
    unsigned short clean_count = 0;
    OS_STK *tmp = stk;

    while (*stk++ == 0xABCD) {
        clean_count++;
    }
    for (unsigned int i = 0; i < OS_STACK_SIZE; i++) {
        if (*tmp++ == 0xABCD)
            printf(".");
        else
            pritnf("!");
    }
    return clean_count;
}
#endif


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
 //           tasksWDT |= 0x0002;
        }
        putchar(c);
        *tmp++ = (char) c;
    } while (c != 0x0D);

    putchar(0x0A);
    *(--tmp) = '\0';
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
* Description : Check FLASH Configuration Struct Checksum
* Input       :
* Return      : Checksum if GOOD, 0 if BAD
*******************************************************************************/
static uint16_t CheckConfigChecksum(void) {
    uint16_t checksum = 0xCCCC; // checksum initialization
    uint16_t *pConfig = (uint16_t*)FLASH_CONFIG_PAGE;

    // checksum is last half-word in struct
    // checksum field not included in checksum
    for (uint16_t i = 0; i < (sizeof(config)/2 - 1); i++) {
        checksum ^= (i ^ *pConfig++);
    }

    if (checksum == *pConfig) {
        return checksum;
    } else {
        return 0;
    }
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
            RadioInit(MY_PAN_ID, MY_ADDR, RFChanValues[config.rfChan]);
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
          case 'h': // read ADC battry voltage
            
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
* Description : Get Configuration
* Input       :
* Return      : -
*******************************************************************************/
static void GetConfig(uint16_t idx, uint16_t val, uint16_t val2) {
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

    printf("\n\n\r FIRMWARE VERSION : %d.%d.%d\n\r", TK_VERSION_MAJOR, TK_VERSION_MINOR, TK_VERSION_REVISION);
    printf(" %s : %s \r\n\n",date_str,time_str);
    printf(
"\
[-] checksum    : %04X\n\r\
[0] productID   : %X\n\r\
[1] serialNum   : %X\n\r\
[2] panId       : %04X\n\r\
[3] mySrcAddr   : %04X\n\r\
[4] rfChan      : %02X\n\r\
[5] rfChanMin   : %02X\n\r\
[6] rfChanMax   : %02X\n\r\
[7] TxPower     : %02X\n\r\
[8] TestMode    : %d\n\r\
[.] IMU Dump    :\n\r"
 ,
        config.checksum,
        config.productID,
        config.serialNum,
        config.panId,
        config.mySrcAddr,
        config.rfChan,
        config.rfChanMin,
        config.rfChanMax,
        TxAmpValues[config.TxPower],
        config.TestMode  
        );
}

/*******************************************************************************
* Description    : [Task] Startup functions for Ethernet comms
* Input          :
* Return         :
*******************************************************************************/
static void StartTask (void* pdata){

 while (1) {  

  
   if(!(pEther_status->BT_Link)){   // see if we have found the BT server
          if(pEther_status->Probe_cnt){
                  discovery_probe();
                  pEther_status->Probe_cnt--;
                  CoTickDelay(100);
          }else{
                discovery_probe();
                CoTickDelay(500);
            }
   }else{
 //       BTHostTimeOut =  TimeoutVal;  // reset timeout
     BTHostTimeOut =  1000;  // reset timeout
        printf("Suspending Start Task\n\r");
        CoSuspendTask(StarttskId);  // found the srver so end this task
        
   }
}
  
} 
/*******************************************************************************
* Description    : [Task] Process Incoming Radio Packets
* Input          :
* Return         :
*******************************************************************************/
static void Task1 (void* pdata){
    static rxPkt_t *pRxPkt;
 //   static uint16_t i;
//    static uint16_t *pWord;
    
static uint8_t count;    
static uint16_t PayloadCount;
    while (1) {
        if (pRxPkt = RadioRxPkt(1)) {   // will wait

#ifdef TK_TestBed
          if(config.RxPrint){
          for (PayloadCount =0; PayloadCount < pRxPkt->payloadSize; PayloadCount++){
          printf("%x, ", pRxPkt->payload[PayloadCount]);
          }
          printf("RSSI:%x \n\r",pRxPkt->payload[pRxPkt->payloadSize]);
          }

#else          
          
          HwLEDToggle(LED3);
            if (rxPktCount++ == 50) {
                HwLEDToggle(LED4);
                rxPktCount = 0;
            }

            if (pRxPkt->srcAddr == PINGER_ADDR) {
                /* Ping Request Packet */
                pRxPkt->payload[pRxPkt->payloadSize] = pRxPkt->rssi;
                pRxPkt->payload[pRxPkt->payloadSize + 1] = pRxPkt->lqi;
                RadioTxPkt(pRxPkt->srcAddr, 0, pRxPkt->payloadSize + 2, pRxPkt->payload, 1);
                HwLEDToggle(LED1);
                if (txPktCount++ == 50) {
                    HwLEDToggle(LED2);
                    txPktCount = 0;
                }
                continue;
            } else if (pRxPkt->payloadSize == (sizeof(txTestStr) + 2)) {
                /* Ping Reply Packet */
  //              printf("PING ECHO: ");
 //               for (int i = 0; i < pRxPkt->payloadSize; i++) {
 //                   printf("%02X ", pRxPkt->payload[i]);
 //               }
 //               printf("(%02X %02X)\n\r", pRxPkt->rssi, pRxPkt->lqi);
 //               continue;
              printf("PING ECHO: Remote RSSI: %02X:",pRxPkt->payload[(pRxPkt->payloadSize)-2]);
                     printf("Local RSSI:%02X", pRxPkt->rssi);
#ifdef Debug_pkt_num
                     printf("Rx'd Packet#:%d",pRxPkt->seqNumber);
#endif
                     printf("\n\r");
               continue; 
              
            }

            /* IMU Data Packet */
#ifdef Ether_active
            ParseBeaconData(pRxPkt);
#else            
            
            pWord = (uint16_t*)pRxPkt->payload;
            // skip first 6-bytes : {seq#, button_state, beaconRSSI, led0Id, led1Id, led2Id}
            pWord = pWord + 3;
            for (i = 0; i < 8; i++) { // 8 samples per packet
                switch (outputMode) {
                    case HEX:
                        OutputHex(pWord);
                        break;
                    case DEC:
                        OutputDec(pWord);
                        break;
                    case BIN:
                        OutputBin(pWord);
                        break;
                }
                pWord += 6; // 6x 16-bit words per IMU sample
            }
#endif  
#endif            
        }
    }
}

/*******************************************************************************
* Description    : [Task] Send (10Hz) RF Sync Packets
* Input          :
* Return         :
*******************************************************************************/
static void Task2 (void* pdata){

    while (1) {
        // RF Sync Packet Payload :
        // {FrameID[7:0], FrameID[15:8], FrameID[23:16], FrameID[31:24]}
        *(uint32_t*)txBuf = rfBeaconFrameId;
        RadioTxPkt(BASIC_RF_BROADCAST_ADDR, 1, 4, txBuf, 0); // beacon frame, delayed-send
        //RadioTxPkt(BASIC_RF_BROADCAST_ADDR, 1, 4, txBuf, 1); // beacon frame, immediate-send
        rfBeaconLoaded = 1; // indicate to ISR ok to STXON

        HwLEDToggle(LED1);
        if (txPktCount++ == 50) {
            HwLEDToggle(LED2);
            txPktCount = 0;
        }

        /* since we did a (delayed) STXON => [CC2520 Bug#1] */
        CoWaitForSingleFlag(flagRFBeaconSent, 0);
        RadioSTXONBugFlush();

        while ((!rfBeaconEn)) {
            CoSuspendTask(task2Id);
            rfBeaconFrameId = 0; // indicate FrameID is stale now
        }
    }
}

/*******************************************************************************
* Description    : [Task] Implements Simple Text Command Console
* Input          :
* Return         :
*******************************************************************************/
static void Task3 (void* pdata){

    static char s[40] = "";
    static int i;
    static char cmd,addr,value2;
    static int  value;  
  

    while (1) {
  
        // get a line of values from stdin
        printf("> ");
        mygets(s);

        // parse the line of hexadecimal values
//        i = sscanf(s, "%c %x %x", &cmd, &addr, &value);
        i = sscanf(s, "%c %c %x %c", &cmd, &addr, &value, &value2);
        if (i < 1) continue;
        
//        switch (getchar()) {
        switch (cmd) {
            case 'd': // Display configuration
                if (i != 1) continue;
                PrintConfig();
                break;
        case 's': // Set configuration
              //  if (i != 3) continue;
//                if (i != 2) continue;
                SetConfig(addr, value, value2 );
                break;

         case 'r': //Get configuration
              //  if (i != 3) continue;
//                if (i != 2) continue;
               GetConfig(addr, value, value2 );
                break;
        
        
        case 'v': // saVe configuration
                if (i != 1) continue;
                WriteConfig();
                break;     
                
            case '(':  // decrement RF channel
              if(config.rfChan <= config.rfChanMin){
               config.rfChan = config.rfChanMax;
               RadioSetRFChan(config.rfChan);  
              }else{
                 -- config.rfChan;
              RadioSetRFChan(config.rfChan);
              }
                printf("RF CH: %X\n\r", config.rfChan);
              
                break;
            case ')': // incrementRF channel
                if(config.rfChan >= config.rfChanMax){
               config.rfChan = config.rfChanMin;
               RadioSetRFChan(config.rfChan);  
              }else{
                 ++config.rfChan;
              RadioSetRFChan(config.rfChan);
              }
                printf("RF CH: %X\n\r", config.rfChan);
              
                break;
               
            case 'c':  // enable/ disable sysnc transmit
                rfBeaconEn ^= 1;
                printf("RF Beacon %s\n\r", rfBeaconEn ? "ENABLED" : "DISABLED");
                if (rfBeaconEn) {
                    RadioSetPanIdShortAddr(MY_PAN_ID, MY_ADDR); // change back addr
                    CoAwakeTask(task2Id);
                }
                break;
            case '-':  // dec Tx power
               if(config.TxPower ==0){
                config.TxPower =8; // sizeof(uint8_t TxAmpValues);
              }else{
                config.TxPower--;
              }
              RadioSetRFLevel(config.TxPower);
                printf("RF Tx Level: %X\n\r", TxAmpValues[config.TxPower]);
                break;
            case '+':  // increment Tx power
                if(config.TxPower >= 8){
                config.TxPower = 0; // sizeof(uint8_t TxAmpValues);
              }else{
                config.TxPower++;
              }
              RadioSetRFLevel(config.TxPower);
                printf("RF Tx Level: %X\n\r", TxAmpValues[config.TxPower]);
                break;
            case 'x':   // hexadecimal output
                outputMode = HEX;
                break;
            case 'o':   // signed-decimal output
                outputMode = DEC;
                break;
            case 'b':   // binary output
                outputMode = BIN;
                break;
            case '1':   // no decimation
                decimMask = 0x0000;
                break;
            case '2':   // 1:2 decimation
                decimMask = 0x0001;
                break;
            case '4':   // 1:4 decimation
                decimMask = 0x0003;
                break;
            case '8':   // 1:8 decimation
                decimMask = 0x0007;
                break;
                
            case 'p': // Ping TK 1x
                if (i != 2) continue;
                printf("PING REQUESTED : %d\n\r", addr);
                rfBeaconEn =0;
                txCount = addr;
                CoAwakeTask(task4Id);
                break;
        
             case '.':  // dump IMU data as rxd from Beacon
                printf("IMU Debug Print out: \n\r");
                IMU_Dbg_Prt = (!(IMU_Dbg_Prt));
                break;   
              
              case '!': // special message
                printf("Mat Rocks!\n\r");
        case '=': // enable 2520 temperature readout
          
              break;
                
        default:
          break;
        }
    }
}

/*******************************************************************************
* Description    : [Task] Pinger
* Input          :
* Return         :
*******************************************************************************/
static void Task4(void* pdata){
    while (1) {
        RadioSetPanIdShortAddr(MY_PAN_ID, PINGER_ADDR); // change addr
        memcpy((void *)txBuf, (void *)txTestStr, sizeof(txTestStr));

        rfBeaconEn =0;
        while (txCount) {
            RadioTxPkt(BASIC_RF_BROADCAST_ADDR, 0, sizeof(txTestStr), txBuf, 1);
            HwLEDToggle(LED1);
            if (txPktCount++ == 50) {
                HwLEDToggle(LED2);
                txPktCount = 0;
            }
            txCount--;
            CoTickDelay(10);
        }

        //RadioSetPanIdShortAddr(MY_PAN_ID, MY_ADDR); // change back addr
        rfBeaconEn =1;
        RadioSetPanIdShortAddr(MY_PAN_ID, MY_ADDR); // change back addr
        CoAwakeTask(task2Id);
        CoSuspendTask(task4Id);
    }
}

/*****************************************************************************/
//Description    : Ethernet task
// Input          :
// Return         : -
/***************************************************************************/
static void Ethertask(void* pdata){

    while (1) {
        
            uIP_Loop();
           
        if (BTHostTimeOut){
          if(!(-- BTHostTimeOut)){
  //          printf("BTHost EtherTimeout \n\r");
            pEther_status->BT_Link=0;
            pEther_status->Probe_cnt=10;
            CoAwakeTask(StarttskId);
        }
          
        }      
         
     CoTickDelay(1);
        
    }
}
/*****************************************************************************/
//Description    : Transmiter temperature task
// Input          :
// Return         : -
/***************************************************************************/
static void TxTempTask(void* pdata){

    while (1) {
    if(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)== SET){
            RadioTemperature.RadioTemp_AtoD = (ADC_GetConversionValue(ADC1));
           // printf("A2D Value= %x \n\r",Temp_A2D);
             /* Start ADC1 Software Conversion */ 
            ADC_SoftwareStartConvCmd(ADC1, ENABLE);
            CoTickDelay(100);
    }else{
      CoTickDelay(5);
    }         

         
     CoTickDelay(100);
       
    }
}

/*******************************************************************************
* Description    : Output in HEX
* Input          :
* Return         : -
*******************************************************************************/
static void OutputHex (uint16_t *data) {
    static uint16_t modulo = 0;
    uint16_t i;

    if ((modulo++ & decimMask) != 0) return;

    for (i = 0; i < 6; i++) {
        printf("%04X", *data++);
    }
    printf("\n\r");
}

/*******************************************************************************
* Description    : Output in DEC
* Input          :
* Return         : -
*******************************************************************************/
static void OutputDec (uint16_t *data) {
    static uint16_t modulo = 0;
    uint16_t i;

    if ((modulo++ & decimMask) != 0) return;

    for (i = 0; i < 6; i++) {
        printf("%+6d ", *(int16_t*)data++);
    }
    printf("\n\r");
}

/*******************************************************************************
* Description    : Output in BIN
* Input          :
* Return         : -
*******************************************************************************/
static void OutputBin (uint16_t *data) {
    static uint16_t modulo = 0;
    uint8_t seqNum = 0x55;

    if ((modulo++ & decimMask) != 0) return;

    __write(_LLIO_STDOUT, (unsigned char*)data, 12); // 6x 2-Byte words per IMU sample
    __write(_LLIO_STDOUT, (unsigned char*)&seqNum, 1);
    //seqNum++;
}

/* PUBLIC FUNCTION PROTOTYPES ------------------------------------------------*/

/*******************************************************************************
* Description    : Main routine.
* Input          : -
* Return         : -
*******************************************************************************/
void  main(void) {

    SysTick_Config(SystemCoreClock / 100); // SysTick Interrupt Freq = (1/100)s = 10ms

    HwPeriphInit();
    

#ifndef STDIO_TO_USART
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Timer_Config();
    USB_Init();
#endif


 
    GetARM_UUID(); // init and load the UUID into the struct
    uIP_timer_setup(); // get the timer started       
    
    printf("\n\r TK Version:: %d.%d.%d\n\r\n\r", TK_VERSION_MAJOR, TK_VERSION_MINOR, TK_VERSION_REVISION); 
    printf("  %s %s \n\r",date_str,time_str);
    printf("Hardware version %c \n\r",HW_Version);
    
       
//   Ethernet_Test();    // setup the ethernet i/o hardware   

//   uIP_Setup();    // Init the stack
 
//    init_ether_comm();
     

 
 //   if (CheckConfigChecksum()) {
 //       memcpy((void *)&config, (void *)FLASH_CONFIG_PAGE, sizeof(config));
 //   } else {
 //       printf("**ERROR** FLASH Configuration Checksum Bad!!\n\r");
  //      WriteConfig();
  //  }
    
   CoInitOS();

      RadioInit(MY_PAN_ID, MY_ADDR, MY_RF_CHAN);

   flagRFBeaconSent = CoCreateFlag(1, 0);  // auto-reset, flag cleared

#if 0
    OsStackPaint(task1Stack, OS_STACK_SIZE);
    OsStackPaint(task2Stack, OS_STACK_SIZE);
    OsStackPaint(task3Stack, OS_STACK_SIZE);
    OsStackPaint(task4Stack, OS_STACK_SIZE);
#endif
    
//    task1Id = CoCreateTask  (Task1, (void*)0, 0, &task1Stack[OS_STACK_SIZE-1], OS_STACK_SIZE);
    task1Id = CoCreateTaskEx (Task1, (void*)0, 0, &task1Stack[OS_STACK_SIZE-1], OS_STACK_SIZE,(uint32_t)0, (uint32_t)0);
//    task2Id = CoCreateTaskEx(Task2, (void*)0, 1, &task2Stack[OS_STACK_SIZE-1], OS_STACK_SIZE, (uint32_t)0, (uint32_t)0);
    task2Id = CoCreateTaskEx(Task2, (void*)0, 1, &task2Stack[OS_STACK_SIZE-1], OS_STACK_SIZE, (uint32_t)0, (uint32_t)1);

    task3Id = CoCreateTask  (Task3, (void*)0, 9, &task3Stack[OS_STACK_SIZE-1], OS_STACK_SIZE);
 
    task4Id = CoCreateTaskEx(Task4, (void*)0, 9, &task4Stack[OS_STACK_SIZE-1], OS_STACK_SIZE, (uint32_t)0, (uint32_t)1);
    EthertaskId = CoCreateTaskEx(Ethertask, (void*)0,10, &EthertaskStack[OS_STACK_SIZE-1], OS_STACK_SIZE,(uint32_t)0, (uint32_t)1);
    StarttskId=CoCreateTaskEx(StartTask, (void*)0,1, &StarttskStack[OS_STACK_SIZE-1], OS_STACK_SIZE,(uint32_t)0, (uint32_t)1);
    TxTempTaskId= CoCreateTaskEx(TxTempTask,(void*)0,10, &TxTempTaskStack[OS_STACK_SIZE-1], OS_STACK_SIZE,(uint32_t)0, (uint32_t)1); 

    CoStartOS();

    while (1);
}

#if (defined USE_MY_ASSERT) || (defined USE_FULL_ASSERT)
/*******************************************************************************
* Description    : Reports the name of the source file and the source line number
*                  where the assert error has occurred.
* Input          :
* Return         : -
*******************************************************************************/
void assert_failed(uint8_t *file, uint32_t line) {
#ifdef COOS
    CoSchedLock();
#endif
    while (1) {
        printf("!!!ASSERT FAILED!!! %s : %d\n\r", file, line);
        HwLEDOn(LED1); HwLEDOn(LED4); HwLEDOff(LED2); HwLEDOff(LED3);
        HwWait(50);
        HwLEDOn(LED2); HwLEDOn(LED3); HwLEDOff(LED1); HwLEDOff(LED4);
        HwWait(50);
    }
}
#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/