/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
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

#include "hardware.h"
#include "CoOS.h"
#include "basic_rf.h"
#include "stm32f10x_it.h"
#include "stm32f10x_iwdg.h"
#include <stdlib.h>

#include "usb_regs.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "ether_1.h"
#include "ethernet.h"
#include "tst_dbg.h"
#include "i2c_ee.h"
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
#include "flash_map.h"
#include "util.h"
#include <ctype.h>
#include "i2c_ee.h"
#include "console_tail.h"
#include "VersionNo.h"

extern uint32_t magicNumber;
extern uint32_t rxErrors;
extern uint8_t rxCount;
extern realTime rxWaitTime;
extern realTime rxInTime;
extern realTime rxFIFOTime;
extern uint8_t exc0;
extern uint8_t exc1;
extern uint8_t exc2;
extern uint16_t rxLine;

extern uint32_t hf_counter;

/* PRIVATE TYPEDEF -----------------------------------------------------------*/

typedef enum {
    DEC, HEX, BIN
} OutputMode_t;



/* PRIVATE DEFINES -----------------------------------------------------------*/

#define OS_STACK_SIZE   512 //128         // it is word == 4 * 128
#define MY_PAN_ID       0x0001
#define MY_ADDR         0xABCD
#define MY_RF_CHAN      15
#define PINGER_ADDR     0x1234
#define FLASH_CONFIG_PAGE   0x0803F800   // Page#127 (last page of main memory)

#ifdef TK_V3_hdwr
#define HW_Version 'C'
#endif
#ifdef TK_V2_hdwr
#define HW_Version 'B'
#endif

/* PRIVATE MACROS ------------------------------------------------------------*/

/* EXTERN VARIABLES ----------------------------------------------------------*/
#if (defined USE_MY_ASSERT) || (defined USE_FULL_ASSERT)
static uint8_t assert_loop = 1;
#endif

/* PRIVATE VARIABLES ---------------------------------------------------------*/

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;

__no_init tDiagnostics       diagnostic;

#ifdef WDT_ENABLE
uint8_t  watchdog_active = 1;
#endif

static OS_STK radioRxStack[OS_STACK_SIZE];
static OS_STK configTaskStack[OS_STACK_SIZE];
static OS_STK EthertaskStack[OS_STACK_SIZE];
static OS_STK WatchDogTaskStack[OS_STACK_SIZE];

OS_TID radioTaskId;
OS_TID configTaskId;
OS_TID etherTaskId;


static uint8_t txBuf[FRAME_LENGTH_MAX - MHR_SIZE - MFR_SIZE];   // tx-payload buffer

static uint16_t rxPktCount = 0;


extern struct Disc_Packet D_Packet;

extern struct Ether_status *pEther_status;
extern struct GenericSendHddr *pGenericSendHddr;
extern uint32_t BTHostTimeOut;
extern uint32_t IMU_Dbg_Prt, SeqNum_Dbg_Prt;
extern uint32_t TxSendError; // dsplay # of times the RX was active into the STXON period
extern uint32_t sec;

uint32_t radioPackets;

/*PUBLIC VARIABLES ----------------------------------------------------------*/

// EtherTask will be waiting and processing msg in queue
#define TASK_MSG_QUEUE_SIZE 40       // TODO!!! test to get a better value
OS_EventID ethTaskMsgQueue;
void *ethTaskMsgQueueBuf[TASK_MSG_QUEUE_SIZE];

MsgHeader wakeupMsgEntity;
MsgHeader* wakeupMsg = &wakeupMsgEntity;


// for outbound ethernet frames
#define TX_MSG_QUEUE_SIZE 12       // TODO!!! test to get a better value
OS_EventID ethTxMsgQueue;
void *ethTxMsgQueueBuf[TX_MSG_QUEUE_SIZE];

OS_EventID semRFRxFrames;   // Number of RF frames received in Rx-FIFO. It is notified by RX_FRM_DONE interrupt
uint8_t task_ticks[3];

config_t config = {
    .config_version     = CURRENT_CONFIG_VERSION,
    .mac_address        = {0x00, 0x25, 0x62, 0x00, 0x0C, 0x03},
    .my_ip              = ((10 << 24) | (133 << 16) | (1 << 8) | 100),
    .dest_ip            = ((2 << 24) | (0 << 16) | (0 << 8) | 200),
    .netmask            = 0xFF000000,
    .my_port            = RT_Listen_Port,
    .dest_port          = RT_Send_Port,
    .productID          = RT_PRODUCT_ID, //0xCD10,
    .serialNum          = 0x555,
    .panId              = 0x23,
    .mySrcAddr          = 0xABCD,
    .rfChan             = 26,
#ifdef _DEBUG
    .flags              = FLAG_TRACE_ENABLE |
#else
    .flags              =
#endif
                        FLAG_ETHERNET_ON |
                        FLAG_TRACE_UDP | FLAG_FRAMEID_24BITS,
};


/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/

//static void OsStackPaint(OS_STK *stk, unsigned short size);
//static unsigned short OsStackCheck(OS_STK *stk);
static void RadioRxTask (void* pdata);
static void ConfigTask (void* pdata);
static void Ethertask (void* pdata);
static void PrintDiagnostic(tDiagnostics *pDiagnostics);
static void mygets(char *str);


/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/

static uint8_t str2byte(char *p, uint8_t *addr, uint8_t iter)
{
  char *pp = NULL;
  for (int i = 0; i < iter; i++) {
    while (*p && !isdigit(*(p))){p++;}
    if (!*p) {
      return 0;
    }
    *(addr+i) = strtoul(p, &pp, 10);
    p = pp;
  }
  return 1;
}

/*******************************************************************************
* Description : Get a line from STDIN
* Input       :
* Return      : -
*******************************************************************************/
static void mygets(char *str) {
    char *tmp = str;
    int c;

    func = __func__;
    do {
      line = __LINE__;
        while( (c = getchar()) == EOF ) {
            // 100ms => humans don't type >10 char/sec
          func = __func__;
          line = __LINE__;
            CoTickDelay(100);
            task_ticks[configTaskId - 1] = 0;
            RELOAD_WATCHDOG
        }
      line = __LINE__;
        putchar(c);
      line = __LINE__;
        *tmp++ = (char) c;
    } while (c != 0x0D);
      line = __LINE__;

    putchar(0x0A);
      line = __LINE__;
    *(--tmp) = '\0';
      line = __LINE__;
}


/*******************************************************************************
* Description : Set Configuration
* Input       :
* Return      : -
*******************************************************************************/
void SetConfig(/*uint16_t idx, uint16_t val*/ const char *cmd) {
   char *p = NULL;
    uint16_t idx = strtoul(cmd, &p, 10);
    uint16_t val = 0;
    func = __func__;
    if (idx < 20 || idx == 22) {
      while (*p && isspace(*(++p)));
      if (!*p) {
        return;
      }
      val = strtoul(p, NULL, 10);
    }
    switch (idx) {
        case 0x0:
            config.productID = val;
            break;
        case 0x1:
            config.serialNum = val;
            break;
        case 0x2:
            config.panId = val;
            RadioSetPanIdShortAddr(config.panId, config.mySrcAddr);
            break;
        case 0x3:
            config.mySrcAddr = val;
            RadioSetPanIdShortAddr(config.panId, config.mySrcAddr);
            break;
        case 0x4:
            if(val >= OLD_RF_CHANNEL_MIN && val <= OLD_RF_CHANNEL_MAX)
            {
              config.rfChan = val;
              RadioSetRFChan(val);
            }
            break;

        case 0x7:
      //      config.TxPower = val;
           break;
          // For idx >= 20 value may be non-numeric
        case 22:
            if(val >= RF_CHANNEL_MIN && val <= RF_CHANNEL_MAX)
            {
              config.rfChan = val;
              RadioSetRFChan(val);
            }
            break;

       case 30: //my port
         {
           config.my_port = val;
           SaveConfig(&config);
           TRACE("Restart required\n\r");
         }
        break;
        case 31: //dest IP - value in form of '192.168.0.1'
         {
           config.dest_port = val;
           SaveConfig(&config);
           TRACE("Restart required\n\r");
         }
        break;
       case 32: //my IP - value in form of '192.168.0.1'
         {
            uint8_t buf[4];
            if(str2byte(p, buf, 4)) {
              config.my_ip = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
              TRACE("Restart required\n\r");
            } else {
              TRACE("Incorrect format\r\n");
            }
         }
        break;
        case 33: //dest IP - value in form of '192.168.0.1'
         {
            uint8_t buf[4];
            if(str2byte(p, buf, 4)) {
              config.dest_ip = (buf[0] << 24) | (buf[1] << 16) |( buf[2] << 8) | buf[3];
              TRACE("Restart required\n\r");
            } else {
              TRACE("Incorrect format\r\n");
            }
         }
        break;
        case 34: //netmask - value in form of '192.168.0.1'
         {
            uint8_t buf[4];
            if(str2byte(p, buf, 4)) {
              config.netmask = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
              TRACE("Restart required\n\r");
            } else {
              TRACE("Incorrect format\r\n");
            }
         }
        break;
        case 35: //mac address - value in form of '255.255.255.255.255.255'
         {
            uint8_t buf[6];
            if(str2byte(p, buf, 6)) {
              memcpy(&config.mac_address, buf, 6);
              TRACE("Restart required\n\r");
            } else {
              TRACE("Incorrect format\r\n");
            }
         }
        break;
        default:
            TRACE("**ERROR** Unrecognized Configuration Setting\n\r");
            break;
    }
}
/*******************************************************************************
* Description : Print Configuration
* Input       : -
* Return      : -
*******************************************************************************/
static void PrintConfig(void) {

#ifdef _DEBUG
    TRACE("\n\r FIRMWARE VERSION: " STRINGIFY(THIS_MAJOR) "." STRINGIFY(THIS_MINOR) "." STRINGIFY(THIS_PATCH) "." STRINGIFY(THIS_REVISION) " RT DEBUG \n\r\n\r");
#else
    TRACE("\n\r FIRMWARE VERSION: " STRINGIFY(THIS_MAJOR) "." STRINGIFY(THIS_MINOR) "." STRINGIFY(THIS_PATCH) "." STRINGIFY(THIS_REVISION) " RT RELEASE \n\r\n\r");
#endif
    TRACE(" "__DATE__" : "__TIME__" \r\n\n");
    TRACE(" Tx Errors %d \n\n\r",TxSendError);
    TRACE("[0] productID   : %X\n\r", config.productID);
    TRACE("[1] serialNum   : %X\n\r", config.serialNum);
    TRACE("[2] panId       : %04X\n\r", config.panId);
    TRACE("[3] mySrcAddr   : %04X\n\r", config.mySrcAddr);
    TRACE("[4] rfChan      : %02X\n\r", config.rfChan);
    TRACE("MAC address: %02X-%02X-%02X-%02X-%02X-%02X\n\r", config.mac_address[0],
          config.mac_address[1], config.mac_address[2], config.mac_address[3],
          config.mac_address[4],config.mac_address[5]);
    CoTickDelay(10);
    TRACE("My IP: %u.%u.%u.%u Dest IP: %u.%u.%u.%u\n\r", (config.my_ip >> 24),
          (config.my_ip >> 16) &0xFF, (config.my_ip >> 8) &0xFF, config.my_ip & 0xFF,
          (config.dest_ip >> 24), (config.dest_ip >> 16) &0xFF,
          (config.dest_ip >> 8) &0xFF, config.dest_ip & 0xFF);

    TRACE("Netmask: %u.%u.%u.%u My Port: %u Dest Port: %u\n\r",
          (config.netmask >> 24),
          (config.netmask >> 16) &0xFF, (config.netmask >> 8) &0xFF,
          config.netmask & 0xFF, config.my_port, config.dest_port);
    TRACE("Trace flags=0x%08X\r\n", config.flags);
    TRACE("\nBuilt on "__DATE__" "__TIME__"\n\r" );
}

/*******************************************************************************
* Description    : [Task] Process Incoming Radio Packets
* Input          :
* Return         :
*******************************************************************************/
static void RadioRxTask (void* pdata){
    static rxPkt_t *pRxPkt;
 //   static uint16_t i;
//    static uint16_t *pWord;


    while (1) {
        RELOAD_WATCHDOG
        SAVE_POINT
        rxLine = __LINE__;
        RadioWaitGrabSPI();
        SAVE_POINT
        rxLine = __LINE__;
        pRxPkt = RadioRxPkt();
        SAVE_POINT
        rxLine = __LINE__;
        RadioReleaseSPI();
        task_ticks[radioTaskId - 1] = 0;
        if (pRxPkt != NULL) {
            rxLine = line = __LINE__;
            HwLEDToggle(LED3);
            if (rxPktCount++ == 50) {
                HwLEDToggle(LED4);
                rxPktCount = 0;
            }

            if (pRxPkt->srcAddr == PINGER_ADDR) {
                continue;       // Do not anything in this version. RF does not transmit
            } else {
                  radioPackets++;
                  rxLine = line = __LINE__;
                  // Trace battery packets
                  if (pRxPkt->payload[1] == 0xBA) {
                    if( pRxPkt->payloadSize == sizeof(struct Beacon_BatData)) {
                      struct Beacon_BatData *battData = (struct Beacon_BatData*) pRxPkt->payload;
                      if (config.flags & FLAG_TRACE_ADC) {
                      TRACE("Bat (src %d): %d.%02d V %d percents %s charging\n\r", pRxPkt->srcAddr,
                        battData->voltiCents/100, battData->voltiCents%100, battData->percents,
                        ((battData->flags & 0x03) == 0x03)?"":"not");
                      }
                    }
                  }
                  rxLine = line = __LINE__;
                  uint8_t buf[sizeof(struct TKFrame) + sizeof(struct TKBCPacketForwardAny)];
                  // drop packets from timekeeper
                  // timekkeper will have srcAddr > 40000
                  if (pRxPkt->srcAddr < 40000 /*!= config.mySrcAddr*/ &&
                      pRxPkt->destAddr == config.mySrcAddr &&
                        pRxPkt->panId == config.panId) {
                    struct TKFrame *pFrame = (struct TKFrame*) &buf[0];
                    struct TKBCPacketForwardAny *pFa = (struct TKBCPacketForwardAny*) &buf[sizeof(struct TKFrame)];

                    uint16_t faSize = sizeof(buf);// - sizeof(pFa->payload) + pRxPkt->payloadSize;
                    pFa->header.packetID = IncrementPacketId();
                    pFa->header.contentType = BC_PT_TKBCForwardAny;
                    pFa->header.contentLength = sizeof(struct TKBCPacketForwardAny);// - sizeof(pFa->payload) + pFa->payloadSize;
                    pFa->msgType = 'FABC';
                    pFa->routerId = 0; //TODO
                    pFa->destAddr = pRxPkt->destAddr;
                    pFa->srcAddr = pRxPkt->srcAddr;
                    pFa->seqNumber = pRxPkt->seqNumber;
                    pFa->payloadSize = pRxPkt->payloadSize;
                    memcpy((unsigned char*) &pFa->payload[0], pRxPkt->payload, pRxPkt->payloadSize);
                    pFrame->contentLength = faSize/* - sizeof(struct TKFrame)*/;
                    pFrame->lostRequests = 0; //lostRequests; //never increments
                    pFrame->magicNumber = magicNumber;
                    pFrame->signature = TK_FRAME_SIGNATURE;
                    pFrame->timekeeperID = pGenericSendHddr->timekeeperID;
                    pFrame->version_build = RT_VERSION_BUILD;
                    pFrame->version_major = RT_VERSION_MAJOR;
                    pFrame->version_minor = RT_VERSION_MINOR;
                    pFrame->version_revision = RT_VERSION_REVISION;
                    rxLine = line = __LINE__;
                    EthAsyncSendPacket((unsigned char*) &buf[0], faSize);
                    func = __func__;
                    rxLine = line = __LINE__;
                  }
                  continue;
            }
        }
    }
}

/*******************************************************************************
* Description : Write Configuration to FLASH
* Input       : -
* Return      : -
*******************************************************************************/
/*
typedef struct {
  uint32_t address;
  uint16_t count;
  uint8_t data[PAGE_SIZE];
}tPage;
*/
#define FLASH_NO_ADDRESS_CHECK

uint32_t WritePage(tPage *page) {

  uint8_t *pointer = (uint8_t*) page->address;
  uint32_t result = 0;
  uint16_t cycles = (page->count + 3)/4;

  FLASH_Unlock();

  do {
#ifndef FLASH_NO_ADDRESS_CHECK
    if (page->address < APP_UPLOAD_ADDRESS || page->address > 0x803FFFF) {
      page->count = 0;
      result = ERROR_FLASH_ADDRESS;
      break;
    }
#endif
    if ((result = FLASH_ErasePage(page->address)) != FLASH_COMPLETE) {
      page->count = 0;
      break;
    }

    result = 0;
    for (uint16_t i = 0; i < cycles; i++) {
      uint32_t offset = i << 2;
      uint32_t word = page->data[offset + 3] << 24;
      word |= page->data[offset + 2] << 16;
      word |= page->data[offset + 1] << 8;
      word |= page->data[offset];

        if ((result = FLASH_ProgramWord(page->address + offset, word)) != FLASH_COMPLETE) {
            page->count = offset;
            break;
        }
    }
    result = 0;
    // verification
    for (uint16_t i = 0; i < page->count; i++) {
      if (*(pointer) != page->data[i]) {
        result = ERROR_FLASH_VERIFICATION;
        page->count = i;
        break;
      }
      pointer++;
    }

  } while (0);

  FLASH_Lock();

  return result;
}

#if 1

uint32_t ReadPage(tPage *page) {

  uint8_t *pointer = (uint8_t *) page->address;
  uint32_t result = 0;


  do {
#ifndef FLASH_NO_ADDRESS_CHECK
    if (page->address < 0x8000000 || page->address > 0x803FFFF) {
      page->count = 0;
      result = ERROR_FLASH_ADDRESS;
      break;
    }
#endif
    // verification
    for (uint16_t i = 0; i < page->count; i++) {
      page->data[i] = *pointer;
      pointer++;
    }

  } while (0);

  return result;

}
#endif

/**
  * @brief  Download a file via serial port
  * @param  None
  * @retval None
  */
uint8_t tab_1024[1024];

static tPage page = {FLASH_PROD_AREA};
//static uint32_t CurrentUploadAddress = APP_UPLOAD_ADDRESS;
static uint16_t CurrentDataPointer = 0x00;
//static uint8_t mem_data[PAGE_SIZE];
static uint16_t packet = 0;

uint32_t SerialDownload(const char* decoded, uint16_t len, uint8_t type, uint16_t size)
{
  struct FirmwarePacketHeader *frmHeader;

  uint8_t copy_len = 0;

  uint32_t ret = 0;
  // Data comes in format:
  //U 0 0 DATA\r\n
  // so, DATA starts from 6th index
  do {
    if (len < (sizeof(struct FirmwarePacketHeader))) {
      ret = ERROR_WRONG_PACKET;
      break;
    }
    frmHeader = (struct FirmwarePacketHeader *) (decoded);
    copy_len = size - /*7*/ (sizeof(struct PacketHeader) + sizeof(struct FirmwarePacketHeader));
    if (
        ((type == DEV_CMD_SET_PROD_AREA) &&
        (copy_len > 128 || (copy_len + CurrentDataPointer) > (FLASH_PROD_AREA + FLASH_PROD_AREA_SIZE)))
        ||
        frmHeader->index != packet) {
      ret = ERROR_WRONG_PACKET;
      break;
    }
    if (copy_len) {
      memcpy(&page.data[CurrentDataPointer], (decoded + /*4*/ sizeof(struct FirmwarePacketHeader)), copy_len);
      CurrentDataPointer += copy_len;
      page.count += copy_len;
      if ((frmHeader->index + 1) == frmHeader->count || CurrentDataPointer == sizeof(page.data)) {
        //time to write
        WritePage(&page);
        page.address = FLASH_PROD_AREA;
        CurrentDataPointer = 0;
        page.count = 0;
        if ((frmHeader->index + 1) == frmHeader->count) {
          // finished
          page.address = FLASH_PROD_AREA;
        }
      }
    }
    ret = frmHeader->index << 16;
  } while (0);
  // reset for error
  if (ret & 0xFFFF) {
    CurrentDataPointer = 0;
    page.address = FLASH_PROD_AREA;
    page.count = 0;
    packet = 0;
  } else {
    packet++;
  }
  return ret;
}


#define AIRCR_VECTKEY_MASK    ((u32)0x05FA0000)

/*******************************************************************************
* Function Name  : NVIC_GenerateSystemReset
* Description    : Generates a system reset.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_GenerateSystemReset(void)
{
  SCB->AIRCR = AIRCR_VECTKEY_MASK | (u32)0x04;
}

char *s;
char buf[260] = "";
/*******************************************************************************
* Description    : [Task] Implements Simple Text Command Console
* Input          :
* Return         :
*******************************************************************************/
static void ConfigTask (void* pdata){

    s = buf;
    char cmd;
    static int addr;
    uint32_t value;

    while (1) {

        func = __func__;
        line = __LINE__;
        s = buf;
        // get a line of values from stdin
        mygets(s);
        if (ValidateCommandLine(s, strlen(s)) == 0) {
          TRACE("Incorrect packet\r\n");
          continue;
        }        
        line = __LINE__;
        cmd = *s;
        switch (cmd) {
            case 'd': // Display configuration
                PrintConfig();
                break;
        case 's': // Set configuration
                while (*s && isspace(*(++s)));
                if (*s) {
                  SetConfig(s);
                }
                break;
          case 'v': // saVe configuration
                SaveConfig(&config);
                break;
            case '(':  // decrement RF channel
              if(config.rfChan <= RF_CHANNEL_MIN)
              {
               config.rfChan = RF_CHANNEL_MAX;
               RadioSetRFChan(config.rfChan);
              }
              else
              {
                --config.rfChan;
                RadioSetRFChan(config.rfChan);
              }
                TRACE("RF CH: %X\n\r", config.rfChan);

                break;
            case ')': // incrementRF channel
                if(config.rfChan >= RF_CHANNEL_MAX)
                {
                  config.rfChan = RF_CHANNEL_MIN;
                  RadioSetRFChan(config.rfChan);
                }
                else
                {
                  ++config.rfChan;
                  RadioSetRFChan(config.rfChan);
                }
                TRACE("RF CH: %X\n\r", config.rfChan);

                break;

              case '!': // special message
                NVIC_GenerateSystemReset();
                break;
              case '~': // Sends test ether packet

                break;
            case 't':   // trace / debug
                {
                  char *p = NULL;
                  while (*s && isspace(*(++s)));
                       if (!*s) {
                         break;
                       }
                    addr = strtoul(s, &p, 10);
                  while (*p && isspace(*(++p)));
                       if (*p) {
                        value = strtoul(p, NULL, 10);
                       } else {
                        value = 0;
                       }

                  switch (addr) {
                  case 0: {
                        config.flags ^= FLAG_TRACE_ENABLE;
                      SaveConfig(&config);
                    }
                    break;
                  case 8: {
                        struct WhoAmI me;
                        memcpy(me.id, (uint8_t *)(0x1FFFF7E8), 12);
                        me.type = 2;
                        me.module = 2;
                        __writeCmdLineRespPacket((uint8_t*) &me, sizeof(me), DEV_RESP_WHOAMI);
                    }
                    break;
                  case 9: {
                      config.flags ^= FLAG_TRACE_CRC;
                      SaveConfig(&config);
                    }
                    break;
                  case 11: {
                      config.flags ^= FLAG_TRACE_ADC;
                      SaveConfig(&config);
                    }
                    break;
                  case 49: {
                      TRACE(": realTime %d.%d rxErrors=%d rxCount=%d\n\r", sec, TIM1->CNT, rxErrors, rxCount);
                      TRACE(": rxWait %d.%d rxIn %d.%d\n\r", rxWaitTime.sec, rxWaitTime.uSec, rxInTime.sec, rxInTime.uSec);
                      TRACE(": rxFIFOTime %d.%d\n\r", rxFIFOTime.sec, rxFIFOTime.uSec);
                      CoTickDelay(10);
              //        TRACE("UDP: dropped_packets=%u accepted_packets=%u\n\r", dropped_packets, accepted_packets);
                      TRACE("Radio: accepted_packets=%u\r\n", radioPackets);
                    }
                    break;
                  case 52: {
                      uint32_t ram = (uint32_t) value & 0xFFFFFFFE;
                      if (ram >= 0x20000000 && ram <= 0x2000ffff) {
                        uint32_t atRam = *((uint32_t*) (ram));
                        TRACE("Mem at 0x%08X=0x%08X\n\r", ram, atRam);
                      } else {
                        TRACE("Mem addr error\n\r");
                      }
                    }
                    break;
                 case 96:
                    {
                      tDiagnostics diag;
                      I2C_EE_BufferRead((uint8_t*) &diag, EEPROM_DEBUG_DIAGNOSTIC, sizeof(diag));
                      PrintDiagnostic(&diag);
                    }
                  break;
                  case 97:
                    {
                      PrintDiagnostic(&diagnostic);
                    }
                    break;

#ifdef TASKS_PROFILE

                  case 98: {
                      __disable_interrupt();


                      uint32_t perfIdle         = CoGetTaskScheduledCount(0);
                      uint32_t stackRadio            = CoGetStackDepth(radioTaskId);
                      uint32_t perfRadio        = CoGetTaskScheduledCount(radioTaskId);
                      uint16_t lineRadio        = CoGetTaskLine(radioTaskId);
                      const char *funcRadio     = CoGetTaskFunc(radioTaskId);
                      uint32_t stackConf            = CoGetStackDepth(configTaskId);
                      uint32_t perfConf         = CoGetTaskScheduledCount(configTaskId);
                      uint16_t lineConf         = CoGetTaskLine(configTaskId);
                      const char *funcConf      = CoGetTaskFunc(configTaskId);
                      uint32_t stackEther            = CoGetStackDepth(etherTaskId);
                      uint32_t perfEther         = CoGetTaskScheduledCount(etherTaskId);
                      uint16_t lineEther         = CoGetTaskLine(etherTaskId);
                      const char *funcEther      = CoGetTaskFunc(etherTaskId);
                      __enable_interrupt();
                      extern const char *spi_func;
                      extern const char *old_func;
                      extern uint16_t old_line;
                      extern uint16_t old_spi_line, spiLine;
                      TRACE("old_func=%s:%d\n\r", old_func, old_line);
                      TRACE("spi_func=%s:%d spiLine=%d\n\r", spi_func, old_spi_line, spiLine);
                      TRACE("rxLine=%d exc0=0x%02X exc1=0x%02X exc2=0x%02X\n\r",
                            rxLine, exc0, exc1, exc2);
                      TRACE("IdleTask: %d \n\r");
                      TRACE("RadioTask [avail stack:%d]:%d @ %s():%d\n\r", stackRadio, perfRadio, funcRadio, lineRadio);
                      TRACE("ConfTask [avail stack:%d]:%d @ %s():%d\n\r", stackConf, perfConf, funcConf, lineConf);
                      TRACE("EtherTask [avail stack:%d]:%d @ %s():%d\n\r", stackEther, perfEther, funcEther, lineEther);
                    }
                    break;
                  case 99:
                    {
                      uint32_t scheduled_idle = CoGetTaskScheduledCount(0);
                      uint32_t scheduled_RadioTask = CoGetTaskScheduledCount(radioTaskId);
                      uint32_t scheduled_ConfTask = CoGetTaskScheduledCount(configTaskId);
                      uint32_t scheduled_EtherTask = CoGetTaskScheduledCount(etherTaskId);
                      TRACE("PERF: IdleTask: %u RadioTask: %u\n\r", scheduled_idle, scheduled_RadioTask);
                      TRACE("PERF: ConfTask: %u EtherTask: %u\n\r", scheduled_ConfTask, scheduled_EtherTask);

                    }
                    break;
#endif
                  }
                }

                break;
                        case 'U':
            {       // Extended command line that use base64 encoded parameters
                static uint8_t encoded[512];
                static uint8_t decoded[512];
                uint8_t *_decoded = &decoded[0];
                uint16_t update_flags = 0;
                char *p = NULL;
                while (*s && isspace(*(++s)));
                if (!*s) {
                  break;
                }

                addr = strtoul(s, &p, 10);
                s = p;
                while (*s && isspace(*(++s)));
                if (!*s) {
                  break;
                }
                uint16_t value = strtoul(s, &p, 10);
                s = p;
                while (*s && isspace(*(++s)));
                if (!*s) {
                  break;
                }
                //i = sscanf(s, "%c %d %d %s", &cmd, &addr, &value, encoded);
                int ret = b64_pton(/*encoded*/ p, decoded, sizeof(decoded));

                if (ret < sizeof(struct PacketHeader)) {
                    TRACE("Incorrect packet\r\n");
                    continue;
                }
                struct PacketHeader *packetHeader = (struct PacketHeader *)decoded;
                if (packetHeader->size > ret) {
                    TRACE("Incorrect packet size\r\n");
                    continue;
                }
		if (ValidateCommandLine((char*)decoded, ret) == 0) {
                    TRACE("Incorrect packet\r\n");
                    continue;
		}
                
                if (packetHeader->type == DEV_CMD_CONFIG_REQ) {
                  ex_config_t old_config;
                  CopyConfigToExConfig(&config, &old_config);
                    __writeCmdLineRespPacket((void *)&old_config,  sizeof(old_config), DEV_RESP_CONFIG);
                }  else if(packetHeader->type == DEV_CMD_GET_VERSION) {
                   struct RespFirmwareVersion respFirm;
                   respFirm.major = THIS_MAJOR;
                   respFirm.minor = THIS_MINOR;
                   respFirm.patch = THIS_PATCH;
                   respFirm.reserved = 0;                       // For compatible reason
                   respFirm.revision = THIS_REVISION;
                   strcpy((char*) respFirm.dateString, __DATE__);
                   strcpy((char*) respFirm.timeString, __TIME__);
                  __writeCmdLineRespPacket((void *)&respFirm,  sizeof(respFirm), DEV_RESP_VERSION);
                } else if (packetHeader->type == DEV_CMD_RUNNING_STATUS_REQ) {
                  struct RouterRunningStatus status;
                  status.errorCode = 0;
                  status.radioOnOff = 1;
                  __writeCmdLineRespPacket((void *)&status,  sizeof(status), DEV_RESP_RUNNING_STATUS);
                } else if (packetHeader->type == DEV_CMD_SET_CONFIG) {
                    ex_config_t* newConfig;
                    newConfig = (ex_config_t*)(decoded + sizeof(struct PacketHeader));
                    CopyExConfigToConfig(newConfig, &config);
                    RadioSetPanIdShortAddr(config.panId, config.mySrcAddr);
                } else if (packetHeader->type == DEV_CMD_SET_EEPROM_DATA) {
                  struct RespUpdate up;
                  up.index = 0;
                  if (packetHeader->size < sizeof(struct PacketHeader) + 32) { // if data body greater than 32, only 32 bytes are written 
                    up.errorCode = ERROR_WRONG_PACKET;
                  } else {
                    uint8_t *data = (uint8_t *) decoded + sizeof(struct PacketHeader);
                    I2C_EE_BufferWrite(data, EEPROM_APP_ADDRESS, EEPROM_APP_SIZE);
                    up.errorCode = 0;
                  }
                  uint8_t type = DEV_RESP_SET_EEPROM_DATA;
                  __writeCmdLineRespPacket((unsigned char*) &up, 4, type);
                } else if (packetHeader->type == DEV_CMD_GET_EEPROM_DATA) {
                  struct RespEepromData resp;
                  I2C_EE_BufferRead(&resp.data[0], EEPROM_APP_ADDRESS, EEPROM_APP_SIZE);
                  resp.errorCode = 0;
                  __writeCmdLineRespPacket((unsigned char*) &resp, sizeof(resp), DEV_RESP_GET_EEPROM_DATA);
                } else if (packetHeader->type == DEV_CMD_SET_PROD_AREA) {
                  struct RespUpdate up;
                  uint8_t type = DEV_RESP_SET_PROD_AREA;
                  _decoded += sizeof(struct PacketHeader);
                  ret = SerialDownload((char*) _decoded, ret, packetHeader->type, packetHeader->size);
                  up.index = (ret & 0xFFFF0000) >> 16;
                  up.errorCode = ret & 0xFFFF;
                  __writeCmdLineRespPacket((unsigned char*) &up, 4, type);
                } else if (packetHeader->type == DEV_CMD_GET_PROD_AREA) {
                    _decoded += sizeof(struct PacketHeader);
                  struct GetProdAreaPacketHeader *prodHeader = (struct GetProdAreaPacketHeader*) _decoded;
                  struct RespProdArea resp;
                  static tPage page;
                  if (prodHeader->index < FLASH_PROD_AREA_SIZE/sizeof(resp.data)) {
                    page.address = FLASH_PROD_AREA;
                    page.count = FLASH_PROD_AREA_SIZE;
                    ret = ReadPage(&page);
                  } else {
                    ret = ERROR_FLASH_ADDRESS;
                  }
                  resp.errorCode = ret;
                  resp.index = prodHeader->index;
                  if (!ret) {
                    memcpy(resp.data, &page.data[prodHeader->index*sizeof(resp.data)], sizeof(resp.data));
                  }
                  __writeCmdLineRespPacket((unsigned char*) &resp, sizeof(resp), DEV_RESP_GET_PROD_AREA);
                }


            }
            break;

#if (defined USE_MY_ASSERT) || (defined USE_FULL_ASSERT)
            case 'a':
                {
                    assert_loop = 0;
                }
                break;
#endif
        default:
          break;
        }
        if (cmd != 'U') {
            TRACE("> ");
        }
    }
}

/*****************************************************************************/
//Description    : Ethernet task
// Input          :
// Return         : -
/***************************************************************************/
static void Ethertask(void* pdata){
    uIP_Loop();
}

static void WatchDogTask(void* pdata) {

  while (1) {
    RELOAD_WATCHDOG
    // every 100 millisec
    CoTickDelay(100);
    // 3200 mSec watchdog timeout
    if (task_ticks[0] & 0xE0) {
      diagnostic.stopped_task = radioTaskId;
    }
    if (task_ticks[1] & 0xE0) {
      diagnostic.stopped_task = configTaskId;
    }
    if (task_ticks[2] & 0xE0) {
      diagnostic.stopped_task = etherTaskId;
    }
    if (diagnostic.stopped_task) {
      __disable_interrupt();
      diagnostic.taskRadioTx_line = CoGetTaskLine(radioTaskId);
      diagnostic.taskRadioTx_func = CoGetTaskFunc(radioTaskId);
      diagnostic.taskConfig_line = CoGetTaskLine(configTaskId);
      diagnostic.taskConfig_func = CoGetTaskFunc(configTaskId);
      diagnostic.taskEther_line = CoGetTaskLine(etherTaskId);
      diagnostic.taskEther_func = CoGetTaskFunc(etherTaskId);
      __enable_interrupt();
      I2C_EE_BufferWrite((uint8_t*) &diagnostic, EEPROM_DEBUG_DIAGNOSTIC, sizeof(diagnostic));
      while (1);

     } else {
      // RELOAD_WATCHDOG
       task_ticks[0]++;
       task_ticks[0]++;
       task_ticks[0]++;
     }
  }
}

/*******************************************************************************
* Description    : Output in DEC
* Input          :
* Return         : -
*******************************************************************************/

/* PUBLIC FUNCTION PROTOTYPES ------------------------------------------------*/

/*******************************************************************************
* Description    : Main routine.
* Input          : -
* Return         : -
*******************************************************************************/
void  main(void) {

#ifdef BOOTLOADER
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0xC000);
#endif

    SysTick_Config(SystemCoreClock / 1000); // SysTick Interrupt Freq = (1/1000)s = 1ms

    HwI2CInit();
    I2C_EE_BufferRead((uint8_t*) &hf_counter, EEPROM_DEBUG_COUNTER, EEPROM_DEBUG_COUNTER_SIZE);
    hf_counter++;

#ifdef WDT_ENABLE
    uint8_t hf_flags = 0;
    I2C_EE_BufferRead((uint8_t*) &hf_flags, EEPROM_DEBUG_DIAGNOSTIC+54, 1);

    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET) {
      if (hf_flags & DIAGNOSTICS_NOTFLAG_HARDFAULT) {
        // watchdog clicked, but we did not get to hardfault, nothing saved
        // save it now
          I2C_EE_BufferWrite((uint8_t*) &hf_counter, EEPROM_DEBUG_COUNTER, EEPROM_DEBUG_COUNTER_SIZE);
          hf_counter++;
          hf_flags &= ~DIAGNOSTICS_LASTEVENT_HARDFAULT;
      }

        RCC_ClearFlag();
      }
    hf_flags |= DIAGNOSTICS_NOTFLAG_HARDFAULT;
    I2C_EE_BufferWrite((uint8_t*) &hf_flags, EEPROM_DEBUG_DIAGNOSTIC+54, 1);

#endif

    memset( radioRxStack, 0xAE, sizeof(radioRxStack));
    memset( configTaskStack, 0xAC, sizeof(configTaskStack));
    memset( EthertaskStack, 0xAD, sizeof(EthertaskStack));


    memset((void*) &diagnostic, 0, sizeof(diagnostic));
    diagnostic.version = DIAGNOSTIC_VERSION;
    diagnostic.size = sizeof(diagnostic);
    I2C_EE_BufferRead((uint8_t*) &diagnostic.event_count, EEPROM_DEBUG_DIAGNOSTIC, sizeof(diagnostic.event_count));
    diagnostic.event_count++;
    LoadConfig(&config);
#if 0
      {
        config_t ee_config;
        I2C_EE_BufferRead((uint8_t*) &ee_config, EEPROM_CONFIG_ADDRESS, sizeof(ee_config));
        if (CheckConfigChecksum((uint16_t*) &ee_config)) {
          memcpy((void*) &config, &ee_config, sizeof(config));
        } else {
          WriteConfig();
        }
        I2C_EE_BufferRead(mac, OLD_EEPROM_MAC_ADDRESS, 6);
        uint8_t check = 0;
        for (uint8_t u = 0; u < 6; u++) {
          check ^= mac[u];
        }
        if (check == 0) {
          mac[0] = 0x00; mac[1] = 0x25; mac[2] = 0x62; mac[3] = 0x00; mac[4] = 0x0C; mac[5] = 0x01;
         I2C_EE_BufferWrite(ip_me, OLD_EEPROM_MAC_ADDRESS, 6);
         }
        check = 0;
        I2C_EE_BufferRead(ip_me, OLD_EEPROM_MYIP_ADDRESS, 4);
        for (uint8_t u = 0; u < 4; u++) {
          check ^= ip_me[u];
        }
        if (check == 0) {
          ip_me[0] = 10; ip_me[1] = 133; ip_me[2] = 0; ip_me[3] = 100;
          I2C_EE_BufferWrite(ip_me, OLD_EEPROM_MYIP_ADDRESS, 4);
        }
        check = 0;
        I2C_EE_BufferRead(ip_dest, OLD_EEPROM_DESTIP_ADDRESS, 4);
        for (uint8_t u = 0; u < 4; u++) {
          check ^= ip_dest[u];
        }
        if (check == 0) {
          ip_dest[0] = 10; ip_dest[1] = 133; ip_dest[2] = 0; ip_dest[3] = 200;
          I2C_EE_BufferWrite(ip_dest, OLD_EEPROM_DESTIP_ADDRESS, 4);
        }
        check = 0;
        I2C_EE_BufferRead(netmask, OLD_EEPROM_NETMASK_ADDRESS, 4);
        for (uint8_t u = 0; u < 4; u++) {
          check ^= netmask[u];
        }
        if (check == 0) {
          netmask[0] = 255; netmask[1] = 0; netmask[2] = 0; netmask[3] = 0;
          I2C_EE_BufferWrite(netmask, OLD_EEPROM_NETMASK_ADDRESS, 4);
        }
      }
#endif
    CoInitOS();

    __enable_interrupt();

    ethTaskMsgQueue = CoCreateQueue (ethTaskMsgQueueBuf, TASK_MSG_QUEUE_SIZE, EVENT_SORT_TYPE_FIFO);
    assert(ethTaskMsgQueue != E_CREATE_FAIL);

    ethTxMsgQueue = CoCreateQueue (ethTxMsgQueueBuf, TX_MSG_QUEUE_SIZE, EVENT_SORT_TYPE_FIFO);
    assert(ethTxMsgQueue != E_CREATE_FAIL);

    SET_WAKEUP_FRAME(wakeupMsg->msgType);
    wakeupMsg->msgLength = 0;

    semRFRxFrames = CoCreateSem (0, 127, EVENT_SORT_TYPE_FIFO);   //Number of RF frames received in Rx-FIFO. It is notified by RX_FRM_DONE interrupt
    assert(semRFRxFrames != E_CREATE_FAIL);

#if 0
    // Check to see if we had a wdg reset
    if( RCC_GetFlagStatus(RCC_FLAG_IWDGRST) )
    {
      RCC_ClearFlag();
      // inc and store
      WriteConfig();
    }
#endif
    HwPeriphInit();

      USBD_Init(&USB_OTG_dev,
            USB_OTG_FS_CORE_ID,
            &USR_desc,
            &USBD_CDC_cb,
            &USR_cb);

      GetARM_UUID(); // init and load the UUID into the struct
    uIP_timer_setup(); // get the timer started


     Ethernet_Init();    // setup the ethernet i/o hardware
     uIP_Setup(config.mac_address, config.my_ip, config.dest_ip,
        config.netmask, config.my_port, config.dest_port);    // Init the stack

     init_ether_comm();

   RadioInit(config.panId, config.mySrcAddr,config.rfChan);


    radioTaskId = CoCreateTaskEx (RadioRxTask, (void*)0, 1, &radioRxStack[OS_STACK_SIZE-1], OS_STACK_SIZE,(uint32_t)0, (uint32_t)0);
    configTaskId = CoCreateTask  (ConfigTask, (void*)0, 9, &configTaskStack[OS_STACK_SIZE-1], OS_STACK_SIZE);
    etherTaskId = CoCreateTaskEx(Ethertask, (void*)0,1, &EthertaskStack[OS_STACK_SIZE-1], OS_STACK_SIZE,(uint32_t)0, (uint32_t)0);

    CoCreateTaskEx(WatchDogTask, (void*)0,0, &WatchDogTaskStack[OS_STACK_SIZE-1], OS_STACK_SIZE,(uint32_t)0, (uint32_t)0);

    WDTimerInit();

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
    // CoSchedLock();
    while (assert_loop) {
        TRACE("!!!ASSERT FAILED!!! %s : %d\n\r", file, line);
        HwLEDOn(LED1); HwLEDOn(LED4); HwLEDOff(LED2); HwLEDOff(LED3);
        HwWait(50);
        HwLEDOn(LED2); HwLEDOn(LED3); HwLEDOff(LED1); HwLEDOff(LED4);
        HwWait(50);
    }
    assert_loop = 1;
}
#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

// malloc a block for message with size
void* MallocMsg(uint16_t size) {
    U32 allocSize = sizeof(MsgHeader) + size;
    void* ptr = CoKmalloc(allocSize);
    return ptr;
}

// skip the header
void* GetMsgBody(void* msg) {
    return (uint8_t*)msg + sizeof(MsgHeader);
}

// header
MsgHeader* GetMsgHeader(void* msg) {
  return msg;
}

static void PrintDiagnostic(tDiagnostics *pDiagnostics)
{
  TRACE("-- Diagnostic --\r\n");
  TRACE("Watchdog count=%u\r\n", hf_counter);
  TRACE("Evt counter=%u\r\n", pDiagnostics->event_count);
  TRACE("radioRxErrors=%u ethTxDropped=%u ethTxRecvd=%u\n\r", pDiagnostics->rfTxErrors,
        pDiagnostics->ethRxDropped, pDiagnostics->ethRxRcvdOk);
  TRACE("pendSemRFRxFrames=%u leaveFlagSPIIODone=%u enterFlagSPIIODone=%u\n\r",
        pDiagnostics->pendSemRFRxFrames, pDiagnostics->leaveFlagSPIIODone,
        pDiagnostics->enterFlagSPIIODone);
  CoTickDelay(10);
  TRACE("clrSPIDone=%u waitSPIDone=%u\n\r",
        pDiagnostics->clrSPIDone, pDiagnostics->waitSPIDone);
  TRACE("setEtherTxDone=%u acceptEtherTxDone=%u\n\r", pDiagnostics->setEtherTxDone,
        pDiagnostics->acceptEtherTxDone);
  TRACE("setSemRFRxFrames=%u postEthTaskQueue=%u postEthernet=%u\n\r",
        pDiagnostics->setSemRFRxFrames, pDiagnostics->postEthTaskQueue,
        pDiagnostics->postEthernetRcvd);
  CoTickDelay(10);
  TRACE("setFlagSPIDone1=%u setFlagSPIDone2=%u setFlagSPIDone3=%u\n\r",
        pDiagnostics->setFlagSPIDone1, pDiagnostics->setFlagSPIDone2,
        pDiagnostics->setFlagSPIDone3);
  TRACE("stopped_task=%d\n\r");
  TRACE("RadioTxTask: func=0x%08X line=%u\n\r",
        pDiagnostics->taskRadioTx_func, pDiagnostics->taskRadioTx_line);
  CoTickDelay(10);
  TRACE("ConfigTask: func=0x%08X line=%u\n\r",
        pDiagnostics->taskConfig_func, pDiagnostics->taskConfig_line);
  TRACE("EtherTask: func=0x%08X line=%u\n\r",
        pDiagnostics->taskEther_func, pDiagnostics->taskEther_line);
  TRACE("\n\r");
  /*
  StatusType            clrSPIDone;
  StatusType            waitSPIDone;
*/

}



