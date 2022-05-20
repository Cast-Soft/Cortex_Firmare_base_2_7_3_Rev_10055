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
#include <ctype.h>
#include "radio.h"
#include "radio_defs.h"
#include "packets.h"
#include "FlashUpdate.h"
#include "stm32f10x_flash.h"
#include "main.h"
#include "flash_map.h"
#include "util.h"

#include "i2c_ee.h"
#include "config.h"
#include "udp_process.h"
#include "console_tail.h"
#include "VersionNo.h" 

BOOL usePreciseClock = Co_FALSE;

/* PRIVATE TYPEDEF -----------------------------------------------------------*/




/* PRIVATE DEFINES -----------------------------------------------------------*/

#define OS_STACK_SIZE   256 //196 //128         // it is word == 4 * 128
#define MY_PAN_ID       0x0001      // TODO!!! choose / set a pan id
#define MY_ADDR         0xABCD

#ifdef TK_V3_hdwr
#define HW_Version 'C'
#endif

// for outbound ethernet frames
#define TX_MSG_QUEUE_SIZE 12       // TODO!!! test to get a better value

// EtherTask will be waiting and processing msg in queue
#define TASK_MSG_QUEUE_SIZE 40       // TODO!!! test to get a better value

#define FLASH_NO_ADDRESS_CHECK
#define AIRCR_VECTKEY_MASK    ((u32)0x05FA0000)

/* PRIVATE MACROS ------------------------------------------------------------*/


/* EXTERN VARIABLES ----------------------------------------------------------*/


/* PRIVATE VARIABLES ---------------------------------------------------------*/
#if (defined USE_MY_ASSERT) || (defined USE_FULL_ASSERT)
static uint8_t  assert_loop = 1;
#endif

static uint32_t frameOptiHub;
static uint32_t frameDiff;
static uint32_t old;
static uint32_t rfId;
static OS_STK   taskRadioTxStack[OS_STACK_SIZE];
static OS_STK   taskConfigStack[OS_STACK_SIZE];
static OS_STK   EthertaskStack[OS_STACK_SIZE];
static OS_STK   WatchDogTaskStack[OS_STACK_SIZE];

static uint8_t  txBuf[FRAME_LENGTH_MAX - MHR_SIZE - MFR_SIZE];   // tx-payload buffer
static void *   ethTaskMsgQueueBuf[TASK_MSG_QUEUE_SIZE];
static MsgHeader       wakeupMsgEntity;
static void *ethTxMsgQueueBuf[TX_MSG_QUEUE_SIZE];
static uint32_t atDma;
static tPage    page = {FLASH_PROD_AREA};
static uint16_t CurrentDataPointer = 0x00;
static uint16_t packet = 0;
static uint32_t flags = 0;
static uint8_t watchdog_active = 1;

struct DeviceSerialNumber ARM_proc_SN;
struct DeviceSerialNumber *pARM_proc_SN = &ARM_proc_SN;

/*PUBLIC VARIABLES ----------------------------------------------------------*/


__no_init tDiagnostics       diagnostic;

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;
uint8_t         rx_delay = 2;
uint8_t         tick;
uint32_t *      pRxFrameId = (uint32_t*) &txBuf[0];
tDataSend       dataFramesTx[4];
MsgHeader*      wakeupMsg = &wakeupMsgEntity;
OS_FlagID       flagLoadFrameId;
OS_FlagID       flagRFBeaconSent;
StatusType       waitLoadFrame;
OS_EventID      ethTaskMsgQueue;
OS_EventID      ethTxMsgQueue;
OS_TID          taskRadioTxId;
OS_TID          taskConfigId;
OS_TID          taskEtherId;

uint8_t task_ticks[3] = {0,0,0};

uint32_t session_random = 0;
uint16_t offsetDelta = 0;               // Default (classic) T.K. this value is 0 but new T.K. might be vary  

config_t config = {
    .config_version     = CURRENT_CONFIG_VERSION,
    .mac_address        = {0x00, 0x25, 0x62, 0x00, 0x0C, 0x03},
    .my_ip              = ((10 << 24) | (133 << 16) | (1 << 8) | 100),
    .dest_ip            = ((2 << 24) | (0 << 16) | (0 << 8) | 200),
    .netmask            = 0xFF000000,
    .my_port            = TK_Listen_Port,
    .dest_port          = TK_Send_Port,
    .productID          = TK_PRODUCT_ID, //0xCD10,
    .serialNum          = 0x555,
    .panId              = 0x23,
    .mySrcAddr          = 0xABCD,
    .rfChan             = 26,
    .TxPower            = 8,    // index of tx power
    .TestMode           = 0x00,
    .SyncOutEn          = 0x01,
    //.Dummy          = 0,
    .activeChannelsBitMask = 0x0010, //channel 15
    .rf_scan            = 0x01,
    .time_adjust        = 0,
#ifdef _DEBUG
    .flags              = FLAG_TRACE_ENABLE |
#else
    .flags              =
#endif
//#ifdef Ether_active   By default is off
//                       FLAG_ETHERNET_ON |
//#endif
                        FLAG_TRACE_UDP | FLAG_FRAMEID_24BITS,
    .frameOffset        = FRAME_OFFSET_DEFAULT,
    .frameClock         = FRAME_CLOCK_100
};

int syncPulse = 100;      // interval of sync pluse (int tick)

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/

#ifdef Ether_active
static void Ethertask (void* pdata);
#endif

static void mygets(char *str);
static void PrintDiagnostic(tDiagnostics *pDiagnostics);


/* PRIVATE FUNCTIONS ---------------------------------------------------------*/
uint32_t atDma;

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

    do {
        while( (c = getchar()) == EOF ) {
            // 100ms => humans don't type >10 char/sec
            CoTickDelay(10);
            SAVE_POINT
            task_ticks[taskConfigId - 1] = 0;
            diagnostic.taskConfigCycles++;
        }
        putchar(c);
        SAVE_POINT
        *tmp++ = (char) c;
    } while (c != 0x0D);
    SAVE_POINT
    putchar(0x0A);
    *(--tmp) = '\0';
}




/*******************************************************************************
* Description : Set Configuration
* Input       :
* Return      : -
*******************************************************************************/
static void SetConfig(/*uint16_t idx, uint32_t val*/ char *cmd) {
   char *p = NULL;
    uint16_t idx = strtoul(cmd, &p, 10);
    uint16_t val = 0;
    if (idx < 20 || idx == 22) {
      while (*p && isspace(*(++p)));
      if (!*p) {
        return;
      }
      val = strtoul(p, NULL, 10);
    }
    switch (idx) {
        /*
            0 : productID
            1 : serialNum
            2 : panId
            3 : mySrcAddr
            4 : rfChan
            7 : TxPower
            8 : TestMode
            9 : OptiHubOutEn
        */
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
          case 0x09:
            config.SyncOutEn = (val) ? 1 : 0;
            if(config.SyncOutEn) // val == enable which is reverse logic
            {
              HwGPOLow(GPO_OH_SYNC_OUT_EN);
            }
            else
            {
              HwGPOHigh(GPO_OH_SYNC_OUT_EN);
            }
            break;
        case 14:
          {
            config.TxPower = val;
            RadioSetRFLevel(config.TxPower);
          }
            break;
        case 22:
        case 0x4:
            if(idx == 4 && (val >= OLD_RF_CHANNEL_MIN && val <= OLD_RF_CHANNEL_MAX))
            {
              val = (1 << (val - OLD_RF_CHANNEL_MIN));
            } else if (idx == 22 && (val >= RF_CHANNEL_MIN && val <= RF_CHANNEL_MAX)) {
#ifndef MULTI_CHANNEL
              config.rfChan = val;
              RadioSetRFChan(config.rfChan);
#endif              
              val = (1 << (val - RF_CHANNEL_MIN));
            } else {
              break;
            }
          //  no break;
          // case 18 must follow
#ifdef MULTI_CHANNEL            
        case 18:
          {
            config.activeChannelsBitMask = val;
            // need to set rfChan to lowest channel
            if (val) {
              uint32_t index = 1;
              uint8_t channel = 0;
              while (( index & val) == 0) {
                channel++;
                index <<= 1;
              }
              config.rfChan = channel + RF_CHANNEL_MIN;
              RadioSetRFChan(config.rfChan);
            }
          }
#endif          
          break;
        case 17:
          switch (val) {
            case FRAME_CLOCK_120:
            case FRAME_CLOCK_180:
            case FRAME_CLOCK_240:
              config.frameClock = val;
            break;
           default:
              config.frameClock = FRAME_CLOCK_100;
          }
          //TODO reInit LED clock
          HwTIM3_UpdatePrescaler(config.frameClock);
          SaveConfig(&config);
          break;
        case 19:
          {
            config.rf_scan = val;
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
    TRACE("\n\r FIRMWARE VERSION: " STRINGIFY(THIS_MAJOR) "." STRINGIFY(THIS_MINOR) "." STRINGIFY(THIS_PATCH) "." STRINGIFY(THIS_REVISION) " TK DEBUG\n\r\n\r");
#else
    TRACE("\n\r FIRMWARE VERSION: " STRINGIFY(THIS_MAJOR) "." STRINGIFY(THIS_MINOR) "." STRINGIFY(THIS_PATCH) "." STRINGIFY(THIS_REVISION) " TK RELEASE\n\r\n\r");
#endif
    TRACE(" "__DATE__" : "__TIME__" \r\n\n");
    TRACE("panId       : %04X\n\r", config.panId);
    TRACE("mySrcAddr   : %04X\n\r", config.mySrcAddr);
    TRACE("rfChan      : %02X\n\r", config.rfChan);
    CoTickDelay(10);
    TRACE("TxPower     : %02X\n\r", TxAmpValues[config.TxPower]);
    TRACE("OptiHubOutEn: %d\n\r", config.SyncOutEn);
#ifdef Ether_active
    if (config.flags & FLAG_ETHERNET_ON) {
      TRACE("Ethernet %s\n\r", (flags & FLAG_ETHERNET_UP)?"is active":"not started (restart needed)");
    } else {
      TRACE("Ethernet is disabled\n\r");
    }
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

#endif
    TRACE("activeChannelsBitsMask: %08X\n\r", config.activeChannelsBitMask);
    CoTickDelay(10);
    TRACE("Frame clock =%u Hz\r\n", config.frameClock);
    TRACE("Timer adjust: %d\n\r", config.time_adjust);
    TRACE("Frame offset: %d\n\r", config.frameOffset);
    TRACE("FrameId 24 bit wrap-around:%s\n\r",(config.flags & FLAG_FRAMEID_24BITS)?"yes":"no");
}


/*******************************************************************************
* Description    : [Task] Send (10Hz) RF Sync Packets
* Input          :
* Return         :
*******************************************************************************/
static void TaskRadioTx (void* pdata){


    while (1) {
        SAVE_POINT
        RadioWaitGrabSPI();
        SAVE_POINT
        diagnostic.waitLoadFrame = CoWaitForSingleFlag(flagLoadFrameId, 1100);
        task_ticks[taskRadioTxId - 1] = 0;
        diagnostic.taskRadioCycles++;
        SAVE_POINT

        if(!session_random) {
          session_random = RadioGetRandom();
        }

        HwGPOHigh(GPO_TP49);
        struct BeaconOldStruct *beacon = (struct BeaconOldStruct*) txBuf;    
        // RF Sync Packet Payload :
        // {FrameID[7:0], FrameID[15:8], FrameID[23:16], FrameID[31:24]}
        if (config.flags & FLAG_FRAMEID_24BITS) {
          *(uint32_t*)txBuf = rfId = ((rfBeaconFrameId + 2 + config.frameOffset) % 16777212);             // as 16777212 % 12 == 0
        } else {
          *(uint32_t*)txBuf = rfId = rfBeaconFrameId + 2 + config.frameOffset;
        }
        beacon->magic = 'BT';   // This is actually a bug. only 'B' is assigned. To be fixed
#if 0   // Beacon occasionally stop reacts to T.K. with these new fields. Reason unknown. So roll back to old format         
        // Two new fields
        beacon->session = session_random;    // For session approach
        extern uint16_t offsetDelta;  
        beacon->offsetDelta = offsetDelta;   // new type of T.K. may have different offset delta        
#endif
        beacon->frameClock = config.frameClock;
#ifdef MULTI_CHANNEL        
        beacon->tick = tick;
#else
        beacon->tick = tick++;
#endif        
        beacon->crc8 = crc8(txBuf, sizeof(struct BeaconOldStruct) - 1);
        old = *frameId;
        SAVE_POINT
        RadioTxPkt(BASIC_RF_BROADCAST_ADDR, 1, /*4*/ sizeof(struct BeaconOldStruct), txBuf, 0); // beacon frame, delayed-send
        SAVE_POINT
        HwLEDToggle(LED1);
        HwLEDOn(LED2);
        HwGPOLow(GPO_TP49);
        frameOptiHub = *frameId;
        atDma = dma_done;
          frameDiff = (frameOptiHub - *(uint32_t*)txBuf);

#ifdef SEND_PACKETS
      // lookup frames we have for this channel
    for (uint8_t u = 0; u < sizeof(dataFramesTx)/sizeof(tDataSend); u++) {
      if (dataFramesTx[u].channel == (rfChannel + RF_CHANNEL_MIN) && dataFramesTx[u].payloadSize) {
        HwWait(2);
        SAVE_POINT
        RadioTxPkt(dataFramesTx[u].destAddr, 0,
                   dataFramesTx[u].payloadSize,
                   dataFramesTx[u].payload, 1); // beacon frame, delayed-send
#if 1
        dataFramesTx[u].channel = 0;
        dataFramesTx[u].payloadSize = 0;
#endif
        break;
      }
    }
    SAVE_POINT
#endif
        RadioReleaseSPI();
#ifdef MULTI_CHANNEL    
        uint8_t channel = rfChannel + RF_CHANNEL_MIN;
        if (channel <= RF_CHANNEL_MAX) {
          config.rfChan = channel;
          RadioSetRFChan(config.rfChan);
        }
#endif        
    }
}

/*******************************************************************************
* Description : Write Configuration to FLASH
* Input       : -
* Return      : -
*******************************************************************************/

static uint32_t WritePage(tPage *page) {

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

static uint32_t ReadPage(tPage *page) {

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


static uint32_t SerialDownload(const char* decoded, uint16_t len, uint8_t type, uint16_t size)
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

/*******************************************************************************
* Function Name  : NVIC_GenerateSystemReset
* Description    : Generates a system reset.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void NVIC_GenerateSystemReset(void)
{
  SCB->AIRCR = AIRCR_VECTKEY_MASK | (u32)0x04;
}

/*******************************************************************************
* Description    : [Task] Implements Simple Text Command Console
* Input          :
* Return         :
*******************************************************************************/
static void TaskConfig (void* pdata){

    static char buf[260] = "";
    char *s = buf;
    char cmd;
    static int addr, value;


    while (1) {

        // get a line of values from stdin
        s = buf;
        mygets(s);
        if (ValidateCommandLine(s, strlen(s)) == 0) {
          TRACE("Incorrect packet\r\n");
          continue;
        }        
        cmd = *s;
        SAVE_POINT
        switch (cmd) {
            case 'd': // Display configuration
                PrintConfig();
                break;
            case 's': // Set configuration
                s+= 2;
                SetConfig(s);
                break;
            case 'v': // saVe configuration
                SaveConfig(&config);
                break;
            case '!': // special message
                NVIC_GenerateSystemReset();
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
                  case 0:
                    config.flags ^= FLAG_TRACE_ENABLE;
                    break;
                  case 8:
                    {
                      struct WhoAmI me;
                      memcpy(me.id, (uint8_t *)(0x1FFFF7E8), 12);
                      me.type = 1;
                      me.module = 2;
                      __writeCmdLineRespPacket((uint8_t*) &me, sizeof(me), DEV_RESP_WHOAMI);
                    }
                    break;
                  case 9:
                    {
                      config.flags |= FLAG_ETHERNET_ON;
                      TRACE("Ethernet activated. Restart required.\n\r");
                      SaveConfig(&config);
                    }
                   break;
                  case 10:
                    {
                      config.flags &= ~FLAG_ETHERNET_ON;
                      TRACE("Ethernet deactivated. Restart required.\n\r");
                      SaveConfig(&config);
                    }
                    break;
                  case 11:
                    {
                      config.flags |= FLAG_TRACE_UDP;
                      SaveConfig(&config);
                    }
                    break;
                  case 12:
                    {
                      config.flags &= ~FLAG_TRACE_UDP;
                      SaveConfig(&config);
                    }
                    break;
                  case 17:
                    {// for consistency with beacon 0x11
                     config.time_adjust = value;
                  SaveConfig(&config);
                  whole_time_adjust = config.time_adjust/10;
                  part_time_adjust = config.time_adjust%10;
                    }
                    break;
                  case 40:
                    {
                      //erase EEPROM
                      char buf[64];
                      memset(buf, value, 64);
                      I2C_EE_BufferWrite(buf, 0, 64);
                    }
                    break;
#if 1
                  case 41: //test
                    {
                      __disable_interrupt();
                      for (uint8_t u = 0; u < 4; u++) {
                        if (!dataFramesTx[u].payloadSize) {
                          dataFramesTx[u].payloadSize = 4; //passThrough->payloadSize;
                          dataFramesTx[u].channel = config.rfChan; //passThrough->channel;
                          dataFramesTx[u].destAddr = value; //passThrough->destAddr;
                          dataFramesTx[u].payload[0] = 0xFE;
                          dataFramesTx[u].payload[1] = 0xAD;
                          dataFramesTx[u].payload[2] = 0x00;
                          dataFramesTx[u].payload[3] = 0x04;
                          //memcpy(dataFramesTx[u].payload, /*passThrough->payload*/, passThrough->payloadSize);
                          //error = 0;
                          break;
                        }
                      }
                      __enable_interrupt();
                    }
                    break;
#endif
                  case 42:
                    // erase EEPROM
                    {
                      if (value == 996) {
                        // I2C_EE_EraseAll();
                        TRACE("Feature not implemented \r\n");
                      } else {
                        TRACE("EEPROM is NOT deleted, key is wrong\r\n");
                      }

                    }
                    break;
                  case 46:
                    rx_delay = value;
                    break;
                  case 47:
                    *frameId = value;
                    break;
                  case 48:
                    {
                      config.flags ^= FLAG_FRAMEID_24BITS;
                  SaveConfig(&config);
                    }
                    break;
                  case 49:
                    {
                      TRACE("counters: frbeaconFrameId=%d frameOptiHub=%d\n\r", rfBeaconFrameId, frameOptiHub);
                      TRACE("counters: frameDiff=%d, atDMa=%d old=%d\n\r", frameDiff, atDma, old);
                      TRACE("counters: rfId=%d\n\r", rfId);
                    }
                    break;
                  case 51:
                    {
                                 config.frameOffset = value;
                  SaveConfig(&config);

                    }
                    break;
                  case 52:
                    {
                      uint32_t ram = (uint32_t) value & 0xFFFFFFFE;
                      if (ram >= 0x20000000 && ram <= 0x2000ffff) {
                        uint32_t atRam = *((uint32_t*) (ram));
                        TRACE("Mem at 0x%08X=0x%08X\n\r", ram, atRam);
                      } else {
                        TRACE("Mem addr error\n\r");
                      }
                    }
                    break;
                  case 53:
                    {
                      uint32_t rand = RadioGetRandom();
                      TRACE("Random=0x%08X\n\r", rand);
                    }
                    break;
                  case 54:
                    {
                      uint32_t rand = RadioGetRandom();
                      TRACE("TK: New session with rnd=0x%08X old sesssion=0x%08X\n\r", rand, session_random);
                      session_random = rand;
                    }
                    break;
                  case 55:
                    {
                      TRACE("Offset delta=%d\n\r", offsetDelta);
                    }
                    break;
                  case 56:
                    {
                      offsetDelta = value;
                      TRACE("Offset delta=%d\n\r", offsetDelta);
                    }
                    break;                    
                    case 95:
                    {
                      watchdog_active = value;
                      TRACE("Watchdog is %s\r\n", value?"active":"not active");
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
                     // TRACE("setLoadFrame=%d setRFBeaconSent=%d\r\n", diagnostic.setLoadFrame,  diagnostic.setRFBeaconSent);
                     // TRACE("setRadioTxDone=%d setSPIMachineDone=%d\r\n", diagnostic.setRadioTxDone,  diagnostic.setSPIMachineDone);
                     // TRACE("setEtherTxDone=%d acceptEtherTxDone=%d\r\n", diagnostic.setEtherTxDone,  diagnostic.acceptEtherTxDone);
                     // TRACE("waitLoadFrame=%d waitRFBeaconSent=%d\r\n", diagnostic.waitLoadFrame,  diagnostic.waitRFBeaconSent);
                    }
                    break;
                    
                  case 100:      //  Read hardware version
                    {
                      struct RespHardwareVersion hwVersion;
                      if (usePreciseClock) {
                        hwVersion.version = 12345;
                      }
                      else {
                        hwVersion.version = 1;
                      }                    
                       __writeCmdLineRespPacket((unsigned char*) &hwVersion, sizeof(hwVersion), DEV_RESP_HARDWARE_VERSION);
                      break;          
                    }
                  } // of "t switch)
                } // of t cmd
                break;
              case 'U':
              {       // Extended command line that use base64 encoded parameters
                static uint8_t decoded[512];
                uint8_t *_decoded = &decoded[0];
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
                int ret = b64_pton(p, decoded, sizeof(decoded));

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
                } else if (packetHeader->type == DEV_RESP_SEND_BEACON_DATA) {
                  tDataSend *newData = (tDataSend*) (decoded + sizeof(struct PacketHeader));
                  struct RespSendBeaconData resp;
                  resp.status =  SEND_BEACON_DATA_BUSY;
                  __disable_interrupt();
                      for (uint8_t u = 0; u < sizeof(dataFramesTx)/sizeof(dataFramesTx[0]); u++) {
                        if (!dataFramesTx[u].payloadSize) {
                          dataFramesTx[u].payloadSize = newData->payloadSize;
                          dataFramesTx[u].channel = newData->channel;
                          dataFramesTx[u].destAddr = newData->destAddr;
                          memcpy(dataFramesTx[u].payload, newData->payload, newData->payloadSize);
                          resp.status = SEND_BEACON_DATA_OK;
                          break;
                        }
                      }
                      __enable_interrupt();
                  __writeCmdLineRespPacket((void *) &resp,  sizeof(resp), DEV_RESP_SEND_BEACON_DATA);
                } else if(packetHeader->type == DEV_CMD_GET_VERSION) {
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
                    SaveConfig(&config);
                    RadioSetPanIdShortAddr(config.panId, config.mySrcAddr);
                    RadioSetRFLevel(config.TxPower);


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
           case 'Y': // hardware version printout
              {
                struct RespHardwareVersion hwVersion;
                if (usePreciseClock) {
                  hwVersion.version = 12345;
                }
                else {
                  hwVersion.version = 1;
                }     
                TRACE("Hardware version %d\n\r", (uint32_t)hwVersion.version);
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

#ifdef Ether_active
static void Ethertask(void* pdata){
    uIP_Loop();
}
#endif

static void WatchDogTask(void* pdata) {
static uint8_t wrote_eeprom = 0;
  while (1) {
    // every 100 millisec
    CoTickDelay(100);
    // 3200 mSec watchdog timeout
    if (task_ticks[0] & 0xE0) {
      diagnostic.stopped_task = taskRadioTxId;
    }
    if (task_ticks[1] & 0xE0) {
      diagnostic.stopped_task = taskConfigId;
    }
    if (task_ticks[2] & 0xE0) {
      diagnostic.stopped_task = taskEtherId;
    }
    if (diagnostic.stopped_task && !wrote_eeprom) {
      __disable_interrupt();
      diagnostic.taskRadioTx_line = CoGetTaskLine(taskRadioTxId);
      diagnostic.taskRadioTx_func = CoGetTaskFunc(taskRadioTxId);
      diagnostic.taskConfig_line = CoGetTaskLine(taskConfigId);
      diagnostic.taskConfig_func = CoGetTaskFunc(taskConfigId);
      diagnostic.taskEther_line = CoGetTaskLine(taskEtherId);
      diagnostic.taskEther_func = CoGetTaskFunc(taskEtherId);
      __enable_interrupt();
      I2C_EE_BufferWrite((uint8_t*) &diagnostic, EEPROM_DEBUG_DIAGNOSTIC, sizeof(diagnostic));
#if 0
      while (1);
#else
      wrote_eeprom = 1;
#endif
     } else {
      // RELOAD_WATCHDOG
       task_ticks[0]++;
       task_ticks[0]++;
       task_ticks[0]++;
     }
  }
}

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
#if 0
    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET) {
        I2C_EE_BufferWrite((uint8_t*) &diagnostic, EEPROM_DEBUG_DIAGNOSTIC, sizeof(diagnostic));
        /*if (hf_caught == 0xDEADBEEF) {
         hf_caugh = 0;

        }*/
        RCC_ClearFlag();
    }
#endif
    memset((void*) &diagnostic, 0, sizeof(diagnostic));
    diagnostic.version = DIAGNOSTIC_VERSION;
    diagnostic.size = sizeof(diagnostic);
    I2C_EE_BufferRead((uint8_t*) &diagnostic.event_count, EEPROM_DEBUG_DIAGNOSTIC, sizeof(diagnostic.event_count));
    diagnostic.event_count++;
    LoadConfig(&config);
    if (config.activeChannelsBitMask == 0 || config.panId == 0) {
      config.activeChannelsBitMask = 0x2000;
      // need to set rfChan to lowest channel
      config.rfChan = 13 + RF_CHANNEL_MIN;
     // RadioSetRFChan(config.rfChan);
      SaveConfig(&config);
    }
    if (config.TxPower == 0 || config.TxPower > 8) {
      config.TxPower = 8;
      SaveConfig(&config);
    }
    whole_time_adjust = config.time_adjust/10;
    part_time_adjust = config.time_adjust%10;

    whole_time_adjust = config.time_adjust/10;
    part_time_adjust = config.time_adjust%10;

    CoInitOS();
    __enable_interrupt();

    ethTaskMsgQueue = CoCreateQueue (ethTaskMsgQueueBuf, TASK_MSG_QUEUE_SIZE, EVENT_SORT_TYPE_FIFO);
    assert(ethTaskMsgQueue != E_CREATE_FAIL);

    ethTxMsgQueue = CoCreateQueue (ethTxMsgQueueBuf, TX_MSG_QUEUE_SIZE, EVENT_SORT_TYPE_FIFO);
    assert(ethTxMsgQueue != E_CREATE_FAIL);

    SET_WAKEUP_FRAME(wakeupMsg->msgType);
    wakeupMsg->msgLength = 0;

    HwPeriphInit();
    HwTIM3_UpdatePrescaler(config.frameClock);

    USBD_Init(&USB_OTG_dev,
      USB_OTG_FS_CORE_ID,
      &USR_desc,
      &USBD_CDC_cb,
      &USR_cb);

    GetARM_UUID(); // init and load the UUID into the struct

#ifdef Ether_active
    uIP_timer_setup(); // get the timer started
    if (config.flags & FLAG_ETHERNET_ON) {
      if (!Ethernet_Init()) {    // setup the ethernet i/o hardware
      uIP_Setup(config.mac_address, config.my_ip, config.dest_ip,
                   config.netmask, config.my_port, config.dest_port);    // Init the stack
        init_ether_comm();
        flags |= FLAG_ETHERNET_UP;
      }
    }
#endif

    WDTimerInit();

    RadioInit(config.panId, config.mySrcAddr,config.rfChan);

    flagRFBeaconSent = CoCreateFlag(1, 0);  // auto-reset, flag cleared
    flagLoadFrameId = CoCreateFlag(1, 0);


    taskRadioTxId = CoCreateTaskEx(TaskRadioTx, (void*)0, 1, &taskRadioTxStack[OS_STACK_SIZE-1], OS_STACK_SIZE, (uint32_t)0, (uint32_t)0);

    taskConfigId = CoCreateTask  (TaskConfig, (void*)0, 8, &taskConfigStack[OS_STACK_SIZE-1], OS_STACK_SIZE);
#ifdef Ether_active
    taskEtherId = CoCreateTaskEx(Ethertask, (void*)0,7, &EthertaskStack[OS_STACK_SIZE-1], OS_STACK_SIZE,(uint32_t)0, (uint32_t)0);
#endif
    CoCreateTaskEx(WatchDogTask, (void*)0,0, &WatchDogTaskStack[OS_STACK_SIZE-1], OS_STACK_SIZE,(uint32_t)0, (uint32_t)0);
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
  TRACE("Evt counter=%u\r\n", pDiagnostics->event_count);
  TRACE("radioRxErrors=%u ethTxDropped=%u ethTxRecvd=%u\n\r", pDiagnostics->rfTxErrors,
        pDiagnostics->ethRxDropped, pDiagnostics->ethRxRcvdOk);
  TRACE("setLoadFrame=%u setRFbeaconSent=%u setRadioTxDone=%u\n\r",
        pDiagnostics->setLoadFrame, pDiagnostics->setRFBeaconSent,
        pDiagnostics->setRadioTxDone);
  CoTickDelay(10);
  TRACE("setSPIMachineDone=%u setEtherTxDone=%u acceptEtherTxDone=%u\n\r",
        pDiagnostics->setSPIMachineDone, pDiagnostics->setEtherTxDone,
        pDiagnostics->acceptEtherTxDone);
  TRACE("waitLoadFrame=%u waitRFBeaconSent=%u postEthernet=%u\n\r",
        pDiagnostics->waitLoadFrame, pDiagnostics->waitRFBeaconSent,
        pDiagnostics->postEthernetRcvd);
  TRACE("waitRadioTxDone=%u waitSPIMachineDone=%u\r\n",
        pDiagnostics->waitRadioTxDone, pDiagnostics->waitSPIMachineDone);
  TRACE("RadioCycles=%u EtherCycles=%u ConfigCycles=%u\r\n",  pDiagnostics->taskRadioCycles,
    pDiagnostics->taskEtherCycles,
    pDiagnostics->taskConfigCycles);

  TRACE("stopped_task=%d\n\r");
  CoTickDelay(10);
  TRACE("RadioTxTask: func=0x%08X line=%u\n\r",
        pDiagnostics->taskRadioTx_func, pDiagnostics->taskRadioTx_line);
  TRACE("ConfigTask: func=0x%08X line=%u\n\r",
        pDiagnostics->taskConfig_func, pDiagnostics->taskConfig_line);
  TRACE("EtherTask: func=0x%08X line=%u\n\r",
        pDiagnostics->taskEther_func, pDiagnostics->taskEther_line);
  TRACE("\n\r");

}

void GetARM_UUID(void){
    ARM_proc_SN.a = *(__IO uint32_t *)(0x1FFFF7E8);
    ARM_proc_SN.b = *(__IO uint32_t *)(0x1FFFF7EC);
    ARM_proc_SN.c = *(__IO uint32_t *)(0x1FFFF7F0);
}

