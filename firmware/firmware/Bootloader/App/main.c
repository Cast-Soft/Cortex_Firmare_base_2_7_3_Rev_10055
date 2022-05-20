/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
* File Name          : main.c
* Author             : ?
* Version            : V1.0
* Date               : 09/17/2013
* Description        : Main Application for Bootloader
*******************************************************************************/


/* INCLUDES ------------------------------------------------------------------*/
#include "VersionNo.h"

#ifdef USE_RADIO
#include "basic_rf.h"
#endif

#include "stm32f10x_it.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <yfuns.h>
#include "stm32f10x_flash.h"
#ifdef BEACON
#include "stm32f10x_adc.h"
#endif
#include "main.h"
#include "hardware.h"
#include "common.h"

#include "usb_regs.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "base64.h"
#include "packets.h"
#include "md5.h"
#include "flash_map.h"
#include "util.h"
#include "i2c_ee.h"
#include "console_tail.h"

//# define USB_NEW_DETECTION

//typedef  void (*pFunction)(void);
pFunction JumpToApp;

uint32_t flags;
extern uint8_t on_usb_power;

uint8_t hasMainFirmware = 0;

unsigned long long     OSTickCnt = 0;                  /*!< Current system tick counter      */
#ifdef MAIN_MEMORY
#ifdef BEACON
#define BOOTLOADER_VERSION  "Beacon Bootloader Upgrader Built on "__DATE__" "__TIME__ "\r\n"
#define SHORT_NAME "UG_BC"
#else
 #ifdef ROUTER
  #define  BOOTLOADER_VERSION  "Router Bootloader Upgrader Built on "__DATE__" "__TIME__ "\r\n"
  #define SHORT_NAME "UG_RT" 
 #else
  #ifdef TIMEKEEPER
   #define  BOOTLOADER_VERSION  "Timekeeper Bootloader Upgarder Built on "__DATE__" "__TIME__ "\r\n"
   #define SHORT_NAME "UG_TK" 
  #else
   #define  BOOTLOADER_VERSION  "Cast-soft Bootloader Upgrader Built on "__DATE__" "__TIME__ "\r\n"
   #define SHORT_NAME "UG_UG"
  #endif
 #endif
#endif

#else
#ifdef BEACON
#define  BOOTLOADER_VERSION "Beacon Bootloader Built on "__DATE__" "__TIME__ "\r\n"
#define SHORT_NAME "BL_BC"
#else
 #ifdef ROUTER
  #define  BOOTLOADER_VERSION  "Router Bootloader Built on "__DATE__" "__TIME__ "\r\n"
  #define SHORT_NAME "BL_RT"
 #else
  #ifdef TIMEKEEPER
   #define  BOOTLOADER_VERSION  "Timekeeper Bootloader Built on "__DATE__" "__TIME__ "\r\n"
   #define SHORT_NAME "BL_TK"
  #else
   #define  BOOTLOADER_VERSION  "Cast-soft Bootloader Built on "__DATE__" "__TIME__ "\r\n"
   #define SHORT_NAME "BL_BL" 
  #endif
 #endif
#endif
#endif

#ifdef DEBUG
const char *version =  BOOTLOADER_VERSION"\n\r FIRMWARE VERSION: " STRINGIFY(THIS_MAJOR) "." STRINGIFY(THIS_MINOR) "." STRINGIFY(THIS_PATCH) "." STRINGIFY(THIS_REVISION) " " SHORT_NAME " DEBUG \n\r\n\r";
#else
const char *version =  BOOTLOADER_VERSION"\n\r FIRMWARE VERSION: " STRINGIFY(THIS_MAJOR) "." STRINGIFY(THIS_MINOR) "." STRINGIFY(THIS_PATCH) "." STRINGIFY(THIS_REVISION) " " SHORT_NAME " RELEASE \n\r\n\r";
#endif

/* PRIVATE TYPEDEF -----------------------------------------------------------*/

typedef enum {
    DEC, HEX, BIN
} OutputMode_t;


/* PRIVATE DEFINES -----------------------------------------------------------*/

#define FLASH_CONFIG_PAGE   0x0803F800   // Page#127 (last page of main memory)


/* PRIVATE MACROS ------------------------------------------------------------*/

/* EXTERN VARIABLES ----------------------------------------------------------*/
#if (defined USE_MY_ASSERT) || (defined USE_FULL_ASSERT)
static uint8_t assert_loop = 1;
#endif

#ifdef USE_ETHERNET
extern void OnEtherMsg();
#define WAIT_TIMEOUT    5000
#else
#define WAIT_TIMEOUT    1000
#endif
extern uint32_t SysTickCounter;

/* PRIVATE VARIABLES ---------------------------------------------------------*/

uint32_t flags = 0;
__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;


#ifdef _DEBUG
uint32_t TRACE_ENABLE = 1;
#else
uint32_t TRACE_ENABLE = 0;
#endif

/*
extern struct GenericSendHddr *pGenericSendHddr;
extern uint32_t BTHostTimeOut;
void *Memtest,*LastMem,*FirstMem;
extern uint32_t IMU_Dbg_Prt, SeqNum_Dbg_Prt;
*/

#ifdef USE_ETHERNET
void uIP_Loop();
void uIP_timer_setup();
void EthernetInit();
void uIP_Setup();
#endif

/*PUBLIC VARIABLES ----------------------------------------------------------*/

#ifdef USE_ETHERNET
// EtherTask will be waiting and processing msg in queue
#define TASK_MSG_QUEUE_SIZE 40       // TODO!!! test to get a better value
//OS_EventID ethTaskMsgQueue;
//void *ethTaskMsgQueueBuf[TASK_MSG_QUEUE_SIZE];

MsgHeader wakeupMsgEntity;
MsgHeader* wakeupMsg = &wakeupMsgEntity;


// for outbound ethernet frames
#define TX_MSG_QUEUE_SIZE 12       // TODO!!! test to get a better value
//OS_EventID ethTxMsgQueue;
void *ethTxMsgQueueBuf[TX_MSG_QUEUE_SIZE];
#endif


/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/
void WriteConfig(void);


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/*******************************************************************************
* Description : Write Configuration to FLASH
* Input       : -
* Return      : -
*******************************************************************************/
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

        if (FLASH_ProgramWord(page->address + offset, word) != FLASH_COMPLETE) {
            result = ERROR_FLASH_PROGRAMM;
            page->count = offset;
            break;
        }
    }
    if (result != 0) {
      break;
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

uint8_t tab_1024[1024];

static tPage page = {APP_UPLOAD_ADDRESS};
//static uint32_t CurrentUploadAddress = APP_UPLOAD_ADDRESS;
static uint16_t CurrentDataPointer = 0x00;
static uint16_t packet = 0;

static struct PacketHeader *header;
uint32_t SerialDownload(const char* decoded, uint16_t len, uint8_t type)
{
  struct FirmwarePacketHeader *frmHeader;

  uint8_t copy_len = 0;

  uint32_t ret = 0;
  // Data comes in format:
  //U 0 0 DATA\r\n
  // so, DATA starts from 6th index
  flags |= FLAG_LOADING;
  flags &= ~FLAG_LOADED;
  do {
    if (len < (sizeof(struct FirmwarePacketHeader))) {
      ret = ERROR_WRONG_PACKET;
      break;
    }
    frmHeader = (struct FirmwarePacketHeader *) (decoded);
    copy_len = header->size - /*7*/ (sizeof(struct PacketHeader) + sizeof(struct FirmwarePacketHeader));
    if (
#ifdef MAIN_MEMORY
        ((type == DEVCMD_UPD_PACKET) &&
        (copy_len > 128))
#else
        ((type == DEVCMD_UPD_PACKET) &&
        (copy_len > 128 || (copy_len + CurrentDataPointer) > PAGE_SIZE))
        ||
        ((type == DEV_CMD_SET_PROD_AREA) &&
        (copy_len > 128 || (copy_len + CurrentDataPointer) > (FLASH_PROD_AREA + FLASH_PROD_AREA_SIZE)))
#endif
        ||
        frmHeader->index != packet) {
      ret = ERROR_WRONG_PACKET;
      break;
    }
#ifndef MAIN_MEMORY
     if (type == DEV_CMD_SET_PROD_AREA && frmHeader->index == 0) {
      page.address = FLASH_PROD_AREA;
     }
#endif
    if (copy_len) {
#ifdef MAIN_MEMORY
      //Special case for MAIN_MEMORY:
      //First 2048 bytes are not flashed, instead we copy our first PAGE_SIZE (2048) bytes
      //to start location and remember PAGE_SIZE bytes which we received for the future
      //As at start Cortex M3 has a vector table, replacing current vector table with
      //our should allow us to simply jump to out "Bootloader Upgrader" if somewhere
      //on the road bootloader upgarde fails. Only when last sector is written,
      //we will replace first PAGE_SIZE bytes with real ones, previously saved.
      static uint8_t originalFirstPacket[PAGE_SIZE];
      static uint16_t originalCount;
      if (frmHeader->index == 0) {
        originalCount = 0;
        //Copy our 2048 bytes
        page.address = MY_ADDRESS;
        page.count = PAGE_SIZE;
        FLASH_ClearFlag(FLASH_FLAG_PGERR);
        ReadPage(&page);
        page.address = APP_UPLOAD_ADDRESS;
        WritePage(&page);
        //at this point we shuld be safe
        page.address = APP_UPLOAD_ADDRESS + PAGE_SIZE;
        page.count = 0;
        CurrentDataPointer = 0;
      }
      if (originalCount < PAGE_SIZE) {
        uint16_t copy_count = PAGE_SIZE - originalCount;
        if (copy_len < copy_count) {
          copy_count = copy_len;
        }
        memcpy(originalFirstPacket + originalCount, decoded + /*4*/ sizeof(struct FirmwarePacketHeader), copy_count);
        originalCount += copy_count;
        copy_len -= copy_count;
      }

      if (copy_len) {
        //copy to flash memory
        memcpy(&page.data[CurrentDataPointer], (decoded + /*4*/ sizeof(struct FirmwarePacketHeader)), copy_len);
        CurrentDataPointer += copy_len;
        page.count += copy_len;
        if ((frmHeader->index + 1) == frmHeader->count || CurrentDataPointer == sizeof(page.data)) {
          //time to write
          WritePage(&page);
          page.address += CurrentDataPointer;
          CurrentDataPointer = 0;
          page.count = 0;
        }
        if ((frmHeader->index + 1) == frmHeader->count) {
          //time to write saved first page.
          memcpy(&page.data[0], originalFirstPacket, PAGE_SIZE);
          page.address = APP_UPLOAD_ADDRESS;
          page.count = PAGE_SIZE;
          //need to wait for previous complete

          if (FLASH_WaitForLastOperation(10000000) != FLASH_COMPLETE) {
            ret = ERROR_FLASH_TIMEOUT;
          } else {
            WritePage(&page);
          }
          flags |= FLAG_LOADED;
          flags &= ~FLAG_LOADING;
        }
      }
#else
      memcpy(&page.data[CurrentDataPointer], (decoded + /*4*/ sizeof(struct FirmwarePacketHeader)), copy_len);
      CurrentDataPointer += copy_len;
      page.count += copy_len;
      if ((frmHeader->index + 1) == frmHeader->count || CurrentDataPointer == sizeof(page.data)) {
        //time to write
        WritePage(&page);
        page.address += CurrentDataPointer;
        CurrentDataPointer = 0;
        page.count = 0;
        if ((frmHeader->index + 1) == frmHeader->count) {
          // finished
          page.address = APP_UPLOAD_ADDRESS;
          flags |= FLAG_LOADED;
          flags &= ~FLAG_LOADING;
        }
      }
#endif
    }
    ret = frmHeader->index << 16;
  } while (0);
  // reset for error
  if (ret & 0xFFFF) {
    CurrentDataPointer = 0;
    page.address = APP_UPLOAD_ADDRESS;
    page.count = 0;
    packet = 0;
    flags &= ~FLAG_LOADING;
  } else {
    packet++;
  }
  return ret;
}

uint32_t GetMd5(const uint8_t *decoded, uint16_t len, char *md5)
{
  uint32_t ret = 0;
  unsigned int processed =0;
  struct FirmwareVerifyPacketHeader *verify;
  MD5_CTX ctx;
  uint8_t buf[64];

  do {
    if (len < (sizeof(struct FirmwareVerifyPacketHeader))) {
      ret = ERROR_WRONG_PACKET;
      break;
    }

    verify = (struct FirmwareVerifyPacketHeader *) (decoded);
    if (verify->address < 0x08000000 || verify->address > 0x0801FFFF ||
        (verify->address + verify->count) > 0x0801FFFF) {
          ret = ERROR_WRONG_DATA;
          break;
    }

    MD5Init(&ctx);

    do {
      memset(buf, 0, sizeof(buf));
      processed = 0;
      for (int i = 0; i < sizeof(buf); i++) {
        if (!verify->count) {
          break;
        }

        buf[i] = *((uint8_t*) verify->address);
        verify->address++;
        verify->count--;
        processed++;
      }
      MD5Update(&ctx, buf, processed);
    }while(verify->count);

    MD5Final(md5, &ctx);

  } while (0);

  return ret;
}
uint32_t doPackets(const char *encoded, uint8_t peer)
{
  struct FirmwarePacketHeader *frmHeader;

  uint8_t _decoded[256];
  uint8_t *decoded = &_decoded[0];
  uint8_t copy_len = 0;

  int ret = 0;
  // Data comes in format:
  //U 0 0 DATA\r\n
  // so, DATA starts from 6th index
  uint16_t len = strlen(encoded);
  do {
    if (len < 6) {
      ret = ERROR_WRONG_PACKET;
      break;
    }
    len -= 6;
    ret = b64_pton(encoded +6, decoded, sizeof(_decoded));
    if (ret <= 2) {
      ret = ERROR_WRONG_PACKET;
      break;
    }

    if (ValidateCommandLine((char*)decoded, ret) == 0) {
      ret = ERROR_WRONG_PACKET;
      break;
    }       
        
    header = (struct PacketHeader *) decoded;
    decoded += sizeof(struct PacketHeader);
#if 0 //defined(TIMEKEEPER) || defined(ROUTER)
    // set flag to avoid to jump to firmware
    // after 5 seconds
    flags |= FLAG_UDP_CONNECT;
#endif
#ifdef MAIN_MEMORY
    if (header->type == DEVCMD_UPD_PACKET) {
#else
    if (header->type == DEVCMD_UPD_PACKET || header->type == DEV_CMD_SET_PROD_AREA) {
#endif
      struct RespUpdate up;
      flags |= FLAG_LOADING;
      flags &= ~FLAG_LOADED;
      uint8_t type = DEV_RESP_UPD;
#ifndef MAIN_MEMORY
      if (header->type == DEV_CMD_SET_PROD_AREA) {
        type = DEV_RESP_SET_PROD_AREA;
      }
#endif
      ret = SerialDownload(decoded, ret, header->type);
      up.index = (ret & 0xFFFF0000) >> 16;
      up.errorCode = ret & 0xFFFF;
      if (peer == PEER_USB) {
        __writeCmdLineRespPacket((unsigned char*) &up, 4, type);
#if defined(TIMEKEEPER) || defined(ROUTER)
      } else {
        __writeNetworkResponse(type, (unsigned char*) &up, 4);
#endif
      }
    } else if (header->type == DEV_CMD_FIRMWARE_VERIFY) {
      struct RespFirmwareSum resp;
      uint8_t md5[16];
      ret = GetMd5(decoded, ret, md5);
      resp.errorCode = (ret & 0xFFFF);
      if (!ret) {
        memcpy(resp.sum, md5, sizeof(md5));
      }
      if (peer == PEER_USB) {
        __writeCmdLineRespPacket((unsigned char*) &resp, sizeof(resp), DEV_RESP_FIRMWARE_SUM);
#if defined(TIMEKEEPER) || defined(ROUTER)
      } else {
        __writeNetworkResponse(DEV_RESP_FIRMWARE_SUM, (unsigned char*) &resp, sizeof(resp));
#endif
      }
    } else if(header->type == DEV_CMD_GET_VERSION) {
        struct RespFirmwareVersion respFirm;
        respFirm.major = THIS_MAJOR;
        respFirm.minor = THIS_MINOR;
        respFirm.patch = THIS_PATCH;
        respFirm.reserved = 0;                       // For compatible reason
        respFirm.revision = THIS_REVISION;
#ifdef MAIN_MEMORY
        strcpy((char*) respFirm.dateString, "Bootloader Upgrader "__DATE__);
#else
        strcpy((char*) respFirm.dateString, __DATE__);
#endif
        strcpy((char*) respFirm.timeString, __TIME__);
        ret = 0;
        if (peer == PEER_USB) {
          __writeCmdLineRespPacket((void *)&respFirm,  sizeof(respFirm), DEV_RESP_VERSION);
#if defined(TIMEKEEPER) || defined(ROUTER)
        } else {
          __writeNetworkResponse(DEV_RESP_VERSION, (void *)&respFirm,  sizeof(respFirm));
#endif
        }
#ifndef MAIN_MEMORY
      }else if (header->type == DEV_CMD_GET_PROD_AREA) {
        struct GetProdAreaPacketHeader *prodHeader = (struct GetProdAreaPacketHeader*) decoded;
        static tPage page;
        struct RespProdArea resp;
        page.address = FLASH_PROD_AREA;
        page.count = 2048;
        ret = ReadPage(&page);
        resp.errorCode = ret;
        resp.index = prodHeader->index;
        if (!ret) {
          memcpy(resp.data, &page.data[prodHeader->index*sizeof(resp.data)], sizeof(resp.data));
        }
        if (peer == PEER_USB) {
          __writeCmdLineRespPacket((unsigned char*) &resp, sizeof(resp), DEV_RESP_GET_PROD_AREA);
#if defined(TIMEKEEPER) || defined(ROUTER)
        } else {
          __writeNetworkResponse(DEV_RESP_GET_PROD_AREA, (unsigned char*) &resp, sizeof(resp));
#endif
        }
#endif
    }
  } while (0);
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


void jump() {
    uint32_t JumpAddress = *(__IO uint32_t*) (ApplicationAddress + 4);
#if 0           // Not working. Maybe main firmware has this signature as well. Also caller should avoid reentering here as well
    if (JumpAddress == 0xFFFFFFFF) {
      // No program in flash
      return;
    }
#endif    
    JumpToApp = (pFunction) JumpAddress;

    // turn off USB
    DCD_DevDisconnect(&USB_OTG_dev);
    USB_OTG_DisableCommonInt(&USB_OTG_dev);

    //Jump to user application
   // uint32_t ApplicationAddress = 0x8010000;

#ifndef REVJ_BEACON
    HwLEDOff(LED1);
    HwLEDOn(LED2);
#endif

    // MUST ENABLE INTERRUPTS IN MAIN APP
    __disable_interrupt();
    //Initialize user application's Stack Pointer
    __set_MSP(*(__IO uint32_t*) ApplicationAddress);
    JumpToApp();
}

uint32_t result;

/*******************************************************************************
* Description    : [Task] Implements Simple Text Command Console
* Input          :
* Return         :
*******************************************************************************/
static void onLineIn (char* data){

  static char cmd;
  static int addr;
  static int32_t value;
     
  uint8_t loading = 0;

  if (ValidateCommandLine(data, strlen(data)) == 0) {
    TRACE("Incorrect packet\r\n");
    return;
  }
  
  do {
    // parse the line of hexadecimal values
    cmd = *data;
    char *s = data;
#if defined(TIMEKEEPER) || defined(ROUTER)
    flags |= FLAG_BOOTLOADER_CONNECTED;
#endif
      switch (cmd) {
#if defined(BEACON) || defined(ROUTER)  || defined(TIMEKEEPER)
        case 'v': // version
              printf(/*BOOTLOADER_VERSION*/ version);
              break;
#endif           
        case 'U':
          {
            loading = 1;
             result = doPackets(data, PEER_USB);
             if (result & 0xFFFF) {
               loading = 0;
             } else {
              loading = (flags & FLAG_LOADING);
             }
          }
          break;
#ifndef MAIN_MEMORY
        case 'B':
          {
              uint32_t stackPointer = ApplicationAddress;
              uint32_t val = *((uint32_t*) stackPointer);
              if ((val & 0xFFF00000) == 0x20000000) {
                jump();
              }
          }
          break;
#endif
      case 't':
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
            
          if (addr == 8) {              // WhoAMI command
            struct WhoAmI me;
            memcpy(me.id, (uint8_t *)(0x1FFFF7E8), 12);
#ifdef MAIN_MEMORY
            me.module = 3;
#else
            me.module = 1;
#endif

#if defined(TIMEKEEPER)
        me.type = 1;
#else
  #if defined(ROUTER)
        me.type = 2;
  #else
    #if defined(BEACON)
         me.type = 3;
    #endif
  #endif
#endif
            __writeCmdLineRespPacket((unsigned char*) &me, sizeof(me), DEV_RESP_WHOAMI);
          }
          else if (addr == 52) {
            uint32_t ram = (uint32_t) value & 0xFFFFFFFE;
            if (ram >= 0x20000000 && ram <= 0x2000ffff) {
              uint32_t atRam = *((uint32_t*) (ram));
              TRACE("Mem at 0x%08X=0x%08X\n\r", ram, atRam);
            } else {
              TRACE("Mem addr error\n\r");
            }
          }      
        }
        break;
#if defined(BEACON) || defined(ROUTER) || defined(TIMEKEEPER)
          case '!': // special message
  #if defined(BEACON)
              // if we jump to bootloader, first we have to clear EEPROM
              // in order to stop in bootloader
              uint8_t jumpToMain = 0xFF;
              I2C_EE_BufferRead((uint8_t*) &jumpToMain, EEPROM_BEACON_FLAG_ADDRESS, 1);
              if (jumpToMain != 0xFF) {
                jumpToMain = 0xFF;
                I2C_EE_BufferWrite((uint8_t*) &jumpToMain, EEPROM_BEACON_FLAG_ADDRESS, 1);
              }
  #endif
            NVIC_GenerateSystemReset();
            break;
          case '~': // Sends test ether packet

                break;
#endif
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
    } while (0);
  if (!loading) {
    CurrentDataPointer = 0;
    page.address = APP_UPLOAD_ADDRESS;
    page.count = 0;
    packet = 0;
    flags &= ~FLAG_LOADING;

  }
}

static void power_off()
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  GPIO_ResetBits(GPIOA, GPIO_Pin_10);
  flags |= 0;
  __disable_interrupt();

#ifndef REVJ_BEACON
  HwLEDOff(LED1);
#endif

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_ResetBits(GPIOA, GPIO_Pin_9);
  while(1);

}

static void ConfigureGPIO()
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* Enable the GPIO Power Clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /*PA9 - USB power detect*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // For MAIN_MEMORY we only care about USB power
#ifdef BEACON
#ifndef MAIN_MEMORY

    /* Configure the GPIO Power button input pin */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // buttons A && B
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

#endif
#endif

}

#ifdef BEACON
static void LockPowerSupply()
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* Enable the GPIO Power Clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /*PA10 - switch to power on*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_SetBits(GPIOA, GPIO_Pin_10);

}

static uint8_t BatteryHasGoodLevel()
{
  ADC_InitTypeDef ADC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  uint16_t ADC1ConvertedValue = 0, ADC3ConvertedValue = 0;

  /* Enable the ADC_EN Power Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  /*PC7 - ADC_EN*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_Init(GPIOC, &GPIO_InitStructure);

// Init the data pointer to the battery data conversion union type
//        pBattUnion = &BattUnion;
//        BattUnion.Battery_AtoD= 0xD700; // int to 4.0 volts

    /* Enable peripheral clocks --------------------------------------------------*/
    RCC_ADCCLKConfig(RCC_PCLK2_Div4);

    /* Enable ADC1 and GPIOC clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);

    /* Configure  PC.03 , ADC Channel13  as analog inputs */
    // GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3 ;
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    // GPIO_Init(GPIOC, &GPIO_InitStructure);
    /* Configure  PA.06 , ADC Channel6  as analog inputs */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ADC1 configuration ------------------------------------------------------*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    /* ADC1 regular channels configuration */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_28Cycles5);
    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);
    /* Enable ADC1 reset calibaration register */
    ADC_ResetCalibration(ADC1);
    /* Check the end of ADC1 reset calibration register */

    while(ADC_GetResetCalibrationStatus(ADC1));

    /* Start ADC1 calibaration */
    ADC_StartCalibration(ADC1);
    /* Check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(ADC1));

    /* Start ADC1 Software Conversion */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)== RESET){
            ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    }

    GPIO_SetBits(GPIOC, GPIO_Pin_7);
    // wait 1 for input to stabilise
    uint32_t ticks = SysTickCounter + 3;
    while (ticks > SysTickCounter);
        // trigger a to d
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        // wait 1 for data sample
    ticks = SysTickCounter + 3;
    while (ticks > SysTickCounter);
        // get sample data, disable input
    //55040 == 4.0V
    //49530 == 3.6V
    uint16_t value = (ADC_GetConversionValue(ADC1));
    GPIO_ResetBits(GPIOC, GPIO_Pin_7);
    return (value > 49530)?1:0;
}

#endif

uint32_t val ;
uint8_t stayInFlag = 0x00;

uint8_t checkLatency = 1;
uint32_t bootLatency[5];
//#define CheckBootLatency(x) { assert(x < 5); if (checkLatency) bootLatency[x] = SysTickCounter; }
#define CheckBootLatency(x) 

/* PUBLIC FUNCTION PROTOTYPES ------------------------------------------------*/

/*******************************************************************************
* Description    : Main routine.
* Input          : -
* Return         : -
*******************************************************************************/
void  main(void) {

  uint32_t power = 0;
  flags = 0;

#ifdef MAIN_MEMORY
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0xC000);
#endif

    SysTick_Config(SystemCoreClock / 1000); CheckBootLatency(0); // SysTick Interrupt Freq = (1/1000)s = 1ms

    //HwPeriphInit();
#ifdef BEACON
    // BEACON - ALL
    LockPowerSupply();
#endif
    ConfigureGPIO();

#ifdef BEACON
  #ifndef MAIN_MEMORY
    HwTIM1Init();
  #endif
#endif

    HwI2CInit();

    uint8_t has_usb_power = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9);
   
    {
        uint32_t stackPointer = ApplicationAddress;
        val = *((uint32_t*) stackPointer);
        if ((val & 0xFFF00000) == 0x20000000) {
          hasMainFirmware = 1;
        }
    }
#ifdef BEACON

  #ifndef MAIN_MEMORY
    // BEACON - BOOTLOADER

    /*  Step 1: If no USB power - check voltage
     *  treshold is 3.6V  - 3.6 V or less is not enough,
     *  considered as Beacon turning on itself IF there is
     *  no button pressed
     *  //Check for condition when Beacon powers on itself.
     *  //If PA0 is "0" (button is not pressed) and
     *  //no USB voltage on PA9, do not turn on!.
     *
     */
    if (!has_usb_power) {
      // check voltage
      if (!BatteryHasGoodLevel()){
        // check for button press in a loop
        uint16_t loop = 10000;
        while (--loop) {
          if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_SET) {
            break;
          }
        }
        if (loop == 0) {
          power_off();
        }
      }
      
      if (hasMainFirmware) {
        jump();
      }
    }

    //Check for pressed A or B button, if pressed, stay in bootloader
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == RESET ||
          GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == RESET) {
           stayInFlag = 0xFF;
    } else {
        // Check EEPROM EEPROM_BEACON_FLAG_ADDRESS address, if 0xFF, stay in bootloader
        I2C_EE_BufferRead((uint8_t *) &stayInFlag, EEPROM_BEACON_FLAG_ADDRESS, 1);
    }

  #endif  //MAIN_MEMORY
#endif  //BEACON

#ifdef MAIN_MEMORY
    __enable_interrupt();
#endif

#ifdef USE_ETHERNET
    uIP_timer_setup(); // get the timer started

    EthernetInit();    // setup the ethernet i/o hardware
    uIP_Setup();    // Init the stack
#endif
    CheckBootLatency(1);
      USBD_Init(&USB_OTG_dev,
            USB_OTG_FS_CORE_ID,
            &USR_desc,
            &USBD_CDC_cb,
            &USR_cb);

#ifndef REVJ_BEACON
#if defined(BEACON) || defined(ROUTER) || defined(TIMEKEEPER)
      HwLEDInit(LED1);
      HwLEDInit(LED2);
      HwLEDInit(LED3);
      HwLEDInit(LED4);
      HwLEDOff(LED1);
      HwLEDOff(LED2);
      HwLEDOff(LED3);
      HwLEDOff(LED4);
#endif
#endif

    CheckBootLatency(2);

    while (1) {
#ifndef USB_NEW_DETECTION
      //If more then 1 second passed and not connected to USB
      //check for valid program and jump there if such program
      //loaded
      //UPDATE (Beacon): apparently it is not enough to rely on
      //FLAG_USB_CONNECTED, as sometimes flag is not set
      //especially it may be the case with simple battery charger
      //PC9 is connected to battery charger status.
      //May rely on this for now

#ifndef MAIN_MEMORY
      // moved back to status
      if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9)) {
        flags |= FLAG_USB_CONNECTED;
      }
#endif
      
#else
      
    extern uint32_t APP_Tx_ptr_out;
    extern uint32_t APP_Tx_ptr_in;
    
    if (!(flags & FLAG_USB_CONNECTED)) {
      if (APP_Tx_ptr_out != APP_Tx_ptr_in) {
        flags |= FLAG_USB_CONNECTED;
      }
    }
#endif
    
#ifdef BEACON
      if (SysTickCounter > 2000 && GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {
       {
          if (power == 0) {
            power = SysTickCounter;
          } else {
            if (SysTickCounter - power > 1300 ||
                power - SysTickCounter > 1300) {
                  // turn power off
                  power_off();
                }
          }
        }
      } else {
        power = 0;
      }
#endif
#ifndef MAIN_MEMORY
#ifdef BEACON
      if (SysTickCounter > 1000 && ((flags & FLAG_USB_CONNECTED) == 0 ||
          stayInFlag != 0xFF)) {
#else
      if (SysTickCounter > WAIT_TIMEOUT && (
          (flags & FLAG_BOOTLOADER_CONNECTED) == 0)) {
#endif
        if (hasMainFirmware) {
          jump();
        }
      }
#endif

      /* MAIN LOOP */
#ifdef USB_NEW_DETECTION
      if (flags & FLAG_USB_CONNECTED) {
#endif
        
      static uint8_t line[260];
      static uint8_t next_char = 0;
      int8_t c = getchar();
#ifdef USE_ETHERNET
      if (flags & FLAG_ETHER_RX_PACKET) {
        OnEtherMsg();
        flags &= ~FLAG_ETHER_RX_PACKET;
      }
#endif
    CheckBootLatency(3);
#ifdef BEACON
#ifndef MAIN_MEMORY
      if (!on_usb_power && hasMainFirmware) {
         jump();
      }
#endif
#endif

      if (c != EOF) {
        //putchar(c);
        line[next_char++] = (char) c;
        if ((c == 0x0D) || (next_char == sizeof(line))) {
            line[--next_char] = 0;
            onLineIn((char*) line);
            next_char = 0;
        }
      }
      }
#ifdef USB_NEW_DETECTION      
    }
#endif
      CheckBootLatency(4); checkLatency = 0;   
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
    /*    printf("!!!ASSERT FAILED!!! %s : %d\n\r", file, line);
        HwLEDOn(LED1); HwLEDOn(LED4); HwLEDOff(LED2); HwLEDOff(LED3);
        DelayMs(50);
        HwLEDOn(LED2); HwLEDOn(LED3); HwLEDOff(LED1); HwLEDOff(LED4);
        DelayMs(50);*/
    }
    assert_loop = 1;
}
#endif




