#ifndef _FLASH_MAP__
#define _FLASH_MAP__

#include "stm32f10x.h"

#define FLASH_PROD_AREA                 0x0800B800
#define FLASH_PROD_AREA_SIZE            0x800

#define ApplicationAddress    0x0800C000

#if defined (STM32F10X_MD) || defined (STM32F10X_MD_VL)
 #define PAGE_SIZE                         (0x400)    /* 1 Kbyte */
 #define FLASH_SIZE                        (0x20000)  /* 128 KBytes */
#elif defined STM32F10X_CL
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x40000)  /* 256 KBytes */
#elif defined STM32F10X_HD || defined (STM32F10X_HD_VL)
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x80000)  /* 512 KBytes */
#elif defined STM32F10X_XL
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x100000) /* 1 MByte */
#else
 #error "Please select first the STM32 device to be used (in stm32f10x.h)"
#endif

#define ERROR_FLASH_ADDRESS             0x100
#define ERROR_FLASH_VERIFICATION        0x101
#define ERROR_WRONG_PACKET              0x102
#define ERROR_WRONG_DATA                0x103
#define ERROR_FLASH_TIMEOUT             0x104
#define ERROR_FLASH_PROGRAMM            0x105

#ifdef MAIN_MEMORY
#define MY_ADDRESS                      0x0800C000
// 48k used for Bootloader
#define APP_UPLOAD_ADDRESS              0x08000000
#else
// 48k used for Bootloader
#define APP_UPLOAD_ADDRESS              0x0800C000
#endif

typedef struct {
  uint32_t address;
  uint16_t count;
  uint8_t data[PAGE_SIZE];
}tPage;

// 0x00 - 0x7D - config area
#define EEPROM_CONFIG_ADDRESS   0x00
#if defined(TIMEKEEPER_APP) || defined(ROUTER_APP)

  #define OLD_EEPROM_FRAME_OFFSET_ADDRESS    0x78 //Yes, same place as OLD_EEPROM_DESTIP_ADDRESS//0x6D
  #define OLD_EEPROM_FRAME_OFFSET_SIZE       0x01

  #define OLD_EEPROM_MAC_ADDRESS      0x6E    /* 6 bytes */
  #define OLD_EEPROM_MYIP_ADDRESS     0x74    /* 4 bytes */
  #define OLD_EEPROM_DESTIP_ADDRESS   0x78    /* 4 bytes */
  #define OLD_EEPROM_NETMASK_ADDRESS  0x7C    /* 4 bytes */

#endif

// 0x7E - 0x7F - common area between bootloader and main app

#if defined(BEACON) || defined(BEACON_APP)

  #define BEACON_FLAG_JUMP_TO_MAIN        0x01    /* If cleared (0xFE value), bootloader jumps*/
  #define EEPROM_BEACON_FLAG_ADDRESS      0x7F    /* 1 byte common flag - used to tell
                                           bootloader to stay in bootloader */
  #define EEPROM_BEACON_FLAG_SIZE         0x01

#endif

#if defined(BEACON_APP) || defined(TIMEKEEPER_APP)

  #define OLD_EEPROM_FRAMECLOCK_ADDRESS               0x7A
  #define OLD_EEPROM_FRAMECLOCK_SIZE                  0x01

  #define OLD_EEPROM_FRAMEID24BITS_ADDRESS            0x7B    // 1 bytes
  #define OLD_EEPROM_FRAMEID24BITS_SIZE               0x01

  #define OLD_EEPROM_FRAMECOUNT_NOSYNC_ADDRESS        0x7C    // 2 bytes
  #define OLD_EEPROM_FRAMECOUNT_NOSYNC_SIZE           0x02

  #if defined(BEACON_APP)
    #define OLD_EEPROM_TIMEADJUST_ADDRESS               0x78    // 2 bytes
    #define OLD_EEPROM_TIMEADJUST_SIZE                  0x02
  #else //TIMEKEEPER_APP
    #define OLD_EEPROM_TIMEADJUST_ADDRESS               0x7E    // 2 bytes
    #define OLD_EEPROM_TIMEADJUST_SIZE                  0x02
  #endif
#endif

// 0x80 - 0xDF - debug area
#define EEPROM_DEBUG_DIAGNOSTIC                       0x80
// size is arbitrary, will allocate 64 bytes,
// will save actual size, this is to indicate allocated space
#define EEPROM_DEBUG_DIAGNOSTIC_SIZE                  0x80


#define EEPROM_DEBUG_STACKED_LR                       0xD0
#define EEPROM_DEBUG_STACKED_LR_SIZE                    4
#define EEPROM_DEBUG_STACKED_PC                       0xD4
#define EEPROM_DEBUG_STACKED_PC_SIZE                    4
#define EEPROM_DEBUG_STACKED_PSR                      0xD8
#define EEPROM_DEBUG_STACKED_PSR_SIZE                   4
#define EEPROM_DEBUG_COUNTER                          0xDC
#define EEPROM_DEBUG_COUNTER_SIZE                       4

#define EEPROM_APP_ADDRESS      0xE0    /* At 0xE0 - 0x20 bytes of data
                                           may be read/saved
                                           by UI application - firmware does not
                                           use this memory*/
#define EEPROM_APP_SIZE         0x20

#endif
