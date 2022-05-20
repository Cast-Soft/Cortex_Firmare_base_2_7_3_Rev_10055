#ifndef _CONFIG_H__
#define _CONFIG_H__

#define VALID_CONFIG_KEY        0x7C00
#define CURRENT_CONFIG_VERSION  (VALID_CONFIG_KEY | 0x02)
#define IS_VALID_CONFIG_KEY(A) ((A & VALID_CONFIG_KEY) == VALID_CONFIG_KEY)

#define FLAG_TRACE_ENABLE                       0x01
#define FLAG_TRACE_UDP                          0x02
#define FLAG_TRACE_CRC                          0x04
#define FLAG_TRACE_ADC                          0x08
#define FLAG_TRACE_USE_PROTO_CMD_LINE_RESP      0x10
#define FLAG_TRACE_USE_TIMESLOT                 0x20
#define FLAG_TRACE_IMU_QUEUE_FULL               0x40
#define FLAG_TRACE_SYNC                         0x80
#define FLAG_TRACE_TIMESLOT                     0x100
#define FLAG_TRACE_BEACON                       0x200
#define FLAG_TRACE_SLOTS                        0x400
#define FLAG_TRACE_ASYNC                        0x800
#define FLAG_TRACE_ADJUST                       0x1000
#define FLAG_SEND_BATTERY_INFO                  0x2000
#define FLAG_DEBUG                              0x4000


#define FLAG_FRAMEID_24BITS     0x80000000
//#define FLAG_ETHERNET_ON        0x40000000
//#define FLAG_ETHERNET_UP        0x20000000      // not persistent, not saved to config
                                                // resides in flags variable, shows
                                                // actual Ethernet status
#define DEFAULT_FRAME_COUNT_NO_SYNC     50      // 5 sec

#include "flash_map.h"

#pragma pack(push, 1)

extern uint16_t offsetDelta;

typedef struct {
  uint16_t      size;
  uint16_t      checksum;
//  from config_t without checksum and u32IwdgResetEvents;
  uint16_t    productID;
  uint16_t    serialNum;
  uint16_t    panId;
  uint16_t    mySrcAddr;
  uint16_t    routerDstAddr;
  uint16_t    ledOnOffs;
  uint16_t    ledOffOffs;
  uint16_t    ledDAC;
  uint8_t     rfChan;
  uint8_t     rfTimeSlot;
  uint8_t     led0Id;
  uint8_t     led1Id;
  uint8_t     led2Id;
  uint8_t     TestMode;
  uint8_t     TxLevel;
  uint8_t     radioPacketFlags;

  uint32_t      led0IdPattern;
  uint32_t      led1IdPattern;
  uint32_t      led2IdPattern;
  uint32_t      led0Index;
  uint32_t      led1Index;
  uint32_t      led2Index;
  uint8_t       frameBits;
//#ifndef OLD_CONFIG
  //uint16_t      imuSlots; // by default 0xFFFF, which means
                          // all slots are for IMU. Any cleared
                          // bit is timeslot dedicated for random sending
                          // battery status. Corresponds to config.rfTimeSlot
  //uint8_t       debounce_time; // *10 to get millisec to debounce button
                               // range 1- 9; default 4 (40 mS)
  //uint8_t       doubleclick_time; // *100 time to wait for double click
                                  // range 1- 9, default 5 (500 mS)
//#endif

} ex_config_t;

typedef struct {
  uint16_t      config_version;
  uint32_t      written_by_ver; // combination of   (BC_VERSION_MAJOR << 24) |
                            //                  (BC_VERSION_MINOR << 16) |
                            //                  (BC_VERSION_REVISION << 8) |
                            //                  (BC_VERSION_BUILD)
 // uint16_t      size;
 // uint16_t      checksum;
//  from config_t without checksum and u32IwdgResetEvents;
  uint16_t    productID;
  uint16_t    serialNum;
  uint16_t    panId;
  uint16_t    mySrcAddr;
  uint16_t    routerDstAddr;
  uint16_t    ledOnOffs;
  uint16_t    ledOffOffs;
  uint16_t    ledDAC;
  uint8_t     rfChan;
  uint8_t     rfTimeSlot;

  uint8_t     led0Id;
  uint8_t     led1Id;
  uint8_t     led2Id;
  uint8_t     TestMode;

  uint8_t     TxLevel;
  uint8_t     radioPacketFlags;

  uint32_t      led0IdPattern;
  uint32_t      led1IdPattern;
  uint32_t      led2IdPattern;
  uint32_t      led0Index;
  uint32_t      led1Index;
  uint32_t      led2Index;
  uint8_t       frameBits;
  uint16_t      imuSlots; // by default 0xFFFF, which means
                          // all slots are for IMU. Any cleared
                          // bit is timeslot dedicated for random sending
                          // battery status. Corresponds to config.rfTimeSlot
  uint8_t       debounce_time; // *10 to get millisec to debounce button
                               // range 1- 9; default 4 (40 mS)
  uint8_t       doubleclick_time; // *100 time to wait for double click
                                  // range 1- 9, default 5 (500 mS)
  uint8_t       hold_time;
  uint8_t       dummyPad;

  uint32_t      flags;
  int16_t       time_adjust;
  uint16_t      frameCountNoSync;

} config_t;


extern config_t config;

extern uint32_t rt_flags;
#define RT_FLAG_USB_CONNECTED         0x100

#pragma pack(pop)

void CopyConfigToExConfig(config_t *config, ex_config_t *exConfig);
int CopyExConfigToConfig(ex_config_t *exConfig, config_t *config);

void LoadConfig(config_t *config);
void SaveConfig(config_t *config);



#endif