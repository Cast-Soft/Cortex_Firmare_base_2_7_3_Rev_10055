#ifndef _CONFIG_H__
#define _CONFIG_H__

#define VALID_CONFIG_KEY        0x7C00
#define CURRENT_CONFIG_VERSION  (VALID_CONFIG_KEY | 0x02)
#define IS_VALID_CONFIG_KEY(A) ((A & VALID_CONFIG_KEY) == VALID_CONFIG_KEY)

#define FLAG_TRACE_ENABLE       0x01
#define FLAG_TRACE_UDP          0x02
#define FLAG_TRACE_CRC          0x04
#define FLAG_TRACE_ADC          0x08

#define FLAG_FRAMEID_24BITS     0x80000000
#define FLAG_ETHERNET_ON        0x40000000
#define FLAG_ETHERNET_UP        0x20000000      // not persistent, not saved to config
                                                // resides in flags variable, shows
                                                // actual Ethernet status

#include "flash_map.h"

#pragma pack(push, 1)

typedef struct {
    uint16_t    productID;
    uint16_t    serialNum;
    uint16_t    panId;
    uint16_t    mySrcAddr;
    uint8_t     rfChan;
    uint8_t     TxPower;
    uint8_t     TestMode;
    uint8_t     SyncOutEn;
    uint32_t    activeChannelsBitMask;

    /* !! ABOVE MUST HAVE EVEN # BYTES!! */
    uint16_t    checksum;   // MUST BE LAST!!
} ex_config_t;

typedef struct {
  uint16_t      config_version;
  uint32_t      written_by_ver; // combination of   (TK_VERSION_MAJOR << 24) |
                            //                  (TK_VERSION_MINOR << 16) |
                            //                  (TK_VERSION_REVISION << 8) |
                            //                  (TK_VERSION_BUILD)
  uint8_t       mac_address[6];
  uint32_t      my_ip;
  uint32_t      dest_ip;
  uint32_t      netmask;
  uint16_t      my_port;
  uint16_t      dest_port;
  uint16_t      productID;
  uint16_t      serialNum;
  uint16_t      panId;
  uint16_t      mySrcAddr;
  uint8_t       rfChan;
  uint32_t      flags;

} config_t;


extern config_t config;


#pragma pack(pop)

void CopyConfigToExConfig(config_t *config, ex_config_t *exConfig);
int CopyExConfigToConfig(ex_config_t *exConfig, config_t *config);

void LoadConfig(config_t *config);
void SaveConfig(config_t *config);



#endif