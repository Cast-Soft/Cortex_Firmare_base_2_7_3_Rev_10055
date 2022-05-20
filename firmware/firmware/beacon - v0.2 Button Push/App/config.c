#include "VersionNo.h"

#include <stm32f10x.h>
#include <string.h>
#include "config.h"
#include "i2c_ee.h"
#include "packets.h"

#include "flash_map.h"

uint16_t offsetDelta = 0;                       // Could be overridden by T.K. sync packet

config_t  backup_config = {
    //.size = sizeof(beacon_config_t) - sizeof(uint16_t)*2,
    //.checksum = 3, //TODO
    .config_version = CURRENT_CONFIG_VERSION,
    .productID      = 0xBC10,
    .serialNum      = 0x444,
    .panId          = 0x23,
    .mySrcAddr      = 0x5678,
    .routerDstAddr      = 0xABCD,
    .ledOnOffs      = 52000,
    .ledOffOffs     = 4000,
    .ledDAC         = 3840,
    .rfChan         = 15,
    .rfTimeSlot     = 2,
    .led0Id         = 0xFE,
    .led1Id         = 0xFD,
    .led2Id         = 0xFB,
    .TestMode       = 0x00,
    .TxLevel        = 8,
    .radioPacketFlags          = 0xFF,
    .led0IdPattern = 0xFFFFFFFF,
    .led1IdPattern = 0xFFFFFFFE,
    .led2IdPattern = 0xFFFFFFFD,
    .led0Index     = 1,
    .led1Index     = 2,
    .led2Index     = 3,
    .frameBits = 8,
#ifndef OLD_CONFIG
    .imuSlots =      0xFFFF,
    .debounce_time = 4, //40 millisec
    .doubleclick_time = 5,      // 500 millisec
    .hold_time = 10,
#endif

    .dummyPad   = 0xFF,
    .flags      =
#ifdef _DEBUG
                  FLAG_TRACE_ENABLE |
#else

#endif
      FLAG_FRAMEID_24BITS | FLAG_TRACE_USE_PROTO_CMD_LINE_RESP,
    .time_adjust = 0,
    .frameCountNoSync   = DEFAULT_FRAME_COUNT_NO_SYNC
};

static uint16_t CalcConfigChecksum(ex_config_t *config) {
    uint16_t checksum = 0xCCCC; // checksum initialization

    // checksum is last half-word in struct
    // checksum field not included in checksum
    uint8_t *p = (uint8_t*) config;
    for (uint16_t i = 0; i < (sizeof(*config)/2 - 1); i++) {

        checksum ^= (i ^ (*p)++);
    }
    return checksum;
}

void CopyConfigToExConfig(config_t *config, ex_config_t *exConfig)
{
  memcpy( (uint8_t*) &exConfig->productID, (uint8_t*) &config->productID, sizeof(*exConfig) - sizeof(uint16_t));
  exConfig->size = sizeof(*exConfig);
  exConfig->checksum = CalcConfigChecksum(exConfig);
}

/* returns 0 if no changes */
int CopyExConfigToConfig(ex_config_t *exConfig, config_t *config)
{
  int8_t ret = memcmp((uint8_t*) &exConfig->productID, (uint8_t*) &config->productID, sizeof(*exConfig) - sizeof(uint16_t));
  if ( ret != 0) {
    memcpy((uint8_t*) &config->productID, (uint8_t*) &exConfig->productID, sizeof(*exConfig) - sizeof(uint16_t));
  }
  return ret;
}

void LoadConfig(config_t *config)
{
  uint16_t key = 0;
  I2C_EE_BufferRead((uint8_t*) &key, EEPROM_CONFIG_ADDRESS, sizeof(uint16_t));
  if (key != 0xFFFF &&  key != 0 && !IS_VALID_CONFIG_KEY(key)) { // old format
    ex_config_t old_config;
    I2C_EE_BufferRead((uint8_t*) &old_config, EEPROM_CONFIG_ADDRESS, sizeof(old_config));
    CopyExConfigToConfig(&old_config, config);

    config->config_version = CURRENT_CONFIG_VERSION;
    config->flags =
#ifdef _DEBUG
                  FLAG_TRACE_ENABLE |
#else

#endif
      FLAG_FRAMEID_24BITS | FLAG_TRACE_USE_PROTO_CMD_LINE_RESP;
    config->time_adjust = 0;
    config->frameCountNoSync   = DEFAULT_FRAME_COUNT_NO_SYNC;

    SaveConfig(config);
  } else if (key == CURRENT_CONFIG_VERSION) {
    I2C_EE_BufferRead((uint8_t*) config, EEPROM_CONFIG_ADDRESS, sizeof(*config));
  } else {
    memcpy(config, (void *)&backup_config, sizeof(*config));
    SaveConfig(config);
  }
}

void SaveConfig(config_t *config)
{
    config->written_by_ver =    (THIS_MAJOR << 24) |
                                (THIS_MINOR << 16) |
                                (THIS_PATCH << 8) |
                                ((char)THIS_REVISION);          // Revision low 8 byte only

    I2C_EE_BufferWrite((uint8_t*) config, EEPROM_CONFIG_ADDRESS, sizeof(*config));
}

