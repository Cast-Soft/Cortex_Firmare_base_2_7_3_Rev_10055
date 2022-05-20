#include "VersionNo.h"

#include <stm32f10x.h>
#include <string.h>
#include "config.h"
#include "i2c_ee.h"
#include "packets.h"
#include "main.h"
#include "flash_map.h"


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
  if (key != 0xFFFF && !IS_VALID_CONFIG_KEY(key)) { // old format
    ex_config_t old_config;
    I2C_EE_BufferRead((uint8_t*) &old_config, EEPROM_CONFIG_ADDRESS, sizeof(old_config));
    CopyExConfigToConfig(&old_config, config);

    config->config_version = CURRENT_CONFIG_VERSION;
    SaveConfig(config);
  } else if (key == CURRENT_CONFIG_VERSION) {
    I2C_EE_BufferRead((uint8_t*) config, EEPROM_CONFIG_ADDRESS, sizeof(*config));
  } else {
    // do nothing, load defaults..
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

