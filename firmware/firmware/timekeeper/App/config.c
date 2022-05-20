#include "VersionNo.h"

#include <stm32f10x.h>
#include <string.h>
#include "config.h"
#include "i2c_ee.h"
#include "packets.h"
#include "main.h"
#include "flash_map.h"
#include "basic_rf.h"
#include "tst_dbg.h"


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

int validateConfig(config_t *config)
{
  int ret = 0;
  if (config->activeChannelsBitMask == 0 || config->rfChan < RF_CHANNEL_MIN ||
      config->rfChan > RF_CHANNEL_MAX) {
      config->rfChan = 26;
      config->activeChannelsBitMask = 1 << (config->rfChan - RF_CHANNEL_MIN);
    ret = 1;
  }

  if (config->dest_ip = 0xFFFFFFFF || config->dest_ip == 0) {
      config->dest_ip = ((2 << 24) | (0 << 16) | (0 << 8) | 200);
    ret = 1;
  }
  if (config->dest_port == 0xFFFF || config->dest_port == 0) {
    config->dest_port = TK_Send_Port;
    ret = 1;
  }
  switch (config->frameClock) {
  case FRAME_CLOCK_100:
  case FRAME_CLOCK_120:
  case FRAME_CLOCK_180:
  case FRAME_CLOCK_240:
    break;
  default:
    config->frameClock = FRAME_CLOCK_100;
    ret = 1;
  }

  if (config->mySrcAddr == 0xFFFF || config->mySrcAddr == 0x0) {
    config->mySrcAddr = 0xABCD;
    ret = 1;
  }
  if (config->my_ip == 0xFFFFFFFF || config->my_ip == 0) {
    config->my_ip = ((10 << 24) | (133 << 16) | (1 << 8) | 100);
    ret = 1;
  }
  if (config->my_port == 0 || config->my_port == 0xFFFF) {
      config->my_port = TK_Listen_Port;
      ret = 1;
  }
  if (config->netmask == 0xFFFFFFFF || config->netmask == 0x0) {
    config->netmask = 0xFF000000;
    ret = 1;
  }
  if (config->TxPower == 0  || config->TxPower > 8) {
      config->TxPower = 8;
      ret = 1;
  }
  if (config->panId == 0 && (ret || config->config_version != CURRENT_CONFIG_VERSION)) {
    config->panId = 35;
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

    I2C_EE_BufferRead((uint8_t*) &config->rf_scan, sizeof(ex_config_t), sizeof(uint16_t));

    I2C_EE_BufferRead(&config->frameOffset, OLD_EEPROM_FRAME_OFFSET_ADDRESS, OLD_EEPROM_FRAME_OFFSET_SIZE);
    if (config->frameOffset == 0xFF) {
      config->frameOffset = FRAME_OFFSET_DEFAULT;
    }

    I2C_EE_BufferRead((uint8_t*) &config->time_adjust, OLD_EEPROM_TIMEADJUST_ADDRESS, OLD_EEPROM_TIMEADJUST_SIZE);
    if (((uint16_t) config->time_adjust) == 0xFFFF) {
      config->time_adjust = 0;
    }
    // Do not read frameId24bits - it is set by default
    config->flags |= FLAG_FRAMEID_24BITS;
    /*
    I2C_EE_BufferRead(&config->frameId24bits, EEPROM_FRAMEID24BITS_ADDRESS, EEPROM_FRAMEID24BITS_SIZE);
    if (config->frameId24bits == 0xFF) {
        config->frameId24bits = 1;
    }
*/
    config->frameClock = FRAME_CLOCK_100;
    config->config_version = CURRENT_CONFIG_VERSION;
    validateConfig(config);
    SaveConfig(config);
  } else if (key == CURRENT_CONFIG_VERSION) {
    I2C_EE_BufferRead((uint8_t*) config, EEPROM_CONFIG_ADDRESS, sizeof(*config));
    if (validateConfig(config)) {
      SaveConfig(config);
    }
  } else {
    // do nothing, load defaults..
    validateConfig(config);
    config->config_version = CURRENT_CONFIG_VERSION;
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

