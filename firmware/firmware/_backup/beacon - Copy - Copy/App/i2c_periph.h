/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
* File Name          : basic_rf.h
* Author             : ?
* Version            : V1.0
* Date               : 6/2/2011
* Description        : ?
*******************************************************************************/

/* DEFINE TO PREVENT RECURSIVE INCLUSION -------------------------------------*/
#ifndef __I2C_PERIPH_H
#define __I2C_PERIPH_H

/* INCLUDES ------------------------------------------------------------------*/

#include <stdint.h>

/* EXPORTED TYPES ------------------------------------------------------------*/
extern const uint8_t I2C_INIT_STATE;                    // Can start an new I2C operation from this state only.
extern const uint8_t I2C_7ADDR_W_STATE;                 // Send 7bit IC slave address + write bit
extern const uint8_t I2C_7ADDR_R_STATE;                 // Send 7bit IC slave address + read bit
extern const uint8_t I2C_UPLOAD_STATE;                  // Upload only.        
extern const uint8_t I2C_DOWNLOAD_STATE;                // Download only.
extern const uint8_t I2C_STOP_STATE;                    // sends stop bit to end I2C transmission
/* EXPORTED CONSTANTS --------------------------------------------------------*/
/* EXPORTED MACROS -----------------------------------------------------------*/

/* EXPORTED DEFINES ----------------------------------------------------------*/
#define battmonitor_address     0x55
#define shunt1_address          0x56
#define shunt2_address          0x57
#define shunt3_address          0x58

//battery specs: using model: AE503759P
//3.7VDC 1300mAh, 4.8 Wh
#define batt_designCapacity     1300
#define batt_termVoltage        3.0
#define batt_taperCurrent       15


/* EXPORTED FUNCTIONS --------------------------------------------------------*/
void init_coosi2c(void);
uint8_t init_batmonitor(uint16_t designCapacity_mAh, uint16_t term_Voltage, uint16_t taperCurrent);
void i2c_bat_cmd_write(uint8_t command, uint16_t* data);
void i2c_bat_cmd_read( uint8_t command, uint16_t* data);
void i2c_write(uint8_t Periph_Addr, const uint8_t* pBuffer, uint8_t size);
void i2c_read(uint8_t Periph_Addr, uint8_t* pBuffer, uint8_t size);
/* EXTERNAL VARIABLES --------------------------------------------------------*/

/* --- FOR ISR-USE ONLY --- */



#endif  /*__BASIC_RF_H*/
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/