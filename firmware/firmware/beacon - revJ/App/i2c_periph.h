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
#include "CoOS.h"
/* EXPORTED TYPES ------------------------------------------------------------*/
extern const uint8_t shunt1_address;
extern const uint8_t shunt2_address;
extern const uint8_t shunt3_address;
extern const uint8_t battmonitor_address;
extern const uint8_t pressure_address;
/* EXPORTED CONSTANTS --------------------------------------------------------*/
/* EXPORTED MACROS -----------------------------------------------------------*/
typedef enum{
    I2C_INIT_MODE  =0,
    I2C_WRITE_MODE =1,
    I2C_READ_MODE  =2,
    I2C_BURSTREAD_MODE =3
}I2C_State;

/* EXPORTED DEFINES ----------------------------------------------------------*/
//the following values were calculated to fit the BLKT revJ specs.
#define ina219_calibration_value			0x1062	//calculated value.
#define ina219_currentDivider_mA			5		//calculated value. 

//battery specs: 
#define batt_designCapacity     1300
#define batt_termVoltage        3000
#define batt_taperCurrent       15

#define BATT_TERM_PERCENT       10
/* EXPORTED FUNCTIONS --------------------------------------------------------*/
void init_coosi2c(void);
void WaitGrabI2C(void);
void ReleaseI2C(void);

void init_ledshunt(void);
int16_t ledbuffer_set(uint8_t addr, uint8_t * errorcheck);
void i2c_write(uint8_t Periph_Addr, const uint8_t* pBuffer, uint8_t size);
void i2c_read(uint8_t Periph_Addr, uint8_t* pBuffer, uint8_t size);
void i2c_burstread(uint8_t Periph_Addr, uint8_t subaddr, uint8_t* pBuffer, uint8_t size);
uint8_t I2C_pressureID();

#ifdef BQ27441_BAT_MONITOR
uint16_t init_batmonitor(uint16_t designCapacity_mAh, uint16_t term_Voltage, uint16_t taperCurrent);
void i2c_bat_cmd_write(uint8_t command, uint16_t* data);
void i2c_bat_cmd_read( uint8_t command, uint16_t* data);
#endif

/* EXTERNAL VARIABLES --------------------------------------------------------*/
extern uint8_t*            pI2CTxBuf;
extern uint8_t*            pI2CRxBuf;
/* --- FOR ISR-USE ONLY --- */
extern volatile uint8_t I2CTxRxByteCount;
extern volatile I2C_State I2CTxRxByteState;   
extern volatile uint8_t I2C_ERR;
extern OS_FlagID   flagI2CMachineDone;

#endif  /*__BASIC_RF_H*/
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
