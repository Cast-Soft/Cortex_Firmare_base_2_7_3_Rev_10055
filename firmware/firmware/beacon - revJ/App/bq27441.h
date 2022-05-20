/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
* File Name          : IMU_defs.h
* Author             : Matthew Mak			
* Version            : V1.0
* Date               : 3/21/2019
* Description        : This header file is used to interface with the INA219
					   exclusively.
*******************************************************************************/

/* DEFINE TO PREVENT RECURSIVE INCLUSION -------------------------------------*/
#ifndef __BQ27441_DEFS_H
#define __BQ27441_DEFS_H

/* INCLUDES ------------------------------------------------------------------*/

/* EXPORTED TYPES ------------------------------------------------------------*/

/* EXPORTED CONSTANTS --------------------------------------------------------*/

/* EXPORTED MACROS -----------------------------------------------------------*/

/* EXPORTED DEFINES ----------------------------------------------------------*/
#define HAL_BQ27441_TIMEOUT                 5
#define BQ27441_DELAY                       1

#define BQ27441_CONTROL_LOW                 0x00
#define BQ27441_CONTROL_HIGH                0x01
#define BQ27441_TEMP_LOW                    0x02
#define BQ27441_TEMP_HIGH                   0x03
#define BQ27441_VOLTAGE_LOW                 0x04
#define BQ27441_VOLTAGE_HIGH                0x05
#define BQ27441_FLAGS_LOW                   0x06
#define BQ27441_FLAGS_HIGH                  0x07
#define BQ27441_NOM_AVAILABLE_CAP_LOW       0x08
#define BQ27441_NOM_AVAILABLE_CAP_HIGH      0x09
#define BQ27441_FULL_AVAILABLE_CAP_LOW      0x0A
#define BQ27441_FULL_AVAILABLE_CAP_HIGH     0x0B
#define BQ27441_REMAINING_CAP_LOW           0x0C
#define BQ27441_REMAINING_CAP_HIGH          0x0D
#define BQ27441_FULL_CHARGE_CAP_LOW         0x0E
#define BQ27441_FULL_CHARGE_CAP_HIGH        0x0F
#define BQ27441_AVG_CURRENT_LOW             0x10
#define BQ27441_AVG_CURRENT_HIGH            0x11
#define BQ27441_STANDBY_CURRENT_LOW         0x12
#define BQ27441_STANDBY_CURRENT_HIGH        0x13
#define BQ27441_MAX_LOAD_CURRENT_LOW        0x14
#define BQ27441_MAX_LOAD_CURRENT_HIGH       0x15
#define BQ27441_AVG_POWER_LOW               0x18
#define BQ27441_AVG_POWER_HIGH              0x19
#define BQ27441_STATE_OF_CHARGE_LOW         0x1C
#define BQ27441_STATE_OF_CHARGE_HIGH        0x1D
#define BQ27441_INT_TEMP_LOW                0x1E
#define BQ27441_INT_TEMP_HIGH               0x1F
#define BQ27441_STATE_OF_HEALTH_LOW         0x20
#define BQ27441_STATE_OF_HEALTH_HIGH        0x21
#define BQ27441_REMAINING_CAP_UNFILT_LOW    0x28
#define BQ27441_REMAINING_CAP_UNFILT_HIGH   0x29
#define BQ27441_REMAINING_CAP_FILT_LOW      0x2A
#define BQ27441_REMAINING_CAP_FILT_HIGH     0x2B
#define BQ27441_FULL_CHARGE_UNFILT_CAP_LOW  0x2C
#define BQ27441_FULL_CHARGE_UNFILT_CAP_HIGH 0x2D
#define BQ27441_FULL_CHARGE_FILT_CAP_LOW    0x2E
#define BQ27441_FULL_CHARGE_FILT_CAP_HIGH   0x2F
#define BQ27441_STATE_OF_CHARGE_UNFILT_LOW  0x30
#define BQ27441_STATE_OF_CHARGE_UNFILT_HIGH 0x31
#define BQ27441_OPCONFIG_LOW                0x3A
#define BQ27441_OPCONFIG_HIGH               0x3B
#define BQ27441_DESIGN_CAP_LOW              0x3C
#define BQ27441_DESIGN_CAP_HIGH             0x3D
#define BQ27441_DATA_CLASS                  0x3E
#define BQ27441_DATA_BLOCK                  0x3F
#define BQ27441_BLOCK_DATA_START            0x40
#define BQ27441_BLOCK_DATA_END              0x5F
#define BQ27441_BLOCK_DATA_CHECKSUM         0x60
#define BQ27441_BLOCK_DATA_CONTROL          0x61
#define BQ27441_CONTROL_STATUS              0x0000
#define BQ27441_CONTROL_DEVICE_TYPE         0x0001
#define BQ27441_CONTROL_FW_VERSION          0x0002
#define BQ27441_CONTROL_DM_CODE             0x0004
#define BQ27441_CONTROL_PREV_MACWRITE       0x0007
#define BQ27441_CONTROL_CHEM_ID             0x0008
#define BQ27441_CONTROL_BAT_INSERT          0x000C
#define BQ27441_CONTROL_BAT_REMOVE          0x000D
#define BQ27441_CONTROL_SET_HIBERNATE       0x0011
#define BQ27441_CONTROL_CLEAR_HIBERNATE     0x0012
#define BQ27441_CONTROL_SET_CFGUPDATE       0x0013
#define BQ27441_CONTROL_SHUTDOWN_ENABLE     0x001B
#define BQ27441_CONTROL_SHUTDOWN            0x001C
#define BQ27441_CONTROL_SEALED              0x0020
#define BQ27441_CONTROL_TOGGLE_GPOUT        0x0023
#define BQ27441_CONTROL_RESET               0x0041
#define BQ27441_CONTROL_SOFT_RESET          0x0042
#define BQ27441_CONTROL_EXIT_CFGUPDATE      0x0043
#define BQ27441_CONTROL_EXIT_RESIM          0x0044
#define BQ27441_CONTROL_UNSEAL              0x8000
/* EXPORTED FUNCTIONS --------------------------------------------------------*/

/* EXTERNAL VARIABLES --------------------------------------------------------*/
#endif /* __BQ27441_DEFS_H */
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
