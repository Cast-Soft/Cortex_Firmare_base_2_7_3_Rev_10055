/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
* File Name          : IMU_defs.h
* Author             : Matthew Mak			
* Version            : V1.0
* Date               : 3/1/2019
* Description        : This header file is used to interface with the ICM-20948
					   exclusively.
*******************************************************************************/

/* DEFINE TO PREVENT RECURSIVE INCLUSION -------------------------------------*/
#ifndef __IMU_DEFS_H
#define __IMU_DEFS_H

/* INCLUDES ------------------------------------------------------------------*/

/* EXPORTED TYPES ------------------------------------------------------------*/

/* EXPORTED CONSTANTS --------------------------------------------------------*/

/* EXPORTED MACROS -----------------------------------------------------------*/

/* EXPORTED DEFINES ----------------------------------------------------------*/
//note the method of interfacing with the ICM-20948 with SPI is on pg31 on the datasheet
#define BANK0						0x00
#define BANK1						0x10
#define BANK2						0x20
#define BANK3 						0x30
#define IMU_REG_BANK_SEL			0x7F
#define IMU_READBIT					0x80
#define IMU_WRITEBIT				0x00

/*Following are for the User Bank 0 Register Map within the ICM-20948*/
#define IMU_WHO_AM_I				0x00
#define IMU_USER_CTRL				0x03
#define IMU_LP_CONFIG				0x05
#define IMU_PWR_MGMT_1				0x06
#define IMU_PWR_MGMT_2				0x07
#define IMU_INT_PIN_CFG				0x0F
#define IMU_ENABLE					0x10
#define IMU_ENABLE_1				0x11
#define IMU_ENABLE_2				0x12
#define IMU_ENABLE_3				0x13
#define IMU_I2C_MST_STATUS			0x17
#define IMU_INT_STATUS				0x19
#define IMU_INT_STATUS_1			0x1A
#define IMU_INT_STATUS_2			0x1B
#define IMU_INT_STATUS_3			0x1C
#define IMU_DELAY_TIMEH				0x28
#define IMU_DELAY_TIMEL				0x29
#define IMU_ACCEL_XOUT_H			0x2D
#define IMU_ACCEL_XOUT_L			0x2E
#define IMU_ACCEL_YOUT_H			0x2F
#define IMU_ACCEL_YOUT_L			0x30
#define IMU_ACCEL_ZOUT_H			0x31
#define IMU_ACCEL_ZOUT_L			0x32
#define IMU_GYRO_XOUT_H				0x33
#define IMU_GYRO_XOUT_L				0x34
#define IMU_GYRO_YOUT_H				0x35
#define IMU_GYRO_YOUT_L				0x36
#define IMU_GYRO_ZOUT_H				0x37
#define IMU_GYRO_ZOUT_L				0x38
#define IMU_TEMP_OUT_H				0x39
#define IMU_TEMP_OUT_L				0x3A
#define IMU_EXT_SLV_SENS_DATA_00	0x3B
#define IMU_EXT_SLV_SENS_DATA_01	0x3C
#define IMU_EXT_SLV_SENS_DATA_02	0x3D
#define IMU_EXT_SLV_SENS_DATA_03	0x3E
#define IMU_EXT_SLV_SENS_DATA_04	0x3F
#define IMU_EXT_SLV_SENS_DATA_05	0x40
#define IMU_EXT_SLV_SENS_DATA_06	0x41
#define IMU_EXT_SLV_SENS_DATA_07	0x42
#define IMU_EXT_SLV_SENS_DATA_08	0x43
#define IMU_EXT_SLV_SENS_DATA_09	0x44
#define IMU_EXT_SLV_SENS_DATA_10	0x45
#define IMU_EXT_SLV_SENS_DATA_11	0x46
#define IMU_EXT_SLV_SENS_DATA_12	0x47
#define IMU_EXT_SLV_SENS_DATA_13	0x48
#define IMU_EXT_SLV_SENS_DATA_14	0x49
#define IMU_EXT_SLV_SENS_DATA_15	0x4A
#define IMU_EXT_SLV_SENS_DATA_16	0x4B
#define IMU_EXT_SLV_SENS_DATA_17	0x4C
#define IMU_EXT_SLV_SENS_DATA_18	0x4D
#define IMU_EXT_SLV_SENS_DATA_19	0x4E
#define IMU_EXT_SLV_SENS_DATA_20	0x4F
#define IMU_EXT_SLV_SENS_DATA_21	0x50
#define IMU_EXT_SLV_SENS_DATA_22	0x51
#define IMU_EXT_SLV_SENS_DATA_23	0x52
#define IMU_FIFO_EN_1				0x66
#define IMU_FIFO_EN_2				0x67
#define IMU_FIFO_RST				0x68
#define IMU_FIFO_MODE				0x69
#define IMU_FIFO_COUNTH				0x70
#define IMU_FIFO_COUNTL				0x71
#define IMU_FIFO_R_W				0x72
#define IMU_DATA_RDY_STATUS			0x74
#define IMU_FIFO_CFG				0x76

/*Following are for the User Bank 1 Register Map within the ICM-20948*/
#define IMU_SELF_TEST_X_GYRO		0x02
#define IMU_SELF_TEST_Y_GYRO		0x03
#define IMU_SELF_TEST_Z_GYRO		0x04
#define IMU_SELF_TEST_X_ACCEL		0x0E
#define IMU_SELF_TEST_Y_ACCEL		0x0F
#define IMU_SELF_TEST_Z_ACCEL		0x10
#define IMU_XA_OFFS_H				0x14
#define IMU_XA_OFFS_L				0x15
#define IMU_YA_OFFS_H				0x17
#define IMU_YA_OFFS_L				0x18
#define IMU_ZA_OFFS_H				0x1A
#define IMU_ZA_OFFS_L				0x1B
#define IMU_TIMEBASE_CORRECTION_PLL	0x28

/*Following are for the User Bank 2 Register Map within the ICM_20948*/
#define IMU_GYRO_SMPLRT_DIV			0x00
#define IMU_GYRO_CONFIG_1			0x01
#define IMU_GYRO_CONFIG_2			0x02
#define IMU_XG_OFFS_USRH			0x03
#define IMU_XG_OFFS_USRL			0x04
#define IMU_YG_OFFS_USRH			0x05
#define IMU_YG_OFFS_USRL			0x06
#define IMU_ZG_OFFS_USRH			0x07
#define IMU_ZG_OFFS_USRL			0x08
#define IMU_ODR_ALIGN_EN			0x09
#define IMU_ACCEL_SMPLRT_DIV_1		0x10
#define IMU_ACCEL_SMPLRT_DIV_2		0x11
#define IMU_ACCEL_INTEL_CTRL		0x12
#define IMU_ACCEL_WOM_THR			0x13
#define IMU_ACCEL_CONFIG			0x14
#define IMU_ACCEL_CONGIG_2			0x15
#define IMU_FSYNC_CONFIG			0x52
#define IMU_TEMP_CONFIG				0x53
#define IMU_MOD_CTRL_USR			0x54

/*Following are for the User Bank 3 register Map within the ICM-20948*/
#define IMU_I2C_MST_ODR_CONFIG		0x00
#define IMU_I2C_MST_CTRL			0x01
#define IMU_I2C_MST_DELAY_CTRL		0x02
#define IMU_I2C_SLV0_ADDR			0x03
#define IMU_I2C_SLV0_REG			0x04
#define IMU_I2C_SLV0_CTRL			0x05
#define IMU_I2C_SLV0_DO				0x06
#define IMU_I2C_SLV1_ADDR			0x07
#define IMU_I2C_SLV1_REG			0x08
#define IMU_I2C_SLV1_CTRL			0x09
#define IMU_I2C_SLV1_DO				0x0A
#define IMU_I2C_SLV2_ADDR			0x0B
#define IMU_I2C_SLV2_REG			0x0C
#define IMU_I2C_SLV2_CTRL			0x0D
#define IMU_I2C_SLV2_DO				0x0E
#define IMU_I2C_SLV3_ADDR			0x0F
#define IMU_I2C_SLV3_REG			0x10
#define IMU_I2C_SLV3_CTRL			0x11
#define IMU_I2C_SLV3_DO				0x12
#define IMU_I2C_SLV4_ADDR			0x13
#define IMU_I2C_SLV4_REG			0x14
#define IMU_I2C_SLV4_CTRL			0x15
#define IMU_I2C_SLV4_DO				0x16
#define IMU_I2C_SLV4_DI				0x17

/* EXPORTED FUNCTIONS --------------------------------------------------------*/

/* EXTERNAL VARIABLES --------------------------------------------------------*/


#endif /*__IMU_DEFS_H*/
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/