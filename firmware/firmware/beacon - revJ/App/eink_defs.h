/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
* File Name          : eink_defs.h
* Author             : Matthew Mak			
* Version            : V1.0
* Date               : 5/16/2019
* Description        : This header file is used to interface with the Plastic Logic 700756
					   display exclusively.
*******************************************************************************/

/* DEFINE TO PREVENT RECURSIVE INCLUSION -------------------------------------*/
#ifndef __EINK_DEFS_H
#define __EINK_DEFS_H

/* INCLUDES ------------------------------------------------------------------*/

/* EXPORTED TYPES ------------------------------------------------------------*/

/* EXPORTED CONSTANTS --------------------------------------------------------*/

/* EXPORTED MACROS -----------------------------------------------------------*/

/* EXPORTED DEFINES ----------------------------------------------------------*/
#define EINK_REVISION							0x00
#define EINK_PANEL_SETTING						0x01
#define EINK_DRIVER_VOLTAGE_SETTING				0x02
#define EINK_POWER_CONTROL_SETTING				0x03
#define EINK_BOOST_SETTING						0x04
#define EINK_VCOM_AND_DATA_INTERVAL_SETTING		0x05
#define EINK_TCOM_TIMING_SETTING				0x06
#define EINK_TEMPERATURE_SENSOR_CONFIGURATION	0x07
#define EINK_TEMPERATURE_VALUE_REGISTER			0x08
#define EINK_GPIO_CONFIGURATION_REGISTER		0x09
#define EINK_GPIO_INTERRUPT_REGISTER			0x0A
#define EINK_GPIO_PORT_TYPE_REGISTER			0x0B
#define EINK_PANEL_RESOLUTION_SETTING			0x0C
#define EINK_WRITE_PIXEL_RECTANGULAR_SETTING	0x0D
#define EINK_PIXEL_ACCESS_POSITION_SETTING		0x0E
#define EINK_DATA_ENTRY_MODE_SETTING			0x0F
#define EINK_WRITE_RAM							0x10
#define EINK_READ_RAM							0x11
#define EINK_BYPASS_UPDATE_SETTING				0x12
#define EINK_INITIAL_UPDATE_SETTING				0x13
#define EINK_DISPLAY_ENGINE_CONTROL_REGISTER	0x14
#define EINK_STATUS_REGISTER					0x15
#define EINK_INTERRUPT_ENABLE_REGISTER			0x16
#define EINK_INTERRUPT_STATUS_REGISTER			0x17
#define EINK_VCOM_CONFIGURATION_REGISTER		0x18
#define EINK_AUTO_MEASURE_VCOM					0x19
#define EINK_VCOM_MEASURE_VALUE					0x1A
#define EINK_VCOM_DC_SETTING					0x1B
#define EINK_WAVEFORM_LUT_SETTING				0x1C
#define EINK_VBORDER_SETTING					0x1D
#define EINK_POWER_SEQUENCE_SETTING				0x1F
#define EINK_SOFTWARE_RESET						0x20
#define EINK_SLEEP_MODE							0x21
#define EINK_PROGRAM_WS_MTP						0x40
#define EINK_MTP_ADDRESS_SETTING				0x41
#define EINK_MTP_ONE_BYTE_PROGRAM				0x42
#define EINK_MTP_READ							0x43
#define EINK_UNDOCUMENTED_REG  					0x44	//found in displayswithultrachip pdf file
/* EXPORTED FUNCTIONS --------------------------------------------------------*/

/* EXTERNAL VARIABLES --------------------------------------------------------*/


#endif /*__EINK_DEFS_H*/
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/