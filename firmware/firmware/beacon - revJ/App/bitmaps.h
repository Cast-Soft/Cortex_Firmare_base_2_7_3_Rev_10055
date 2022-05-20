/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
* File Name          : eink.h
* Author             : Matthew Mak
* Version            : V1.0
* Date               : 5/17/2019
* Description        : This header file is used to interface with the Plastic Logic 700756
					   display exclusively.
*******************************************************************************/

/* DEFINE TO PREVENT RECURSIVE INCLUSION -------------------------------------*/
#ifndef BITMAPS_H_
#define BITMAPS_H_

/* INCLUDES ------------------------------------------------------------------*/
#include <stdint.h>
/* EXPORTED TYPES ------------------------------------------------------------*/

/* EXPORTED MACROS -----------------------------------------------------------*/

/* EXPORTED DEFINES ----------------------------------------------------------*/
#define logo_width 80
#define logo_height 80
#define stringer_width 30
#define stringer_height 30
#define batterytext_width 51
#define batterytext_height 9
#define button_width 92
#define button_height 30
#define statusscreen_width	146
#define statusscreen_height	240
#define battery_width 20
#define battery_height 9

#define logo_dimensions			80,80
#define stringer_dimensions		30,30
#define battery_text_dimensions	51,9
#define button_dimensions		92,30
#define statusscreen_dimensions	146,240
#define battery_dimensions		20,9

#define UI_LOGO_COORD			33,80
#define UIPOWER_TEXT_COORD		72,218
#define UI_BATTERY_TEXT_COORD	84,10
#define UI_BATTERY_COORD		135,10

#define UIHOME_DURATION_COORD	9,10
#define UIHOME_NAMETOP_COORD	72,69
#define UIHOME_NAMEBOTTOM_COORD	72,90
#define UIHOME_ID_COORD			72,142
#define UIHOME_STRINGER1_COORD	11,199
#define UIHOME_STRINGER2_COORD	58,199
#define UIHOME_STRINGER3_COORD	105,199
#define UIMENU_HOME_COORD		27,60
#define UIMENU_STATUS_COORD		27,98
#define UIMENU_POWER_COORD		27,136
#define UISTATUS_COORD			0,0
/* EXPORTED CONSTANTS --------------------------------------------------------*/

/* EXPORTED FUNCTIONS --------------------------------------------------------*/

/* EXTERNAL VARIABLES --------------------------------------------------------*/
extern const uint8_t logo[];
extern const uint8_t stringer1disable[];
extern const uint8_t stringer1enable[];
extern const uint8_t stringer2disable[];
extern const uint8_t stringer2enable[];
extern const uint8_t stringer3disable[];
extern const uint8_t stringer3enable[];
extern const uint8_t battery100[];
extern const uint8_t battery80[];
extern const uint8_t battery60[];
extern const uint8_t battery40[];
extern const uint8_t battery20[];
extern const uint8_t button_homedisable[];
extern const uint8_t button_homeenable[];
extern const uint8_t button_powerdisable[];
extern const uint8_t button_powerenable[];
extern const uint8_t button_statusdisable[];
extern const uint8_t button_statusenable[];
extern const uint8_t status_radio_static[];
extern const uint8_t status_battery_static[];
#endif /* BITMAPS_H_ */
/************************END OF FILE*******************************************/
