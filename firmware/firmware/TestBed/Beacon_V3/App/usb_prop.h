/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : usb_prop.h
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : All processing related to Virtual COM Port Demo (Endpoint 0)
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* DEFINE TO PREVENT RECURSIVE INCLUSION -------------------------------------*/
#ifndef __USB_PROP_H
#define __USB_PROP_H

/* INCLUDES ------------------------------------------------------------------*/

/* EXPORTED TYPES ------------------------------------------------------------*/

/* EXPORTED CONSTANTS --------------------------------------------------------*/

/* EXPORTED MACROS -----------------------------------------------------------*/

/* EXPORTED DEFINES ----------------------------------------------------------*/

#define Virtual_Com_Port_GetConfiguration          NOP_Process
//#define Virtual_Com_Port_SetConfiguration          NOP_Process
#define Virtual_Com_Port_GetInterface              NOP_Process
#define Virtual_Com_Port_SetInterface              NOP_Process
#define Virtual_Com_Port_GetStatus                 NOP_Process
#define Virtual_Com_Port_ClearFeature              NOP_Process
#define Virtual_Com_Port_SetEndPointFeature        NOP_Process
#define Virtual_Com_Port_SetDeviceFeature          NOP_Process
//#define Virtual_Com_Port_SetDeviceAddress          NOP_Process

#define SEND_ENCAPSULATED_COMMAND   0x00
#define GET_ENCAPSULATED_RESPONSE   0x01
#define SET_COMM_FEATURE            0x02
#define GET_COMM_FEATURE            0x03
#define CLEAR_COMM_FEATURE          0x04
#define SET_LINE_CODING             0x20
#define GET_LINE_CODING             0x21
#define SET_CONTROL_LINE_STATE      0x22
#define SEND_BREAK                  0x23

/* EXPORTED FUNCTIONS --------------------------------------------------------*/

void Virtual_Com_Port_init(void);
void Virtual_Com_Port_Reset(void);
void Virtual_Com_Port_SetConfiguration(void);
void Virtual_Com_Port_SetDeviceAddress(void);
void Virtual_Com_Port_Status_In(void);
void Virtual_Com_Port_Status_Out(void);
RESULT Virtual_Com_Port_Data_Setup(uint8_t);
RESULT Virtual_Com_Port_NoData_Setup(uint8_t);
RESULT Virtual_Com_Port_Get_Interface_Setting(uint8_t Interface, uint8_t AlternateSetting);
uint8_t *Virtual_Com_Port_GetDeviceDescriptor(uint16_t);
uint8_t *Virtual_Com_Port_GetConfigDescriptor(uint16_t);
uint8_t *Virtual_Com_Port_GetStringDescriptor(uint16_t);

uint8_t *Virtual_Com_Port_GetLineCoding(uint16_t Length);
uint8_t *Virtual_Com_Port_SetLineCoding(uint16_t Length);

/* EXTERNAL VARIABLES --------------------------------------------------------*/

extern DEVICE_PROP Device_Property;
extern DEVICE Device_Table;
extern USER_STANDARD_REQUESTS User_Standard_Requests;

#endif  /* __USB_PROP_H */
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/