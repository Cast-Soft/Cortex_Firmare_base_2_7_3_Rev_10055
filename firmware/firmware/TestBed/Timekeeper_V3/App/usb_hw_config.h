/******************** (C) COPYRIGHT 2011 NaturalPoint, Inc. ********************
* File Name          : usb_hw_config.h
* Author             : ?
* Version            : V1.0
* Date               : 6/2/2011
* Description        : USB CDC Virtual Com Port Hardware Setup
*******************************************************************************/

/* DEFINE TO PREVENT RECURSIVE INCLUSION -------------------------------------*/
#ifndef __USB_HW_CONFIG_H
#define __USB_HW_CONFIG_H

/* INCLUDES ------------------------------------------------------------------*/

#include "usb_type.h"

/* EXPORTED TYPES ------------------------------------------------------------*/

/* EXPORTED CONSTANTS --------------------------------------------------------*/

/* EXPORTED MACROS -----------------------------------------------------------*/

/* EXPORTED DEFINES ----------------------------------------------------------*/

#define USART_RX_DATA_SIZE 1048
#define USB_RX_BUF_SIZE 1024

/* EXPORTED FUNCTIONS --------------------------------------------------------*/

void Set_USBClock(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Interrupts_Config(void);
void USB_Timer_Config(void);
void USB_OTG_BSP_uDelay(const uint32_t usec);
void USB_Cable_Config(FunctionalState NewState);
void USB_To_USART_Send_Data(uint8_t *data_buffer, uint8_t Nb_bytes);
void Handle_USBAsynchXfer(void);
void Get_SerialNum(void);
//size_t __write(int handle, const unsigned char *buffer, size_t size);
//size_t __read(int handle, unsigned char *buffer, size_t size);

/* EXTERNAL VARIABLES --------------------------------------------------------*/

extern uint8_t USART_Rx_Buffer [];
extern uint32_t USART_Rx_ptr_out;
extern uint32_t USART_Rx_length;
extern uint8_t USB_Tx_State;

#endif  /*__USB_HW_CONFIG_H*/
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/