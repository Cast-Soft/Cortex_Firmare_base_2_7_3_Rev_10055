/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : usb_endp.c
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Endpoint routines
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/

#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_hw_config.h"
#include "usb_istr.h"
#include "usb_pwr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Interval between sending IN packets in frame number (1 frame = 1ms) */
#define VCOMPORT_IN_FRAME_INTERVAL             5

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static uint8_t USB_Rx_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback(void) {
    uint16_t USB_Tx_ptr;
    uint16_t USB_Tx_length;

    if (USB_Tx_State == 1) {
        if (USART_Rx_length == 0) {
            USB_Tx_State = 0;
        } else {
            if (USART_Rx_length > VIRTUAL_COM_PORT_DATA_SIZE) {
                USB_Tx_ptr = USART_Rx_ptr_out;
                USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;

                USART_Rx_ptr_out += VIRTUAL_COM_PORT_DATA_SIZE;
                USART_Rx_length -= VIRTUAL_COM_PORT_DATA_SIZE;
            } else {
                USB_Tx_ptr = USART_Rx_ptr_out;
                USB_Tx_length = USART_Rx_length;

                USART_Rx_ptr_out += USART_Rx_length;
                USART_Rx_length = 0;
            }

            USB_SIL_Write(EP1_IN, &USART_Rx_Buffer[USB_Tx_ptr], USB_Tx_length);
        }
    }
}

/*******************************************************************************
* Function Name  : EP3_OUT_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP3_OUT_Callback(void) {
    uint16_t USB_Rx_Cnt;

    /* Get the received data buffer and update the counter */
    USB_Rx_Cnt = USB_SIL_Read(EP3_OUT, USB_Rx_Buffer);

    /* USB data will be immediately processed, this allow next USB traffic being
    NAKed till the end of the USART Xfer */

    USB_To_USART_Send_Data(USB_Rx_Buffer, USB_Rx_Cnt);
}

/*******************************************************************************
* Function Name  : EP5_OUT_Callback
* Description    : Data from host to control Beacon
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP5_OUT_Callback(void) {
    uint16_t USB_Rx_Cnt;

    /* Get the received data buffer and update the counter */
   USB_Rx_Cnt = USB_SIL_Read(EP5_OUT, USB_Rx_Buffer);


    
    /* USB data will be immediately processed, this allow next USB traffic being
    NAKed till the end of the USART Xfer */

    USB_To_USART_Send_Data(USB_Rx_Buffer, USB_Rx_Cnt);
}


/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP7_IN_Callback(void) {
    uint16_t USB_Tx_ptr;
    uint16_t USB_Tx_length;

    if (USB_Tx_State == 1) {
        if (USART_Rx_length == 0) {
            USB_Tx_State = 0;
        } else {
            if (USART_Rx_length > VIRTUAL_COM_PORT_DATA_SIZE) {
                USB_Tx_ptr = USART_Rx_ptr_out;
                USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;

                USART_Rx_ptr_out += VIRTUAL_COM_PORT_DATA_SIZE;
                USART_Rx_length -= VIRTUAL_COM_PORT_DATA_SIZE;
            } else {
                USB_Tx_ptr = USART_Rx_ptr_out;
                USB_Tx_length = USART_Rx_length;

                USART_Rx_ptr_out += USART_Rx_length;
                USART_Rx_length = 0;
            }

            USB_SIL_Write(EP7_IN, &USART_Rx_Buffer[USB_Tx_ptr], USB_Tx_length);
        }
    }
}
/*******************************************************************************
* Function Name  : SOF_Callback / INTR_SOFINTR_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void INTR_SOFINTR_Callback(void) {
    static uint32_t FrameCount = 0;

    if (bDeviceState == CONFIGURED) {
        if (FrameCount++ == VCOMPORT_IN_FRAME_INTERVAL) {
            /* Reset the frame counter */
            FrameCount = 0;

            /* Check the data to be sent through IN pipe */
            Handle_USBAsynchXfer();
        }
    }
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/