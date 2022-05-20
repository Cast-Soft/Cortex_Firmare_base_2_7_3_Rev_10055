/******************** (C) COPYRIGHT 2011 NaturalPoint, Inc. ********************
* File Name          : usb_hw_config.c
* Author             : ?
* Version            : V1.0
* Date               : 6/2/2011
* Description        : USB CDC Virtual Com Port Hardware Setup
*******************************************************************************/

/* INCLUDES ------------------------------------------------------------------*/

#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_hw_config.h"
#include "usb_pwr.h"
#include "stm32f10x_tim.h"
#include <yfuns.h>

/* PRIVATE TYPEDEF -----------------------------------------------------------*/

/* PRIVATE DEFINES -----------------------------------------------------------*/

/* PRIVATE MACROS ------------------------------------------------------------*/

/* EXTERN VARIABLES ----------------------------------------------------------*/

/* PRIVATE VARIABLES ---------------------------------------------------------*/

static uint32_t USART_Rx_ptr_in = 0;

static volatile uint8_t USB_Rx_Buffer[USB_RX_BUF_SIZE];
static volatile uint32_t USB_Rx_Buffer_Head = 0;
static volatile uint32_t USB_Rx_Buffer_Tail = 0;

/* PUBLIC VARIABLES ----------------------------------------------------------*/

uint8_t USART_Rx_Buffer[USART_RX_DATA_SIZE];
uint32_t USART_Rx_ptr_out = 0;
uint32_t USART_Rx_length = 0;
uint8_t USB_Tx_State = 0;

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/

static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len);

/* PRIVATE FUNCTIONS ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len) {
    uint8_t idx = 0;

    for (idx = 0; idx < len; idx++) {
        if (((value >> 28)) < 0xA) {
            pbuf[2 * idx] = (value >> 28) + '0';
        } else {
            pbuf[2 * idx] = (value >> 28) + 'A' - 10;
        }

        value = value << 4;

        pbuf[2 * idx + 1] = 0;
    }
}

/* PUBLIC FUNCTIONS ----------------------------------------------------------*/

/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void) {
    /* Select USBCLK source */
    RCC_OTGFSCLKConfig(RCC_OTGFSCLKSource_PLLVCO_Div3);

    /* Enable the USB clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_OTG_FS, ENABLE);
}

/*******************************************************************************
* Function Name  : Enter_LowPowerMode
* Description    : Power-off system clocks and power while entering suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void) {
    /* Set the device state to suspend */
    bDeviceState = SUSPENDED;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void) {
    DEVICE_INFO *pInfo = &Device_Info;

    /* Set the device state to the correct state */
    if (pInfo->Current_Configuration != 0) {
        /* Device configured */
        bDeviceState = CONFIGURED;
    } else {
        bDeviceState = ATTACHED;
    }
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config
* Description    : Configures the USB interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* Enable the USB Interrupts */
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : USB_Timer_Config
* Description    : Configures TIM7 for use by USB_OTG_BSP_uDelay()
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_Timer_Config(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    /* TIM7 configuration */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    TIM_TimeBaseStructure.TIM_Prescaler = 71;       // 72MHz/72 = 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

    TIM_Cmd(TIM7, ENABLE);
    TIM7->CR1 |= TIM_CR1_URS;   // Only Counter OF/UF generates UEV
    TIM7->CR1 &= ~TIM_CR1_UDIS; // UEV Enabled

    /* For Calibration (PD.4) */
	/*RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
    while (1) {
        GPIOD->ODR ^= GPIO_Pin_4;
        USB_OTG_BSP_uDelay(5);
    }*/
}

/*******************************************************************************
* Function Name  : USB_OTG_BSP_uDelay.
* Description    : provide delay (usec).
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_OTG_BSP_uDelay(const uint32_t usec) {
    //assert_param(usec < 0xFFFF); // [[DEBUG]]
    TIM7->ARR = (uint16_t) (usec - 2);
    TIM7->SR &= ~TIM_SR_UIF;            // Clear UIF
    TIM7->EGR = TIM_PSCReloadMode_Immediate;
    while (!(TIM7->SR & TIM_SR_UIF));   // Wait for UIF
}

/*******************************************************************************
* Function Name  : USB_To_USART_Send_Data.
* Description    : send the received data from USB to the UART 0.
* Input          : data_buffer: data address.
*                  Nb_bytes: number of bytes to send.
* Return         : none.
*******************************************************************************/
void USB_To_USART_Send_Data(uint8_t *data_buffer, uint8_t Nb_bytes) {
    while (Nb_bytes > 0) {
        uint32_t head = USB_Rx_Buffer_Head + 1;

        if ((head % USB_RX_BUF_SIZE) == USB_Rx_Buffer_Tail)
            return; // incoming buffer overflow

        USB_Rx_Buffer[USB_Rx_Buffer_Head++] = *data_buffer++;

        if (USB_Rx_Buffer_Head == USB_RX_BUF_SIZE)
            USB_Rx_Buffer_Head = 0; // wrap-around
        Nb_bytes--;
    }
}

/*******************************************************************************
* Function Name  : Handle_USBAsynchXfer.
* Description    : send data to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void Handle_USBAsynchXfer(void) {
    uint16_t USB_Tx_ptr;
    uint16_t USB_Tx_length;

    if (USB_Tx_State != 1) {
        if (USART_Rx_ptr_out == USART_RX_DATA_SIZE) {
            USART_Rx_ptr_out = 0;
        }

        if (USART_Rx_ptr_out == USART_Rx_ptr_in) {
            USB_Tx_State = 0;
            return;
        }

        if (USART_Rx_ptr_out > USART_Rx_ptr_in) { /* rollback */
            USART_Rx_length = USART_RX_DATA_SIZE - USART_Rx_ptr_out;
        } else {
            USART_Rx_length = USART_Rx_ptr_in - USART_Rx_ptr_out;
        }

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
        USB_Tx_State = 1;

        USB_SIL_Write(EP1_IN, &USART_Rx_Buffer[USB_Tx_ptr], USB_Tx_length);
    }
}

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void) {
    uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

    Device_Serial0 = *(__IO uint32_t *)(0x1FFFF7E8);
    Device_Serial1 = *(__IO uint32_t *)(0x1FFFF7EC);
    Device_Serial2 = *(__IO uint32_t *)(0x1FFFF7F0);

    Device_Serial0 += Device_Serial2;

    if (Device_Serial0 != 0) {
        IntToUnicode(Device_Serial0, &Virtual_Com_Port_StringSerial[2], 8);
        IntToUnicode(Device_Serial1, &Virtual_Com_Port_StringSerial[18], 4);
    }
}

#ifndef STDIO_TO_USART
/*******************************************************************************
* Description : STDOUT Low-Level write
* Input       :
* Return      :
*******************************************************************************/
size_t __write(int handle, const unsigned char *buffer, size_t size) {
    size_t nChars = 0;
    uint32_t USART_Rx_ptr_in_next;

    if ((buffer == 0) || (handle == -1)) {
        /*
         * This means that we should flush internal buffers.  Since we
         * don't we just return.  (Remember, "handle" == -1 means that all
         * handles should be flushed.)
         */
        return 0;
    }

    /* This template only writes to "standard out" and "standard err",
     * for all other file handles it returns failure. */
    if (handle != _LLIO_STDOUT && handle != _LLIO_STDERR) {
        return _LLIO_ERROR;
    }

    while (size) {
        USART_Rx_ptr_in_next = (USART_Rx_ptr_in + 1) % USART_RX_DATA_SIZE;

        if (USART_Rx_ptr_in_next == USART_Rx_ptr_out) break; // outgoing buffer full
        USART_Rx_Buffer[USART_Rx_ptr_in] = *buffer++;
        USART_Rx_ptr_in = USART_Rx_ptr_in_next;

        nChars++;
        size--;
    }

    return nChars;
}

/*******************************************************************************
* Description : STDIN Low-Level read
* Input       :
* Return      :
*******************************************************************************/
size_t __read(int handle, unsigned char *buffer, size_t size) {
    size_t nChars = 0;

    /* This template only reads from "standard in", for all other file
    * handles it returns failure. */
    if (handle != _LLIO_STDIN) {
        return _LLIO_ERROR;
    }

    while (size) {
        uint32_t tail = USB_Rx_Buffer_Tail;
        // will NOT wait for char
        if (USB_Rx_Buffer_Head == tail) break;

        *buffer++ = USB_Rx_Buffer[USB_Rx_Buffer_Tail];
        USB_Rx_Buffer_Tail++;
        if (USB_Rx_Buffer_Tail >= USB_RX_BUF_SIZE)
            USB_Rx_Buffer_Tail = 0; // wrap-around

        nChars++;
        size--;
    }

    return nChars;
}
#endif /* STDIO_TO_USART */

/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/