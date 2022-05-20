/******************** (C) COPYRIGHT 2011 NaturalPoint, Inc. ********************
* File Name          : stm32f10x_it.c
* Author             : ???
* Version            : V1.0
* Date               : 6/2/2011
* Description        : ???
*******************************************************************************/
/*
    NVIC Priority (Highest first):
        - OTG_FS_IRQ
        - TIM3_IRQn
        - EXTI2_IRQn (GPI_RADIO_GPIO0_IRQn)
        - SPI3_IRQn (SPI_RADIO_IRQn)
        - EXTI1_IRQn (GPI_RADIO_GPIO1_IRQn)
        - DMA1_Channel4_IRQn (COM2_DMA_IRQn)
*/

/* INCLUDES ------------------------------------------------------------------*/
#include "stm32f10x_conf.h"
#include "hardware.h"
#include "stm32f10x_it.h"
//#include "stm32f10x_exti.h"
//#include "stm32f10x_dma.h"

#include "usb_istr.h"

//#include "basic_rf.h"
//#include "radio_defs.h"
#ifdef COOS
#include "CoOS.h"
#endif
#include <stdio.h>

/* PRIVATE TYPEDEF -----------------------------------------------------------*/

/* PRIVATE DEFINES -----------------------------------------------------------*/

/* PRIVATE MACROS ------------------------------------------------------------*/

#ifdef COOS
#define ENTER_ISR() CoEnterISR()
#define EXIT_ISR() CoExitISR()
#define ISR_SETFLAG(x) isr_SetFlag(x);
#else
#define ENTER_ISR()
#define EXIT_ISR()
#define ISR_SETFLAG(x) ( x = 1; )
#endif

/* EXTERN VARIABLES ----------------------------------------------------------*/


/* PRIVATE VARIABLES ---------------------------------------------------------*/

/* PUBLIC VARIABLES ----------------------------------------------------------*/

#ifndef COOS
volatile unsigned int SysTickCounter = 0;
#endif

volatile uint32_t rfBeaconFrameId = 0;
volatile uint16_t rfBeaconLoaded;

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/

/* PRIVATE FUNCTIONS ---------------------------------------------------------*/

/* PUBLIC FUNCTIONS ----------------------------------------------------------*/

/*******************************************************************************
* Description : This function handles NMI exception.
* Input       : -
* Return      : -
*******************************************************************************/
void NMI_Handler(void)
{
}

/*******************************************************************************
* Description : This function handles Hard Fault exception.
* Input       : -
* Return      : -
*******************************************************************************/
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1);
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
/*******************************************************************************
* Description : This function handles NMI exception.
* Input       : -
* Return      : -
*******************************************************************************/
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1);
}

/*******************************************************************************
* Description : This function handles Bus Fault exception.
* Input       : -
* Return      : -
*******************************************************************************/
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1);
}

/*******************************************************************************
* Description : This function handles Usage Fault exception.
* Input       : -
* Return      : -
*******************************************************************************/
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1);
}

/*******************************************************************************
* Description : This function handles SVCall exception.
* Input       : -
* Return      : -
*******************************************************************************/
void SVC_Handler(void)
{
}

/*******************************************************************************
* Description : This function handles Debug Monitor exception.
* Input       : -
* Return      : -
*******************************************************************************/
void DebugMon_Handler(void)
{
}

#ifndef COOS
/*******************************************************************************
* Description : This function handles PendSVC exception.
* Input       : -
* Return      : -
*******************************************************************************/
void PendSV_Handler(void)
{
}

/*******************************************************************************
* Description : This function handles SysTick Handler.
* Input       : -
* Return      : -
*******************************************************************************/
void SysTick_Handler(void)
{
    SysTickCounter++;
}
#endif

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/*******************************************************************************
* Description : This function handles USB-On-The-Go FS global interrupt request.
* Input       : -
* Return      : -
*******************************************************************************/
#ifndef STDIO_TO_USART
void OTG_FS_IRQHandler(void) {
    STM32_PCD_OTG_ISR_Handler();
}
#endif

/*******************************************************************************
* Description : This function handles TIM3 global interrupt request.
*               TIM3: 100Hz OH Sync-Out, 10Hz RF-Sync Pkt
* Input       : -
* Return      : -
*******************************************************************************/
void TIM3_IRQHandler(void) {

}

/*******************************************************************************
* Description : This function handles External line 2 interrupt request.
*               GPI_RADIO_GPIO0 (RX_FRM_DONE) = PD.2
* Input       : -
* Return      : -
*******************************************************************************/
void EXTI2_IRQHandler(void) {

}

/*******************************************************************************
* Description : This function handles SPI3 (SPI_RADIO_IRQn) global interrupt request.
* Input       : -
* Return      : -
*******************************************************************************/
void SPI3_IRQHandler(void) {

}

/*******************************************************************************
* Description : This function handles External line 2 interrupt request.
*               GPI_RADIO_GPIO1 (TX_FRM_DONE) = PD.1
* Input       : -
* Return      : -
*******************************************************************************/
void EXTI1_IRQHandler(void) {

}

/*******************************************************************************
* Description : This function handles DMA1 Channel 4 interrupt request.
*               DMA4_Ch4: USART1-TX
* Input       : -
* Return      : -
*******************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{
 //   if(DMA_GetITStatus(DMA1_IT_TC4)) {
 ///       /* Clear DMA1 Channel4 Transfer Complete interrupt pending bit */
 //       DMA_ClearITPendingBit(DMA1_IT_TC4);

//        DMA_Cmd(COM2_DMA_CHAN_TX, DISABLE);
//    } else assert(0);
//    }     
}

/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
