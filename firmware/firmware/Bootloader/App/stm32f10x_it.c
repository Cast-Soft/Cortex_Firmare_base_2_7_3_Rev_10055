/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
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
#include "stm32f10x_it.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_dma.h"
#include "stm32_eth.h"
#include "main.h"

//#include "usb_istr.h"
#include "hardware.h"
#include "basic_rf.h"
#include "radio_defs.h"
#include "usb_core.h"
#include "usb_dcd_int.h"

#include <stdio.h>

/* PRIVATE TYPEDEF -----------------------------------------------------------*/

/* PRIVATE DEFINES -----------------------------------------------------------*/

/* PRIVATE MACROS ------------------------------------------------------------*/

/* PRIVATE VARIABLES ---------------------------------------------------------*/

/* PUBLIC VARIABLES ----------------------------------------------------------*/

volatile uint32_t SysTickCounter = 0;

uint32_t TxSendError =0;

extern USB_OTG_CORE_HANDLE           USB_OTG_dev;


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

void hard_fault_handler_c(unsigned int * hardfault_args)
{
	unsigned int stacked_r0;
	unsigned int stacked_r1;
	unsigned int stacked_r2;
	unsigned int stacked_r3;
	unsigned int stacked_r12;
	unsigned int stacked_lr;
	unsigned int stacked_pc;
	unsigned int stacked_psr;

	stacked_r0 = ((unsigned long) hardfault_args[0]);
	stacked_r1 = ((unsigned long) hardfault_args[1]);
	stacked_r2 = ((unsigned long) hardfault_args[2]);
	stacked_r3 = ((unsigned long) hardfault_args[3]);

	stacked_r12 = ((unsigned long) hardfault_args[4]);
	stacked_lr = ((unsigned long) hardfault_args[5]);
	stacked_pc = ((unsigned long) hardfault_args[6]);
	stacked_psr = ((unsigned long) hardfault_args[7]);

	printf ("[Hard fault handler]\n");
	printf ("R0 = %x\n", stacked_r0);
	printf ("R1 = %x\n", stacked_r1);
	printf ("R2 = %x\n", stacked_r2);
	printf ("R3 = %x\n", stacked_r3);
	printf ("R12 = %x\n", stacked_r12);
	printf ("LR = %x\n", stacked_lr);
	printf ("PC = %x\n", stacked_pc);
	printf ("PSR = %x\n", stacked_psr);
	printf ("BFAR = %x\n", (*((volatile unsigned long *)(0xE000ED38))));
	printf ("CFSR = %x\n", (*((volatile unsigned long *)(0xE000ED28))));
	printf ("HFSR = %x\n", (*((volatile unsigned long *)(0xE000ED2C))));
	printf ("DFSR = %x\n", (*((volatile unsigned long *)(0xE000ED30))));
	printf ("AFSR = %x\n", (*((volatile unsigned long *)(0xE000ED3C))));

	return;
}

/*******************************************************************************
* Description : This function handles Hard Fault exception.
* Input       : -
* Return      : -
*******************************************************************************/
void HardFault_Handler(void)
{
    // Go to infinite loop when Hard Fault exception occurs
#if defined(BEACON) || defined(ROUTER) || defined(TIMEKEEPER)
        asm("TST LR, #4");
  	asm("ITE EQ");
  	asm("MRSEQ R0, MSP");
  	asm("MRSNE R0, PSP");
  	asm("B hard_fault_handler_c");
#endif
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

#if defined(BEACON) || defined(ROUTER) || defined(TIMEKEEPER)
    //SysTickCounter 10 ms timer
    //2.5 times per sec LED blink
  /*  if (flags & FLAG_LOADED) {
      HwLEDOn(LED1);
    } else {*/
      uint16_t speed = 400;
      if (flags & FLAG_LOADING) {
        speed = 40;
      }
      if (!(SysTickCounter % speed)) {
#ifndef REVJ_BEACON
#ifdef MAIN_MEMORY
#ifdef BEACON
        HwLEDToggle(LED3);
#else
        HwLEDToggle(LED2);
#endif
#else
        HwLEDToggle(LED1);
#endif
#endif
      }
 //   }
#endif

}


/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/


/*******************************************************************************
* Description : This function handles USB-On-The-Go FS global interrupt request.
* Input       : -
* Return      : -
*******************************************************************************/
void OTG_FS_IRQHandler(void) {

  //flags |= FLAG_USB_CONNECTED;

  USBD_OTG_ISR_Handler (&USB_OTG_dev);
}

/*******************************************************************************
* Description : This function handles External line 2 interrupt request.
*               GPI_RADIO_GPIO0 (RX_FRM_DONE) = PD.2
* Input       : -
* Return      : -
*******************************************************************************/
void EXTI2_IRQHandler(void) {

  EXTI_ClearITPendingBit(EXTI_Line2);
  //StatusType statusType = isr_PostSem(semRFRxFrames);
}


/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

extern void RX_Pkt_Handler(void);
extern ETH_DMADESCTypeDef  *DMATxDescToSet;
extern ETH_DMADESCTypeDef  *DMARxDescToGet;

#ifdef USE_ETHERNET

/**
  * @brief  This function handles ETH interrupt request.
  * @param  None
  * @retval None
  */
void ETH_IRQHandler(void) {
    if (ETH_GetDMAITStatus(ETH_DMA_IT_T) == SET) {      // interrupt of transmit
        assert((DMATxDescToSet->Status & ETH_DMATxDesc_OWN) == (u32)RESET);
        flags &= ~FLAG_ETHER_TX_PACKET;
        /* Clear the Eth DMA Tx IT pending bits */
        ETH_DMAClearITPendingBit(ETH_DMA_IT_T);
    }

    if (ETH_GetDMAITStatus(ETH_DMA_IT_R) == SET) {  // interrupt of receive

        /* Handles all the received frames */
        while(ETH_GetRxPktSize() != 0) {
            RX_Pkt_Handler();
        }

        /* Clear the Eth DMA Rx IT pending bits */
        ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
    }
    ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
}
#endif

extern void jump();

#ifdef BEACON
 // #ifndef MAIN_MEMORY
uint8_t on_usb_power = 0xFF;
void TIM1_UP_IRQHandler(void) {
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) {
      // Clear TIM1 Capture compare interrupt pending bit
      TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
       on_usb_power <<= 1;
       on_usb_power |= GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9);

    }
}

 // #endif
#endif

