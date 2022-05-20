/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
* File Name          : stm32f10x_it.h
* Author             : ???
* Version            : V1.0
* Date               : 6/2/2011
* Description        : This file contains the headers of the interrupt handlers.
*******************************************************************************/

/* DEFINE TO PREVENT RECURSIVE INCLUSION -------------------------------------*/
#ifndef __STM32F10x_IT_H
#define __STM32F10x_IT_H

/* INCLUDES ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* EXPORTED TYPES ------------------------------------------------------------*/

/* EXPORTED CONSTANTS --------------------------------------------------------*/

/* EXPORTED MACROS -----------------------------------------------------------*/

/* EXPORTED DEFINES ----------------------------------------------------------*/

/* EXPORTED FUNCTIONS --------------------------------------------------------*/

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);

void OTG_FS_IRQHandler(void);
void TIM3_IRQHandler(void);
void SPI3_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI1_IRQHandler(void);
void DMA1_Channel5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void TIM6_IRQHandler(void);
void TIM5_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM1_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
void I2C1_EV_IRQHandler(void);

uint16_t Tim4GetMovAvg(void);
uint16_t Tim3GetMovAvg(void);
uint16_t Tim2GetMovAvg(void);

/* EXTERNAL VARIABLES --------------------------------------------------------*/

extern volatile uint32_t frameIdAtSync;
extern volatile uint16_t txRetryState;
extern volatile uint16_t numTxRetries;

extern volatile uint32_t IMUSampleTime;

#endif  /*__STM32F10x_IT_H*/
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/