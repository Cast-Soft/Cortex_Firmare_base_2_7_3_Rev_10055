/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
* File Name          : hardware.c
* Author             : ?
* Version            : V1.0
* Date               : 6/2/2011
* Description        : ?
*******************************************************************************/

/*
    NVIC Priority (Highest first):
        - TIM3_IRQn
        - EXTI2_IRQn (GPI_RADIO_GPIO0_IRQn)
        - SPI3_IRQn (SPI_RADIO_IRQn)
        - EXTI1_IRQn (GPI_RADIO_GPIO1_IRQn)
        - DMA1_Channel4_IRQn (COM2_DMA_IRQn)
*/

/* INCLUDES ------------------------------------------------------------------*/
#include "hardware.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_dbgmcu.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_iwdg.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include "main.h"
#include "i2c_ee.h"

#define I2C_Speed 100000
uint16_t EEPROM_ADDRESS = 0xA0;

/* PRIVATE TYPEDEF -----------------------------------------------------------*/

/* PRIVATE DEFINES -----------------------------------------------------------*/

GPIO_TypeDef*           GPO_PORT[] = {GPO_PHY_RESET_PORT};
const uint32_t          GPO_CLK[] =  {GPO_PHY_RESET_CLK};
const uint16_t          GPO_PIN[] =  {GPO_PHY_RESET_PIN};

/* LED1, LED2, LED3, LED4 */
GPIO_TypeDef*           LED_PORT[] = {LED1_PORT, LED2_PORT, LED3_PORT, LED4_PORT,LED5_PORT};
const uint32_t          LED_CLK[] =  {LED1_CLK, LED2_CLK, LED3_CLK, LED4_CLK,LED5_CLK};
const uint16_t          LED_PIN[] =  {LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN,LED5_PIN};


/* PUBLIC VARIABLES ----------------------------------------------------------*/

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/




/*******************************************************************************
* Description : DMA Initialization for COM2 USART
* Input       : -
* Return      : -
*******************************************************************************/

/* PUBLIC FUNCTIONS ----------------------------------------------------------*/
/*******************************************************************************
* Description : Configures GPO GPIO Open Drain
* Input       :
* Return      :
*******************************************************************************/
void HwGPOInitOD(HwGPO_TypeDef GPO) {
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* Enable the GPIO_GPO Clock */
    RCC_APB2PeriphClockCmd(GPO_CLK[GPO], ENABLE);

    /* Configure the GPIO_GPO pin */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPO_PIN[GPO];
    GPIO_Init(GPO_PORT[GPO], &GPIO_InitStructure);
}

/*******************************************************************************
* Description : Output Low on selected GPO
* Input       :
* Return      :
*******************************************************************************/
void HwGPOLow(HwGPO_TypeDef GPO) {
    GPO_PORT[GPO]->BRR = GPO_PIN[GPO];
}

/*******************************************************************************
* Description : Output High on selected GPO
* Input       :
* Return      :
*******************************************************************************/
void HwGPOHigh(HwGPO_TypeDef GPO) {
    GPO_PORT[GPO]->BSRR = GPO_PIN[GPO];
}

/*******************************************************************************
* Description : Toggles the selected GPO
* Input       :
* Return      :
*******************************************************************************/
void HwGPOToggle(HwGPO_TypeDef GPO) {
    __disable_interrupt();
    GPO_PORT[GPO]->ODR ^= GPO_PIN[GPO];
    __enable_interrupt();
}

/*******************************************************************************
* Description : Toggles the selected LED
* Input       :
* Return      :
*******************************************************************************/
void HwLEDToggle(HwLED_TypeDef Led) {
    LED_PORT[Led]->ODR ^= LED_PIN[Led];
}

/*******************************************************************************
* Description : Configures LED GPIO
* Input       :
* Return      :
*******************************************************************************/
void HwLEDInit(HwLED_TypeDef Led) {
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* Enable the GPIO_LED Clock */
    RCC_APB2PeriphClockCmd(LED_CLK[Led], ENABLE);

    /* Configure the GPIO_LED pin */
 //   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = LED_PIN[Led];
    GPIO_Init(LED_PORT[Led], &GPIO_InitStructure);
}

/*******************************************************************************
* Description : Turns selected LED On
* Input       :
* Return      :
*******************************************************************************/
void HwLEDOn(HwLED_TypeDef Led) {
    LED_PORT[Led]->BRR = LED_PIN[Led];
}

/*******************************************************************************
* Description : Turns selected LED Off
* Input       :
* Return      :
*******************************************************************************/
void HwLEDOff(HwLED_TypeDef Led) {
    LED_PORT[Led]->BSRR = LED_PIN[Led];
}

#ifdef BEACON
  #ifndef MAIN_MEMORY
void HwTIM1Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    TIM_DeInit(TIM1);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    DBGMCU_Config(DBGMCU_TIM1_STOP, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler = 119 ;     //600 kHz
    TIM_TimeBaseStructure.TIM_Period = 59999;       // 10 Hz

    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_ARRPreloadConfig(TIM1, ENABLE); // ARR Preload Enable

    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    // Enable and Set Interrupt Priority
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9; // 9th Highest
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM1, ENABLE);
}
  #endif
#endif

void HwI2CInit(void){

  I2C_InitTypeDef  I2C_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  uint8_t recovery = 9;

  /* GPIO Periph clock enable */
  RCC_APB2PeriphClockCmd(I2C_EE_GPIO_CLK, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    /* I2C Periph clock enable */
  RCC_APB1PeriphClockCmd(I2C_EE_CLK, ENABLE);

  /* Now first thing would be to bit bang trying to recocver
   * from broken transmission
   * send up to 9 pulses on SCL until SDA is high
   */
   /* Set SDA to input and make sure it is high */
   /* ------ START I2C bit bang reset ------ */
   GPIO_InitStructure.GPIO_Pin =  I2C_EE_SDA;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(I2C_EE_GPIO, &GPIO_InitStructure);

   GPIO_InitStructure.GPIO_Pin =  I2C_EE_SCL;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
   GPIO_Init(I2C_EE_GPIO, &GPIO_InitStructure);

   GPIO_SetBits(GPIOB, I2C_EE_SCL);
   uint32_t time = SysTick->VAL ;
   while (SysTick->VAL == time);
   while (GPIO_ReadInputDataBit(GPIOB, I2C_EE_SDA) == Bit_RESET) {
     GPIO_ResetBits(GPIOB, I2C_EE_SCL);
     time =SysTick->VAL ;
     while (SysTick->VAL == time);
     GPIO_SetBits(GPIOB, I2C_EE_SCL);
     time = SysTick->VAL ;
     while (SysTick->VAL == time);
     recovery--;
     if (recovery == 0) {
        break;
     }
   }

   /* ------  END I2C bit bang reset  ------ */

  /* Enable alternate function remapping if necessary */
  GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);

  GPIO_InitStructure.GPIO_Pin =  I2C_EE_SCL | I2C_EE_SDA;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(I2C_EE_GPIO, &GPIO_InitStructure);


  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

  /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0; //I2C_SLAVE_ADDRESS7;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;

  /* I2C Peripheral Enable */
      I2C_Cmd(I2C_EE, ENABLE);

  /* Apply I2C configuration after enabling it */
  I2C_Init(I2C_EE, &I2C_InitStructure);




 // TRACE("I2C CR1: %x \n\r",I2C_ReadRegister(I2C1,I2C_Register_CR1));
//TRACE("I2C CR2: %x \n\r",I2C_ReadRegister(I2C1,I2C_Register_CR2));
// TRACE("I2C SR1: %x \n\r",I2C_ReadRegister(I2C1,I2C_Register_SR1));
// TRACE("I2C SR2: %x \n\r",I2C_ReadRegister(I2C1,I2C_Register_SR2));
//  TRACE("I2C_Register_CCR: %x \n\r",I2C_ReadRegister(I2C1,I2C_Register_CCR)) ;
// TRACE("I2C_Register_TRISE: %x \n\r",I2C_ReadRegister(I2C1,I2C_Register_TRISE)) ;
}


/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/

