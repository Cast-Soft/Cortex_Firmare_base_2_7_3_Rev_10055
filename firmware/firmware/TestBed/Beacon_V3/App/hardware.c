/******************** (C) COPYRIGHT 2011 NaturalPoint, Inc. ********************
* File Name          : hardware.c
* Author             : ?
* Version            : V1.0
* Date               : 6/2/2011
* Description        : ?
*******************************************************************************/
/*
    NVIC Priority (Highest first):
        - OTG_FS_IRQ
        - TIM3_IRQn
        - SPI3_IRQn (SPI_RADIO_IRQn)
        - EXTI2_IRQn (GPI_RADIO_GPIO0_IRQn)
        - EXTI1_IRQn (GPI_RADIO_GPIO1_IRQn)
        - DMA1_Channel4_IRQn (SPI_IMU_RX_DMA_IRQ)
        - EXTI9_5_IRQn (GPI_IMU_DIO1_IRQn)
        - TIM6_IRQn
        - TIM5_IRQn
        - TIM2_IRQn
*/

/* INCLUDES ------------------------------------------------------------------*/
#include "hardware.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_iwdg.h"
#include "stm32f10x_dbgmcu.h"
#include "stm32f10x_adc.h"
#ifdef COOS
#include "CoOS.h"
#else
#include "stm32f10x_it.h"
#endif
#ifdef STDIO_TO_USART
#include <yfuns.h>
#endif

/* PRIVATE TYPEDEF -----------------------------------------------------------*/

/* PRIVATE DEFINES -----------------------------------------------------------*/

//#define STDIO_TO_USART

/* PRIVATE MACROS ------------------------------------------------------------*/

/* EXTERN VARIABLES ----------------------------------------------------------*/

/* PRIVATE VARIABLES ---------------------------------------------------------*/

#ifdef COOS
static U64          SysTickTimerExp = 0;
#else
static unsigned int SysTickTimerExp = 0;
#endif

// This is the SPI command string sent to the IMU to trigger a burst read of all registers
// We only read the 1st 7 which include xGyro,YGyro,ZGyro,x Accl,Yaccl, Zaccl 

const uint16_t spiIMUTxBuf[8] = {0x3E00, 0, 0, 0, 0, 0, 0, 0};

#ifdef BC_HW_REVB

/* GPI_RADIO_GPIO0, GPI_RADIO_GPIO1, GPI_RADIO_GPIO2, GPI_SW_PWR, GPI_IMU_DIO1, GPI_IRLED_DAC */
      GPIO_TypeDef*     GPI_PORT[] = {GPI_RADIO_GPIO0_PORT, GPI_RADIO_GPIO1_PORT, GPI_RADIO_GPIO2_PORT, GPI_SW_PWR_PORT, GPI_IMU_DIO1_PORT, GPI_IRLED_DAC_PORT};
const uint32_t          GPI_CLK[] =  {GPI_RADIO_GPIO0_CLK,  GPI_RADIO_GPIO1_CLK,  GPI_RADIO_GPIO2_CLK,  GPI_SW_PWR_CLK,  GPI_IMU_DIO1_CLK,  GPI_IRLED_DAC_CLK};
const uint16_t          GPI_PIN[] =  {GPI_RADIO_GPIO0_PIN,  GPI_RADIO_GPIO1_PIN,  GPI_RADIO_GPIO2_PIN,  GPI_SW_PWR_PIN,  GPI_IMU_DIO1_PIN,  GPI_IRLED_DAC_PIN};
const GPIOMode_TypeDef  GPI_TYPE[] = {GPI_RADIO_GPIO0_TYPE, GPI_RADIO_GPIO1_TYPE, GPI_RADIO_GPIO2_TYPE, GPI_SW_PWR_TYPE, GPI_IMU_DIO1_TYPE, GPI_IRLED_DAC_TYPE};

/* GPO_RADIO_GPIO2, GPO_PWRON, GPO_IRLED0, GPO_IRLED1, GPO_IRLED2, GPO_TP10, GPO_TP11, GPO_TP12 */
      GPIO_TypeDef*     GPO_PORT[] = {GPO_RADIO_GPIO2_PORT, GPO_PWRON_PORT, GPO_IRLED0_PORT, GPO_IRLED1_PORT, GPO_IRLED2_PORT, GPO_TP10_PORT, GPO_TP11_PORT, GPO_TP12_PORT};
const uint32_t          GPO_CLK[] =  {GPO_RADIO_GPIO2_CLK,  GPO_PWRON_CLK,  GPO_IRLED0_CLK,  GPO_IRLED1_CLK,  GPO_IRLED2_CLK,  GPO_TP10_CLK,  GPO_TP11_CLK,  GPO_TP12_CLK};
const uint16_t          GPO_PIN[] =  {GPO_RADIO_GPIO2_PIN,  GPO_PWRON_PIN,  GPO_IRLED0_PIN,  GPO_IRLED1_PIN,  GPO_IRLED2_PIN,  GPO_TP10_PIN,  GPO_TP11_PIN,  GPO_TP12_PIN};

/* LED1, LED2, LED3, LED4 */
      GPIO_TypeDef*     LED_PORT[] = {LED1_PORT, LED2_PORT, LED3_PORT, LED4_PORT};
const uint32_t          LED_CLK[] =  {LED1_CLK, LED2_CLK, LED3_CLK, LED4_CLK};
const uint16_t          LED_PIN[] =  {LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN};


#endif



#ifdef BC_HW_REVC
/* GPI_RADIO_GPIO0, GPI_RADIO_GPIO1, GPI_RADIO_GPIO2, GPI_SW_PWR, GPI_IMU_DIO1, GPI_IRLED_DAC */
      GPIO_TypeDef*     GPI_PORT[] = {GPI_RADIO_GPIO0_PORT, GPI_RADIO_GPIO1_PORT, GPI_RADIO_GPIO2_PORT, GPI_SW_PWR_PORT, GPI_IMU_DIO1_PORT, GPI_IRLED_DAC_PORT, GPI_USB_VBUS_PORT,
                                      GPI_VBAT_ADC_PORT,GPI_CHG_STAT_PORT, GPI_AtoDin3_PORT};
      
const uint32_t          GPI_CLK[] =  {GPI_RADIO_GPIO0_CLK,  GPI_RADIO_GPIO1_CLK,  GPI_RADIO_GPIO2_CLK,  GPI_SW_PWR_CLK,  GPI_IMU_DIO1_CLK,  GPI_IRLED_DAC_CLK, GPI_USB_VBUS_CLK,
                                      GPI_VBAT_ADC_CLK, GPI_CHG_STAT_CLK, GPI_AtoDin3_CLK };

const uint16_t          GPI_PIN[] =  {GPI_RADIO_GPIO0_PIN,  GPI_RADIO_GPIO1_PIN,  GPI_RADIO_GPIO2_PIN,  GPI_SW_PWR_PIN,  GPI_IMU_DIO1_PIN,  GPI_IRLED_DAC_PIN,GPI_USB_VBUS_PIN,
                                      GPI_VBAT_ADC_PIN, GPI_CHG_STAT_PIN, GPI_AtoDin3_PIN};

const GPIOMode_TypeDef  GPI_TYPE[] = {GPI_RADIO_GPIO0_TYPE, GPI_RADIO_GPIO1_TYPE, GPI_RADIO_GPIO2_TYPE, GPI_SW_PWR_TYPE, GPI_IMU_DIO1_TYPE, GPI_IRLED_DAC_TYPE, GPI_USB_VBUS_TYPE,
                                      GPI_VBAT_ADC_TYPE,GPI_CHG_STAT_TYPE, GPI_AtoDin3_TYPE  };

/* GPO_RADIO_GPIO2, GPO_PWRON, GPO_IRLED0, GPO_IRLED1, GPO_IRLED2, GPO_TP10, GPO_TP11, GPO_TP12 */
      GPIO_TypeDef*     GPO_PORT[] = {GPO_RADIO_GPIO2_PORT, GPO_PWRON_PORT, GPO_IRLED0_PORT, GPO_IRLED1_PORT, GPO_IRLED2_PORT, GPO_TP10_PORT, GPO_TP11_PORT, GPO_TP12_PORT, GPO_2520_RST_PORT,
                                      GPO_RF_EN_PORT, GPO_5V_IMU_EN_PORT, GPO_VBATT_ADC_EN_PORT, GPO_IMU_RST_PORT, GPO_RF_HGM_PORT, GPO_TP50_PORT, GPO_IMU_CLK_PORT};
const uint32_t          GPO_CLK[] =  {GPO_RADIO_GPIO2_CLK,  GPO_PWRON_CLK,  GPO_IRLED0_CLK,  GPO_IRLED1_CLK,  GPO_IRLED2_CLK,  GPO_TP10_CLK,  GPO_TP11_CLK,  GPO_TP12_CLK, GPO_2520_RST_CLK,
                                      GPO_RF_EN_CLK, GPO_5V_IMU_EN_CLK, GPO_VBATT_ADC_EN_CLK, GPO_IMU_RST_CLK, GPO_RF_HGM_CLK, GPO_TP50_CLK, GPO_IMU_CLK_CLK};
const uint16_t          GPO_PIN[] =  {GPO_RADIO_GPIO2_PIN,  GPO_PWRON_PIN,  GPO_IRLED0_PIN,  GPO_IRLED1_PIN,  GPO_IRLED2_PIN,  GPO_TP10_PIN,  GPO_TP11_PIN,  GPO_TP12_PIN, GPO_2520_RST_PIN,
                                      GPO_RF_EN_PIN, GPO_5V_IMU_EN_PIN, GPO_VBATT_ADC_EN_PIN, GPO_IMU_RST_PIN, GPO_RF_HGM_PIN, GPO_TP50_PIN, GPO_IMU_CLK_PIN};


/* LED1, LED2, LED3, LED4 */
      GPIO_TypeDef*     LED_PORT[] = {LED1_PORT, LED2_PORT, LED3_PORT, LED4_PORT,LED5_PORT};
const uint32_t          LED_CLK[] =  {LED1_CLK, LED2_CLK, LED3_CLK, LED4_CLK,LED5_CLK};
const uint16_t          LED_PIN[] =  {LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN,LED5_PIN};

/* IRLED1, IRLED2, IRLED3 */
      GPIO_TypeDef*     IRLED_PORT[] = {GPO_IRLED0_PORT, GPO_IRLED1_PORT, GPO_IRLED2_PORT};
const uint32_t          IRLED_CLK[] =  {GPO_IRLED0_CLK, GPO_IRLED1_CLK, GPO_IRLED2_CLK};
const uint16_t          IRLED_PIN[] =  {GPO_IRLED0_PIN, GPO_IRLED1_PIN, GPO_IRLED2_PIN};

#endif



/* BUTTON1, BUTTON2 */
      GPIO_TypeDef*     BUTTON_PORT[] = {BUTTON1_PORT, BUTTON2_PORT};
const uint32_t          BUTTON_CLK[] =  {BUTTON1_CLK, BUTTON2_CLK};
const uint16_t          BUTTON_PIN[] =  {BUTTON1_PIN, BUTTON2_PIN};

/* COM1 */
      USART_TypeDef*    COM_USART[] =     {COM1_USART};
const uint32_t          COM_USART_CLK[] = {COM1_USART_CLK};
const uint32_t          COM_REMAP[] =     {COM1_REMAP};
      GPIO_TypeDef*     COM_PORT[] =      {COM1_PORT};
const uint32_t          COM_CLK[] =       {COM1_CLK};
const uint16_t          COM_PIN_RX[] =    {COM1_PIN_RX};
const uint16_t          COM_PIN_TX[] =    {COM1_PIN_TX};

/* SPI_RADIO, SPI_IMU */
      SPI_TypeDef*      SPI_SPI[] =      {SPI_RADIO_SPI, SPI_IMU_SPI};
const uint32_t          SPI_SPI_CLK[] =  {SPI_RADIO_SPI_CLK, SPI_IMU_SPI_CLK};
const uint32_t          SPI_REMAP[] =    {SPI_RADIO_REMAP, SPI_IMU_REMAP};
      GPIO_TypeDef*     SPI_PORT[] =     {SPI_RADIO_PORT, SPI_IMU_PORT};
const uint32_t          SPI_CLK[] =      {SPI_RADIO_CLK, SPI_IMU_CLK};
const uint16_t          SPI_PIN_SCK[] =  {SPI_RADIO_PIN_SCK, SPI_IMU_PIN_SCK};
const uint16_t          SPI_PIN_MOSI[] = {SPI_RADIO_PIN_MOSI, SPI_IMU_PIN_MOSI};
const uint16_t          SPI_PIN_MISO[] = {SPI_RADIO_PIN_MISO, SPI_IMU_PIN_MISO};
      GPIO_TypeDef*     SPI_SS_PORT[] =  {SPI_RADIO_SS_PORT, SPI_IMU_SS_PORT};
const uint16_t          SPI_SS_CLK[] =   {SPI_RADIO_SS_CLK, SPI_IMU_SS_CLK};
const uint16_t          SPI_SS_PIN[] =   {SPI_RADIO_SS_PIN, SPI_IMU_SS_PIN};

/* PUBLIC VARIABLES ----------------------------------------------------------*/

volatile uint16_t spiIMURxBuf[8];

Batt_Union_t BattUnion, *pBattUnion;

uint16_t imu_data_test;
extern uint16_t IMURegRd(uint8_t addr);

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/
static void HwTIM1Init(void);
static void HwTIM3Init(void);
static void HwTIM4Init(void);
static void HwTIM5Init(void);
static void HwTIM6Init(void);

static void HwRadioSPIInit(void);
static void HwIMUPeriphInit(void);
static void HwDACInit(void);
static void HwAtoDInit(void);
/* PRIVATE FUNCTIONS ---------------------------------------------------------*/

/*******************************************************************************
* Description : Initialization of TIM1 (Fre running 500uS counter for IMU Timestamp)
* Input       : -
* Return      : -
*******************************************************************************/
static void HwTIM1Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef     TIM_OCInitStructure;
     GPIO_InitTypeDef GPIO_InitStructure;

    /* Stop TIM2 when in debugger */
 //   DBGMCU_Config(DBGMCU_TIM1_STOP, ENABLE);

    /* TIM2 configuration */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    
    TIM_TimeBaseStructure.TIM_Prescaler = 35999;      // 72MHz/36000 = 2.0kHz
    TIM_TimeBaseStructure.TIM_Period = 0x2047;       // 0x800 (2048 d)rollover..each tick 500uS
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

   

    TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_Active;
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable;
     TIM_OCInitStructure.TIM_Pulse=0x0ff;
    TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCPolarity_Low ;
    TIM_OCInitStructure.TIM_OCIdleState=0;
    TIM_OCInitStructure.TIM_OCNIdleState=0;
    
    TIM_OC2Init(TIM1,&TIM_OCInitStructure) ;
     TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Disable);
    
     TIM_ARRPreloadConfig(TIM1, ENABLE); // ARR Preload Enable
      /* Enable the GPIO_GPO Clock */

 //     GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);

//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
     
 //  GPIO_InitStructure.GPIO_Pin =10;

  //     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
 //  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 //   GPIO_Init(GPIOE, &GPIO_InitStructure);

 //     HwGPOLow(GPO_IMU_CLK);
  //  GPIO_SetBits(GPIOA, GPIO_Pin_7);
 //    GPIO_SetBits(GPIOA, GPIO_Pin_8);

    TIM_Cmd(TIM1, ENABLE);

    
    
}



/*******************************************************************************
* Description : Initialization of TIM2 (10Hz Radio TX Inhibit)
* Input       : -
* Return      : -
*******************************************************************************/
static void HwTIM2Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Stop TIM2 when in debugger */
    DBGMCU_Config(DBGMCU_TIM2_STOP, ENABLE);

    /* TIM2 configuration */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseStructure.TIM_Prescaler = 119;      // 72MHz/120 = 600.0kHz
    TIM_TimeBaseStructure.TIM_Period = 59999;       // 600.0khz/60,000 = 10Hz
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* Configure Input Capture 1 */
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    /* Configure Output Compare 2 & 3 */
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM2, ENABLE); // ARR Preload Enable
    TIM_Cmd(TIM2, ENABLE);

    /* Enable the Capture Compare 2 & 3 Interrupt Request */
    TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);

    /* Enable and Set Interrupt Priority */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9; // 9th Highest
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Description : Initialization of TIM3 (100Hz IRLED)
* Input       : -
* Return      : -
*******************************************************************************/
static void HwTIM3Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Stop TIM3 when in debugger */
    DBGMCU_Config(DBGMCU_TIM3_STOP, ENABLE);

    /* TIM3 configuration */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_TimeBaseStructure.TIM_Prescaler = 11;       // 72MHz/12 = 6.000MHz
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 59999;       // 6.000Mhz/60,000 = 100Hz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* Configure Input Capture 1 */
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    /* Configure Output Compare 2 & 3 */
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE); // ARR Preload Enable
    TIM_Cmd(TIM3, ENABLE);

    /* Enable the Capture Compare 2 & 3 Interrupt Request */
    TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);

    /* Enable and Set Interrupt Priority */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 1st Highest
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
 //   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
       NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Description : Initialization of TIM4 (Beacon 10Hz RF-Sync Freq Measurement)
* Input       : -
* Return      : -
*******************************************************************************/
static void HwTIM4Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;

    /* Stop TIM4 when in debugger */
    DBGMCU_Config(DBGMCU_TIM4_STOP, ENABLE);

    /* TIM4 configuration */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_TimeBaseStructure.TIM_Prescaler = 239;      // 72MHz/240 = 300.0kHz
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;      // max 218ms
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    /* Configure Input Capture 1 */
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    TIM_ARRPreloadConfig(TIM4, ENABLE); // ARR Preload Enable
//    TIM_Cmd(TIM4, ENABLE);
}

/*******************************************************************************
* Description : Initialization of TIM5 (STXONCCA Retries)
* Input       : -
* Return      : -
*******************************************************************************/
static void HwTIM5Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Stop TIM5 when in debugger */
    DBGMCU_Config(DBGMCU_TIM5_STOP, ENABLE);

    /* TIM5 configuration */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    TIM_TimeBaseStructure.TIM_Prescaler = 71;       // 72MHz/72 = 1MHz
    TIM_TimeBaseStructure.TIM_Period = 999;         // 1ms
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    TIM5->CR1 &= ~TIM_CR1_ARPE;     // ARR Preload Disable
    TIM5->CR1 |= TIM_CR1_OPM;       // One-Pulse Mode
    TIM5->CR1 |= TIM_CR1_URS;       // Only Counter OF/UF generates UEV
    TIM5->CR1 &= ~TIM_CR1_UDIS;     // UEV Enabled

    /*  Clear and Enable the Update Interrupt Request */
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

    /* Enable and Set Interrupt Priority */
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8; // 8th Highest
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Description : Initialization of TIM6 (TX_FRM_DONE to flagRadioTxDone Delay)
* Input       : -
* Return      : -
*******************************************************************************/
static void HwTIM6Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Stop TIM6 when in debugger */
    DBGMCU_Config(DBGMCU_TIM6_STOP, ENABLE);

    /* TIM6 configuration */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    TIM_TimeBaseStructure.TIM_Prescaler = 71;       // 72MHz/72 = 1MHz
    TIM_TimeBaseStructure.TIM_Period = 349;         // 350us
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

    TIM6->CR1 &= ~TIM_CR1_ARPE;     // ARR Preload Disable
    TIM6->CR1 |= TIM_CR1_OPM;       // One-Pulse Mode
    TIM6->CR1 |= TIM_CR1_URS;       // Only Counter OF/UF generates UEV
    TIM6->CR1 &= ~TIM_CR1_UDIS;     // UEV Enabled

    /*  Clear and Enable the Update Interrupt Request */
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    /* Enable and Set Interrupt Priority */
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7; // 7th Highest
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}




/*******************************************************************************
* Description : Initialization of SPI peripheral used by the Radio
* Input       : -
* Return      : -
*******************************************************************************/
static void HwRadioSPIInit(void) {
    SPI_InitTypeDef SPI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Initialize SPI_RADIO */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    HwSPIInit(SPI_RADIO, &SPI_InitStructure);

    /* Default to Disable RXNE Interrupt Request */
    SPI_I2S_ITConfig(SPI_RADIO_SPI, SPI_I2S_IT_RXNE, DISABLE);

    /* Enable and Set Interrupt Priority */
    NVIC_InitStructure.NVIC_IRQChannel = SPI_RADIO_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // 2nd Highest
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Description : Initialization of Peripherals used by the IMU
* Input       : -
* Return      : -
*******************************************************************************/
static void HwIMUPeriphInit(void) {
    SPI_InitTypeDef SPI_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    /* Initialize SPI_IMU */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    HwSPIInit(SPI_IMU, &SPI_InitStructure);

    /* Enable SPI_MASTER DMA Rx & Tx Request */
    SPI_I2S_DMACmd(SPI_IMU_SPI, SPI_I2S_DMAReq_Rx, ENABLE);
    SPI_I2S_DMACmd(SPI_IMU_SPI, SPI_I2S_DMAReq_Tx, ENABLE);

    /* Enable DMA clock for SPI_IMU */
    RCC_AHBPeriphClockCmd(SPI_IMU_DMA_CLK, ENABLE);

    /* Initialize Rx DMA for SPI_IMU */
    DMA_DeInit(SPI_IMU_RX_DMA_CHAN);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(SPI_IMU_SPI->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)spiIMURxBuf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = sizeof(spiIMUTxBuf);
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(SPI_IMU_RX_DMA_CHAN, &DMA_InitStructure);

    /* Initialize Tx DMA for SPI_IMU */
    DMA_DeInit(SPI_IMU_TX_DMA_CHAN);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)spiIMUTxBuf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = sizeof(spiIMURxBuf);
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_Init(SPI_IMU_TX_DMA_CHAN, &DMA_InitStructure);

    /* Enable DMA Channel Transfer Complete Interrupt Request */
    DMA_ITConfig(SPI_IMU_RX_DMA_CHAN, DMA_IT_TC, ENABLE);

    /* Enable and Set Interrupt Priority */
    NVIC_InitStructure.NVIC_IRQChannel = SPI_IMU_RX_DMA_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5; // 5th Highest
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Configure GPI_IMU_DIO1 (PD.8) as EXTI Interrupt */

    // Connect EXTI Line to GPI_IMU_DIO1 Pin
    GPIO_EXTILineConfig(GPI_IMU_DIO1_PORT_SRC, GPI_IMU_DIO1_PIN_SRC);
    // Configure EXTI line
    EXTI_InitStructure.EXTI_Line = GPI_IMU_DIO1_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    // Enable and Set EXTI Interrupt Priority
    NVIC_InitStructure.NVIC_IRQChannel = GPI_IMU_DIO1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; // 6th Highest
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Description : Initialization of SPI peripheral used by the Radio
* Input       : -
* Return      : -
*******************************************************************************/
static void HwDACInit(void) {
    DAC_InitTypeDef DAC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

    /* DAC Channel2 Configuration */
    DAC_DeInit();
    DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
    DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
    DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
    DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
    DAC_Init(DAC_Channel_2, &DAC_InitStructure);

    DAC_Cmd(DAC_Channel_2, ENABLE);
}

/* PUBLIC FUNCTIONS ----------------------------------------------------------*/

/*******************************************************************************
* Description : Configures GPI GPIO
* Input       :
* Return      :
*******************************************************************************/
void HwGPIInit(HwGPI_TypeDef GPI) {
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* Enable the GPIO_GPI Clock */
    RCC_APB2PeriphClockCmd(GPI_CLK[GPI], ENABLE);

    /* Configure the GPIO_GPI pin */
    GPIO_InitStructure.GPIO_Mode = GPI_TYPE[GPI];
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPI_PIN[GPI];
    GPIO_Init(GPI_PORT[GPI], &GPIO_InitStructure);
}

/*******************************************************************************
* Description : Returns the selected GPI state
* Input       :
* Return      :
*******************************************************************************/
uint32_t HwGPIState(HwGPI_TypeDef GPI) {
    return(GPI_PORT[GPI]->IDR & GPI_PIN[GPI]);
}

/*******************************************************************************
* Description : Configures GPO GPIO
* Input       :
* Return      :
*******************************************************************************/
void HwGPOInit(HwGPO_TypeDef GPO) {
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* Enable the GPIO_GPO Clock */
    RCC_APB2PeriphClockCmd(GPO_CLK[GPO], ENABLE);

    /* Configure the GPIO_GPO pin */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPO_PIN[GPO];
    GPIO_Init(GPO_PORT[GPO], &GPIO_InitStructure);
}


/*******************************************************************************
* Description : Configures GPO GPIO open collector
* Input       :
* Return      :
*******************************************************************************/
void HwGPOInitOC(HwGPO_TypeDef GPO) {
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
* Description : Configures GPO GPIO Alternate Function
* Input       :
* Return      :
*******************************************************************************/
void HwGPOInitAF(HwGPO_TypeDef GPO) {
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* Enable the GPIO_GPO Clock */
    RCC_APB2PeriphClockCmd(GPO_CLK[GPO], ENABLE);

    /* Configure the GPIO_GPO pin */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
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
* Description : Configures LED GPIO
* Input       :
* Return      :
*******************************************************************************/
void HwLEDInit(HwLED_TypeDef Led) {
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* Enable the GPIO_LED Clock */
    RCC_APB2PeriphClockCmd(LED_CLK[Led], ENABLE);

    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
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

/*******************************************************************************
* Description : Toggles the selected LED
* Input       :
* Return      :
*******************************************************************************/
void HwLEDToggle(HwLED_TypeDef Led) {
    __disable_interrupt();
    LED_PORT[Led]->ODR ^= LED_PIN[Led];
    __enable_interrupt();
}
/*******************************************************************************
* Description : Turns selected LED On
* Input       :
* Return      :
*******************************************************************************/
void HwIRLEDOn(HwLED_TypeDef IRLed) {
    IRLED_PORT[IRLed]->BRR = IRLED_PIN[IRLed];
}

/*******************************************************************************
* Description : Turns selected LED Off
* Input       :
* Return      :
*******************************************************************************/
void HwIRLEDOff(HwLED_TypeDef IRLed) {
    IRLED_PORT[IRLed]->BSRR = IRLED_PIN[IRLed];
}

/*******************************************************************************
* Description : Toggles the selected LED
* Input       :
* Return      :
*******************************************************************************/
void HwIRLEDToggle(HwLED_TypeDef IRLed) {
    __disable_interrupt();
    IRLED_PORT[IRLed]->ODR ^= IRLED_PIN[IRLed];
    __enable_interrupt();
}

/*******************************************************************************
* Description : Configures Button GPIO
* Input       :
* Return      :
*******************************************************************************/
void HwButtonInit(HwButton_TypeDef Button) {
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable Button GPIO clock */
    RCC_APB2PeriphClockCmd(BUTTON_CLK[Button], ENABLE);

    /* Configure Button pin as input pull-up */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = BUTTON_PIN[Button];
    GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStructure);
}

/*******************************************************************************
* Description : Returns the selected Button state
* Input       :
* Return      :
*******************************************************************************/
uint32_t HwButtonPressed(HwButton_TypeDef Button) {
    return((~BUTTON_PORT[Button]->IDR) & BUTTON_PIN[Button]);
}

/*******************************************************************************
* Description : Configures COM port.
* Input       : <COM> COM?
*               <USART_InitStruct> Configuration Information
* Return      : -
*******************************************************************************/
void HwCOMInit(HwCOM_TypeDef COM, USART_InitTypeDef* USART_InitStruct) {
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(COM_CLK[COM], ENABLE);

    /* Enable alternate function remapping if necessary */
    if (COM_REMAP[COM]) {
        GPIO_PinRemapConfig(COM_REMAP[COM], ENABLE);
    }

    /* Configure TX as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = COM_PIN_TX[COM];
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(COM_PORT[COM], &GPIO_InitStructure);

    /* Configure RX as input pull-up */
    GPIO_InitStructure.GPIO_Pin = COM_PIN_RX[COM];
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(COM_PORT[COM], &GPIO_InitStructure);

    /* Enable peripheral clock */
    if (COM_USART_CLK[COM] == RCC_APB2Periph_USART1) {
        RCC_APB2PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
    } else {
        RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
    }

    /* Peripheral configuration */
    USART_Init(COM_USART[COM], USART_InitStruct);

    /* Enable peripheral */
    USART_Cmd(COM_USART[COM], ENABLE);
}

/*******************************************************************************
* Description : Configures SPI port
* Input       :
* Return      :
*******************************************************************************/
void HwSPIInit(HwSPI_TypeDef SPI, SPI_InitTypeDef *SPI_InitStruct) {
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(SPI_CLK[SPI] | SPI_SS_CLK[SPI], ENABLE);

    /* Enable alternate function remapping if necessary */
    if (SPI_REMAP[SPI]) {
        GPIO_PinRemapConfig(SPI_REMAP[SPI], ENABLE);
    }

    /* Configure SCK as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = SPI_PIN_SCK[SPI];
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SPI_PORT[SPI], &GPIO_InitStructure);

    /* Configure MOSI as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = SPI_PIN_MOSI[SPI];
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SPI_PORT[SPI], &GPIO_InitStructure);

    /* Configure MISO as input floating */
    GPIO_InitStructure.GPIO_Pin = SPI_PIN_MISO[SPI];
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(SPI_PORT[SPI], &GPIO_InitStructure);

    /* Configure NSS as output push-pull */
    GPIO_InitStructure.GPIO_Pin = SPI_SS_PIN[SPI];
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SPI_SS_PORT[SPI], &GPIO_InitStructure);

    /* Output High (Deassert) on NSS */
    SPI_SS_PORT[SPI]->BSRR = SPI_SS_PIN[SPI];

    /* Enable peripheral clock */
    if (SPI_SPI_CLK[SPI] == RCC_APB2Periph_SPI1) {
        RCC_APB2PeriphClockCmd(SPI_SPI_CLK[SPI], ENABLE);
    } else {
        RCC_APB1PeriphClockCmd(SPI_SPI_CLK[SPI], ENABLE);
    }

    /* Peripheral configuration */
    SPI_Init(SPI_SPI[SPI], SPI_InitStruct);

    /* Enable peripheral */
    SPI_Cmd(SPI_SPI[SPI], ENABLE);

    /* Flush input data register */
    if (SPI_I2S_GetFlagStatus(SPI_SPI[SPI], SPI_I2S_FLAG_RXNE) == SET)
        SPI_I2S_ReceiveData(SPI_SPI[SPI]);
}

/***************************************************************************/
/*******************************************************************************
* Description : Configures AtoD #1 In 18 on Port C3
* Input       :
* Return      :
*******************************************************************************/
void HwAtoDInit(void){

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//#define ADC1_DR_Address    ((uint32_t)0x4001244C)
//#define ADC3_DR_Address    ((uint32_t)0x40013C4C)

/* Private mac/ro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_InitTypeDef ADC_InitStructure;
//DMA_InitTypeDef DMA_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;



__IO uint16_t ADC1ConvertedValue = 0, ADC3ConvertedValue = 0;

// Init the data pointer to the battery data conversion union type
        pBattUnion = &BattUnion;
/* Enable peripheral clocks --------------------------------------------------*/
  /* Enable DMA1 and DMA2 clocks */
//  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_DMA2, ENABLE);

    /* ADCCLK = PCLK2/4 */
    RCC_ADCCLKConfig(RCC_PCLK2_Div4); 

  /* Enable ADC1 and GPIOC clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);

  /* Configure  PC.03 , ADC Channel13  as analog inputs */
 // GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3 ;
 // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
 // GPIO_Init(GPIOC, &GPIO_InitStructure);
/* Configure  PA.06 , ADC Channel6  as analog inputs */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 ;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
     
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  //ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
 // ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
  /* ADC1 regular channels configuration */ 
 // ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_28Cycles5);    
ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_28Cycles5); 
 /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */

 while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));

  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
  
    while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)== RESET){
            ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    }
}



/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */

/**
  * @brief  Configures Vector Table base location.
  * @param  None
  * @retval None
  */
/*
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
*/
  /* Configure and enable ADC interrupt */
/*
  NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

*/ 


/*******************************************************************************
* Description : Assert NSS
* Input       :
* Return      : -
*******************************************************************************/
void HwSPISSAssert(HwSPI_TypeDef SPI) {
    SPI_SS_PORT[SPI]->BRR = SPI_SS_PIN[SPI];
    //if (SPI == SPI_RADIO) HwGPOLow(GPO_TP11); // [[DEBUG]]
}

/*******************************************************************************
* Description : DeAssert NSS
* Input       :
* Return      : -
*******************************************************************************/
void HwSPISSDeAssert(HwSPI_TypeDef SPI) {
    SPI_SS_PORT[SPI]->BSRR = SPI_SS_PIN[SPI];
    //if (SPI == SPI_RADIO) HwGPOHigh(GPO_TP11); // [[DEBUG]]
}

/*******************************************************************************
* Description : Uses SysTick ISR to Spin Wait
* Input       : # of SysTicks to wait
* Return      :
*******************************************************************************/
void HwWait(unsigned int ticks) {
#ifdef COOS
    U64 systick_next = CoGetOSTime() + ticks;
    while (CoGetOSTime() < systick_next);
#else
    unsigned int systick_next = SysTickCounter + ticks;
    while (SysTickCounter < systick_next);
#endif
}

/*******************************************************************************
* Description : Uses SysTick ISR to Set a Timer
* Input       : # of SysTicks to wait
* Return      :
*******************************************************************************/
void HwTimerSet(unsigned int ticks) {
#ifdef COOS
    SysTickTimerExp = CoGetOSTime() + ticks;
#else
    SysTickTimerExp = SysTickCounter + ticks;
#endif
}

/*******************************************************************************
* Description : Uses SysTick ISR to Check if Timer has Expired
* Input       :
* Return      :
*******************************************************************************/
unsigned int HwTimerExpired(void) {
#ifdef COOS
    return (CoGetOSTime() > SysTickTimerExp);
#else
    return (SysTickCounter > SysTickTimerExp);
#endif
}

/*******************************************************************************
* Description : Initialization of Peripherals used on the Beacon
* Input       : -
* Return      : -
*******************************************************************************/
void HwPeriphInit(void) {
    USART_InitTypeDef USART_InitStructure;

    /* Enable AFIO Clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    /* Initialize LEDs */
    HwLEDInit(LED1); HwLEDOff(LED1);
    HwLEDInit(LED2); HwLEDOff(LED2);
    HwLEDInit(LED3); HwLEDOff(LED3);
    HwLEDInit(LED4); HwLEDOff(LED4);
    
#ifdef BC_HW_REVC 
    HwLEDInit(LED5); HwLEDOff(LED5);
    
#endif    
    /* Initialize PushButtons */
    HwButtonInit(BUTTON1);
    HwButtonInit(BUTTON2);

    /* Initialize GPIs */
    HwGPIInit(GPI_RADIO_GPIO0);
    HwGPIInit(GPI_RADIO_GPIO1);
    //HwGPIInit(GPI_RADIO_GPIO2); // use CC2520's GPIO2 for state output [[DEBUG]]
    HwGPIInit(GPI_SW_PWR);
    HwGPIInit(GPI_IMU_DIO1);
    HwGPIInit(GPI_IRLED_DAC); // avoid parasitic consumption

    /* Initialize GPOs */
    HwGPOInit(GPO_RADIO_GPIO2); HwGPOLow(GPO_RADIO_GPIO2); // use CC2520's GPIO2 for command strobes
    HwGPOInit(GPO_PWRON); HwGPOHigh(GPO_PWRON); // assert BC_PWR_ON
    HwGPOInit(GPO_IRLED0); HwGPOHigh(GPO_IRLED0); // active-low
    HwGPOInit(GPO_IRLED1); HwGPOHigh(GPO_IRLED1); // active-low
    HwGPOInit(GPO_IRLED2); HwGPOHigh(GPO_IRLED2); // active-low
    HwGPOInit(GPO_TP10); HwGPOLow(GPO_TP10);
    HwGPOInit(GPO_TP11); HwGPOLow(GPO_TP11);
 //   HwGPOInit(GPO_TP12); HwGPOLow(GPO_TP12);


    HwLEDOn(LED1);
    HwLEDOn(LED2);
    HwLEDOn(LED3);
    HwLEDOn(LED4);
#ifdef BC_HW_REVC    
    HwLEDOn(LED5);
#endif    
    
    
#ifdef BC_HW_REVC   
    HwGPIInit(GPI_AtoDin3);
    
     HwGPOInitOC(GPO_IMU_RST); HwGPOLow(GPO_IMU_RST);
    HwGPOInit(GPO_5V_IMU_EN); HwGPOLow(GPO_5V_IMU_EN);
   HwGPOInit(GPO_2520_RST); HwGPOLow(GPO_2520_RST);
    HwGPOInit(GPO_RF_EN); HwGPOLow(GPO_RF_EN);
    HwGPOInit(GPO_VBATT_ADC_EN);HwGPOLow(GPO_VBATT_ADC_EN);
    HwGPOInit(GPO_RF_HGM);HwGPOLow(GPO_RF_HGM);
    HwGPOInit(GPO_TP50);HwGPOLow(GPO_TP50);
   HwGPOInitAF(GPO_IMU_CLK);HwGPOLow(GPO_IMU_CLK);
    
    HwGPOHigh(GPO_5V_IMU_EN);
    
    HwGPOHigh(GPO_RF_EN);
    
    HwGPOHigh(GPO_IMU_RST);
    HwGPOHigh(GPO_2520_RST);
    HwGPOHigh(GPO_2520_RST);
    HwGPOHigh(GPO_RF_HGM);
    
    HwGPOHigh(GPO_VBATT_ADC_EN);
    
    HwGPOHigh(GPO_IMU_CLK);
#endif   
// Turn this off....not require for V2 or V3 Timekeepers
    /* Initialize COM1
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
    */
#ifdef STDIO_TO_USART
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   HwCOMInit(COM1, &USART_InitStructure);
#endif
    /* NVIC Priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* Initialize TIMs */
    HwTIM1Init();
    HwTIM2Init();
    HwTIM3Init();
    HwTIM4Init();
    HwTIM5Init();
    HwTIM6Init();
/*    
    HwLEDOn(LED1);
    HwLEDOn(LED2);
    HwLEDOn(LED3);
    HwLEDOn(LED4);
#ifdef BC_HW_REVC    
    HwLEDOn(LED5);
#endif
*/

    /* Initialize Radio SPI */
    HwRadioSPIInit();

    /* Initialize IMU Peripherals */
    HwIMUPeriphInit();
    
    HwWait(10);
    
 //   imu_data_test = IMURegRd(0x36);
    

    /* Initialize DAC */
    HwDACInit();

    /*init AtoD */
    HwAtoDInit();
    
#ifdef WDT_ENABLE
    /* Configure and Start IWDT */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_4);    // timeout = 410ms(nom), 273ms(min)
    IWDG_SetReload(0xFFF);                  // ""
    IWDG_Enable();
    /* Stop IWDT when in debugger */
    DBGMCU_Config(DBGMCU_IWDG_STOP, ENABLE);
#endif
}

/*******************************************************************************
* Description : Initialization of EXTIs used by the Radio
* Input       : -
* Return      : -
*******************************************************************************/
void HwRadioEXTIInit(void) {
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure GPI_RADIO_GPIO0(PD.2)=>RX_FRM_DONE as EXTI Interrupt */

    // Connect EXTI Line to GPI_RADIO_GPIO0 Pin
    GPIO_EXTILineConfig(GPI_RADIO_GPIO0_PORT_SRC, GPI_RADIO_GPIO0_PIN_SRC);
    // Configure EXTI line
    EXTI_InitStructure.EXTI_Line = GPI_RADIO_GPIO0_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_ClearITPendingBit(GPI_RADIO_GPIO0_EXTI_LINE);
    // Enable and Set EXTI Interrupt Priority
    NVIC_InitStructure.NVIC_IRQChannel = GPI_RADIO_GPIO0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 3rd Highest
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Configure GPI_RADIO_GPIO1(PD.1)=>TX_FRM_DONE as EXTI Interrupt */

    // Connect EXTI Line to GPI_RADIO_GPIO1 Pin
    GPIO_EXTILineConfig(GPI_RADIO_GPIO1_PORT_SRC, GPI_RADIO_GPIO1_PIN_SRC);
    // Configure EXTI line
    EXTI_InitStructure.EXTI_Line = GPI_RADIO_GPIO1_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_ClearITPendingBit(GPI_RADIO_GPIO1_EXTI_LINE);
    // Enable and Set EXTI Interrupt Priority
    NVIC_InitStructure.NVIC_IRQChannel = GPI_RADIO_GPIO1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4; // 4th Highest
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

#if STDIO_TO_USART
/*******************************************************************************
* Description : STDOUT Low-Level write
* Input       :
* Return      :
*******************************************************************************/
size_t __write(int handle, const unsigned char *buffer, size_t size) {
    size_t nChars = 0;

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

    for (/* Empty */; size != 0; --size) {
        while (USART_GetFlagStatus(COM_USART[COM_STDIO], USART_FLAG_TXE) == RESET);
        USART_SendData(COM_USART[COM_STDIO], *buffer++);
        ++nChars;
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
        // will NOT wait for char
        if (USART_GetFlagStatus(COM_USART[COM_STDIO], USART_FLAG_RXNE) == RESET) break;
        *buffer++ = USART_ReceiveData(COM_USART[COM_STDIO]);
        nChars++;
        size--;
    }

    return nChars;
}
#endif /* STDIO_TO_USART */

/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/

