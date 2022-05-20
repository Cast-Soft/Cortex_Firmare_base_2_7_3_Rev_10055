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
        - OTG_FS_IRQ
        - TIM3_IRQn
        - SPI3_IRQn (SPI_RADIO_IRQn)
        - EXTI2_IRQn (GPI_RADIO_GPIO0_IRQn)
        - EXTI1_IRQn (GPI_RADIO_GPIO1_IRQn)
        - DMA1_Channel4_IRQn (SPI_IMU_RX_DMA_IRQ)
        - EXTI15_10_IRQn (GPI_IMU_DIO1_IRQn)
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
#include "tasks.h"
#include "stm32f10x_i2c.h"
#include "i2c_ee.h"
#include "eink.h"
#include "CoOS.h"
//#include "stm32f10x_rtc.h"
#ifdef STDIO_TO_USART
#include <yfuns.h>
#endif

/* PRIVATE TYPEDEF -----------------------------------------------------------*/

/* PRIVATE DEFINES -----------------------------------------------------------*/

//#define STDIO_TO_USART

/* PRIVATE MACROS ------------------------------------------------------------*/

/* EXTERN VARIABLES ----------------------------------------------------------*/

/* PRIVATE VARIABLES ---------------------------------------------------------*/

static U64          SysTickTimerExp = 0;

// This is the SPI command string sent to the IMU to trigger a burst read of all registers
// We only read the 1st 7 which include xGyro,YGyro,ZGyro,x Accl,Yaccl, Zaccl

//const uint16_t spiIMUTxBuf[8] = {0x3E00, 0, 0, 0, 0, 0, 0, 0};
// Read and auto inc add = 0xC000 | address 0x2800
const uint16_t spiIMUTxBuf[8] = {0xC000 | 0x2800, 0, 0, 0, 0 ,0 ,0 ,0};

#define I2C_Speed              200000
#define I2C_SLAVE_ADDRESS7     0xA0
uint16_t EEPROM_ADDRESS = 0xA0;

//BC_HW_REVJ pin mapping
/* GPI_RADIO_GPIO0, GPI_RADIO_GPIO1, GPI_RADIO_GPIO2, GPI_SW_PWR, GPI_IMU_DIO1, GPI_IMU_DIO2, GPI_IRLED_DAC */
		GPIO_TypeDef*	GPI_PORT[] = {GPI_RADIO_GPIO0_PORT, GPI_RADIO_GPIO1_PORT, GPI_RADIO_GPIO2_PORT, GPI_SW_PWR_PORT, GPI_IMU_INT_PORT, GPI_INK_BUSY_PORT, GPI_IRLED_DAC_PORT, GPI_USB_VBUS_PORT,
                                      GPI_VBAT_ADC_PORT,GPI_CHG_STAT_PORT, GPI_RADIO_GPIO5_PORT, GPI_BAT_INT_PORT};

const uint32_t          GPI_CLK[] =  {GPI_RADIO_GPIO0_CLK,  GPI_RADIO_GPIO1_CLK,  GPI_RADIO_GPIO2_CLK,  GPI_SW_PWR_CLK,  GPI_IMU_INT_CLK, GPI_INK_BUSY_CLK, GPI_IRLED_DAC_CLK, GPI_USB_VBUS_CLK,
                                      GPI_VBAT_ADC_CLK, GPI_CHG_STAT_CLK, GPI_RADIO_GPIO5_CLK, GPI_BAT_INT_CLK};

const uint16_t          GPI_PIN[] =  {GPI_RADIO_GPIO0_PIN,  GPI_RADIO_GPIO1_PIN,  GPI_RADIO_GPIO2_PIN,  GPI_SW_PWR_PIN,  GPI_IMU_INT_PIN, GPI_INK_BUSY_PIN, GPI_IRLED_DAC_PIN,GPI_USB_VBUS_PIN,
                                      GPI_VBAT_ADC_PIN, GPI_CHG_STAT_PIN, GPI_RADIO_GPIO5_PIN, GPI_BAT_INT_PIN};

const GPIOMode_TypeDef  GPI_TYPE[] = {GPI_RADIO_GPIO0_TYPE, GPI_RADIO_GPIO1_TYPE, GPI_RADIO_GPIO2_TYPE, GPI_SW_PWR_TYPE, GPI_IMU_INT_TYPE, GPI_INK_BUSY_TYPE, GPI_IRLED_DAC_TYPE, GPI_USB_VBUS_TYPE,
                                      GPI_VBAT_ADC_TYPE,GPI_CHG_STAT_TYPE, GPI_RADIO_GPIO5_TYPE, GPI_BAT_INT_TYPE};
									  
/* GPO_RADIO_GPIO2, GPO_PWRON, GPO_IRLED0, GPO_IRLED1, GPO_IRLED2, GPO_TP10, GPO_TP11, GPO_TP12 */
      GPIO_TypeDef*     GPO_PORT[] = {GPO_RADIO_GPIO2_PORT, GPO_PWRON_PORT, GPO_IRLED0_PORT, GPO_IRLED1_PORT, GPO_IRLED2_PORT, GPO_2520_RST_PORT,
                                      GPO_RF_EN_PORT, GPO_VBATT_ADC_EN_PORT, GPO_RF_HGM_PORT, GPO_USB_VBUS_PORT, GPO_IMU_FSYNC_PORT, GPO_INK_RST_PORT};
const uint32_t          GPO_CLK[] =  {GPO_RADIO_GPIO2_CLK,  GPO_PWRON_CLK,  GPO_IRLED0_CLK,  GPO_IRLED1_CLK,  GPO_IRLED2_CLK,  GPO_2520_RST_CLK,
                                      GPO_RF_EN_CLK,  GPO_VBATT_ADC_EN_CLK, GPO_RF_HGM_CLK, GPO_USB_VBUS_CLK, GPO_IMU_FSYNC_CLK, GPO_INK_RST_CLK};
const uint16_t          GPO_PIN[] =  {GPO_RADIO_GPIO2_PIN,  GPO_PWRON_PIN,  GPO_IRLED0_PIN,  GPO_IRLED1_PIN,  GPO_IRLED2_PIN, GPO_2520_RST_PIN,
                                      GPO_RF_EN_PIN, GPO_VBATT_ADC_EN_PIN, GPO_RF_HGM_PIN, GPO_USB_VBUS_PIN, GPO_IMU_FSYNC_PIN, GPO_INK_RST_PIN};

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
      SPI_TypeDef*      SPI_SPI[] =      {SPI_RADIO_IMU_SPI, SPI_INK_SPI};
const uint32_t          SPI_SPI_CLK[] =  {SPI_RADIO_IMU_SPI_CLK, SPI_INK_SPI_CLK};
const uint32_t          SPI_REMAP[] =    {SPI_RADIO_IMU_REMAP, SPI_INK_REMAP};
      GPIO_TypeDef*     SPI_PORT[] =     {SPI_RADIO_IMU_PORT, SPI_INK_PORT};
const uint32_t          SPI_CLK[] =      {SPI_RADIO_IMU_CLK, SPI_INK_CLK};
const uint16_t          SPI_PIN_SCK[] =  {SPI_RADIO_IMU_PIN_SCK, SPI_INK_PIN_SCK};
const uint16_t          SPI_PIN_MOSI[] = {SPI_RADIO_IMU_PIN_MOSI, SPI_INK_PIN_MOSI};
const uint16_t          SPI_PIN_MISO[] = {SPI_RADIO_IMU_PIN_MISO, SPI_INK_PIN_MISO};
      GPIO_TypeDef*     SPI_SS_PORT[] =  {SPI_RADIO_SS_PORT, SPI_INK_SS_PORT, SPI_IMU_SS_PORT};
const uint16_t          SPI_SS_CLK[] =   {SPI_RADIO_SS_CLK,  SPI_INK_SS_CLK, SPI_IMU_SS_CLK};
const uint16_t          SPI_SS_PIN[] =   {SPI_RADIO_SS_PIN,  SPI_INK_SS_PIN, SPI_IMU_SS_PIN};

/* PUBLIC VARIABLES ----------------------------------------------------------*/

//volatile uint8_t spiIMURxBufDma[16];
//volatile uint16_t spiIMURxBuf[8];
volatile ImuBuffer_t ImuGyroBuffer, ImuAccelBuffer;


Batt_Union_t BattUnion, *pBattUnion;

ARM_proc_SN_t ARM_proc_SN;

uint16_t imu_data_test, imu_data_test2,imu_data_test3;

extern void IMURegWr(uint8_t addr, uint8_t Sensor, uint16_t val);
extern uint16_t IMURegRd(uint8_t addr, uint8_t Sensor);

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/
static void HwTIM1Init(void);
static void HwTIM3Init(void);
static void HwTIM4Init(void);
static void HwTIM5Init(void);
//static void HwTIM6Init(void);
static void HwTIM7Init(void);

static void HwRadioSPIInit(void);
static void HwInkPeriphInit(void);

static void HwDACInit(void);
static void HwAtoDInit(void);
static void HwGPOInitAF(HwGPO_TypeDef GPO);
/* PRIVATE FUNCTIONS ---------------------------------------------------------*/

/*******************************************************************************
* Description : Initialization of TIM1 (Timer for external IMU clock or triggers Non IMU sends)
*               this is used to start the IMU samples so they end at the right time for TxSlot assigned
*               ideally it should be set to 10ms but to ensure that it is faster than the RF Tx period
*               which is 50ms) we target 9.5ms (47.5ms for 5 packets) giving us a 2.5ms gaurd band
* Input       : -
* Return      : -
*******************************************************************************/
void HwTIM1Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    TIM_DeInit(TIM1);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    DBGMCU_Config(DBGMCU_TIM1_STOP, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler = 1199 ;     //3600 prescall = 20khz//
    TIM_TimeBaseStructure.TIM_Period = 59999;       // try: change to 10: to send faster.    21 count = 9.5 hz (pwm'd to give 9.5hz sq wave

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


/*******************************************************************************
* Description : Initialization of TIM2 (10Hz Radio TX Inhibit)
* Input       : -
* Return      : -
*******************************************************************************/
static void HwTIM2Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    /* Stop TIM2 when in debugger */
    DBGMCU_Config(DBGMCU_TIM2_STOP, ENABLE);

    /* TIM2 configuration */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseStructure.TIM_Prescaler = 119;      // 72MHz/120 = 600.0kHz
    TIM_TimeBaseStructure.TIM_Period = TIM_AUTORELOAD; //59999;       // 600.0khz/60,000 = 10Hz
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
  // Trigger IMU start
       TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM2, ENABLE); // ARR Preload Enable
    TIM_Cmd(TIM2, ENABLE);

    /* Enable the Capture Compare 2 & 3 Interrupt Request */
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);

    /* Enable and Set Interrupt Priority */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//0; //  Highest
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
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

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    /* Stop TIM3 when in debugger */
    DBGMCU_Config(DBGMCU_TIM3_STOP, ENABLE);

    /* TIM3 configuration */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_TimeBaseStructure.TIM_Prescaler = 11; // 3MHz //11;       // 72MHz/12 = 6.000MHz
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down; //TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = TIM3_AUTORELOAD; // 6.000Mhz/60,000 = 100Hz
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

 //   TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
// for outgoung Tx slot data capture start

    TIM_ARRPreloadConfig(TIM3, ENABLE); // ARR Preload Enable
    TIM_Cmd(TIM3, ENABLE);

    /* Enable the Capture Compare 2 & 3 Interrupt Request */
    TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    /* Enable and Set Interrupt Priority */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;// changed to 0 from 2 - with IMU ON LED id may jump 2;//1; // 1st Highest
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

#ifdef USE_TIM4
/*******************************************************************************
* Description : Initialization of TIM4 (Beacon 10Hz RF-Sync Freq Measurement)
* Input       : -
* Return      : -
*******************************************************************************/
static void HwTIM4Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    /* Stop TIM4 when in debugger */
    DBGMCU_Config(DBGMCU_TIM4_STOP, ENABLE);

    /* TIM4 configuration */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_TimeBaseStructure.TIM_Prescaler = 11; //239;      // 72MHz/240 = 300.0kHz
    TIM_TimeBaseStructure.TIM_Period = 59999; //0xFFFF;      // max 218ms
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    /* Configure Input Capture 1 */
#if 1
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
#endif
    TIM_ARRPreloadConfig(TIM4, ENABLE); // ARR Preload Enable
    TIM_Cmd(TIM4, ENABLE);
}
#endif

/*******************************************************************************
* Description : Initialization of TIM5 (STXONCCA Retries)
* Input       : -
* Return      : -
*******************************************************************************/
static void HwTIM5Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

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

#ifdef USE_TIM6
/*******************************************************************************
* Description : Initialization of TIM6 (LED sync for shunt circuit)
                -not sure if needed-
* Input       : -
* Return      : -
*******************************************************************************/
static void HwTIM6Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    /* Stop TIM6 when in debugger */
    //DBGMCU_Config(DBGMCU_TIM6_STOP, ENABLE);

    /* TIM6 configuration */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    TIM_TimeBaseStructure.TIM_Prescaler = 71;       // 72MHz/72 = 1MHz
    TIM_TimeBaseStructure.TIM_Period = 167;         // 168us
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
#endif

/*******************************************************************************
* Description : Initialization of SPI peripheral used by the Radio
				RevJ: also initializes IMU chip select
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
    SPI_I2S_ITConfig(SPI_RADIO_IMU_SPI, SPI_I2S_IT_RXNE, DISABLE);

    /* Enable and Set Interrupt Priority */
    NVIC_InitStructure.NVIC_IRQChannel = SPI_RADIO_IMU_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//2; // 2nd Highest
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Description : Initialization of Peripherals used by the Ink
* Input       : -
* Return      : -
*******************************************************************************/
static void HwInkPeriphInit(void) {
    SPI_InitTypeDef SPI_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    /* Initialize SPI_INK */
    SPI_I2S_DeInit(SPI_INK_SPI);
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; 
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    HwSPIInit(SPI_INK, &SPI_InitStructure);

    SPI_I2S_DMACmd(SPI_INK_SPI, SPI_I2S_DMAReq_Tx, ENABLE);
    SPI_I2S_DMACmd(SPI_INK_SPI, SPI_I2S_DMAReq_Rx, ENABLE);
    /* Enable DMA clock for SPI_INK */
    RCC_AHBPeriphClockCmd(SPI_INK_DMA_CLK, ENABLE);
    
    /* Initialize Rx DMA for SPI_INK */
    DMA_DeInit(SPI_INK_RX_DMA_CHAN);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(SPI_INK_SPI->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)spiInk_RxBuf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = sizeof(spiInk_RxBuf);
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(SPI_INK_RX_DMA_CHAN, &DMA_InitStructure);

    /* Initialize Tx DMA for SPI_INK */
    DMA_DeInit(SPI_INK_TX_DMA_CHAN);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(SPI_INK_SPI->DR); //Address of peripheral the DMA must map
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &spiInk_ImgBuf.regval;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = sizeof(spiInk_ImgBuf);
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(SPI_INK_TX_DMA_CHAN, &DMA_InitStructure);

    DMA_ITConfig(SPI_INK_RX_DMA_CHAN, DMA_IT_TC, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = SPI_INK_RX_DMA_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}	


/*******************************************************************************
* Description : Initialization of Digital to Analog converter used for IR Led brightness
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
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

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
    //__disable_interrupt();
    //GPO_PORT[GPO]->BRR = GPO_PIN[GPO];
    GPIO_ResetBits(GPO_PORT[GPO], GPO_PIN[GPO]);        // Does it equal to above
    //__enable_interrupt();
}

/*******************************************************************************
* Description : Output High on selected GPO
* Input       :
* Return      :
*******************************************************************************/
void HwGPOHigh(HwGPO_TypeDef GPO) {
    //GPO_PORT[GPO]->BSRR = GPO_PIN[GPO];
    GPIO_SetBits(GPO_PORT[GPO], GPO_PIN[GPO]);        // Does it equal to above
}

/*******************************************************************************
* Description : Toggles the selected GPO
* Input       :
* Return      :
*******************************************************************************/
void HwGPOToggle(HwGPO_TypeDef GPO) {
    //x__disable_interrupt();
    if(GPIO_ReadInputDataBit(GPO_PORT[GPO], GPO_PIN[GPO]))      //Does it equal to below?
    {
      HwGPOLow(GPO);
    }
    else
    {
      HwGPOHigh(GPO);
    }
    //GPO_PORT[GPO]->ODR ^= GPO_PIN[GPO];
    //x__enable_interrupt();
}

/*******************************************************************************
* Description : Returns the output pin value of the selected GPO
* Input       :
* Return      :
*******************************************************************************/
uint8_t HwGPOstatus(HwGPO_TypeDef GPO){
    return GPIO_ReadOutputDataBit(GPO_PORT[GPO], GPO_PIN[GPO]);
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

    if(SPI == SPI_RADIO)
    {
      // Configure SPI_IMU SS pin too
      /* Configure NSS as output push-pull */
      GPIO_InitStructure.GPIO_Pin = SPI_SS_PIN[SPI_IMU];
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_Init(SPI_SS_PORT[SPI_IMU], &GPIO_InitStructure);
      /* Output High (Deassert) on NSS */
      SPI_SS_PORT[SPI_IMU]->BSRR = SPI_SS_PIN[SPI_IMU];
    }	

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
    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    __IO uint16_t ADC1ConvertedValue = 0, ADC3ConvertedValue = 0;

// Init the data pointer to the battery data conversion union type
    pBattUnion = &BattUnion;
    BattUnion.Battery_AtoD= 0xD700; // int to 4.0 volts
/* Enable peripheral clocks --------------------------------------------------*/
    /* Enable DMA1 and DMA2 clocks */
    //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_DMA2, ENABLE);

    /* ADCCLK = PCLK2/4 */
    RCC_ADCCLKConfig(RCC_PCLK2_Div4);

    /* Enable ADC1 and GPIOC clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);

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

/*******************************************************************************
* Description : Assert NSS
* Input       :
* Return      : -
*******************************************************************************/
void HwSPISSAssert(HwSPI_TypeDef SPI) {
    SPI_SS_PORT[SPI]->BRR = SPI_SS_PIN[SPI];
}

/*******************************************************************************
* Description : DeAssert NSS
* Input       :
* Return      : -
*******************************************************************************/
void HwSPISSDeAssert(HwSPI_TypeDef SPI) {
    SPI_SS_PORT[SPI]->BSRR = SPI_SS_PIN[SPI];
}

/*******************************************************************************
* Description : DeAssert NSS
* Input       :
* Return      : -
*******************************************************************************/
uint32_t HwGetSPISS(HwSPI_TypeDef SPI) {
	//active low. high means deassert, low means assert
    return((~SPI_SS_PORT[SPI]->ODR) & SPI_SS_PIN[SPI]);
}
/*******************************************************************************
* Description : Uses SysTick ISR to Spin Wait
* Input       : # of SysTicks to wait
* Return      :
*******************************************************************************/
void HwWait(unsigned int ticks) {
    U64 systick_next = CoGetOSTime() + ticks;
    while (CoGetOSTime() < systick_next);
}

/*******************************************************************************
* Description : Uses SysTick ISR to Set a Timer
* Input       : # of SysTicks to wait
* Return      :
*******************************************************************************/
void HwTimerSet(unsigned int ticks) {
    SysTickTimerExp = CoGetOSTime() + ticks;
}

/*******************************************************************************
* Description : Uses SysTick ISR to Check if Timer has Expired
* Input       :
* Return      :
*******************************************************************************/
unsigned int HwTimerExpired(void) {
    return (CoGetOSTime() > SysTickTimerExp);
}
/*******************************************************************************
* Description : Inits Watchdog timer
* Input       :
* Return      :
*******************************************************************************/
void WDTimerInit(void){
#ifdef WDT_ENABLE
    /* Configure and Start IWDT */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    //IWDG_SetPrescaler(IWDG_Prescaler_4);    // timeout = 410ms(nom), 273ms(min)
    IWDG_SetPrescaler(IWDG_Prescaler_16);    // timeout = 1640ms(nom), 1092(min)
    IWDG_SetReload(0xFFF);                  // ""
    IWDG_Enable();
    /* Stop IWDT when in debugger */
    DBGMCU_Config(DBGMCU_IWDG_STOP, ENABLE);
#endif
}
/*******************************************************************************
* Description : Initialization of Peripherals used on the Beacon
* Input       : -
* Return      : -
*******************************************************************************/
void HwPeriphInit(void) {
    /* Enable AFIO Clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    /* Initialize PushButtons */
    HwButtonInit(BUTTON1);
    HwButtonInit(BUTTON2);

    /* Initialize GPIs */
    HwGPIInit(GPI_RADIO_GPIO0);
    HwGPIInit(GPI_RADIO_GPIO1);
    HwGPIInit(GPI_RADIO_GPIO2); 	// use CC2520's GPIO2 for TX state output
    HwGPIInit(GPI_RADIO_GPIO5); 	// use CC2520's GPIO5 for RX state output
    HwGPIInit(GPI_SW_PWR);
    HwGPIInit(GPI_CHG_STAT);
 
	HwGPIInit(GPI_IMU_INT);
	HwGPIInit(GPI_INK_BUSY);

    HwGPIInit(GPI_IRLED_DAC); // avoid parasitic consumption

    HwGPIInit(GPI_USB_VBUS);
    HwGPIInit(GPI_BAT_INT);

    /* Initialize GPOs */
    HwGPOInit(GPO_RADIO_GPIO2); HwGPOLow(GPO_RADIO_GPIO2); // use CC2520's GPIO2 for command strobes
    HwGPOInit(GPO_PWRON);       HwGPOHigh(GPO_PWRON); // assert BC_PWR_ON
    HwGPOInit(GPO_IRLED0);      HwGPOLow(GPO_IRLED0); // active-High
    HwGPOInit(GPO_IRLED1);      HwGPOLow(GPO_IRLED1); // active-High
    HwGPOInit(GPO_IRLED2);      HwGPOLow(GPO_IRLED2); // active-High

    HwGPOInit(GPO_2520_RST);    HwGPOLow(GPO_2520_RST);
    HwGPOInit(GPO_INK_RST);     HwGPOLow(GPO_INK_RST);
    HwGPOInit(GPO_RF_EN);       HwGPOLow(GPO_RF_EN);
    HwGPOInit(GPO_VBATT_ADC_EN);HwGPOLow(GPO_VBATT_ADC_EN);
    HwGPOInit(GPO_RF_HGM);      HwGPOLow(GPO_RF_HGM);

    HwGPOHigh(GPO_RF_EN);  // radio pwr on
    HwGPOHigh(GPO_2520_RST);
    HwGPOHigh(GPO_RF_HGM);
    HwGPOHigh(GPO_VBATT_ADC_EN);
    HwGPOHigh(GPO_INK_RST);

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
    HwTIM7Init();

#ifdef CCA_EN
    HwTIM5Init();
#endif
    /* Initialize Radio SPI */
    HwRadioSPIInit();	//REVJ also initializes IMU here
    /* Initialize EINK SPI */
	HwInkPeriphInit();
    HwWait(20);

    /* Initialize DAC */
    HwDACInit();

    /*init AtoD */
    HwAtoDInit();

    WDTimerInit();
}

/*******************************************************************************
* Description : Configures I2C port
* Input       :
* Return      :
*******************************************************************************/
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
    I2C_InitStructure.I2C_ClockSpeed = I2C_Speed/2;

    /* I2C Peripheral Enable */
        I2C_Cmd(I2C_EE, ENABLE);

    /* Apply I2C configuration after enabling it */
    I2C_Init(I2C_EE, &I2C_InitStructure);
}

void HwI2CInterruptInit(){
      NVIC_InitTypeDef NVIC_InitStructure;
        /* Default to Disable RXNE Interrupt Request */ //REVJ implementation portion of HwI2CInit
    I2C_ITConfig(I2C_EE, I2C_IT_ERR, DISABLE);
    I2C_ITConfig(I2C_EE, I2C_IT_EVT , DISABLE);

    /* Enable and Set Interrupt Priority */
    NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Configure I2C error interrupt to have the higher priority */
    NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
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
    //Need higher interrupt level, as it is important to synchronize clock
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // chnaged from 0 to 2 0;//3; // 3rd Highest
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

    /* Configure GPI_RADIO_GPIO2 => TX_UNDERFLOW | TX_OVERFLOW as EXTI Interrupt */

    // Connect EXTI Line to GPI_RADIO_GPIO2 Pin
    GPIO_EXTILineConfig(GPI_RADIO_GPIO2_PORT_SRC, GPI_RADIO_GPIO2_PIN_SRC);
    // Configure EXTI line
    EXTI_InitStructure.EXTI_Line = GPI_RADIO_GPIO2_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_ClearITPendingBit(GPI_RADIO_GPIO2_EXTI_LINE);
    // Enable and Set EXTI Interrupt Priority
    NVIC_InitStructure.NVIC_IRQChannel = GPI_RADIO_GPIO2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4; // 4th Highest
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Configure GPI_RADIO_GPIO5 => RX_UNDERFLOW | RX_OVERFLOW as EXTI Interrupt */

    // Connect EXTI Line to GPI_RADIO_GPIO5 Pin
    GPIO_EXTILineConfig(GPI_RADIO_GPIO5_PORT_SRC, GPI_RADIO_GPIO5_PIN_SRC);
    // Configure EXTI line
    EXTI_InitStructure.EXTI_Line = GPI_RADIO_GPIO5_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_ClearITPendingBit(GPI_RADIO_GPIO5_EXTI_LINE);
    // Enable and Set EXTI Interrupt Priority
    NVIC_InitStructure.NVIC_IRQChannel = GPI_RADIO_GPIO5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4; // 4th Highest
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

/*******************************************************************************
* Description : Initialization of EXTIs used by the IMU
* Input       : -
* Return      : -
*******************************************************************************/
void HwIMUEXTIInit(void) {
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Connect EXTI Line to GPI_IMU_INT Pin
    GPIO_EXTILineConfig(GPI_IMU_INT_PORT_SRC, GPI_IMU_INT_PIN_SRC);
    // Configure EXTI line
    EXTI_InitStructure.EXTI_Line = GPI_IMU_INT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_ClearITPendingBit(GPI_IMU_INT_EXTI_LINE);
    // Enable and Set EXTI Interrupt Priority
    NVIC_InitStructure.NVIC_IRQChannel = GPI_IMU_INT_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Description : Initialization of EXTIs used by the E-Ink Display
* Input       : -
* Return      : -
*******************************************************************************/
void HwEINKEXTIInit(void){
    EXTI_InitTypeDef EXTI_InitStructure;
    //NVIC_InitTypeDef NVIC_InitStructure;

    // Connect EXTI Line to GPI_INK_BUSY Pin
    GPIO_EXTILineConfig(GPI_INK_PORT_SRC, GPI_INK_PIN_SRC);
    // Configure EXTI line
    EXTI_InitStructure.EXTI_Line = GPI_INK_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_ClearITPendingBit(GPI_INK_EXTI_LINE);

    //The interrupt line used by the busy line is shared with the IMU. IMU initialized first.
    // Enable and Set EXTI Interrupt Priority
    //NVIC_InitStructure.NVIC_IRQChannel = GPI_INK_IRQn;
    //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; // chnaged from 0 to 2 0;//3; // 3rd Highest
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //NVIC_Init(&NVIC_InitStructure);
}
/*-----------------------------------------------------------*/
// ARM processor unique ID
// set at manufacturing time
// 96 bits UUID
/*-----------------------------------------------------------*/
void GetARM_UUID(void){


    ARM_proc_SN.a = *(__IO uint32_t *)(0x1FFFF7E8);
    ARM_proc_SN.b = *(__IO uint32_t *)(0x1FFFF7EC);
    ARM_proc_SN.c = *(__IO uint32_t *)(0x1FFFF7F0);

}

/*******************************************************************************
* Description : Initialization of timer used by user interface to indicate timeout
* Input       : -
* Return      : -
*******************************************************************************/
static void HwTIM7Init(void){
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    /* Stop TIM7 when in debugger */
    //DBGMCU_Config(DBGMCU_TIM7_STOP, ENABLE);

    /* TIM6 configuration */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    TIM_TimeBaseStructure.TIM_Prescaler = 14399;       // 72MHz/14400 = 5kHz
    TIM_TimeBaseStructure.TIM_Period = 49999;         // 5kHz*50000 = 10s
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

    TIM7->CNT=0;
    TIM_Cmd(TIM7,DISABLE);
    /*  Clear and Enable the Update Interrupt Request */
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

    /* Enable and Set Interrupt Priority */
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10; // 10th Highest
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/

