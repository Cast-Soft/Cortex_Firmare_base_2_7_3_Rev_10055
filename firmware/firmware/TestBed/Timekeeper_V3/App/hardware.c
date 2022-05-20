/******************** (C) COPYRIGHT 2011 NaturalPoint, Inc. ********************
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

#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_dbgmcu.h"
#include "stm32f10x_adc.h"
#include "i2c_ee.h"
#include "hardware.h"

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

#define I2C_Speed              200000
#define I2C_SLAVE_ADDRESS7     0xA0
uint16_t EEPROM_ADDRESS;
/* PRIVATE MACROS ------------------------------------------------------------*/

/* EXTERN VARIABLES ----------------------------------------------------------*/

/* PRIVATE VARIABLES ---------------------------------------------------------*/

#ifdef COOS
static U64          SysTickTimerExp = 0;
#else
static unsigned int SysTickTimerExp = 0;
#endif

static uint8_t I2C_Test_Str[] ={0xaa,0x55,0x88,0x55,0x81};

// OH Sync Frame :
// {SOF(0x5A), FRAMEID[7:0], FRAMEID[15:8], FRAMEID[23:16], FRAMEID[31:24], CAMERAID}
static uint8_t COM2TxBuf[] = {0x5A, 0x00, 0x00, 0x00, 0x00, 0x00};

/* GPI_RADIO_GPIO0, GPI_RADIO_GPIO1, GPI_RADIO_GPIO2, GPI_MS_SYNC */
      GPIO_TypeDef*     GPI_PORT[] = {GPI_RADIO_GPIO0_PORT, GPI_RADIO_GPIO1_PORT, GPI_RADIO_GPIO2_PORT, GPI_MS_SYNC_PORT, GPI_AtoDin6_PORT};
const uint32_t          GPI_CLK[] =  {GPI_RADIO_GPIO0_CLK, GPI_RADIO_GPIO1_CLK, GPI_RADIO_GPIO2_CLK, GPI_MS_SYNC_CLK, GPI_AtoDin6_CLK};
const uint16_t          GPI_PIN[] =  {GPI_RADIO_GPIO0_PIN, GPI_RADIO_GPIO1_PIN, GPI_RADIO_GPIO2_PIN, GPI_MS_SYNC_PIN, GPI_AtoDin6_PIN};
const GPIOMode_TypeDef  GPI_TYPE[] = {GPI_RADIO_GPIO0_TYPE, GPI_RADIO_GPIO1_TYPE, GPI_RADIO_GPIO2_TYPE, GPI_MS_SYNC_TYPE, GPI_AtoDin6_TYPE};

/* GPO_RADIO_GPIO2, GPI_OH_SYNC, GPO_TP47, GPO_TP45, GPO_TP49 */
      GPIO_TypeDef*     GPO_PORT[] = {GPO_RADIO_GPIO2_PORT, GPO_OH_SYNC_PORT, GPO_TP47_PORT, GPO_TP45_PORT, GPO_TP49_PORT,GPO_PHY_RESET_PORT,GPO_2520_RST_PORT,
                                      GPO_RF_EN_PORT, GPO_TK_RAD_HGM_PORT, GPO_USB_HOST_EN_PORT};
const uint32_t          GPO_CLK[] =  {GPO_RADIO_GPIO2_CLK, GPO_OH_SYNC_CLK, GPO_TP47_CLK, GPO_TP45_CLK, GPO_TP49_CLK,GPO_PHY_RESET_CLK,GPO_2520_RST_CLK,
                                      GPO_RF_EN_CLK, GPO_TK_RAD_HGM_CLK, GPO_USB_HOST_EN_CLK};
const uint16_t          GPO_PIN[] =  {GPO_RADIO_GPIO2_PIN, GPO_OH_SYNC_PIN, GPO_TP47_PIN, GPO_TP45_PIN, GPO_TP49_PIN,GPO_PHY_RESET_PIN,GPO_2520_RST_PIN,
                                      GPO_RF_EN_PIN, GPO_TK_RAD_HGM_PIN, GPO_USB_HOST_EN_PIN};

/* LED1, LED2, LED3, LED4, LED5 */
      GPIO_TypeDef*     LED_PORT[] = {LED1_PORT, LED2_PORT, LED3_PORT, LED4_PORT, LED5_PORT};
const uint32_t          LED_CLK[] =  {LED1_CLK, LED2_CLK, LED3_CLK, LED4_CLK, LED5_CLK};
const uint16_t          LED_PIN[] =  {LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN, LED5_PIN};

/* BUTTON1, BUTTON2 */
      GPIO_TypeDef*     BUTTON_PORT[] = {BUTTON1_PORT, BUTTON2_PORT};
const uint32_t          BUTTON_CLK[] =  {BUTTON1_CLK, BUTTON2_CLK};
const uint16_t          BUTTON_PIN[] =  {BUTTON1_PIN, BUTTON2_PIN};

/* COM1, COM2 */
      USART_TypeDef*    COM_USART[] =     {COM1_USART, COM2_USART};
const uint32_t          COM_USART_CLK[] = {COM1_USART_CLK, COM2_USART_CLK};
const uint32_t          COM_REMAP[] =     {COM1_REMAP, COM2_REMAP};
      GPIO_TypeDef*     COM_PORT[] =      {COM1_PORT, COM2_PORT};
const uint32_t          COM_CLK[] =       {COM1_CLK, COM2_CLK};
const uint16_t          COM_PIN_RX[] =    {COM1_PIN_RX, COM2_PIN_RX};
const uint16_t          COM_PIN_TX[] =    {COM1_PIN_TX, COM2_PIN_TX};

/* SPI_RADIO */
      SPI_TypeDef*      SPI_SPI[] =      {SPI_RADIO_SPI};
const uint32_t          SPI_SPI_CLK[] =  {SPI_RADIO_SPI_CLK};
const uint32_t          SPI_REMAP[] =    {SPI_RADIO_REMAP};
      GPIO_TypeDef*     SPI_PORT[] =     {SPI_RADIO_PORT};
const uint32_t          SPI_CLK[] =      {SPI_RADIO_CLK};
const uint16_t          SPI_PIN_SCK[] =  {SPI_RADIO_PIN_SCK};
const uint16_t          SPI_PIN_MOSI[] = {SPI_RADIO_PIN_MOSI};
const uint16_t          SPI_PIN_MISO[] = {SPI_RADIO_PIN_MISO};
      GPIO_TypeDef*     SPI_SS_PORT[] =  {SPI_RADIO_SS_PORT};
const uint16_t          SPI_SS_CLK[] =   {SPI_RADIO_SS_CLK};
const uint16_t          SPI_SS_PIN[] =   {SPI_RADIO_SS_PIN};

/* PUBLIC VARIABLES ----------------------------------------------------------*/

uint32_t *frameId = (uint32_t*)&COM2TxBuf[1];
const uint16_t COM2TxBufSize = sizeof(COM2TxBuf);

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/

static void HwTIM3Init(void);
static void HwCOM2DMAInit(void);
static void HwRadioSPIInit(void);

/* PRIVATE FUNCTIONS ---------------------------------------------------------*/

/*******************************************************************************
* Description : Initialization of TIM3 (100Hz OH Sync-Out, 10Hz RF-Sync Pkt)
* Input       : -
* Return      : -
*******************************************************************************/
static void HwTIM3Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Stop TIM3 when in debugger */
    DBGMCU_Config(DBGMCU_TIM3_STOP, ENABLE);

    /* TIM3 configuration */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_DeInit(TIM3);
    TIM_TimeBaseStructure.TIM_Prescaler = 11;       // 72MHz/12 = 6.000MHz
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 59999;       // 6.000Mhz/60,000 = 100Hz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_ARRPreloadConfig(TIM3, ENABLE); // ARR Preload Enable
    //TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  //    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM3, DISABLE);
    /* Enable the Update Interrupt Request */
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    /* Enable and Set Interrupt Priority */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // 1st Highest
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Description : DMA Initialization for COM2 USART
* Input       : -
* Return      : -
*******************************************************************************/
static void HwCOM2DMAInit(void) {
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* DMA clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* DMA configuration */
    DMA_DeInit(COM2_DMA_CHAN_TX);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(COM2_USART->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)COM2TxBuf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 1; // satisfy assert, will be overridden
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(COM2_DMA_CHAN_TX, &DMA_InitStructure);

    /* DMA Disabled until used */
    DMA_Cmd(COM2_DMA_CHAN_TX, DISABLE);

    /* Enable DMA Channel Transfer Complete Interrupt Request */
    DMA_ITConfig(COM2_DMA_CHAN_TX, DMA_IT_TC, ENABLE);

    /* Enable and Set Interrupt Priority */
    NVIC_InitStructure.NVIC_IRQChannel = COM2_DMA_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4; // 5th Highest
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Enable USARTy DMA TX request
    USART_DMACmd(COM2_USART, USART_DMAReq_Tx, ENABLE);
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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; // 3rd Highest
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
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
uint32_t HwGPIState(HwGPI_TypeDef GPI)
{
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
void HwLEDOn(HwLED_TypeDef Led)
{
    LED_PORT[Led]->BRR = LED_PIN[Led];
}

/*******************************************************************************
* Description : Turns selected LED Off
* Input       :
* Return      :
*******************************************************************************/
void HwLEDOff(HwLED_TypeDef Led)
{
    LED_PORT[Led]->BSRR = LED_PIN[Led];
}

/*******************************************************************************
* Description : Toggles the selected LED
* Input       :
* Return      :
*******************************************************************************/
void HwLEDToggle(HwLED_TypeDef Led)
{
    __disable_interrupt();
    LED_PORT[Led]->ODR ^= LED_PIN[Led];
    __enable_interrupt();
}

/*******************************************************************************
* Description : Configures Button GPIO
* Input       :
* Return      :
*******************************************************************************/
void HwButtonInit(HwButton_TypeDef Button)
{
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
uint32_t HwButtonPressed(HwButton_TypeDef Button)
{
    return((~BUTTON_PORT[Button]->IDR) & BUTTON_PIN[Button]);
}

/*******************************************************************************
* Description : Configures COM port.
* Input       : <COM> COM?
*               <USART_InitStruct> Configuration Information
* Return      : -
*******************************************************************************/
void HwCOMInit(HwCOM_TypeDef COM, USART_InitTypeDef* USART_InitStruct)
{
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
* Description : Configures I2C port
* Input       :
* Return      :
*******************************************************************************/
void HwI2CInit(void){
  
I2C_InitTypeDef  I2C_InitStructure;  

GPIO_InitTypeDef GPIO_InitStructure;

  /* I2C Periph clock enable */
  RCC_APB1PeriphClockCmd(I2C_EE_CLK, ENABLE);   
  
  /* GPIO Periph clock enable */
  RCC_APB2PeriphClockCmd(I2C_EE_GPIO_CLK, ENABLE);    

    GPIO_InitStructure.GPIO_Pin =  I2C_EE_SCL | I2C_EE_SDA; 
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(I2C_EE_GPIO, &GPIO_InitStructure);

    /* Enable alternate function remapping if necessary */    
     GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
    

  /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDRESS7;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;
  
  /* I2C Peripheral Enable */
  I2C_Cmd(I2C_EE, ENABLE);
  /* Apply I2C configuration after enabling it */
  I2C_Init(I2C_EE, &I2C_InitStructure);
    

  EEPROM_ADDRESS = EEPROM_HW_ADDRESS; 


//   RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 , ENABLE);
//   I2C_Cmd(I2C1, ENABLE);
//   I2C_Init(I2C1,&I2CInitStruct);  

    
       
        
  printf("I2C CR1: %x \n\r",I2C_ReadRegister(I2C1,I2C_Register_CR1));
printf("I2C CR2: %x \n\r",I2C_ReadRegister(I2C1,I2C_Register_CR2));
 printf("I2C SR1: %x \n\r",I2C_ReadRegister(I2C1,I2C_Register_SR1));       
 printf("I2C SR2: %x \n\r",I2C_ReadRegister(I2C1,I2C_Register_SR2));
  printf("I2C_Register_CCR: %x \n\r",I2C_ReadRegister(I2C1,I2C_Register_CCR)) ;
 printf("I2C_Register_TRISE: %x \n\r",I2C_ReadRegister(I2C1,I2C_Register_TRISE)) ; 
}

/*---------------------------------------------------------------------------*/
// writeI2CTest
//
//
//
/*---------------------------------------------------------------------------*/
void writeI2cTest(void){
  
       I2C_EE_ByteWrite(I2C_Test_Str, 0x00)  ;


  
 
   
}


/*==========================================================================*/
// A to  D converter setup...
//
//
//
/*==========================================================================*/
void HwAtoDInit(void){
 uint16_t    Temp_A2D = 0;  
   GPIO_InitTypeDef GPIO_InitStructure;
   ADC_InitTypeDef  ADC_InitStructure;
  
   
      /* ADCCLK = PCLK2/4 */
    RCC_ADCCLKConfig(RCC_PCLK2_Div4); 

  /* Configure PC.02, PC.03 and PC.04 (ADC Channel12, ADC Channel13 and 
     ADC Channel14) as analog inputs */
 // GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3 ;
 // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
 // GPIO_Init(GPIOC, &GPIO_InitStructure);
 /* Enable ADC1, ADC2, ADC3 and GPIOC clocks */
 // RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 |
 //                        RCC_APB2Periph_ADC3 | RCC_APB2Periph_GPIOC, ENABLE);
  
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
  
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
//  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
  /* ADC1 regular channels configuration */ 
 // ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_28Cycles5);  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_28Cycles5);
// ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 1, ADC_SampleTime_28Cycles5);
  
  /* Enable ADC1 DMA */
  //ADC_DMACmd(ADC1, ENABLE);


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
  
  ADC_TempSensorVrefintCmd(ENABLE);

 /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);


  
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
    HwLEDInit(LED5); HwLEDOff(LED5);

    /* Initialize PushButtons */
    HwButtonInit(BUTTON1);
    HwButtonInit(BUTTON2);

    /* Initialize GPIs */
    HwGPIInit(GPI_RADIO_GPIO0);
    HwGPIInit(GPI_RADIO_GPIO1);
    //HwGPIInit(GPI_RADIO_GPIO2); // use CC2520's GPIO2 for state output
    HwGPIInit(GPI_MS_SYNC); // [[DEBUG]]
    HwGPIInit(GPI_AtoDin6); // for radio temperature sensor

    /* Initialize GPOs */
    HwGPOInit(GPO_RADIO_GPIO2); HwGPOLow(GPO_RADIO_GPIO2); // use CC2520's GPIO2 for command strobes
    HwGPOInit(GPO_TP47); HwGPOLow(GPO_TP47);
    HwGPOInit(GPO_TP45); HwGPOLow(GPO_TP45);
    HwGPOInit(GPO_TP49); HwGPOLow(GPO_TP49);
    HwGPOInitOD(GPO_PHY_RST); HwGPOLow(GPO_PHY_RST);
    
#ifdef   TK_V3_hdwr 
    HwGPOInit(GPO_2520_RST); HwGPOLow(GPO_2520_RST);
    HwGPOInit(GPO_RF_EN); HwGPOLow(GPO_RF_EN);
    HwGPOInit(GPO_TK_RAD_HGM); HwGPOLow(GPO_TK_RAD_HGM);
    HwGPOInit(GPO_USB_HOST_EN); HwGPOLow(GPO_USB_HOST_EN);
#endif    


    /* Initialize COM1
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
    */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    HwCOMInit(COM1, &USART_InitStructure);

    /* Initialize COM2
        - BaudRate = 252525 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
    */
    USART_InitStructure.USART_BaudRate = 252525;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    HwCOMInit(COM2, &USART_InitStructure);
    HwCOM2DMAInit();

    /* NVIC Priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    /* Initialize TIMs */
    HwTIM3Init();

    /* Initialize Radio SPI */
    HwRadioSPIInit();

#ifdef TK_V3_hdwr    
//    HwAtoDInit();
#endif    
    
//    HwGPOHigh(GPO_USB_HOST_EN);
    
 //     HwI2CInit();
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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // 2nd Highest
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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; // 4th Highest
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

#if STDIO_TO_USART
/*******************************************************************************
* Description : STDOUT Low-Level write
* Input       :
* Return      :
*******************************************************************************/
size_t __write(int handle, const unsigned char *buffer, size_t size)
{
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
size_t __read(int handle, unsigned char *buffer, size_t size)
{
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

