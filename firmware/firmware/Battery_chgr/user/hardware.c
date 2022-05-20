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
#include "stm32f10x_conf.h"
#include "hardware.h"

//#include "stm32f10x_gpio.h"
//#include "stm32f10x_spi.h"
//#include "stm32f10x_i2c.h"
//#include "stm32f10x_usart.h"
//#include "stm32f10x_dma.h"
//#include "stm32f10x_exti.h"
//#include "stm32f10x_dbgmcu.h"
#include <stdio.h>

#include "i2c_ee.h"


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


/* PRIVATE FUNCTIONS ---------------------------------------------------------*/

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



    /* Initialize GPIs */

    HwGPIInit(GPI_MS_SYNC); // [[DEBUG]]

    /* Initialize GPOs */
    HwGPOInit(GPO_TP47); HwGPOLow(GPO_TP47);
    HwGPOInit(GPO_TP45); HwGPOLow(GPO_TP45);
    HwGPOInit(GPO_TP49); HwGPOLow(GPO_TP49);
    HwGPOInitOD(GPO_PHY_RST); HwGPOLow(GPO_PHY_RST);

    HwGPOInit(GPO_USB_HOST_EN); HwGPOLow(GPO_USB_HOST_EN);


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
//    HwCOM2DMAInit();

    /* NVIC Priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    //    HwGPOHigh(GPO_USB_HOST_EN);
    
 //     HwI2CInit();
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

