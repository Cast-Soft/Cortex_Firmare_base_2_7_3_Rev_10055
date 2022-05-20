/******************** (C) COPYRIGHT 2011 NaturalPoint, Inc. ********************
* File Name          : hardware.h
* Author             : Wing Poon
* Version            : V1.0
* Date               : 6/2/2011
* Description        : Header file for hardware.c module.
*******************************************************************************/

/* DEFINE TO PREVENT RECURSIVE INCLUSION -------------------------------------*/
#ifndef __HARDWARE_H
#define __HARDWARE_H

/* INCLUDES ------------------------------------------------------------------*/

#include "stm32f10x.h"
#include <ysizet.h>

/* EXPORTED TYPES ------------------------------------------------------------*/

typedef enum
{
    GPI_RADIO_GPIO0 = 0,
    GPI_RADIO_GPIO1 = 1,
    GPI_RADIO_GPIO2 = 2,
    GPI_MS_SYNC = 3,
    GPI_AtoDin6 =4
} HwGPI_TypeDef;

typedef enum
{
    GPO_RADIO_GPIO2 = 0,
    GPO_OH_SYNC = 1,
    GPO_TP47 = 2,
    GPO_TP45 = 3,
    GPO_TP49 = 4,
    GPO_PHY_RST=5,  
    GPO_2520_RST=6,  
    GPO_RF_EN=7,
    GPO_TK_RAD_HGM=8,
    GPO_USB_HOST_EN=9
    
      
} HwGPO_TypeDef;

typedef enum
{
    LED1 = 0,
    LED2 = 1,
    LED3 = 2,
    LED4 = 3,
    LED5 = 4
} HwLED_TypeDef;

typedef enum
{
    BUTTON1 = 0,
    BUTTON2 = 1
} HwButton_TypeDef;

typedef enum
{
    COM1 = 0,
    COM2 = 1
} HwCOM_TypeDef;

typedef enum
{
    SPI_RADIO = 0,
} HwSPI_TypeDef;


typedef union {
uint16_t  RadioTemp_AtoD;
uint8_t   RTemperature[2];
}RadioTemp_Union_t ;

/* EXPORTED CONSTANTS --------------------------------------------------------*/

extern const uint16_t COM2TxBufSize;

/* EXPORTED MACROS -----------------------------------------------------------*/

#ifdef USE_MY_ASSERT
#define assert(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert(expr)
#endif

/* EXPORTED DEFINES ----------------------------------------------------------*/

/*******/
/* GPI */
/*******/

/* GPI_RADIO_GPIO0 */
#define GPI_RADIO_GPIO0_PORT        GPIOD
#define GPI_RADIO_GPIO0_CLK         RCC_APB2Periph_GPIOD
#define GPI_RADIO_GPIO0_PIN         GPIO_Pin_2
#define GPI_RADIO_GPIO0_TYPE        GPIO_Mode_IN_FLOATING

#define GPI_RADIO_GPIO0_PORT_SRC    GPIO_PortSourceGPIOD
#define GPI_RADIO_GPIO0_PIN_SRC     GPIO_PinSource2
#define GPI_RADIO_GPIO0_EXTI_LINE   EXTI_Line2
#define GPI_RADIO_GPIO0_IRQn        EXTI2_IRQn

/* GPI_RADIO_GPIO1 */
#define GPI_RADIO_GPIO1_PORT        GPIOD
#define GPI_RADIO_GPIO1_CLK         RCC_APB2Periph_GPIOD
#define GPI_RADIO_GPIO1_PIN         GPIO_Pin_1
#define GPI_RADIO_GPIO1_TYPE        GPIO_Mode_IN_FLOATING

#define GPI_RADIO_GPIO1_PORT_SRC    GPIO_PortSourceGPIOD
#define GPI_RADIO_GPIO1_PIN_SRC     GPIO_PinSource1
#define GPI_RADIO_GPIO1_EXTI_LINE   EXTI_Line1
#define GPI_RADIO_GPIO1_IRQn        EXTI1_IRQn

/* GPI_RADIO_GPIO2 */
#define GPI_RADIO_GPIO2_PORT        GPIOD
#define GPI_RADIO_GPIO2_CLK         RCC_APB2Periph_GPIOD
#define GPI_RADIO_GPIO2_PIN         GPIO_Pin_0
#define GPI_RADIO_GPIO2_TYPE        GPIO_Mode_IN_FLOATING

/* GPI_MS_SYNC */
#define GPI_MS_SYNC_PORT            GPIOB
#define GPI_MS_SYNC_CLK             RCC_APB2Periph_GPIOB
#define GPI_MS_SYNC_PIN             GPIO_Pin_7
#define GPI_MS_SYNC_TYPE            GPIO_Mode_IN_FLOATING

#define GPI_MS_SYNC_PORT_SRC        GPIO_PortSourceGPIOB
#define GPI_MS_SYNC_PIN_SRC         GPIO_PinSource7
#define GPI_MS_SYNC_EXTI_LINE       EXTI_Line7
#define GPI_MS_SYNC_IRQn            EXTI9_5_IRQn

/* GPI AtoDin6 */
#define GPI_AtoDin6_PORT           GPIOA
#define GPI_AtoDin6_CLK            RCC_APB2Periph_GPIOA
#define GPI_AtoDin6_PIN            GPIO_Pin_6
#define GPI_AtoDin6_TYPE           GPIO_Mode_AIN


/*******/
/* GPO */
/*******/

/* GPO_RADIO_GPIO2 */
#define GPO_RADIO_GPIO2_PORT        GPIOD
#define GPO_RADIO_GPIO2_CLK         RCC_APB2Periph_GPIOD
#define GPO_RADIO_GPIO2_PIN         GPIO_Pin_0

/* GPO_OH_SYNC */
#define GPO_OH_SYNC_PORT            GPIOB
#define GPO_OH_SYNC_CLK             RCC_APB2Periph_GPIOB
#define GPO_OH_SYNC_PIN             GPIO_Pin_6

/* GPO_TP47 */
#define GPO_TP47_PORT               GPIOA
#define GPO_TP47_CLK                RCC_APB2Periph_GPIOA
#define GPO_TP47_PIN                GPIO_Pin_0

/* GPO_TP45 */
#define GPO_TP45_PORT               GPIOA
#define GPO_TP45_CLK                RCC_APB2Periph_GPIOA
#define GPO_TP45_PIN                GPIO_Pin_5

/* GPO_TP49 */
#define GPO_TP49_PORT               GPIOA
#define GPO_TP49_CLK                RCC_APB2Periph_GPIOA
#define GPO_TP49_PIN                GPIO_Pin_6

/*GPO PHY Reset*/
#define GPO_PHY_RESET_PORT          GPIOB
#define GPO_PHY_RESET_CLK           RCC_APB2Periph_GPIOB
#define GPO_PHY_RESET_PIN           GPIO_Pin_10

/*GPO 2520_RST*/
#define GPO_2520_RST_PORT          GPIOD
#define GPO_2520_RST_CLK           RCC_APB2Periph_GPIOB
#define GPO_2520_RST_PIN           GPIO_Pin_5

/*GPO RF_EN */
#define GPO_RF_EN_PORT          GPIOD
#define GPO_RF_EN_CLK           RCC_APB2Periph_GPIOB
#define GPO_RF_EN_PIN           GPIO_Pin_6

/*GPO TK_RAD_HGM */
#define GPO_TK_RAD_HGM_PORT          GPIOD
#define GPO_TK_RAD_HGM_CLK           RCC_APB2Periph_GPIOB
#define GPO_TK_RAD_HGM_PIN           GPIO_Pin_4


/*GPO USB_HOST_EN */
#define GPO_USB_HOST_EN_PORT          GPIOA
#define GPO_USB_HOST_EN_CLK           RCC_APB2Periph_GPIOB
#define GPO_USB_HOST_EN_PIN           GPIO_Pin_10
/*******/
/* LED */
/*******/

/* LED1 */
#define LED1_PORT                   GPIOD
#define LED1_CLK                    RCC_APB2Periph_GPIOD
#define LED1_PIN                    GPIO_Pin_11

/* LED2 */
#define LED2_PORT                   GPIOD
#define LED2_CLK                    RCC_APB2Periph_GPIOD
#define LED2_PIN                    GPIO_Pin_12

/* LED3 */
#define LED3_PORT                   GPIOD
#define LED3_CLK                    RCC_APB2Periph_GPIOD
#define LED3_PIN                    GPIO_Pin_13

/* LED4 */
#define LED4_PORT                   GPIOD
#define LED4_CLK                    RCC_APB2Periph_GPIOD
#define LED4_PIN                    GPIO_Pin_14

/* LED5 */
#define LED5_PORT                   GPIOD
#define LED5_CLK                    RCC_APB2Periph_GPIOD
#define LED5_PIN                    GPIO_Pin_10

/**********/
/* BUTTON */
/**********/

/* BUTTON1 */
#define BUTTON1_PORT                GPIOC
#define BUTTON1_CLK                 RCC_APB2Periph_GPIOC
#define BUTTON1_PIN                 GPIO_Pin_7

/* BUTTON2 */
#define BUTTON2_PORT                GPIOC
#define BUTTON2_CLK                 RCC_APB2Periph_GPIOC
#define BUTTON2_PIN                 GPIO_Pin_8

/*******/
/* COM */
/*******/

#define COM_STDIO                   COM1

/* COM1 */
#define COM1_USART                  USART3                  // USART?
#define COM1_USART_CLK              RCC_APB1Periph_USART3   // RCC_APBI[1|2]Periph_USART?
#define COM1_REMAP                  GPIO_FullRemap_USART3   // 0 or GPIO_[Full]Remap_USART?
#define COM1_PORT                   GPIOD                   // GPIO?
#define COM1_CLK                    RCC_APB2Periph_GPIOD    // RCC_APB2Periph_GPIO?
#define COM1_PIN_TX                 GPIO_Pin_8              // GPIO_Pin_?
#define COM1_PIN_RX                 GPIO_Pin_9              // GPIO_Pin_?

/* COM2 */
#define COM2_USART                  USART1                  // USART?
#define COM2_USART_CLK              RCC_APB2Periph_USART1   // RCC_APBI[1|2]Periph_USART?
#define COM2_REMAP                  GPIO_Remap_USART1       // 0 or GPIO_[Full]Remap_USART?
#define COM2_PORT                   GPIOB                   // GPIO?
#define COM2_CLK                    RCC_APB2Periph_GPIOB    // RCC_APB2Periph_GPIO?
#define COM2_PIN_TX                 GPIO_Pin_6              // GPIO_Pin_?
#define COM2_PIN_RX                 GPIO_Pin_7              // GPIO_Pin_?

/* COM2 DMA */
#define COM2_DMA_CHAN_TX            DMA1_Channel4
#define COM2_DMA_IRQn               DMA1_Channel4_IRQn

/*******/
/* SPI */
/*******/

/* SPI_RADIO */
#define SPI_RADIO_SPI               SPI3
#define SPI_RADIO_SPI_CLK           RCC_APB1Periph_SPI3
#define SPI_RADIO_REMAP             GPIO_Remap_SPI3
#define SPI_RADIO_PORT              GPIOC
#define SPI_RADIO_CLK               RCC_APB2Periph_GPIOC
#define SPI_RADIO_PIN_SCK           GPIO_Pin_10
#define SPI_RADIO_PIN_MOSI          GPIO_Pin_12
#define SPI_RADIO_PIN_MISO          GPIO_Pin_11
#define SPI_RADIO_SS_PORT           GPIOA
#define SPI_RADIO_SS_CLK            RCC_APB2Periph_GPIOA
#define SPI_RADIO_SS_PIN            GPIO_Pin_4

#define SPI_RADIO_IRQn              SPI3_IRQn

/******************************************************************************/

#define TIMEKEEPER
#define INCLUDE_PA

#define GPIO_0_MASK                 0x0004
#define GPIO_1_MASK                 0x0002
#define GPIO_2_MASK                 0x0001

/* EXPORTED FUNCTIONS --------------------------------------------------------*/

void            HwGPIInit(HwGPI_TypeDef GPI);
uint32_t        HwGPIState(HwGPI_TypeDef GPI);

void            HwGPOInit(HwGPO_TypeDef GPO);
void            HwGPOInitOD(HwGPO_TypeDef GPO);   //open drain
void            HwGPOLow(HwGPO_TypeDef GPO);
void            HwGPOHigh(HwGPO_TypeDef GPO);
void            HwGPOToggle(HwGPO_TypeDef GPO);

void            HwLEDInit(HwLED_TypeDef Led);
void            HwLEDOn(HwLED_TypeDef Led);
void            HwLEDOff(HwLED_TypeDef Led);
void            HwLEDToggle(HwLED_TypeDef Led);

void            HwButtonInit(HwButton_TypeDef Button);
uint32_t        HwButtonPressed(HwButton_TypeDef Button);

void            HwCOMInit(HwCOM_TypeDef COM, USART_InitTypeDef *USART_InitStruct);

void            HwSPIInit(HwSPI_TypeDef SPI, SPI_InitTypeDef *SPI_InitStruct);
void            HwSPISSAssert(HwSPI_TypeDef SPI);
void            HwSPISSDeAssert(HwSPI_TypeDef SPI);

void            HwWait(unsigned int ticks);
void            HwTimerSet(unsigned int ticks);
unsigned int    HwTimerExpired(void);

void            HwPeriphInit(void);
void            HwRadioEXTIInit(void);
void            HwI2CInit(void);

void            writeI2cTest(void);

void            HwAtoDInit(void);

//size_t          __write(int handle, const unsigned char *buffer, size_t size);
//size_t          __read(int handle, unsigned char *buffer, size_t size);


/* EXTERNAL VARIABLES --------------------------------------------------------*/

extern uint32_t *frameId;
extern const uint16_t COM2TxBufSize;

#endif /*__HARDWARE_H*/
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/