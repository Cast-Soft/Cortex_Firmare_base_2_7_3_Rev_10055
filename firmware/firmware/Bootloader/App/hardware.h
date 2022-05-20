/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
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
#include "stm32f10x_spi.h"
#include "stm32f10x_usart.h"
#include <ysizet.h>

/* EXPORTED TYPES ------------------------------------------------------------*/


typedef enum
{
    GPO_PHY_RST=0,
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

/* GPI_RADIO_GPIO5 */
#define GPI_RADIO_GPIO5_PORT        GPIOD
#define GPI_RADIO_GPIO5_CLK         RCC_APB2Periph_GPIOD
#define GPI_RADIO_GPIO5_PIN         GPIO_Pin_3
#define GPI_RADIO_GPIO5_TYPE        GPIO_Mode_IN_FLOATING

#define GPI_RADIO_GPIO5_PORT_SRC    GPIO_PortSourceGPIOD
#define GPI_RADIO_GPIO5_PIN_SRC     GPIO_PinSource3
#define GPI_RADIO_GPIO5_EXTI_LINE   EXTI_Line3
#define GPI_RADIO_GPIO5_IRQn        EXTI3_IRQn

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

/* GPO_OH_SYNC_OUT_EN */
#define GPO_OH_SYNC_OUT_EN_PORT     GPIOB
#define GPO_OH_SYNC_OUT_EN_CLK      RCC_APB2Periph_GPIOB
#define GPO_OH_SYNC_OUT_EN_PIN      GPIO_Pin_5

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
#define GPI_TP49_PIN_TYPE           GPIO_Mode_IN_FLOATING


/*GPO PHY Reset*/
#define GPO_PHY_RESET_PORT          GPIOB
#define GPO_PHY_RESET_CLK           RCC_APB2Periph_GPIOB
#define GPO_PHY_RESET_PIN           GPIO_Pin_10


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

#ifdef BEACON

/* LED1 RED A  D6 */
#define LED1_PORT                   GPIOE
#define LED1_CLK                    RCC_APB2Periph_GPIOE
#define LED1_PIN                    GPIO_Pin_0

/* LED2 GRN A D6 */
#define LED2_PORT                   GPIOE
#define LED2_CLK                    RCC_APB2Periph_GPIOE
#define LED2_PIN                    GPIO_Pin_1

/* LED3 RED B  D5 */
#define LED3_PORT                   GPIOE
#define LED3_CLK                    RCC_APB2Periph_GPIOE
#define LED3_PIN                    GPIO_Pin_3

/* LED4 GRN B D5 */
#define LED4_PORT                   GPIOE
#define LED4_CLK                    RCC_APB2Periph_GPIOE
#define LED4_PIN                    GPIO_Pin_2

/* LED5 GRN C  D4 */
#define LED5_PORT                   GPIOE
#define LED5_CLK                    RCC_APB2Periph_GPIOE
#define LED5_PIN                    GPIO_Pin_5

#endif
#if defined(ROUTER) || defined(TIMEKEEPER)

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

#endif

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


#define INCLUDE_PA

#define GPIO_0_MASK                 0x0004
#define GPIO_1_MASK                 0x0002
#define GPIO_2_MASK                 0x0001

/* EXPORTED FUNCTIONS --------------------------------------------------------*/


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

void            HwSPIInit(HwSPI_TypeDef SPI, SPI_InitTypeDef *SPI_InitStruct);
void            HwSPISSAssert(HwSPI_TypeDef SPI);
void            HwSPISSDeAssert(HwSPI_TypeDef SPI);

void            HwWait(unsigned int ticks);
void            HwTimerSet(unsigned int ticks);
unsigned int    HwTimerExpired(void);

void            HwPeriphInit(void);

#ifdef BEACON
  #ifndef MAIN_MEMORY
void            HwTIM1Init(void);
  #endif
#endif

/* EXTERNAL VARIABLES --------------------------------------------------------*/

extern uint32_t *frameId;
extern const uint16_t COM2TxBufSize;
extern uint16_t Enables;

#endif /*__HARDWARE_H*/
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/