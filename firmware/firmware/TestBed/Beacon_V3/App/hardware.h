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

#ifdef BC_HW_REVB

typedef enum
{
    GPI_RADIO_GPIO0 = 0,
    GPI_RADIO_GPIO1 = 1,
    GPI_RADIO_GPIO2 = 2,
    GPI_SW_PWR = 3,
    GPI_IMU_DIO1 = 4,
    GPI_IRLED_DAC = 5
} HwGPI_TypeDef;

typedef enum
{
    GPO_RADIO_GPIO2 = 0,
    GPO_PWRON = 1,
    GPO_IRLED0 = 2,
    GPO_IRLED1 = 3,
    GPO_IRLED2 = 4,
    GPO_TP10 = 5,
    GPO_TP11 = 6,
    GPO_TP12 = 7
   
} HwGPO_TypeDef;

typedef enum
{
    LED1 = 0,
    LED2 = 1,
    LED3 = 2,
    LED4 = 3
} HwLED_TypeDef;


#endif
#ifdef BC_HW_REVC
typedef enum
{
    GPI_RADIO_GPIO0 = 0,
    GPI_RADIO_GPIO1 = 1,
    GPI_RADIO_GPIO2 = 2,
    GPI_SW_PWR = 3,
    GPI_IMU_DIO1 = 4,
    GPI_IRLED_DAC = 5,
    GPI_USB_VBUS = 6,
    GPI_VBAT_ADC = 7,
    GPI_CHG_STAT =8,
    GPI_AtoDin3 = 9   
} HwGPI_TypeDef;

typedef enum
{
    GPO_RADIO_GPIO2 = 0,
    GPO_PWRON = 1,
    GPO_IRLED0 = 2,
    GPO_IRLED1 = 3,
    GPO_IRLED2 = 4,
    GPO_TP10 = 5,
    GPO_TP11 = 6,
    GPO_TP12 = 7,
    GPO_2520_RST=8,
    GPO_RF_EN =9,
    GPO_5V_IMU_EN = 10,
    GPO_VBATT_ADC_EN =11,
    GPO_IMU_RST = 12,
    GPO_RF_HGM =13,
       GPO_TP50 = 14,
    GPO_IMU_CLK =15
} HwGPO_TypeDef;

typedef enum
{
    LED1 = 2,
    LED2 = 3,
    LED3 = 1,
    LED4 = 0,
    LED5 = 4
} HwLED_TypeDef;

typedef enum
{
    IRLED1 = 0,
    IRLED2 = 1,
    IRLED3 = 2,
    
}HwIRLED_TypeDef;


#endif


typedef enum
{
    BUTTON1 = 0,
    BUTTON2 = 1
} HwButton_TypeDef;

typedef enum
{
    COM1 = 0
} HwCOM_TypeDef;

typedef enum
{
    SPI_RADIO = 0,
    SPI_IMU = 1
} HwSPI_TypeDef;

typedef union {
uint16_t  Battery_AtoD;
uint8_t   BatteryLevel[2];
}Batt_Union_t ;


/* EXPORTED CONSTANTS --------------------------------------------------------*/

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

/* GPI_SW_PWR */
#define GPI_SW_PWR_PORT             GPIOA
#define GPI_SW_PWR_CLK              RCC_APB2Periph_GPIOA
#define GPI_SW_PWR_PIN              GPIO_Pin_0
#define GPI_SW_PWR_TYPE             GPIO_Mode_IN_FLOATING

/* GPI_IMU_DIO1 */
#define GPI_IMU_DIO1_PORT           GPIOD
#define GPI_IMU_DIO1_CLK            RCC_APB2Periph_GPIOD
#define GPI_IMU_DIO1_PIN            GPIO_Pin_8
#define GPI_IMU_DIO1_TYPE           GPIO_Mode_IPD

#define GPI_IMU_DIO1_IRQn           EXTI9_5_IRQn

#define GPI_IMU_DIO1_PORT_SRC       GPIO_PortSourceGPIOD
#define GPI_IMU_DIO1_PIN_SRC        GPIO_PinSource8
#define GPI_IMU_DIO1_EXTI_LINE      EXTI_Line8

/* GPI_IRLED_DAC */
#define GPI_IRLED_DAC_PORT          GPIOA
#define GPI_IRLED_DAC_CLK           RCC_APB2Periph_GPIOA
#define GPI_IRLED_DAC_PIN           GPIO_Pin_5
#define GPI_IRLED_DAC_TYPE          GPIO_Mode_AIN


/*GPI USB_VBus */
#define GPI_USB_VBUS_PORT           GPIOA
#define GPI_USB_VBUS_CLK            RCC_APB2Periph_GPIOA
#define GPI_USB_VBUS_PIN            GPIO_Pin_2
#define GPI_USB_VBUS_TYPE           GPIO_Mode_IN_FLOATING

/*GPI AtoD input */
#define GPI_VBAT_ADC_PORT           GPIOC
#define GPI_VBAT_ADC_CLK            RCC_APB2Periph_GPIOC
#define GPI_VBAT_ADC_PIN            GPIO_Pin_8
#define GPI_VBAT_ADC_TYPE           GPIO_Mode_AIN

/*GPI CHG_STAT input */
#define GPI_CHG_STAT_PORT           GPIOC
#define GPI_CHG_STAT_CLK            RCC_APB2Periph_GPIOC
#define GPI_CHG_STAT_PIN            GPIO_Pin_9
#define GPI_CHG_STAT_TYPE           GPIO_Mode_IN_FLOATING

/* GPI AtoDin3 */
#define GPI_AtoDin3_PORT           GPIOA
#define GPI_AtoDin3_CLK            RCC_APB2Periph_GPIOA
#define GPI_AtoDin3_PIN            GPIO_Pin_3
#define GPI_AtoDin3_TYPE           GPIO_Mode_AIN



/*******/
/* GPO */
/*******/

/* GPO_RADIO_GPIO2 */
#define GPO_RADIO_GPIO2_PORT        GPIOD
#define GPO_RADIO_GPIO2_CLK         RCC_APB2Periph_GPIOD
#define GPO_RADIO_GPIO2_PIN         GPIO_Pin_0

/* GPO_PWRON */
#define GPO_PWRON_PORT              GPIOA
#define GPO_PWRON_CLK               RCC_APB2Periph_GPIOA
#define GPO_PWRON_PIN               GPIO_Pin_10



/* GPO_IRLED0 */
#define GPO_IRLED0_PORT             GPIOB
#define GPO_IRLED0_CLK              RCC_APB2Periph_GPIOB
#define GPO_IRLED0_PIN              GPIO_Pin_0

/* GPO_IRLED1 */
#define GPO_IRLED1_PORT             GPIOE
#define GPO_IRLED1_CLK              RCC_APB2Periph_GPIOE
#define GPO_IRLED1_PIN              GPIO_Pin_11

/* GPO_IRLED2 */
#define GPO_IRLED2_PORT             GPIOE
#define GPO_IRLED2_CLK              RCC_APB2Periph_GPIOE
#define GPO_IRLED2_PIN              GPIO_Pin_12

/* GPO_TP10 */
#define GPO_TP10_PORT               GPIOB
#define GPO_TP10_CLK                RCC_APB2Periph_GPIOB
#define GPO_TP10_PIN                GPIO_Pin_1

/* GPO_TP11 */
#define GPO_TP11_PORT               GPIOB
#define GPO_TP11_CLK                RCC_APB2Periph_GPIOB
#define GPO_TP11_PIN                GPIO_Pin_2

/* GPO_TP12 */
#define GPO_TP12_PORT               GPIOE
#define GPO_TP12_CLK                RCC_APB2Periph_GPIOE
#define GPO_TP12_PIN                GPIO_Pin_10

/* Type C hardware additions */

/* GPO_2520_RST */
#define GPO_2520_RST_PORT              GPIOB
#define GPO_2520_RST_CLK               RCC_APB2Periph_GPIOB
#define GPO_2520_RST_PIN               GPIO_Pin_5

/* GPO_RF_EN */
#define GPO_RF_EN_PORT              GPIOB
#define GPO_RF_EN_CLK               RCC_APB2Periph_GPIOB
#define GPO_RF_EN_PIN               GPIO_Pin_6

/* GPO_5V_IMU_EN */
#define GPO_5V_IMU_EN_PORT              GPIOB
#define GPO_5V_IMU_EN_CLK               RCC_APB2Periph_GPIOB
#define GPO_5V_IMU_EN_PIN               GPIO_Pin_7

/*GPO_VBATT_ADC_EN */
#define GPO_VBATT_ADC_EN_PORT              GPIOC
#define GPO_VBATT_ADC_EN_CLK               RCC_APB2Periph_GPIOC
#define GPO_VBATT_ADC_EN_PIN               GPIO_Pin_7

/*GPO_IMU_RST */
#define GPO_IMU_RST_PORT              GPIOD
#define GPO_IMU_RST_CLK               RCC_APB2Periph_GPIOD
#define GPO_IMU_RST_PIN               GPIO_Pin_11

/* GPO_RF_HGM */
#define GPO_RF_HGM_PORT              GPIOD
#define GPO_RF_HGM_CLK               RCC_APB2Periph_GPIOD
#define GPO_RF_HGM_PIN               GPIO_Pin_4

/* GPO_TP_50 */
#define GPO_TP50_PORT               GPIOA
#define GPO_TP50_CLK                RCC_APB2Periph_GPIOA
#define GPO_TP50_PIN                GPIO_Pin_7

/* GPO_IMU_CLK */
#define GPO_IMU_CLK_PORT               GPIOA
#define GPO_IMU_CLK_CLK                RCC_APB2Periph_GPIOA
#define GPO_IMU_CLK_PIN                GPIO_Pin_8

/*******/
/* LED */
/*******/

/* LED1 RED A*/
#define LED1_PORT                   GPIOE
#define LED1_CLK                    RCC_APB2Periph_GPIOE
#define LED1_PIN                    GPIO_Pin_0

/* LED2 GRN A*/
#define LED2_PORT                   GPIOE
#define LED2_CLK                    RCC_APB2Periph_GPIOE
#define LED2_PIN                    GPIO_Pin_1

/* LED3 RED B*/
#define LED3_PORT                   GPIOE
#define LED3_CLK                    RCC_APB2Periph_GPIOE
#define LED3_PIN                    GPIO_Pin_3

/* LED4 GRN B*/
#define LED4_PORT                   GPIOE
#define LED4_CLK                    RCC_APB2Periph_GPIOE
#define LED4_PIN                    GPIO_Pin_2

/* LED5 GRN C*/
#define LED5_PORT                   GPIOE
#define LED5_CLK                    RCC_APB2Periph_GPIOE
#define LED5_PIN                    GPIO_Pin_5

/**********/
/* BUTTON */
/**********/

/* BUTTON1 */
#define BUTTON1_PORT                GPIOA
#define BUTTON1_CLK                 RCC_APB2Periph_GPIOA
#define BUTTON1_PIN                 GPIO_Pin_1

/* BUTTON2 */
#define BUTTON2_PORT                GPIOB
#define BUTTON2_CLK                 RCC_APB2Periph_GPIOB
#define BUTTON2_PIN                 GPIO_Pin_11

/*******/
/* COM */
/*******/

#define COM_STDIO                   COM1

/* COM1 */
#define COM1_USART                  USART2                  // USART?
#define COM1_USART_CLK              RCC_APB1Periph_USART2   // RCC_APBI[1|2]Periph_USART?
#define COM1_REMAP                  0                       // 0 or GPIO_[Full]Remap_USART?
#define COM1_PORT                   GPIOA                   // GPIO?
#define COM1_CLK                    RCC_APB2Periph_GPIOA    // RCC_APB2Periph_GPIO?
#define COM1_PIN_TX                 GPIO_Pin_2              // GPIO_Pin_?
#define COM1_PIN_RX                 GPIO_Pin_3              // GPIO_Pin_?

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

/* SPI_IMU */
#define SPI_IMU_SPI                 SPI2
#define SPI_IMU_SPI_CLK             RCC_APB1Periph_SPI2
#define SPI_IMU_REMAP               0
#define SPI_IMU_PORT                GPIOB
#define SPI_IMU_CLK                 RCC_APB2Periph_GPIOB
#define SPI_IMU_PIN_SCK             GPIO_Pin_13
#define SPI_IMU_PIN_MOSI            GPIO_Pin_15
#define SPI_IMU_PIN_MISO            GPIO_Pin_14
#define SPI_IMU_SS_PORT             GPIOB
#define SPI_IMU_SS_CLK              RCC_APB2Periph_GPIOB
#define SPI_IMU_SS_PIN              GPIO_Pin_12

/* SPI_IMU DMA */
#define SPI_IMU_DMA                 DMA1
#define SPI_IMU_DMA_CLK             RCC_AHBPeriph_DMA1
#define SPI_IMU_RX_DMA_CHAN         DMA1_Channel4
#define SPI_IMU_RX_DMA_FLAG         DMA1_FLAG_TC4
#define SPI_IMU_RX_DMA_IRQ          DMA1_Channel4_IRQn
#define SPI_IMU_TX_DMA_CHAN         DMA1_Channel5
#define SPI_IMU_TX_DMA_FLAG         DMA1_FLAG_TC5

/******************************************************************************/

#define BEACON
#define INCLUDE_PA

#define GPIO_0_MASK                 0x0004
#define GPIO_1_MASK                 0x0002
#define GPIO_2_MASK                 0x0001

/* EXPORTED FUNCTIONS --------------------------------------------------------*/

void            HwGPIInit(HwGPI_TypeDef GPI);
uint32_t        HwGPIState(HwGPI_TypeDef GPI);

void            HwGPOInit(HwGPO_TypeDef GPO);
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

//size_t          __write(int handle, const unsigned char *buffer, size_t size);
//size_t          __read(int handle, unsigned char *buffer, size_t size);


/* EXTERNAL VARIABLES --------------------------------------------------------*/

extern volatile uint16_t spiIMURxBuf[];

#endif /*__HARDWARE_H*/
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/