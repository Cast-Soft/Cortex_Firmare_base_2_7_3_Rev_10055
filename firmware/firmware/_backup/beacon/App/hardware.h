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
#include <ysizet.h>
#include "config.h"

/* EXPORTED TYPES ------------------------------------------------------------*/

#ifdef BC_HW_REVB

typedef enum
{
    GPI_RADIO_GPIO0 = 0,
    GPI_RADIO_GPIO1 = 1,
    GPI_RADIO_GPIO2 = 2,
    GPI_SW_PWR = 3,
    GPI_IMU_DIO1 = 4,
    GPI_IMU_DIO2 = 5,
    GPI_IRLED_DAC = 6
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


#elif defined(BC_HW_REVC)// all hardware Post Rev B

typedef enum
{
    GPI_RADIO_GPIO0 = 0,
    GPI_RADIO_GPIO1 = 1,
    GPI_RADIO_GPIO2 = 2,
    GPI_SW_PWR = 3,
    GPI_IMU_DIO1 = 4,
    GPI_IMU_DIO2 = 5,
    GPI_IRLED_DAC = 6,
    GPI_USB_VBUS = 7,
    GPI_VBAT_ADC = 8,
    GPI_CHG_STAT =9,
    GPI_AtoDin3 = 10,
    GPI_RADIO_GPIO5 = 11
} HwGPI_TypeDef;

typedef enum
{
    GPO_RADIO_GPIO2 = 0,
    GPO_PWRON = 1,
    GPO_IRLED0 = 4, //swapped with GPO_IRLED2 to map to label on case
    GPO_IRLED1 = 3,
    GPO_IRLED2 = 2, //swapped with GPO_IRLED0
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
    GPO_IMU_CLK =15,
    GPO_USB_VBUS=16
} HwGPO_TypeDef;


typedef enum
{
    LED1 = 2,
    LED2 = 3,
    LED3 = 1,
    LED4 = 0,
    LED5 = 4


} HwLED_TypeDef;

#else //BC_HW_REVJ and later

typedef enum
{
    GPI_RADIO_GPIO0 = 0,	
    GPI_RADIO_GPIO1 = 1,	
    GPI_RADIO_GPIO2 = 2,	//both input and output
    GPI_SW_PWR = 3,
	
    GPI_IMU_INT = 4,
    GPI_INK_BUSY = 5,
    
	GPI_IRLED_DAC = 6,		//used for DAC waveform? (A2: a test point)
    GPI_USB_VBUS = 7,		//mapped to a testpoint A2: TP4??
    GPI_VBAT_ADC = 8,		//mapped to wrong GPIO? (C8)
    GPI_CHG_STAT =9,		
    GPI_AtoDin3 = 10,		//same input as GPI_RADIO_GPIO0 but analog input (A3)
    GPI_RADIO_GPIO5 = 11
} HwGPI_TypeDef;

typedef enum
{
    GPO_RADIO_GPIO2 = 0,	//both input and output
    GPO_PWRON = 1,
    GPO_IRLED0 = 4, 		
    GPO_IRLED1 = 3,			//swapped with GPO_IRLED2
    GPO_IRLED2 = 2, 		//swapped with GPO_IRLED1
    GPO_TP6 = 5,			//TP6
    GPO_TP7 = 6,			//TP7
    GPO_TP10 = 7,			//TP10
    GPO_2520_RST=8,			
    GPO_RF_EN =9,
    GPO_5V_IMU_EN = 10,		//mapped to unused GPIO (B7)
    GPO_VBATT_ADC_EN =11,	
    //GPO_IMU_RST = 12,		//mapped to INT2_A in REVH. Interferes with REVJ usage. remove.
    GPO_RF_HGM =12,			
    GPO_TP5 = 13,			//TP5
    GPO_IMU_CLK =14,		//mapped to unused GPIO (A8)
    GPO_USB_VBUS=15,			
	GPO_IMU_FSYNC=16,
	GPO_INK_RST=17
} HwGPO_TypeDef;

//Rev J+ does not include LEDs.
/*typedef enum
{
    LED1 = 2,
    LED2 = 3,
    LED3 = 1,
    LED4 = 0,
    LED5 = 4
} HwLED_TypeDef;*/
/*typedef enum
{
	RADIO_CS = 0,
	IMU_CS = 1
}SPI3_CS_TypeDef;
*/
#endif

typedef struct
{
  uint32_t xSum;
  uint32_t ySum;
  uint32_t zSum;
  uint8_t count;
}ImuBuffer_t;

typedef enum
{
    BUTTON1 = 0,
    BUTTON2 = 1
} HwButton_TypeDef;

typedef enum
{
    COM1 = 0
} HwCOM_TypeDef;

#ifndef BC_HW_REVJ
typedef enum
{
    SPI_RADIO = 0,
    SPI_A_IMU = 1,
    SPI_G_IMU = 2,
} HwSPI_TypeDef;

#else
typedef enum
{
    SPI_RADIO = 0,
	SPI_IMU = 2,
    SPI_INK = 1,
} HwSPI_TypeDef;

#endif

typedef union {
uint16_t  Battery_AtoD;
uint8_t   BatteryLevel[2];
}Batt_Union_t ;


typedef struct {
    uint32_t  a;
    uint32_t b;
    uint32_t  c;
}ARM_proc_SN_t;
extern ARM_proc_SN_t ARM_proc_SN;

/* EXPORTED CONSTANTS --------------------------------------------------------*/

/* EXPORTED MACROS -----------------------------------------------------------*/
/*
#ifdef USE_MY_ASSERT
#define assert(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert(expr)
#endif
*/
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

#define GPI_RADIO_GPIO2_PORT_SRC    GPIO_PortSourceGPIOD
#define GPI_RADIO_GPIO2_PIN_SRC     GPIO_PinSource0
#define GPI_RADIO_GPIO2_EXTI_LINE   EXTI_Line0
#define GPI_RADIO_GPIO2_IRQn        EXTI0_IRQn

/* GPI_RADIO_GPIO5 */
#define GPI_RADIO_GPIO5_PORT        GPIOD
#define GPI_RADIO_GPIO5_CLK         RCC_APB2Periph_GPIOD
#define GPI_RADIO_GPIO5_PIN         GPIO_Pin_3
#define GPI_RADIO_GPIO5_TYPE        GPIO_Mode_IN_FLOATING

#define GPI_RADIO_GPIO5_PORT_SRC    GPIO_PortSourceGPIOD
#define GPI_RADIO_GPIO5_PIN_SRC     GPIO_PinSource3
#define GPI_RADIO_GPIO5_EXTI_LINE   EXTI_Line3
#define GPI_RADIO_GPIO5_IRQn        EXTI3_IRQn

/* GPI_SW_PWR */
#define GPI_SW_PWR_PORT             GPIOA
#define GPI_SW_PWR_CLK              RCC_APB2Periph_GPIOA
#define GPI_SW_PWR_PIN              GPIO_Pin_0
#define GPI_SW_PWR_TYPE             GPIO_Mode_IN_FLOATING

#ifndef BC_HW_REVJ 	//DIO lines for IMU are not used in REVJ
/* GPI_IMU_DIO1 */
#define GPI_IMU_DIO1_PORT           GPIOD
#define GPI_IMU_DIO1_CLK            RCC_APB2Periph_GPIOD
#define GPI_IMU_DIO1_PIN            GPIO_Pin_12
#define GPI_IMU_DIO1_TYPE           GPIO_Mode_IPU

#define GPI_IMU_DIO1_IRQn           EXTI15_10_IRQn

#define GPI_IMU_DIO1_PORT_SRC       GPIO_PortSourceGPIOD
#define GPI_IMU_DIO1_PIN_SRC        GPIO_PinSource12
#define GPI_IMU_DIO1_EXTI_LINE      EXTI_Line12

/* GPI_IMU_DIO2 */
#define GPI_IMU_DIO2_PORT           GPIOD
#define GPI_IMU_DIO2_CLK            RCC_APB2Periph_GPIOD
#define GPI_IMU_DIO2_PIN            GPIO_Pin_8
#define GPI_IMU_DIO2_TYPE           GPIO_Mode_IPU

#define GPI_IMU_DIO2_IRQn           EXTI9_5_IRQn

#define GPI_IMU_DIO2_PORT_SRC       GPIO_PortSourceGPIOD
#define GPI_IMU_DIO2_PIN_SRC        GPIO_PinSource8
#define GPI_IMU_DIO2_EXTI_LINE      EXTI_Line8


#else	//REVJ and above:
/* GPI_IMU_INT*/
#define GPI_IMU_INT_PORT           	GPIOD
#define GPI_IMU_INT_CLK            	RCC_APB2Periph_GPIOD
#define GPI_IMU_INT_PIN            	GPIO_Pin_11
#define GPI_IMU_INT_TYPE           	GPIO_Mode_IN_FLOATING

#define GPI_IMU_INT_PORT_SRC       	GPIO_PortSourceGPIOD
#define GPI_IMU_INT_PIN_SRC        	GPIO_PinSource11
#define GPI_IMU_INT_EXTI_LINE      	EXTI_Line11
#define GPI_IMU_INT_IRQn           	EXTI15_10_IRQn

/*GPI_INK_BUSY*/
#define GPI_INK_BUSY_PORT          GPIOD
#define GPI_INK_BUSY_CLK           RCC_APB2Periph_GPIOD
#define GPI_INK_BUSY_PIN           GPIO_Pin_12
#define GPI_INK_BUSY_TYPE          GPIO_Mode_IPU
#endif

/* GPI_IRLED_DAC */
#define GPI_IRLED_DAC_PORT          GPIOA
#define GPI_IRLED_DAC_CLK           RCC_APB2Periph_GPIOA
#define GPI_IRLED_DAC_PIN           GPIO_Pin_5
#define GPI_IRLED_DAC_TYPE          GPIO_Mode_AIN

#ifdef BC_HW_REVB

/*GPI USB_VBus */
#define GPI_USB_VBUS_PORT           GPIOA
#define GPI_USB_VBUS_CLK            RCC_APB2Periph_GPIOA
#define GPI_USB_VBUS_PIN            GPIO_Pin_2
#define GPI_USB_VBUS_TYPE           GPIO_Mode_IN_FLOATING

#else  // if Rev C and above

#define GPI_USB_VBUS_PORT           GPIOA
#define GPI_USB_VBUS_CLK            RCC_APB2Periph_GPIOA
#define GPI_USB_VBUS_PIN            GPIO_Pin_9
#define GPI_USB_VBUS_TYPE           GPIO_Mode_IN_FLOATING



#endif // REV B

/*GPI AtoD input */	//osbsolete, defined in hardware.c
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

#ifndef BC_HW_REVC	//valid for REVB and REVJ+

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

#else  // if  BC_HW_REVC  Led 0 and 1 swapped from Ver B
//IRLED1 and 2 don't conform to schematic for REVH?

/* GPO_IRLED0 */
#define GPO_IRLED0_PORT             GPIOB
#define GPO_IRLED0_CLK              RCC_APB2Periph_GPIOE
#define GPO_IRLED0_PIN              GPIO_Pin_0
/* GPO_IRLED1 */
#define GPO_IRLED1_PORT             GPIOE
#define GPO_IRLED1_CLK              RCC_APB2Periph_GPIOB
#define GPO_IRLED1_PIN              GPIO_Pin_12

/* GPO_IRLED2 */
#define GPO_IRLED2_PORT             GPIOE
#define GPO_IRLED2_CLK              RCC_APB2Periph_GPIOE
#define GPO_IRLED2_PIN              GPIO_Pin_11

#endif

#ifndef BC_HW_REVJ
/* GPO_TP10 */	//labelled TP6 for REVJ
#define GPO_TP10_PORT               GPIOB
#define GPO_TP10_CLK                RCC_APB2Periph_GPIOB
#define GPO_TP10_PIN                GPIO_Pin_1

/* GPO_TP11 */	//labelled TP7 for REVJ
#define GPO_TP11_PORT               GPIOB
#define GPO_TP11_CLK                RCC_APB2Periph_GPIOB
#define GPO_TP11_PIN                GPIO_Pin_2

/* GPO_TP12 */	//labelled TP8 for REVJ
#define GPO_TP12_PORT               GPIOE
#define GPO_TP12_CLK                RCC_APB2Periph_GPIOE
#define GPO_TP12_PIN                GPIO_Pin_10

#else

/* GPO_TP6 */	//labelled TP6 for REVJ
#define GPO_TP6_PORT               GPIOB
#define GPO_TP6_CLK                RCC_APB2Periph_GPIOB
#define GPO_TP6_PIN                GPIO_Pin_1

/* GPO_TP7 */	//labelled TP7 for REVJ
#define GPO_TP7_PORT               GPIOB
#define GPO_TP7_CLK                RCC_APB2Periph_GPIOB
#define GPO_TP7_PIN                GPIO_Pin_2

/* GPO_TP10 */	//labelled TP8 for REVJ
#define GPO_TP10_PORT               GPIOE
#define GPO_TP10_CLK                RCC_APB2Periph_GPIOE
#define GPO_TP10_PIN                GPIO_Pin_10
#endif


/* Type C hardware additions */

#ifdef BC_HW_REVC
/* GPO_2520_RST */
#define GPO_2520_RST_PORT              GPIOB
#define GPO_2520_RST_CLK               RCC_APB2Periph_GPIOB
#define GPO_2520_RST_PIN               GPIO_Pin_5

#else //BC_HW_REVJ and above
#define GPO_2520_RST_PORT              GPIOD
#define GPO_2520_RST_CLK               RCC_APB2Periph_GPIOD
#define GPO_2520_RST_PIN               GPIO_Pin_9

#endif

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

#ifndef BC_HW_REVJ

/*GPO_IMU_RST */
#define GPO_IMU_RST_PORT              GPIOD
#define GPO_IMU_RST_CLK               RCC_APB2Periph_GPIOD
#define GPO_IMU_RST_PIN               GPIO_Pin_11

#endif

/* GPO_RF_HGM */
#define GPO_RF_HGM_PORT              GPIOD
#define GPO_RF_HGM_CLK               RCC_APB2Periph_GPIOD
#define GPO_RF_HGM_PIN               GPIO_Pin_4

#ifndef BC_HW_REVJ
/* GPO_TP_50 */	//labelled as TP5
#define GPO_TP50_PORT               GPIOA
#define GPO_TP50_CLK                RCC_APB2Periph_GPIOA
#define GPO_TP50_PIN                GPIO_Pin_7

#else 
/* GPO_TP_50 */	//labelled as TP5
#define GPO_TP5_PORT               GPIOA
#define GPO_TP5_CLK                RCC_APB2Periph_GPIOA
#define GPO_TP5_PIN                GPIO_Pin_7

#endif	

/* GPO_IMU_CLK */
#define GPO_IMU_CLK_PORT               GPIOA
#define GPO_IMU_CLK_CLK                RCC_APB2Periph_GPIOA
#define GPO_IMU_CLK_PIN                GPIO_Pin_8

/*GPO USB */
#define GPO_USB_VBUS_PORT           GPIOA
#define GPO_USB_VBUS_CLK            RCC_APB2Periph_GPIOA
#define GPO_USB_VBUS_PIN            GPIO_Pin_9

#ifdef BC_HW_REVJ
/*GPO_IMU_FSYNC*/
#define GPO_IMU_FSYNC_PORT           GPIOD
#define GPO_IMU_FSYNC_CLK            RCC_APB2Periph_GPIOD
#define GPO_IMU_FSYNC_PIN            GPIO_Pin_12

/*GPO_INK_RST*/
#define GPO_INK_RST_PORT           GPIOD
#define GPO_INK_RST_CLK            RCC_APB2Periph_GPIOA
#define GPO_INK_RST_PIN            GPIO_Pin_14

#endif
/************************************************/
// ST IMU defines...
// ST DEVICE  CONNECTOR Beacon processor
//  Int2_A    pin1    TP16
//  CS_G      pin2    PD10
//  SPC       Pin3    PB13 (IMU_SCLK)
//  SDO       pin4    PB14 (IMU_MISO)
//  SDI       pin5    PB15 (IMU_MOSI)
//  CS_A      pin6    PB12 (IMU_SS)
//  INT1_A    pin7    PD8  (IMU_DIO1)
//  Enable    pin8    MU_RST
//  INT1_G    pin9    PD9  (IMU_DIO_2)
//  +5Vo      pin10   +5V
//  +5Vo      pin11   +5V
//  +5Vo      pin12   +5V
//  0V0       pin13   0v
//  0vo       pin14   0V
//  0vo       pin15   0V
//  DEN_G     pin16   NC
//  N/C       pin17   NC
//  N/C       pin18   NC
//   N/C      pin19   NC
//   INT2_G   pin20   NC
//
//
/**************************************************/

/*******/
/* LED */
/*******/
#ifdef BC_HW_REVB

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


#elif defined BC_HW_REVC  // post Rev B hardware

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



#endif	//REVJ does not contain LEDs.

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

#ifndef BC_HW_REVJ	//versions prior to rev J
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
#define SPI_IMU_A_SS_PORT           GPIOB
#define SPI_IMU_A_SS_CLK            RCC_APB2Periph_GPIOB
#define SPI_IMU_A_SS_PIN            GPIO_Pin_12
#define SPI_IMU_G_SS_PORT           GPIOD
#define SPI_IMU_G_SS_CLK            RCC_APB2Periph_GPIOD
#define SPI_IMU_G_SS_PIN            GPIO_Pin_10

/* SPI_IMU DMA */
#define SPI_IMU_DMA                 DMA1
#define SPI_IMU_DMA_CLK             RCC_AHBPeriph_DMA1
#define SPI_IMU_RX_DMA_CHAN         DMA1_Channel4
#define SPI_IMU_RX_DMA_FLAG         DMA1_FLAG_TC4
#define SPI_IMU_RX_DMA_IRQ          DMA1_Channel4_IRQn
#define SPI_IMU_TX_DMA_CHAN         DMA1_Channel5
#define SPI_IMU_TX_DMA_FLAG         DMA1_FLAG_TC5

#else	//REVJ BEACON

/* SPI_RADIO_IMU */
#define SPI_RADIO_IMU_SPI               SPI3
#define SPI_RADIO_IMU_SPI_CLK           RCC_APB1Periph_SPI3
#define SPI_RADIO_IMU_REMAP             GPIO_Remap_SPI3
#define SPI_RADIO_IMU_PORT              GPIOC
#define SPI_RADIO_IMU_CLK               RCC_APB2Periph_GPIOC
#define SPI_RADIO_IMU_PIN_SCK           GPIO_Pin_10
#define SPI_RADIO_IMU_PIN_MOSI          GPIO_Pin_12
#define SPI_RADIO_IMU_PIN_MISO          GPIO_Pin_11
#define SPI_RADIO_SS_PORT           	GPIOA
#define SPI_RADIO_SS_CLK           		RCC_APB2Periph_GPIOA
#define SPI_RADIO_SS_PIN           		GPIO_Pin_4
#define SPI_IMU_SS_PORT					GPIOD
#define SPI_IMU_SS_CLK					RCC_APB2Periph_GPIOD
#define SPI_IMU_SS_PIN					GPIO_Pin_10

#define SPI_RADIO_IMU_IRQn              SPI3_IRQn

/* SPI_INK */
#define SPI_INK_SPI                 SPI2
#define SPI_INK_SPI_CLK             RCC_APB1Periph_SPI2
#define SPI_INK_REMAP               0
#define SPI_INK_PORT                GPIOB
#define SPI_INK_CLK                 RCC_APB2Periph_GPIOB
#define SPI_INK_PIN_SCK             GPIO_Pin_13
#define SPI_INK_PIN_MOSI            GPIO_Pin_15
#define SPI_INK_PIN_MISO            GPIO_Pin_14
#define SPI_INK_SS_PORT           	GPIOB
#define SPI_INK_SS_CLK            	RCC_APB2Periph_GPIOB
#define SPI_INK_SS_PIN            	GPIO_Pin_12

/* SPI_INK DMA */ //previous revC DMA line. Useful when implementing INK screen
#define SPI_INK_DMA                 DMA1
#define SPI_INK_DMA_CLK             RCC_AHBPeriph_DMA1
#define SPI_INK_RX_DMA_CHAN         DMA1_Channel4
#define SPI_INK_RX_DMA_FLAG         DMA1_FLAG_TC4
#define SPI_INK_RX_DMA_IRQ          DMA1_Channel4_IRQn
#define SPI_INK_TX_DMA_CHAN         DMA1_Channel5
#define SPI_INK_TX_DMA_FLAG         DMA1_FLAG_TC5

#endif

/******************************************************************************/

#define INCLUDE_PA

#define GPIO_0_MASK                 0x0004
#define GPIO_1_MASK                 0x0002
#define GPIO_2_MASK                 0x0001

/* EXPORTED FUNCTIONS --------------------------------------------------------*/

void            HwGPIInit(HwGPI_TypeDef GPI);
uint32_t        HwGPIState(HwGPI_TypeDef GPI);

void            HwGPOInit(HwGPO_TypeDef GPO);
void            HwGPOInitOC(HwGPO_TypeDef GPO);
void            HwGPOLow(HwGPO_TypeDef GPO);
void            HwGPOHigh(HwGPO_TypeDef GPO);
void            HwGPOToggle(HwGPO_TypeDef GPO);
#if 0
void            HwLEDInit(HwLED_TypeDef Led);
void            HwLEDOn(HwLED_TypeDef Led);
void            HwLEDOff(HwLED_TypeDef Led);
void            HwLEDToggle(HwLED_TypeDef Led);
#endif
void            HwButtonInit(HwButton_TypeDef Button);
uint32_t        HwButtonPressed(HwButton_TypeDef Button);

void            HwCOMInit(HwCOM_TypeDef COM, USART_InitTypeDef *USART_InitStruct);

void            HwSPIInit(HwSPI_TypeDef SPI, SPI_InitTypeDef *SPI_InitStruct);
void            HwSPISSAssert(HwSPI_TypeDef SPI);
void            HwSPISSDeAssert(HwSPI_TypeDef SPI);
uint32_t 		HwGetSPISS(HwSPI_TypeDef SPI);
void            HwWait(unsigned int ticks);
void            HwTimerSet(unsigned int ticks);
unsigned int    HwTimerExpired(void);

void            HwPeriphInit(void);
void            HwRadioEXTIInit(void);
#ifdef BC_HW_REVJ
void 			HwIMUEXTIInit(void);
#endif

void            GetARM_UUID(void);
void            WDTimerInit(void);
void            HwTIM1Init(void);

void            HwI2CInit(void);

//size_t          __write(int handle, const unsigned char *buffer, size_t size);
//size_t          __read(int handle, unsigned char *buffer, size_t size);

int TRACE(char* fmt, ...);

/* EXTERNAL VARIABLES --------------------------------------------------------*/
extern volatile ImuBuffer_t ImuGyroBuffer, ImuAccelBuffer;
extern volatile uint8_t spiIMURxBufDma[];
extern volatile uint16_t spiIMURxBuf[];
extern volatile uint16_t Enables;

extern int16_t whole_time_adjust;
extern int8_t part_time_adjust;

#define TIM_AUTORELOAD          59999
#define TIM3_AUTORELOAD         29999

#endif /*__HARDWARE_H*/
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/