/******************** (C) COPYRIGHT 2011 NaturalPoint, Inc. ********************
* File Name          : main.c
* Author             : ?
* Version            : V1.0
* Date               : 6/2/2011
* Description        : Main Application for BlackTrax:Beacon
*******************************************************************************/

/*
    LED1 : Toggle on RF Packet TX
    LED2 : Toggle on 50x RF Packet TX'es
    LED3 : Toggle on RF Packet RX
    LED4 : Toggle on 50x RF Packet RX'es
*/

/* INCLUDES ------------------------------------------------------------------*/

#include "hardware.h"
#include "CoOS.h"
#include "tasks.h"
#include "basic_rf.h"
#include "tk_version.h"
#ifndef STDIO_TO_USART
#include "usb_lib.h"
#include "usb_pwr.h"
#include "usb_hw_config.h"
#endif
#include <stdio.h>
#include <string.h>

/* PRIVATE TYPEDEF -----------------------------------------------------------*/

/* PRIVATE DEFINES -----------------------------------------------------------*/

#define FIRMWARE_VERSION "0.6"

#define OS_STACK_SIZE   128

/* PRIVATE MACROS ------------------------------------------------------------*/

/* EXTERN VARIABLES ----------------------------------------------------------*/
extern void WriteConfig(void);
/* PRIVATE VARIABLES ---------------------------------------------------------*/

static OS_STK task1Stack[OS_STACK_SIZE];
static OS_STK task2Stack[OS_STACK_SIZE];
static OS_STK task3Stack[OS_STACK_SIZE];
static OS_STK task4Stack[OS_STACK_SIZE];
static OS_STK task5Stack[OS_STACK_SIZE];
static OS_STK task6Stack[OS_STACK_SIZE];
static OS_STK task8Stack[OS_STACK_SIZE];

/* PUBLIC VARIABLES ----------------------------------------------------------*/

const char firmwareVersion[] = FIRMWARE_VERSION;

OS_TID task1Id;
OS_TID task2Id;
OS_TID task5Id;
OS_TID task8Id;

OS_FlagID flagIMUNewData = 0xFF;
OS_FlagID flagRadioTxReq = 0xFF;

uint8_t* const led0Id = &config.led0Id;
uint8_t* const led1Id = &config.led1Id;
uint8_t* const led2Id = &config.led2Id;

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/

//static void OsStackPaint(OS_STK *stk, unsigned short size);
//static unsigned short OsStackCheck(OS_STK *stk);
static uint16_t CheckConfigChecksum(void);


/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/

#if 0
/*******************************************************************************
* Description    :
* Input          :
* Return         :
*******************************************************************************/
static void OsStackPaint(OS_STK *stk, unsigned short size) {
    while (size--) {
        *stk++ = 0xABCD;
    }
}

/*******************************************************************************
* Description    :
* Input          :
* Return         :
*******************************************************************************/
static unsigned short OsStackCheck(OS_STK *stk) {
    unsigned short clean_count = 0;
    OS_STK *tmp = stk;

    while (*stk++ == 0xABCD) {
        clean_count++;
    }
    for (unsigned int i = 0; i < OS_STACK_SIZE; i++) {
        if (*tmp++ == 0xABCD)
            printf(".");
        else
            pritnf("!");
    }
    return clean_count;
}
#endif

/*******************************************************************************
* Description : Check FLASH Configuration Struct Checksum
* Input       :
* Return      : Checksum if GOOD, 0 if BAD
*******************************************************************************/
static uint16_t CheckConfigChecksum(void) {
    uint16_t checksum = 0xCCCC; // checksum initialization
    uint16_t *pConfig = (uint16_t*)FLASH_CONFIG_PAGE;

    // checksum is last half-word in struct
    // checksum field not included in checksum
    for (uint16_t i = 0; i < (sizeof(config)/2 - 1); i++) {
        checksum ^= (i ^ *pConfig++);
    }

    if (checksum == *pConfig) {
        return checksum;
    } else {
        return 0;
    }
}

/* PUBLIC FUNCTION PROTOTYPES ------------------------------------------------*/

/*******************************************************************************
* Description    : Main routine.
* Input          : -
* Return         : -
*******************************************************************************/
void main(void) {

    SysTick_Config(SystemCoreClock / 100); // SysTick Interrupt Freq = (1/100)s = 10ms

    HwPeriphInit();
    
    GetARM_UUID();
    DAC_SetChannel2Data(DAC_Align_12b_R, config.ledDAC);

#ifndef STDIO_TO_USART
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Timer_Config();
    USB_Init();
#endif




    CoInitOS();

    RadioInit(config.panId, config.mySrcAddr, config.rfChan);

    flagIMUNewData = CoCreateFlag(1, 0);    // auto-reset, flag cleared
    flagRadioTxReq = CoCreateFlag(1, 0);    // auto-reset, flag cleared

#if 0
    OsStackPaint(task1Stack, OS_STACK_SIZE);
    OsStackPaint(task2Stack, OS_STACK_SIZE);
    OsStackPaint(task3Stack, OS_STACK_SIZE);
    OsStackPaint(task4Stack, OS_STACK_SIZE);
    OsStackPaint(task5Stack, OS_STACK_SIZE);
    OsStackPaint(task6Stack, OS_STACK_SIZE);
#endif

    // Task1: Process and Send each new IMU data sample
    task1Id = CoCreateTaskEx(Task1, (void*)0, 0, &task1Stack[OS_STACK_SIZE-1], OS_STACK_SIZE, (uint32_t)0, (uint32_t)1);
    // Task2: Process Incoming Radio Packets
    task2Id = CoCreateTaskEx(Task2, (void*)0, 1, &task2Stack[OS_STACK_SIZE-1], OS_STACK_SIZE, (uint32_t)0, (uint32_t)1);
    // Task3: RF Chan Scan, then Monitor GPO_PWRON Power Switch
              CoCreateTask  (Task3, (void*)0, 0, &task3Stack[OS_STACK_SIZE-1], OS_STACK_SIZE);
    // Task4: Implements Simple Text Command Console
              CoCreateTask  (Task4, (void*)0, 9, &task4Stack[OS_STACK_SIZE-1], OS_STACK_SIZE);
    // Task5: Pinger
    task5Id = CoCreateTaskEx(Task5, (void*)0, 9, &task5Stack[OS_STACK_SIZE-1], OS_STACK_SIZE, (uint32_t)0, (uint32_t)1);
    // Task6: Manage Outgoing (TX) Packet Queue
              CoCreateTask  (Task6, (void*)0, 2, &task6Stack[OS_STACK_SIZE-1], OS_STACK_SIZE);

    // A to D battery monitor// 
     task8Id = CoCreateTaskEx(Task8, (void*)0, 7, &task8Stack[OS_STACK_SIZE-1], OS_STACK_SIZE,(uint32_t)0, (uint32_t)0);
    
    
    CoStartOS();

    while (1);
}

#if (defined USE_MY_ASSERT) || (defined USE_FULL_ASSERT)
/*******************************************************************************
* Description    : Reports the name of the source file and the source line number
*                  where the assert error has occurred.
* Input          :
* Return         : -
*******************************************************************************/
void assert_failed(uint8_t *file, uint32_t line) {
    while (1) {
        printf("!!!ASSERT FAILED!!! %s : %d\n\r", file, line);
        HwLEDOn(LED1); HwLEDOn(LED4); HwLEDOff(LED2); HwLEDOff(LED3);
        HwWait(50);
        HwLEDOn(LED2); HwLEDOn(LED3); HwLEDOff(LED1); HwLEDOff(LED4);
        HwWait(50);
    }
}
#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/