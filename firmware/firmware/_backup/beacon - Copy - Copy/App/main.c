/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
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
#ifndef STDIO_TO_USART
#include "usb_regs.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#endif
#include <stdio.h>
#include <string.h>
#include "stm32f10x_dac.h"
#include "stm32f10x_rcc.h"

#include "i2c_ee.h"
#include "flash_map.h"
#include "config.h"

/* PRIVATE TYPEDEF -----------------------------------------------------------*/

/* PRIVATE DEFINES -----------------------------------------------------------*/

#define FIRMWARE_VERSION "0.6"

#define OS_STACK_SIZE   256

/* PRIVATE MACROS ------------------------------------------------------------*/

/* EXTERN VARIABLES ----------------------------------------------------------*/
extern uint32_t __vector_table;
extern uint8_t bat_slot_numbers;

/* PRIVATE VARIABLES ---------------------------------------------------------*/

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;

static OS_STK task1Stack[OS_STACK_SIZE];
static OS_STK taskRadioRxStack[OS_STACK_SIZE];
static OS_STK task3Stack[OS_STACK_SIZE];
static OS_STK task4Stack[OS_STACK_SIZE];
static OS_STK taskRadioTxStack[OS_STACK_SIZE];
static OS_STK task8Stack[OS_STACK_SIZE];
static OS_STK taskIMU_A_Stack[OS_STACK_SIZE];
static OS_STK taskIMU_G_Stack[OS_STACK_SIZE];
/* PUBLIC VARIABLES ----------------------------------------------------------*/

#ifdef WDT_ENABLE
uint8_t  watchdog_active = 1;
#else
uint8_t  watchdog_active = 0;
#endif

uint8_t asserted;
const char firmwareVersion[] = FIRMWARE_VERSION;

OS_TID  task1Id;
OS_TID  taskRadioRxId;
OS_TID  task3Id;
OS_TID  taskConfigId;
OS_TID  taskRadioTxId;
OS_TID  task8Id;
OS_TID  taskIMUGId;

OS_FlagID   flagIMUNewData = 0xFF;
OS_FlagID   flagIMU_G_DRDY = 0xFF;
OS_FlagID   flagRadioTxReq = 0xFF;
OS_FlagID   flagIMUTimeToSend = 0xFF;
OS_FlagID   flagRadioCCA = 0xFF;

OS_FlagID   flagIMUDataReady = 0xFF;
OS_FlagID   flagBtnDataReady = 0xFF;
OS_FlagID   flagBatDataReady = 0xFF;
OS_FlagID   flagLEDDataReady = 0xFF;

OS_EventID semRFRxFrames;   // Number of RF frames received in Rx-FIFO. It is notified by RX_FRM_DONE interrupt

uint8_t* const led0Id = &config.led0Id;
uint8_t* const led1Id = &config.led1Id;
uint8_t* const led2Id = &config.led2Id;


volatile uint16_t tasksWDT = 0;
/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/

//static void OsStackPaint(OS_STK *stk, unsigned short size);
//static unsigned short OsStackCheck(OS_STK *stk);
/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/


/*******************************************************************************
* Description : Check FLASH Configuration Struct Checksum
* Input       :
* Return      : Checksum if GOOD, 0 if BAD
*******************************************************************************/
uint16_t CalcConfigChecksum(uint16_t* pConfig, uint16_t len) {
    uint16_t checksum = 0xCCCC; // checksum initialization

    // checksum is last half-word in struct
    // checksum field not included in checksum
    for (uint16_t i = 0; i < len/2; i++) {
        checksum ^= (i ^ *pConfig++);
    }

    return checksum;
}

/* PUBLIC FUNCTION PROTOTYPES ------------------------------------------------*/
//volatile       uint8_t buf[64/*sizeof(config_t)*/];
uint8_t loop = 1;

/*******************************************************************************
* Description    : Main routine.
* Input          : -
* Return         : -
*******************************************************************************/
void  main(void) {

#ifdef BOOTLOADER
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0xC000);
#endif

    SysTick_Config(SystemCoreClock / 1000); // SysTick Interrupt Freq = (1/1000)s = 1ms

#ifdef CIRCULAR_LOG
    memset((void*) log, 0, LOG_SIZE);
#endif

      HwI2CInit();

      if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET) {
        RCC_ClearFlag();
      }
      // Read EEPROM_BEACON_FLAG_ADDRESS - if it is not 0xFF, write 0xFF - this flag is used by
      // bootloader - when it is 0xFE, bootloader stays in bootloader when connected by USB
      // used by "Jump to bootloader" CSM command
      uint8_t jumpToMain;
      I2C_EE_BufferRead((uint8_t*) &jumpToMain, EEPROM_BEACON_FLAG_ADDRESS, 1);
      if (jumpToMain != 0xFE) {
        jumpToMain = 0xFE;
        I2C_EE_BufferWrite((uint8_t*) &jumpToMain, EEPROM_BEACON_FLAG_ADDRESS, 1);
      }

         LoadConfig(&config);

    whole_time_adjust = config.time_adjust/10;
    part_time_adjust = config.time_adjust%10;

    if (config.frameCountNoSync == 0xFFFF) {
      config.frameCountNoSync = DEFAULT_FRAME_COUNT_NO_SYNC;
    }

    CoInitOS();

    __enable_interrupt();

    semRFRxFrames = CoCreateSem (0, 127, EVENT_SORT_TYPE_FIFO);   //Number of RF frames received in Rx-FIFO. It is notified by RX_FRM_DONE interrupt
    assert(semRFRxFrames != E_CREATE_FAIL);

    HwPeriphInit();
    //HwLEDOff(LED2);
    DAC_SetChannel2Data(DAC_Align_12b_R, config.ledDAC);

    GetARM_UUID();

    RELOAD_WATCHDOG
#ifndef STDIO_TO_USART
      USBD_Init(&USB_OTG_dev,
            USB_OTG_FS_CORE_ID,
            &USR_desc,
            &USBD_CDC_cb,
            &USR_cb);

    RELOAD_WATCHDOG
#endif


#ifndef STM3210C_EVAL
    RadioInit(config.panId, config.mySrcAddr, config.rfChan, config.TestMode);
#endif
#ifdef BC_HW_REVJ
	IMUInit();
#endif

    flagIMUNewData = 	CoCreateFlag(0, 0);    	// Manual-reset, flag cleared
    flagIMU_G_DRDY = 	CoCreateFlag(1, 0);    	// auto-reset, flag cleared
    flagRadioTxReq = 	CoCreateFlag(1, 0);    	// auto-reset, flag cleared
    flagIMUTimeToSend = CoCreateFlag(1, 0); 	// auto-reset, flag cleared
    flagRadioCCA = 		CoCreateFlag(1, 0);

    flagIMUDataReady    = CoCreateFlag(0, 0); 	//auto-reset, flag clear
    flagBtnDataReady    = CoCreateFlag(0, 0);   //auto-reset, flag clear
    flagBatDataReady    = CoCreateFlag(0, 0);   //auto-reset, flag clear
    flagLEDDataReady    = CoCreateFlag(0, 0);   //auto-reset, flag clear

    // Task1: Process and Send each new IMU data sample
    task1Id = CoCreateTaskEx(Task1, (void*)0, 1, &task1Stack[OS_STACK_SIZE-1], OS_STACK_SIZE, (uint32_t)0, (uint32_t)1);

#ifndef STM3210C_EVAL
    // TaskRadioRx: Process Incoming Radio Packets
    taskRadioRxId 	= CoCreateTaskEx(TaskRadioRx, (void*)0, 2, &taskRadioRxStack[OS_STACK_SIZE-1], OS_STACK_SIZE, (uint32_t)0, (uint32_t)1);
    // Task3: RF Chan Scan, then Monitor GPO_PWRON Power Switch
    task3Id 		= CoCreateTask  (Task3, (void*)0, 0, &task3Stack[OS_STACK_SIZE-1], OS_STACK_SIZE);
#endif
    // TaskConfig: Implements Simple Text Command Console
    taskConfigId 	= CoCreateTaskEx(TaskConfig, (void*)0, 9, &task4Stack[OS_STACK_SIZE-1], OS_STACK_SIZE,(uint32_t)0, (uint32_t)0);
    // TaskRadioTx: Manage Outgoing (TX) Packet Queue. Highest priority
    taskRadioTxId 	= CoCreateTask  (TaskRadioTx, (void*)0, 0, &taskRadioTxStack[OS_STACK_SIZE-1], OS_STACK_SIZE);

    // A to D battery monitor//
    task8Id 		= CoCreateTask  (Task8, (void*)0, 7, &task8Stack[OS_STACK_SIZE-1], OS_STACK_SIZE);

    // IMU Data Read task
	//  CoCreateTask(TaskIMU_A, (void*)0, 3, &taskIMU_A_Stack[OS_STACK_SIZE-1], OS_STACK_SIZE);
    taskIMUGId = CoCreateTaskEx(TaskIMU_G, (void*)0, 1/*3*/, &taskIMU_G_Stack[OS_STACK_SIZE-1], OS_STACK_SIZE, (uint32_t)0, (uint32_t)1);

#ifdef BC_REV_J

    WDTimerInit();    // Init th WD timer

    CoStartOS();

    while (1);
}

#if (defined USE_MY_ASSERT) || (defined USE_FULL_ASSERT)
uint8_t assert_loop = 1;
/*******************************************************************************
* Description    : Reports the name of the source file and the source line number
*                  where the assert error has occurred.
* Input          :
* Return         : -
*******************************************************************************/
void assert_failed(uint8_t *file, uint32_t line) {
    asserted = 1;
    while (assert_loop)
    {
        RELOAD_WATCHDOG
        TRACE("!ASSERT FAILED! @%d:%s\n\r", line, file);
        TRACE("  %s\n\r", file);
        TRACE("  %d\n\r", line);
        //HwLEDOn(LED1); HwLEDOn(LED4); HwLEDOff(LED2); HwLEDOff(LED3);
        HwWait(5000);
        //HwLEDOn(LED2); HwLEDOn(LED3); HwLEDOff(LED1); HwLEDOff(LED4);
        HwWait(5000);
    }
    assert_loop = 1;
    asserted = 0;
    return;
}
#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/