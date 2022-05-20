/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
* File Name          : stm32f10x_it.c
* Author             : ???
* Version            : V1.0
* Date               : 6/2/2011
* Description        : ???
*******************************************************************************/
/*
    NVIC Priority (Highest first):
        - OTG_FS_IRQ
        - TIM3_IRQn
        - EXTI2_IRQn (GPI_RADIO_GPIO0_IRQn)
        - SPI3_IRQn (SPI_RADIO_IRQn)
        - EXTI1_IRQn (GPI_RADIO_GPIO1_IRQn)
        - DMA1_Channel4_IRQn (COM2_DMA_IRQn)
*/

/* INCLUDES ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_dma.h"
#include "stm32_eth.h"
#include "main.h"

#include "usb_istr.h"
#include "hardware.h"
#include "basic_rf.h"
#include "radio_defs.h"

#include "CoOS.h"

#include <stdio.h>
#include "usb_core.h"
#include "clock-arch.h"
#include "i2c_ee.h"

/* PRIVATE TYPEDEF -----------------------------------------------------------*/

/* PRIVATE DEFINES -----------------------------------------------------------*/

/* PRIVATE MACROS ------------------------------------------------------------*/

#define ENTER_ISR() CoEnterISR()
#define EXIT_ISR() CoExitISR()
#define ISR_SETFLAG(x) isr_SetFlag(x);

/* PRIVATE VARIABLES ---------------------------------------------------------*/

/* PUBLIC VARIABLES ----------------------------------------------------------*/

#ifndef COOS
volatile unsigned int SysTickCounter = 0;
#endif

uint32_t hf_counter;
uint32_t TxSendError =0;
uint32_t sec;

extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
extern OS_FlagID flagEtherTxDone;        // emitted by Ethernet TX interrupt
extern uint8_t rxCount;

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/

/* PRIVATE FUNCTIONS ---------------------------------------------------------*/

/* PUBLIC FUNCTIONS ----------------------------------------------------------*/

/*******************************************************************************
* Description : This function handles NMI exception.
* Input       : -
* Return      : -
*******************************************************************************/
void NMI_Handler(void)
{
}


void hard_fault_handler_c(unsigned int * hardfault_args)
{
	unsigned int stacked_r0;
	unsigned int stacked_r1;
	unsigned int stacked_r2;
	unsigned int stacked_r3;
	unsigned int stacked_r12;
	unsigned int stacked_lr;
	unsigned int stacked_pc;
	unsigned int stacked_psr;

	stacked_r0 = ((unsigned long) hardfault_args[0]);
	stacked_r1 = ((unsigned long) hardfault_args[1]);
	stacked_r2 = ((unsigned long) hardfault_args[2]);
	stacked_r3 = ((unsigned long) hardfault_args[3]);

	stacked_r12 = ((unsigned long) hardfault_args[4]);
	stacked_lr = ((unsigned long) hardfault_args[5]);
	stacked_pc = ((unsigned long) hardfault_args[6]);
	stacked_psr = ((unsigned long) hardfault_args[7]);

  I2C_EE_BufferWrite((uint8_t*) &stacked_lr, EEPROM_DEBUG_STACKED_LR, EEPROM_DEBUG_STACKED_LR_SIZE);
  I2C_EE_BufferWrite((uint8_t*) &stacked_pc, EEPROM_DEBUG_STACKED_PC, EEPROM_DEBUG_STACKED_PC_SIZE);
  I2C_EE_BufferWrite((uint8_t*) &stacked_psr, EEPROM_DEBUG_STACKED_PSR, EEPROM_DEBUG_STACKED_PSR_SIZE);

  //I2C_EE_BufferRead((uint8_t*) &hf_counter, EEPROM_DEBUG_COUNTER, EEPROM_DEBUG_COUNTER_SIZE);
  //hf_counter++;
  I2C_EE_BufferWrite((uint8_t*) &hf_counter, EEPROM_DEBUG_COUNTER, EEPROM_DEBUG_COUNTER_SIZE);
  uint8_t hf_flags = 0;
  I2C_EE_BufferRead((uint8_t*) &hf_flags, EEPROM_DEBUG_DIAGNOSTIC+54, 1);
  hf_flags &= ~DIAGNOSTICS_NOTFLAG_HARDFAULT;
  hf_flags |= DIAGNOSTICS_LASTEVENT_HARDFAULT;
  I2C_EE_BufferWrite((uint8_t*) &hf_flags, EEPROM_DEBUG_DIAGNOSTIC+54, 1);



	TRACE ("[Hard fault handler]\n");
	TRACE ("R0 = %x\n", stacked_r0);
	TRACE ("R1 = %x\n", stacked_r1);
	TRACE ("R2 = %x\n", stacked_r2);
	TRACE ("R3 = %x\n", stacked_r3);
	TRACE ("R12 = %x\n", stacked_r12);
	TRACE ("LR = %x\n", stacked_lr);
	TRACE ("PC = %x\n", stacked_pc);
	TRACE ("PSR = %x\n", stacked_psr);
	TRACE ("BFAR = %x\n", (*((volatile unsigned long *)(0xE000ED38))));
	TRACE ("CFSR = %x\n", (*((volatile unsigned long *)(0xE000ED28))));
	TRACE ("HFSR = %x\n", (*((volatile unsigned long *)(0xE000ED2C))));
	TRACE ("DFSR = %x\n", (*((volatile unsigned long *)(0xE000ED30))));
	TRACE ("AFSR = %x\n", (*((volatile unsigned long *)(0xE000ED3C))));

        while (1);
	return;
}

/*******************************************************************************
* Description : This function handles Hard Fault exception.
* Input       : -
* Return      : -
*******************************************************************************/
void HardFault_Handler(void)
{
    // Go to infinite loop when Hard Fault exception occurs
  	asm("TST LR, #4");
  	asm("ITE EQ");
  	asm("MRSEQ R0, MSP");
  	asm("MRSNE R0, PSP");
  	asm("B hard_fault_handler_c");

	while (1);

}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
/*******************************************************************************
* Description : This function handles NMI exception.
* Input       : -
* Return      : -
*******************************************************************************/
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1);
}

/*******************************************************************************
* Description : This function handles Bus Fault exception.
* Input       : -
* Return      : -
*******************************************************************************/
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1);
}

/*******************************************************************************
* Description : This function handles Usage Fault exception.
* Input       : -
* Return      : -
*******************************************************************************/
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1);
}

/*******************************************************************************
* Description : This function handles SVCall exception.
* Input       : -
* Return      : -
*******************************************************************************/
void SVC_Handler(void)
{
}

/*******************************************************************************
* Description : This function handles Debug Monitor exception.
* Input       : -
* Return      : -
*******************************************************************************/
void DebugMon_Handler(void)
{
}

#ifndef COOS
/*******************************************************************************
* Description : This function handles PendSVC exception.
* Input       : -
* Return      : -
*******************************************************************************/
void PendSV_Handler(void)
{
}

/*******************************************************************************
* Description : This function handles SysTick Handler.
* Input       : -
* Return      : -
*******************************************************************************/
void SysTick_Handler(void)
{
    SysTickCounter++;
}
#endif


/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/*******************************************************************************
* Description : This function handles TIM1 global interrupt request.
*               TIM1: 95Hz IMU Sampling
* Priority    : 1
* Input       : -
* Return      : -
*******************************************************************************/
void TIM1_UP_IRQHandler(void) {
//    ENTER_ISR();

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {

    // Clear TIM1 Capture compare interrupt pending bit
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    assert(TIM1->RCR == 0);
    sec++;
//    Tim1Handler();
    // Get IMU sample
    // PDE here is where we trigger the new IMU data is ready to be processed
    // and then transmitted
    //ISR_SETFLAG(flagIMUTimeToSend);
    }

  //  EXIT_ISR();
}


/*******************************************************************************
* Description : This function handles USB-On-The-Go FS global interrupt request.
* Input       : -
* Return      : -
*******************************************************************************/
#ifndef STDIO_TO_USART
void OTG_FS_IRQHandler(void) {
      func = __func__;

  USBD_OTG_ISR_Handler (&USB_OTG_dev);
}
#endif

/*******************************************************************************
* Description : This function handles External line 2 interrupt request.
*               GPI_RADIO_GPIO0 (RX_FRM_DONE) = PD.2
* Input       : -
* Return      : -
*******************************************************************************/
void EXTI2_IRQHandler(void) {
    ENTER_ISR();

        func = __func__;

    diagnostic.setSemRFRxFrames = isr_PostSem(semRFRxFrames);
    // assert(statusType == E_OK); // happen when disconnecting ST-Link in debug mode

    // Clear the  EXTI pending bit
    EXTI_ClearITPendingBit(EXTI_Line2);

    EXIT_ISR();
}

realTime rxFIFOTime;
uint16_t spiLine;
uint32_t spiCount;
const char *spi_func;
uint16_t old_spi_line;
//StatusType setFlagSPIDone1;
//StatusType setFlagSPIDone2;
//StatusType setFlagSPIDone3;

#ifdef TASKS_PROFILE
extern uint32_t globl;
uint32_t glb1, glb2, glb3;
#endif

/*******************************************************************************
* Description : This function handles SPI3 (SPI_RADIO_IRQn) global interrupt request.
* Input       : -
* Return      : -
*******************************************************************************/
void SPI3_IRQHandler(void) {
    ENTER_ISR();
#ifdef TASKS_PROFILE
    old_spi_line = CoGetTaskLine(CoGetCurrentTaskId());
#endif
    spi_func = func;
    func = __func__;

    if (SPI_I2S_GetFlagStatus(SPI_RADIO_SPI, SPI_I2S_FLAG_RXNE) == RESET) {
        EXIT_ISR();
        spiLine = line = __LINE__;
        return;
    }
    spiCount++;
    if (spiTxRxByteCount) {     // just finish up a SPI byte I/O
        spiTxRxByteCount--;
        *pSpiRxBuf = SPI_I2S_ReceiveData(SPI_RADIO_SPI);
        spiLine = line = __LINE__;

        if (spiTxRxByteCount == 0) {  // at last byte. Roll the state machine
          spiLine = line = __LINE__;
            if (spiTxRxByteState == RF_SPI_RX1_UPLOADCMD_STATE) {
            /*    if ((*pSpiRxBuf < 2) || (*pSpiRxBuf > 127)) {   // Frame Length INVALID! RX-FIFO's format: Length byte + MPDU
                    TRACE("Incorrect frame length %d\n\r", (int)(*pSpiRxBuf));
                    HwSPISSDeAssert(SPI_RADIO);
                    // TODO!!! verify TxBuf'space
                    uint8_t* temp = pSpiTxBuf;
                    *temp = CC2520_INS_SFLUSHRX;
                    temp++;
                    *temp = CC2520_INS_SFLUSHRX;    // double flush [CC2520 Bug#1]
                    spiTxRxByteCount = 0x02;        // above 2 instructions

                    spiTxRxByteState = RF_SPI_UPLOAD_ONLY_STATE;           // Command(s) are sent but does not care status return from CC2520
                    HwSPISSAssert(SPI_RADIO);                   // Low down to send
                    SPI_I2S_SendData(SPI_RADIO_SPI, *(pSpiTxBuf++));    // send the first byte, others must wait for the first byte finished (Enter this routine again when finished)
                }
                else {
                    spiTxRxByteState = RF_SPI_RX2_DOWNLOAD_HEADER_STATE;  // start receiving frame
                    spiTxRxByteCount = *(pSpiRxBuf++);    // PHR: Frame Length
                    SPI_I2S_SendData(SPI_RADIO_SPI, *(pSpiTxBuf++));
                }*/
                  spiLine = line = __LINE__;
                if (rxCount) {
                     spiLine = line = __LINE__;

                    spiTxRxByteState = RF_SPI_RX2_DOWNLOAD_HEADER_STATE;  // start receiving frame
                    spiTxRxByteCount = rxCount; //-1;
                      (pSpiRxBuf++);    // PHR: Frame Length
                   // assert(spiTxRxByteCount <= rxCount);
                    if (spiTxRxByteCount) {
                     // *(pSpiTxBuf++) = SPI_I2S_ReceiveData(SPI_RADIO_SPI);
                     // spiTxRxByteCount--;
                      spiTxRxByteCount++;
                      spiLine = line = __LINE__;
                      SPI_I2S_SendData(SPI_RADIO_SPI, *(pSpiTxBuf));
                    }
              } else {
                //ignore it, no data available
                spiLine = line = __LINE__;
                spiTxRxByteState = RF_SPI_INIT_STATE;
                SPI_I2S_ITConfig(SPI_RADIO_SPI, SPI_I2S_IT_RXNE, DISABLE);
                diagnostic.setFlagSPIDone1 = ISR_SETFLAG(flagSPIMachineDone);
                spiLine = line = __LINE__;
                glb1 = globl;
                if (rxCount) {
                        // To flush overflown buffer
                        spiLine = line = __LINE__;
                        rxFIFOError = 1;
                        rxFIFOTime.sec = sec;
                        rxFIFOTime.uSec = TIM1->CNT;

                }
              }
            } else {    // download finished                                    //STATE0, 3, 4, CCA, STXONCCA
                  spiLine = line = __LINE__;

                  if (spiTxRxByteState == RF_SPI_RX2_DOWNLOAD_HEADER_STATE        // Some frame has header only
                    ||  spiTxRxByteState == RF_SPI_RX3_DOWNLOAD_BODY_STATE) {   // RXBUF_Part3 or RXBUF_Part4
                    rxDoneType = 0;     // A successful frame RX. Assume that before rolling the state machine it is 1
                }
                spiLine = line = __LINE__;
                spiTxRxByteState = RF_SPI_INIT_STATE;
                HwSPISSDeAssert(SPI_RADIO);
                SPI_I2S_ITConfig(SPI_RADIO_SPI, SPI_I2S_IT_RXNE, DISABLE);

                diagnostic.setFlagSPIDone2 = ISR_SETFLAG(flagSPIMachineDone);
                spiLine = line = __LINE__;
                glb2 = globl;
            }
        } else {  /* not at last byte */
            spiLine = line = __LINE__;

            pSpiRxBuf++;
            SPI_I2S_SendData(SPI_RADIO_SPI, *(pSpiTxBuf++));
        }
    } else {                /* byteCountLSB == 0 */
        // It happens. SPI-CC2520 should be also working like this:
        // Each SPI_I2S_SendData trigger one interrupt and calls SPI3_IRQHandler
        // Number of time of SPI_I2S_SendData is determined by low byte of spiTxRxByteCount
        // So it could happen if some code does not wait for SPI3_IRQHandler to clean up low byte of spiTxRxByteCount
        // and resets it 0 in advance.
        // Only when SPI action is finished then other tasks are allowed to spiTxRxByteCount
        // spiTxRxByteCount operation in this interrupt service routin is safe but not in other tasks.
       spiLine = line = __LINE__;
       assert(0);
        spiTxRxByteCount = 0;           // for sanity
        spiTxRxByteState = RF_SPI_INIT_STATE;
        HwSPISSDeAssert(SPI_RADIO);     // for sanity
        SPI_I2S_ITConfig(SPI_RADIO_SPI, SPI_I2S_IT_RXNE, DISABLE);
        diagnostic.setFlagSPIDone3 = ISR_SETFLAG(flagSPIMachineDone);
       spiLine = line = __LINE__;
        glb3 = globl;
    }
    // spiLine = line = __LINE__;
    EXIT_ISR();
}

const char *old_func;
uint16_t old_line;

/*******************************************************************************
* Description : This function handles External line 2 interrupt request.
*               GPI_RADIO_GPIO1 (RX_UNDERFLOW | RX OVERFLOW) = PD.1
* Input       : -
* Return      : -
*******************************************************************************/
void EXTI1_IRQHandler(void) {
    // Clear the  EXTI pending bit
    EXTI_ClearITPendingBit(EXTI_Line1);
#ifdef TASKS_PROFILE
    old_line = CoGetTaskLine(CoGetCurrentTaskId());
#endif
    old_func = func;
    func = __func__;

    rxFIFOError = 1;     // failed. No need to lock? double check
}

/*******************************************************************************
* Description : This function handles DMA1 Channel 4 interrupt request.
*               DMA4_Ch4: USART1-TX
* Input       : -
* Return      : -
*******************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{
      func = __func__;

    if(DMA_GetITStatus(DMA1_IT_TC4)) {
        /* Clear DMA1 Channel4 Transfer Complete interrupt pending bit */
        DMA_ClearITPendingBit(DMA1_IT_TC4);

        DMA_Cmd(COM2_DMA_CHAN_TX, DISABLE);
    } else assert(0);
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

extern void RX_Pkt_Handler(void);
extern ETH_DMADESCTypeDef  *DMATxDescToSet;
extern ETH_DMADESCTypeDef  *DMARxDescToGet;

/**
  * @brief  This function handles ETH interrupt request.
  * @param  None
  * @retval None
  */
void ETH_IRQHandler(void) {
    ENTER_ISR();
        func = __func__;

    if (ETH_GetDMAITStatus(ETH_DMA_IT_T) == SET) {      // interrupt of transmit
        assert((DMATxDescToSet->Status & ETH_DMATxDesc_OWN) == (u32)RESET);

        diagnostic.setEtherTxDone = ISR_SETFLAG(flagEtherTxDone);

        // push into message queue of EtherTask. Wake it up ASAP without waiting for timeout
        diagnostic.postEthTaskQueue = isr_PostQueueMail(ethTaskMsgQueue, wakeupMsg);

        /* Clear the Eth DMA Tx IT pending bits */
        ETH_DMAClearITPendingBit(ETH_DMA_IT_T);
    }

    if (ETH_GetDMAITStatus(ETH_DMA_IT_R) == SET) {  // interrupt of receive

        /* Handles all the received frames */
        while(ETH_GetRxPktSize() != 0) {
            RX_Pkt_Handler();
        }

        /* Clear the Eth DMA Rx IT pending bits */
        ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
    }
    ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
    EXIT_ISR();
}



