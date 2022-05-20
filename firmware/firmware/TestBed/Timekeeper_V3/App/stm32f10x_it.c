/******************** (C) COPYRIGHT 2011 NaturalPoint, Inc. ********************
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
#include "usb_istr.h"
#include "hardware.h"
#include "basic_rf.h"
#include "radio_defs.h"
#ifdef COOS
#include "CoOS.h"
#endif
#include <stdio.h>

/* PRIVATE TYPEDEF -----------------------------------------------------------*/

/* PRIVATE DEFINES -----------------------------------------------------------*/

/* PRIVATE MACROS ------------------------------------------------------------*/

#ifdef COOS
#define ENTER_ISR() CoEnterISR()
#define EXIT_ISR() CoExitISR()
#define ISR_SETFLAG(x) isr_SetFlag(x);
#else
#define ENTER_ISR()
#define EXIT_ISR()
#define ISR_SETFLAG(x) ( x = 1; )
#endif

/* EXTERN VARIABLES ----------------------------------------------------------*/

//extern OS_FlagID flagRFBeaconSent;

/* PRIVATE VARIABLES ---------------------------------------------------------*/

/* PUBLIC VARIABLES ----------------------------------------------------------*/

#ifndef COOS
volatile unsigned int SysTickCounter = 0;
#endif

volatile uint32_t rfBeaconFrameId = 0;
volatile uint16_t rfBeaconLoaded;

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

/*******************************************************************************
* Description : This function handles Hard Fault exception.
* Input       : -
* Return      : -
*******************************************************************************/
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
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
* Description : This function handles USB-On-The-Go FS global interrupt request.
* Input       : -
* Return      : -
*******************************************************************************/
#ifndef STDIO_TO_USART
void OTG_FS_IRQHandler(void) {
    STM32_PCD_OTG_ISR_Handler();
}
#endif

/*******************************************************************************
* Description : This function handles TIM3 global interrupt request.
*               TIM3: 100Hz OH Sync-Out, 10Hz RF-Sync Pkt
* Input       : -
* Return      : -
*******************************************************************************/
void TIM3_IRQHandler(void) {
    static uint16_t ohPktCount = 0;
    static uint16_t modulo10 = 0;

//    if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) {
//        /* Clear TIM3 Capture compare interrupt pending bit */
//        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

        // 10Hz RF-Sync Packets
//        if (++modulo10 == 10) {
///            modulo10 = 0;
       
//            if (rfBeaconLoaded) { // ok to STXON
//                rfBeaconLoaded = 0;
//                HwGPOHigh(GPO_RADIO_GPIO2);     // trigger STXON
//                rfBeaconFrameId = *frameId;
//                ISR_SETFLAG(flagRFBeaconSent);
//                HwGPOLow(GPO_RADIO_GPIO2);      // reset strobe
//            }
//        }

        // increment frame-id[31:0]
//        (*frameId)++;
//        if ((*frameId & 0x7) == 0) { // [[DEBUG]]
//            HwGPOHigh(GPO_TP47);
//        } else {
//            HwGPOLow(GPO_TP47);
//        }

        /* send OptiHub Sync Packet */
//        COM2_DMA_CHAN_TX->CNDTR = COM2TxBufSize;
//        assert (USART_GetFlagStatus(COM2_USART, USART_FLAG_TXE) == SET);
//        DMA_Cmd(COM2_DMA_CHAN_TX, ENABLE);
//        if (ohPktCount++ == 50) {
//            HwLEDToggle(LED5);
//            ohPktCount = 0;
//        }
//    } else assert(0);
}

/*******************************************************************************
* Description : This function handles External line 2 interrupt request.
*               GPI_RADIO_GPIO0 (RX_FRM_DONE) = PD.2
* Input       : -
* Return      : -
*******************************************************************************/
void EXTI2_IRQHandler(void) {
    // Clear the  EXTI pending bit
    EXTI_ClearITPendingBit(EXTI_Line2);

    if (!spiTxRxByteCount) {    // SPI not in use
        /* kick off ISR-driven SPI download */
        scratchBuf[0] = CC2520_INS_BCLR;    // reset RX_FRM_DONE signal/exception
        scratchBuf[1] = (CC2520_EXCFLAG1 << 3) | (CC2520_EXC_RX_FRM_DONE - 8);
        scratchBuf[2] = CC2520_INS_SNOP;    // pad to 16-bit word align
        scratchBuf[3] = CC2520_INS_RXBUF;
        spiTxRxByteCount = 0x4005;  // go to RXBUF_Part2
        pSpiTxBuf = scratchBuf;
        pSpiRxBuf = (uint8_t*)&rxPkt;
        HwSPISSAssert(SPI_RADIO);
        SPI_I2S_ITConfig(SPI_RADIO_SPI, SPI_I2S_IT_RXNE, ENABLE);
        SPI_I2S_SendData(SPI_RADIO_SPI, *pSpiTxBuf++);
    } else {
        spiTxRxByteCount |= 0x8000; // request to go to RXBUF_Part1
    }
}

/*******************************************************************************
* Description : This function handles SPI3 (SPI_RADIO_IRQn) global interrupt request.
* Input       : -
* Return      : -
*******************************************************************************/
void SPI3_IRQHandler(void) {
    static uint16_t byteCountLSB;

    ENTER_ISR();

    if (SPI_I2S_GetFlagStatus(SPI_RADIO_SPI, SPI_I2S_FLAG_RXNE) == RESET) {
        EXIT_ISR();
        return;
    }

    byteCountLSB = spiTxRxByteCount & 0x00FF;

    if (byteCountLSB) {     /* byteCountLSB > 0 */
        byteCountLSB--;
        spiTxRxByteCount = (spiTxRxByteCount & 0xFF00) | byteCountLSB;
        *pSpiRxBuf = SPI_I2S_ReceiveData(SPI_RADIO_SPI);

        if (byteCountLSB == 0) {  /* at last byte */
            if (spiTxRxByteCount  & 0x8000) {           // RXBUF_Part1
                HwSPISSDeAssert(SPI_RADIO);
                // kick off ISR-driven SPI download
                scratchBuf[0] = CC2520_INS_BCLR;        // reset RX_FRM_DONE signal/exception
                scratchBuf[1] = (CC2520_EXCFLAG1 << 3) | (CC2520_EXC_RX_FRM_DONE - 8);
                scratchBuf[2] = CC2520_INS_SNOP;        // pad to 16-bit word align
                scratchBuf[3] = CC2520_INS_RXBUF;
                spiTxRxByteCount = 0x4005;              // go to RXBUF_Part2
                pSpiTxBuf = scratchBuf;
                pSpiRxBuf = (uint8_t*)&rxPkt;
                HwSPISSAssert(SPI_RADIO);
                SPI_I2S_SendData(SPI_RADIO_SPI, *pSpiTxBuf++);
            } else if (spiTxRxByteCount & 0x4000) {     // RXBUF_Part2
                if ((*pSpiRxBuf < 2) || (*pSpiRxBuf > 127)) {   // Frame Length INVALID!
                    HwSPISSDeAssert(SPI_RADIO);
                    *pSpiTxBuf++ = CC2520_INS_SFLUSHRX; // get rid of this packet
                    *pSpiTxBuf-- = CC2520_INS_SFLUSHRX; // double flush [CC2520 Bug#1]
                    spiTxRxByteCount = 0x0002;          // above 2 instructions
                    HwSPISSAssert(SPI_RADIO);
                    SPI_I2S_SendData(SPI_RADIO_SPI, *pSpiTxBuf++);
                } else {
                    spiTxRxByteCount = 0x2000 |         // go to RXBUF_Part3
                                       *pSpiRxBuf++;    // PHR: Frame Length
                    SPI_I2S_SendData(SPI_RADIO_SPI, *pSpiTxBuf++);
                }
            } else {
                if (spiTxRxByteCount & 0x2000) {        // RXBUF_Part3
                    ISR_SETFLAG(flagRadioRxFrame);
                }
                spiTxRxByteCount = 0;
                HwSPISSDeAssert(SPI_RADIO);
                SPI_I2S_ITConfig(SPI_RADIO_SPI, SPI_I2S_IT_RXNE, DISABLE);
            }
        } else {  /* not at last byte */
            pSpiRxBuf++;
            SPI_I2S_SendData(SPI_RADIO_SPI, *pSpiTxBuf++);
        }
    } else {                /* byteCountLSB != 0 */
        assert(0); // [[DEBUG]]
        spiTxRxByteCount = 0;           // for sanity
        HwSPISSDeAssert(SPI_RADIO);     // for sanity
        SPI_I2S_ITConfig(SPI_RADIO_SPI, SPI_I2S_IT_RXNE, DISABLE);
    }

    EXIT_ISR();
    
}

/*******************************************************************************
* Description : This function handles External line 2 interrupt request.
*               GPI_RADIO_GPIO1 (TX_FRM_DONE) = PD.1
* Input       : -
* Return      : -
*******************************************************************************/
void EXTI1_IRQHandler(void) {
#ifdef  OS_PRESENT
  ENTER_ISR();

    // Clear the  EXTI pending bit
    EXTI_ClearITPendingBit(EXTI_Line1);
    ISR_SETFLAG(flagRadioTxDone);
    ISR_SETFLAG(flagRadioTxDoneUser);

    EXIT_ISR();
#endif    
}

/*******************************************************************************
* Description : This function handles DMA1 Channel 4 interrupt request.
*               DMA4_Ch4: USART1-TX
* Input       : -
* Return      : -
*******************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TC4)) {
        /* Clear DMA1 Channel4 Transfer Complete interrupt pending bit */
        DMA_ClearITPendingBit(DMA1_IT_TC4);

        DMA_Cmd(COM2_DMA_CHAN_TX, DISABLE);
    } else assert(0);
}

/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
