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
        - SPI3_IRQn (SPI_RADIO_IRQn)
        - EXTI2_IRQn (GPI_RADIO_GPIO0_IRQn)
        - EXTI1_IRQn (GPI_RADIO_GPIO1_IRQn)
        - DMA1_Channel4_IRQn (SPI_IMU_RX_DMA_IRQ)
        - EXTI9_5_IRQn (GPI_IMU_DIO1_IRQn)
        - TIM6_IRQn
        - TIM5_IRQn
        - TIM2_IRQn
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

#define RF_SYNC_PERIOD 30000
#define RF_SYNC_PERIOD_TOL 750

#define RF_SYNC_PERIOD2 2*RF_SYNC_PERIOD
#define RF_SYNC_PERIOD_TOL2 2*RF_SYNC_PERIOD_TOL

#define HIST_INIT_2  RF_SYNC_PERIOD2,RF_SYNC_PERIOD2
#define HIST_INIT_4  HIST_INIT_2,HIST_INIT_2
#define HIST_INIT_8  HIST_INIT_4,HIST_INIT_4
#define HIST_INIT_16 HIST_INIT_8,HIST_INIT_8
#define HIST_INIT_32 HIST_INIT_16,HIST_INIT_16

#define NUM_TX_RETRIES 4

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

/* Variables Defined in main.c */

extern OS_FlagID flagIMUNewData;
extern OS_FlagID flagRadioTxAllow;

extern uint8_t *led0Id;
extern uint8_t *led1Id;
extern uint8_t *led2Id;


/* PRIVATE VARIABLES ---------------------------------------------------------*/

static uint32_t countA = 0;
static uint32_t countB = 0;
static  uint32_t frameId = 0;

static uint16_t tim4HistA[32] = {HIST_INIT_32};
static uint16_t tim3HistA[16];
static uint16_t tim2HistA[16];

static uint16_t sampledCCA;

/* PUBLIC VARIABLES ----------------------------------------------------------*/

#ifndef COOS
volatile unsigned int SysTickCounter = 0;
#endif

volatile uint32_t frameIdAtSync;
volatile uint32_t frameIdCorrection;
volatile uint16_t txRetryState = 0;
volatile uint16_t numTxRetries = 0;
volatile uint32_t IMUSampleTime;
volatile uint16_t MsTimerAtSync;

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/

static inline void Tim4UpdMovAvg(uint16_t newVal);
static inline void Tim3UpdMovAvg(uint16_t newVal);
static inline void Tim2UpdMovAvg(uint16_t newVal);
static inline void RFSyncPktRx(void);

/* PRIVATE FUNCTIONS ---------------------------------------------------------*/

/*******************************************************************************
* Description : Updates Moving Average of TIM4 (10Hz Sync Packet Freq)
* Input       : nominal RF_SYNC_PERIOD
* Return      : -
*******************************************************************************/
static inline void Tim4UpdMovAvg(uint16_t newVal) {
    static uint16_t idxA = 0;

    // will accept ONE missing sync-packet
    if (newVal > (3*RF_SYNC_PERIOD/2)) newVal = (newVal + 1) / 2;
    // reject invalid newVal
    if ( (newVal > (RF_SYNC_PERIOD + RF_SYNC_PERIOD_TOL)) ||
         (newVal < (RF_SYNC_PERIOD - RF_SYNC_PERIOD_TOL)) ) {
        return;
    }

    tim4HistA[idxA] = newVal * 2;   // make use of full dynamic-range
    idxA = (idxA + 1) & 0x1F;       // array size = 32
}

/*******************************************************************************
* Description : Updates Moving Average of TIM3 (100Hz IRLED Phase)
* Input       :
* Return      :
*******************************************************************************/
static inline void Tim3UpdMovAvg(uint16_t newVal) {
    static uint16_t idxA = 0;

    tim3HistA[idxA] = newVal;
    idxA = (idxA + 1) & 0xF;        // array size = 16
}

/*******************************************************************************
* Description : Updates Moving Average of TIM2 (10Hz Radio TX Inhibit Phase)
* Input       :
* Return      :
*******************************************************************************/
static inline void Tim2UpdMovAvg(uint16_t newVal) {
    static uint16_t idxA = 0;

    tim2HistA[idxA] = newVal;
    idxA = (idxA + 1) & 0xF;        // array size = 16
}

/*******************************************************************************
* Description : Received (10Hz) RF Sync Packet from TimeKeeper
* Input       :
* Return      :
*******************************************************************************/
static inline void RFSyncPktRx(void) {
    static uint16_t tim4CntPrev;

    // update moving-average of freq
    Tim4UpdMovAvg(TIM4->CCR1 - tim4CntPrev);
    tim4CntPrev = TIM4->CCR1;

    // update moving-average of phase
    Tim3UpdMovAvg(TIM3->CCR1); // 100Hz IRLED
    Tim2UpdMovAvg(TIM2->CCR1); // 10Hz Radio TX Inhibit
}

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

#ifndef STDIO_TO_USART
/*******************************************************************************
* Description : This function handles USB-On-The-Go FS global interrupt request.
* Priority    : 0
* Input       : -
* Return      : -
*******************************************************************************/
void OTG_FS_IRQHandler(void) {
    STM32_PCD_OTG_ISR_Handler();
}
#endif

/*******************************************************************************
* Description : This function handles TIM3 global interrupt request.
*               TIM3: 100Hz IRLED
* Priority    : 1
* Input       : -
* Return      : -
*******************************************************************************/
void TIM3_IRQHandler(void) {
    uint16_t bitmask;

    if (TIM_GetITStatus(TIM3, TIM_IT_CC2) == SET) {
        // Clear TIM3 Capture compare interrupt pending bit
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);

        // increment FrameID and make any necessary corrections
        frameId = frameId + frameIdCorrection + 1;
        frameIdCorrection = 0;

        /* FrameID synchronized LED lighting */
        bitmask = (1 << (frameId & 0x7)); // CAST chose 8-bit IDs
#ifdef BC_HW_REVA

        if (*led0Id & bitmask) HwGPOLow(GPO_IRLED0); else HwGPOHigh(GPO_IRLED0);
        if (*led1Id & bitmask) HwGPOLow(GPO_IRLED1); else HwGPOHigh(GPO_IRLED1);
        if (*led2Id & bitmask) HwGPOLow(GPO_IRLED2); else HwGPOHigh(GPO_IRLED2);
#else  // for all versions after REV A
       if (*led0Id & bitmask) HwGPOHigh(GPO_IRLED0); else HwGPOLow(GPO_IRLED0);
        if (*led1Id & bitmask) HwGPOHigh(GPO_IRLED1); else HwGPOLow(GPO_IRLED1);
        if (*led2Id & bitmask) HwGPOHigh(GPO_IRLED2); else HwGPOLow(GPO_IRLED2);
#endif

    } else if (TIM_GetITStatus(TIM3, TIM_IT_CC3) == SET) {
        // Clear TIM3 Capture compare interrupt pending bit
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);

        // turn off LEDs now to save power
        HwGPOHigh(GPO_IRLED0); HwGPOHigh(GPO_IRLED1); HwGPOHigh(GPO_IRLED2);
    } else assert(0);
}

/*******************************************************************************
* Description : This function handles SPI3 (SPI_RADIO_IRQn) global interrupt request.
* Priority    : 2
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
                /* kick off ISR-driven SPI download */
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
            } else {                                    // 0x2000, 0x1000, 0x0800, 0x0400
                if (spiTxRxByteCount & 0x3000) {        // RXBUF_Part3 or RXBUF_Part4
                    ISR_SETFLAG(flagRadioRxFrame);
                } else if (spiTxRxByteCount & 0x0800) { // reading FMSTAT1 (CCA Status)
                    sampledCCA = *pSpiRxBuf;
                }
                spiTxRxByteCount = 0;
                HwSPISSDeAssert(SPI_RADIO);
                SPI_I2S_ITConfig(SPI_RADIO_SPI, SPI_I2S_IT_RXNE, DISABLE);
            }
        } else {  /* not at last byte */
            if (spiTxRxByteCount & 0x2000) {            // RXBUF_Part3
                spiTxRxByteCount &= ~0x2000;            // go to RXBUF_Part4
                spiTxRxByteCount |=  0x1000;            // "
                if (*pSpiRxBuf == 0x40) {               // check FCF0 = Beacon Frame
                    RFSyncPktRx();
                }
            }
            pSpiRxBuf++;
            SPI_I2S_SendData(SPI_RADIO_SPI, *pSpiTxBuf++);
        }
    } else {                /* byteCountLSB == 0 */
        assert(0); // [[DEBUG]]
        spiTxRxByteCount = 0;           // for sanity
        HwSPISSDeAssert(SPI_RADIO);     // for sanity
        SPI_I2S_ITConfig(SPI_RADIO_SPI, SPI_I2S_IT_RXNE, DISABLE);
    }

    EXIT_ISR();
}

/*******************************************************************************
* Description : This function handles External line 2 interrupt request.
*               GPI_RADIO_GPIO0 (RX_FRM_DONE) = PD.2
* Priority    : 3
* Input       : -
* Return      : -
*******************************************************************************/
void EXTI2_IRQHandler(void) {
    static uint16_t rxFrmDoneCount = 0; // [[DEBUG]]
    rxFrmDoneCount++; // [[DEBUG]]

    // Clear the  EXTI pending bit
    EXTI_ClearITPendingBit(EXTI_Line2);

    /* capture state in case this is an RF Sync Packet */
    // latch TIM2, TIM3 & TIM4 counters NOW via software CC1 event
    TIM2->EGR = TIM_EventSource_CC1;
    TIM3->EGR = TIM_EventSource_CC1;
    TIM4->EGR = TIM_EventSource_CC1;
    // store current frameId to compare and correct later
    frameIdAtSync = frameId;
    MsTimerAtSync = TIM1->CNT;
    
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
        __disable_interrupt();
        spiTxRxByteCount |= 0x8000; // request to go to RXBUF_Part1
        __enable_interrupt();
    }
}

/*******************************************************************************
* Description : This function handles External line 2 interrupt request.
*               GPI_RADIO_GPIO1 (TX_FRM_DONE) = PD.1
* Priority    : 4
* Input       : -
* Return      : -
*******************************************************************************/
void EXTI1_IRQHandler(void) {
    static uint16_t txFrmDoneCount = 0; // [[DEBUG]]
    txFrmDoneCount++; // [[DEBUG]]

    // Clear the  EXTI pending bit
    EXTI_ClearITPendingBit(EXTI_Line1);

    /* set TIM6 to ring in 350us */
    TIM6->EGR |= TIM_EGR_UG;
    TIM6->CR1 |= TIM_CR1_CEN;
}

/*******************************************************************************
* Description : This function handles DMA1 Channel 4 (SPI_IMU_RX_DMA_IRQ)
*               interrupt request.
* Priority    : 5
* Input       : -
* Return      : -
*******************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{
    ENTER_ISR();

    if(DMA_GetITStatus(DMA1_IT_TC4)) {
        // Clear DMA1 Channel4 Transfer Complete interrupt pending bit
        DMA_ClearITPendingBit(DMA1_IT_TC4);
        HwGPOLow(GPO_TP10);
        
        HwSPISSDeAssert(SPI_IMU);
        if (flagIMUNewData) ISR_SETFLAG(flagIMUNewData);
        countB++;
    } else assert(0);

    EXIT_ISR();
}

/*******************************************************************************
* Description : This function handles External lines 9 to 5 interrupt request.
* Priority    : 6
* Input       : -
* Return      : -
*******************************************************************************/
void EXTI9_5_IRQHandler(void) {
    // GPI_IMU_DIO1 = PD.8
    if(EXTI_GetITStatus(EXTI_Line8) == SET) {
        // Clear the  EXTI pending bit
        EXTI_ClearITPendingBit(EXTI_Line8);
        

// Grab current 32 bit timestamp and flag        
//        IMUSampleTime = frameId;
         IMUSampleTime = TIM1->CNT;
         
         HwGPOHigh(GPO_TP10);
        // Initiate SPI DMA to pull IMU data
        if (countA == countB) {
            countA++;
            HwSPISSAssert(SPI_IMU);
            DMA_Cmd(SPI_IMU_RX_DMA_CHAN, DISABLE);
            DMA_Cmd(SPI_IMU_TX_DMA_CHAN, DISABLE);
            SPI_IMU_RX_DMA_CHAN->CNDTR = 8;
            SPI_IMU_TX_DMA_CHAN->CNDTR = 8;
            DMA_Cmd(SPI_IMU_RX_DMA_CHAN, ENABLE);
            DMA_Cmd(SPI_IMU_TX_DMA_CHAN, ENABLE);
        }
    } else assert(0);
}

/*******************************************************************************
* Description : This function handles TIM6 global interrupt request.
*               TIM6: TX_FRM_DONE to flagRadioTxDone Delay
* Priority    : 7
* Input       : -
* Return      : -
*******************************************************************************/
void TIM6_IRQHandler(void) {
    ENTER_ISR();

    if(TIM_GetITStatus(TIM6, TIM_IT_Update) == SET) {
        /* Clear TIM5 Update interrupt pending bit */
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);

        ISR_SETFLAG(flagRadioTxDone);
        ISR_SETFLAG(flagRadioTxDoneUser);
    }

    EXIT_ISR();
}

/*******************************************************************************
* Description : This function handles TIM5 global interrupt request.
*               TIM5: STXONCCA Retries
* Priority    : 8
* Input       : -
* Return      : -
*******************************************************************************/
void TIM5_IRQHandler(void) {
    static uint16_t numRetried = 0;         // [[DEBUG]]
    static uint16_t numRetriedFail = 0;     // [[DEBUG]]

    ENTER_ISR();

    if(TIM_GetITStatus(TIM5, TIM_IT_Update) == SET) {
        /* Clear TIM5 Update interrupt pending bit */
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);

        assert(txRetryState != 0); // [[DEBUG]]

        if (txRetryState == 1) {            // read FMSTAT1
            /* check if CCA was asserted */
            if (!spiTxRxByteCount) {        // SPI not in use
                /* read FSMSTAT1 register */
                scratchBuf[0] = (CC2520_INS_REGRD | CC2520_FSMSTAT1);
                scratchBuf[1] = 0;          // pad byte to push out reg val
                spiTxRxByteCount = 0x0802;  // indicate FMSTAT1 read
                pSpiTxBuf = scratchBuf;
                pSpiRxBuf = scratchBuf;     // self-clobbering ... that's ok
                HwSPISSAssert(SPI_RADIO);
                SPI_I2S_ITConfig(SPI_RADIO_SPI, SPI_I2S_IT_RXNE, ENABLE);
                SPI_I2S_SendData(SPI_RADIO_SPI, *pSpiTxBuf++);
                txRetryState++;             // go to next state
            }                               // else, remain in this state
        } else if (txRetryState == 2) {     // check SAMPLED_CCA bit, retry TX if necessary

            if (sampledCCA & CC2520_FSMSTAT1_SAMPLED_CCA_BM) {
                /* CCA asserted, packet is going (has gone) out */
                txRetryState = 0;
                EXIT_ISR();
                return;                     // don't set timer
            }

            /* CCA NOT asserted, packet isn't going out ... try STXONCCA again now */
            putchar('!'); // [[DEBUG]]
            numRetried++; // [[DEBUG]]

            if (numTxRetries < NUM_TX_RETRIES) {
                if (!spiTxRxByteCount) {        // SPI not in use
                    spiTxRxByteCount = 0x0401;  // indicate STXONCCA retry
                    pSpiRxBuf = scratchBuf;     // don't care
                    HwSPISSAssert(SPI_RADIO);
                    SPI_I2S_ITConfig(SPI_RADIO_SPI, SPI_I2S_IT_RXNE, ENABLE);
                    SPI_I2S_SendData(SPI_RADIO_SPI, CC2520_INS_STXONCCA);
                    numTxRetries++;
                    txRetryState = 1;
                }                               // else, remain in this state, retry later
            } else { // done retrying, no more
                putchar('@'); // [[DEBUG]]
                numRetriedFail++; // [[DEBUG]]
                txRetryState = 0;
                // fake TX_FRM_DONE
                ISR_SETFLAG(flagRadioTxDone);
                ISR_SETFLAG(flagRadioTxDoneUser);
                EXIT_ISR();
                return;                         // don't set timer
            }

        } else assert(0);

        /* set timer to ring again in 1ms */
        TIM5->EGR |= TIM_EGR_UG;
        TIM5->CR1 |= TIM_CR1_CEN;
    }

    EXIT_ISR();
}

/*******************************************************************************
* Description : This function handles TIM2 global interrupt request.
*               TIM2: 10Hz Radiox TX Inhibit
* Priority    : 9
* Input       : -
* Return      : -
*******************************************************************************/
void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_CC2) == SET) {
        // Clear TIM2 Capture compare interrupt pending bit
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);

        /* START of Inhibition */
        txInhibit = 1;
        CoClearFlag(flagRadioTxAllow);
        HwGPOHigh(GPO_TP11); // [[DEBUG]]
    } else if (TIM_GetITStatus(TIM2, TIM_IT_CC3) == SET) {
        // Clear TIM2 Capture compare interrupt pending bit
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);

        /* END of Inhibition */
        txInhibit = 0;
        ISR_SETFLAG(flagRadioTxAllow);
        HwGPOLow(GPO_TP11); // [[DEBUG]]
    } else assert(0);
}

/*******************************************************************************
* Description : Calculates Moving Average of TIM4 (10Hz Sync Packet Freq)
* Input       : -
* Return      : nominal 2 x RF_SYNC_PERIOD
*******************************************************************************/
uint16_t Tim4GetMovAvg(void) {
    static uint32_t acc = RF_SYNC_PERIOD2;
    uint16_t *pVal;
    uint16_t i;

    pVal = tim4HistA;
    acc = 0;
    for (i = 0; i < 32; i++) {      // array size = 32
        acc += *pVal++;
    }

    return (acc/32);
}

/*******************************************************************************
* Description : Calculates Moving Average of TIM3 (100Hz IRLED Phase)
* Input       : -
* Return      :
*******************************************************************************/
uint16_t Tim3GetMovAvg(void) {
    static uint32_t acc;
    uint16_t *pVal;
    uint16_t i;

    pVal = tim3HistA;
    acc = 0;
    for (i = 0; i < 16; i++) {      // array size = 16
        acc += *pVal++;
    }

    return (acc/16);
}

/*******************************************************************************
* Description : Calculates Moving Average of TIM2 (10Hz Radio TX Inhibit Phase)
* Input       : -
* Return      :
*******************************************************************************/
uint16_t Tim2GetMovAvg(void) {
    static uint32_t acc;
    uint16_t *pVal;
    uint16_t i;

    pVal = tim2HistA;
    acc = 0;
    for (i = 0; i < 16; i++) {      // array size = 16
        acc += *pVal++;
    }

    return (acc/16);
}

/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
