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
#include "usb_dcd_int.h"
#include "i2c_ee.h"


/* PRIVATE TYPEDEF -----------------------------------------------------------*/

/* PRIVATE DEFINES -----------------------------------------------------------*/

/* PRIVATE MACROS ------------------------------------------------------------*/

#define ENTER_ISR() CoEnterISR()
#define EXIT_ISR() CoExitISR()
#define ISR_SETFLAG(x) isr_SetFlag(x);

/* EXTERN VARIABLES ----------------------------------------------------------*/
extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
extern ETH_DMADESCTypeDef  *DMATxDescToSet;
extern ETH_DMADESCTypeDef  *DMARxDescToGet;

/* PRIVATE VARIABLES ---------------------------------------------------------*/

/* PUBLIC VARIABLES ----------------------------------------------------------*/

/*
StatusType      setLoadFrame;
StatusType      setRFBeaconSent;
StatusType      setRadioTxDone;
StatusType      setSPIMachineDone;
StatusType      setEtherTxDone;*/

uint32_t        dma_done;
uint32_t        hf_counter;
int16_t         whole_time_adjust;
uint8_t         rfChannel;
int8_t          part_time_adjust;

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

  I2C_EE_BufferRead((uint8_t*) &hf_counter, EEPROM_DEBUG_COUNTER, EEPROM_DEBUG_COUNTER_SIZE);
  hf_counter++;
  I2C_EE_BufferWrite((uint8_t*) &hf_counter, EEPROM_DEBUG_COUNTER, EEPROM_DEBUG_COUNTER_SIZE);

  printf ("[Hard fault handler]\n");
  printf ("R0 = %x\n", stacked_r0);
  printf ("R1 = %x\n", stacked_r1);
  printf ("R2 = %x\n", stacked_r2);
  printf ("R3 = %x\n", stacked_r3);
  printf ("R12 = %x\n", stacked_r12);
  printf ("LR = %x\n", stacked_lr);
  printf ("PC = %x\n", stacked_pc);
  printf ("PSR = %x\n", stacked_psr);
  printf ("BFAR = %x\n", (*((volatile unsigned long *)(0xE000ED38))));
  printf ("CFSR = %x\n", (*((volatile unsigned long *)(0xE000ED28))));
  printf ("HFSR = %x\n", (*((volatile unsigned long *)(0xE000ED2C))));
  printf ("DFSR = %x\n", (*((volatile unsigned long *)(0xE000ED30))));
  printf ("AFSR = %x\n", (*((volatile unsigned long *)(0xE000ED3C))));


  while (1) ;

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
  asm("ITE EQ \n"
      "MRSEQ R0, MSP \n"
      "MRSNE R0, PSP");
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
static void onTimer();
uint32_t pers;
uint32_t detectingTIM1 = 1;
uint32_t countTIM1 = 0;
void TIM1_UP_IRQHandler(void) {
      countTIM1++;      // to detect existence of accurate external clock
      TIM_ClearFlag(TIM1, TIM_FLAG_Update);
      ENTER_ISR();

      // pers++;
      if (!detectingTIM1) {      
        onTimer();
      }
      
      EXIT_ISR();
  /*    if ((++count) & 0x01) {
        GPIO_SetBits(GPIOE, GPIO_Pin_15);
      } else {
        GPIO_ResetBits(GPIOE, GPIO_Pin_15);
      }*/
}

#ifdef DIAGNOSTICS
uint32_t tim2_counter;

void TIM2_IRQHandler(void)
{
    tim2_counter++;
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
}
#endif

/*******************************************************************************
* Description : This function handles USB-On-The-Go FS global interrupt request.
* Input       : -
* Return      : -
*******************************************************************************/
void OTG_FS_IRQHandler(void) {
  USBD_OTG_ISR_Handler (&USB_OTG_dev);
}


uint32_t stxon_counter;
uint16_t stxon_tim3;
uint32_t mytim3_counter;

uint32_t marks[200];
uint16_t off;

/*******************************************************************************
* Description : This function handles TIM3 global interrupt request.
*               TIM3: 100Hz OH Sync-Out, 10Hz RF-Sync Pkt
* Input       : -
* Return      : -
*******************************************************************************/
static void onTimer()
{
    static uint16_t ohPktCount = 0;
    static uint8_t tim3_counter = 0;
       GPIO_SetBits(GPIOE, GPIO_Pin_15);
        tim3_counter++;
        mytim3_counter++;
#if 0
        if (tim3_counter == /*100*/ config.frameClock) {
          tim3_counter = 0;
        }
        if (!(tim3_counter%(config.frameClock/10))) {
          uint16_t reload;
          if (!tim3_counter) {
            reload = TIM3_AUTORELOAD + whole_time_adjust + part_time_adjust;
          } else {
            reload = TIM3_AUTORELOAD + whole_time_adjust;
          }
          if (reload != 0) {
            TIM3->ARR = reload;
          } else {
            TIM3->ARR = TIM3_AUTORELOAD;
          }
        } else {
          TIM3->ARR = TIM3_AUTORELOAD;
        }
#endif
        // 1Hz RF-Sync Packets
        uint32_t modulo = (*frameId % /*100*/config.frameClock);
        tick = (uint8_t) (*frameId / config.frameClock);
        //every 50 millisec
        uint8_t channel_modulo = modulo%(config.frameClock/20)/*CHANNEL_FRAME_SHIFT*/;
        if (channel_modulo == 0) {
          rfChannel = modulo/(config.frameClock/20);//CHANNEL_FRAME_SHIFT;
          rfBeaconFrameId = *frameId;
          if ((( 1 << rfChannel ) & config.activeChannelsBitMask) == 0) {
            rfChannel = NUM_RF_CHANNELS;
          }
          diagnostic.setLoadFrame = ISR_SETFLAG(flagLoadFrameId);
        } else if (channel_modulo == 2 && rfChannel < NUM_RF_CHANNELS) {
         if(!(HwGPIState(GPI_RADIO_GPIO5)))
            {
              { // ok to STXON
                    HwGPOHigh(GPO_RADIO_GPIO2);     // trigger STXON
                    diagnostic.setRFBeaconSent = ISR_SETFLAG(flagRFBeaconSent);
                    HwGPOLow(GPO_RADIO_GPIO2);      // reset strobe
              }
            }
            else
            {
              rfBeaconLoaded = 0;
              //rfBeaconFrameId = *frameId;
              diagnostic.setRFBeaconSent = ISR_SETFLAG(flagRFBeaconSent);
              diagnostic.rfTxErrors++;
            }/////...
        }

        // increment frame-id[31:0]
        (*frameId)++;
        if (config.flags & FLAG_FRAMEID_24BITS) {
          *frameId = ((*frameId) % 16777212);           // as 16777212 % 12 == 0
        }
        if (((*frameId)%12) == 11) {
          GPIO_ResetBits(GPIOE, GPIO_Pin_15);
#if 0
          if (off < ((sizeof(marks)) >> 3)) {
          marks[off++] = pers;
          marks[off++] = TIM3->CNT;
          } else {
            off =  0xFFFF;
          }
#endif
        }
        /* send OptiHub Sync Packet */
        COM2_DMA_CHAN_TX->CNDTR = COM2TxBufSize;
        assert (USART_GetFlagStatus(COM2_USART, USART_FLAG_TXE) == SET);
        DMA_Cmd(COM2_DMA_CHAN_TX, ENABLE);
        if (ohPktCount++ == 50) {
            HwLEDToggle(LED5);
            ohPktCount = 0;
        }

}

void TIM3_IRQHandler(void) {
    static uint16_t ohPktCount = 0;
    static uint8_t tim3_counter = 0;

    //static uint16_t modulo10 = 0;
    ENTER_ISR();
    if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) {
        /* Clear TIM3 Capture compare interrupt pending bit */
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        if (usePreciseClock)
        {
          //it is done in TIM1_IRQHandler()
       //   onTimer();
          pers++;
        }
        else 
        {
        tim3_counter++;
        mytim3_counter++;
        if (tim3_counter == /*100*/ config.frameClock) {
          tim3_counter = 0;
        }
        if (!(tim3_counter%(config.frameClock/10))) {
          uint16_t reload;
          if (!tim3_counter) {
            reload = TIM3_AUTORELOAD + whole_time_adjust + part_time_adjust;
          } else {
            reload = TIM3_AUTORELOAD + whole_time_adjust;
          }
          if (reload != 0) {
            TIM3->ARR = reload;
          } else {
            TIM3->ARR = TIM3_AUTORELOAD;
          }
        } else {
          TIM3->ARR = TIM3_AUTORELOAD;
        }

        // 1Hz RF-Sync Packets
        uint32_t modulo = (*frameId % /*100*/config.frameClock);
#ifdef MULTI_CHANNEL        
        tick = (uint8_t) (*frameId / config.frameClock);
        //every 50 millisec
        uint8_t channel_modulo = modulo%(config.frameClock/20)/*CHANNEL_FRAME_SHIFT*/;
        if (channel_modulo == 0) {
          rfChannel = modulo/(config.frameClock/20);//CHANNEL_FRAME_SHIFT;
          rfBeaconFrameId = *frameId;
          if ((( 1 << rfChannel ) & config.activeChannelsBitMask) == 0) {
            rfChannel = NUM_RF_CHANNELS;
          }
          diagnostic.setLoadFrame = ISR_SETFLAG(flagLoadFrameId);
        } else if (channel_modulo == 2 && rfChannel < NUM_RF_CHANNELS) {
            
#else 
        if (!modulo) {
          rfBeaconFrameId = *frameId;
          diagnostic.setLoadFrame = ISR_SETFLAG(flagLoadFrameId);
        }  else if (modulo == 14) {
          /*14 is a "2 + 12" to keep offset same we have 2, but suspicios is that 20 millisec (2)
            is a little, so we added 12 for 12 bit frameId to keep same LED is*/
        
#endif     
         if(!(HwGPIState(GPI_RADIO_GPIO5)))
            {
              { // ok to STXON
                    HwGPOHigh(GPO_RADIO_GPIO2);     // trigger STXON
                    diagnostic.setRFBeaconSent = ISR_SETFLAG(flagRFBeaconSent);
                    HwGPOLow(GPO_RADIO_GPIO2);      // reset strobe
              }
            }
            else
            {
              rfBeaconLoaded = 0;
              //rfBeaconFrameId = *frameId;
              diagnostic.setRFBeaconSent = ISR_SETFLAG(flagRFBeaconSent);
              diagnostic.rfTxErrors++;
            }/////...
          }

        // increment frame-id[31:0]
        (*frameId)++;
        if (config.flags & FLAG_FRAMEID_24BITS) {
          *frameId = ((*frameId) % 16777212);           // as 16777212 % 12 == 0        
        }
        /* send OptiHub Sync Packet */
        COM2_DMA_CHAN_TX->CNDTR = COM2TxBufSize;
        assert (USART_GetFlagStatus(COM2_USART, USART_FLAG_TXE) == SET);
        DMA_Cmd(COM2_DMA_CHAN_TX, ENABLE);
        if (ohPktCount++ == 50) {
            HwLEDToggle(LED5);
            ohPktCount = 0;
        }
        }
    } else assert(0);
    EXIT_ISR();
}

/*******************************************************************************
* Description : This function handles External line 2 interrupt request.
*               GPI_RADIO_GPIO0 (TX_OVERFLOW or TX_UNDERFLOW) = PD.2
* Input       : -
* Return      : -
*******************************************************************************/
void EXTI2_IRQHandler(void) {
    ENTER_ISR();
    // Clear the  EXTI pending bit
    EXTI_ClearITPendingBit(EXTI_Line2);

    txFIFOError = 1;     // failed. no need to lock? double check
    diagnostic.setRadioTxDone = ISR_SETFLAG(flagRadioTxDone);       // notify TX task a transimit is done (but failed)
    EXIT_ISR();
}

/*******************************************************************************
* Description : This function handles SPI3 (SPI_RADIO_IRQn) global interrupt request.
* Input       : -
* Return      : -
*******************************************************************************/
void SPI3_IRQHandler(void) {
    ENTER_ISR();

    if (SPI_I2S_GetFlagStatus(SPI_RADIO_SPI, SPI_I2S_FLAG_RXNE) == RESET) {
        EXIT_ISR();
        return;
    }

    if (spiTxRxByteCount) {     // just finish up a SPI byte I/O
        spiTxRxByteCount--;
        *pSpiRxBuf = SPI_I2S_ReceiveData(SPI_RADIO_SPI);

        if (spiTxRxByteCount == 0) {  // at last byte. Roll the state machine
            assert(spiTxRxByteState == RF_SPI_UPLOAD_ONLY_STATE);
            txDoneType = 0;
            spiTxRxByteState = RF_SPI_INIT_STATE;
            HwSPISSDeAssert(SPI_RADIO);
            SPI_I2S_ITConfig(SPI_RADIO_SPI, SPI_I2S_IT_RXNE, DISABLE);

            diagnostic.setSPIMachineDone = ISR_SETFLAG(flagSPIMachineDone);

        } else {  /* not at last byte */
            assert(spiTxRxByteState == RF_SPI_UPLOAD_ONLY_STATE);
            pSpiRxBuf++;
            SPI_I2S_SendData(SPI_RADIO_SPI, *pSpiTxBuf++);
        }
    } else {
        assert(0); // [[DEBUG]]
        spiTxRxByteCount = 0;           // for sanity
        spiTxRxByteState = RF_SPI_INIT_STATE;
        HwSPISSDeAssert(SPI_RADIO);     // for sanity
        SPI_I2S_ITConfig(SPI_RADIO_SPI, SPI_I2S_IT_RXNE, DISABLE);
        diagnostic.setSPIMachineDone = ISR_SETFLAG(flagSPIMachineDone);
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
    ENTER_ISR();

    // Clear the  EXTI pending bit
    EXTI_ClearITPendingBit(EXTI_Line1);

    diagnostic.setRadioTxDone = ISR_SETFLAG(flagRadioTxDone);

    EXIT_ISR();
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
        dma_done = *frameId;
        DMA_Cmd(COM2_DMA_CHAN_TX, DISABLE);
    } else assert(0);
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

#ifdef Ether_active
/**
  * @brief  This function handles ETH interrupt request.
  * @param  None
  * @retval None
  */
void ETH_IRQHandler(void) {
    ENTER_ISR();
    if (ETH_GetDMAITStatus(ETH_DMA_IT_T) == SET) {      // interrupt of transmit
        assert((DMATxDescToSet->Status & ETH_DMATxDesc_OWN) == (u32)RESET);

        diagnostic.setEtherTxDone = ISR_SETFLAG(flagEtherTxDone);

        // push into message queue of EtherTask. Wake it up ASAP without waiting for timeout
        StatusType status = isr_PostQueueMail(ethTaskMsgQueue, wakeupMsg);

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


#endif

#ifdef NUCLEAR

uint32_t cnt;
void EXTI9_5_IRQHandler()
{
  if (EXTI_GetITStatus(EXTI_Line5) == SET) {
   EXTI_ClearITPendingBit(EXTI_Line5);
   if (((++cnt)%10) == 0) {
      if (off < ((sizeof(marks)) >> 4)) {
          marks[off++] = cnt;
          marks[off++] = pers;
          marks[off++] = TIM3->CNT; //TIM1->CNT;
      } else {
          off =  0xFFFF;
      }
   }

  }
}
#endif