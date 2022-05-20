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
#include "stm32f10x_tim.h"
#include "usb_istr.h"
#include "hardware.h"
#include "basic_rf.h"
#include "radio_defs.h"
#include "tasks.h"
#include "CoOS.h"
#include <stdio.h>
#include "usb_core.h"
#include "packets.h"
#include "i2c_ee.h"
#include "flash_map.h"
#include "config.h"

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


/* PRIVATE MACROS ------------------------------------------------------------*/

#define ENTER_ISR() CoEnterISR()
#define EXIT_ISR() CoExitISR()
#define ISR_SETFLAG(x) isr_SetFlag(x);

/* EXTERN VARIABLES ----------------------------------------------------------*/

/* Variables Defined in main.c */
extern tBtn_State btnA;
extern tBtn_State btnB;

extern OS_FlagID flagBTNNewData;
extern OS_FlagID flagIMUNewData;
extern OS_FlagID flagIMU_G_DRDY;
extern OS_FlagID flagRadioTxAllow;
extern OS_FlagID flagIMUTimeToSend;
extern OS_FlagID flagRadioCCA;
extern OS_EventID semRFRxFrames;   // Number of RF frames received in Rx-FIFO. It is notified by RX_FRM_DONE interrupt

extern int TRACE(char* fmt, ...);
extern uint32_t lostSync;
//extern SPI3_CS_TypeDef SPI3_CS;
extern void ISR_RadioReleaseSPI(void);
/* PRIVATE VARIABLES ---------------------------------------------------------*/

static uint32_t countA = 2; // Init to 2 not zero because we read before the DRDY gets tripped
static uint32_t countB = 0;
static  uint32_t frameId = 0;

// history size
#define TIM3_HIS_SIZE 1
#define TIM2_HIS_SIZE 1

static uint16_t tim4HistA[32] = {HIST_INIT_32};
static uint16_t tim3HistA[16];
static uint16_t tim2HistA[16];

uint32_t drift;
int32_t last_drift;

uint16_t sampledCCA;


extern uint8_t radio_off;

/* PUBLIC VARIABLES ----------------------------------------------------------*/

volatile int32_t frameIdCorrection;
volatile uint16_t txRetryState = 0;
volatile uint16_t numTxRetries = 0;
volatile uint32_t IMUSampleTime;
volatile uint16_t MsTimerAtSync;
volatile uint32_t frameIdAtSync;
volatile uint8_t  IMUPktNumAtSync;
uint32_t          real_sec;
uint16_t        tim2_phase;
uint16_t        tim3_phase;

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/

static inline void Tim4UpdMovAvg(uint16_t newVal);
static inline void Tim3UpdMovAvg(uint16_t newVal);
static inline void Tim2UpdMovAvg(uint16_t newVal);
static inline void RFSyncPktRx(void);

extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);

/* PRIVATE FUNCTIONS ---------------------------------------------------------*/
#ifdef USE_TIM4
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
#endif

static uint8_t tim3_update_count;
/*******************************************************************************
* Description : Updates Moving Average of TIM3 (100Hz IRLED Phase)
* Input       :
* Return      :
*******************************************************************************/
static inline void Tim3UpdMovAvg(uint16_t newVal) {
    static uint16_t idxA = 0;
    static uint32_t last_sec = 0;
    // if missed 1 update, reset sequence
    if (real_sec - last_sec > 1) {
      idxA = 0;
      tim3_update_count = 0;
    }
    tim3HistA[idxA] = newVal;
    idxA = (++idxA) & (TIM3_HIS_SIZE - 1);
    if (!tim3_update_count & 0xF0) {
      tim3_update_count++;
    }
    last_sec = real_sec;
}

static uint8_t tim2_update_count;
/*******************************************************************************
* Description : Updates Moving Average of TIM2 (10Hz Radio TX Inhibit Phase)
* Input       :
* Return      :
*******************************************************************************/
static inline void Tim2UpdMovAvg(uint16_t newVal) {
    static uint16_t idxA = 0;
    static uint32_t last_sec = 0;

    // if missed 1 update, reset sequence
    if (real_sec - last_sec > 1) {
      idxA = 0;
      tim2_update_count = 0;
    }
    tim2HistA[idxA] = newVal;
    idxA = (++idxA) & (TIM2_HIS_SIZE - 1);
    if (!tim2_update_count & 0xF0) {
      tim2_update_count++;
    }
    last_sec = real_sec;
}
#define BTNPRESSED    1
#define BTNDEPRESSED  0
/*******************************************************************************
* Description : Updates Button States with TIM2 (10Hz)
* Input       :
* Return      :
*******************************************************************************/
static inline uint8_t chkbtnstate(tBtn_State *pbtn, uint8_t btnval){
  uint8_t x = 0;
  switch(pbtn->state){
    case BTNSTATE_OFF:
      if(btnval == BTNPRESSED){
        if(pbtn->hcount++ >= config.debounce_time){
          pbtn->actual_dbleclick_time = config.doubleclick_time;
          pbtn->state = BTNSTATE_ON;
        }
      }
      break;
    case BTNSTATE_ON:
      pbtn->actual_dbleclick_time = (pbtn->actual_dbleclick_time == 0)?0:pbtn->actual_dbleclick_time--;
      if(btnval == BTNPRESSED){
        if(pbtn->hcount++ >= config.hold_time){
          pbtn->update_msg = BTNMSG_HOLD;
          x=1;
        }
      }
      else{
        pbtn->hcount = 0; 
        pbtn->state = BTNSTATE_PRESSDETECT;
      }
      break;
    case BTNSTATE_PRESSDETECT:
      pbtn->actual_dbleclick_time = (pbtn->actual_dbleclick_time == 0)?0:pbtn->actual_dbleclick_time--;
      if(btnval == BTNPRESSED){
        //doubleclick
        if(pbtn->hcount++ >= config.debounce_time){
          pbtn->state = BTNSTATE_OFF;
          pbtn->update_msg = BTNMSG_DBLCLK;
          x=1;
        }
      }
      else if(!pbtn->actual_dbleclick_time){
        //single click
        pbtn->state = BTNSTATE_OFF;
        pbtn->update_msg = BTNMSG_PRESS;
        x=1;
      }
      //return to initialstate
      break;
    case BTNSTATE_HOLD:
      if(btnval != BTNPRESSED){
        pbtn->state = BTNSTATE_OFF;
        pbtn->hcount = 0;
        pbtn->update_msg = BTNMSG_RELEASE;
        x=1;
      }
    break;
  }
  return x;
}

uint32_t syncPackets = 0;
/*******************************************************************************
* Description : Received (10Hz) RF Sync Packet from TimeKeeper
* Input       :
* Return      :
*******************************************************************************/
static inline void RFSyncPktRx(void) {
    // update moving-average of phase
    Tim3UpdMovAvg(TIM3->CCR1); // 100Hz IRLED
    Tim2UpdMovAvg(TIM2->CCR1); // 10Hz Radio TX Inhibit

    // Reset to frameCountNoSync/10 seconds: a 10Hz timer will decrease the value. 100 ms / tick
    __disable_interrupt();
    remainOutOfSyncTime = config.frameCountNoSync;
    syncPackets++;
     __enable_interrupt();
}

__no_init  unsigned int stacked_lr;
__no_init  unsigned int stacked_pc;
__no_init  unsigned int stacked_psr;

uint32_t hf_counter;

/* PUBLIC FUNCTIONS ----------------------------------------------------------*/
void hard_fault_handler_c(unsigned int * hardfault_args)
{
  unsigned int stacked_r0;
  unsigned int stacked_r1;
  unsigned int stacked_r2;
  unsigned int stacked_r3;
  unsigned int stacked_r12;

  stacked_r0 = ((unsigned long) hardfault_args[0]);
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  stacked_r3 = ((unsigned long) hardfault_args[3]);

  stacked_r12 = ((unsigned long) hardfault_args[4]);
  stacked_lr = ((unsigned long) hardfault_args[5]);
  stacked_pc = ((unsigned long) hardfault_args[6]);
  stacked_psr = ((unsigned long) hardfault_args[7]);


  //HwLEDOff(LED1);
  //HwLEDOn(LED2);
  //HwLEDOn(LED3);
  //HwLEDOff(LED4);
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
  USBD_OTG_ISR_Handler (&USB_OTG_dev);
}
#endif
uint32_t sec;
/*******************************************************************************
* Description : This function handles TIM1 global interrupt request.
*               TIM1: 95Hz IMU Sampling
* Priority    : 1
* Input       : -
* Return      : -
*******************************************************************************/
void TIM1_UP_IRQHandler(void) {
    ENTER_ISR();

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {

    // Clear TIM1 Capture compare interrupt pending bit
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    assert(TIM1->RCR == 0);
    sec++;
    if (sec == 20) {
      drift = 0;
      last_drift = 0;
    }
    HwGPOToggle(GPO_TP5);// test

    // Get IMU sample
    // PDE here is where we trigger the new IMU data is ready to be processed
    // and then transmitted
    //ISR_SETFLAG(flagIMUTimeToSend);
    }

    EXIT_ISR();
}

uint32_t old_t1;
uint16_t over1;

/*******************************************************************************
* Description : This function handles TIM3 global interrupt request.
*               TIM3: 100Hz IRLED
* Priority    : 1
* Input       : -
* Return      : -
*******************************************************************************/
static uint16_t delete_me;
static uint32_t prev_frame_id;
uint32_t myt1, myold_t1;
uint16_t adjusted;
uint8_t adjusted_changed;
extern uint16_t tim_at_sec;
uint8_t led_blinking = 0;
uint16_t last_reload;
uint32_t lastFrameIdAtCorrection;
uint32_t trace_irq;
extern uint16_t use_tim3_phase;
extern uint8_t got_beacon;

uint32_t delayedFrameIdCorrection;

void TIM3_IRQHandler(void) {
    uint32_t bitmask;
    static uint8_t tim3_counter;
    static int16_t my_tim3_phase;
    static uint8_t fillbits = 0;
    static uint16_t over = 0;
    static int32_t halfreload = (TIM3_AUTORELOAD + 1) >> 1;
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
    {
      __disable_interrupt();
      TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
      trace_irq = 2;
      tim3_counter++;
      // increment FrameID and make any necessary corrections
      frameId = frameId + 1 + delayedFrameIdCorrection;
      if (config.flags & FLAG_FRAMEID_24BITS) {
        frameId &= 0xFFFFFF;
      }
      __enable_interrupt();
#ifdef CIRCULAR_LOG
      if (delayedFrameIdCorrection != 0) {
        WRITE_LOG(CoGetOSTime(), LOG_TYPE_TIMER, frameId);
      }
#endif

      if (/*tim3_counter == 100*/got_beacon) {
        //frameId += frameIdCorrection;
        delayedFrameIdCorrection = frameIdCorrection;
        got_beacon = 0;
        if (frameIdCorrection != 0) {
          lastFrameIdAtCorrection = frameId;
        }
        frameIdCorrection = 0;
          tim_at_sec = TIM1->CNT;
          if (use_tim3_phase) {
            tim3_phase = use_tim3_phase;
            use_tim3_phase = 0;
          }
          my_tim3_phase = TIM3_AUTORELOAD - tim3_phase;
          uint16_t reload = TIM3_AUTORELOAD - tim3_phase;
          if (reload/*tim3_phase*/ < halfreload) {
            reload += TIM3_AUTORELOAD;
          }
          tim3_phase = 0;
          //increment every second
          real_sec++;
          tim3_counter = 0;
          TIM3->ARR =  reload;
          last_reload = reload;
          if (TIM3->ARR == 0) {
            TIM3->ARR = TIM3_AUTORELOAD;
          }
          adjusted = TIM3->ARR;
          adjusted_changed = 1;
          #ifdef CIRCULAR_LOG
            WRITE_LOG(CoGetOSTime(), LOG_TYPE_ARR, TIM3->ARR);
          #endif
      } else {
        TIM3->ARR = TIM3_AUTORELOAD;
        delayedFrameIdCorrection = 0;
      }
      return;
    }

    if (TIM_GetITStatus(TIM3, TIM_IT_CC2) == SET)
    {
        //HwGPOToggle(GPO_TP50);// test
        // Clear TIM3 Capture compare interrupt pending bit
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
        int32_t t1 = TIM1->CNT;
        int32_t d = t1 + 60000;
        d = (d - old_t1)%60000;
        if (d > 660 || d < 540) {
          drift++;
          last_drift = d;
        }
        old_t1 = t1;
       // TIM1->CNT = 0;
        uint32_t ledIdBit = 0;
        if (config.frameBits == 12) { // workaround for byte swap
          ledIdBit = (frameId)%config.frameBits;
        } else {
          ledIdBit = frameId%config.frameBits;
        }
        /* FrameID synchronized LED lighting */
        bitmask = (1 << ledIdBit); // CAST chose 8-bit IDs. Which bit needs to be checked
        if (!radio_off) {
          led_blinking = 1;
          if (config.led0IdPattern & bitmask) {
            HwGPOHigh(GPO_IRLED0);
          } else {
            HwGPOLow(GPO_IRLED0);
          }

          if (config.led1IdPattern & bitmask) {
            HwGPOHigh(GPO_IRLED1);
          } else {
            HwGPOLow(GPO_IRLED1);
          }

          if (config.led2IdPattern & bitmask) {
            HwGPOHigh(GPO_IRLED2);
          } else {
            HwGPOLow(GPO_IRLED2);
          }
        }
    }
    else if (TIM_GetITStatus(TIM3, TIM_IT_CC3) == SET)
    {
        // Clear TIM3 Capture compare interrupt pending bit
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);

        // turn off LEDs now to save power
#ifdef BC_HW_REVA
        HwGPOHigh(GPO_IRLED0); HwGPOHigh(GPO_IRLED1); HwGPOHigh(GPO_IRLED2);
#else
        // disable TIM 3 CC4
        HwGPOLow(GPO_IRLED0); HwGPOLow(GPO_IRLED1); HwGPOLow(GPO_IRLED2);
#endif
    }
    else
    {
      assert(0);
    }
}
extern uint8_t rxCount;

uint32_t spi3_errors;
struct realTime spiTime;
struct realTime rxFIFOTime;
extern uint8_t readCCA;
uint8_t ccaVal;
uint8_t dummy;

struct realTime frameTime;

/*******************************************************************************
* Description : This function handles SPI3 (SPI_RADIO_IRQn) global interrupt request.
* Priority    : 2
* Input       : -
* Return      : -
*******************************************************************************/
void SPI3_IRQHandler(void) {
    ENTER_ISR();
	if (SPI_I2S_GetFlagStatus(SPI_RADIO_IMU_SPI, SPI_I2S_FLAG_RXNE) == RESET) {
		EXIT_ISR();
		spi3_errors++;
		return;
	}
	spiTime.sec = sec;
	spiTime.uSec = TIM1->CNT;
	
	if(HwGetSPISS(SPI_RADIO)){	//check current chip selection SS is config to radio.
	//if(SPI3_CS==RADIO_CS){
		if (spiTxRxByteCount) {     // just finish up a SPI byte I/O
			spiTxRxByteCount--;
			*pSpiRxBuf = SPI_I2S_ReceiveData(SPI_RADIO_IMU_SPI);
			if (spiTxRxByteCount == 0) {  // at last byte. Roll the state machine
				if (spiTxRxByteState == RF_SPI_RX1_UPLOADCMD_STATE) {
					if (rxCount) {
						spiTxRxByteState = RF_SPI_RX2_DOWNLOAD_HEADER_STATE;  // start receiving frame
						spiTxRxByteCount = rxCount; //-1;
						(pSpiRxBuf++);    // PHR: Frame Length
						// assert(spiTxRxByteCount <= rxCount);
						if (spiTxRxByteCount) {
							// *(pSpiTxBuf++) = SPI_I2S_ReceiveData(SPI_RADIO_IMU_SPI);
							// spiTxRxByteCount--;
							spiTxRxByteCount++;
							SPI_I2S_SendData(SPI_RADIO_IMU_SPI, *(pSpiTxBuf));
						}
					} else {
                //ignore it, no data available
						spiTxRxByteState = RF_SPI_INIT_STATE;
						SPI_I2S_ITConfig(SPI_RADIO_IMU_SPI, SPI_I2S_IT_RXNE, DISABLE);
						ISR_SETFLAG(flagSPIMachineDone);
						if (rxCount) {
							// To flush overflown buffer
							rxFIFOError = 1;
							rxFIFOTime.sec = sec;
							rxFIFOTime.uSec = TIM1->CNT;
						}
					}
				} else {    // download finished                                    //STATE0, 3, 4, CCA, STXONCCA
					if (spiTxRxByteState == RF_SPI_UPLOAD_ONLY_STATE) {         // Upload frame is done
						txDoneType = 0;
					}
					else if (spiTxRxByteState == RF_SPI_RX2_DOWNLOAD_HEADER_STATE        // Some frame has header only
						||  spiTxRxByteState == RF_SPI_RX3_DOWNLOAD_BODY_STATE) {   // RXBUF_Part3 or RXBUF_Part4
						rxDoneType = 0;     // A successful frame RX. Assume that before rolling the state machine it is 1
					} else if (spiTxRxByteState == RF_SPI_CCA_CMD_STATE) { // reading FMSTAT1 (CCA Status)
						sampledCCA = *pSpiRxBuf;
					}
					spiTxRxByteState = RF_SPI_INIT_STATE;
					HwSPISSDeAssert(SPI_RADIO);
					SPI_I2S_ITConfig(SPI_RADIO_IMU_SPI, SPI_I2S_IT_RXNE, DISABLE);

					ISR_SETFLAG(flagSPIMachineDone);
				}
			} else {  /* not at last byte */
				if (spiTxRxByteState == RF_SPI_RX2_DOWNLOAD_HEADER_STATE) {            // RXBUF_Part3: Receiving frame
					spiTxRxByteState = RF_SPI_RX3_DOWNLOAD_BODY_STATE;            // go to RXBUF_Part4 (Received first byte of frame then turn to state 4
					if (*pSpiRxBuf == 0x40) {               // check FCF0 = Beacon Frame
						RFSyncPktRx();
						frameTime.sec = sec;
						frameTime.uSec = TIM1->CNT;
					}
				}
				pSpiRxBuf++;
				SPI_I2S_SendData(SPI_RADIO_IMU_SPI, *(pSpiTxBuf++));
			}
		} else {                /* byteCountLSB == 0 */
			// It happens. SPI-CC2520 should be also working like this:
			// Each SPI_I2S_SendData trigger one interrupt and calls SPI3_IRQHandler
			// Number of time of SPI_I2S_SendData is determined by low byte of spiTxRxByteCount
			// So it could happen if some code does not wait for SPI3_IRQHandler to clean up low byte of spiTxRxByteCount
			// and resets it 0 in advance.
			// Only when SPI action is finished then other tasks are allowed to spiTxRxByteCount
			// spiTxRxByteCount operation in this interrupt service routin is safe but not in other tasks.
			assert(0);
			spiTxRxByteCount = 0;           // for sanity
			spiTxRxByteState = RF_SPI_INIT_STATE;
			HwSPISSDeAssert(SPI_RADIO);     // for sanity
			SPI_I2S_ITConfig(SPI_RADIO_IMU_SPI, SPI_I2S_IT_RXNE, DISABLE);
			ISR_SETFLAG(flagSPIMachineDone);
		}
	}
	else{	//current chip select configured as IMU.
		*pSpiRxBuf_IMU = SPI_I2S_ReceiveData(SPI_RADIO_IMU_SPI);
		if(spiIMUCount){
			spiIMUCount--;
			//don't increment rx buffer if the previous sent byte refers to register address 
			if(spiIMUByteState != IMU_SPI_INIT_STATE){
				pSpiRxBuf_IMU++;
				SPI_I2S_SendData(SPI_RADIO_IMU_SPI, *(pSpiTxBuf_IMU++));	
			} 
			else{
				SPI_I2S_SendData(SPI_RADIO_IMU_SPI, *(pSpiTxBuf_IMU++));
				spiIMUByteState = IMU_SPI_BODY_STATE;
			}
		}
		else{	//finished obtaining all data from IMU
			*pSpiRxBuf_IMU = SPI_I2S_ReceiveData(SPI_RADIO_IMU_SPI);
			spiIMUByteState = IMU_SPI_INIT_STATE;
			HwSPISSDeAssert(SPI_IMU);
			SPI_I2S_ITConfig(SPI_RADIO_IMU_SPI, SPI_I2S_IT_RXNE, DISABLE);
			ISR_SETFLAG(flagIMUNewData);
		}
	}
    EXIT_ISR();
}
struct realTime semTime;
uint16_t tim3_at_radio;
uint32_t rxPackets;
int frameIdInced = 0;

/*******************************************************************************
* Description : This function handles External line 2 interrupt request.
*               GPI_RADIO_GPIO0 (RX_FRM_DONE) = PD.2
* Priority    : 3
* Input       : -
* Return      : -
*******************************************************************************/
void EXTI2_IRQHandler(void) {
  rxPackets++;
  if (frameIdFlag) {
    /* capture state in case this is an RF Sync Packet */       // As each timer has 4 channels to handle differrent events, they do not affect main counter
    // latch TIM2, TIM3 & TIM4 counters NOW via software CC1 event
    TIM2->EGR = TIM_EventSource_CC1;        // Must catch it to align 10Hz timer. Albert
#ifdef USE_TIM4
    TIM4->EGR = TIM_EventSource_CC1;
#endif
#if 0
    TIM3->EGR = TIM_EventSource_CC1;        // simulate a capture / compare event generated on channel 1 as wireless does not physicsly send signal to channel 1
    HwGPOToggle(GPO_TP12);
    //HwGPOHigh(GPO_TP50);// test

    // store current frameId to compare and correct later
      frameIdAtSync = frameId;
      if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET && (TIM3->CCR1 > 15000)) {
        frameIdAtSync++;
        frameIdInced = 1;
      } else {
        frameIdInced = 0;
      }
#else
    HwGPOToggle(GPO_TP10);
    frameIdAtSync = frameId;
    TIM3->EGR = TIM_EventSource_CC1;        // simulate a capture / compare event generated on channel 1 as wireless does not physicsly send signal to channel 1
    // store current frameId to compare and correct later
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) {
        frameIdAtSync++;// = frameId;
        TIM3->EGR = TIM_EventSource_CC1;        // simulate a capture / compare event generated on channel 1 as wireless does not physicsly send signal to channel 1
        frameIdInced = 1;
      } else {
        frameIdInced = 0;
      }

#endif
#ifdef CIRCULAR_LOG
      WRITE_LOG(CoGetOSTime(), LOG_TYPE_RXPACKET, frameIdAtSync);
#endif
      frameIdFlag = 0;
    tim3_at_radio = TIM3->CNT;
    MsTimerAtSync = TIM1->CNT;
    IMUPktNumAtSync = test_imu_pkt_ctr;
    //TIM1->CNT = 0;
    ENTER_ISR();        // Above should be executed ASAP while ENTER_ISR could last long time
    semTime.sec = sec;
    semTime.uSec = TIM1->CNT;
    StatusType statusType = isr_PostSem(semRFRxFrames);
    // assert(statusType == E_OK); // happen when disconnecting ST-Link in debug mode
   EXIT_ISR();
  } else {
        errorFrameId++;
  }
    // Clear the  EXTI pending bit
    EXTI_ClearITPendingBit(EXTI_Line2);
}

uint32_t rx_overflow;
struct realTime semTime2;

/*******************************************************************************
* Description : This function handles External line 3 interrupt request.
*               GPI_RADIO_GPIO5 (RX_UNDERFLOW | RX OVERFLOW)
* Input       : -
* Return      : -
*******************************************************************************/
void EXTI3_IRQHandler(void) {
    ENTER_ISR(); 
    
    // Clear the  EXTI pending bit
    EXTI_ClearITPendingBit(GPI_RADIO_GPIO5_EXTI_LINE);
    rx_overflow++;
    semTime2.sec = sec;
    semTime2.uSec = TIM1->CNT;

    isr_PostSem(semRFRxFrames);
    rxFIFOError = 1;     // failed. No need to lock? double check
    
    EXIT_ISR();    
}
uint32_t tx_overflow;
/*******************************************************************************
* Description : This function handles External line 0 interrupt request.
*               GPI_RADIO_GPIO2 (TX_OVERFLOW or TX_UNDERFLOW)
* Input       : -
* Return      : -
*******************************************************************************/
void EXTI0_IRQHandler(void) {
    ENTER_ISR();
    // Clear the  EXTI pending bit
    EXTI_ClearITPendingBit(GPI_RADIO_GPIO2_EXTI_LINE);
    tx_overflow++;
    txFIFOError = 1;     // failed. no need to lock? double check
    ISR_SETFLAG(flagRadioTxDone);       // notify TX task a transimit is done (but failed)
    EXIT_ISR();
}

uint32_t txFrmDoneCount = 0; // [[DEBUG]]

/*******************************************************************************
* Description : This function handles External line 2 interrupt request.
*               GPI_RADIO_GPIO1 (TX_FRM_DONE) = PD.1
* Priority    : 4
* Input       : -
* Return      : -
*******************************************************************************/
void EXTI1_IRQHandler(void) {

    ENTER_ISR();

    txFrmDoneCount++; // [[DEBUG]]

    if (!radio_off) {
      //HwLEDToggle(LED2);
      //HwLEDToggle(LED1);
    }


    // Clear the  EXTI pending bit
    EXTI_ClearITPendingBit(EXTI_Line1);

    //HwGPOLow(GPO_TP50);

    if (config.flags & FLAG_TRACE_TIMESLOT) {
        // time slot debug
        extern uint16_t txTimeSlot;
        uint16_t checkPoint = TIM_GetCounter(TIM2);
        uint16_t slotDelta = 2100;
        uint16_t slot2Offset = 60000 / 2 +  + txTimeSlot;
        BOOL error = 0;
        if (   (checkPoint >= txTimeSlot && checkPoint < txTimeSlot + slotDelta)
            || (checkPoint >= slot2Offset && checkPoint < slot2Offset + slotDelta)
                ) {
                    error = 0;
                }
        else {
            error = 1;
        }
        TRACE("(%d %c)\n\r", checkPoint, error == 0 ? ' ' : 'E');
    }

    ISR_SETFLAG(flagRadioTxDone);

    EXIT_ISR();
}
#if 0
uint32_t irq_spi_dma = 0;

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
    irq_spi_dma++;
    if(DMA_GetITStatus(DMA1_IT_TC4)) {
        // Clear DMA1 Channel4 Transfer Complete interrupt pending bit
        DMA_ClearITPendingBit(DMA1_IT_TC4);
        //HwGPOLow(GPO_TP10);

        HwSPISSDeAssert(SPI_A_IMU);
        HwSPISSDeAssert(SPI_G_IMU);

            ISR_SETFLAG(flagIMUNewData);

        countB++;
    } else
      assert(0);

    EXIT_ISR();
}
#endif
uint32_t counter;
StatusType setIMUGRdy;
uint16_t l;

/*******************************************************************************
* Description : This function handles External lines 15 to 10 interrupt request.
				Triggers when the IMU interrupt is asserted.
* Priority    : 6
* Input       : -
* Return      : -
*******************************************************************************/
void EXTI15_10_IRQHandler(void)
{
	ENTER_ISR();
	if(EXTI_GetITStatus(EXTI_Line11) == SET){
	    EXTI_ClearITPendingBit(EXTI_Line11);
		if(1){
			counter++;
			l=__LINE__;
			countA+=2;
			setIMUGRdy = ISR_SETFLAG(flagIMU_G_DRDY);
			l=__LINE__;
			test_imu_pkt_ctr++;
		}
	}
	else{}
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
    ENTER_ISR();

    if(TIM_GetITStatus(TIM5, TIM_IT_Update) == SET) {
        /* Clear TIM5 Update interrupt pending bit */
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);

        ISR_SETFLAG(flagRadioCCA);

    }

    EXIT_ISR();
}

volatile uint32_t secs = 0;
int16_t whole_time_adjust;
int8_t part_time_adjust;

/*******************************************************************************
* Description : This function handles TIM2 global interrupt request.
*               TIM2: 10Hz Radiox TX Inhibit
* Priority    : 9
* Input       : -
* Return      : -
*******************************************************************************/
void TIM2_IRQHandler(void) {
  static uint8_t tim2_counter = 0;
  static int16_t my_tim2_phase;
  volatile uint8_t btnmsg1=0;
  volatile uint8_t btnmsg2=0;
  volatile uint8_t btnval;
    ENTER_ISR();
    // CC2 Inhibit Begin
    // CC3 Inhibit End
    // CC4 IMU Sync
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
      TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
      tim2_counter++;
      if (tim2_counter == 100) {
       tim2_counter = 0;
      }
      if (!(tim2_counter%10)) {
          //increment every second
         // real_sec++;
          if (tim2_counter == 0) {
            TIM2->ARR = tim2_phase + whole_time_adjust + part_time_adjust;
          } else {
            TIM2->ARR = tim2_phase + whole_time_adjust;
          }
          if (TIM2->ARR == 0) {
            TIM2->ARR = TIM_AUTORELOAD;
          }
      } else {
        TIM2->ARR = TIM_AUTORELOAD;
      }
       secs++;

      // ++ Button state process
      btnval = HwButtonPressed(BUTTON1);
      btnmsg1 = chkbtnstate(&btnA, btnval);
      btnval = HwButtonPressed(BUTTON2);
      btnmsg2 = chkbtnstate(&btnB, btnval);

      if(btnmsg1 || btnmsg2){
        ISR_SETFLAG(flagBTNNewData);
      }
      // -- Button state process

#if 0
      // ++ Button state process
      if (buttonA.actual_dblclick_time) {
        buttonA.actual_dblclick_time--;
        if (buttonA.actual_dblclick_time == 0) {
          // Either click or button press
          if (buttonA.events & 0x01 && buttonA.last_state & 0x01) {
            button_state = (button_state & BUTTON_B) | BUTTON_PRESS;
          //  buttons.events = 0;
          } else if (buttonA.events & 0x02) {
            button_state = (button_state & BUTTON_B) | BUTTON_CLICK;
            buttonA.events = 0;
          }
        }
      }

      if (buttonB.actual_dblclick_time) {
        buttonB.actual_dblclick_time--;
        if (buttonB.actual_dblclick_time == 0) {
          // Either click or button press
          if (buttonB.events & 0x01 && buttonB.last_state & 0x01) {
            button_state = (button_state & BUTTON_A) | (BUTTON_PRESS << 4);
          //  buttons.events = 0;
          } else if (buttonB.events & 0x02) {
            button_state = (button_state & BUTTON_A) | (BUTTON_CLICK << 4);
            buttonB.events = 0;
          }
        }
      }
      // -- Button state process
#endif

    } else if (TIM_GetITStatus(TIM2, TIM_IT_CC2) == SET) {
        // Clear TIM2 Capture compare interrupt pending bit
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);

        __disable_interrupt();
        if (remainOutOfSyncTime > 0) {
          remainOutOfSyncTime--;
          if (remainOutOfSyncTime == 0) {
            lostSync++;
          }
        }
        __enable_interrupt();

    }
    else if (TIM_GetITStatus(TIM2, TIM_IT_CC3) == SET) {            // start of timeslot
        // Clear TIM2 Capture compare interrupt pending bit
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
        /* END of Inhibition */
        ISR_SETFLAG(flagRadioTxAllow);
    }
    else if (TIM_GetITStatus(TIM2, TIM_IT_CC4) == SET) {            // Allow for IMU to have a second chance to transmit it is a half of 60000 period???
        // Clear TIM3 Capture compare interrupt pending bit
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
        //if (secs%2==0) {
        ISR_SETFLAG(flagRadioTxAllow);
      }
    /*else
      assert(0);*/
    EXIT_ISR();
}
#ifdef USE_TIM4
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
#endif

/*******************************************************************************
* Description : Calculates Moving Average of TIM2 (10Hz Radio TX Inhibit Phase)
* Input       : -
* Return      :
*******************************************************************************/
uint16_t Tim2GetMovAvg(void) {
    int32_t acc;
    uint16_t *pVal;
    uint16_t i;

    pVal = tim2HistA;
    acc = 0;
    for (i = 0; i < tim2_update_count; i++) {      // array size = 16
      uint16_t val =  *pVal++;
      if (val > (TIM_AUTORELOAD>>1)) {
        acc -=  (TIM_AUTORELOAD - val);
      } else {
        acc += val;
      }
    }

    if (tim2_update_count) {
      acc = (acc/tim2_update_count);
    }
    return acc;

}

/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
