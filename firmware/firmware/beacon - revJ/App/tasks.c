/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
* File Name          : tasks.c
* Author             : ?
* Version            : V1.0
* Date               : 6/2/2011
* Description        : All the various System Tasks
*******************************************************************************/

#include "VersionNo.h"

/* INCLUDES ------------------------------------------------------------------*/

#include "tasks.h"
#include "CoOS.h"
#include "hardware.h"
#include "basic_rf.h"
#include "radio.h"
#include "stm32f10x_it.h"
#include "stm32f10x_iwdg.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_adc.h"
#include "i2c_ee.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "radio.h"
#include "radio_defs.h"
#include "packets.h"
#include "flash_map.h"
#include "util.h"
#include "config.h"
#include "console_tail.h"

#include "i2c_periph.h"
#include "eink.h"
#include "blkt_gfx.h"
#include "bitmaps.h"
/* PRIVATE TYPEDEF -----------------------------------------------------------*/

typedef struct {
    uint16_t    dstAddr;
    uint8_t     payloadSize;
    uint8_t     payload[FRAME_LENGTH_MAX - MHR_SIZE - MFR_SIZE];
} txPktRec_t;

typedef enum{
  UI_POWERON_STATE        =0,
  UI_HOME_STATE           =1,
  UI_MENU_STATE           =2,
  UI_STATUS_RADIO_STATE   =3,
  UI_STATUS_BATTERY_STATE =4,
  UI_ID_STATE             =5,
  //UI_CHARGING_STATE       =6,
  UI_POWEROFF_STATE       =7,
}UI_States;

typedef enum{
  UI_MENU_HOME_SUBSTATE =0,
  UI_MENU_STATUS_SUBSTATE =1,
  UI_MENU_ID_SUBSTATE =2,
  UI_MENU_POWER_SUBSTATE =3,
}UI_SubStates;
/* PRIVATE DEFINES -----------------------------------------------------------*/

#ifdef STDIO_TO_USART
#define DISABLE_PWR_SW
#endif

#define PINGER_ADDR         0x1234
#define TX_PKT_QUEUE_SIZE   8

#define UPDATE_FLAG_PANID       0x01
#define UPDATE_FLAG_DSTADDR     0x02
#define UPDATE_FLAG_DAC         0x04
#define UPDATE_FLAG_RFCHAN      0x08
#define UPDATE_FLAG_TXLEVEL     0x10

/*
 Beacon Pkt Duration = 0.70ms
 Max TX Pkt Duration = 3.5ms
 Inhibit Guard-Band Pre = 4.0ms
 Inhibit Guard-Band Post = 0.5ms
*/
#define SOH 0x01
#define EOT 0x04
#define ACK 0x06

#define NUM_TX_RETRIES 4

#define DEBOUNCE_MIN    1
#define DEBOUNCE_MAX    9
#define DBLCLICK_MIN_TIME       1
#define DBLCLICK_MAX_TIME       9

#define BUTTON_DEBOUNCE 1

#define UI_PWRBTNFLAG 0x01
#define UI_BTNAFLAG   0x02
#define UI_BTNBFLAG   0x04
#define UI_IRLED0FLAG 0x01
#define UI_IRLED1FLAG 0x02
#define UI_IRLED2FLAG 0x04
/* PRIVATE MACROS ------------------------------------------------------------*/

/* EXTERN VARIABLES ----------------------------------------------------------*/


/* Variables Defined in main.c */

extern char firmwareVersion[];
extern uint8_t radio_off;

extern uint32_t txFrmDoneCount;
extern uint32_t irq_spi_dma;
//uint32_t task2Counter;
extern StatusType task2StatusType;
extern uint32_t task2enter;

extern OS_TID task1Id;
extern OS_TID taskRadioRxId;
extern OS_TID task3Id;
extern OS_TID taskConfigId;
extern OS_TID taskRadioTxId;
extern OS_TID task8Id;
extern OS_TID taskIMUGId;
extern OS_TID taskLEDId;
extern OS_TID taskUIId;

extern OS_FlagID flagIMUNewData;
extern OS_FlagID flagIMU_G_DRDY;
extern OS_FlagID flagRadioTxReq;
extern OS_FlagID flagIMUTimeToSend;
extern OS_FlagID flagRadioCCA;

extern uint32_t drift;
extern int32_t last_drift;

extern OS_EventID semIMUAllow;
extern OS_EventID semInkUI;
extern OS_EventID semEinkBufferAllow;

extern OS_FlagID flagIMU_read;
extern OS_FlagID flagIMUDataReady;
extern OS_FlagID flagLEDSync;
extern OS_FlagID flagEInk_RDY;
extern OS_FlagID flagEInk_DMA_Done;
extern OS_FlagID flagBtn_UI;
extern OS_FlagID flagLED_UI;
extern OS_FlagID flagTimeout_UI;
extern OS_FlagID flagUSBdata_UI;
extern volatile uint32_t IMUSampleTime;
extern volatile uint16_t MsTimerAtSync;
extern volatile uint8_t IMUPktNumAtSync;

extern Batt_Union_t BattUnion, *pBattUnion;
//extern SPI3_CS_TypeDef SPI3_CS;

extern uint32_t rxErrors;
extern uint32_t sec;
extern struct realTime semTime2;
extern struct realTime rxFIFOTime;

extern volatile uint16_t tasksWDT;
#if (defined USE_MY_ASSERT) || (defined USE_FULL_ASSERT)
extern uint8_t  assert_loop;
#endif
extern uint16_t routerAddr;
extern uint8_t asserted;
extern uint8_t cc2520_flags0;
extern uint8_t cc2520_flags1;
extern uint8_t cc2520_flags2;
extern uint16_t sampledCCA;

#ifdef CIRCULAR_LOG
tLogStruct log[LOG_SIZE/sizeof(tLogStruct)];
uint16_t  log_index_in;
#endif

uint8_t last_percents;
uint32_t Valids;
uint32_t notValids;
uint32_t lostSync;
uint32_t changeClocks;
uint32_t newbcn;
uint32_t frameIdCorrectionCount;
//uint8_t TRACE_ADJUST = 0;
uint32_t oldFrameIdAtSync, newFrameIdAtSync;

//extern uint32_t rx_overflow;
//extern uint32_t tx_overflow;
//extern uint32_t syncPackets;

extern uint16_t adjusted;
extern uint8_t adjusted_changed;


int TRACE(char* fmt, ...);
size_t __writeIMU(const unsigned char *buffer, size_t size);

size_t __writeCmdLineRespPacket(const unsigned char *buffer, size_t size, uint8_t contentType);
uint16_t tim_at_sec;

uint32_t rt_flags;

uint32_t outidxcheck=0;
uint32_t inidxcheck=0;
uint32_t txradioaccess_cnt=0;
uint32_t imuaccess=0;
extern uint32_t imu_irq_cnt;
/* PRIVATE VARIABLES ---------------------------------------------------------*/
//static uint16_t tim4MovAvgMin = UINT16_MAX, tim4MovAvgMax = 0;
static volatile uint16_t halted = 0;
static uint8_t beaconInSync = 0;
static uint8_t txTestStr[] = {0xAA, 0x55, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
static uint8_t txBuf[FRAME_LENGTH_MAX - MHR_SIZE - MFR_SIZE];  // tx-payload buffer
static uint8_t rfChan;
static uint8_t beaconRSSI;
static txPktRec_t txPktQueue[TX_PKT_QUEUE_SIZE];
static uint16_t inIdx = 0;
static uint16_t outIdx = 0;
static struct realTime startRadioTx;
static struct realTime endRadioTx;
static uint8_t queue_full;
//static uint8_t send_battery_info = 1;

volatile int32_t remainOutOfSyncTime;

extern volatile uint32_t frameId;
extern uint16_t tim3PhaseAtSync;

static struct Beacon_BatData lastBatStatus;
static tButton ButtonA;
static tButton ButtonB;
IRled_status IRled_flags; 
UI_States UI_StateMachine = UI_POWERON_STATE;
static uint8_t beaconName[32];
volatile uint8_t frameIdFlag = 1;       // for task2() and EXTI2_IRQHandler() synchronization
                                        // frameIdFlag =0 1 means that task2() is not processing packets,  EXTI2_IRQHandler() is OK to accept now.
                                        // if task2() is still processing then EXTI2_IRQHandler() has to give up and wait for next packet interrupt (multiple packets in FIFO). It is OK for beacon to ignore some sync packets
                                        // frameIdFlag == 0 means that EXTI2_IRQHandler() detected packet arrival

uint32_t         errorFrameId;

//#define EIGHTSAMPLEOFFSET 48299   // 80.5mS / 1.666 uS
#define EIGHTSAMPLEOFFSET 48089   // 80.0mS / 1.666 uS
#define FIVESAMPLEOFFSET  0   // 3.5mS / 1.666 uS

// Timeslots in uS
//  0, 876, 5,532, 10,188, 14,844, 19,500, 24,156, 28,812, 33,468, 38,124, 42,780
//  47,436, 52,092, 56,748, 61,404, 66,060, 70,716, 75,372, 80,028, 84,684, 89,340
//  93,996
// timeslot offset run from 1.666uS counter
//
/*uint16_t TimeSlotVals[] = {0, 525, 3319, 6112, 8906, 11700, 14493, 17287, 20080, 22874,
          25667, 28461, 31254, 34048, 36841, 39635, 42429, 45222, 48016, 50809,
        53603, 56396 };*/

/* same as above but now using 3.0 ms time slots instead of 4.65ms
1800 counts per 3.0ms*/

//3.5ms slots
/*uint16_t TimeSlotVals[] = {0,   525,  2625, 4725, 6825, 8925,11025,13125,15225,17325,
                          19425,21525,23625,25725,};*/
static uint16_t TimeSlotVals[] = {0,   0, 2100, 4200, 6300, 8400,10500,12600,14700,16800,
                          18900,21000,23100,25200, 27300, 29400};

#define MAX_NUMBER_OF_TIMESLOTS sizeof(TimeSlotVals) / sizeof(uint16_t)

//static uint8_t durationstring[6]="1.25H";
//static uint8_t powerstring[13]="loading. . .";
//static uint8_t IDstring[6]= "ID120";
/* PUBLIC VARIABLES ----------------------------------------------------------*/
__no_init uint32_t random;

config_t config;

struct realTime oldFrameTime;
struct realTime newFrameTime;

uint32_t tkFrameId;             // received T.K. frame Id
uint32_t lastTKFrameId = 0;
uint32_t lastFrameIdCorrection;
int8_t FRAME_DIFF_DELTA = 5;            // from T.K. sending out sync packet to beacon receiving it, there is a intrinsic latency. In frame pair format, it is Delta(FRAME_DIFF_DELTA, PHASE_DELTA)
                                        // both FRAME_DIFF_DELTA, PHASE_DELTA are experimental values that we have to measure and find out
                                        // PHASE_DELTA will be handled by Motive settings   
uint32_t successBeacons;
uint16_t routerAddr = 0;
uint8_t use_sync = 1;
uint16_t RfTxLevel;
uint32_t IMUdbgPrt = 0;
uint16_t frameOffset;
uint8_t bat_slot_numbers;

uint32_t last_bat_sent;

uint8_t test_imu_pkt_ctr = 0; // IMU packet ctr from IMU interrupt

uint16_t txTimeSlot = 40000;

uint8_t firstTime = 1;  // first need to initialize timer of time slot
uint16_t random_slot1 = 0xFFFF;
uint16_t random_slot2 = 0xFFFF;

volatile ledcheck led_select = select_none;

//User Interface Variables
uint8_t UI_IRLEDdata = 0;
uint8_t UI_BUTTONdata = 0;
/* EXTERNAL FUNCTION PROTOTYPES ---------------------------------------------*/
#if 0
uint8_t IMURegRd(uint8_t addr, HwSPI_TypeDef Sensor);
void IMURegWr(uint8_t addr, HwSPI_TypeDef Sensor, uint8_t val);
#endif
/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/
static void GetAverageImuData(uint16_t *pBuff);
static void InputDataIntoBuffer(uint8_t volatile *pImuData);

static void mygets(char *str);

static void SetConfig(uint16_t idx, uint32_t val, uint32_t pattern, uint8_t ledBits, uint8_t ledId);
static void PrintConfig(void);

static uint8_t RadioTxPktQueue(uint16_t dstAddr, uint8_t payloadSize, uint8_t *payload);

static void setHomeScreen(uint8_t stringname[],uint8_t addr[]);
static void setStringerGraphics(uint8_t led_flags);
static void resetUITimeout(void);
static void disableUITimeout(void);
/* PRIVATE FUNCTIONS ---------------------------------------------------------*/
/*******************************************************************************
* Description : Sets Outgoing Tx Timeslot
* Input       :
* Return      : -
*******************************************************************************/
void SetTimeSlot(void){

uint16_t TimeSlotTemp = 40000;

  if((config.rfTimeSlot > 1) && (config.rfTimeSlot < MAX_NUMBER_OF_TIMESLOTS))
  {
    TimeSlotTemp = TimeSlotVals[config.rfTimeSlot];
    //TimeSlotTemp -= FIVESAMPLEOFFSET;
    /*if(TimeSlotTemp > FIVESAMPLEOFFSET)
    {
        TimeSlotTemp -= FIVESAMPLEOFFSET;
    }
    else
    {
      TimeSlotTemp = FIVESAMPLEOFFSET -  (TimeSlotVals[config.rfTimeSlot]);
      TimeSlotTemp = 60000 - TimeSlotTemp;
    }*/
    txTimeSlot =TimeSlotTemp;
    firstTime = 1;           // reset timeslot
  }
  else
  {
    config.rfTimeSlot = 2;
    //recursive call
    SetTimeSlot();
  }
}


/*******************************************************************************
* Description : Get a line from STDIN
* Input       :
* Return      : -
*******************************************************************************/
static void mygets(char *str) {
    char *tmp = str;
    int c;
    SAVE_POINT
    do {
        while( (c = getchar()) == EOF ) {
            SAVE_POINT
            CoTickDelay(10);
            SAVE_POINT
            // reset tasksWDT
            tasksWDT |= 0x0002;
        }
        putchar(c);
        *tmp++ = (char) c;
    } while (c != '\r');
    SAVE_POINT

    putchar('\n');
    *(--tmp) = '\0';
}

#if 0
/*******************************************************************************
* Description : Write to an IMU Register
* Input       :
* Return      : -
*******************************************************************************/
void IMURegWr(uint8_t addr, HwSPI_TypeDef Sensor, uint8_t val) {
    SPI_I2S_ReceiveData(SPI_RADIO_IMU_SPI); // flush RXNE

    // write LSB
    HwSPISSAssert(Sensor);
    SPI_I2S_SendData(SPI_RADIO_IMU_SPI, ((addr << 8) | val) );
    //TK_BK_SPI_WAIT_RXRDY();
    while (SPI_I2S_GetFlagStatus(SPI_RADIO_IMU_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    SPI_I2S_ReceiveData(SPI_RADIO_IMU_SPI); // flush RXNE
    HwSPISSDeAssert(Sensor);
}
#endif

#define AIRCR_VECTKEY_MASK    ((u32)0x05FA0000)

/*******************************************************************************
* Function Name  : NVIC_GenerateSystemReset
* Description    : Generates a system reset.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_GenerateSystemReset(void)
{
  SCB->AIRCR = AIRCR_VECTKEY_MASK | (u32)0x04;
}

#if 0
/*******************************************************************************
* Description : Read from an IMU Register
* Input       :
* Return      : -
*******************************************************************************/
uint8_t IMURegRd(uint8_t addr, HwSPI_TypeDef Sensor) {

    uint16_t value;

    SPI_I2S_ReceiveData(SPI_RADIO_IMU_SPI); // flush RXNE

    // send register addr
    HwSPISSAssert(Sensor);
    // With Read Bit set 0x8000
    SPI_I2S_SendData(SPI_RADIO_IMU_SPI, 0x8000 | (addr << 8));
    // TK_BK_SPI_WAIT_RXRDY();
    while (SPI_I2S_GetFlagStatus(SPI_RADIO_IMU_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    value = SPI_I2S_ReceiveData(SPI_RADIO_IMU_SPI);
    HwSPISSDeAssert(Sensor);

    return (uint8_t)value;
}
#endif
/*******************************************************************************
* Description : Reads all the IMU buffered data and averages it
* Input       :
* Return      : - Averaged and packed data according to old ADI format
*                 BK_IMUData.
*******************************************************************************/
static void GetAverageImuData(uint16_t *pBuff)
{
  // Get Data from Gyro Buffer
  //x__disable_interrupt();
  pBuff[0] = ImuGyroBuffer.xSum;
  pBuff[1] = ImuGyroBuffer.ySum;
  pBuff[2] = ImuGyroBuffer.zSum;
  pBuff[3] = ImuAccelBuffer.xSum;
  pBuff[4] = ImuAccelBuffer.ySum;
  pBuff[5] = ImuAccelBuffer.zSum;
  //x__enable_interrupt();
}

/*******************************************************************************
* Description : Inputs the XYZ data into IMU Buffer
* Input       :   Buffer of Data and Buffer
* Return      : -
*******************************************************************************/
static void InputDataIntoBuffer(uint8_t volatile *pImuData)
{
  //x__disable_interrupt();
  //data scaling done on blkt software.
	ImuBuffer_t volatile *pImuBuff= &ImuGyroBuffer;
	pImuBuff->xSum = (*(uint16_t*)&pImuData[6]);
	pImuBuff->ySum = (*(uint16_t*)&pImuData[8]);
	pImuBuff->zSum = (*(uint16_t*)&pImuData[10]);
	pImuBuff->count = 1;
	
	pImuBuff = &ImuAccelBuffer;
	pImuBuff->xSum = *((uint16_t*)&pImuData[0]);
	pImuBuff->ySum = *((uint16_t*)&pImuData[2]);
	pImuBuff->zSum = *((uint16_t*)&pImuData[4]);
	pImuBuff->count = 1;
  //x__enable_interrupt();
}

/*******************************************************************************
* Description : Inputs the XYZ data into IMU Buffer
* Input       :   Buffer of Data and Buffer
* Return      : -
*******************************************************************************/
static uint8_t getButtonState(tButton *Button){
  uint32_t x  =HwButtonPressed(Button->ButtonID);
  switch(Button->current_state){
    case BUTTON_OFF:
      Button->debounce_count = (x!=0)? Button->debounce_count +1 : 0;
      Button->current_state = (Button->debounce_count >= BUTTON_DEBOUNCE)? BUTTON_PRESSED:BUTTON_OFF;
    break;
    case BUTTON_PRESSED:
      if(x==0){
        Button->debounce_count =0;
        Button->current_state = BUTTON_OFF;
        Button->event_flag = 1; 
        return 1;
      }
    break;
    //other states not implemented. to be implemented
  }
  return 0;
}
/*******************************************************************************
* Description : Set Configuration
* Input       :
* Return      : -
*******************************************************************************/
static void SetConfig(uint16_t idx, uint32_t val, uint32_t pattern, uint8_t ledBits, uint8_t ledId) {
    SAVE_POINT
    switch (idx) {
        /*
            0 : productID
            1 : serialNum
            2 : panId
            3 : mySrcAddr
            4 : tkDstAddr
            5 : ledOnOffs
            6 : ledOffOffs
            7 : ledDAC
            8 : rfChan
            9 : led0Id
            A : led1Id
            B : led2Id
            C : TestMode
        */
        case 0x0:
            config.productID = val;
            break;
        case 0x1:
            config.serialNum = val;
            break;
        case 0x2:
            config.panId = val;
            RadioSetPanIdShortAddr(config.panId, config.mySrcAddr);
            break;
        case 0x3:
            config.mySrcAddr = val;
            RadioSetPanIdShortAddr(config.panId, config.mySrcAddr);
            CoSetFlag(flagUSBdata_UI);
            SAVE_POINT
            break;
        case 0x4:
            config.routerDstAddr = val;
            break;
        case 0x5:
            config.ledOnOffs = val;
            break;
        case 0x6:
            config.ledOffOffs = val;
            break;
        case 0x7:
            config.ledDAC = val;
            DAC_SetChannel2Data(DAC_Align_12b_R, val);
            break;
        case 0x8:
            if(val >= OLD_RF_CHANNEL_MIN && val <= OLD_RF_CHANNEL_MAX)
            {
              config.rfChan = val;
              SAVE_POINT
              RadioSetRFChan(val);
            }
            break;
        case 0x9:
            config.led0Id = val;
            break;
        case 0xA:
            config.led1Id = val;
            break;
        case 0xB:
            config.led2Id = val;
            break;
        case 0xC:
          config.TestMode = val;
          break;
        case 0xD:
          config.rfTimeSlot = val;
          SetTimeSlot();
          break;
       case 0xE:
          config.TxLevel = val;
          SAVE_POINT
          RadioSetRFLevel(config.TxLevel);
          SAVE_POINT
          break;
    case 0x10:
          config.radioPacketFlags = val;
          break;
    case 0x13:
          config.led0IdPattern = pattern;
          config.led0Index = val;
          config.frameBits = ledBits;
          config.led0Id = ledId;
         break;
    case 0x14:
          config.led1IdPattern = pattern;
          config.led1Index = val;
          config.frameBits = ledBits;
          config.led1Id = ledId;
          break;
    case 0x15:
          config.led2IdPattern = pattern;
          config.led2Index = val;
          config.frameBits = ledBits;
          config.led2Id = ledId;
          break;
    case 22:
            if(val >= RF_CHANNEL_MIN && val <= RF_CHANNEL_MAX)
            {
              config.rfChan = val;
              SAVE_POINT
              RadioSetRFChan(val);
            }
            break;
#ifndef OLD_CONFIG
    case 24:
          if (val >= DEBOUNCE_MIN && val <= DEBOUNCE_MAX) {
            config.debounce_time = val;
          }
          break;
    case 25:
          if (val >= DBLCLICK_MIN_TIME && val <= DBLCLICK_MAX_TIME) {
            config.doubleclick_time = val;
          }
          break;
#endif
    default:
            TRACE("**ERROR** Unrecognized Configuration Setting\n\r");
            break;

    }
}
/*******************************************************************************
* Description : Print Configuration
* Input       : -
* Return      : -
*******************************************************************************/
static void PrintConfig(void) {
    SAVE_POINT


#ifdef _DEBUG
    TRACE("\n\r FIRMWARE VERSION: " STRINGIFY(THIS_MAJOR) "." STRINGIFY(THIS_MINOR) "." STRINGIFY(THIS_PATCH) "." STRINGIFY(THIS_REVISION) " BC DEBUG \n\r\n\r");
#else
    TRACE("\n\r FIRMWARE VERSION: " STRINGIFY(THIS_MAJOR) "." STRINGIFY(THIS_MINOR) "." STRINGIFY(THIS_PATCH) "." STRINGIFY(THIS_REVISION) " BC RELEASE \n\r\n\r");
#endif
    TRACE(" "__DATE__" : "__TIME__" \n\n\r");
    TRACE("ARM Serial Number 0x%X,0x%X,0x%X \n\n\r",ARM_proc_SN.a,ARM_proc_SN.b, ARM_proc_SN.c);

    if(IMUPresent){
       TRACE("IMU Present\n\n\r");
    }else{
      TRACE(" No IMU Installed\n\n\r");
    }

TRACE("[0] productID   : %X\n\r", config.productID);
TRACE("[1] serialNum   : %X\n\r", config.serialNum);
TRACE("[2] panId       : %04X\n\r", config.panId);
TRACE("[3] mySrcAddr   : %04X\n\r", config.mySrcAddr);
TRACE("[4] routerDstAddr   : %04X\n\r", config.routerDstAddr);
TRACE("[5] ledOnOffs   : %X\n\r", config.ledOnOffs);
TRACE("[6] ledOffOffs  : %X\n\r", config.ledOffOffs);
TRACE("[7] ledDAC      : %X\n\r", config.ledDAC);
TRACE("[8] rfChan      : %02X\n\r", config.rfChan);
TRACE("[?] rfTimeSlot  : %02X\n\r", config.rfTimeSlot);
TRACE("[9] led0Id      : %02X\n\r", config.led0Id);
TRACE("[A] led1Id      : %02X\n\r", config.led1Id);
TRACE("[B] led2Id      : %02X\n\r", config.led2Id);
TRACE("[C] TestMode    : %d\n\r", config.TestMode);
TRACE("[J] led0IdPattern : %08X\n\r", config.led0IdPattern);
TRACE("[K] led1IdPattern : %08X\n\r", config.led1IdPattern);
TRACE("[L] led2IdPattern : %08X\n\r", config.led2IdPattern);
TRACE("[M] led0Index : %08X\n\r", config.led0Index);
TRACE("[N] led1Index : %08X\n\r", config.led1Index);
TRACE("[O] led2Index : %08X\n\r", config.led2Index);
TRACE("[Y] frameBits   : %d\n\r", config.frameBits);
TRACE("[Z] Tx RF Level : %X\n\r", TxAmpValues[config.TxLevel]);
TRACE("[X] Radio: %s\n\r", radio_off == 0? "on":"off");

  TRACE("Built on "__DATE__" "__TIME__"\n\r");
  TRACE("Flags = 0x%08X \r\n", config.flags);
  TRACE("Timer adjust: %d\n\r", config.time_adjust);
  TRACE("Timekeeper sync: %s\n\r", use_sync?"yes":"no");
  TRACE("No sync timeout sec: %d\n\r", config.frameCountNoSync/10);
#ifndef OLD_CONFIG
  TRACE("FrameId 24 bits wrap around: %s\n\r", (config.flags & FLAG_FRAMEID_24BITS)?"yes":"no");
  TRACE("Debounce value[1-9]: %u, doubleclick value[1-9]: %u\r\n", config.debounce_time,
        config.doubleclick_time);
#endif
}


uint32_t txCalls;
uint32_t txCalls2;
StatusType setRadioTx;

/*******************************************************************************
* Description : Add Packet to Radio Transmit Queue
* Input       : -
* Return      : 0 if buffer full otherwise just returns payloadSize back
*******************************************************************************/
static uint8_t RadioTxPktQueue(uint16_t dstAddr, uint8_t payloadSize, uint8_t *payload) {
    // critical section.
    SAVE_POINT
    __disable_interrupt();
    if ( ((inIdx + 1) % TX_PKT_QUEUE_SIZE) == outIdx )
    {
        __enable_interrupt();
          SAVE_POINT
          return 0; // queue is full => ignore request
    }
    txCalls++;
    txPktQueue[inIdx].dstAddr = dstAddr;
    txPktQueue[inIdx].payloadSize = payloadSize;
    memcpy((void *)txPktQueue[inIdx].payload, (void *)payload, payloadSize);
    inIdx = (inIdx + 1) % TX_PKT_QUEUE_SIZE;
    inidxcheck++;
    __enable_interrupt();

    setRadioTx = CoSetFlag(flagRadioTxReq);  // Wake up TX task if it is waiting for sending
    txCalls2++;
    return payloadSize;
}

uint32_t calls;
uint32_t pushed_times;

StatusType semAllowPostTask;
uint8_t semAllow = 0;

/* PUBLIC FUNCTIONS ----------------------------------------------------------*/
extern uint32_t secs;

/*******************************************************************************
* Description    : [Task] Process and Send each new IMU data sample
* Input          :
* Return         :
*******************************************************************************/
void Task1(void* pdata){
    static uint8_t seqNum = 0;
    static uint16_t *pBuf;
    static uint16_t i = 0;
    static struct Beacon_Data_pkt *pBeacon_Data_pkt;
    static uint8_t last_button_state = 0;
    static uint16_t battery_minutes = 0;

    while (1) {
        RELOAD_WATCHDOG
        SAVE_POINT

        pBeacon_Data_pkt = (struct Beacon_Data_pkt*)txBuf;

        pBeacon_Data_pkt->BK_Preamble.button_pr = (HwButtonPressed(BUTTON2) ? 0x01 : 0x00) |
                    (HwButtonPressed(BUTTON1) ? 0x02 : 0x00) ;
        pBeacon_Data_pkt->BK_Preamble.Seq_Num=  seqNum++ ;
        pBeacon_Data_pkt->BK_Preamble.BeaconRSSI= beaconRSSI;
        pBeacon_Data_pkt->BK_Preamble.IRLed0= config.led0Id;
        pBeacon_Data_pkt->BK_Preamble.IRLed1= config.led1Id;
        pBeacon_Data_pkt->BK_Preamble.IRLed2= config.led2Id;
        pBeacon_Data_pkt->BK_Preamble.Battery_lev = BattUnion.BatteryLevel[1];
        pBeacon_Data_pkt->BK_Preamble.SyncFrameIMU=frameIdAtSync;
        pBeacon_Data_pkt->BK_Preamble.MsTimerIMU=MsTimerAtSync;
        pBeacon_Data_pkt->BK_Preamble.IMUPktNum=IMUPktNumAtSync;

        SAVE_POINT
        
        static uint8_t last_test_imu_pkt_ctr = 255;     // to detect if a IMU packet was sent.  
        // Five IMU samples per data Packet
        i  = 0;
        while (i < NUM_OF_IMU_PKTS_IN_RF_PKT)
        {
             SAVE_POINT
             pBuf = (uint16_t*) &pBeacon_Data_pkt->BeaconIMUData[i].gyroscopeX;
             CoSetFlag(flagIMU_read);
             CoWaitForSingleFlag(flagIMUDataReady, 0);

             SAVE_POINT
             GetAverageImuData(pBuf);

            last_test_imu_pkt_ctr = test_imu_pkt_ctr;
            pBeacon_Data_pkt->BeaconIMUData[i].Timestamp = test_imu_pkt_ctr;

            ++i;      // next packet
        }

        SAVE_POINT  
        CoPendSem(semIMUAllow, 0);
        SAVE_POINT
        calls++;
        //HwGPOToggle(GPO_TP50);// test
          uint8_t pushed = 0;

        if (IMUdbgPrt) {         // It is for USB output now
            __writeIMU((unsigned char*) pBeacon_Data_pkt, sizeof(struct Beacon_Data_pkt));
        }
        else {

          // conditional send based on radio packets flags;
          if ((config.radioPacketFlags & RADIOPACKET_IMU)
            || ((config.radioPacketFlags & RADIOPACKET_BUTTONPRESS) && (last_button_state != pBeacon_Data_pkt->BK_Preamble.button_pr))
            || ((config.radioPacketFlags & RADIOPACKET_BATTERY) && (battery_minutes != (secs/600)))
			) {
              if(!(pushed = RadioTxPktQueue(routerAddr, sizeof(struct Beacon_Data_pkt) , txBuf))) {
                if (config.flags & FLAG_TRACE_IMU_QUEUE_FULL) {
                    TRACE("ERROR! Tx Buffer full\n\r");
                }
                SAVE_LINE
              } else {
                pushed_times++;
                SAVE_LINE
              }
             SAVE_FUNC
          }
        }
          if (!pushed || radio_off) {
            semAllowPostTask = CoPostSem(semIMUAllow);
            semAllow = 2;
          }

          last_button_state = pBeacon_Data_pkt->BK_Preamble.button_pr;
          battery_minutes =  secs/600;
          // reset tasksWDT
          tasksWDT |= 0x0001;
    }
}
uint32_t now_sec;
uint32_t tim;
uint32_t old_sec;
uint32_t old_tim;

#ifndef STM3210C_EVAL
extern OS_EventID semRFRxFrames;
StatusType task2StatusType;
uint32_t task2enter;
uint8_t rx_reload;
uint32_t rxReloaded;
uint32_t task2_errors;
extern uint8_t rxCount;

struct realTime radioRxStart;
struct realTime radioRxEnd;
struct realTime radioRxWait;

uint32_t rxTotalRcvd;
uint32_t rxNotEmpty;
extern uint32_t rxPackets;
uint32_t lastTime;
uint32_t span;
uint16_t tim4_phase;

int32_t acc_time_adjust;
uint8_t  acc_adjust_count;
uint8_t acc_done = 0;

int16_t accs[32];
struct realTime lastFrameTime;
extern struct realTime frameTime;
uint32_t savedFrameIdAtSync;
uint16_t newTim3Phase, oldTim3Phase, lastTim3Phase;
extern uint32_t trace_irq;

extern int frameIdInced;

/*******************************************************************************
* Description    : [Task] Process Incoming Radio Packets
* Input          :
* Return         :
*******************************************************************************/
void TaskRadioRx(void* pdata) {

    static rxPkt_t *pRxPkt;
    static uint8_t lastFrameClock = 0;

    while (1) {
      RELOAD_WATCHDOG
      SAVE_POINT
      radioRxWait.sec = sec;
      radioRxWait.uSec = TIM1->CNT;

      __disable_interrupt();
      frameIdFlag = 1;
      __enable_interrupt();
      
      task2StatusType = CoPendSem(semRFRxFrames, 0);
      SAVE_POINT
      radioRxStart.sec = sec;
      radioRxStart.uSec = TIM1->CNT;
      assert(task2StatusType == E_OK);

      SAVE_POINT
      RadioIMU_WaitGrabSPI();
      SAVE_POINT
      if (rxFIFOError) {
        rxFIFOError = 0;
        ProcessRXError();
        pRxPkt = NULL;
      } else {
        pRxPkt = RadioRxPkt();
        rxTotalRcvd++;
      }
      SAVE_POINT

      radioRxEnd.sec = sec;
      radioRxEnd.uSec = TIM1->CNT;
      RadioIMU_ReleaseSPI();
      if (pRxPkt == NULL) {
        if (!radio_off) {
          TRACE("bc=%d NULL packet, rxCount=%u @sec=%d\r\n", config.mySrcAddr, rxCount, sec);
        }
        continue;
      }

      SAVE_LINE
      rxNotEmpty++;
      // Check for Beacon frame types from the TimeKeeper.  Ignore other data packets
      // from beacons
      if (!((pRxPkt->fcf0 & 0x07) == 0 && pRxPkt->panId == config.panId
          && (pRxPkt->destAddr == 0xFFFF))) { // FCF[2:0] = 802.15.4 Beacon Frame Type
        continue;          
      }
      //additional check may be on source address
      /* RF Sync (Beacon) Packet */
#ifdef NEW_BEACON
      /* RF Sync (Beacon) Packet */
      struct BeaconOldStruct *beacon = (struct BeaconOldStruct*) pRxPkt->payload;
      if (pRxPkt->payloadSize == sizeof(struct BeaconOldStruct)) {
        //check for new structure
        // uint32_t frameId
        // uint8_t  0xA5 //magic for this packet version
        // uint8_t sec;  // consecutive sec, increments every sec
        // uint8_t frameClock; //100, 120, 180, 240 support
        // uint8_t crc8

        if (beacon->magic != 'BT') {
          notValids++;
          TRACE("Beacon received: unknown magic 0x%04X\n\r", beacon->magic);
          continue;
        }
        uint8_t check = crc8(pRxPkt->payload, sizeof(struct BeaconOldStruct) - 1);
        if (check != beacon->crc8) {
          notValids++;
          TRACE("Beacon received: CRC8 failed \r\n");
          continue;
        }
        switch (beacon->frameClock) {
        case FRAME_CLOCK_100:
        case FRAME_CLOCK_120:
        case FRAME_CLOCK_180:
        case FRAME_CLOCK_240:
          break;
        default:
          TRACE("Beacon received: Unsupported frame clock %d\n\r", beacon->frameClock);
          notValids++;
          continue;
        }
        static uint32_t lastFrameId = 0;
        static uint8_t lastTick = 0;
        static uint8_t changeFrameClock = 0;
        uint8_t valid = 0;

        if (lastFrameClock == beacon->frameClock ) {
          if (beacon->tick > lastTick) {
            uint32_t diff = beacon->tick - lastTick;
            diff *= lastFrameClock;
            if ((beacon->frameId - lastFrameId) == diff) {
              valid = 1;
              if (changeFrameClock) {
                // TODO
                TRACE("Beacon: changing frameClock to %u\r\n", lastFrameClock);
              }
              changeFrameClock = 0;
            }
          }
        } else {
          changeFrameClock = 1;
          if (lastFrameId != 0) {
            changeClocks++;
          }
        }
        int32_t diffFrameId = beacon->frameId - lastFrameId;
        uint16_t diffTick = (((uint16_t) beacon->tick + 256) - lastTick)%256;
        /*Here seems is a problem:
          When signal weak, it may catch lost sync, then before ticks
          run off 256 (less then 256 sec later) it catches another sync,
          by that time frameId may be already off by 1, if it receives consecutive sync
          it may recover, but if not, it may stack with 1 frame offset.
        Solution: limit valid span for consecutive syncs to 10 sec,
         use diffSec*/
        uint32_t diffSec = diffFrameId/beacon->frameClock;
        if ((diffTick * beacon->frameClock) == diffFrameId && diffSec < 10) {
          valid = 1;
          Valids++;
        }
        if (!valid && lastFrameId != 0) {
          TRACE("New bcn id:%u>%u tick:%u>%u clk:%u>%u\r\n",
          lastFrameId, beacon->frameId, lastTick, beacon->tick,
          lastFrameClock, beacon->frameClock);
          newbcn++;
        }

        if (lastFrameId != beacon->frameId ||
          lastFrameClock != beacon->frameClock ||
          lastTick != beacon->tick) {
          lastFrameId = beacon->frameId;
          lastFrameClock = beacon->frameClock;
          lastTick = beacon->tick;
        }
        if (!valid) {
          continue;
        }
      } else /*if (pRxPkt->payloadSize != 4)*/{
        continue;
      }

#endif
      if (use_sync) {
        trace_irq = 1;
        tim4_phase = TIM4->CCR1;
      }

      if (firstTime == 1) {
        TIM_SetAutoreload(TIM3, TIM3_AUTORELOAD);
        TIM_SetAutoreload(TIM2, TIM_AUTORELOAD);
        TIM_SetCompare2(TIM3, TIM3_AUTORELOAD - (config.ledOnOffs >> 1));
        TIM_SetCompare3(TIM3, TIM3_AUTORELOAD - (config.ledOffOffs >> 1));
        // Instead of modulus maybe boundry check!
        TIM_SetCompare3(TIM2, ( (uint16_t)(txTimeSlot) ) );
        // Second 50ms (or half of a TK Tx Beacon interval later) - 3000 (5ms)
        // 5ms is the combination of the first two time slots which don't get a
        // Edit -- Can't subtract 5ms off second because it takes 47.5ms before
        // the next set of 5 IMU packets are ready.  This is a waste of BW.
        // maybe could use it some other way.
        // second transmission per frame.
        TIM_SetCompare4(TIM2, ( (uint16_t)((((TIM_AUTORELOAD + 1) >> 1) + txTimeSlot) - 0) ) );
        firstTime = 0;
      }

      // synchronize local to remote FrameIDs
      if (!(*(uint32_t*)pRxPkt->payload)) {
        continue;
      }
      
      static uint32_t newTKFrameId = 0; newTKFrameId = *(uint32_t*)pRxPkt->payload;

      static int skipPackets = 0;    // TODO! this is a temporary solution. There must be an issue in T.K. that sends out frame sequency like this 100, 200, 200, 300, 500, 600 
      if (skipPackets > 0) {
        TRACE("bc=%d skip TK frame %d detected tick=%d. AtSync(%d, %d)", config.mySrcAddr, newTKFrameId, beacon->tick, frameIdAtSync, tim3PhaseAtSync);
        --skipPackets;
        continue;
      }
      
      if (newTKFrameId == tkFrameId) {
        TRACE("bc=%d stale TK frame %d detected tick=%d. AtSync(%d, %d)", config.mySrcAddr, newTKFrameId, beacon->tick, frameIdAtSync, tim3PhaseAtSync);
        skipPackets = 3;
        continue;       // ignore stale FrameIDs
      } 
      
      tkFrameId = newTKFrameId;
          
      // A frameId pair (frameId, phase) represents time: frameId * (TIM3_AUTORELOAD + 1) + (TIM3_AUTORELOAD - phase)
      // Timekeeper sends out its frame Id at phase TIM3_AUTORELOAD --> (tkFrameId, TIM3_AUTORELOAD - 0)              // coundown perspective
      // While beacon local frame is (frameIdAtSync, tim3PhaseAtSync)
      // new time formula: (Fa - Fs + Fr) + Delta  ==> (Frame pair being adjusted - Frame pair at sync + Remote frame pair) + Delta
      // or, formula: (frameId - frameIdAtSync + tkFrameId + frameAdjust) * (TIM_AUTORELOAD + 1) - tim3_phase + tim3PhaseAtSync;  
      // enter critical section make sure that TIM3 is NOT processing frameId increment or TIM3 is about to underflow  
      static uint32_t tim3_phase;      
      while(1) {
        __disable_interrupt();
        
        if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) {
          __enable_interrupt();
          HwWait(1);
          continue;
        }
        
        tim3_phase = TIM3->CNT;
        if (tim3_phase < 10) {  
          __enable_interrupt();   
          HwWait(1);
          continue;
        }
        
        break;
      }
                          
      static uint32_t frameIdAdjusting = 0;     
      static int32_t frameDiff = 0; frameDiff = frameId - frameIdAdjusting; 
      frameIdAdjusting = frameId;   
      static uint16_t adjustedPhase = 0;
      
      frameId = frameIdAdjusting + ((int32_t)tkFrameId - frameIdAtSync) + FRAME_DIFF_DELTA;     // frameAdjust is a constant of this system, experimental value              
      
      if (tim3_phase > tim3PhaseAtSync) {
        --frameId;
        adjustedPhase = 0 - (tim3PhaseAtSync - tim3_phase);
      } else {
        adjustedPhase = TIM3_AUTORELOAD - (tim3PhaseAtSync - tim3_phase);
      } 
                      
      TIM_SetCounter(TIM3,(uint16_t)adjustedPhase);
      TIM_SetCounter(TIM2,(uint16_t)adjustedPhase);

      // pair subtraction and turns to phase only for measurement 
      static uint32_t adjustedFrameId = 0; adjustedFrameId = frameId;
      static int32_t phaseDiff = 0; phaseDiff = ((int)frameId - frameIdAdjusting) * (TIM3_AUTORELOAD + 1) + (TIM3_AUTORELOAD - tim3_phase) - (TIM3_AUTORELOAD - adjustedPhase);             // could overflow but should get back to normal if beacon synced with T.K. 
      static int32_t avgPhaseDiffPerFrame = 0xFFFFFF;
      static uint32_t oldARR = 0; oldARR = TIM3->ARR;
      if (frameDiff > 0) {
        avgPhaseDiffPerFrame = phaseDiff / frameDiff;
        uint16_t newARR = oldARR + avgPhaseDiffPerFrame;
        if (newARR!= 0 && abs(newARR - TIM3_AUTORELOAD) <= 10) {         // greater than 10 / frame, crystal of the beacon is not accurate enough and not suitable for this application
          TIM_SetAutoreload(TIM3, newARR);
          TIM_SetAutoreload(TIM2, newARR);
        } else {
          avgPhaseDiffPerFrame = 99;      // set to max
        }  
      } else {
        avgPhaseDiffPerFrame = 99;      // set to max
      }

#ifdef CIRCULAR_LOG
      W_LOG(CoGetOSTime(), LOG_TYPE_FRAMEADJUST, tkFrameId);
      W_LOG(CoGetOSTime(), LOG_TYPE_CAPTURE, tim3_phase);
      W_LOG(CoGetOSTime(), LOG_TYPE_FRAMEDIFF, phaseDiff);
#endif
      // after inerrupt enabled, these value can be modified at once. So save a copy for this function call for tracing purpose
      static int savedFrameIdInced = 0; savedFrameIdInced = frameIdInced;
      static uint32_t saved_trace_irq = 0; saved_trace_irq = trace_irq;
      __enable_interrupt();

      if (config.flags & FLAG_TRACE_SYNC) {
        //TRACE(" tim3_phase=%u inced=%u, phase diff=%d avg phase diff=%d\r\n", tim3_phase, savedFrameIdInced, phaseDiff, avgPhaseDiffPerFrame);
        TRACE("bc=%d tk(%d,,%3d) AtSync(%d, %d) Adjusting(%d, %d) adjusted(%d, %d) diff(%d, %d, arr%u)\r\n", config.mySrcAddr, tkFrameId, beacon->tick, frameIdAtSync, tim3PhaseAtSync, frameIdAdjusting, tim3_phase, adjustedFrameId, adjustedPhase, phaseDiff, avgPhaseDiffPerFrame, oldARR);
      }
      
      if (config.flags & FLAG_DEBUG) {
        uint32_t tr = saved_trace_irq;
        TRACE("trace_irq=%d at tim3Phase=%d\r\n", tr, tim3_phase);
        TRACE("tkFrameId=%d frameIdAtSync=%d tim3PhaseAtSync=%d localFrameId=%d adjustedPhase=%d\r\n", tkFrameId, frameIdAtSync, tim3PhaseAtSync, frameId);
      }
      
      if (config.flags & FLAG_TRACE_ADJUST) {
        TRACE("adjust=%d tim3_phase=%u tim4_phase=%u @%d.%d\r\n",
              config.time_adjust,
              tim3_phase, tim4_phase, sec, TIM1->CNT);
      }
      
      if ((sec - lastTime) > span) {
        span = sec - lastTime;
        TRACE("No TK sync for %d sec @ %d\r\n", span, sec);
      }
      lastTime = sec;
      frameOffset = TIM1->CNT;
      successBeacons++;

      // record RSSI of Sync Packet to relay back to TK
      beaconRSSI =  0x5A; // temp change to store signature. // pRxPkt->rssi;

      // allow up to 3 tx pkts per sync pkt
      beaconInSync++;
      if(beaconInSync > 3)
      {
        beaconInSync = 3;
      }
      // Issue TODO - if Beacon out of syncronization,
      // it keeps trying to send (Task 6), may not receive
      // anything, as this task has lower priority
      // CoAwakeTask(task1Id);
      // CoAwakeTask(taskRadioTxId);
    }   //of while
}


/*******************************************************************************
* Description    : [Task] RF Chan Scan, then Monitor GPO_PWRON Power Switch
* Input          :
* Return         :
*******************************************************************************/
void Task3(void* pdata){
  static StatusType result;
  static uint16_t holdCount = 0;
  rfChan = config.rfChan;
  static uint8_t pwrbtnflag=0;
  static uint8_t task3event=0;

  //initialize routine, debounce pwr button. 
  while(HwGPIState(GPI_SW_PWR)||HwGPIState(GPI_USB_VBUS)){
    RELOAD_WATCHDOG
    SAVE_POINT
    if(holdCount++>=4) break;
    CoTickDelay(50);
  }
  if(holdCount<4 || BattUnion.BatteryLevel[1]<0xBB)  //device not held or low battery
     {                 //and if device not connected external source
    CoSchedLock();                                    //shut down sequence
    HwGPOLow(GPO_RF_EN);
    HwGPOLow(GPO_VBATT_ADC_EN);
    HwGPOInitOC(GPO_USB_VBUS); 
    HwGPOLow(GPO_USB_VBUS);  
    SAVE_POINT
    HwGPOLow(GPO_PWRON); 
    while (1);
  }
  UI_StateMachine = UI_POWERON_STATE; //make sure statemachine in correct state prior to initializing UI
  CoAwakeTask(taskUIId);    

  // NO RF Scan => send IMU data to TK address stored in FLASH Config   
  IMUInit();
  halted = (IMUPresent==1)? 1:0;
  routerAddr = config.routerDstAddr;
  SetTimeSlot();
  /* we're now in business */
  TRACE("RF CHAN: %d\n\r", rfChan);

  ButtonA = (tButton) {.ButtonID = BUTTON1, .debounce_count = 0, .current_state = BUTTON_OFF, .event_flag = 0};
  ButtonB = (tButton) {.ButtonID = BUTTON2, .debounce_count = 0, .current_state = BUTTON_OFF, .event_flag = 0};
  
  CoAwakeTask(task1Id);  // start imu process
  CoAwakeTask(taskRadioRxId);
CoAwakeTask(taskIMUGId); 
  CoPendSem(semInkUI,0);  //hack: wait till the home screen has been loaded.
  CoPostSem(semInkUI);
  while(HwGPIState(GPI_SW_PWR)){
    CoTickDelay(50);  //wait for user to release pwr button before entering to main task3 loop. don't want to jump directly to menu screen.
  }
  holdCount=0;
	while (1) {
		SAVE_POINT
		CoTickDelay(100);
		SAVE_POINT
		RELOAD_WATCHDOG
#ifndef DISABLE_PWR_SW
    if(HwGPIState(GPI_SW_PWR)||((BattUnion.BatteryLevel[1])<=0xBB)){
      if(holdCount++>30 || (BattUnion.BatteryLevel[1]<=0xBB)){
        CoPendSem(semInkUI,0);
        GFX_FillBuffer(black);
        GFX_DrawBitMap(UI_LOGO_COORD, logo, logo_dimensions);
        setFont(smallfont);
        setCursor(UIPOWER_TEXT_COORD);
        GFX_Print("Turning Off", strlen("Turning Off"),centeralign, black, gray_l);
        WaitGrabEInk();
        update(fullrefresh);
        ReleaseEInk();
        GFX_FillBuffer(black);
        WaitGrabEInk();
        update(fullrefresh);
        ReleaseEInk();
        CoSchedLock();                      //shut down sequence
        HwGPOLow(GPO_RF_EN);
        HwGPOLow(GPO_VBATT_ADC_EN);
        HwGPOInitOC(GPO_USB_VBUS); 
        HwGPOLow(GPO_USB_VBUS);  
        HwGPOLow(GPO_PWRON) ; 
        while (1);
      }
    }
    else{
      //if(holdCount == BUTTON_DEBOUNCE){
      if(holdCount>=BUTTON_DEBOUNCE){
        //X button was pressed
        task3event = 1;
        pwrbtnflag=1;
      }
      holdCount=0;
    }
#endif // disable power switch

    task3event |= getButtonState(&ButtonA);
    task3event |= getButtonState(&ButtonB);

    if(task3event){
      if(UI_StateMachine==UI_HOME_STATE){
        //1. notify task1 if buttons A or B were pressed
        //2. if button X was pressed, notify UI
        if(pwrbtnflag && CoAcceptSem(semInkUI)==E_OK){
          task3event=0;
          pwrbtnflag=0;
          UI_BUTTONdata |= UI_PWRBTNFLAG;
          CoSetFlag(flagBtn_UI);
          CoPostSem(semInkUI);
        }
        if (config.flags & FLAG_TRACE_ASYNC) {
          if(ButtonA.event_flag){ButtonA.event_flag=0; TRACE("Btn A PRESS\r\n");}
          if(ButtonB.event_flag){ButtonB.event_flag=0; TRACE("Btn B PRESS\r\n");}
        }
      }
      else if(CoAcceptSem(semInkUI)==E_OK){   //any other state. 
        if(pwrbtnflag) UI_BUTTONdata |= UI_PWRBTNFLAG;
        if(ButtonA.event_flag) UI_BUTTONdata |= UI_BTNAFLAG;
        if(ButtonB.event_flag) UI_BUTTONdata |= UI_BTNBFLAG;
        pwrbtnflag=0; ButtonA.event_flag=0;  ButtonB.event_flag=0; task3event =0;
        CoSetFlag(flagBtn_UI);
        CoPostSem(semInkUI);
      }
      else{
        if(config.flags & FLAG_TRACE_UI_STATEMACHINE) TRACE("semaphore blocked\r\n");
      }
    }
  }
}

#endif

/*******************************************************************************
* Description : Write Configuration to FLASH
* Input       : -
* Return      : -
*******************************************************************************/
#define FLASH_NO_ADDRESS_CHECK

uint32_t WritePage(tPage *page) {

  uint8_t *pointer = (uint8_t*) page->address;
  uint32_t result = 0;
  uint16_t cycles = (page->count + 3)/4;

  FLASH_Unlock();

  do {
#ifndef FLASH_NO_ADDRESS_CHECK
    if (page->address < APP_UPLOAD_ADDRESS || page->address > 0x803FFFF) {
      page->count = 0;
      result = ERROR_FLASH_ADDRESS;
      break;
    }
#endif
    if ((result = FLASH_ErasePage(page->address)) != FLASH_COMPLETE) {
      page->count = 0;
      break;
    }

    result = 0;
    for (uint16_t i = 0; i < cycles; i++) {
      uint32_t offset = i << 2;
      uint32_t word = page->data[offset + 3] << 24;
      word |= page->data[offset + 2] << 16;
      word |= page->data[offset + 1] << 8;
      word |= page->data[offset];

        if ((result = FLASH_ProgramWord(page->address + offset, word)) != FLASH_COMPLETE) {
            page->count = offset;
            break;
        }
    }
    result = 0;
    // verification
    for (uint16_t i = 0; i < page->count; i++) {
      if (*(pointer) != page->data[i]) {
        result = ERROR_FLASH_VERIFICATION;
        page->count = i;
        break;
      }
      pointer++;
    }

  } while (0);

  FLASH_Lock();

  return result;
}

#if 1

uint32_t ReadPage(tPage *page) {

  uint8_t *pointer = (uint8_t *) page->address;
  uint32_t result = 0;


  do {
#ifndef FLASH_NO_ADDRESS_CHECK
    if (page->address < 0x8000000 || page->address > 0x803FFFF) {
      page->count = 0;
      result = ERROR_FLASH_ADDRESS;
      break;
    }
#endif
    // verification
    for (uint16_t i = 0; i < page->count; i++) {
      page->data[i] = *pointer;
      pointer++;
    }

  } while (0);

  return result;

}
#endif

/**
  * @brief  Download a file via serial port
  * @param  None
  * @retval None
  */
uint8_t tab_1024[1024];

static tPage page = {FLASH_PROD_AREA};
//static uint32_t CurrentUploadAddress = APP_UPLOAD_ADDRESS;
static uint16_t CurrentDataPointer = 0x00;
//static uint8_t mem_data[PAGE_SIZE];
static uint16_t packet = 0;

static struct PacketHeader *header;
uint32_t SerialDownload(const char* decoded, uint16_t len, uint8_t type)
{
  struct FirmwarePacketHeader *frmHeader;

  uint8_t copy_len = 0;

  uint32_t ret = 0;
  // Data comes in format:
  //U 0 0 DATA\r\n
  // so, DATA starts from 6th index
  do {
    if (len < (sizeof(struct FirmwarePacketHeader))) {
      ret = ERROR_WRONG_PACKET;
      break;
    }
    frmHeader = (struct FirmwarePacketHeader *) (decoded);
    copy_len = header->size - /*7*/ (sizeof(struct PacketHeader) + sizeof(struct FirmwarePacketHeader));
    if (
        ((type == DEV_CMD_SET_PROD_AREA) &&
        (copy_len > 128 || (copy_len + CurrentDataPointer) > (FLASH_PROD_AREA + FLASH_PROD_AREA_SIZE)))
        ||
        frmHeader->index != packet) {
      ret = ERROR_WRONG_PACKET;
      break;
    }
    if (copy_len) {
      memcpy(&page.data[CurrentDataPointer], (decoded + /*4*/ sizeof(struct FirmwarePacketHeader)), copy_len);
      CurrentDataPointer += copy_len;
      page.count += copy_len;
      if ((frmHeader->index + 1) == frmHeader->count || CurrentDataPointer == sizeof(page.data)) {
        //time to write
        WritePage(&page);
        page.address = FLASH_PROD_AREA;
        CurrentDataPointer = 0;
        page.count = 0;
        if ((frmHeader->index + 1) == frmHeader->count) {
          // finished
          page.address = FLASH_PROD_AREA;
        }
      }
    }
    ret = frmHeader->index << 16;
  } while (0);
  // reset for error
  if (ret & 0xFFFF) {
    CurrentDataPointer = 0;
    page.address = FLASH_PROD_AREA;
    page.count = 0;
    packet = 0;
  } else {
    packet++;
  }
  return ret;
}

tPage page;
uint16_t use_tim3_phase = 0;

/*******************************************************************************
* Description    : [Task] Implements Simple Text Command Console
* Input          :
* Return         :
*******************************************************************************/
void TaskConfig(void* pdata) {
  static char buff[512] = "";
  char *s = buff;
//    static int i;
  static char cmd;
  static int addr;
  static int32_t value;

  while (1) {
    SAVE_POINT
    // get a line of values from stdin
    s = buff;
    mygets(s); 	
    if (ValidateCommandLine(s, strlen(s)) == 0) {
      TRACE("Incorrect packet\r\n");
      continue;
    }
            
    SAVE_POINT
    RELOAD_WATCHDOG
    // parse the line of hexadecimal values
    //i = sscanf(s, "%c %d %d", &cmd, &addr, &value);
    //if (i < 1) continue;
    cmd = *s;
    switch (cmd) {
      case '2':
        RadioPrint2520Registers(addr);
        SAVE_POINT
        break;
      case 'd': // Display configuration
        //if (i != 1) continue;
        PrintConfig();
        SAVE_POINT
        break;
      case '1':
        // Test Reading IMU DMA frame   // TODO!!! IMU accessing must be critical section
        CoClearFlag(flagIMUNewData); 
				RadioIMU_WaitGrabSPI();
				IMUProcess();
        TRACE("%X, %X, %X\n\r",*(uint16_t*)&IMU_RawData[0], *(uint16_t*)&IMU_RawData[2], *(uint16_t*)&IMU_RawData[4]);
        TRACE("%X, %X, %X\n\r",*(uint16_t*)&IMU_RawData[6], *(uint16_t*)&IMU_RawData[8], *(uint16_t*)&IMU_RawData[10]);
        SAVE_POINT
				RadioIMU_ReleaseSPI();
        SAVE_POINT
        break;
      case 's': // Set configuration
        {
        uint32_t      pattern = 0;
        uint8_t       ledBits = 0;
        uint8_t       ledId  = 0;
        // if (addr >= 19 && addr <= 21) {
        //need to do additional scanf - it has form of
        //s 19 8bit 32bit 8bit
        // all values are decimal
        //uint8_t       ledId = value;
        char *p = s;
        //skip s addr value
        while (*p != 0 &&!isspace(*p)) { p++;} p++;
        addr = strtoul(p, NULL, 10);
        while (*p != 0 &&!isspace(*p)) { p++;} p++;
        value = strtoul(p, NULL, 10);
        while (*p != 0 &&!isspace(*p)) { p++;} p++;
        pattern = strtoul(p, NULL, 10);
        while (*p != 0 &&!isspace(*p)) { p++;} p++;
        ledBits = strtoul(p, NULL, 10);
        while (*p != 0 &&!isspace(*p)) { p++;} p++;
        ledId = strtoul(p, NULL, 10);
        // }
        SetConfig(addr, value,  pattern, ledBits, ledId);
        }
        SAVE_POINT
        break;
      case 'v': // saVe configuration
        //if (i != 1) continue;
        WaitGrabI2C();
        SaveConfig(&config);
        ReleaseI2C();
        break;
      case 'h': // Halt IMU Tx
        //if (i != 1) continue;
  			EXTI->IMR &= ~GPI_IMU_INT_EXTI_LINE;
        halted = 1;
        break;
      case 'c': // Continue IMU Tx
        //if (i != 1) continue;
        EXTI->IMR |= GPI_IMU_INT_EXTI_LINE;
				halted = 0;
      break;
#if 0
      case 'w': // Write IMU_A-Reg
        //if (i != 3) continue;
        //CoTickDelay(2);
        IMURegWr(addr, SPI_A_IMU, value);
        break;
      case 'r': // Read IMU_A-Reg
        //if (i != 2) continue;
        //CoTickDelay(2);
        TRACE("Accel Reg%02X = %04X\n\r", addr, (int)IMURegRd(addr, SPI_A_IMU));
        break;
      case 'W': // Write IMU_G-Reg
        //if (i != 3) continue;
        //CoTickDelay(2);
        IMURegWr(addr, SPI_G_IMU, value);
        break;
      case 'R': // Read IMU_G-Reg
        //if (i != 2) continue;
        //CoTickDelay(2);
        TRACE("Gyro Reg%02X = %04X\n\r", addr, (int)IMURegRd(addr, SPI_G_IMU));
        break;
#endif
      case '-':  // dec Tx power
        if(config.TxLevel ==0 ||config.TxLevel > 8 ){
          config.TxLevel =8; // sizeof(uint8_t TxAmpValues);
        }else{
          config.TxLevel--;
        }
        RadioSetRFLevel(config.TxLevel);
        SAVE_POINT
        TRACE("RF Tx Level: %X\n\r", TxAmpValues[config.TxLevel]);
        break;
      case '+':  // increment Tx power
        if(config.TxLevel >= 8){
          config.TxLevel = 0; // sizeof(uint8_t TxAmpValues);
        }else{
          config.TxLevel++;
        }
        RadioSetRFLevel(config.TxLevel);
        SAVE_POINT
        TRACE("RF Tx Level: %X\n\r", TxAmpValues[config.TxLevel]);
        break;
      case '.': // IMU debug print
        //if (i != 1) continue;
        TRACE("IMU Debug print:\n\r");
        IMUdbgPrt = !IMUdbgPrt;
        break;
      case 'y': // Serial Number printout
        //if (i != 1) continue;
        TRACE("CPU Serial #%04x:%04x:%04x \n\r",ARM_proc_SN.a,ARM_proc_SN.b,ARM_proc_SN.c );
        break;
      case 'F':
        TRACE("Framebits: %d\r\n", config.frameBits);
        break;
#if 0
      case '~':  // reset default settings
        TRACE("Restoring Default Settings \n\r");
        memcpy((void *)&config, (void *)&backup_config, sizeof(config));
        break;
#endif
      case 't':   // trace / debug
        {
        char *p = NULL;
        while (*s && isspace(*(++s)));
        if (!*s) {
          break;
        }
        addr = strtoul(s, &p, 10);
        while (*p && isspace(*(++p)));
        if (*p) {
          value = strtoul(p, NULL, 10);
        } else {
          value = 0;
        }
        switch (addr) {
          case 0:
            {
              config.flags ^= FLAG_TRACE_ENABLE;
            }
            break;
          case 1:
            {
              config.flags ^= FLAG_TRACE_TIMESLOT;
            }
            break;
          case 2:
            {
              config.flags ^= FLAG_TRACE_SYNC;
            }
            break;
          case 3:
            {
              config.flags ^= FLAG_TRACE_IMU_QUEUE_FULL;
            }
            break;
          case 4:
            {
              config.flags ^= FLAG_TRACE_USE_TIMESLOT;
            }
            break;
          case 5:
            {
              config.flags ^= FLAG_TRACE_USE_PROTO_CMD_LINE_RESP;
            }
            break;
          case 6:
            {
              // radio off
              radio_off = 1;
            }
            break;
          case 7:
            {
              // radio on
              radio_off = 0;
            }
            break;
          case 8:
            {
              struct WhoAmI me;
              memcpy(me.id, (uint8_t *)(0x1FFFF7E8), 12);
              me.type = 3; // beacon
              me.module = 2;
              __writeCmdLineRespPacket((void *)&me,  sizeof(me), DEV_RESP_WHOAMI);
            }
            break;
          case 9:
            {
              config.flags ^= FLAG_TRACE_BEACON;
            }
            break;
          case 10:
            {
              config.flags ^= FLAG_TRACE_ASYNC;
            }
            break;
          case 11:
            {
              config.flags ^= FLAG_TRACE_CRC;
            }
            break;
          case 12:
            {
              //send_battery_info = 0;
              config.flags &= ~FLAG_SEND_BATTERY_INFO;
            }
            break;
          case 13:
            {
              //send_battery_info = 1;
              config.flags |= FLAG_SEND_BATTERY_INFO;
            }
            break;
          case 17: 
            {
              config.time_adjust = (int16_t) value;
              //I2C_EE_BufferWrite((uint8_t*) &time_adjust, 126, 2);
              WaitGrabI2C();
              SaveConfig(&config);
              whole_time_adjust = config.time_adjust/10;
              part_time_adjust = config.time_adjust%10;
              ReleaseI2C();
            } //no break comment? potential bug?
          case 18:
            {
              use_sync = 0;
            }
            break;
          case 19:
            {
              use_sync = 1;
            }
            break;
          case 20:
            {
              WaitGrabI2C();
              config.frameCountNoSync = value;
              SaveConfig(&config);
              ReleaseI2C();
            }
            break;
          case 41:
            {
              use_tim3_phase = value;
              break;
            }
          case 42:
            {
              config.flags ^= FLAG_DEBUG;
            }
            break;
          case 43:
            {
              TRACE("oldFrameTime @%d.%d frame=%u\n\r", oldFrameTime.sec, oldFrameTime.uSec, oldFrameIdAtSync);
              TRACE("newFrameTime @%d.%d frame=%u MsTimerAtSync=%u\n\r", newFrameTime.sec, newFrameTime.uSec, newFrameIdAtSync, MsTimerAtSync);
              TRACE("oldTim3Phase=%u newTim3Phase=%u tim_at_sec=%u\n\r", oldTim3Phase, newTim3Phase, tim_at_sec);
            }
            break;
          case 44:
            {
              config.flags |= FLAG_TRACE_ADJUST;
            }
          break;
          case 45:
            {
              config.flags &= ~FLAG_TRACE_ADJUST;
            }
          break;
          case 46:
            {
              TRACE("rxPkts=%u rxTotal=%u rxNotEmpty=%u sec=%u\r\n", rxPackets, rxTotalRcvd, rxNotEmpty, sec);
            }
            break;
          case 47:
            {
              FRAME_DIFF_DELTA = (int8_t) value;
            }
            break;
          case 48:
            {
              config.flags ^= FLAG_FRAMEID_24BITS;
              WaitGrabI2C();
              SaveConfig(&config);
              ReleaseI2C();
            }
            break;
          case 49:
            {
              TRACE(": rxWait @ %d.%d rxStart @ %d.%d\n\r", radioRxWait.sec, radioRxWait.uSec, radioRxStart.sec, radioRxStart.uSec);
              TRACE(": txStart @ %d.%d txEnd @ %d.%d queue_full=%d queue_size=%d\n\r", startRadioTx.sec, startRadioTx.uSec, endRadioTx.sec, endRadioTx.uSec, queue_full, inIdx);
              TRACE(": rxCount=%d rxFIFOError=%d rxErrors=%d\n\r", rxCount, rxFIFOError, rxErrors);
              CoTickDelay(10);
              TRACE(": rxFIFOTime %d.%d semTime2 %d.%d\n\r", rxFIFOTime.sec, rxFIFOTime.uSec, semTime2.sec, semTime2.uSec);
              TRACE(": excFlag0=0x%02X excFlag1=0x%02X excFlag2=0x%02X\n\r",cc2520_flags0, cc2520_flags1, cc2520_flags2);
              TRACE(": successBeacons=%d frameOffset=%d frameAdjust=%d\n\r", successBeacons, frameOffset, FRAME_DIFF_DELTA);
              TRACE(": remainOutOfSyncTime=%d frameIdCorrection=%d\n\r", remainOutOfSyncTime, lastFrameIdCorrection);
              CoTickDelay(10);
              TRACE(": tkFrameId=%d asserted=%d\r\n", tkFrameId, asserted);
              TRACE("errorFrameId=%u valids=%d notValids=%d lostSync=%d\r\n", errorFrameId, Valids, notValids, lostSync);
              TRACE("changeClocks=%d newbcn=%d corCount=%d drifts=%u last_drift=%d\r\n", changeClocks, newbcn, frameIdCorrectionCount, drift, last_drift);
              extern uint8_t led_blinking;
              TRACE("led blinking=%d\r\n", led_blinking);
            }
            break;
          case 52:
            {
              uint32_t ram = (uint32_t) value & 0xFFFFFFFE;
              if (ram >= 0x20000000 && ram <= 0x2000ffff) {
                uint32_t atRam = *((uint32_t*) (ram));
                TRACE("Mem at 0x%08X=0x%08X\n\r", ram, atRam);
              } else {
                TRACE("Mem addr error\n\r");
              }
            }
            break;
          case 54:
            {
              RadioPrint2520Registers(addr);
            }
            break;
          case 55:
            {
              if (value > 0 && value < 30000) {
                txTimeSlot = value;
                firstTime = 1;
                TRACE("New timeslot offset = %u\r\n", txTimeSlot);
              }
            }
            break;
#ifdef CIRCULAR_LOG
          case 56:
            {
              uint16_t indx = 0;
              uint16_t size = (LOG_SIZE/sizeof(tLogStruct));
              TRACE("Current SysTickCount: %d, size=%d\n\r", (uint32_t) CoGetOSTime(), LOG_SIZE/sizeof(tLogStruct));
              for (; (indx < size) && (log[indx].type); indx++) {
                if ((indx & 0x07 == 0)) {
                  CoTickDelay(20);
                }
                TRACE(":%u @%u t:%d d:%u\n\r", indx, log[indx].timestamp, log[indx].type, log[indx].frameId);
                CoTickDelay(2);
              }
            }
            break;
          case 57:
            {
              if (value ==1313) {
                __disable_interrupt();
                memset((void*) log, 0, LOG_SIZE);
                log_index_in = 0;
                __enable_interrupt();
                TRACE("Erased\n\r");
              } else {
                TRACE("ooopss.. wrong value\n\r");
              }
            }
            break;
#endif
          case 58:
            {
              config.flags |= FLAG_TRACE_LEDSHUNTSYNC;    
              break;                  
            }
            break;
          case 59:
            {
              config.flags &= ~FLAG_TRACE_LEDSHUNTSYNC;
              break;
            }
            break;
          case 60:
            {
              config.flags |= FLAG_TRACE_UI_STATEMACHINE;
              break;
            }
            break;
          case 61:
            {
              config.flags &= ~FLAG_TRACE_UI_STATEMACHINE;
              break;
            }
            break;
          case 62:
            {
              config.chgstate_radio_en = 1; 
              break;
            }
          case 63:
            {
              config.chgstate_radio_en = 0;
              break;
            }
          case 94:
            {
              if (value == 16385) {
                TRACE("Generating hard fault\r\n");
                CoTickDelay(100);
                uint32_t *p = (uint32_t*) 0xDE002319;
                uint32_t j = *p;
              }
            }
            break;
          case 95:
            {
              watchdog_active = value;
              TRACE("Watchdog is %s\r\n", value?"active":"not active");
            }
            break;
          case 96:
            {
              extern uint32_t stacked_lr, stacked_pc, stacked_psr;
              uint32_t saved_lr, saved_pc, saved_psr;
              uint32_t fault_counter;
              WaitGrabI2C();
              I2C_EE_BufferRead((uint8_t*) &saved_lr, EEPROM_DEBUG_STACKED_LR, EEPROM_DEBUG_STACKED_LR_SIZE);
              I2C_EE_BufferRead((uint8_t*) &saved_pc, EEPROM_DEBUG_STACKED_PC, EEPROM_DEBUG_STACKED_PC_SIZE);
              I2C_EE_BufferRead((uint8_t*) &saved_psr, EEPROM_DEBUG_STACKED_PSR, EEPROM_DEBUG_STACKED_PSR_SIZE);
              I2C_EE_BufferRead((uint8_t*) &fault_counter, EEPROM_DEBUG_COUNTER, EEPROM_DEBUG_COUNTER_SIZE);
              ReleaseI2C();
              TRACE("stacked_lr=0x%08X stacked_pc=0x%08X\r\n",
                    stacked_lr, stacked_pc);
              TRACE("stacked_psr=0x%08X saved_lr=0x%08X\r\n",
                    stacked_psr, saved_lr);
              TRACE("saved_pc=0x%08X saved_psr=0x%08X\r\n", saved_pc, saved_psr);
              TRACE("fault_counter = %u\r\n", fault_counter);
            }
            break;
          case 97:
            {
              stacked_lr = stacked_pc = stacked_psr = 0;
            }
            break;
#ifdef TASKS_PROFILE
          case 98:
            {
                __disable_interrupt();
                uint32_t stack1              = CoGetStackDepth(task1Id);
                uint32_t perfTask1           = CoGetTaskScheduledCount(task1Id);
                uint16_t lineTask1           = CoGetTaskLine(task1Id);
                const char *funcTask1        = CoGetTaskFunc(task1Id);
                uint32_t stackRadioRx        = CoGetStackDepth(taskRadioRxId);
                uint32_t perfRadioRx         = CoGetTaskScheduledCount(taskRadioRxId);
                uint16_t lineRadioRx         = CoGetTaskLine(taskRadioRxId);
                const char *funcRadioRx      = CoGetTaskFunc(taskRadioRxId);
                uint32_t stack3              = CoGetStackDepth(task3Id);
                uint32_t perfTask3           = CoGetTaskScheduledCount(task3Id);
                uint16_t lineTask3           = CoGetTaskLine(task3Id);
                const char *funcTask3        = CoGetTaskFunc(task3Id);

                uint32_t stackConfig         = CoGetStackDepth(taskConfigId);
                uint32_t perfTaskConfig      = CoGetTaskScheduledCount(taskConfigId);
                uint16_t lineTaskConfig      = CoGetTaskLine(taskConfigId);
                const char *funcTaskConfig   = CoGetTaskFunc(taskConfigId);
                uint32_t stackRadioTx        = CoGetStackDepth(taskRadioTxId);
                uint32_t perfRadioTx         = CoGetTaskScheduledCount(taskRadioTxId);
                uint16_t lineRadioTx         = CoGetTaskLine(taskRadioTxId);
                const char *funcRadioTx      = CoGetTaskFunc(taskRadioTxId);

                uint32_t stack8              = CoGetStackDepth(task8Id);
                uint32_t perfTask8           = CoGetTaskScheduledCount(task8Id);
                uint16_t lineTask8           = CoGetTaskLine(task8Id);
                const char *funcTask8        = CoGetTaskFunc(task8Id);
                uint32_t stackIMU            = CoGetStackDepth(taskIMUGId);
                uint32_t perfTaskIMU         = CoGetTaskScheduledCount(taskIMUGId);
                uint16_t lineTaskIMU         = CoGetTaskLine(taskIMUGId);
                const char *funcTaskIMU      = CoGetTaskFunc(taskIMUGId);

                __enable_interrupt();


                TRACE("Task1 [avail stack:%d]:%d @ %s():%d\n\r", stack1, perfTask1, funcTask1, lineTask1);
                TRACE("RadioRxTask [avail stack:%d]:%d @ %s():%d\n\r", stackRadioRx, perfRadioRx, funcRadioRx, lineRadioRx);
                TRACE("Task3 [avail stack:%d]:%d @ %s():%d\n\r", stack3, perfTask3, funcTask3, lineTask3);
                TRACE("TaskConfig [avail stack:%d]:%d @ %s():%d\n\r", stackConfig, perfTaskConfig, funcTaskConfig, lineTaskConfig);
                TRACE("RadioTxTask [avail stack:%d]:%d @ %s():%d\n\r", stackRadioTx, perfRadioTx, funcRadioTx, lineRadioTx);
                TRACE("Task8 [avail stack:%d]:%d @ %s():%d\n\r", stack8, perfTask8, funcTask8, lineTask8);
                TRACE("TaskIMU [avail stack:%d]:%d @ %s():%d\n\r", stackIMU, perfTaskIMU, funcTaskIMU, lineTaskIMU);

          }
          break;
          case 99:
            {
            uint32_t scheduled_idle = CoGetTaskScheduledCount(0);
            uint32_t scheduled_task1 = CoGetTaskScheduledCount(task1Id);
            uint32_t scheduled_taskRadioRx = CoGetTaskScheduledCount(taskRadioRxId);
            uint32_t scheduled_task3 = CoGetTaskScheduledCount(task3Id);
            uint32_t scheduled_taskConfig = CoGetTaskScheduledCount(taskConfigId);
            uint32_t scheduled_taskRadioTx = CoGetTaskScheduledCount(taskRadioTxId);
            uint32_t scheduled_task8 = CoGetTaskScheduledCount(task8Id);
            uint32_t scheduled_taskIMU_G = CoGetTaskScheduledCount(taskIMUGId);
            TRACE("PERF: IdleTask: %u Task1: %u RxTask: %u Task3: %u\n\r",
                  scheduled_idle, scheduled_task1, scheduled_taskRadioRx, scheduled_task3);
            TRACE("PERF: TaskConfig: %u TxTask: %u\n\r",
                  scheduled_taskConfig, scheduled_taskRadioTx);
            TRACE("PERF: Task8: %u TaskIMU: %u\n\r", scheduled_task8, scheduled_taskIMU_G);
          }
          break;
#endif
          }
        }
        break;
      case 'U':
        {       // Extended command line that use base64 encoded parameters
          static uint8_t encoded[512];
          static uint8_t decoded[512];
          //uint8_t *_decoded = &decoded[0];
          uint16_t update_flags = 0;
          char *p = NULL;
          while (*s && isspace(*(++s)));
          if (!*s) {
            break;
          }

          addr = strtoul(s, &p, 10);
          s = p;
          while (*s && isspace(*(++s)));
            if (!*s) {
              break;
            }
          uint16_t value = strtoul(s, &p, 10);
            s = p;
            while (*s && isspace(*(++s)));
            if (!*s) {
              break;
            }
          strcpy((char*) encoded, s);
          int ret = b64_pton(encoded, decoded, sizeof(decoded));

          if (ret < sizeof(struct PacketHeader)) {
            TRACE("Incorrect packet\r\n");
            continue;
          }
          struct PacketHeader *packetHeader = (struct PacketHeader *)decoded;
          if (packetHeader->size > ret) {
            TRACE("Incorrect packet size\r\n");
            continue;
          }
          if (ValidateCommandLine((char*)decoded, ret) == 0) {
            TRACE("Incorrect packet\r\n");
            continue;
          }                

          if (packetHeader->type == DEV_CMD_BAT_STATUS) {
            __writeCmdLineRespPacket((void*) &lastBatStatus, sizeof(lastBatStatus), DEV_RESP_BAT_STATUS);
          } else if (packetHeader->type == DEV_CMD_CONFIG_REQ) {
            ex_config_t my_config;
            CopyConfigToExConfig(&config, &my_config);
            my_config.checksum = CalcConfigChecksum(&my_config.productID, my_config.size);
            __writeCmdLineRespPacket((void *)&my_config,  sizeof(my_config), DEV_RESP_CONFIG);
          } else if(packetHeader->type == DEV_CMD_GET_VERSION) {
            struct RespFirmwareVersion respFirm;
            respFirm.major = THIS_MAJOR;
            respFirm.minor = THIS_MINOR;
            respFirm.patch = THIS_PATCH;
            respFirm.reserved = 0;                       // For compatible reason
            respFirm.revision = THIS_REVISION;
            strcpy((char*) respFirm.dateString, __DATE__);
            strcpy((char*) respFirm.timeString, __TIME__);
            __writeCmdLineRespPacket((void *)&respFirm,  sizeof(respFirm), DEV_RESP_VERSION);
          }else if (packetHeader->type == DEV_CMD_RUNNING_STATUS_REQ) {
            struct BeaconRunningStatus bat;
            bat.errorCode = 0;
            bat.index = BattUnion.BatteryLevel[1];
            bat.radioOnOff = (radio_off == 0);
            __writeCmdLineRespPacket((void *)&bat,  sizeof(bat), DEV_RESP_RUNNING_STATUS);
          } else if (packetHeader->type == DEV_CMD_SET_CONFIG) {
            ex_config_t* newConfig;
            newConfig = (ex_config_t*)(decoded + sizeof(struct PacketHeader));
            if (newConfig->panId != config.panId ||
              newConfig->mySrcAddr != config.mySrcAddr) {
              update_flags |= UPDATE_FLAG_PANID;
            }
            if (newConfig->routerDstAddr != config.routerDstAddr) {
              update_flags |= UPDATE_FLAG_DSTADDR;
            }
            if (newConfig->ledDAC != config.ledDAC) {
              update_flags |= UPDATE_FLAG_DAC;
            }
            if (newConfig->rfChan != config.rfChan) {
              update_flags |= UPDATE_FLAG_RFCHAN;
            }
            if (newConfig->TxLevel != config.TxLevel) {
              update_flags |= UPDATE_FLAG_TXLEVEL;
            }
            CopyExConfigToConfig(newConfig, &config);
            // memcpy((void *)&config, (void *)newConfig, sizeof(beacon_config_t));
            WaitGrabI2C();
            SaveConfig(&config);
            ReleaseI2C();
            if (update_flags & UPDATE_FLAG_PANID) {
              RadioSetPanIdShortAddr(config.panId, config.mySrcAddr);
              CoSetFlag(flagUSBdata_UI);
            }
            if (update_flags & UPDATE_FLAG_DSTADDR) {
              routerAddr = config.routerDstAddr;
            }
            if (update_flags & UPDATE_FLAG_RFCHAN) {
              RadioSetRFChan(rfChan);
            }
            if (update_flags & UPDATE_FLAG_TXLEVEL) {
              RadioSetRFLevel(config.TxLevel);
            }
          } else if (packetHeader->type == DEV_CMD_SET_EEPROM_DATA) {
            struct RespUpdate up;
            up.index = 0;
            if (packetHeader->size < sizeof(struct PacketHeader) + 32) { // if data body greater than 32, only 32 bytes are written 
              up.errorCode = ERROR_WRONG_PACKET;
            } else {
              WaitGrabI2C();
              uint8_t *data = (uint8_t *) decoded + sizeof(struct PacketHeader);
              I2C_EE_BufferWrite(data, 128, 32);
              up.errorCode = 0;
              ReleaseI2C();
              CoSetFlag(flagUSBdata_UI);
            }
            uint8_t type = DEV_RESP_SET_EEPROM_DATA;
            __writeCmdLineRespPacket((unsigned char*) &up, 4, type);
          } else if (packetHeader->type == DEV_CMD_GET_EEPROM_DATA) {
            struct RespEepromData resp;
            WaitGrabI2C();
            I2C_EE_BufferRead(&resp.data[0], 128, 32);
            ReleaseI2C();
            resp.errorCode = 0;
            __writeCmdLineRespPacket((unsigned char*) &resp, sizeof(resp), DEV_RESP_GET_EEPROM_DATA);
          } else if (packetHeader->type == DEV_CMD_SET_PROD_AREA) {
            struct RespUpdate up;
            uint8_t type = DEV_RESP_SET_PROD_AREA;
            ret = SerialDownload((char*) decoded, ret, packetHeader->type);
            up.index = (ret & 0xFFFF0000) >> 16;
            up.errorCode = ret & 0xFFFF;
            __writeCmdLineRespPacket((unsigned char*) &up, 4, type);
          } else if (packetHeader->type == DEV_CMD_GET_PROD_AREA) {
            struct GetProdAreaPacketHeader *prodHeader = (struct GetProdAreaPacketHeader*) decoded;
            struct RespProdArea resp;
            page.address = FLASH_PROD_AREA;
            page.count = 2048;
            ret = ReadPage(&page);
            resp.errorCode = ret;
            resp.index = prodHeader->index;
            if (!ret) {
              memcpy(resp.data, &page.data[prodHeader->index*sizeof(resp.data)], sizeof(resp.data));
            }
            __writeCmdLineRespPacket((unsigned char*) &resp, sizeof(resp), DEV_RESP_GET_PROD_AREA);
          }
        }
        break;
      case '!':
        {
          uint8_t jumpToMain = 0xFF;
          WaitGrabI2C();
          I2C_EE_BufferRead((uint8_t*) &jumpToMain, EEPROM_BEACON_FLAG_ADDRESS, 1);
          if (jumpToMain != 0xFF) {
            jumpToMain = 0xFF;
            I2C_EE_BufferWrite((uint8_t*) &jumpToMain, EEPROM_BEACON_FLAG_ADDRESS, 1);
          }
          ReleaseI2C();
          NVIC_GenerateSystemReset();
        }
        break;
#if (defined USE_MY_ASSERT) || (defined USE_FULL_ASSERT)
      case 'a':
        {
            assert_loop = 0;
        }
        break;
#endif
      default:
        TRACE("**ERROR** Unrecognized Command '%c'\n\r", cmd);
        break;
    }

    if (cmd != 'U') {
        TRACE("> ");
    }
  }
}

uint32_t mycalls;
uint32_t radioTxEntries;
StatusType lastRadioTx;
StatusType setTxDone;

/*******************************************************************************
* Description    : [Task] Manage Outgoing (TX) Packet Queue
* Input          :
* Return         :
*******************************************************************************/
void TaskRadioTx(void* pdata){
    SAVE_LINE
    while (1) {
        // avoid busy waiting. sleep a tick or until wake up by other task
        SAVE_FUNC
        radioTxEntries++;
        lastRadioTx = CoWaitForSingleFlag(flagRadioTxReq, 1);
        SAVE_POINT
        while (1) {
            SAVE_FUNC
            RELOAD_WATCHDOG
            __disable_interrupt();
#ifndef NO_TK
            if (remainOutOfSyncTime <= 0) {         // out of synchronization. Need to stop sending so that T.K. can occupy timeslot to sync again
            //    CoSuspendTask(task1Id);
            //    CoSuspendTask(taskRadioTxId);

              __enable_interrupt();
              SAVE_LINE
                break;
            }
#endif
            if (inIdx == outIdx) { // queue is NOT empty.       // TODO!!! Double check if producer can be run and modifies inIdx. If so it is a critical section.
                queue_full = 1;
                SAVE_LINE
                __enable_interrupt();
                break;
            } else {
              
              queue_full = 0;
            }
            __enable_interrupt();

            SAVE_POINT
            RadioIMU_WaitGrabSPI();
            SAVE_POINT
            startRadioTx.sec = sec;
            startRadioTx.uSec = TIM1->CNT;
            // outIdx is modified by Tx task only. So no lock required
            RadioTxPkt(txPktQueue[outIdx].dstAddr,
                       0,   // not a beacon frame
                       txPktQueue[outIdx].payloadSize,
                       txPktQueue[outIdx].payload,
                       1);  // transmit immediately
            SAVE_POINT
            mycalls++;
            endRadioTx.sec = sec;
            endRadioTx.uSec = TIM1->CNT;
#ifndef CCA_EN
            RadioIMU_ReleaseSPI();      // TODO!!! verify if SPIIODone is correctly waiting
#endif

            // ring buffer operation is in critical section
            __disable_interrupt();
            outIdx = (outIdx + 1) % TX_PKT_QUEUE_SIZE;
            outidxcheck++;
            __enable_interrupt();

            numTxRetries = 0; // reset counter as we initiate tx
            txRetryState = 1; // indicate STXONCCA has been issued. TODO!!! verify the meaning

            /* fire up TIM5 to ring check CCA after 1ms */
#ifdef CCA_EN
            uint8_t txRetryState = 1;
            uint8_t numTxRetries = NUM_TX_RETRIES;

            do {
              TIM5->EGR |= TIM_EGR_UG;
              TIM5->CR1 |= TIM_CR1_CEN;
              CoWaitForSingleFlag(flagRadioCCA, 1, 0);
              SAVE_POINT

              assert(txRetryState != 0); // [[DEBUG]]

              if (txRetryState == 1) {            // read FMSTAT1
                /* check if CCA was asserted */
                if (!spiTxRxByteCount) {        // SPI not in use
                  /* read FSMSTAT1 register */
                  scratchBuf[0] = (CC2520_INS_REGRD | CC2520_FSMSTAT1);
                  scratchBuf[1] = 0;          // pad byte to push out reg val
                  spiTxRxByteCount = 0x02;  // indicate FMSTAT1 read
                  spiTxRxByteState = RF_SPI_CCA_CMD_STATE;
                  pSpiTxBuf = scratchBuf;
                  pSpiRxBuf = scratchBuf;     // self-clobbering ... that's ok
                  HwSPISSAssert(SPI_RADIO);
                  SPI_I2S_ITConfig(SPI_RADIO_SPI, SPI_I2S_IT_RXNE, ENABLE);
                  SPI_I2S_SendData(SPI_RADIO_SPI, *pSpiTxBuf++);
                  txRetryState++;             // go to next state
                } else {
                  break;
                }
            } else if (txRetryState == 2) {     // check SAMPLED_CCA bit, retry TX if necessary
              if (sampledCCA & CC2520_FSMSTAT1_SAMPLED_CCA_BM) {
                /* CCA asserted, packet is going (has gone) out */
                txRetryState = 0;
                break;
              }

              /* CCA NOT asserted, packet isn't going out ... try STXONCCA again now */

              if (numTxRetries < NUM_TX_RETRIES) {
                if (!spiTxRxByteCount) {        // SPI not in use
                    spiTxRxByteCount = 0x01;  // indicate STXONCCA retry
                    spiTxRxByteState = RF_SPI_STXONCCA_CMD_STATE;
                    pSpiRxBuf = scratchBuf;     // don't care
                    HwSPISSAssert(SPI_RADIO);
                    SPI_I2S_ITConfig(SPI_RADIO_SPI, SPI_I2S_IT_RXNE, ENABLE);
                    SPI_I2S_SendData(SPI_RADIO_SPI, CC2520_INS_STXONCCA);
                    numTxRetries++;
                    txRetryState = 1;
                }                               // else, remain in this state, retry later
            } else { // done retrying, no more
                txRetryState = 0;
                // fake TX_FRM_DONE
                setTxDone = CoSetFlag(flagRadioTxDone);
                break;
            }
            SAVE_POINT
        } else assert(0);


            } while (1);
            RadioIMU_ReleaseSPI();
#endif
        }
    }
}

float fVolt = 0.0f;

/*******************************************************************************
* Description    : [Task]A to D conversion for battery voltage
* Input          :
* Return         :
* Calculation: resistive divider
*  4.99k + 4.3k
*  After divider have: 4.3/(4.3+4.99) = 0.462365 of original Vbat value
*  With 4.2V nominal value will have 4.2V*0.462365=1.941935V at ADC
*  With ADC Vref= V+ = 2.2V will have 1.941935V/2.2V = 0.8826979 of full scale
*  With full scale of 65536 will have 0.8826979*65536=57848
*  In hexadecimal 4.2V at battery will be reported as 0xE17F
*  Other values calculated using linear scale
*
* There is no chip to calculate battery level, so following is data measured
* with radio ON and no LED
* DISCHARGE - starting at 4.2V:
* after 1hr=4.12V 2hr = 4.07V 3hr = 4.03V 4hr = 3.99V, up to 9hr rate is 0.03V per hour
* after 9hr rate up to 16 hr rate 0.02V, then 0.03V 2 hours up to 3.65V, then
* during 1hr goes to 3.5V and turns off
* CHARGE - 15 minutes to get to 3.92V, then steady rate 0.06V/hour
* after reaching 4.21V 45 minutes to full charge
*Data for V#3 hardware.. 4.2v= E1, 4.0v=D7, 3.9v=D0, 3.8v=CA, 3.7v=C6, 3.6v=C1
*                         3.5v=BB, 3.4v=B6, 3.3v=B0,3.2v=AB, 3.1v= A5, 3.0v= A0
*                         2.9v= 9A, 2.8v= 96, 2.7v= 90, 2.6v= 8B
*******************************************************************************/
#define DEVICE_CHARGING 0x02  //change this after boardspin -> this only indicates usb connection, not 'charging'
float fLast[10];

void Task8(void* pdata){
  while (1) {
      SAVE_POINT
    RELOAD_WATCHDOG
        // enable a to d battery input
    HwGPOHigh(GPO_VBATT_ADC_EN);
        // wait 1 for input to stabilise
    CoTickDelay(1);
    SAVE_POINT
        // trigger a to d
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        // wait 1 for data sample
    CoTickDelay(2);
    SAVE_POINT
        // get sample data, disable input
    BattUnion.Battery_AtoD = (ADC_GetConversionValue(ADC1));
    HwGPOHigh(GPO_VBATT_ADC_EN);
        // wait 100
    CoTickDelay(100);
    SAVE_POINT
    //CoTickDelay(1000);
    
    static uint32_t iter = 0;
    float fCurr = 4.2f * (float)BattUnion.Battery_AtoD;
    fCurr /= (float) 0xE1F8;
    fLast[iter%10] = fCurr;
    iter++;
    uint8_t cnt = (iter>10)?10:iter;
    fVolt = 0.0;
    for (uint8_t k = 0; k < cnt; k++) {
      fVolt += fLast[k];
    }
    fVolt /= (float) cnt;
   /* if (iter == 10)*/ {
      SAVE_LINE
  //    iter = 0;
      lastBatStatus.type = 0xBA;
      lastBatStatus.version = 0x01;
      lastBatStatus.flags = HwGPIState(GPI_CHG_STAT)?0x00:0x01;
      if (HwGPIState(GPI_USB_VBUS)) {
        lastBatStatus.flags |= 0x02;
 //       if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9)) {
        rt_flags |= RT_FLAG_USB_CONNECTED;
 //     }
      } else {
        rt_flags &= ~RT_FLAG_USB_CONNECTED;
      }
      lastBatStatus.minToRun = 0xFFFF;
      //fVolt /= (float) 10.0;
      float percents = 0;
      if (lastBatStatus.flags == 0x03) { //charging
        if (fVolt > 4.20) {
          percents = 95.0;
        } else if (fVolt > 3.92) {
          percents = 4.0 + 91.0*(fVolt - 3.92)/0.28;
        } else {
          percents = 4.0;
        }
      } else {
        if (fVolt > 4.1 && (lastBatStatus.flags & 0x01) == 0) {
          percents = 100.0;
        } else if (fVolt > 4.07) {
          percents = 90.0 + 10.0 *(fVolt - 4.07)/0.14;
          if (percents >= 100.0) {
            percents = 99.0;
          }
        } else if (fVolt > 4.03) {
          percents = 85.0 + 5.0*(fVolt - 4.03)/0.04;
        } else if (fVolt > 3.84) {
          percents = 55.0 + 30.0*(fVolt - 3.84)/0.19;
        } else if (fVolt > 3.71) {
          percents = 20.0 + 35.0*(fVolt -3.71)/0.13;
        } else if (fVolt > 3.65) {
          percents = 10.0 + (fVolt - 3.65)/0.006;
        } else {
          percents = (fVolt - 3.5)/10.0;
          if (percents < 0) {
            percents = 0.0;
          }
        }
      }
      lastBatStatus.voltiCents = (uint16_t) ((fVolt + 0.005)*100.0);
      last_percents = lastBatStatus.percents = (uint8_t) percents;
    }
  }
}

StatusType setIMUReady;
uint32_t counterIMU;

extern uint16_t over1;
/*******************************************************************************
* Description    : [Task]9 Used for reading the IMU reg data and packing
* Input          :
* Return         :
*
*******************************************************************************/
void TaskIMU_G(void* pdata){
  while(1)
  {
    SAVE_POINT
    if (adjusted_changed) {
      adjusted_changed = 0;
      if (config.flags & FLAG_TRACE_ADJUST) {
        TRACE("adjusted=%u over=%d @%d.%d\r\n", adjusted, over1, sec, tim_at_sec);
      }
    }
    RELOAD_WATCHDOG
    CoWaitForSingleFlag(flagIMU_read,0);
    SAVE_POINT
    CoWaitForSingleFlag(flagIMU_G_DRDY, 0);
    counterIMU++;
    SAVE_POINT
    CoClearFlag(flagIMUNewData);        // DO NOT USE auto reset Flag as the DMA interrupt may take place before CoWaitForSingleFlag(flagIMUNewData, 0);  
    RadioIMU_WaitGrabSPI();
    imuaccess++;
    IMUProcess();
    RadioIMU_ReleaseSPI();
    SAVE_POINT
    InputDataIntoBuffer(&IMU_RawData[0]);
    setIMUReady = CoSetFlag(flagIMUDataReady);
    SAVE_POINT;
  }
}



/*******************************************************************************
* Description    : [Task] Used to determine the status of the infrared LEDs
* Input          :
* Return         :
*
*******************************************************************************/
void TaskLED(void* pdata){
  static StatusType result;
  static uint8_t i = 0;
  static ledcheck checkled = select_IRLED0;
  static float led_value;
  static float value;
  static uint8_t led_errorcheck = 0;
  IRled_flags = (IRled_status) {.IRled0_DC=0, .IRled1_DC=0, .IRled2_DC=0};
  while(1){
    SAVE_POINT
    RELOAD_WATCHDOG
    CoTickDelay(1);
    CoClearFlag(flagLEDSync);
    led_select = checkled;
    CoWaitForSingleFlag(flagLEDSync,0);
    led_select = select_none;
    SAVE_POINT
    CoTickDelay(1);
    WaitGrabI2C();
    SAVE_POINT
    switch(checkled){
        case select_IRLED0:
          if(!HwGPOstatus(GPO_IRLED0)){ //confirm led was not turned off in cc3
            if(config.flags & FLAG_TRACE_LEDSHUNTSYNC) TRACE("I2C irled%i not on\n\r", checkled);
            led_errorcheck=0;
          }
          else
            led_value = ledbuffer_set(shunt1_address, &led_errorcheck);
        break;
        case select_IRLED1:
          if(!HwGPOstatus(GPO_IRLED1)){ //confirm led was not turned off in cc3
            if(config.flags & FLAG_TRACE_LEDSHUNTSYNC) TRACE("I2C irled%i not on\n\r", checkled);
            led_errorcheck=0;
          }
          else
            led_value = ledbuffer_set(shunt2_address, &led_errorcheck);
        break;
        case select_IRLED2: 
          if(!HwGPOstatus(GPO_IRLED2)){ //confirm led was not turned off in cc3
            if(config.flags & FLAG_TRACE_LEDSHUNTSYNC) TRACE("I2C irled%i not on\n\r", checkled);
            led_errorcheck=0;
          }
          else
            led_value = ledbuffer_set(shunt3_address, &led_errorcheck);
        break;      
    }
    SAVE_POINT
    ReleaseI2C();
     SAVE_POINT
    if(led_errorcheck){ //done outside of mutex to avoid unnecessary time within critical region for other potential threads
      SAVE_POINT
      value = (float) led_value / ina219_currentDivider_mA;
      switch(checkled){
        case select_IRLED0:
        IRled_flags.IRled0_DC |= (value > 10.0)? 1:0;
        break;
        case select_IRLED1: 
        IRled_flags.IRled1_DC |= (value > 10.0)? 1:0;
        break;
        case select_IRLED2: 
        IRled_flags.IRled2_DC |= (value > 10.0)? 1:0;
        break;
      }
      checkled = (checkled+1 <= select_IRLED2)? checkled+1:select_IRLED0;
      if(checkled == select_IRLED0){
        if(++i>=IRLED_SAMPLES){
          i=0;

          if( IRled_flags.IRled0_DC != (UI_IRLEDdata & UI_IRLED0FLAG)>>0 ||
              IRled_flags.IRled1_DC != (UI_IRLEDdata & UI_IRLED1FLAG)>>1 ||
              IRled_flags.IRled2_DC != (UI_IRLEDdata & UI_IRLED2FLAG)>>2
            ){
              CoPendSem(semInkUI,0);
              UI_IRLEDdata = (IRled_flags.IRled0_DC<<0) |(IRled_flags.IRled1_DC<<1) | (IRled_flags.IRled2_DC<<2);
              CoSetFlag(flagLED_UI);
              CoPostSem(semInkUI);
            }
          if(config.flags & FLAG_TRACE_LEDSHUNTSYNC){
            TRACE("IRled0 = %i, IRled1 = %i, IRled2 = %i\r\n", IRled_flags.IRled0_DC, IRled_flags.IRled1_DC, IRled_flags.IRled2_DC);
          }
          IRled_flags.IRled0_DC = IRled_flags.IRled1_DC = IRled_flags.IRled2_DC = 0;
          CoTickDelay(1000);          
        }
      }
    }
    else{
      SAVE_POINT
      if(config.flags & FLAG_TRACE_LEDSHUNTSYNC) TRACE("retry I2C irled%i\n\r", checkled);
    }
  }
}

/*******************************************************************************
* Description    : [Task] Used to control the user interface statemachine
* Input          :
* Return         :
*
*******************************************************************************/
void TaskUI(void *pdata){
  static uint16_t var = 0;     //used for a) check hold count until task3 takes on this functionality and b) checks if EPD requires update
  static uint64_t timesequence;
  static uint64_t currentTime;
  static uint32_t getFlags;
  static StatusType result;
  static uint8_t led_flags=0;
  static uint8_t refresh=0;
  static UI_SubStates MenuSubState = UI_MENU_HOME_SUBSTATE;
  static uint8_t addressfield[8];

  WaitGrabEInk();
  EInk_Present = (Eink_Init())? 1:0;
  if(EInk_Present == 0){ 
    CoSchedLock();                      //shut down sequence
    HwGPOLow(GPO_RF_EN);
    HwGPOLow(GPO_VBATT_ADC_EN);
    HwGPOInitOC(GPO_USB_VBUS); 
    HwGPOLow(GPO_USB_VBUS);  
    SAVE_POINT
    HwGPOLow(GPO_PWRON); 
    while (1);
  }
  GFX_FillBuffer(black);
  GFX_DrawBitMap(UI_LOGO_COORD, logo, logo_dimensions);
  setFont(smallfont);
  setCursor(UIPOWER_TEXT_COORD);
  GFX_Print("loading...", strlen("loading..."),centeralign, black, gray_l);
  update(fullrefresh);
  ReleaseEInk();

  WaitGrabI2C();
  I2C_EE_BufferRead(&beaconName[0], 128, 12);
  ReleaseI2C();
  sprintf(addressfield, "ID%i", config.mySrcAddr);
  CoTickDelay(1000);                    //allow time for loading screen to be lit
  
  WaitGrabEInk();
  UI_StateMachine=UI_HOME_STATE;
  setHomeScreen(beaconName,addressfield);
  setStringerGraphics(led_flags);
  update(fullrefresh);
  ReleaseEInk();

  CoPostSem(semInkUI);                  //allow UI functionality from other threads when released.
  CoAwakeTask(taskLEDId);
  
  while(1){
    RELOAD_WATCHDOG
    SAVE_POINT
    //Wait for event that affects UI -> button presses (A,B,X), LED Stringers
    getFlags = CoWaitForMultipleFlags(1<<flagBtn_UI | 
                                      1<< flagLED_UI | 
                                      1<< flagTimeout_UI |
                                      1<< flagUSBdata_UI,
                                      OPT_WAIT_ANY, 0, &result);   
    SAVE_POINT
    if(result == E_OK){  
      if(getFlags == 1<<flagBtn_UI){
        CoClearFlag(flagBtn_UI);
        CoPendSem(semInkUI,0);
        switch(UI_StateMachine){
          case UI_HOME_STATE:
            if(UI_BUTTONdata&UI_PWRBTNFLAG){
              UI_StateMachine = UI_MENU_STATE;
              CoPostSem(semInkUI);
              MenuSubState = UI_MENU_HOME_SUBSTATE; 
              if(config.flags & FLAG_TRACE_UI_STATEMACHINE) TRACE("X btn: Home -> Menu\r\n");
              GFX_FillBuffer(black);
						  setFont(smallfont);
						  setCursor(UIHOME_DURATION_COORD);
              GFX_Print("8.00H", strlen("8.00H"),leftalign, black, gray_l);
						  GFX_DrawBitMap(UI_BATTERY_TEXT_COORD, battery100, battery_text_dimensions);
						  GFX_DrawBitMap(UIMENU_HOME_COORD,button_homeenable, button_dimensions);
						  GFX_DrawBitMap(UIMENU_STATUS_COORD,button_statusdisable, button_dimensions);
						  GFX_DrawBitMap(UIMENU_POWER_COORD,button_powerdisable, button_dimensions);
              resetUITimeout();
              WaitGrabEInk();
              update(fullrefresh);
              ReleaseEInk();
            }else CoPostSem(semInkUI);  //sanity check
          break;
          case UI_MENU_STATE:
            if(UI_BUTTONdata&UI_BTNBFLAG){
              CoPostSem(semInkUI);
              resetUITimeout();
              switch(MenuSubState){
                case UI_MENU_HOME_SUBSTATE:
                  MenuSubState = UI_MENU_STATUS_SUBSTATE;
                  if(config.flags & FLAG_TRACE_UI_STATEMACHINE) TRACE("A btn: in menu status substate\r\n");
						      GFX_DrawBitMap(UIMENU_HOME_COORD,button_homedisable, button_dimensions);
						      GFX_DrawBitMap(UIMENU_STATUS_COORD,button_statusenable, button_dimensions);
                  WaitGrabEInk();
                  update(partialrefresh);
                  ReleaseEInk();
                break;
                case UI_MENU_STATUS_SUBSTATE:
                  MenuSubState = UI_MENU_POWER_SUBSTATE;
                  if(config.flags & FLAG_TRACE_UI_STATEMACHINE) TRACE("A btn: in menu ID substate\r\n");
						      GFX_DrawBitMap(UIMENU_STATUS_COORD,button_statusdisable, button_dimensions);
						      GFX_DrawBitMap(UIMENU_POWER_COORD,button_powerenable, button_dimensions);
                  WaitGrabEInk();
                  update(partialrefresh);
                  ReleaseEInk();
                break;
#if 0
                case UI_MENU_ID_SUBSTATE:
                  MenuSubState = UI_MENU_POWER_SUBSTATE;
                  if(config.flags & FLAG_TRACE_UI_STATEMACHINE) TRACE("A btn: in menu power substate\r\n");
                break;
#endif
                case UI_MENU_POWER_SUBSTATE:
                  MenuSubState = UI_MENU_HOME_SUBSTATE;
                  if(config.flags & FLAG_TRACE_UI_STATEMACHINE) TRACE("A btn: in menu home substate\r\n");
                  GFX_DrawBitMap(UIMENU_HOME_COORD,button_homeenable, button_dimensions);
						      GFX_DrawBitMap(UIMENU_POWER_COORD,button_powerdisable, button_dimensions);
                  WaitGrabEInk();
                  update(partialrefresh);
                  ReleaseEInk();
                break;
              }
            }
            else if(UI_BUTTONdata&UI_BTNAFLAG){
              disableUITimeout();
              switch(MenuSubState){
                case UI_MENU_HOME_SUBSTATE:
                  UI_StateMachine = UI_HOME_STATE;
                  CoPostSem(semInkUI);
                  if(config.flags & FLAG_TRACE_UI_STATEMACHINE) TRACE("B btn: menu->home state\r\n");
                  setHomeScreen(beaconName,addressfield);
                  setStringerGraphics(led_flags);
                  WaitGrabEInk();
                  update(fullrefresh);
                  ReleaseEInk();
                break;
                case UI_MENU_STATUS_SUBSTATE:
                  UI_StateMachine = UI_STATUS_RADIO_STATE;
                  CoPostSem(semInkUI);
                  if(config.flags & FLAG_TRACE_UI_STATEMACHINE) TRACE("B btn: menu->status radio state\r\n");
                  //GFX_FillBuffer(black);
						      GFX_DrawBitMap(UISTATUS_COORD,status_radio_static, statusscreen_dimensions);
                  setFont(smallfont);
                  setCursor(UIHOME_DURATION_COORD);
                  GFX_Print("8.00H", strlen("8.00H"),leftalign, black, gray_l);
                  WaitGrabEInk();
                  update(fullrefresh);
                  ReleaseEInk();
                break;
#if 0
                case UI_MENU_ID_SUBSTATE:
                  UI_StateMachine = UI_ID_STATE;
                  CoPostSem(semInkUI);
                  if(config.flags & FLAG_TRACE_UI_STATEMACHINE) TRACE("B btn: menu->ID state\r\n");
                break;
#endif
                case UI_MENU_POWER_SUBSTATE:
                  //power down sequence
                  if(config.flags & FLAG_TRACE_UI_STATEMACHINE) TRACE("PWR DOWN SEQUENCE\r\n");
                  GFX_FillBuffer(black);
                  GFX_DrawBitMap(UI_LOGO_COORD, logo, logo_dimensions);
                  setFont(smallfont);
                  setCursor(UIPOWER_TEXT_COORD);
                  GFX_Print("Turning Off", strlen("Turning Off"),centeralign, black, gray_l);
                  WaitGrabEInk();
                  update(fullrefresh);
                  ReleaseEInk();
                  GFX_FillBuffer(black);
                  WaitGrabEInk();
                  update(fullrefresh);
                  ReleaseEInk();
                  CoSchedLock();                      //shut down sequence
                  HwGPOLow(GPO_RF_EN);
                  HwGPOLow(GPO_VBATT_ADC_EN);
                  HwGPOInitOC(GPO_USB_VBUS); 
                  HwGPOLow(GPO_USB_VBUS);  
                  HwGPOLow(GPO_PWRON) ; 
                  while (1);
                break;
              }
            } else CoPostSem(semInkUI); //for sanity
          break;
          case UI_STATUS_RADIO_STATE:
            if(UI_BUTTONdata&UI_PWRBTNFLAG 
            //||UI_BUTTONdata&UI_BTNBFLAG
            ){
              UI_StateMachine = UI_HOME_STATE;
              CoPostSem(semInkUI);
              if(config.flags & FLAG_TRACE_UI_STATEMACHINE) TRACE("X/B btn: status radio -> home\r\n");
              setHomeScreen(beaconName,addressfield);
              setStringerGraphics(led_flags);
              WaitGrabEInk();
              update(fullrefresh);
              ReleaseEInk();
            }
            else if(UI_BUTTONdata&UI_BTNBFLAG){
              UI_StateMachine = UI_STATUS_BATTERY_STATE;
              CoPostSem(semInkUI);
              if(config.flags & FLAG_TRACE_UI_STATEMACHINE) TRACE("A btn: status radio -> status battery\r\n");
              //GFX_FillBuffer(black);
						  GFX_DrawBitMap(UISTATUS_COORD,status_battery_static, statusscreen_dimensions);
              setFont(smallfont);
              setCursor(UIHOME_DURATION_COORD);
              GFX_Print("8.00H", strlen("8.00H"),leftalign, black, gray_l);
              WaitGrabEInk();
              update(fullrefresh);
              ReleaseEInk();
            } 
            else CoPostSem(semInkUI);
          break;
          case UI_STATUS_BATTERY_STATE:
            if(UI_BUTTONdata&UI_PWRBTNFLAG){
              UI_StateMachine = UI_HOME_STATE;
              CoPostSem(semInkUI);
              if(config.flags & FLAG_TRACE_UI_STATEMACHINE) TRACE("Battery -> Home\r\n");
              setHomeScreen(beaconName,addressfield);
              setStringerGraphics(led_flags);
              WaitGrabEInk();
              update(fullrefresh);
              ReleaseEInk();
            }
            else if(UI_BUTTONdata&UI_BTNBFLAG){
              UI_StateMachine = UI_STATUS_RADIO_STATE;
              CoPostSem(semInkUI);
              if(config.flags & FLAG_TRACE_UI_STATEMACHINE) TRACE("A btn: status battery -> status radio\r\n");
              //GFX_FillBuffer(black);
						  GFX_DrawBitMap(UISTATUS_COORD,status_radio_static, statusscreen_dimensions);
              setFont(smallfont);
              setCursor(UIHOME_DURATION_COORD);
              GFX_Print("8.00H", strlen("8.00H"),leftalign, black, gray_l);
              WaitGrabEInk();
              update(fullrefresh);
              ReleaseEInk();              
            }CoPostSem(semInkUI);
          break;
#if 0
          case UI_ID_STATE:
            //tradeshow: still screen. Any button returns to Home state
            UI_StateMachine = UI_HOME_STATE;
            CoPostSem(semInkUI);
            if(config.flags & FLAG_TRACE_UI_STATEMACHINE) TRACE("ID -> Home\r\n");
          break;
#endif
        }
        UI_BUTTONdata=0;
      }
      else if(getFlags == 1<<flagLED_UI){
        CoClearFlag(flagLED_UI);
        CoPendSem(semInkUI, 0);
        
        if((UI_IRLEDdata & UI_IRLED0FLAG)>>0 != (led_flags & UI_IRLED0FLAG) >>0){
          if(config.flags & FLAG_TRACE_UI_STATEMACHINE) TRACE("update irled0\r\n");
        }
        if((UI_IRLEDdata & UI_IRLED1FLAG)>>1 != (led_flags & UI_IRLED1FLAG) >>1){
          if(config.flags & FLAG_TRACE_UI_STATEMACHINE) TRACE("update irled1\r\n");
        }
        if((UI_IRLEDdata & UI_IRLED2FLAG)>>2 != (led_flags & UI_IRLED2FLAG) >>2){
          if(config.flags & FLAG_TRACE_UI_STATEMACHINE) TRACE("update irled2\r\n");
        }
        led_flags = UI_IRLEDdata;
        CoPostSem(semInkUI);
        if(UI_StateMachine == UI_HOME_STATE){ //for sanity
          setHomeScreen(beaconName,addressfield);
          setStringerGraphics(led_flags);
          WaitGrabEInk();
          update(partialrefresh);
          ReleaseEInk();
        }
      }
      else if(getFlags == 1<<flagTimeout_UI){
        CoClearFlag(flagTimeout_UI);
        if(UI_StateMachine == UI_MENU_STATE){
          UI_StateMachine = UI_HOME_STATE;
          setHomeScreen(beaconName,addressfield);
          setStringerGraphics(led_flags);
          WaitGrabEInk();
          update(fullrefresh);
          ReleaseEInk();
        }
      }
      else{ //if(getFlags == 1<<flagUSBdata_UI){
        CoClearFlag(flagUSBdata_UI);
        WaitGrabI2C();
        I2C_EE_BufferRead(&beaconName[0], 128, 12);
        ReleaseI2C();
        CoPendSem(semInkUI,0);
        sprintf(addressfield, "ID%i", config.mySrcAddr);
        CoPostSem(semInkUI);
        if(UI_StateMachine == UI_HOME_STATE){ //for sanity
          setHomeScreen(beaconName,addressfield);
          setStringerGraphics(led_flags);
          WaitGrabEInk();
          update(fullrefresh);
          ReleaseEInk();
        }
      }
    }
    else{
      if(config.flags & FLAG_TRACE_UI_STATEMACHINE) TRACE("result was error\r\n");
    }
  }
}

static void setHomeScreen(uint8_t stringname[], uint8_t addr[]){
  uint8_t * pos;
  uint8_t len;
  GFX_FillBuffer(black);
  GFX_DrawBitMap(UI_BATTERY_TEXT_COORD, battery100, battery_text_dimensions);
  fillRoundRect(27, 136, button_dimensions, 5, gray_d);
  setFont(smallfont);
  setCursor(UIHOME_DURATION_COORD);
  GFX_Print("8.00H", strlen("8.00H"),leftalign, black, gray_l);
  setFont(largefont);
  setCursor(UIHOME_NAMETOP_COORD);
  pos=strchr(stringname, ' ');
  if(pos==NULL){
    GFX_Print(stringname, strlen(stringname),centeralign, black, gray_l);
  }
  else{
    GFX_Print(stringname, pos-stringname, centeralign, black, gray_l);
    setCursor(UIHOME_NAMEBOTTOM_COORD);
    len=strlen(stringname)-(++pos-stringname);
    GFX_Print(pos,len,centeralign,black,gray_l);
  }
  setCursor(UIHOME_ID_COORD);
  GFX_Print(addr, strlen(addr),centeralign, gray_d, white);
  
}

static void setStringerGraphics(uint8_t led_flags){
  if(led_flags & UI_IRLED0FLAG)
    GFX_DrawBitMap(UIHOME_STRINGER1_COORD, stringer1enable, stringer_dimensions);
  else
    GFX_DrawBitMap(UIHOME_STRINGER1_COORD, stringer1disable, stringer_dimensions);
  
  if(led_flags & UI_IRLED1FLAG)
    GFX_DrawBitMap(UIHOME_STRINGER2_COORD, stringer2enable, stringer_dimensions);
  else  
    GFX_DrawBitMap(UIHOME_STRINGER2_COORD, stringer2disable, stringer_dimensions);
  
  if(led_flags & UI_IRLED2FLAG)
    GFX_DrawBitMap(UIHOME_STRINGER3_COORD, stringer3enable, stringer_dimensions);
  else
    GFX_DrawBitMap(UIHOME_STRINGER3_COORD, stringer3disable, stringer_dimensions);
}

static void resetUITimeout(void){
  TIM7->CNT=0;
  TIM_Cmd(TIM7,ENABLE);
}

static void disableUITimeout(void){
  TIM_Cmd(TIM7,DISABLE);
}
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
