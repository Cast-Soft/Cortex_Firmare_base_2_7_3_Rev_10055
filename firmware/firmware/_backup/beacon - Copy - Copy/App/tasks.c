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

/* PRIVATE TYPEDEF -----------------------------------------------------------*/

typedef struct {
    uint16_t    dstAddr;
    uint8_t     payloadSize;
    uint8_t     payload[FRAME_LENGTH_MAX - MHR_SIZE - MFR_SIZE];
} txPktRec_t;

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

/**  IMU LSM330DLC definitions **/

#define IMU_CTRL_REG1         0x20
#define IMU_CTRL_REG2         0x21
#define IMU_CTRL_REG3         0x22
#define IMU_CTRL_REG4         0x23
#define IMU_CTRL_REG5         0x24
#define IMU_WHOAMI_REG        0x0F
#define IMU_IM_LSM330         0xD4

#define IMU_A_RATE_1HZ          0x10
#define IMU_A_RATE_10HZ         0x20
#define IMU_A_RATE_25HZ         0x30
#define IMU_A_RATE_50HZ         0x40
#define IMU_A_RATE_100HZ        0x50
#define IMU_A_RATE_200HZ        0x60
#define IMU_A_RATE_400HZ        0x70

#define IMU_A_AXIS_X_EN         0x01
#define IMU_A_AXIS_Y_EN         0x02
#define IMU_A_AXIS_Z_EN         0x04

#define IMU_BIG_ENDIAN          0x40
#define IMU_H_RESOL             0x08

#define BTNPRESSED    1
#define BTNDEPRESSED  0

#define BTN_BURST_SEND 5

#define NUM_TX_RETRIES 4

#define DEBOUNCE_MIN    1
#define DEBOUNCE_MAX    9
#define DBLCLICK_MIN_TIME       1
#define DBLCLICK_MAX_TIME       9

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

extern OS_FlagID flagIMUNewData;
extern OS_FlagID flagIMU_G_DRDY;
extern OS_FlagID flagRadioTxReq;
extern OS_FlagID flagIMUTimeToSend;
extern OS_FlagID flagRadioCCA;

extern uint32_t drift;
extern int32_t last_drift;

extern OS_EventID semIMUAllow;
extern OS_FlagID   flagIMUDataReady;
extern OS_FlagID   flagBtnDataReady;
extern OS_FlagID   flagBatDataReady;
extern OS_FlagID   flagLEDDataReady;

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
extern uint16_t tim2_phase;
extern uint16_t tim3_phase;
//extern uint32_t syncPackets;

extern uint16_t adjusted;
extern uint8_t adjusted_changed;


int TRACE(char* fmt, ...);
size_t __writeIMU(const unsigned char *buffer, size_t size);
extern void RadioIMU_WaitGrabSPI();
extern void RadioIMU_ReleaseSPI(void);

void IMUWaitGrabSPI();
void IMUReleaseSPI();

size_t __writeCmdLineRespPacket(const unsigned char *buffer, size_t size, uint8_t contentType);
uint16_t tim_at_sec;

uint32_t rt_flags;

/* PRIVATE VARIABLES ---------------------------------------------------------*/
//static uint16_t tim4MovAvgMin = UINT16_MAX, tim4MovAvgMax = 0;
static volatile uint16_t halted = 0;
static uint8_t beaconInSync = 0;
static uint16_t txCount;        // ping count
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

static struct Beacon_BatData lastBatStatus;

uint8_t          frameIdFlag = 1;
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

/* PUBLIC VARIABLES ----------------------------------------------------------*/


__no_init uint32_t random;

config_t config;
tButton buttonA;
tButton buttonB;

struct realTime oldFrameTime;
struct realTime newFrameTime;

uint32_t realFrameId;
uint32_t lastFrameIdAtSync;
uint32_t lastFrameIdCorrection;
int8_t frameAdjust = 4;
uint32_t successBeacons;
uint16_t routerAddr = 0;
uint8_t use_sync = 1;
uint16_t RfTxLevel;
uint32_t IMUdbgPrt = 0;
uint16_t frameOffset;
uint8_t bat_slot_numbers;
static uint32_t bat_send_time;

tBtn_State btnA;
tBtn_State btnB;
//IMUData IMU_Packets;

//I2C_ADDR_TypeDef ADDR_I2C;

uint8_t button_state;
uint32_t last_bat_sent;

uint8_t IMUPresent = 0;   // imu present absent flag
uint8_t test_imu_pkt_ctr = 0; // IMU packet ctr from IMU interrupt

uint16_t txTimeSlot = 40000;

uint8_t firstTime = 1;  // first need to initialize timer of time slot
uint16_t random_slot1 = 0xFFFF;
uint16_t random_slot2 = 0xFFFF;

/* EXTERNAL FUNCTION PROTOTYPES ---------------------------------------------*/
uint8_t IMURegRd(uint8_t addr, HwSPI_TypeDef Sensor);
void IMURegWr(uint8_t addr, HwSPI_TypeDef Sensor, uint8_t val);

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/
static void GetAverageImuData(uint16_t *pBuff);
static void InputDataIntoBuffer(uint8_t volatile *pImuData);

static void mygets(char *str);
//static void IMURegWr(uint8_t addr, uint16_t val);
//static uint16_t IMURegRd(uint8_t addr);
static void SetConfig(uint16_t idx, uint32_t val, uint32_t pattern, uint8_t ledBits, uint8_t ledId);
static void PrintConfig(void);

static uint8_t RadioTxPktQueue(uint16_t dstAddr, uint8_t payloadSize, uint8_t *payload);
uint16_t fastdivide250(uint16_t);

static uint8_t __task1_imu_pack(uint8_t * ptr);
static uint8_t __task1_btn_pack(uint8_t * ptr);
static uint8_t __task1_btnimu_pack(uint8_t * ptr);
static uint8_t __task1_bat_pack(uint8_t * ptr);
static uint8_t __task1_batimu_pack(uint8_t * ptr);
static uint8_t __task1_batbtn_pack(uint8_t * ptr);
static uint8_t __task1_batbtnimu_pack(uint8_t * ptr);
static uint8_t __task1_led_pack(uint8_t * ptr);
static uint8_t __task1_ledimu_pack(uint8_t * ptr);
static uint8_t __task1_ledbtn_pack(uint8_t * ptr);
static uint8_t __task1_ledbtnimu_pack(uint8_t * ptr);
static uint8_t __task1_ledbat_pack(uint8_t * ptr);
static uint8_t __task1_ledbatimu_pack(uint8_t * ptr);
static uint8_t __task1_ledbatbtn_pack(uint8_t * ptr);
static uint8_t __task1_ledbatbtnimu_pack(uint8_t * ptr);
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


uint16_t fastdivide250(uint16_t val){
  uint32_t x = ((val*0x8312)>>16)>>7;
  return (uint16_t) x;
}
/*******************************************************************************
* Description : Inputs the XYZ data into IMU Buffer
* Input       :   Buffer of Data and Buffer
* Return      : -
*******************************************************************************/
static void InputDataIntoBuffer(uint8_t volatile *pImuData)
{
	ImuBuffer_t volatile *pImuBuff = &ImuGyroBuffer;
	pImuBuff->xSum = (*(uint16_t*)&pImuData[6])/250;
	pImuBuff->ySum = (*(uint16_t*)&pImuData[8])/250;
	pImuBuff->zSum = (*(uint16_t*)&pImuData[10])/250;
	pImuBuff->count = 1;
	
	pImuBuff = &ImuAccelBuffer;
	pImuBuff->xSum = *((uint16_t*)&pImuData[0]) >>1;
	pImuBuff->ySum = *((uint16_t*)&pImuData[2]) >>1;
	pImuBuff->zSum = *((uint16_t*)&pImuData[4]) >>1;
	pImuBuff->count = 1;
  //x__disable_interrupt();
  
  
  //x__enable_interrupt();
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


/*
#define RADIOPACKET_IMU         0x01
#define RADIOPACKET_BUTTONPRESS 0x02
#define RADIOPACKET_BATTERY     0x04
*/
typedef enum{
  no_msg_flag     = 0x00,
  imu_packet_flag = 0x01,
  btn_packet_flag = 0x02,
  bat_packet_flag = 0x04,
  led_packet_flag = 0x08
}packetdatacheck;

typedef enum{
  no_msg =0,
  imu_msg=1,
  btn_msg=2,
  btn_imu_msg=3,
  bat_msg=4,
  bat_imu_msg=5,
  bat_btn_msg=6,
  bat_btn_imu_msg=7,
  led_msg=8,
  led_imu_msg=9,
  led_btn_msg=10,
  led_btn_imu_msg=11,
  led_bat_msg=12,
  led_bat_imu_msg=13,
  led_bat_btn_msg=14,
  led_bat_btn_imu_msg=15
}messagetype;


uint8_t filltxbuf(uint8_t messagetype,uint8_t * ptr){
  uint8_t size=0;
  switch(messagetype){
    case  imu_msg:
      size = __task1_imu_pack(ptr);
      break;
    case  btn_msg:
      size = __task1_btn_pack(ptr);
      break;
    case  btn_imu_msg:
      size = __task1_btnimu_pack(ptr);
      break;
    case  bat_msg:
      size = __task1_bat_pack(ptr);
      break;
    case  bat_imu_msg:
      size = __task1_batimu_pack(ptr);
      break;
    case  bat_btn_msg:
      size = __task1_batbtn_pack(ptr);
      break;
    case  bat_btn_imu_msg:
      size = __task1_batbtnimu_pack(ptr);
      break;
    case  led_msg:
      size = __task1_led_pack(ptr);
      break;
    case  led_imu_msg:
      size = __task1_ledimu_pack(ptr);
      break;
    case  led_btn_msg:
      size = __task1_ledbtn_pack(ptr);
      break;
    case  led_btn_imu_msg:
      size = __task1_ledbtnimu_pack(ptr);
      break;
    case  led_bat_msg:
      size = __task1_ledbat_pack(ptr);
      break;
    case  led_bat_imu_msg:
      size = __task1_ledbatimu_pack(ptr);
      break;
    case  led_bat_btn_msg:
      size = __task1_ledbatbtn_pack(ptr);
      break;
    case  led_bat_btn_imu_msg:
      size = __task1_ledbatbtnimu_pack(ptr);
      break; 
    default:
      size = 0;
      break;  
  }
  return size;
}

void fillmsgPreamble(uint8_t*ptr,uint8_t msgtype){
  static uint8_t seqNum = 0;
  SAVE_POINT
#ifdef msgmethod1
  ptr[0]= seqNum;
  ptr[1]= msgtype;
  ptr[2]= beaconRSSI;
  ptr[3]= frameIdAtSync;
  ptr[4]= MsTimerAtSync;
  ptr[5]= IMUPktNumAtSync;
#else
  struct Beacon_Preamble * dataptr;
  dataptr = (struct Beacon_Preamble*) ptr;

  dataptr->seq_Num= seqNum;
  dataptr->type = msgtype;
  dataptr->BeaconRSSI= beaconRSSI;
  pBeacon_Data_pkt->SyncFrameIMU=frameIdAtSync;
  pBeacon_Data_pkt->MsTimerIMU=MsTimerAtSync;
  pBeacon_Data_pkt->IMUPktNum=IMUPktNumAtSync;
#endif
  SAVE_POINT
  seqNum++;
}
#if 0
void fillmsgIMU(struct BK_IMUData*ptr){
  uint16_t * dataptr;
  uint8_t i = 0;
  uint8_t last_test_imu_pkt_ctr = 255;

  while(i < NUM_OF_IMU_PKTS_IN_RF_PKT){
      SAVE_POINT
      dataptr = (uint16_t*) &ptr->gyroscopeX;
      CoWaitForSingleFlag(flagIMUDataReady,0);
      CoClearFlag(flagIMUDataReady);
      GetAverageImuData(dataptr);
      last_test_imu_pkt_ctr = test_imu_pkt_ctr;
      dataptr->Timestamp = test_imu_pkt_ctr;
  }
  SAVE_POINT
}
#endif

void fillmsgBTN(uint8_t*ptr){
  SAVE_POINT
#ifdef msgmethod1
  ptr[0] = btnA.message;      //button A event
  ptr[1] = btnB.message;      //button B event
  ptr[2] = btnA.sequence;     //button A sequence 
  ptr[3] = btnB.sequence;     //button B sequence
#else
  struct BK_BTNData * dataptr;
  dataptr = (struct BK_BTNData*) ptr;
  dataptr->buttonA_events = btnA.message;
  dataptr->buttonB_events = btnB.message;
  dataptr->buttonA_tick = btnA.sequence;
  dataptr->buttonB_tick = btnB.sequence;
#endif
  SAVE_POINT
  if(btnA.mcount!=0) btnA.mcount--;
  if(btnB.mcount!=0) btnB.mcount--;
  if(btnA.mcount==0 && btnB.mcount==0){
    CoClearFlag(flagBtnDataReady);
  }
  SAVE_POINT
}
void fillmsgBAT(uint8_t*ptr){
#ifdef msgmethod1
  ptr[0] = 0;               //battery_lev
  ptr[1] = 0;               //charge_cycle
#else 
  struct BK_BATData * dataptr;
  dataptr = (struct BK_BATData*) ptr;
  dataptr->Battery_lev = 0; //TBA
  dataptr->charge_cycle = 0;//TBA
#endif
  CoClearFlag(flagBatDataReady);
  SAVE_POINT
}

void fillmsgLED(uint8_t*ptr){
  SAVE_POINT
#ifdef msgmethod1
  ptr[0]= 0;              //led1id tbd
  ptr[1]= 0;              //led2id tbd
  ptr[2]= 0;              //led3id tbd
  ptr[3]= 0;              //led1stat tbd
  ptr[4]= 0;              //led2stat tbd
  ptr[5]= 0;              //led3stat tbd
#else 
  struct BK_LEDData * dataptr;
  dataptr = (struct BK_LEDData*) ptr;
  dataptr->led1ID = 0;
  dataptr->led2ID = 0;
  dataptr->led3ID = 0;
  dataptr->led1stat = 0;
  dataptr->led2stat = 0;
  dataptr->led3stat = 0;
#endif
  CoClearFlag(flagLEDDataReady);
  SAVE_POINT
}

static uint8_t __task1_imu_pack(uint8_t * ptr){
  struct IMU_Data_pkt *pData_pkt;
  pData_pkt = (struct IMU_Data_pkt*) ptr;
  uint16_t * pBuf;
  uint8_t i=0;
  uint8_t last_test_imu_pkt_ctr = 255;

  fillmsgPreamble(&pData_pkt->BK_Preamble.Seq_Num,BC_IMU_PacketFrame);
  while(i< NUM_OF_IMU_PKTS_IN_RF_PKT){
    SAVE_POINT
    pBuf = (uint16_t*) &pData_pkt->BeaconIMUData[i].gyroscopeX;
    CoWaitForSingleFlag(flagIMUDataReady, 0);
    SAVE_POINT
    GetAverageImuData(pBuf);
    last_test_imu_pkt_ctr = test_imu_pkt_ctr;
    pData_pkt->BeaconIMUData[i].Timestamp = test_imu_pkt_ctr;
    ++i;
  }
  return sizeof(struct IMU_Data_pkt);
}

static uint8_t __task1_btn_pack(uint8_t * ptr){
  struct BTN_Data_pkt *pData_pkt;
  pData_pkt = (struct BTN_Data_pkt*) ptr;
  fillmsgPreamble(&pData_pkt->BK_Preamble.Seq_Num,BC_BTN_PacketFrame);
  fillmsgBTN((uint8_t*) &pData_pkt->BeaconBTNData.buttonA_events);

  return sizeof(struct BTN_Data_pkt);
}

static uint8_t __task1_btnimu_pack(uint8_t * ptr){
  struct BTN_IMU_Data_pkt *pData_pkt;
  pData_pkt = (struct BTN_IMU_Data_pkt*) ptr;
  uint16_t * pBuf;
  uint8_t i=0;
  uint8_t last_test_imu_pkt_ctr = 255;
  fillmsgPreamble(&pData_pkt->BK_Preamble.Seq_Num,BC_BTN_IMU_PacketFrame);

  while(i< NUM_OF_IMU_PKTS_IN_RF_PKT){
    SAVE_POINT
    pBuf = (uint16_t*) &pData_pkt->BeaconIMUData[i].gyroscopeX;
    CoWaitForSingleFlag(flagIMUDataReady, 0);
    SAVE_POINT
    GetAverageImuData(pBuf);
    last_test_imu_pkt_ctr = test_imu_pkt_ctr;
    pData_pkt->BeaconIMUData[i].Timestamp = test_imu_pkt_ctr;
    ++i;
  }
  fillmsgBTN((uint8_t*) &pData_pkt->BeaconBTNData.buttonA_events);

  return sizeof(struct BTN_IMU_Data_pkt);
}

static uint8_t __task1_bat_pack(uint8_t * ptr){
  struct BAT_Data_pkt *pData_pkt;
  pData_pkt = (struct BAT_Data_pkt*) ptr; 
  fillmsgPreamble(&pData_pkt->BK_Preamble.Seq_Num,BC_BAT_PacketFrame);

  fillmsgBAT((uint8_t*) &pData_pkt->BeaconBATData.Battery_lev);

  return sizeof(struct BAT_Data_pkt);
}

static uint8_t __task1_batimu_pack(uint8_t * ptr){
  struct BAT_IMU_Data_pkt *pData_pkt;
  pData_pkt = (struct BAT_IMU_Data_pkt*) ptr;
  uint16_t * pBuf;
  uint8_t i=0;
  uint8_t last_test_imu_pkt_ctr = 255;
  fillmsgPreamble(&pData_pkt->BK_Preamble.Seq_Num,BC_BAT_IMU_PacketFrame);

  while(i< NUM_OF_IMU_PKTS_IN_RF_PKT){
    SAVE_POINT
    pBuf = (uint16_t*) &pData_pkt->BeaconIMUData[i].gyroscopeX;
    CoWaitForSingleFlag(flagIMUDataReady, 0);
    SAVE_POINT
    GetAverageImuData(pBuf);
    last_test_imu_pkt_ctr = test_imu_pkt_ctr;
    pData_pkt->BeaconIMUData[i].Timestamp = test_imu_pkt_ctr;
    ++i;
  }
  fillmsgBAT((uint8_t*) &pData_pkt->BeaconBATData.Battery_lev);

  return sizeof(struct BAT_IMU_Data_pkt);
}

static uint8_t __task1_batbtn_pack(uint8_t * ptr){
  struct BAT_BTN_Data_pkt *pData_pkt;
  pData_pkt = (struct BAT_BTN_Data_pkt*) ptr;
  fillmsgPreamble(&pData_pkt->BK_Preamble.Seq_Num,BC_BAT_BTN_PacketFrame);

  fillmsgBTN((uint8_t*) &pData_pkt->BeaconBTNData.buttonA_events);
  fillmsgBAT((uint8_t*) &pData_pkt->BeaconBATData.Battery_lev);

  return sizeof(struct BAT_BTN_Data_pkt);
}

static uint8_t __task1_batbtnimu_pack(uint8_t * ptr){
  struct BAT_BTN_IMU_Data_pkt *pData_pkt;
  pData_pkt = (struct BAT_BTN_IMU_Data_pkt*) ptr;
  uint16_t * pBuf;
  uint8_t i=0;
  uint8_t last_test_imu_pkt_ctr = 255;
  fillmsgPreamble(&pData_pkt->BK_Preamble.Seq_Num,BC_BAT_BTN_IMU_PacketFrame);

  while(i< NUM_OF_IMU_PKTS_IN_RF_PKT){
    SAVE_POINT
    pBuf = (uint16_t*) &pData_pkt->BeaconIMUData[i].gyroscopeX;
    CoWaitForSingleFlag(flagIMUDataReady, 0);
    SAVE_POINT
    GetAverageImuData(pBuf);
    last_test_imu_pkt_ctr = test_imu_pkt_ctr;
    pData_pkt->BeaconIMUData[i].Timestamp = test_imu_pkt_ctr;
    ++i;
  }
  fillmsgBTN((uint8_t*) &pData_pkt->BeaconBTNData.buttonA_events);
  fillmsgBAT((uint8_t*) &pData_pkt->BeaconBATData.Battery_lev);

  return sizeof(struct BAT_BTN_IMU_Data_pkt);
}

static uint8_t __task1_led_pack(uint8_t * ptr){
  struct LED_Data_pkt *pData_pkt;
  pData_pkt = (struct LED_Data_pkt*) ptr;
  fillmsgPreamble(&pData_pkt->BK_Preamble.Seq_Num,BC_LED_PacketFrame);

  fillmsgLED((uint8_t*) &pData_pkt->BeaconLEDData.led1ID);

  return sizeof(struct LED_Data_pkt);
}

static uint8_t __task1_ledimu_pack(uint8_t * ptr){
  struct LED_IMU_Data_pkt *pData_pkt;
  pData_pkt = (struct LED_IMU_Data_pkt*) ptr;
  uint16_t * pBuf;
  uint8_t i=0;
  uint8_t last_test_imu_pkt_ctr = 255;
  fillmsgPreamble(&pData_pkt->BK_Preamble.Seq_Num,BC_LED_IMU_PacketFrame);

  while(i< NUM_OF_IMU_PKTS_IN_RF_PKT){
    SAVE_POINT
    pBuf = (uint16_t*) &pData_pkt->BeaconIMUData[i].gyroscopeX;
    CoWaitForSingleFlag(flagIMUDataReady, 0);
    SAVE_POINT
    GetAverageImuData(pBuf);
    last_test_imu_pkt_ctr = test_imu_pkt_ctr;
    pData_pkt->BeaconIMUData[i].Timestamp = test_imu_pkt_ctr;
    ++i;
  }
  fillmsgLED((uint8_t*) &pData_pkt->BeaconLEDData.led1ID);

  return sizeof(struct LED_IMU_Data_pkt);
}

static uint8_t __task1_ledbtn_pack(uint8_t * ptr){
  struct LED_BTN_Data_pkt *pData_pkt;
  pData_pkt = (struct LED_BTN_Data_pkt*) ptr;
  fillmsgPreamble(&pData_pkt->BK_Preamble.Seq_Num,BC_LED_BTN_PacketFrame);

  fillmsgBTN((uint8_t*) &pData_pkt->BeaconBTNData.buttonA_events);
  fillmsgLED((uint8_t*) &pData_pkt->BeaconLEDData.led1ID);

  return sizeof(struct LED_BTN_Data_pkt);
}

static uint8_t __task1_ledbtnimu_pack(uint8_t * ptr){
  struct LED_BTN_IMU_Data_pkt *pData_pkt;
  pData_pkt = (struct LED_BTN_IMU_Data_pkt*) ptr;
  uint16_t * pBuf;
  uint8_t i=0;
  uint8_t last_test_imu_pkt_ctr = 255;
  
  fillmsgPreamble(&pData_pkt->BK_Preamble.Seq_Num,BC_LED_BTN_IMU_PacketFrame);

  while(i< NUM_OF_IMU_PKTS_IN_RF_PKT){
    SAVE_POINT
    pBuf = (uint16_t*) &pData_pkt->BeaconIMUData[i].gyroscopeX;
    CoWaitForSingleFlag(flagIMUDataReady, 0);
    SAVE_POINT
    GetAverageImuData(pBuf);
    last_test_imu_pkt_ctr = test_imu_pkt_ctr;
    pData_pkt->BeaconIMUData[i].Timestamp = test_imu_pkt_ctr;
    ++i;
  }
  fillmsgBTN((uint8_t*) &pData_pkt->BeaconBTNData.buttonA_events);
  fillmsgLED((uint8_t*) &pData_pkt->BeaconLEDData.led1ID);

  return sizeof(struct LED_BTN_IMU_Data_pkt);
}

static uint8_t __task1_ledbat_pack(uint8_t * ptr){
  struct LED_BAT_Data_pkt *pData_pkt;
  pData_pkt = (struct LED_BAT_Data_pkt*) ptr;  
  fillmsgPreamble(&pData_pkt->BK_Preamble.Seq_Num,BC_LED_BAT_PacketFrame);

  fillmsgBAT((uint8_t*) &pData_pkt->BeaconBATData.Battery_lev);
  fillmsgLED((uint8_t*) &pData_pkt->BeaconLEDData.led1ID);

  return sizeof(struct LED_BAT_Data_pkt);
}

static uint8_t __task1_ledbatimu_pack(uint8_t * ptr){
  struct LED_BAT_IMU_Data_pkt *pData_pkt;
  pData_pkt = (struct LED_BAT_IMU_Data_pkt*) ptr;
  uint16_t * pBuf;
  uint8_t i=0;
  uint8_t last_test_imu_pkt_ctr = 255;
  fillmsgPreamble(&pData_pkt->BK_Preamble.Seq_Num,BC_LED_BAT_IMU_PacketFrame);

  while(i< NUM_OF_IMU_PKTS_IN_RF_PKT){
    SAVE_POINT
    pBuf = (uint16_t*) &pData_pkt->BeaconIMUData[i].gyroscopeX;
    CoWaitForSingleFlag(flagIMUDataReady, 0);
    SAVE_POINT
    GetAverageImuData(pBuf);
    last_test_imu_pkt_ctr = test_imu_pkt_ctr;
    pData_pkt->BeaconIMUData[i].Timestamp = test_imu_pkt_ctr;
    ++i;
  }
  fillmsgBAT((uint8_t*) &pData_pkt->BeaconBATData.Battery_lev);
  fillmsgLED((uint8_t*) &pData_pkt->BeaconLEDData.led1ID);

  return sizeof(struct LED_BAT_IMU_Data_pkt);
}

static uint8_t __task1_ledbatbtn_pack(uint8_t * ptr){
  struct LED_BAT_BTN_Data_pkt *pData_pkt;
  pData_pkt = (struct LED_BAT_BTN_Data_pkt*) ptr;
  fillmsgPreamble(&pData_pkt->BK_Preamble.Seq_Num,BC_LED_BAT_BTN_PacketFrame);

  fillmsgBTN((uint8_t*) &pData_pkt->BeaconBTNData.buttonA_events);
  fillmsgBAT((uint8_t*) &pData_pkt->BeaconBATData.Battery_lev);
  fillmsgLED((uint8_t*) &pData_pkt->BeaconLEDData.led1ID);

  return sizeof(struct LED_BAT_BTN_Data_pkt);
}

static uint8_t __task1_ledbatbtnimu_pack(uint8_t * ptr){
  struct LED_BAT_BTN_IMU_Data_pkt *pData_pkt;
  pData_pkt = (struct LED_BAT_BTN_IMU_Data_pkt*) ptr; 
  uint16_t * pBuf;
  uint8_t i=0;
  uint8_t last_test_imu_pkt_ctr = 255;
  
  fillmsgPreamble(&pData_pkt->BK_Preamble.Seq_Num,BC_LED_BAT_BTN_IMU_PacketFrame);
  while(i< NUM_OF_IMU_PKTS_IN_RF_PKT){
    SAVE_POINT
    pBuf = (uint16_t*) &pData_pkt->BeaconIMUData[i].gyroscopeX;
    CoWaitForSingleFlag(flagIMUDataReady, 0);
    SAVE_POINT
    GetAverageImuData(pBuf);
    last_test_imu_pkt_ctr = test_imu_pkt_ctr;
    pData_pkt->BeaconIMUData[i].Timestamp = test_imu_pkt_ctr;
    ++i;
  }
  fillmsgBTN((uint8_t*) &pData_pkt->BeaconBTNData.buttonA_events);
  fillmsgBAT((uint8_t*) &pData_pkt->BeaconBATData.Battery_lev);
  fillmsgLED((uint8_t*) &pData_pkt->BeaconLEDData.led1ID);

  return sizeof(struct LED_BAT_BTN_IMU_Data_pkt);
}
/*******************************************************************************
* Description    : [Task] Process and Send each new IMU data sample
* Input          :
* Return         :
*******************************************************************************/
void Task1(void* pdata){
  uint8_t messagetype;
  static uint16_t *pBuf;
  static uint8_t bufsize;
  StatusType err_det;
  static uint32_t target;

  while (1) {
    RELOAD_WATCHDOG
    SAVE_POINT
    //wait for a message that came from other tasks
    messagetype = no_msg_flag;
    if(!(config.radioPacketFlags & RADIOPACKET_IMU)){
      target = CoWaitForMultipleFlags(flagIMUDataReady|flagBtnDataReady|flagLEDDataReady|flagBatDataReady,
                                      OPT_WAIT_ANY,     //waits for any of the flags to occur
                                      0,                //wait indefinitely until a flag occurs
                                      &err_det);        //error pointer
      if(target == flagIMUDataReady){
        messagetype |= imu_packet_flag;   //in case config flag was asserted during waiting period & no other event occurs
      }
    }
    else{
      messagetype |= imu_packet_flag;
    }
    if(CoAcceptSingleFlag(flagBtnDataReady)==E_OK){
      messagetype |= btn_packet_flag;
    }
    if(CoAcceptSingleFlag(flagBatDataReady)==E_OK){
      messagetype |= bat_packet_flag;
    }
    if(CoAcceptSingleFlag(flagLEDDataReady)==E_OK){
      messagetype |= led_packet_flag;
    }
    SAVE_POINT 
    bufsize = filltxbuf(messagetype, txBuf); //Fill out temp buffer with the appropriate data.  

    SAVE_POINT
    if(bufsize != 0){     //no message found. for peace of mind.
      CoPendSem(semIMUAllow, 0);
      SAVE_POINT
      calls++;
      uint8_t pushed = 0;
      if (IMUdbgPrt) {         // It is for USB output now
        __writeIMU((unsigned char*) txBuf, bufsize);
      }
      else {
        // conditional send based on radio packets flags;
        if(!(pushed = RadioTxPktQueue(routerAddr, bufsize , txBuf))) {
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
      if (!pushed || radio_off) {
        semAllowPostTask = CoPostSem(semIMUAllow);
        semAllow = 2;
      }
    }
    SAVE_POINT
    //last_button_state = pBeacon_Data_pkt->BK_Preamble.button_pr;
    //battery_minutes =  secs/600;
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
uint8_t got_beacon = 0;

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
      frameIdFlag = 1;
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
      if (pRxPkt != NULL) {
          SAVE_LINE
          rxNotEmpty++;
          // Check for Beacon frame types from the TimeKeeper.  Ignore other data packets
          // from beacons
            if ((pRxPkt->fcf0 & 0x07) == 0 && pRxPkt->panId == config.panId
                && (pRxPkt->destAddr == 0xFFFF)) { // FCF[2:0] = 802.15.4 Beacon Frame Type
                  //additional check may be on source address
                /* RF Sync (Beacon) Packet */
#ifdef NEW_BEACON
                /* RF Sync (Beacon) Packet */
                if (pRxPkt->payloadSize == sizeof(struct BeaconOldStruct)) {
                    //check for new structure
                    // uint32_t frameId
                    // uint8_t  0xA5 //magic for this packet version
                    // uint8_t sec;  // consecutive sec, increments every sec
                    // uint8_t frameClock; //100, 120, 180, 240 support
                    // uint8_t crc8

                    struct BeaconOldStruct *beacon = (struct BeaconOldStruct*) pRxPkt->payload;
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
                            TRACE("Beacon: changing frameClock to %u\n\r", lastFrameClock);
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
                got_beacon = 0; //effectively disables frame correction in TIM3 interrupt
                tim2_phase = TIM2->CCR1;
                tim3_phase = TIM3->CCR1;
                trace_irq = 1;
                TIM2->CCR1 = 0;
                TIM3->CCR1 = 0;
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
                if (*(uint32_t*)pRxPkt->payload) { // ignore stale FrameIDs
                  realFrameId = *(uint32_t*)pRxPkt->payload;
                    frameIdCorrection = *(uint32_t*)pRxPkt->payload - frameIdAtSync + frameAdjust;/* - 4*/;//  + 2;
#ifdef CIRCULAR_LOG
                    __disable_interrupt();
                    W_LOG(CoGetOSTime(), LOG_TYPE_FRAMEADJUST, realFrameId);
                    W_LOG(CoGetOSTime(), LOG_TYPE_CAPTURE, tim3_phase);
                    W_LOG(CoGetOSTime(), LOG_TYPE_FRAMEDIFF, frameIdCorrection);
                    __enable_interrupt();
#endif
                    int corrected = 0;

                    if (/*(frameIdCorrection < 0) &&*/ (tim3_phase > (TIM3_AUTORELOAD >> 1))) {
                      frameIdCorrection++;
                      corrected = 1;
                    }
                    
                    if (config.flags & FLAG_TRACE_SYNC) {
                      TRACE(" Phase=%u inced=%u corr=%d\n\r", tim3_phase, frameIdInced, corrected);
                    }
                    
                    if (config.flags & FLAG_DEBUG) {
                      uint32_t tr = trace_irq;
                      TRACE("trace_irq=%d frIdCor=%d at tim3_phase=%u\n\r", tr, frameIdCorrection, tim3_phase);
                      TRACE("realFrameId=%u frameIdAtSync=%u\n\r", realFrameId, frameIdAtSync);
                    }
                    if (frameIdCorrection != lastFrameIdCorrection || corrected) {
                      oldFrameTime = lastFrameTime;
                      newFrameTime = frameTime;
                      newFrameIdAtSync = frameIdAtSync;
                      oldFrameIdAtSync = savedFrameIdAtSync;
                      newTim3Phase = tim3_phase;
                      oldTim3Phase = lastTim3Phase;
                      
                      if (config.flags & FLAG_TRACE_SYNC) {                      
                        TRACE("frIdCorr=%d lastFrIdCorr=%d @%d last @%d tim3_ph=%d\r\n", frameIdCorrection,
                              lastFrameIdCorrection, sec, lastTime, tim3_phase);
                        TRACE("realFrameId=%u frameIdAtSync=%u\n\r", realFrameId, frameIdAtSync);
                      }
                      
                      if (frameIdCorrection != 0 && sec > 20) {
                        frameIdCorrectionCount++;
                      }

                    } else {
                      static uint8_t seek = 0;
                      uint32_t rsec  = (frameIdAtSync-lastFrameIdAtSync)/lastFrameClock;
                      int32_t cur_diff = (tim3_phase/rsec);
                      lastFrameTime = frameTime;
                      savedFrameIdAtSync = frameIdAtSync;
                      lastTim3Phase = tim3_phase;

                    }
                    if (config.flags & FLAG_TRACE_ADJUST) {
                      TRACE("adjust=%d tim3_phase=%u tim4_phase=%u @%d.%d\r\n",
                            config.time_adjust,
                            tim3_phase, tim4_phase, sec, TIM1->CNT);
                    }
                    lastFrameIdCorrection = frameIdCorrection;
                    lastFrameIdAtSync = frameIdAtSync;
                    if ((sec - lastTime) > span) {
                      span = sec - lastTime;
                      TRACE("No TK sync for %d sec @ %d\r\n", span, sec);
                    }
                    lastTime = sec;
                    frameOffset = TIM1->CNT;
                    successBeacons++;
                    frameIdFlag = 1;
                    got_beacon = 1;
                }

                // record RSSI of Sync Packet to relay back to TK
                beaconRSSI =  0x5A; // temp change to store signature. // pRxPkt->rssi;

                // allow up to 3 tx pkts per sync pkt
                beaconInSync++;
                if(beaconInSync > 3)
                {
                  beaconInSync = 3;
                }

                //HwLEDToggle(LED4);
                //HwLEDOn(LED3);
              // Issue TODO - if Beacon out of syncronization,
              // it keeps trying to send (Task 6), may not receive
              // anything, as this task has lower priority
             //   CoAwakeTask(task1Id);
             //   CoAwakeTask(taskRadioTxId);
            }
      } else {
        if (!radio_off) {
        TRACE("NULL packet, rxCount=%u @sec=%d\r\n", rxCount, sec);
        }
      }
    }
}

/*******************************************************************************
* Description    : [Function] controls the targeted button's state machine
* Input          :
* Return         :
*******************************************************************************/
uint8_t chkbtnstate(tBtn_State *pbtn, uint8_t btnval){
  uint8_t val =0;
  switch(pbtn->state){
    case BTNSTATE_OFF:
      if(btnval == BTNPRESSED){
        pbtn->state = BTNSTATE_ON;
        pbtn->hcount=1;
        pbtn->prevstate=BTNSTATE_OFF;
      }
      break;
    case BTNSTATE_ON:
      if(btnval == BTNPRESSED){
        pbtn->hcount++;
        if(pbtn->hcount > 10){
          pbtn->state = BTNSTATE_HOLD;
          pbtn->hcount=0;
        }
      }
      else{
        pbtn->state = BTNSTATE_PRESSDETECT;
        pbtn->hcount = 0;
      }
      break;
    case BTNSTATE_HOLD:
      //configure sending message
      //enable sending with holding state.
      if(pbtn->prevstate == BTNSTATE_OFF){
        pbtn->prevstate = BTNSTATE_HOLD;
        //val=1;  //let task3 know to send msg. outside SOW;
        //pbtn->message = BTNMSG_HOLD;
      }
      if(btnval!=BTNPRESSED){
        pbtn->prevstate = BTNSTATE_OFF;
        pbtn->state = BTNSTATE_OFF;     //return to initialstate
        //pbtn->message = BTNMSG_RELEASE;
      } 
      break;
    case BTNSTATE_PRESSDETECT:
      if(btnval==BTNPRESSED){
      //configure sending message to doublepress
      //enable sending with this message
        //val=1;  //Let task3 know to send msg. ouside SOW;
        pbtn->prevstate = BTNSTATE_PRESSDETECT;
        pbtn->state = BTNSTATE_OFF;
        //pbtn->message = BTNMSG_DBLCLK;
      }
      else{
      //configure sending message to singlepress
      //enable sending with this message
        val=1;
        pbtn->prevstate = BTNSTATE_OFF;
        pbtn->state = BTNSTATE_OFF;
        pbtn->message = BTNMSG_PRESS;
        pbtn->mcount = BTN_BURST_SEND;
      }
      //return to initialstate
      break;
    case BTNSTATE_UICTRL:
      break;
  }
  return val;
}

/*******************************************************************************
* Description    : [Task] RF Chan Scan, then Monitor GPO_PWRON Power Switch
* Input          :
* Return         :
*******************************************************************************/
void Task3(void* pdata){
  static uint16_t pwr_holdCount = 0;
  static uint32_t btnval;
  static uint8_t btnmsg1;
  static uint8_t btnmsg2;


  rfChan = config.rfChan;
  /* search for RF Chan on Power-Up */
  static int wait_cnt = 0;

  while (1) {
    RELOAD_WATCHDOG
    SAVE_POINT
    if ((HwGPIState(GPI_SW_PWR))||((BattUnion.BatteryLevel[1])<=0xBB)){
#ifdef DISABLE_PWR_SW
      pwr_holdCount = 0;
#endif
      if (pwr_holdCount++ > 2) {
        CoSchedLock();
        //HwLEDOff(LED1); HwLEDOff(LED2); HwLEDOff(LED3); HwLEDOff(LED4);HwLEDOff(LED5);
#ifndef BC_HW_REVB
        HwGPOLow(GPO_5V_IMU_EN);
        HwGPOLow(GPO_RF_EN);
        HwGPOLow(GPO_VBATT_ADC_EN);
        HwGPOInitOC(GPO_USB_VBUS); // init the Vbus pin as output
        HwGPOLow(GPO_USB_VBUS);  // and then pull it low to ensure turnoff
#endif
        SAVE_POINT
        HwGPOLow(GPO_PWRON) ; // turn board OFF
        while (1);
      }
    }
    else{
      pwr_holdCount = 0;
    }
    break;
		
		//note that the IMU is already initialized prior to multithread start. it has already been counting.
		for(wait_cnt =0;wait_cnt < 40; wait_cnt++){
      CoTickDelay(10);
      SAVE_POINT
      if(test_imu_pkt_ctr >= 1) break;      // was > 10. Maybe the IMU interrupt is not enable ... TODO!!! check
    }
    //HwLEDToggle(LED2); HwLEDToggle(LED4);
    assert(test_imu_pkt_ctr >= 1);          // was > 10
    if(test_imu_pkt_ctr >= 1){              // was > 10
      IMUPresent=1;
      break;
    }else{
      IMUPresent=0;
       //EXTI->IMR &= ~GPI_IMU_DIO1_EXTI_LINE; // kill the interrupt
      halted=1;  // no imu tx
      break;
    }
	}
  routerAddr = config.routerDstAddr;  // NO RF Scan => send IMU data to TK address stored in FLASH Config
  SetTimeSlot();
  /* we're now in business */
  TRACE("RF CHAN: %d\n\r", rfChan);

  CoAwakeTask(task1Id);  // start imu process
  CoAwakeTask(taskRadioRxId);
  CoAwakeTask(taskIMUGId);
  
  btnA.state = BTNSTATE_OFF;
  btnB.state = BTNSTATE_OFF;
	while (1) {
		SAVE_POINT
   	CoTickDelay(100);  //either wait for an event or timeout to check Battery level
		SAVE_POINT
		RELOAD_WATCHDOG
#ifndef DISABLE_PWR_SW
		if ((HwGPIState(GPI_SW_PWR))||((BattUnion.BatteryLevel[1])<=0xBB)) {
			if (pwr_holdCount++ > 10) {
				CoSchedLock();
        //HwLEDOff(LED1); HwLEDOff(LED2); HwLEDOff(LED3); HwLEDOff(LED4);HwLEDOff(LED5);
        // This where we should sense if we are connected to the USB charger input
        // if so...we can't turn off so should go into some sort of low power mode
        // perhaps set the USB power in as an interrupt so if it goes away we can really shut down
				HwGPOLow(GPO_5V_IMU_EN);
				HwGPOLow(GPO_RF_EN);
				HwGPOLow(GPO_VBATT_ADC_EN);
				HwGPOInitOC(GPO_USB_VBUS);// init the Vbus pin as output
				HwGPOLow(GPO_USB_VBUS);// and then pull it low to ensure turnoff
				HwGPOLow(GPO_PWRON); // turn board OFF
				while (1);
      }
		} else {
			pwr_holdCount = 0;
		}
#endif // disable power switch
    SAVE_POINT
    btnmsg1=0;
    btnmsg2=0;

    btnval = HwButtonPressed(BUTTON1);
    btnmsg1=chkbtnstate(&btnA, btnval);
    SAVE_POINT
    btnval = HwButtonPressed(BUTTON2);
    btnmsg2=chkbtnstate(&btnB, btnval);
    SAVE_POINT

    if((btnmsg1||btnmsg2)&&config.radioPacketFlags & RADIOPACKET_BUTTONPRESS){ 
      CoSetFlag(flagBtnDataReady);
    }
    SAVE_POINT
    
#if 0
        if (IMUdbgPrt) {         // It is for USB output now
            __writeIMU((unsigned char*) pBeacon_Data_pkt, sizeof(struct Beacon_Data_pkt));
        }
        else {

          // conditional send based on radio packets flags;
          if ((config.radioPacketFlags & RADIOPACKET_IMU)) {
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
#endif
#if 0
	  //if(config.radioPacketFlags & RADIOPACKET_BUTTONPRESS && last_button_state != pBeacon_Data_pkt->BK_Preamble.button_pr) 
    //((config.radioPacketFlags & RADIOPACKET_BUTTONPRESS) && (last_button_state != pBeacon_Data_pkt->BK_Preamble.button_pr)
    if (button_state) {
		  static uint8_t tick;
		  struct ButtonClick btnClick;
		  btnClick.button_events = button_state;
		  btnClick.tick = tick++;
		  btnClick.type = 0xBC;
		  btnClick.version = PACKET_VERSION;
		  btnClick.crc8 = crc8(&btnClick.type, sizeof(btnClick) - 1);
	  	if (config.flags & FLAG_TRACE_ASYNC) {
	  		TRACE("BtnClick 0x%02X\r\n", btnClick.button_events);
	  	}
	  	RadioTxPktQueue(config.routerDstAddr, sizeof(btnClick), (uint8_t *) &btnClick);
    }
    //Trace to debug
    if ((button_state & BUTTON_A)) {
      if (config.flags & FLAG_TRACE_ASYNC) {
        switch (button_state & BUTTON_A) {
          case BUTTON_PRESS:
            TRACE("Btn A PRESS\r\n");
            break;
          case BUTTON_CLICK:
            TRACE("Btn A CLICK\r\n");
					  break;
          case BUTTON_DBLCLICK:
            TRACE("Btn A dblClick\r\n");
            break;
          case BUTTON_RELEASE:
            TRACE("Btn A Release\r\n");
            break;
    		}
		  }
      button_state &= BUTTON_B;
    }
    if ((button_state & BUTTON_B)) {
      if (config.flags & FLAG_TRACE_ASYNC) {
        switch ((button_state & BUTTON_B) >>4) {
          case BUTTON_PRESS:
            TRACE("Btn B PRESS\r\n");
            break;
          case BUTTON_CLICK:
            TRACE("Btn B CLICK\r\n");
            break;
          case BUTTON_DBLCLICK:
            TRACE("Btn B dblClick\r\n");
            break;
          case BUTTON_RELEASE:
            TRACE("Btn B Release\r\n");
            break;
        }
      }
      button_state &= BUTTON_A;
    } 
#endif   
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
extern uint16_t tim3_at_radio;
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
                SaveConfig(&config);
                break;
            case 'h': // Halt IMU Tx
                //if (i != 1) continue;
                //EXTI->IMR &= ~GPI_IMU_DIO1_EXTI_LINE;
				EXTI->IMR &= ~GPI_IMU_INT_EXTI_LINE;
                halted = 1;
                break;
            case 'c': // Continue IMU Tx
                //if (i != 1) continue;
                //EXTI->IMR |= GPI_IMU_DIO1_EXTI_LINE;
                //EXTI->IMR |= GPI_IMU_DIO2_EXTI_LINE;
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
                case 17: {
                    config.time_adjust = (int16_t) value;
                    //I2C_EE_BufferWrite((uint8_t*) &time_adjust, 126, 2);
                    SaveConfig(&config);
                    whole_time_adjust = config.time_adjust/10;
                    part_time_adjust = config.time_adjust%10;
                  }
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
                    config.frameCountNoSync = value;
                    SaveConfig(&config);
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
                    TRACE("oldFrameTime @%d.%d frame=%u, tim3_at_radio=%u\n\r", oldFrameTime.sec, oldFrameTime.uSec, oldFrameIdAtSync, tim3_at_radio);
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
                    frameAdjust = (int8_t) value;
                  }
                  break;
                case 48:
                  {
                    config.flags ^= FLAG_FRAMEID_24BITS;
                    SaveConfig(&config);
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
                    TRACE(": successBeacons=%d frameOffset=%d frameAdjust=%d\n\r", successBeacons, frameOffset, frameAdjust);
                    TRACE(": remainOutOfSyncTime=%d frameIdCorrection=%d\n\r", remainOutOfSyncTime, lastFrameIdCorrection);
                    CoTickDelay(10);
                    TRACE(": realFrameId=%d lastFrIdAtSync=%d asserted=%d\r\n", realFrameId, lastFrameIdAtSync, asserted);
                    TRACE("errorFrameId=%u valids=%d notValids=%d lostSync=%d\r\n", errorFrameId, Valids, notValids, lostSync);
                    TRACE("changeClocks=%d newbcn=%d corCount=%d drifts=%u last_drift=%d\r\n", changeClocks, newbcn, frameIdCorrectionCount, drift, last_drift);
                   extern uint8_t led_blinking;
                    TRACE("led blinking=%d\r\n", led_blinking);
                  }
                  break;
                case 50:
                  {
                    switch (value) {
                    case 2:
                      //HwLEDToggle(LED2);
                      break;
                    case 3:
                      //HwLEDToggle(LED3);
                      break;
                    case 4:
                      //HwLEDToggle(LED4);
                      break;
                    default:
                      //HwLEDToggle(LED1);
                    }
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
                    I2C_EE_BufferRead((uint8_t*) &saved_lr, EEPROM_DEBUG_STACKED_LR, EEPROM_DEBUG_STACKED_LR_SIZE);
                    I2C_EE_BufferRead((uint8_t*) &saved_pc, EEPROM_DEBUG_STACKED_PC, EEPROM_DEBUG_STACKED_PC_SIZE);
                    I2C_EE_BufferRead((uint8_t*) &saved_psr, EEPROM_DEBUG_STACKED_PSR, EEPROM_DEBUG_STACKED_PSR_SIZE);
                    I2C_EE_BufferRead((uint8_t*) &fault_counter, EEPROM_DEBUG_COUNTER, EEPROM_DEBUG_COUNTER_SIZE);

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
                    SaveConfig(&config);
                    if (update_flags & UPDATE_FLAG_PANID) {
                      RadioSetPanIdShortAddr(config.panId, config.mySrcAddr);
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
						uint8_t *data = (uint8_t *) decoded + sizeof(struct PacketHeader);
						I2C_EE_BufferWrite(data, 128, 32);
						up.errorCode = 0;
					}
					uint8_t type = DEV_RESP_SET_EEPROM_DATA;
					__writeCmdLineRespPacket((unsigned char*) &up, 4, type);
                } else if (packetHeader->type == DEV_CMD_GET_EEPROM_DATA) {
					struct RespEepromData resp;
					I2C_EE_BufferRead(&resp.data[0], 128, 32);
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
            I2C_EE_BufferRead((uint8_t*) &jumpToMain, EEPROM_BEACON_FLAG_ADDRESS, 1);
            if (jumpToMain != 0xFF) {
              jumpToMain = 0xFF;
              I2C_EE_BufferWrite((uint8_t*) &jumpToMain, EEPROM_BEACON_FLAG_ADDRESS, 1);
            }
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

float fLast[10];

void Task8(void* pdata){
  static uint8_t curr_chg;
  static uint8_t chargeflag=0;
  static uint8_t prev_capacity;
  static uint8_t val;
  config.
  while (1) {
    SAVE_POINT
    RELOAD_WATCHDOG
    
    CoTickDelay(60000);
    SAVE_POINT
    WaitGrabI2C();
    SAVE_POINT
    curr_chg = i2cBattRoutine(); //grab value from battery monitor IC.
    SAVE_POINT
    ReleaseI2C();
    SAVE_POINT

    lastBatStatus.flags = HwGPIState(GPI_CHG_STAT)?0:1;
    if (HwGPIState(GPI_USB_VBUS)) {
      lastBatStatus.flags |= 0x02;
      rt_flags |= RT_FLAG_USB_CONNECTED;
    } 
    else {
      rt_flags &= ~RT_FLAG_USB_CONNECTED;
    }

    switch(batterystatus){
      case 0x03:    //usb is connected and charging
        if(!chargeflag){
          chargeflag=1;
          prev_capacity = curr_chg;
        }
      break;
      case 0x02:    //usb is connected and either in power down mode OR completely charged
        if(curr_chg == 100){    //confirm that it is completely charged

        }
      break;
      default:
        if(chargeflag){ //the device was charging
          chargeflag=0;
          val = curr_chg-prev_capacity;
          
          if(val != 0){
            config.charge_capacity = curr_chg - prev_capacity;
            if(config.charge_capacity >= 100){
              config.charge_cycle++;
              config.charge_capacity-=100;
            }
            config_update = 1;
          }
        }
      break;
    }
 	  setIMUReady = CoSetFlag(flagBATDataReady);
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
  //static count;
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
    CoWaitForSingleFlag(flagIMU_G_DRDY, 0);
    counterIMU++;
    SAVE_POINT
    CoClearFlag(flagIMUNewData);        // DO NOT USE auto reset Flag as the DMA interrupt may take place before CoWaitForSingleFlag(flagIMUNewData, 0);  
  	//REV J 
  	RadioIMU_WaitGrabSPI();
  	IMUProcess();
  	RadioIMU_ReleaseSPI();
  	SAVE_POINT
  	InputDataIntoBuffer(&IMU_RawData[0]);

  	setIMUReady = CoSetFlag(flagIMUDataReady);
  	SAVE_POINT;
  }
}
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
