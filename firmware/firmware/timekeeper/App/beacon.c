/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
******************************************************************************/
//Beacon.c
//  Beacon managment functions abstracted from Hardware
//
//
//
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/


#include "includes.h"
#include "stm32f10x_it.h"
#include "packets.h"
#include "basic_rf.h"
#include "beacon.h"
#include "Tst_Dbg.h"
#include "main.h"

#define MAXBEACONS 10
#define RAM32Boundary  0x20007D00

uint16_t beacon_found[MAXBEACONS]={0};
BOOL detailChanged = 0;   // if beacon list detail changed, need to notify Bridge. reset after notification

struct BeaconData* CurrBeacons[MAXBEACONS]={0};
struct BeaconData **ppCurrBeacons;
extern uint32_t NumBeaconsActive;
uint32_t SendBeaconIndex;   // current beacon index for ether send setup
struct BeaconData *pBeaconData;

uint8_t IMU_Data[1500] = {0};

uint8_t IMU_Test[256] ={0};
uint8_t Test_Payoad[256] ={0};

uint32_t TempCnt = 0;

uint8_t *pNextRam;
uint8_t *UpperMem;

uint32_t IMU_Dbg_Prt =0, SeqNum_Dbg_Prt=0;

// Beacon com init
// inits all structs etc for beacon incomming data handlers...

void BeaconComInit(void){
    memset(&CurrBeacons[0],0x00,sizeof CurrBeacons);  // array or 32 bit pointers
    memset(&beacon_found[0],0x00,sizeof beacon_found);//Rf ID of found beacons
    NumBeaconsActive=0;
    SendBeaconIndex=0;
    pNextRam = (void*)RAM32Boundary;// load up the base ram
}

uint32_t GetPktSyncTime(uint8_t Index){
    return CurrBeacons[Index]->arrivalTime;
}

uint8_t GetButtonState(uint8_t Index){
    return CurrBeacons[Index]->button_state;
}


