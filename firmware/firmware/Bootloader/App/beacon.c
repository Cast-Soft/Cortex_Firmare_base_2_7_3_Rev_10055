/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
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

#define MAXBEACONS 10
#define RAM32Boundary  0x20007D00

uint16_t beacon_found[MAXBEACONS]={0};
BOOL detailChanged = 0;   // if beacon list detail changed, need to notify Bridge. reset after notification 

struct BeaconData* CurrBeacons[MAXBEACONS]={0};
struct BeaconData **ppCurrBeacons;
extern uint32_t NumBeaconsActive;
struct BeaconData *pBeaconData;

uint8_t IMU_Data[1500] = {0};

uint8_t IMU_Test[256] ={0};
uint8_t Test_Payoad[256] ={0};

uint32_t TempCnt = 0;

uint8_t *pNextRam;
uint8_t *UpperMem;

uint32_t IMU_Dbg_Prt =0, SeqNum_Dbg_Prt=0;

void RefreshDetails(void);

// Beacon com init
// inits all structs etc for beacon incomming data handlers...

void BeaconComInit(void){
    memset(&CurrBeacons[0],0x00,sizeof CurrBeacons);  // array or 32 bit pointers
    memset(&beacon_found[0],0x00,sizeof beacon_found);//Rf ID of found beacons
    NumBeaconsActive=0;
    pNextRam = (void*)RAM32Boundary;// load up the base ram
}


// This is where we capture Bcon data
// we do a quick look to se if we have this beacon already...if so copy data for later
// if not add the ID to the active table
// beaconfound[] is an array of uint16 beacon addresses
//CurrBeacons[] is an array of pointers to structs of type BeaconData
// pRxPktData is a pointer to the recieved datapacket from the beacon

void  ParseBeaconData( rxPkt_t *pRxPktData){
    uint32_t count, x;
    struct BCPacketIMU *pIMUdataPkt;

    for(count = 0; count < MAXBEACONS; count++){
        if ((beacon_found[count] ) && (beacon_found[count] == pRxPktData->srcAddr)) {
            // copy to the buffer ...past the local TK variable..starting at the beginning of the payload data
            memcpy((&(CurrBeacons[count]->seqNum)),&((pRxPktData->payload)),(sizeof( struct BCPacketIMU)));//

            if(IMU_Dbg_Prt) {
                pIMUdataPkt = (struct BCPacketIMU*) &(pRxPktData->payload);
                for(x=0;x < NUM_OF_IMU_PKTS_IN_RF_PKT; x++) {
                    printf(" %x, %2x, %2x, %2x, %2x, %2x, %2x \n\r",pIMUdataPkt->imuData[x].Timestamp,pIMUdataPkt->imuData[x].gyroscopeX, pIMUdataPkt->imuData[x].gyroscopeY,
                    pIMUdataPkt->imuData[x].gyroscopeZ,pIMUdataPkt->imuData[x].accelerationX,pIMUdataPkt->imuData[x].accelerationY,pIMUdataPkt->imuData[x].accelerationZ);
                }
            }
            else if(SeqNum_Dbg_Prt) {
                printf("%d, ", CurrBeacons[count]->seqNum);
                printf("-%d-, ", pRxPktData->seqNumber);
            }
            CurrBeacons[count]->arrivalTime = 0; // TODO!!! other way to synchronize
            CurrBeacons[count]->NewBKData= 0xff;  // set new data flag
            CurrBeacons[count]->TK_RxRSSI=pRxPktData->payload[pRxPktData->payloadSize] ;

            EtherSendIMU(count);   // if packet above requires no answer ie" discarded..check for outgoing data
            
            break;
        } 
        else if(!(beacon_found[count]) ){
            beacon_found[count]= pRxPktData->srcAddr;
            printf("Beacon Found slot# %d Id#:%x \n\r",count, beacon_found[count]);
            detailChanged = 1;    // to notify Bridge
               
            CurrBeacons[count]=(struct BeaconData*) pNextRam;

            pNextRam = pNextRam + sizeof( struct BeaconData);
            memset(pNextRam,0,sizeof( struct BeaconData));

            //printf("Location of CurrBeacons[0]:%x\n\r", &CurrBeacons[0]);
            //printf("Memory assignement:%x sizof Mem:%x \n\r",CurrBeacons[count],sizeof( struct BeaconData));

            memcpy((&(CurrBeacons[count]->seqNum)),&(pRxPktData->payload),(sizeof(struct BCPacketIMU)));

            memcpy(&Test_Payoad[0],&(pRxPktData->payload),(sizeof(struct BCPacketIMU)));
            CurrBeacons[count]->arrivalTime= 0;     // TODO!!! other way to indicate
            CurrBeacons[count]->NewBKData= 0xff;
            CurrBeacons[count]->TK_RxRSSI=pRxPktData->payload[pRxPktData->payloadSize];

            NumBeaconsActive++;
            
            EtherSendIMU(count);   // if packet above requires no answer ie" discarded..check for outgoing data
            break;      // added and exits loop
        }
    }   //for end
}

uint32_t GetPktSyncTime(uint8_t Index){
    return CurrBeacons[Index]->arrivalTime;
}

uint8_t GetButtonState(uint8_t Index){
    return CurrBeacons[Index]->button_state;
}


//sizof 101 (inside payload)
//struct BCPacketIMU
//	uint8_t buttonState; //
//	uint8_t signalStrength; //
//	uint8_t led1State; //
//	uint8_t led2State; //
//	uint8_t led3State; //
//      uint8_t   Battery_lev;
//      uint8_t   SyncFrameIMU;
//      uint8_t   MsTimerIMU;
//	struct BCIMUData imuData[8]; //


void EtherSendIMU(int beaconIndex) {
    uint32_t SendBeaconIndex = beaconIndex;
    uint16_t EthSendIndex =0;
    void   *pIMU_Data,*pIMU_Data2,*pIMUData3 ;

    extern uint32_t magicNumber;         // session
    extern uint32_t lostRequests ;    // accumulates lost requests from Bridge  

    // printf("-");
    assert(NumBeaconsActive > 0);
    assert(SendBeaconIndex >= 0 && SendBeaconIndex < NumBeaconsActive);

    // TODO!!! post to Ethertask
    if(CurrBeacons[SendBeaconIndex]->NewBKData) { //check new data flag
        // printf("+");

        CurrBeacons[SendBeaconIndex]->NewBKData = 0; // reset newdata flag
        EthSendIndex = sizeof(struct TKFrame); // start data insertion beyond header info
        if((CurrBeacons[SendBeaconIndex]->button_state & 0x03) != CurrBeacons[SendBeaconIndex]->LastButtonState) {
            CurrBeacons[SendBeaconIndex]->LastButtonState = (CurrBeacons[SendBeaconIndex]->button_state & 0x03);  // set them to the same for next time
            // printf(" Beacon:%d Button:%x\n\r",SendBeaconIndex, CurrBeacons[SendBeaconIndex]->LastButtonState);

            // button status packet
            struct TKBCPacketButton* pIMU_Data = (struct TKBCPacketButton*) &IMU_Data[EthSendIndex]  ;
            pIMU_Data->header.packetID = IncrementPacketId();
            pIMU_Data->header.contentType= TK_PT_TKBCPacketButton;
            pIMU_Data->header.contentLength= sizeof(struct TKBCPacketButton);
            pIMU_Data->beaconID=beacon_found[SendBeaconIndex];    // TODO!!! add an assertion to test the range of vector
            pIMU_Data->buttonState=CurrBeacons[SendBeaconIndex]->LastButtonState;
            EthSendIndex += sizeof(struct TKBCPacketButton);
        } // end of button if
        // IMU Data packet
        // 1st see if IMU ethersend enabled
        // CurrBeacons[SendBeaconIndex]-> BC_PowerMode |= TK_BC_PM_IMU_ON;
        
        // if((CurrBeacons[SendBeaconIndex]-> BC_PowerMode & TK_BC_PM_IMU_ON))      // TODO !!! remove this control word. always allow
        {
            struct TKBCPacketIMU* pIMU_Data2 = ( struct TKBCPacketIMU*)&IMU_Data[EthSendIndex]; // load pointer              
            // fill IMU header info                     
            pIMU_Data2->header.packetID = IncrementPacketId(); // inc packet ID #
            pIMU_Data2->header.contentType=TK_PT_TKBCPacketIMU; // tell it's IMU data
            pIMU_Data2->header.contentLength= sizeof(struct TKBCIMUEthPacket); // tell length
            pIMU_Data2->beaconID= beacon_found[SendBeaconIndex]; // tell who this is from (beacon #)
            pIMU_Data2->sequenceNumber=CurrBeacons[SendBeaconIndex]->seqNum; // tell the beacon sequence #

            pIMU_Data2->ArrivalTime=CurrBeacons[SendBeaconIndex]->arrivalTime;  // tell 32 bit arrival time

            EthSendIndex += sizeof(struct TKBCPacketIMU); // inc index by size of above header info
            // EthSendIndex +=  sizeof(struct GenericPHeader);
            // copy all IMU data to packet buffer
            memcpy(&IMU_Data[EthSendIndex],&(CurrBeacons[SendBeaconIndex]->SyncFrameIMU),  sizeof(struct TKEtherIMUSend));

            // memcpy(&IMU_Test[0],&(CurrBeacons[SendBeaconIndex]->SyncFrameIMU),  sizeof(struct TKEtherIMUSend));
            // printf("Sizeof BCIMUDSTRCT:%d",sizeof(struct BCIMUDSTRCT));


            EthSendIndex +=  sizeof(struct TKEtherIMUSend); // increment index by size of TKEtherIMUSend struct for header size
        } // end of IMU send enabled IF
        
        {
            // set up TK send header with overall length and ID of this TK
            struct TKFrame* pIMUData3 = (struct TKFrame*) &IMU_Data[0];  // set pointer to the end of the data in send buffer for data copy
        
            pIMUData3->signature = TK_FRAME_SIGNATURE;
            pIMUData3->version_major = TK_VERSION_MAJOR;
            pIMUData3->version_minor = TK_VERSION_MINOR;
            pIMUData3->version_revision = TK_VERSION_REVISION;
            pIMUData3->version_build = TK_VERSION_BUILD;              
            pIMUData3->magicNumber = magicNumber;    // 0 means not initialized yet 
            pIMUData3->lostRequests = lostRequests;                          
            pIMUData3->timekeeperID = GetTKId();

            // EthSendIndex = 110;
            pIMUData3->contentLength=EthSendIndex;
            // printf("TKID: %x Len:%x ",pIMUData3->timekeeperID,pIMUData3->contentLength);
        }

        if(EthSendIndex > sizeof(struct TKFrame)){ // don't send unless length more than just header
            EthAsyncSendPacket(&IMU_Data[0],EthSendIndex);
        }
    } //end of check new data flag
        
    // refresh detail in need 
    RefreshDetails();        
}


void RefreshDetails(void){
    if (detailChanged == 0) return;   
    // if changed, need to notify Bridge 
    detailChanged = 0;      // may be overridden by anoither task's setting          // TODO!!! find out a swap oprand
 
    extern uint8_t TxDataBuf[];
    extern struct GenericSendHddr *pGenericSendHddr;

    void *pTxBuffHead;
    uint32_t TxTotLength; 
    struct TKFrame *pTKFrame,*pTKFrameSend;    
    
    pTxBuffHead=(&TxDataBuf);
    pTKFrameSend=(struct TKFrame*)pTxBuffHead;
    pTKFrameSend->signature = TK_FRAME_SIGNATURE;
    pTKFrameSend->version_major = TK_VERSION_MAJOR;
    pTKFrameSend->version_minor = TK_VERSION_MINOR;
    pTKFrameSend->version_revision = TK_VERSION_REVISION;
    pTKFrameSend->version_build = TK_VERSION_BUILD;
    pTKFrameSend->timekeeperID= pGenericSendHddr->timekeeperID;
    pTKFrameSend->contentLength = (sizeof(struct TKFrame));
    
    // lazy initialization: T.K. does not have a system timer to create a random number seed 
    // magicNumber / session. If Bridge detects this number changed, it means a new session started
    if (magicNumber == 0) {
        clock_time_t ct = clock_time();
        srand(ct);
        magicNumber = rand();
        if (magicNumber == 0) magicNumber = 1;    
    }
    
    pTKFrameSend->magicNumber = magicNumber;
    pTKFrameSend->lostRequests = lostRequests;
    TxTotLength=sizeof(struct TKFrame); 
    
    struct TKPacketStatusDetail *pTKPacketStatusDetail;    
    pTKPacketStatusDetail = (struct TKPacketStatusDetail*) &TxDataBuf[TxTotLength];
    pTKPacketStatusDetail->header.packetID = pGenericSendHddr->packetID++; // get the packet ID Incrementing number
    pTKPacketStatusDetail->header.contentType = TK_PT_TKPacketStatusDetail;
    pTKPacketStatusDetail->numberBeacons = NumBeaconsActive;
    pTKPacketStatusDetail->beacons = NULL;
    uip_gethostaddr(pTKPacketStatusDetail->ipaddress);   // get the hosts IP adress
    
    // Status data from Beacons
    // setup reply buffer pointers
    pTKPacketStatusDetail->header.contentLength=sizeof(struct TKPacketStatusDetail) + (NumBeaconsActive *( sizeof(struct TKBeaconStatusDetail)));
    TxTotLength= TxTotLength + sizeof(struct TKPacketStatusDetail);

    if(NumBeaconsActive) {
        for(int i32 =0;i32 < NumBeaconsActive;i32++) {
            struct TKBeaconStatusDetail* pTKBeaconStatusDetail = (struct TKBeaconStatusDetail*) &TxDataBuf[TxTotLength];
            pTKBeaconStatusDetail->beaconID = beacon_found[i32];
            pTKBeaconStatusDetail->mySrcAddr = beacon_found[i32]; // get the beacon ID for this entry from beacon_found[] aray
            struct BeaconData* pBeacon_Data =(struct BeaconData*)  CurrBeacons[i32]; // get address of this beacon struct from CurrBeacons[] array
            pTKBeaconStatusDetail->beaconTimeLastActivity = pBeacon_Data->arrivalTime;// GetPktSyncTime(i32);
            pTKBeaconStatusDetail->beaconStatus=0;
            pTKBeaconStatusDetail->beaconBattery = pBeacon_Data->beaconBattery;
            pTKBeaconStatusDetail->beaconSignalTK=pBeacon_Data->TK_RxRSSI; // rssi of signal from BK at TK
            pTKBeaconStatusDetail->beaconSignalBC=pBeacon_Data->beaconRSSI; // rssi at the beacon from the TK
            pTKBeaconStatusDetail->radioStatus=0;
            pTKBeaconStatusDetail->syncStatus=0;
            pTKBeaconStatusDetail->RxErrors=0;
            pTKBeaconStatusDetail->irLed0= pBeacon_Data->led0Id;
            pTKBeaconStatusDetail->irLed1= pBeacon_Data->led1Id;
            pTKBeaconStatusDetail->irLed2= pBeacon_Data->led2Id;

            // memset(&(pTKBeaconStatusDetail->beaconStatus),0,8);
            TxTotLength= TxTotLength + sizeof(struct TKBeaconStatusDetail);
        }
    }

    pTKFrameSend->contentLength = TxTotLength;
    EthAsyncSendPacket(pTxBuffHead,TxTotLength);
    TxTotLength=0;   
}

