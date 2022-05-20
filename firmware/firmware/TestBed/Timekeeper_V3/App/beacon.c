/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
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

uint32_t IMU_Dbg_Prt =0;

// Beacon com init
// inits all structs etc for beacon incomming data handlers...

void BeaconComInit(void){
  
        memset(&CurrBeacons[0],0x00,sizeof CurrBeacons);  // array or 32 bit pointers
        memset(&beacon_found[0],0x00,sizeof beacon_found);//Rf ID of found beacons
        NumBeaconsActive=0;
        SendBeaconIndex=0;
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

            for(count =0; count < MAXBEACONS; count++){
              if ((beacon_found[count] )&& (beacon_found[count] == pRxPktData->srcAddr)){
// copy to the buffer ...past the local TK variable..starting at the beginning of the payload data                
                memcpy((&(CurrBeacons[count]->seqNum)),&((pRxPktData->payload)),(sizeof( struct BCPacketIMU)));//
//               printf("Sizof struct BCPacketIMU:%d",sizeof(struct BCPacketIMU));       
     //  memcpy(&Test_Payoad[0],&(pRxPktData->payload),(sizeof(struct BCPacketIMU)));
                if(IMU_Dbg_Prt){
                pIMUdataPkt = (struct BCPacketIMU*) &(pRxPktData->payload);
        for(x=0;x < 8; x++){
           printf(" %x, %2x, %2x, %2x, %2x, %2x, %2x \n\r",pIMUdataPkt->imuData[x].Timestamp,pIMUdataPkt->imuData[x].gyroscopeX, pIMUdataPkt->imuData[x].gyroscopeY,
                  pIMUdataPkt->imuData[x].gyroscopeZ,pIMUdataPkt->imuData[x].accelerationX,pIMUdataPkt->imuData[x].accelerationY,pIMUdataPkt->imuData[x].accelerationZ);
                }
            }
               CurrBeacons[count]->arrivalTime = rfBeaconFrameId; // get the current value of the TK sync counter
               CurrBeacons[count]->NewBKData= 0xff;  // set new data flag
               CurrBeacons[count]->TK_RxRSSI=pRxPktData->payload[pRxPktData->payloadSize] ;
                
 
/*
               printf("IMU DATA:%x%x",pRxPktData->payload[7],pRxPktData->payload[6]);
               
               for(x=80; x<102;x++){
                 printf("%x ",pRxPktData->payload[x]);
               }
                          printf("\n\r");    
*/               
               
              break;
              }else if(!(beacon_found[count]) ){
               beacon_found[count]= pRxPktData->srcAddr;
               printf("Beacon Found slot# %d Id#:%x \n\r",count, beacon_found[count]);

               CurrBeacons[count]=(struct BeaconData*) pNextRam;
   
             pNextRam = pNextRam + sizeof( struct BeaconData);
             memset(pNextRam,0,sizeof( struct BeaconData));
             
//              printf("Location of CurrBeacons[0]:%x\n\r", &CurrBeacons[0]);
        //      printf("Memory assignement:%x sizof Mem:%x \n\r",CurrBeacons[count],sizeof( struct BeaconData));
              
              memcpy((&(CurrBeacons[count]->seqNum)),&(pRxPktData->payload),(sizeof(struct BCPacketIMU)));
   
          memcpy(&Test_Payoad[0],&(pRxPktData->payload),(sizeof(struct BCPacketIMU)));
          
          
          
              CurrBeacons[count]->arrivalTime= rfBeaconFrameId; 
              CurrBeacons[count]->NewBKData= 0xff;
               CurrBeacons[count]->TK_RxRSSI=pRxPktData->payload[pRxPktData->payloadSize] ;

               
              NumBeaconsActive++;
               count=MAXBEACONS;
              }
            }//for end
            
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


void EtherSendIMU(void){
uint16_t EthSendIndex =0;  
void   *pIMU_Data,*pIMU_Data2,*pIMUData3 ;


  //    printf("-");
  
        if( NumBeaconsActive){      // check for active beacons....
          if(CurrBeacons[SendBeaconIndex]->NewBKData ){ //check new data flag
            
//            printf("+");
            
            CurrBeacons[SendBeaconIndex]->NewBKData = 0; // reset newdata flag
             EthSendIndex = sizeof(struct TKFrame); // start data insertion beyond header info
            if((CurrBeacons[SendBeaconIndex]->button_state & 0x03) != CurrBeacons[SendBeaconIndex]->LastButtonState){
             CurrBeacons[SendBeaconIndex]->LastButtonState = (CurrBeacons[SendBeaconIndex]->button_state & 0x03);  // set them to the same for next time
   //           printf(" Beacon:%d Button:%x\n\r",SendBeaconIndex, CurrBeacons[SendBeaconIndex]->LastButtonState);
 
             
// button status packet

             struct TKBCPacketButton* pIMU_Data = (struct TKBCPacketButton*) &IMU_Data[EthSendIndex]  ;
             pIMU_Data->header.packetID = IncrementPacketId();
              pIMU_Data->header.contentType= TK_PT_TKBCPacketButton;	
                pIMU_Data->header.contentLength= sizeof(struct TKBCPacketButton);
              pIMU_Data->beaconID=SendBeaconIndex;
                pIMU_Data->buttonState=CurrBeacons[SendBeaconIndex]->LastButtonState;
              EthSendIndex += sizeof(struct TKBCPacketButton);
            } // end of button if
// IMU Data packet      
// 1st see if IMU ethersend enabled        
//           CurrBeacons[SendBeaconIndex]-> BC_PowerMode |= TK_BC_PM_IMU_ON;
            
            if((CurrBeacons[SendBeaconIndex]-> BC_PowerMode & TK_BC_PM_IMU_ON)){ 
            
           struct TKBCPacketIMU* pIMU_Data2 = ( struct TKBCPacketIMU*)&IMU_Data[EthSendIndex]; // load pointer
// fill IMU headder info
           pIMU_Data2->header.packetID = IncrementPacketId(); // inc packet ID #
              pIMU_Data2->header.contentType=TK_PT_TKBCPacketIMU; // tell it's IMU data
               pIMU_Data2->header.contentLength= sizeof(struct TKBCIMUEthPacket); // tell length
               pIMU_Data2->beaconID= SendBeaconIndex; // tell who this is from (beacon #)
               pIMU_Data2->sequenceNumber=CurrBeacons[SendBeaconIndex]->seqNum; // tell the beacon sequence #
               
       pIMU_Data2->ArrivalTime=CurrBeacons[SendBeaconIndex]->arrivalTime;  // tell 32 bit arrival time
               
               
               EthSendIndex +=  sizeof(struct TKBCPacketIMU); // inc index by size of above header info
 //              EthSendIndex +=  sizeof(struct GenericPHeader);
// copy all IMU data to packet buffer
              memcpy(&IMU_Data[EthSendIndex],&(CurrBeacons[SendBeaconIndex]->SyncFrameIMU),  sizeof(struct TKEtherIMUSend));
              
 //          memcpy(&IMU_Test[0],&(CurrBeacons[SendBeaconIndex]->SyncFrameIMU),  sizeof(struct TKEtherIMUSend));
//          printf("Sizeof BCIMUDSTRCT:%d",sizeof(struct BCIMUDSTRCT));
              
              
              EthSendIndex +=  sizeof(struct TKEtherIMUSend); // increment index by size of TKEtherIMUSend struct for header size
          } // end of IMU send enabled IF
// set up TK send header with overall length and ID of this TK             
             struct TKFrame* pIMUData3 = (struct TKFrame*) &IMU_Data[0];  // set pointer to the end of the data in send buffer for data copy
             pIMUData3->timekeeperID = GetTKId();

 //            EthSendIndex = 110;
             
           pIMUData3->contentLength=EthSendIndex;

//            printf("TKID: %x Len:%x ",pIMUData3->timekeeperID,pIMUData3->contentLength);
             
           if(EthSendIndex > sizeof(struct TKFrame)){ // don't send unless length more than just header
             
             Send_Brdcst_packet(&IMU_Data[0],EthSendIndex);
              }
            }
          if((++SendBeaconIndex) >= NumBeaconsActive){
            SendBeaconIndex = 0;
          }
        }
         
}

  