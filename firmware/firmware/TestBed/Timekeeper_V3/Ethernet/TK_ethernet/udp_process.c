/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
// UDP_Process.C
// Processes UDP connection data as supplied from uIP stack in uip.c
// uip.c  calls UIP_UDP_APPCALL when UDP packets are received and ready
// UDP_Process takes the data in the rx buffer and dipatches it to the correct
// application...uses UDP port #s in header 
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
#include "tk_version.h"
#include "includes.h"
#include "udp_process.h"
#include "packets.h"
#include "dhcpc.h"
#include "tst_dbg.h"
#include "uip.h"
#include "ethernet.h"
#include "basic_rf.h"
#include "beacon.h"

extern struct uip_udp_conn *uip_udp_conn;

#define DHCPC_SERVER_PORT  67
#define DHCPC_CLIENT_PORT  68

#define TK_Listen_Port  23001
#define TK_Send_Port  23000
#define TK_Rand_Port 20000

udp_hddr_t  PayLoad;
extern void *uip_sappdata;
 
struct  GenericSendHddr genericsendhddr,*pGenericSendHddr;
extern uint32_t *pHeader_union32 ;
extern uint8_t  *pHeader_union8 ;
//extern hddr_union_t header_union ,*pHeader_union;
//hddr_union_t rx_union_bffr;  // rx data parsing buffer
struct Ether_status ether_status;
struct Ether_status  *pEther_status = &ether_status;

extern uint16_t beacon_found[];

struct BeaconData *pBeacon_Data; 
extern struct BeaconData* CurrBeacons[];
/*----------------------------------------------------------*/
struct TKPacketStatusDetail *pTKPacketStatusDetail;
struct TKPacketRadioControlACK *pTKPacketRadioControlACK;
struct TKPacketSyncControlACK *pTKPacketSyncControlACK;
struct TKPacketReadEEPROMResponse *pTKPacketReadEEPROMResponse;
struct TKBCPacketButton *pTKBCPacketButton;
struct TKBCPacketFlashLEDACK *pTKBCPacketFlashLEDACK;
struct TKBeaconStatusDetail   *pTKBeaconStatusDetail;
struct TKBCPacketSetBeaconNumberACK *pTKBCPacketSetBeaconNumberACK;
struct TKBCPacketSetBeaconNumber *pTKBCPacketSetBeaconNumber;
struct TKBCPacketPowerMode *pTKBCPacketPowerMode;
struct TKBCPacketPowerModeACK *pTKBCPacketPowerModeACK;
/*------------------------------------------------------------*/

struct TKEEPROMData EEPromData;
struct TKEEPROMData *pEEPromData = &EEPromData;
extern struct uip_eth_addr mac_addr;

#define TimeoutVal  1000;
uint32_t BTHostTimeOut = TimeoutVal;

uint8_t RxDataBuff[1500];
uint8_t TxDataBuf[1500];  
  
uint32_t NumBeaconsActive; 
/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
// This initts all of the ether comm status stuff
//
//
//

void init_ether_comm(void){
    pEther_status= &ether_status;       // init the pointer
    pGenericSendHddr= &genericsendhddr; // init the pointer
      
    pEther_status->Probe_cnt =5;      // send probe count init
    pEther_status->BT_Link=0;       // set up as not linked to BT server
     pGenericSendHddr->packetID=0;  // init the packet counter
     pGenericSendHddr->timekeeperID=(( mac_addr.addr[4] <<8) | (mac_addr.addr[5]));
     pEEPromData = &EEPromData;
     memset(pEEPromData,0,sizeof(struct TKEEPROMData));
      NumBeaconsActive=0;
              
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
//struct uip_udp_conn {
//  uip_ipaddr_t ripaddr;   /**< The IP address of the remote peer. */
//  u16_t lport;        /**< The local port number in network byte order. */
//  u16_t rport;        /**< The remote port number in network byte order. */
//  u8_t  ttl;          /**< Default time-to-live. */


void udp_process(void){
uint16_t local_port; 
uint16_t remote_port;

          local_port=(uip_udp_conn->lport);
          remote_port=(uip_udp_conn->rport);
          
          if(local_port == (HTONS(DHCPC_CLIENT_PORT))){
            dhcpc_appcall();
            
           }else if(local_port == (HTONS(TK_Listen_Port))){ 
 //           }else if(remote_port == (HTONS(TK_Listen_Port))){
             if(uip_len){         // see if we have data in the buffer
//               printf("Got a Data Pkt LocalP: %d Remot: %d \n\r",(HTONS(local_port)),(HTONS(remote_port)));
                    Packet_demux();
             }
                    
           
           }else{
//            printf("No Port match \n\r");
//            printf("local port:%d \n\r",(HTONS(uip_udp_conn->lport)));
//            printf("Remote port:%d \n\r",(HTONS(uip_udp_conn->rport)));
            
           }
  
  }     

/*-------------------------------------------------------------------*/
// Packet demux reads header at start of payload packet and takes appropriate action
//
//uint32_t packet_type
//uint32_t packet_len;
/*---------------------------------------------------------------------*/


void  Packet_demux(void){
//uint32_t Pckt_Id, Pckt_Len;  
struct TKFrame *pTKFrame,*pTKFrameSend;
struct GenericPHeader *pGHeader;
struct TKPacketStatusSummary *pTKPacketStatusSummary;
struct TKPacketStatusDetail *pTKPacketStatusDetail;
void  *pTxBuffHead, *pTxBuffnext,*pTKGenHeaderSend;

uint32_t RxTotLength,TxTotLength, i32; 
uint16_t i16;
uint8_t i8;
// set up to decode Rx packet
     
      pTKFrame = (struct TKFrame*) uip_sappdata; // uip_sappdata has incomming data
    //  if(pTKFrame->signature == TK_FRAME_SIGNATURE &&
         if(pTKFrame->timekeeperID ==pGenericSendHddr->timekeeperID){   // see if this is for us
        
        BTHostTimeOut =  TimeoutVal;  // heard from BT server ...reset timeout
// set up for return frame
       pTxBuffHead=(&TxDataBuf);  
        pTKFrameSend=(struct TKFrame*)pTxBuffHead;
        pTKFrameSend->signature = TK_FRAME_SIGNATURE;
        pTKFrameSend->version_major = TK_VERSION_MAJOR;
        pTKFrameSend->version_minor = TK_VERSION_MINOR;
        pTKFrameSend->version_revision = TK_VERSION_REVISION;
        pTKFrameSend->version_build = TK_VERSION_BUILD;
        pTKFrameSend->timekeeperID= pGenericSendHddr->timekeeperID;
        pTKFrameSend->contentLength = (sizeof(struct TKFrame));
        pTKFrameSend->data = NULL;
        TxTotLength=sizeof(struct TKFrame);
        
        memcpy(&RxDataBuff,uip_sappdata,pTKFrame->contentLength);
        RxTotLength=sizeof(struct TKFrame);
      

 //      printf("Got a demux Length:%d \n\r",pTKFrame->contentLength);  

        
        while (pTKFrame->contentLength > RxTotLength){
                  pGHeader = (struct GenericPHeader*)(&RxDataBuff[RxTotLength]);
 
//                  printf("Packet type %x Pckt Len %d \n\r",pGHeader->contentType,pGHeader->contentLength);
                  
                  
            switch(pGHeader->contentType){
   
                case TK_PT_TKPacketDiscoveryACK: // if we get this we have successful discovery
                printf("ACK from Btrx \n\r");
                 pEther_status->BT_Link=1;    // tell we have link up
                  RxTotLength=RxTotLength + sizeof(struct TKPacketDiscoveryACK);
                  break;
                 
          case TK_PT_TKPacketStatusSummaryRequest:
                   pTKPacketStatusSummary= (struct TKPacketStatusSummary*)&TxDataBuf[TxTotLength]; // set up for reply
                   pTKPacketStatusSummary->header.packetID=pGenericSendHddr->packetID++; // get the packet ID Incrementing number
                    pTKPacketStatusSummary->header.contentType = TK_PT_TKPacketStatusSummary; 
                    pTKPacketStatusSummary->header.contentLength = (sizeof (struct TKPacketStatusSummary)) ; 
                    TxTotLength= TxTotLength + sizeof(struct TKPacketStatusSummary);
                    RxTotLength=RxTotLength + sizeof(struct TKPacketStatusSummaryRequest);
                  break;

          case TK_PT_TKPacketStatusDetailRequest:
                   pTKPacketStatusDetail= (struct TKPacketStatusDetail*) &TxDataBuf[TxTotLength];
                    pTKPacketStatusDetail->header.packetID=pGenericSendHddr->packetID++; // get the packet ID Incrementing number
                    pTKPacketStatusDetail->header.contentType= TK_PT_TKPacketStatusDetail;
                    pTKPacketStatusDetail->numberBeacons = NumBeaconsActive;
                    pTKPacketStatusDetail->beacons = NULL;
                   uip_gethostaddr(pTKPacketStatusDetail->ipaddress);   // get the hosts IP adress
// Status data from Beacons
// setup reply buffer pointers                   
                    pTKPacketStatusDetail->header.contentLength=sizeof(struct TKPacketStatusDetail)+ (NumBeaconsActive *( sizeof(struct TKBeaconStatusDetail)));
                    TxTotLength= TxTotLength + sizeof(struct TKPacketStatusDetail);
// if Beacons active >0
// step through beacons       
/*struct TKBeaconStatusDetail
{
  	uint16 beaconIDs; //ID of beacon ...data follows
	uint16 beaconTimeLastActivity; // Last sync tick of last recieved packet
	uchar beaconStatus; // size of dynamic area is equal to numberConnectedBeacons
	uchar beaconBattery; //  empty 0, full at 255
	uchar beaconSignal; // RSSI of last packet Rx'd
	uchar radioStatus; // TK_RADIO_STATUS
	uchar syncStatus; // TK_SY N        
        uchar RxErrors;   // rx errors since last status poll....
        uint16 pad2;
};
*/                    
                    
                    if(NumBeaconsActive){
                      for(i32 =0;i32 < NumBeaconsActive;i32++){
                       pTKBeaconStatusDetail= (struct TKBeaconStatusDetail*) &TxDataBuf[TxTotLength]; 
                       pTKBeaconStatusDetail->beaconID = i32;
                       pTKBeaconStatusDetail-> mySrcAddr = beacon_found[i32]; // get the beacon ID for this entry from beacon_found[] aray
                        pBeacon_Data =(struct BeaconData*)  CurrBeacons[i32]; // get address of this beacon struct from CurrBeacons[] array
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
                   
//                       memset(&(pTKBeaconStatusDetail->beaconStatus),0,8);
                       TxTotLength= TxTotLength + sizeof(struct TKBeaconStatusDetail);
                      }
                      
                    }; // end if
                    
                    RxTotLength=RxTotLength + sizeof(struct TKPacketStatusDetailRequest);
                    
                  break;  
                  case TK_PT_TKPacketRadioControl:
                    pTKPacketRadioControlACK=(struct TKPacketRadioControlACK*)&TxDataBuf[TxTotLength];
//                TK_PT_TKPacketRadioControlACK
                    pTKPacketRadioControlACK->header.packetID=pGenericSendHddr->packetID++; // get the packet ID Incrementing number
                    pTKPacketRadioControlACK->header.contentType= TK_PT_TKPacketRadioControlACK;
                    pTKPacketRadioControlACK->header.contentLength=sizeof(struct TKPacketRadioControlACK);
                    RxTotLength=RxTotLength + sizeof(struct TKPacketRadioControl);
                    TxTotLength= TxTotLength + sizeof(struct TKPacketRadioControlACK);
                 break;
                 case TK_PT_TKPacketSyncControl:
                  pTKPacketSyncControlACK=(struct TKPacketSyncControlACK*)&TxDataBuf[TxTotLength];
//                TK_PT_TKPacketSyncControlACK  
                  pTKPacketSyncControlACK->header.packetID=pGenericSendHddr->packetID++; // get the packet ID Incrementing number
                    pTKPacketSyncControlACK->header.contentType=  TK_PT_TKPacketSyncControlACK ;
                    pTKPacketSyncControlACK->header.contentLength=sizeof(struct TKPacketSyncControlACK);
                  RxTotLength=RxTotLength + sizeof(struct TKPacketSyncControl); 
                   TxTotLength= TxTotLength + sizeof(struct TKPacketSyncControlACK);
                  break;
                  case TK_PT_TKPacketSetEEPROM:
                  pTKPacketReadEEPROMResponse = (struct TKPacketReadEEPROMResponse*)&TxDataBuf[TxTotLength];
                  pTKPacketReadEEPROMResponse ->header.packetID=pGenericSendHddr->packetID++; // get the packet ID Incrementing number
                  pTKPacketReadEEPROMResponse ->header.contentType= TK_PT_TKPacketReadEEPROMResponse;
                  pTKPacketReadEEPROMResponse ->header.contentLength= sizeof(struct TKPacketReadEEPROMResponse)+ sizeof(struct TKEEPROMData); 
                  TxTotLength= TxTotLength + sizeof(struct TKPacketReadEEPROMResponse);
                  memcpy(&TxDataBuf[TxTotLength],pEEPromData,sizeof(struct TKEEPROMData));// copy the eeprom data
                
//                TK_PT_TKPacketTKReadEEPROMResponse   
                  RxTotLength=RxTotLength + sizeof(struct TKPacketSetEEPROM);
                   TxTotLength= TxTotLength + sizeof(struct TKEEPROMData);
                   break;

            case TK_PT_TKPacketReadEEPROM:
//                TK_PT_TKPacketTKReadEEPROMResponse  
                  pTKPacketReadEEPROMResponse = (struct TKPacketReadEEPROMResponse*)&TxDataBuf[TxTotLength];
                  pTKPacketReadEEPROMResponse ->header.packetID=pGenericSendHddr->packetID++; // get the packet ID Incrementing number
                  pTKPacketReadEEPROMResponse ->header.contentType= TK_PT_TKPacketReadEEPROMResponse;
                  pTKPacketReadEEPROMResponse ->header.contentLength= sizeof(struct TKPacketReadEEPROMResponse)+ sizeof(struct TKEEPROMData); 
                  TxTotLength= TxTotLength + sizeof(struct TKPacketReadEEPROMResponse);
                  memcpy(&TxDataBuf[TxTotLength],pEEPromData,sizeof(struct TKEEPROMData));// copy the eeprom data
                    RxTotLength=RxTotLength + sizeof(struct TKPacketReadEEPROM);
                  break;

            case TK_PT_TKBCPacketIMU:
                  RxTotLength=RxTotLength + sizeof(struct TKPacketReadEEPROM);

                  break;
 
            case TK_PT_TKBCPacketButton:
                 
                    RxTotLength=RxTotLength + sizeof(struct TKBCPacketButton); 
                  
                  break;
                  case TK_PT_TKBCPacketRadioFrequency:
                    RxTotLength=RxTotLength + sizeof (struct TKBCPacketRadioFrequency);
                    
                    
                   break;
 
            case TK_PT_TKBCPacketStatus:
                  RxTotLength=RxTotLength + sizeof (struct TKBCPacketStatus);  
                  break;
 
            

            case TK_PT_TKBCPacketSetBeaconNumber:
              
                  pTKBCPacketSetBeaconNumber= ( struct TKBCPacketSetBeaconNumber*)&RxDataBuff[RxTotLength]; 
                  pTKBCPacketSetBeaconNumberACK = (struct TKBCPacketSetBeaconNumberACK*)&TxDataBuf[TxTotLength]; 
                  
                 pTKBCPacketSetBeaconNumberACK->sourcePacketID = pTKBCPacketSetBeaconNumber->header.packetID;
                 pTKBCPacketSetBeaconNumberACK->header.contentType=TK_PT_TKBCPacketSetBeaconNumberACK;
                pTKBCPacketSetBeaconNumberACK->header.contentLength= sizeof (struct TKBCPacketSetBeaconNumberACK);
               
                  RxTotLength=RxTotLength + sizeof (struct TKBCPacketSetBeaconNumber);  
                  TxTotLength= TxTotLength + sizeof(struct TKBCPacketSetBeaconNumberACK); 
                    
//                
                  break;
                  
                  
                  case TK_PT_TKBCPacketFlashLED:
                  RxTotLength=RxTotLength + sizeof(struct TKBCPacketFlashLED); 
                    pTKBCPacketFlashLEDACK=( struct TKBCPacketFlashLEDACK*)&TxDataBuf[TxTotLength];
                    pTKBCPacketFlashLEDACK->header.contentType =  TK_PT_TKBCPacketFlashLEDACK;
                    pTKBCPacketFlashLEDACK ->header.contentLength= sizeof(struct TKBCPacketFlashLEDACK);
                    pTKBCPacketFlashLEDACK ->sourcePacketID = pGHeader->packetID;
                    pTKBCPacketFlashLEDACK ->header.packetID =pGenericSendHddr->packetID++; // get the packet ID Incrementing number
                  TxTotLength= TxTotLength + sizeof(struct TKBCPacketFlashLEDACK);
                    break;

            
            case TK_PT_TKBCPacketPowerMode:
               pTKBCPacketPowerMode = (struct TKBCPacketPowerMode*)&RxDataBuff[RxTotLength];
                  if(pTKBCPacketPowerMode->beaconID < NumBeaconsActive){   // see if we have this beacon active
   
                    pBeacon_Data = CurrBeacons[pTKBCPacketPowerMode->beaconID]; // set the pointer to the correct beacon index
               pBeacon_Data->BC_PowerMode  = pTKBCPacketPowerMode-> powerMode;
               pTKBCPacketPowerModeACK= (struct TKBCPacketPowerModeACK*) &TxDataBuf[TxTotLength];
                 pTKBCPacketPowerModeACK->sourcePacketID = pTKBCPacketPowerMode->header.packetID;
                 pTKBCPacketPowerModeACK->header.contentType= TK_PT_TKBCPacketPowerModeACK;
                 pTKBCPacketPowerModeACK->header.contentLength= sizeof(struct TKBCPacketPowerModeACK);
                   pTKBCPacketPowerModeACK->header.packetID=pGenericSendHddr->packetID++;
               TxTotLength= TxTotLength + sizeof(struct TKBCPacketPowerModeACK);
               RxTotLength=RxTotLength + sizeof(struct TKBCPacketPowerMode);
                  }else{
                  RxTotLength=RxTotLength + sizeof(struct TKBCPacketPowerMode);  
                  TxTotLength=0;
                  }
                    
                    
                   break;
                   default:
                  printf("Unknown Packet type \n\r");
                  break;
      
                }// end of case
        }// end of while

  if(TxTotLength >= sizeof(struct TKFrame)){ // see if we have any sort of data to send...
    pTKFrameSend->contentLength =TxTotLength;
    Send_Brdcst_packet(pTxBuffHead,TxTotLength);
    TxTotLength=0;
  }
  
      }


}  

void Reply_Packet(uint8_t* pDataOut){
struct TKFrame *pTKFrameSend;
      pTKFrameSend= (struct TKFrame*)pDataOut;

        Send_Brdcst_packet(pDataOut,pTKFrameSend->contentLength);
  
 //       pGenericSendHddr->packetID ++ ;  
  
} 

uint32_t IncrementPacketId(void){
       return( pGenericSendHddr->packetID ++) ; 
}   

uint16_t  GetTKId(void){
      return(pGenericSendHddr->timekeeperID); 
}  