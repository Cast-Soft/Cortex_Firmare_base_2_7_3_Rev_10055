/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
// UDP_Process.C
// Processes UDP connection data as supplied from uIP stack in uip.c
// uip.c  calls UIP_UDP_APPCALL when UDP packets are received and ready
// UDP_Process takes the data in the rx buffer and dipatches it to the correct
// application...uses UDP port #s in header
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
#include "clock.h"
#include "includes.h"
#include "udp_process.h"
#include "packets.h"
#include "dhcpc.h"
#include "tst_dbg.h"
#include "uip.h"
#include "ethernet.h"
#include "basic_rf.h"
#include "beacon.h"
#include "main.h"

extern struct uip_udp_conn *uip_udp_conn;

#define DHCPC_SERVER_PORT  67
#define DHCPC_CLIENT_PORT  68

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

extern BOOL detailChanged;    // if changed, need to notify Bridge

struct BeaconData *pBeacon_Data;

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

uint32_t magicNumber = 0;         // session. Will be init when receiving the first packet
uint32_t lostRequests = 0;    // accumulates lost requests from Bridge
/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
// This initts all of the ether comm status stuff
//
//
//

void init_ether_comm(void){
    pEther_status= &ether_status;       // init the pointer
    pGenericSendHddr= &genericsendhddr; // init the pointer

    pEther_status->Probe_cnt =5;      // send probe count init
    pEther_status->BT_Link=1;         // TODO!!! remove  it. Not connection based.
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


void udp_process(void) {
    uint16_t local_port;

    local_port=(uip_udp_conn->lport);

    if(local_port == (HTONS(DHCPC_CLIENT_PORT))) {
        dhcpc_appcall();
    }
    else if(local_port == (HTONS(TK_Listen_Port))) {
        // }else if(remote_port == (HTONS(TK_Listen_Port))){
        if(uip_len) {         // see if we have data in the buffer
            TRACE(".");
            Packet_demux();
        }
    }
    else {
        // printf("No Port match \n\r");
        // printf("local port:%d \n\r",(HTONS(uip_udp_conn->lport)));
        // printf("Remote port:%d \n\r",(HTONS(uip_udp_conn->rport)));
    }
}

/*-------------------------------------------------------------------*/
// Packet demux reads header at start of payload packet and takes appropriate action
//
//uint32_t packet_type
//uint32_t packet_len;
/*---------------------------------------------------------------------*/
uint8_t content_type;

void  Packet_demux(void) {
    //uint32_t Pckt_Id, Pckt_Len;
    struct TKFrame *pTKFrame,*pTKFrameSend;
    struct GenericPHeader *pGHeader;
    void  *pTxBuffHead;

    uint32_t RxTotLength,TxTotLength;
    // set up to decode Rx packet

    pTKFrame = (struct TKFrame*) uip_sappdata; // uip_sappdata has incomming data
    //  if(pTKFrame->signature == TK_FRAME_SIGNATURE &&
#if 0
    if(pTKFrame->timekeeperID != pGenericSendHddr->timekeeperID) {
        return;
    }   // see if this is for us
#endif
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

        memcpy(&RxDataBuff,uip_sappdata,pTKFrame->contentLength);
        RxTotLength=sizeof(struct TKFrame);

        // printf("Got a demux Length:%d \n\r",pTKFrame->contentLength);


        while (pTKFrame->contentLength > RxTotLength) {
            pGHeader = (struct GenericPHeader*)(&RxDataBuff[RxTotLength]);
            content_type = pGHeader->contentType;
            // printf("Packet type %x Pckt Len %d \n\r",pGHeader->contentType,pGHeader->contentLength);
            switch(pGHeader->contentType) {
            case TK_PT_TKPacketPassThroughRequest:
              {
                uint8_t error = 1;
                struct PassThroughHddr * passThrough = NULL;
                if (pGHeader->contentLength >= (sizeof(struct PassThroughHddr))){
                  passThrough = (struct PassThroughHddr*) (&RxDataBuff[RxTotLength]);
                  if (passThrough->payloadSize > 127 || passThrough->payloadSize == 0) {
                    error = 2;
                  }
                  if (config.flags & FLAG_TRACE_UDP) {
                    TRACE("UDP to beacon %d at channel %d\n\r",  passThrough->destAddr,
                         passThrough->channel);
                    TRACE("  (%d bytes): 0x%02X 0x%02X 0x%02X 0x%02X ..\n\r",
                        passThrough->payloadSize, passThrough->payload[0],passThrough->payload[1],
                        passThrough->payload[2],passThrough->payload[3]);
                  }
                  __disable_interrupt();
                  for (uint8_t u = 0; u < 4; u++) {
                    if (!dataFramesTx[u].payloadSize) {
                      dataFramesTx[u].payloadSize = passThrough->payloadSize;
                      dataFramesTx[u].channel = passThrough->channel;
                      dataFramesTx[u].destAddr = passThrough->destAddr;
                      memcpy(dataFramesTx[u].payload, passThrough->payload, passThrough->payloadSize);
                      error = 0;
                      break;
                    }
                  }
                  __enable_interrupt();

                }
                  struct PassThroughHddrStatus * status = (struct PassThroughHddrStatus*) &TxDataBuf[TxTotLength];
                  status->header.packetID=pGenericSendHddr->packetID++; // get the packet ID Incrementing number
                  status->header.contentType= TK_PT_TKPacketPassThroughRequestStatus;
                  status->header.contentLength=sizeof(struct PassThroughHddrStatus);
                  status->sourcePacketID =  pGHeader->packetID;
                  status->result = error;
                  RxTotLength=RxTotLength + sizeof(struct PassThroughHddr);
                  if (passThrough) {
                    RxTotLength += passThrough->payloadSize - 1;
                  }
                  TxTotLength= TxTotLength + sizeof(struct PassThroughHddrStatus);
              }
              break;
                default:
                    {
                        TRACE("Unknown Packet type \n\r");
                        return;
                    }
            }// end of case
        }// end of while

        if(TxTotLength >= sizeof(struct TKFrame)){ // see if we have any sort of data to send...
            pTKFrameSend->contentLength = TxTotLength;
            EthAsyncSendPacket(pTxBuffHead,TxTotLength);
            TxTotLength=0;
        }
}

void Reply_Packet(uint8_t* pDataOut) {
    struct TKFrame *pTKFrameSend;
    pTKFrameSend= (struct TKFrame*)pDataOut;
    EthAsyncSendPacket(pDataOut,pTKFrameSend->contentLength);
    // pGenericSendHddr->packetID ++;
}

uint32_t IncrementPacketId(void){
    return( pGenericSendHddr->packetID++);
}

uint16_t GetTKId(void){
    return(pGenericSendHddr->timekeeperID);
}


