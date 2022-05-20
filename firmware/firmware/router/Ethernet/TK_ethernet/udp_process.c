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
#if 0
struct BeaconData *pBeacon_Data;
extern struct BeaconData* CurrBeacons[];
#endif

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
            // TRACE("Got a Data Pkt LocalP: %d Remot: %d \n\r",(HTONS(local_port)),(HTONS(remote_port)));
            TRACE(".");
            Packet_demux();
        }
    }
    else {
        // TRACE("No Port match \n\r");
        // TRACE("local port:%d \n\r",(HTONS(uip_udp_conn->lport)));
        // TRACE("Remote port:%d \n\r",(HTONS(uip_udp_conn->rport)));
    }
}

/*-------------------------------------------------------------------*/
// Packet demux reads header at start of payload packet and takes appropriate action
//
//uint32_t packet_type
//uint32_t packet_len;
/*---------------------------------------------------------------------*/

void  Packet_demux(void) {
    //uint32_t Pckt_Id, Pckt_Len;
    struct TKFrame *pTKFrame,*pTKFrameSend;
    struct GenericPHeader *pGHeader;
//    struct TKPacketStatusSummary *pTKPacketStatusSummary;
//    struct TKPacketStatusDetail *pTKPacketStatusDetail;
    void  *pTxBuffHead;

    uint32_t RxTotLength,TxTotLength;
    // set up to decode Rx packet

    func = __func__;
    line = __LINE__;

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
        pTKFrameSend->version_major = RT_VERSION_MAJOR;
        pTKFrameSend->version_minor = RT_VERSION_MINOR;
        pTKFrameSend->version_revision = RT_VERSION_REVISION;
        pTKFrameSend->version_build = RT_VERSION_BUILD;
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

        // TRACE("Got a demux Length:%d \n\r",pTKFrame->contentLength);

        func = __func__;
        line = __LINE__;

        while (pTKFrame->contentLength > RxTotLength) {
            pGHeader = (struct GenericPHeader*)(&RxDataBuff[RxTotLength]);
            // TRACE("Packet type %x Pckt Len %d \n\r",pGHeader->contentType,pGHeader->contentLength);
            switch(pGHeader->contentType) {
            case TK_PT_TKPacketRadioControl:
                    {
                        func = __func__;
                        line = __LINE__;

                        pTKPacketRadioControlACK=(struct TKPacketRadioControlACK*)&TxDataBuf[TxTotLength];
                        // TK_PT_TKPacketRadioControlACK
                        pTKPacketRadioControlACK->header.packetID=pGenericSendHddr->packetID++; // get the packet ID Incrementing number
                        pTKPacketRadioControlACK->header.contentType= TK_PT_TKPacketRadioControlACK;
                        pTKPacketRadioControlACK->header.contentLength=sizeof(struct TKPacketRadioControlACK);
                        RxTotLength=RxTotLength + sizeof(struct TKPacketRadioControl);
                        TxTotLength= TxTotLength + sizeof(struct TKPacketRadioControlACK);
                    }
                    break;
                case TK_PT_TKPacketSyncControl:
                    {
                        func = __func__;
                        line = __LINE__;
                        pTKPacketSyncControlACK=(struct TKPacketSyncControlACK*)&TxDataBuf[TxTotLength];
                        // TK_PT_TKPacketSyncControlACK
                        pTKPacketSyncControlACK->header.packetID=pGenericSendHddr->packetID++; // get the packet ID Incrementing number
                        pTKPacketSyncControlACK->header.contentType=  TK_PT_TKPacketSyncControlACK ;
                        pTKPacketSyncControlACK->header.contentLength=sizeof(struct TKPacketSyncControlACK);
                        RxTotLength=RxTotLength + sizeof(struct TKPacketSyncControl);
                        TxTotLength= TxTotLength + sizeof(struct TKPacketSyncControlACK);
                    }
                    break;
                case TK_PT_TKPacketSetEEPROM:
                    {
                        func = __func__;
                        line = __LINE__;
                        pTKPacketReadEEPROMResponse = (struct TKPacketReadEEPROMResponse*)&TxDataBuf[TxTotLength];
                        pTKPacketReadEEPROMResponse ->header.packetID=pGenericSendHddr->packetID++; // get the packet ID Incrementing number
                        pTKPacketReadEEPROMResponse ->header.contentType= TK_PT_TKPacketReadEEPROMResponse;
                        pTKPacketReadEEPROMResponse ->header.contentLength= sizeof(struct TKPacketReadEEPROMResponse)+ sizeof(struct TKEEPROMData);
                        TxTotLength= TxTotLength + sizeof(struct TKPacketReadEEPROMResponse);
                        memcpy(&TxDataBuf[TxTotLength],pEEPromData,sizeof(struct TKEEPROMData));// copy the eeprom data

                        // TK_PT_TKPacketTKReadEEPROMResponse
                        RxTotLength=RxTotLength + sizeof(struct TKPacketSetEEPROM);
                        TxTotLength= TxTotLength + sizeof(struct TKEEPROMData);
                    }
                    break;
                case TK_PT_TKPacketReadEEPROM:
                    {
                        func = __func__;
                        line = __LINE__;
                        // TK_PT_TKPacketTKReadEEPROMResponse
                        pTKPacketReadEEPROMResponse = (struct TKPacketReadEEPROMResponse*)&TxDataBuf[TxTotLength];
                        pTKPacketReadEEPROMResponse ->header.packetID=pGenericSendHddr->packetID++; // get the packet ID Incrementing number
                        pTKPacketReadEEPROMResponse ->header.contentType= TK_PT_TKPacketReadEEPROMResponse;
                        pTKPacketReadEEPROMResponse ->header.contentLength= sizeof(struct TKPacketReadEEPROMResponse)+ sizeof(struct TKEEPROMData);
                        TxTotLength= TxTotLength + sizeof(struct TKPacketReadEEPROMResponse);
                        memcpy(&TxDataBuf[TxTotLength],pEEPromData,sizeof(struct TKEEPROMData));// copy the eeprom data
                        RxTotLength=RxTotLength + sizeof(struct TKPacketReadEEPROM);
                    }
                    break;
                case TK_PT_TKBCPacketIMU:
                    {
                        RxTotLength=RxTotLength + sizeof(struct TKPacketReadEEPROM);
                    }
                    break;
                case TK_PT_TKBCPacketButton:
                    {
                        RxTotLength=RxTotLength + sizeof(struct TKBCPacketButton);
                    }
                    break;
                case TK_PT_TKBCPacketRadioFrequency:
                    {
                        RxTotLength=RxTotLength + sizeof (struct TKBCPacketRadioFrequency);
                    }
                    break;
                case TK_PT_TKBCPacketStatus:
                    {
                        RxTotLength=RxTotLength + sizeof (struct TKBCPacketStatus);
                    }
                    break;
                case TK_PT_TKBCPacketSetBeaconNumber:
                    {
                        pTKBCPacketSetBeaconNumber= ( struct TKBCPacketSetBeaconNumber*)&RxDataBuff[RxTotLength];
                        pTKBCPacketSetBeaconNumberACK = (struct TKBCPacketSetBeaconNumberACK*)&TxDataBuf[TxTotLength];

                        pTKBCPacketSetBeaconNumberACK->sourcePacketID = pTKBCPacketSetBeaconNumber->header.packetID;
                        pTKBCPacketSetBeaconNumberACK->header.contentType=TK_PT_TKBCPacketSetBeaconNumberACK;
                        pTKBCPacketSetBeaconNumberACK->header.contentLength= sizeof (struct TKBCPacketSetBeaconNumberACK);

                        RxTotLength=RxTotLength + sizeof (struct TKBCPacketSetBeaconNumber);
                        TxTotLength= TxTotLength + sizeof(struct TKBCPacketSetBeaconNumberACK);
                    }
                    break;
                case TK_PT_TKBCPacketFlashLED:
                    {
                        RxTotLength=RxTotLength + sizeof(struct TKBCPacketFlashLED);
                        pTKBCPacketFlashLEDACK=( struct TKBCPacketFlashLEDACK*)&TxDataBuf[TxTotLength];
                        pTKBCPacketFlashLEDACK->header.contentType =  TK_PT_TKBCPacketFlashLEDACK;
                        pTKBCPacketFlashLEDACK ->header.contentLength= sizeof(struct TKBCPacketFlashLEDACK);
                        pTKBCPacketFlashLEDACK ->sourcePacketID = pGHeader->packetID;
                        pTKBCPacketFlashLEDACK ->header.packetID =pGenericSendHddr->packetID++; // get the packet ID Incrementing number
                        TxTotLength= TxTotLength + sizeof(struct TKBCPacketFlashLEDACK);
                    }
                    break;
                case TK_PT_TKBCPacketPowerMode:
                    {
                        pTKBCPacketPowerMode = (struct TKBCPacketPowerMode*)&RxDataBuff[RxTotLength];
                            RxTotLength=RxTotLength + sizeof(struct TKBCPacketPowerMode);
                            TxTotLength=0;
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
            func = __func__;
            line = __LINE__;
            pTKFrameSend->contentLength = TxTotLength;
            EthAsyncSendPacket(pTxBuffHead,TxTotLength);
            func = __func__;
            line = __LINE__;
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


