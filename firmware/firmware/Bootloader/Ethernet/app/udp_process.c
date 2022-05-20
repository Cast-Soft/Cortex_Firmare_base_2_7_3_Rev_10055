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
#include "uip.h"
#include "ethernet.h"
#include "basic_rf.h"
#include "beacon.h"
#include "main.h"

extern struct uip_udp_conn *uip_udp_conn;

#define DHCPC_SERVER_PORT  67
#define DHCPC_CLIENT_PORT  68
/*
#define TK_Listen_Port  23001
#define TK_Send_Port  23000
#define TK_Rand_Port 20000
*/

extern uint16_t uip_slen;

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
//    uint16_t remote_port;

    local_port=(uip_udp_conn->lport);
    //remote_port=(uip_udp_conn->rport);
    if(local_port == (HTONS(Bootloader_Listen_Port))) {
      if(uip_len) {
        
        struct RespUpdate up;
        *((uint8_t*) uip_appdata + uip_len) = 0;
      /*  uint32_t ret = SerialDownload(uip_appdata);
        up.index = (ret & 0xFFFF0000) >> 16;
        up.errorCode = ret & 0xFFFF;
        memcpy(uip_sappdata, (void*) &up, 4);
        uip_slen = 4;*/
        
        doPackets(uip_appdata, PEER_UDP);
      }
    }
}

size_t __writeNetworkResponse(uint8_t content_type, const unsigned char *buffer, size_t size)
{
  struct USBDebugInfoHeader dbgInfoheader;   
  int headerSize = sizeof(dbgInfoheader);
  uint8_t * resp_data = (uint8_t*) uip_sappdata;
  
  dbgInfoheader.signature = TK_FRAME_SIGNATURE;
  dbgInfoheader.version = 1; 
  dbgInfoheader.checkSum = 0;        // TODO!!!
  dbgInfoheader.magicNumber = 0;     // TODO!!! random number / session id
  dbgInfoheader.packetID = 0;        // TODO!!! Incrementing number
  dbgInfoheader.contentType = content_type;
  dbgInfoheader.contentLength = size + headerSize;     // Entire packet length including Header         
  
  const unsigned char *headerBuffer = (const unsigned char *)&dbgInfoheader; 
  memcpy(resp_data, (void*) &dbgInfoheader, headerSize);
  resp_data += headerSize;
  memcpy(resp_data, buffer, size);
  uip_slen = headerSize + size;
  return uip_slen;
  
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


