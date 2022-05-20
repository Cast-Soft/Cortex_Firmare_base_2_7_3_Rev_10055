/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
******************************************************************************/
// Test debug Code
//
//
//
//
/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/

#include "includes.h"
#include "basic_rf.h"
#include "tst_dbg.h"
#include "uip_arp.h"


#include "packets.h"
#include "main.h"

/*----------------------------------------------------------------*/
// UDP includes
//
//TimeKeeper -> BlackTrax Server "238.210.10.1:23000"
//BlackTrax Server->TimeKeeper "238.210.10.0:23001"



// TK_Snd_MCast  238.210.10.1



// TK_Rx_MCast  238.210.10.1

const uint8_t Mcast1[4]={238,210,10,1};
const struct uip_eth_addr Mcast_Eth={0x01,0,0x5E,0,0,0};
struct uip_eth_addr Ethe_addr ;



//struct Disc_Packet D_Packet, *pD_Packet;

//uint8_t Ether_pkt[1500];
uint8_t Ether_cnt = 0;

uint16_t IMU_dta_cnt = 0;

struct uip_udp_conn *uni_conn1, *Mcst_conn1;


udp_hddr_t udp_hddr_str;
//hddr_union_t header_union, *pHeader_union;

//uint32_t *pHeader_union32 =  &header_union.Ether_pkt_32[0];
//uint8_t  *pHeader_union8 =  &header_union.Ether_pkt[0];

uint8_t SendBuff[1500];

extern void *uip_sappdata;

void Init_TK_ports(uint16_t src_port, uint16_t dest_port);
void EthAsyncSendPacket(uint8_t* Tx_data, uint16_t len);
void IP_packet_test(uint32_t);
void InitMultiCast(void);

extern void ETH_MACAddressConfig(uint32_t MacAddr, u8 *Addr);
extern void ETH_MACAddressPerfectFilterCmd(uint32_t MacAddr, FunctionalState NewState);

extern struct Ether_status *pEther_status;
extern  uip_ipaddr_t multiCastGroup;

struct DeviceSerialNumber ARM_proc_SN;
struct DeviceSerialNumber *pARM_proc_SN = &ARM_proc_SN;

//extern struct Ether_status;

/*----------------------------------------------------------------------*/

/*typedef struct {
    uint8_t seqNumber;
    uint16_t srcAddr;
    uint16_t srcPanId;
    int8_t length;
    uint8_t* pPayload;
    uint8_t ackRequest;
    int8_t rssi;
    volatile uint8_t isReady;
    uint8_t status;
} basicRfRxInfo_t;
*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
//Checks Ether Link status
//
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

void Chek_lnk(void){
uint16_t  link_stts;

extern unsigned int PhyAddr;

 link_stts = ETH_ReadPHYRegister(PhyAddr, PHY_BSR);
 if(link_stts & PHY_Linked_Status ){
   if(!(pEther_status->Link_Up)){
     TRACE("Ether link UP \n\r");
     pEther_status->Link_Up=1;
   }
 }else if(pEther_status->Link_Up){
   pEther_status->Link_Up=0;
   TRACE("Ether Link DOWN \n\r");
    }
}

/*-----------------------------------------------------------*/
// ARM processor unique ID
// set at manufacturing time
// 96 bits UUID
/*-----------------------------------------------------------*/
void GetARM_UUID(void){


    ARM_proc_SN.a = *(__IO uint32_t *)(0x1FFFF7E8);
    ARM_proc_SN.b = *(__IO uint32_t *)(0x1FFFF7EC);
    ARM_proc_SN.c = *(__IO uint32_t *)(0x1FFFF7F0);




}




/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
//  Ether net Comm stuff
//
//
// Set up Ethernet com ports and addresses
// Initialise the ARP cache with the Multicast group address and  Multicast Ether address
// Init the UDP connection with IP addresses Host and Mcast group and UDP ports
//
//  struct uip_udp_conn {
//  uip_ipaddr_t ripaddr;   /**< The IP address of the remote peer. */
//  u16_t lport;        /**< The local port number in network byte order. */
//  u16_t rport;        /**< The remote port number in network byte order. */
//  u8_t  ttl;          /**< Default time-to-live. */
//
//
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

void Init_TK_ports(uint16_t src_port, uint16_t dest_port){  // setup udp ports for send and listen
    InitMultiCast();  // set up arp cache for multicast

    uip_ipaddr_t addr;
    uip_ipaddr(addr, Mcast1[0],Mcast1[1],Mcast1[2],Mcast1[3]); // convert MCast IP to packed 16 bit

    Mcst_conn1 = uip_udp_new(&addr, HTONS(/*TK_Send_Port*/ dest_port));   // init a new UDP connection for mcast output
    assert(Mcst_conn1 != NULL);

    {
        uip_ipaddr_t all_zeroes_addr = {0x0000,0x0000};
        uni_conn1 = uip_udp_new(&all_zeroes_addr, 0);   // init a new uDP connection for listen
        assert(uni_conn1 != NULL);
        uip_udp_bind(uni_conn1, HTONS(/*TK_Listen_Port*/src_port));
    }
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
// Multicast setup
// puts multi cast addr in ARP table
// 01,00,5E,00,00,00 MAC address
// 238,210,10,1 IP Multicast address
// Multicast Mac address 01,00,5E,52,0A,01
// outgoing...set multicast MAC in ARP cache and multicast IP
// uip_arp_update(u16_t *ipaddr, struct uip_eth_addr *ethaddr)
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

void InitMultiCast(void){


 // set up the Multicast ether addr
// bytes 0,1,2 of   IANA OUI 01,00,5E
// bytes 3,4,5 contain bytes 1,2 and 3 of multicast group IP addr
// with msb of byte 1 set to 0  (&0x7F)

  Ethe_addr.addr[0]= Mcast_Eth.addr[0];
  Ethe_addr.addr[1]= Mcast_Eth.addr[1];
  Ethe_addr.addr[2]= Mcast_Eth.addr[2];
  Ethe_addr.addr[3]= (Mcast1[1] & 0x7F) ;
  Ethe_addr.addr[4]= Mcast1[2];
  Ethe_addr.addr[5]= Mcast1[3];

// convert multicast Tx group IP addr to packed 16 bit

    uip_ipaddr(multiCastGroup, Mcast1[0],Mcast1[1],Mcast1[2],Mcast1[3]);
// Send mcast Ether addres and mcast IP addr to ARP table
//  uip_arp_update(multiCastGroup,&Ethe_addr);
     uip_static_arp_update(multiCastGroup,&Ethe_addr);
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
// send_IMU_ether
// send imu data from radio rx to multicast group via ether
// sends 10 complete radio packets of 102 bytes each
// Total of 1020 bytes plus 8 bytes (2 x 32 bit words) preamble
// word 1 packet type (imu data) word 2 (overall length) 1028 / 32 = 257
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

void send_IMU_ether(uint8_t* pIMU_dta){
uint8_t count;


      for(count=0; count < 102;count ++){

 //           header_union.Ether_pkt[offset++]= pIMU_dta[count];
            }
          IMU_dta_cnt++;

          if(IMU_dta_cnt >= 10){

 //           header_union.Ether_pkt_32[0]=TK_PT_TKBCPacketIMU;
//            header_union.Ether_pkt_32[1]=(((IMU_dta_cnt * 102) + 8)/4) ;
            IMU_dta_cnt=0   ;      // reset
//            EthAsyncSendPacket(header_union.Ether_pkt, 1028);
          }

}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
// IP_packet test...udp test packet send..triggered from debug serial port
//
//
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

void IP_packet_test(uint32_t TK_Pkt_typ){

}
//UIP_IPUDPH_LEN = 28

// push into message queue
// This function might be called from different tasks. It posts the message to EtherTask queue for processing
// Directly sending in other tasks results in race condition
void EthAsyncSendPacket(uint8_t* Tx_data, uint16_t len){
    void* msg = MallocMsg(len);
    if (msg == NULL) {
        assert(msg != NULL);
    }
    void* body = GetMsgBody(msg);                   // skip the header
    MsgHeader* msgHeader = GetMsgHeader(msg);      // the header
    // Set to 0, so it will not have DO_NOT_FREE flag
    // and will be disposed in MsgLoop
    msgHeader->msgType = 0;
    SET_TX_PACKET(msgHeader->msgType);
    msgHeader->msgLength = len;
    memcpy((u8_t*)body, Tx_data, len);

    StatusType status = CoPostQueueMail(ethTaskMsgQueue, msg);

    if (status == E_QUEUE_FULL) {
#if 0
        StatusType result;
        void *txMsg = CoAcceptQueueMail(ethTxMsgQueue, &result);
        CoKfree(txMsg);
        status = CoPostQueueMail(ethTxMsgQueue, msg);
#endif
        // No need to check for DO_NOT_FREE
        CoKfree(msg);
    }
    else {
        assert(status == E_OK);     // debug if it is full: Found wireless ready but ethenet not
    }
}

/*-----------------------------------------------------------------------*/
// Send uni-cast Packet
// sends a packet to the multicast Tx address:port
// UDP multicast connection struct: Mcst_conn1
// needs pointer to data and length of data
//
/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/




void Send_Unicast_packet(uint8_t* Tx_data, uint16_t len){

}




//#endif // ethernet