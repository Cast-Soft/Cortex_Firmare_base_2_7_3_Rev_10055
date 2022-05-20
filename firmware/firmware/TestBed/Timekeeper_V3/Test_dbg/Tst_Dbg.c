/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
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


#ifdef Ether_active
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


  
struct Disc_Packet D_Packet, *pD_Packet;
  
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

void Init_TK_ports(void);
void Send_Brdcst_packet(uint8_t* Tx_data, uint16_t len);
void IP_packet_test(uint32_t);
void InitMultiCast(void);

extern void ETH_MACAddressConfig(uint32_t MacAddr, u8 *Addr);
extern void ETH_MACAddressPerfectFilterCmd(uint32_t MacAddr, FunctionalState NewState);

extern struct Ether_status *pEther_status;

struct DeviceSerialNumber ARM_proc_SN , *pARM_proc_SN ;


//extern struct Ether_status;
#endif
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
     printf("Ether link UP \n\r");
     pEther_status->Link_Up=1;
   }
 }else if(pEther_status->Link_Up){
   pEther_status->Link_Up=0;
   printf("Ether Link DOWN \n\r");
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
void  test_stuff(void){
uint8_t   Loop_Char;
// check for new char
// and parse , s triggers send of 1 packet, t sends 10 packets and y sends 100 packets
          
          Loop_Char = getchar();
        
        if(Loop_Char == '\x1B'){
          //main_loop_ctl= 1;
          
        }else if(Loop_Char == 'l'){
         //LoadTestString();

        }else if(Loop_Char == 't'){
         //Tx_loop_ctr(10);
       
         
         }else if(Loop_Char == 'y'){
           Tx_loop_ctr(100);
//         set_seq_num(0);
//         printf("\n\r Test 100 Chan: %d \n\r",Test_chan);
         
        }else if(Loop_Char == 's'){
         Tx_loop_ctr(1);
        }    
#ifdef Ether_active 
         
       else if (Loop_Char == 'u'){
//         IP_packet_test(Imu_Packet);
       }else if (Loop_Char == 'i'){
//         IP_packet_test(Discovery_Probe); 
       }else if (Loop_Char == 'o'){
//         IP_packet_test(TKPacketStatusSummary); 
         }else if (Loop_Char == 'p'){
//         IP_packet_test(Discovery_ACK); 
         
        }else if (Loop_Char == 'w'){
         Init_TK_ports();
        }
  
#endif 

}


#ifdef Ether_active
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

void Init_TK_ports(void){  // setup udp ports for send and listen
uip_ipaddr_t addr;
  
    InitMultiCast();  // set up arp cache for multicast
    
    uip_ipaddr(addr, Mcast1[0],Mcast1[1],Mcast1[2],Mcast1[3]); // convert MCast IP to packed 16 bit
#ifdef TimeKeeper   
    Mcst_conn1 = uip_udp_new(&addr, HTONS(TK_Send_Port));   // init a new uDP connection for mcast
    if(Mcst_conn1  != NULL) {
         uip_udp_bind(Mcst_conn1, HTONS(TK_Listen_Port));     // bind the udp listen port to mcast listen
//        uip_udp_bind(Mcst_conn1, HTONS(TK_Rand_Port));     // bind the udp listen port to mcast listen
        }
#else //If Olimex for Multicast test
    
      Mcst_conn1 = uip_udp_new(&addr, HTONS(TK_Listen_Port));   // init a new uDP connection for mcast
       if(Mcst_conn1  != NULL) {
          uip_udp_bind(Mcst_conn1, HTONS(TK_Send_Port));     // bind the udp listen port to mcast listen
           }   
#endif    
       
       
    printf("Init UDP address table \n\r");    

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
extern  uip_ipaddr_t multiCastGroup;  

 
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
  uip_arp_update(multiCastGroup,&Ethe_addr);
  
  // set up MAC to pass all multicast 
    ETH_MACAddressConfig(ETH_MAC_Address1, Ethe_addr.addr);
    // put multicast address in MAC alternate address register
    // this is where we can put the call to the IGMP code to Join the multicast group
     ETH_MACAddressPerfectFilterCmd(ETH_MAC_Address1, ENABLE);

  
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
uint16_t offset;

        
          offset = ((IMU_dta_cnt * 102)+8);
            for(count=0; count < 102;count ++){
              
 //           header_union.Ether_pkt[offset++]= pIMU_dta[count];
            }
          IMU_dta_cnt++;
          
          if(IMU_dta_cnt >= 10){
            
 //           header_union.Ether_pkt_32[0]=TK_PT_TKBCPacketIMU;
//            header_union.Ether_pkt_32[1]=(((IMU_dta_cnt * 102) + 8)/4) ; 
            IMU_dta_cnt=0   ;      // reset
//            Send_Brdcst_packet(header_union.Ether_pkt, 1028);
          }
  
}  

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
// Discovery probe
// Sends discovery packet to BT server
//
//
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
void discovery_probe(void){
struct TKFrame *pTemp1;   
struct TKPacketDiscovery *pTemp2;

extern struct  GenericSendHddr *pGenericSendHddr;

        pTemp1= (struct TKFrame*)(&SendBuff);
 
       
       
        pTemp1->timekeeperID= pGenericSendHddr->timekeeperID;
        pTemp1->contentLength= sizeof( struct TKFrame);
        pTemp2 =(struct TKPacketDiscovery*)(&SendBuff[sizeof( struct TKFrame)]); 
        
        pTemp2->header.packetID= pGenericSendHddr->packetID; // get the packet ID
        IncrementPacketId();                          // and increment it for the next guy
        pTemp2->header.contentType=  TK_PT_TKPacketDiscovery;
        pTemp2->header.contentLength= sizeof( struct TKPacketDiscovery);
        pTemp1->contentLength= pTemp1->contentLength + pTemp2->header.contentLength;
        
 
        Send_Brdcst_packet(SendBuff,pTemp1->contentLength);
  
}  

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
// IP_packet test...udp test packet send..triggered from debug serial port
//
//
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

void IP_packet_test(uint32_t TK_Pkt_typ){

uint32_t   udp_dta_len;
          
 
 //         TK_Pkt_typ = Imu_Packet; 
//          header_union.Ether_pkt_32[0]= TK_Pkt_typ ;// type

          udp_dta_len= 104; 
//          header_union.Ether_pkt_32[1]= udp_dta_len ;  // length
          
//          Send_Brdcst_packet(header_union.Ether_pkt, 960);
            
  
//    Send_Brdcst_packet((uint8_t*) tst_tx_string, sizeof tst_tx_string);
}  
//UIP_IPUDPH_LEN = 28

/*-----------------------------------------------------------------------*/
// Send Brdcast Packet
// sends a packet to the multicast Tx address:port
// UDP multicast connection struct: Mcst_conn1
// needs pointer to data and length of data
//
/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/

void Send_Brdcst_packet(uint8_t* Tx_data, uint16_t len){

  
    uip_sappdata=(&uip_buf[UIP_IPUDPH_LEN + UIP_LLH_LEN]) ; // get pointer to correct place in out buffer
    uip_send(Tx_data, len); // coppy data to uip buffer
    uip_udp_conn = Mcst_conn1; // load connection data..dest addr:port our addr:port
   uip_process(UIP_UDP_SEND_CONN); // processes and builds out packet
   if(uip_len > 0){     // if we have a length we have data to send
     uip_arp_out();     // use arp to fill in Ether addr
    tapdev_send(uip_buf,uip_len); // off to the Mac/Phy?wire
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




#endif // ethernet