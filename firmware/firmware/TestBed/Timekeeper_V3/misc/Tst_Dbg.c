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


#ifdef Ether_active
/*----------------------------------------------------------------*/
// UDP includes
//
//TimeKeeper -> BlackTrax Server "238.210.10.1:23000"
//BlackTrax Server->TimeKeeper "238.210.10.0:23001"



// TK_Snd_MCast  238.210.10.1
#define TK_Send_Port  23000
#define TK_Listen_Port  23001
#define Imu_Packet 0x00000101
#define Discovery_Probe 0x00000102
#define Discovery_ACK 0x00000103
#define Discovery_Echo 0x00000104


// TK_Rx_MCast  238.210.10.1

const uint8_t Mcast1[4]={238,210,10,1};
const struct uip_eth_addr Mcast_Eth={0x01,0,0x5E,0,0,0};
struct uip_eth_addr Ethe_addr ;

//uint8_t Ether_pkt[1500];
uint8_t Ether_cnt = 0;

struct uip_udp_conn *uni_conn1, *Mcst_conn1;

typedef struct{
  uint32_t packet_type;
  uint32_t packet_len;
}udp_hddr_t;

udp_hddr_t udp_hddr_str, PayLoad;

typedef union {
uint8_t Ether_pkt[1500];
uint32_t Ether_pkt_32[375];
} hddr_union;

hddr_union header_union;
hddr_union *pHeader_union;

extern void *uip_sappdata;




void Init_TK_ports(void);
void Send_Brdcst_packet(uint8_t* Tx_data, uint16_t len);
void IP_packet_test(uint32_t);
void InitMultiCast(void);

extern void ETH_MACAddressConfig(uint32_t MacAddr, u8 *Addr);
extern void ETH_MACAddressPerfectFilterCmd(uint32_t MacAddr, FunctionalState NewState);
#endif
/*----------------------------------------------------------------------*/





/***************************************************************************/
/***************************************************************************/
/**************************************************************************/
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

//




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
         IP_packet_test(Imu_Packet);
       }else if (Loop_Char == 'i'){
         IP_packet_test(Discovery_Probe); 
       }else if (Loop_Char == 'o'){
         IP_packet_test(Discovery_Echo); 
         }else if (Loop_Char == 'p'){
         IP_packet_test(Discovery_ACK); 
         
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
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

void Init_TK_ports(void){  // setup udp ports for send and listen
uip_ipaddr_t addr;
  
    InitMultiCast();  // set up arp cache for multicast
    
    uip_ipaddr(addr, Mcast1[0],Mcast1[1],Mcast1[2],Mcast1[3]); // convert MCast IP to packed 16 bit
#ifdef TimeKeeper   
    Mcst_conn1 = uip_udp_new(&addr, HTONS(TK_Send_Port));   // init a new uDP connection for mcast
    if(Mcst_conn1  != NULL) {
          uip_udp_bind(Mcst_conn1, HTONS(TK_Listen_Port));     // bind the udp listen port to mcast listen
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


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
// IP_packet test...udp test packet send..triggered from debug serial port
//
//
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/


void IP_packet_test(uint32_t TK_Pkt_typ){

uint32_t   udp_dta_len;
          
 
 //         TK_Pkt_typ = Imu_Packet; 
          header_union.Ether_pkt_32[0]= TK_Pkt_typ ;// type

          udp_dta_len= 104; 
          header_union.Ether_pkt_32[1]= udp_dta_len ;  // length
          
          Send_Brdcst_packet(header_union.Ether_pkt, 960);
            
  
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
   if(uip_len > 0){     // if we have alength we have data to send
     uip_arp_out();     // use arp to fill in Ether addr
    tapdev_send(uip_buf,uip_len); // off to the Mac/Phy?wire
    }
}





void Send_Unicast_packet(uint8_t* Tx_data, uint16_t len){
  
}

/*-------------------------------------------------------------------*/
// Packet demux reads header at start of payload packet and takes appropriate action
//
//uint32_t packet_type
//uint32_t packet_len;
/*---------------------------------------------------------------------*/


void  Packet_demux(void){
//uint32_t Pckt_Id, Pckt_Len;  


      memcpy(&PayLoad , uip_sappdata,8);  
 
  printf("Packet type %x Pckt Len %d \n\r",PayLoad.packet_type,PayLoad.packet_len);

      switch(PayLoad.packet_type){
    
       case Discovery_ACK:
         printf("ACK from Btrx \n\r");
      break;
      
      default:
        break;
      
  }



}  



#endif // ethernet