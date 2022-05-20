  /*************************************************************************
 *
 *    Used with ARM IAR C/C++ Compiler
 *
 *    (c) Copyright IAR Systems 2009
 *
 *    $Revision: #1 $
 **************************************************************************/
#include "webserver.h"
#include "ethernet.h"
#include "dhcpc.h"
#include <stdio.h>
#include <string.h>
#include "basic_rf.h"
#
//#include "tst_dbg.h"
#include "beacon.h"
#include "hardware.h"
#include "main.h"

#define BUF ((struct uip_eth_hdr *)&uip_buf[0])
/*
#define DHCPC_SERVER_PORT  67
#define DHCPC_CLIENT_PORT  68
*/

//#define TK_Rand_Port 20000


//OS_FlagID flagEtherTxDone = 0xFF;        // emitted by Ethernet TX interrupt
extern struct uip_udp_conn *uni_conn1, *Mcst_conn1;

extern void *uip_sappdata;
uint16_t tempx;
struct timer PHY_Delay;

struct uip_eth_addr mac_addr;
int st_len;
struct timer periodic_timer, arp_timer;

extern unsigned int PhyAddr;
uint16_t  ether_rx_packet_length;

#pragma data_alignment=4
uint8_t RxBuff[EMAC_MAX_PACKET_SIZE];
#pragma data_alignment=4
uint8_t TxBuff[EMAC_MAX_PACKET_SIZE];

void SendToEthDriver(void *pPacket, uint32_t size);

   
//const uint8_t Mcast1[4]={238,210,10,1};
const uint8_t Mcast1[4]={10,1,1,137};

const struct uip_eth_addr Mcast_Eth={0x01,0,0x5E,0,0,0};
struct uip_eth_addr Ethe_addr ;

uint8_t ether_rx_buffer[UIP_CONF_BUFFER_SIZE];


//uint8_t Ether_pkt[1500];
uint8_t Ether_cnt = 0;

uint16_t IMU_dta_cnt = 0;

struct uip_udp_conn *uni_conn1, *Mcst_conn1;

   
/*************************************************/
/*************************************************************************
 * Function Name: uip_log
 * Parameters: none
 *
 * Return: none
 *
 * Description: Events loggin
 *
 *************************************************************************/
void uip_log (char *m)
{
  printf("uIP log message: %s\n", m);
}

// skip the header
void* GetMsgBody(void* msg) {
    return (uint8_t*)msg + sizeof(MsgHeader);
}
  
// header
MsgHeader* GetMsgHeader(void* msg) {
  return msg;
}

/*************************************************************************/
// Tic timer setup
// Uses Timer 2 interrupt to set up system tic at 100mS
//
//
/*************************************************************************/
  // Sys timer init 1/100 sec tick

void uIP_timer_setup(){
//  clock_init(2);

  timer_set(&periodic_timer, CLOCK_SECOND / 2);
  timer_set(&arp_timer, CLOCK_SECOND * 10);
  
}

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
//  uip_arp_update(multiCastGroup,&Ethe_addr);
     uip_static_arp_update(multiCastGroup,&Ethe_addr);
#if 0
  // set up MAC to pass all multicast
    ETH_MACAddressConfig(ETH_MAC_Address1, Ethe_addr.addr);
    // put multicast address in MAC alternate address register
    // this is where we can put the call to the IGMP code to Join the multicast group
     ETH_MACAddressPerfectFilterCmd(ETH_MAC_Address1, ENABLE);
#endif

}

void InitPorts(void){  // setup udp ports for send and listen
    InitMultiCast();  // set up arp cache for multicast

    uip_ipaddr_t addr;    
    uip_ipaddr(addr, Mcast1[0],Mcast1[1],Mcast1[2],Mcast1[3]); // convert MCast IP to packed 16 bit
#if defined(ROUTER) || defined(TIMEKEEPER)
    Mcst_conn1 = uip_udp_new(&addr, HTONS(Bootloader_Send_Port));   // init a new UDP connection for mcast output
    assert(Mcst_conn1 != NULL);
    {
        uip_ipaddr_t all_zeroes_addr = {0x0000,0x0000};   
        uni_conn1 = uip_udp_new(&all_zeroes_addr, 0);   // init a new uDP connection for listen
        assert(uni_conn1 != NULL);
        uip_udp_bind(uni_conn1, HTONS(Bootloader_Listen_Port));
    }
    
#else //If Olimex for Multicast test
  //  assert(0);      // not support
    Mcst_conn1 = uip_udp_new(&addr, HTONS(Bootloader_Send_Port));   // init a new uDP connection for mcast
    if(Mcst_conn1  != NULL) {
        uip_udp_bind(Mcst_conn1, HTONS(Bootloader_Listen_Port));     // bind the udp listen port to mcast listen
    }
#endif


    printf("Init UDP address table \n\r");

}

/*************************************************************************
 * Function Name: uIPMain
 * Parameters: none
 *
 * Return: uint32_t
 *
 * Description: Inits all timers and Tape Dev
 *
 *************************************************************************/
uint32_t uIP_Setup(void)
{
  
  extern struct Ether_status *pEther_status;  
  uip_ipaddr_t ipaddr;
  
  // Initialize the ethernet device driver
  // Init MAC
  // Phy network negotiation
  tapdev_init();
  
 //***************************************** 

//#ifdef TimeKeeper
  
  mac_addr.addr[0] = 0x00;  // o0
  mac_addr.addr[1] = 0x25;  //
  mac_addr.addr[2] = 0x62;
  mac_addr.addr[3] = 0x00;
  mac_addr.addr[4] = 0x0C;
  mac_addr.addr[5] = 0x02;
//#endif  

  ETH_MACAddressConfig(ETH_MAC_Address0, mac_addr.addr);
  
  ETH_GetMACAddress(ETH_MAC_Address0, mac_addr.addr);
  
 // uip_setethaddr(mac_addr);
  
  printf(" MAC ADDR Dec :%d %d %d %d %d %d \n\r ", mac_addr.addr[0], mac_addr.addr[1],mac_addr.addr[2],mac_addr.addr[3],mac_addr.addr[4],mac_addr.addr[5]);
  printf(" MAC ADDR Hex :%x %x %x %x %x %x \n\r ", mac_addr.addr[0], mac_addr.addr[1],mac_addr.addr[2],mac_addr.addr[3],mac_addr.addr[4],mac_addr.addr[5]);

  uip_ethaddr.addr[0] = mac_addr.addr[0];
  uip_ethaddr.addr[1] = mac_addr.addr[1];
  uip_ethaddr.addr[2] = mac_addr.addr[2];
  uip_ethaddr.addr[3] = mac_addr.addr[3];
  uip_ethaddr.addr[4] = mac_addr.addr[4];
  uip_ethaddr.addr[5] = mac_addr.addr[5];

  
  //*******************************************************************
  
  
  
  // uIP web server
  // Initialize the uIP TCP/IP stack.
  uip_init();
  uip_arp_init();

  timer_set(&PHY_Delay, CLOCK_SECOND/5);
  while((!(timer_expired(&PHY_Delay))) && (!( tempx = (ETH_ReadPHYRegister(PhyAddr, PHY_BSR)&PHY_Linked_Status))));
   

#ifdef STM3210C_EVAL

#if 0  
  uip_ipaddr(ipaddr, 192,168,2,100);
  uip_sethostaddr(ipaddr);
  uip_ipaddr(ipaddr, 192,168,2,9);
  uip_setdraddr(ipaddr);
  uip_ipaddr(ipaddr, 255,255,255,0);
  uip_setnetmask(ipaddr);
#else
  uip_ipaddr(ipaddr, 10,1,1,23);
  uip_sethostaddr(ipaddr);
  uip_ipaddr(ipaddr, 10,1,1,150);
  uip_setdraddr(ipaddr);
  uip_ipaddr(ipaddr, 255,255,255,0);
  uip_setnetmask(ipaddr);
#endif
  
#else  
  uip_ipaddr(ipaddr, 10,133,0,100);
 // printf("IP Address: 10.133.0.100\n\r");
  
  uip_sethostaddr(ipaddr);
  uip_ipaddr(ipaddr, 10,133,0,200);
  uip_setdraddr(ipaddr);
  uip_ipaddr(ipaddr, 255,0,0,0);
  uip_setnetmask(ipaddr);
#endif  
  
  InitPorts();   // timekeeper UDP ports
  
  return 1;
}



/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
//MsgHeader* msgHeader;
void OnEtherMsg() {
  memcpy(uip_buf, ether_rx_buffer, ether_rx_packet_length);          
  uip_len = ether_rx_packet_length;  
    if(BUF->type == htons(UIP_ETHTYPE_IP)) {
        uip_arp_ipin();
        uip_process(UIP_DATA);  // more confused using this: uip_input();
          /* If the above function invocation resulted in data that
             should be sent out on the network, the global variable
             uip_len is set to a value > 0. */
        if(uip_len > 0) {
            uip_arp_out();
            SendToEthDriver(uip_buf, uip_len);  
        }
    }
    else if(BUF->type == htons(UIP_ETHTYPE_ARP)) {
        uip_arp_arpin();
      /* If the above function invocation resulted in data that
         should be sent out on the network, the global variable
         uip_len is set to a value > 0. */
        if(uip_len > 0) {
          // This flags is reset when packet is sent in interrupt handler
          SendToEthDriver(uip_buf, uip_len);  
        }
    }
}


uint32_t tapdev_read(void * pPacket) {
    return 0;
}

//======================================== Interrupt based Ethernet interface =============================================================

#define  ETH_DMARxDesc_FrameLengthShift           16
#define  ETH_ERROR              ((u32)0)
#define  ETH_SUCCESS            ((u32)1)

#define ETH_RXBUFNB        4
#define ETH_TXBUFNB        2

uint8_t MACaddr[6];
ETH_DMADESCTypeDef  DMARxDscrTab[ETH_RXBUFNB], DMATxDscrTab[ETH_TXBUFNB];/* Ethernet Rx & Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RXBUFNB][ETH_MAX_PACKET_SIZE], Tx_Buff[ETH_TXBUFNB][ETH_MAX_PACKET_SIZE];/* Ethernet buffers */

ETH_DMADESCTypeDef  *DMATxDesc = DMATxDscrTab;
extern ETH_DMADESCTypeDef  *DMATxDescToSet;
extern ETH_DMADESCTypeDef  *DMARxDescToGet;

typedef struct{
u32 length;
u32 buffer;
ETH_DMADESCTypeDef *descriptor;
}FrameTypeDef;

FrameTypeDef ETH_RxPkt_ChainMode(void);
u32 ETH_GetCurrentTxBuffer(void);
u32 ETH_TxPkt_ChainMode(u16 FrameLength);

void tapdev_init(void) {
    /* Initialize Tx Descriptors list: Chain Mode */
    ETH_DMATxDescChainInit(DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);
    /* Initialize Rx Descriptors list: Chain Mode  */
    ETH_DMARxDescChainInit(DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);  
  
    /* Enable Ethernet Rx interrrupt */
    { 
        int i;
        for(i=0; i<ETH_RXBUFNB; i++)
        {
            ETH_DMARxDescReceiveITConfig(&DMARxDscrTab[i], ENABLE);
        }
    }
    
    /* Enable Ethernet Tx interrrupt */
    { 
        int i;
        for(i=0; i<ETH_TXBUFNB; i++)
        {
            ETH_DMATxDescTransmitITConfig(&DMATxDscrTab[i], ENABLE);
        }
    }    
    
//#define CHECKSUM_BY_HARDWARE      // Don't do this. It changes checksum for ICMP too
#ifdef CHECKSUM_BY_HARDWARE
    /* Enable the checksum insertion for the Tx frames */
    { 
        int i;
        for(i=0; i<ETH_TXBUFNB; i++)
        {
            ETH_DMATxDescChecksumInsertionConfig(&DMATxDscrTab[i], ETH_DMATxDesc_ChecksumTCPUDPICMPFull);
        }
    }
#endif  
    
   // flagEtherTxDone = CoCreateFlag(1, 1); // auto-reset, flag set. Emitted by Ethernet TX interrupt  

    ETH_Start();    
}

typedef struct staticPacket {
  uint8_t buf[2048+4]; // 4 for message header
}tPacket;


tPacket packet_pool[3] = {{DO_NOT_FREE}, {DO_NOT_FREE}, {DO_NOT_FREE}};
/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
void* ethertnet_read() {
    u16_t len;
    FrameTypeDef frame;
    u8 *buffer;
    void *msg = NULL;
    
    for (int i = 0; i < sizeof(packet_pool)/sizeof(tPacket); i++) {
      
      MsgHeader* msgHeader = GetMsgHeader(&packet_pool[i]);
      if (!(IS_BUSY(msgHeader->msgType))) {
        SET_FLAG_BUSY(msgHeader->msgType);
        msg = &packet_pool[i];
        break;
      }
    }
    frame = ETH_RxPkt_ChainMode();
    /* Obtain the size of the packet and put it into the "len"
     variable. */
    len = frame.length;
    
    assert(len > 0);

    if (msg != NULL) {
        void* body = GetMsgBody(msg);           // skip the header
        MsgHeader* msgHeader = GetMsgHeader(msg);      // the header

        SET_RX_FRAME(msgHeader->msgType);
        msgHeader->msgLength = len;

        buffer = (u8 *)frame.buffer;

        memcpy((u8_t*)body, (u8_t*)&buffer[0], len);
    }
    /* Set Own bit of the Rx descriptor Status: gives the buffer back to ETHERNET DMA */
    frame.descriptor->Status = ETH_DMARxDesc_OWN; 

    /* When Rx Buffer unavailable flag is set: clear it and resume reception */
    if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET)  
    {
        /* Clear RBUS ETHERNET DMA flag */
        ETH->DMASR = ETH_DMASR_RBUS;
        /* Resume DMA reception */
        ETH->DMARPDR = 0;
    }
        
    return msg;
}

/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
void RX_Pkt_Handler(void) {
    
  FrameTypeDef frame = ETH_RxPkt_ChainMode();
  // drop packets if buffer is full
  if (!(flags & FLAG_ETHER_RX_PACKET)) {
    memcpy(ether_rx_buffer, (const uint8_t *) frame.buffer, frame.length);     // TODO!!! remove uip_buf 
    ether_rx_packet_length = frame.length;
    flags |= FLAG_ETHER_RX_PACKET;
  }
  
  /* Set Own bit of the Rx descriptor Status: gives the buffer back to ETHERNET DMA */
  frame.descriptor->Status = ETH_DMARxDesc_OWN; 

  /* When Rx Buffer unavailable flag is set: clear it and resume reception */
  if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET)  
  {
    /* Clear RBUS ETHERNET DMA flag */
    ETH->DMASR = ETH_DMASR_RBUS;
    /* Resume DMA reception */
    ETH->DMARPDR = 0;
  }

}

// push into a transmit queue so that caller is not blocked
void tapdev_send(void *pPacket, uint32_t size) {   
#if 0    
  // push into message queue of EtherTx
    void* msg = MallocMsg(size);
    if (msg == NULL) {
        // assert(msg != NULL);     happens
        return;
    }
    void* body = GetMsgBody(msg);                   // skip the header
    MsgHeader* msgHeader = GetMsgHeader(msg);      // the header
    //To clear DO_NOT_DELETE flag
    msgHeader->msgType = 0;
    SET_TX_FRAME(msgHeader->msgType);
    msgHeader->msgLength = size;  
    memcpy((u8_t*)body, (u8_t*)pPacket, size);    
    
    StatusType status = CoPostQueueMail(ethTxMsgQueue, msg);
    
    if (status == E_QUEUE_FULL) {
        CoKfree(msg);
    }
    else {
        assert(status == E_OK);     // debug if it is full: Found wireless ready but ethenet not
        // TODO!!! wakeup EtherTask if it is sleeping (Maybe not useful: think about 2 or more msgs are waiting: one wakeup is not enough        
    }
#endif
    
}

void SendToEthDriver(void *pPacket, uint32_t size) { 
    while (flags & FLAG_ETHER_TX_PACKET);
    flags |= FLAG_ETHER_TX_PACKET;
    u8 *buffer =  (u8 *)ETH_GetCurrentTxBuffer();  
    memcpy((u8_t*)&buffer[0], pPacket, size);
    ETH_TxPkt_ChainMode(size);
    return;  
}

/*******************************************************************************
* Function Name  : ETH_RxPkt_ChainMode
* Description    : Receives a packet.
* Input          : None
* Output         : None
* Return         : frame: farme size and location
*******************************************************************************/
FrameTypeDef ETH_RxPkt_ChainMode(void)
{ 
  u32 framelength = 0;
  FrameTypeDef frame = {0,0}; 

  /* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
  if((DMARxDescToGet->Status & ETH_DMARxDesc_OWN) != (u32)RESET)
  {	
	frame.length = ETH_ERROR;

    if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET)  
    {
      /* Clear RBUS ETHERNET DMA flag */
      ETH->DMASR = ETH_DMASR_RBUS;
      /* Resume DMA reception */
      ETH->DMARPDR = 0;
    }

	/* Return error: OWN bit set */
    return frame; 
  }
  
  if(((DMARxDescToGet->Status & ETH_DMARxDesc_ES) == (u32)RESET) && 
     ((DMARxDescToGet->Status & ETH_DMARxDesc_LS) != (u32)RESET) &&  
     ((DMARxDescToGet->Status & ETH_DMARxDesc_FS) != (u32)RESET))  
  {      
    /* Get the Frame Length of the received packet: substruct 4 bytes of the CRC */
    framelength = ((DMARxDescToGet->Status & ETH_DMARxDesc_FL) >> ETH_DMARxDesc_FrameLengthShift) - 4;
	
	/* Get the addrees of the actual buffer */
	frame.buffer = DMARxDescToGet->Buffer1Addr;	
  }
  else
  {
    /* Return ERROR */
    framelength = ETH_ERROR;
  }

  frame.length = framelength;


  frame.descriptor = DMARxDescToGet;
  
  /* Update the ETHERNET DMA global Rx descriptor with next Rx decriptor */      
  /* Chained Mode */    
  /* Selects the next DMA Rx descriptor list for next buffer to read */ 
  DMARxDescToGet = (ETH_DMADESCTypeDef*) (DMARxDescToGet->Buffer2NextDescAddr);    
  
  /* Return Frame */
  return (frame);  
}

/*******************************************************************************
* Function Name  : ETH_TxPkt_ChainMode
* Description    : Transmits a packet, from application buffer, pointed by ppkt.
* Input          : - FrameLength: Tx Packet size.
* Output         : None
* Return         : ETH_ERROR: in case of Tx desc owned by DMA
*                  ETH_SUCCESS: for correct transmission
*******************************************************************************/
u32 ETH_TxPkt_ChainMode(u16 FrameLength)
{   
  /* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
  if((DMATxDescToSet->Status & ETH_DMATxDesc_OWN) != (u32)RESET)
  {  
	/* Return ERROR: OWN bit set */
    return ETH_ERROR;
  }
        
  /* Setting the Frame Length: bits[12:0] */
  DMATxDescToSet->ControlBufferSize = (FrameLength & ETH_DMATxDesc_TBS1);

  /* Setting the last segment and first segment bits (in this case a frame is transmitted in one descriptor) */    
  DMATxDescToSet->Status |= ETH_DMATxDesc_LS | ETH_DMATxDesc_FS;

  /* Set Own bit of the Tx descriptor Status: gives the buffer back to ETHERNET DMA */
  DMATxDescToSet->Status |= ETH_DMATxDesc_OWN;

  /* When Tx Buffer unavailable flag is set: clear it and resume transmission */
  if ((ETH->DMASR & ETH_DMASR_TBUS) != (u32)RESET)
  {
    /* Clear TBUS ETHERNET DMA flag */
    ETH->DMASR = ETH_DMASR_TBUS;
    /* Resume DMA transmission*/
    ETH->DMATPDR = 0;
  }
  
  /* Update the ETHERNET DMA global Tx descriptor with next Tx decriptor */  
  /* Chained Mode */
  /* Selects the next DMA Tx descriptor list for next buffer to send */ 
  DMATxDescToSet = (ETH_DMADESCTypeDef*) (DMATxDescToSet->Buffer2NextDescAddr);    

  /* Return SUCCESS */
  return ETH_SUCCESS;   
}

/*******************************************************************************
* Function Name  : ETH_GetCurrentTxBuffer
* Description    : Return the address of the buffer pointed by the current descritor.
* Input          : None
* Output         : None
* Return         : Buffer address
*******************************************************************************/
u32 ETH_GetCurrentTxBuffer(void)
{ 
  /* Return Buffer address */
  return (DMATxDescToSet->Buffer1Addr);   
}
