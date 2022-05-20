  /*************************************************************************
 *
 *    Used with ARM IAR C/C++ Compiler
 *
 *    (c) Copyright IAR Systems 2009
 *
 *    $Revision: #1 $
 **************************************************************************/
#include "ethernet.h"
#include "dhcpc.h"
#include <stdio.h>
#include <string.h>
#include "basic_rf.h"
#include "tst_dbg.h"
#include "beacon.h"
#include "hardware.h"
#include "main.h"

#ifdef Ether_active
#define BUF ((struct uip_eth_hdr *)&uip_buf[0])

StatusType       acceptEtherTxDone;
StatusType       postEthernetRcvd;

OS_FlagID flagEtherTxDone = 0xFF;        // emitted by Ethernet TX interrupt
extern struct uip_udp_conn *uni_conn1, *Mcst_conn1;

extern void *uip_sappdata;
uint16_t tempx;
struct timer PHY_Delay;

struct uip_eth_addr mac_addr;
int st_len;
struct timer periodic_timer, arp_timer;

extern unsigned int PhyAddr;
extern struct Ether_status *pEther_status;

#pragma data_alignment=4
uint8_t RxBuff[EMAC_MAX_PACKET_SIZE];
#pragma data_alignment=4
uint8_t TxBuff[EMAC_MAX_PACKET_SIZE];

void SendToEthDriver(void *pPacket, uint32_t size);

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
  TRACE("uIP log message: %s\n", m);
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
/*************************************************************************
 * Function Name: uIPMain
 * Parameters: none
 *
 * Return: uint32_t
 *
 * Description: Inits all timers and Tape Dev
 *
 *************************************************************************/
uint32_t uIP_Setup(uint8_t m_addr[], uint32_t my_ip, uint32_t dest_ip,
                   uint32_t netmask, uint16_t my_port, uint16_t dest_port)
{

  // Initialize the ethernet device driver
  // Init MAC
  // Phy network negotiation
  tapdev_init();

 //*****************************************


  for (int i = 0; i < 6; i++) {
    mac_addr.addr[i] = m_addr[i];
  }

  ETH_MACAddressConfig(ETH_MAC_Address0, mac_addr.addr);

  ETH_GetMACAddress(ETH_MAC_Address0, mac_addr.addr);


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

  //    Lets wait for 1 sec To see if we can link.....
  // Do I need it? Comment out?
  timer_set(&PHY_Delay, 200);
  while((!(timer_expired(&PHY_Delay))) && (!( tempx = (ETH_ReadPHYRegister(PhyAddr, PHY_BSR)&PHY_Linked_Status))));


  Static_Setup(my_ip, dest_ip, netmask);

  Init_TK_ports(my_port, dest_port);   // timekeeper UDP ports


  return 1;
}

uip_ipaddr_t ipaddr;

void Static_Setup(uint32_t my_ip, uint32_t dest_ip, uint32_t netmask){

  uip_ipaddr(ipaddr, (my_ip >> 24), (my_ip >> 16) & 0xFF, (my_ip >> 8) & 0xFF, my_ip & 0xFF);
  uip_sethostaddr(ipaddr);
  uip_ipaddr(ipaddr, (dest_ip >> 24), (dest_ip >> 16) & 0xFF, (dest_ip >> 8) & 0xFF, dest_ip & 0xFF);
  uip_setdraddr(ipaddr);
  uip_ipaddr(ipaddr, (netmask >> 24), (netmask >> 16) & 0xFF, (netmask >> 8) & 0xFF, netmask & 0xFF);
  uip_setnetmask(ipaddr);

  /*
  uip_ipaddr(ipaddr, 10,133,1,100);
  uip_sethostaddr(ipaddr);
  uip_ipaddr(ipaddr, 10,133,1,200);
  uip_setdraddr(ipaddr);
  uip_ipaddr(ipaddr, 255,0,0,0);
  uip_setnetmask(ipaddr);
  */
}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
MsgHeader* msgHeader;
void MsgLoop(StatusType result, void *msg) {
    if (result == E_OK) {
        msgHeader = GetMsgHeader(msg);                   // the header
        SAVE_POINT
        if (IS_WAKEUP(msgHeader->msgType)) {
            // Do nothing, it is a pre-defined waitup message. Read only, never destroy
          SAVE_POINT
        }
        else if (IS_RX_FRAME(msgHeader->msgType)) {          // Incoming ethernet frame
            SAVE_POINT
            assert(msgHeader->msgLength <= sizeof(uip_buf));
            assert(msgHeader->msgLength > 0);
            memcpy(uip_buf, GetMsgBody(msg), msgHeader->msgLength);     // TODO!!! remove uip_buf
            uip_len = msgHeader->msgLength;

            if (IS_ON_HEAP(msgHeader->msgType)) {
              CoKfree(msg);       // ASAP
            } else {
              CLEAR_FLAG_BUSY(msgHeader->msgType);
            }
            SAVE_POINT
            if(BUF->type == htons(UIP_ETHTYPE_IP)) {
                SAVE_POINT
                uip_arp_ipin();
                uip_process(UIP_DATA);  // more confused using this: uip_input();
                  /* If the above function invocation resulted in data that
                     should be sent out on the network, the global variable
                     uip_len is set to a value > 0. */
                SAVE_POINT
                if(uip_len > 0) {
                    uip_arp_out();
                    tapdev_send(uip_buf,uip_len);
                }
            }
            else if(BUF->type == htons(UIP_ETHTYPE_ARP)) {
                SAVE_POINT
                uip_arp_arpin();
              /* If the above function invocation resulted in data that
                 should be sent out on the network, the global variable
                 uip_len is set to a value > 0. */
                SAVE_POINT
                if(uip_len > 0) {
                    tapdev_send(uip_buf,uip_len);
                }
            }
        }
        else if (IS_TX_PACKET(msgHeader->msgType)) {     // outgoing Ethernet frame
            // TODO!!! remove the buffer
            SAVE_POINT
            assert(msgHeader->msgLength <= sizeof(uip_buf));
            assert(msgHeader->msgLength > 0);
            uip_sappdata=(&uip_buf[UIP_IPUDPH_LEN + UIP_LLH_LEN]) ; // get pointer to correct place in out buffer
            uip_send(GetMsgBody(msg), msgHeader->msgLength);        // copy data to uip buffer
            SAVE_POINT
            if (IS_ON_HEAP(msgHeader->msgType)) {
              CoKfree(msg);       // ASAP
            } else {
              CLEAR_FLAG_BUSY(msgHeader->msgType);
            }


            uip_udp_conn = Mcst_conn1; // load connection data..dest addr:port our addr:port
            uip_process(UIP_UDP_SEND_CONN);
            SAVE_POINT
            if(uip_len > 0){     // if we have a length we have data to send
                uip_arp_out();     // use arp to fill in Ether addr
                tapdev_send(uip_buf,uip_len); // off to the Mac/Phy?wire
            }
        }
        else {
           if (IS_ON_HEAP(msgHeader->msgType)) {
              CoKfree(msg);       // ASAP
            } else {
              CLEAR_FLAG_BUSY(msgHeader->msgType);
            }
            SAVE_POINT
            assert(0);
        }
    }

    // Check outgoing frames
    {
      SAVE_POINT
        if ((acceptEtherTxDone = CoAcceptSingleFlag(flagEtherTxDone)) == E_OK) {
            // TODO!!! assert DMA ready
            // process TX queue
            StatusType result;
            SAVE_POINT
            void *txMsg = CoAcceptQueueMail(ethTxMsgQueue, &result);
            assert(result == E_QUEUE_EMPTY || result == E_OK);
            if (result == E_QUEUE_EMPTY) {
                diagnostic.setEtherTxDone = CoSetFlag(flagEtherTxDone);
                SAVE_POINT
            }
            else {
                SAVE_POINT
                msgHeader = GetMsgHeader(txMsg);                        // the header
                assert(IS_TX_FRAME(msgHeader->msgType));    // TODO!!! process more types
                assert(msgHeader->msgLength <= sizeof(uip_buf));
                assert(msgHeader->msgLength > 0);
                SendToEthDriver(GetMsgBody(txMsg), msgHeader->msgLength);
                SAVE_POINT
                if (IS_ON_HEAP(msgHeader->msgType)) {
                  CoKfree(txMsg);       // ASAP
                } else {
                  CLEAR_FLAG_BUSY(msgHeader->msgType);
                }

            }
        }
    }

    if (result == E_TIMEOUT) {      // no message to process. Does some stack's chores
        SAVE_POINT
        if(timer_expired(&periodic_timer)) {
            uint32_t i;
            timer_reset(&periodic_timer);
            for(i = 0; i < UIP_CONNS; i++)
            {
                uip_periodic(i);
                /* If the above function invocation resulted in data that
                   should be sent out on the network, the global variable
                   uip_len is set to a value > 0. */
                if(uip_len > 0)
                {
                  uip_arp_out();
                  tapdev_send(uip_buf,uip_len);
                }
            }
            SAVE_POINT
        #if UIP_UDP
            for(i = 0; i < UIP_UDP_CONNS; i++) {
                uip_udp_periodic(i);
                /* If the above function invocation resulted in data that
                   should be sent out on the network, the global variable
                   uip_len is set to a value > 0. */
                if(uip_len > 0) {
                  uip_arp_out();
                //          tapdev_send();   /* this is the way I found it..Fixed to reflect above */
                  tapdev_send(uip_buf,uip_len);
                }
            }
        #endif /* UIP_UDP */
            /* Call the ARP timer function every 10 seconds. */
            SAVE_POINT
            if(timer_expired(&arp_timer))
            {
                timer_reset(&arp_timer);
                uip_arp_timer();
            }
        }
    }
}

void *msg;

StatusType result;      // save stack space
void uIP_Loop(void) {
    while (1) {     // TODO!!! a msg to end the loop
        msg = CoPendQueueMail(ethTaskMsgQueue, 10, &result);         // timeout 10ms
        SAVE_POINT
        MsgLoop(result, msg);
        task_ticks[taskEtherId - 1] = 0;
        diagnostic.taskEtherCycles++;
        SAVE_POINT
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

    flagEtherTxDone = CoCreateFlag(1, 1); // auto-reset, flag set. Emitted by Ethernet TX interrupt

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

    void* rxMsg = ethertnet_read();

    if (rxMsg == NULL) {
        return;
    }

    // push into message queue of EtherTask
    diagnostic.postEthernetRcvd = isr_PostQueueMailWithWaterMark(ethTaskMsgQueue, rxMsg);
    if (postEthernetRcvd == E_OK) {
      diagnostic.ethRxRcvdOk++;
    } else {
      diagnostic.ethRxDropped++;
    }

}

// push into a transmit queue so that caller is not blocked
void tapdev_send(void *pPacket, uint32_t size) {
    // push into message queue of EtherTx
    void* msg = MallocMsg(size);
    if (msg == NULL) {
        // assert(msg != NULL);     happens
        return;
    }
    void* body = GetMsgBody(msg);                   // skip the header
    MsgHeader* msgHeader = GetMsgHeader(msg);      // the header
    // Set to to allow freeing memory
    msgHeader->msgType = 0;
    SET_TX_FRAME(msgHeader->msgType);

    msgHeader->msgLength = size;
    memcpy((u8_t*)body, (u8_t*)pPacket, size);

    StatusType status = CoPostQueueMail(ethTxMsgQueue, msg);

    if (status == E_QUEUE_FULL) {
#if 0
        StatusType result;
        void *txMsg = CoAcceptQueueMail(ethTxMsgQueue, &result);
        CoKfree(txMsg);
        status = CoPostQueueMail(ethTxMsgQueue, msg);
#endif
        CoKfree(msg);
    }
    else {
        assert(status == E_OK);     // debug if it is full: Found wireless ready but ethenet not
        // TODO!!! wakeup EtherTask if it is sleeping (Maybe not useful: think about 2 or more msgs are waiting: one wakeup is not enough
    }
}

void SendToEthDriver(void *pPacket, uint32_t size) {
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
#endif
