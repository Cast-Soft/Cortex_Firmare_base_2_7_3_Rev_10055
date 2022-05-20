/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
// UDP_Process.H
// Header file for UDP_Process.C
//
//
//
//
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

#ifndef __UDP_PROCESS_H__
#define __UDP_PROCESS_H__



//#define UIP_UDP_APPCALL dhcpc_appcall

#define UIP_UDP_APPCALL udp_process
void udp_process(void);
void  Packet_demux(void);
void Reply_Packet(uint8_t* );
void init_ether_comm(void);
uint32_t IncrementPacketId(void);
uint16_t  GetTKId(void);

#endif