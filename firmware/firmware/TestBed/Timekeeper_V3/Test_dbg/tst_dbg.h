/* tst_dbg.h            */

#ifndef TST_DBG_H
#define TST_DBG_H

#define TK_Send_Port  23000
#define TK_Listen_Port  23001
#define TK_Rand_Port 20000



/* Includes ------------------------------------------------------------------ */
struct Disc_Packet{     // contents of discovery packet
uint32_t  sync_enab;
uint32_t  sync_val;
uint32_t  radio_en;
};  

void LoadTestString(void);
void  Tx_loop_ctr(uint8_t);
void  test_stuff(void);
void Init_TK_ports(void);
void Packet_demux(void);
void send_IMU_ether(uint8_t*);
void Chek_lnk(void);
void discovery_probe(void);
void Send_Brdcst_packet(uint8_t*, uint16_t);
void GetARM_UUID(void);
//void Test_Parse(basicRfRxInfo_t*);



#endif  // end of TST_DBG_H