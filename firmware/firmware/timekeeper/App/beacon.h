/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
******************************************************************************/
//
// beacon.h
//
//
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

#ifndef BEACON_H
#define BEACON_H


void BeaconComInit(void);
void ParseBeaconData(rxPkt_t*);
uint32_t GetPktSyncTime(uint8_t);
uint8_t GetButtonState(uint8_t);
void EtherSendIMU(void);


#endif
