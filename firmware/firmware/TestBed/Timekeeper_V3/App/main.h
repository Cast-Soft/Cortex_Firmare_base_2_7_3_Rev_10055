/**********************************************************************/
// main.h
//
//
//
//
/**********************************************************************/
/* DEFINE TO PREVENT RECURSIVE INCLUSION -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H


typedef struct {
    uint16_t    productID;
    uint16_t    serialNum;
    uint16_t    panId;
    uint16_t    mySrcAddr;
    uint8_t     rfChan;
    uint8_t     rfChanMin;
    uint8_t     rfChanMax;
    uint8_t     TxPower;
    uint8_t     TestMode;
    uint8_t     RxPrint;
 
    /* !! ABOVE MUST HAVE EVEN # BYTES!! */
    uint16_t    checksum;   // MUST BE LAST!!
} config_t;

extern config_t config;
extern uint16_t rfBeaconEn ;

#endif