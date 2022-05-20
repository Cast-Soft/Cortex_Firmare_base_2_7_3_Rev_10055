/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
* File Name          : eink_comms.c
* Author             : ?
* Version            : V1.0
* Date               : 5/17/2019
* Description        : All SPI routines related to EINK screen
*******************************************************************************/

/* INCLUDES ------------------------------------------------------------------*/

#include "hardware.h"
#include "eink_comms.h"
#include "eink_defs.h"
//#include "CoOS.h"
#include "basic_rf.h"
#include "eink.h"
/* PRIVATE TYPEDEF -----------------------------------------------------------*/

/* PRIVATE DEFINES -----------------------------------------------------------*/

/* PRIVATE MACROS ------------------------------------------------------------*/

/* EXTERN VARIABLES ----------------------------------------------------------*/

/* PRIVATE VARIABLES ---------------------------------------------------------*/

// Recommended register settings which differ from the data sheet

/* PUBLIC VARIABLES ----------------------------------------------------------*/

/* EXTERNAL FUNCTION PROTOTYPES ---------------------------------------------*/

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/

static uint8_t          EINK_SPI_RX(void);
static void             EINK_SPI_TX(uint8_t x);
static uint8_t          EINK_SPI_TXRX(uint8_t x);
void EINK_SPI_WAIT_RXRDY(void);

/* PRIVATE FUNCTIONS ---------------------------------------------------------*/

/*******************************************************************************
* Description :
* Input       : -
* Return      :
*******************************************************************************/
static uint8_t EINK_SPI_RX(void) {
	return (SPI_I2S_ReceiveData(SPI_INK_SPI));
}

/*******************************************************************************
* Description :
* Input       :
* Return      : -
*******************************************************************************/
static void EINK_SPI_TX(uint8_t x) {
	SPI_I2S_SendData(SPI_INK_SPI, x);
}

/*******************************************************************************
* Description :
* Input       :
* Return      :
*******************************************************************************/
static uint8_t EINK_SPI_TXRX(uint8_t x) {
    EINK_SPI_TX(x);
    EINK_SPI_WAIT_RXRDY();
    return EINK_SPI_RX();
}

/*******************************************************************************
* Description :
* Input       : -
* Return      : -
*******************************************************************************/
void EINK_SPI_WAIT_RXRDY(void) {
	while (RESET == SPI_I2S_GetFlagStatus(SPI_INK_SPI, SPI_I2S_FLAG_RXNE));
}

/* PUBLIC FUNCTIONS ----------------------------------------------------------*/
/*******************************************************************************
* Description : Read one register byte
* Input       :
* Return      :
*******************************************************************************/
uint8_t EINK_RegisterRead(uint8_t addr) {
    assert(!(addr & ~0x3F));
    addr = addr |0x80;
    HwSPISSAssert(SPI_INK);
    EINK_SPI_TXRX(addr);
    uint8_t value = EINK_SPI_TXRX(0x00);
    HwSPISSDeAssert(SPI_INK);
    return value;
}

void EINK_RegisterReadBytes(uint8_t addr, uint8_t * buf, uint8_t cnt){
	uint8_t i=0;
	assert(!(addr & ~0x3F));
	addr = addr | 0x80;
	HwSPISSAssert(SPI_INK);
	EINK_SPI_TXRX(addr);
	for(i=0; cnt-- != 0; i++){
		buf[i] = EINK_SPI_TXRX(0x00);
	}
	HwSPISSDeAssert(SPI_INK);
	return;
}

uint8_t EINK_MTP_RAM_Read(uint8_t addr){
    assert(!(addr & ~0x3F));
    HwSPISSAssert(SPI_INK);
    EINK_SPI_TXRX(0x80 | addr);
    EINK_SPI_TXRX(0x00);
    uint8_t value = EINK_SPI_TXRX(0x00);
    HwSPISSDeAssert(SPI_INK);
    return value;
}

void EINK_RegisterWrite(uint8_t reg, uint8_t val1, uint8_t val2, uint8_t val3, uint8_t val4, uint8_t count){
	uint8_t x=0;
    HwSPISSAssert(SPI_INK);
    EINK_SPI_TXRX(reg);
    while(count--){
        switch(x){
            case 0:
                EINK_SPI_TXRX(val1);
            break;
            case 1:
                EINK_SPI_TXRX(val2);
            break;
            case 2:
                EINK_SPI_TXRX(val3);
            break;
            case 3:
                EINK_SPI_TXRX(val4);
            break;
        }
        x++;
    }
    HwSPISSDeAssert(SPI_INK);
}

void EINK_write2ram(){
	int8_t x=0;
    HwSPISSAssert(SPI_INK);
    EINK_SPI_TXRX(0x10);
    for(int i=0; i< EPD_BUFFERSIZE; i++){
    	EINK_SPI_TXRX(spiInk_ImgBuf.buffer[i]);
    }
    HwSPISSDeAssert(SPI_INK);
}

void EINK_readram(){
	int i;
	HwSPISSAssert(SPI_INK);
	EINK_SPI_TXRX(0x11|0x80);
	EINK_SPI_TXRX(0x00);
	for(i=0; i< EPD_BUFFERSIZE;i++){
		spiInk_ImgBuf.buffer[i]=EINK_SPI_TXRX(0x00);
	}
	HwSPISSDeAssert(SPI_INK);
}
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
