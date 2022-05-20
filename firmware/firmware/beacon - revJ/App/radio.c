/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
* File Name          : imu_radio.c
* Author             : ?
* Version            : V1.0
* Date               : 6/2/2011
* Description        : All routines related to CC2520 Radio
*******************************************************************************/

/* INCLUDES ------------------------------------------------------------------*/

#include "hardware.h"
#include "radio.h"
#include "radio_defs.h"
#include "imu_defs.h"
#include "CoOS.h"

/* PRIVATE TYPEDEF -----------------------------------------------------------*/

/* PRIVATE DEFINES -----------------------------------------------------------*/

/* PRIVATE MACROS ------------------------------------------------------------*/

/* EXTERN VARIABLES ----------------------------------------------------------*/

/* PRIVATE VARIABLES ---------------------------------------------------------*/

// Recommended register settings which differ from the data sheet

/* PUBLIC VARIABLES ----------------------------------------------------------*/

/* EXTERNAL FUNCTION PROTOTYPES ---------------------------------------------*/

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/

static uint8_t          TK_BK_SPI_RX(void);
static void             TK_BK_SPI_TX(uint8_t x);
static uint8_t          TK_BK_SPI_TXRX(uint8_t x);
void TK_BK_SPI_WAIT_RXRDY(void);

/* PRIVATE FUNCTIONS ---------------------------------------------------------*/

/*******************************************************************************
* Description :
* Input       : -
* Return      :
*******************************************************************************/
static uint8_t TK_BK_SPI_RX(void) {
	return (SPI_I2S_ReceiveData(SPI_RADIO_IMU_SPI));
}

/*******************************************************************************
* Description :
* Input       :
* Return      : -
*******************************************************************************/
static void TK_BK_SPI_TX(uint8_t x) {
	SPI_I2S_SendData(SPI_RADIO_IMU_SPI, x);
}

/*******************************************************************************
* Description :
* Input       :
* Return      :
*******************************************************************************/
static uint8_t TK_BK_SPI_TXRX(uint8_t x) {
    TK_BK_SPI_TX(x);
    TK_BK_SPI_WAIT_RXRDY();
    return TK_BK_SPI_RX();
}

/*******************************************************************************
* Description :
* Input       : -
* Return      : -
*******************************************************************************/
void TK_BK_SPI_WAIT_RXRDY(void) {
	while (RESET == SPI_I2S_GetFlagStatus(SPI_RADIO_IMU_SPI, SPI_I2S_FLAG_RXNE));
}

/* PUBLIC FUNCTIONS ----------------------------------------------------------*/

/*******************************************************************************
* Description :
* Input       :
* Return      :
*******************************************************************************/
uint8_t Send_SPI_byte(uint16_t s_data) {
    HwSPISSAssert(SPI_RADIO);
    uint8_t r_status = TK_BK_SPI_TXRX(s_data);
    HwSPISSDeAssert(SPI_RADIO);
    return (r_status);
}

/*******************************************************************************
* Description :
* Input       :
* Return      : -
*******************************************************************************/
void Send_SPI_2byte(uint16_t command, uint16_t s_data) {
    HwSPISSAssert(SPI_RADIO);
    TK_BK_SPI_TXRX(command);
    TK_BK_SPI_TXRX(s_data);
    HwSPISSDeAssert(SPI_RADIO);
}

/*******************************************************************************
* Description : Write memory 8 bits
* Input       :
* Return      : -
*******************************************************************************/
void TK_BK_MEMWR8(uint16_t addr, uint8_t value) {
    HwSPISSAssert(SPI_RADIO);
    TK_BK_SPI_TXRX(CC2520_INS_MEMWR | HI_UINT16(addr));
    TK_BK_SPI_TXRX(LO_UINT16(addr));
    TK_BK_SPI_TXRX(value);
    HwSPISSDeAssert(SPI_RADIO);
}

uint8_t TK_BK_MEMRD8(uint16_t addr) {
    HwSPISSAssert(SPI_RADIO);
    TK_BK_SPI_TXRX(CC2520_INS_MEMRD | HI_UINT16(addr));
    TK_BK_SPI_TXRX(LO_UINT16(addr));
    uint8_t value = TK_BK_SPI_TXRX(0x00);
    HwSPISSDeAssert(SPI_RADIO);
    return value;
}
/*******************************************************************************
* Description : Write memory 16 bits
* Input       :
* Return      : -
*******************************************************************************/
void TK_BK_MEMWR16(uint16_t addr, uint16_t value) {
    HwSPISSAssert(SPI_RADIO);
    TK_BK_SPI_TXRX(CC2520_INS_MEMWR | HI_UINT16(addr));
    TK_BK_SPI_TXRX(LO_UINT16(addr));
    TK_BK_SPI_TXRX(LO_UINT16(value));
    TK_BK_SPI_TXRX(HI_UINT16(value));
    HwSPISSDeAssert(SPI_RADIO);
}

/*******************************************************************************
* Description : Read one register byte
* Input       :
* Return      :
*******************************************************************************/
uint8_t TK_BK_REGRD8(uint8_t addr) {
    assert(!(addr & ~0x3F));
    HwSPISSAssert(SPI_RADIO);
    TK_BK_SPI_TXRX(CC2520_INS_REGRD | addr);
    uint8_t value = TK_BK_SPI_TXRX(0x00);
    HwSPISSDeAssert(SPI_RADIO);
    return value;
}

/*******************************************************************************
* Description : Write one register byte
* Input       :
* Return      : -
*******************************************************************************/
void TK_BK_REGWR8(uint8_t addr, uint8_t value) {
    assert(!(addr & ~0x3F));
    HwSPISSAssert(SPI_RADIO);
    TK_BK_SPI_TXRX(CC2520_INS_REGWR | addr);
    TK_BK_SPI_TXRX(value);
    HwSPISSDeAssert(SPI_RADIO);
    return;
}

uint8_t IMU_ReadOneByte(uint8_t reg){
	reg = reg | 0x80;
	HwSPISSAssert(SPI_IMU);
	TK_BK_SPI_TXRX(reg);
	uint8_t val = TK_BK_SPI_TXRX(0x00);
	HwSPISSDeAssert(SPI_IMU);
        return val;
}

void IMU_WriteOneByte(uint8_t reg, uint8_t Data){
	reg = reg & 0x7F;
	HwSPISSAssert(SPI_IMU);
	TK_BK_SPI_TXRX(reg);
    TK_BK_SPI_TXRX(Data);
	HwSPISSDeAssert(SPI_IMU);	
}

void IMU_SelectBank(uint8_t bank) {
	IMU_WriteOneByte(IMU_REG_BANK_SEL, bank);
}

uint32_t RadioGetRandom() {
    uint32_t ret = 0;
    HwSPISSAssert(SPI_RADIO);
    TK_BK_SPI_TXRX(CC2520_INS_RANDOM);
    for (int i = 0; i < 6; i++) {
      ret <<= 8;
      ret |= TK_BK_SPI_TXRX(0x00);
    }
    HwSPISSDeAssert(SPI_RADIO);
    return ret;
}

/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
