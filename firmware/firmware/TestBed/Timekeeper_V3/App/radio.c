/******************** (C) COPYRIGHT 2011 NaturalPoint, Inc. ********************
* File Name          : radio.c
* Author             : ?
* Version            : V1.0
* Date               : 6/2/2011
* Description        : All routines related to CC2520 Radio
*******************************************************************************/

/* INCLUDES ------------------------------------------------------------------*/

#include "hardware.h"
#include "radio.h"
#include "radio_defs.h"

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
static void             TK_BK_SPI_WAIT_RXRDY(void);

/* PRIVATE FUNCTIONS ---------------------------------------------------------*/

/*******************************************************************************
* Description :
* Input       : -
* Return      :
*******************************************************************************/
static uint8_t TK_BK_SPI_RX(void) {
    return (SPI_I2S_ReceiveData(SPI_RADIO_SPI));
}

/*******************************************************************************
* Description :
* Input       :
* Return      : -
*******************************************************************************/
static void TK_BK_SPI_TX(uint8_t x) {
    SPI_I2S_SendData(SPI_RADIO_SPI, x);
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
static void TK_BK_SPI_WAIT_RXRDY(void) {
    while (RESET == SPI_I2S_GetFlagStatus(SPI_RADIO_SPI, SPI_I2S_FLAG_RXNE));
}

/* PUBLIC FUNCTIONS ----------------------------------------------------------*/

/*******************************************************************************
* Description :
* Input       :
* Return      :
*******************************************************************************/
uint8_t Send_SPI_byte(uint16_t s_data) {
    HwSPISSAssert(SPI_RADIO);
    uint8_t radio_status = TK_BK_SPI_TXRX(s_data);
    HwSPISSDeAssert(SPI_RADIO);
    return (radio_status);
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

/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
