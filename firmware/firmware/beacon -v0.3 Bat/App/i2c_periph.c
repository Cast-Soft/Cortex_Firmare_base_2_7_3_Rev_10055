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
#include "i2c_ee.h"
#include "stm32f10x_i2c.h"
#include "bq27441.h"

/* PRIVATE TYPEDEF -----------------------------------------------------------*/
const uint8_t I2C_INIT_MODE=0xFF;                    // Can start an new I2C operation from this state only.
const uint8_t I2C_WRITE_MODE=0x10;                 // Send 7bit IC slave address + write bit
const uint8_t I2C_READ_MODE=0x01;                 // Send 7bit IC slave address + read bit
const uint8_t I2C_UPLOAD_STATE=0x20;                  // Upload only.        
const uint8_t I2C_DOWNLOAD_STATE=0x02;                // Download only.
const uint8_t I2C_STOP_STATE=0x55;                    // sends stop bit to end I2C transmission
/* PRIVATE DEFINES -----------------------------------------------------------*/
#define I2C_LENGTH_MAX 10
/* PRIVATE MACROS ------------------------------------------------------------*/

/* EXTERN VARIABLES ----------------------------------------------------------*/

/* PRIVATE VARIABLES ---------------------------------------------------------*/
static uint8_t          i2c_txBuf[I2C_LENGTH_MAX];
static uint8_t          i2c_rxBuf[I2C_LENGTH_MAX];
static uint8_t          scratch_val;
uint8_t                 I2C_ERR=0;
/* PUBLIC VARIABLES ----------------------------------------------------------*/
volatile uint8_t I2CTxRxByteCount = 0;
volatile uint8_t I2CTxRxByteState = 0xFF;   // RF_SPI_INIT_STATE;

OS_MutexID 		  flagI2CIODone;       // For exclusive accessing I2C line
OS_EventID        semI2CAllow;
OS_FlagID flagI2CMachineDone = 0xFF;

volatile uint16_t txDoneType = 1;        // 0: successful, 1: failed
volatile uint16_t rxDoneType = 1;        // 0: successful, 1: failed
volatile uint16_t rxFIFOError = 0;       // 0: no error, 1: with rror
volatile uint16_t txFIFOError = 0;       // 0: no error, 1: with rror


uint8_t*            pI2CTxBuf;
uint8_t*            pI2CRxBuf;

/* EXTERNAL FUNCTION PROTOTYPES ---------------------------------------------*/

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/

//init battery gauge i2c peripheral
//battery gauge i2c setup statemachine

//init shunt i2c peripheral x3
//shunt i2c setup statemachine

//interrupt i2c start

/* PRIVATE FUNCTIONS ---------------------------------------------------------*/
void init_coosi2c(void){
    flagI2CIODone     = CoCreateMutex(); // since multiple tasks will be using the line.

    flagI2CMachineDone = CoCreateFlag(0, 0); // manual-reset, flag clear
    semI2CAllow        = CoCreateSem(1, 1, EVENT_SORT_TYPE_FIFO);   // since multiple tasks will be using the line.

    init_batmonitor(batt_designCapacity, batt_termVoltage, batt_taperCurrent);
}

/*******************************************************************************
* Description : Tasks access I2C exclusively
* Input       : -
* Return      : -
*******************************************************************************/
void WaitGrabI2C() {
  CoEnterMutexSection(flagI2CIODone);
}

/*******************************************************************************
* Description : Release I2C for other tasks
* Input       : -
* Return      : -
*******************************************************************************/
void ReleaseI2C(void) {
    CoLeaveMutexSection(flagI2CIODone);
}

/*******************************************************************************
* Description : Prepares the i2c tx/rx buffers and initializes the routine.
* Input       : -
* Return      : -
*******************************************************************************/
uint16_t i2cBattRoutine(){
    //1. move the internal pointer to the correct address
    //2. read data of register
    //3. set data into appropriate places
    //4. end.
    i2c_txBuf[0]=addr;
    i2c_txBuf[1]=reg;
    i2cWriteRoutine(battmonitor_address, BQ27441_STATE_OF_CHARGE_LOW, 2);
    i2cReadRoutine(battmonitor_address, &i2c_rxBuf[0],payloadSize);
    return i2c_rxBuf[0] | (i2c_rxBuf[1]<<8);
}

/*******************************************************************************
* Description : Prepares the i2c tx buffer and initializes I2C interrupt routine.
* Input       : -
* Return      : -
*******************************************************************************/
void i2cWriteRoutine(uint8_t addr, uint8_t reg, uint8_t * payload, uint8_t payloadSize){
    uint8_t i=0;

    I2CTxRxByteCount = payloadSize;
    assert(I2CTxRxByteState == I2C_INIT_STATE);
    I2CTxRxByteState = I2C_WRITE_MODE;   
    
    pI2CTxBuf = payload;
    pI2cRxBuf = &scratch_val;
    CoClearFlag(flagI2CMachineDone);
    /* Enable Event and Error Interrupts */
    
    I2C_ITConfig(I2C_EE, I2C_IT_ERR|I2C_IT_EVT, ENABLE);    
    I2C_GenerateSTART(I2C_EE, ENABLE);

    CoWaitForSingleFlag(flagI2CMachineDone, 0);
    //check for errors.
    if(I2C_ERR){
        return NULL;
    }
}

/*******************************************************************************
* Description : Prepares the i2c rx buffer and initializes I2C interrupt routine.
* Input       : -
* Return      : -
*******************************************************************************/
void i2cReadRoutine(uint8_t addr, uint8_t *buffer, uint8_t payloadSize){
    I2CTxRxByteCount = payloadSize;
    I2CTxRxByteState = I2C_READ_MODE;
    
    scratch_val = addr;
    pI2CTxBuf = &scratch_val;
    pI2CRxBuf = buffer;
    CoClearFlag(flagI2CMachineDone);

    I2C_ITConfig(I2C_EE, I2C_IT_ERR|I2C_IT_EVT, ENABLE);    
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    I2C_GenerateSTART(I2Cx, ENABLE);

    CoWaitForSingleFlag(flagI2CMachineDone, 0);
    if(I2C_ERR){
        return NULL;
    }
}
/*******************************************************************************
* Description : [API] initialize the battery monitor BQ27441 on the I2C line
* Input       : -
* Return      : -
*******************************************************************************/
uint8_t init_batmonitor(uint16_t designCapacity_mAh, uint16_t term_Voltage, uint16_t taperCurrent){
    //follow the variable setup and instruction flow chart laid out in sluuap7.pdf
    uint16_t designEnergy_mWh, taperRate, flags, checksumOld, checksumRead;
    uint8_t checksumNew;
    
    designEnergy_mWh = 3.7 * designCapacity_mAh;
    taperRate = designCapacity_mAh / ( 0.1 * taperCurrent_mA );
    // Unseal gauge
    i2c_bat_ctrl_write( BQ27441_CONTROL_UNSEAL );
    i2c_bat_ctrl_write( BQ27441_CONTROL_UNSEAL );

    // Send CFG_UPDATE
    i2c_bat_ctrl_write( BQ27441_CONTROL_SET_CFGUPDATE );

    do{
        i2c_bat_cmd_read( BQ27441_FLAGS_LOW, &flags );
        if( !(flags & 0x0010) )
        {
            HwWait(2);
        //    HAL_Delay( 50 );
        }
    }
    while( !(flags & 0x0010) );

    // Enable Block Data Memory Control
    i2c_bat_cmd_write( BQ27441_BLOCK_DATA_CONTROL, 0x0000 );
    
    // Access State subclass
    i2c_bat_cmd_write( BQ27441_DATA_CLASS, 0x0052 );

    // Write the block offset
    i2c_bat_cmd_write( BQ27441_DATA_BLOCK, 0x0000 );

    // Read block checksum
    i2c_bat_cmd_read( BQ27441_BLOCK_DATA_CHECKSUM, &checksumOld );

    //Read 32-byte block of data
    uint8_t block[32];
    for(uint8_t i = 0; i < 32; i++ ){
        block[i] = 0x00;
    }

    i2c_bat_read_data_block( 0x00, block, 32 );

    // Calculate checksum
    uint8_t checksumCalc = 0x00;

    for(uint8_t i = 0; i < 32; i++ ){
        checksumCalc += block[i];
    }
    checksumCalc = 0xFF - checksumCalc;

    // Update design capacity
    block[10] = (uint8_t)( designCapacity_mAh >> 8 );
    block[11] = (uint8_t)( designCapacity_mAh & 0x00FF );
    // Update design energy
    block[12] = (uint8_t)( designEnergy_mWh >> 8 );
    block[13] = (uint8_t)( designEnergy_mWh & 0x00FF );
    // Update terminate voltage
    block[16] = (uint8_t)( terminateVoltage_mV >> 8 );
    block[17] = (uint8_t)( terminateVoltage_mV & 0x00FF );
    // Update taper rate
    block[27] = (uint8_t)( taperRate >> 8 );
    block[28] = (uint8_t)( taperRate & 0x00FF );

    // Calculate new checksum
    checksumNew = 0x00;
    for(int i = 0; i < 32; i++ ){
        checksumNew += block[i];
    }
    checksumNew = 0xFF - checksumNew;

    // Enable Block Data Memory Control
    i2c_bat_cmd_write( BQ27441_BLOCK_DATA_CONTROL, 0x0000 );

    HwWait(BQ27441_DELAY);
    //HAL_Delay( BQ27441_DELAY );
    
    // Access State subclass
    i2c_bat_cmd_write( BQ27441_DATA_CLASS, 0x0052 );

    // Write the block offset
    i2c_bat_cmd_write( BQ27441_DATA_BLOCK, 0x0000 );

    // Write 32-byte block of updated data
    i2c_bat_cmd_write( 0x00, block, 32 );

    // Write new checksum
    i2c_bat_cmd_write( BQ27441_BLOCK_DATA_CHECKSUM, checksumNew );

    // Access State subclass
    i2c_bat_cmd_write( BQ27441_DATA_CLASS, 0x0052 );

    // Write the block offset
    i2c_bat_cmd_read( BQ27441_DATA_BLOCK, 0x0000 );

    // Read block checksum
    i2c_bat_cmd_read( BQ27441_BLOCK_DATA_CHECKSUM, &checksumRead );

    // Verify
    if( checksumRead != (uint8_t)checksumNew ){
        return false;
    }

    // Enable Block Data Memory Control
    i2c_bat_cmd_write( BQ27441_BLOCK_DATA_CONTROL, 0x0000 );

    HwWait(BQ27441_DELAY);
    //HAL_Delay( BQ27421_DELAY );
    
    // Access Registers subclass
    i2c_bat_cmd_write( BQ27441_DATA_CLASS, 0x0040 );

    // Write the block offset
    i2c_bat_cmd_write( BQ27441_DATA_BLOCK, 0x0000 );

    // Read block checksum
    i2c_bat_cmd_read( BQ27441_BLOCK_DATA_CHECKSUM, &checksumOld );

    // Read 32-byte block of data
    for(uint8_t i = 0; i < 32; i++ ){
        block[i] = 0x00;
    }

    i2c_bat_read_data_block( 0x00, block, 32 );

    // Calculate checksum
    checksumCalc = 0x00;

    for(uint8_t i = 0; i < 32; i++ ){
        checksumCalc += block[i];
    }
    checksumCalc = 0xFF - checksumCalc;

    // Update OpConfig
    block[0] = 0x05;

    // Calculate new checksum
    checksumNew = 0x00;
    for(int i = 0; i < 32; i++ ){
        checksumNew += block[i];
    }
    checksumNew = 0xFF - checksumNew;

    // Enable Block Data Memory Control
    i2c_bat_cmd_write( BQ27441_BLOCK_DATA_CONTROL, 0x0000 );

    HwWait(BQ27441_DELAY);
    //HAL_Delay( BQ27421_DELAY );
    
    // Access Registers subclass
    i2c_bat_cmd_write( BQ27441_DATA_CLASS, 0x0040 );

    // Write the block offset
    i2c_bat_cmd_write( BQ27421_DATA_BLOCK, 0x0000 );

    // Write 32-byte block of updated data
    i2c_bat_write_data_block( 0x00, block, 32 );

    // Write new checksum
    i2c_bat_cmd_write( BQ27441_BLOCK_DATA_CHECKSUM, checksumNew );

    // Access Registers subclass
    i2c_bat_cmd_write( BQ27441_DATA_CLASS, 0x0040 );

    // Write the block offset
    i2c_bat_cmd_write( BQ27441_DATA_BLOCK, 0x0000 );

    // Read block checksum
    i2c_bat_cmd_read( BQ27441_BLOCK_DATA_CHECKSUM, &checksumRead );

    // Verify
    if( checksumRead != (uint8_t)checksumNew )
    {
        return false;
    }

    // Configure BAT_DET
    i2c_bat_cmd_write( BQ27441_CONTROL_BAT_INSERT );

    // Send Soft Reset
    i2c_bat_cmd_write( BQ27441_CONTROL_SOFT_RESET );

    // Poll flags
    do
    {
        i2c_bat_cmd_read( BQ27441_FLAGS_LOW, &flags );
        if( !(flags & 0x0010) )
        {
            HwWait(2);
            //HAL_Delay( 50 );
        }
    }
    while( (flags & 0x0010) );

    // Seal gauge
    i2c_bat_cmd_write( BQ27441_CONTROL_SEALED );

    return true;

}

/*******************************************************************************
* Description : [API] write subcommand to the control register on the battery monitor I2C line
* Input       : -
* Return      : -
*******************************************************************************/
void i2c_bat_ctrl_write(uint16_t subcommand){
	uint8_t subCommandMSB = (subcommand >> 8);
	uint8_t subCommandLSB = (subcommand & 0x00FF);
	uint8_t data[3];
    data[0] = BQ27441_CONTROL_LOW;
    data[1] = subCommandLSB;
    data[2] = subCommandMSB;
	i2c_write(battmonitor_address, &data[0], 3);
    //delay;
}

/*******************************************************************************
* Description : [API] read content of subcommand within control register for battery monitor
* Input       : -
* Return      : -
*******************************************************************************/
void i2c_bat_ctrl_read(uint16_t subcommand, uint16_t * data){
    uint8_t i2c_data[2];
    i2c_write(battmonitor_address, &command, 1);
    i2c_read(battmonitor_address, &data[0], 2);
    *data = (i2c_data[1]<<8) |i2c_data[0];
    //delay;
}

/*******************************************************************************
* Description : [API] write data to a command/register on the I2C battery monitor
* Input       : -
* Return      : -
*******************************************************************************/
void i2c_bat_cmd_write(uint8_t command, uint16_t* data){
    uint8_t i2c_data[3];
    i2c_data[0] = command;
    i2c_data[1] = (*data) & 0x00FF;
    i2c_data[2] = (*data) >> 8;
    i2c_write(battmonitor_address, &data[0],3);
    //delay;  
}
/*******************************************************************************
* Description : [API]read data from a command/register on the I2C battery monitor
* Input       : -
* Return      : -
*******************************************************************************/
void i2c_bat_cmd_read( uint8_t command, uint16_t* data){
   uint8_t i2c_data[2];
   i2c_write(battmonitor_address, &command, 1);  
   i2c_read(battmonitor_address, &i2c_data[0], 2);
   *data = (i2c_data[1]<<8)|i2c_data[0];
}

/*******************************************************************************
* Description : [API]read data from a command/register on the I2C battery monitor
* Input       : -
* Return      : -
*******************************************************************************/
static void i2c_bat_write_data_block(uint8_t offset, uint8_t *data, uint8_t bytes){
    uint8_t i2c_data[2], i;
    for(i=0; i< bytes; i++){
        i2c_data[0]= BQ27441_BLOCK_DATA_START +offset+i;
        i2c_data[1]= data[i];
        i2c_write(battmonitor_address, &i2c_data[0],2);
        //delay
    }

}

/*******************************************************************************
* Description : [API]read data from a command/register on the I2C battery monitor
* Input       : -
* Return      : -
*******************************************************************************/
static void i2c_bat_read_data_block(uint8_t offset, uint8_t *data, uint8_t bytes){
    uint8_t i2c_data = BQ27441_BLOCK_DATA_START + offset;
    i2c_write(battmonitor_address, &i2c_data,1);
    i2c_read(battmonitor_address, data,bytes);
}

/*******************************************************************************
* Description : [API] general purpose write on the I2C line
* Input       : -
* Return      : -
*******************************************************************************/
void i2c_write(uint8_t Periph_Addr, const uint8_t* pBuffer, uint8_t size){
  /* Send START condition */
    if(*pBuffer ==NULL || (size >0 && size < 255))
        return;

    I2C_GenerateSTART(I2C_EE, ENABLE);
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));               /* Test on EV5 and clear it */

    I2C_Send7bitAddress(I2C_EE, Periph_addr, I2C_Direction_Transmitter);           /* Send target address for write */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); /* Test on EV6 and clear it */

    while(size){
        I2C_SendData(I2C_EE, *pBuffer++);                                       /* Send the byte to be written */
        while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));      /* Test on EV8 and clear it */
        size--;
    }  
    /* Send STOP condition */
    I2C_GenerateSTOP(I2C_EE, ENABLE);
}

/*******************************************************************************
* Description : [API] general purpose byte read on the I2C line
* Input       : -
* Return      : -
*******************************************************************************/
void i2c_read(uint8_t Periph_Addr, uint8_t* pBuffer, uint8_t size){
    I2C_GenerateSTART(I2C_EE, ENABLE);                                          //generate start bit
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));                

    I2C_Send7bitAddress(I2C_EE, Periph_Addr, I2C_Direction_Receiver);           /* Send target address for write */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));    
    
    while(size){
        if(size == 1){
            /* Disable Acknowledgement */
            I2C_AcknowledgeConfig(I2C_EE, DISABLE);
            /* Send STOP Condition */
            //I2C_GenerateSTOP(I2C_EE, ENABLE);
        }
        /* Test on EV7 and clear it */
        if(I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_RECEIVED)){      
            *pBuffer = I2C_ReceiveData(I2C_EE);            /* Read a byte from the EEPROM */
            pBuffer++; /* Point to the next location where the byte read will be saved */
            size--;  /* Decrement the read bytes counter */      
        }   
    }
    /* Send STOP condition */
    I2C_GenerateSTOP(I2C_EE, ENABLE);
    //I2C_EE_WaitEepromStandbyState();
    I2C_AcknowledgeConfig(I2C_EE, ENABLE);  /* Enable Acknowledgement to be ready for another reception */
}
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
