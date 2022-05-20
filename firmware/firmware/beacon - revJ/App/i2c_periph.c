/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
* File Name          : i2c_periph.c
* Author             : ?
* Version            : V1.0
* Date               : 6/2/2011
* Description        : All routines related to I2C with exception of EEPROM
* Future iterations should merge i2c_ee with i2c_periph
*******************************************************************************/

/* INCLUDES ------------------------------------------------------------------*/

#include "hardware.h"
#include "i2c_ee.h"
#include "stm32f10x_i2c.h"

#include "i2c_periph.h"
//#include "bq27441.h"
#include "ina219_defs.h"
#include "CoOS.h"
#include "config.h"
/* PRIVATE TYPEDEF -----------------------------------------------------------*/

/* PRIVATE DEFINES -----------------------------------------------------------*/
#define I2C_LENGTH_MAX 10
const uint8_t shunt1_address        =   0x40<<1;    //0x41<<1;
const uint8_t shunt2_address        =   0x44<<1;
const uint8_t shunt3_address        =   0x41<<1;    //0x40<<1;
const uint8_t battmonitor_address   =   0x55<<1;
const uint8_t pressure_address      =   0x76<<1;
/* PRIVATE MACROS ------------------------------------------------------------*/
#define assert(expr)
/* EXTERN VARIABLES ----------------------------------------------------------*/

/* PRIVATE VARIABLES ---------------------------------------------------------*/
static uint8_t          i2c_txBuf[I2C_LENGTH_MAX];
static uint8_t          i2c_rxBuf[I2C_LENGTH_MAX];
static uint8_t          scratch_val;

/* PUBLIC VARIABLES ----------------------------------------------------------*/
volatile uint8_t    I2CTxRxByteCount = 0;
volatile I2C_State  I2CTxRxByteState = I2C_INIT_MODE;
volatile uint8_t I2C_ERR = 0;

U64 I2COSTime;
OS_MutexID  flagI2CIODone;
OS_FlagID   flagI2CMachineDone = 0xFF;
//OS_EventID  semI2CAllow;

uint8_t*            pI2CTxBuf;
uint8_t*            pI2CRxBuf;
/* EXTERNAL FUNCTION PROTOTYPES ---------------------------------------------*/

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/
void init_ina219(uint8_t shunt_address);

void i2cWriteRoutine(uint8_t * payload, uint8_t payloadSize);
uint8_t i2cReadRoutine(uint8_t addr, uint8_t payloadSize);

#ifdef BQ27441_BAT_MONITOR
void i2c_bat_read_data_block(uint8_t offset, uint8_t *data, uint8_t bytes);
void i2c_bat_ctrl_write(uint16_t subcommand);
void i2c_bat_ctrl_read(uint16_t subcommand, uint16_t * data);
void i2c_bat_cmd_write(uint8_t command, uint16_t* data);
void i2c_bat_cmd_read( uint8_t command, uint16_t* data);
void i2c_bat_write_data_block(uint8_t offset, uint8_t *data, uint8_t bytes);
#endif

/* PRIVATE FUNCTIONS ---------------------------------------------------------*/

/*******************************************************************************
* Description : Setup RTOS dependent variables for I2C.
* Input       : -
* Return      : -
*******************************************************************************/
void init_coosi2c(void){
    flagI2CIODone     = CoCreateMutex(); // since multiple tasks will be using the line.
    flagI2CMachineDone = CoCreateFlag(1, 0); // auto-reset, flag clear
    //semI2CAllow        = CoCreateSem(0, 1, EVENT_SORT_TYPE_FIFO);   // since multiple tasks will be using the line.
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
* Description : Prepares the i2c tx buffer and initializes I2C interrupt routine.
* Input       : -
* Return      : -
*******************************************************************************/
void i2cWriteRoutine(uint8_t * payload, uint8_t payloadSize){
    I2CTxRxByteCount = payloadSize;
    assert(I2CTxRxByteState == I2C_INIT_MODE);
    I2CTxRxByteState = I2C_WRITE_MODE;   
    
    pI2CTxBuf = payload;
    pI2CRxBuf = &scratch_val;
    CoClearFlag(flagI2CMachineDone);
    /* Enable Event and Error Interrupts */
    I2C_ITConfig(I2C_EE, I2C_IT_ERR|I2C_IT_EVT, ENABLE);    
    I2C_GenerateSTART(I2C_EE, ENABLE);
    
    CoWaitForSingleFlag(flagI2CMachineDone, 0);
}

/*******************************************************************************
* Description : Prepares the i2c rx buffer and initializes I2C interrupt routine.
* Input       : -
* Return      : -
*******************************************************************************/
uint8_t i2cReadRoutine(uint8_t addr, uint8_t payloadSize){
    uint8_t count=0;
    StatusType result;
    I2CTxRxByteCount = payloadSize;
    I2CTxRxByteState = I2C_READ_MODE;
    
    pI2CTxBuf = &addr;
    pI2CRxBuf = i2c_rxBuf;
    CoClearFlag(flagI2CMachineDone);
    I2C_ERR=0;
    
    if(I2C_GetFlagStatus(I2C_EE, I2C_FLAG_BUSY)){
        CoTickDelay(1);
        if(++count > 2) return 0; 
    }
    //I2COSTime = CoGetOSTime();   
    I2C_AcknowledgeConfig(I2C_EE, ENABLE);
    I2C_ITConfig(I2C_EE, I2C_IT_EVT|I2C_IT_ERR|I2C_IT_BUF, ENABLE);
    I2C_GenerateSTART(I2C_EE, ENABLE);

//Check for potential errors, if error occurred, make sure to reset the line
    //result = CoAcceptSingleFlag(flagI2CMachineDone);
    while((result=CoAcceptSingleFlag(flagI2CMachineDone))!=E_OK && count < 6){
        count++;
        CoTickDelay(1);
       //result = CoAcceptSingleFlag(flagI2CMachineDone);
    }
    if(result!=E_OK || I2C_ERR==1){
        I2C_ITConfig(I2C_EE, (I2C_IT_EVT|I2C_IT_ERR|I2C_IT_BUF), DISABLE); 
        I2CTxRxByteCount = 0;
        I2CTxRxByteState = I2C_INIT_MODE;
        if(config.flags & FLAG_TRACE_LEDSHUNTSYNC){
            if(count>=6)
                TRACE("I2C Timeout\r\n");
            else{
                TRACE("I2C Subroutine error\r\n");
            }
        }
        //HwI2CReset();
        HwI2CInit();
        return 0;       
    }
    return 1;
    /*
    result = CoWaitForSingleFlag(flagI2CMachineDone, 0);

    if(result == E_TIMEOUT || I2C_ERR==1){
        I2C_ITConfig(I2C_EE, (I2C_IT_EVT|I2C_IT_ERR|I2C_IT_BUF), DISABLE); 
        I2CTxRxByteCount = 0;
        I2CTxRxByteState = I2C_INIT_MODE;
        I2C_GenerateSTOP (I2C_EE, ENABLE);
        I2C_AcknowledgeConfig(I2C_EE, ENABLE);
        while((I2C1->SR2&0x0002) == 0x0002);
        return 0;
    }
    else return 1;*/
}

/*******************************************************************************
* Description : [API] selects the 3 INA219 ICs to initialize 
* Input       : -
* Return      : -
*******************************************************************************/
void init_ledshunt(){
	init_ina219(shunt1_address);
	init_ina219(shunt2_address);
	init_ina219(shunt3_address);
}

/*******************************************************************************
* Description : [API] initialize targetted INA219 on the I2C line
* Input       : -
* Return      : -
*******************************************************************************/
void init_ina219(uint8_t shunt_address){
    uint16_t config;
    uint8_t data[3];
    //Set calibration register with calculated calibration value defined in header file
    data[0] = INA219_CALIBRATION;
    data[1] = ina219_calibration_value >> 8;
    data[2] = ina219_calibration_value & 0xFF;
    i2c_write(shunt_address, data,3);

    // Set Config register to take into account the settings above    
    config = INA219_CONFIG_BVOLTAGERANGE_16V |
             INA219_CONFIG_GAIN_2_80MV |
             INA219_CONFIG_BADCRES_9BIT |
             INA219_CONFIG_SADCRES_9BIT_1S_84US |
             INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
    
    data[0] = INA219_CONFIGURATION;
    data[1] = config >> 8;
    data[2] = config & 0xFF;
    i2c_write(shunt_address, data,3);

    // Set the pointer to current register. Note. can read the shunt voltage register too
    data[0] = INA219_CURRENT;
    i2c_write(shunt_address, data,1);
}

/*******************************************************************************
* Description : [API] Set up  rx and tx buffers used to read the led.
* Input       : -
* Return      : -
*******************************************************************************/
int16_t ledbuffer_set(uint8_t addr, uint8_t * errorcheck){
	*errorcheck = i2cReadRoutine(addr, 2);
	return  (i2c_rxBuf[0]<<8 | i2c_rxBuf[1]);
}

/*******************************************************************************
* Description : [API] initialize the battery monitor BQ27441 on the I2C line
* Input       : -
* Return      : -
*******************************************************************************/
uint8_t I2C_pressureID(){
	const uint8_t chipID_addr = 0x00;
	uint8_t addr = pressure_address;
	i2c_write(addr, &chipID_addr, 1);
    i2c_read(addr, i2c_rxBuf, 1);
    return i2c_rxBuf[0];
}

#ifdef BQ27441_BAT_MONITOR
/*******************************************************************************
* Description : [API] initialize the battery monitor BQ27441 on the I2C line
* Input       : -
* Return      : -
*******************************************************************************/
uint16_t init_batmonitor(uint16_t designCapacity_mAh, uint16_t term_Voltage, uint16_t taperCurrent){
    //follow the variable setup and instruction flow chart laid out in sluuap7.pdf
    uint16_t designEnergy_mWh, taperRate, flags, checksumOld, checksumRead;
    uint8_t checksumNew;
    uint16_t device_ID;
    designEnergy_mWh = 3.7 * designCapacity_mAh;
    taperRate = designCapacity_mAh / ( 0.1 * taperCurrent );
    i2c_bat_ctrl_read( BQ27441_CONTROL_DEVICE_TYPE, &device_ID );
    return device_ID;
#if 0
    if(device_ID != 0x421) return;
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
    block[16] = (uint8_t)( term_Voltage >> 8 );
    block[17] = (uint8_t)( term_Voltage & 0x00FF );
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
    i2c_bat_write_data_block( 0x00, block, 32 );

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
        return 0;
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
    i2c_bat_cmd_write( BQ27441_DATA_BLOCK, 0x0000 );

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
        return 0;
    }

    // Configure BAT_DET
    i2c_bat_ctrl_write( BQ27441_CONTROL_BAT_INSERT );

    // Send Soft Reset
    i2c_bat_ctrl_write( BQ27441_CONTROL_SOFT_RESET );

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
    i2c_bat_ctrl_write( BQ27441_CONTROL_SEALED );
    return 1;
#endif

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
    i2c_txBuf[0]=battmonitor_address;
    i2c_txBuf[1]=BQ27441_STATE_OF_CHARGE_LOW;
    i2cWriteRoutine(i2c_txBuf, 2);
    if(I2C_ERR){
        return 0;
    }    
    i2cReadRoutine(battmonitor_address,2);
    if(I2C_ERR){
        return 0;
    }
    return i2c_rxBuf[0] | (i2c_rxBuf[1]<<8);
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
	i2c_write(battmonitor_address, data, 3);
    //delay;
}

/*******************************************************************************
* Description : [API] read content of subcommand within control register for battery monitor
* Input       : -
* Return      : -
*******************************************************************************/
void i2c_bat_ctrl_read(uint16_t subcommand, uint16_t * data){
	uint8_t subCommandMSB = (subcommand >> 8);
	uint8_t subCommandLSB = (subcommand & 0x00FF);
	uint8_t i2c_data[3];
	i2c_data[0] = BQ27441_CONTROL_LOW;
	i2c_data[1] = subCommandLSB;
	i2c_data[2] = subCommandMSB;

	i2c_write(battmonitor_address, i2c_data, 3);
	i2c_write(battmonitor_address, i2c_data, 1);
    i2c_read(battmonitor_address, i2c_data, 2);
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
    i2c_write(battmonitor_address, &i2c_data[0],3);
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
void i2c_bat_write_data_block(uint8_t offset, uint8_t *data, uint8_t bytes){
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
void i2c_bat_read_data_block(uint8_t offset, uint8_t *data, uint8_t bytes){
    uint8_t i2c_data = BQ27441_BLOCK_DATA_START + offset;
    i2c_write(battmonitor_address, &i2c_data,1);
    i2c_read(battmonitor_address, data,bytes);
}
#endif

/*******************************************************************************
* Description : [API] general purpose write on the I2C line
* Input       : -
* Return      : -
*******************************************************************************/
void i2c_write(uint8_t Periph_Addr, const uint8_t* pBuffer, uint8_t size){
	/* While the bus is busy */
	while(I2C_GetFlagStatus(I2C_EE, I2C_FLAG_BUSY));

	/* Send START condition */
    I2C_GenerateSTART(I2C_EE, ENABLE);
    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));
    /* Send target address for write */
    I2C_Send7bitAddress(I2C_EE, Periph_Addr, I2C_Direction_Transmitter);
    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

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
	while(I2C_GetFlagStatus(I2C_EE, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C_EE, ENABLE);                                          //generate start bit
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));                

    I2C_Send7bitAddress(I2C_EE, Periph_Addr, I2C_Direction_Receiver);           /* Send target address for write */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));    

    I2C_AcknowledgeConfig(I2C_EE, ENABLE);
    while(size--){
        if(!size){
            I2C_AcknowledgeConfig(I2C_EE, DISABLE);
        }
        /* Test on EV7 and clear it */
        while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_RECEIVED));
        *pBuffer++ = I2C_ReceiveData(I2C_EE);            /* Read a byte from the EEPROM */
            //pBuffer++; /* Point to the next location where the byte read will be saved */
            //size--;  /* Decrement the read bytes counter */
    }
	I2C_AcknowledgeConfig(I2C_EE, DISABLE);
    I2C_GenerateSTOP(I2C_EE, ENABLE);
    I2C_AcknowledgeConfig(I2C_EE, ENABLE);  /* Enable Acknowledgement to be ready for another reception */
}

/*******************************************************************************
* Description : [API] general purpose byte read on the I2C line
* Input       : -
* Return      : -
*******************************************************************************/
void i2c_burstread(uint8_t Periph_Addr, uint8_t subaddr, uint8_t* pBuffer, uint8_t size){
	while(I2C_GetFlagStatus(I2C_EE, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C_EE, ENABLE);                                          //generate start bit
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C_EE, Periph_Addr, I2C_Direction_Transmitter);           /* Send target address for write */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    I2C_SendData(I2C_EE, 0);
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTOP(I2C_EE, ENABLE);
	I2C_GenerateSTART(I2C_EE, ENABLE);
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C_EE, Periph_Addr, I2C_Direction_Receiver);
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    I2C_AcknowledgeConfig(I2C_EE, ENABLE);
    while(size--){
        if(!size){
            I2C_AcknowledgeConfig(I2C_EE, DISABLE);
        }
        /* Test on EV7 and clear it */
        while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_RECEIVED));
        *pBuffer++ = I2C_ReceiveData(I2C_EE);            /* Read a byte from the EEPROM */
            //pBuffer++; /* Point to the next location where the byte read will be saved */
            //size--;  /* Decrement the read bytes counter */
    }
	I2C_AcknowledgeConfig(I2C_EE, DISABLE);
    I2C_GenerateSTOP(I2C_EE, ENABLE);
    I2C_AcknowledgeConfig(I2C_EE, ENABLE);  /* Enable Acknowledgement to be ready for another reception */
}
/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
