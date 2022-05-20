/********************  All title and rights are owned by  *********************
*******  the CAST Group of Companies Inc. or licensors thereto. ***************
*******************************************************************************
* File Name          : eink.c
* Author             : ?
* Version            : V1.0
* Date               : 5/17/2019
* Description        : All SPI routines related to EINK screen
*******************************************************************************/

/* INCLUDES ------------------------------------------------------------------*/
#include "stm32f10x_dma.h"
#include "hardware.h"
#include "eink_comms.h"
#include "eink_defs.h"
#include "eink.h"
#include <string.h>

#include "CoOS.h"

/* PRIVATE TYPEDEF -----------------------------------------------------------*/

/* PRIVATE DEFINES -----------------------------------------------------------*/
#define EINK_ID 0x56
/* PRIVATE MACROS ------------------------------------------------------------*/

/* EXTERN VARIABLES ----------------------------------------------------------*/

/* PRIVATE VARIABLES ---------------------------------------------------------*/

/* PUBLIC VARIABLES ----------------------------------------------------------*/
OS_MutexID  flagEinkIODone;
OS_EventID	semEinkBufferAllow;

volatile EPD_buffer spiInk_ImgBuf;
volatile uint8_t spiInk_RxBuf[5] ={0,0,0,0,0};
volatile uint8_t spiInk_TxBuf[5] ={0,0,0,0,0};
uint8_t EInk_Present=0;
/* EXTERNAL FUNCTION PROTOTYPES ---------------------------------------------*/
extern OS_FlagID flagEInk_RDY;
extern OS_FlagID flagEInk_DMA_Done;
/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/
uint8_t get_display_size(void);
uint8_t read_MTP_address(const uint16_t addr);
uint8_t MTPvalidation(const uint8_t * display_type_string);

void clear();
void invert();

uint16_t powerOn();
void powerOff();

void WriteBuffer();
void drawPixel(int16_t x, int16_t y, uint16_t color);
int getPixel(int x, int y);

void readSerialNo_MTP();
void WFversionNo_MTP();

void EINK_write_DMA_Image(uint8_t reg, uint16_t payload);
void EINK_write_DMA_Image_Single(uint8_t reg, uint16_t payload, uint8_t * value);
void EINK_DMA_Read_Register(uint8_t reg, uint8_t payload);
void EINK_DMA_Write_Register(uint8_t reg, uint8_t payload, uint8_t val1, uint8_t val2, uint8_t val3, uint8_t val4);

/* PRIVATE FUNCTIONS ---------------------------------------------------------*/
/*******************************************************************************
* Description : Sets the buffer for DMA transfer. Writes to the register
* Input       :
* Return      :
*******************************************************************************/
void EINK_write_DMA_Image(uint8_t reg, uint16_t payload){
	CoClearFlag(flagEInk_DMA_Done);
	HwSPISSAssert(SPI_INK);
	DMA_Cmd(SPI_INK_RX_DMA_CHAN, DISABLE);
    DMA_Cmd(SPI_INK_TX_DMA_CHAN, DISABLE);
	spiInk_ImgBuf.regval = reg;
	SPI_INK_RX_DMA_CHAN->CCR &= (uint16_t)(~DMA_CCR4_MINC);		//disable memory increment
	SPI_INK_TX_DMA_CHAN->CCR |= (uint16_t)(DMA_CCR5_MINC);		//enable memory increment
	SPI_INK_TX_DMA_CHAN->CMAR = (uint32_t) &spiInk_ImgBuf.regval;
	SPI_INK_RX_DMA_CHAN->CNDTR = payload;
    SPI_INK_TX_DMA_CHAN->CNDTR = payload;
	DMA_Cmd(SPI_INK_RX_DMA_CHAN, ENABLE);
    DMA_Cmd(SPI_INK_TX_DMA_CHAN, ENABLE);
	CoWaitForSingleFlag(flagEInk_DMA_Done,0);
}

/*******************************************************************************
* Description : Sets the buffer for DMA transfer. Reads the register and values
* Input       :
* Return      :
*******************************************************************************/
void EINK_DMA_Read_Register(uint8_t reg, uint8_t payload){
	CoClearFlag(flagEInk_DMA_Done);
	spiInk_TxBuf[0]=reg|0x80;
	HwSPISSAssert(SPI_INK);
	DMA_Cmd(SPI_INK_RX_DMA_CHAN, DISABLE);
	DMA_Cmd(SPI_INK_TX_DMA_CHAN, DISABLE);
	SPI_INK_RX_DMA_CHAN->CCR |= (uint16_t)(DMA_CCR4_MINC);		//enable memory increment
	SPI_INK_TX_DMA_CHAN->CCR |= (uint16_t)(DMA_CCR5_MINC);		//enable memory increment
	SPI_INK_TX_DMA_CHAN->CMAR = (uint32_t) spiInk_TxBuf;
	SPI_INK_RX_DMA_CHAN->CNDTR = payload+1;
	SPI_INK_TX_DMA_CHAN->CNDTR = payload+1;
    DMA_Cmd(SPI_INK_RX_DMA_CHAN, ENABLE);
    DMA_Cmd(SPI_INK_TX_DMA_CHAN, ENABLE);
	CoWaitForSingleFlag(flagEInk_DMA_Done,0);	
}

/*******************************************************************************
* Description : Sets the buffer for DMA transfer. Reads the register and values
* Input       :
* Return      :
*******************************************************************************/
void EINK_DMA_Write_Register(uint8_t reg, uint8_t val1, uint8_t val2, uint8_t val3, uint8_t val4, uint8_t payload){
	CoClearFlag(flagEInk_DMA_Done);
	spiInk_TxBuf[0]=reg; spiInk_TxBuf[1] = val1; spiInk_TxBuf[2] = val2; spiInk_TxBuf[3] = val3; spiInk_TxBuf[4]=val4;
	HwSPISSAssert(SPI_INK);
	DMA_Cmd(SPI_INK_RX_DMA_CHAN, DISABLE);
	DMA_Cmd(SPI_INK_TX_DMA_CHAN, DISABLE);
	SPI_INK_RX_DMA_CHAN->CCR |= (uint16_t)(DMA_CCR4_MINC);		//enable memory increment
	SPI_INK_TX_DMA_CHAN->CCR |= (uint16_t)(DMA_CCR5_MINC);		//enable memory increment
	SPI_INK_TX_DMA_CHAN->CMAR = (uint32_t) spiInk_TxBuf;
	SPI_INK_RX_DMA_CHAN->CNDTR = payload+1;
	SPI_INK_TX_DMA_CHAN->CNDTR = payload+1;
    DMA_Cmd(SPI_INK_RX_DMA_CHAN, ENABLE);
    DMA_Cmd(SPI_INK_TX_DMA_CHAN, ENABLE);
	CoWaitForSingleFlag(flagEInk_DMA_Done,0);	
}

/*******************************************************************************
* Description : Activates high voltages required to update the screen.
* 				Called prior to triggering a image update.
* Input       :
* Return      :
*******************************************************************************/
uint16_t powerOn(){
	uint16_t count =0;
	EINK_DMA_Read_Register(EINK_POWER_CONTROL_SETTING,1);
	EINK_DMA_Write_Register(EINK_POWER_CONTROL_SETTING, 
							spiInk_RxBuf[1]|0x11, 0, 0, 0, 
							1);
	//EINK_RegisterWrite(	EINK_POWER_CONTROL_SETTING,
	//					EINK_RegisterRead(EINK_POWER_CONTROL_SETTING)|0x11,
	//					0,0,0,1);
	EINK_DMA_Read_Register(EINK_STATUS_REGISTER,1);	
	while(spiInk_RxBuf[1]!=4 && count<1000){
		count++;
		CoTickDelay(1);
		EINK_DMA_Read_Register(EINK_STATUS_REGISTER,1);	
	}
	return count;
}

/*******************************************************************************
* Description : Deactivates high voltages required to update the screen.
* 				Called after triggering a image update.
* Input       :
* Return      :
*******************************************************************************/
void powerOff(){
	uint8_t status;
	EINK_DMA_Read_Register(EINK_POWER_CONTROL_SETTING,1);
	status = spiInk_RxBuf[1];
	status &= ~0x01;
	EINK_DMA_Write_Register(EINK_POWER_CONTROL_SETTING, 
							status, 0,0,0,
							1);
	CoWaitForSingleFlag(flagEInk_RDY,0);

	status &= ~0x10;
	EINK_DMA_Write_Register(EINK_POWER_CONTROL_SETTING, 
							status, 0,0,0,
							1);
}


/*******************************************************************************
* Description : gets display size dimensions.
* Input       :
* Return      :
*******************************************************************************/
uint8_t get_display_size(){
	uint16_t start_addr = 0x4d0;
	uint8_t display_type_string[10];
	uint8_t i;
	uint8_t x;

	EINK_RegisterWrite(EINK_PROGRAM_WS_MTP, 0xF2, 0,0,0,1);			//0x40 switch to type2 MTP area

	for(i=0;i<9;i++)
		display_type_string[i] = read_MTP_address(start_addr+i);
	display_type_string[9]='\0';
	x=MTPvalidation(display_type_string);
	if(x==0){
		start_addr = 0x4f0;
		for(i=0;i<9;i++)
			display_type_string[i] = read_MTP_address(start_addr+i);
		display_type_string[9]='\0';
		x=MTPvalidation(display_type_string);
		if(x==0) return 0;
	}
	EINK_RegisterWrite(EINK_PROGRAM_WS_MTP, 0xF0, 0,0,0,1);		//restore waveform
	return 1;
}

/*******************************************************************************
* Description : Test function that reads from the MTTP address. 
				Called by get_display_size
* Input       :
* Return      :
*******************************************************************************/
uint8_t read_MTP_address(const uint16_t addr){
	EINK_RegisterWrite(EINK_MTP_ADDRESS_SETTING, addr&0xFF, (addr>>8)&0x07,0,0,2);
	return EINK_MTP_RAM_Read(EINK_MTP_READ);
}

/*******************************************************************************
* Description : Test function that checks the size of the plastic logic screen
				Called by get_display_size
* Input       :
* Return      :
*******************************************************************************/
uint8_t MTPvalidation(const uint8_t * display_type_string){
	if(strcmp(display_type_string, "S021_T1.1")==0){
		return 1;
	}
	else return 0;
}

/*******************************************************************************
* Description : image update routine which clears the eink screen
* Input       :
* Return      :
*******************************************************************************/
void whiteerase(){
	clear();
	update(fullrefresh);
}

/*******************************************************************************
* Description : resets display buffer to white
* Input       :
* Return      :
*******************************************************************************/
void clear(){
	memset((uint8_t*)&spiInk_ImgBuf.buffer[0],0xFF,sizeof(spiInk_ImgBuf.buffer));
}

/*******************************************************************************
* Description : routine to update image on eink screen
* Input       :
* Return      :
*******************************************************************************/
void update(EPD_refresh type){
	uint16_t val;
	volatile uint16_t check_count;

	WriteBuffer();
	check_count = powerOn();
	//0xF0|0x00 gives 4 level gray scale. 0xF0 is default setting. Unless changed, leave uncommented
	//EINK_RegisterWrite(EINK_PROGRAM_WS_MTP,0xF0,0,0,0,1);	
	if(type == fullrefresh)	
		EINK_DMA_Write_Register(EINK_DISPLAY_ENGINE_CONTROL_REGISTER, 
								0x03, 0,0,0,
								1);		
	else
		EINK_DMA_Write_Register(EINK_DISPLAY_ENGINE_CONTROL_REGISTER, 
								0x07, 0,0,0,
								1);		

	CoWaitForSingleFlag(flagEInk_RDY,0);

	powerOff();
}

/*******************************************************************************
* Description : Writes to the RAM buffer of the Eink screen for image update.
* Input       :
* Return      :
*******************************************************************************/
void WriteBuffer(){
	uint8_t x;
	EINK_DMA_Read_Register(EINK_DATA_ENTRY_MODE_SETTING,1);
	x= spiInk_RxBuf[1];
	EINK_DMA_Write_Register(EINK_DATA_ENTRY_MODE_SETTING, 
							x&(~0x10), 0, 0, 0, 
							1);
	EINK_write_DMA_Image(EINK_WRITE_RAM, sizeof(spiInk_ImgBuf));
}

/*******************************************************************************
* Description : Test routine to read Serial Number from MTP
* Input       :
* Return      :
*******************************************************************************/
void readSerialNo_MTP(){
	uint16_t start_addr = 0x4d0;
	volatile uint8_t display_serial_no[32];
	uint8_t i;

	//default value for register 0x40: 0xF0
	EINK_RegisterWrite(EINK_PROGRAM_WS_MTP, 0xF2, 0,0,0,1);			//0x40 switch to type2 MTP area
	for(i=0;i<32;i++)
		display_serial_no[i] = read_MTP_address(start_addr+i);
	display_serial_no[31]= '\0';

	EINK_RegisterWrite(EINK_PROGRAM_WS_MTP, 0xF0, 0,0,0,1);		//restore waveform
}

/*******************************************************************************
* Description : Test routine to read Version Number from MTP
* Input       :
* Return      :
*******************************************************************************/
void WFversionNo_MTP(){
	uint16_t start_addr = 0x4d0;
	volatile uint8_t display_type[16];
	uint8_t i;

	//default value for register 0x40: 0xF0
	EINK_RegisterWrite(EINK_PROGRAM_WS_MTP, 0xF2, 0,0,0,1);			//0x40 switch to type2 MTP area
	for(i=0;i<16;i++)
		display_type[i] = read_MTP_address(start_addr+i);
	display_type[15]= '\0';

	EINK_RegisterWrite(EINK_PROGRAM_WS_MTP, 0xF0, 0,0,0,1);		//restore waveform
}

/* PUBLIC FUNCTIONS ----------------------------------------------------------*/
/*******************************************************************************
* Description : Initialize routine for the Eink Display
* Input       :
* Return      :
*******************************************************************************/
uint8_t Eink_Init() {
    while(!HwGPIState(GPI_INK_BUSY));
    HwGPOLow(GPO_INK_RST);
	CoTickDelay(1);
	HwGPOHigh(GPO_INK_RST);
    while(!HwGPIState(GPI_INK_BUSY));
    CoTickDelay(5);

	//confirm that the E-Ink display was attached
	if(EINK_RegisterRead(EINK_REVISION) != EINK_ID){
		return 0;
	}
    
    //register setup
    EINK_RegisterWrite(EINK_PANEL_SETTING, 0x10, 0, 0, 0,1);						//0x01
    EINK_RegisterWrite(EINK_DRIVER_VOLTAGE_SETTING, 0x25, 0xff, 0, 0,2);			//0x02
	EINK_RegisterWrite(EINK_TCOM_TIMING_SETTING,0x67,0x55,0,0,2);					//0x06
    EINK_RegisterWrite(EINK_TEMPERATURE_SENSOR_CONFIGURATION, 0x0A, 0, 0, 0,1);		//0x07
	EINK_RegisterWrite(EINK_PANEL_RESOLUTION_SETTING,0,239,0,159,4);				//0x0C
    EINK_RegisterWrite(EINK_WRITE_PIXEL_RECTANGULAR_SETTING, 0, 239, 0, 145,4);		//0x0D
	EINK_RegisterWrite(EINK_PIXEL_ACCESS_POSITION_SETTING,0,0,0,0,2);				//0x0E
	EINK_RegisterWrite(EINK_DATA_ENTRY_MODE_SETTING,0x24,0,0,0,1); 					//0x0F make 0x24 for portrait mode
    EINK_RegisterWrite(EINK_VCOM_CONFIGURATION_REGISTER, 0x00, 0x00, 0x24, 0x07,4); //0x18

    EINK_RegisterWrite(EINK_VBORDER_SETTING, 0x04, 0, 0, 0,1);						//0x1D
	EINK_RegisterWrite(EINK_POWER_SEQUENCE_SETTING,0,0,0,0,3);						//0x1F
    EINK_RegisterWrite(EINK_UNDOCUMENTED_REG, 0x60, 0, 0, 0,1);						//0x44

    //Testing EINK screen reads
    //EINK_RegisterReadBytes(EINK_VCOM_DC_SETTING, buffertest,1);	//vcom reading -> 3.18v
    //readSerialNo_MTP();
    //WFversionNo_MTP();
	HwEINKEXTIInit();
    return 1;
}

/*******************************************************************************
* Description : Setup RTOS dependent variables for EInk SPI Line.
* Input       : -
* Return      : -
*******************************************************************************/
void init_coos_eink(void){
    flagEinkIODone     = CoCreateMutex(); // since multiple tasks will be using the line.
	semEinkBufferAllow = CoCreateSem(0,1,EVENT_SORT_TYPE_FIFO);
}

/*******************************************************************************
* Description : Tasks access EInk SPI exclusively
* Input       : -
* Return      : -
*******************************************************************************/
void WaitGrabEInk(void) {
  CoEnterMutexSection(flagEinkIODone);
}

/*******************************************************************************
* Description : Release EInk SPI for other tasks
* Input       : -
* Return      : -
*******************************************************************************/
void ReleaseEInk(void) {
    CoLeaveMutexSection(flagEinkIODone);
}

/******************* (C) COPYRIGHT 2011 NaturalPoint, Inc. *****END OF FILE****/
