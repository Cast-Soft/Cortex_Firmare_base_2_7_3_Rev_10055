/**
  ******************************************************************************
  * @file    I2C/EEPROM/i2c_ee.c
  * @author  MCD Application Team
  * @version V3.1.2
  * @date    09/28/2009
  * @brief   This file provides a set of functions needed to manage the
  *          communication between I2C peripheral and I2C M24CXX EEPROM.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c_ee.h"
#include "stm32f10x_i2c.h"
/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup I2C_EEPROM
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define I2C_SLAVE_ADDRESS7     0xA0

#if defined (EE_M24C02)
#define I2C_FLASH_PAGESIZE      8
#elif defined (EE_M24C08)
 #define I2C_FLASH_PAGESIZE    16
#elif defined (EE_M24C64_32)
 #define I2C_FLASH_PAGESIZE    32
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint16_t EEPROM_ADDRESS;

/* Private function prototypes -----------------------------------------------*/
//void GPIO_Configuration(void);
//void I2C_Configuration(void);

/* Private functions ---------------------------------------------------------*/



/**
  * @brief  Writes one byte to the I2C EEPROM.
  * @param  pBuffer : pointer to the buffer  containing the data to be
  *   written to the EEPROM.
  * @param  WriteAddr : EEPROM's internal address to write to.
  * @retval None
  */
static void I2C_EE_ByteWrite(const uint8_t* pBuffer, uint16_t WriteAddr)
{
  /* Send STRAT condition */
  I2C_GenerateSTART(I2C_EE, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send EEPROM address for write */
  I2C_Send7bitAddress(I2C_EE, EEPROM_ADDRESS, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

#ifndef EE_M24C64_32

  /* Send the EEPROM's internal address to write to : only one byte Address */
  I2C_SendData(I2C_EE, WriteAddr);

#else

  /* Send the EEPROM's internal address to write to : MSB of the address first */
  I2C_SendData(I2C_EE, (uint8_t)((WriteAddr & 0xFF00) >> 8));

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the EEPROM's internal address to write to : LSB of the address */
  I2C_SendData(I2C_EE, (uint8_t)(WriteAddr & 0x00FF));

#endif /* EE_M24C64_32 */

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the byte to be written */
  I2C_SendData(I2C_EE, *pBuffer);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STOP condition */
  I2C_GenerateSTOP(I2C_EE, ENABLE);
  I2C_EE_WaitEepromStandbyState();
}

/**
  * @brief  Reads a block of data from the EEPROM.
  * @param  pBuffer : pointer to the buffer that receives the data read
  *   from the EEPROM.
  * @param  ReadAddr : EEPROM's internal address to read from.
  * @param  NumByteToRead : number of bytes to read from the EEPROM.
  * @retval None
  */
void I2C_EE_BufferRead(uint8_t* pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead)
{
    /* While the bus is busy */
  while(I2C_GetFlagStatus(I2C_EE, I2C_FLAG_BUSY));

  /* Send START condition */
  I2C_GenerateSTART(I2C_EE, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send EEPROM address for write */
  I2C_Send7bitAddress(I2C_EE, EEPROM_ADDRESS, /*I2C_Direction_Receiver*/I2C_Direction_Transmitter);


  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C_EE, /*I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED*/I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

#ifndef EE_M24C64_32

  /* Send the EEPROM's internal address to read from: Only one byte address */
  I2C_SendData(I2C_EE, ReadAddr);

#else

  /* Send the EEPROM's internal address to read from: MSB of the address first */
  I2C_SendData(I2C_EE, (uint8_t)((ReadAddr & 0xFF00) >> 8));

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the EEPROM's internal address to read from: LSB of the address */
  I2C_SendData(I2C_EE, (uint8_t)(ReadAddr & 0x00FF));

#endif /* EE_M24C64_32 */

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));


  /* Send STRAT condition a second time */
  I2C_GenerateSTART(I2C_EE, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send EEPROM address for read */
  I2C_Send7bitAddress(I2C_EE, EEPROM_ADDRESS, I2C_Direction_Receiver);


  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));


  /* While there is data to be read */
  while(NumByteToRead)
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(I2C_EE, DISABLE);

      /* Send STOP Condition */
 //     I2C_GenerateSTOP(I2C_EE, ENABLE);
    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
      /* Read a byte from the EEPROM */
      *pBuffer = I2C_ReceiveData(I2C_EE);

      /* Point to the next location where the byte read will be saved */
      pBuffer++;

      /* Decrement the read bytes counter */
      NumByteToRead--;
    }
  }
      I2C_GenerateSTOP(I2C_EE, ENABLE);

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2C_EE, ENABLE);
}

/**
  * @brief  Writes buffer of data to the I2C EEPROM.
  * @param  pBuffer : pointer to the buffer  containing the data to be
  *   written to the EEPROM.
  * @param  WriteAddr : EEPROM's internal address to write to.
  * @param  NumByteToWrite : number of bytes to write to the EEPROM.
  * @retval None
  */
void I2C_EE_BufferWrite(const uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite)
{
  uint8_t NumOfPage = 0, NumOfSingle = 0, count = 0;
  uint16_t Addr = 0;

  Addr = WriteAddr % I2C_FLASH_PAGESIZE;
  count = I2C_FLASH_PAGESIZE - Addr;
  NumOfPage =  NumByteToWrite / I2C_FLASH_PAGESIZE;
  NumOfSingle = NumByteToWrite % I2C_FLASH_PAGESIZE;

  /* If WriteAddr is I2C_FLASH_PAGESIZE aligned  */
  if(Addr == 0)
  {
    /* If NumByteToWrite < I2C_FLASH_PAGESIZE */
    if(NumOfPage == 0)
    {
      I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
    //  I2C_EE_WaitEepromStandbyState();
    }
    /* If NumByteToWrite > I2C_FLASH_PAGESIZE */
    else
    {
      while(NumOfPage--)
      {
        I2C_EE_PageWrite(pBuffer, WriteAddr, I2C_FLASH_PAGESIZE);
      //  I2C_EE_WaitEepromStandbyState();
        WriteAddr +=  I2C_FLASH_PAGESIZE;
        pBuffer += I2C_FLASH_PAGESIZE;
      }

      if(NumOfSingle!=0)
      {
        I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
      //  I2C_EE_WaitEepromStandbyState();
      }
    }
  }
  /* If WriteAddr is not I2C_FLASH_PAGESIZE aligned  */
  else
  {
    /* If NumByteToWrite < I2C_FLASH_PAGESIZE */
    if(NumOfPage== 0)
    {
      /* If the number of data to be written is more than the remaining space
      in the current page: */
      if (NumByteToWrite > count)
      {
        /* Write the data conained in same page */
        I2C_EE_PageWrite(pBuffer, WriteAddr, count);
      //  I2C_EE_WaitEepromStandbyState();

        /* Write the remaining data in the following page */
        I2C_EE_PageWrite((uint8_t*)(pBuffer + count), (WriteAddr + count), (NumByteToWrite - count));
     //   I2C_EE_WaitEepromStandbyState();
      }
      else
      {
        I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
      //  I2C_EE_WaitEepromStandbyState();
      }
    }
    /* If NumByteToWrite > I2C_FLASH_PAGESIZE */
    else
    {
      NumByteToWrite -= count;
      NumOfPage =  NumByteToWrite / I2C_FLASH_PAGESIZE;
      NumOfSingle = NumByteToWrite % I2C_FLASH_PAGESIZE;

      if(count != 0)
      {
        I2C_EE_PageWrite(pBuffer, WriteAddr, count);
        I2C_EE_WaitEepromStandbyState();
        WriteAddr += count;
        pBuffer += count;
      }

      while(NumOfPage--)
      {
        I2C_EE_PageWrite(pBuffer, WriteAddr, I2C_FLASH_PAGESIZE);
        I2C_EE_WaitEepromStandbyState();
        WriteAddr +=  I2C_FLASH_PAGESIZE;
        pBuffer += I2C_FLASH_PAGESIZE;
      }
      if(NumOfSingle != 0)
      {
        I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
    //    I2C_EE_WaitEepromStandbyState();
      }
    }
  }
}

/**
  * @brief  Writes more than one byte to the EEPROM with a single WRITE cycle.
  * @note   The number of byte can't exceed the EEPROM page size.
  * @param  pBuffer : pointer to the buffer containing the data to be
  *   written to the EEPROM.
  * @param  WriteAddr : EEPROM's internal address to write to.
  * @param  NumByteToWrite : number of bytes to write to the EEPROM.
  * @retval None
  */
void I2C_EE_PageWrite(const uint8_t* pBuffer, uint16_t WriteAddr, uint8_t NumByteToWrite)
{

  /* While the bus is busy */
  while(I2C_GetFlagStatus(I2C_EE, I2C_FLAG_BUSY));

  /* Send START condition */
  I2C_GenerateSTART(I2C_EE, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send EEPROM address for write */
  I2C_Send7bitAddress(I2C_EE, EEPROM_ADDRESS, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

#ifndef EE_M24C64_32

  /* Send the EEPROM's internal address to write to : only one byte Address */
  I2C_SendData(I2C_EE, WriteAddr);

#else

  /* Send the EEPROM's internal address to write to : MSB of the address first */
  I2C_SendData(I2C_EE, (uint8_t)((WriteAddr & 0xFF00) >> 8));

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the EEPROM's internal address to write to : LSB of the address */
  I2C_SendData(I2C_EE, (uint8_t)(WriteAddr & 0x00FF));

#endif /* EE_M24C64_32 */

  /* Test on EV8 and clear it */
  while(! I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* While there is data to be written */
  while(NumByteToWrite--)
  {
    /* Send the current byte */
    I2C_SendData(I2C_EE, *pBuffer);

    /* Point to the next byte to be written */
    pBuffer++;

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  }

  /* Send STOP condition */
  I2C_GenerateSTOP(I2C_EE, ENABLE);
  I2C_EE_WaitEepromStandbyState();
}

/**
  * @brief  Wait for EEPROM Standby state
  * @param  None
  * @retval None
  */
static void I2C_EE_WaitEepromStandbyState(void)
{
  __IO uint16_t SR1_Tmp = 0;

  do
  {
    /* Send START condition */
    I2C_GenerateSTART(I2C_EE, ENABLE);

    /* Read I2C_EE SR1 register to clear pending flags */
    SR1_Tmp = I2C_ReadRegister(I2C_EE, I2C_Register_SR1);

    /* Send EEPROM address for write */
    I2C_Send7bitAddress(I2C_EE, EEPROM_ADDRESS, I2C_Direction_Transmitter);

  }while(!(I2C_ReadRegister(I2C_EE, I2C_Register_SR1) & 0x0002));

  /* Clear AF flag */
  I2C_ClearFlag(I2C_EE, I2C_FLAG_AF);

  /* STOP condition */
  I2C_GenerateSTOP(I2C_EE, ENABLE);
}

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
