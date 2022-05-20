/**
  ******************************************************************************
  * @file    usbd_cdc_vcp.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   Generic media access Layer.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

#define min(a,b)        (a < b ? a : b)
     
     
/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_vcp.h"
#include "usb_conf.h"
#include <ysizet.h>
#include <yfuns.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "packets.h"
#include "main.h"
    
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
LINE_CODING linecoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
  };


#define APP_TX_BUFFER_SIZE      4096/*1024*/
static uint8_t APP_Tx_Buffer[APP_TX_BUFFER_SIZE];
static uint32_t APP_Tx_ptr_out = 0;
static uint32_t APP_Tx_ptr_in = 0;

/* These are external variables imported from CDC core to be used for IN 
   transfer management. */
extern uint8_t  APP_Rx_Buffer []; /* Write CDC received data in this buffer.
                                     These data will be sent over USB IN endpoint
                                     in the CDC core functions. */
extern uint32_t APP_Rx_ptr_in;    /* Increment this pointer or roll it back to
                                     start address when writing received data
                                     in the buffer APP_Rx_Buffer. */
extern uint32_t  APP_Rx_ptr_out;

/* Private function prototypes -----------------------------------------------*/
static uint16_t VCP_Init     (void);
static uint16_t VCP_DeInit   (void);
static uint16_t VCP_Ctrl     (uint32_t Cmd, uint8_t* Buf, uint32_t Len);
static uint16_t VCP_DataTx   (const uint8_t* Buf, uint32_t Len);
static uint16_t VCP_DataRx   (uint8_t* Buf, uint32_t Len);

//static uint16_t VCP_COMConfig(uint8_t Conf);

CDC_IF_Prop_TypeDef VCP_fops = 
{
  VCP_Init,
  VCP_DeInit,
  VCP_Ctrl,
  VCP_DataTx,
  VCP_DataRx
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  VCP_Init
  *         Initializes the Media on the STM32
  * @param  None
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
static uint16_t VCP_Init(void)
{
  
  return USBD_OK;
}

/**
  * @brief  VCP_DeInit
  *         DeInitializes the Media on the STM32
  * @param  None
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
static uint16_t VCP_DeInit(void)
{

  return USBD_OK;
}

//uint32_t debug_count = 0;

/**
  * @brief  VCP_Ctrl
  *         Manage the CDC class requests
  * @param  Cmd: Command code            
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
static uint16_t VCP_Ctrl (uint32_t Cmd, uint8_t* Buf, uint32_t Len)
{ 
  switch (Cmd)
  {
  case SEND_ENCAPSULATED_COMMAND:
    /* Not  needed for this driver */
    break;

  case GET_ENCAPSULATED_RESPONSE:
    /* Not  needed for this driver */
    break;

  case SET_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case GET_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case CLEAR_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case SET_LINE_CODING:
    linecoding.bitrate = (uint32_t)(Buf[0] | (Buf[1] << 8) | (Buf[2] << 16) | (Buf[3] << 24));
    linecoding.format = Buf[4];
    linecoding.paritytype = Buf[5];
    linecoding.datatype = Buf[6];
    /* Set the new configuration */
   // VCP_COMConfig(OTHER_CONFIG);
//    debug_count = 0;
//      printf(BOOTLOADER_VERSION);
    break;

  case GET_LINE_CODING:
    Buf[0] = (uint8_t)(linecoding.bitrate);
    Buf[1] = (uint8_t)(linecoding.bitrate >> 8);
    Buf[2] = (uint8_t)(linecoding.bitrate >> 16);
    Buf[3] = (uint8_t)(linecoding.bitrate >> 24);
    Buf[4] = linecoding.format;
    Buf[5] = linecoding.paritytype;
    Buf[6] = linecoding.datatype; 
    break;

  case SET_CONTROL_LINE_STATE:
    /* Not  needed for this driver */
    break;

  case SEND_BREAK:
    /* Not  needed for this driver */
    break;    
    
  default:
    break;
  }

  return USBD_OK;
}

/**
  * @brief  VCP_DataTx
  *         CDC received data to be send over USB IN endpoint are managed in 
  *         this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion: USBD_OK or number of characters not copied
  */
static uint16_t VCP_DataTx (const uint8_t* Buf, uint32_t Len)
{
  uint16_t ret = USBD_OK;
  
  do { 
    uint32_t CopyLen = Len;
    uint32_t Avail = APP_RX_DATA_SIZE;

    if (Len == 0) {
      break;
    }
    ret = Len;
    if (APP_Rx_ptr_out <= APP_Rx_ptr_in) {
      // first part fill out to the end
      Avail = APP_RX_DATA_SIZE - APP_Rx_ptr_in;
      CopyLen = min(Avail, CopyLen);
      if (CopyLen) {
        memcpy(&APP_Rx_Buffer[APP_Rx_ptr_in], Buf, CopyLen);
        APP_Rx_ptr_in += CopyLen;
        Buf += CopyLen;
        Len -= CopyLen;
        if (APP_Rx_ptr_in >= APP_RX_DATA_SIZE) {
          APP_Rx_ptr_in = 0;
        }
        ret = Len;
        if (!Len) {
          break;
        }
      } else {
        break;
      } 
    }
    // copy rest if any 
    Avail = APP_Rx_ptr_out - APP_Rx_ptr_in;
    CopyLen = min(Avail, Len/*CopyLen*/);
    if (CopyLen) {
      memcpy(&APP_Rx_Buffer[APP_Rx_ptr_in], Buf, CopyLen);
      APP_Rx_ptr_in += CopyLen;
      Len -= CopyLen;
      ret = Len;
    }
  } while (0);
  return ret;
}

/**
  * @brief  VCP_DataRx
  *         Data received over USB OUT endpoint are sent over CDC interface 
  *         through this function.
  *           
  *         @note
  *         This function will block any OUT packet reception on USB endpoint 
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result 
  *         in receiving more data while previous ones are still not sent.
  *                 
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
  */
static uint16_t VCP_DataRx (uint8_t* Buf, uint32_t Len)
{
    uint16_t ret = USBD_OK;

   // VCP_DataTx(Buf, Len);
  //  debug_count += Len;
  do { 
    uint32_t CopyLen = Len;
    uint32_t Avail = APP_TX_BUFFER_SIZE;

    if (Len == 0) {
      break;
    }
    ret = Len;
    if (APP_Tx_ptr_out <= APP_Tx_ptr_in) {
      // first part fill out to the end
      Avail = APP_TX_BUFFER_SIZE - APP_Tx_ptr_in;
      CopyLen = min(Avail, CopyLen);
      if (CopyLen) {
        memcpy(&APP_Tx_Buffer[APP_Tx_ptr_in], Buf, CopyLen);
        APP_Tx_ptr_in += CopyLen;
        Buf += CopyLen;
        Len -= CopyLen;
        if (APP_Tx_ptr_in >= APP_TX_BUFFER_SIZE) {
          APP_Tx_ptr_in = 0;
        }
        ret = Len;
        if (!Len) {
          break;
        }
      } else {
        break;
      } 
    }
    // copy rest if any 
    Avail = APP_Tx_ptr_out - APP_Tx_ptr_in;
    CopyLen = min(Avail, Len);
    if (CopyLen) {
      memcpy(&APP_Tx_Buffer[APP_Tx_ptr_in], Buf, CopyLen);
      APP_Tx_ptr_in += CopyLen;
      Len -= CopyLen;
      ret = Len;
    }
  } while (0);
  return ret;
 
}

/*******************************************************************************
* Description : STDOUT Low-Level write
* Input       :
* Return      :
*******************************************************************************/
size_t __write(int handle, const unsigned char *buffer, size_t size) {

    if ((buffer == 0) || (handle == -1)) {
        /*
         * This means that we should flush internal buffers.  Since we
         * don't we just return.  (Remember, "handle" == -1 means that all
         * handles should be flushed.)
         */
        return 0;
    }

    /* This template only writes to "standard out" and "standard err",
     * for all other file handles it returns failure. */
    if (handle != _LLIO_STDOUT && handle != _LLIO_STDERR) {
        return _LLIO_ERROR;
    }

    return size - VCP_DataTx(buffer, size);
}

/*******************************************************************************
* Description : STDIN Low-Level read
* Input       :
* Return      :
*******************************************************************************/
size_t __read(int handle, unsigned char *buffer, size_t size) {
    size_t nChars = 0;
    size_t Avail = 0;
    
    /* This template only reads from "standard in", for all other file
    * handles it returns failure. */
    if (handle != _LLIO_STDIN) {
        return _LLIO_ERROR;
    }

    if (APP_Tx_ptr_out == APP_Tx_ptr_in || !size) {
      return 0;
    }
   // __disable_interrupt();
    if (APP_Tx_ptr_out == APP_Tx_ptr_in) {
    //  __enable_interrupt();
      return 0;
    }
    if (APP_Tx_ptr_out > APP_Tx_ptr_in) {
      Avail = APP_TX_BUFFER_SIZE - APP_Tx_ptr_out;
      size_t CopyLen = min(Avail, size);
      memcpy(buffer, &APP_Tx_Buffer[APP_Tx_ptr_out], CopyLen);
      APP_Tx_ptr_out += CopyLen;
      size -= CopyLen;
      nChars += CopyLen;
      buffer += nChars;
      if (APP_Tx_ptr_out == APP_TX_BUFFER_SIZE) {
        APP_Tx_ptr_out = 0;
      }
    }  

    if (size) {
      if (APP_Tx_ptr_out < APP_Tx_ptr_in) {
        Avail = APP_Tx_ptr_in - APP_Tx_ptr_out;
        size_t CopyLen = min(Avail, size);
        memcpy(buffer, &APP_Tx_Buffer[APP_Tx_ptr_out], CopyLen);
        APP_Tx_ptr_out += CopyLen;
        size -= CopyLen;
        nChars += CopyLen;
        if (APP_Tx_ptr_out == APP_TX_BUFFER_SIZE) {
          APP_Tx_ptr_out = 0;
        }
      }
    }
     // __enable_interrupt();
    return nChars;
}


static size_t write_header(uint8_t content_type, uint16_t length) {
  
  struct USBDebugInfoHeader dbgInfoheader;   
  int headerSize = sizeof(dbgInfoheader);
        
  dbgInfoheader.signature = TK_FRAME_SIGNATURE;
  dbgInfoheader.version = 1; 
  dbgInfoheader.checkSum = 0;        // TODO!!!
  dbgInfoheader.magicNumber = 0;     // TODO!!! random number / session id
  dbgInfoheader.packetID = 0;        // TODO!!! Incrementing number
  dbgInfoheader.contentType = content_type;
  dbgInfoheader.contentLength = length + headerSize;     // Entire packet length including Header         
  
  const unsigned char *headerBuffer = (const unsigned char *)&dbgInfoheader; 
  
  return VCP_DataTx(headerBuffer, headerSize);
}

/*******************************************************************************
* Description : STDOUT Low-Level write for IMU raw data
* Input       :
* Return      :
*******************************************************************************/
size_t __writeIMU(const unsigned char *buffer, size_t size) 
{
  size_t ret = 0;

  do {
    if (write_header(1, size)) {
      break;
    }
    ret = __write(_LLIO_STDOUT, buffer, size);
  } while (0);
  
return ret;
   
}

size_t __writeResponse(uint8_t content_type, const unsigned char *buffer, size_t size) 
{
  size_t ret = 0;

 // debug_packets ++;
  do {
    if (write_header(content_type, size)) {
      break;
    }
    ret = __write(_LLIO_STDOUT, buffer, size);
  } while (0);
  
return ret;
   
}
#if 0
// intercept and add a header
char trace_format_buf[256];
int TRACE(char* fmt, ...) {
    extern uint32_t TRACE_ENABLE;
    if (!TRACE_ENABLE) {
        return 0;
    }
   
    va_list argptr; /* Set up the variable argument list here */
    va_start(argptr, fmt); /* Start up variable arguments */
    vsprintf(trace_format_buf, fmt, argptr); /* print the variable arguments to buffer */
    va_end(argptr);  /* Signify end of processing of variable arguments */
    
    // Send out a header
    extern uint32_t TRACE_USE_PROTO_CMD_LINE_RESP;
    
    if (TRACE_USE_PROTO_CMD_LINE_RESP) {
      if (write_header(0, strlen(trace_format_buf))) {
        return 0;
      }
    }
   
    // send body
    return printf(trace_format_buf);      
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
