/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : usb_prop.c
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : All processing related to Virtual Com Port Demo
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* INCLUDES ------------------------------------------------------------------*/

#include "usb_lib.h"
#include "usb_conf.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "usb_hw_config.h"

/* PRIVATE TYPEDEF -----------------------------------------------------------*/

typedef struct {
    uint32_t bitrate;
    uint8_t format;
    uint8_t paritytype;
    uint8_t datatype;
} LINE_CODING;

/* PRIVATE DEFINES -----------------------------------------------------------*/

/* PRIVATE MACROS ------------------------------------------------------------*/

/* EXTERN VARIABLES ----------------------------------------------------------*/

/* PRIVATE VARIABLES ---------------------------------------------------------*/

static uint8_t Request = 0;

static LINE_CODING linecoding = {
    115200, /* baud rate*/
    0x00,       /* stop bits-1*/
    0x00,       /* parity - none*/
    0x08        /* no. of bits 8*/
    };

static ONE_DESCRIPTOR Device_Descriptor = {
    (uint8_t *)Virtual_Com_Port_DeviceDescriptor, VIRTUAL_COM_PORT_SIZ_DEVICE_DESC
};

static ONE_DESCRIPTOR Config_Descriptor = {
    (uint8_t *)Virtual_Com_Port_ConfigDescriptor, VIRTUAL_COM_PORT_SIZ_CONFIG_DESC
};

static ONE_DESCRIPTOR String_Descriptor[4] = {
    { (uint8_t *)Virtual_Com_Port_StringLangID, VIRTUAL_COM_PORT_SIZ_STRING_LANGID },
    { (uint8_t *)Virtual_Com_Port_StringVendor, VIRTUAL_COM_PORT_SIZ_STRING_VENDOR },
    { (uint8_t *)Virtual_Com_Port_StringProduct, VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT },
    { (uint8_t *)Virtual_Com_Port_StringSerial, VIRTUAL_COM_PORT_SIZ_STRING_SERIAL }
};

/* PUBLIC VARIABLES ----------------------------------------------------------*/

DEVICE_PROP Device_Property = {
    Virtual_Com_Port_init, Virtual_Com_Port_Reset, Virtual_Com_Port_Status_In,
    Virtual_Com_Port_Status_Out, Virtual_Com_Port_Data_Setup, Virtual_Com_Port_NoData_Setup,
    Virtual_Com_Port_Get_Interface_Setting, Virtual_Com_Port_GetDeviceDescriptor,
    Virtual_Com_Port_GetConfigDescriptor, Virtual_Com_Port_GetStringDescriptor, 0,
    0x40 /*MAX PACKET SIZE*/
};

DEVICE Device_Table = {
    EP_NUM, 1
};

USER_STANDARD_REQUESTS User_Standard_Requests = {
    Virtual_Com_Port_GetConfiguration, Virtual_Com_Port_SetConfiguration,
    Virtual_Com_Port_GetInterface, Virtual_Com_Port_SetInterface,
    Virtual_Com_Port_GetStatus, Virtual_Com_Port_ClearFeature,
    Virtual_Com_Port_SetEndPointFeature, Virtual_Com_Port_SetDeviceFeature,
    Virtual_Com_Port_SetDeviceAddress
};

/* PRIVATE FUNCTION PROTOTYPES -----------------------------------------------*/

/* PRIVATE FUNCTIONS ---------------------------------------------------------*/

/* PUBLIC FUNCTIONS ----------------------------------------------------------*/

/*******************************************************************************
* Function Name  : Virtual_Com_Port_init.
* Description    : Virtual COM Port Mouse init routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Virtual_Com_Port_init(void) {
    /* Update the serial number string descriptor with the data from the unique
    ID*/
    Get_SerialNum();

    pInformation->Current_Configuration = 0;

    /* Connect the device */
    PowerOn();

    /* Perform basic device initialization operations */
    USB_SIL_Init();

    bDeviceState = UNCONNECTED;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Reset
* Description    : Virtual_Com_Port Mouse reset routine
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Virtual_Com_Port_Reset(void) {
    /* Set Virtual_Com_Port DEVICE as not configured */
    pInformation->Current_Configuration = 0;

    /* Current Feature initialization */
    pInformation->Current_Feature = Virtual_Com_Port_ConfigDescriptor[7];

    /* Set Virtual_Com_Port DEVICE with the default Interface*/
    pInformation->Current_Interface = 0;

    /* EP0 is already configured by USB_SIL_Init() function */

    /* Init EP1 IN as Bulk endpoint */
    OTG_DEV_EP_Init(EP1_IN, OTG_DEV_EP_TYPE_BULK, VIRTUAL_COM_PORT_DATA_SIZE);

    /* Init EP2 IN as Interrupt endpoint */
    OTG_DEV_EP_Init(EP2_IN, OTG_DEV_EP_TYPE_INT, VIRTUAL_COM_PORT_INT_SIZE);

    /* Init EP3 OUT as Bulk endpoint */
    OTG_DEV_EP_Init(EP3_OUT, OTG_DEV_EP_TYPE_BULK, VIRTUAL_COM_PORT_DATA_SIZE);

    bDeviceState = ATTACHED;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_SetConfiguration.
* Description    : Update the device state to configured.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Virtual_Com_Port_SetConfiguration(void) {
    DEVICE_INFO *pInfo = &Device_Info;

    if (pInfo->Current_Configuration != 0) {
        /* Device configured */
        bDeviceState = CONFIGURED;
    }
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_SetConfiguration.
* Description    : Update the device state to addressed.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Virtual_Com_Port_SetDeviceAddress(void) {
    bDeviceState = ADDRESSED;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Status_In.
* Description    : Virtual COM Port Status In Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Virtual_Com_Port_Status_In(void) {
    if (Request == SET_LINE_CODING) {
        Request = 0;
    }
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Status_Out
* Description    : Virtual COM Port Status OUT Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Virtual_Com_Port_Status_Out(void) {
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Data_Setup
* Description    : handle the data class specific requests
* Input          : Request Nb.
* Output         : None.
* Return         : USB_UNSUPPORT or USB_SUCCESS.
*******************************************************************************/
RESULT Virtual_Com_Port_Data_Setup(uint8_t RequestNo) {
    uint8_t *( *CopyRoutine)(uint16_t);

    CopyRoutine = NULL;

    if (RequestNo == GET_LINE_CODING) {
        if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) {
            CopyRoutine = Virtual_Com_Port_GetLineCoding;
        }
    } else if (RequestNo == SET_LINE_CODING) {
        if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) {
            CopyRoutine = Virtual_Com_Port_SetLineCoding;
        }
        Request = SET_LINE_CODING;
    }

    if (CopyRoutine == NULL) {
        return USB_UNSUPPORT;
    }

    pInformation->Ctrl_Info.CopyData = CopyRoutine;
    pInformation->Ctrl_Info.Usb_wOffset = 0;
    ( *CopyRoutine)(0);
    return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_NoData_Setup.
* Description    : handle the no data class specific requests.
* Input          : Request Nb.
* Output         : None.
* Return         : USB_UNSUPPORT or USB_SUCCESS.
*******************************************************************************/
RESULT Virtual_Com_Port_NoData_Setup(uint8_t RequestNo) {
    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) {
        if (RequestNo == SET_COMM_FEATURE) {
            return USB_SUCCESS;
        } else if (RequestNo == SET_CONTROL_LINE_STATE) {
            return USB_SUCCESS;
        }
    }

    return USB_UNSUPPORT;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_GetDeviceDescriptor.
* Description    : Gets the device descriptor.
* Input          : Length.
* Output         : None.
* Return         : The address of the device descriptor.
*******************************************************************************/
uint8_t *Virtual_Com_Port_GetDeviceDescriptor(uint16_t Length) {
    return Standard_GetDescriptorData(Length, &Device_Descriptor);
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_GetConfigDescriptor.
* Description    : get the configuration descriptor.
* Input          : Length.
* Output         : None.
* Return         : The address of the configuration descriptor.
*******************************************************************************/
uint8_t *Virtual_Com_Port_GetConfigDescriptor(uint16_t Length) {
    return Standard_GetDescriptorData(Length, &Config_Descriptor);
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_GetStringDescriptor
* Description    : Gets the string descriptors according to the needed index
* Input          : Length.
* Output         : None.
* Return         : The address of the string descriptors.
*******************************************************************************/
uint8_t *Virtual_Com_Port_GetStringDescriptor(uint16_t Length) {
    uint8_t wValue0 = pInformation->USBwValue0;

    if (wValue0 > 4) {
        return NULL;
    } else {
        return Standard_GetDescriptorData(Length, &String_Descriptor[wValue0]);
    }
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Get_Interface_Setting.
* Description    : test the interface and the alternate setting according to the
*                  supported one.
* Input1         : uint8_t: Interface : interface number.
* Input2         : uint8_t: AlternateSetting : Alternate Setting number.
* Output         : None.
* Return         : The address of the string descriptors.
*******************************************************************************/
RESULT Virtual_Com_Port_Get_Interface_Setting(uint8_t Interface, uint8_t AlternateSetting) {
    if (AlternateSetting > 0) {
        return USB_UNSUPPORT;
    } else if (Interface > 1) {
        return USB_UNSUPPORT;
    }
    return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_GetLineCoding.
* Description    : send the linecoding structure to the PC host.
* Input          : Length.
* Output         : None.
* Return         : Linecoding structure base address.
*******************************************************************************/
uint8_t *Virtual_Com_Port_GetLineCoding(uint16_t Length) {
    if (Length == 0) {
        pInformation->Ctrl_Info.Usb_wLength = sizeof(linecoding);
        return NULL;
    }
    return (uint8_t *) &linecoding;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_SetLineCoding.
* Description    : Set the linecoding structure fields.
* Input          : Length.
* Output         : None.
* Return         : Linecoding structure base address.
*******************************************************************************/
uint8_t *Virtual_Com_Port_SetLineCoding(uint16_t Length) {
    if (Length == 0) {
        pInformation->Ctrl_Info.Usb_wLength = sizeof(linecoding);
        return NULL;
    }
    return (uint8_t *) &linecoding;
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/