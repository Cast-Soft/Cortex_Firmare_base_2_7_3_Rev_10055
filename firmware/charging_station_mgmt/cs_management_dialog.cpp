
// beacon_dialog.cpp : implementation file
//

#include "stdafx.h"

#include <assert.h>
#include <sstream>

#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/date_time/local_time/local_time.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/math/constants/constants.hpp>

#include <boost/dynamic_bitset.hpp>
#include <boost/tokenizer.hpp>

#include "VersionNo.h"

#include "cs_management.h"
#include "cs_management_dialog.h"
#include "afxdialogex.h"

#include <common/md5.h>
#include <common/base64.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

std::string CCSMDlg::BTPRJ_HEAD = "V0.01";      // define data serialization version
std::string CCSMDlg::cfgFile = "charging_station_mgmt.cfg";

BOOST_CLASS_TRACKING( PersistentSet, boost::serialization::track_never );

// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CCSMDlg dialog




CCSMDlg::CCSMDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CCSMDlg::IDD, pParent)
    , logToFile(FALSE)
    , logData(TRUE)
	, logToScreen(TRUE)
    , dirtyLog(false)
    , cmdParam(_T(""))
    , cmdIndex(0)
    , firmwareUploadEnabled(FALSE)
    , writtenCount(0)
	, productID("CAST BT")
	, serialNumber("1")
	, ledID1(255)
	, ledID2(255)
	, ledID3(255)
	, beaconID(0)
	, routerAddress(0)
	, ledOnOffset(0)
	, ledOffOffset(0)
	, wirelessChannel(26)
	, wirelessTimeSlot(0)
	, txPowerLevel(8)
	, panID(0)

	, flashUpdateType(0)

	, currentDeviceIndex(-1)
	, cmdApplyType(0)
	, jumpOnConnect(NO_JUMP)
	, radioOnOnConnect(FALSE)
	, enableIMU(FALSE)
	, enableButton(FALSE)
	, enableBattery(FALSE)
	, currentDeviceID(_T(""))
	, numberOfBits(1)
	, beaconNumBit(1)
	, hopChannels(_T(""))
{
	bitArray.push_back(8);
	bitArray.push_back(12);
	bitArray.push_back(14);
	bitArray.push_back(16);

	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
    serialComMessenger.reset(new ConcreteSerialComMessenger(this));

    Command cmd;

    cmd.name = "d";
    cmd.desc = "Display settings";
    commands.push_back(cmd);

    cmd.name = ".";
    cmd.desc = "Switch on / off IMU dumping";
    commands.push_back(cmd);

    cmd.name = "B";
    cmd.desc = "Jump to Firmware";
    commands.push_back(cmd);

    cmd.name = "!";
    cmd.desc = "Jump to bootloader (For firmware update)";
    commands.push_back(cmd);

    cmd.name = "t 0";
    cmd.desc = "Trace Enable / Disable";
    commands.push_back(cmd);

    cmd.name = "t 1";
    cmd.desc = "Trace Time Slot On / Off";
    commands.push_back(cmd);

    cmd.name = "t 2";
    cmd.desc = "Trace LED Pulse Synchronization  On / Off";
    commands.push_back(cmd);

    cmd.name = "t 3";
    cmd.desc = "Trace IMU Queue On / Off";
    commands.push_back(cmd);

    cmd.name = "t 4";
    cmd.desc = "Trace Use Time Slot  On / Off";
    commands.push_back(cmd);

    cmd.name = "t 5";
    cmd.desc = "Command line responses in protocol format On / Off";
    commands.push_back(cmd);

    cmd.name = "t 6";
    cmd.desc = "Turn off Radio";
    commands.push_back(cmd);

    cmd.name = "t 7";
    cmd.desc = "Turn on Radio";
    commands.push_back(cmd);


    cmd.name = "t 8";
    cmd.desc = "Who am I? It could be boot loader or main firmware";
    commands.push_back(cmd);

    cmd.name = "";
    cmd.desc = "Any legacy command typed down below";
    commands.push_back(cmd);

	beacon::Config config;

	SetLocalConfig(config);	// default value
}

void CCSMDlg::RepopulateDeviceProperties() {
    serial_com::SerialComConnectionInfo* connectionInfo = GetCurSerialComConnectionInfo();
	boost::optional<WhoAmI> whoAmI;
	if (connectionInfo) {
		whoAmI = connectionInfo->whoAmI;

		std::string usbDeviceStr = ToUSBDeviceId((const uint8_t*)connectionInfo->firmwareDeviceId.c_str());
		currentDeviceID = CA2T(usbDeviceStr.c_str());
	}

	CWnd* windLBTimeSlot = GetDlgItem(IDC_LB_TIME_SLOT);
	CWnd* windLBLED1ID = GetDlgItem(IDC_LB_LED1_ID);
	CWnd* windLBLED2ID = GetDlgItem(IDC_LB_LED2_ID);
	CWnd* windLBLED3ID = GetDlgItem(IDC_LB_LED3_ID);

	CWnd* windLBHopChannel = GetDlgItem(IDC_LB_HOP_CH);

	CWnd* windCtrlHopChannel = GetDlgItem(IDC_HOP_CH);
	CWnd* windCtlSetHopChannel = GetDlgItem(IDC_BT_SET_HOP_CH);

	// Controls of single operation  

    if (cmdApplyType != 0) {		// multiple -- allow for batch setting commands only
		ctlGetConfig.ShowWindow(SW_HIDE);
		ctlStoreConfig.ShowWindow(SW_HIDE);
		ctlDefaultConfig.ShowWindow(SW_HIDE);				
	}
	else {
		ctlGetConfig.ShowWindow(SW_SHOW);
		ctlStoreConfig.ShowWindow(SW_SHOW);
		ctlDefaultConfig.ShowWindow(SW_SHOW);
		if (connectionInfo == NULL || !whoAmI || whoAmI->module != 2) {		// for main firmware only
			ctlGetConfig.EnableWindow(FALSE);
			ctlStoreConfig.EnableWindow(FALSE);
			ctlDefaultConfig.EnableWindow(FALSE);	
		}
		else {
			ctlGetConfig.EnableWindow(TRUE);

			if (connectionInfo->configBodyPacket.empty()) {
				ctlStoreConfig.EnableWindow(FALSE);
			}
			else {
				ctlStoreConfig.EnableWindow(TRUE);
			}

			ctlDefaultConfig.EnableWindow(TRUE);
		}
	}

	// Control of editing
	if (cmdApplyType == 0) {	// single
		if (connectionInfo == NULL || connectionInfo->productInfo.empty() || !whoAmI || whoAmI->module != 2) {		// connected, received config, known who & in main firmware
			ctlProductID.ShowWindow(SW_HIDE);
			ctlSerialNum.ShowWindow(SW_HIDE);
			ctlSetProductID.ShowWindow(SW_HIDE);
		}
		else {
			ctlProductID.ShowWindow(SW_SHOW);
			ctlSerialNum.ShowWindow(SW_SHOW);
			ctlSetProductID.ShowWindow(SW_SHOW);
		}

		if (connectionInfo == NULL || connectionInfo->configBodyPacket.empty() || !whoAmI || whoAmI->module != 2) {		// connected, received config, known who & in main firmware
			ctlChannel.ShowWindow(SW_HIDE);
			ctlTxLevel.ShowWindow(SW_HIDE);
			ctlPanID.ShowWindow(SW_HIDE);

			ctlRouterAddr.ShowWindow(SW_HIDE);
			ctlLEDOnOffset.ShowWindow(SW_HIDE);
			ctlLEDOffOffset.ShowWindow(SW_HIDE);
			ctlLED1ID.ShowWindow(SW_HIDE);
			ctlLED2ID.ShowWindow(SW_HIDE);
			ctlLED3ID.ShowWindow(SW_HIDE);
			ctlLED1Pattern.ShowWindow(SW_HIDE);
			ctlLED2Pattern.ShowWindow(SW_HIDE);
			ctlLED3Pattern.ShowWindow(SW_HIDE);

			ctlBeaconID.ShowWindow(SW_HIDE);
			ctlTimeSlot.ShowWindow(SW_HIDE);
			ctlBeaconNumBit.ShowWindow(SW_HIDE);

			ctlSetChannel.ShowWindow(SW_HIDE);
			ctlSetTxLevel.ShowWindow(SW_HIDE);
			ctlSetPanID.ShowWindow(SW_HIDE);

			ctlSetRouterAddr.ShowWindow(SW_HIDE);
			ctlSetLEDOnOffset.ShowWindow(SW_HIDE);
			ctlSetLEDOffOffset.ShowWindow(SW_HIDE);
			ctlSetLED1ID.ShowWindow(SW_HIDE);
			ctlSetLED2ID.ShowWindow(SW_HIDE);
			ctlSetLED3ID.ShowWindow(SW_HIDE);
			ctlSetBeaconID.ShowWindow(SW_HIDE);
			ctlSetTimeSlot.ShowWindow(SW_HIDE);

			windLBTimeSlot->ShowWindow(SW_HIDE);
			windLBLED1ID->ShowWindow(SW_HIDE);
			windLBLED2ID->ShowWindow(SW_HIDE);
			windLBLED3ID->ShowWindow(SW_HIDE);

			windLBHopChannel->ShowWindow(SW_HIDE);
			windCtrlHopChannel->ShowWindow(SW_HIDE);
			windCtlSetHopChannel->ShowWindow(SW_HIDE);

			ctrlEnableIMU.ShowWindow(SW_HIDE);
			ctrlEnableButton.ShowWindow(SW_HIDE);
			ctrlEnableBattery.ShowWindow(SW_HIDE);
		}
		else {
			ctlChannel.ShowWindow(SW_SHOW);
			ctlTxLevel.ShowWindow(SW_SHOW);
			ctlPanID.ShowWindow(SW_SHOW);

			ctlSetChannel.ShowWindow(SW_SHOW);
			ctlSetTxLevel.ShowWindow(SW_SHOW);
			ctlSetPanID.ShowWindow(SW_SHOW);

			if (whoAmI->type == 3) {		// beacon
				ctlRouterAddr.ShowWindow(SW_SHOW);
				ctlLEDOnOffset.ShowWindow(SW_SHOW);
				ctlLEDOffOffset.ShowWindow(SW_SHOW);
				ctlLED1ID.ShowWindow(SW_SHOW);
				ctlLED2ID.ShowWindow(SW_SHOW);
				ctlLED3ID.ShowWindow(SW_SHOW);
				ctlLED1Pattern.ShowWindow(SW_SHOW);
				ctlLED2Pattern.ShowWindow(SW_SHOW);
				ctlLED3Pattern.ShowWindow(SW_SHOW);

				ctlBeaconNumBit.ShowWindow(SW_SHOW);
				ctlBeaconID.ShowWindow(SW_SHOW);
				ctlTimeSlot.ShowWindow(SW_SHOW);
				ctlBit.ShowWindow(SW_SHOW);
				ctlSetRouterAddr.ShowWindow(SW_SHOW);
				ctlSetLEDOnOffset.ShowWindow(SW_SHOW);
				ctlSetLEDOffOffset.ShowWindow(SW_SHOW);
				ctlSetLED1ID.ShowWindow(SW_SHOW);
				ctlSetLED2ID.ShowWindow(SW_SHOW);
				ctlSetLED3ID.ShowWindow(SW_SHOW);
				ctlSetBeaconID.ShowWindow(SW_SHOW);
				ctlSetTimeSlot.ShowWindow(SW_SHOW);

				windLBTimeSlot->ShowWindow(SW_SHOW);
				windLBLED1ID->ShowWindow(SW_SHOW);
				windLBLED2ID->ShowWindow(SW_SHOW);
				windLBLED3ID->ShowWindow(SW_SHOW);

				windLBHopChannel->ShowWindow(SW_HIDE);
				windCtrlHopChannel->ShowWindow(SW_HIDE);
				windCtlSetHopChannel->ShowWindow(SW_HIDE);

				ctrlEnableIMU.ShowWindow(SW_SHOW);
				ctrlEnableButton.ShowWindow(SW_SHOW);
				ctrlEnableBattery.ShowWindow(SW_SHOW);

				beacon::Config* config;
				config = reinterpret_cast<beacon::Config*>(const_cast<char*>(connectionInfo->configBodyPacket.c_str()));

				SetLocalConfig(*config);
			}
			else {		// T.K. or Router
				ctlRouterAddr.ShowWindow(SW_HIDE);
				ctlLEDOnOffset.ShowWindow(SW_HIDE);
				ctlLEDOffOffset.ShowWindow(SW_HIDE);
				ctlLED1ID.ShowWindow(SW_HIDE);
				ctlLED2ID.ShowWindow(SW_HIDE);
				ctlLED3ID.ShowWindow(SW_HIDE);
				ctlLED1Pattern.ShowWindow(SW_HIDE);
				ctlLED2Pattern.ShowWindow(SW_HIDE);
				ctlLED3Pattern.ShowWindow(SW_HIDE);

				ctlBeaconNumBit.ShowWindow(SW_HIDE);
				ctlBeaconID.ShowWindow(SW_HIDE);
				ctlTimeSlot.ShowWindow(SW_HIDE);
				ctlBit.ShowWindow(SW_HIDE);
				ctlSetRouterAddr.ShowWindow(SW_HIDE);
				ctlSetLEDOnOffset.ShowWindow(SW_HIDE);
				ctlSetLEDOffOffset.ShowWindow(SW_HIDE);
				ctlSetLED1ID.ShowWindow(SW_HIDE);
				ctlSetLED2ID.ShowWindow(SW_HIDE);
				ctlSetLED3ID.ShowWindow(SW_HIDE);
				ctlSetBeaconID.ShowWindow(SW_HIDE);
				ctlSetTimeSlot.ShowWindow(SW_HIDE);

				windLBTimeSlot->ShowWindow(SW_HIDE);
				windLBLED1ID->ShowWindow(SW_HIDE);
				windLBLED2ID->ShowWindow(SW_HIDE);
				windLBLED3ID->ShowWindow(SW_HIDE);

				if (whoAmI->type == 1) {
					windLBHopChannel->ShowWindow(SW_SHOW);
					windCtrlHopChannel->ShowWindow(SW_SHOW);
					windCtlSetHopChannel->ShowWindow(SW_SHOW);
				}
				else {
					windLBHopChannel->ShowWindow(SW_HIDE);
					windCtrlHopChannel->ShowWindow(SW_HIDE);
					windCtlSetHopChannel->ShowWindow(SW_HIDE);					
				}

				ctrlEnableIMU.ShowWindow(SW_HIDE);
				ctrlEnableButton.ShowWindow(SW_HIDE);
				ctrlEnableBattery.ShowWindow(SW_HIDE);

				timekeeper::Config* config;
				config = reinterpret_cast<timekeeper::Config*>(const_cast<char*>(connectionInfo->configBodyPacket.c_str()));

				SetLocalConfig(*config);
			}	

			PopulateProductInfo(connectionInfo);

			this->UpdateData(false);					
		}
	}
	else { // multiple
		ctlProductID.ShowWindow(SW_SHOW);
		ctlSerialNum.ShowWindow(SW_SHOW);
		ctlSetProductID.ShowWindow(SW_SHOW);


		ctlChannel.ShowWindow(SW_SHOW);
		ctlTxLevel.ShowWindow(SW_SHOW);
		ctlPanID.ShowWindow(SW_SHOW);

		ctlRouterAddr.ShowWindow(SW_SHOW);
		ctlLEDOnOffset.ShowWindow(SW_SHOW);
		ctlLEDOffOffset.ShowWindow(SW_SHOW);

		ctlLED1ID.ShowWindow(SW_HIDE);
		ctlLED2ID.ShowWindow(SW_HIDE);
		ctlLED3ID.ShowWindow(SW_HIDE);
		ctlLED1Pattern.ShowWindow(SW_HIDE);
		ctlLED2Pattern.ShowWindow(SW_HIDE);
		ctlLED3Pattern.ShowWindow(SW_HIDE);

		ctlBeaconID.ShowWindow(SW_HIDE);
		ctlTimeSlot.ShowWindow(SW_HIDE);	
		ctlBeaconNumBit.ShowWindow(SW_HIDE);
		ctlSetChannel.ShowWindow(SW_SHOW);
		ctlSetTxLevel.ShowWindow(SW_SHOW);
		ctlSetPanID.ShowWindow(SW_SHOW);

		ctlSetRouterAddr.ShowWindow(SW_SHOW);
		ctlSetLEDOnOffset.ShowWindow(SW_SHOW);
		ctlSetLEDOffOffset.ShowWindow(SW_SHOW);

		ctlSetLED1ID.ShowWindow(SW_HIDE);
		ctlSetLED2ID.ShowWindow(SW_HIDE);
		ctlSetLED3ID.ShowWindow(SW_HIDE);
		ctlSetBeaconID.ShowWindow(SW_HIDE);
		ctlSetTimeSlot.ShowWindow(SW_HIDE);

		windLBTimeSlot->ShowWindow(SW_HIDE);
		windLBLED1ID->ShowWindow(SW_HIDE);
		windLBLED2ID->ShowWindow(SW_HIDE);
		windLBLED3ID->ShowWindow(SW_HIDE);

		windLBHopChannel->ShowWindow(SW_HIDE);
		windCtrlHopChannel->ShowWindow(SW_HIDE);
		windCtlSetHopChannel->ShowWindow(SW_HIDE);					

		ctrlEnableIMU.ShowWindow(SW_SHOW);
		ctrlEnableButton.ShowWindow(SW_SHOW);
		ctrlEnableBattery.ShowWindow(SW_SHOW);
	}
}

void CCSMDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);

	DDX_Control(pDX, IDC_LOG, log);
	DDX_Control(pDX, IDC_CMD_LIST, commandListCtl);
	DDX_Control(pDX, IDC_BATTERY, batteryStatus);
	DDX_Control(pDX, IDC_STATUS_ENABLED, statusEnabled);

	DDX_Check(pDX, IDC_LOG_TO_FILE, logToFile);
	DDX_Check(pDX, IDC_LOG_DATA, logData);
	DDX_Text(pDX, IDC_CMD_PARAM, cmdParam);
	DDX_CBIndex(pDX, IDC_CMD_LIST, cmdIndex);
	DDX_Check(pDX, IDC_FW_UPLOAD_ENABLE, firmwareUploadEnabled);

	DDX_CBIndex(pDX, IDC_JUMP_ON_CONNECT, jumpOnConnect);
	DDV_MinMaxInt(pDX, jumpOnConnect, 0, 2);

	DDX_Radio(pDX, IDC_UPDATE_TYPE, flashUpdateType);
	DDX_Control(pDX, IDC_DEVICE_LIST, deviceListCtrl);
	DDX_Radio(pDX, IDC_CMD_APPLY_SINGLE, cmdApplyType);
	DDX_Check(pDX, IDC_CHECK_RADIO_ON, radioOnOnConnect);

	DDX_Text(pDX, IDC_PRODUCTID, productID);
	DDX_Text(pDX, IDC_SERIAL_NUM, serialNumber);

	DDX_Text(pDX, IDC_ROUTER_ADDR, routerAddress);
	DDV_MinMaxInt(pDX, routerAddress, 0, 65535);
	DDX_CBIndex(pDX, IDC_CHANNEL, wirelessChannel);
	DDV_MinMaxInt(pDX, wirelessChannel, 0, 26 - 11);
	DDX_CBIndex(pDX, IDC_TX_LEVEL, txPowerLevel);
	DDV_MinMaxInt(pDX, txPowerLevel, 0, 7);
	DDX_Text(pDX, IDC_PAN_ID, panID);
	DDV_MinMaxInt(pDX, panID, 0, 65535);

	DDX_Text(pDX, IDC_LED_ON_OFFSET, ledOnOffset);
	DDV_MinMaxInt(pDX, ledOnOffset, 0, 60000);
	DDX_Text(pDX, IDC_LED_OFF_OFFSET, ledOffOffset);
	DDV_MinMaxInt(pDX, ledOffOffset, 0, 65535);
	DDX_Text(pDX, IDC_LED1_ID, ledID1);
	DDX_Text(pDX, IDC_LED2_ID, ledID2);
	DDX_Text(pDX, IDC_LED3_ID, ledID3);
	DDX_Text(pDX, IDC_LED1_PATTERN, led1Pattern);
	DDX_Text(pDX, IDC_LED2_PATTERN, led2Pattern);
	DDX_Text(pDX, IDC_LED3_PATTERN, led3Pattern);

	DDX_Text(pDX, IDC_BEACON_ID, beaconID);
	DDV_MinMaxInt(pDX, beaconID, 0, 65535);
	DDX_CBIndex(pDX, IDC_TIME_SLOT, wirelessTimeSlot);
	DDV_MinMaxInt(pDX, wirelessTimeSlot, 0, 13);
	DDX_CBIndex(pDX, IDC_BITS, numberOfBits);
	DDV_MinMaxInt(pDX, numberOfBits, 0, 3);

	DDX_Control(pDX, IDC_GET_CONFIG, ctlGetConfig);
	DDX_Control(pDX, IDC_STORE_CONFIG, ctlStoreConfig);
	DDX_Control(pDX, IDC_DEFAULT_CONFIG, ctlDefaultConfig);

	DDX_Control(pDX, IDC_PRODUCTID, ctlProductID);
	DDX_Control(pDX, IDC_SERIAL_NUM, ctlSerialNum);
	DDX_Control(pDX, IDC_ROUTER_ADDR, ctlRouterAddr);
	DDX_Control(pDX, IDC_CHANNEL, ctlChannel);
	DDX_Control(pDX, IDC_TX_LEVEL, ctlTxLevel);
	DDX_Control(pDX, IDC_PAN_ID, ctlPanID);
	DDX_Control(pDX, IDC_LED_ON_OFFSET, ctlLEDOnOffset);
	DDX_Control(pDX, IDC_LED_OFF_OFFSET, ctlLEDOffOffset);
	DDX_Control(pDX, IDC_LED1_ID, ctlLED1ID);
	DDX_Control(pDX, IDC_LED2_ID, ctlLED2ID);
	DDX_Control(pDX, IDC_LED3_ID, ctlLED3ID);
	DDX_Control(pDX, IDC_LED1_PATTERN, ctlLED1Pattern);
	DDX_Control(pDX, IDC_LED2_PATTERN, ctlLED2Pattern);
	DDX_Control(pDX, IDC_LED3_PATTERN, ctlLED3Pattern);

	DDX_Control(pDX, IDC_BEACON_ID, ctlBeaconID);
	DDX_Control(pDX, IDC_TIME_SLOT, ctlTimeSlot);
	DDX_Control(pDX, IDC_BITS, ctlBit);

	DDX_Control(pDX, IDC_SET_PRODUCT_ID, ctlSetProductID);
	DDX_Control(pDX, IDC_SET_ROUTER_ADDR, ctlSetRouterAddr);
	DDX_Control(pDX, IDC_BT_SET_RF_CHANNEL, ctlSetChannel);
	DDX_Control(pDX, IDC_BT_SET_POWER, ctlSetTxLevel);
	DDX_Control(pDX, IDC_BT_SET_PANID, ctlSetPanID);
	DDX_Control(pDX, IDC_BT_LEDON_OFFSET, ctlSetLEDOnOffset);
	DDX_Control(pDX, IDC_BT_LEDOFF_OFFSET, ctlSetLEDOffOffset);
	DDX_Control(pDX, IDC_SET_LED1ID, ctlSetLED1ID);
	DDX_Control(pDX, IDC_LED2ID, ctlSetLED2ID);
	DDX_Control(pDX, IDC_LED3ID, ctlSetLED3ID);
	DDX_Control(pDX, IDC_BT_SET_BEACONID, ctlSetBeaconID);
	DDX_Control(pDX, IDC_BT_SET_TIMESLOT, ctlSetTimeSlot);
	DDX_Control(pDX, IDC_CHECK_ENABLE_IMU, ctrlEnableIMU);
	DDX_Control(pDX, IDC_CHECK_ENABLE_BTN, ctrlEnableButton);
	DDX_Control(pDX, IDC_CHECK_ENABLE_BTRY, ctrlEnableBattery);
	DDX_Check(pDX, IDC_CHECK_ENABLE_IMU, enableIMU);
	DDX_Check(pDX, IDC_CHECK_ENABLE_BTN, enableButton);
	DDX_Check(pDX, IDC_CHECK_ENABLE_BTRY, enableBattery);
	DDX_Text(pDX, IDC_EDIT_DEVICE_ID, currentDeviceID);
	DDV_MaxChars(pDX, serialNumber, 12);
	DDV_MaxChars(pDX, productID, 12);
	DDX_Text(pDX, IDC_EDIT_NUM_BITS, beaconNumBit);
	DDX_Control(pDX, IDC_EDIT_NUM_BITS, ctlBeaconNumBit);
	DDX_Text(pDX, IDC_HOP_CH, hopChannels);
	DDV_MaxChars(pDX, hopChannels, 4);
	DDX_Check(pDX, IDC_LOG_TO_SCREEN, logToScreen);
}

BEGIN_MESSAGE_MAP(CCSMDlg, CDialog)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
    ON_BN_CLICKED(IDC_LOG_TO_FILE, &CCSMDlg::OnBnClickedLogToFile)
    ON_WM_CLOSE()
    ON_BN_CLICKED(IDC_CLEAR_LOG, &CCSMDlg::OnBnClickedClearLog)
    ON_BN_CLICKED(IDC_LOG_DATA, &CCSMDlg::OnBnClickedLogData)
    ON_WM_TIMER()
	ON_MESSAGE(ConcreteSerialComMessenger::WM_SERIAL_COM_IO, &CCSMDlg::OnSerialComIO)
    ON_BN_CLICKED(IDC_SEND_CMD, &CCSMDlg::OnBnClickedSendCmd)
    ON_BN_CLICKED(IDC_BTN_READ_FW, &CCSMDlg::OnBnClickedBtnReadFw)
    ON_BN_CLICKED(IDC_FW_UPLOAD_ENABLE, &CCSMDlg::OnBnClickedFwUploadEnable)
    ON_BN_CLICKED(IDC_BTN_UPLOAD_NOW, &CCSMDlg::OnBnClickedBtnUploadNow)
	ON_WM_ACTIVATE()
	ON_MESSAGE(WM_DEVICECHANGE, &CCSMDlg::OnDevicechange)

	ON_BN_CLICKED(IDC_GET_CONFIG, &CCSMDlg::OnBnClickedGetConfig)
	ON_BN_CLICKED(IDC_STORE_CONFIG, &CCSMDlg::OnBnClickedStoreConfig)
	ON_BN_CLICKED(IDC_DEFAULT_CONFIG, &CCSMDlg::OnBnClickedDefaultConfig)
	ON_BN_CLICKED(IDC_GET_STATUS, &CCSMDlg::OnBnClickedGetStatus)
	ON_BN_CLICKED(IDC_FW_VERIFY, &CCSMDlg::OnBnClickedFwVerify)
	ON_BN_CLICKED(IDC_BT_SET_PANID, &CCSMDlg::OnBnClickedBtSetPanid)
	ON_BN_CLICKED(IDC_BT_SET_BEACONID, &CCSMDlg::OnBnClickedBtSetBeaconid)
	ON_BN_CLICKED(IDC_BT_LEDON_OFFSET, &CCSMDlg::OnBnClickedBtLedonOffset)
	ON_BN_CLICKED(IDC_BT_LEDOFF_OFFSET, &CCSMDlg::OnBnClickedBtLedoffOffset)
	ON_BN_CLICKED(IDC_BT_SET_RF_CHANNEL, &CCSMDlg::OnBnClickedBtSetRfChannel)
	ON_BN_CLICKED(IDC_BT_SET_TIMESLOT, &CCSMDlg::OnBnClickedBtSetTimeslot)
	ON_BN_CLICKED(IDC_BT_SET_POWER, &CCSMDlg::OnBnClickedBtSetPower)
	ON_BN_CLICKED(IDC_UPDATE_TYPE, &CCSMDlg::OnBnClickedUpdateType)
	ON_BN_CLICKED(IDC_RADIO2, &CCSMDlg::OnBnClickedRadio2)

	ON_NOTIFY(LVN_ITEMCHANGED, IDC_DEVICE_LIST, &CCSMDlg::OnLvnItemchangedDeviceList)
	ON_BN_CLICKED(IDC_CMD_APPLY_SELECTED, &CCSMDlg::OnBnClickedCmdApplySelected)
	ON_BN_CLICKED(IDC_CMD_APPLY_ALL, &CCSMDlg::OnBnClickedCmdApplyAll)
    ON_CBN_SELCHANGE(IDC_JUMP_ON_CONNECT, &CCSMDlg::OnCbnSelchangeJumpOnConnect)
    ON_CBN_SELCHANGE(IDC_BITS, &CCSMDlg::OnCbnSelchangeNumBit)
	ON_BN_CLICKED(IDC_CHECK_RADIO_ON, &CCSMDlg::OnBnClickedCheckRadioOn)
	ON_BN_CLICKED(IDC_SET_PRODUCT_ID, &CCSMDlg::OnBnClickedSetProductId)
	ON_BN_CLICKED(IDC_SET_ROUTER_ADDR, &CCSMDlg::OnBnClickedSetRouterAddr)
	ON_BN_CLICKED(IDC_SET_LED1ID, &CCSMDlg::OnBnClickedSetLed1id)
	ON_BN_CLICKED(IDC_LED2ID, &CCSMDlg::OnBnClickedLed2id)
	ON_BN_CLICKED(IDC_LED3ID, &CCSMDlg::OnBnClickedLed3id)
	ON_BN_CLICKED(IDC_CMD_APPLY_SINGLE, &CCSMDlg::OnBnClickedCmdApplySingle)
	ON_CBN_SELCHANGE(IDC_CMD_LIST, &CCSMDlg::OnCbnSelchangeCmdList)
	ON_BN_CLICKED(IDC_CHECK_ENABLE_IMU, &CCSMDlg::OnBnClickedCheckEnableImu)
	ON_BN_CLICKED(IDC_CHECK_ENABLE_BTRY, &CCSMDlg::OnBnClickedCheckEnableBtry)
	ON_BN_CLICKED(IDC_CHECK_ENABLE_BTN, &CCSMDlg::OnBnClickedCheckEnableBtn)
	ON_BN_CLICKED(IDC_BT_SET_HOP_CH, &CCSMDlg::OnBnClickedBtSetHopCh)
	ON_BN_CLICKED(IDC_LOG_TO_SCREEN, &CCSMDlg::OnBnClickedLogToScreen)
	ON_BN_CLICKED(IDC_FLUSH_BTN, &CCSMDlg::OnBnClickedFlushBtn)
END_MESSAGE_MAP()


// CCSMDlg message handlers

BOOL CCSMDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// Set applciation version 
	{
		std::string dispStr("CSM ");
		std::string appVersion;

		int ver[] = {FILEVER};
		appVerVector.assign(ver, ver + sizeof(ver) / sizeof(ver[0]));
		
		for (size_t index = 0; index < appVerVector.size(); ++index) {
			int verNum = appVerVector.at(index);
			std::string verNumStr = boost::lexical_cast<std::string>(verNum);
			if (index == 0) {
				appVersion = verNumStr;
			}
			else {
				appVersion += ".";
				appVersion += verNumStr;
			}
		}

		dispStr.append(appVersion);
		this->SetWindowText(CString(dispStr.c_str()).GetBuffer(0));
	}

	// TODO: Add extra initialization here

    for (size_t i = 0; i < commands.size(); i++)
    {
        commandListCtl.AddString(CString(commands[i].desc.c_str()));
    }
#if 0
    // Test
    std::string source("\r\naj;sljrop23q    lknaslk;alskdfasdf  26= dnkl;vxc, ./zc.,xzgsl;kdj");
    char target[2048];
    int sourceLength = source.length();
    int ret = b64_ntop((const unsigned char *)source.c_str(), sourceLength, target, 2048);
    
    unsigned char decodeTarget[2048];
    int ret2 = b64_pton((char const *)target, decodeTarget, 2048);
#endif

	deviceListCtrl.InsertColumn(COL_DEV_ID, _T("Device ID"), LVCFMT_LEFT, 100);
	deviceListCtrl.InsertColumn(COL_DEV_TYPE, _T("Device Type"), LVCFMT_LEFT, 80);
	deviceListCtrl.InsertColumn(COL_DEV_PORT, _T("Port"), LVCFMT_LEFT, 80);
	deviceListCtrl.InsertColumn(COL_DEV_STATUS, _T("Status"), LVCFMT_LEFT, 120);
	deviceListCtrl.InsertColumn(COL_DEV_FIRMWARE_TYPE, _T("Firmware Type"), LVCFMT_LEFT, 100);
	deviceListCtrl.InsertColumn(COL_FIRM_VERSION, _T("Version"), LVCFMT_LEFT, 100);
	deviceListCtrl.InsertColumn(COL_DEV_SENT, _T("Sent"), LVCFMT_LEFT, 80);
	deviceListCtrl.InsertColumn(COL_DEV_RECEIVED, _T("Received"), LVCFMT_LEFT, 80);
	deviceListCtrl.InsertColumn(COL_MEMO, _T("Memo"), LVCFMT_LEFT, 120);

	deviceListCtrl.SetExtendedStyle(deviceListCtrl.GetExtendedStyle() | LVS_EX_HEADERDRAGDROP | 
		LVS_EX_COLUMNOVERFLOW | LVS_EX_DOUBLEBUFFER | LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES | LVS_EX_LABELTIP | LVS_EX_INFOTIP);


	filtr.dbcc_size = sizeof(filtr);
	filtr.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;
	//filtr.dbcc_classguid = 

	notify = RegisterDeviceNotification(this->m_hWnd, &filtr, DEVICE_NOTIFY_WINDOW_HANDLE);
	if (notify == NULL) {
		DWORD error = 0;
		error = ::GetLastError();
		assert(0);
	}

	RepopulateDeviceProperties();
	m_com_manager.reset(new ComDeviceManager(this->GetSafeHwnd(), this));

	SetTimer(DeviceStatusTimer, 30 * 1000, 0);		// every 30 seconds

    this->Load();

	ReloadLEDMapping();

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CCSMDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CCSMDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CCSMDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

void CCSMDlg::OnCancel()
{
    if (AfxMessageBox (_T("Do you want to exit?"), 
        MB_YESNO | MB_ICONINFORMATION | MB_DEFBUTTON2) == IDYES) {

        this->UpdateData(true);
        this->Save();

        if (m_com_manager != NULL) {
		    m_com_manager.reset(NULL);
        }

        if (serialComMessenger != NULL) {
            serialComMessenger->Stop();
            serialComMessenger.reset();
        }
        CDialog::OnCancel();
    }

}

void CCSMDlg::OnOK()

{
    // WinDialogCommon::OnOK();
}

void CCSMDlg::LogToFile(const CString& str) {
    if (!logToFile) return;
	std::stringstream ss;
    if (logFileStream.is_open() == false) {

        std::string pathStr("C:\\bt_run_time\\");
        CreateDirectory(CA2T(pathStr.c_str()),NULL); 

        // lazy initialization
        int pid = _getpid();
        
		ss.str("");
        CTime now = CTime::GetCurrentTime();
        CString dtString = now.Format("%Y%d%m %H%M");
        ss << pathStr.c_str() << "cs_management" << CT2A(dtString) << " " << pid <<".log";       //todo!!! registry
        logFileStream.open(ss.str().c_str());
    }

    if (logFileStream.bad()) {
        log.AddString(str);
        log.SetCurSel(log.GetCount() - 1);
        logToFile = false;      // cannto handle
        this->UpdateData(false);
        return;
    }

    {
        ss.str("");
        boost::posix_time::ptime logime(boost::posix_time::second_clock::local_time()); 
        ss << "\n[" << logime << "] " << CT2A(str);
        logFileStream.write(ss.str().c_str(), ss.str().size());
        dirtyLog = true;
    }
}


void CCSMDlg::LogString(const CString& str) {
    if (logToFile) {
        this->LogToFile(str);
    }

	if (logToScreen == TRUE) {
		if (this->GetSafeHwnd() == NULL) return;    // Log starts earlier than window
		if (this->m_hWnd == NULL) return;
		if (log.m_hWnd == NULL) return;
		log.AddString(str);
		log.SetCurSel(log.GetCount() - 1);
	}
}

void CCSMDlg::LogString(const std::string str) {
    LogString(CString(str.c_str()));
}

void CCSMDlg::LogData(const std::string& str) {
    if (!logData) return;
    std::stringstream ss;
    ss << "[DATA]: " << str;  
    LogString(CString(ss.str().c_str()));
}

void CCSMDlg::OnBnClickedLogToFile()
{
    BOOL ret = this->UpdateData(true);

	if (logToFile == TRUE) {
		log.AddString(_T("Log to file enabled"));
	}
	else {
		log.AddString(_T("Log to file disabled"));
	}

    log.SetCurSel(log.GetCount() - 1);
}

void CCSMDlg::OnBnClickedClearLog()
{
    // Delete every other item from the list box.
    log.SetRedraw(FALSE);
    int count = log.GetCount();
    for (int i = count - 1; i >= 0; --i) {
       log.DeleteString(i);
    }
    log.SetRedraw(TRUE);
    log.Invalidate();
    log.UpdateWindow();
}


void CCSMDlg::OnBnClickedLogData()
{
    this->UpdateData(true);
}


void CCSMDlg::OnTimer(UINT_PTR nIDEvent)
{
    if (nIDEvent == LogRefreshTimer) {
        if (dirtyLog) {
            dirtyLog = false;
#ifdef _DEBUG
            logFileStream.flush();
#endif
        }
    }
	else if ( nIDEvent == DeviceStatusTimer) {
		int cmdApplyTypeSave = cmdApplyType;
		cmdApplyType = 2;		// a all refesh selection
		ExecuteSelectedDevices(&CCSMDlg::CurrentDeviceGetStatus);
		cmdApplyType = cmdApplyTypeSave;
	}

    __super::OnTimer(nIDEvent);
}


void CCSMDlg::OnBnClickedSendCmd() {
	ExecuteSelectedDevices(&CCSMDlg::CurrentDeviceSendCmd);
}

void CCSMDlg::CurrentDeviceSendCmd() {
	serial_com::SerialComConnectionInfo* curSerialCom = GetCurSerialComConnectionInfo();
    if (curSerialCom == NULL) {
        return;
    }

	if (curSerialCom->status != serial_com::SerialConnected) {
        this->LogString(std::string("not connected")); 		
	}

    BOOL ret = this->UpdateData(true);
    if (!ret) return;

    if (cmdIndex < 0 || cmdIndex >= (int)commands.size()) {
        this->LogString(std::string("Incorrect command ID")); 
        return;    
    }

    std::string cmdStr;
	if (commands[cmdIndex].name.empty() == false) {
		cmdStr = commands[cmdIndex].name + " ";
	}

	cmdStr += std::string(CT2A(cmdParam)) + " \r";
	SendHelper(curSerialCom, cmdStr);

	this->LogString(cmdStr); 	
    return;
}

void CCSMDlg::ConnectDevice(std::string portString, const FirmwareDeviceId id) {
    {
        std::stringstream ss;
        ss << "Opening port " << portString;
        this->LogString(ss.str());
    }

    std::string firmwareDeviceIdtr((const char*)id, sizeof(FirmwareDeviceId));

    // Find existing connection (can be closed)
    serial_com::SerialComConnectionInfo* existingConn = GetSerialComConnectionInfo(id);
    if (existingConn) {
        if (existingConn->status != serial_com::SerialClosed) {
		    this->LogString(std::string("The connection is not closed yet"));
            // return;
        }

        // reconnect closed one. Need to remove old object -- requirement of photon

	    serial_com::SharedSerialComConnectionInfo connectionInfo(boost::make_shared<serial_com::SerialComConnectionInfo>(portString, existingConn->blockId));
        this->serialComMessenger->AllocateConnectSessionId(connectionInfo);
        assert(connectionInfo->connectSessionId);
	    serialComConnectionInfoSet[connectionInfo->connectSessionId] = connectionInfo;

        connectionInfo->firmwareDeviceId = firmwareDeviceIdtr;
        connectionInfo->status = serial_com::SerialConnecting;
	    connectionInfo->endpoint.comPort = portString;

       this->serialComMessenger->AsyncConnect(connectionInfo->endpoint, connectionInfo->connectSessionId);

        auto it = serialComConnectionInfoSet.find(existingConn->connectSessionId);
        assert(it != serialComConnectionInfoSet.end());
        serialComConnectionInfoSet.erase(it);

		this->LogString(std::string("Re-opening connection"));

        RepopulateRow(connectionInfo.get());			// re-populate the new row associated with info object
        return;
    }

    assert(GetActiveSerialComConnectionInfo(portString) == NULL);       // should not happen

	CommContext blockId = deviceListCtrl.GetItemCount();

	serial_com::SharedSerialComConnectionInfo connectionInfo(boost::make_shared<serial_com::SerialComConnectionInfo>(portString, blockId));
    this->serialComMessenger->AllocateConnectSessionId(connectionInfo);
    assert(connectionInfo->connectSessionId);
	serialComConnectionInfoSet[connectionInfo->connectSessionId] = connectionInfo;

    connectionInfo->firmwareDeviceId = firmwareDeviceIdtr;
    connectionInfo->status = serial_com::SerialConnecting;
	connectionInfo->endpoint.comPort = portString;
    this->serialComMessenger->AsyncConnect(connectionInfo->endpoint, connectionInfo->connectSessionId);

    {
	    LVITEM lvi;
	    CString strItem;
	    lvi.iItem = blockId;
	    lvi.mask =  LVIF_TEXT;

	    lvi.iSubItem = COL_DEV_ID;
	    lvi.pszText = (LPTSTR)(LPCTSTR)(_T(""));
	    deviceListCtrl.InsertItem(&lvi);
    }

    RepopulateRow(connectionInfo.get());			// re-populate the new row associated with info object

    return;
}

void CCSMDlg::ParseProtocolSerialData(serial_com::SerialComConnectionInfo* serialCom, const std::string str) {
	assert(serialCom);

    std::string& serialBuffer = serialCom->serialBuffer;
	serialBuffer.append(str);
    DWORD signature = TK_FRAME_SIGNATURE;
    std::string signatureStr((char *)&signature, sizeof(signature));        // TODO!!! to constructor
    while(true) {
        size_t headerPos = serialBuffer.find(signatureStr);

        if (headerPos == std::string::npos) {
            return;     // not found
        }

        // Found, remove not complete last packet
        if (headerPos != 0) {
            serialBuffer = serialBuffer.substr(headerPos);                  
        }

        if (serialBuffer.size() < sizeof(USBDebugInfoHeader)) {
            return;
        }

        char* charBuffer = const_cast<char *>(serialBuffer.c_str());
        USBDebugInfoHeader* header =  reinterpret_cast<USBDebugInfoHeader*>(charBuffer);
        if (header->contentLength > serialBuffer.size() || header->contentLength < sizeof(USBDebugInfoHeader)) {
            return;
        }

        std::string body(charBuffer + sizeof(USBDebugInfoHeader), header->contentLength - sizeof(USBDebugInfoHeader));
        OnSerialPacket(serialCom, *header, body);
    
        // remove a packet;
        serialBuffer = serialBuffer.substr(header->contentLength);
    }
}

void CCSMDlg::OnSerialPacket(serial_com::SerialComConnectionInfo* connectionInfo, const USBDebugInfoHeader& header, std::string body) {
    if (header.contentType == DEV_RESP_INFO) {
        this->LogData(body);
        return;
    }
	else if (header.contentType == DEV_RESP_IMU_DATA) {
		// TODO!!! Use switch
		// TODO!!! re-factor. Beacon header file shared with beacon firmware 

		#pragma pack(push, 1)

		#define BC_NUM_OF_IMU_PKTS_IN_RF_PKT 5

		struct BK_IMUData {
			uint8_t  Timestamp;
  			uint16_t gyroscopeX;
			uint16_t gyroscopeY;
			uint16_t gyroscopeZ;
			uint16_t accelerationX;
			uint16_t accelerationY;
			uint16_t accelerationZ;
		};

		struct Beacon_Preamble {
			 uint8_t   Seq_Num;
			 uint8_t   button_pr;
			 uint8_t   BeaconRSSI;
			uint8_t   IRLed0;
			uint8_t   IRLed1;
			uint8_t   IRLed2;
			// uint16_t  mySrcAddr;        // extended for USB communication only but not for configuration version
			uint8_t   Battery_lev;
			uint8_t   SyncFrameIMU;
			uint8_t   MsTimerIMU;
			uint8_t   IMUPktNum;
		};
    
		struct Beacon_Data_pkt {
			struct Beacon_Preamble BK_Preamble;
			struct BK_IMUData BeaconIMUData[BC_NUM_OF_IMU_PKTS_IN_RF_PKT];
		};

		#pragma pack(pop)

		Beacon_Data_pkt* beaconPacket;
		beaconPacket = reinterpret_cast<Beacon_Data_pkt* >(const_cast<char*>(body.c_str()));


		TKBCIMUEthPacket imuPacket;
		for (int i = 0; i < BC_NUM_OF_IMU_PKTS_IN_RF_PKT; ++i) {
			memcpy(&(imuPacket.BC_IMUData.ImuData[i]), &(beaconPacket->BeaconIMUData[i]), sizeof(BK_IMUData));
			BCIMUData& data = imuPacket.BC_IMUData.ImuData[i];
			//LogIMUData(0, data);
		}	
	}
    else if (header.contentType == DEV_RESP_UPD || header.contentType == DEV_RESP_SET_PROD_AREA) {
        RespUpdate* respUpdate;
        respUpdate = reinterpret_cast<RespUpdate* >(const_cast<char*>(body.c_str()));
        if (respUpdate->errorCode == 0) {            
			{
				std::stringstream ss;
				ss << "Response: " << (int)respUpdate->index;

				int ctrlIndex = connectionInfo->blockId;

				LVITEM lvi;
				CString strItem;
				lvi.mask =  LVIF_TEXT;
				lvi.iItem = ctrlIndex;
				strItem.Format(_T("%s"), CA2T(ss.str().c_str()));
				lvi.iSubItem = COL_MEMO;
				lvi.pszText = (LPTSTR)(LPCTSTR)(strItem);
				deviceListCtrl.SetItem(&lvi);
			}

			int& currentFirmwareSegmentIndex = connectionInfo->currentFirmwareSegmentIndex;

            assert(respUpdate->index == currentFirmwareSegmentIndex);
            if (currentFirmwareSegmentIndex >= 0) {     // In an upload session
                ++currentFirmwareSegmentIndex;
                if (currentFirmwareSegmentIndex >= (int)firmwareSegments.size()) {
                    // Session is done
                    currentFirmwareSegmentIndex = -1;
 
					{
						std::stringstream ss;
						ss << "Upload done";

						int ctrlIndex = connectionInfo->blockId;

						LVITEM lvi;
						CString strItem;
						lvi.mask =  LVIF_TEXT;
						lvi.iItem = ctrlIndex;
						strItem.Format(_T("%s"), CA2T(ss.str().c_str()));
						lvi.iSubItem = COL_MEMO;
						lvi.pszText = (LPTSTR)(LPCTSTR)(strItem);
						deviceListCtrl.SetItem(&lvi);
					}
                }
                else {
                    UploadFirmwarePacketSegment(connectionInfo);
                }
            }
        }
        else {
            std::stringstream ss;
            ss << "Error updating firmware. Error code: " << (int)respUpdate->errorCode;
            this->LogString(ss.str());      
        }
        return;
    }

	else if (header.contentType == DEV_RESP_CONFIG) {
        connectionInfo->configBodyPacket = body;

		RepopulateRow(connectionInfo);
        RepopulateDeviceProperties();
        this->LogString(std::string("Received configuration"));   
	}

	else if (header.contentType == DEV_RESP_RUNNING_STATUS) {
		BeaconRunningStatus* runningStatus;
        runningStatus = reinterpret_cast<BeaconRunningStatus*>(const_cast<char*>(body.c_str()));

		if (runningStatus->errorCode != 0) {
			std::stringstream ss;
			ss << "Running status error: " << runningStatus->errorCode;
			this->LogString(ss.str());
			return;
		}

		BeaconRunningStatus* ds;
        ds = reinterpret_cast<BeaconRunningStatus*>(const_cast<char*>(body.c_str()));

		boost::optional<BeaconRunningStatus>& deviceStatus = connectionInfo->deviceStatus;
		deviceStatus = *ds;

		RepopulateRow(connectionInfo);		
	}
	else if (header.contentType == DEV_RESP_FIRMWARE_SUM) {
        std::stringstream ss;
        ss << "Received MD5 checksum";
        this->LogString(ss.str());

		RespFirmwareSum* sum;
        sum = reinterpret_cast<RespFirmwareSum*>(const_cast<char*>(body.c_str()));

		if (sum->errorCode != 0) {
			std::stringstream ss;
			ss << "Checking sum error: " << sum->errorCode;
			this->LogString(ss.str());
			return;
		}
		if (memcmp(sum->sum, md5Sum, sizeof(sum->sum)) == 0) {
			std::stringstream ss;
			ss << "Checking sum correct!";
			this->LogString(ss.str());
			return;		
		}
		else {
			std::stringstream ss;
			ss << "Checking sum incorrect!";
			this->LogString(ss.str());
			return;			
		}
	}
	else if (header.contentType == DEV_RESP_GET_EEPROM_DATA) {
		{
			std::stringstream ss;
			ss << "Received product information";
			this->LogString(ss.str());
		}

		RespEepromData* area;
        area = reinterpret_cast<RespEepromData*>(const_cast<char*>(body.c_str()));

		if (area->errorCode != 0) {
			std::stringstream ss;
			ss << "Get product information error: " << area->errorCode;
			this->LogString(ss.str());
			return;
		}

		connectionInfo->productInfo = std::string((const char*)&area->data[0], sizeof(area->data));

		RepopulateRow(connectionInfo);	
		RepopulateDeviceProperties();
		return;		
	}
	else if (header.contentType == DEV_RESP_WHOAMI) {
		{
			std::stringstream ss;
			ss << "Received device information";
			this->LogString(ss.str());
		}

		WhoAmI* who;
        who = reinterpret_cast<WhoAmI*>(const_cast<char*>(body.c_str()));
		boost::optional<WhoAmI>& whoAmI = connectionInfo->whoAmI;
		whoAmI = *who;

        {
            std::string usbId = ToUSBDeviceId(whoAmI->id);
			std::stringstream ss;
			ss << usbId;
			this->LogString(ss.str());  
        }

		// Auto jump
		if (whoAmI->module == 1 && jumpOnConnect == JUMP_TO_MAIN) {
			std::stringstream ss;
			ss << "B \r";

			SendHelper(connectionInfo, ss.str());
			this->LogString(std::string("Jump to main firmware"));  
		}
		else if ((whoAmI->module == 2 || (whoAmI->module == 3)) && jumpOnConnect == JUMP_TO_BOOTLOADER) {
			std::stringstream ss;
			ss << "! \r";

			SendHelper(connectionInfo, ss.str());
			this->LogString(std::string("Jump to boot loader"));  
		}

		{
			PacketHeader header;
			header.type = DEV_CMD_GET_VERSION;
			// No packet body
			header.size = sizeof(header);

			// construct packet
			std::string source;
			source.append(reinterpret_cast<char*>(&header), sizeof(header));

			SendPacket(connectionInfo, source);
		}

		// main firmware, check if need to turn off radio
		if (whoAmI->module == 2) {

			if (radioOnOnConnect == FALSE) {
				std::stringstream ss;
				ss << "t 6 \r";
				SendHelper(connectionInfo, ss.str());
				this->LogString(std::string("Turn off radio")); 
			}
			else {
				std::stringstream ss;
				ss << "t 7 \r";
				SendHelper(connectionInfo, ss.str());
				this->LogString(std::string("Turn on radio")); 
			}

			{
				PacketHeader header;
				header.type = DEV_CMD_CONFIG_REQ;
				// No packet body
				header.size = sizeof(header);

				// construct packet
				std::string source;
				source.append(reinterpret_cast<char*>(&header), sizeof(header));

				SendPacket(connectionInfo, source);
			}

			{	// Get manufacturer information
				PacketHeader header;
				header.type = DEV_CMD_GET_EEPROM_DATA;

				// No packet body
				header.size = sizeof(header);

				header.size = sizeof(header);
				std::string source;
				source.append(reinterpret_cast<char*>(&header), sizeof(header));

				SendPacket(connectionInfo, source);				
			}
		}

		RepopulateRow(connectionInfo);

		return;	
	}
	else if (header.contentType == DEV_RESP_VERSION) {

		RespFirmwareVersion* firmVersion;
        firmVersion = reinterpret_cast<RespFirmwareVersion*>(const_cast<char*>(body.c_str()));

		boost::optional<RespFirmwareVersion>& version = connectionInfo->version;
		version = *firmVersion;

		RepopulateRow(connectionInfo);	

		return;
	}
	else if (header.contentType == DEV_RESP_SET_EEPROM_DATA) {
		// Simply ignore
	}
	else if (header.contentType == DEV_RESP_BAT_STATUS) {
		Beacon_BatData* data;
        data = reinterpret_cast<Beacon_BatData*>(const_cast<char*>(body.c_str()));

		boost::optional<Beacon_BatData>& batteryStatus = connectionInfo->batteryStatus;
		batteryStatus = *data;

		RepopulateRow(connectionInfo);		
	}
	else {
        std::stringstream ss;
        ss << "Wrong response";
        this->LogString(ss.str());      
	}
}

void CCSMDlg::LogIMUData(int beaconID, const BCIMUData& imuData) {
//	std::ofstream ss("imu_data.log", std::ios::app);
	std::stringstream ss; 
	ss << std::uppercase;

	ss << std::setw(4) << beaconID << ",";
	ss << std::setw(4) << (int)imuData.Timestamp << ": ";
	ss << std::hex;

	ss << std::setfill('0') << std::setw(5) << (int)imuData.accelerationX << ", ";
	ss << std::setfill('0') << std::setw(5) << (int)imuData.accelerationY << ", ";
	ss << std::setfill('0') << std::setw(5) << (int)imuData.accelerationZ << ", ";
	ss << std::setfill('0') << std::setw(5) << (int)imuData.gyroscopeX << ", ";
	ss << std::setfill('0') << std::setw(5) << (int)imuData.gyroscopeY << ", ";
	ss << std::setfill('0') << std::setw(5) << (int)imuData.gyroscopeZ;
	this->LogData(ss.str()); 
}


void CCSMDlg::OnBnClickedBtnReadFw() {
    CFileDialog dlg(TRUE, "bin", "Firmware Binary File", OFN_HIDEREADONLY|OFN_FILEMUSTEXIST, 
	"Firmware Binary Files (*.bin)|*.bin|*.*||", this);
    dlg.m_ofn.lpstrTitle="Open Firmware Binary File";
 
    CString filename;
 
    if(dlg.DoModal() != IDOK) {
        return;
    }    

    filename = dlg.GetPathName(); // return full path and filename
	
    // Read in image

    TCHAR	szBuffer[MaxSegmentSize]; 

    CFile	myFile;

    int segmentCount = 0;

	bool hasUpload = false;
	// Check if there is any ongoing update
	for (SerialComConnectionInfoSet::iterator it = serialComConnectionInfoSet.begin(); it != serialComConnectionInfoSet.end(); ++it) {
		if (it->second->currentFirmwareSegmentIndex >= 0) {
			hasUpload = true;
		}
	}
	if (hasUpload == true) {
		this->LogString(CString("There are ongoing upload(s)"));
		return;
	}

	firmwareSegments.clear();
	encodedSegments.clear();
	verifyHeader.reset();

	MD5_CTX mdContext;
	MD5Init(&mdContext);

	int size = 0;
    try {
        if (myFile.Open(filename.GetBuffer(), CFile::modeRead) == FALSE) {
            this->LogString(CString("Failed reading file: ") + filename);
            return;
        }

        do {
            UINT nActual = 0; 
            nActual = myFile.Read( szBuffer, MaxSegmentSize); 
            if (nActual > 0) {
                assert(nActual <= MaxSegmentSize);
                std::string segment(szBuffer, nActual);
                firmwareSegments.push_back(segment);
                segmentCount++;
            }
			MD5Update(&mdContext, (unsigned char*)szBuffer, nActual);			// TODO!!! Unicode Compatible
			size += nActual;
            if (nActual < MaxSegmentSize) {         // last segment
                break;                
            }
        } while (true);

        myFile.Close();
    }
    catch(...) {
        firmwareSegments.clear();
		encodedSegments.clear();
        this->LogString(CString("Failed reading file: ") + filename);

        // use try again?
        if (myFile.m_hFile != CFile::hFileNull) {
            myFile.Close();
        }

        return;
    }
    
	CString file = filename.Mid(filename.ReverseFind('\\')+1);
	loadedFirmwareType = DetectFirmwareType(std::string(CT2A(file)));

	MD5Final(md5Sum, &mdContext);
	FirmwareVerifyPacketHeader header;
	header.address = 0x0800C000;
	header.count = size;
	verifyHeader = boost::optional<FirmwareVerifyPacketHeader>(header);

	// TODO Checksum

    {
        std::stringstream ss;
        ss << segmentCount << " segments read from " << CT2A(filename);
        this->LogString(ss.str()); 
    }

}

void CCSMDlg::OnBnClickedFwUploadEnable() {
    BOOL ret = this->UpdateData(true);
    if (!ret) return;
}


void CCSMDlg::OnBnClickedBtnUploadNow() {
    if (firmwareSegments.size() == 0) {
        this->LogString(std::string("Image is not loaded")); 
        return;    
    }

	ExecuteSelectedDevices(&CCSMDlg::CurrentDeviceUploadNow);
}

void CCSMDlg::CurrentDeviceUploadNow() {
	serial_com::SerialComConnectionInfo* curSerialCom = GetCurSerialComConnectionInfo();
    if (curSerialCom == NULL) {
        this->LogString(std::string("Please select a device")); 
        return;
    }

	if (curSerialCom->status != serial_com::SerialConnected) {
        this->LogString(std::string("USB Serial port is not open yet")); 
        return;
    }

	if (!curSerialCom->whoAmI || (curSerialCom->whoAmI->module != 1 && curSerialCom->whoAmI->module != 3)) {		// command for boot loader / bootloader upgrader only
		this->LogString(std::string("Command is for boot loader / bl_upgrader only")); 
		return;		
	}

	if ((flashUpdateType == 0) && (curSerialCom->whoAmI->type != loadedFirmwareType)) {		// firmware upload but firmware upload type is not correct
		this->LogString(std::string("Firmware is not for this device.")); 
		return;		
	}

    if (!this->UpdateData(true)) {
        return;
    }

	curSerialCom->currentFirmwareSegmentIndex = 0;
    UploadFirmwarePacketSegment(curSerialCom);

    return;        
}

void CCSMDlg::SendPacket(serial_com::SerialComConnectionInfo* serialCom, std::string source) {	// Encode and send a packet out
	assert(serialCom);

    // Encode Packet
    char target[MaxSegmentSize * 3 + 128];
    int sourceLength = source.length();
    int ret = b64_ntop((const unsigned char *)source.c_str(), sourceLength, target, sizeof(target));
	// encodedSegments.push_back(std::string(target, ret));
    // Send
    assert(ret != -1 && ret  <= sizeof(target));
    std::string param(target, ret);
    std::string cmdStr = "U 0 0 " + param + " \r";
	// encodedSegments.push_back(cmdStr);
    SendHelper(serialCom, cmdStr);
}

void CCSMDlg::UploadFirmwarePacketSegment(serial_com::SerialComConnectionInfo* serialCom) {
    // Construct Packet for next segment
    PacketHeader header;

	if (flashUpdateType == 0) {				// Upload firmware
		header.type = DEVCMD_UPD_PACKET;
	}
	else {									// Upload manufacturer info
		header.type = DEV_CMD_SET_PROD_AREA;		
	}

	int& currentFirmwareSegmentIndex = serialCom->currentFirmwareSegmentIndex;

    FirmwarePacketHeader firmwarePacketHeader;
    firmwarePacketHeader.count = firmwareSegments.size();
	firmwarePacketHeader.index = currentFirmwareSegmentIndex;

    std::string segement = firmwareSegments[currentFirmwareSegmentIndex];

    std::string firmwarePacketBody;
    firmwarePacketBody.append(reinterpret_cast<char*>(&firmwarePacketHeader), sizeof(firmwarePacketHeader));
    firmwarePacketBody.append(segement);

    header.size = sizeof(header) + firmwarePacketBody.size();
	if (currentFirmwareSegmentIndex + 1 != firmwareSegments.size()) {
		assert(header.size == 135);
	}
    std::string source;
    source.append(reinterpret_cast<char*>(&header), sizeof(header));
    source.append(firmwarePacketBody);

	SendPacket(serialCom, source);
}

void CCSMDlg::OnActivate(UINT nState, CWnd* pWndOther, BOOL bMinimized)
{
	__super::OnActivate(nState, pWndOther, bMinimized);

	// TODO: Add your message handler code here
}

afx_msg LRESULT CCSMDlg::OnDevicechange(WPARAM wParam, LPARAM lParam)
{
    if (m_com_manager != NULL) {
	    m_com_manager->update(wParam, lParam);
    }

	return 0;
}

void CCSMDlg::OnBnClickedGetConfig() {
	serial_com::SerialComConnectionInfo* curSerialCom = GetCurSerialComConnectionInfo();
    if (curSerialCom == NULL) {
        this->LogString(std::string("Please select a device")); 
        return;
    }

	if (curSerialCom->status != serial_com::SerialConnected) {
        this->LogString(std::string("USB Serial port is not open yet")); 
        return;
    }

	if (!curSerialCom->whoAmI || curSerialCom->whoAmI->module != 2) {		// command for main firmware only
        this->LogString(std::string("Command is for main firmware only")); 
        return;		
	}

    PacketHeader header;
    header.type = DEV_CMD_CONFIG_REQ;
	// No packet body
	header.size = sizeof(header);

	// construct packet
    std::string source;
    source.append(reinterpret_cast<char*>(&header), sizeof(header));

	SendPacket(curSerialCom, source);
}

void CCSMDlg::OnBnClickedStoreConfig() {
	serial_com::SerialComConnectionInfo* curSerialCom = GetCurSerialComConnectionInfo();
    if (curSerialCom == NULL) {
        this->LogString(std::string("Please select a device")); 
        return;
    }

    if (curSerialCom->status != serial_com::SerialConnected) {
        this->LogString(std::string("USB Serial port is not open yet")); 
        return;
    }

	if (!curSerialCom->whoAmI || curSerialCom->whoAmI->module != 2) {		// command for main firmware only
        this->LogString(std::string("Command is for main firmware only")); 
        return;		
	}

	boost::optional<WhoAmI>& whoAmI = curSerialCom->whoAmI;
	if (!whoAmI) {
        this->LogString(std::string("Device type unknown")); 
        return;		
	}

	if (!this->UpdateData(true)) {
        return;
    }

	if (numberOfBits < 0 || numberOfBits >= (int)bitArray.size()) {
		return;
	}

	boost::uint8_t frameBits = bitArray.at(numberOfBits);

    PacketHeader header;
    header.type = DEV_CMD_SET_CONFIG;
	std::string source;

	if (whoAmI->type == 3) {		// beacon
		header.size = sizeof(header) + sizeof(beacon::Config);

		// construct packet
		beacon::Config config;

		config.size = sizeof(config) - sizeof(config.size)- sizeof(config.checksum); 
		config.panId          = panID;
		config.mySrcAddr      = beaconID;
		config.tkDstAddr      = routerAddress;
		config.ledOnOffs      = ledOnOffset;
		config.ledOffOffs     = ledOffOffset;
		config.ledDAC         = 3840;
		config.rfChan         = GetWirelessChannelFromIndex(wirelessChannel);	// Dropdownlist Index to channel ID
		config.rfTimeSlot     = wirelessTimeSlot + 2;	// Dropdownlist Index to slot ID
		config.led0Index         = ledID1;
		config.led1Index         = ledID2;
		config.led2Index         = ledID3;

		auto data = m_mapping_information.GetLEDInfoByIndex(this->ledID1);
		if ( data == NULL) {
			this->LogString(std::string("LED 1 out of scope"));
			return;
		}
		config.led0IdPattern = data->bitPattern;
		config.led0Id = data->lcid;

		data = m_mapping_information.GetLEDInfoByIndex(this->ledID2);
		if ( data == NULL) {
			this->LogString(std::string("LED 2 out of scope"));
			return;
		}

		config.led1IdPattern = data->bitPattern;
		config.led1Id = data->lcid;

		data = m_mapping_information.GetLEDInfoByIndex(this->ledID3);
		if ( data == NULL) {
			this->LogString(std::string("LED 3 out of scope"));
			return;
		}

		config.led2IdPattern = data->bitPattern;
		config.led2Id = data->lcid;

		config.frameBits = frameBits;

		config.TestMode       = 0x00;
		config.TxLevel        = txPowerLevel + 1;	// Dropdownlist Index to power level

		uint8_t radioPacketFlags = 0;
		if (enableIMU == TRUE) radioPacketFlags |= RADIOPACKET_IMU;
		if (enableButton == TRUE) radioPacketFlags |= RADIOPACKET_BUTTONPRESS;
		if (enableBattery == TRUE) radioPacketFlags |= RADIOPACKET_BATTERY;
		config.radioPacketFlags = radioPacketFlags;

		source.append(reinterpret_cast<char*>(&header), sizeof(header));
		source.append(reinterpret_cast<char*>(&config), sizeof(beacon::Config));
	}
	else {
		assert(whoAmI->type == 1|| whoAmI->type == 2);

		header.size = sizeof(header) + sizeof(timekeeper::Config);

		// construct packet
		timekeeper::Config config;

		config.panId          = panID;
		config.mySrcAddr      = beaconID;
		config.rfChan         = GetWirelessChannelFromIndex(wirelessChannel);	// Dropdownlist Index to channel ID
		config.TxPower        = txPowerLevel + 1;	// Dropdownlist Index to power level
		config.TestMode       = 0x00;
		config.SyncOutEn	  = 0x01;
		config.u32IwdgResetEvents = GetHopChannelsParaValue(hopChannels);	

		std::string source;
		source.append(reinterpret_cast<char*>(&header), sizeof(header));
		source.append(reinterpret_cast<char*>(&config), sizeof(timekeeper::Config));
	}

	SendPacket(curSerialCom, source);
}

void CCSMDlg::SetLocalConfig(const beacon::Config& config) {
	panID = config.panId;
	beaconID = config.mySrcAddr;
	routerAddress = config.tkDstAddr;
	ledOnOffset = config.ledOnOffs;
	ledOffOffset = config.ledOffOffs;

	wirelessChannel = GetIndexFromWirelessChannel(config.rfChan);			// to Dropdownlist index
	wirelessTimeSlot = config.rfTimeSlot -2 ;
	ledID1 = config.led0Index;
	ledID2 = config.led1Index;
	ledID3 = config.led2Index;

    CString str;

    {
        auto data = m_mapping_information.GetLEDInfoByIndex(this->ledID1);
        if (data == NULL) {
            str = _T("EI");
        }
        else {
            if (data->bitPattern != config.led0IdPattern) {
                str.Format(_T("EP: %X"), config.led0IdPattern);
            }
            else if (data->lcid != config.led0Id) {
                str.Format(_T("EC: %d"), config.led0Id);
             
            }
            else {
                str.Format(_T("%X %d"), config.led0IdPattern, config.led0IdPattern);
            }
        }
        led1Pattern = str;
    }

    {
        auto data = m_mapping_information.GetLEDInfoByIndex(this->ledID2);
        if (data == NULL) {
            str = _T("EI");
        }
        else {
            if (data->bitPattern != config.led1IdPattern) {
                str.Format(_T("EP: %X"), config.led1IdPattern);
            }
            else if (data->lcid != config.led1Id) {
                str.Format(_T("EC: %d"), config.led1Id);
             
            }
            else {
                str.Format(_T("%X %d"), config.led1IdPattern, config.led1IdPattern);
            }
        }
        led2Pattern = str;
    }

    {
        auto data = m_mapping_information.GetLEDInfoByIndex(this->ledID3);
        if (data == NULL) {
            str = _T("EI");
        }
        else {
            if (data->bitPattern != config.led2IdPattern) {
                str.Format(_T("EP: %X"), config.led2IdPattern);
            }
            else if (data->lcid != config.led2Id) {
                str.Format(_T("EC: %d"), config.led2Id);
             
            }
            else {
                str.Format(_T("%X %d"), config.led2IdPattern, config.led2IdPattern);
            }
        }
        led3Pattern = str;
    }

	beaconNumBit = config.frameBits;

	txPowerLevel = config.TxLevel - 1;

	uint16_t radioPacketFlags = config.radioPacketFlags;
	if (radioPacketFlags & RADIOPACKET_IMU) {
		enableIMU = TRUE;
	}
	else {
		enableIMU = FALSE;
	}

	if (radioPacketFlags & RADIOPACKET_BUTTONPRESS) {
		enableButton = TRUE;
	}
	else {
		enableButton = FALSE;
	}

	if (radioPacketFlags & RADIOPACKET_BATTERY) {
		enableBattery = TRUE;
	}
	else {
		enableBattery = FALSE;
	}
}

void CCSMDlg::SetLocalConfig(const timekeeper::Config& config) {
	panID = config.panId;
	beaconID = config.mySrcAddr;
	routerAddress = config.mySrcAddr;
	wirelessChannel = GetIndexFromWirelessChannel(config.rfChan);			// to Dropdownlist index
	txPowerLevel = config.TxPower - 1;

	std::stringstream ss;
	ss << std::hex << config.u32IwdgResetEvents;
	hopChannels = CString(ss.str().c_str());
}

void CCSMDlg::OnBnClickedDefaultConfig() {
	

	beacon::Config config;

	SetLocalConfig(config);

	if (!this->UpdateData(false)) {
        return;
    }
}

void CCSMDlg::OnBnClickedGetStatus() {
	this->LogString(std::string("Refresh status")); 

	ExecuteSelectedDevices(&CCSMDlg::CurrentDeviceGetStatus);
}

void CCSMDlg::CurrentDeviceGetStatus() {
	serial_com::SerialComConnectionInfo* curSerialCom = GetCurSerialComConnectionInfo();
    if (curSerialCom == NULL) {
        // this->LogString(std::string("Please select a device")); 
        return;
    }

    if (curSerialCom->status != serial_com::SerialConnected) {
        // this->LogString(std::string("USB Serial port is not open yet")); 
        return;
    }

	if (!curSerialCom->whoAmI || curSerialCom->whoAmI->module != 2) {		// command for main firmware only
        // this->LogString(std::string("Command is for main firmware only")); 
        return;		
	}

	{
		PacketHeader header;
		header.type = DEV_CMD_RUNNING_STATUS_REQ;
		// No packet body
		header.size = sizeof(header);

		// construct packet
		std::string source;
		source.append(reinterpret_cast<char*>(&header), sizeof(header));

		SendPacket(curSerialCom, source);
	}

	if (curSerialCom->whoAmI->type == 3) {		// request battery status
		PacketHeader header;
		header.type = DEV_CMD_BAT_STATUS;
		// No packet body
		header.size = sizeof(header);

		// construct packet
		std::string source;
		source.append(reinterpret_cast<char*>(&header), sizeof(header));

		SendPacket(curSerialCom, source);
	}
}


void CCSMDlg::OnBnClickedFwVerify() {
	if (flashUpdateType == 0) {
		if (!verifyHeader) {
			this->LogString(std::string("Image is not loaded into memory for comparison yet")); 
			return;	
		}
		ExecuteSelectedDevices(&CCSMDlg::CurrentDeviceVerify);
	}
	else {
		CurrentDeviceVerify();
	}
}

void CCSMDlg::CurrentDeviceVerify() {
	serial_com::SerialComConnectionInfo* curSerialCom = GetCurSerialComConnectionInfo();
    if (curSerialCom == NULL) {
        this->LogString(std::string("Please select a device")); 
        return;
    }

    if (curSerialCom->status != serial_com::SerialConnected) {
        this->LogString(std::string("USB Serial port is not open yet")); 
        return;
    }

	if (flashUpdateType == 0) {		// Firmware verification
		if (!curSerialCom->whoAmI || (curSerialCom->whoAmI->module != 1 && curSerialCom->whoAmI->module != 3) ) {		// command for boot loader & bootloader upgrader only
			this->LogString(std::string("Command is for boot loader / bl-upgrader only")); 
			return;		
		}

		PacketHeader header;
		header.type = DEV_CMD_FIRMWARE_VERIFY;
		// No packet body
		header.size = sizeof(header);

		FirmwareVerifyPacketHeader vHeader = this->verifyHeader.get();

		header.size = sizeof(header) + sizeof(vHeader);

		std::string source;
		source.append(reinterpret_cast<char*>(&header), sizeof(header));
		source.append(reinterpret_cast<char*>(&vHeader), sizeof(vHeader));

		SendPacket(curSerialCom, source);
	}
}

float CCSMDlg::GetBatteryPercentage(boost::uint8_t battery) {			// Code from Bridge
	if (battery >= 225) return 100.0;
	if (battery <= 187) return 0.0;
	float p = (float)((1.0 * battery - 187.0) / (225.0 - 187.0) * 100.0);
	if (p > 100.0) p = 100;
	return p;
}

void CCSMDlg::addDevice(const std::basic_string<TCHAR>& device, const std::string& usbDeviceId) {
	{
		std::stringstream ss;
		ss << "Device " << device.c_str() << " : " << usbDeviceId << " added";
		this->LogString(ss.str());  
	}

    FirmwareDeviceId id;
    bool ret = ToFirmwareDeviceId(id, usbDeviceId);
	if (ret == false) {
		std::stringstream ss;
		ss << "Warning! unrecognized device " << device.c_str() << " : " << usbDeviceId;
		this->LogString(ss.str()); 

		RepopulateDeviceProperties();
		return;
	}

	this->ConnectDevice(device.c_str(), id);

	RepopulateDeviceProperties();
}

void CCSMDlg::removeDevice(const std::basic_string<TCHAR>& device, const std::string& usbDeviceId) {
	{
		std::stringstream ss;
		ss << "Device " << device.c_str() << " removed";
		this->LogString(ss.str());  
	}

	{
		std::stringstream ss;
		ss << "Closing " << device.c_str();
		this->LogString(ss.str());  
		serial_com::SerialComConnectionInfo* active = GetActiveSerialComConnectionInfo(device);
		if (active == NULL) {
			this->LogString(std::string("Device is closed already"));  			
		}
		CloseHelper(active);
	}
}

void CCSMDlg::OnBnClickedAutoJmpMain()
{
    this->UpdateData(true);
}


void CCSMDlg::OnBnClickedUpdateType()
{
    BOOL ret = this->UpdateData(true);
	if (ret == FALSE) {
		return;
	}
}

void CCSMDlg::OnBnClickedRadio2()
{
    BOOL ret = this->UpdateData(true);
	if (ret == FALSE) {
		return;
	}
}

LRESULT CCSMDlg::OnSerialComIO(WPARAM wParam, LPARAM lParam) {
    return serialComMessenger->OnSerialComIO(wParam, lParam);
}

void CCSMDlg::OnLvnItemchangedDeviceList(NMHDR *pNMHDR, LRESULT *pResult)
{
    LPNMLISTVIEW pNMListView = reinterpret_cast<LPNMLISTVIEW>(pNMHDR);

    if ((pNMListView->uChanged & LVIF_STATE) && (pNMListView->uNewState & LVNI_SELECTED)) { 
    //if ((pNMListView->uChanged & LVIF_STATE) && (pNMListView->uOldState & LVNI_SELECTED) && !(pNMListView->uNewState & LVNI_SELECTED)) { 
        UINT uSelectedCount = deviceListCtrl.GetSelectedCount();
        int  nItem = -1;

        if (uSelectedCount != 1) {
            currentDeviceIndex = -1;
            RepopulateDeviceProperties();
            *pResult = 0;
            return;
        }

        currentDeviceIndex = deviceListCtrl.GetNextItem(nItem, LVNI_SELECTED);
        RepopulateDeviceProperties();
		this->UpdateData(FALSE);
    }

    *pResult = 0;
}

serial_com::SerialComConnectionInfo* CCSMDlg::GetCurSerialComConnectionInfo() {
    for(SerialComConnectionInfoSet::iterator it = serialComConnectionInfoSet.begin(); it != serialComConnectionInfoSet.end(); ++it) {
		serial_com::SharedSerialComConnectionInfo& connectionInfo = it->second;
		if (connectionInfo->blockId == currentDeviceIndex) {
			return connectionInfo.get();
		}
	}

	return NULL;
}

void CCSMDlg::RepopulateDeviceList() {		// TODO!!! save config of each device
    deviceListCtrl.DeleteAllItems();

    int index = 0;
    for(SerialComConnectionInfoSet::iterator it = serialComConnectionInfoSet.begin(); it != serialComConnectionInfoSet.end(); ++it) {
		serial_com::SharedSerialComConnectionInfo& connectionInfo = it->second;

		connectionInfo->blockId = index;

		LVITEM lvi;
		CString strItem;
		lvi.iItem = index;
		lvi.mask =  LVIF_TEXT;

		lvi.iSubItem = COL_DEV_ID;
		lvi.pszText = (LPTSTR)(LPCTSTR)(_T(""));
		deviceListCtrl.InsertItem(&lvi);

		RepopulateRow(connectionInfo.get());

        ++index;
    }

	if(index > 0) {
		deviceListCtrl.SetItemState(0, LVIS_SELECTED, LVIS_SELECTED);
	}

	RepopulateDeviceProperties();
}

void CCSMDlg::RepopulateRow(serial_com::SerialComConnectionInfo* connectionInfo) {			// re-populate the row associated with info object
	int index = connectionInfo->blockId;
    LVITEM lvi;
    CString strItem;
    lvi.iItem = index;
    lvi.mask =  LVIF_TEXT;

	const boost::optional<WhoAmI>& whoAmI = connectionInfo->whoAmI;

	std::string type("Unkown");
	std::string firmwareType("Unkown");
	CString id("");

	if (whoAmI) {
		if (whoAmI->type == 1) {
			type = "Timekeeper";
		}
		else if (whoAmI->type == 2) {
			type = "Router";
		}
		else if (whoAmI->type == 3) {
			if (connectionInfo->configBodyPacket.empty() == false) {
				beacon::Config* config;
				config = reinterpret_cast<beacon::Config*>(const_cast<char*>(connectionInfo->configBodyPacket.c_str()));

				std::stringstream ss;
				ss << "beacon: " << (int)config->mySrcAddr;
				type = ss.str();
			}
			else {
				type = "beacon";
			}
		}

		if (whoAmI->module == 1) {
			firmwareType = "Boot Loader";
		}
		else if (whoAmI->module == 2) {
			firmwareType = "Main Firmware";		
		}
		else if (whoAmI->module == 3) {
			firmwareType = "Upgrader";				
		}

		id.Format(_T("...%02X%02X%02X%02X%02X"), whoAmI->id[7], whoAmI->id[8], whoAmI->id[9], whoAmI->id[10] ,whoAmI->id[11]);
	}

	lvi.iSubItem = COL_DEV_ID;
	lvi.pszText = (LPTSTR)(LPCTSTR)(id);
	deviceListCtrl.SetItem(&lvi);

	strItem.Format(_T("%s"), CA2T(type.c_str()));
	lvi.iSubItem = COL_DEV_TYPE;
	lvi.pszText = (LPTSTR)(LPCTSTR)(strItem);
	deviceListCtrl.SetItem(&lvi);

	strItem.Format(_T("%s"), CA2T(firmwareType.c_str()));
	lvi.iSubItem = COL_DEV_FIRMWARE_TYPE;
	lvi.pszText = (LPTSTR)(LPCTSTR)(strItem);
	deviceListCtrl.SetItem(&lvi);


	const boost::optional<RespFirmwareVersion>& version = connectionInfo->version;
	std::string versionStr("");
	if (version) {
		std::stringstream ss;
		if (IsVersionCompatible(*version) ==false) {
			ss << "NOT compatible ";
		}

		ss << (int)version->major << "." << (int)version->minor << "." << (int)version->revision << "." << (int)version->build << "  ";

		std::string strDate((const char*)&version->dateString[0]);
		std::string strTime((const char*)&version->timeString[0]);
		ss << std::string(strDate) << "  " << std::string(strTime);
		versionStr = ss.str();
	}

	strItem.Format(_T("%s"), CA2T(versionStr.c_str()));
    lvi.iSubItem =COL_FIRM_VERSION;
    lvi.pszText = (LPTSTR)(LPCTSTR)(strItem);
    deviceListCtrl.SetItem(&lvi); 

	strItem.Format(_T("%s"), CA2T(connectionInfo->endpoint.comPort.c_str()));
    lvi.iSubItem =COL_DEV_PORT;
    lvi.pszText = (LPTSTR)(LPCTSTR)(strItem);
    deviceListCtrl.SetItem(&lvi);    
        
	if (connectionInfo->status == serial_com::SerialClosing) {
		strItem.Format(_T("%s"), _T("Closing"));
	}
	else if (connectionInfo->status == serial_com::SerialConnecting) {
		strItem.Format(_T("%s"), _T("Connecting"));
	}
	else if (connectionInfo->status == serial_com::SerialConnected) {
		strItem.Format(_T("%s"), _T("Connected"));
	}
	else {
		strItem.Format(_T("%s"), _T("Closed"));
	}

	if (connectionInfo->batteryStatus) {
		std::stringstream ss;
		// ss << std::fixed << std::setw(5) << std::setprecision( 1 ) << GetBatteryPercentage((uint8_t)connectionInfo->deviceStatus->index);		// old frame format
		ss << std::fixed << std::setw(5) << (int)connectionInfo->batteryStatus->percents;
		strItem = strItem + _T(" ") + (CA2T(ss.str().c_str())) + _T("%");
	}

	if (connectionInfo->deviceStatus) {
		std::stringstream ss;
		ss << (connectionInfo->deviceStatus->radioOnOff == 0 ? "Off" : "On");
		strItem = strItem + _T(" ") + (CA2T(ss.str().c_str()));
	}

    lvi.iSubItem = COL_DEV_STATUS;
    lvi.pszText = (LPTSTR)(LPCTSTR)(strItem);
    deviceListCtrl.SetItem(&lvi);


	strItem.Format(_T("%d"), connectionInfo->msgReceived);
	lvi.iSubItem = COL_DEV_RECEIVED;
	lvi.pszText = (LPTSTR)(LPCTSTR)(strItem);
	deviceListCtrl.SetItem(&lvi);

	strItem.Format(_T("%d"), connectionInfo->msgSent);
	lvi.iSubItem = COL_DEV_SENT;
	lvi.pszText = (LPTSTR)(LPCTSTR)(strItem);
	deviceListCtrl.SetItem(&lvi);
}	

serial_com::SerialComConnectionInfo* CCSMDlg::GetActiveSerialComConnectionInfo(std::string port) {
	for (SerialComConnectionInfoSet::iterator it = serialComConnectionInfoSet.begin(); it != serialComConnectionInfoSet.end(); ++it) {
		serial_com::SharedSerialComConnectionInfo& connectionInfo = it->second;
		if (connectionInfo->endpoint.comPort == port && connectionInfo->status != serial_com::SerialClosed) {
			return connectionInfo.get();
		}
	}
	return NULL;
}

serial_com::SerialComConnectionInfo* CCSMDlg::GetSerialComConnectionInfo(const FirmwareDeviceId firmwareDeviceId) {
    std::string firmwareDeviceIdtr((const char*)firmwareDeviceId, sizeof(FirmwareDeviceId));
	for (SerialComConnectionInfoSet::iterator it = serialComConnectionInfoSet.begin(); it != serialComConnectionInfoSet.end(); ++it) {
		serial_com::SharedSerialComConnectionInfo& connectionInfo = it->second;
		if (connectionInfo->firmwareDeviceId == firmwareDeviceIdtr) {
			return connectionInfo.get();
		}
	}
	return NULL;
}

void CCSMDlg::OnSerialComConnect(serial_com::SharedSerialComConnectionInfo& connectionInfo) {
	connectionInfo->status = serial_com::SerialConnected;  // roll the state machine

    std::stringstream ss;
    ss << connectionInfo->name << " is connected";
    this->LogString(ss.str());

	RepopulateRow(connectionInfo.get());

	// Send "Who am I"
	{
		std::stringstream ss;
		ss << "t 8 \r";

		SendHelper(connectionInfo.get(), ss.str());
	}
	{
		std::stringstream ss;
		ss << "WhoAmI command sent to " << connectionInfo->name;
		this->LogString(ss.str());  
	}
}

void CCSMDlg::OnSerialComWrite(serial_com::SharedSerialComConnectionInfo& connectionInfo) {
	RepopulateRow(connectionInfo.get());
}

void CCSMDlg::OnSerialComReceive(serial_com::SharedSerialComConnectionInfo& connectionInfo, std::vector<unsigned char>& buffer) {
	RepopulateRow(connectionInfo.get());

	std::string str( buffer.begin(), buffer.end() );
	ParseProtocolSerialData(connectionInfo.get(), str);
}

void CCSMDlg::OnSerialComCloseConnection(serial_com::SharedSerialComConnectionInfo& connectionInfo, CommContext closeConnId, boost::system::error_code error) {
    if (error) {
		if (connectionInfo->status == serial_com::SerialConnecting) {
			std::stringstream ss;
			ss << "Failed connecting to " << connectionInfo->name;
			this->LogString(ss.str());
		}
    }

	RepopulateRow(connectionInfo.get());

	connectionInfo->status = serial_com::SerialClosed;  // roll the state machine

	for(SerialComConnectionInfoSet::iterator it = serialComConnectionInfoSet.begin(); it != serialComConnectionInfoSet.end(); ++it) {
		serial_com::SharedSerialComConnectionInfo& tempConnectionInfo = it->second;

		if (connectionInfo.get() == tempConnectionInfo.get()) {
			std::stringstream ss;
			ss << connectionInfo->name << " closed";
			this->LogString(ss.str());

			RepopulateRow(connectionInfo.get());
			break;
		}
	}
}

void CCSMDlg::SendHelper(serial_com::SerialComConnectionInfo* serialCom, std::string cmdStr) {
	std::vector<unsigned char> buffer(cmdStr.begin(), cmdStr.end());
    this->serialComMessenger->AsyncWrite(buffer, serialCom->connectSessionId, serialCom->writeId);
}

void CCSMDlg::CloseHelper(serial_com::SerialComConnectionInfo* serialCom) {
	serialCom->status = serial_com::SerialClosing;
	this->serialComMessenger->AsyncCloseConnection(serialCom->connectSessionId, false, 0);	
}

void CCSMDlg::ExecuteSelectedDevices(GUICommand guiCmd) {
	std::set<int> rows;

	if (cmdApplyType == 0 || cmdApplyType == 1) {
		UINT uSelectedCount = deviceListCtrl.GetSelectedCount();
		int  nItem = -1;
		for (UINT i = 0; i < uSelectedCount; i++)
		{
			nItem = deviceListCtrl.GetNextItem(nItem, LVNI_SELECTED);
			rows.insert(nItem);
		}
	}
	else {
		UINT count = deviceListCtrl.GetItemCount();
		for (UINT i = 0; i < count; i++)
		{
			rows.insert(i);
		}
	}

	int save = currentDeviceIndex;
	for (auto it = rows.begin(); it != rows.end(); ++it) {
		int row = *(it);
		currentDeviceIndex = *(it);
		(this->*(guiCmd))();
	}
	currentDeviceIndex = save;
}

void CCSMDlg::OnBnClickedCmdApplySingle()
{
    BOOL ret = this->UpdateData(true);
	RepopulateDeviceProperties();
}

void CCSMDlg::OnBnClickedCmdApplySelected()
{
    BOOL ret = this->UpdateData(true);
	RepopulateDeviceProperties();
}

void CCSMDlg::OnBnClickedCmdApplyAll()
{
    BOOL ret = this->UpdateData(true);
	RepopulateDeviceProperties();
}

CCSMDlg::FirmwareType CCSMDlg::DetectFirmwareType(std::string fileName) {

	std::string upFileName = fileName;
	boost::to_upper(upFileName);

	if (upFileName.compare(0, 2, "BC") == 0) return FIRM_BC;
	if (upFileName.compare(0, 2, "TK") == 0) return FIRM_TK;
	if (upFileName.compare(0, 2, "RT") == 0) return FIRM_RT;
	if (upFileName.compare(0, strlen("BOOTLOADER4TK"), "BOOTLOADER4TK") == 0) return FIRM_TK;
	if (upFileName.compare(0, strlen("BOOTLOADER4RT"), "BOOTLOADER4RT") == 0) return FIRM_RT;
	if (upFileName.compare(0, strlen("BOOTLOADER4BC"), "BOOTLOADER4BC") == 0) return FIRM_BC;
	if (upFileName.compare(0, strlen("BOOTLOADER4TIME"), "BOOTLOADER4TIME") == 0) return FIRM_TK;
	if (upFileName.compare(0, strlen("BOOTLOADER4ROUTER"),  "BOOTLOADER4ROUTER") == 0) return FIRM_RT;
	if (upFileName.compare(0, strlen("BOOTLOADER4BEACON"),  "BOOTLOADER4BEACON") == 0) return FIRM_BC;
	return FIRM_OTHER;
}

void CCSMDlg::OnCbnSelchangeJumpOnConnect() {
    BOOL ret = this->UpdateData(true);
	if (ret == FALSE) {
		return;
	}
}

void CCSMDlg::OnCbnSelchangeNumBit() {
    BOOL ret = this->UpdateData(true);
	if (ret == FALSE) {
		return;
	}

	ReloadLEDMapping();

	if (numberOfBits < 0 || numberOfBits >= (int)bitArray.size()) {
		return;
	}

	boost::uint8_t frameBits = bitArray.at(numberOfBits);

	CString str;
	str.Format(_T("Loaded %d bit LED pattern set"), (int)frameBits);
    this->LogString(str);

	RepopulateDeviceProperties();
}

void CCSMDlg::ReloadLEDMapping() {
	if (numberOfBits < 0 || numberOfBits >= (int)bitArray.size()) {
		return;
	}

	boost::uint8_t frameBits = bitArray.at(numberOfBits);
	m_mapping_information.Load(frameBits);	
}

afx_msg void CCSMDlg::OnBnClickedCheckRadioOn() {
    BOOL ret = this->UpdateData(true);
	if (ret == FALSE) {
		return;
	}	
}


void CCSMDlg::Save() {
    try {
        std::ofstream ofs(cfgFile.c_str());
        if (!ofs.good()) {
            LogString(std::string("error saving configuration file"));
            return;
        }

        PersistentSet persistentSet;

        persistentSet.logData = logData;
        persistentSet.logToFile = logToFile;

        persistentSet.cmdApplyType = cmdApplyType;
        persistentSet.radioOnOnConnect = radioOnOnConnect;
        persistentSet.jumpOnConnect = jumpOnConnect;

        persistentSet.productID = productID;
        persistentSet.serialNumber = serialNumber;


        persistentSet.ledID1 = ledID1;
        persistentSet.ledID2 = ledID2;
        persistentSet.ledID3 = ledID3;
        persistentSet.beaconID = beaconID;
        persistentSet.routerAddress = routerAddress;
        persistentSet.ledOnOffset = ledOnOffset;    
        persistentSet.ledOffOffset = ledOffOffset;
        persistentSet.wirelessChannel = wirelessChannel;
        persistentSet.wirelessTimeSlot = wirelessTimeSlot;
        persistentSet.txPowerLevel = txPowerLevel;
        persistentSet.panID = panID;

        persistentSet.numberOfBits = numberOfBits;

        ofs << BTPRJ_HEAD << "$";     // add a version header

        boost::archive::text_oarchive oa(ofs);
        oa << persistentSet;
    }
    catch(...) {
        LogString(std::string("error saving configuration file"));
        return;
    }
}

void CCSMDlg::Load() {
    try {
        std::ifstream ifs(cfgFile.c_str());

        if (!ifs.good()) {
            return;         // will use default values only
        }

        std::stringbuf sb;
        ifs.get(sb, '$');
        std::string head = sb.str();
        if (head != BTPRJ_HEAD) {
            LogString(std::string("Cannot load configuration file"));
            return;
        }
        char ch = ifs.get();
        boost::archive::text_iarchive ia(ifs);
        PersistentSet persistentSet;
        ia >> persistentSet;

        logData = persistentSet.logData;
        logToFile = persistentSet.logToFile;

        cmdApplyType = persistentSet.cmdApplyType;
        radioOnOnConnect = persistentSet.radioOnOnConnect;
        jumpOnConnect = persistentSet.jumpOnConnect;

		productID = CString(persistentSet.productID.c_str());
        serialNumber = CString(persistentSet.serialNumber.c_str());
        ledID1 = persistentSet.ledID1;
        ledID2 = persistentSet.ledID2;
        ledID3 = persistentSet.ledID3;
        beaconID = persistentSet.beaconID;
        routerAddress = persistentSet.routerAddress;
        ledOnOffset = persistentSet.ledOnOffset;    
        ledOffOffset = persistentSet.ledOffOffset;
        wirelessChannel = persistentSet.wirelessChannel;
        wirelessTimeSlot = persistentSet.wirelessTimeSlot;
        txPowerLevel = persistentSet.txPowerLevel;
        panID = persistentSet.panID;

        numberOfBits = persistentSet.numberOfBits;
    }
    catch(...) {
        LogString(std::string("Cannot load configuration file"));
        return;
    }

    BOOL ret = this->UpdateData(false);
}

std::string CCSMDlg::ToUSBDeviceId(const FirmwareDeviceId id) {
    size_t idSize = sizeof(FirmwareDeviceId);

    CString formatId;
    for (size_t i  = 0; i < idSize; ++i) {
        CString formatByte;
        formatByte.Format(_T("%02X"), id[i]);  
        formatId.Append(formatByte);
    }
 
    return std::string(CT2A(formatId));
}

bool CCSMDlg::ToFirmwareDeviceId(FirmwareDeviceId id, const std::string& usbDeviceId) {
    size_t idSize = sizeof(FirmwareDeviceId);

    size_t pos = usbDeviceId.find_last_of('\\');
    if (pos == std::string::npos) {
        return false;
    }

    std::string deviceIdStr = usbDeviceId.substr(pos + 1);
    
    if (deviceIdStr.size() != idSize * 2) {
        return false;
    }

    static char xlate[] = "0123456789ABCDEF";

    for (size_t i = 0; i < idSize; ++i) {
        char char1 = deviceIdStr.at(i * 2);
        char char2 = deviceIdStr.at(i * 2 + 1);
        id[i] = ((strchr(xlate, char1) - xlate) * 16) + ((strchr(xlate, char2) - xlate));
    }

    return true;
}


void CCSMDlg::OnCbnSelchangeCmdList()
{
    BOOL ret = this->UpdateData(true);
}


void CCSMDlg::PopulateProductInfo(serial_com::SerialComConnectionInfo* connectInfo) {
	assert(connectInfo);

	if (connectInfo->productInfo.size() != sizeof(((RespEepromData *)0)->data)) {
		productID = "";
		serialNumber = "";
	}
	else {
		productID = CString(connectInfo->productInfo.substr(0, SizeProductID).c_str());
		serialNumber = CString(connectInfo->productInfo.substr(0 + SizeProductID, SizeSerialNum).c_str());
	}
}

boost::uint8_t CCSMDlg::GetWirelessChannelFromIndex(int index) {
	//assert(index >= 0 && index <= (26 - 11));
	return index + 11;
}

int CCSMDlg::GetIndexFromWirelessChannel(boost::uint8_t channel) {
	//assert(channel >= 11 && channel <= 26);
	if (channel < 11 && channel > 26) {
		CString str;

		str.Format(_T("Incorrect channel: %d. Default channel 11 is used"), (int)channel);
		this->LogString(str);
		return 0;
	}
	return channel - 11;
}

std::string CCSMDlg::GetHopChannelsParaStr(CString hopChannels) {
	std::stringstream ss;
	ss << GetHopChannelsParaValue(hopChannels);

	return ss.str();
}

boost::uint32_t CCSMDlg::GetHopChannelsParaValue(CString hopChannels) {
	boost::uint32_t sum = 0;

	if (hopChannels.GetLength() != 4) {
		goto ErrorOut;
	}

	hopChannels = hopChannels.MakeUpper();

	for (int i = 0; i < hopChannels.GetLength(); ++i) {
		char c = hopChannels[i];
		if (c >= '0' && c <= '9') {
			sum = c - '0' + sum * 16;		
		}
		else if (c >= 'A' && c <= 'F') {
			sum = c - 'A' + 10 + sum * 16;	
		}
		else {
			goto ErrorOut;
		}
	}

	return sum;

ErrorOut:
	this->LogString(CString(_T("Incorrect hopping channels. Set to default 0000")));
	return 0;
}

std::vector<int> CCSMDlg::GetFirmwareVersion(std::string& firmVersionStr) {
	unsigned pos = firmVersionStr.find(" "); 
	size_t len = firmVersionStr.size();
	if (pos != std::string::npos) {
		len  = pos;
	}
	std::string VersionStr = firmVersionStr.substr(0, len); 

    boost::char_separator<char> sep("."); 
	typedef boost::tokenizer<boost::char_separator<char> >Tokenizer;

	Tokenizer tok(VersionStr, sep);

	std::vector<int> verSet;

	using boost::bad_lexical_cast;

	int count = 0;

	for (auto it = tok.begin(); it != tok.end(); ++it) {
		
		int ver = 0;
		try {
			ver = boost::lexical_cast<boost::int32_t>(*it);
		}
        catch(bad_lexical_cast &) {
			verSet.push_back(ver);
        }

		++count;

		if (count >= 4) {
			break;
		}
	}

	return verSet;
}

bool CCSMDlg::IsVersionCompatible(const RespFirmwareVersion& firmVersion) {
	assert(appVerVector.size() == 4);
	if (appVerVector.at(0) > firmVersion.major) return true;
	if (appVerVector.at(0) < firmVersion.major) return false;

	if (appVerVector.at(1) > firmVersion.minor) return true;
	if (appVerVector.at(1) < firmVersion.minor) return false;

	if (appVerVector.at(2) > firmVersion.revision) return true;
	if (appVerVector.at(2) < firmVersion.revision) return false;

	return true;
}

void CCSMDlg::OnBnClickedLogToScreen() {
    BOOL ret = this->UpdateData(true);
}


void CCSMDlg::OnBnClickedFlushBtn() {
	logFileStream.flush();
}
