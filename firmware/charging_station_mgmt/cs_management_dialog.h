#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <vector>

#include <boost/make_shared.hpp>
#include <boost/scoped_ptr.hpp>

#include <common/packets.h>
#include "afxwin.h"
#include "Dbt.h"
#include "ComDeviceManager.h"
#include "win_serial_com_messenger.h"
#include <btcommon\led_mapping.h>

struct PersistentSet {
    BOOL logData;
    BOOL logToFile;
	BOOL logToScreen;

	int cmdApplyType;
	BOOL radioOnOnConnect;
	int jumpOnConnect;

	std::string productID;			// now is product info
	std::string serialNumber;
	int ledID1;
	int ledID2;
	int ledID3;
	int beaconID;
	int routerAddress;
	int ledOnOffset;
	int ledOffOffset;
	int wirelessChannel;
	int wirelessTimeSlot;
	int txPowerLevel;
	int panID;

	int numberOfBits;		// BlackTrax supporting pattern bits are 8, 12, 14 and 16, according indexes are 0, 1, 2 and 3

    template<class Archive> void serialize(Archive & ar, unsigned int version) {
        ar & logData;
        ar & logToFile;
		ar & logToScreen;

	    ar & cmdApplyType;
	    ar & radioOnOnConnect;
	    ar & jumpOnConnect;

	    ar & productID;
	    ar & serialNumber;
	    ar & ledID1;
	    ar & ledID2;
	    ar & ledID3;
	    ar & beaconID;
	    ar & routerAddress;
	    ar & ledOnOffset;
	    ar & ledOffOffset;
	    ar & wirelessChannel;
	    ar & wirelessTimeSlot;
	    ar & txPowerLevel;
	    ar & panID;

        ar & numberOfBits;
    }

};

// CCSMDlg dialog
class CCSMDlg : public CDialog, public ComDeviceObserver
{
    friend class serial_com::MessengerHook<CCSMDlg>;
	friend class ConcreteSerialComMessenger;
public:
	CCSMDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	enum { IDD = IDD_CSM_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support

public:
    void LogString(const CString& str);
    void LogString(const std::string str);
    void LogData(const std::string& str);

public:
	virtual void addDevice(const std::basic_string<TCHAR>& device, const std::string& usbDeviceId);
	virtual void removeDevice(const std::basic_string<TCHAR>& device, const std::string& usbDeviceId);

private:
    std::ofstream logFileStream;
    void LogToFile(const CString& str);
    bool dirtyLog;
    static const int LogRefreshTimer = 1;
	LEDMappingInfo m_mapping_information;
protected:
    BOOL logToFile;
    BOOL logData;
	BOOL logToScreen;
    CListBox log;

    virtual void OnCancel();
    virtual void OnOK();

    afx_msg void OnBnClickedLogToFile();
    afx_msg void OnBnClickedClearLog();
    afx_msg void OnTimer(UINT_PTR nIDEvent);
    afx_msg void OnBnClickedLogData();
// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()

protected:
	void IMUPacketCallback(TKBCIMUEthPacket * packet);
	void LogIMUData(int beaconID, const BCIMUData& imuData);

// Serial port communication facilities
protected:
	// Create a concrete type from template, paramter is the message loop window
	typedef SerialCom::WinMessenger<CCSMDlg> ConcreteSerialComMessenger;
	boost::scoped_ptr<ConcreteSerialComMessenger> serialComMessenger;

    LRESULT OnSerialComIO(WPARAM wParam, LPARAM lparam);
    void OnSerialComConnect(serial_com::SharedSerialComConnectionInfo& connectionInfo);
    void OnSerialComWrite(serial_com::SharedSerialComConnectionInfo& connectionInfo);
    void OnSerialComReceive(serial_com::SharedSerialComConnectionInfo& connectionInfo, std::vector<unsigned char>& buffer);
	void OnSerialComCloseConnection(serial_com::SharedSerialComConnectionInfo& connectionInfo, CommContext closeConnId, boost::system::error_code error);

	// helpers
	boost::uint8_t GetWirelessChannelFromIndex(int index);		// dropdown index to
	int GetIndexFromWirelessChannel(boost::uint8_t channel);

private:
    int writtenCount;
	DEV_BROADCAST_DEVICEINTERFACE filtr;
	HDEVNOTIFY notify;

public:
    afx_msg void OnBnClickedSendCmd();
    CString cmdParam;
    CComboBox commandListCtl;

    struct Command {        // must be copiable
        std::string name;
        std::string desc;
    };
    typedef std::vector<Command> Commands;
    Commands commands;
    int cmdIndex;

    // firmware image
	enum FirmwareType{			// should move to packets.h
		FIRM_OTHER = 0,			// other  like manufacturer info -- to be extended
		FIRM_TK = 1,
		FIRM_RT = 2,
		FIRM_BC = 3,
	};

	FirmwareType loadedFirmwareType;			// the file's type
	FirmwareType DetectFirmwareType(std::string);

    typedef std::vector<std::string> FirmwareSegments;
    FirmwareSegments firmwareSegments;
	FirmwareSegments encodedSegments;

	boost::optional<FirmwareVerifyPacketHeader> verifyHeader;		// If firmware is loaded, this variable is not null
	unsigned char md5Sum[16];										// MD5 sum of firmware

    static const int MaxSegmentSize = 128;
	static const int MaxPacketBuffer = 2048;

	static const int SizeProductID = 12;		// fixed size product ID string (info)
	static const int SizeSerialNum = 12;		// fixed size serial number string

    afx_msg void OnBnClickedBtnReadFw();

private:
    BOOL firmwareUploadEnabled;

protected:
    afx_msg void OnBnClickedFwUploadEnable();
    afx_msg void OnBnClickedBtnUploadNow();
	afx_msg void OnActivate(UINT nState, CWnd* pWndOther, BOOL bMinimized);
	afx_msg LRESULT OnDevicechange(WPARAM wParam, LPARAM lParam);
	afx_msg void OnBnClickedGetConfig();
	afx_msg void OnBnClickedStoreConfig();
	afx_msg void OnBnClickedDefaultConfig();

protected:
	CString productID;
	CString serialNumber;
	int ledID1;
	int ledID2;
	int ledID3;
	CString led1Pattern;
	CString led2Pattern;
	CString led3Pattern;

	int beaconID;
	int routerAddress;
	int ledOnOffset;
	int ledOffOffset;
	int wirelessChannel;
	int wirelessTimeSlot;
	int txPowerLevel;
	int panID;
	BOOL enableIMU;
	BOOL enableButton;
	BOOL enableBattery;

protected:
	void RepopulateDeviceProperties();		// Current
	void SetLocalConfig(const beacon::Config& config);
	void SetLocalConfig(const timekeeper::Config& config);
	void PopulateProductInfo(serial_com::SerialComConnectionInfo*);

	float GetBatteryPercentage(boost::uint8_t battery);		// Code from Bridge
public:
	afx_msg void OnBnClickedGetStatus();
	afx_msg void OnBnClickedFwVerify();
	CEdit batteryStatus;
	afx_msg void OnBnClickedBtSetPanid();
	afx_msg void OnBnClickedBtSetBeaconid();
	afx_msg void OnBnClickedBtLedonOffset();
	afx_msg void OnBnClickedBtLedoffOffset();
	afx_msg void OnBnClickedBtSetRfChannel();
	afx_msg void OnBnClickedBtSetTimeslot();
	afx_msg void OnBnClickedBtSetPower();

	CEdit statusEnabled;

	boost::scoped_ptr<ComDeviceManager> m_com_manager;
	afx_msg void OnBnClickedAutoJmpMain();
protected:
	int jumpOnConnect;					
	enum {NO_JUMP = 0, JUMP_TO_BOOTLOADER = 1, JUMP_TO_MAIN = 2};
    afx_msg void OnCbnSelchangeJumpOnConnect();
    afx_msg void OnCbnSelchangeNumBit();
	int flashUpdateType;
public:
	afx_msg void OnBnClickedUpdateType();
	afx_msg void OnBnClickedRadio2();

// Device List 
protected:
    afx_msg void OnLvnItemchangedDeviceList(NMHDR *pNMHDR, LRESULT *pResult);
    CListCtrl deviceListCtrl;
    enum {
            COL_DEV_ID = 0
          , COL_DEV_TYPE
          , COL_DEV_PORT
          , COL_DEV_STATUS
		  , COL_DEV_FIRMWARE_TYPE
		  , COL_FIRM_VERSION
          , COL_DEV_SENT
          , COL_DEV_RECEIVED
		  , COL_MEMO
    };
    int currentDeviceIndex;

    typedef uint8_t FirmwareDeviceId[12];       // move to packets.h

    std::string ToUSBDeviceId(const FirmwareDeviceId deviceId);
    bool ToFirmwareDeviceId(FirmwareDeviceId, const std::string& usbDeviceId);

	// Quick prototype
	// Each session occupy one line. Index is session id
	typedef std::map<CommContext, serial_com::SharedSerialComConnectionInfo> SerialComConnectionInfoSet;
	SerialComConnectionInfoSet serialComConnectionInfoSet;

	serial_com::SerialComConnectionInfo* GetCurSerialComConnectionInfo();
	void RepopulateRow(serial_com::SerialComConnectionInfo* info);			// re-populate the row associated with info object
	void RepopulateDeviceList();        // not using for the moment

	serial_com::SerialComConnectionInfo* GetActiveSerialComConnectionInfo(std::string port);	// Current active (not closed) one
	serial_com::SerialComConnectionInfo* GetSerialComConnectionInfo(const FirmwareDeviceId);

	void SendHelper(serial_com::SerialComConnectionInfo* serialCom, std::string cmdStr);
	void CloseHelper(serial_com::SerialComConnectionInfo* serialCom);
    void ParseProtocolSerialData(serial_com::SerialComConnectionInfo* serialCom, const std::string str);
    void OnSerialPacket(serial_com::SerialComConnectionInfo* serialCom, const USBDebugInfoHeader& header, std::string body);
	void SendPacket(serial_com::SerialComConnectionInfo* serialCom, std::string source);	// Encode and send a packet out	
    void UploadFirmwarePacketSegment(serial_com::SerialComConnectionInfo* serialCom);

	// For sending commands in single / batch
	typedef afx_msg void (CCSMDlg::*GUICommand)();
	void ExecuteSelectedDevices(GUICommand guiCmd);
	// helpers for single device
	void CurrentDeviceSendCmd();
	void CurrentDeviceUploadNow();
	void CurrentDeviceVerify();		// firmware only.not for manufacturer info
	void CurrentDeviceGetStatus();

	// For set config in single / batch. dedicated for config commands. They are too simular
	typedef std::string (CCSMDlg::*GetCfgString)(uint8_t deviceType);
	void ConfigSelectedDevices(GetCfgString getCfgString);		// Batch
	void ConfigSelectedDevice(GetCfgString getCfgString);		// Single

	// helpers for single device
	std::string CurrentDevicePanIDCfgString(uint8_t deviceType);		// device type: See Packets.h WhoAmI
	std::string CurrentDeviceChannelCfgString(uint8_t deviceType);
	std::string CurrentDevicePowerCfgString(uint8_t deviceType);
	std::string CurrentDeviceRouterAddrCfgString(uint8_t deviceType);
	std::string CurrentDeviceLedOnOffsetCfgString(uint8_t deviceType);
	std::string CurrentDeviceLedOffOffsetCfgString(uint8_t deviceType);
	std::string CurrentDeviceLed1IdCfgString(uint8_t deviceType);
	std::string CurrentDeviceLed2IdCfgString(uint8_t deviceType);
	std::string CurrentDeviceLed3IdCfgString(uint8_t deviceType);
	std::string CurrentDeviceBeaconIdCfgString(uint8_t deviceType);
	std::string CurrentDeviceTimeslotCfgString(uint8_t deviceType);

	std::string CurrentDeviceCheckEnableImuString(uint8_t deviceType);
	std::string CurrentDeviceCheckEnableBtryString(uint8_t deviceType);
	std::string CurrentDeviceCheckEnableBtnString(uint8_t deviceType);

	std::string CurrentDeviceHoppingChannelCfgString(uint8_t deviceType);

	// for product info
	std::string CurrentDeviceProductInfoString(uint8_t deviceType);
	void ProductConfigSelectedDevices(GetCfgString getCfgString);		// Batch   TODO!!! unify

public:
	afx_msg void OnBnClickedCmdApplySelected();
	afx_msg void OnBnClickedCmdApplyAll();
protected:
	int cmdApplyType;
	BOOL radioOnOnConnect;
	afx_msg void OnBnClickedCheckRadioOn();

    static const int DeviceStatusTimer = 2;			// device status timer

    void ConnectDevice(std::string devicePort, const FirmwareDeviceId id);
public:
	afx_msg void OnBnClickedSetProductId();
	afx_msg void OnBnClickedSetRouterAddr();
	afx_msg void OnBnClickedSetLed1id();
	afx_msg void OnBnClickedLed2id();
	afx_msg void OnBnClickedLed3id();
	afx_msg void OnBnClickedCmdApplySingle();

protected: // serialization
    static std::string BTPRJ_HEAD;      // define data serialization version
    static std::string cfgFile;
    void Save();
    void Load();

	CButton ctlGetConfig;
	CButton ctlStoreConfig;
	CButton ctlDefaultConfig;

	CEdit ctlProductID;
	CEdit ctlSerialNum;
	CEdit ctlRouterAddr;
	CEdit ctlChannel;
	CEdit ctlTxLevel;
	CEdit ctlPanID;
	CEdit ctlLEDOnOffset;
	CEdit ctlLEDOffOffset;
	CEdit ctlLED1ID;
	CEdit ctlLED2ID;
	CEdit ctlLED3ID;
	CEdit ctlLED1Pattern;
	CEdit ctlLED2Pattern;
	CEdit ctlLED3Pattern;

	CEdit ctlBeaconID;
	CEdit ctlTimeSlot;
	CEdit ctlBit;

	CEdit ctlSetProductID;
	CEdit ctlSetRouterAddr;
	CEdit ctlSetChannel;
	CEdit ctlSetTxLevel;
	CEdit ctlSetPanID;
	CEdit ctlSetLEDOnOffset;
	CEdit ctlSetLEDOffOffset;
	CEdit ctlSetLED1ID;
	CEdit ctlSetLED2ID;
	CEdit ctlSetLED3ID;
	CEdit ctlSetBeaconID;
	CEdit ctlSetTimeSlot;

	CButton ctrlEnableIMU;
	CButton ctrlEnableButton;
	CButton ctrlEnableBattery;
public:
	afx_msg void OnCbnSelchangeCmdList();
	afx_msg void OnBnClickedCheck1();
	afx_msg void OnBnClickedCheckEnableImu();
	afx_msg void OnBnClickedCheckEnableBtry();
	afx_msg void OnBnClickedCheckEnableBtn();
protected:
	CString currentDeviceID;
private:
	int numberOfBits;
	std::vector<int> bitArray;

	void ReloadLEDMapping();
public:
	int beaconNumBit;
	CEdit ctlBeaconNumBit;
	afx_msg void OnBnClickedBtSetHopCh();
protected:
	CString hopChannels;

	std::string GetHopChannelsParaStr(CString hopChannels);
	boost::uint32_t GetHopChannelsParaValue(CString hopChannels);

protected:
	std::vector<int> GetFirmwareVersion(std::string& versionStr);

	bool IsVersionCompatible(const RespFirmwareVersion& firmVersion);

	std::vector<int> appVerVector;
public:
	afx_msg void OnBnClickedLogToScreen();
	afx_msg void OnBnClickedFlushBtn();
};
