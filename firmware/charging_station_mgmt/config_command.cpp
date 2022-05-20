#include "stdafx.h"

#include <assert.h>
#include <sstream>

#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/date_time/local_time/local_time.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>

#include "cs_management.h"
#include "cs_management_dialog.h"
#include "afxdialogex.h"

#include <common/md5.h>
#include <common/base64.h>


#ifdef _DEBUG
#define new DEBUG_NEW
#endif

void CCSMDlg::ConfigSelectedDevice(GetCfgString getCfgString) {		// Single
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

    BOOL ret = this->UpdateData(true);

    if (!ret) return;

	std::string cmdStr = (this->*(getCfgString))(curSerialCom->whoAmI->type);

	if (cmdStr.empty() == false) {
		SendHelper(curSerialCom, cmdStr);
		// save
		{
			std::stringstream ss;
			ss << "v \r";

			SendHelper(curSerialCom, ss.str());
			this->LogString(ss.str());  
		}
	}

	{
		PacketHeader header;
		header.type = DEV_CMD_CONFIG_REQ;
		// No packet body
		header.size = sizeof(header);

		// construct packet
		std::string source;
		source.append(reinterpret_cast<char*>(&header), sizeof(header));

		SendPacket(curSerialCom, source);
	}
}

void CCSMDlg::ConfigSelectedDevices(GetCfgString getCfgString) {
	BOOL ret = this->UpdateData(true);
	if (!ret) return;

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


		serial_com::SerialComConnectionInfo* curSerialCom = GetCurSerialComConnectionInfo();
		if (curSerialCom == NULL) {
			continue;
		}

		if (curSerialCom->status != serial_com::SerialConnected) {
			continue;
		}

		if (!curSerialCom->whoAmI || curSerialCom->whoAmI->module != 2) {		// command for main firmware only		
			continue;		
		}

		std::string cmdStr = (this->*(getCfgString))(curSerialCom->whoAmI->type);

		if (cmdStr.empty() == false) {
			SendHelper(curSerialCom, cmdStr);
			// save
			{
				std::stringstream ss;
				ss << "v \r";

				SendHelper(curSerialCom, ss.str());
				this->LogString(ss.str());  
			}
		}

		{
			PacketHeader header;
			header.type = DEV_CMD_CONFIG_REQ;
			// No packet body
			header.size = sizeof(header);

			// construct packet
			std::string source;
			source.append(reinterpret_cast<char*>(&header), sizeof(header));

			SendPacket(curSerialCom, source);
		}
	}
	currentDeviceIndex = save;
}

// =========================  Config commands  =========================

void CCSMDlg::OnBnClickedBtSetPanid() {
	ConfigSelectedDevices(&CCSMDlg::CurrentDevicePanIDCfgString);
}

void CCSMDlg::OnBnClickedBtSetRfChannel() {
	ConfigSelectedDevices(&CCSMDlg::CurrentDeviceChannelCfgString);
}

void CCSMDlg::OnBnClickedBtSetPower() {
	ConfigSelectedDevices(&CCSMDlg::CurrentDevicePowerCfgString);
}

void CCSMDlg::OnBnClickedSetRouterAddr()
{
	ConfigSelectedDevices(&CCSMDlg::CurrentDeviceRouterAddrCfgString);
}

void CCSMDlg::OnBnClickedBtLedonOffset()
{
	ConfigSelectedDevices(&CCSMDlg::CurrentDeviceLedOnOffsetCfgString);
}

void CCSMDlg::OnBnClickedBtLedoffOffset()
{
	ConfigSelectedDevices(&CCSMDlg::CurrentDeviceLedOffOffsetCfgString);
}

void CCSMDlg::OnBnClickedSetLed1id()
{
	ConfigSelectedDevice(&CCSMDlg::CurrentDeviceLed1IdCfgString);
}

void CCSMDlg::OnBnClickedLed2id()
{
	ConfigSelectedDevice(&CCSMDlg::CurrentDeviceLed2IdCfgString);
}

void CCSMDlg::OnBnClickedLed3id()
{
	ConfigSelectedDevice(&CCSMDlg::CurrentDeviceLed3IdCfgString);
}

void CCSMDlg::OnBnClickedBtSetBeaconid() {
	ConfigSelectedDevice(&CCSMDlg::CurrentDeviceBeaconIdCfgString);
}

void CCSMDlg::OnBnClickedBtSetTimeslot() {
	ConfigSelectedDevice(&CCSMDlg::CurrentDeviceTimeslotCfgString);
}

void CCSMDlg::OnBnClickedCheckEnableImu() {
	ConfigSelectedDevices(&CCSMDlg::CurrentDeviceCheckEnableImuString);
}

void CCSMDlg::OnBnClickedCheckEnableBtry() {
	ConfigSelectedDevices(&CCSMDlg::CurrentDeviceCheckEnableBtryString);
}

void CCSMDlg::OnBnClickedCheckEnableBtn() {
	ConfigSelectedDevices(&CCSMDlg::CurrentDeviceCheckEnableBtnString);
}

void CCSMDlg::OnBnClickedBtSetHopCh() {
	ConfigSelectedDevices(&CCSMDlg::CurrentDeviceHoppingChannelCfgString);
}


std::string CCSMDlg::CurrentDevicePanIDCfgString(uint8_t deviceType) {
	std::stringstream ss;
	ss << "s 2 " << panID << " \r";
	return ss.str();
}

std::string CCSMDlg::CurrentDeviceChannelCfgString(uint8_t deviceType) {
	std::stringstream ss;
	ss << "s 22 " << (int)GetWirelessChannelFromIndex(wirelessChannel) << " \r";

	return ss.str();
}

std::string CCSMDlg::CurrentDevicePowerCfgString(uint8_t deviceType) {
	std::stringstream ss;
	ss << "s 14 " << txPowerLevel + 1 << " \r";
	return ss.str();
}

std::string CCSMDlg::CurrentDeviceRouterAddrCfgString(uint8_t deviceType) {
	if (deviceType == 3) {
		std::stringstream ss;
		ss << "s 4 " << this->routerAddress << " \r";
		return ss.str();
	}
	else {
		assert(deviceType == 1 || deviceType == 2);
		return std::string();							// TK & RT do not have this setting
	}
}

std::string CCSMDlg::CurrentDeviceLedOnOffsetCfgString(uint8_t deviceType) {
	if (deviceType == 3) {
		std::stringstream ss;
		ss << "s 5 " << this->ledOnOffset << " \r";
		return ss.str();
	}
	else {
		assert(deviceType == 1 || deviceType == 2);
		return std::string();							// TK & RT do not have this setting
	}
}

std::string CCSMDlg::CurrentDeviceLedOffOffsetCfgString(uint8_t deviceType) {
	if (deviceType == 3) {
		std::stringstream ss;
		ss << "s 6 " << this->ledOffOffset << " \r";
		return ss.str();
	}
	else {
		assert(deviceType == 1 || deviceType == 2);
		return std::string();							// TK & RT do not have this setting
	}
}

std::string CCSMDlg::CurrentDeviceLed1IdCfgString(uint8_t deviceType) {
	if(deviceType == 3) {
        auto data = m_mapping_information.GetLEDInfoByIndex(this->ledID1);
		if(data == NULL) {
			CString str;
			str.Format(_T("%d is out of scope"), this->ledID1);
			this->LogString(str); 
			return std::string();
		}
		std::stringstream ss;
		ss << "s 19 " << (int)this->ledID1 << " " << (int)data->bitPattern << " " << (int)bitArray.at(numberOfBits)  << " " << (int)data->lcid << " \r";
		return ss.str();
	} else {
		assert(deviceType == 1 || deviceType == 2);
		return std::string();							// TK & RT do not have this setting
	}
}

std::string CCSMDlg::CurrentDeviceLed2IdCfgString(uint8_t deviceType) {
	if(deviceType == 3) {
        auto data = m_mapping_information.GetLEDInfoByIndex(this->ledID2);
		if (data == NULL) {
			CString str;
			str.Format(_T("%d is out of scope"), this->ledID2);
			this->LogString(str); 		
			return std::string();
		}
		std::stringstream ss;
		ss << "s 20 " << (int)this->ledID2 << " " << (int)data->bitPattern << " " << (int)bitArray.at(numberOfBits)  << " " << (int)data->lcid << " \r";
		return ss.str();
	} else {
		assert(deviceType == 1 || deviceType == 2);
		return std::string();							// TK & RT do not have this setting
	}
}

std::string CCSMDlg::CurrentDeviceLed3IdCfgString(uint8_t deviceType) {
	if(deviceType == 3 ) {
		std::stringstream ss;
        auto data = m_mapping_information.GetLEDInfoByIndex(this->ledID3);
		if (data == NULL) {
			CString str;
			str.Format(_T("%d is out of scope"), this->ledID3);
			this->LogString(str); 
			return std::string();
		}
		ss << "s 21 " << (int)this->ledID3 << " " << (int)data->bitPattern << " " << (int)bitArray.at(numberOfBits)  << " " << (int)data->lcid << " \r";
		return ss.str();
	} else {
		assert(deviceType == 1 || deviceType == 2);
		return std::string();							// TK & RT do not have this setting
	}
}

std::string CCSMDlg::CurrentDeviceBeaconIdCfgString(uint8_t deviceType) {
	if (deviceType == 3) {
		std::stringstream ss;
		ss << "s 3 " << beaconID << " \r";
		return ss.str();
	}
	else {
		assert(deviceType == 1 || deviceType == 2);
		return std::string();							// TK & RT do not have this setting
	}
}

std::string CCSMDlg::CurrentDeviceTimeslotCfgString(uint8_t deviceType) {
	if (deviceType == 3) {
		std::stringstream ss;
		ss << "s 13 " << wirelessTimeSlot + 2 << " \r";		// +2: Dropdownlist Index to Slot ID  
		return ss.str();
	}
	else {
		assert(deviceType == 1 || deviceType == 2);
		return std::string();							// TK & RT do not have this setting
	}
}

std::string CCSMDlg::CurrentDeviceCheckEnableImuString(uint8_t deviceType) {
	uint8_t radioPacketFlags = 0;
	if (enableIMU == TRUE) radioPacketFlags |= RADIOPACKET_IMU;
	if (enableButton == TRUE) radioPacketFlags |= RADIOPACKET_BUTTONPRESS;
	if (enableBattery == TRUE) radioPacketFlags |= RADIOPACKET_BATTERY;

	std::stringstream ss;
	ss << "s 16 " << (int)radioPacketFlags << " \r";
	return ss.str();
}

std::string CCSMDlg::CurrentDeviceHoppingChannelCfgString(uint8_t deviceType) {
	std::stringstream ss;
	ss << "s 18 " << GetHopChannelsParaStr(hopChannels) << " \r";
	return ss.str();
}

std::string CCSMDlg::CurrentDeviceCheckEnableBtryString(uint8_t deviceType) {
	return CurrentDeviceCheckEnableImuString(deviceType);
}

std::string CCSMDlg::CurrentDeviceCheckEnableBtnString(uint8_t deviceType) {
	return CurrentDeviceCheckEnableImuString(deviceType);
}

std::string CCSMDlg::CurrentDeviceProductInfoString(uint8_t deviceType) {
    // Construct Packet for next segment
    PacketHeader header;

	header.type = DEV_CMD_SET_EEPROM_DATA;		

    std::string packetBody;

	std::string productIDStr = CT2A(productID);
	productIDStr.resize(SizeProductID, '\0');
	std::string serialNumStr = CT2A(serialNumber);
	serialNumStr.resize(SizeSerialNum, '\0');

    packetBody.append(productIDStr);
    packetBody.append(serialNumStr);
	packetBody.resize(sizeof(((RespEepromData *)0)->data));		// response and request data buffers are the same size

	header.size = sizeof(header) + packetBody.size();		

    std::string source;
    source.append(reinterpret_cast<char*>(&header), sizeof(header));
    source.append(packetBody);

	return source;
}

// production info settings use another set of commands:

void CCSMDlg::OnBnClickedSetProductId() {
	ProductConfigSelectedDevices(&CCSMDlg::CurrentDeviceProductInfoString);
}

void CCSMDlg::ProductConfigSelectedDevices(GetCfgString getCfgString) {		// TODO!!! unify
	BOOL ret = this->UpdateData(true);
	if (!ret) return;

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


		serial_com::SerialComConnectionInfo* curSerialCom = GetCurSerialComConnectionInfo();
		if (curSerialCom == NULL) {
			continue;
		}

		if (curSerialCom->status != serial_com::SerialConnected) {
			continue;
		}

		if (!curSerialCom->whoAmI || curSerialCom->whoAmI->module != 2) {		// command for main firmware only		
			continue;		
		}

		std::string cmdStr = (this->*(getCfgString))(curSerialCom->whoAmI->type);

		SendPacket(curSerialCom, cmdStr);

		{
			PacketHeader header;
			header.type = DEV_CMD_GET_EEPROM_DATA;

			// No packet body
			header.size = sizeof(header);

			header.size = sizeof(header);
			std::string source;
			source.append(reinterpret_cast<char*>(&header), sizeof(header));

			SendPacket(curSerialCom, source);		
		}
	
	}
	currentDeviceIndex = save;
}

