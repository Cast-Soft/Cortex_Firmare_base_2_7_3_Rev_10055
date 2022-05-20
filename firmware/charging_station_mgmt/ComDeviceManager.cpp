#include "stdafx.h"
#include <Windows.h>
#include <WinUser.h>

#include <SetupAPI.h>
#include "ComDeviceManager.h"
#include <algorithm>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <tchar.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#define BUFFER_SIZE 4000            // from http://social.msdn.microsoft.com/Forums/vstudio/en-US/76315dfa-6764-4feb-a3e2-1f173fc5bdfd/how-to-get-usb-vendor-and-product-id-programmatically-in-c?forum=vcgeneral

ComDeviceManager::ComDeviceManager(HANDLE hwnd, ComDeviceObserver* observer)
	: m_device_type(_T("USB"))
    , observer(observer)
{
	static const GUID GUID_DEVINTERFACE_LIST[] = 
	{
		// GUID_DEVINTERFACE_USB_DEVICE
		{ 0xA5DCBF10, 0x6530, 0x11D2, { 0x90, 0x1F, 0x00, 0xC0, 0x4F, 0xB9, 0x51, 0xED } },

		// GUID_DEVINTERFACE_DISK
		{ 0x53f56307, 0xb6bf, 0x11d0, { 0x94, 0xf2, 0x00, 0xa0, 0xc9, 0x1e, 0xfb, 0x8b } },

		// GUID_DEVINTERFACE_HID, 
		{ 0x4D1E55B2, 0xF16F, 0x11CF, { 0x88, 0xCB, 0x00, 0x11, 0x11, 0x00, 0x00, 0x30 } },
			 
		// GUID_NDIS_LAN_CLASS
		{ 0xad498944, 0x762f, 0x11d0, { 0x8d, 0xcb, 0x00, 0xc0, 0x4f, 0xc3, 0x35, 0x8c } }

		//// GUID_DEVINTERFACE_COMPORT
		//{ 0x86e0d1e0, 0x8089, 0x11d0, { 0x9c, 0xe4, 0x08, 0x00, 0x3e, 0x30, 0x1f, 0x73 } },

		//// GUID_DEVINTERFACE_SERENUM_BUS_ENUMERATOR
		//{ 0x4D36E978, 0xE325, 0x11CE, { 0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18 } },

		//// GUID_DEVINTERFACE_PARALLEL
		//{ 0x97F76EF0, 0xF883, 0x11D0, { 0xAF, 0x1F, 0x00, 0x00, 0xF8, 0x00, 0x84, 0x5C } },

		//// GUID_DEVINTERFACE_PARCLASS
		//{ 0x811FC6A5, 0xF728, 0x11D0, { 0xA5, 0x37, 0x00, 0x00, 0xF8, 0x75, 0x3E, 0xD1 } }
	};
	//Get notification for devices
	HDEVNOTIFY hDevNotify;
    DEV_BROADCAST_DEVICEINTERFACE NotificationFilter;
    ZeroMemory( &NotificationFilter, sizeof(NotificationFilter) );
    NotificationFilter.dbcc_size = sizeof(DEV_BROADCAST_DEVICEINTERFACE);
    NotificationFilter.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;
	for(int i=0; i<sizeof(GUID_DEVINTERFACE_LIST)/sizeof(GUID); i++) {
		NotificationFilter.dbcc_classguid = GUID_DEVINTERFACE_LIST[i];
		hDevNotify = RegisterDeviceNotification(hwnd, &NotificationFilter, DEVICE_NOTIFY_WINDOW_HANDLE);
		if(!hDevNotify ) {
			continue;
		}
	}

	initializeDeviceList();
}

void ComDeviceManager::initializeDeviceList() {
	SP_DEVINFO_DATA spDevInfoData;
	std::basic_string<TCHAR> szClass(m_device_type);

	std::vector<TCHAR> data(szClass.begin(), szClass.end());
	data.push_back(TCHAR(0));
	
	HDEVINFO hDevInfo = SetupDiGetClassDevs(NULL, data.data(), NULL, DIGCF_PRESENT | DIGCF_ALLCLASSES);
	if(INVALID_HANDLE_VALUE == hDevInfo ) {
		assert(0);
		return;
	}
	spDevInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
	for(int i=0; SetupDiEnumDeviceInfo(hDevInfo, i, &spDevInfoData); i++) {
		DWORD nSize=0 ;
		TCHAR buf[BUFFER_SIZE];
		if ( !SetupDiGetDeviceInstanceId(hDevInfo, &spDevInfoData, buf, sizeof(buf), &nSize) ) {
			continue;
		}
		std::basic_string<TCHAR> name = findFriendlyName(spDevInfoData, hDevInfo);

	    std::string com_port = filterDevice(name);
	    if(com_port.empty()) continue;

        std::string usbDeviceId(buf);
		active_devices.insert(com_port);
		observer->addDevice(com_port, usbDeviceId);
	}

	SetupDiDestroyDeviceInfoList(hDevInfo);
}

const std::string ComDeviceManager::filterDevice(const std::basic_string<TCHAR>& device) {
	if(device.find(_T("STMicroelectronics Virtual COM Port ")) == std::string::npos) return "";
	auto start = device.find_first_of('(') + 1; 
	auto end = device.find_first_of(')');
	auto len = end - start;
	std::string com_port = device.substr(start, len);
	return com_port;
}

bool ComDeviceManager::update(WPARAM wParam, LPARAM lParam) {
	if ( DBT_DEVICEARRIVAL == wParam || DBT_DEVICEREMOVECOMPLETE == wParam ) {
		PDEV_BROADCAST_HDR pHdr = (PDEV_BROADCAST_HDR)lParam;
		PDEV_BROADCAST_DEVICEINTERFACE pDevInf;
		if( pHdr->dbch_devicetype ==  DBT_DEVTYP_DEVICEINTERFACE) {
				pDevInf = (PDEV_BROADCAST_DEVICEINTERFACE)pHdr;
				UpdateDevice(pDevInf, wParam);
				return true;
		}
	}
    return false;
}

std::basic_string<TCHAR> ComDeviceManager::findFriendlyName(SP_DEVINFO_DATA& spDevInfoData, HDEVINFO& hDevInfo) {
	DWORD DataT;
	TCHAR buf[BUFFER_SIZE];
	DWORD nSize = 0;

	// get Friendly Name or Device Description
	if ( SetupDiGetDeviceRegistryProperty(hDevInfo, &spDevInfoData, 
		SPDRP_FRIENDLYNAME, &DataT, (PBYTE)buf, sizeof(buf), &nSize) ) {
	} else if ( SetupDiGetDeviceRegistryProperty(hDevInfo, &spDevInfoData, 
		SPDRP_DEVICEDESC, &DataT, (PBYTE)buf, sizeof(buf), &nSize) ) {
	} else {
		return std::basic_string<TCHAR>(_T("Unknown"));
	}
	return std::basic_string<TCHAR>(buf, buf + nSize);
}

void ComDeviceManager::UpdateDevice(PDEV_BROADCAST_DEVICEINTERFACE pDevInf, WPARAM wParam) {
	auto size(pDevInf->dbcc_size);
	std::basic_string<TCHAR> input(pDevInf->dbcc_name, pDevInf->dbcc_name + size);
	input.erase(std::remove(input.begin(), input.end(), char(0)), input.end());
	input = input.substr(4);
	std::vector<std::basic_string<TCHAR> > split_input;
	boost::split(split_input, input, boost::is_any_of(_T("#")));
	input = split_input[0] + _T("\\") + split_input[1] + _T("\\") + split_input[2];
	//for(std::size_t i = 3; i < split_input.size() - 1; i++) {
	//	input += _T("#") + split_input[i];
	//}
	boost::to_upper(input);
	auto szClass = split_input[0];
	if(szClass != m_device_type) return;

	std::vector<TCHAR> data(szClass.begin(), szClass.end());
	data.push_back(TCHAR(0));
	
	HDEVINFO hDevInfo = SetupDiGetClassDevs(NULL, data.data(), NULL, DIGCF_ALLCLASSES);
	if(INVALID_HANDLE_VALUE == hDevInfo ) {
		assert(0);
		return;
	}

	SP_DEVINFO_DATA spDevInfoData;
    std::string usbDeviceId(input);
	input.push_back(0);	

    if(FindDevice(hDevInfo, input, spDevInfoData) ) {       // TODO!!! issue found. during processing if the USB device's status change again. FindDevice will get wrong state. Better to get name from pDevInf
		std::basic_string<TCHAR> name = findFriendlyName(spDevInfoData, hDevInfo);

	    std::string com_port = filterDevice(name);
	    if(com_port.empty()) goto LEAVE;
	    auto findResultInActive = active_devices.find(com_port);

	    if(findResultInActive == active_devices.end()) { //add the device
		    active_devices.insert(com_port);
			observer->addDevice(com_port, usbDeviceId);
	    } 
        else  {
		    active_devices.erase(com_port);
			observer->removeDevice(com_port, usbDeviceId);
	    }
	}

LEAVE:
	SetupDiDestroyDeviceInfoList(hDevInfo);
}

bool ComDeviceManager::FindDevice(HDEVINFO& hDevInfo, std::basic_string<TCHAR>& szDevId, SP_DEVINFO_DATA& spDevInfoData) {
	spDevInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
	for(int i=0; SetupDiEnumDeviceInfo(hDevInfo, i, &spDevInfoData); i++) {
		DWORD nSize=0 ;
		TCHAR buf[BUFFER_SIZE];
		if ( !SetupDiGetDeviceInstanceId(hDevInfo, &spDevInfoData, buf, sizeof(buf), &nSize) ) {
			return FALSE;
		}
		std::basic_string<TCHAR> value(buf, buf+nSize);
		if (szDevId == value) {
			return true;
		}
	}
	return false;
}

ComDeviceManager::~ComDeviceManager(void)
{

}
