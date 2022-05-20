#pragma once
#pragma comment (lib, "setupapi.lib")
#include <set>
#include <string>
#include <list>
#include <Dbt.h>
#include "SetupAPI.h"


class ComDeviceObserver {
public:
	ComDeviceObserver() {}
	~ComDeviceObserver() {}
	virtual void addDevice(const std::basic_string<TCHAR>& device, const std::string& usbDeviceId) = 0;
	virtual void removeDevice(const std::basic_string<TCHAR>& device, const std::string& usbDeviceId) = 0;
};

class ComDeviceManager
{
public:
	typedef std::set<std::basic_string<TCHAR> > device_list;
private:
	device_list active_devices;
	ComDeviceObserver* observer;
	std::basic_string<TCHAR> m_device_type;

public:
	ComDeviceManager(HANDLE hwnd, ComDeviceObserver* observer);
	~ComDeviceManager();
	bool update(WPARAM wParam, LPARAM lParam);

private:
	void initializeDeviceList();
	std::basic_string<TCHAR> findFriendlyName(SP_DEVINFO_DATA& spDevInfoData, HDEVINFO& hDevInfo);
	void UpdateDevice(PDEV_BROADCAST_DEVICEINTERFACE pDevInf, WPARAM wParam);
	bool FindDevice(HDEVINFO& hDevInfo, std::basic_string<TCHAR>& szDevId, SP_DEVINFO_DATA& spDevInfoData);
	const std::string filterDevice(const std::basic_string<TCHAR>& device);
};

