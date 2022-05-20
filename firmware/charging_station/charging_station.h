#ifndef CHARGE_STATION_H
#define CHARGE_STATION_H

#include <windows.h>
#include <QtGui/QMainWindow>
#include "ui_charging_station.h"
#include <sstream>
#include <boost\lexical_cast.hpp>
#include <qcheckbox.h>
#include "BeaconTable.h"
#include "ComDeviceManager.h"
#include "qt_serial_com_messenger.h"
#include <list>
#include <common/packets.h>
#include <map>
#include <QTimer>


struct station_data {
	int current_display_stack_index;
	station_data() : current_display_stack_index(0) {}
};


struct show_configuration_data {
	typedef std::list<std::string> beacon_list;
	typedef uint8_t led_value_type;
	typedef std::list<led_value_type> led_list_type;

	led_list_type m_led1_list;
	led_list_type m_led2_list;
	led_list_type m_led3_list;
	std::map<std::string, std::list<uint8_t> > router_device_to_timeslot_list;
	std::map<std::string, beacon_list> router_to_beacon_list;
	uint8_t global_channel;

	struct config_beacon_data {
		config_beacon_data() : led1(0), led2(0), led3(0), slot(1), channel(1) {}
		std::string device;
		led_value_type led1;
		led_value_type led2;
		led_value_type led3;
		uint8_t slot;
		uint8_t channel;
	};

	struct config_router_data {
		config_router_data() : device("default"), channel(18) {}
		config_router_data(const std::string& device, const uint8_t channel)
			: device(device), channel(channel) {}
		std::string device;
		uint8_t channel;
	};

	struct config_timekeeper_data {
		std::string device;
		uint8_t channel;
	};

	std::list<config_beacon_data> beacons_config_list;
	std::list<config_router_data> router_config_list;
	std::map<std::string, config_router_data> device_to_router_config;
	std::list<config_timekeeper_data> timekeeper_config_list;

	show_configuration_data()
		: global_channel(18)
	{
		std::srand ( unsigned ( std::time(0) ) );
		led_value_type l1[12] = {187, 221, 238, 183, 219, 237, 246, 123, 189, 222, 175, 215};
		led_value_type l2[12] = {127, 191, 223, 239, 247, 251, 253, 254, 119, 111, 95, 107};
		led_value_type l3[12] = {235, 245, 250, 125, 190, 181, 218, 109, 182, 91, 173, 87};
		std::random_shuffle(l1, l1+12);
		std::random_shuffle(l2, l2+12);
		std::random_shuffle(l3, l3+12);
		m_led1_list.insert(m_led1_list.begin(), l1, l1+12);
		m_led2_list.insert(m_led2_list.begin(), l2, l2+12);
		m_led3_list.insert(m_led3_list.begin(), l3, l3+12);
	}

	bool emptyLed() const {
		return m_led2_list.empty();
	}

	bool emptyTimeSlot(const std::string& device) const {
		const auto it = router_device_to_timeslot_list.find(device);
		if(it == router_device_to_timeslot_list.end()) return true;
		return it->second.empty();
	}

	void add(const std::string& timekeeper_device,
			const std::string router_device,
			const uint8_t timekeeper_channel) {
		addRouter(router_device, timekeeper_channel);
	}

	config_beacon_data GetBeaconConfiguration(const std::string& device) {
		auto ret = router_to_beacon_list.begin();
		auto end = router_to_beacon_list.end();
		for(auto it = router_to_beacon_list.begin(); it != end; ++it) {
			auto& cur = it->second;
			auto& prev = ret->second;
			if(cur.size() < prev.size()) {
				ret = it;
			}
		}
		auto& list = ret->second;
		list.push_back(device);
		const std::string& router_device = ret->first;

		config_beacon_data config;
		config.device = device;
		if(!emptyLed()) {
			auto leds = GetLedValues();
			config.led1 = std::get<0>(leds);
			config.led2 = std::get<1>(leds);
			config.led3 = std::get<2>(leds);
		}
		if(!emptyTimeSlot(router_device)) {
			config.slot = GetTimeSlot(router_device);
		}
		const auto& router_config = device_to_router_config[ret->first];
		config.channel = router_config.channel;
		return config;
	}

	const uint8_t GetTimeSlot(const std::string& device) {
		auto& list = router_device_to_timeslot_list[device];
		const auto slot = list.front();
		list.pop_front();
		return slot;
	}

private:
	void addTimekeeper(const std::string& device, const uint8_t channel) {
		global_channel = channel;
	}

	void addRouter(const std::string& device, const uint8_t channel) {
		assert(router_device_to_timeslot_list.find(device) ==  router_device_to_timeslot_list.end());
		static const uint8_t timeslots[14] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
		auto& list = router_device_to_timeslot_list[device];
		list.insert(list.begin(), timeslots, timeslots+14);
		router_to_beacon_list[device];
		device_to_router_config[device] = config_router_data(device, channel);
	}

	std::tuple<led_value_type, led_value_type, led_value_type> GetLedValues() {
		const auto l1 = m_led1_list.front();
		const auto l2 = m_led2_list.front();
		const auto l3 = m_led3_list.front();
		m_led1_list.pop_front();
		m_led2_list.pop_front();
		m_led3_list.pop_front();
		return std::make_tuple(l1, l2, l3);
	}
};

class charging_station : public QMainWindow , public ComDeviceObserver
{
	Q_OBJECT
	friend class serial_com::MessengerHook<charging_station>;
	typedef std::map<std::string, TableData> ComDataMapping;
	typedef std::map<CommContext, serial_com::SharedSerialComConnectionInfo> SerialComConnectionInfoSet;
	boost::scoped_ptr<ComDeviceManager> m_com_manager;
	station_data model_data;
	SerialComConnectionInfoSet serialComConnectionInfoSet;
	std::map<std::string, TableData> m_comport_to_beaconData;
	std::map<std::string, TableData> m_comport_to_TimekeeperData;
	std::map<std::string, TableData> m_comport_to_RouterData;
	std::map<std::string, int> m_device_to_id;
	std::map<std::string, beacon::Config> m_comport_to_beaconConfig;
	std::map<std::string, timekeeper::Config> m_comport_to_timekeeperConfig;
	std::map<std::string, timekeeper::Config> m_comport_to_routerConfig;
	std::vector<std::string> m_firmware_segment;
	boost::optional<FirmwareVerifyPacketHeader> verifyHeader;
	std::set<std::string> devices_to_update_firmware_on;
	unsigned char m_md5Sum[16];
	std::string current_timekeeper_comport;

	QTimer timer;

	enum FirmwareType {			// should move to packets.h
		FIRM_OTHER = 0,			// other  like manufacturer info -- to be extended
		FIRM_TK = 1,
		FIRM_RT = 2,
		FIRM_BC = 3
	};
public:
	charging_station(QWidget *parent = 0, Qt::WFlags flags = 0);
	~charging_station();
	virtual void addDevice(const std::basic_string<TCHAR>& device, const std::string& usbDeviceId);
	virtual void removeDevice(const std::basic_string<TCHAR>& device, const std::string& usbDeviceId);
	bool winEvent( MSG * message, long * result );
	void updateView();
	
public slots:
	void on_toolButtonBeacon_clicked();
	void on_toolButtonTimekeeper_clicked();
	void on_toolButtonRouter_clicked();
	void on_readImage_clicked();
	void on_configureShowDevices_clicked();
	void on_tkSetWirelessChannel_clicked();
	void updateProperties(const QItemSelection selected, const QItemSelection deselected);
	void SendBatteryRequest();
	
private:
	Ui::charge_stationClass ui;
	DeviceTable beacon_table;
	DeviceTable timekeeper_table;
	DeviceTable router_table;

protected:
	virtual void closeEvent(QCloseEvent *event);

	// Serial port communication facilities
protected:
	// Create a concrete type from template, paramter is the message loop window
	typedef SerialCom::QtMessenger<charging_station> ConcreteSerialComMessenger;
	boost::scoped_ptr<ConcreteSerialComMessenger> serialComMessenger;
    bool eventFilter(QObject *obj, QEvent *event);  // Communication event hook 

    //LRESULT OnSerialComIO(WPARAM wParam, LPARAM lparam);
    void OnSerialComConnect(serial_com::SharedSerialComConnectionInfo& connectionInfo);
    void OnSerialComWrite(serial_com::SharedSerialComConnectionInfo& connectionInfo);
    void OnSerialComReceive(serial_com::SharedSerialComConnectionInfo& connectionInfo, std::vector<unsigned char>& buffer);
    void OnSerialComCloseConnection(serial_com::SharedSerialComConnectionInfo& connectionInfo, CommContext closeConnId, boost::system::error_code error);

private:
	void ParseProtocolSerialData(serial_com::SharedSerialComConnectionInfo& serialCom, const std::string str);
	void OnSerialPacket(serial_com::SharedSerialComConnectionInfo& connectionInfo, const USBDebugInfoHeader& header, std::string body);
	void LogData(const std::string& sevarity, const std::string& comport, const std::string& str);

	void SendPacket(serial_com::SharedSerialComConnectionInfo& serialCom, std::string source);
	void SendPacket(serial_com::SharedSerialComConnectionInfo& serialCom, uint8_t type);
	void SendCommand(serial_com::SharedSerialComConnectionInfo& serialCom, const std::string& command);
	void UploadFirmwarePacketSegment(serial_com::SharedSerialComConnectionInfo& serialCom); 
	void updateFirmware(ComDataMapping& mapping);
	FirmwareType DetectFirmwareType(std::string fileName);
	void Verify(serial_com::SharedSerialComConnectionInfo& serialCom);
	void FinishWithUploading(serial_com::SharedSerialComConnectionInfo& serialCom);
	void FailedWithUploading(serial_com::SharedSerialComConnectionInfo& serialCom);
	void initalizeLed();
	TableData* GetData(const std::string& comport);
	ComDataMapping* GetMapping(const std::string& comport);
	float GetBatteryPercentage(boost::uint8_t battery);
};

#endif // CHARGE_STATION_H
