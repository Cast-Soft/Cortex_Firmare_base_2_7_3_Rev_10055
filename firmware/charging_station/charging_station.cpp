
#include "charging_station.h"
#include "BeaconTable.h"
#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>
#include <stdio.h>
#include <common/base64.h>
#include <QItemSelectionModel>
#include <QtGui/QFileDialog>
#include <fstream>
#include <iterator>
#include <common/md5.h>
#include <common/base64.h>
#include <ctime>
#include <cstdlib> 

charging_station::charging_station(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
	, beacon_table(this) 
	, timekeeper_table(this)
	, router_table(this)
{
	installEventFilter(this);
	ui.setupUi(this);
	ui.beaconTableView->setModel(&beacon_table);
	ui.timekeepeTableView->setModel(&timekeeper_table);
	ui.routerTableView->setModel(&router_table);
	serialComMessenger.reset(new ConcreteSerialComMessenger(this));
	m_com_manager.reset(new ComDeviceManager(winId(), this));
	timer.start(30000);
	connect(ui.beaconTableView->selectionModel(), SIGNAL(selectionChanged(QItemSelection, QItemSelection)), this, SLOT(updateProperties(QItemSelection, QItemSelection)));
	connect(ui.timekeepeTableView->selectionModel(), SIGNAL(selectionChanged(QItemSelection, QItemSelection)), this, SLOT(updateProperties(QItemSelection, QItemSelection)));
	connect(ui.routerTableView->selectionModel(), SIGNAL(selectionChanged(QItemSelection, QItemSelection)), this, SLOT(updateProperties(QItemSelection, QItemSelection)));
	connect(&timer, SIGNAL(timeout()), this, SLOT(SendBatteryRequest()));
}


charging_station::~charging_station()
{
}

bool charging_station::eventFilter(QObject *obj, QEvent *event)
{
    if (!serialComMessenger) return false;
    bool ret = serialComMessenger->eventFilter(obj, event);      // link to Com event filter
    return ret;
}

bool charging_station::winEvent( MSG * message, long * result )  {
	if(m_com_manager) {
		return m_com_manager->update(message->wParam, message->lParam);
	}
	return false;
}

void charging_station::addDevice(const std::basic_string<TCHAR>& device, const std::string& usbDeviceId) {
	std::string dev(device.begin(), device.end());
	LogData("Info", dev, "Found new device");
	static int i = 0;
	m_device_to_id[dev] = i++;
	int index = m_device_to_id[dev];
	serial_com::SharedSerialComConnectionInfo connectionInfo(boost::make_shared<serial_com::SerialComConnectionInfo>(dev, index));
	assert(serialComConnectionInfoSet.find(index) == serialComConnectionInfoSet.end());
	serialComConnectionInfoSet[index] = connectionInfo;
    serialComMessenger->AllocateConnectSessionId(connectionInfo);
    assert(connectionInfo->connectSessionId);
    connectionInfo->status = serial_com::SerialConnecting;
	connectionInfo->endpoint.comPort = dev;
    serialComMessenger->AsyncConnect(connectionInfo->endpoint, connectionInfo->connectSessionId);
}

void charging_station::removeDevice(const std::basic_string<TCHAR>& device, const std::string& usbDeviceId) {
	std::string comport(device.begin(), device.end());
	LogData("Info", comport, "Removing");
	int index = m_device_to_id[comport];
	auto serialCom = serialComConnectionInfoSet[index];
	serialComMessenger->AsyncCloseConnection(serialCom->connectSessionId, false, 0);

	auto mapping = GetMapping(comport);
	if(mapping) {
		auto found_beacon = mapping->find(comport);
		if(found_beacon != mapping->end()) {
			mapping->erase(found_beacon);
			m_device_to_id.erase(m_device_to_id.find(comport));
		}
	}
	updateView();
}

void charging_station::updateView() {
	ui.stackedWidget->setCurrentIndex(model_data.current_display_stack_index);
	std::map<std::string, TableData>* mapping;
	DeviceTable* table;
	
	if(model_data.current_display_stack_index == 0) {
		mapping = &m_comport_to_beaconData;
		table = &beacon_table;
	} else if(model_data.current_display_stack_index == 1) {
		mapping = &m_comport_to_TimekeeperData;
		table = &timekeeper_table;
	} else {
		mapping = &m_comport_to_RouterData;
		table = &router_table;
	}
	table->clear();
	auto end = mapping->end();
	for(auto it = mapping->begin(); it != end; ++it) {
		table->add(it->second);
	}
	bool allow_gui_action = m_comport_to_TimekeeperData.size() == 1 && 
		m_comport_to_timekeeperConfig.size() == 1 &&
		m_comport_to_RouterData.size() == 1 &&
		m_comport_to_beaconData.size() > 0 &&
		devices_to_update_firmware_on.empty();
	ui.tkWirelessDropDown->setEnabled(allow_gui_action);
	ui.configureShowDevices->setEnabled(allow_gui_action);
	bool read_file = devices_to_update_firmware_on.empty() && 
		(m_comport_to_timekeeperConfig.size() == 1 ||
		m_comport_to_RouterData.size() == 1 ||
		m_comport_to_beaconData.size() > 0);
	ui.readImage->setEnabled(read_file);
}

void charging_station::on_toolButtonBeacon_clicked() {
	model_data.current_display_stack_index = 0;
	updateView();
}

void charging_station::on_toolButtonTimekeeper_clicked() {
	model_data.current_display_stack_index = 1;
	updateView();
}
void charging_station::on_toolButtonRouter_clicked() {
	model_data.current_display_stack_index = 2;
	updateView();
}

void charging_station::OnSerialComConnect(serial_com::SharedSerialComConnectionInfo& connectionInfo) {
	const std::string& comPort = connectionInfo->endpoint.comPort;
	SendCommand(connectionInfo, "t 8 \r");
	SendCommand(connectionInfo, "t 6 \r");
	SendPacket(connectionInfo, DEV_CMD_CONFIG_REQ);
}

void charging_station::OnSerialComWrite(serial_com::SharedSerialComConnectionInfo& connectionInfo) {
	const std::string& comport = connectionInfo->endpoint.comPort;
	auto data = GetData(comport);
	if(data) {
		data->sent_data = connectionInfo->msgSent;
	}
}

void charging_station::OnSerialComReceive(serial_com::SharedSerialComConnectionInfo& connectionInfo, std::vector<unsigned char>& buffer) {
	const std::string& comport = connectionInfo->endpoint.comPort;
	auto data = GetData(comport);
	if(data) {
		data->receive_data = connectionInfo->msgReceived;
	}
	std::string str(buffer.begin(), buffer.end());
	ParseProtocolSerialData(connectionInfo, str);
}

void charging_station::OnSerialComCloseConnection(serial_com::SharedSerialComConnectionInfo& connectionInfo, CommContext closeConnId, boost::system::error_code error) {
    int ctrlIndex = connectionInfo->blockId;
	/*
	for(SerialComConnectionInfoSet::iterator it = serialComConnectionInfoSet.begin(); it != serialComConnectionInfoSet.end(); ++it) {
		serial_com::SharedSerialComConnectionInfo& tempConnectionInfo = it->second;

		if (connectionInfo.get() == tempConnectionInfo.get()) {
			serialComConnectionInfoSet.erase(it);
			RepopulateDeviceList();
			break;
		}
	}
	*/
}

void charging_station::ParseProtocolSerialData(serial_com::SharedSerialComConnectionInfo& serialCom, const std::string str) {
	assert(serialCom);

    std::string& serialBuffer = serialCom->serialBuffer;
	serialBuffer.append(str);
    DWORD signature = TK_FRAME_SIGNATURE;
    std::string signatureStr((char *)&signature, sizeof(signature));        // TODO!!! to constructor
    while(true) {
        size_t headerPos = serialBuffer.find(signatureStr);
        if (headerPos == std::string::npos) return;
        // Found, remove not complete last packet
        if (headerPos != 0) {
            serialBuffer = serialBuffer.substr(headerPos);                  
        }
        if (serialBuffer.size() < sizeof(USBDebugInfoHeader)) return;
        char* charBuffer = const_cast<char *>(serialBuffer.c_str());
        USBDebugInfoHeader* header =  reinterpret_cast<USBDebugInfoHeader*>(charBuffer);
        if (header->contentLength > serialBuffer.size()) return;
        std::string body(charBuffer + sizeof(USBDebugInfoHeader), header->contentLength - sizeof(USBDebugInfoHeader));
        OnSerialPacket(serialCom, *header, body);
        // remove a packet;
        serialBuffer = serialBuffer.substr(header->contentLength);
    }
}

void charging_station::LogData(const std::string& sevarity, const std::string& comport, const std::string& str) {
	std::string output = sevarity + ":";
	auto const * data = GetData(comport);
	output += " " + comport;
	if(data && data->id != "?") {
		output += " , ID " + data->id;
	}
	output += " - " + str;
	QString out(output.c_str());
	if(sevarity == "Error") {
		QString html("<font color=\"#FF8000\">");
		QString end("</font><br>");
		out = html+  out + end;
	} else if(sevarity == "Debug") {
		return;
		QString html("<font color=\"Aqua\">");
		QString end("</font><br>");
		out = html+  out + end;
	} else if(sevarity == "Info") {
		QString html("<font color=\"#FFFFFF\">");
		QString end("</font><br>");
		out = html+  out + end;
	} else {
		return;
	}
	ui.logs->moveCursor(QTextCursor::End);
	ui.logs->textCursor().insertHtml(out);
	ui.logs->moveCursor(QTextCursor::End);
}

void charging_station::OnSerialPacket(serial_com::SharedSerialComConnectionInfo& connectionInfo, const USBDebugInfoHeader& header, std::string body) {
	const auto& comport = connectionInfo->endpoint.comPort;
    if (header.contentType == DEV_RESP_INFO) {
        LogData("Info", comport, body);
    } else if (header.contentType == DEV_RESP_IMU_DATA) {	
	} else if (header.contentType == DEV_RESP_UPD || header.contentType == DEV_RESP_SET_PROD_AREA) {
        RespUpdate* respUpdate;
        respUpdate = reinterpret_cast<RespUpdate* >(const_cast<char*>(body.c_str()));
        if (respUpdate->errorCode == 0) {            
			int& currentFirmwareSegmentIndex = connectionInfo->currentFirmwareSegmentIndex;
            assert(respUpdate->index == currentFirmwareSegmentIndex);
            if (currentFirmwareSegmentIndex < 0) return;
            ++currentFirmwareSegmentIndex;
            if (currentFirmwareSegmentIndex >= (int)m_firmware_segment.size()) {
				LogData("Info", comport, "Upload complete");
				Verify(connectionInfo);
            } else {
                UploadFirmwarePacketSegment(connectionInfo);
            }
        } else {
            std::string s = "Error updating firmware. Error code: " + boost::lexical_cast<std::string>(respUpdate->errorCode);
            LogData("Error", comport, s);
        }
		updateView();
    } else if (header.contentType == DEV_RESP_CONFIG) {
		const boost::optional<WhoAmI>& whoAmI = connectionInfo->whoAmI;
		if (!whoAmI) {
			LogData("Error", comport, "Unkown device type");
			return;
		}
		LogData("Info", comport, "Received configuration");
		if (whoAmI->type == 3) {
			assert(header.contentLength == sizeof(beacon::Config) + sizeof(header));
			beacon::Config* config;
			config = reinterpret_cast<beacon::Config*>(const_cast<char*>(body.c_str()));
			if(config == NULL) return;
			m_comport_to_beaconConfig[comport] = *config;
			const std::string id = boost::lexical_cast<std::string>(config->mySrcAddr);
			m_comport_to_beaconData[comport].id = id;
		} else if(whoAmI->type == 1) {
			assert(header.contentLength == sizeof(timekeeper::Config) + sizeof(header));		
			timekeeper::Config* config;
			config = reinterpret_cast<timekeeper::Config*>(const_cast<char*>(body.c_str()));
			m_comport_to_timekeeperConfig[comport] = *config;
			const std::string id = boost::lexical_cast<std::string>(config->mySrcAddr);
			m_comport_to_TimekeeperData[comport].id = id;
		} else if(whoAmI->type == 2) {
			assert(header.contentLength == sizeof(timekeeper::Config) + sizeof(header));	
			timekeeper::Config* config;
			config = reinterpret_cast<timekeeper::Config*>(const_cast<char*>(body.c_str()));
			m_comport_to_routerConfig[comport] = *config;
			const std::string id = boost::lexical_cast<std::string>(config->mySrcAddr);
			m_comport_to_RouterData[comport].id = id;
		}
		updateView();
	} else if (header.contentType == DEV_RESP_RUNNING_STATUS) {
		LogData("Info", comport, "Status update");
		BeaconRunningStatus* runningStatus;
        runningStatus = reinterpret_cast<BeaconRunningStatus*>(const_cast<char*>(body.c_str()));
		if (runningStatus->errorCode != 0) {
			std::stringstream ss;
			ss << "Running status error: " << runningStatus->errorCode;
			LogData("Error", comport, ss.str());
			return;
		}
		BeaconRunningStatus* ds;
        ds = reinterpret_cast<BeaconRunningStatus*>(const_cast<char*>(body.c_str()));
		auto battery = GetBatteryPercentage(ds->index);
		std::stringstream ss;
		ss << battery << "%";
		m_comport_to_beaconData[comport].connection_status = ss.str();
		updateView();
	} else if (header.contentType == DEV_RESP_FIRMWARE_SUM) {
        LogData("Info",comport, "Received MD5 checksum");
        RespFirmwareSum* sum = reinterpret_cast<RespFirmwareSum*>(const_cast<char*>(body.c_str()));
		if (sum->errorCode != 0) {
			const std::string error = boost::lexical_cast<std::string>(sum->errorCode);
			LogData("Error", comport, "Checking sum error - " + error);
			return;
		}
		if (memcmp(sum->sum, m_md5Sum, sizeof(sum->sum)) == 0) {
			LogData("Info", comport, "Correct MD5 Hash");
			FinishWithUploading(connectionInfo);		
		} else {
			FailedWithUploading(connectionInfo);	
		}
	} else if (header.contentType == DEV_RESP_GET_PROD_AREA) {
	} else if (header.contentType == DEV_RESP_WHOAMI) {
		LogData("Info", comport, "Received who am I command");
		WhoAmI* who;
        who = reinterpret_cast<WhoAmI*>(const_cast<char*>(body.c_str()));
		boost::optional<WhoAmI>& whoAmI = connectionInfo->whoAmI;
		whoAmI = *who;
		std::stringstream ss;
		ss << std::hex << std::setfill('0');
		for (int i = 0; i < 12; ++i) {
			ss << std::setw(2) << static_cast<unsigned>(whoAmI->id[i]);
		}
		const std::string deviceId(ss.str());
		TableData * data;
		if(whoAmI->type == 3) { //beacon
			data = &m_comport_to_beaconData[comport];
		} else if(whoAmI->type == 1) { //timekeeper
			data = &m_comport_to_TimekeeperData[comport];
		} else if(whoAmI->type == 2) { //router
			data = &m_comport_to_RouterData[comport];
		}
		data->m_com_port = comport;
		data->deviceId = deviceId;
		data->connection_status = "Connected";
		if (whoAmI->module == 1) {
			data->current_mode_type = "Boot Loader";
			bool update_device_firmware = devices_to_update_firmware_on.find(deviceId) != devices_to_update_firmware_on.end();
			if(update_device_firmware) {
				connectionInfo->currentFirmwareSegmentIndex = 0;
				UploadFirmwarePacketSegment(connectionInfo);
			} else {
				SendCommand(connectionInfo, "B \r");
			}
		} else if (whoAmI->module == 2) {
			data->current_mode_type = "Main Firmware";		
		}
		updateView();
	} else {
        LogData("Error", comport, "Wrong response");      
	}
}

void charging_station::closeEvent(QCloseEvent *event) {
	serialComMessenger->Stop();
}

void charging_station::SendPacket(
		serial_com::SharedSerialComConnectionInfo& serialCom,
		std::string source) {	// Encode and send a packet out
    // Encode Packet
    char target[128*4];
    int sourceLength = source.length();
    int ret = b64_ntop((const unsigned char *)source.c_str(), sourceLength, target, sizeof(target));
    assert(ret != -1 && ret  <= sizeof(target));
    std::string param(target, ret);
    std::string cmdStr = "U 0 0 " + param + " \r";
	SendCommand(serialCom, cmdStr);
}

void charging_station::SendPacket(
		serial_com::SharedSerialComConnectionInfo& serialCom, uint8_t type) {
    PacketHeader header;
    header.type = type;
	header.size = sizeof(header);
    std::string source;
    source.append(reinterpret_cast<char*>(&header), sizeof(header));
	SendPacket(serialCom, source);
}

void charging_station::updateProperties(const QItemSelection selected,
	const QItemSelection deselected) {
	QModelIndexList indexList = selected.indexes();
	if(indexList.size() != 1) return; //TODO: add multiple selection
	const QModelIndex selected_index = indexList[0];
	int row = selected_index.row();

	if(model_data.current_display_stack_index == 0) {
		auto data = beacon_table.getData(row);
		const std::string comport(data[1]);
		auto found = m_comport_to_beaconConfig.find(comport);
		if( found == m_comport_to_beaconConfig.end()) return;
		beacon::Config& config = m_comport_to_beaconConfig[comport];
		ui.bcProductID->setText(QString::number(config.productID));
		ui.bcSerialNumber->setText(QString::number(config.serialNum));
		ui.bcPanID->setText(QString::number(config.panId));
		ui.bcBeaconID->setText(QString::number(config.mySrcAddr));
		ui.bcLedOnOffset->setText(QString::number(config.ledOnOffs));
		ui.bcLedOffOffset->setText(QString::number(config.ledOffOffs));
		ui.bcWirelessChannel->setText(QString::number(config.rfChan));
		ui.bcWirelessTimeSlot->setText(QString::number(config.rfTimeSlot));
		ui.bcLed1->setText(QString::number(config.led0Id));
		ui.bcLed2->setText(QString::number(config.led1Id));
		ui.bcLed3->setText(QString::number(config.led2Id));
		ui.bcTxPowerLevel->setText(QString::number(config.TxLevel));
	} else if(model_data.current_display_stack_index == 1) {
		auto data = timekeeper_table.getData(row);
		const std::string comport(data[1]);
		auto found = m_comport_to_timekeeperConfig.find(comport);
		if( found == m_comport_to_timekeeperConfig.end()) return;
		timekeeper::Config& config = m_comport_to_timekeeperConfig[comport];
		ui.tkProductId->setText(QString::number(config.productID));
		ui.tkSerialNumber->setText(QString::number(config.serialNum));
		ui.tkPanId->setText(QString::number(config.panId));
		ui.tkWirelessDropDown->setCurrentIndex(config.rfChan - 9);
		ui.tkPowerLevel->setText(QString::number(config.TxPower));
		current_timekeeper_comport = comport;
	} else if(model_data.current_display_stack_index == 2) {
		auto data = router_table.getData(row);
		const std::string comport(data[1]);
		auto found = m_comport_to_RouterData.find(comport);
		if( found == m_comport_to_RouterData.end()) return;
		timekeeper::Config& config = m_comport_to_routerConfig[comport];
		ui.rtProductId->setText(QString::number(config.productID));
		ui.rtSerialNumber->setText(QString::number(config.serialNum));
		ui.rtPanId->setText(QString::number(config.panId));
		ui.rtWirelessChannel->setText(QString::number(config.rfChan));
		ui.rtPowerLevel->setText(QString::number(config.TxPower));
	}
}

void charging_station::on_readImage_clicked() {
	QFileDialog fileDlg(this, tr("Firmware Binary File"), tr(""), tr("Firmware Binary Files (*.bin)"));
	if (fileDlg.exec() != QDialog::Accepted) return;
	QString filepath = fileDlg.selectedFiles().front();
	const std::string filename = filepath.toStdString();
	std::ifstream is(filename.c_str(), std::ios_base::out|std::ios_base::binary);
	int readCount = 128;
	std::streamsize segmentCount = 0;
	std::size_t size = 0;
	static const std::streamsize BUFFER_SIZE = 128;
	static char buf[BUFFER_SIZE];
	MD5_CTX mdContext;
	MD5Init(&mdContext);
	m_firmware_segment.clear();
	while(is.read(buf, BUFFER_SIZE) || (readCount = is.gcount()) != 0 ) {
		std::string segment(buf, buf + readCount);
		m_firmware_segment.push_back(segment);
		MD5Update(&mdContext, (unsigned char *)buf, readCount);
		size += readCount;
		segmentCount++;
	}
	MD5Final(m_md5Sum, &mdContext);
	FirmwareVerifyPacketHeader header;
	header.address = 0x0800C000;
	header.count = size;
	verifyHeader = boost::optional<FirmwareVerifyPacketHeader>(header);
	const std::string segment_str = boost::lexical_cast<std::string>(segmentCount);
	auto pos = filename.rfind('//') + 1;
	const std::string friendly_name = filename.substr(pos, -1);
	LogData("Info", friendly_name, segment_str + " Segments loaded into memory");
	std::stringstream ss;
	ss << std::hex << std::setfill('0');
	for (int i = 0; i < 16; ++i) {
		ss << std::setw(2) << static_cast<unsigned>(m_md5Sum[i]);
	}
	LogData("Info", friendly_name, "MD5 Hash - " + ss.str());
	auto type = DetectFirmwareType(filename); 
	if(type == FIRM_BC) {
		updateFirmware(m_comport_to_beaconData);
	} else if(type == FIRM_TK) {
		updateFirmware(m_comport_to_TimekeeperData);
	} else if (type == FIRM_RT) {
		updateFirmware(m_comport_to_RouterData);
	}
}

void charging_station::updateFirmware(ComDataMapping& mapping) {
	auto end = mapping.end();
	for(auto it = mapping.begin(); it != end; ++it) {
		const std::string& comport = it->first;
		auto index = m_device_to_id[comport];
		auto& serialCom = serialComConnectionInfoSet[index];
		const auto& data = it->second;
		devices_to_update_firmware_on.insert(data.deviceId);
		if(serialCom->whoAmI->module == 1) { //we are in bootloader
			serialCom->currentFirmwareSegmentIndex = 0;
			UploadFirmwarePacketSegment(serialCom);
		} else {
			LogData("Info", comport, "Switching to bootloader");
			SendCommand(serialCom, "! \r");
		}
	}
}

charging_station::FirmwareType charging_station::DetectFirmwareType(std::string fileName) {
	auto pos = fileName.rfind('//') + 1;
	std::string prefix = fileName.substr(pos, 2);
	boost::to_upper(prefix);
	if (prefix == "BC") return FIRM_BC;
	if (prefix == "TK") return FIRM_TK;
	if (prefix == "RT") return FIRM_RT;
	return FIRM_OTHER;
}

void charging_station::UploadFirmwarePacketSegment(serial_com::SharedSerialComConnectionInfo& serialCom) {
    // Construct Packet for next segment
    PacketHeader header;
	header.type = DEVCMD_UPD_PACKET;
	int& currentFirmwareSegmentIndex = serialCom->currentFirmwareSegmentIndex;
    FirmwarePacketHeader firmwarePacketHeader;
    firmwarePacketHeader.count = m_firmware_segment.size();
	firmwarePacketHeader.index = currentFirmwareSegmentIndex;
    std::string segement = m_firmware_segment[currentFirmwareSegmentIndex];
    std::string firmwarePacketBody;
    firmwarePacketBody.append(reinterpret_cast<char*>(&firmwarePacketHeader), sizeof(firmwarePacketHeader));
    firmwarePacketBody.append(segement);
    header.size = sizeof(header) + firmwarePacketBody.size();
	if (currentFirmwareSegmentIndex + 1 != m_firmware_segment.size()) {
		assert(header.size == 135);
	}
    std::string source;
    source.append(reinterpret_cast<char*>(&header), sizeof(header));
    source.append(firmwarePacketBody);
	SendPacket(serialCom, source);
}

void charging_station::SendCommand(serial_com::SharedSerialComConnectionInfo& serialCom, const std::string& command) {
	const std::string& comport = serialCom->endpoint.comPort;
	std::vector<unsigned char> buffer(command.begin(), command.end());
	serialComMessenger->AsyncWrite(buffer, serialCom->connectSessionId, serialCom->writeId);
	LogData("Debug", comport, command);
}

void charging_station::Verify(serial_com::SharedSerialComConnectionInfo& serialCom) {
	PacketHeader header;
	header.type = DEV_CMD_FIRMWARE_VERIFY;
	header.size = sizeof(header); // No packet body
	FirmwareVerifyPacketHeader vHeader = verifyHeader.get();
	header.size = sizeof(header) + sizeof(vHeader);
	std::string source;
	source.append(reinterpret_cast<char*>(&header), sizeof(header));
	source.append(reinterpret_cast<char*>(&vHeader), sizeof(vHeader));
	SendPacket(serialCom, source);
}

void charging_station::FinishWithUploading(serial_com::SharedSerialComConnectionInfo& serialCom) {
	const std::string& comport = serialCom->endpoint.comPort;
	auto mapping = GetMapping(comport);
	if(!mapping) return;
	const std::string& deviceId = (*mapping)[comport].deviceId;
	auto found_device = devices_to_update_firmware_on.find(deviceId);
	if(found_device != devices_to_update_firmware_on.end()) {
		devices_to_update_firmware_on.erase(found_device);
	}
	SendCommand(serialCom, "B \r");
}

void charging_station::FailedWithUploading(serial_com::SharedSerialComConnectionInfo& serialCom) {
	const std::string& comport = serialCom->endpoint.comPort;
	LogData("Error", comport, "Failed to upload firmware");
	//TODO: handle failed uploads another way.
	FinishWithUploading(serialCom);
}
TableData* charging_station::GetData(const std::string& comport) {
	if(m_comport_to_beaconData.find(comport) != m_comport_to_beaconData.end()) {
		return &m_comport_to_beaconData[comport];
	} else if (m_comport_to_TimekeeperData.find(comport) != m_comport_to_TimekeeperData.end()) {
		return &m_comport_to_TimekeeperData[comport];
	} else if(m_comport_to_RouterData.find(comport) != m_comport_to_RouterData.end()) {
		return &m_comport_to_RouterData[comport];
	}
	return NULL;
}

charging_station::ComDataMapping* charging_station::GetMapping(const std::string& comport) {
	if(m_comport_to_beaconData.find(comport) != m_comport_to_beaconData.end()) {
		return &m_comport_to_beaconData;
	} else if (m_comport_to_TimekeeperData.find(comport) != m_comport_to_TimekeeperData.end()) {
		return &m_comport_to_TimekeeperData;
	} else if(m_comport_to_RouterData.find(comport) != m_comport_to_RouterData.end()) {
		return &m_comport_to_RouterData;
	}

	return NULL;
}

void charging_station::on_configureShowDevices_clicked() {
	std::string timekeeper_device("default");
	auto channel(18);
	auto timekeeper_data_it = m_comport_to_TimekeeperData.begin();
	if(timekeeper_data_it != m_comport_to_TimekeeperData.end()) {
		timekeeper_device = timekeeper_data_it->second.deviceId;
	}
	auto timekeeper_config_it = m_comport_to_timekeeperConfig.begin();
	if(timekeeper_config_it != m_comport_to_timekeeperConfig.end()) {
		channel = timekeeper_config_it->second.rfChan;
	}
	std::string router_device("default");
	auto router_it = m_comport_to_RouterData.begin();
	if(router_it != m_comport_to_RouterData.end()) {
		router_device = router_it->second.deviceId;
		std::stringstream ss;
		ss << "s 4 " << channel << " \r";
		const auto index = m_device_to_id[router_it->first];
		auto& serialCom = serialComConnectionInfoSet[index];
		SendCommand(serialCom, ss.str());
		SendCommand(serialCom, "v \r");
		SendPacket(serialCom, DEV_CMD_CONFIG_REQ);
	}
	show_configuration_data show;
	show.add(timekeeper_device, router_device, channel); 
	auto end = m_comport_to_beaconData.end();
	for(auto it = m_comport_to_beaconData.begin(); it != end; ++it) {
		std::stringstream ss;
		const std::string& device = it->second.deviceId;
		auto data = show.GetBeaconConfiguration(device);
		const std::string& comport = it->first;
		const auto index = m_device_to_id[comport];
		auto& serialCom = serialComConnectionInfoSet[index];
		ss << "s 8 " << static_cast<int>(data.channel) << " \r\n";
		ss << "s 9 " << static_cast<int>(data.led1) << " \r\n";
		ss << "s 10 " << static_cast<int>(data.led2) << " \r\n";
		ss << "s 11 " << static_cast<int>(data.led3) << " \r\n";
		ss << "s 13 " << static_cast<int>(data.slot) << " \r\n";
		ss << "v \r\n";
		std::string command;
		while(std::getline(ss, command)) {
			SendCommand(serialCom, command);
		}
		SendPacket(serialCom, DEV_CMD_CONFIG_REQ);
	}
}

void charging_station::on_tkSetWirelessChannel_clicked() {
	auto found = m_comport_to_TimekeeperData.find(current_timekeeper_comport);
	if(found == m_comport_to_TimekeeperData.end()) return;
	int channel = ui.tkWirelessDropDown->currentIndex() + 9;
	updateView();
	const auto idx = m_device_to_id[current_timekeeper_comport];
	auto& serialCom = serialComConnectionInfoSet[idx];
	std::stringstream ss;
	m_comport_to_timekeeperConfig.erase(current_timekeeper_comport); //invalidate the current config
	ss << "s 4 " << channel << " \r";
	SendCommand(serialCom, ss.str());
	SendCommand(serialCom, "v \r");
	SendPacket(serialCom, DEV_CMD_CONFIG_REQ);
	current_timekeeper_comport = "";
}

void charging_station::SendBatteryRequest() {
	auto end = m_comport_to_beaconData.end();
	for(auto it = m_comport_to_beaconData.begin(); it != end; ++it) {
		const std::string& comport = it->first;
		int index = m_device_to_id[comport];
		auto& serialCom = serialComConnectionInfoSet[index];
		SendPacket(serialCom, DEV_CMD_RUNNING_STATUS_REQ);
	}
}

float charging_station::GetBatteryPercentage(boost::uint8_t battery) {			// Code from Bridge
	if (battery >= 225) return 100.0;
	if (battery <= 187) return 0.0;
	float p = (float)((1.0 * battery - 187.0) / (225.0 - 187.0) * 100.0);
	if (p > 100.0) p = 100;
	return p;
}



