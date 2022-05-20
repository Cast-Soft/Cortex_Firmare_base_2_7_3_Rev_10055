#pragma once
#include <QAbstractTableModel>
#include <QString>
#include <string>
#include <boost\lexical_cast.hpp>
#include <QPixmap>

class TableData {
public:
	std::string connection_status;
	std::string deviceId;
	std::string m_com_port;
	std::string current_mode_type;
	std::string id;
	unsigned int sent_data;
	unsigned int receive_data;

public:
	TableData()
		: deviceId("?")
		, connection_status("Disconnected")
		, m_com_port("COM?")
		, current_mode_type("?")
		, id("?")
		, sent_data(0)
		, receive_data(0)
	{}

	TableData(const std::string& com_port)
		: deviceId("?")
		, connection_status("Disconnected")
		, m_com_port(com_port)
		, current_mode_type("?")
		, id()
		, sent_data(0)
		, receive_data(0)
	{}

	//TODO: remove this and access the data directly.
	void data(int index, const std::string& value) {
		switch(index) {
		case 0: {
			deviceId = value;
				}
		case 1: {
			m_com_port = value;
				}
		case 2: {
			connection_status = value;
				}
		case 3: {
			current_mode_type = value;
				}
		case 4: {
			id = value;
				}
		case 5:{
			sent_data = std::stoi(value);
			   }
		case 6: {
			receive_data = std::stoi(value);
				}
		}
	}

	const std::string operator[](int index) const {
		switch(index) {
		case 0: return deviceId;
		case 1: return m_com_port;
		case 2: return connection_status;
		case 3: return current_mode_type;
		case 4: return id;
		case 5: return boost::lexical_cast<std::string>(sent_data);
		case 6: return boost::lexical_cast<std::string>(receive_data);
		}

		return "unknow index";
	}
};

class DeviceTable : public QAbstractTableModel {
	QList<TableData> m_data;
public:
    DeviceTable(QObject *parent);
    int rowCount(const QModelIndex &parent = QModelIndex()) const ;
    int columnCount(const QModelIndex &parent = QModelIndex()) const;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
    bool setData(const QModelIndex & index, const QVariant & value, int role = Qt::EditRole);
	QVariant headerData (int section, Qt::Orientation orientation, int role = Qt::DisplayRole ) const;
    Qt::ItemFlags flags(const QModelIndex & index) const;
	int add(TableData d);
	void edit(int row, int col, const std::string& value);
	void clear();
	const TableData getData(int row) const;
	//void update(int index);
};

inline DeviceTable::DeviceTable(QObject *parent) {

}

inline Qt::ItemFlags DeviceTable::flags(const QModelIndex & index) const {
    return Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsUserCheckable;
}

inline QVariant DeviceTable::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (role != Qt::DisplayRole ) return QVariant();
	if (orientation != Qt::Horizontal)  return QVariant();
	switch(section) {
	case 0: return QString("Device ID");
	case 1: return QString("COM Port");
	case 2: return QString("Connection Status");
	case 3: return QString("Firmware");
	case 4: return QString("ID");
	case 5: return QString("Sent");
	case 6: return QString("Recieve");
	}
	return QVariant();
}

inline QVariant DeviceTable::data(const QModelIndex &index, int role) const
{
    int row = index.row();
    int col = index.column();

	if(role == Qt::DecorationRole && col == 2) {
		const auto& t = m_data.at(row);
		std::string value = t[col];
		value.erase(std::remove(value.begin(), value.end(), '%'), value.end());
		std::stringstream ss(value);
		float percent = 0.0f;
		if(ss >> percent) {
			if (percent < 10.0) {
				return QPixmap(":/Battery/battery0.png").scaledToHeight(12);
			} else if (percent < 30.0) {
				return QPixmap(":/Battery/battery1.png").scaledToHeight(12);;
			} else if (percent < 50.0) {
				return QPixmap(":/Battery/battery2.png").scaledToHeight(12);;
			} else if (percent < 70.0) {
				return QPixmap(":/Battery/battery3.png").scaledToHeight(12);;
			} else if (percent < 90.0) {
				return QPixmap(":/Battery/battery4.png").scaledToHeight(12);;
			} else {
				return QPixmap(":/Battery/battery5.png").scaledToHeight(12);;
			}
		}
	}

	if (role != Qt::DisplayRole) return QVariant();
	if(m_data.size() > row) {
		const auto& t = m_data.at(row);
		const std::string value = t[col];
		return QString(value.c_str());
	}
    return QVariant();
}

inline bool DeviceTable::setData(const QModelIndex & index, const QVariant & value, int role)
{
	auto row = index.row();
    auto col = index.column();
    if (role == Qt::EditRole) {
		auto & t = m_data[row];
		t.data(col, value.toString().toStdString());
    }
    return true;
}

inline int DeviceTable::rowCount(const QModelIndex &parent) const {
	return m_data.size();
}

inline int DeviceTable::columnCount(const QModelIndex &parent) const {
	return 7;
}

inline int DeviceTable::add(TableData d) {
	auto index = m_data.count();

	beginInsertRows(QModelIndex(), index, index);
	m_data.push_back(d);
	endInsertRows();

	std::size_t cols = columnCount(QModelIndex());
	QModelIndex top = createIndex(index - 1, 0, 0);
    QModelIndex bottom = createIndex(index - 1, cols, 0);
    emit dataChanged(top, bottom);
	return index;
}

inline void DeviceTable::edit(int row, int col, const std::string& value) {
	auto & b = m_data[row];
	b.data(col, value);
}

inline void DeviceTable::clear() {
	m_data.clear();
	this->reset();
}

inline const TableData DeviceTable::getData(int row) const {
	return m_data.at(row);
}

