#pragma once
#include <QAbstractTableModel>
#include <QString>

namespace blacktrax {
namespace table {

template<typename T>
struct Flags {
	Qt::ItemFlags operator()(const QModelIndex & index) const {
		switch(index.row()) {
		case 0: return Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsUserCheckable;
		case 1: return Qt::ItemIsEnabled;
		}

		return Qt::ItemIsEnabled;
	}
};

template<typename T>
class table_model : public QAbstractTableModel {
	QList<T> m_data;
	Flags<T> m_flags;
public:
    table_model(QObject *parent);
    int rowCount(const QModelIndex &parent = QModelIndex()) const ;
    int columnCount(const QModelIndex &parent = QModelIndex()) const;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
    bool setData(const QModelIndex & index, const QVariant & value, int role = Qt::EditRole);
	QVariant headerData (int section, Qt::Orientation orientation, int role = Qt::DisplayRole ) const;
    Qt::ItemFlags flags(const QModelIndex & index) const;
	void add(T t);
	void update(int index);
};

template<typename T> 
inline table_model<T>::table_model(QObject *parent) {

}

template<typename T> 
inline Qt::ItemFlags table_model<T>::flags(const QModelIndex & index) const {
    return m_flags(index);
}

template<typename T>
inline QVariant table_model<T>::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (role != Qt::DisplayRole ) return QVariant();
	if (orientation != Qt::Horizontal)  return QVariant();
	std::string s = T::header(section);
	return QString(s.c_str());
}

template<typename T>
inline QVariant table_model<T>::data(const QModelIndex &index, int role) const
{
    int row = index.row();
    int col = index.column();
	if (role != Qt::DisplayRole) return QVariant();
	if(m_data.size() > row) {
		const T& t = m_data.at(row);
		const std::string value = t[col];
		return QString(value.c_str());
	}
    return QVariant();
}

template<typename T>
inline bool table_model<T>::setData(const QModelIndex & index, const QVariant & value, int role)
{
	auto row = index.row();
    auto col = index.column();
    if (role == Qt::EditRole) {
		T & t = m_data[row];
		t.data(col, value.toString().toStdString());
    }
    return true;
}

template<typename T>
inline int table_model<T>::rowCount(const QModelIndex &parent) const {
	return m_data.size();
}

template<typename T>
inline int table_model<T>::columnCount(const QModelIndex &parent) const {
	return T::size();
}

template<typename T>
void table_model<T>::add(T t) {
	auto index = m_data.count();
	beginInsertRows(QModelIndex(), index, index);
	m_data.push_back(t);
	endInsertRows();

	std::size_t cols = T::size();
	QModelIndex top = createIndex(index - 1, 0, 0);
    QModelIndex bottom = createIndex(index - 1, cols, 0);
    emit dataChanged(top, bottom);
}

template<typename T>
void table_model<T>::update(int index) {
}

}
}
