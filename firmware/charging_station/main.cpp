#include "charging_station.h"
#include <QtGui/QApplication>

int main(int argc, char *argv[]) {
	QApplication a(argc, argv);
	charging_station w;
	w.show();
	QFile File(":/Styling/stylesheet.css");
	File.open(QFile::ReadOnly);
	QString StyleSheet = QLatin1String(File.readAll());
	a.setStyleSheet(StyleSheet);
	return a.exec();
}
