// SerialPortAssist.cpp : 定义应用程序的入口点。
//

#include "framework.h"
#include "SerialPortAssist.h"

int WINAPI WinMain(__in HINSTANCE hInstance,
	__in_opt HINSTANCE hPrevInstance,
	__in LPSTR lpCmdLine,
	__in int nShowCmd)
{
	QApplication app(__argc, __argv);
	QSerialPort m_reader;
	foreach (const QSerialPortInfo & info, QSerialPortInfo::availablePorts())
	{
		QString name = "Name: " + info.portName();
		qDebug(name.toUtf8().data());
		//qDebug() << "Description : " << info.description();
		//qDebug() << "Manufacturer: " << info.manufacturer();
		qDebug(info.serialNumber().toUtf8().data());
		QString manufactuer = "Manufacturer: " + info.manufacturer();
		qDebug(manufactuer.toUtf8().data());
		if (info.manufacturer() == "Silicon Labs")
		{
			m_reader.setPort(info);
			m_reader.setParity(QSerialPort::NoParity);
			//m_reader.setDataBits(QSerialPort::UnknownDataBits);
			//m_reader.setStopBits(QSerialPort::UnknownStopBits);
			m_reader.setBaudRate(QSerialPort::Baud115200);
			m_reader.setFlowControl(QSerialPort::NoFlowControl);

			m_reader.clearError();
			m_reader.clear();

		}
		//qDebug() << "System Location : " << info.systemLocation();
	}

	app.exec();
	return 0;
}

