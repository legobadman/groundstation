// SerialPortAssist.cpp : 定义应用程序的入口点。
//

#include "framework.h"
#include "SerialPortAssist.h"
#include "MainWindow.h"

int WINAPI WinMain(__in HINSTANCE hInstance,
	__in_opt HINSTANCE hPrevInstance,
	__in LPSTR lpCmdLine,
	__in int nShowCmd)
{
	QApplication app(__argc, __argv);
	QSerialPort m_reader;
	GroundStation station(NULL);
	station.show();
	app.exec();
	return 0;
}

