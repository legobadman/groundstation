#ifndef __GROUND_STATION_H__
#define __GROUND_STATION_H__

#include <QMainWindow>
#include "qcustomplot.h"

class GroundStation : public QMainWindow
{
	Q_OBJECT
public:
	GroundStation(QWidget* parent);
	~GroundStation();
	void init();
	void initPort();
	void initAccelPlot();
	void initGyroPlot();

public slots:
	void onReadyRead();
	void MyRealtimeDataSlot();
	void onAccelTimeout();
	void onGyroTimeout();

protected:
	void paintEvent(QPaintEvent* event);

private:
	QTextBrowser* m_pTextBrowser;
	QCustomPlot* m_gyro;
	QCustomPlot* m_accel;
	QSerialPort m_serialPort;
	QString m_msgStream;
	QList<int> m_accelX;
	QList<int> m_accelY;
	QList<int> m_accelZ;

	QVector<int> m_winX;
	QList<int> m_winY;
	QList<int> m_winZ;
	int m_currIdx;
	QTimer dtaccel;
	QTimer dtgyro;
};


#endif // __GROUND_STATION_H__
