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
	void initPlot();
	void setup();

public slots:
	void onReadyRead();
	void MyRealtimeDataSlot();

protected:
	void paintEvent(QPaintEvent* event);

private:
	QTextBrowser* m_pTextBrowser;
	QCustomPlot* m_plot;
	QSerialPort m_serialPort;
	QString m_msgStream;
	QList<int> m_accelX;
	QList<int> m_accelY;
	QList<int> m_accelZ;

	QVector<int> m_winX;
	QList<int> m_winY;
	QList<int> m_winZ;
	int m_currIdx;
	QTimer dataTimer;
};


#endif // __GROUND_STATION_H__
