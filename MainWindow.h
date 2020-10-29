#ifndef __GROUND_STATION_H__
#define __GROUND_STATION_H__

#include <QMainWindow>
#include "qcustomplot.h"

class GroundStation : public QMainWindow
{
	Q_OBJECT
	enum ZeroPadding
	{
		NO_ZEROPAD,
		ZEROPADDING,
		ZEROPADDED,
	};
	enum PlotType
	{
		ORIGINAL,		//MPU6050原始数据
		ZeroPad,		//零偏结果的显示	
		UnitTransfer,	//经单位换算以后的数据
	};

public:
	GroundStation(QWidget* parent);
	~GroundStation();
	void init();
	void initPort();
	void initAccelPlot();
	void initGyroPlot();
	void initAnglePlot();

public slots:
	void onReadyRead();
	void MyRealtimeDataSlot();
	void onAccelTimeout();
	void onGyroTimeout();
	void prepare_zeropadding(bool);
	void calculated_zeropad();
	void onPlotAccelChanged(QAbstractButton* pClickedButton);
	void onPlotGyroChanged(QAbstractButton* pClickedButton);

protected:
	void paintEvent(QPaintEvent* event);

private:
	QTextBrowser* m_pTextBrowser;
	QCheckBox* cbXAccel, *cbYAccel, *cbZAccel, *cbZeroPadAccel, *cbUnitAccel, *cbXGyro, *cbYGyro, *cbZGyro, *cbZeroPadGyro, *cbUnitGyro;

	QCustomPlot* m_gyro;
	QCustomPlot* m_accel;
	QCustomPlot* m_angle;
	QSerialPort m_serialPort;
	QString m_msgStream;
	QVector<int> m_accelX;
	QVector<int> m_accelY;
	QVector<int> m_accelZ;
	QVector<int> m_gyroX;
	QVector<int> m_gyroY;
	QVector<int> m_gyroZ;
	int m_currIdx;
	QTimer dtaccel;
	QTimer dtgyro;
	QTimer dtangle;
	QTimer zeropad_timer;
	ZeroPadding zeropad;
	PlotType m_plotAccel;
	PlotType m_plotGyro;
};


#endif // __GROUND_STATION_H__
