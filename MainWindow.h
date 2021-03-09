#ifndef __GROUND_STATION_H__
#define __GROUND_STATION_H__

#include <QMainWindow>
#include "qcustomplot.h"
#include "NewtonCalibration.h"
#include <Eigen/Eigen>

using namespace Eigen;

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
		UNKNOWN,
		ORIGINAL,		//MPU6050原始数据
		ZeroPad,		//零偏结果的显示
		Angle,			//观察角度
		UnitTransfer,	//经单位换算以后的数据
	};

public:
	GroundStation(QWidget* parent);
	~GroundStation();
	void init();
	void initSimple();
	void initPort();
	void initSimplePlot();
	void initGlossPlot();
	void initAccelPlot();
	void initGyroPlot();
	void initAnglePlot();
	void initParameters();

signals:
	void operate(const QString&);
	void calibrationReady(const MatrixXf& X);

public slots:
	void accel_calibration();
	void onReadyRead();
	void onCustomCOMRead();
	void MyRealtimeDataSlot();
	void onAccelTimeout();
	void onGyroTimeout();
	void prepare_zeropadding(bool);
	void calculated_zeropad();
	void onPlotAccelChanged(QAbstractButton* pClickedButton);
	void onPlotGyroChanged(QAbstractButton* pClickedButton);
	void onCalibrationReady(const MatrixXf& X);
	void setButtonIcon(int);

protected:
	void paintEvent(QPaintEvent* event);

private:
	QTextBrowser* m_pTextBrowser;
	QCheckBox* cbXAccel, *cbYAccel, *cbZAccel, *cbZeroPadAccel, *cbUnitAccel, *cbXGyro, *cbYGyro, *cbZGyro, *cbZeroPadGyro, *cbUnitGyro;

	QCustomPlot* m_gyro;
	QCustomPlot* m_accel;
	QCustomPlot* m_angle;
	QCustomPlot* m_gloss;
	QPushButton* m_pBtn;
	QMovie* myMovie;
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
	QTimer dtgloss;
	QTimer zeropad_timer;
	ZeroPadding zeropad;
	PlotType m_plotAccel;
	PlotType m_plotGyro;
	PlotType mainType;

	//加速度标定参数
	Matrix3f TK;		//倾斜角矩阵
	Vector3f b;		//bias;

	NewtonCalibration* pCalibration;
	MatrixXf X;
	int m_idxX;
	bool m_bCalibrating;
	bool m_bCollect4Calibrate;

	static float g;
};

#endif // __GROUND_STATION_H__
