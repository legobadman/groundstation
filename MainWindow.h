#ifndef __GROUND_STATION_H__
#define __GROUND_STATION_H__

#include <QMainWindow>
#include "qcustomplot.h"
#include "NewtonCalibration.h"
#include <Eigen/Eigen>

using namespace Eigen;

//struct Quaternion
//{
//	float q0;
//	float q1;
//	float q2;
//	float q3;
//	Quaternion() : q0(0), q1(0), q2(0), q3(0) {}
//	Quaternion(float q0, float q1, float q2, float q3) : q0(q0), q1(q1), q2(q2), q3(q3) {}
//};

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

	enum CALIBRATION_TYPE
	{
		ONE_SIDE,
		SIX_SIDE
	};

	enum CALIBRATION_SIDE
	{
		SIDE_UP,
		SIDE_DOWN,
		SIDE_LEFT,
		SIDE_RIGHT,
		SIDE_FRONT,
		SIDE_BACK
	};

public:
	GroundStation(QWidget* parent);
	~GroundStation();
	void init();
	QWidget* initCalibrationPage();
	QWidget* initAttitudePage();
	void initPort();
	void initCalibrationPlot();
	void initAttitudePlot();
	void initParameters();

signals:
	void operate(const QString&);
	void calibrationReady(const MatrixXf& X);

public slots:
	void accel_calibration();
	void on_start_collect4calibration();
	void six_sided_calibration();
	void onCustomCOMRead();
	void MyRealtimeDataSlot();
	void onAccelTimeout();
	void onGyroTimeout();
	void onPlotAccelChanged(QAbstractButton* pClickedButton);
	void onPlotGyroChanged(QAbstractButton* pClickedButton);
	void onCalibrationReady(const MatrixXf& X);
	void setButtonIcon(int);
	void onGyroCalibration();

private:
	QQuaternion m_Q;

protected:
	void paintEvent(QPaintEvent* event);

private:
	QTextBrowser* m_pTextBrowser;
	QCheckBox* cbXAccel, *cbYAccel, *cbZAccel, *cbZeroPadAccel, *cbUnitAccel, *cbXGyro, *cbYGyro, *cbZGyro, *cbZeroPadGyro, *cbUnitGyro;

	QCustomPlot* m_gyro;
	QCustomPlot* m_accel;
	QCustomPlot* m_angle;
	QCustomPlot* m_gloss;

	QCustomPlot* m_attitude_accel;
	QCustomPlot* m_attitude_gyro;
	QCustomPlot* m_attitude_com_filter;
	QCustomPlot* m_attitude_mixed;
	
	QTabWidget* m_pTabWidget;
	QWidget* m_pCalibrationPage;
	QWidget* m_pAttitudePage;

	QPushButton* m_pOneSideBtn;
	QPushButton* m_pSixSideBtn;
	QPushButton* m_pGyroCalibrateBtn;
	QMovie* myMovie;
	QLabel* m_pTip;
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
	QTimer dtAttitude_accel;
	QTimer dtAttitude_gyro;
	QTimer dtAttitude_com;
	QTimer dtAttitude_mixed;
	ZeroPadding zeropad;
	PlotType m_plotAccel;
	PlotType m_plotGyro;
	PlotType mainType;

	//加速度标定参数
	Matrix3f TKa;	//倾斜角矩阵
	Matrix3f Tkg;	//陀螺仪参数矩阵
	Vector3f ba;		//bias;
	Vector3f bg;
	Vector3f bg_;	//用于收集。

	NewtonCalibration* m_pCalibration;
	MatrixXf X;
	MatrixXf X_sixSide;
	int m_idxX;
	int m_idxGyro;
	bool m_bCollect4Calibrate;
	bool m_bGyroCalibrate;
	bool m_bAverage4sixside;
	CALIBRATION_SIDE m_side;
	CALIBRATION_TYPE m_type;
	static float g;
};

#endif // __GROUND_STATION_H__
