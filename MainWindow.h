#ifndef __GROUND_STATION_H__
#define __GROUND_STATION_H__

#include <QMainWindow>
#include "qcustomplot.h"
#include "NewtonCalibration.h"
#include <Eigen/Eigen>

using namespace Eigen;


struct Angular_Pos
{
	float roll;
	float pitch;
	float yaw;
	Angular_Pos() :roll(0), pitch(0), yaw(0) {}
	Angular_Pos(float p, float r, float y) : roll(r), pitch(p), yaw(y) {}
};

struct Original_Data
{
	float ax;
	float ay;
	float az;
	float gx;
	float gy;
	float gz;

	Original_Data() :ax(0), ay(0), az(0), gx(0), gy(0), gz(0) {}
	Original_Data(float ax, float ay, float az, float gx, float gy, float gz)
		: ax(ax), ay(ay), az(az), gx(gx), gy(gy), gz(gz) {}
};

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
		ORIGINAL,		//MPU6050ԭʼ����
		ZeroPad,		//��ƫ�������ʾ
		Angle,			//�۲�Ƕ�
		UnitTransfer,	//����λ�����Ժ������
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

	enum ESTIMATION_WAY
	{
		NONE,
		COMPLEMENTION_FILTER,	//�����˲�
		ACCELMETER,	//���ü��ٶȼ��㡣
		GYRO,		//���������Ǽ��㡣
		EKF,		//��չ�������˲�
	};

public:
	GroundStation(QWidget* parent);
	~GroundStation();
	void init();
	QWidget* initCalibrationPage();
	QWidget* initAttitudePage();
	QWidget* initEKFPage();
	void initPort();
	void initCalibrationPlot();
	void initAttitudePlot();
	void initParameters();
	void initMatrix();

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
	void estimateByEKFOffline();

private:
	Angular_Pos EstimateByAccel(float xaccel, float yaccel, float zaccel);
	Angular_Pos EstimateByGyro(float gyro_x, float gyro_y, float gyro_z);
	Angular_Pos EstimateByComplementry(float ax, float ay, float az, float gx, float gy, float gz);
	Angular_Pos EstimateByEKF(float ax, float ay, float az, float gx, float gy, float gz);

	QQuaternion MatrixDotQ(QMatrix4x4 M, QQuaternion q);

private:
	Vector4f m_qk;
	//QQuaternion m_qekf;	 //EKFר�õ���Ԫ����
	QQuaternion m_Qgyro; //���ٶȼ������ʱר�õ���Ԫ����
	Matrix4f m_P;		//ekf�����Э�������
	Matrix4f m_Q;		//ekf����ģ�͵������������
	Matrix3f m_V;		//��������Ԫ���ĵ�����
	Matrix3f m_R;
protected:
	void paintEvent(QPaintEvent* event);

private:
	QTextBrowser* m_pTextBrowser;
	QCheckBox* cbXAccel, *cbYAccel, *cbZAccel, *cbZeroPadAccel, *cbUnitAccel, *cbXGyro, *cbYGyro, *cbZGyro, *cbZeroPadGyro, *cbUnitGyro;

	QTextBrowser *m_pStateBrowser, *m_pKKBrowser, *m_pHKBrowser, *m_pPBrowser;

	QCustomPlot* m_gyro;
	QCustomPlot* m_accel;
	QCustomPlot* m_angle;
	QCustomPlot* m_gloss;

	QCustomPlot* m_attitude_accel;
	QCustomPlot* m_attitude_gyro;
	QCustomPlot* m_attitude_com_filter;
	QCustomPlot* m_attitude_mixed;
	QCustomPlot* m_attitude_ekf;
	
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
	QTimer dtAttitude_ekf;
	QTimer dtAttitude_com;
	QTimer dtAttitude_mixed;
	ZeroPadding zeropad;
	PlotType m_plotAccel;
	PlotType m_plotGyro;
	PlotType mainType;

	//���ٶȱ궨����
	Matrix3f TKa;	//��б�Ǿ���
	Matrix3f Tkg;	//�����ǲ�������
	Vector3f ba;		//bias;
	Vector3f bg;
	Vector3f bg_;	//�����ռ���

	std::vector<Original_Data> m_vecCollectData;

	NewtonCalibration* m_pCalibration;
	MatrixXf X;
	MatrixXf X_sixSide;
	int m_idxX;
	int m_idxGyro;
	bool m_bCollect4Calibrate;
	bool m_bGyroCalibrate;
	bool m_bAverage4sixside;
	bool m_bEnableEKFFilter;
	bool m_bCollectingData;		//�ռ������������ڿ������˲�

	CALIBRATION_SIDE m_side;
	CALIBRATION_TYPE m_type;
	ESTIMATION_WAY m_estimation_way;
	static float g;
};

#endif // __GROUND_STATION_H__
