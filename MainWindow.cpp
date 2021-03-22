#include "framework.h"
#include "MainWindow.h"
#include "moc_MainWindow.cpp"

float angle[3] = { 0 };
float Q[4] = { 1, 0, 0, 0 };
float pitch_com = 0, roll_com = 0;
float T = 0.01f;

const float fRad2Deg = 57.295779513f;	//弧度换算角度乘的系数
const float fDeg2Rad = 0.01745329252f;	

#define MIN std::min
#define MAX std::max
#define BOUND_ABS_ONE(x) (MAX(-1.0f, MIN(1.0f, (x))))
#define CALIBRATION_SAMPLES 1000	//每一面较准所采样的数量。

float GroundStation::g = 9.7883f;


GroundStation::GroundStation(QWidget* parent)
	: QMainWindow(parent)
	, m_currIdx(0)
	, zeropad(NO_ZEROPAD)
	, m_plotAccel(ORIGINAL)
	, m_plotGyro(ORIGINAL)
	, mainType(UNKNOWN)
	, m_pCalibration(NULL)
	, m_pSixSideBtn(NULL)
	, m_bCollect4Calibrate(false)
	, m_idxX(0)
	, m_idxGyro(0)
	, m_side(SIDE_UP)
	, m_type(ONE_SIDE)
	, m_pTip(NULL)
	, m_bAverage4sixside(false)
	, m_bGyroCalibrate(false)
	, m_pTabWidget(NULL)
	, m_pCalibrationPage(NULL)
	, m_pAttitudePage(NULL)
	, m_estimation_way(NONE)
	, m_bEnableEKFFilter(false)
	, m_bCollectingData(false)
{
	init();
}

GroundStation::~GroundStation()
{
	if (m_pCalibration) {
		delete m_pCalibration;
	}
}

void GroundStation::init()
{
	QTabWidget* pTabWidget = new QTabWidget(this);

	QWidget* pCalibrationPage = initCalibrationPage();
	QWidget* pAttitudePage = initAttitudePage();
	QWidget* pEKFPage = initEKFPage();
	pTabWidget->addTab(pCalibrationPage, u8"传感器标定");
	pTabWidget->addTab(pAttitudePage, u8"姿态解算");
	pTabWidget->addTab(pEKFPage, u8"扩展卡尔曼滤波");
	pTabWidget->setCurrentIndex(2);

	initCalibrationPlot();
	initAttitudePlot();
	initPort();
	initParameters();
	initMatrix();

	setCentralWidget(pTabWidget);
	resize(1304, 691);
}

QWidget* GroundStation::initCalibrationPage()
{
	m_pCalibrationPage = new QWidget;

	m_accel = new QCustomPlot(m_pCalibrationPage);
	m_accel->setMinimumHeight(600);
	QVBoxLayout* pMainLayout = new QVBoxLayout;

	QHBoxLayout* pHBoxLayout = new QHBoxLayout;

	m_pOneSideBtn = new QPushButton(m_pCalibrationPage);
	m_pOneSideBtn->setText(u8"加速度计较准");
	pHBoxLayout->addWidget(m_pOneSideBtn);

	m_pSixSideBtn = new QPushButton(m_pCalibrationPage);
	m_pSixSideBtn->setText(u8"六面较准");
	pHBoxLayout->addWidget(m_pSixSideBtn);

	m_pGyroCalibrateBtn = new QPushButton(m_pCalibrationPage);
	m_pGyroCalibrateBtn->setText(u8"陀螺仪零偏较准");
	pHBoxLayout->addWidget(m_pGyroCalibrateBtn);

	m_pOneSideBtn->setIcon(QIcon(":/images/copy.png"));

	//myMovie = new QMovie("circle.gif", QByteArray(), m_pOneSideBtn);
	//myMovie->setScaledSize(QSize(20, 20));
	//connect(myMovie, SIGNAL(frameChanged(int)), this, SLOT(setButtonIcon(int)));
	//// if movie doesn't loop forever, force it to.
	//if (myMovie->loopCount() != -1)
	//	connect(myMovie, SIGNAL(finished()), myMovie, SLOT(start()));
	//if (myMovie->isValid())
	//	myMovie->start();

	connect(m_pOneSideBtn, SIGNAL(clicked()), this, SLOT(accel_calibration()));
	connect(m_pSixSideBtn, SIGNAL(clicked()), this, SLOT(six_sided_calibration()));
	connect(m_pGyroCalibrateBtn, SIGNAL(clicked()), this, SLOT(onGyroCalibration()));

	pHBoxLayout->addSpacerItem(new QSpacerItem(100, 20, QSizePolicy::Preferred, QSizePolicy::Expanding));

	m_pTip = new QLabel(m_pCalibrationPage);
	pHBoxLayout->addWidget(m_pTip);
	m_pTip->setText(u8"陀螺仪向上：");

	QPushButton* pStartBtn = new QPushButton(m_pCalibrationPage);
	pStartBtn->setText(u8"开始收集");
	connect(pStartBtn, SIGNAL(clicked()), this, SLOT(on_start_collect4calibration()));
	pHBoxLayout->addWidget(pStartBtn);

	pMainLayout->addItem(pHBoxLayout);

	m_pTextBrowser = new QTextBrowser(m_pCalibrationPage);
	m_pTextBrowser->setText("");

	pMainLayout->addWidget(m_pTextBrowser);
	pMainLayout->addWidget(m_accel);

	m_pCalibrationPage->setLayout(pMainLayout);
	return m_pCalibrationPage;
}

#define PLOT_ONLY_MIXED2	//

QWidget* GroundStation::initAttitudePage()
{
	QWidget* pPage = new QWidget;
	QGridLayout* pGridLayout = new QGridLayout;

#ifndef PLOT_ONLY_MIXED
	m_attitude_accel = new QCustomPlot(pPage);
	m_attitude_gyro = new QCustomPlot(pPage);
	m_attitude_com_filter = new QCustomPlot(pPage);
	pGridLayout->addWidget(m_attitude_accel, 0, 0);
	pGridLayout->addWidget(m_attitude_gyro, 0, 1);
	pGridLayout->addWidget(m_attitude_com_filter, 1, 0);
#else
	m_attitude_mixed = new QCustomPlot(pPage);
	pGridLayout->addWidget(m_attitude_mixed, 0, 0);
#endif

	pPage->setLayout(pGridLayout);
	return pPage;
}

QWidget* GroundStation::initEKFPage()
{
	QWidget* pPage = new QWidget;
	QHBoxLayout* pHBoxLayout = new QHBoxLayout;
	QVBoxLayout* pRightSide = new QVBoxLayout;

	m_attitude_ekf = new QCustomPlot(pPage);
	m_attitude_ekf->setMinimumWidth(800);

	m_pStateBrowser = new QTextBrowser(pPage);
	m_pKKBrowser = new QTextBrowser(pPage);
	m_pHKBrowser = new QTextBrowser(pPage);
	m_pPBrowser = new QTextBrowser(pPage);

	QPushButton* pBtn = new QPushButton(pPage);
	pBtn->setText(u8"开始滤波");
	QObject::connect(pBtn, &QPushButton::clicked, [=]() {
			m_bEnableEKFFilter = true;
			//estimateByEKFOffline();
		});

	QPushButton* pCollectBtn = new QPushButton(pPage);
	pCollectBtn->setText(u8"收集数据");
	QObject::connect(pCollectBtn, &QPushButton::clicked, [=]() {
			if (m_bCollectingData)
			{
				pCollectBtn->setText(u8"开始收集");
				//写盘。
				QFile fn("X.txt");
				fn.open(QIODevice::ReadWrite);
				for (int i = 0; i < m_vecCollectData.size(); i++)
				{
					float ax = m_vecCollectData[i].ax, ay = m_vecCollectData[i].ay, az = m_vecCollectData[i].az,
						gx = m_vecCollectData[i].gx, gy = m_vecCollectData[i].gy, gz = m_vecCollectData[i].gz;
					QString line = QString("%1, %2, %3, %4, %5, %6\n").arg(ax).arg(ay).arg(az).arg(gx).arg(gy).arg(gz);
					fn.write(line.toUtf8());
				}
				fn.close();
				m_vecCollectData.clear();
				m_bCollectingData = false;
			}
			else
			{
				pCollectBtn->setText(u8"停止收集");
				m_bCollectingData = true;
			}
		});

	//QAction* playAct = new QAction(QIcon(":/images/play_btn.png"), tr("&Run"), pPage);
	//playAct->setCheckable(true);
	//playAct->setStatusTip(tr("Run physics"));

	pRightSide->addWidget(pBtn);
	pRightSide->addWidget(pCollectBtn);
	pRightSide->addStretch();

	pRightSide->addWidget(new QLabel(u8"当前状态：", pPage));
	pRightSide->addWidget(m_pStateBrowser);

	pRightSide->addWidget(new QLabel(u8"增益矩阵Kk：", pPage));
	pRightSide->addWidget(m_pKKBrowser);

	pRightSide->addWidget(new QLabel(u8"H矩阵：", pPage));
	pRightSide->addWidget(m_pHKBrowser);

	pRightSide->addWidget(new QLabel(u8"误差协方差矩阵：", pPage));
	pRightSide->addWidget(m_pPBrowser);

	pHBoxLayout->addWidget(m_attitude_ekf);
	pHBoxLayout->addStretch();
	pHBoxLayout->addLayout(pRightSide);
	pHBoxLayout->addStretch();

	pPage->setLayout(pHBoxLayout);
	return pPage;
}

void GroundStation::onPlotAccelChanged(QAbstractButton* pClickedButton)
{
	if (pClickedButton->text() == u8"原始数据")
	{
		m_plotAccel = ORIGINAL;
		m_accel->yAxis->setRange(0, 65535, Qt::AlignCenter);
	}
	else if (pClickedButton->text() == u8"单位换算")
	{
		m_plotAccel = UnitTransfer;
		m_accel->yAxis->setRange(0, 4, Qt::AlignCenter);
	}
	else if (pClickedButton->text() == u8"零点漂移")
	{
		m_plotAccel = ZeroPad;
		m_accel->yAxis->setRange(0, 65535, Qt::AlignCenter);
	}
}

void GroundStation::onPlotGyroChanged(QAbstractButton* pClickedButton)
{
	if (pClickedButton->text() == u8"原始数据")
	{
		m_plotGyro = ORIGINAL;
		m_gyro->yAxis->setRange(0, 65535, Qt::AlignCenter);
	}
	else if (pClickedButton->text() == u8"单位换算")
	{
		m_plotGyro = UnitTransfer;
		m_gyro->yAxis->setRange(0, 360, Qt::AlignCenter);
	}
	else if (pClickedButton->text() == u8"零点漂移")
	{
		m_plotGyro = ZeroPad;
		m_gyro->yAxis->setRange(0, 1000, Qt::AlignCenter);
	}
}

void GroundStation::initPort()
{
	foreach(const QSerialPortInfo & info, QSerialPortInfo::availablePorts())
	{
		QString manufacturer = info.manufacturer();
		if (manufacturer == "Silicon Labs" || manufacturer == "FTDI")
		{
			m_serialPort.setPort(info);
			m_serialPort.setParity(QSerialPort::EvenParity);
			m_serialPort.setBaudRate(QSerialPort::Baud115200);
			m_serialPort.clearError();
			m_serialPort.clear();

			connect(&m_serialPort, SIGNAL(readyRead()), this, SLOT(onCustomCOMRead()));
			connect(this, SIGNAL(calibrationReady(const MatrixXf&)), this,
				SLOT(onCalibrationReady(const MatrixXf&)));
			m_serialPort.open(QIODevice::ReadOnly);
		}
	}
}

void GroundStation::onAccelTimeout()
{
	m_accel->xAxis->setRange(m_currIdx, 500, Qt::AlignRight);
	m_accel->replot(QCustomPlot::rpQueuedReplot);
}

void GroundStation::onGyroTimeout()
{
	m_gyro->xAxis->setRange(m_currIdx, 500, Qt::AlignRight);
	m_gyro->replot(QCustomPlot::rpQueuedReplot);
}

static void checkIcon(const QIcon& i1)
{
	for (int modeNo = 0; modeNo <= QIcon::Selected; modeNo++) {
		QIcon::Mode mode = (QIcon::Mode)modeNo;
		for (int stateNo = 0; stateNo <= QIcon::On; stateNo++) {
			QIcon::State state = (QIcon::State)stateNo;
			QList<QSize> sizes = i1.availableSizes(mode, state);
			qDebug("%d sizes for %d %d", sizes.size(), mode, state);
		}
	}
}

void GroundStation::paintEvent(QPaintEvent* event)
{
	checkIcon(m_pOneSideBtn->icon());
	QMainWindow::paintEvent(event);
}

void GroundStation::initCalibrationPlot()
{
	m_accel->yAxis->setRange(-50, 180);
	m_accel->addGraph();
	m_accel->graph()->setPen(QPen(Qt::blue));

	m_accel->addGraph();
	m_accel->graph()->setPen(QPen(Qt::red));

	m_accel->addGraph();
	m_accel->graph()->setPen(QPen(Qt::green));

	// make left and bottom axes transfer their ranges to right and top axes:
	connect(m_accel->xAxis, SIGNAL(rangeChanged(QCPRange)), m_accel->xAxis2, SLOT(setRange(QCPRange)));
	connect(m_accel->yAxis, SIGNAL(rangeChanged(QCPRange)), m_accel->yAxis2, SLOT(setRange(QCPRange)));
	connect(&dtaccel, SIGNAL(timeout()), this, SLOT(MyRealtimeDataSlot()));
	dtaccel.start(0);
}

void GroundStation::initAttitudePlot()
{
	if (m_attitude_accel)
	{
		m_attitude_accel->yAxis->setRange(-180, 180);
		{
			m_attitude_accel->addGraph();
			m_attitude_accel->graph()->setPen(QPen(Qt::blue));
			m_attitude_accel->addGraph();
			m_attitude_accel->graph()->setPen(QPen(Qt::red));

			connect(m_attitude_accel->xAxis, SIGNAL(rangeChanged(QCPRange)), m_attitude_accel->xAxis2, SLOT(setRange(QCPRange)));
			connect(m_attitude_accel->yAxis, SIGNAL(rangeChanged(QCPRange)), m_attitude_accel->yAxis2, SLOT(setRange(QCPRange)));
			connect(&dtAttitude_accel, SIGNAL(timeout()), this, SLOT(MyRealtimeDataSlot()));
		}
	}
	if (m_attitude_gyro)
	{
		m_attitude_gyro->yAxis->setRange(-180, 180);
		{
			m_attitude_gyro->addGraph();
			m_attitude_gyro->graph()->setPen(QPen(Qt::blue));
			m_attitude_gyro->addGraph();
			m_attitude_gyro->graph()->setPen(QPen(Qt::red));

			connect(m_attitude_gyro->xAxis, SIGNAL(rangeChanged(QCPRange)), m_attitude_gyro->xAxis2, SLOT(setRange(QCPRange)));
			connect(m_attitude_gyro->yAxis, SIGNAL(rangeChanged(QCPRange)), m_attitude_gyro->yAxis2, SLOT(setRange(QCPRange)));
			connect(&dtAttitude_gyro, SIGNAL(timeout()), this, SLOT(MyRealtimeDataSlot()));
		}
	}
	if (m_attitude_com_filter)
	{
		m_attitude_com_filter->yAxis->setRange(-180, 180);
		{
			m_attitude_com_filter->addGraph();
			m_attitude_com_filter->graph()->setPen(QPen(Qt::blue));
			m_attitude_com_filter->addGraph();
			m_attitude_com_filter->graph()->setPen(QPen(Qt::red));

			connect(m_attitude_com_filter->xAxis, SIGNAL(rangeChanged(QCPRange)), m_attitude_com_filter->xAxis2, SLOT(setRange(QCPRange)));
			connect(m_attitude_com_filter->yAxis, SIGNAL(rangeChanged(QCPRange)), m_attitude_com_filter->yAxis2, SLOT(setRange(QCPRange)));
			connect(&dtAttitude_com, SIGNAL(timeout()), this, SLOT(MyRealtimeDataSlot()));
		}
	}
	if (m_attitude_mixed)
	{
		m_attitude_mixed->yAxis->setRange(-180, 180);
		m_attitude_mixed->addGraph();
		m_attitude_mixed->graph()->setPen(QPen(Qt::blue));
		m_attitude_mixed->addGraph();
		m_attitude_mixed->graph()->setPen(QPen(Qt::red));

		m_attitude_mixed->addGraph();
		m_attitude_mixed->graph()->setPen(QPen(QColor(153, 217, 234)));
		m_attitude_mixed->addGraph();
		m_attitude_mixed->graph()->setPen(QPen(QColor(255, 128, 255)));

		connect(m_attitude_mixed->xAxis, SIGNAL(rangeChanged(QCPRange)), m_attitude_mixed->xAxis2, SLOT(setRange(QCPRange)));
		connect(m_attitude_mixed->yAxis, SIGNAL(rangeChanged(QCPRange)), m_attitude_mixed->yAxis2, SLOT(setRange(QCPRange)));
		connect(&dtAttitude_mixed, SIGNAL(timeout()), this, SLOT(MyRealtimeDataSlot()));
	}
	if (m_attitude_ekf)
	{
		m_attitude_ekf->yAxis->setRange(-180, 180);
		{
			m_attitude_ekf->addGraph();
			m_attitude_ekf->graph()->setPen(QPen(Qt::blue));
			m_attitude_ekf->addGraph();
			m_attitude_ekf->graph()->setPen(QPen(Qt::red));

			connect(m_attitude_ekf->xAxis, SIGNAL(rangeChanged(QCPRange)), m_attitude_ekf->xAxis2, SLOT(setRange(QCPRange)));
			connect(m_attitude_ekf->yAxis, SIGNAL(rangeChanged(QCPRange)), m_attitude_ekf->yAxis2, SLOT(setRange(QCPRange)));
			connect(&dtAttitude_ekf, SIGNAL(timeout()), this, SLOT(MyRealtimeDataSlot()));
		}
	}
}

void GroundStation::initParameters()
{
	X = MatrixXf(CALIBRATION_SAMPLES, 3);
	m_bAverage4sixside = false;
	if (m_bAverage4sixside)
		X_sixSide = MatrixXf(6, 3);
	else
		X_sixSide = MatrixXf(6 * CALIBRATION_SAMPLES, 3);

	//上一次的测量结果。
	//(1.00229812, -0.00134844379, 0.000177600203)
	//(0.00134844379, 1.06192446, -0.00104694557)
	//(-0.000177600203, 0.00104694557, 0.981337011)
	//TKa << 1, 0, 0,
	//	0, 1, 0,
	//	0, 0, 1;
	Q[0] = 1; Q[1] = 0; Q[2] = 0; Q[3] = 0;
	TKa << 1.00229812f, -0.00134844379f, 0.000177600203f,
		0.00134844379f, 1.06192446f, -0.00104694557f,
		-0.000177600203f, 0.00104694557f, 0.981337011f;
	Tkg << 1, 0, 0,
		0, 1, 0,
		0, 0, 1;
	//(-0.0504232645, 0.00252529467, 0.171601593)
	ba << -0.0504232645f, 0.00252529467f, 0.171601593f;
	//ba << 0, 0, 0;
	//(1.76019275, 0.981384218, 0.419433594)
	bg << 1.76019275f, 0.981384218f, 0.419433594f;
	//bg << 0, 0, 0;
	bg_ << 0, 0, 0;
	m_qk << 1, 0, 0, 0;
}

void GroundStation::MyRealtimeDataSlot()
{
	if (m_accel)
	{
		m_accel->xAxis->setRange(m_currIdx, 1000, Qt::AlignRight);
		m_accel->replot(QCustomPlot::rpQueuedReplot);
	}
	if (m_gyro)
	{
		m_gyro->xAxis->setRange(m_currIdx, 500, Qt::AlignRight);
		m_gyro->replot(QCustomPlot::rpQueuedReplot);
	}
	if (m_angle)
	{
		m_angle->xAxis->setRange(m_currIdx, 500, Qt::AlignRight);
		m_angle->replot(QCustomPlot::rpQueuedReplot);
	}
	if (m_gloss)
	{
		m_gloss->xAxis->setRange(m_currIdx, 1000, Qt::AlignRight);
		m_gloss->replot(QCustomPlot::rpQueuedReplot);
	}
	if (m_attitude_accel)
	{
		m_attitude_accel->xAxis->setRange(m_currIdx, 1000, Qt::AlignRight);
		m_attitude_accel->replot(QCustomPlot::rpQueuedReplot);
	}
	if (m_attitude_gyro)
	{
		m_attitude_gyro->xAxis->setRange(m_currIdx, 1000, Qt::AlignRight);
		m_attitude_gyro->replot(QCustomPlot::rpQueuedReplot);
	}
	if (m_attitude_com_filter)
	{
		m_attitude_com_filter->xAxis->setRange(m_currIdx, 1000, Qt::AlignRight);
		m_attitude_com_filter->replot(QCustomPlot::rpQueuedReplot);
	}
	if (m_attitude_mixed)
	{
		m_attitude_mixed->xAxis->setRange(m_currIdx, 1000, Qt::AlignRight);
		m_attitude_mixed->replot(QCustomPlot::rpQueuedReplot);
	}
	if (m_attitude_ekf)
	{
		m_attitude_ekf->xAxis->setRange(m_currIdx, 1000, Qt::AlignRight);
		m_attitude_ekf->replot(QCustomPlot::rpQueuedReplot);
	}
}

void GroundStation::setButtonIcon(int)
{
	m_pOneSideBtn->setIcon(QIcon(myMovie->currentPixmap()));
}

QQuaternion Quaternion_Diff(Vector3f w, QQuaternion q)
{
	QMatrix4x4 A(0, -w(0)/2.0, -w(1)/2.0, -w(2)/2.0,
				w(0)/2.0, 0, w(2)/2.0, -w(1)/2.0,
				w(1)/2.0, -w(2)/2.0, 0, w(0)/2.0,
				w(2)/2.0, w(1)/2.0, -w(0)/2.0, 0);

	QVector4D q_(q.scalar(), q.x(), q.y(), q.z());
	QVector4D q_diff_vec = A * q_;
	QQuaternion q_diff(q_diff_vec[0], q_diff_vec[1], q_diff_vec[2], q_diff_vec[3]);
	return q_diff;
}

QQuaternion Quaternion_RungeKutta4(QQuaternion q0, Vector3f w)
{
	q0 = q0.normalized();
	QQuaternion K1 = Quaternion_Diff(w, q0);
	QQuaternion q1 = (q0 + T / 2.0 * K1).normalized();
	QQuaternion K2 = Quaternion_Diff(w, q1);
	QQuaternion q2 = (q0 + T / 2.0 * K2).normalized();
	QQuaternion K3 = Quaternion_Diff(w, q2);
	QQuaternion q3 = (q0 + T * K3);
	QQuaternion K4 = Quaternion_Diff(w, q3);
	QQuaternion q = q0 + T / 6.0 * (K1 + 2 * K2 + 2 * K3 + K4);
	q = q.normalized();
	return q;
}

Angular_Pos GroundStation::EstimateByAccel(float ax, float ay, float az)
{
	//1.角速度积分得到的姿态。
	float pitch_a = asin(BOUND_ABS_ONE(-ax)) * fRad2Deg;
	float roll_a = atan(ay / az) * fRad2Deg;
	return Angular_Pos(pitch_a, roll_a, 0);
}

Angular_Pos GroundStation::EstimateByGyro(float gx, float gy, float gz)
{
	float wx = gx * fDeg2Rad;
	float wy = gy * fDeg2Rad;
	float wz = gz * fDeg2Rad;

	Vector3f w(wx, wy, wz);
	float pitch_g = asin(BOUND_ABS_ONE(2 * (m_Qgyro.scalar() * m_Qgyro.y() - m_Qgyro.x() * m_Qgyro.z()))) * fRad2Deg;
	float roll_g = atan2(2 * m_Qgyro.y() * m_Qgyro.z() + 2 * m_Qgyro.scalar() * m_Qgyro.x(),
		-2 * m_Qgyro.x() * m_Qgyro.x() - 2 * m_Qgyro.y() * m_Qgyro.y() + 1) * fRad2Deg;
	m_Qgyro = Quaternion_RungeKutta4(m_Qgyro, w);
	return Angular_Pos(pitch_g, roll_g, 0);
}

QQuaternion GroundStation::MatrixDotQ(QMatrix4x4 M, QQuaternion q)
{
	QVector4D qv(q.scalar(), q.x(), q.y(), q.z());	//q直接转4元数，第一个数在最后
	QVector4D result = M * qv;
	return QQuaternion(result[0], result[1], result[2], result[3]);
}

Angular_Pos GroundStation::EstimateByComplementry(float ax, float ay, float az, float gx, float gy, float gz)
{
	const float factor = 0.9f;
	static float Ts = 0.01f;
	
	float wx = gx * fDeg2Rad;
	float wy = gy * fDeg2Rad;
	float wz = gz * fDeg2Rad;

	Angular_Pos pos_accel = EstimateByAccel(ax, ay, az);
	
	pitch_com = factor * (pitch_com + Ts * wy) + (1 - factor) * pos_accel.pitch;
	roll_com = factor * (roll_com + Ts * wx) + (1 - factor) * pos_accel.roll;
	return Angular_Pos(pitch_com, roll_com, 0);
}

void GroundStation::initMatrix()
{
	m_Q = 0.000001 * Matrix4f::Identity();
	m_R = 2 * Matrix3f::Identity();
	m_P << 0.125f, 0.0003f, 0.0003f, 0.0003f,
		0.0003f, 0.125f, 0.0003f, 0.0003f,
		0.0003f, 0.0003f, 0.125f, 0.0003f,
		0.0003f, 0.0003f, 0.0003f, 0.125f;
	m_P = 1 * m_P;
	m_V = Matrix3f::Identity();
}

void GroundStation::estimateByEKFOffline()
{
	//读取离线数据。
	QFile fn("X.txt");
	fn.open(QIODevice::ReadOnly);
	QByteArray buffer = fn.readAll();
	QByteArrayList list = buffer.split('\n');
	int n = list.size();
	for (int i = 0; i < list.size(); i++)
	{
		QByteArrayList pairs = list[i].split(',');
		float ax = pairs[0].toFloat(), ay = pairs[1].toFloat(), az = pairs[2].toFloat(),
			gx = pairs[3].toFloat(), gy = pairs[4].toFloat(), gz = pairs[5].toFloat();
		Angular_Pos pos_ekf = EstimateByEKF(ax, ay, az, gx, gy, gz);
	}
}

Vector4f normalize_quaternion(Vector4f v)
{
	float len = v.norm();
	return v / len;
}

Angular_Pos GroundStation::EstimateByEKF(float ax, float ay, float az, float gx, float gy, float gz)
{
	float wx = gx * fDeg2Rad, wy = gy * fDeg2Rad, wz = gz * fDeg2Rad;	//论文说以度数为单位。
	float q0 = m_qk(0), q1 = m_qk(1), q2 = m_qk(2), q3 = m_qk(3);

	//m_pStateBrowser->append(QString("q0:%1  q1:%2  q2:%3  q3:%4").arg(m_qekf.scalar()).arg(m_qekf.x()).arg(m_qekf.y()).arg(m_qekf.z()));
	Matrix4f Wx;
	Wx << 0., -wx, -wy, -wz,
		wx, 0, wz, -wy,
		wy, -wz, 0, wx,
		wz, wy, -wx, 0;

	Matrix4f Pk_1 = m_P;
	Matrix4f Qk = m_Q;

	Matrix4f I = Matrix4f::Identity();
	Matrix4f Ak = (I + 1./2 * T * Wx);
	Vector4f q_k_minus_1 = m_qk;
	Vector4f q_k_ = Ak * q_k_minus_1;
	q_k_ = normalize_quaternion(q_k_);
	Matrix4f Pk_ = Ak * Pk_1 * Ak.transpose() + Qk;		//先验的误差协方差矩阵。

	Matrix<float, 3, 4> Hk;
	Hk << -2*q2, 2*q3, -2*q0, 2*q1,
			2*q1, 2*q0, 2*q3, 2*q2,
			2*q0, -2*q1, -2*q2, 2*q3;

	Matrix3f det;	//倒数那部分
	det = Hk * Pk_ * Hk.transpose() + m_V * m_R * m_V.transpose();

	Matrix<float, 4, 3> Kk = Pk_ * Hk.transpose() * det.inverse();

	Vector3f zk(ax, ay, az);
	//k时刻观察项的估计。
	Vector3f h1_qk_(2*q1*q3-2*q0*q2, 2*q0*q1+2*q2*q3, q0*q0-q1*q1-q2*q2+q3*q3);

	//状态更新
	Vector4f obs_e = Kk * (zk - h1_qk_);
	obs_e(3) = 0;
	Vector4f qk = q_k_ + obs_e;
	m_qk = normalize_quaternion(qk);

	// 更新Pk
	m_P = (I - Kk * Hk) * Pk_;

	float pitch = asin(BOUND_ABS_ONE(2 * (m_qk(0) * m_qk(2) - m_qk(1) * m_qk(3)))) * fRad2Deg;
	float roll = atan2(2 * m_qk(2) * m_qk(3) + 2 * m_qk(0) * m_qk(1),
		-2 * m_qk(1) * m_qk(1) - 2 * m_qk(2) * m_qk(2) + 1) * fRad2Deg;

	return Angular_Pos(pitch, roll, 0);
}

void GroundStation::onCustomCOMRead()
{
	static bool bInit = false;
	QByteArray buffer = m_serialPort.read(1000);
	QString item = buffer.constData();
	item = item.trimmed();

	QByteArrayList list = buffer.split('\n');
	for (int i = 0; i < list.length(); i++)
	{
		QByteArrayList pairs = list[i].split(',');
		int n = pairs.length();
		if (!bInit)
		{
			if (n == 6)
			{
				// 要观察数据是否为MPU6050传感器的原始输出，还是除以量程得到的数据。
				float ax = pairs[0].toFloat(), ay = pairs[1].toFloat(), az = pairs[2].toFloat();
				m_accel->yAxis->setRange(0, 4, Qt::AlignCenter);
				mainType = UnitTransfer;
				bInit = true;
			}
			else
			{
				continue;
			}
		}

		if (mainType == Angle && (n == 3 || n ==2))
		{
			float roll = pairs[0].toFloat();
			float pitch = pairs[1].toFloat();
			m_accel->graph(0)->addData(m_currIdx, roll);
			m_accel->graph(1)->addData(m_currIdx, pitch);
			m_currIdx++;
		}
		else if (n == 6)
		{
			float x_accel = pairs[0].toFloat();
			float y_accel = pairs[1].toFloat();
			float z_accel = pairs[2].toFloat();
			float x_gyro = pairs[3].toFloat();
			float y_gyro = pairs[4].toFloat();
			float z_gyro = pairs[5].toFloat();

			Vector3f a_(x_accel, y_accel, z_accel);
			Vector3f a_calibrated = TKa * (a_ + ba);
			Vector3f g_(x_gyro, y_gyro, z_gyro);
			Vector3f g_calibrated = Tkg * (g_ + bg);

			m_pTextBrowser->append(QString("%1, %2, %3, %4, %5, %6\n").arg(a_calibrated(0)).arg(a_calibrated(1)).arg(a_calibrated(2)).arg(g_calibrated(0)).arg(g_calibrated(1)).arg(g_calibrated(2)));

			float ax = a_calibrated(0), ay = a_calibrated(1), az = a_calibrated(2);
			float gx = g_calibrated(0), gy = g_calibrated(1), gz = g_calibrated(2);
			Angular_Pos pos_a, pos_g, pos_com, pos_ekf;

			//1.角速度积分得到的姿态。
			pos_a = EstimateByAccel(ax, ay, az);

			//2.角速度积分得到的姿态。
			pos_g = EstimateByGyro(gx, gy, gz);

			//3.互补滤波得到的姿态。
			pos_com = EstimateByComplementry(ax, ay, az, gx, gy, gz);

			if (m_bEnableEKFFilter)
				pos_ekf = EstimateByEKF(ax, ay, az, gx, gy, gz);

			m_accel->graph(0)->addData(m_currIdx, ax);
			m_accel->graph(1)->addData(m_currIdx, ay);
			m_accel->graph(2)->addData(m_currIdx, az);

#ifdef PLOT_ONLY_MIXED
			m_attitude_mixed->graph(0)->addData(m_currIdx, pos_a.pitch);
			m_attitude_mixed->graph(1)->addData(m_currIdx, pos_a.roll);
			m_attitude_mixed->graph(2)->addData(m_currIdx, pos_com.pitch);
			m_attitude_mixed->graph(3)->addData(m_currIdx, pos_com.roll);
#else
			m_attitude_accel->graph(0)->addData(m_currIdx, pos_a.pitch);
			m_attitude_accel->graph(1)->addData(m_currIdx, pos_a.roll);
			m_attitude_gyro->graph(0)->addData(m_currIdx, pos_g.pitch);
			m_attitude_gyro->graph(1)->addData(m_currIdx, pos_g.roll);
			m_attitude_com_filter->graph(0)->addData(m_currIdx, pos_com.pitch);
			m_attitude_com_filter->graph(1)->addData(m_currIdx, pos_com.roll);
#endif
			if (m_bEnableEKFFilter)
			{
				m_attitude_ekf->graph(0)->addData(m_currIdx, pos_ekf.pitch);
				m_attitude_ekf->graph(1)->addData(m_currIdx, pos_ekf.roll);
			}

			if (m_bCollect4Calibrate)
			{
				if (m_idxX < CALIBRATION_SAMPLES)
				{
					X(m_idxX, 0) = x_accel;
					X(m_idxX, 1) = y_accel;
					X(m_idxX, 2) = z_accel;
					m_idxX++;
				}
				else
				{
					m_bCollect4Calibrate = false;
					m_idxX = 0;
					emit calibrationReady(X);
				}
			}
			if (m_bCollectingData)
			{
				m_vecCollectData.push_back(Original_Data(ax, ay, az, gx, gy, gz));
			}
			if (m_bGyroCalibrate)
			{
				if (m_idxGyro < CALIBRATION_SAMPLES)
				{
					bg_(0) += x_gyro;
					bg_(1) += y_gyro;
					bg_(2) += z_gyro;
					m_idxGyro++;
				}
				else
				{
					m_bGyroCalibrate = false;
					m_idxGyro = 0;
					bg = -bg_ / CALIBRATION_SAMPLES;
				}
			}
			m_currIdx++;
		}
	}
}

void GroundStation::accel_calibration()
{
	m_type = ONE_SIDE;
	m_side = SIDE_UP;
	m_pTip->setText(u8"陀螺仪向上：");
}

void GroundStation::six_sided_calibration()
{
	m_type = SIX_SIDE;
	m_side = SIDE_UP;
	m_pTip->setText(u8"陀螺仪向上：");
}

void GroundStation::onGyroCalibration()
{
	m_bGyroCalibrate = true;
}

void GroundStation::on_start_collect4calibration()
{
	m_bCollect4Calibrate = true;
}

void GroundStation::onCalibrationReady(const MatrixXf& X)
{
	if (m_type == ONE_SIDE) {
		m_pCalibration = new NewtonCalibration(X);
		m_pCalibration->solve();
		QMessageBox(QMessageBox::NoIcon, "", u8"较准完成").exec();
		m_pCalibration->get_optimized_result(TKa, ba);
	}
	else if (m_type == SIX_SIDE)
	{
		int k = m_side;
		if (m_bAverage4sixside) {
			float x_accel_sum = 0, y_accel_sum = 0, z_accel_sum = 0;
			for (int i = 0; i < CALIBRATION_SAMPLES; i++)
			{
				x_accel_sum += X(i, 0);
				y_accel_sum += X(i, 1);
				z_accel_sum += X(i, 2);
			}
			x_accel_sum /= CALIBRATION_SAMPLES;
			y_accel_sum /= CALIBRATION_SAMPLES;
			z_accel_sum /= CALIBRATION_SAMPLES;
			X_sixSide(k, 0) = x_accel_sum;
			X_sixSide(k, 1) = y_accel_sum;
			X_sixSide(k, 2) = z_accel_sum;
		}
		else {
			for (int i = 0; i < CALIBRATION_SAMPLES; i++)
			{
				X_sixSide(k * CALIBRATION_SAMPLES + i, 0) = X(i, 0);
				X_sixSide(k * CALIBRATION_SAMPLES + i, 1) = X(i, 1);
				X_sixSide(k * CALIBRATION_SAMPLES + i, 2) = X(i, 2);
			}
		}
		if (m_side < SIDE_BACK) {
			m_side = (CALIBRATION_SIDE)((int)m_side + 1);
			switch (m_side)
			{
			case SIDE_UP: m_pTip->setText(u8"陀螺仪向上："); break;
			case SIDE_DOWN: m_pTip->setText(u8"陀螺仪向下："); break;
			case SIDE_LEFT: m_pTip->setText(u8"陀螺仪向左："); break;
			case SIDE_RIGHT: m_pTip->setText(u8"陀螺仪向右："); break;
			case SIDE_FRONT: m_pTip->setText(u8"陀螺仪向前："); break;
			case SIDE_BACK: m_pTip->setText(u8"陀螺仪向后："); break;
			}
		}
		else {
			//收集完成，开始
			m_pCalibration = new NewtonCalibration(X_sixSide);
			m_pCalibration->solve();
			QMessageBox(QMessageBox::NoIcon, "", u8"较准完成").exec();
			m_pCalibration->get_optimized_result(TKa, ba);
		}
	}
}