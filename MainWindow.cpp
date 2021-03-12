#include "framework.h"
#include "MainWindow.h"
#include "moc_MainWindow.cpp"

const float dt = 0.005f; //时间周期
float angle[3] = { 0 };
float Q[4] = { 1, 0, 0, 0 };
float pitch_com = 0, roll_com = 0;

const float fRad2Deg = 57.295779513f;	//弧度换算角度乘的系数
const float fDeg2Rad = 0.01745329252;	

#define MIN std::min
#define MAX std::max
#define BOUND_ABS_ONE(x) (MAX(-1.0f, MIN(1.0f, (x))))

#define PROCESS_COM

#define X_ACCEL_ORI 0
#define Y_ACCEL_ORI 1
#define Z_ACCEL_ORI 2
#define X_GYRO_ORI 3
#define Y_GYRO_ORI 4
#define Z_GYRO_ORI 5

#define CALIBRATION_SAMPLES 1000	//每一面较准所采样的数量。

//#define USE_MODULE
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
	pTabWidget->addTab(pCalibrationPage, u8"传感器标定");
	pTabWidget->addTab(pAttitudePage, u8"姿态解算");

	initCalibrationPlot();
	initAttitudePlot();
	initPort();
	initParameters();

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

QWidget* GroundStation::initAttitudePage()
{
	QWidget* pPage = new QWidget;

	//QVBoxLayout* pMainLayout = new QVBoxLayout;
	//QHBoxLayout* pHBoxLayout = new QHBoxLayout;
	QGridLayout* pGridLayout = new QGridLayout;

	m_attitude_accel = new QCustomPlot(pPage);
	m_attitude_gyro = new QCustomPlot(pPage);
	m_attitude_mixed = new QCustomPlot(pPage);

	pGridLayout->addWidget(m_attitude_accel, 0, 0);
	pGridLayout->addWidget(m_attitude_gyro, 0, 1);
	pGridLayout->addWidget(m_attitude_mixed, 1, 0);

	//pHBoxLayout->addWidget(m_attitude_accel);
	//pHBoxLayout->addWidget(m_attitude_gyro);
	//pMainLayout->addLayout(pHBoxLayout);
	//pMainLayout->addWidget(m_attitude_mixed);

	pPage->setLayout(pGridLayout);
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
			#ifdef PROCESS_COM
			connect(&m_serialPort, SIGNAL(readyRead()), this, SLOT(onCustomCOMRead()));
			connect(this, SIGNAL(calibrationReady(const MatrixXf&)), this,
				SLOT(onCalibrationReady(const MatrixXf&)));
			#else
			#endif
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

void GroundStation::paintEvent(QPaintEvent* event)
{
	QMainWindow::paintEvent(event);
}

void GroundStation::initCalibrationPlot()
{
	//m_accel->yAxis->setRange(0, 180, Qt::AlignCenter);
	m_accel->yAxis->setRange(-50, 180);
	m_accel->addGraph();
	m_accel->graph()->setPen(QPen(Qt::blue));
	//m_plot->graph()->setBrush(QBrush(QColor(0, 0, 255, 20)));

	m_accel->addGraph();
	m_accel->graph()->setPen(QPen(Qt::red));

	m_accel->addGraph();
	m_accel->graph()->setPen(QPen(Qt::green));
	//m_plot->graph()->setBrush(QBrush(QColor(0, 0, 255, 20)));
	// make left and bottom axes transfer their ranges to right and top axes:
	connect(m_accel->xAxis, SIGNAL(rangeChanged(QCPRange)), m_accel->xAxis2, SLOT(setRange(QCPRange)));
	connect(m_accel->yAxis, SIGNAL(rangeChanged(QCPRange)), m_accel->yAxis2, SLOT(setRange(QCPRange)));
	connect(&dtaccel, SIGNAL(timeout()), this, SLOT(MyRealtimeDataSlot()));
	dtaccel.start(0);
}

void GroundStation::initAttitudePlot()
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

	m_attitude_mixed->yAxis->setRange(-180, 180);
	{
		m_attitude_mixed->addGraph();
		m_attitude_mixed->graph()->setPen(QPen(Qt::blue));
		m_attitude_mixed->addGraph();
		m_attitude_mixed->graph()->setPen(QPen(Qt::red));

		connect(m_attitude_mixed->xAxis, SIGNAL(rangeChanged(QCPRange)), m_attitude_mixed->xAxis2, SLOT(setRange(QCPRange)));
		connect(m_attitude_mixed->yAxis, SIGNAL(rangeChanged(QCPRange)), m_attitude_mixed->yAxis2, SLOT(setRange(QCPRange)));
		connect(&dtAttitude_mix, SIGNAL(timeout()), this, SLOT(MyRealtimeDataSlot()));
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
	if (m_attitude_mixed)
	{
		m_attitude_mixed->xAxis->setRange(m_currIdx, 1000, Qt::AlignRight);
		m_attitude_mixed->replot(QCustomPlot::rpQueuedReplot);
	}
}

void GroundStation::setButtonIcon(int)
{
	m_pOneSideBtn->setIcon(QIcon(myMovie->currentPixmap()));
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

			m_accel->graph(0)->addData(m_currIdx, a_calibrated(0));
			m_accel->graph(1)->addData(m_currIdx, a_calibrated(1));
			m_accel->graph(2)->addData(m_currIdx, a_calibrated(2));

			//1.角速度积分得到的姿态。
			float pitch_a = asin(BOUND_ABS_ONE(a_calibrated(0))) * fRad2Deg;
			float roll_a = atan(a_calibrated(1) / a_calibrated(2)) * fRad2Deg;
			m_attitude_accel->graph(0)->addData(m_currIdx, pitch_a);
			m_attitude_accel->graph(1)->addData(m_currIdx, roll_a);

			//2.角速度积分得到的姿态。
			float wx = g_calibrated(0) * fDeg2Rad;
			float wy = g_calibrated(1) * fDeg2Rad;
			float wz = g_calibrated(2) * fDeg2Rad;
			static float Ts = 0.01f;

			Q[0] = Q[0] + (-wx * Q[1] - wy * Q[2] - wz * Q[3]) * Ts;
			Q[1] = Q[1] + (wx * Q[0] + wz * Q[2] - wy * Q[3]) * Ts;
			Q[2] = Q[2] + (wy * Q[0] - wz * Q[1] + wx * Q[3]) * Ts;
			Q[3] = Q[3] + (wz * Q[0] + wy * Q[1] - wx * Q[2]) * Ts;

			//yaw = atan2(2 * Q[1] * Q[2] + 2 * Q[0] * Q[3], -2 * Q[2] * Q[2] - 2 * Q[3] * Q[3] + 1) * 57.3;
			float pitch_g = asin(BOUND_ABS_ONE(-2 * Q[1] * Q[3] + 2 * Q[0] * Q[2])) * 57.3;
			float roll_g = atan2(2 * Q[2] * Q[3] + 2 * Q[0] * Q[1], -2 * Q[1] * Q[1] - 2 * Q[2] * Q[2] + 1) * 57.3;
			m_attitude_gyro->graph(0)->addData(m_currIdx, pitch_g);
			m_attitude_gyro->graph(1)->addData(m_currIdx, roll_g);

			//3.互补滤波得到的姿态。
			const float factor = 0.9f;
			pitch_com = factor * (pitch_com + Ts * wy) + (1 - factor) * pitch_a;
			roll_com = factor * (roll_com + Ts * wx) + (1 - factor) * roll_a;
			m_attitude_mixed->graph(0)->addData(m_currIdx, pitch_com);
			m_attitude_mixed->graph(1)->addData(m_currIdx, roll_com);

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