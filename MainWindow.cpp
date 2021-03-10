#include "framework.h"
#include "MainWindow.h"
#include "moc_MainWindow.cpp"

const float dt = 0.005f; //时间周期
float angle[3] = { 0 };

const float fRad2Deg = 57.295779513f; //弧度换算角度乘的系数

#define MIN std::min
#define MAX std::max

#define PROCESS_COM

#define X_ACCEL_ORI 0
#define Y_ACCEL_ORI 1
#define Z_ACCEL_ORI 2
#define X_GYRO_ORI 3
#define Y_GYRO_ORI 4
#define Z_GYRO_ORI 5

#define CALIBRATION_SAMPLES 300	//每一面较准所采样的数量。

//#define USE_MODULE
float GroundStation::g = 9.7883f;

#ifndef USE_MODULE
static qint16 zeropadding_offset[6] = {
	466,
	-32,
	5216,
	58,
	-99,
	4
};
#else
static qint16 zeropadding_offset[6] = {
	-4746,
	80,
	7998,
	-19,
	23,
	-3
};
#endif



GroundStation::GroundStation(QWidget* parent)
	: QMainWindow(parent)
	, m_currIdx(0)
	, zeropad(NO_ZEROPAD)
	, m_plotAccel(ORIGINAL)
	, m_plotGyro(ORIGINAL)
	, mainType(UNKNOWN)
	, pCalibration(NULL)
	, m_pSixSideBtn(NULL)
	, m_bCollect4Calibrate(false)
	, m_idxX(0)
	, m_side(SIDE_UP)
	, m_type(ONE_SIDE)
	, m_pTip(NULL)
	, m_bAverage4sixside(false)
{
#ifdef PROCESS_COM
	initSimple();
	initSimplePlot();
	X = MatrixXf(CALIBRATION_SAMPLES, 3);
	m_bAverage4sixside = false;
	if (m_bAverage4sixside)
		X_sixSide = MatrixXf(6, 3);
	else
		X_sixSide = MatrixXf(6 * CALIBRATION_SAMPLES, 3);
#else
	init();
	initAccelPlot();
	initGyroPlot();
	initAnglePlot();
#endif
	initPort();
	initParameters();
}

GroundStation::~GroundStation()
{
}

void GroundStation::init()
{
	m_accel = new QCustomPlot(this);
	m_accel->setMinimumWidth(400);
	m_gyro = new QCustomPlot(this);
	m_angle = new QCustomPlot(this);
	QHBoxLayout* pMainLayout = new QHBoxLayout;

	QVBoxLayout* pLeftLayout = new QVBoxLayout;
	QPushButton* pZeroPadding = new QPushButton(u8"零偏校准");
	pLeftLayout->addWidget(pZeroPadding);
	connect(pZeroPadding, SIGNAL(clicked(bool)), this, SLOT(prepare_zeropadding(bool)));
	pMainLayout->addLayout(pLeftLayout);

	m_pTextBrowser = new QTextBrowser(this);
	m_pTextBrowser->setText("");
	pMainLayout->addWidget(m_pTextBrowser);
	
	QVBoxLayout* pVLayout = new QVBoxLayout;
	pVLayout->addWidget(m_accel);
	{
		QHBoxLayout* pAccelHLayout = new QHBoxLayout;
		cbXAccel = new QCheckBox(u8"X轴");
		cbYAccel = new QCheckBox(u8"Y轴");
		cbZAccel = new QCheckBox(u8"Z轴");
		cbXAccel->setChecked(true);
		cbYAccel->setChecked(true);
		cbZAccel->setChecked(true);

		QButtonGroup* pButtonGroup = new QButtonGroup(this);
		pButtonGroup->setExclusive(true);
		QRadioButton* pOriButton = new QRadioButton(u8"原始数据", this);
		QRadioButton* pUnitButton = new QRadioButton(u8"单位换算", this);
		QRadioButton* pZeroButton = new QRadioButton(u8"零点漂移", this);
		pOriButton->setChecked(true);
		pButtonGroup->addButton(pOriButton);
		pButtonGroup->addButton(pUnitButton);
		pButtonGroup->addButton(pZeroButton);
		connect(pButtonGroup, SIGNAL(buttonClicked(QAbstractButton*)), this, SLOT(onPlotAccelChanged(QAbstractButton*)));

		pAccelHLayout->addWidget(cbXAccel);
		pAccelHLayout->addWidget(cbYAccel);
		pAccelHLayout->addWidget(cbZAccel);
		pAccelHLayout->addWidget(pOriButton);
		pAccelHLayout->addWidget(pZeroButton);
		pAccelHLayout->addWidget(pUnitButton);
		pVLayout->addLayout(pAccelHLayout);
	}

	pVLayout->addWidget(m_gyro);
	{
		QHBoxLayout* pGyroHLayout = new QHBoxLayout;
		cbXGyro = new QCheckBox(u8"X轴");
		cbYGyro = new QCheckBox(u8"Y轴");
		cbZGyro = new QCheckBox(u8"Z轴");
		cbXGyro->setChecked(true);
		cbYGyro->setChecked(true);
		cbZGyro->setChecked(true);

		QButtonGroup* pButtonGroup = new QButtonGroup(this);
		pButtonGroup->setExclusive(true);
		QRadioButton* pOriButton = new QRadioButton(u8"原始数据", this);
		QRadioButton* pZeroButton = new QRadioButton(u8"零点漂移", this);
		QRadioButton* pUnitButton = new QRadioButton(u8"单位换算", this);
		
		pOriButton->setChecked(true);
		pButtonGroup->addButton(pOriButton);
		pButtonGroup->addButton(pUnitButton);
		pButtonGroup->addButton(pZeroButton);
		connect(pButtonGroup, SIGNAL(buttonClicked(QAbstractButton*)), this, SLOT(onPlotGyroChanged(QAbstractButton*)));

		pGyroHLayout->addWidget(cbXGyro);
		pGyroHLayout->addWidget(cbYGyro);
		pGyroHLayout->addWidget(cbZGyro);
		pGyroHLayout->addWidget(pOriButton);
		pGyroHLayout->addWidget(pZeroButton);
		pGyroHLayout->addWidget(pUnitButton);
		pVLayout->addLayout(pGyroHLayout);
	}
	pVLayout->addWidget(m_angle);

	pMainLayout->addLayout(pVLayout);

	QWidget* pCentralWidget = new QWidget(this);
	pCentralWidget->setLayout(pMainLayout);
	this->setCentralWidget(pCentralWidget);
	this->resize(1304, 691);
}

void GroundStation::initSimple()
{
	m_accel = new QCustomPlot(this);
	m_accel->setMinimumHeight(600);
	QVBoxLayout* pMainLayout = new QVBoxLayout;

	QHBoxLayout* pHBoxLayout = new QHBoxLayout;

	m_pOneSideBtn = new QPushButton(this);
	m_pOneSideBtn->setText(u8"加速度计较准");
	pHBoxLayout->addWidget(m_pOneSideBtn);

	m_pSixSideBtn = new QPushButton(this);
	m_pSixSideBtn->setText(u8"六面较准");
	pHBoxLayout->addWidget(m_pSixSideBtn);

	myMovie = new QMovie("circle.gif", QByteArray(), m_pOneSideBtn);
	myMovie->setScaledSize(QSize(20, 20));
	connect(myMovie, SIGNAL(frameChanged(int)), this, SLOT(setButtonIcon(int)));
	// if movie doesn't loop forever, force it to.
	if (myMovie->loopCount() != -1)
		connect(myMovie, SIGNAL(finished()), myMovie, SLOT(start()));
	if (myMovie->isValid())
		myMovie->start();

	connect(m_pOneSideBtn, SIGNAL(clicked()), this, SLOT(accel_calibration()));
	connect(m_pSixSideBtn, SIGNAL(clicked()), this, SLOT(six_sided_calibration()));

	pHBoxLayout->addSpacerItem(new QSpacerItem(100, 20, QSizePolicy::Preferred, QSizePolicy::Expanding));

	m_pTip = new QLabel(this);
	pHBoxLayout->addWidget(m_pTip);
	m_pTip->setText(u8"陀螺仪向上：");

	QPushButton* pStartBtn = new QPushButton(this);
	pStartBtn->setText(u8"开始收集");
	connect(pStartBtn, SIGNAL(clicked()), this, SLOT(on_start_collect4calibration()));
	pHBoxLayout->addWidget(pStartBtn);

	pMainLayout->addItem(pHBoxLayout);

	m_pTextBrowser = new QTextBrowser(this);
	m_pTextBrowser->setText("");

	pMainLayout->addWidget(m_pTextBrowser);
	pMainLayout->addWidget(m_accel);

	QWidget* pCentralWidget = new QWidget(this);
	pCentralWidget->setLayout(pMainLayout);
	this->setCentralWidget(pCentralWidget);
	this->resize(1304, 691);
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

void GroundStation::initSimplePlot()
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

void GroundStation::initAccelPlot()
{
	m_accel->yAxis->setRange(0, 65535, Qt::AlignCenter);
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

void GroundStation::initGyroPlot()
{
	m_gyro->yAxis->setRange(0, 2000, Qt::AlignCenter);
	m_gyro->addGraph();
	m_gyro->graph()->setPen(QPen(Qt::blue));
	//m_plot->graph()->setBrush(QBrush(QColor(0, 0, 255, 20)));

	m_gyro->addGraph();
	m_gyro->graph()->setPen(QPen(Qt::red));

	m_gyro->addGraph();
	m_gyro->graph()->setPen(QPen(Qt::green));
	//m_plot->graph()->setBrush(QBrush(QColor(0, 0, 255, 20)));
	// make left and bottom axes transfer their ranges to right and top axes:
	connect(m_gyro->xAxis, SIGNAL(rangeChanged(QCPRange)), m_gyro->xAxis2, SLOT(setRange(QCPRange)));
	connect(m_gyro->yAxis, SIGNAL(rangeChanged(QCPRange)), m_gyro->yAxis2, SLOT(setRange(QCPRange)));
	connect(&dtgyro, SIGNAL(timeout()), this, SLOT(MyRealtimeDataSlot()));
	dtgyro.start(0);
}

void GroundStation::initAnglePlot()
{
	m_angle->yAxis->setRange(0, 360, Qt::AlignCenter);
	m_angle->addGraph();
	m_angle->graph()->setPen(QPen(Qt::blue));
	//m_plot->graph()->setBrush(QBrush(QColor(0, 0, 255, 20)));

	m_angle->addGraph();
	m_angle->graph()->setPen(QPen(Qt::red));

	m_angle->addGraph();
	m_angle->graph()->setPen(QPen(Qt::green));
	//m_plot->graph()->setBrush(QBrush(QColor(0, 0, 255, 20)));
	// make left and bottom axes transfer their ranges to right and top axes:
	connect(m_angle->xAxis, SIGNAL(rangeChanged(QCPRange)), m_angle->xAxis2, SLOT(setRange(QCPRange)));
	connect(m_angle->yAxis, SIGNAL(rangeChanged(QCPRange)), m_angle->yAxis2, SLOT(setRange(QCPRange)));
	connect(&dtangle, SIGNAL(timeout()), this, SLOT(MyRealtimeDataSlot()));
	dtangle.start(0);
}

void GroundStation::initParameters()
{
	TK << 1, 0, 0,
		0, 1, 0,
		0, 0, 1;
	b << 0, 0, 0;
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
}

void GroundStation::setButtonIcon(int)
{
	m_pOneSideBtn->setIcon(QIcon(myMovie->currentPixmap()));
}

void GroundStation::prepare_zeropadding(bool)
{
	QMessageBox msgBox(QMessageBox::Information, "", u8"请将飞控板置于参考平面", QMessageBox::Ok | QMessageBox::Cancel);
	int ret = msgBox.exec();
	if (ret == QMessageBox::Ok) {
		zeropad = ZEROPADDING;
		connect(&zeropad_timer, SIGNAL(timeout()), this, SLOT(calculated_zeropad()));
		zeropad_timer.start(30000);
	}
	else {
		zeropad = NO_ZEROPAD;
	}
}

void GroundStation::calculated_zeropad()
{
	int sum_accelX = 0, sum_accelY = 0, sum_accelZ = 0, sum_gyroX = 0, sum_gyroY = 0, sum_gyroZ = 0;
	int n = m_accelX.size();
	for (int i = 0; i < n; i++)
	{
		sum_accelX += m_accelX[i];
		sum_accelY += m_accelY[i];
		sum_accelZ += m_accelZ[i];
		sum_gyroX += m_gyroX[i];
		sum_gyroY += m_gyroY[i];
		sum_gyroZ += m_gyroZ[i];
	}
	zeropadding_offset[X_ACCEL_ORI] = sum_accelX / n;
	zeropadding_offset[Y_ACCEL_ORI] = sum_accelY / n;
	zeropadding_offset[Z_ACCEL_ORI] = sum_accelZ / n;
	zeropadding_offset[X_GYRO_ORI] = sum_gyroX / n;
	zeropadding_offset[Y_GYRO_ORI] = sum_gyroY / n;
	zeropadding_offset[Z_GYRO_ORI] = sum_gyroZ / n;
	zeropad = ZEROPADDED;
	QMessageBox msgBox(QMessageBox::Information, "", u8"校准完成。", QMessageBox::Ok);
	msgBox.exec();
	zeropad_timer.stop();
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
			Vector3f a_calibrated = TK * (a_ + b);

			m_pTextBrowser->append(QString("%1, %2, %3\n").arg(a_calibrated(0)).arg(a_calibrated(1)).arg(a_calibrated(2)));

			m_accel->graph(0)->addData(m_currIdx, a_calibrated(0));
			m_accel->graph(1)->addData(m_currIdx, a_calibrated(1));
			m_accel->graph(2)->addData(m_currIdx, a_calibrated(2));

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

void GroundStation::on_start_collect4calibration()
{
	m_bCollect4Calibrate = true;
}

void GroundStation::onCalibrationReady(const MatrixXf& X)
{
	if (m_type == ONE_SIDE) {
		pCalibration = new NewtonCalibration(X);
		pCalibration->solve();
		QMessageBox(QMessageBox::NoIcon, "", u8"较准完成").exec();
		pCalibration->get_optimized_result(TK, b);
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
			pCalibration = new NewtonCalibration(X_sixSide);
			pCalibration->solve();
			QMessageBox(QMessageBox::NoIcon, "", u8"较准完成").exec();
			pCalibration->get_optimized_result(TK, b);
		}
	}
}