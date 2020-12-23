#include "framework.h"
#include "MainWindow.h"
#include "moc_MainWindow.cpp"

const float dt = 0.005; //时间周期
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

//#define USE_MODULE

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
{
#ifdef PROCESS_COM
	initSimple();
	initSimplePlot();
#else
	init();
	initAccelPlot();
	initGyroPlot();
	initAnglePlot();
#endif
	initPort();
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
	m_accel->setMinimumHeight(800);
	QVBoxLayout* pMainLayout = new QVBoxLayout;

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
			#else
			connect(&m_serialPort, SIGNAL(readyRead()), this, SLOT(onReadyRead()));
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

void GroundStation::MyRealtimeDataSlot()
{
	if (m_accel)
	{
		m_accel->xAxis->setRange(m_currIdx, 3000, Qt::AlignRight);
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
	static const int nBuffer = 4;
	static float angle_last[3] = { 0 };
	float temp[3] = { 0 };
	QByteArray buffer = m_serialPort.read(1000);
	QString item = buffer.constData();
	item = item.trimmed();
	m_pTextBrowser->append(item);

	QByteArrayList list = buffer.split('\n');
	for (int i = 0; i < list.length(); i++)
	{
		QByteArrayList pairs = list[i].split(',');
		if (pairs.length() != 2)
			return;
		float roll = pairs[0].toFloat();
		float pitch = pairs[1].toFloat();
		m_accel->graph(0)->addData(m_currIdx, roll);
		m_accel->graph(1)->addData(m_currIdx, pitch);
		m_currIdx++;
	}
}

void GroundStation::onReadyRead()
{
	static const int nBuffer = 4;
	static float angle_last[3] = { 0 };
	float temp[3] = { 0 };
	QByteArray line = m_serialPort.readLine();
	QString item = line.constData();
	const auto&& parts = item.split(',');
	if (parts.length() != 6)
		return;
	
	qint16 x_accel = parts[0].toInt();
	qint16 y_accel = parts[1].toInt();
	qint16 z_accel = parts[2].toInt();
	qint16 x_gyro = parts[3].toInt();
	qint16 y_gyro = parts[4].toInt();
	qint16 z_gyro = parts[5].toInt();
	double x_a = x_accel - zeropadding_offset[X_ACCEL_ORI];
	double y_a = y_accel - zeropadding_offset[Y_ACCEL_ORI];
	double z_a = z_accel - zeropadding_offset[Z_ACCEL_ORI] + 8192.0f;
	double x_g = x_gyro - zeropadding_offset[X_GYRO_ORI];
	double y_g = y_gyro - zeropadding_offset[Y_GYRO_ORI];
	double z_g = z_gyro - zeropadding_offset[Z_GYRO_ORI];
	double X_Angle = 0, Y_Angle = 0, Z_Angle = 0;
	double x_a_ = x_a / 8192.0f;
	double y_a_ = y_a / 8192.0f;
	double z_a_ = z_a / 8192.0f;
	double x_g_ = x_g / 16.384f;
	double y_g_ = y_g / 16.384f;
	double z_g_ = z_g / 16.384f;

	switch (zeropad)
	{
	case NO_ZEROPAD:
	case ZEROPADDED:
	{
		if (m_currIdx % nBuffer == 0)
		{
			//简单计算倾角
			{
				temp[0] = sqrt(y_a_ * y_a_ + z_a_ * z_a_);
				temp[1] = sqrt(x_a_ * x_a_ + z_a_ * z_a_);

				static float R = 0.98f;

				bool bUseSimple = true;
				if (bUseSimple)
				{
					//X_Angle = acos(x_a_); X_Angle *= 57.29577;
					//Y_Angle = acos(y_a_); Y_Angle *= 57.29577;
					//Z_Angle = acos(z_a_); Z_Angle *= 57.29577;
					angle[0] = R * (angle_last[0] + x_g_ * dt) + (1 - R) * fRad2Deg * acos(MAX(MIN(x_a_, 1.), -1.));
					angle[1] = R * (angle_last[1] + y_g_ * dt) + (1 - R) * fRad2Deg * acos(MAX(MIN(y_a_, 1.), -1.));
					angle[2] = angle_last[2] + z_g_ * dt;
					angle_last[0] = angle[0];
					angle_last[1] = angle[1];
					angle_last[2] = angle[2];
				}
				else
				{
					//pitch and roll
					angle[0] = R * (angle_last[0] + x_g_ * dt) + (1 - R) * fRad2Deg * atan(x_a_ / temp[0]);
					angle_last[0] = angle[0];
					angle[1] = R * (angle_last[1] + y_g_ * dt) + (1 - R) * fRad2Deg * atan(y_a_ / temp[1]);
					angle_last[1] = angle[1];
					angle[2] = angle_last[2] + z_g_ * dt;
					angle_last[2] = angle[2];
				}
				m_angle->graph(0)->addData(m_currIdx, angle[0]);
				m_angle->graph(1)->addData(m_currIdx, angle[1]);
				m_angle->graph(2)->addData(m_currIdx, angle[2]);
			}

			if (cbXAccel->isChecked())
			{
				switch (m_plotAccel)
				{
				case ORIGINAL:	m_accel->graph(X_ACCEL_ORI)->addData(m_currIdx, x_accel); break;
				case ZeroPad:	m_accel->graph(X_ACCEL_ORI)->addData(m_currIdx, x_a); break;
				case UnitTransfer: m_accel->graph(X_ACCEL_ORI)->addData(m_currIdx, x_a_); break;
				}
			}
			if (cbYAccel->isChecked())
			{
				switch (m_plotAccel)
				{
				case ORIGINAL:	m_accel->graph(Y_ACCEL_ORI)->addData(m_currIdx, y_accel); break;
				case ZeroPad:	m_accel->graph(Y_ACCEL_ORI)->addData(m_currIdx, y_a); break;
				case UnitTransfer: m_accel->graph(Y_ACCEL_ORI)->addData(m_currIdx, y_a_); break;
				}
			}
			if (cbZAccel->isChecked())
			{
				switch (m_plotAccel)
				{
				case ORIGINAL:	m_accel->graph(Z_ACCEL_ORI)->addData(m_currIdx, z_accel); break;
				case ZeroPad:	m_accel->graph(Z_ACCEL_ORI)->addData(m_currIdx, z_a); break;
				case UnitTransfer: m_accel->graph(Z_ACCEL_ORI)->addData(m_currIdx, z_a_); break;
				}
			}
			//m_pTextBrowser->append(QString::number(y_g));
			if (cbXGyro->isChecked())
			{
				switch (m_plotGyro)
				{
				case ORIGINAL:		m_gyro->graph(X_GYRO_ORI - 3)->addData(m_currIdx, x_gyro); break;
				case ZeroPad:		m_gyro->graph(X_GYRO_ORI - 3)->addData(m_currIdx, x_g); break;
				case UnitTransfer:	m_gyro->graph(X_GYRO_ORI - 3)->addData(m_currIdx, x_g_); break;
				}
			}
			if (cbYGyro->isChecked())
			{
				switch (m_plotGyro)
				{
				case ORIGINAL:		m_gyro->graph(Y_GYRO_ORI - 3)->addData(m_currIdx, y_gyro); break;
				case ZeroPad:		m_gyro->graph(Y_GYRO_ORI - 3)->addData(m_currIdx, y_g); break;
				case UnitTransfer:	m_gyro->graph(Y_GYRO_ORI - 3)->addData(m_currIdx, y_g_); break;
				}
			}
			if (cbZGyro->isChecked())
			{
				switch (m_plotGyro)
				{
				case ORIGINAL:		m_gyro->graph(Z_GYRO_ORI - 3)->addData(m_currIdx, z_gyro); break;
				case ZeroPad:		m_gyro->graph(Z_GYRO_ORI - 3)->addData(m_currIdx, z_g); break;
				case UnitTransfer:	m_gyro->graph(Z_GYRO_ORI - 3)->addData(m_currIdx, z_g_); break;
				}
			}
		}
		break;
	}
	case ZEROPADDING:
		m_accelX.append(x_accel);
		m_accelY.append(y_accel);
		m_accelZ.append(z_accel);
		m_gyroX.append(x_gyro);
		m_gyroY.append(y_gyro);
		m_gyroZ.append(z_gyro);
		break;
	}
	m_currIdx++;
}