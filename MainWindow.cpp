#include "framework.h"
#include "MainWindow.h"
#include "moc_MainWindow.cpp"

#define X_ACCEL_ORI 0
#define Y_ACCEL_ORI 1
#define Z_ACCEL_ORI 2
#define X_GYRO_ORI 3
#define Y_GYRO_ORI 4
#define Z_GYRO_ORI 5

static qint16 zeropadding_offset[6] = { 0 };


GroundStation::GroundStation(QWidget* parent)
	: QMainWindow(parent)
	, m_currIdx(0)
	, zeropad(NO_ZEROPAD)
{
	init();
	initPort();
	initAccelPlot();
	initGyroPlot();
}

GroundStation::~GroundStation()
{
}

void GroundStation::init()
{
	m_accel = new QCustomPlot(this);
	m_accel->setMinimumWidth(800);
	//m_accel->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Preferred);
	m_gyro = new QCustomPlot(this);
	QHBoxLayout* pMainLayout = new QHBoxLayout;

	QVBoxLayout* pLeftLayout = new QVBoxLayout;
	QPushButton* pZeroPadding = new QPushButton(u8"零偏校准");
	pLeftLayout->addWidget(pZeroPadding);
	connect(pZeroPadding, SIGNAL(clicked(bool)), this, SLOT(prepare_zeropadding(bool)));
	pMainLayout->addLayout(pLeftLayout);

	m_pTextBrowser = new QTextBrowser(this);
	m_pTextBrowser->setText("");
	pMainLayout->addWidget(m_pTextBrowser);
	
	//QSpacerItem* pHorizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
	//pHLayout->addSpacerItem(pHorizontalSpacer);

	QVBoxLayout* pVLayout = new QVBoxLayout;
	pVLayout->addWidget(m_accel);
	{
		QHBoxLayout* pAccelHLayout = new QHBoxLayout;
		cbXAccel = new QCheckBox(u8"X轴");
		cbYAccel = new QCheckBox(u8"Y轴");
		cbZAccel = new QCheckBox(u8"Z轴");
		cbZeroPadAccel = new QCheckBox(u8"零点漂移");
		cbUnitAccel = new QCheckBox(u8"单位换算(g)");
		cbXAccel->setChecked(true);
		cbYAccel->setChecked(true);
		cbZAccel->setChecked(true);

		pAccelHLayout->addWidget(cbXAccel);
		pAccelHLayout->addWidget(cbYAccel);
		pAccelHLayout->addWidget(cbZAccel);
		pAccelHLayout->addWidget(cbZeroPadAccel);
		pAccelHLayout->addWidget(cbUnitAccel);
		pVLayout->addLayout(pAccelHLayout);
	}

	pVLayout->addWidget(m_gyro);
	{
		QHBoxLayout* pGyroHLayout = new QHBoxLayout;
		cbXGyro = new QCheckBox(u8"X轴");
		cbYGyro = new QCheckBox(u8"Y轴");
		cbZGyro = new QCheckBox(u8"Z轴");
		cbZeroPadGyro = new QCheckBox(u8"零点漂移");
		cbUnitGyro = new QCheckBox(u8"单位换算(g)");
		cbXGyro->setChecked(true);
		cbYGyro->setChecked(true);
		cbZGyro->setChecked(true);

		pGyroHLayout->addWidget(cbXGyro);
		pGyroHLayout->addWidget(cbYGyro);
		pGyroHLayout->addWidget(cbZGyro);
		pGyroHLayout->addWidget(cbZeroPadGyro);
		pGyroHLayout->addWidget(cbUnitGyro);
		pVLayout->addLayout(pGyroHLayout);
	}

	pMainLayout->addLayout(pVLayout);

	QWidget* pCentralWidget = new QWidget(this);
	pCentralWidget->setLayout(pMainLayout);
	this->setCentralWidget(pCentralWidget);
	this->resize(1304, 691);
}

void GroundStation::initPort()
{
	foreach(const QSerialPortInfo & info, QSerialPortInfo::availablePorts())
	{
		if (info.manufacturer() == "Silicon Labs")
		{
			m_serialPort.setPort(info);
			m_serialPort.setParity(QSerialPort::EvenParity);
			m_serialPort.setBaudRate(QSerialPort::Baud115200);
			m_serialPort.clearError();
			m_serialPort.clear();
			connect(&m_serialPort, SIGNAL(readyRead()), this, SLOT(onReadyRead()));
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

void GroundStation::MyRealtimeDataSlot()
{
	m_accel->xAxis->setRange(m_currIdx, 500, Qt::AlignRight);
	m_accel->replot(QCustomPlot::rpQueuedReplot);

	m_gyro->xAxis->setRange(m_currIdx, 500, Qt::AlignRight);
	m_gyro->replot(QCustomPlot::rpQueuedReplot);
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

void GroundStation::onReadyRead()
{
	static const int nBuffer = 4;
	static int temp = 0;
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

	switch (zeropad)
	{
	case NO_ZEROPAD:
	case ZEROPADDED:
	{
		if (m_currIdx % nBuffer == 0)
		{
			if (cbXAccel->isChecked())
				m_accel->graph(X_ACCEL_ORI)->addData(m_currIdx, x_accel - zeropadding_offset[X_ACCEL_ORI]);
			if (cbYAccel->isChecked())
				m_accel->graph(Y_ACCEL_ORI)->addData(m_currIdx, y_accel - zeropadding_offset[Y_ACCEL_ORI]);
			if (cbZAccel->isChecked())
				m_accel->graph(Z_ACCEL_ORI)->addData(m_currIdx, z_accel - zeropadding_offset[Z_ACCEL_ORI] + 8192);
			//m_pTextBrowser->append("Hello, World");
			if (cbXGyro->isChecked())
				m_gyro->graph(X_GYRO_ORI - 3)->addData(m_currIdx, x_gyro - zeropadding_offset[X_GYRO_ORI]);
			if (cbYGyro->isChecked())
				m_gyro->graph(Y_GYRO_ORI - 3)->addData(m_currIdx, y_gyro - zeropadding_offset[Y_GYRO_ORI]);
			if (cbZGyro->isChecked())
				m_gyro->graph(Z_GYRO_ORI - 3)->addData(m_currIdx, z_gyro - zeropadding_offset[Z_GYRO_ORI]);
			temp = 0;
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