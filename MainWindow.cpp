#include "framework.h"
#include "MainWindow.h"
#include "moc_MainWindow.cpp"

#define X_ACCEL_ORI 0
#define Y_ACCEL_ORI 1
#define Z_ACCEL_ORI 2
#define X_GYRO_ORI 0
#define Y_GYRO_ORI 1
#define Z_GYRO_ORI 2


GroundStation::GroundStation(QWidget* parent)
	: QMainWindow(parent)
	, m_currIdx(0)
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
	m_gyro = new QCustomPlot(this);
	QHBoxLayout* pHLayout = new QHBoxLayout;

	//m_pTextBrowser = new QTextBrowser(this);
	//m_pTextBrowser->setText("");
	//pHLayout->addWidget(m_pTextBrowser);
	
	//QSpacerItem* pHorizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
	//pHLayout->addSpacerItem(pHorizontalSpacer);

	QVBoxLayout* pVLayout = new QVBoxLayout;
	pVLayout->addWidget(m_accel);
	pVLayout->addWidget(m_gyro);
	pHLayout->addLayout(pVLayout);

	QWidget* pCentralWidget = new QWidget(this);
	pCentralWidget->setLayout(pHLayout);
	this->setCentralWidget(pCentralWidget);
	this->resize(1304, 1191);
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
	m_gyro->yAxis->setRange(0, 1000, Qt::AlignCenter);
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

void GroundStation::onReadyRead()
{
	static const int nBuffer = 1;
	static int temp = 0;
	QByteArray line = m_serialPort.readLine();
	QString item = line.constData();
	const auto&& parts = item.split(',');
	if (parts.length() != 6)
		return;
	
	int x_accel = parts[0].toDouble();
	int y_accel = parts[1].toDouble();
	int z_accel = parts[2].toDouble();
	int x_gyro = parts[3].toDouble();
	int y_gyro = parts[4].toDouble();
	int z_gyro = parts[5].toDouble();

	if (m_currIdx % nBuffer == 0)
	{
		m_accel->graph(X_ACCEL_ORI)->addData(m_currIdx, x_accel);
		m_accel->graph(Y_ACCEL_ORI)->addData(m_currIdx, y_accel);
		m_accel->graph(Z_ACCEL_ORI)->addData(m_currIdx, z_accel);

		m_gyro->graph(X_GYRO_ORI)->addData(m_currIdx, x_gyro);
		m_gyro->graph(Y_GYRO_ORI)->addData(m_currIdx, y_gyro);
		m_gyro->graph(Z_GYRO_ORI)->addData(m_currIdx, z_gyro);
		temp = 0;
	}
	m_currIdx++;
}