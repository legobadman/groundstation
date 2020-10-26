#include "framework.h"
#include "MainWindow.h"
#include "moc_MainWindow.cpp"

GroundStation::GroundStation(QWidget* parent)
	: QMainWindow(parent)
	, m_currIdx(0)
{
	init();
	initPort();
	initPlot();
	//setup();
}

GroundStation::~GroundStation()
{
}

void GroundStation::init()
{
	QSpacerItem* pHorizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
	
	m_plot = new QCustomPlot(this);
	QHBoxLayout* pHLayout = new QHBoxLayout;
	pHLayout->addWidget(m_plot);
	//pHLayout->addSpacerItem(pHorizontalSpacer);

	//m_pTextBrowser = new QTextBrowser(this);
	//m_pTextBrowser->setText("fewfewfwe");
	//pHLayout->addWidget(m_pTextBrowser);
	
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

void GroundStation::paintEvent(QPaintEvent* event)
{
	if (false)
	{
		m_serialPort.isOpen();
		m_serialPort.openMode();
	}
	QMainWindow::paintEvent(event);
}

void GroundStation::setup()
{
	m_plot->addGraph(); // blue line
	m_plot->graph(0)->setPen(QPen(QColor(40, 110, 255)));
	m_plot->addGraph(); // red line
	m_plot->graph(1)->setPen(QPen(QColor(255, 110, 40)));

	QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
	timeTicker->setTimeFormat("%h:%m:%s");
	m_plot->xAxis->setTicker(timeTicker);
	m_plot->axisRect()->setupFullAxesBox();
	m_plot->yAxis->setRange(-1.2, 1.2);

	// make left and bottom axes transfer their ranges to right and top axes:
	connect(m_plot->xAxis, SIGNAL(rangeChanged(QCPRange)), m_plot->xAxis2, SLOT(setRange(QCPRange)));
	connect(m_plot->yAxis, SIGNAL(rangeChanged(QCPRange)), m_plot->yAxis2, SLOT(setRange(QCPRange)));

	// setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
	connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
	dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

void GroundStation::initPlot()
{
	m_plot->xAxis->setRange(0, 500, Qt::AlignLeft);
	m_plot->yAxis->setRange(0, 65535, Qt::AlignCenter);
	m_plot->addGraph();
	m_plot->graph()->setPen(QPen(Qt::blue));
	m_plot->graph()->setBrush(QBrush(QColor(0, 0, 255, 20)));
	// make left and bottom axes transfer their ranges to right and top axes:
	connect(m_plot->xAxis, SIGNAL(rangeChanged(QCPRange)), m_plot->xAxis2, SLOT(setRange(QCPRange)));
	connect(m_plot->yAxis, SIGNAL(rangeChanged(QCPRange)), m_plot->yAxis2, SLOT(setRange(QCPRange)));
	connect(&dataTimer, SIGNAL(timeout()), this, SLOT(MyRealtimeDataSlot()));
	dataTimer.start(0);
}

void GroundStation::MyRealtimeDataSlot()
{
	m_plot->replot();
}

//#define MULTILINE

#ifdef MULTILINE
void GroundStation::onReadyRead()
{
	static QTime startTime(QTime::currentTime());
	double key = startTime.secsTo(QTime::currentTime());
	static double lastPointKey = 0;
	
	QByteArray arr = m_serialPort.readAll();
	if (key - lastPointKey > 0.002)
	{
		QList<QByteArray> L = arr.split('\n');
		for (int i = 1; i < L.length() - 1; i++)
		{
			QString item = L[i].constData();
			QRegExp rx("[^\\d]+");
			const auto&& parts = item.split(rx, Qt::SkipEmptyParts);
			m_plot->graph(0)->addData(key + i, parts[0].toInt());
		}
		lastPointKey = key;
		m_plot->xAxis->setRange(key, L.length() - 2, Qt::AlignRight);
	}

	static double lastFpsKey = 0;
	if (key - lastFpsKey > 2)
	{
		m_plot->replot();
		lastFpsKey = key;
	}
}
#else
void GroundStation::onReadyRead()
{
	static QTime startTime(QTime::currentTime());
	double key = startTime.secsTo(QTime::currentTime());
	static double lastPointKey = 0;

	QByteArray line = m_serialPort.readLine();
	QString item = line.constData();
	if (item.startsWith("AC", Qt::CaseInsensitive))
	{
		QRegExp rx("[^\\d]+");
		const auto&& parts = item.split(rx, Qt::SkipEmptyParts);
		int x = parts[0].toInt();
		m_accelX.append(x);
		//m_accelY.append(parts[1].toInt());
		//m_accelZ.append(parts[2].toInt());		
		m_plot->graph(0)->addData(m_currIdx++, x);
	}
	m_plot->xAxis->setRange(m_currIdx, 100, Qt::AlignRight);
}
#endif