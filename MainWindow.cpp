#include "framework.h"
#include "MainWindow.h"
#include "moc_MainWindow.cpp"

GroundStation::GroundStation(QWidget* parent)
	: QMainWindow(parent)
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
	m_plot->xAxis->setRange(0, 50, Qt::AlignLeft);
	m_plot->yAxis->setRange(0, 65535, Qt::AlignCenter);
	m_plot->addGraph();
	m_plot->graph()->setPen(QPen(Qt::blue));
	m_plot->graph()->setBrush(QBrush(QColor(0, 0, 255, 20)));
	// make left and bottom axes transfer their ranges to right and top axes:
	connect(m_plot->xAxis, SIGNAL(rangeChanged(QCPRange)), m_plot->xAxis2, SLOT(setRange(QCPRange)));
	connect(m_plot->yAxis, SIGNAL(rangeChanged(QCPRange)), m_plot->yAxis2, SLOT(setRange(QCPRange)));
}

void GroundStation::realtimeDataSlot()
{
	static QRandomGenerator generator;
	static QTime startTime(QTime::currentTime());
	double key = startTime.secsTo(QTime::currentTime());
	//QElapsedTimer timer;
	//timer.start();

	//Sleep(3);
	static double lastPointKey = 0;

	//double key = timer.elapsed();
	if (key - lastPointKey > 0.002)
	{
		// add data to lines:
		int r1 = generator.generate();
		double r1_ = r1 / (double)generator.max();
		m_plot->graph(0)->addData(key, qSin(key) + r1_ * 1 * qSin(key / 0.3843));

		int r2 = generator.generate();
		double r2_ = r2 / (double)generator.max();
		m_plot->graph(1)->addData(key, qCos(key) + r2_ * 0.5 * qSin(key / 0.4364));
		// rescale value (vertical) axis to fit the current data:
		//ui->customPlot->graph(0)->rescaleValueAxis();
		//ui->customPlot->graph(1)->rescaleValueAxis(true);
		lastPointKey = key;
	}

	// make key axis range scroll with the data (at a constant range size of 8):
	m_plot->xAxis->setRange(key, 8, Qt::AlignRight);
	m_plot->replot();

	// calculate frames per second:
	static double lastFpsKey = 0;
	static int frameCount;
	++frameCount;
	if (key - lastFpsKey > 2) // average fps over 2 seconds
	{
		//ui->statusBar->showMessage(
		//	QString("%1 FPS, Total Data points: %2")
		//	.arg(frameCount / (key - lastFpsKey), 0, 'f', 0)
		//	.arg(ui->customPlot->graph(0)->data()->size() + ui->customPlot->graph(1)->data()->size())
		//	, 0);
		lastFpsKey = key;
		frameCount = 0;
	}
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
	m_plot->replot();
}
#else
void GroundStation::onReadyRead()
{
	static QTime startTime(QTime::currentTime());
	double key = startTime.secsTo(QTime::currentTime());
	static double lastPointKey = 0;
	//QByteArray arr = m_serialPort.readAll();
	//QList<QByteArray> L =  arr.split('\n');

	QByteArray line = m_serialPort.readLine();
	QString item = line.constData();
	if (item.startsWith("AC", Qt::CaseInsensitive) && 
		key - lastPointKey > 0.002)
	{
		QRegExp rx("[^\\d]+");
		const auto&& parts = item.split(rx, Qt::SkipEmptyParts);
		//m_accelX.append(parts[0].toInt());
		//m_accelY.append(parts[1].toInt());
		//m_accelZ.append(parts[2].toInt());
		
		m_plot->graph(0)->addData(key, parts[0].toInt());
		//m_plot->graph(0)->addData(1, m_accelX[0]);
		//m_plot->graph(0)->addData(1, m_accelX[0]);
		lastPointKey = key;
	}

	m_plot->xAxis->setRange(key, 1000, Qt::AlignRight);
	m_plot->replot(QCustomPlot::rpQueuedReplot);
}
#endif