#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QSerialPort>
#include <QLabel>
#include <QLineEdit>
#include <QFile>
#include <QString>
#include <QMessageBox>
#include <QFileDialog>
#include <QtSerialPort/QSerialPortInfo>
#include <QDialog>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>
#include <QThread>
#include <QFontMetrics>

QSerialPort *serial;
QByteArray ReceivedData;
QByteArray TransmitData;
int portstatus = 0;
char datax1[400];
char datax2[40];

void MainWindow::serialReceived(){
    ReceivedData.clear();
}

void MainWindow::replyAction(){
    TransmitData.clear();
	
    TransmitData.append("{");//"{000}\n"
    sprintf(datax1, "%03d", ui->dial_Serv_1->value());
    TransmitData.append(datax1);
	TransmitData.append("}\n");

    serial->write(TransmitData);
}

void MainWindow::handleBytesWritten(qint64 bytes){
	replyAction();
}
void MainWindow::on_actionConnectButtonAction_triggered(){
	uint16_t vid = 0;
	uint16_t pid = 0;
	int fr = 0;
    if (portstatus == 0){
		fr = 1;
		QString RPT;
		for (QSerialPortInfo port : QSerialPortInfo::availablePorts()){
			vid = port.vendorIdentifier();
			pid = port.productIdentifier();
			if (vid == 1155){
				if(pid == 22336){
					RPT.clear();
					RPT.append(port.portName());
					serial->setPortName(RPT);
                    serial->setBaudRate(230400);
					serial->setDataBits(QSerialPort::Data8);
					serial->setParity(QSerialPort::NoParity);
					serial->setStopBits(QSerialPort::OneStop);
					serial->setFlowControl(QSerialPort::NoFlowControl);
					if((serial->open(QIODevice::ReadWrite) == true)){
						ui->ConnectButton->setText("Disconnect");
						portstatus = 1;
						ui->StatusLabel->setText("Connected!");
						fr = 2;
						replyAction();
					}
				}
			}
		}
    }
    else{
        serial->close();
        ui->ConnectButton->setText("Connect");
        portstatus = 0;
		ui->StatusLabel->setText("Disonnected!");
    }
	if (fr == 1){
		ui->StatusLabel->setText("Device not found!");
	}
}

//MainWindow
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow){
    //Serial port
    serial = new QSerialPort(this);
    connect(serial,SIGNAL(readyRead()),this,SLOT(serialReceived()));
	connect(serial, SIGNAL(bytesWritten(qint64)), SLOT(handleBytesWritten(qint64)));
    //UI
    setFixedSize(400, 440);
    ui->setupUi(this);
}
MainWindow::~MainWindow(){
    delete ui;
    serial->close();
}

