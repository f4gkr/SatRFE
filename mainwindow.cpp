//==========================================================================================
// + + +   This Software is released under the "Simplified BSD License"  + + +
// Copyright 2014-2017 F4GKR Sylvain AZARIAN . All rights reserved.
//
//Redistribution and use in source and binary forms, with or without modification, are
//permitted provided that the following conditions are met:
//
//   1. Redistributions of source code must retain the above copyright notice, this list of
//	  conditions and the following disclaimer.
//
//   2. Redistributions in binary form must reproduce the above copyright notice, this list
//	  of conditions and the following disclaimer in the documentation and/or other materials
//	  provided with the distribution.
//
//THIS SOFTWARE IS PROVIDED BY Sylvain AZARIAN F4GKR ``AS IS'' AND ANY EXPRESS OR IMPLIED
//WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
//FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Sylvain AZARIAN OR
//CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
//ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
//NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
//ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//The views and conclusions contained in the software and documentation are those of the
//authors and should not be interpreted as representing official policies, either expressed
//or implied, of Sylvain AZARIAN F4GKR.
//==========================================================================================
#include "mainwindow.h"
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QDesktopWidget>
#include <QApplication>
#include <QLabel>
#include <QPalette>
#include <QMessageBox>
#include <stdint.h>

#include "core/controller.h"
#include "hardware/gpdsd.h"
#include "common/QLogger.h"
#include "common/constants.h"
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    radio = NULL ;
    setAttribute(Qt::WA_DeleteOnClose);

    QWidget *center_widget = new QWidget;
    QVBoxLayout *vlayout = new QVBoxLayout;
    center_widget->setLayout(vlayout);
    vlayout->setContentsMargins( 1,1,1,1);
    vlayout->setAlignment( Qt::AlignTop );

    // Create top band
    QWidget *top_band = new QWidget;
    QHBoxLayout *tb_layout = new QHBoxLayout;
    tb_layout->setAlignment( Qt::AlignLeft );
    tb_layout->setContentsMargins( 1,1,1,1);

    QPushButton *startButton = new QPushButton("START");
    startButton->setMaximumWidth(50);
    startButton->setToolTip(tr("Start SDR"));
    tb_layout->addWidget( startButton );

    QPushButton *stopButton = new QPushButton("STOP");
    stopButton->setMaximumWidth(50);
    stopButton->setToolTip(tr("Stop SDR"));
    tb_layout->addWidget( stopButton );

    mainFDisplay = new CFreqCtrl();
    mainFDisplay->setMinimumWidth(200);
    mainFDisplay->setMinimumWidth(500);
    tb_layout->addWidget(mainFDisplay);
    top_band->setMaximumHeight(40);
    top_band->setLayout(tb_layout);
    vlayout->addWidget( top_band );

    // center band
    QWidget* center_band = new QWidget();
    QHBoxLayout *cb_layout = new QHBoxLayout;
    center_band->setLayout(cb_layout);
    cb_layout->setAlignment( Qt::AlignLeft | Qt::AlignTop );
    cb_layout->setContentsMargins( 1,1,1,1);

    plot = new SpectrumPlot();
    plot->setMinMaxScales( -90, -50 );
    plot->setMinimumHeight( 100 );
    cb_layout->addWidget( plot );

    // center left
    QWidget *cl_widget = new QWidget;
    cl_widget->setMaximumWidth(170);
    QVBoxLayout *cllayout = new QVBoxLayout;
    cl_widget->setLayout(cllayout);
    cllayout->setContentsMargins( 1,1,1,1);
    cllayout->setAlignment(Qt::AlignLeft | Qt::AlignTop  );

    gain_rx = new gkDial(4,tr("RF Gain"));
    gain_rx->setScale(0,40);
    gain_rx->setValue(10);
    cllayout->addWidget(gain_rx);

    detection_threshold= new gkDial(4,tr("Threshold"));
    detection_threshold->setScale(2,20);
    detection_threshold->setValue(DEFAULT_DETECTION_THRESHOLD);
    cllayout->addWidget(detection_threshold);

    zuluDisplay = new QLCDNumber(11);
    zuluDisplay->setSegmentStyle(QLCDNumber::Flat);
    zuluDisplay->display( "00:00:00.0" );
    zuluDisplay->setToolTip(tr("UTC Time"));
    QPalette zpalette = zuluDisplay->palette() ;
    zpalette.setColor( zpalette.WindowText, QColor(85, 85, 255)) ;
    zpalette.setColor( zpalette.Background, QColor(0, 170, 255));
    zpalette.setColor (zpalette.Dark, QColor(0, 255, 0));
    zuluDisplay->setPalette(zpalette) ;
    cllayout->addWidget(zuluDisplay);

    z_latitude = new QLCDNumber(11);
    z_latitude->setSegmentStyle(QLCDNumber::Flat);
    z_latitude->display( "0.0000000" );
    z_latitude->setToolTip(tr("Latitude"));
    z_latitude->setPalette(zpalette) ;
    cllayout->addWidget(z_latitude);

    z_longitude = new QLCDNumber(11);
    z_longitude->setSegmentStyle(QLCDNumber::Flat);
    z_longitude->display( "0.0000000" );
    z_longitude->setToolTip(tr("Longitude"));
    z_longitude->setPalette(zpalette) ;
    cllayout->addWidget(z_longitude);


    levelWidget = new IndicatorWidget( "Level", 0, 80, "dBc");
    levelWidget->setMinimumWidth(150);
    levelWidget->setMaximumHeight(150);
    cllayout->addWidget( levelWidget);
    //cllayout->addWidget( new QLabel(tr("Max Level:")));



    satDistance = new QLineEdit();
    satDistance->setToolTip(tr("UAV uavDistance"));
    satDistance->setReadOnly(true);
    satDistance->setMaxLength(5);
    satDistance->setMaximumWidth(150);
    cllayout->addWidget( satDistance);

    satElevation = new QLineEdit();
    satElevation->setToolTip(tr("UAV uavElevation"));
    satElevation->setReadOnly(true);
    satElevation->setMaxLength(5);
    satElevation->setMaximumWidth(150);
    cllayout->addWidget( satElevation);

    satAzimuth = new QLineEdit();
    satAzimuth->setToolTip(tr("UAV uavAzimuth"));
    satAzimuth->setReadOnly(true);
    satAzimuth->setMaxLength(5);
    satAzimuth->setMaximumWidth(150);
    cllayout->addWidget( satAzimuth);


    cb_layout->addWidget( cl_widget);

    // Create the bands showing the received portion
    seg_rx = new SpectrumSegment("DATA");
    seg_rx->setColor( Qt::green   );
    seg_rx->setInterval( FRAME_OFFSET_LOW/1e6, (FRAME_OFFSET_LOW +  DEMODULATOR_SAMPLERATE)/1e6 );
    seg_rx->setVisible( true );
    seg_rx->attach( plot );

    vlayout->addWidget( center_band );

    // detection levels etc
    QWidget *plotWidget = new QWidget();
    QVBoxLayout *pwlayout = new QVBoxLayout();
    pwlayout->setContentsMargins( 0, 0, 0, 0 );
    plotWidget->setLayout( pwlayout );



    levelplot = new QCustomPlot();
    levelplot->addGraph();
    levelplot->xAxis->setLabel(tr("Frame"));
    levelplot->yAxis->setLabel("Level");
    levelplot->xAxis->setRange(0, 1);
    levelplot->yAxis->setRange(-70, -50);
    levelplot->setMinimumHeight(150);
    levelplot->addGraph();
    levelplot->graph(1)->setPen(QPen(Qt::red));
    min_level = 0 ;
    max_level = -100 ;

    pwlayout->addWidget( levelplot );

    vlayout->addWidget( plotWidget );


    setCentralWidget(center_widget);
    // resize
    //QDesktopWidget *desktop = QApplication::desktop();
    //resize( desktop->availableGeometry(this).size() * .7 );

    connect( startButton, SIGNAL(pressed()), this, SLOT(SLOT_startPressed()));
    connect( stopButton, SIGNAL(pressed()), this, SLOT(SLOT_stopPressed()));

    connect( mainFDisplay, SIGNAL(newFrequency(qint64)),
             this, SLOT(SLOT_userTunesFreqWidget(qint64)) );
    connect( gain_rx, SIGNAL(valueChanged(int)), this, SLOT(SLOT_setRxGain(int)));
    connect( detection_threshold, SIGNAL(valueChanged(int)), this, SLOT(SLOT_setDetectionThreshold(int)));

    GPSD& gpsd= GPSD::getInstance() ;
    connect( &gpsd, SIGNAL(hasGpsFix(double,double)), this,
             SLOT(SLOT_hasGpsFix(double,double)), Qt::QueuedConnection );
    connect( &gpsd, SIGNAL(hasGpsTime(int,int,int,int,int,int,int)), this,
             SLOT(SLOT_hasGpsTime(int,int,int,int,int,int,int)),Qt::QueuedConnection);

}

void MainWindow::setRadio( RTLSDR* device ) {
    radio = device ;
    mainFDisplay->setup( 11, radio->getMin_HWRx_CenterFreq() ,
                         radio->getMax_HWRx_CenterFreq(),
                         10, UNITS_MHZ );
    mainFDisplay->resetToFrequency( radio->getRxCenterFreq() );
    plot->setNewParams( radio->getRxCenterFreq(), radio->getRxSampleRate() ) ;

    Controller& ctrl = Controller::getInstance() ;
    connect( &ctrl, SIGNAL(newSpectrumAvailable(int, double, double)),  this,
             SLOT(SLOT_newSpectrum(int, double, double)), Qt::QueuedConnection );
    connect( &ctrl, SIGNAL(powerLevel(float)), this, SLOT(SLOT_powerLevel(float)), Qt::QueuedConnection );


    gain_rx->setValue( device->getRxGain()  );
}

void MainWindow::SLOT_userTunesFreqWidget(qint64 newFrequency) {    

    plot->razMaxHold();

    double fmin = (double)newFrequency + FRAME_OFFSET_LOW ;
    double fmax = (double)newFrequency + FRAME_OFFSET_LOW +  DEMODULATOR_SAMPLERATE;

    seg_rx->setInterval(  fmin/1e6, fmax/1e6);

    Controller& ctrl = Controller::getInstance() ;
    ctrl.setRxCenterFrequency( newFrequency );
}

// start SDR pressed
void MainWindow::SLOT_startPressed() {
   qint64 newFrequency = 436470.8e3;
   Controller& ctrl = Controller::getInstance() ;

    if( radio == NULL )
        return ;

    if( ctrl.isAcquiring()  )
            return ;

    received_frame = msg_count = 0 ;
    mainFDisplay->resetToFrequency(  newFrequency );
    plot->setNewParams(  newFrequency, radio->getRxSampleRate() ) ;
    double fmin = (double)newFrequency + FRAME_OFFSET_LOW ;
    double fmax = (double)newFrequency + FRAME_OFFSET_LOW +  DEMODULATOR_SAMPLERATE;
    seg_rx->setInterval(  fmin/1e6, fmax/1e6);

    ctrl.setRxCenterFrequency( newFrequency  );
    ctrl.startAcquisition();
}

void MainWindow::SLOT_stopPressed() {
    Controller& ctrl = Controller::getInstance() ;
    if( radio == NULL )
        return ;
    if( !ctrl.isAcquiring() ) {
        return ;
    }
    ctrl.stopAcquisition();
}


void MainWindow::SLOT_newSpectrum( int len , double smin,  double smax ) {
    double power_dB[len] ;
    float bw ;
    //bool rescale = false ;setRTLGain

    uint64_t rx_center_frequency = radio->getRxCenterFreq() ;
    bw = radio->getRxSampleRate() ;
    Controller& ctrl = Controller::getInstance() ;
    ctrl.getSpectrum( power_dB );

//    if( smin < plot->getMinScale() ) {
//         rescale = true ;
//    } else {
//        if( abs( smin - plot->getMinScale() ) > 25 ) {
//             rescale = true ;
//        }
//    }

//    if( smax > plot->getMaxScale() ) {
//         rescale = true ;
//    } else {
//        if( abs( smax - plot->getMaxScale() ) > 25 ) {
//             rescale = true ;
//        }
//    }

//    if( rescale ) {
//        plot->setMinMaxScales( .1*smin + .9*plot->getMinScale(),
//                               .1*smax  + .9*plot->getMaxScale()  );
//    }

    plot->setPowerTab(rx_center_frequency, power_dB,  len, bw );
    double fmin = (double)rx_center_frequency + FRAME_OFFSET_LOW ;
    double fmax = (double)rx_center_frequency +  FRAME_OFFSET_LOW +  DEMODULATOR_SAMPLERATE ;
    seg_rx->setInterval(  fmin/1e6, fmax/1e6);
}

MainWindow::~MainWindow()
{
    SLOT_stopPressed();

    GPSD& gpsd= GPSD::getInstance() ;
    gpsd.shutdown();

    Controller& ctrl = Controller::getInstance() ;
    ctrl.close();

}

void MainWindow::SLOT_powerLevel( float level )  {
    //qDebug() << "level=" << level ;
    levelWidget->setValue( level );    
    levelplot->graph(0)->addData( msg_count/10.0, level);
    levelplot->xAxis->setRange( msg_count/10.0,60,Qt::AlignRight );
    levelplot->replot();
    msg_count++ ;
}


void MainWindow::SLOT_hasGpsFix(double latitude, double longitude ) {
    z_latitude->display( QString::number( latitude, 'f', 8));
    z_longitude->display( QString::number( longitude, 'f', 8));
}

void MainWindow::SLOT_hasGpsTime(int year, int month, int day, int hour, int min, int sec, int msec) {
    Q_UNUSED(msec);
    QString zuluTime = QString("%1").arg(hour, 2, 10, QChar('0')) + ":" +
            QString("%1").arg(min, 2, 10, QChar('0')) + ":" +
            QString("%1").arg(sec, 2, 10, QChar('0')) + "." + QString::number(msec);


    zuluDisplay->display( zuluTime );
}

void MainWindow::SLOT_setRxGain(int g) {
        if( radio == NULL )
            return ;
        radio->setRTLGain( g );
}

void MainWindow::SLOT_setDetectionThreshold(int level) {
      Controller& ctrl = Controller::getInstance() ;
      ctrl.setDetectionThreshold(level);
}
