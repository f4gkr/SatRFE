//==========================================================================================
// + + +   This Software is released under the "Simplified BSD License"  + + +
// Copyright F4GKR Sylvain AZARIAN . All rights reserved.
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
#include <QDebug>
#include <QApplication>
#include <QStyleFactory>
#include <QSplashScreen>
#include <QMessageBox>

#include "common/constants.h"

#include "mainwindow.h"
#include "core/controller.h"

#include "hardware/rxhardwareselector.h"
#include "hardware/gpdsd.h"
#include "common/QLogger.h"

#include "httpserver/httplistener.h"
#include "webinterface/webservice.h"


int main(int argc, char *argv[])
{

    RxDevice *radio = NULL ;

    HttpListener* webserver;
    QApplication a(argc, argv);
    QApplication::setStyle(QStyleFactory::create("plastique"));

    // load configuration file
    GlobalConfig& global = GlobalConfig::getInstance() ;

    QLogger::QLoggerManager *manager = QLogger::QLoggerManager::getInstance();
    manager->addDestination( LOGGER_FILENAME, QStringList( LOGGER_NAME ),  QLogger::TraceLevel);
    QLogger::QLog_Trace( LOGGER_NAME, "------------------------------------------------------------------------------" );
    QLogger::QLog_Trace( LOGGER_NAME, "Build date : " + QString(BUILD_DATE));
    QLogger::QLog_Trace( LOGGER_NAME, "Starting" );


    Controller& control = Controller::getInstance() ;

    QMessageBox msgBox;
    RxHardwareSelector *rxs = new RxHardwareSelector();
    radio = rxs->getReceiver() ;
    if( radio == NULL ) {
        QLogger::QLog_Error( LOGGER_NAME, "No SDR device connected, cannot continue.");
        msgBox.setWindowTitle( VER_PRODUCTNAME_STR );
        msgBox.setText("ERROR:  No SDR device detected !");
        msgBox.setStandardButtons(QMessageBox::Yes );
        msgBox.exec() ;
        return(-1);
    }


    control.setRadio( radio );
    control.start();

    // start web server
    QSettings settings( QApplication::applicationDirPath() + "/" + QString(CONFIG_FILENAME), QSettings::IniFormat);
    settings.beginGroup("WebServer");
    WebService *ws = new WebService(&a);
    webserver = new HttpListener( &settings, ws, &a);
    control.setWebService( ws );

    MainWindow *w = new MainWindow();
    w->setRadio( radio );
    w->setWebService( ws );
    w->show();

    return a.exec();
}
