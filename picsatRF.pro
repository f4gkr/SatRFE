#==========================================================================================
# + + +   This Software is released under the "Simplified BSD License"  + + +
# Copyright 2014-2017 F4GKR Sylvain AZARIAN . All rights reserved.
#
#Redistribution and use in source and binary forms, with or without modification, are
#permitted provided that the following conditions are met:
#
#   1. Redistributions of source code must retain the above copyright notice, this list of
#	  conditions and the following disclaimer.
#
#   2. Redistributions in binary form must reproduce the above copyright notice, this list
#	  of conditions and the following disclaimer in the documentation and/or other materials
#	  provided with the distribution.
#
#THIS SOFTWARE IS PROVIDED BY Sylvain AZARIAN F4GKR ``AS IS'' AND ANY EXPRESS OR IMPLIED
#WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
#FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Sylvain AZARIAN OR
#CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
#NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#The views and conclusions contained in the software and documentation are those of the
#authors and should not be interpreted as representing official policies, either expressed
#or implied, of Sylvain AZARIAN F4GKR.
#==========================================================================================
#-------------------------------------------------

QT       += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = picsatRF
TEMPLATE = app

DEFINES += BUILD_DATE='"\\\"$(shell  date +\"%Y%m%d\")\\\""'

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

include( qwt/qwt.pri )
include( httpserver/httpserver.pri)

LIBS +=  -lusb-1.0 -lpthread -lrtlsdr  -lfftw3f -lm
win32 {

} else {
    LIBS += -lgps
}

SOURCES += main.cpp\
    mainwindow.cpp \
    ui/freqctrl.cpp \
    hardware/rtlsdr.cpp \
    common/QLogger.cpp \
    common/samplefifo.cpp \
    ui/spectrumplot.cpp \
    core/controller.cpp \
    dsp/overlapsave.cpp \
    ui/indicatorwidget.cpp \
    ui/qcustomplot.cpp \
    hardware/gpdsd.cpp \
    hardware/windows/tinygps.cpp \
    hardware/windows/rs232.c \
    ui/gkdial.cpp \
    dsp/frameprocessor.cpp \
    dsp/frametodecoder.cpp \
    core/sampleblock.cpp \
    common/constants.cpp \
    ui/plotter.cpp \
    ui/bookmarks.cpp \
    webinterface/webservice.cpp \
    dsp/activity.cpp

HEADERS  += mainwindow.h \
    ui/freqctrl.h \
    hardware/rtlsdr.h \
    common/QLogger.h \
    common/constants.h \
    common/samplefifo.h \
    ui/spectrumplot.h \
    core/controller.h \
    dsp/overlapsave.h \
    ui/indicatorwidget.h \
    ui/qcustomplot.h \
    hardware/gpdsd.h \
    hardware/windows/rs232.h \
    hardware/windows/tinygps.h \
    ui/gkdial.h \
    dsp/frameprocessor.h \
    dsp/frametodecoder.h \
    core/sampleblock.h \
    ui/plotter.h \
    ui/bookmarks.h \
    webinterface/webservice.h \
    dsp/activity.h
