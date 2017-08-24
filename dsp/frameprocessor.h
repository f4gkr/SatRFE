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
#ifndef UAVPROCESSOR_H
#define UAVPROCESSOR_H

#include <QObject>
#include <QQueue>
#include "common/samplefifo.h"
#include "common/datatypes.h"

class SampleBlock {
public:
    SampleBlock( TYPECPX* IQsamples, int L );
    ~SampleBlock();

    void markAsLastBlock();
    TYPECPX *getData() { return( memory ) ; }
    int getLength() { return( mSize ) ; }
private:
    TYPECPX *memory ;
    int mSize ;
    bool lastBlock ;
};

#define DEFAULT_DETECTION_THRESHOLD (10)

class FrameProcessor : public QObject
{
    Q_OBJECT
public:

    explicit FrameProcessor(QObject *parent = 0);

    // change detection threshold for the correlator
    // default is 30dB
    void setDetectionThreshold(float level);

    // call this function each time a paquet of samples is available
    int newData(TYPECPX* IQsamples, int L , int sampleRate );

    void raz();

signals:
    void powerLevel( float level ) ;
    void frameDetected( float signal_level  ) ;

public slots:

private:
    enum  { sInit=0 ,
            sMeasureNoise=1,
            sSearchFrame=2,
            sFrameStart=3,
            sFrameEnds=4
          } ;
    int m_state, next_state ;
    QQueue<SampleBlock*> queue ;
    long queueSampleCount ;

    int m_bandwidth ;
    float threshold ; // effective RMS value considered for frame proc
    float noise_floor;
    float detection_threshold ; // snr required for detection

    double rms_power ;
    long samples_for_powerestimation ;

    float rmsp(TYPECPX *samples, int L );

    QString stateToS(int s);
    void flushQueue(int L);
    void writeQueueToFile( QString filename, long samples );
};

#endif // UAVPROCESSOR_H
