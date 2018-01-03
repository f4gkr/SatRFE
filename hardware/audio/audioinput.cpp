//==========================================================================================
// + + +   This Software is released under the "Simplified BSD License"  + + +
// Copyright 2014 F4GKR Sylvain AZARIAN . All rights reserved.
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
#include "audioinput.h"
#include <QDebug>


#define TBUFF_LEN (65536*2)

AudioInput::AudioInput(QString inputName, int desired_sr )
{
    closing = false ;
    outgoing_fifo = new SampleFifo();
    m_samplingRate = 0 ;
    xn_1.im = xn_1.re = 0 ;
    yn_1.im = yn_1.re = 0 ;

    format.setSampleRate(desired_sr);
    format.setChannelCount(2);
    format.setSampleSize(16);
    format.setCodec("audio/pcm");
    format.setByteOrder(QAudioFormat::LittleEndian);
    format.setSampleType(QAudioFormat::SignedInt);
    inputDevice = NULL ;

    buff_len  = 2 * TBUFF_LEN * sizeof(short int);
    audio_buff = (short int *)malloc( buff_len );
    wr_pos = 0 ;

    foreach (const QAudioDeviceInfo &deviceInfo, QAudioDeviceInfo::availableDevices(QAudio::AudioInput)) {
        QString deviceName = deviceInfo.deviceName();

        qDebug() << "interface audio" << deviceName ;

        if( deviceName.indexOf( inputName ) > 0 ) {
            qDebug() << "Device name: " << deviceInfo.deviceName();
            QList<int> rates = deviceInfo.supportedSampleRates() ;
            for( int i=0 ; i < rates.size() ; i++ ) {
                int ssr = rates.at(i);
                qDebug() << "supported sample rate :" << i << ssr ;
            }

            if( deviceInfo.isFormatSupported( format )) {
                qDebug() << "using " << deviceName ;
                audio = new QAudioInput( deviceInfo, format, this );
                audio->setVolume(1);
                connect( audio, SIGNAL(stateChanged(QAudio::State)), this, SLOT(stateChanged(QAudio::State)));
                //audio->setBufferSize( 1024 );
                //audio->setNotifyInterval(1);
                m_samplingRate = audio->format().sampleRate() ;
                qDebug() << "sample rate set to " << m_samplingRate ;
                return ;
            }
        }

    }


}

bool AudioInput::startAudio() {
    if( inputDevice != NULL ) {
        return(true);
    }

    QIODevice::open(QIODevice::ReadWrite);
    audio->start(this);

    qDebug() << "AudioInput::startAudio()" ;
    return( true );
}

void AudioInput::stopAudio() {
    audio->stop();
}


void AudioInput::stateChanged(QAudio::State state) {
    qDebug() << "AudioInput::stateChanged" ;
    switch( state ) {
    case QAudio::ActiveState:
        qDebug() << "ActiveState" ;
        m_samplingRate = audio->format().sampleRate() ;
        qDebug() << "sample rate set to " << m_samplingRate ;
        break ;
    case QAudio::SuspendedState: qDebug() << "SuspendedState" ; break ;
    case QAudio::StoppedState: qDebug() << "StoppedState" ; break ;
    case QAudio::IdleState: qDebug() << "IdleState" ; break ;
    }
}

void AudioInput::setIQCorrectionFactors( TYPEREAL gainI, TYPEREAL gainQ,
                                         TYPEREAL crossGain ) {
    AmPhAAAA = gainI ;
    AmPhDDDD = gainQ ;
    AmPhCCCC = crossGain ;

}

void AudioInput::getIQCorrectionFactors( TYPEREAL *gainI,
                                         TYPEREAL *gainQ,
                                         TYPEREAL *crossGain ) {
    *gainI = AmPhAAAA ;
    *gainQ = AmPhDDDD ;
    *crossGain = AmPhCCCC ;
}

#define BUFF_LEN (4096)
#define ALPHA_DC (0.996)


qint64 AudioInput::readData(char* data, qint64 maxLen)
{
    Q_UNUSED(data);
    Q_UNUSED(maxLen);
    return 0;
}

qint64 AudioInput::writeData(const char *data, qint64 len)
{
    TYPEREAL I,Q ;
    TYPECPX* buff ;
    int nbytes = (2*sizeof(short int)) ;
    char *src = (char *)data ;
    qint64 remain = len ;

    //qDebug() << "AudioInput::writeData" << len ;
    qint64 room_left = buff_len - wr_pos ;
    short int *wr = audio_buff ;
    wr += wr_pos ;
    if( room_left >= remain ) {
        //qDebug() << "AudioInput::writeData adding data to audio_buff : " << len ;
        memcpy( (void *)wr, (void *)src, remain );
        wr_pos += remain ;
        remain = 0 ;
    }

    //qDebug() << "AudioInput::writeData" << room_left << wr_pos << remain;

    int nsample_pairs = wr_pos / nbytes ;
    if( nsample_pairs > 16384 ) {
        buff = (TYPECPX *)malloc( nsample_pairs*sizeof( TYPECPX ));
        for( int i=0 ; i < nsample_pairs ; i++ ) {
            int j=2*i ;
            I = ((TYPEREAL)audio_buff[j  ])/32768.0 ;
            Q = ((TYPEREAL)audio_buff[j+1])/32768.0 ;
            buff[i].re = I ;
            buff[i].im = Q ;
        }

        qint64 bytes_consumed = nbytes * nsample_pairs ;
        wr_pos -= bytes_consumed ;
        src += bytes_consumed ;

        qDebug() << "AudioInput::writeData" << room_left << wr_pos << remain << nsample_pairs;
        if( outgoing_fifo->EnqueueData( (void *)buff, nsample_pairs, 0, NULL ) < 0 ) {
            qDebug() << " AudioInput::run() queue size !!!!" ;
        }
    }

    if( remain > 0 ) {
        room_left = buff_len - wr_pos ;
        wr = audio_buff ;
        wr += wr_pos ;
        //qDebug() << "AudioInput::writeData adding last data to audio_buff : " << remain ;
        memcpy( (void *)wr, (void *)src, remain );
        wr_pos += remain ;
    }
    return(len) ;
}
