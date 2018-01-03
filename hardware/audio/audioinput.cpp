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
            if( deviceInfo.isFormatSupported( format )) {
                qDebug() << "using " << deviceName ;
                audio = new QAudioInput( deviceInfo, format, this );
                connect( audio, SIGNAL(stateChanged(QAudio::State)), this, SLOT(stateChanged(QAudio::State)));
                audio->setBufferSize( 1024 );
                //audio->setNotifyInterval(1);
                m_samplingRate = desired_sr ;
                //audio->reset();

                return ;
            }
        }

    }


}

bool AudioInput::startAudio() {
    if( inputDevice != NULL ) {
        return(true);
    }

//    audioSamples = new RingBuffer();
//    connect( audioSamples, SIGNAL(readyRead()), this, SLOT(newAudioSamples()));
//    connect( audioSamples, SIGNAL(bytesWritten(qint64)), this, SLOT(bytesWritten(qint64)));
//    audioSamples->open(QIODevice::ReadWrite);
//    audio->start( audioSamples );

    QIODevice::open(QIODevice::ReadWrite);
    audio->start(this);

    qDebug() << "AudioInput::startAudio()" ;
    return( true );
}

void AudioInput::stopAudio() {
    audio->stop();
    //audioSamples->deleteLater();
}

void AudioInput::bytesWritten(qint64 bytes) {
    qDebug() << "AudioInput::bytesWritten " << bytes ;
}

void AudioInput::stateChanged(QAudio::State state) {
    qDebug() << "AudioInput::stateChanged" ;
    switch( state ) {
    case QAudio::ActiveState:
        qDebug() << "ActiveState" ;

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
        memcpy( (void *)wr, (void *)src, len );
        wr_pos += len ;
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
             wr_pos -= nbytes ;
             remain -= nbytes ;
             src += nbytes ;
        }

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
        wr_pos += len ;
    }
    return(len) ;
}

void AudioInput::newAudioSamples() {
//    short int *audio_buff ;
//    TYPECPX* buff ;
//    TYPECPX tmp ;
//    TYPEREAL I,Q ;

//    int cnt = 0 ;
//    qint16 sI, sQ ;

//    //audioSamples->seek(0) ;//
//    int blockSize = 0 ; //audioSamples->size() ;
//    int n_samples = blockSize / (2* sizeof( short int ));
//    if( n_samples == 0 ) {
//        return ;
//    }

//    buff = (TYPECPX *)malloc( n_samples*sizeof( TYPECPX ));

//    QDataStream sampleStream(audioSamples);
//    while( !sampleStream.atEnd() ) {
//        sampleStream >> sI ;
//        sampleStream >> sQ ;

//        I = ((TYPEREAL)(sI))/32768.0 ;
//        Q = ((TYPEREAL)(sQ))/32768.0 ;

//        // IQ Correction
//        I *= AmPhAAAA ;
//        Q = I*AmPhCCCC + Q*AmPhDDDD ;
//        // DC
//        // y[n] = x[n] - x[n-1] + alpha * y[n-1]
//        // see http://peabody.sapp.org/class/dmp2/lab/dcblock/
//        tmp.re = I - xn_1.re + ALPHA_DC * yn_1.re ;
//        tmp.im = Q - xn_1.im + ALPHA_DC * yn_1.im ;

//        xn_1.re = I ;
//        xn_1.im = Q ;

//        yn_1.re = tmp.re ;
//        yn_1.im = tmp.im ;

//        //qDebug() << I << Q ;
//        buff[cnt].re = I ;
//        buff[cnt].im = Q ;

//        cnt++ ;
//    }

//    qDebug() << "cnt=" << cnt ;
//    if( outgoing_fifo->EnqueueData( (void *)buff, cnt, 0, NULL ) < 0 ) {
//        qDebug() << " AudioInput::run() queue size !!!!" ;
//    }

//    audioSamples->seek(0) ;
//    int blockSize = audioSamples->size() ;
//    qDebug() << "blockSize=" << blockSize ;
//    data = audioSamples->readAll() ;
//    qDebug() << "QByteArray size:" << data.size() ;
//    audioSamples->reset() ;

    /*
    int n_samples = audioSamples->bytesAvailable() / (2* sizeof( short int ));
    size_t buff_size = n_samples * 2 * sizeof( short int ) ;
    qDebug() << "AudioInput::newAudioSamples() adding " << n_samples << " in queue" ;

    audio_buff = ( short int * )malloc( buff_size );
    qint64 bytesRead = audioSamples->read( (char *)audio_buff, buff_size );

    if( bytesRead > 0 ) {
        short int *pt = audio_buff ;
        buff = (TYPECPX *)malloc( n_samples*sizeof( TYPECPX ));
        for( int i=0 ; i < n_samples ; i++ ) {
            I = ((TYPEREAL)(*pt))/32768.0 ;
            pt++ ;
            Q = ((TYPEREAL)(*pt))/32768.0 ;
            pt++ ;

            // IQ Correction
            I *= AmPhAAAA ;
            Q = I*AmPhCCCC + Q*AmPhDDDD ;
            // DC
            // y[n] = x[n] - x[n-1] + alpha * y[n-1]
            // see http://peabody.sapp.org/class/dmp2/lab/dcblock/
            tmp.re = I - xn_1.re + ALPHA_DC * yn_1.re ;
            tmp.im = Q - xn_1.im + ALPHA_DC * yn_1.im ;

            xn_1.re = I ;
            xn_1.im = Q ;

            yn_1.re = tmp.re ;
            yn_1.im = tmp.im ;

            //qDebug() << I << Q ;
            buff[i].re = I ;
            buff[i].im = Q ;
        }
        if( outgoing_fifo->EnqueueData( (void *)buff, n_samples, 0, NULL ) < 0 ) {
            qDebug() << " AudioInput::run() queue size !!!!" ;
        }
    }
    free(audio_buff);
    */
}

/*
void AudioInput::run() {
    short int *audio_buff ;
    TYPECPX* buff ;
    TYPECPX tmp ;
    TYPEREAL I,Q ;
    size_t buff_size ;
    struct rf_context *ctx ;

    audio->reset();
    QIODevice *in_stream = audio->start();
    if( in_stream != NULL ) {
        qDebug() << "Audio stream ok" ;
    }

    closing = false ;
    buff_size = BUFF_LEN * 2 * sizeof( short int ) ;
    audio_buff = ( short int * )malloc( buff_size );
    while( !closing ) {
        qint64 bytesRead = in_stream->read( (char *)audio_buff, buff_size );
        if( bytesRead > 0 ) {
            ctx = NULL ;
            int n_samples = bytesRead / (2*sizeof( short int ) );
            short int *pt = audio_buff ;
            buff = (TYPECPX *)malloc( n_samples*sizeof( TYPECPX ));
            for( int i=0 ; i < n_samples ; i++ ) {
                 I = ((TYPEREAL)(*pt))/32768.0 ;
                 pt++ ;
                 Q = ((TYPEREAL)(*pt))/32768.0 ;
                 pt++ ;

                 // IQ Correction
                 I *= AmPhAAAA ;
                 Q = I*AmPhCCCC + Q*AmPhDDDD ;
                 // DC
                 // y[n] = x[n] - x[n-1] + alpha * y[n-1]
                 // see http://peabody.sapp.org/class/dmp2/lab/dcblock/
                 tmp.re = I - xn_1.re + ALPHA_DC * yn_1.re ;
                 tmp.im = Q - xn_1.im + ALPHA_DC * yn_1.im ;

                 xn_1.re = I ;
                 xn_1.im = Q ;

                 yn_1.re = tmp.re ;
                 yn_1.im = tmp.im ;

                 //qDebug() << I << Q ;
                 buff[i].re = I ;
                 buff[i].im = Q ;
            }
            if( 0 ) {
                FILE *f = fopen( "audioinput_raw.dat", "ab" );
                fwrite( buff, sizeof( TYPECPX), n_samples, f );
                fclose( f );
            }
            qDebug() << "AudioInput::run() adding " << n_samples << " in queue" ;
            if( outgoing_fifo->EnqueueData( (void *)buff, n_samples, 0, ctx ) < 0 ) {
                qDebug() << " AudioInput::run() queue size !!!!" ;
            }
        }
    }
    qDebug() << "end of run()" ;
}
*/
