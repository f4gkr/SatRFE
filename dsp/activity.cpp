#include <QDebug>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "activity.h"


#define FFT_SIZE (512)

#define BUFFER_SIZE (2048)
#define DEBUG_ADETECTOR (1)

ActivityDetector::ActivityDetector() : QObject(NULL)
{

    fftin = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * FFT_SIZE);
    buffer = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * BUFFER_SIZE);
    plan = fftwf_plan_dft_1d( FFT_SIZE, fftin, fftin, FFTW_FORWARD, FFTW_ESTIMATE );
    plan_rev = fftwf_plan_dft_1d( FFT_SIZE, fftin, fftin, FFTW_BACKWARD, FFTW_ESTIMATE );
    w_pos = 0 ;
    A = B = C = -100 ;
    _last = QDateTime::currentDateTimeUtc() ;

    channelMachine = new QStateMachine(this);
    cmOn = new QState(channelMachine);
    cmOn->setObjectName("channelOn");

    cmPending = new QState(channelMachine);
    cmPending->setObjectName("channelPending");

    cmOff = new QState(channelMachine);
    cmOff->setObjectName("channelOff");

    channelMachine->setInitialState(cmOff);

    cmOff->addTransition( this, SIGNAL(active()), cmOn);
    cmOn->addTransition( this, SIGNAL(inactive()), cmPending) ;
    cmPending->addTransition( this, SIGNAL(active()), cmOn );
    cmOff->addTransition( this, SIGNAL(inactive()), cmOff );

    connect( cmOn, SIGNAL(entered()), this, SLOT(channelOn()));
    connect( cmOn, SIGNAL(exited()), this, SLOT(channelOff()));

    channelMachine->start();
}

void ActivityDetector::channelOn() {
    if( DEBUG_ADETECTOR ) qDebug() << "RepeaterChannel::channelOn()" ;
    _last.setMSecsSinceEpoch( QDateTime::currentDateTimeUtc().toMSecsSinceEpoch() );
}

void ActivityDetector::channelOff() {
    long duration = _last.msecsTo( QDateTime::currentDateTimeUtc() ) ;
    if( DEBUG_ADETECTOR ) qDebug() << "RepeaterChannel::channelOff()" ;
    if( DEBUG_ADETECTOR ) qDebug() << " active time was : " << duration << " milliseconds" ;

}


float ActivityDetector::modulus(int i) {
    float a = fftin[i][0];
    float b = fftin[i][1];
    return( a*a + b*b );
}

#define CX1 (.5)
#define CX2 (.5)

void ActivityDetector::processSamples(TYPECPX *in, int len ) {
    fftwf_complex *pt ;
    int size = qMin(BUFFER_SIZE - w_pos, len ) ;
    if( size <= 0 )
        return ;

    pt = buffer + w_pos ;
    memcpy( (void *)pt, (void *)in, size * sizeof(fftwf_complex));
    w_pos += len ;

    while( w_pos > (FFT_SIZE/2) ) {
           // compute  https://stackoverflow.com/questions/3949324/calculate-autocorrelation-using-fft-in-matlab
           // x = [ ... ];
           // x_pad = [x zeros(size(x))];
           memcpy( (void *)fftin, (void *)buffer, (FFT_SIZE/2) * sizeof( fftwf_complex));

           pt = fftin + (FFT_SIZE/2) ;
           memset( (void *)pt, 0, (FFT_SIZE/2)*sizeof(fftwf_complex));
           // X  = fft(x_pad);
           fftwf_execute( plan );
           // X_psd = abs(X).^2;
           for( int i=0 ; i < FFT_SIZE ; i++ ) {
                fftin[i][0] = modulus(i) ;
                fftin[i][1] = 0 ;
           }
           // r_xx = ifft(X_psd);
           fftwf_execute(plan_rev);
           float root = modulus(0);
           A = CX1*A + CX2*(20*log10f(modulus(1)/root));
           B = CX1*B + CX2*(20*log10f(modulus(4)/root));
           C = CX1*C + CX2*(20*log10f(modulus(8)/root));

           if(  DEBUG_ADETECTOR ) {
               char filename[255];
               sprintf( filename, "fac_channel.dat") ;
               FILE *fw = fopen(filename, "a+") ;
               fprintf( fw, "%.3f;%.3f;%.3f\n", A, B,C);
               fclose(fw);
           }

           root = A + B + C ;
           if( root > -30 ) {
               emit active();
           } else {
               emit inactive();
           }

           // prepare data for next run
           pt = buffer + (FFT_SIZE/2) ;
           w_pos -= (FFT_SIZE/2) ;
           memmove( (void *)buffer, (void *)pt, (BUFFER_SIZE-w_pos) * sizeof( fftwf_complex));
    }
}
