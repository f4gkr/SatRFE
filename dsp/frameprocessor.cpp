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
#include "frameprocessor.h"
#include <stdlib.h>
#include <QDebug>
#include <math.h>

#include "common/constants.h"

//-- Class SampleBlock

SampleBlock::SampleBlock( TYPECPX* IQsamples, int L ) {
    mSize = L ;
    memory = (TYPECPX *)malloc( mSize * sizeof(TYPECPX));
    memcpy( memory, IQsamples, mSize * sizeof(TYPECPX));
    lastBlock = false ;
}

SampleBlock::~SampleBlock() {
    free( memory );
    memory = NULL ;
    mSize = 0 ;
}

void SampleBlock::markAsLastBlock() {
    lastBlock = true ;
}

//



FrameProcessor::FrameProcessor(QObject *parent) : QObject(parent)
{
    m_bandwidth = 0 ;
    detection_threshold = 30 ;
    noise_floor = 0 ;
    m_state = next_state = FrameProcessor::sInit ;
    queueSampleCount = 0 ;
}

void FrameProcessor::setDetectionThreshold(float level) {
    detection_threshold = level ;
}



void FrameProcessor::raz() {
    flushQueue();
    next_state = sInit ;
}

QString FrameProcessor::stateToS(int s) {
    switch(s) {
    case sInit: return( QString("sInit"));
    case sMeasureNoise: return( QString("sMeasureNoise"));
    case sSearchFrame: return( QString("sSearchFrame"));
    case sFrameStart: return( QString("sFrameStart"));
    case sFrameEnds: return( QString("sFrameEnds"));
    }
    return("");
}

void FrameProcessor::newData( TYPECPX* IQsamples, int L , int sampleRate ) {
    int length ;
    float v ;
    TYPECPX* start = IQsamples ;
    int remaining_samples = L ;
    SampleBlock *sb ;

    while( remaining_samples > 0 ) {
        if( next_state != m_state ) {
            qDebug() << " state transition from " << stateToS(m_state) << " to " << stateToS(next_state);

            m_state = next_state ;
        }

        switch( m_state ) {
        case sInit:
            noise_floor = -1 ;
            rms_power = 0 ;
            next_state = sMeasureNoise ;
            samples_for_powerestimation = 2*sampleRate ; // use 2 seconds of signal
            continue ;

            // estimate noise level
        case sMeasureNoise:
            length = remaining_samples ;
            if( length > samples_for_powerestimation )
                length = samples_for_powerestimation ;
            rms_power += rmsp( IQsamples, length );
            samples_for_powerestimation -= length ;

            if( samples_for_powerestimation <= 0 ) {
                noise_floor = (float)20*log10(sqrt( 1.0/(2*sampleRate) * rms_power ));
                threshold = noise_floor + detection_threshold ;
                next_state = sSearchFrame ;
            }
            remaining_samples -= length ;
            break ;

        case sSearchFrame:
            if( remaining_samples >= PREAMBLE_LENGTH ) {
                v = rmsp( start, PREAMBLE_LENGTH, false ) ;

                emit powerLevel(v);
                if( v >= threshold ) {
                    // considered block contains enough energy (RMS(block) > (noise level + threshold)
                    // start to queue samples
                    queueSampleCount = PREAMBLE_LENGTH ;
                    SampleBlock *sb = new SampleBlock( start, PREAMBLE_LENGTH );
                    queue.enqueue( sb );
                    next_state = sFrameStart ;
                    emit frameDetected( v ) ;
                }
                start += PREAMBLE_LENGTH ;
                remaining_samples -= PREAMBLE_LENGTH ;
            } else {
                remaining_samples = 0 ; // todo : pushback the remaining samples in case frame starts here
            }
            break ;


        case sFrameStart:
            // test queue length
            length = queueSampleCount / DEMODULATOR_SAMPLERATE ; // int number of seconds in queue
            if( length > MAXSECONDS_IN_QUEUE ) {
                // not normal ? maybe level is wrong
                flushQueue();
                next_state = sMeasureNoise ;
                return ;
            }

            // put block in queue
            queueSampleCount+= remaining_samples ;
            sb = new SampleBlock( start, remaining_samples );
            queue.enqueue( sb );

            v = rmsp( start, remaining_samples, false ) ;
            emit powerLevel(v);
            remaining_samples = 0 ;

            qDebug() << "queueSampleCount=" << queueSampleCount ;


            // check what is the power level at the end of the block
            if( queueSampleCount > MINFRAME_LENGTH ) {
                start = IQsamples + L - PREAMBLE_LENGTH ;
                v = rmsp( start, PREAMBLE_LENGTH, false ) ;
                if( v < threshold ) {
                    sb->markAsLastBlock(); // this is the end of the sequence
                    next_state = sFrameEnds ;
                }
            }

            break ;

        case sFrameEnds:
            flushQueue();
            next_state = sSearchFrame ;
            break ;

        }
    }
}


float FrameProcessor::rmsp( TYPECPX *samples, int L, bool partial ) {
    float res = 0 ;
    int k = 0 ;
    for( k=0 ; k < L ; k++ ) {
        res += samples[k].re*samples[k].re + samples[k].im*samples[k].im ;
    }
    if( !partial)
        res = 20*log10(sqrtf( 1.0/L * res ));
    return(res);
}

void FrameProcessor::flushQueue() {
    while( !queue.isEmpty() ) {
        SampleBlock *b = queue.dequeue() ;
        delete b ;
    }
}

//Normalize to [-180,180):
inline double constrainAngle(double x){
    x = fmod(x + M_PI,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}
// convert to [-360,360]
inline double angleConv(double angle){
    return fmod(constrainAngle(angle),2*M_PI);
}
inline double angleDiff(double a,double b){
    double dif = fmod(b - a + M_PI,2*M_PI);
    if (dif < 0)
        dif += 2*M_PI;
    return dif - M_PI;
}

inline double unwrap(double previousAngle,double newAngle){
    return previousAngle - angleDiff(newAngle,angleConv(previousAngle));
}

