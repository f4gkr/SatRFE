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
#include <stdio.h>
#include "common/constants.h"
#include "frametodecoder.h"

#define FRAME_DEBUG (1)

//-- Class SampleBlock
FrameProcessor::FrameProcessor(QObject *parent) : QObject(parent)
{
    m_bandwidth = 0 ;
    detection_threshold = DEFAULT_DETECTION_THRESHOLD ;
    noise_floor = 0 ;
    m_state = next_state = FrameProcessor::sInit ;
    queueSampleCount = 0 ;
    m_detectorMethod = sUseRmsPower ;
    adt = new ActivityDetector();
}

float FrameProcessor::setDetectionThreshold(float level) {
    detection_threshold = level ;
    threshold = noise_floor + detection_threshold ;
    if( FRAME_DEBUG ) qDebug() << " noise level was " << noise_floor ;
    if( FRAME_DEBUG ) qDebug() << " threshold is now " << threshold ;
    return( threshold );
}



void FrameProcessor::raz() {
    flushQueue(0);
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

int FrameProcessor::newData( TYPECPX* IQsamples, int L , int sampleRate ) {
    if( m_detectorMethod == sUseRmsPower ) {
        return( processDataRMS(IQsamples, L, sampleRate));
    }
    return( processDataAD(IQsamples, L, sampleRate));
}

int FrameProcessor::processDataAD( TYPECPX* IQsamples, int L , int sampleRate ) {
    Q_UNUSED(sampleRate ) ;
    adt->processSamples( IQsamples, L );
    return(0);
}

int FrameProcessor::processDataRMS( TYPECPX* IQsamples, int L , int sampleRate ) {
    int length ;
    float v ;
    TYPECPX* start = IQsamples ;
    int remaining_samples = L ;
    SampleBlock *sb ;

    //qDebug() << "FrameProcessor::newData()" << L ;

    while( remaining_samples > 0 ) {
        if( next_state != m_state ) {
            if( FRAME_DEBUG )  qDebug() << " state transition from " << stateToS(m_state) << " to " << stateToS(next_state);
            m_state = next_state ;
            emit newState( stateToS(m_state));
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
            if( remaining_samples >= PREAMBLE_LENGTH ) {
                samples_for_powerestimation -= PREAMBLE_LENGTH ;
                if( samples_for_powerestimation > 0 ) {
                    v = rmsp( start, PREAMBLE_LENGTH ) ;
                    emit powerLevel(v);
                    rms_power += v ;
                    start += PREAMBLE_LENGTH ;
                    remaining_samples -= PREAMBLE_LENGTH ;

                } else {
                    rms_power /= (2*sampleRate/PREAMBLE_LENGTH) ; // how many times we accumulated power
                    noise_floor = rms_power ;
                    threshold = noise_floor + detection_threshold ;
                    next_state = sSearchFrame ;

                    if( FRAME_DEBUG )  qDebug() << " noise level is void SLOT_DeleteWriter( FrameToDecoder *writer );set to " << noise_floor ;
                    if( FRAME_DEBUG )  qDebug() << " threshold is " << threshold ;

                    emit newSNRThreshold(threshold);
                }
            } else {
                return(remaining_samples);
            }
            break ;

        case sSearchFrame:
            if( remaining_samples >= PREAMBLE_LENGTH ) {
                v = rmsp( start, PREAMBLE_LENGTH ) ;
                //qDebug() << v << threshold ;
                emit powerLevel(v);
                if( v >= threshold ) {
                    // considered block contains enough energy (RMS(block) > (noise level + threshold)
                    // start to queue samples

                    if( (L-remaining_samples) > 0 ) {
                        TYPECPX* cstart = start ;
                        cstart -= (L-remaining_samples) ;
                        SampleBlock *sb = new SampleBlock( cstart, (L-remaining_samples) + PREAMBLE_LENGTH );
                        queue.enqueue( sb );
                        queueSampleCount = (L-remaining_samples) + PREAMBLE_LENGTH ;

                    } else {
                        SampleBlock *sb = new SampleBlock( start, PREAMBLE_LENGTH );
                        queue.enqueue( sb );
                        queueSampleCount = PREAMBLE_LENGTH ;
                    }
                    next_state = sFrameStart ;
                    emit frameDetected( v ) ;
                    start += PREAMBLE_LENGTH ;
                    remaining_samples -= PREAMBLE_LENGTH ;

                } else {
                    start += PREAMBLE_LENGTH/2 ;
                    remaining_samples -= PREAMBLE_LENGTH/2 ;
                }
            } else {
                return(remaining_samples);
            }
            break ;


        case sFrameStart:
            // test queue length
            length = queueSampleCount / DEMODULATOR_SAMPLERATE ; // int number of seconds in queue
            if( length > MAXSECONDS_IN_QUEUE ) {
                // not normal ? maybe level is wrong
                flushQueue(queueSampleCount);
                next_state = sMeasureNoise ;
                return(0) ;
            }

            // put block in queue
            queueSampleCount+= remaining_samples ;
            sb = new SampleBlock( start, remaining_samples );
            queue.enqueue( sb );

            v = rmsp( start, remaining_samples ) ;
            emit powerLevel(v);
            remaining_samples = 0 ;
            // check what is the power level at the end of the block
            if( queueSampleCount > MINFRAME_LENGTH ) {
                start = IQsamples + L - PREAMBLE_LENGTH ;
                v = rmsp( start, PREAMBLE_LENGTH ) ;
                if( v < threshold ) {
                    sb->markAsLastBlock(); // this is the end of the sequence
                    next_state = sFrameEnds ;
                }
            }

            break ;

        case sFrameEnds:
            flushQueue(queueSampleCount);
            queueSampleCount = 0 ;
            next_state = sSearchFrame ;
            break ;

        }
    }
    return(0) ;
}



float FrameProcessor::rmsp( TYPECPX *samples, int L ) {
    float res = 0 ;
    int k = 0 ;
    for( k=0 ; k < L ; k++ ) {
        res += samples[k].re*samples[k].re + samples[k].im*samples[k].im ;
    }
    res = 20*log10(sqrtf( 1.0/L * res ));
    return(res);
}

void FrameProcessor::SLOT_DeleteWriter( FrameToDecoder *writer ) {
    qDebug() << " deleting writer " ;
    writer->deleteLater();
}

void FrameProcessor::flushQueue(int L) {
    int saved_count = 0 ;


    if( L == 0 ) {
        while( !queue.isEmpty() ) {
            SampleBlock *b = queue.dequeue() ;
            bool waslast = b->isLastBlock() ;
            delete b ;
            if( waslast ) {
                return ;
            }
        }
        return ;
    }

    if( FrameToDecoder::onWrite() ) {
        // busy
        qDebug() << "Cannot write to file now, busy from previous operation" ;
        return ;
    }

    FrameToDecoder* fichier = new FrameToDecoder();
    connect( fichier, SIGNAL(fileWritten(FrameToDecoder*)), this, SLOT(SLOT_DeleteWriter(FrameToDecoder*)),     Qt::QueuedConnection );

    while( !queue.isEmpty() && (saved_count<L)) {
        SampleBlock *b = queue.dequeue() ;
        fichier->addBlock(b);
        if( b->isLastBlock() )
            break ;
    }
    fichier->start();

}


