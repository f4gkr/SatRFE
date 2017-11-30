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
#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <QObject>

#define LOGGER_NAME  "PicsatRFE"
#define LOGGER_FILENAME "picsatrfe.log"
#define VER_PRODUCTNAME_STR "picsatrfe"
/**
  In current version the frame is 100 KHz wide
  we sample 110 Khz. To avoid RX Dc at center, RX windows is 10 K above rx center
  example : RTLSDR rx center at 430.000
  window starts at 430.010
                center at 430.065
                   ends at 430.120
  */
#define FRAME_OFFSET_LOW 10e3

#define FIFO_FILENAME "/tmp/test"

#define SYMBOL_RATE 9600 /* picsat modem bitrate */
#define OVERSAMPLE_RATIO (4)
#define DEMODULATOR_SAMPLERATE (9600*OVERSAMPLE_RATIO) /* baseband sample rate */
#define PREAMBLE_LENGTH (OVERSAMPLE_RATIO*10*8)
#define BASEBAND_BLOCK_SIZE (SYMBOL_RATE)
#define MAXSECONDS_IN_QUEUE (10)
#define MINFRAME_LENGTH (OVERSAMPLE_RATIO*1024)
#define RX_OFFSET (1e3) /* how far away from rx center we shift to avoid DC component */

#define CONFIG_FILENAME "picsatrfe.conf"
#define DEFAULT_RX_FREQUENCY (436.4708*1e6)

class GlobalConfig : public QObject
{
    Q_OBJECT
public:
    static GlobalConfig& getInstance()  {
        static GlobalConfig instance;
        return instance;
    }

    QString cFIFO_FileName ;
    qint64 cRX_FREQUENCY ;

private:
    GlobalConfig();

    GlobalConfig(const GlobalConfig &); // hide copy constructor
    GlobalConfig& operator=(const GlobalConfig &);

};

#endif // CONSTANTS_H
