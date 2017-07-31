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

#define LOGGER_NAME  "PicsatRFE"
#define LOGGER_FILENAME "picsatrfe.log"
#define VER_PRODUCTNAME_STR "picsatrfe"

#define SYMBOL_RATE 9600 /* picsat modem bitrate */
#define DEMODULATOR_SAMPLERATE (9600*4) /* baseband sample rate */
#define PREAMBLE_LENGTH (4*56)
#define BASEBAND_BLOCK_SIZE (SYMBOL_RATE)
#define MAXSECONDS_IN_QUEUE (10)
#define MINFRAME_LENGTH (4*1024)
#define RX_OFFSET (1e3) /* how far away from rx center we shift to avoid DC component */

#define CONFIG_FILENAME "picsatrfe.conf"
#endif // CONSTANTS_H
