 **SatRFE**
Simple radio frontend for custom processing - Receives samples from SDR device and extracts sub-band and finally sends IQ samples in float pairs via ZeroMQ

In current version the frame is (SymbolRate x Oversample ratio) wide. This is the **DEMODULATOR_SAMPLERATE** constant 
  
For example : 
>Symbol Rate is 9600 , Oversample Ratio = 4
we have a sampling rate of 38400 Hz
DEMODULATOR_SAMPLERATE = 38400
                
                
To avoid RX Dc at center and enable energy detction (power based or autocorrelation), we shift the subband of interest by FRAME_OFFSET_LOW Hz + DEMODULATOR_SAMPLERATE/2
        example : Offset set at 10 000 Hz 
>sub-band is centered at (FRAME_OFFSET_LOW Hz + DEMODULATOR_SAMPLERATE/2)
        =10 000 + 38400/2
        = 29200 Hz
        
Finally:
    if frequency of interst is f0 :
    - receiver is tuned to f0 -> we have a DC residual
    - we extract a subwindow (using Overalp/save channelizer) of (SymbolRate x Oversample ratio) Hz
    - This subband is not centered at f0, but at f0 - FFAME_OFFSET_LOW Hz + DEMODULATOR_SAMPLERATE/2
  
  example : 
    >- want to extract signal Symbol Rate is 9600 , Oversample Ratio = 4, centered at f = 436.500 MHz
    - DEMODULATOR_SAMPLERATE = 38400
    - (FRAME_OFFSET_LOW Hz + DEMODULATOR_SAMPLERATE/2)= 29200 Hz
    - Rx is tuned to f = 436.500 MHz - 29200 Hz ;
    - Channelizer (ddc) is centerd at +(FRAME_OFFSET_LOW Hz + DEMODULATOR_SAMPLERATE/2) = 29200
    - Received bandwidth is DEMODULATOR_SAMPLERATE = 38400


Required Qt Modules :
qt5-default
libqt5svg5-dev

Required librairies :
- libusb-1.0-0-dev 
- libgps-dev (connects to gpsd to retrieve time, otherwise on fail uses system time)
- libfftw3-dev
- libczmq-dev

clone the repository, then from the folder :
- generate makefile : 
   qmake
- compile
   make

- run
  ./picsatRF
  
  Current version only supports RTLSDR (FunCube to come)
  
# Configuration
on startup looks for existing (or creates a new one) configuration file called picsatrfe.conf in program dir
example
  

[Radio]
RX_FREQUENCY=436.4708

[WebServer]
port=8001

  # Webserver for remote control
Unless http port is set to 0 in config file, starts a http server listening by default on port 8001

supported requests are so far :
* http://<< ip >>:<< port >>/start
* http://<< ip >>:<< port >>/stop
* http://<< ip >>:<< port >>/status : returns a description of the current status (ongoing, not finished)
* http://<< ip >>:<< port >>/tune/<frequency in MHz> 
    tunes to desired freq
    for example : * http://<< ip >>:<< port >>/tune/438.256 to receive 438.256000 MHz
    
    
