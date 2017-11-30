# SatRFE
Simple radio frontend for custom processing

Required Qt Modules :
qt5-default
libqt5svg5-dev

Required librairies :
- libusb-1.0-0-dev 
- libgps-dev (connects to gpsd to retrieve time, otherwise on fail uses system time)
- libfftw3-dev

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
  
[Interface]
FIFO_FILENAME=/tmp/test

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
    
    
