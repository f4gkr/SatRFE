#include "rxhardwareselector.h"
#include "common/constants.h"
#include "hardware/funcube/funcube.h"
#include "hardware/rtlsdr.h"
#include "hardware/miricscpp.h"

RxHardwareSelector::RxHardwareSelector(QObject *parent) : QObject(parent)
{

}

RxDevice *RxHardwareSelector::getReceiver() {
    FUNCube *fcdboard = NULL ;
    RTLSDR *dongle = NULL ;
    MiricsSDR *rsp = NULL ;

    rsp = new MiricsSDR(0);
    if( rsp->getDeviceCount() > 0 ) {
        return( rsp );
    }
    delete rsp ;
    rsp = NULL ;

    fcdboard = new FUNCube();
    if( fcdboard->getDeviceCount() > 0 ) {
        return(fcdboard);
    }
    delete fcdboard ;
    fcdboard = NULL ;

    dongle = new RTLSDR(0);
    if( dongle->getDeviceCount() > 0 ) {
        if( dongle->setRxSampleRate( SYMBOL_RATE * 200 ) < 0 ) { // sampling rate is 1.92 MHz            
            return(dongle);
        }
    }
    delete dongle ;
    dongle = NULL ;
    return(NULL);
}
