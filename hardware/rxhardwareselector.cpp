#include "rxhardwareselector.h"
#include "common/constants.h"
#include "hardware/funcube/funcube.h"
#include "hardware/rtlsdr.h"
#include "hardware/miricsCpp.h"

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

    fcdboard = new FUNCube();
    if( fcdboard->getDeviceCount() > 0 ) {
        return(fcdboard);
    }

    dongle = new RTLSDR(0);
    if( dongle->getDeviceCount() > 0 ) {
        if( dongle->setRxSampleRate( SYMBOL_RATE * 200 ) < 0 ) { // sampling rate is 1.92 MHz
            delete dongle ;
            return(NULL);
        }
    }
    return(dongle);
}
