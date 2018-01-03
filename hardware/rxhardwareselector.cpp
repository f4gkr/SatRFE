#include "rxhardwareselector.h"
#include "common/constants.h"

RxHardwareSelector::RxHardwareSelector(QObject *parent) : QObject(parent)
{

}

RxDevice *RxHardwareSelector::getReceiver() {
    FUNCube *fcdboard = NULL ;
    RTLSDR *dongle = NULL ;

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
