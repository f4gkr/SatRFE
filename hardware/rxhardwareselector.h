#ifndef RXHARDWARESELECTOR_H
#define RXHARDWARESELECTOR_H

#include <QObject>
#include "hardware/rxdevice.h"
#include "hardware/funcube/funcube.h"
#include "hardware/rtlsdr.h"

class RxHardwareSelector : public QObject
{
    Q_OBJECT
public:

    explicit RxHardwareSelector(QObject *parent = 0);
    RxDevice *getReceiver() ;

signals:

public slots:
};

#endif // RXHARDWARESELECTOR_H
