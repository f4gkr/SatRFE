#ifndef REPEATERCHANNEL_H
#define REPEATERCHANNEL_H

#include <QObject>
#include <fftw3.h>
#include <QStateMachine>
#include <QState>
#include <QDateTime>
#include "common/datatypes.h"

class ActivityDetector : public QObject
{
    Q_OBJECT
public:
    explicit ActivityDetector();
    void processSamples(TYPECPX *in, int len ) ;

signals:
    void active();
    void inactive();

public slots:

private slots:
    void channelOn();
    void channelOff();

private:

    int w_pos ;

    fftwf_complex *fftin ;
    fftwf_complex *buffer ;
    fftwf_plan plan, plan_rev ;

    float modulus(int i);
    float A,B,C;
    QStateMachine *channelMachine ;
    QState *cmOn ;
    QState *cmOff ;
    QState *cmPending;
    QDateTime _last ;
};

#endif // REPEATERCHANNEL_H
