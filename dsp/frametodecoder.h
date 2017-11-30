#ifndef FRAMETODECODER_H
#define FRAMETODECODER_H

#include <QObject>
#include <QThread>
#include <QSemaphore>

#include "core/sampleblock.h"
class FrameToDecoder : public QThread
{
    Q_OBJECT
public:
    explicit FrameToDecoder(QObject *parent = 0);

    void addBlock( SampleBlock *b );
    void run();

    static QSemaphore *synchro ;
    static int instanceCount ;

    static bool onWrite() {
        bool result = false ;
        synchro->acquire(1);
        result = instanceCount > 0 ;
        synchro->release(1);
        return( result );
    }

signals:
    void fileWritten( FrameToDecoder *writer );

public slots:
private:
    QQueue<SampleBlock*> queue ;
    int L ;
};

#endif // FRAMETODECODER_H
