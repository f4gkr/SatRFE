#include "frametodecoder.h"
#include "common/constants.h"

QSemaphore *FrameToDecoder::synchro = NULL ;
int FrameToDecoder::instanceCount = 0 ;

FrameToDecoder::FrameToDecoder(QObject *parent) : QThread(parent)
{
    L = 0 ;
}

void FrameToDecoder::addBlock( SampleBlock *b ) {
    if( b == NULL )
        return ;
    queue.enqueue(b);
    L += b->getLength() ;
}


void FrameToDecoder::run() {
    int saved_count = 0 ;
    int block_count = 0 ;
    size_t bytes = 0 ;
    char filename[255] ;
    FILE* data ;

    synchro->acquire(1);
    instanceCount++ ;

    sprintf( filename, FIFO_FILENAME);
    data = fopen( filename, "wb");
    bytes = fwrite(  &L, sizeof(int),1, data);

    while( !queue.isEmpty() && (saved_count<L)) {
        SampleBlock *b = queue.dequeue() ;
        TYPECPX *samples = b->getData() ;
        saved_count += b->getLength() ;
        bytes += fwrite( samples, 1, b->getLength()*sizeof(TYPECPX), data);
        delete b ;
        block_count++ ;
    }
    fclose( data );

    qDebug() << "----------------------------------------------------" ;
    qDebug() << "file  :" << QString::fromLocal8Bit(filename );
    qDebug() << "saved " << saved_count << " samples, size=" << saved_count*sizeof(TYPECPX) << bytes ;
    qDebug() << "blocks :" << block_count ;

    instanceCount-- ;

    synchro->release(1);
    emit fileWritten(this);
}
