#ifndef __ALIGENIE_FIFO_H__
#define __ALIGENIE_FIFO_H__
#include <pthread.h>

typedef struct
{
    unsigned char    *buffer;
    unsigned int     size;
    unsigned int     in;
    unsigned int     out;
    unsigned int     outback;
    pthread_mutex_t  lock;
} RecvDataCacheInfo;

void RecvDataCacheInit(RecvDataCacheInfo *fifo,unsigned char *buffer, unsigned int size );
void RecvDataCacheFree(RecvDataCacheInfo *fifo);
unsigned int RecvDataCachLen(const RecvDataCacheInfo *fifo);//RecvDataCachLen
unsigned int RecvDataCacheAviable(const RecvDataCacheInfo *fifo);
unsigned int RecvDataCacheGet(RecvDataCacheInfo *fifo, unsigned char *buffer, unsigned int size);
unsigned int RecvDataCacheGetReadonly(RecvDataCacheInfo *fifo, unsigned char *buffer, unsigned int size);
unsigned int RecvDataCachePut(RecvDataCacheInfo *fifo, unsigned char *buffer, unsigned int size);
void RecvDataCacheReset(RecvDataCacheInfo *fifo);

///extern RecvDataCacheInfo CacheRecvDataFFlag;

#endif


