#include "RecvDataCache.h"
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <stdarg.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <signal.h>
#include <sys/prctl.h>
#include <stdbool.h>
#include <fcntl.h>
#include <stdint.h>
#include <time.h>
//#include "logging.h"

#ifndef min
#define min(a, b) ((a < b) ? a : b)
#endif

void RecvDataCacheInit(RecvDataCacheInfo *fifo,unsigned char *buffer, unsigned int size)
{
    fifo->buffer = buffer;
    fifo->size = size;
    fifo->in = 0;
    fifo->out = 0;
    ///pthread_mutex_init(&fifo->lock, NULL);
}

void RecvDataCacheFree(RecvDataCacheInfo *fifo)
{
    if(fifo)
    {
        if (fifo->buffer)
        {
            free(fifo->buffer);
            fifo->buffer = NULL;
        }
        //free(fifo);
        fifo = NULL;
    }
}

unsigned int __RecvDataCachLen(const RecvDataCacheInfo *fifo)
{
    unsigned int len = 0;
    //pthread_mutex_lock(&fifo->lock);
    len =fifo->in - fifo->out;
    //pthread_mutex_unlock(&fifo->lock);
    return len;
}

unsigned int __RecvDataCacheAviable(const RecvDataCacheInfo *fifo)
{
    unsigned int len = 0;
    ///pthread_mutex_lock(&fifo->lock);
    len =fifo->in - fifo->out;
    len = fifo->size - len;
    ///pthread_mutex_unlock(&fifo->lock);;
    return len;

}

unsigned int __RecvDataCacheGet(RecvDataCacheInfo *fifo, unsigned char* buffer, unsigned int size)
{
    unsigned int len = 0;
    unsigned int remain_Len = 0;
    ///pthread_mutex_lock(&fifo->lock);
    size  = min(size, fifo->in - fifo->out);
    /* first get the data from fifo->out until the end of the buffer */
    remain_Len = fifo->out % fifo->size;
    //len = min(size, fifo->size - (fifo->out & (fifo->size - 1)));
    len = min(size, fifo->size - remain_Len);
    // memcpy(buffer, fifo->buffer + (fifo->out & (fifo->size - 1)), len);
    memcpy(buffer, fifo->buffer + remain_Len, len);
    /* then get the rest (if any) from the beginning of the buffer */
    memcpy(buffer + len, fifo->buffer, size - len);
    fifo->out += size;
    if (fifo->in == fifo->out)
        fifo->in = fifo->out = 0;
    ///pthread_mutex_unlock(&fifo->lock);
    return size;
}

unsigned int __RecvDataCacheGetReadonly(RecvDataCacheInfo *fifo, unsigned char* buffer, unsigned int size)
{
    unsigned int len = 0;
    unsigned int remain_Len = 0;
    if(fifo == NULL)
    {
	   printf("fifo is null\n");
    }
    ///pthread_mutex_lock(&fifo->lock);
    size  = min(size, fifo->in - fifo->out);
    /* first get the data from fifo->out until the end of the buffer */
    remain_Len = fifo->out % fifo->size;
    //len = min(size, fifo->size - (fifo->out & (fifo->size - 1)));
    len = min(size, fifo->size - remain_Len);
    // memcpy(buffer, fifo->buffer + (fifo->out & (fifo->size - 1)), len);
   // printf("00fifo->buffer:%p,buffer:%p,size:%d,len:%d,remain_size:%d\n",fifo->buffer,buffer,size,len,remain_Len);
    memcpy(buffer, fifo->buffer + remain_Len, len);
    /* then get the rest (if any) from the beginning of the buffer */
    memcpy(buffer + len, fifo->buffer, size - len);
    //fifo->out += size;
    //printf("11fifo->buffer:%p,buffer:%p,size:%d,len:%d,remain_size:%d\n",fifo->buffer,buffer,size,len,remain_Len);
    ///pthread_mutex_unlock(&fifo->lock);
    return size;
}

unsigned int __RecvDataCachePut(RecvDataCacheInfo *fifo, unsigned char *buffer, unsigned int size)
{
    unsigned int len = 0;
    unsigned int remain_Len = 0;
    ///pthread_mutex_lock(&fifo->lock);
    size = min(size, fifo->size - fifo->in + fifo->out);
    /* first put the data starting from fifo->in to buffer end */
    remain_Len = fifo->in % fifo->size;
    // len  = min(size, ring_buf->size - (ring_buf->in & (ring_buf->size - 1)));
    len  = min(size, fifo->size - remain_Len);
    // memcpy(ring_buf->buffer + (ring_buf->in & (ring_buf->size - 1)), buffer, len);
    memcpy(fifo->buffer + remain_Len, buffer, len);
    /* then put the rest (if any) at the beginning of the buffer */
    memcpy(fifo->buffer, buffer + len, size - len);
    fifo->in += size;
    ///pthread_mutex_unlock(&fifo->lock);
    return size;
}

unsigned int RecvDataCachLen(const RecvDataCacheInfo *fifo)
{
    unsigned int len = 0;
    len = __RecvDataCachLen(fifo);
    return len;
}

unsigned int RecvDataCacheAviable(const RecvDataCacheInfo *fifo)
{
    unsigned int len = 0;
    len = __RecvDataCacheAviable(fifo);
    return len;

}

unsigned int RecvDataCacheGet(RecvDataCacheInfo *fifo, unsigned char *buffer, unsigned int size)
{
    unsigned int ret;
    ret = __RecvDataCacheGet(fifo, buffer, size);
    return ret;
}

unsigned int RecvDataCacheGetReadonly(RecvDataCacheInfo *fifo, unsigned char *buffer, unsigned int size)
{
    unsigned int ret;
    ret = __RecvDataCacheGetReadonly(fifo, buffer, size);
    {
		if (fifo->in == fifo->out)
		{
			//printf("fifo->in = fifo->out\n");
			//fifo->in = fifo->out = 0;
		}
    }
    return ret;
}

unsigned int RecvDataCachePut(RecvDataCacheInfo *fifo, unsigned char *buffer, unsigned int size)
{
    unsigned int ret;
    ret = __RecvDataCachePut(fifo, buffer, size);
    return ret;
}

void RecvDataCacheReset(RecvDataCacheInfo *fifo)
{
    //pthread_mutex_lock(&fifo->lock);
    fifo->in = fifo->out = 0;
    //pthread_mutex_unlock(&fifo->lock); 
}





