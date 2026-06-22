#ifndef __DUERWEN_ALSA_H_
#define __DUERWEN_ALSA_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <string.h>
#include <alloca.h>
#include <alsa/asoundlib.h>

typedef struct {
	snd_pcm_t *pcm_handle;
    snd_pcm_hw_params_t *hw_params;
    int channels;
    unsigned int sample_rate;
    char pcm_device[128];
    snd_pcm_format_t format;/// = SND_PCM_FORMAT_S16_LE;
    snd_pcm_uframes_t frames;// = BUFFER_SIZE;
} DuerWenAlsaParams;

/*
 * @description : alsa句柄初始化
 * @param - handle :alsa句柄
 * @param - devicename :声卡设备名
 * @param - channels :通道数
 * @param - rate :采样率
 * @param - mod :录/播  
 *      record  -- SND_PCM_STREAM_CAPTURE  
 *      aplay   -- SND_PCM_STREAM_PLAYBACK
 * @return : 0成功-1失败
*/
int duerwen_alsa_init(void **handle,char *devicename,unsigned int channels,unsigned int rate,unsigned int mod);

int duerwen_alsa_read(void *handle,short *buffer);
/*
 * @description : 往声卡设备送音频
 * @param - handle :alsa 声卡设备句柄
 * @param - buffer :音频
 * @return : 0成功-1失败
*/
int duerwen_alsa_write(void *handle,short *buffer);

int duerwen_alsa_clearcache(void *handle);

int duerwen_alsa_unit(void *handle);


#endif


