#include "duerwen_alsa.h"

#define BUFFER_SIZE 1024

int duerwen_alsa_init(void **handle,char *devicename,unsigned int channels,unsigned int rate,unsigned int mod) ///8
{
	int err;
	*handle = NULL;//(DuerWenAlsaParams *)
	DuerWenAlsaParams *Params = (DuerWenAlsaParams *)malloc(sizeof(DuerWenAlsaParams));
	memset(Params->pcm_device,0x00,128);
	memcpy(Params->pcm_device,devicename,strlen(devicename));
	Params->sample_rate =rate;
	Params->channels = channels;
  Params->format = SND_PCM_FORMAT_S16_LE;
  Params->frames = BUFFER_SIZE;
    // 打开 PCM 设备
    err = snd_pcm_open(&Params->pcm_handle, Params->pcm_device, mod, 0);
    if (err < 0) {
        printf("无法打开 PCM 设备 %s: %s\n", Params->pcm_device, snd_strerror(err));
        return -1;
    }
    // 分配硬件参数结构体
    snd_pcm_hw_params_alloca(&Params->hw_params);
    snd_pcm_hw_params_any(Params->pcm_handle,Params->hw_params); // 设置任何参数  
    err = snd_pcm_hw_params_set_access(Params->pcm_handle,Params->hw_params,SND_PCM_ACCESS_MMAP_INTERLEAVED);
    if (err < 0) {
        printf("无法设置 PCM 访问类型: %s\n",snd_strerror(err));
        snd_pcm_close(Params->pcm_handle);
        return -1;
    }
    err = snd_pcm_hw_params_set_format(Params->pcm_handle,Params->hw_params,SND_PCM_FORMAT_S16_LE);
    if (err < 0) {
        printf("无法设置 PCM 数据格式: %s\n", snd_strerror(err));
        snd_pcm_close(Params->pcm_handle);
        return -1;
    }
    err = snd_pcm_hw_params_set_rate_near(Params->pcm_handle,Params->hw_params,&Params->sample_rate, 0);
    if (err < 0) {
        printf("无法设置 PCM 采样率: %s\n", snd_strerror(err));
        snd_pcm_close(Params->pcm_handle);
        return -1;
    }
    err = snd_pcm_hw_params_set_channels(Params->pcm_handle,Params->hw_params,Params->channels);
    if (err < 0) {
        printf("无法设置 PCM 通道数: %s\n", snd_strerror(err));
        snd_pcm_close(Params->pcm_handle);
        return -1;
    }
    err = snd_pcm_hw_params_set_period_size_near(Params->pcm_handle, Params->hw_params, &Params->frames, NULL);
    if(err < 0)
    {
    	printf("无法设置获取帧数为具有最接近目标的周期大小 (%s)\n", snd_strerror(err));
        snd_pcm_close(Params->pcm_handle);
        return -1;
    }
    printf("frames = %ld\n",Params->frames);
    
    err = snd_pcm_hw_params_get_period_size(Params->hw_params, &Params->frames, NULL);
    if(err < 0)
    {
    	printf("无法获取帧数 (%s)\n", snd_strerror(err));
        snd_pcm_close(Params->pcm_handle);
        return -1;
    }
    printf("frames = %ld\n",Params->frames);
    err = snd_pcm_hw_params(Params->pcm_handle,Params->hw_params); // 应用硬件参数
    if (err < 0) {
        printf("无法应用 PCM 硬件参数: %s\n", snd_strerror(err));
        snd_pcm_close(Params->pcm_handle);
        return -1;
    }
    // 准备 PCM 设备
    err = snd_pcm_prepare(Params->pcm_handle);
    if (err < 0) {
        printf("无法准备 PCM 设备: %s\n", snd_strerror(err));
        snd_pcm_close(Params->pcm_handle);
        return -1;
    }

    printf("PCM ALSA OK!!!\n");
    *handle = Params;
    return 0;
}

int duerwen_alsa_read(void *handle,short *buffer)
{
	DuerWenAlsaParams *Params = (DuerWenAlsaParams *)handle;
	if(Params==NULL) return -1;
  //printf("duerwen_alsa_read\n");
  int err = snd_pcm_mmap_readi(Params->pcm_handle, buffer,Params->frames);
	//printf("get read%d\n",err);
  if (err == -EPIPE){
        //缓冲区已耗尽，重新启动PCM设备
         printf("缓冲区已耗尽,重新启动PCM设备\n");
        snd_pcm_prepare(Params->pcm_handle);
        return -2;///continue;
  } else if (err < 0) {
        printf("录音错误: %s\n", snd_strerror(err));
        //break;
        return -3;
  }
  return err;
}

int duerwen_alsa_write(void *handle,short *buffer)
{
    DuerWenAlsaParams *Params = (DuerWenAlsaParams *)handle;
    if(Params==NULL) return -1;
    int err = snd_pcm_mmap_writei(Params->pcm_handle, buffer,Params->frames);
    if (err == -EPIPE){
            //缓冲区已耗尽，重新启动PCM设备
            printf("缓冲区已耗尽,重新启动PCM设备\n");
            snd_pcm_prepare(Params->pcm_handle);
            return -2;///continue;
    } else if (err < 0) {
            printf("播放错误: %s\n", snd_strerror(err));
            //break;
            return -3;
    }
    return err;

}

int duerwen_alsa_clearcache(void *handle)
{
    DuerWenAlsaParams *Params = (DuerWenAlsaParams *)handle;
    if(Params==NULL) return -1;

    int err = snd_pcm_drop(Params->pcm_handle);
	err = snd_pcm_prepare(Params->pcm_handle); 
    return err;

}

int duerwen_alsa_unit(void *handle)
{
	DuerWenAlsaParams *Params = (DuerWenAlsaParams *)handle;
	if(Params==NULL) return -1;
	snd_pcm_close(Params->pcm_handle);
	free(Params);
  Params=NULL;
  return 0;
}

