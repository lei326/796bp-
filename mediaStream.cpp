#include "doordvr_export.h"
#include "FramePackage.h"
#include "ua_dvr_video_define.h"
#include "doordvr_system_global.h"
#include "rk_mpi_venc.h"
#include "rk_mpi_mb.h"
static DVR_I32_T channel_stream[DVR_ALL_CH_NUM];
static pthread_mutex_t enc_lock[DVR_ALL_CH_NUM];
static DVR_U32_T stream_statistics_count[DVR_ALL_CH_NUM];
static long long stream_statistics_time[DVR_ALL_CH_NUM];
VENC_CHN_ATTR_S g_venc_expect_attr[DOORDVR_CHANNEL_NUM];
static void (*adas_event_call)(VI_CHN channel, FRAME_INFO_T *m_frameInfo,unsigned char**pdata);

int ADAS_RegEventCallback(void (*call)(VI_CHN channel, FRAME_INFO_T *m_frameInfo,unsigned char**pdata))
{
	printf("ADAS_RegEventCallback in streamserver.c, set adas_event_call = %p\n", call);
    adas_event_call = call;
    return 0;
}

void ADAS_OnEncodedFrame(VI_CHN channel, FRAME_INFO_T *frame, unsigned char **pdata)
{
    if (adas_event_call) {
        adas_event_call(channel, frame, pdata);
    }
}

int ADAS_UNRegEventCallback(void)
{
    adas_event_call = NULL;
    return 0;
}

DVR_I32_T DVR_CreateH264DecStream(DVR_U8_T chn)
{
    return Hqt_Venc_StartStream(chn);
}


/***********************************************************************************************************
**函数:
**功能:
**输入参数:
**返回值:
***********************************************************************************************************/
DVR_I32_T   DVR_CreateH264MediaStreamServer(DVR_H264_Profile_T *DTH264Profile)
{
#ifdef  CONFIG_STREAMSERVER
	int i = 0;

	bzero(&channel_stream,sizeof(DVR_I32_T)*DVR_ALL_CH_NUM);
	for (i=0; i<DVR_ALL_CH_NUM; i++)
    {
  	    pthread_mutex_init(&enc_lock[i], NULL);
    }

	for(i = 0; i < DOORDVR_CHANNEL_NUM; i++)
	{
		DVR_CreateH264DecStream(i);
	}
#ifdef SUPPORT_NET_STREAM	
	for(;i<DVR_ALL_CH_NUM;i++)
	{
		DVR_CreateH264DecStream(i);
	}
#endif		  
#endif
    return 0;
}
/***********************************************************************************************************
**函数:
**输入参数:
**功能:
**返回值:
***********************************************************************************************************/
DVR_I32_T   DVR_DestroyH264MediaStreamServer(void)
{
#ifdef  CONFIG_STREAMSERVER
	int i = 0;

#ifdef SUPPORT_NET_STREAM	
	for(;i < DVR_ALL_CH_NUM; i++)
	{
		pParam=ComGetPthreadParamAddr(PTHREAD_MAIN_IDENC1_ID+i);
	    ComExitThread(pParam);
	}
#endif		
	for (i=0; i<DVR_ALL_CH_NUM; i++)
    {
  	  pthread_mutex_destroy(&enc_lock[i]);
    }

	for(i = 0; i < DOORDVR_CHANNEL_NUM; i++)
	{
		Hqt_Venc_StopStream(i);
	}
#endif
    return 0;
}
/***********************************************************************************************************
**函数:
**输入参数:
**功能:
**返回值:
***********************************************************************************************************/
DVR_I32_T   DVR_PauseH264MediaStreamServer(void)
{
#ifdef  CONFIG_STREAMSERVER
    PTHREAD_PARAM_T *pParam;
    pParam=ComGetPthreadParamAddr(PTHREAD_MAIN_IDENC1_ID);
    ComPauseThread(pParam);
#endif
    return 0;
}
/***********************************************************************************************************
**函数:
**输入参数:
**功能:
**返回值:
***********************************************************************************************************/
DVR_I32_T   DVR_RunH264MediaStreamServer(void)
{
#ifdef  CONFIG_STREAMSERVER
    PTHREAD_PARAM_T *pParam;
    pParam=ComGetPthreadParamAddr(PTHREAD_MAIN_IDENC1_ID);
    ComRunThread(pParam);
#endif
    return 0;
}
/***********************************************************************************************************
**函数:
**输入参数:
**功能:
**返回值:
***********************************************************************************************************/
void AddGetChStream(DVR_U8_T ch)
{
    pthread_mutex_lock(&enc_lock[ch]);
    channel_stream[ch]=1;
    pthread_mutex_unlock(&enc_lock[ch]);
}
/***********************************************************************************************************
**函数:
**输入参数:
**功能:
**返回值:
***********************************************************************************************************/
void RemoveGetChStream(DVR_U8_T ch)
{
    pthread_mutex_lock(&enc_lock[ch]);
    channel_stream[ch]=0;
    pthread_mutex_unlock(&enc_lock[ch]);
}

void *DVR_H264_StreamServer(void *vpARG)
{
	printf("Enter VR_H264_StreamServer\n");
#ifdef  CONFIG_STREAMSERVER
	DVR_I32_T i=0;
	struct timeval TimeoutVal;

	DVR_I32_T s32ret;
	DVR_U32_T  dwFrameNumber = 0;

	DVR_I32_T VeChannel=*(DVR_I32_T*)vpARG ;
	VENC_STREAM_S stStream;
	VENC_PACK_S   *pack = NULL;
	DVR_U8_T FrameType;
	DVR_U32_T buf_size;
	DVR_I32_T maxfd = 0;
	long long  current_time;
	char encparam_count=0;
	long long osd_startTime=0,osd_endTime=0;
	PTHREAD_PARAM_T *pThreadParam ;
	int quit=0;;
  	int states=STATE_UNKNOW;
	FRAME_INFO_T frame_info;
	_H264_ENC_PARAM_T enc_coder_param,enc_param;
	//MEDIA_BUFFER mb = NULL;
	DVR_U32_T err_count = 0;
	if (vpARG == NULL)
  	{
    	printf(" DVR_H264_StreamServer  ---------error");
    	return NULL;
  	}

  	//printf("h264 stream server thread running........");
  	printf("start h264 stream %d server thread running........thread pid = %u",VeChannel,(unsigned int)syscall(224));

 	pThreadParam=ComGetPthreadParamAddr(PTHREAD_MAIN_IDENC1_ID + VeChannel);
  	AddGetChStream(VeChannel);
#ifdef CONFIG_OSD	
	Hqt_Osd_InitRegions(VeChannel);
  	SYS_Osd_Create(VeChannel);
#endif
	while(!quit)
	{
		switch (pThreadParam->states)
		{
			case STATE_PAUSED:
      {
        if (states!=STATE_PAUSED)
        {
          states=STATE_PAUSED;
        }
        printf("h264 stream server new state:STATE_PAUSED");
        ComMutexCond_Notice_Signal(pThreadParam);
        ComMutexCond_Wait(true,pThreadParam);
      }
      break;
      case STATE_STOPPED:
      {
        printf("preview thread  new state:STATE_STOPPED");
        ComMutexCond_Notice_Signal(pThreadParam);
        ComMutexCond_Wait(true,pThreadParam);
        break;
      }
      case STATE_INITIAL:
      {
        printf("preview thread new state:STATE_INITIAL");
        quit=1;
        break;
      }
      case STATE_RUNNING:
      {
        if (states!=STATE_RUNNING)
        {
          printf("h264 stream server  new state:STATE_RUNNING");
          states=STATE_RUNNING;
        }

        if (pThreadParam->states!=STATE_RUNNING)
          break;

        current_time=local_get_curtime();
        osd_endTime=current_time;
        if(osd_startTime==0)
        {
            osd_startTime=osd_endTime;
        }
		memset(&stStream, 0, sizeof(stStream));
        s32ret = RK_MPI_VENC_GetStream(VeChannel, &stStream, -1);
		pack = &stStream.pstPack[0];
        if (s32ret != RK_SUCCESS) {
            printf("RK_MPI_VENC_GetStream failed, chn = %d, ret = 0x%x\n",
                   VeChannel, s32ret);
            err_count++;
			if(err_count > 2)
			{
				DVR_SYSTEM_INFO_T *sysInfo;
				sysInfo = SystemGlobal_GetSystemGlobalContext();
                    //sysInfo->RbootfOrPowerDown = POWER_REBOOT;
                    sysInfo->RequireReStartVenc[VeChannel] = 1;
                    quit=1;
                    break;
			}
			continue;
		}
		else{
			err_count = 0;
		}
		bzero(&frame_info,sizeof(FRAME_INFO_T));
		frame_info.encoder_format = 0;//h264
		H264E_NALU_TYPE_E naluType = pack->DataType.enH264EType;

		if (naluType == H264E_NALU_ISLICE || naluType == H264E_NALU_IDRSLICE) {
    		frame_info.keyFrame  = 1;
    		frame_info.frameType = 0;  // I
		} else {
    		frame_info.keyFrame  = 0;
    		frame_info.frameType = 1;  // P/B
		}
		frame_info.channel=VeChannel%DOORDVR_CHANNEL_NUM;
		frame_info.frame_id = dwFrameNumber++;
		VENC_CHN_ATTR_S stChnAttr;
		memset(&stChnAttr, 0, sizeof(stChnAttr));
		RK_MPI_VENC_GetChnAttr(VeChannel, &stChnAttr);
    	frame_info.width  = stChnAttr.stVencAttr.u32PicWidth;
    	frame_info.height = stChnAttr.stVencAttr.u32PicHeight;
		frame_info.time = local_get_curtime();
		frame_info.relativeTime = pack->u64PTS;
		frame_info.length = pack->u32Len;
		static int cb_debug_cnt = 0;
		RK_U8 *base = (RK_U8 *)RK_MPI_MB_Handle2VirAddr(pack->pMbBlk);
		frame_info.pData  = base + pack->u32Offset;
		if (frame_info.length > 4 && frame_info.pData[3] != 1)
        {
          printf("----------------------------------------------------------ERROR----------------------------------------------------------------frame_info.pData[3]=%d",frame_info.pData[3]);
        }
		if(adas_event_call)
        {
			if (cb_debug_cnt % 100 == 0) {
        printf("DVR_H264_StreamServer: adas_event_call=%p\n", adas_event_call);
    	}
    		cb_debug_cnt++;
          (*adas_event_call)(frame_info.channel,&frame_info,&frame_info.pData);	
        }

		pthread_mutex_lock(&enc_lock[VeChannel]);

		if(VeChannel < DOORDVR_CHANNEL_NUM)
		{
			Hqt_Mpi_PutRecVideoFrame(VeChannel,RECVIDEO_STREAM_ID,&frame_info,VIDEO_IPCAM);/*放入录像缓冲*/
            Hqt_Mpi_PutNetMainVideoFrame(VeChannel%DOORDVR_CHANNEL_NUM,NETMAINVIDEO_STREAM_ID,&frame_info,VIDEO_IPCAM);/*放入网络主码流缓冲*/
            Hqt_Mpi_PutRecVideoFrame(VeChannel,PRERECVIDEO_STREAM_ID,&frame_info,VIDEO_IPCAM);/*放入预录像缓冲*/
		}
		else
		{
			Hqt_Mpi_PutNetSubVideoFrame(VeChannel%DOORDVR_CHANNEL_NUM,NETSUUBVIDEO_STREAM_ID,&frame_info,VIDEO_IPCAM);/*放入网络主码流缓冲*/
		}
		RK_MPI_VENC_ReleaseStream(VeChannel, &stStream);
		pthread_mutex_unlock(&enc_lock[VeChannel]);

		pthread_mutex_lock(DVR_GetEncoderMutexLock(VeChannel, DVR_STREAM_TYPE_H264));
        pthread_cond_broadcast(DVR_GetEncoderCondSignal(VeChannel, DVR_STREAM_TYPE_H264));
        pthread_mutex_unlock(DVR_GetEncoderMutexLock(VeChannel, DVR_STREAM_TYPE_H264));

		if (encparam_count >= 50) {
		VENC_CHN_ATTR_S curAttr;
    	memset(&curAttr, 0, sizeof(curAttr));
		RK_S32 ret = RK_MPI_VENC_GetChnAttr(VeChannel, &curAttr);
    	if (ret != RK_SUCCESS) {
        printf("RK_MPI_VENC_GetChnAttr failed, chn = %d, ret = 0x%x\n",VeChannel, ret);
    	}else{
			RK_BOOL Restart = RK_FALSE;
			if (curAttr.stVencAttr.u32PicWidth  != g_venc_expect_attr[VeChannel].stVencAttr.u32PicWidth ||
            curAttr.stVencAttr.u32PicHeight != g_venc_expect_attr[VeChannel].stVencAttr.u32PicHeight) {
            Restart = RK_TRUE;
        }
		if (Restart) {
            printf("venc param changed, restart encoder chn = %d\n", VeChannel);
            Hqt_Venc_CloseEncoder(VeChannel);
            Hqt_Venc_InitEncoder(VeChannel);
#ifdef CONFIG_OSD						
            if((enc_param.srcWidth!=enc_coder_param.srcWidth)||(enc_param.srcHeight!=enc_coder_param.srcHeight))
            {
        	   SYS_Osd_Destroy(VeChannel);
               SYS_Osd_Create(VeChannel);
            }
#endif		
			g_venc_expect_attr[VeChannel] = curAttr;
        }
		}
		encparam_count=0;
		}
		encparam_count++;
#ifdef CONFIG_OSD
        //osd
        if((osd_endTime>osd_startTime?(osd_endTime-osd_startTime):(osd_startTime-osd_endTime))>500000)//0.5 s
        {
            osd_startTime=osd_endTime;
            Sys_Osd_Update(VeChannel);
        }
#endif	

	}
	}
};
#ifdef CONFIG_OSD	
  SYS_Osd_Destroy(VeChannel);
#endif
	if(vpARG!=NULL)
	{
		free(vpARG);
		vpARG = NULL;
	}

	  printf("stop h264 stream %d server thread running........",VeChannel);
  pThreadParam->states=STATE_INITIAL;

#endif
  return NULL;
}
