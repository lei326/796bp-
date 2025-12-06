#include "ms_netdvr.h"
#include "doordvr_export.h"
#include <doordvr_config.h>
#include <doordvr_getsetconfig.h>
//#include <ms_netdvr_common_tool.h>
#include "customer.h"
#include <doordvr_system_global.h>
#include <FrameDataManage.h>
#include <ms_netdvr_common_tool.h>
#include <backboardmanage.h>
//#include <record_alarm_io.h>
#include "doordvr_getsetconfig.h"
#include "ms_netdvr_vi_interface.h"
#include "FramePackage.h"
#include "ms_playback_vdec.h"

static void InitVo(void);
static void InitVi(void);
static int EncoderParamTrans(const EncoderParam_t* recorde_param, ENCODER_PARAM_S* encoder_param);
//static void FormatStateCB(DISK_FORMATING_STATE_E state);
//static void DiskRunStateCB(DISK_RUN_STATE_E state);
static int GpsDataCb(VENC_GPS_OSD_S* pstGpsData);

static int8_t i8FormatState = 0;	//0等待 1成功 -1失败

void InitViVo(void)
{
    MSLOG_DEBUG("start vio ... !\n");
	InitVo();
	InitVi();

//#if defined(MSDVR588)
	//ShowLive(L_NINE_PICTURE, NULL);
	//DVR_U32_T normal_display;
	//DVR_GetConfig(oper_normal_display_type, (DVR_U8_T*)&normal_display, 0);
	//ShowLive((1L << 31) | normal_display, NULL);
//#endif

	MSLOG_DEBUG("start vio ok !\n");

}
/*
void InitManageDisk(void)
{
	DISK_FORMAT_MODE_CONFIG_S config = { 0,1000,0 };
	MsNetdvr_Disk_ManageInit(config);
	MsNetdvr_Disk_SetDiskRunStateCallback(DiskRunStateCB);
}
*/
void InitSystemMediaEncoder(void)
{
	AllConfigDef_t* pallconfig = SystemGlobal_GetAllconfigDefContext();
	CAM_CHN_VENC_ATTR_S stVencAttr[DOORDVR_CHANNEL_NUM];
	EncoderParam_t* recorde_param = pallconfig->encoderset.recorde_param;
	memset(stVencAttr, 0, sizeof(stVencAttr));

	for (uint8_t i = 0; i < DOORDVR_CHANNEL_NUM; i++)
	{
		stVencAttr[i].u8Chn = i;
		//初始化主码流设置
		EncoderParamTrans(&recorde_param[i], &stVencAttr[i].stEncoderParam[0]);
		//初始化子码流设置
		EncoderParamTrans(&recorde_param[i + DOORDVR_CHANNEL_NUM], &stVencAttr[i].stEncoderParam[1]);
	}

	MsNetdvr_Venc_ManageInit(DOORDVR_CHANNEL_NUM, stVencAttr);
	UpdateChStringOsd();
	UpdateBasicParam();
	UpdateGpsParam();
}

void CreateSystemMediaEncoder(DVR_U8_T ch)
{
	MsNetdvr_Venc_Start(ch);
}


void DestroySystemMediaEncoder(DVR_U8_T ch)
{
	MsNetdvr_Venc_Stop(ch);
}
/*
void StartRecordSchedule(void)
{
	RecordPlan_t recordplan;	//录像计划
	RecodCtrol_t    recordctrol;	//录像控制

	DVR_GetConfig(oper_recordset_type, (uint8_t*)&recordplan, DOORDVR_CHANNEL_NUM);
	DVR_GetConfig(oper_recordctrol_type, (uint8_t*)&recordctrol, DOORDVR_CHANNEL_NUM);

	RECORD_PLAN_CHN_ATTR_S* pstChnAttr = new RECORD_PLAN_CHN_ATTR_S[DOORDVR_CHANNEL_NUM];

	for (uint8_t i = 0; i < DOORDVR_CHANNEL_NUM; i++)
	{
		pstChnAttr[i].u8Chn = i;
		pstChnAttr[i].enRecordctrol = (RECORD_CONTROL_TYPE_E)recordctrol.recordctrol[i];
		memcpy(&pstChnAttr[i].stRecordWeekPlan, &recordplan.record_week_plan[i], sizeof(RECORD_WEEK_PLAN_S));
	}

	//MsNetdvr_Record_ManageInit(sizeof(pstChnAttr) / sizeof(RECORD_PLAN_CHN_ATTR_S), pstChnAttr);
	//RegisterIOAlarm();
	BasicParam_t  plate_name;
	CommonSet_t     commonparam;
	RECORD_VEHICLE_INFO_S stVehicleInfo;
	DVR_GetConfig(oper_basicparam_type, (uint8_t*)(&plate_name), DOORDVR_CHANNEL_NUM);
	DVR_GetConfig(oper_commonset_type, (uint8_t*)&commonparam, DOORDVR_CHANNEL_NUM);
	MsNetdvr_Record_UpdateVideoFormat(commonparam.video_format == SYSTEM_PAL ? VIDEO_PAL: VIDEO_NTSC);
	memcpy(stVehicleInfo.szVehicleNum, plate_name.VehicleNum, sizeof(stVehicleInfo.szVehicleNum));
	memset(stVehicleInfo.szPhoneNum, 0, sizeof(stVehicleInfo.szPhoneNum));
	memcpy(stVehicleInfo.szLicenseNum, plate_name.LicenseNum, sizeof(stVehicleInfo.szLicenseNum));
	MsNetdvr_Record_UpdateVehicleInfo(&stVehicleInfo);
	MsNetdvr_Record_ManageStart();
	if (pstChnAttr)
	{
		delete[] pstChnAttr;
	}
}
*/
void InitAudio(void)
{
	AIO_CHN_ATTR_S ai_attr = { 0 };
	AIO_CHN_ATTR_S ao_attr = { 0 };

	ai_attr.pcAudioNode = (char*)"hw:1,0";
	ai_attr.u32BitWidth = 16;
	ai_attr.u32Channels = 8;
	ai_attr.u32SampleRate = 16000;
	ai_attr.u32NbSamples = (SAMPLE_AUDIO_PTNUMPERFRM*8);

	ao_attr.pcAudioNode = (char*)"hw:0,0";
	ao_attr.u32BitWidth = 16;
	ao_attr.u32Channels = 2;
	ao_attr.u32SampleRate = 8000;
	ao_attr.u32NbSamples = SAMPLE_AUDIO_PTNUMPERFRM*2;

	MsNetdvr_Aio_ManageInit(&ai_attr, &ao_attr, AUDIO_DECODER_TYPE);
}

int UpdateEncoderParam(EncoderSet_t* EncoderSet)
{
	ENCODER_PARAM_S param;
	int ret = 0;
	for (int i = 0; i < DOORDVR_CHANNEL_NUM; i++)
	{
		EncoderParamTrans(&EncoderSet->recorde_param[i], &param);
		ret |= MsNetdvr_Venc_UpdateEncoder(i, true, &param);
	}
	return ret;
}
/*
int UpdateRecordCtrParma(RecodCtrol_t* recordctrol_change)
{
	int ret = 0;
	for (int i = 0; i < DOORDVR_CHANNEL_NUM; i++)
	{
		if (recordctrol_change->recordctrol[i] < RECORD_CONTROL_BUTT)
		{
			ret |= MsNetdvr_Record_UpdateRecordCtrl(i, (RECORD_CONTROL_TYPE_E)recordctrol_change->recordctrol[i]);
		}
		else
		{
			ret = -1;
			MSLOG_ERROR("RecordCtrl param error! chn(%d)=%d", i, recordctrol_change->recordctrol[i]);
		}
	}
	return ret;
}

int UpdateRecordSetParam(RecordSet_t* recordSet_change)
{
	int ret = 0;
	for (int i = 0; i < DOORDVR_CHANNEL_NUM; i++)
	{
		ret |= MsNetdvr_Record_UpdateRecordPlan(i,
			(const RECORD_WEEK_PLAN_S*)&recordSet_change->record_paln.record_week_plan[i]);
	}
	return ret;
}
*/
int UpdateBasicParam(BasicParam_t* basicParam_change)
{
	BasicParam_t basicParam;
	int ret = 0;
	if (basicParam_change == nullptr)
	{
		DVR_GetConfig(oper_basicparam_type, (DVR_U8_T*)(void*)(&basicParam), DOORDVR_CHANNEL_NUM);
	}
	else
	{
		basicParam = *basicParam_change;
	}
	for (int i = 0; i < DOORDVR_CHANNEL_NUM; i++)
	{
		ret |= MsNetdvr_Venc_SetOsdPlateNumber(i, basicParam.LicenseNum,
			strlen(basicParam.LicenseNum));
	}
	return ret;
}

int UpdateChStringOsd(LocalDisplay_T* display)
{
	LocalDisplay_T ch_name;
	int ret = 0;
	if (display == nullptr)
	{
		DVR_GetConfig(oper_localdisplay, (DVR_U8_T*)(void*)(&ch_name), DOORDVR_CHANNEL_NUM);
	}
	else
	{
		ch_name = *display;
	}

	for (int i = 0; i < DOORDVR_CHANNEL_NUM; i++)
	{
		ret |= MsNetdvr_Venc_SetOsdChannelName(i, (const char*)ch_name.channel_name[i],
			strlen((const char*)ch_name.channel_name[i]));
	}
	return ret;
}

int UpdateGpsParam()
{
	int ret = 0;
	for (int i = 0; i < DOORDVR_CHANNEL_NUM; i++)
	{
		ret |= MsNetdvr_Venc_SetOsdGps(i, VENC_OSD_BD, GpsDataCb, nullptr);
	}
	return 0;
}
/*
bool ManageDisk_Format(uint8_t disk_num)
{
	DISK_FORMAT_MODE_CONFIG_S config;
	config.u8ManageDiskMode = 1;
	config.u32DiskFileDuration = 500;
	int ret = MsNetdvr_Disk_Format(disk_num, config, FormatStateCB);
	if (!ret)
	{
		i8FormatState = 0;
		while (!i8FormatState)
		{
			usleep(100 * 1000);
		}
	}
	else
	{
		return false;
	}

	return i8FormatState == 1;
}
*/
void InitVo(void)
{
    MSLOG_DEBUG("start init vo ... !\n");
	Outputadjust_t   outputadjust;
	CommonSet_t     commonparam;

	VO_PARAM_S stVoParam[] = {
		{0, 1280,720, 30, DISPLAY_TYPE_LCD},
		//{1, 1280, 800, 30, DISPLAY_TYPE_MIPI},
	};
	if (ReadLocalParam() < 0)
	{
		MSLOG_ERROR("ReadLocalParam fail\n");
		exit(-1);
	}
	DVR_GetConfig(oper_commonset_type, (DVR_U8_T*)&commonparam, DOORDVR_CHANNEL_NUM);
	DVR_GetConfig(oper_advanceset_outputadjust_type, (DVR_U8_T*)&outputadjust, DOORDVR_CHANNEL_NUM);

	switch (outputadjust.vga_resolution_ratio)
	{
	case 0:
		stVoParam[0].u32DispWidth = 800;
		stVoParam[0].u32DispHeight = 600;
		break;
	case 1:
		stVoParam[0].u32DispWidth = 1280;
		stVoParam[0].u32DispHeight = 720;
		break;
	case 2:
		stVoParam[0].u32DispWidth = 1920;
		stVoParam[0].u32DispHeight = 1080;
		break;
	}

	if (commonparam.video_format == 0)
	{
		stVoParam[0].u32DispFrmRt = 30;
	}
	else
	{
		stVoParam[0].u32DispFrmRt = 25;
	}
	MsNetdvr_Vo_ManageInit(sizeof(stVoParam) / sizeof(VO_PARAM_S), stVoParam);
	MsNetdvr_Vo_ManageStart();
}

void InitVi(void)
{
    MSLOG_DEBUG("start init vi ... !\n");
	EncoderSet_t    encoderset;
	CommonSet_t commonset;

	DVR_GetConfig(oper_encoderset_type, (DVR_U8_T*)&encoderset, DVR_ALL_CH_NUM);
	DVR_GetConfig(oper_commonset_type, (DVR_U8_T*)&commonset, DVR_ALL_CH_NUM);
//#if defined(MSDVR588)
	VIDEO_CHN_ATTR_S pstVideoAttr[DOORDVR_CHANNEL_NUM] = {
		{VIDEO_TYPE_ANALOG, 0, {"/dev/video11", {1280,720}, FMT_YUV420SP, false, false, 25}},
		{VIDEO_TYPE_ANALOG, 1, {"/dev/video12", {1280,720}, FMT_YUV420SP, false, false, 25}},
		{VIDEO_TYPE_ANALOG, 2, {"/dev/video13", {1280,720}, FMT_YUV420SP, false, false, 25}},
		{VIDEO_TYPE_ANALOG, 3, {"/dev/video14", {1280,720}, FMT_YUV420SP, false, false, 25}},

		{VIDEO_TYPE_ANALOG, 4, {"/dev/video0", {1280,720}, FMT_YUV420SP, false, false, 25}},
		{VIDEO_TYPE_ANALOG, 5, {"/dev/video1", {1280,720}, FMT_YUV420SP, false, false, 25}},
		//{VIDEO_TYPE_ANALOG, 6, {"/dev/video2", {1280,720}, FMT_YUV420SP, false, false, 25}},
		//{VIDEO_TYPE_ANALOG, 7, {"/dev/video3", {1280,720}, FMT_YUV420SP, false, false, 25}},
	};

	if (encoderset.recorde_param[0].encoder_size == 5)
	{
		for (int i = 0; i < 4; i++)
		{
			pstVideoAttr[i].VideoAttr.stAttrVi.stSize.u32Width = 1920;
			pstVideoAttr[i].VideoAttr.stAttrVi.stSize.u32Height = 1080;
		}
	}

	for (int i = 0; i < DOORDVR_CHANNEL_NUM; i++)
	{
		if (encoderset.recorde_param[i].mirror == 1 && encoderset.recorde_param[i].flip == 1)
		{
			pstVideoAttr[i].VideoAttr.stAttrVi.bMirror = true;
			pstVideoAttr[i].VideoAttr.stAttrVi.bFlip = true;
		}
		else if (encoderset.recorde_param[i].mirror == 1)
		{
			pstVideoAttr[i].VideoAttr.stAttrVi.bMirror = true;
		}
		else if (encoderset.recorde_param[i].flip == 1)
		{
			pstVideoAttr[i].VideoAttr.stAttrVi.bFlip = true;
		}
	}

	if (commonset.video_format == SYSTEM_NTSC)
	{
		for (int i = 0; i < DOORDVR_CHANNEL_NUM; i++)
		{
			pstVideoAttr[i].VideoAttr.stAttrVi.u32FrameRate = 30;
			pstVideoAttr[i].VideoAttr.stAttrVi.u32FrameRate = 30;
		}
		MSLOG_ERROR("ReadLocalParam VIDEO FORMAT SYSTEM_NTSC\n");
	}else{

		MSLOG_ERROR("ReadLocalParam VIDEO FORMAT SYSTEM_PAL\n");

	}
//#endif



	MsNetdvr_Vi_ManageInit((uint8_t)(sizeof(pstVideoAttr) / sizeof(VIDEO_CHN_ATTR_S)), pstVideoAttr);
	MsNetdvr_Vi_ManageStart();
}

int EncoderParamTrans(const EncoderParam_t* recorde_param, ENCODER_PARAM_S* encoder_param)
{
	encoder_param->stTimeOverlyPos.u32Pos_x = recorde_param->timeoverly_pos.pos_x;
	encoder_param->stTimeOverlyPos.u32Pos_y = recorde_param->timeoverly_pos.pos_y;
	encoder_param->stStringOverlyPos.u32Pos_x = recorde_param->stringoverly_pos.pos_x;
	encoder_param->stStringOverlyPos.u32Pos_y = recorde_param->stringoverly_pos.pos_y;
	encoder_param->stGpsOverlyPos.u32Pos_x = recorde_param->gpsoverly_pos.pos_x;
	encoder_param->stGpsOverlyPos.u32Pos_y = recorde_param->gpsoverly_pos.pos_y;
	encoder_param->stPlateOverlyPos.u32Pos_x = recorde_param->plateoverly_pos.pos_x;
	encoder_param->stPlateOverlyPos.u32Pos_y = recorde_param->plateoverly_pos.pos_y;
	encoder_param->enEncoderSize = (ENCODER_SIZE_E)recorde_param->encoder_size;
	encoder_param->enBitstreameControl = (ENCODER_RC_MODE_E)recorde_param->bitstreame_control;
	//stVencAttr[i].stEncoderParam[0].u32BitstreameQuality = recorde_param[i].bitstreame_quality;
	Hqt_Common_BitStreamSizeToValue(true, (ENCODER_SIZE_E)recorde_param->encoder_size, (tagBitStreamSize)recorde_param->bitstreame_size, &encoder_param->u32BitstreameSize);
	encoder_param->u8FrameRate = recorde_param->framerate;
	encoder_param->u8EnableTimeOverly = recorde_param->enable_time_overly;
	encoder_param->u8EnableStringOverly = recorde_param->enable_string_overly;
	encoder_param->u8EnableGpsOverly = recorde_param->enable_gps_overly;
	encoder_param->u8EnablePlateOverly = recorde_param->enable_plate_overly;
	encoder_param->enEncoderFormat = (MS_VENC_MODTYPE_E)recorde_param->encoder_format;
	encoder_param->u8EnableEncoder = 1;
	encoder_param->u8EncoderType = recorde_param->encType;
    //MSLOG_DEBUG("recorde_param->enable_string_overly:%d  encoder_param->u8EnableStringOverly:%d ",
	//	         recorde_param->enable_string_overly,encoder_param->u8EnableStringOverly );
	//MSLOG_DEBUG("recorde_param->enable_time_overly:%d  encoder_param->u8EnableTimeOverly:%d ",
	//	         recorde_param->enable_time_overly,encoder_param->u8EnableTimeOverly );
	//MSLOG_DEBUG("recorde_param->enable_gps_overly:%d  encoder_param->u8EnableGpsOverly:%d ",
	//	         recorde_param->enable_gps_overly,encoder_param->u8EnableGpsOverly );
	return 0;
}
/*
void FormatStateCB(DISK_FORMATING_STATE_E state)
{
	if (state == DISK_FORMAT_ERROR_STATE)
	{
		i8FormatState = -1;
	}
	else if (state == DISK_FORMAT_SUCCESS_STATE)
	{
		i8FormatState = 1;
	}
}

void DiskRunStateCB(DISK_RUN_STATE_E state)
{
	MSLOG_DEBUG("Disk state change %d\n", state);
	if (DISK_ERROR_STATE == state)
	{
		MSLOG_ERROR("format err 2 times: POWER_REBOOT ");
		if (SystemGlobal_GetSystemCurrentState() == DVR_STATE_RUNNING)
		{
			DVR_SYSTEM_INFO_T* sysInfo = SystemGlobal_GetSystemGlobalContext();
			sysInfo->RbootfOrPowerDown = POWER_REBOOT;
		}
		else
		{
			SystemGlobal_Reboot();
		}
	}
}
*/
int GpsDataCb(VENC_GPS_OSD_S* pstGpsData)
{
	JGYDFIM_MODE gpsinfo;
	memset(&gpsinfo, 0, sizeof(JGYDFIM_MODE));
	if (GetBackBoardInfo(&gpsinfo) < 0)
	{
		memset(pstGpsData, 0, sizeof(VENC_GPS_OSD_S));
		return -1;
	}

	pstGpsData->ydcGpsStatus = gpsinfo.ydgps.ydcGpsStatus;
	pstGpsData->ydusSpeed = gpsinfo.ydgps.ydusSpeed;
	pstGpsData->ydcLatitudeDegree = gpsinfo.ydgps.ydcLatitudeDegree;
	pstGpsData->ydcLatitudeCent = gpsinfo.ydgps.ydcLatitudeCent;
	pstGpsData->ydcLongitudeDegree = gpsinfo.ydgps.ydcLongitudeDegree;
	pstGpsData->ydcLongitudeCent = gpsinfo.ydgps.ydcLongitudeCent;
	pstGpsData->ydlLatitudeSec = gpsinfo.ydgps.ydlLatitudeSec;
	pstGpsData->ydlLongitudeSec = gpsinfo.ydgps.ydlLongitudeSec;
	pstGpsData->ydcDirectionLatitude = gpsinfo.ydgps.ydcDirectionLatitude;
	pstGpsData->ydcDirectionLongitude = gpsinfo.ydgps.ydcDirectionLongitude;
	pstGpsData->ydfractionlen = gpsinfo.ydgps.ydfractionlen;
	
	return 0;
}




static int MainstreamEncoderDataCallback(FRAME_INFO_EX* frame_buf, void* pUsrData)
{
	FRAME_INFO_T frame_info;
	////if(frame_buf->channel == 0){
	//  MSLOG_DEBUG("receive channel(%ld),type(%ld),encode(%d),length(%ld)\n", frame_buf->channel, frame_buf->frameType, frame_buf->encoder_format,frame_buf->length);
	//}
	if (frame_buf->channel >= DEVICE_CHANNEL_NUM) return 0;


	bzero(&frame_info,sizeof(FRAME_INFO_T));
	frame_info.encoder_format = frame_buf->encoder_format;//h264
	frame_info.keyFrame = frame_buf->keyFrame;
	frame_info.channel = frame_buf->channel;
    frame_info.frameType = frame_buf->frameType;
	frame_info.frame_id=frame_buf->frame_id;

	frame_info.width= frame_buf->width;
	frame_info.height=  frame_buf->height;
	frame_info.time = frame_buf->time;
	frame_info.relativeTime = frame_buf->relativeTime;
	frame_info.length = frame_buf->length;
	frame_info.pData = frame_buf->pData;
	unsigned char *pdata = frame_info.pData;
	ADAS_OnEncodedFrame((VI_CHN)frame_info.channel, &frame_info, &pdata);
	Hqt_Mpi_PutRecVideoFrame(frame_info.channel,RECVIDEO_STREAM_ID,&frame_info,VIDEO_IPCAM);/*放入录像缓冲*/
	Hqt_Mpi_PutNetMainVideoFrame(frame_info.channel,NETMAINVIDEO_STREAM_ID,&frame_info,VIDEO_IPCAM);/*放入网络主码流缓冲*/		
	Hqt_Mpi_PutNetMainVideoFrame(frame_info.channel,NETSUUBVIDEO_STREAM_ID,&frame_info,VIDEO_IPCAM);/*放入网络子码流缓冲*/				
	Hqt_Mpi_PutRecVideoFrame(frame_info.channel,PRERECVIDEO_STREAM_ID,&frame_info,VIDEO_IPCAM);/*放入预录像缓冲*/
	pthread_mutex_lock(DVR_GetEncoderMutexLock(frame_info.channel, DVR_STREAM_TYPE_H264));
	pthread_cond_broadcast(DVR_GetEncoderCondSignal(frame_info.channel, DVR_STREAM_TYPE_H264));
	pthread_mutex_unlock(DVR_GetEncoderMutexLock(frame_info.channel, DVR_STREAM_TYPE_H264));



	return 0;
}

static int MainstreamEncoderDataCallbackWithADAS(FRAME_INFO_EX* frame_buf, void* pUsrData)
{
    int ret = MainstreamEncoderDataCallback(frame_buf, pUsrData);

    if (!frame_buf || !frame_buf->pData) {
        return ret;
    }
    FRAME_INFO_T frame;
    memset(&frame, 0, sizeof(frame));

    frame.keyFrame       = frame_buf->keyFrame;
    frame.frameType      = frame_buf->frameType;
    frame.length         = frame_buf->length;
    frame.width          = frame_buf->width;
    frame.height         = frame_buf->height;
    frame.channel        = frame_buf->channel;
    frame.frame_id       = frame_buf->frame_id;
    frame.encoder_format = frame_buf->encoder_format;
    frame.time           = frame_buf->time;
    frame.relativeTime   = frame_buf->relativeTime;
    frame.pData          = frame_buf->pData;
     frame.frameIndex  = frame_buf->frameIndex;
     frame.frameAttrib = frame_buf->frameAttrib;
     frame.streamID    = frame_buf->streamID;

    unsigned char *pdata = frame.pData;

    ADAS_OnEncodedFrame((VI_CHN)frame.channel, &frame, &pdata);

    return ret;
}


int MainStream_Venc_StartStream(DVR_U8_T ch)
{
    std::shared_ptr<CMsVideoInputBase> pVideoInput = nullptr;
	pVideoInput = MsNetdvr_Vi_GetVideoInput(ch);
	if (pVideoInput)
	{
		//设置编码数据回调函数
		pVideoInput->RegisterMainEncoderDataCallback(MainstreamEncoderDataCallback, nullptr);
		pVideoInput->RegisterAudioEncoderDataCallback(MainstreamEncoderDataCallback, nullptr);

	}else{
		
	  MSLOG_DEBUG("channel %d start stream failed !\n",ch);
      return -1;
	}


    return 0;
}

int MainStream_Venc_StopStream(DVR_U8_T ch)
{
    std::shared_ptr<CMsVideoInputBase> pVideoInput = nullptr;
	pVideoInput = MsNetdvr_Vi_GetVideoInput(ch);
	if (pVideoInput)
	{
		//设置编码数据回调函数
		pVideoInput->UnRegisterMainEncoderDataCallback(MainstreamEncoderDataCallback, nullptr);
		pVideoInput->UnRegisterAudioEncoderDataCallback(MainstreamEncoderDataCallback, nullptr);

	}else{
		
	  MSLOG_DEBUG("channel %d start stream failed !\n",ch);
      return -1;
	}


    return 0;
}

int MsNetdvr_Playback_SendEncoderData(int chn, int frame_len, char* frame_buf, bool bEndOfFrame)
{
	FRAME_INFO_EX frame_info;
	frame_info.channel = chn;
	frame_info.length = frame_len;
	frame_info.pData = (unsigned char*)frame_buf;

	return CMsPLaybackVideoDec::Instance()->VideoEncoderDataSendVdec(&frame_info, bEndOfFrame);
}





