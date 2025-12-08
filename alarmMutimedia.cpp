#include <stdio.h>
#include <string.h>
#include <time.h>

#include "doordvr_export.h"
#include "doordvr_jpeg.h"
#include "doordvr_schedule.h"
#include "device.h"
#include "FramePackage.h"
#include "doordvr_typesdefine.h"
#include "snap.h"
#include "MP4Encoder.h"
// #include "file.h"
#include "doordvr_algo_subiao.h"
#include "mspublic.h"
#include "ms_netdvr_common_tool.h"
#define ALGO_MUTIMEDIA_SAVE_PATH "/userdata/ADAS_Alarm"
#include <pthread.h>

#define ATTACH_MAX_NUM 5
#define SNAP_MAX_NUM 3

static pthread_mutex_t g_attachLock = PTHREAD_MUTEX_INITIALIZER;

static DVR_U32_T g_cachedAlarmTime = 0;

static uint8_t g_snapCount = 0;
static char g_snapPaths[SNAP_MAX_NUM][128] = {{0}};
static char g_mp4Path[128] = {0};

static bool g_snapReady = false;
static bool g_mp4Ready = false;

static void ResetAttachCache(AlarmMutimedia *self, DVR_U32_T alarmtime)
{
    g_cachedAlarmTime = alarmtime;

    g_snapCount = 0;
    memset(g_snapPaths, 0, sizeof(g_snapPaths));
    memset(g_mp4Path, 0, sizeof(g_mp4Path));
    g_snapReady = false;
    g_mp4Ready = false;

    if (self)
    {
        self->m_event_alarm.attachmentnum = 0;
        for (int i = 0; i < ATTACH_MAX_NUM; ++i)
        {
            memset(self->m_event_alarm.attachfile[i], 0, sizeof(self->m_event_alarm.attachfile[i]));
        }
    }
}

static void UpdateBaseAlarmFields_NoClear(AlarmMutimedia *self, AiAlarmItem *param)
{
    if (!self || !param)
        return;

    self->m_event_alarm.devicetype = param->devicetype;
    self->m_event_alarm.alarmtime = param->alarmtime;
    self->m_event_alarm.alarmchannel = param->alarmchannel;
    self->m_event_alarm.alarmlevel = param->alarmlevel;
    self->m_event_alarm.alarmstatus = param->alarmstatus;
    self->m_event_alarm.alarmtype = param->alarmtype;
    self->m_event_alarm.fatigueDegree = param->fatigueDegree;
}

static void BuildCombinedAttachList(AlarmMutimedia *self)
{
    if (!self)
        return;

    self->m_event_alarm.attachmentnum = 0;
    for (int i = 0; i < ATTACH_MAX_NUM; ++i)
    {
        memset(self->m_event_alarm.attachfile[i], 0, sizeof(self->m_event_alarm.attachfile[i]));
    }

    int idx = 0;

    for (int i = 0; i < (int)g_snapCount && idx < ATTACH_MAX_NUM; ++i)
    {
        strncpy(self->m_event_alarm.attachfile[idx], g_snapPaths[i],
                sizeof(self->m_event_alarm.attachfile[idx]) - 1);
        self->m_event_alarm.attachfile[idx][sizeof(self->m_event_alarm.attachfile[idx]) - 1] = '\0';
        idx++;
    }

    if (g_mp4Path[0] != '\0' && idx < ATTACH_MAX_NUM)
    {
        strncpy(self->m_event_alarm.attachfile[idx], g_mp4Path,
                sizeof(self->m_event_alarm.attachfile[idx]) - 1);
        self->m_event_alarm.attachfile[idx][sizeof(self->m_event_alarm.attachfile[idx]) - 1] = '\0';
        idx++;
    }

    self->m_event_alarm.attachmentnum = (uint8_t)idx;
}

AlarmMutimedia *AlarmMutimedia::m_pInstance = NULL;

int (*algoEventCallback)(AiAlarmItem *item, void *args) = NULL;
static void *algoEventCallbackargs = NULL;
MP4Encoder *MP4Encoder::m_pInstance = NULL;

MP4Encoder::MP4Encoder(void) : m_videoId(NULL),
                               m_nWidth(0),
                               m_nHeight(0),
                               m_nTimeScale(0),
                               m_nFrameRate(0)
{
}

void MP4Encoder::CloseMP4File()
{
    if (m_hMp4File)
    {
        MP4Close(m_hMp4File);
        m_hMp4File = NULL;
    }
}

int MP4Encoder::Mp4WriteOneFrame(char *pBytes, uint32_t numBytes)
{
    char *pBuf = pBytes;
    pBuf[0] = ((numBytes - 4) >> 24) & 0xFF;
    pBuf[1] = ((numBytes - 4) >> 16) & 0xFF;
    pBuf[2] = ((numBytes - 4) >> 8) & 0xFF;
    pBuf[3] = ((numBytes - 4) >> 0) & 0xFF;
    return MP4WriteSample(m_hMp4File, m_videoId, (const uint8_t *)(pBuf), numBytes, MP4_INVALID_DURATION, 0, 1);
}

int MP4Encoder::Mp4WriteH264IFrame(char *pBytes, uint32_t numBytes)
{
    bool ret = false;
    uint32_t uDealLen = 0;
    int uNaluLen = 0;

    char *pCurBuf = pBytes;
    unsigned char naluType;

    while (uDealLen < numBytes)
    {
        char *pNalu;
        // printf("uNaluLen=%d,numBytes=%d,uDealLen=%d ",uNaluLen,numBytes,uDealLen);
        uNaluLen = getOneNalu(pCurBuf, numBytes - uDealLen, &pNalu);
        // printf("uNaluLen=%d,numBytes=%d, uDealLen=%d",uNaluLen,numBytes,uDealLen);

        if (uNaluLen <= 0)
            break;
        pCurBuf += uNaluLen;
        uDealLen += uDealLen;

        naluType = pNalu[4] & 0x1F;

        switch (naluType)
        {
        case 7: // SPS
            ret = Mp4WriteOneFrame(pNalu, uNaluLen);
            break;
        case 8: // PPS
            ret = Mp4WriteOneFrame(pNalu, uNaluLen);
            uint32_t dataLen = pBytes + numBytes - pNalu - uNaluLen;
            ret = Mp4WriteOneFrame(pNalu + uNaluLen, dataLen);
            return ret;
        }
    }
    return ret;
}

int MP4Encoder::Mp4WriteH265IFrame(char *pBytes, uint32_t numBytes)
{
    bool ret = true;
    uint32_t uNaluLen = 0, uDealLen = 0;
    char *pCurBuf = pBytes;
    unsigned char naluType;

    while (uDealLen < numBytes)
    {
        char *pNalu;
        uNaluLen = getOneNalu(pCurBuf, numBytes - uDealLen, &pNalu);
        if (uNaluLen <= 0)
            break;
        pCurBuf += uNaluLen;
        uDealLen += uDealLen;

        naluType = (pNalu[4] & 0x7E) >> 1;

        switch (naluType)
        {
        case 32: // VPS
            ret = Mp4WriteOneFrame(pNalu, uNaluLen);
            break;
        case 33: // SPS
            ret = Mp4WriteOneFrame(pNalu, uNaluLen);
            break;
        case 34: // PPS
            ret = Mp4WriteOneFrame(pNalu, uNaluLen);
            uint32_t dataLen = pBytes + numBytes - pNalu - uNaluLen;
            ret = Mp4WriteOneFrame(pNalu + uNaluLen, dataLen);
            return ret;
        }
    }
    return ret;
}

MP4Encoder *MP4Encoder::Instance(void)
{
    if (m_pInstance == NULL)
    {
        m_pInstance = new MP4Encoder;
    }
    return m_pInstance;
}

MP4Encoder::~MP4Encoder(void)
{
    if (m_pInstance != NULL)
    {
        delete m_pInstance;
    }
}

AlarmMutimedia::AlarmMutimedia(void)
{
    m_encodeOnceFlag = true;

    m_haveRecordFlag = false;

    memset(&m_event_alarm, 0, sizeof(AiAlarmItem));
    snprintf(m_acH264Path, sizeof(m_acH264Path), "%s", ALGO_MUTIMEDIA_SAVE_PATH);

    printf("AlarmMutimedia::AlarmMutimedia: m_acH264Path=[%s]\n", m_acH264Path);
}

bool AlarmMutimedia::GetHaveRecordFlag()
{
    return m_haveRecordFlag;
}

void AlarmMutimedia::SetHaveRecoedFlag(bool flag)
{
    m_haveRecordFlag = flag;
}

int RegisterEventCb(int (*cbFunc)(AiAlarmItem *item, void *args), void *cbArgs)
{
    algoEventCallback = cbFunc;
    algoEventCallbackargs = cbArgs;
    return 0;
}

int UNRegisterEventCb(void)
{
    algoEventCallback = NULL;
    algoEventCallbackargs = NULL;
    return 0;
}

int MP4Encoder::getOneNalu(char *pFrameBuf, uint32_t numBytes, char **pNalu)
{
    unsigned char c;
    int pos = 0;
    int beg = 0;
    int len;

    if (!pFrameBuf)
    {
        printf(" ");
        return -1;
    }

    if (!(*pNalu))
    {
        printf(" ");
        return -1;
    }

    if (numBytes < 4)
    {
        printf("numBytes=%d", numBytes);
        return -1;
    }

    while (beg < numBytes - 4)
    {
        if (pFrameBuf[beg + 0] == 0 && pFrameBuf[beg + 1] == 0 && pFrameBuf[beg + 2] == 0 && pFrameBuf[beg + 3] == 1)
            break;
        beg++;
    }

    // printf("beg=%d",beg);

    if (beg == numBytes - 4)
    {
        printf("beg=%d, numBytes=%d", beg, numBytes);
        return -1;
    }

    *pNalu = pFrameBuf + beg;
    pos = beg + 4;

    while (pos < numBytes - 4)
    {
        if (pFrameBuf[pos + 0] == 0 && pFrameBuf[pos + 1] == 0 && pFrameBuf[pos + 2] == 0 && pFrameBuf[pos + 3] == 1)
        {
            break;
        }
        pos++;
    }

    if (pos == numBytes - 4)
    {
        len = numBytes - beg;
    }
    else
    {
        len = pos - beg;
    }
    // printf(" return len=%d",len);

    return len;
}

MP4FileHandle MP4Encoder::CreateMP4File(const char *pFileName, int width, int height, int timeScale /* = 90000*/, int frameRate /* = 25*/)
{
    if (pFileName == NULL)
    {
        return;
    }
    // create mp4 file
    m_hMp4File = MP4Create(pFileName);
    if (m_hMp4File == MP4_INVALID_FILE_HANDLE)
    {
        printf("ERROR:Open file fialed.\n");
        return MP4_INVALID_FILE_HANDLE;
    }
    m_nWidth = width;
    m_nHeight = height;
    m_nTimeScale = 90000;
    m_nFrameRate = frameRate;
    MP4SetTimeScale(m_hMp4File, m_nTimeScale);
    return m_hMp4File;
}

int GetSystemTimeFromSecond(time_t Second_cnt, datetime_t *date_time)
{
    tm *p;
    time_t timep = Second_cnt;
    p = gmtime(&timep);
    printf("p->tm_year=%d", p->tm_year);
    // date_time->year = 1900 + p->tm_year;
    date_time->year = (1900 + p->tm_year) % 1000;
    printf("date_time->year =%d", date_time->year);
    date_time->month = 1 + p->tm_mon;
    date_time->day = p->tm_mday;
    date_time->hour = p->tm_hour;
    date_time->minute = p->tm_min;
    date_time->second = p->tm_sec;

    return 0;
}

MP4FileHandle MP4Encoder::getMp4FileHander()
{
    return m_hMp4File;
}

MP4TrackId MP4Encoder::getMp4VideoId()
{
    return m_videoId;
}

MP4TrackId MP4Encoder::getMp4AudioId()
{
    return m_audioId;
}

void MP4Encoder::setMp4VideoId(MP4TrackId videoId)
{
    m_videoId = videoId;
}

void MP4Encoder::setMp4AudioId(MP4TrackId audioId)
{
    m_audioId = audioId;
}

int MP4Encoder::getChannelId()
{
    return m_channel_Id;
}

void MP4Encoder::setChannelId(int channelId)
{
    m_channel_Id = channelId;
}

AlarmMutimedia *AlarmMutimedia::Instance(void)
{
    if (m_pInstance == NULL)
    {
        m_pInstance = new AlarmMutimedia;
    }
    return m_pInstance;
}

AlarmMutimedia::~AlarmMutimedia(void)
{
    if (m_pInstance != NULL)
    {
        delete m_pInstance;
    }
}

int AlarmMutimedia::MakeDirectory(char *_pcPath, int _iMode)
{
    int iRet = 0;
    unsigned int iLocalIndex = 0;
    char cTmpPath[256];
    struct stat TDevState;

    if (_pcPath == NULL)
    {
        printf("[%s] Illegal param!\n", __func__);
        return -1;
    }

    if (_pcPath[iLocalIndex] == '/')
    {
        printf("[%s] absolute path, skip the first\n", __func__);
        cTmpPath[iLocalIndex] = _pcPath[iLocalIndex];
        iLocalIndex++;
    }

    while (iLocalIndex < sizeof(cTmpPath))
    {
        cTmpPath[iLocalIndex] = _pcPath[iLocalIndex];
        if (cTmpPath[iLocalIndex] == '\0')
        {
            printf("[%s] reaching the end, the string %s is over\n", __func__, cTmpPath);
            break;
        }

        if (cTmpPath[iLocalIndex] == '/')
        {
            cTmpPath[iLocalIndex] = '\0';

            iRet = stat(cTmpPath, &TDevState);
            if (iRet != 0)
            {
                iRet = mkdir(cTmpPath, _iMode);
                printf("[%s] creating directory %s, iRet = %d\n", __func__, cTmpPath, iRet);
            }

            cTmpPath[iLocalIndex] = '/';
        }

        iLocalIndex++;
    }

    iRet = stat(cTmpPath, &TDevState);
    if (iRet != 0)
    {
        iRet = mkdir(cTmpPath, _iMode);
        if (iRet != 0)
        {
            perror("mkdir .......");
        }
        printf("[%s] creating directory %s, iRet = %d\n", __func__, cTmpPath, iRet);
    }

    return iRet;
}

int32_t AlarmMutimedia::CreateMediaDir(char *dir)
{
    struct stat devstat;

    if (0 == stat(dir, &devstat))
    {
        if (!S_ISDIR(devstat.st_mode))
        {
            printf("1111111111111111111111111111, dir[%s]", dir);
            unlink(dir);
            printf("2222222222222222222222222222, dir[%s]", dir);
            if (0 != MakeDirectory(dir, S_IRWXU | S_IXGRP | S_IXOTH))
            {
                printf("create %s failed!\n", dir);
                return -1;
            }
        }
    }
    else
    {
        if (0 != MakeDirectory(dir, S_IRWXU | S_IRWXG | S_IRWXO))
        {
            printf("create %s failed!\n", dir);
            return -2;
        }
    }

    return 0;
}

int AlarmMutimedia::Doordvr_Snap_StartJpeg(VI_CHN channel, PIC_SIZE_E picSize, AiAlarmItem *param)
{
    (void)picSize;

    if (param == NULL)
    {
        printf("Doordvr_Snap_StartJpeg: param is NULL\n");
        return -1;
    }

    if (param->alarmtime == 0)
    {
        param->alarmtime = (DVR_U32_T)time(NULL);
    }

    pthread_mutex_lock(&g_attachLock);

    if (g_cachedAlarmTime == 0 || g_cachedAlarmTime != param->alarmtime)
    {
        ResetAttachCache(this, param->alarmtime);
    }

    UpdateBaseAlarmFields_NoClear(this, param);

    pthread_mutex_unlock(&g_attachLock);

    int successCount = 0;

    for (int i = 0; i < 3; ++i)
    {
        int ret = SnapOneChannel(channel);
        if (ret != 0)
        {
            printf("Doordvr_Snap_StartJpeg: SnapOneChannel failed, ch=%d, idx=%d, ret=%d\n", channel, i, ret);
            continue;
        }

        const char *path = SnapGetLastFilePath(channel);
        if (path == NULL || path[0] == '\0')
        {
            printf("Doordvr_Snap_StartJpeg: SnapGetLastFilePath NULL/empty, ch=%d, idx=%d\n", channel, i);
            continue;
        }

        if (successCount >= SNAP_MAX_NUM)
        {
            break;
        }

        pthread_mutex_lock(&g_attachLock);

        strncpy(g_snapPaths[successCount], path, sizeof(g_snapPaths[successCount]) - 1);
        g_snapPaths[successCount][sizeof(g_snapPaths[successCount]) - 1] = '\0';

        pthread_mutex_unlock(&g_attachLock);

        printf("Doordvr_Snap_StartJpeg: capture %d ok, file=%s\n", successCount, g_snapPaths[successCount]);

        ++successCount;
    }

    pthread_mutex_lock(&g_attachLock);

    g_snapCount = (uint8_t)successCount;
    g_snapReady = (successCount > 0);

    pthread_mutex_unlock(&g_attachLock);

    if (successCount > 0)
    {
        printf("Doordvr_Snap_StartJpeg: snap ok, count=%d (will report with mp4 together)\n",
               successCount);
        return 0;
    }
    else
    {
        printf("Doordvr_Snap_StartJpeg: no picture captured\n");
        return -1;
    }
}

int anologDmsSnap(void)
{
    static unsigned int alarmSnapTime = 0;
    static unsigned int alarmTime = 0;
    PTHREAD_BUF send_buf;
    memset(&send_buf, 0, sizeof(send_buf));

    send_buf.m_value = 0x65;
    send_buf.m_channel = 1;
    send_buf.m_time = (DVR_U32_T)time(NULL);

    send_buf.m_buffer[0] = 2;   // 报警类型
    send_buf.m_buffer[1] = 1;   // 报警等级
    send_buf.m_buffer[2] = 0x1; // 报警状态：0x01开始    0x02结束
    send_buf.m_buffer[3] = 0x1; // 疲劳程序DSM

    if (send_buf.m_time >= alarmSnapTime + 30)
    {
        if (send_buf.m_time >= alarmTime + 30)
        {
            send_buf.start_id = PTHREAD_ALARM_RECORD_ID;
            send_buf.m_signal = 0;
            send_buf.m_buffer[4] = 4;
            printf("record##### send_buf.m_signal=%d,send_buf.m_time=%d", send_buf.m_signal, send_buf.m_time);
            ComThread_SendData(&send_buf, PTHREAD_ALARM_RECORD_ID);
            alarmTime = send_buf.m_time;
            AlarmMutimedia::Instance()->SetHaveRecoedFlag(true);
        }
        else
        {
            send_buf.m_buffer[4] = 3;
            AlarmMutimedia::Instance()->SetHaveRecoedFlag(false);
        }

#ifdef CONFIG_JPEG1
        send_buf.start_id = PTHREAD_SNAPJPEG1_ID;
        send_buf.m_signal = PIC_CIF;

        printf("anologDmsSnap: snap@@@@ ch=%d, signal=%d, time=%u, mediaNum=%d\n",
               send_buf.m_channel,
               send_buf.m_signal,
               send_buf.m_time,
               send_buf.m_buffer[4]);

        ComThread_SendData(&send_buf, PTHREAD_SNAPJPEG1_ID);
#else
        printf("anologDmsSnap: CONFIG_JPEG not defined, snap will NOT be triggered\n");
#endif

        alarmSnapTime = send_buf.m_time;
    }
    else
    {
        printf("anologDmsSnap: skip, lastSnapTime=%u, now=%u\n",
               alarmSnapTime, send_buf.m_time);
    }

    return 0;
}

void AdasProcessVideoDataBehind5S(VI_CHN channel, FRAME_INFO_T *m_frameInfo, unsigned char **pdata)
{
    //	printf("enter");
    int rc = 0;
    int m_channel;
    static int fisrtIframe = 0;
    static int frameCnt = 0;
    static bool onceFlag = true;
    unsigned long frameLen;
    unsigned char *m_data = NULL;

    u_int8_t encType, framerate, encoder_format;

    MP4FileHandle hMp4File;
    MP4TrackId m_video, m_audio;
    FRAME_INFO_T *temframeinfo;

    hMp4File = MP4Encoder::Instance()->getMp4FileHander();
    m_video = MP4Encoder::Instance()->getMp4VideoId();
    m_audio = MP4Encoder::Instance()->getMp4AudioId();
    m_channel = MP4Encoder::Instance()->getChannelId();
    // printf("m_video=%d,m_audio=%d,m_channel=%d",m_video,m_audio,m_channel);

    AllConfigDef_t *pAllConfig = SystemGlobal_GetAllconfigDefContext();
    encType = pAllConfig->encoderset.recorde_param[m_channel].encType;
    framerate = pAllConfig->encoderset.recorde_param[m_channel].framerate;
    encoder_format = pAllConfig->encoderset.recorde_param[channel].encoder_format;

    // printf("pAllConfig->encoderset.recorde_param[%d].encType=%d,framerate=%d",m_channel,encType,framerate);

    temframeinfo = (FRAME_INFO_T *)m_frameInfo;

    m_data = new unsigned char[m_frameInfo->length];
    if (!m_data)
    {
        printf("m_data new fail\n");
        return;
    }

    memcpy(m_data, m_frameInfo->pData, m_frameInfo->length);

    // m_data = *pdata;

    {
        if (temframeinfo && m_data)
        {
            if (channel != m_channel)
            {
                // printf("channel=%d",channel);
                return;
            }

            frameLen = temframeinfo->length;

            if (frameCnt > (framerate * 10))
            {
                printf("get record OK.\n");
                ADAS_UNRegEventCallback();
                MP4Encoder::Instance()->CloseMP4File();
                printf("WriteMP4File done.\n");

                fisrtIframe = 0;
                frameCnt = 0;

                pthread_mutex_lock(&g_attachLock);

                BuildCombinedAttachList(AlarmMutimedia::Instance());

                printf("Combined attachnum=%d, "
                       "[%s,%s,%s,%s]\n",
                       AlarmMutimedia::Instance()->m_event_alarm.attachmentnum,
                       AlarmMutimedia::Instance()->m_event_alarm.attachfile[0],
                       AlarmMutimedia::Instance()->m_event_alarm.attachfile[1],
                       AlarmMutimedia::Instance()->m_event_alarm.attachfile[2],
                       AlarmMutimedia::Instance()->m_event_alarm.attachfile[3]);

                pthread_mutex_unlock(&g_attachLock);

#if 1
                if (algoEventCallback)
                {
                    (*algoEventCallback)(&AlarmMutimedia::Instance()->m_event_alarm, algoEventCallbackargs);
                }
#endif

                pthread_mutex_lock(&g_attachLock);
                ResetAttachCache(AlarmMutimedia::Instance(), 0);
                pthread_mutex_unlock(&g_attachLock);

                onceFlag = true;
                return;
            }

#if 0
		if(onceFlag == true)
{
			if(pAllConfig->encoderset.recorde_param[channel].encType ==0)
			{
				if(temframeinfo->encoder_format == 1)
				{
					m_video = MP4AddH265VideoTrack(hMp4File, 900000, 900000 /framerate,
						temframeinfo->width, temframeinfo->height, 0x64, 0x00, 0x1f, 3);
					printf("MP4AddH265VideoTrack");
				}
				else
				{
					m_video = MP4AddH264VideoTrack(hMp4File, 900000, 900000 / framerate,
						temframeinfo->width, temframeinfo->height, 0x64, 0x00, 0x1f, 3);
					printf("MP4AddH264VideoTrack");
				}
				if (m_video == MP4_INVALID_TRACK_ID) {
					printf("MP4AddH264VideoTrack error");
					return;
				}
				MP4Encoder::Instance()->setMp4VideoId(m_video);
			}

			else if(pAllConfig->encoderset.recorde_param[channel].encType ==1)
			{

				if(temframeinfo->encoder_format == 1)
				{
						m_video = MP4AddH265VideoTrack(hMp4File, 900000, 900000 / framerate,
						temframeinfo->width, temframeinfo->height, 0x64, 0x00, 0x1f, 3);
					printf("MP4AddH265VideoTrack");
				}
				else
				{
					m_video = MP4AddH264VideoTrack(hMp4File, 900000, 900000 / framerate,
						temframeinfo->width, temframeinfo->height, 0x64, 0x00, 0x1f, 3);
					printf("MP4AddH264VideoTrack");
				}
				if (m_video == MP4_INVALID_TRACK_ID) {
					printf("MP4AddH264VideoTrack error");
					//return;
				}
				MP4Encoder::Instance()->setMp4VideoId(m_video);

				m_audio = MP4AddULawAudioTrack(hMp4File, 900000);
				printf("MP4AddULawAudioTrack");

				if (m_audio == MP4_INVALID_TRACK_ID) {
					printf("MP4AddULawAudioTrack error");
					//return;
				}						
				MP4Encoder::Instance()->setMp4AudioId(m_audio);
			}	
	onceFlag = false;			
}
#endif

            // printf("@@@frameType[%d],length[%d],height[%d],width[%d],keyFrame[%d],time[%lld],frame_id[%d]",
            // 		temframeinfo->frameType,temframeinfo->length,temframeinfo->height,temframeinfo->width,
            // 		temframeinfo->keyFrame,temframeinfo->time,temframeinfo->frame_id);

            if (temframeinfo->keyFrame)
            {
                fisrtIframe = 1;
            }

            if (fisrtIframe)
            {
                if (temframeinfo->frameType == 0)
                {
                    if (encoder_format == 0)
                    {
                        printf("Mp4WriteH264IFrame@@@frameType[%d],length[%d],height[%d],width[%d],keyFrame[%d],time[%lld],frame_id[%d]",
                               temframeinfo->frameType, temframeinfo->length, temframeinfo->height, temframeinfo->width,
                               temframeinfo->keyFrame, temframeinfo->time, temframeinfo->frame_id);

                        rc = MP4Encoder::Instance()->Mp4WriteH264IFrame((char *)m_data, frameLen);
                    }
                    else
                    {
                        rc = MP4Encoder::Instance()->Mp4WriteH265IFrame((char *)m_data, frameLen);
                    }
                    frameCnt++;
                }
                else if (temframeinfo->frameType == 1)
                {
                    char *pBuf = (char *)m_data;
                    pBuf[0] = ((frameLen - 4) >> 24) & 0xFF;
                    pBuf[1] = ((frameLen - 4) >> 16) & 0xFF;
                    pBuf[2] = ((frameLen - 4) >> 8) & 0xFF;
                    pBuf[3] = ((frameLen - 4) >> 0) & 0xFF;

                    // printf("MP4WriteSample@@@frameType[%d],length[%d],height[%d],width[%d],keyFrame[%d],time[%lld],frame_id[%d]",
                    // temframeinfo->frameType,temframeinfo->length,temframeinfo->height,temframeinfo->width,
                    // temframeinfo->keyFrame,temframeinfo->time,temframeinfo->frame_id);

                    rc = MP4WriteSample(hMp4File, m_video, (const uint8_t *)(m_data), frameLen, MP4_INVALID_DURATION, 0, 1);
                    frameCnt++;
                }
                else
                {
                    // printf("MP4WriteSample@@@frameType[%d],length[%d],height[%d],width[%d],keyFrame[%d],time[%lld],frame_id[%d]",
                    // temframeinfo->frameType,temframeinfo->length,temframeinfo->height,temframeinfo->width,
                    // temframeinfo->keyFrame,temframeinfo->time,temframeinfo->frame_id);

                    rc = MP4WriteSample(hMp4File, m_audio, (const uint8_t *)m_data, frameLen, MP4_INVALID_DURATION, 0, 1);
                }
            }
        }
    }
    if (m_data)
    {
        // printf("free(m_data)");
        free(m_data);
    }
}

int AlarmMutimedia::Doordvr_Record_StartMp4(VI_CHN channel, DVR_U8_T encoder_size, AiAlarmItem *param)
{
    printf("Enter\n");

    if (param == NULL)
    {
        printf("Doordvr_Record_StartMp4: param is NULL\n");
        return -1;
    }

    char gpsTime[16] = {0};
    char acMp4Path[128] = {0};
    char iframeStart = 0;

    int quit = 0;
    int i = 0;
    int rc = 0;

    int frameType = 0;
    int frameLen = 0;
    int height = 0;
    int width = 0;
    unsigned long framenum = 0;

    u_int8_t encType = 0, framerate = 25, encoder_format = 0;

    datetime_t dateTime;
    FRAME_INFO_EX *m_frameInfo = NULL;
    FRAME_INFO_EX *temframeinfo = NULL;

    MP4FileHandle hMp4File;
    MP4TrackId m_video;
    MP4TrackId m_audio;

    AllConfigDef_t *pAllConfig = SystemGlobal_GetAllconfigDefContext();
    encType = pAllConfig->encoderset.recorde_param[channel].encType;
    framerate = pAllConfig->encoderset.recorde_param[channel].framerate;
    encoder_format = pAllConfig->encoderset.recorde_param[channel].encoder_format;

    Hqt_Common_VideoSizeToValue(encoder_size, (DVR_U32_T *)&width, (DVR_U32_T *)&height);

    width = 1280;
    height = 720;

    printf("pAllConfig->encoderset.recorde_param[%d].encType=%d,framerate=%d,encoder_format=%d\n",
           channel, encType, framerate, encoder_format);
    printf("encoder_size=%d,width=%d,height=%d\n", encoder_size, width, height);

    MP4Encoder::Instance()->setChannelId(channel);

    while (!quit)
    {
        if (VideoLost_GetEvent(channel) == false)
        {
            if (param->alarmtime == 0)
            {
                param->alarmtime = (DVR_U32_T)time(NULL);
            }

            pthread_mutex_lock(&g_attachLock);

            if (g_cachedAlarmTime == 0 || g_cachedAlarmTime != param->alarmtime)
            {
                ResetAttachCache(this, param->alarmtime);
            }

            UpdateBaseAlarmFields_NoClear(this, param);

            pthread_mutex_unlock(&g_attachLock);

            GetSystemTimeFromSecond(param->alarmtime, &dateTime);

            printf("current gpstime is %04d-%02d-%02d %02d:%02d:%02d, devicetype=%x\n",
                   dateTime.year, dateTime.month, dateTime.day,
                   dateTime.hour, dateTime.minute, dateTime.second,
                   param->devicetype);

            snprintf(gpsTime, sizeof(gpsTime), "%02d%02d%02d%02d%02d%02d",
                     dateTime.year, dateTime.month, dateTime.day,
                     dateTime.hour, dateTime.minute, dateTime.second);

            int ret = CreateMediaDir(m_acH264Path);
            if (ret != 0)
            {
                printf("CreateMediaDir failed@@@@@@@@@@@@@@@\n");
            }

            snprintf(acMp4Path, sizeof(acMp4Path), "%s/%s_%02x_%02d_%02d.mp4",
                     m_acH264Path, gpsTime,
                     param->devicetype, param->alarmtype, param->alarmchannel);

            pthread_mutex_lock(&g_attachLock);

            strncpy(g_mp4Path, acMp4Path, sizeof(g_mp4Path) - 1);
            g_mp4Path[sizeof(g_mp4Path) - 1] = '\0';
            g_mp4Ready = true;

            pthread_mutex_unlock(&g_attachLock);

            printf("acMp4Path cached:[%s]\n", acMp4Path);

            hMp4File = MP4Encoder::Instance()->CreateMP4File(acMp4Path, width, height, 90000, framerate);
            if (hMp4File == MP4_INVALID_FILE_HANDLE)
            {
                printf("CreateMP4File failed\n");
                quit = 1;
                break;
            }
#if 1
            framenum = Hqt_Mpi_GetPreVideoFrame(channel, &m_frameInfo);

#if 1
            int m_channelFramecount = 0;
            int m_otherFramecount = 0;
            int m_totalFramecount = 0;

            for (i = framenum; i > 0; i--)
            {
                if (m_frameInfo[i].channel == channel)
                {
                    m_channelFramecount++;
                }
                else
                {
                    m_otherFramecount++;
                }

                if (m_channelFramecount >= 15 * framerate)
                {
                    m_totalFramecount = m_channelFramecount + m_otherFramecount;
                    printf("m_totalFramecount: %d, m_channelFramecount: %d, m_otherFramecount: %d\n",
                           m_totalFramecount, m_channelFramecount, m_otherFramecount);
                    break;
                }
            }

            i = framenum - m_totalFramecount;
            printf("i=%d,framenum=%lu,m_totalFramecount=%d\n", i, framenum, m_totalFramecount);
#else
            i = framenum - 13 * framerate;
            printf("i=%d,framenum=%lu,framerate=%d\n", i, framenum, framerate);
#endif

            if (pAllConfig->encoderset.recorde_param[channel].encType == 0)
            {
                if (encoder_format == 1)
                {
                    printf("MP4AddH265VideoTrack before\n");
                    m_video = MP4AddH265VideoTrack(hMp4File, 900000, 900000 / framerate,
                                                   m_frameInfo[i].width, m_frameInfo[i].height, 0x64, 0x00, 0x1f, 3);
                    printf("MP4AddH265VideoTrack\n");
                }
                else
                {
                    printf("MP4AddH264VideoTrack before\n");
                    m_video = MP4AddH264VideoTrack(hMp4File, 900000, 900000 / framerate,
                                                   m_frameInfo[i].width, m_frameInfo[i].height, 0x64, 0x00, 0x1f, 3);
                    printf("MP4AddH264VideoTrack\n");
                }

                if (m_video == MP4_INVALID_TRACK_ID)
                {
                    printf("MP4Add video track error\n");
                    Hqt_Mpi_Delete((unsigned char *)m_frameInfo);
                    return -3;
                }
                MP4Encoder::Instance()->setMp4VideoId(m_video);
            }
            else if (pAllConfig->encoderset.recorde_param[channel].encType == 1)
            {
                if (encoder_format == 1)
                {
                    printf("MP4AddH265VideoTrack before\n");
                    m_video = MP4AddH265VideoTrack(hMp4File, 900000, 900000 / framerate,
                                                   m_frameInfo[i].width, m_frameInfo[i].height, 0x64, 0x00, 0x1f, 3);
                    printf("MP4AddH265VideoTrack\n");
                }
                else
                {
                    printf("MP4AddH264VideoTrack before\n");
                    m_video = MP4AddH264VideoTrack(hMp4File, 900000, 900000 / framerate,
                                                   m_frameInfo[i].width, m_frameInfo[i].height, 0x64, 0x00, 0x1f, 3);
                    printf("MP4AddH264VideoTrack\n");
                }

                if (m_video == MP4_INVALID_TRACK_ID)
                {
                    printf("MP4Add video track error\n");
                    Hqt_Mpi_Delete((unsigned char *)m_frameInfo);
                    return -3;
                }
                MP4Encoder::Instance()->setMp4VideoId(m_video);

                printf("MP4AddULawAudioTrack before\n");
                m_audio = MP4AddULawAudioTrack(hMp4File, 900000);
                printf("MP4AddULawAudioTrack\n");

                if (m_audio == MP4_INVALID_TRACK_ID)
                {
                    printf("MP4Add audio track error\n");
                    Hqt_Mpi_Delete((unsigned char *)m_frameInfo);
                    return -3;
                }
                MP4Encoder::Instance()->setMp4AudioId(m_audio);
            }

            for (; i < (int)framenum; i++)
            {
                if (m_frameInfo[i].channel == channel)
                {
                    temframeinfo = m_frameInfo;
                    frameType = temframeinfo[i].frameType;
                    frameLen = temframeinfo[i].length;

                    if (temframeinfo[i].keyFrame == 1)
                    {
                        iframeStart = 1;
                        printf("iframe start i=%d\n", i);
                    }

                    if (iframeStart) // I帧开始写录像
                    {
                        if (temframeinfo[i].frameType == 0)
                        {
                            if (encoder_format == 0)
                            {
                                rc = MP4Encoder::Instance()->Mp4WriteH264IFrame((char *)temframeinfo[i].pData, frameLen);
                            }
                            else
                            {
                                rc = MP4Encoder::Instance()->Mp4WriteH265IFrame((char *)temframeinfo[i].pData, frameLen);
                            }
                        }
                        else if (temframeinfo[i].frameType == 1)
                        {
                            char *pBuf = (char *)temframeinfo[i].pData;
                            pBuf[0] = ((frameLen - 4) >> 24) & 0xFF;
                            pBuf[1] = ((frameLen - 4) >> 16) & 0xFF;
                            pBuf[2] = ((frameLen - 4) >> 8) & 0xFF;
                            pBuf[3] = ((frameLen - 4) >> 0) & 0xFF;

                            rc = MP4WriteSample(hMp4File, m_video, (const uint8_t *)(pBuf),
                                                frameLen, MP4_INVALID_DURATION, 0, 1);
                        }
                        else if (temframeinfo[i].frameType == 2)
                        {
                            rc = MP4WriteSample(hMp4File, m_audio, (const uint8_t *)temframeinfo[i].pData,
                                                frameLen, MP4_INVALID_DURATION, 0, 1);
                        }
                    }
                }
            }

            if (framenum)
            {
                Hqt_Mpi_Delete((unsigned char *)m_frameInfo);
            }
#endif
            ADAS_RegEventCallback(AdasProcessVideoDataBehind5S);

            quit = 1;
        }
        else
        {
            quit = 1;
        }
    }

    printf("%s exit!\n", __func__);
    return 0;
}
