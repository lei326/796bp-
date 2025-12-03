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
//#include "MP4Encoder.h"
//#include "file.h"
#include "doordvr_algo_subiao.h"
#include "mspublic.h"

#define ALGO_MUTIMEDIA_SAVE_PATH "/mnt/hddisk11/jpeg"

AlarmMutimedia *AlarmMutimedia::m_pInstance = NULL;

int (*algoEventCallback)(AiAlarmItem *item, void *args) = NULL;
static void *algoEventCallbackargs = NULL;

AlarmMutimedia::AlarmMutimedia(void)
{
    m_encodeOnceFlag = true;

    m_haveRecordFlag = false;

    memset(&m_event_alarm, 0, sizeof(AiAlarmItem));
    snprintf(m_acH264Path, sizeof(m_acH264Path), "%s", ALGO_MUTIMEDIA_SAVE_PATH);

    printf("AlarmMutimedia::AlarmMutimedia: m_acH264Path=[%s]\n", m_acH264Path);
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
        m_pInstance = NULL;
    }
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

int AlarmMutimedia::Doordvr_Snap_StartJpeg(VI_CHN channel, PIC_SIZE_E picSize, AiAlarmItem *param)
{
    (void)picSize; 

    if (param == NULL)
    {
        printf("Doordvr_Snap_StartJpeg: param is NULL\n");
        return -1;
    }

    memset(&m_event_alarm, 0, sizeof(m_event_alarm));
    memcpy(&m_event_alarm, param, sizeof(AiAlarmItem));

    if (m_event_alarm.alarmtime == 0)
    {
        m_event_alarm.alarmtime = (DVR_U32_T)time(NULL);
    }

    m_event_alarm.attachmentnum = 0;
    for (int i = 0; i < 5; ++i)
    {
        memset(m_event_alarm.attachfile[i], 0, sizeof(m_event_alarm.attachfile[i]));
    }

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

        if (successCount >= 5)
        {
            printf("Doordvr_Snap_StartJpeg: successCount(%d) overflow attachfile size, ignore extra pictures\n", successCount);
            break;
        }

        strncpy(m_event_alarm.attachfile[successCount], path, sizeof(m_event_alarm.attachfile[successCount]) - 1);
        m_event_alarm.attachfile[successCount][sizeof(m_event_alarm.attachfile[successCount]) - 1] = '\0';

        printf("Doordvr_Snap_StartJpeg: capture %d ok, file=%s\n", successCount, m_event_alarm.attachfile[successCount]);

        ++successCount;
    }

    m_event_alarm.attachmentnum = (uint8_t)successCount;

    if (successCount > 0)
    {

        if (algoEventCallback)
        {
            printf("Doordvr_Snap_StartJpeg: notify SuBiao, attachmentnum=%d\n", successCount);
            (*algoEventCallback)(&m_event_alarm, algoEventCallbackargs);
        }
        else
        {
            printf("Doordvr_Snap_StartJpeg: success, but algoEventCallback NULL\n");
        }

        printf("%s exit (ok)!\n", __func__);
        return 0;
    }
    else
    {
        printf("Doordvr_Snap_StartJpeg: no picture captured, exit with error\n");
        return -1;
    }
}

int anologDmsSnap(void)
{
    static unsigned int alarmSnapTime = 0;

    PTHREAD_BUF send_buf;
    memset(&send_buf, 0, sizeof(send_buf));

    send_buf.m_value   = 0x65;                  
    send_buf.m_channel = 1;                    
    send_buf.m_time    = (DVR_U32_T)time(NULL);

    send_buf.m_buffer[0] = 2;   
    send_buf.m_buffer[1] = 1;   
    send_buf.m_buffer[2] = 0x1;
    send_buf.m_buffer[3] = 0x1; 

    if (send_buf.m_time >= alarmSnapTime + 30)
    {
        send_buf.m_buffer[4] = 3; 

        AlarmMutimedia::Instance()->SetHaveRecoedFlag(false);

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
