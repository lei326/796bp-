#include "doordvr_typesdefine.h"
#if algo
#include "dmsdetect.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h> 
#include <iostream>
#include <atomic>

#include "doordvr_config.h"
#include "alarm_dms.h"
#include "mystd.h"
#include "face_box.h"
#include "doordvr_system_global.h"
#include "doordvr_getsetconfig.h"
#include "rk_mpi_mb.h"
#include "rk_mpi_sys.h"
#include "rk_mpi_vi.h"
#include <pthread.h>
#include "drmrga.h"
#include "rga.h"
#include "RgaApi.h"
#include "doordvr_pcmplayer.h"
#include "doordvr_message.h"
#include "doordvr_messagequeue.h"
#include "backboardmanage.h"
#include "IDAFrameDraw.h"
extern "C" AI *AI_CREATE(const char *module);
#define DMS_AI_INPUT_WIDTH 640
#define DMS_AI_INPUT_HEIGHT 360
#define DMS_VI_DEV_ID 3
static alarm_dms g_dms_alarm;
static CDMSDetect *g_dms_self = NULL;
extern int g_mirrorenable;

int global_curSpeed = 0;
int global_speedThreshold[8] = {0};

enum DmsEventIndex
{
    EVT_IDX_NOBODY = 0,
    EVT_IDX_CAMERA_COVERED,
    EVT_IDX_LOOKING_AROUND,
    EVT_IDX_LOOKING_DOWN,
    EVT_IDX_EYE_CLOSED,
    EVT_IDX_YAWN,
    EVT_IDX_SMOKING,
    EVT_IDX_CALLING,
    EVT_IDX_COUNT // = 8
};

static const int ALARM_EVENT_FLAG[EVT_IDX_COUNT] = {
    DMS::NOBODY,
    DMS::CAMERA_COVERED,
    DMS::LOOKING_AROUND,
    DMS::LOOKING_DOWN,
    DMS::EYE_CLOSED,
    DMS::YAWN,
    DMS::SMOKING,
    DMS::CALLING};

static const int ALARM_TYPE_BIT[EVT_IDX_COUNT] = {
    0, 1, 4, 5, 6, 7, 8, 9};

static const int ALARM_PARAM_INDEX[EVT_IDX_COUNT] = {
    0, 1, 2, 3, 4, 5, 6, 7};


#define COLOR_BUF_PAGE_SIZE 4096


static uint8_t *s_colorBuf = NULL;
static int s_colorBufBytes = 0;      
static uint8_t s_colorR = 0, s_colorG = 0, s_colorB = 0;

static void FreeColorBuffer()
{
    if (s_colorBuf)
    {
        free(s_colorBuf);
        s_colorBuf = NULL;
    }
    s_colorBufBytes = 0;
}

static bool EnsureColorBuffer(int pixelCount, uint8_t r, uint8_t g, uint8_t b)
{
    int needBytes = pixelCount * 3;

    if (needBytes < COLOR_BUF_PAGE_SIZE)
        needBytes = COLOR_BUF_PAGE_SIZE;
    needBytes = (needBytes + COLOR_BUF_PAGE_SIZE - 1) & ~(COLOR_BUF_PAGE_SIZE - 1);

    bool needAlloc = (!s_colorBuf || s_colorBufBytes < needBytes);
    bool needRefill = needAlloc;

    if (needAlloc)
    {
        FreeColorBuffer();

        int ret = posix_memalign((void **)&s_colorBuf, COLOR_BUF_PAGE_SIZE, needBytes);
        if (ret != 0 || !s_colorBuf)
        {
            printf("[DMS] posix_memalign failed: ret=%d size=%d\n", ret, needBytes);
            s_colorBuf = NULL;
            s_colorBufBytes = 0;
            return false;
        }
        s_colorBufBytes = needBytes;
    }

    if (needRefill || r != s_colorR || g != s_colorG || b != s_colorB)
    {
        s_colorR = r;
        s_colorG = g;
        s_colorB = b;
        int totalPixels = s_colorBufBytes / 3;
        for (int i = 0; i < totalPixels; i++)
        {
            s_colorBuf[i * 3]     = r;
            s_colorBuf[i * 3 + 1] = g;
            s_colorBuf[i * 3 + 2] = b;
        }
    }
    return true;
}

static int BlitColorRect(void *nv12Addr, int nv12W, int nv12H,
                          int x, int y, int w, int h,
                          uint8_t r, uint8_t g, uint8_t b)
{
    if (!nv12Addr || w < 2 || h < 2)
        return -1;

    int srcStride = w;
    if (srcStride < 64)
        srcStride = 64;
    srcStride = (srcStride + 15) & ~15;

    if (!EnsureColorBuffer(srcStride * h, r, g, b))
        return -2;

    rga_info_t src;
    rga_info_t dst;
    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));

    src.virAddr = s_colorBuf;
    src.mmuFlag = 1;
    rga_set_rect(&src.rect, 0, 0, w, h, srcStride, h, RK_FORMAT_RGB_888);

    dst.virAddr = nv12Addr;
    dst.mmuFlag = 1;
    rga_set_rect(&dst.rect, x, y, w, h, nv12W, nv12H, RK_FORMAT_YCbCr_420_SP);

    int ret = c_RkRgaBlit(&src, &dst, NULL);
    if (ret != 0)
    {
        usleep(1000);
        memset(&src, 0, sizeof(src));
        memset(&dst, 0, sizeof(dst));
        src.virAddr = s_colorBuf;
        src.mmuFlag = 1;
        rga_set_rect(&src.rect, 0, 0, w, h, srcStride, h, RK_FORMAT_RGB_888);
        dst.virAddr = nv12Addr;
        dst.mmuFlag = 1;
        rga_set_rect(&dst.rect, x, y, w, h, nv12W, nv12H, RK_FORMAT_YCbCr_420_SP);
        ret = c_RkRgaBlit(&src, &dst, NULL);
    }
    if (ret != 0)
    {
        usleep(3000);
        memset(&src, 0, sizeof(src));
        memset(&dst, 0, sizeof(dst));
        src.virAddr = s_colorBuf;
        src.mmuFlag = 1;
        rga_set_rect(&src.rect, 0, 0, w, h, srcStride, h, RK_FORMAT_RGB_888);
        dst.virAddr = nv12Addr;
        dst.mmuFlag = 1;
        rga_set_rect(&dst.rect, x, y, w, h, nv12W, nv12H, RK_FORMAT_YCbCr_420_SP);
        ret = c_RkRgaBlit(&src, &dst, NULL);
    }
    if (ret != 0)
    {
        printf("[DMS] BlitColorRect FINAL FAIL after 3 tries: ret=%d rect(%d,%d,%d,%d)\n",
               ret, x, y, w, h);
    }
    return ret;
}


static void DrawBoxOnNv12(void *nv12Addr, int nv12W, int nv12H,
                           int imgW, int imgH,
                           int left, int top, int right, int bottom,
                           int lineW,
                           uint8_t r, uint8_t g, uint8_t b)
{
    if (!nv12Addr || left >= right || top >= bottom)
        return;

    left   = left & ~1;
    top    = top & ~1;
    right  = (right + 1) & ~1;
    bottom = (bottom + 1) & ~1;
    if (lineW < 2) lineW = 2;
    lineW  = (lineW + 1) & ~1;

    if (left < 0)       left = 0;
    if (top < 0)        top = 0;
    if (right > imgW)   right = imgW;
    if (bottom > imgH)  bottom = imgH;

    int boxW = right - left;
    int boxH = bottom - top;
    boxW = boxW & ~1;
    boxH = boxH & ~1;

    if (boxW < lineW * 2 || boxH < lineW * 2)
        return;

    BlitColorRect(nv12Addr, nv12W, nv12H,
                  left, top, boxW, lineW, r, g, b);

    int bottomY = (bottom - lineW) & ~1;
    if (bottomY > top + lineW)
        BlitColorRect(nv12Addr, nv12W, nv12H,
                      left, bottomY, boxW, lineW, r, g, b);

    int innerTop = top + lineW;
    int innerH = ((bottomY - innerTop) & ~1);
    if (innerH >= 2)
        BlitColorRect(nv12Addr, nv12W, nv12H,
                      left, innerTop, lineW, innerH, r, g, b);

    int rightX = (right - lineW) & ~1;
    if (rightX > left + lineW && innerH >= 2)
        BlitColorRect(nv12Addr, nv12W, nv12H,
                      rightX, innerTop, lineW, innerH, r, g, b);
}


static pthread_mutex_t s_faceLock = PTHREAD_MUTEX_INITIALIZER;
static bool s_faceValid  = false;
static int  s_faceLeft   = 0;
static int  s_faceTop    = 0;
static int  s_faceRight  = 0;
static int  s_faceBottom = 0;
static long long s_lastFaceDetectedUs = 0;

#define NOFACE_DEBOUNCE_COUNT 3
static int s_nofaceCounter = 0;

static const float SMOOTH_ALPHA = 0.3f;
static float s_smoothLeft   = 0.0f;
static float s_smoothTop    = 0.0f;
static float s_smoothRight  = 0.0f;
static float s_smoothBottom = 0.0f;
static bool  s_smoothInited = false;

#define DMS_AI_RGB_SIZE (DMS_AI_INPUT_WIDTH * DMS_AI_INPUT_HEIGHT * 3)

static pthread_t     s_aiThread = 0;
static pthread_mutex_t s_aiLock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  s_aiCond = PTHREAD_COND_INITIALIZER;
static bool   s_aiNewData = false;
static bool   s_aiQuit    = false;
static bool   s_aiBusy    = false;
static uint8_t s_aiRgbBuf[DMS_AI_RGB_SIZE];
static int    s_aiFrameW  = 0;
static int    s_aiFrameH  = 0;
static DMS   *s_aiEngine  = NULL;
static unsigned long long s_aiSn = 0;

static void *DmsAiWorkerProc(void * /*arg*/)
{
    printf("[DMS_AI] AI worker thread started\n");

    while (true)
    {
        pthread_mutex_lock(&s_aiLock);
        while (!s_aiNewData && !s_aiQuit)
            pthread_cond_wait(&s_aiCond, &s_aiLock);

        if (s_aiQuit)
        {
            pthread_mutex_unlock(&s_aiLock);
            break;
        }

        s_aiNewData = false;
        s_aiBusy = true;
        int frameW = s_aiFrameW;
        int frameH = s_aiFrameH;
        pthread_mutex_unlock(&s_aiLock);

        if (s_aiEngine)
        {
            s_aiEngine->update(s_aiSn++, (const char *)s_aiRgbBuf, 640, 360);
        }

        if (g_face_found)
        {
            float scaleX = (float)frameW / (float)DMS_AI_INPUT_WIDTH;
            float scaleY = (float)frameH / (float)DMS_AI_INPUT_HEIGHT;

            float rawL = g_face_box.left   * scaleX;
            float rawT = g_face_box.top    * scaleY;
            float rawR = g_face_box.right  * scaleX;
            float rawB = g_face_box.bottom * scaleY;

            if (rawL < 0.0f)            rawL = 0.0f;
            if (rawT < 0.0f)            rawT = 0.0f;
            if (rawR > (float)frameW)   rawR = (float)frameW;
            if (rawB > (float)frameH)   rawB = (float)frameH;

            if (!s_smoothInited)
            {
                s_smoothLeft   = rawL;
                s_smoothTop    = rawT;
                s_smoothRight  = rawR;
                s_smoothBottom = rawB;
                s_smoothInited = true;
            }
            else
            {
                float a = SMOOTH_ALPHA;
                float b = 1.0f - a;
                s_smoothLeft   = a * rawL + b * s_smoothLeft;
                s_smoothTop    = a * rawT + b * s_smoothTop;
                s_smoothRight  = a * rawR + b * s_smoothRight;
                s_smoothBottom = a * rawB + b * s_smoothBottom;
            }

            pthread_mutex_lock(&s_faceLock);
            s_faceLeft   = (int)(s_smoothLeft);
            s_faceTop    = (int)(s_smoothTop);
            s_faceRight  = (int)(s_smoothRight);
            s_faceBottom = (int)(s_smoothBottom);
            s_faceValid  = true;
            s_lastFaceDetectedUs = local_get_curtime();
            s_nofaceCounter = 0;
            pthread_mutex_unlock(&s_faceLock);
        }
        else
        {
            pthread_mutex_lock(&s_faceLock);
            s_nofaceCounter++;
            if (s_nofaceCounter >= NOFACE_DEBOUNCE_COUNT)
            {
                s_faceValid = false;
                s_smoothInited = false;
                s_nofaceCounter = 0;
            }
            pthread_mutex_unlock(&s_faceLock);
        }

        pthread_mutex_lock(&s_aiLock);
        s_aiBusy = false;
        pthread_mutex_unlock(&s_aiLock);
    }

    printf("[DMS_AI] AI worker thread exited\n");
    return NULL;
}

static void StartAiThread(DMS *engine)
{
    if (s_aiThread != 0)
        return;

    s_aiEngine = engine;
    s_aiSn = 0;
    s_aiQuit = false;
    s_aiNewData = false;
    s_aiBusy = false;

    pthread_mutex_lock(&s_faceLock);
    s_faceValid = false;
    s_smoothInited = false;
    s_nofaceCounter = 0;
    pthread_mutex_unlock(&s_faceLock);

    pthread_create(&s_aiThread, NULL, DmsAiWorkerProc, NULL);
}

static void StopAiThread()
{
    if (s_aiThread == 0)
        return;

    pthread_mutex_lock(&s_aiLock);
    s_aiQuit = true;
    pthread_cond_signal(&s_aiCond);
    pthread_mutex_unlock(&s_aiLock);

    pthread_join(s_aiThread, NULL);
    s_aiThread = 0;
    s_aiEngine = NULL;

    pthread_mutex_lock(&s_faceLock);
    s_faceValid = false;
    s_smoothInited = false;
    s_nofaceCounter = 0;
    pthread_mutex_unlock(&s_faceLock);
}

static void SubmitToAiThread(uint8_t *rgbData, int frameW, int frameH)
{
    pthread_mutex_lock(&s_aiLock);
    if (!s_aiBusy && !s_aiNewData)
    {
        memcpy(s_aiRgbBuf, rgbData, DMS_AI_RGB_SIZE);
        s_aiFrameW = frameW;
        s_aiFrameH = frameH;
        s_aiNewData = true;
        pthread_cond_signal(&s_aiCond);
    }
    pthread_mutex_unlock(&s_aiLock);
}


void CDMSDetect::PauseForMenu()
{
    CDMSDetect *p = CDMSDetect::Instance();
    if (!p)
        return;
    p->ResetAlarmState(true);
    p->Pause();
}

void CDMSDetect::ResumeFromMenu()
{
    CDMSDetect *p = CDMSDetect::Instance();
    if (!p)
        return;
    p->Resume();
}

int CDMSDetect::FilterAlarmEventBySpeed(int e)
{
    int filtered = 0;
    uint32_t alarmType = m_cachedAlarmType;

    for (int i = 0; i < EVT_IDX_COUNT; i++)
    {
        int flag = ALARM_EVENT_FLAG[i];
        int paramIdx = ALARM_PARAM_INDEX[i];
        int typeBit = ALARM_TYPE_BIT[i];

        if (!(e & flag))
            continue;

        bool speedOk = (global_curSpeed >= m_cachedSpeedThreshold[i]);
        bool typeOn = ((alarmType >> typeBit) & 1) != 0;

        if (speedOk && typeOn)
        {
            filtered |= flag;
        }
        else
        {
            MSLOG_DEBUG("FilterAlarm: event[%d] flag=0x%x BLOCKED: "
                        "speed=%d threshold=%d(paramIdx=%d) speedOk=%d, "
                        "alarmType=0x%x bit=%d typeOn=%d\n",
                        i, flag,
                        global_curSpeed, m_cachedSpeedThreshold[i], paramIdx, speedOk,
                        alarmType, typeBit, typeOn);
        }
    }
    return filtered;
}

bool CDMSDetect::Nv12ToRgb888(void *vi_nv12, int srcW, int srcH)
{
    if (!vi_nv12 || srcW <= 0 || srcH <= 0)
        return false;

    const int dstW = DMS_AI_INPUT_WIDTH;
    const int dstH = DMS_AI_INPUT_HEIGHT;
    const int rgbSize = dstW * dstH * 3;

    if (!imageBuffer)
    {
        imageBuffer = new uint8_t[rgbSize];
        if (!imageBuffer)
        {
            printf("alloc imageBuffer failed\n");
            return false;
        }
    }

    rga_info_t src;
    rga_info_t dst;
    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));

    src.virAddr = vi_nv12;
    src.mmuFlag = 1;
    rga_set_rect(&src.rect, 0, 0, srcW, srcH, srcW, srcH, RK_FORMAT_YCbCr_420_SP);

    dst.virAddr = imageBuffer;
    dst.mmuFlag = 1;
    rga_set_rect(&dst.rect, 0, 0, dstW, dstH, dstW, dstH, RK_FORMAT_RGB_888);

    int ret = c_RkRgaBlit(&src, &dst, NULL);
    if (ret)
    {
        printf("c_RkRgaBlit failed ret=%d\n", ret);
        return false;
    }
    return true;
}

int CDMSDetect::SwitchDmsChannel(uint8_t uiChn)
{
    pthread_mutex_lock(&m_dms_mutex);
    m_switch_ui_chn = (int)uiChn;
    MSLOG_DEBUG("SwitchDmsChannel: request_ui=%u current_ui=%u current_vi=%d pending=%d\n",
                uiChn, Ui_Current_Chn, m_dms_video_chn, m_switch_ui_chn);
    pthread_mutex_unlock(&m_dms_mutex);
    return 0;
}

int CDMSDetect::ApplySwitchChn()
{
    uint8_t uiChn = 0;

    pthread_mutex_lock(&m_dms_mutex);
    if (m_switch_ui_chn < 0)
    {
        pthread_mutex_unlock(&m_dms_mutex);
        return 0;
    }
    uiChn = (uint8_t)m_switch_ui_chn;
    m_switch_ui_chn = -1;
    pthread_mutex_unlock(&m_dms_mutex);

    int newViChn = DmsChannelToVi(uiChn);

    pthread_mutex_lock(&m_dms_mutex);
    if (uiChn == Ui_Current_Chn && newViChn == m_dms_video_chn)
    {
        pthread_mutex_unlock(&m_dms_mutex);
        return 0;
    }

    int oldViChn = m_dms_video_chn;
    uint8_t oldUiChn = Ui_Current_Chn;
    Ui_Current_Chn = uiChn;
    m_dms_video_chn = newViChn;
    pthread_mutex_unlock(&m_dms_mutex);

    MSLOG_DEBUG("ApplySwitchChn: ui old=%u new=%u, vi old=%d new=%d\n",
                oldUiChn, uiChn, oldViChn, newViChn);

    if (oldUiChn != uiChn)
        ResetAlarmState(true);

    if (newViChn < 0)
    {
        MSLOG_DEBUG("ApplySwitchChn: DMS disabled\n");
        return 0;
    }
    MSLOG_DEBUG("ApplySwitchChn: switch to VI chn=%d\n", newViChn);
    return 0;
}

int CDMSDetect::DmsChannelToVi(uint8_t uiChn)
{
    if (uiChn == 0)
        return -1;
    if (uiChn >= 1 && uiChn <= 6)
        return (int)uiChn - 1;
    MSLOG_DEBUG("DmsChannelToVi invalid uiChn=%u, force disable\n", uiChn);
    return -1;
}

CDMSDetect::CDMSDetect()
    : imageBuffer(NULL),
      Ui_Current_Chn(0),
      m_switch_ui_chn(-1),
      m_dms_video_chn(-1),
      m_thread_quit(0),
      m_threadState(DMS_THREAD_STATE_STOPPED),
      ai_dms(NULL),
      sn(0),
      m_event(0),
      m_sn(0),
      m_event_time_us(0),
      m_alarm_active(false),
      m_alarm_status_on(false),
      m_last_alarm_time_us(0),
      m_last_tts_time_us(0),
      m_alarm_ui_chn(0),
      m_cachedAlarmType(0)
{
    pthread_mutex_init(&m_dms_mutex, NULL);
    pthread_mutex_init(&m_event_mutex, NULL);
    memset(m_cachedSpeedThreshold, 0, sizeof(m_cachedSpeedThreshold));
    m_threadId = 0;
    g_dms_self = this;
}

bool CDMSDetect::initializeRGA_LibRga()
{
    int ret = c_RkRgaInit();
    if (ret < 0)
    {
        printf("ERROR: c_RkRgaInit failed! ret=%d\n", ret);
        return false;
    }
    return true;
}

int CDMSDetect::init()
{
    if (ai_dms != NULL)
    {
        MSLOG_DEBUG("CDMSDetect::init already inited, ai_dms=%p\n", ai_dms);
        return 0;
    }

    if (!initializeRGA_LibRga())
    {
        printf("CDMSDetect::init initializeRGA_LibRga failed\n");
        return -10;
    }

    ai_dms = (DMS *)AI_CREATE("dms");
    if (!ai_dms)
    {
        c_RkRgaDeInit();
        return -1;
    }

    int ret = ai_dms->init(CDMSDetect::OnDmsEventCallback, 0.40f);
    MSLOG_DEBUG("CDMSDetect::init ai_dms->init ret=%d, ai_dms=%p\n", ret, ai_dms);

    if (ret != 0)
    {
        printf("CDMSDetect::init dms init failed, ret=%d\n", ret);
        delete ai_dms;
        ai_dms = NULL;
        return -2;
    }

    sn = 0;

    EnsureColorBuffer(640 * 1024, 255, 0, 0);

    StartAiThread(ai_dms);

    MSLOG_DEBUG("CDMSDetect::init done: video_chn=%d\n", m_dms_video_chn);
    return 0;
}

void CDMSDetect::ResetChannelState()
{
    pthread_mutex_lock(&m_dms_mutex);
    m_switch_ui_chn = -1;
    Ui_Current_Chn = 0;
    m_dms_video_chn = -1;
    pthread_mutex_unlock(&m_dms_mutex);
    MSLOG_DEBUG("ResetChannelState: ui=0 vi=-1 pending=-1\n");
}

int CDMSDetect::Start()
{
    AllConfigDef_t *pAllConfig = SystemGlobal_GetAllconfigDefContext();
    uint8_t cfgUiChn = pAllConfig->abdparam.algVideoChnSelect.DMS_VideoChn_num;

    if (GetStatus() && m_threadState != DMS_THREAD_STATE_STOPPED)
    {
        MSLOG_DEBUG("CDMSDetect::Start already running, just switch channel to %u\n", cfgUiChn);
        SwitchDmsChannel(cfgUiChn);
        return 0;
    }

    ResetChannelState();
    m_thread_quit = 0;
    m_threadState = DMS_THREAD_STATE_RUNNING;

    int ret = init();
    if (ret != 0)
        return ret;

    SwitchDmsChannel(cfgUiChn);
    return CLocalThread::Start("CDMSDetect");
}

int CDMSDetect::Stop()
{
    MSLOG_DEBUG("CDMSDetect::Stop in: video_chn=%d\n", m_dms_video_chn);

    m_thread_quit = 1;
    SetStatus(0);

    if (m_threadId != 0)
    {
        MSLOG_DEBUG("CDMSDetect::Stop: joining thread...\n");
        int waitCount = 0;
        while (m_threadState != DMS_THREAD_STATE_STOPPED && waitCount < 300)
        {
            delayMs(10);
            waitCount++;
        }
        if (m_threadState != DMS_THREAD_STATE_STOPPED)
            MSLOG_DEBUG("CDMSDetect::Stop: WARNING thread not stopped after 3s\n");

        pthread_join(m_threadId, NULL);
        m_threadId = 0;
        MSLOG_DEBUG("CDMSDetect::Stop: thread joined\n");
    }

    ResetAlarmState(true);
    StopAiThread();

    if (imageBuffer)
    {
        delete[] imageBuffer;
        imageBuffer = NULL;
    }
    if (ai_dms)
    {
        delete ai_dms;
        ai_dms = NULL;
    }

    c_RkRgaDeInit();
    sn = 0;
    ResetChannelState();

    pthread_mutex_lock(&s_faceLock);
    s_faceValid = false;
    s_smoothInited = false;
    s_nofaceCounter = 0;
    pthread_mutex_unlock(&s_faceLock);

    FreeColorBuffer();

    MSLOG_DEBUG("CDMSDetect::Stop out: video_chn=%d\n", m_dms_video_chn);
    return 0;
}

void CDMSDetect::quit()
{
    MSLOG_DEBUG("CDMSDetect::quit in\n");
    m_thread_quit = 1;
    SetStatus(0);
    MSLOG_DEBUG("CDMSDetect::quit out: m_thread_quit=%d status=%d\n",
                m_thread_quit, GetStatus());
}

int CDMSDetect::dynamicSetParamProcess(uint8_t param_dms_video_chn, uint8_t dms_encode_size)
{
    (void)dms_encode_size;
    MSLOG_DEBUG("dynamicSetParamProcess: request uiChn=%u\n", param_dms_video_chn);
    return SwitchDmsChannel(param_dms_video_chn);
}

int CDMSDetect::Pause()
{
    pthread_mutex_lock(&m_dms_mutex);
    MSLOG_DEBUG("CDMSDetect::Pause in: state=%d\n", m_threadState);
    m_threadState = DMS_THREAD_STATE_PAUSED;
    MSLOG_DEBUG("CDMSDetect::Pause out: state=%d\n", m_threadState);
    pthread_mutex_unlock(&m_dms_mutex);
    return 0;
}

int CDMSDetect::Resume()
{
    pthread_mutex_lock(&m_dms_mutex);
    MSLOG_DEBUG("CDMSDetect::Resume in: state=%d\n", m_threadState);
    m_threadState = DMS_THREAD_STATE_RUNNING;
    MSLOG_DEBUG("CDMSDetect::Resume out: state=%d\n", m_threadState);
    pthread_mutex_unlock(&m_dms_mutex);
    return 0;
}

DMS_ThreadState CDMSDetect::getThreadStates()
{
    pthread_mutex_lock(&m_dms_mutex);
    DMS_ThreadState s = m_threadState;
    pthread_mutex_unlock(&m_dms_mutex);
    return s;
}

DMS_ThreadState CDMSDetect::getThreadStateUnsafe()
{
    return m_threadState;
}

void CDMSDetect::PthreadProc()
{
    m_threadId = pthread_self();
    SetStatus(1);

    while (GetStatus() && !m_thread_quit)
    {
        DMS_ThreadState curState;
        pthread_mutex_lock(&m_dms_mutex);
        curState = m_threadState;
        pthread_mutex_unlock(&m_dms_mutex);

        if (curState == DMS_THREAD_STATE_PAUSED)
        {
            ProcessDmsAlarmState();
            delayMs(100);
            continue;
        }

        ApplySwitchChn();
        startDMSDetect();
        ProcessDmsAlarmState();
        delayMs(10);
    }

    ResetAlarmState(true);

    pthread_mutex_lock(&m_dms_mutex);
    m_threadState = DMS_THREAD_STATE_STOPPED;
    pthread_mutex_unlock(&m_dms_mutex);

    printf("=== Exit CDMSDetect::PthreadProc ===\n");
    MSLOG_DEBUG("Exit CDMSDetect::PthreadProc\n");
}

void CDMSDetect::OnDmsEventCallback(unsigned long long sn, int e, void *x1, void *x2)
{
    if (g_dms_self)
        g_dms_self->HandleDmsEvent(sn, e, x1, x2);
    else
        MSLOG_DEBUG("OnDmsEventCallback: g_dms_self is NULL, sn=%llu e=0x%x\n", sn, e);
}

void CDMSDetect::HandleDmsEvent(unsigned long long eventSn, int e, void *x1, void *x2)
{
    (void)x1;
    (void)x2;

    pthread_mutex_lock(&m_event_mutex);
    m_sn = eventSn;
    m_event |= e;
    m_event_time_us = local_get_curtime();
    pthread_mutex_unlock(&m_event_mutex);
}

bool CDMSDetect::IsAlarmEvent(int e)
{
    const int alarmMask = DMS::NOBODY | DMS::CAMERA_COVERED | DMS::LOOKING_DOWN |
                          DMS::LOOKING_AROUND | DMS::EYE_CLOSED | DMS::YAWN |
                          DMS::CALLING | DMS::SMOKING;
    return (e & alarmMask) != 0;
}

void CDMSDetect::PlayAlarmVoiceAndReport(unsigned long long eventSn, int e)
{
    CommonSet_t commonparam = {};
    AllConfigDef_t *pAllConfig = SystemGlobal_GetAllconfigDefContext();
    if (pAllConfig)
        commonparam.language = pAllConfig->commonset.language;

    PcmPlayer::Instance()->ClearTTS();

    if (e & DMS::NOBODY)
    {
        logw("[%d] [   ]", (int)eventSn);
        g_dms_alarm.event(DMS::NOBODY);
        if (commonparam.language == 1)
            PcmPlayer::Instance()->AddTTS("noperson");
        else if (commonparam.language == 5)
            PcmPlayer::Instance()->AddTTS("notdetected");
        else
            PcmPlayer::Instance()->AddTTS("镜头前无人");
    }
    else if (e & DMS::CAMERA_COVERED)
    {
        logw("[%d] [###]", (int)eventSn);
        g_dms_alarm.event(DMS::CAMERA_COVERED);
        if (commonparam.language == 1)
            PcmPlayer::Instance()->AddTTS("cover");
        else if (commonparam.language == 5)
            PcmPlayer::Instance()->AddTTS("cover");
        else
            PcmPlayer::Instance()->AddTTS("镜头遮挡");
    }
    else if (e & DMS::LOOKING_DOWN)
    {
        logw("[%d] [v_v]", (int)eventSn);
        g_dms_alarm.event(DMS::LOOKING_DOWN);
        if (commonparam.language == 1)
            PcmPlayer::Instance()->AddTTS("lookdown");
        else if (commonparam.language == 5)
            PcmPlayer::Instance()->AddTTS("Donotlookdown");
        else
            PcmPlayer::Instance()->AddTTS("低头");
    }
    else if (e & DMS::LOOKING_AROUND)
    {
        logw("[%d] [>_>]", (int)eventSn);
        g_dms_alarm.event(DMS::LOOKING_AROUND);
        if (commonparam.language == 1)
            PcmPlayer::Instance()->AddTTS("lookaround");
        else if (commonparam.language == 5)
            PcmPlayer::Instance()->AddTTS("Stayfocused");
        else
            PcmPlayer::Instance()->AddTTS("请勿分心");
    }
    else if (e & DMS::EYE_CLOSED)
    {
        logw("[%d] [-_-]", (int)eventSn);
        g_dms_alarm.event(DMS::EYE_CLOSED);
        if (commonparam.language == 1)
            PcmPlayer::Instance()->AddTTS("fatigue");
        else if (commonparam.language == 5)
            PcmPlayer::Instance()->AddTTS("fatigue");
        else
            PcmPlayer::Instance()->AddTTS("闭眼打哈欠");
    }
    else if (e & DMS::YAWN)
    {
        logw("[%d] [^0^]", (int)eventSn);
        g_dms_alarm.event(DMS::YAWN);
        if (commonparam.language == 1)
            PcmPlayer::Instance()->AddTTS("fatigue");
        else if (commonparam.language == 5)
            PcmPlayer::Instance()->AddTTS("fatigue");
        else
            PcmPlayer::Instance()->AddTTS("闭眼打哈欠");
    }
    else if (e & DMS::CALLING)
    {
        logw("[%d] [[] ]", (int)eventSn);
        g_dms_alarm.event(DMS::CALLING);
        if (commonparam.language == 1)
            PcmPlayer::Instance()->AddTTS("call");
        else if (commonparam.language == 5)
            PcmPlayer::Instance()->AddTTS("phone");
        else
            PcmPlayer::Instance()->AddTTS("打电话");
    }
    else if (e & DMS::SMOKING)
    {
        logw("[%d] [.__]", (int)eventSn);
        g_dms_alarm.event(DMS::SMOKING);
        if (commonparam.language == 1)
            PcmPlayer::Instance()->AddTTS("smoke");
        else if (commonparam.language == 5)
            PcmPlayer::Instance()->AddTTS("smoke");
        else
            PcmPlayer::Instance()->AddTTS("吸烟");
    }
    else
    {
        MSLOG_DEBUG("PlayAlarmVoiceAndReport: unknown event raw=0x%x\n", e);
    }
}

void CDMSDetect::ProcessDmsAlarmState()
{
    unsigned long long eventSn = 0;
    int e = 0;
    int filteredEvent = 0;
    long long eventTimeUs = 0;

    pthread_mutex_lock(&m_event_mutex);
    eventSn = m_sn;
    e = m_event;
    eventTimeUs = m_event_time_us;
    m_sn = 0;
    m_event = 0;
    m_event_time_us = 0;
    pthread_mutex_unlock(&m_event_mutex);

    filteredEvent = FilterAlarmEventBySpeed(e);

    if (e != 0 && filteredEvent == 0)
    {
        MSLOG_DEBUG("ProcessDmsAlarmState: raw event=0x%x ALL FILTERED OUT! "
                    "speed=%d alarmType=0x%x\n",
                    e, global_curSpeed, m_cachedAlarmType);
    }

    if (eventTimeUs > 0 && filteredEvent != 0)
    {
        bool needEnable = false;
        bool needStatusOn = false;
        bool needTts = false;
        uint8_t curUiChn = 0;
        uint8_t alarmChn = 0;

        pthread_mutex_lock(&m_dms_mutex);
        curUiChn = Ui_Current_Chn;
        pthread_mutex_unlock(&m_dms_mutex);

        pthread_mutex_lock(&m_event_mutex);

        m_last_alarm_time_us = eventTimeUs;

        if (!m_alarm_active)
        {
            m_alarm_active = true;
            m_alarm_ui_chn = curUiChn;
            needEnable = true;
        }
        if (!m_alarm_status_on)
        {
            m_alarm_status_on = true;
            needStatusOn = true;
        }
        if (eventTimeUs - m_last_tts_time_us >= 5000000)
        {
            m_last_tts_time_us = eventTimeUs;
            needTts = true;
        }
        alarmChn = m_alarm_ui_chn;
        pthread_mutex_unlock(&m_event_mutex);

        if (needStatusOn)
            CIDAConfig::Instance()->SetDMSAlarmFlag(true);

        if (needEnable)
        {
            MSLOG_DEBUG("ProcessDmsAlarmState: send ENABLE to UI, chn=%u, "
                        "event=0x%x filtered=0x%x speed=%d\n",
                        alarmChn, e, filteredEvent, global_curSpeed);
            MessageQueue_Send_Process(MSP_CMD_DMS_ALARM_ENABLE,
                                      (DVR_U8_T *)&alarmChn,
                                      sizeof(alarmChn),
                                      SEND_MESSAGE_TO_UI);
        }
        if (needTts)
            PlayAlarmVoiceAndReport(eventSn, filteredEvent);
    }

    long long now = local_get_curtime();
    bool needOff = false;
    bool needDisable = false;
    uint8_t alarmChn = 0;

    pthread_mutex_lock(&m_event_mutex);

    if (m_alarm_status_on &&
        m_last_alarm_time_us > 0 &&
        (now - m_last_alarm_time_us > ALARM_STATUS_LAST_TIME))
    {
        m_alarm_status_on = false;
        needOff = true;
    }
    if (m_alarm_active &&
        m_last_alarm_time_us > 0 &&
        (now - m_last_alarm_time_us > ALARM_WIND_POP_LAST_TIME))
    {
        m_alarm_active = false;
        alarmChn = m_alarm_ui_chn;
        m_alarm_ui_chn = 0;
        m_last_alarm_time_us = 0;
        m_last_tts_time_us = 0;
        needDisable = true;
        needOff = true;
    }
    pthread_mutex_unlock(&m_event_mutex);

    if (needOff)
    {
        MSLOG_DEBUG("ProcessDmsAlarmState: SetDMSAlarmFlag(false)\n");
        CIDAConfig::Instance()->SetDMSAlarmFlag(false);
        PcmPlayer::Instance()->ClearTTS();
    }
    if (needDisable)
    {
        MSLOG_DEBUG("ProcessDmsAlarmState: send DISABLE to UI, chn=%u\n", alarmChn);
        MessageQueue_Send_Process(MSP_CMD_DMS_ALARM_DISABLE,
                                  (DVR_U8_T *)&alarmChn, sizeof(alarmChn),
                                  SEND_MESSAGE_TO_UI);
    }
}

void CDMSDetect::ResetAlarmState(bool notifyUi)
{
    bool needDisable = false;
    bool needOff = false;
    uint8_t alarmChn = 0;

    pthread_mutex_lock(&m_event_mutex);

    needDisable = m_alarm_active;
    needOff = (m_alarm_active || m_alarm_status_on);
    alarmChn = m_alarm_ui_chn;

    m_alarm_active = false;
    m_alarm_status_on = false;
    m_last_alarm_time_us = 0;
    m_last_tts_time_us = 0;
    m_alarm_ui_chn = 0;
    m_event = 0;
    m_sn = 0;
    m_event_time_us = 0;

    pthread_mutex_unlock(&m_event_mutex);

    if (needOff)
    {
        CIDAConfig::Instance()->SetDMSAlarmFlag(false);
        PcmPlayer::Instance()->ClearTTS();
    }
    if (notifyUi && needDisable)
    {
        MSLOG_DEBUG("ResetAlarmState: send DISABLE to UI, chn=%u\n", alarmChn);
        MessageQueue_Send_Process(MSP_CMD_DMS_ALARM_DISABLE,
                                  (DVR_U8_T *)&alarmChn, sizeof(alarmChn),
                                  SEND_MESSAGE_TO_UI);
    }
}

#define FACE_BOX_MAX_HOLD_US 2000000

void CDMSDetect::GetVi()
{
    ABDParam_t abdParam;
    AllConfigDef_t *pAllConfig = SystemGlobal_GetAllconfigDefContext();
    DVR_GetIdaPara(&abdParam, &pAllConfig->abdparam);

    VIDEO_FRAME_INFO_S stFrame;
    memset(&stFrame, 0, sizeof(stFrame));

    IDA_FRAME_POSITION sDMSFaceInfoFrame;
    memset(&sDMSFaceInfoFrame, 0, sizeof(IDA_FRAME_POSITION));

    IDA_Info_Speed info_speed;
    SSYFI_GPS m_gps;
    memset(&m_gps, 0, sizeof(SSYFI_GPS));
    GetBackBoardGpsData(&m_gps);
    info_speed.gpsSpeed = m_gps.kiloSpeed;
    info_speed.curSpeed = m_gps.kiloSpeed;
    global_curSpeed = info_speed.curSpeed;

    for (int i = 0; i < EVT_IDX_COUNT; i++)
    {
        int paramIdx = ALARM_PARAM_INDEX[i];
        m_cachedSpeedThreshold[i] = abdParam.IdaCfgSet.dmsConfig.AlarmParam[paramIdx].speedThreshold;
        global_speedThreshold[i] = m_cachedSpeedThreshold[i];
    }
    m_cachedAlarmType = abdParam.IdaCfgSet.dmsConfig.alarmType;

    int viChn;
    pthread_mutex_lock(&m_dms_mutex);
    viChn = m_dms_video_chn;
    pthread_mutex_unlock(&m_dms_mutex);

    if (viChn < 0)
    {
        delayMs(50);
        return;
    }

    RK_S32 ret = RK_MPI_VI_GetChnFrame(DMS_VI_DEV_ID, viChn, &stFrame, 100);
    if (ret != RK_SUCCESS)
    {
        MSLOG_DEBUG("RK_MPI_VI_GetChnFrame failed ret=%d viChn=%d\n", ret, viChn);
        return;
    }

    void *nv12 = RK_MPI_MB_Handle2VirAddr(stFrame.stVFrame.pMbBlk);
    if (!nv12)
    {
        RK_MPI_VI_ReleaseChnFrame(DMS_VI_DEV_ID, viChn, &stFrame);
        return;
    }

    int width     = stFrame.stVFrame.u32Width;
    int height    = stFrame.stVFrame.u32Height;
    int fmt       = stFrame.stVFrame.enPixelFormat;
    int virWidth  = stFrame.stVFrame.u32VirWidth;
    int virHeight = stFrame.stVFrame.u32VirHeight;
    if (virWidth <= 0)  virWidth = width;
    if (virHeight <= 0) virHeight = height;

    if (fmt != RK_FMT_YUV420SP)
    {
        MSLOG_DEBUG("VI fmt is not NV12/YUV420SP, fmt=%d\n", fmt);
        RK_MPI_VI_ReleaseChnFrame(DMS_VI_DEV_ID, viChn, &stFrame);
        return;
    }

    bool aiIdle = false;
    pthread_mutex_lock(&s_aiLock);
    aiIdle = (!s_aiBusy && !s_aiNewData);
    pthread_mutex_unlock(&s_aiLock);

    if (aiIdle)
    {
        if (Nv12ToRgb888(nv12, width, height))
        {
            SubmitToAiThread(imageBuffer, width, height);
        }
    }

    bool      snapValid  = false;
    int       snapLeft   = 0;
    int       snapTop    = 0;
    int       snapRight  = 0;
    int       snapBottom = 0;
    long long snapTimeUs = 0;

    pthread_mutex_lock(&s_faceLock);
    snapValid  = s_faceValid;
    snapLeft   = s_faceLeft;
    snapTop    = s_faceTop;
    snapRight  = s_faceRight;
    snapBottom = s_faceBottom;
    snapTimeUs = s_lastFaceDetectedUs;
    pthread_mutex_unlock(&s_faceLock);

    if (snapValid)
    {
        long long now = local_get_curtime();
        if (now - snapTimeUs > FACE_BOX_MAX_HOLD_US)
        {
            snapValid = false;
            pthread_mutex_lock(&s_faceLock);
            s_faceValid = false;
            s_smoothInited = false;
            s_nofaceCounter = 0;
            pthread_mutex_unlock(&s_faceLock);
        }
    }

    if (snapValid)
    {
        DrawBoxOnNv12(nv12, virWidth, virHeight, width, height,
                      snapLeft, snapTop, snapRight, snapBottom,
                      10, 255, 0, 0);

        sDMSFaceInfoFrame.bValid   = true;
        sDMSFaceInfoFrame.iLeft    = snapLeft;
        sDMSFaceInfoFrame.iRight   = snapRight;
        sDMSFaceInfoFrame.iTop     = snapTop;
        sDMSFaceInfoFrame.iBottom  = snapBottom;
    }

    CIDAConfig::Instance()->SetDMSFrame(sDMSFaceInfoFrame);

    RK_MPI_VI_ReleaseChnFrame(DMS_VI_DEV_ID, viChn, &stFrame);
}

int CDMSDetect::startDMSDetect()
{
    GetVi();
    return 0;
}
#endif
