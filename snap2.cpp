#include "snap.h"

#include "ms_netdvr_vi.h"
#include "ms_netdvr_common.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

#include <string>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <cstdint>

static const int SNAP_TIMEOUT_MS = 1000;
#define SNAP_BASE_DIR "/userdata/ADAS_Alarm"
struct SnapContext
{
    int         channel        = -1;  
    std::string filePath;            

    std::mutex              mtx;
    std::condition_variable cv;

    bool        gotFrame      = false; 
    int         result        = -1;  

    bool        cbRegistered  = false; 
    bool        cbSupported   = true;  
};

static SnapContext       g_snapCtx[DEVICE_CHANNEL_NUM];
static VIDEO_CB_PARAM_S  g_cbParam[DEVICE_CHANNEL_NUM];
static std::once_flag    g_cbParamInitFlag;
static std::mutex        g_regMtx;

static std::string MakeSnapFilePath(int ch);
static int SnapJpegCallback(uint8_t u8Chn, void* pBuf, int nBufLen, int nWidth, int nHeight, void* args);
static int EnsureDir(const char *dir)
{
    struct stat st;
    if (stat(dir, &st) == 0)
    {
        if (S_ISDIR(st.st_mode))
        {
            return 0; 
        }
        else
        {
            printf("EnsureDir: %s exists but is not a directory\n", dir);
            return -1;
        }
    }

    int ret = mkdir(dir, 0777);
    if (ret != 0 && errno != EEXIST)
    {
        printf("EnsureDir: mkdir %s failed, errno=%d\n", dir, errno);
        return -1;
    }
    return 0;
}

static void InitCbParamArray()
{
    for (int ch = 0; ch < DEVICE_CHANNEL_NUM; ++ch)
    {
        memset(&g_cbParam[ch], 0, sizeof(VIDEO_CB_PARAM_S));
        g_cbParam[ch].stVideoType = MS_VI_BYPASS_JPEG;
        g_cbParam[ch].stJpegAttr.stSize.u32Width  = 1280;
        g_cbParam[ch].stJpegAttr.stSize.u32Height = 720;
        g_cbParam[ch].stJpegAttr.u32Qfactor       = 70;    
        g_cbParam[ch].stJpegAttr.bMirror          = false;
        g_cbParam[ch].stJpegAttr.bFlip            = false;
        g_cbParam[ch].stJpegAttr.u32RecvPicNum    = 1;   

        g_snapCtx[ch].channel = ch;
    }
}

static int EnsureSnapCallbackRegistered(int ch)
{
    if (ch < 0 || ch >= DEVICE_CHANNEL_NUM)
        return -1;
    std::call_once(g_cbParamInitFlag, InitCbParamArray);

    SnapContext& ctx = g_snapCtx[ch];
    if (!ctx.cbSupported)
    {
        printf("EnsureSnapCallbackRegistered: channel %d not supported (previous failure)\n", ch);
        return -2;
    }

    if (ctx.cbRegistered)
        return 0;

    std::lock_guard<std::mutex> regLock(g_regMtx);

    if (ctx.cbRegistered)
        return 0;

    int ret = MsNetdvr_Vi_RegistVideoCallback(
        (uint8_t)ch,
        &g_cbParam[ch],
        SnapJpegCallback,
        &ctx
    );

    if (ret != 0)
    {
        ctx.cbSupported = false;
        printf("EnsureSnapCallbackRegistered: MsNetdvr_Vi_RegistVideoCallback failed, ch=%d, ret=%d\n",
               ch, ret);
        return ret;
    }

    ctx.cbRegistered = true;
    printf("EnsureSnapCallbackRegistered: channel %d registered\n", ch);
    return 0;
}

static std::string MakeSnapFilePath(int ch)
{
    std::string base = SNAP_BASE_DIR;
    if (!base.empty() && base.back() != '/' && base.back() != '\\')
    {
        base.push_back('/');
    }

    if (EnsureDir(base.c_str()) != 0)
    {
        printf("MakeSnapFilePath: EnsureDir(%s) failed, snap may fail\n", base.c_str());
    }

    char fileName[128] = {0};

    struct timeval tv;
    gettimeofday(&tv, NULL);

    time_t now = tv.tv_sec;
    struct tm t;
    memset(&t, 0, sizeof(t));
    localtime_r(&now, &t);

    int ms = tv.tv_usec / 1000; 

    snprintf(
        fileName,
        sizeof(fileName),
        "ch%d_%04d%02d%02d_%02d%02d%02d_%03d.jpg",
        ch,
        t.tm_year + 1900,
        t.tm_mon + 1,
        t.tm_mday,
        t.tm_hour,
        t.tm_min,
        t.tm_sec,
        ms
    );

    std::string fullPath = base + fileName;
    return fullPath;
}

static int SnapJpegCallback(uint8_t u8Chn, void* pBuf, int nBufLen, int nWidth, int nHeight, void* args)
{
    SnapContext* ctx = (SnapContext*)args;
    if (!ctx)
        return -1;

    std::unique_lock<std::mutex> lk(ctx->mtx);

    if (ctx->gotFrame)
    {
        return 0;
    }

    if (!pBuf || nBufLen <= 0)
    {
        ctx->result   = -2;
        ctx->gotFrame = true;
        lk.unlock();
        ctx->cv.notify_one();
        printf("SnapJpegCallback: invalid buffer, ch=%d\n", u8Chn);
        return -2;
    }

    FILE* fp = fopen(ctx->filePath.c_str(), "wb");
    if (!fp)
    {
        ctx->result   = -3;
        ctx->gotFrame = true;
        lk.unlock();
        ctx->cv.notify_one();
        printf("SnapJpegCallback: fopen failed, path=%s\n", ctx->filePath.c_str());
        return -3;
    }

    size_t written = fwrite(pBuf, 1, (size_t)nBufLen, fp);
    fclose(fp);

    if (written != (size_t)nBufLen)
    {
        ctx->result = -4;
        printf("SnapJpegCallback: fwrite short, ch=%d, len=%d, written=%zu\n",
               u8Chn, nBufLen, written);
    }
    else
    {
        ctx->result = 0;
        printf("SnapJpegCallback: success, ch=%d, file=%s, size=%d bytes (%dx%d)\n",
               u8Chn, ctx->filePath.c_str(), nBufLen, nWidth, nHeight);
    }

    ctx->gotFrame = true;
    lk.unlock();
    ctx->cv.notify_one();

    return ctx->result;
}

int SnapOneChannel(int ch)
{
    if (ch < 0 || ch >= DEVICE_CHANNEL_NUM)
    {
        printf("SnapOneChannel: invalid channel %d\n", ch);
        return -1;
    }

    SnapContext& ctx = g_snapCtx[ch];

    {
        std::lock_guard<std::mutex> lk(ctx.mtx);
        ctx.filePath = MakeSnapFilePath(ch);
        ctx.gotFrame = false;
        ctx.result   = -1;
    }

    int ret = EnsureSnapCallbackRegistered(ch);
    if (ret != 0)
    {
        printf("SnapOneChannel: EnsureSnapCallbackRegistered failed, ch=%d, ret=%d\n",
               ch, ret);
        return -2;
    }

    {
        std::unique_lock<std::mutex> lk(ctx.mtx);
        bool ok = ctx.cv.wait_for(
            lk,
            std::chrono::milliseconds(SNAP_TIMEOUT_MS),
            [&ctx]() { return ctx.gotFrame; }
        );

        if (!ok)
        {
            ctx.result = -5;
            printf("SnapOneChannel: timeout waiting JPEG frame, ch=%d\n", ch);
        }
    }

    if (ctx.result == 0)
    {
        printf("SnapOneChannel: success, ch=%d, file=%s\n",
               ch, ctx.filePath.c_str());
    }
    else
    {
        printf("SnapOneChannel: failed, ch=%d, err=%d\n", ch, ctx.result);
    }

    return ctx.result;
}


int SnapChannels_0_to_5(void)
{
    int lastErr = 0;

    for (int ch = 0; ch < 6; ++ch)
    {
        int ret = SnapOneChannel(ch);
        if (ret != 0)
        {
            lastErr = ret;
        }
    }

    return lastErr;
}

const char* SnapGetLastFilePath(int ch)
{
    if (ch < 0 || ch >= DEVICE_CHANNEL_NUM)
        return NULL;
    return g_snapCtx[ch].filePath.c_str();
}
