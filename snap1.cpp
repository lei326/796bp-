//更新抓拍实现
#include "snap.h"

#include "ms_netdvr_vi.h"
#include "ms_netdvr_common.h"

#include <cstdio>    
#include <cstring>              
#include <ctime>           
#include <string>               
#include <mutex>               
#include <condition_variable>  
#include <chrono>              
#include <cstdint>            
#include <unistd.h> 
#include <sys/time.h> 
static const int SNAP_TIMEOUT_MS = 1000;

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

static SnapContext g_snapCtx[DEVICE_CHANNEL_NUM];
static VIDEO_CB_PARAM_S g_cbParam[DEVICE_CHANNEL_NUM];
static std::once_flag g_cbParamInitFlag;
static std::mutex g_regMtx;

static std::string MakeSnapFilePath(int ch)
{
    char fileName[128] = {0};

    struct timeval tv;
    gettimeofday(&tv, nullptr);

    time_t now = tv.tv_sec;
    struct tm t;
    std::memset(&t, 0, sizeof(t));
    localtime_r(&now, &t);

    int ms = tv.tv_usec / 1000; 

    std::snprintf(
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

    std::string fullPath = "/opt/796bp/";
    fullPath += fileName;
    return fullPath;
}


static int SnapJpegCallback(uint8_t u8Chn,
                            void* pBuf,
                            int nBufLen,
                            int nWidth,
                            int nHeight,
                            void* args);


static void InitCbParamArray()
{
    for (int ch = 0; ch < DEVICE_CHANNEL_NUM; ++ch)
    {
        std::memset(&g_cbParam[ch], 0, sizeof(VIDEO_CB_PARAM_S));
        g_cbParam[ch].stVideoType = MS_VI_BYPASS_JPEG;
        g_cbParam[ch].stJpegAttr.stSize.u32Width  = 1280;
        g_cbParam[ch].stJpegAttr.stSize.u32Height = 720;
        g_cbParam[ch].stJpegAttr.u32Qfactor    = 70;    
        g_cbParam[ch].stJpegAttr.bMirror       = false;
        g_cbParam[ch].stJpegAttr.bFlip         = false;
        g_cbParam[ch].stJpegAttr.u32RecvPicNum = 1;    
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
        std::printf("EnsureSnapCallbackRegistered: channel %d not supported (previous failure)\n", ch);
        return -2;
    }

    if (ctx.cbRegistered)
        return 0;

    std::lock_guard<std::mutex> regLock(g_regMtx);

    if (ctx.cbRegistered)
        return 0;

    int ret = MsNetdvr_Vi_RegistVideoCallback(
        static_cast<uint8_t>(ch),
        &g_cbParam[ch],     
        SnapJpegCallback,
        &ctx                   
    );

    if (ret != 0)
    {
        ctx.cbSupported = false;
        std::printf("EnsureSnapCallbackRegistered: MsNetdvr_Vi_RegistVideoCallback failed, ch=%d, ret=%d\n",
                    ch, ret);
        return ret;
    }

    ctx.cbRegistered = true;
    std::printf("EnsureSnapCallbackRegistered: channel %d registered\n", ch);
    return 0;
}

static int SnapJpegCallback(uint8_t u8Chn, void* pBuf, int nBufLen, int nWidth, int nHeight, void* args)
{
    SnapContext* ctx = static_cast<SnapContext*>(args);
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
        std::printf("SnapJpegCallback: invalid buffer, ch=%d\n", u8Chn);
        return -2;
    }

    FILE* fp = std::fopen(ctx->filePath.c_str(), "wb");
    if (!fp)
    {
        ctx->result   = -3;
        ctx->gotFrame = true;
        lk.unlock();
        ctx->cv.notify_one();
        std::printf("SnapJpegCallback: fopen failed, path=%s\n", ctx->filePath.c_str());
        return -3;
    }

    size_t written = std::fwrite(pBuf, 1, static_cast<size_t>(nBufLen), fp);
    std::fclose(fp);

    if (written != static_cast<size_t>(nBufLen))
    {
        ctx->result = -4;
        std::printf("SnapJpegCallback: fwrite short, ch=%d, len=%d, written=%zu\n",
                    u8Chn, nBufLen, written);
    }
    else
    {
        ctx->result = 0;
        std::printf("SnapJpegCallback: success, ch=%d, file=%s, size=%d bytes (%dx%d)\n",
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
        std::printf("SnapOneChannel: invalid channel %d (valid: 0~%d)\n",
                    ch, DEVICE_CHANNEL_NUM - 1);
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
        std::printf("SnapOneChannel: EnsureSnapCallbackRegistered failed, ch=%d, ret=%d\n",
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
            std::printf("SnapOneChannel: timeout waiting JPEG frame, ch=%d\n", ch);
        }
    }

    if (ctx.result == 0)
    {
        std::printf("SnapOneChannel: success, ch=%d, file=%s\n",
                    ch, ctx.filePath.c_str());
    }
    else
    {
        std::printf("SnapOneChannel: failed, ch=%d, err=%d\n", ch, ctx.result);
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
