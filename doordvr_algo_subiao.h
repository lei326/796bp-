#ifndef  __DOORDVR_ALGO_SUBIAO_H__
#define __DOORDVR_ALGO_SUBIAO_H__

//#include "ll_headers.h"
#include "tools.h"
#include "device.h"

typedef struct{
    uint8_t devicetype; //设备类型            0x64 ADAS(高级驾驶辅助系统)         0x65 DSM (驾驶员状态监测系统) 0x66 TPMS(轮胎气压检测系统) 0x67 BSD(盲点监测系统)
    uint8_t alarmtype;  //报警类型

    uint8_t alarmstatus;  //标志状态        BYTE     0x01开始    0x02 结束
    uint8_t alarmlevel;   //报警级别
    uint8_t alarmchannel;  //报警通道

    uint8_t fatigueDegree; //疲劳程度 DSM
    uint8_t attachmentnum;  //附件个数
    uint8_t speed;
    time_t alarmtime;    
    char attachfile[5][128];
                 
}AiAlarmItem;

// 事件通知函数

int (AiEventCallback)(AiAlarmItem *item, void *args);
int  anologDmsSnap(void);

int   SpeedReporting(void);
long long dir_size(const char *path);

void *pthread_Snap_Manage(void *args);
int  anologDmsSnap(void);
int  zjAlarmRecord(int8_t channel);

class AlarmMutimedia
{
public:
	AlarmMutimedia(void);
	~AlarmMutimedia(void);
public:

    int Doordvr_Snap_StartJpeg(VI_CHN channel, PIC_SIZE_E picSize,AiAlarmItem *param);
    int Doordvr_Record_StartMp4(VI_CHN channel, DVR_U8_T encoder_size,AiAlarmItem *param);

	static AlarmMutimedia *m_pInstance;
	static AlarmMutimedia *Instance();
    void  AlgoAlarmStatus(uint8_t alarmtype, uint8_t alarmstatus);
    void  NotWearSeatBeltAlarmFinished();
    bool  GetHaveRecordFlag();
    void   SetHaveRecoedFlag(bool flag);
public:
 	AiAlarmItem  m_event_alarm;
private:
    int32_t CreateMediaDir(char *dir);
    int MakeDirectory(char * _pcPath, int _iMode);

private:
	char m_acH264Path[128];
    bool m_encodeOnceFlag;
    bool  m_haveRecordFlag;

};

#endif
