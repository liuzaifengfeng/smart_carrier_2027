#include "../src/serial_frame.h"
#include "../src/continuous_trial.h"
#include <stdlib.h>
NativeSerial Serial;
static bool serialDebugMode=true,active=false;
static int begins=0,nodes=0,launches=0,allowed=0,policies=0;
static const char *prepareError=nullptr;
static continuous::Config captured;
static bool alignmentEnabled=true;
static int xAlignmentMotionMutex=1,locks=0,handled=0;
static bool capturedWritable=false;
constexpr int portMAX_DELAY=1,pdTRUE=1;
int xSemaphoreTake(int,int){++locks;return pdTRUE;}
void xSemaphoreGive(int){}
void serialCfgError(const SerialFrame &frame,const char *reason){serialError(frame,reason);}
bool HandleRuntimeParameters(const char *const *,size_t,bool writable){
    ++handled;capturedWritable=writable;return true;
}
bool ContinuousTrialActive(){return active;}
const char *ContinuousTrialBegin(const continuous::Config &c){captured=c;++begins;return nullptr;}
const char *ContinuousTrialPolicy(uint32_t id,uint8_t mode,float outset,float margin,uint32_t geometry){
    if(id!=42||mode!=1||outset!=60||margin!=20||geometry!=1831827565u)return "POLICY_TEST";
    ++policies;return nullptr;
}
const char *ContinuousTrialNode(uint32_t id,uint8_t i,uint8_t n,float r,float x,float y){
    if(id!=42||i!=0||n!=0||r!=0||x!=310||y!=240)return "NODE_TEST";
    ++nodes;return nullptr;
}
const char *ContinuousTrialPrepare(uint32_t){return prepareError;}
void ContinuousTrialLaunch(){
    if(Serial.text.find("{RSP,NAV,TRIAL_START,ACK,42}")==std::string::npos)exit(10);
    ++launches;
}
const char *ContinuousTrialStop(uint32_t){return nullptr;}
const char *ContinuousTrialStatus(uint32_t){return nullptr;}

// 运行测试会从实际 serial_commands.inc 生成这两个函数，避免复制协议逻辑。
#include "trial_dispatch_actual.inc"

#define CHECK(x) do {if(!(x)){fprintf(stderr,"dispatch check line %d: %s\n",__LINE__,#x);return 2;}} while(0)
static bool dispatch(const char *text,bool testGate=false,bool testCfg=false){
    char line[160];strncpy(line,text,sizeof(line));line[159]=0;SerialFrame f;
    if(!parseSerialFrame(line,f))return false;
    if(testGate){trialGate(f);return true;}
    if(testCfg){trialCfgDispatch(f);return true;}
    return serialTrial(f);
}
int main(){
    CHECK(dispatch("{CMD,NAV,TRIAL_BEGIN,42,3,180,270,80,200,300,13.3,13.6,2,45,20}"));
    CHECK(begins==1&&captured.id==42&&captured.count==3&&captured.endHeading==270);
    CHECK(Serial.text.find("TRIAL_BEGIN,ACK,42")!=std::string::npos);
    CHECK(dispatch("{CMD,NAV,TRIAL_POLICY,42,1,60,20,1831827565}"));CHECK(policies==1);
    CHECK(dispatch("{CMD,NAV,TRIAL_POLICY,42,1,60,20,4294967296}"));CHECK(policies==1);
    CHECK(dispatch("{CMD,NAV,TRIAL_NODE,42,0,0,0,310,240}"));CHECK(nodes==1);
    CHECK(dispatch("{CMD,NAV,TRIAL_START,42}"));CHECK(launches==1);
    prepareError="NOT_CALIBRATED";Serial.text.clear();
    CHECK(dispatch("{CMD,NAV,TRIAL_START,42}"));CHECK(launches==1);
    CHECK(Serial.text.find("TRIAL_START,ERR,42,NOT_CALIBRATED")!=std::string::npos);
    Serial.text.clear();CHECK(dispatch("{CMD,NAV,TRIAL_BEGIN,42,3,180,270,80,200,300,13.3,13.6,2,10,20}"));
    CHECK(begins==1&&Serial.text.find("ERR,RANGE")!=std::string::npos);
    CHECK(dispatch("{CMD,NAV,TRIAL_NODE,42,0,0,120}"));CHECK(nodes==1);
    serialDebugMode=false;Serial.text.clear();CHECK(dispatch("{CMD,NAV,TRIAL_START,42}"));
    CHECK(Serial.text.find("ERR,MODE")!=std::string::npos);serialDebugMode=true;
    active=true;
    for(const char *text:{"{CMD,CHASSIS,MOVE,0,80,0}","{CMD,POSE,SET,310,240,180}",
        "{CMD,SYS,RELEASE}","{CMD,VISION,ALIGN_START}","{CMD,ARM,INIT_START}","{CMD,CFG,COMMIT,1}"}) {
        Serial.text.clear();CHECK(dispatch(text,true));CHECK(allowed==0);
        CHECK(Serial.text.find("ERR,BUSY")!=std::string::npos);
    }
    for(const char *text:{"{CMD,IMU,GET}","{CMD,IMU,HEADING}","{CMD,NAV,TRIAL_STOP,42}",
        "{CMD,NAV,TRIAL_STATUS,42}","{CMD,SYS,HELLO}"})CHECK(dispatch(text,true));
    CHECK(allowed==5);
    active=false;
    // 上电默认视觉开启：只读回读必须成功，且不占用视觉运动锁。
    CHECK(dispatch("{CMD,CFG,GET,42}",false,true));
    CHECK(handled==1&&locks==0&&!capturedWritable&&alignmentEnabled);
    Serial.text.clear();CHECK(dispatch("{CMD,CFG,BEGIN,42}",false,true));
    CHECK(handled==1&&locks==0&&Serial.text.find("BUSY_OR_MODE")!=std::string::npos);
    alignmentEnabled=false;
    CHECK(dispatch("{CMD,CFG,BEGIN,42}",false,true));
    CHECK(handled==2&&locks==1&&capturedWritable);
    alignmentEnabled=true;serialDebugMode=false;
    CHECK(dispatch("{CMD,CFG,GET,42}",false,true));
    CHECK(handled==3&&locks==1&&!capturedWritable&&alignmentEnabled);
    printf("dispatch and active motion gate OK\n");return 0;
}
