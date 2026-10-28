#include "turret_motor.h"
#include "emm_feedback.h"
#include <assert.h>
#include <math.h>
#include <string.h>

NativeSerial Serial;
int64_t measuredPosition=0;
uint16_t measuredEncoder=0;
int16_t measuredRpm=0;uint8_t measuredFlags=3;
uint32_t clockMs=0;uint32_t millis(){return clockMs;}
bool flashHasPid=false,pidWriteAvailable=true;
uint32_t flashPid[3]={},driverPid[3]={62000,100,62000};
int pidBaselineWrites=0,pidWrites=0;
bool available=true,flashHasZero=false,storeAvailable=true,writeAvailable=true;
uint16_t flashZero=0;
int writes=0,zeroCommands=0,moves=0,transactionDepth=0;
bool queryGap=false;
uint8_t lastId=0,direction=0,acceleration=0;
uint16_t speed=0;
uint32_t targetPulses=0;
bool absolute=false,sync=true;
struct MotorBusGuard {MotorBusGuard(){++transactionDepth;}~MotorBusGuard(){--transactionDepth;}};
void vTaskDelay(uint32_t ms){assert(transactionDepth>0&&ms==5);queryGap=true;clockMs+=ms;}
class Preferences {
public:
    bool begin(const char *name,bool readOnly){assert(!strcmp(name,"turret"));return storeAvailable&&(!readOnly||flashHasZero);}
    void end(){}
    size_t getBytes(const char *key,void *data,size_t size){assert(!strcmp(key,"pid_base")&&size==sizeof(flashPid));memcpy(data,flashPid,size);return size;}
    size_t putBytes(const char *key,const void *data,size_t size){assert(!strcmp(key,"pid_base")&&size==sizeof(flashPid));memcpy(flashPid,data,size);flashHasPid=true;++pidBaselineWrites;return size;}
    bool isKey(const char *key){if(!strcmp(key,"pid_base"))return flashHasPid;assert(!strcmp(key,"zero_enc"));return flashHasZero;}
    uint16_t getUShort(const char *key){assert(!strcmp(key,"zero_enc"));return flashZero;}
    size_t putUShort(const char *key,uint16_t value){
        assert(!strcmp(key,"zero_enc"));if(!writeAvailable)return 0;
        flashZero=value;flashHasZero=true;++writes;return sizeof(value);
    }
};
const char *ReadMotorEncoderSnapshot(uint8_t id,uint16_t &encoder,int64_t &position,int16_t &rpm,uint8_t &flags){
    assert(transactionDepth>0&&queryGap);queryGap=false;lastId=id;
    encoder=measuredEncoder;position=measuredPosition;rpm=measuredRpm;flags=measuredFlags;return available?nullptr:"TIMEOUT";
}
const char *ReadMotorPositionPid(uint8_t id,uint32_t (&values)[3]){assert(id==7&&transactionDepth>0&&queryGap);queryGap=false;memcpy(values,driverPid,sizeof(driverPid));return nullptr;}
const char *WriteMotorPositionPid(uint8_t id,const uint32_t (&values)[3]){assert(id==7&&transactionDepth>0);if(!pidWriteAvailable)return "REJECTED";memcpy(driverPid,values,sizeof(driverPid));++pidWrites;return nullptr;}
const char *ZeroMotorEncoderPosition(uint8_t id){assert(transactionDepth>0);lastId=id;++zeroCommands;measuredPosition=0;return nullptr;}
void Emm_V5_Pos_Control(uint8_t id,uint8_t dir,uint16_t rpm,uint8_t acc,uint32_t pulses,bool ra,bool sn){
    ++moves;lastId=id;direction=dir;speed=rpm;acceleration=acc;targetPulses=pulses;absolute=ra;sync=sn;
}
#include "turret_motor_actual.inc"

void snapshot(int64_t originCounts,int64_t position){
    // 电机上电原点位于机械originCounts位置；原始编码器CW为正，机械角CCW为正。
    measuredEncoder=static_cast<uint16_t>(int64_t(flashZero)-originCounts+position);
    measuredPosition=position;
}
int main(){
    float angle=777;
    assert(!TurretMotorReadAngle(angle)&&angle==777&&moves==0);
    measuredEncoder=65500;measuredPosition=12345;
    assert(TurretMotorSetZero(angle)&&angle==0&&lastId==7);
    assert(flashHasZero&&flashZero==65500&&writes==1&&zeroCommands==1&&moves==0&&!transactionDepth);
    snapshot(0,0);measuredEncoder=static_cast<uint16_t>(measuredEncoder+1);
    assert(TurretMotorReadAngle(angle)&&angle<0&&angle>-0.01f); // 零位误差一格不能被重建成接近360度。
    // 主控重启：只清掉RAM状态，模拟NVS仍保留。电机此时在90度上电。
    zeroLoaded=false;zeroEncoder=0;motorOriginAngle=999;
    snapshot(16384,0);
    assert(TurretMotorReadAngle(angle)&&angle==90&&writes==1&&zeroCommands==1);
    TurretMotorMove(0,20);
    assert(direction==0&&targetPulses==800&&speed==83&&acceleration==126&&absolute&&!sync);
    TurretMotorMove(360,500);
    assert(direction==1&&targetPulses==2400&&acceleration==126&&absolute);
    assert(driverPid[0]==62000&&driverPid[1]==100&&driverPid[2]==62000&&pidBaselineWrites==0&&pidWrites==0);
    driverPid[2]=20000;
    assert(TurretMotorMove(360,500)&&driverPid[2]==20000&&pidWrites==0);
    snapshot(16384,-49152);
    assert(TurretMotorReadAngle(angle)&&angle==360);
    TurretMotorMove(0,20);assert(direction==0&&targetPulses==800&&absolute); // 从360返回0，不就近取等效角。
    snapshot(16384,-65536);
    assert(TurretMotorReadAngle(angle)&&angle==450); // 完整累计圈数继续保留。
    snapshot(16384,32768);
    assert(TurretMotorReadAngle(angle)&&angle==-90);
    // 只给电机断电：主控RAM不复位，180度重新上电后仍恢复正确坐标。
    snapshot(32768,0);
    assert(TurretMotorReadAngle(angle)&&angle==180);
    TurretMotorMove(90,600);assert(direction==0&&targetPulses==800&&acceleration==126);
    available=false;angle=777;
    assert(!TurretMotorReadAngle(angle)&&angle==777);
    const int previousZeros=zeroCommands;
    assert(!TurretMotorSetZero(angle)&&zeroCommands==previousZeros);
    available=true;writeAvailable=false;
    assert(!TurretMotorSetZero(angle)&&angle==777&&writes==1);
    writeAvailable=true;
    assert(TurretMotorSetZero(angle)&&angle==0&&writes==2&&!transactionDepth);
    TurretMotorMove(-90,20);assert(direction==0&&targetPulses==800);
    TurretMotorMove(360,20);assert(direction==1&&targetPulses==3200);
    TurretMotorMove(0,20);assert(targetPulses==0&&absolute);
    assert(TurretMotorMove(90,20));
    const int commandsBeforeTrace=moves;
    snapshot(0,-17476);measuredRpm=-5;measuredFlags=1;clockMs+=50;
    TurretMotorPoll();assert(TurretMotorTracking()&&trace.overshoot>5.9f&&moves==commandsBeforeTrace);
    snapshot(0,-16384);measuredRpm=0;measuredFlags=3;clockMs+=50;
    TurretMotorPoll();assert(TurretMotorTracking()&&moves==commandsBeforeTrace);
    clockMs+=50;TurretMotorPoll();clockMs+=50;TurretMotorPoll();
    assert(!TurretMotorTracking()&&moves==commandsBeforeTrace);
    assert(Serial.text.find("SETTLED")!=std::string::npos);
    // PID写入接口不可用不应阻断位置运动，也不自动覆盖手动参数。
    pidWriteAvailable=false;assert(TurretMotorMove(90,20)&&driverPid[2]==20000&&pidWrites==0);
    TurretMotorCancelTrace();assert(!TurretMotorTracking());
    snapshot(0,0);measuredRpm=0;measuredFlags=3;
    assert(TurretMotorTrial(90,120,200,10,100));
    assert(speed==20&&acceleration==56&&targetPulses==800&&trace.durationMs==100);
    snapshot(0,-16384);
    for(int i=0;i<4;++i){clockMs+=10;TurretMotorPoll();assert(TurretMotorTracking());}
    assert(trace.stableSamples==0); // 即使到位也必须继续观察，不能提前结束。
    clockMs+=100;TurretMotorPoll();assert(!TurretMotorTracking());
    assert(Serial.text.find("OBSERVED")!=std::string::npos);
    assert(driverPid[2]==20000&&pidWrites==0);
    // 官方0x43示例帧：单圈编码器与累计位置来自同一回复，位置为-65536。
    EmmFeedbackParser parser;EmmReply reply;
    const uint8_t bytes[]={7,0x43,0x1F,9,0x6B,0x67,0,3,0x43,0xEB,1,0,1,0,0,0,0,0,1,0,1,0,0,1,0,0,0,8,3,3,0x6B};
    for(size_t i=0;i<sizeof(bytes)-1;++i)assert(!parser.feed(bytes[i],reply));
    assert(parser.feed(bytes[sizeof(bytes)-1],reply)&&reply.id==7&&reply.size==31);
    assert(EmmFeedbackParser::snapshotEncoder(reply)==17387&&EmmFeedbackParser::snapshotPosition(reply)==-65536);
}
