#include "arm_axis_trial.h"
#include "chassis.h"
#include <assert.h>
#include <math.h>
#include <string.h>

NativeSerial Serial;
float HEIGHT_PULSE=80,LENGTH_PULSE=30.2f;
uint32_t clockMs=100;
uint32_t millis(){return clockMs;}
int transactionDepth=0,reads=0,moves=0;
bool available=true;
uint8_t feedbackMotor=0,lastId=0,lastDir=0,lastAcc=0;
uint16_t lastRpm=0,encoderValue=1234;
uint32_t lastPulses=0;
int64_t countValue=0;
int16_t rpmValue=0;uint8_t flagsValue=3;
struct MotorBusGuard {
    MotorBusGuard(){++transactionDepth;}
    ~MotorBusGuard(){--transactionDepth;}
};
void vTaskDelay(uint32_t ms){assert(transactionDepth>0&&ms==5);clockMs+=ms;}
const char *ReadMotorEncoderSnapshot(uint8_t id,uint16_t &encoder,int64_t &counts,int16_t &rpm,uint8_t &flags) {
    assert(transactionDepth>0&&(id==5||id==6));++reads;feedbackMotor=id;
    if(!available)return "MOTOR_FEEDBACK_TIMEOUT";
    encoder=encoderValue;counts=countValue;rpm=rpmValue;flags=flagsValue;return nullptr;
}
void Emm_V5_Pos_Control(uint8_t id,uint8_t dir,uint16_t rpm,uint8_t acc,uint32_t pulses,bool absolute,bool sync) {
    assert(transactionDepth>0&&absolute&&!sync);++moves;
    lastId=id;lastDir=dir;lastRpm=rpm;lastAcc=acc;lastPulses=pulses;
}
#include "arm_axis_trial_actual.inc"

void commands() {
    assert(!ArmAxisTrialStart(5,120,1000,100,20,1000));
    assert(lastId==5&&lastDir==1&&lastPulses==9600&&lastRpm==1000&&lastAcc==254);
    assert(feedbackMotor==5&&transactionDepth==0);
    unsigned motor=0,rpm=0,acc=0;unsigned long record=0,started=0;float target=0,initial=1;
    assert(sscanf(Serial.text.c_str(),"{EVT,AXIS,COMMAND,%u,%lu,%lu,%f,%f,%u,%u}",
        &motor,&record,&started,&target,&initial,&rpm,&acc)==7);
    assert(motor==5&&record==1&&started==clockMs&&target==120&&initial==0&&rpm==1000&&acc==254);
    const int previous=moves;
    assert(!strcmp(ArmAxisTrialStart(6,50,500,100,20,1000),"BUSY")&&moves==previous);
    ArmAxisTrialCancel();
    assert(!ArmAxisTrialActive()&&moves==previous&&ArmAxisTrialNextDelay()==20);
    assert(!ArmAxisTrialStart(6,50,500,100,20,1000));
    assert(lastId==6&&feedbackMotor==6&&lastPulses==1510&&lastAcc==252);
    ArmAxisTrialCancel();
    // 协议加减速档位的量化、上下限和0值。
    for(auto sample:{std::initializer_list<int>{1,65535,1},{65535,1,255},{1000,0,0}}) {
        auto value=sample.begin();const int rpm=*value++,ramp=*value++,acc=*value;
        assert(!ArmAxisTrialStart(5,0,rpm,ramp,20,100));assert(lastAcc==acc&&lastPulses==0);
        ArmAxisTrialCancel();
    }
}

void sampling() {
    assert(!ArmAxisTrialStart(5,100,600,200,20,1000));
    const uint32_t start=clockMs;
    assert(ArmAxisTrialNextDelay()==20);
    clockMs+=19;ArmAxisTrialPoll();assert(reads==1&&ArmAxisTrialNextDelay()==1);
    clockMs+=1;countValue=-172032;rpmValue=-42;flagsValue=1;ArmAxisTrialPoll();
    assert(fabs(trace.measured-105)<0.001&&trace.overshoot==5&&ArmAxisTrialActive());
    assert(Serial.text.find(",100.000,105.000,42,1,5.000,1234,-172032,5}")!=std::string::npos);
    assert(transactionDepth==0&&moves==1);
    countValue=-163840;rpmValue=0;flagsValue=3;clockMs=start+500;ArmAxisTrialPoll();
    assert(ArmAxisTrialActive()); // 到位标志不会提前结束记录。
    clockMs=start+1000;ArmAxisTrialPoll();assert(!ArmAxisTrialActive()&&!ArmAxisTrialError());
    assert(Serial.text.find(",OBSERVED,100.000,100.000,5.000}")!=std::string::npos);
}

void reverse() {
    countValue=-98304; // 6号起点约159mm，回到100mm，保留反向过冲。
    assert(!ArmAxisTrialStart(6,100,500,100,20,500));
    countValue=-57344;rpmValue=30;clockMs+=20;ArmAxisTrialPoll();
    assert(trace.measured<100&&trace.overshoot>7&&trace.overshoot<8);
    assert(Serial.text.find(",-30,3,")!=std::string::npos);
    countValue=65536;rpmValue=INT16_MIN;clockMs+=20;ArmAxisTrialPoll();
    assert(trace.measured<0); // 不能用绝对值掩盖越过零点的方向错误。
    assert(Serial.text.find(",32768,3,")!=std::string::npos);
    ArmAxisTrialCancel();const int previous=reads;clockMs+=1000;ArmAxisTrialPoll();
    assert(reads==previous&&!ArmAxisTrialError()&&moves==1&&transactionDepth==0);
    assert(Serial.text.find(",CANCELLED,")!=std::string::npos);
}

void invalid() {
    for(uint8_t id:{0,1,4,7,255})assert(ArmAxisTrialStart(id,10,100,100,20,1000));
    for(float target:{-1.0f,175.1f,NAN,INFINITY})assert(ArmAxisTrialStart(5,target,100,100,20,1000));
    assert(ArmAxisTrialStart(6,170.1f,100,100,20,1000));
    assert(ArmAxisTrialStart(5,0,0,100,20,1000));
    assert(ArmAxisTrialStart(5,0,100,100,0,1000));
    assert(ArmAxisTrialStart(5,0,100,100,20,0));
    for(float scale:{0.0f,-1.0f,NAN,INFINITY}) {
        HEIGHT_PULSE=scale;assert(!strcmp(ArmAxisTrialStart(5,100,100,100,20,1000),"CALIBRATION"));
    }
    HEIGHT_PULSE=1e20f;assert(ArmAxisTrialStart(5,175,100,100,20,1000));
    assert(!moves&&!reads&&!transactionDepth&&!ArmAxisTrialActive());
    HEIGHT_PULSE=80;assert(!ArmAxisTrialStart(5,175,100,100,20,1000));ArmAxisTrialCancel();
    assert(!ArmAxisTrialStart(6,170,100,100,20,1000));ArmAxisTrialCancel();
}

void failures() {
    available=false;
    assert(!strcmp(ArmAxisTrialStart(5,100,100,100,20,100),"MOTOR_FEEDBACK_TIMEOUT"));
    assert(!moves&&!ArmAxisTrialActive()&&!transactionDepth);
    available=true;assert(!ArmAxisTrialStart(6,50,100,100,20,100));
    const uint32_t start=clockMs;
    available=false;clockMs+=20;ArmAxisTrialPoll();assert(ArmAxisTrialActive());
    available=true;clockMs+=20;ArmAxisTrialPoll();assert(!ArmAxisTrialError());
    available=false;clockMs=start+100;ArmAxisTrialPoll();
    assert(!ArmAxisTrialActive()&&!strcmp(ArmAxisTrialError(),"MOTOR_FEEDBACK_TIMEOUT"));
    assert(Serial.text.find(",MOTOR_FEEDBACK_TIMEOUT,")!=std::string::npos);
    // 失败不设置永久锁；恢复通信即可开始下一段。
    available=true;assert(!ArmAxisTrialStart(5,80,100,100,20,100));
    assert(!ArmAxisTrialError()&&lastId==5&&transactionDepth==0);ArmAxisTrialCancel();
}

void timing() {
    clockMs=UINT32_MAX-15;
    assert(!ArmAxisTrialStart(5,50,100,100,1000,30));
    assert(ArmAxisTrialNextDelay()==30);clockMs+=29;ArmAxisTrialPoll();
    assert(reads==1&&ArmAxisTrialNextDelay()==1);
    clockMs+=1;ArmAxisTrialPoll();
    assert(reads==2&&!ArmAxisTrialActive()&&!ArmAxisTrialError());
    assert(Serial.text.find(",OBSERVED,")!=std::string::npos); // 周期大于窗口时仍能准时结束。
    // 单次试验固定标定系数；界面参数后续改变不会让记录的单位跳变。
    clockMs=100;HEIGHT_PULSE=80;
    assert(!ArmAxisTrialStart(5,40,100,100,20,100));HEIGHT_PULSE=160;
    countValue=-65536;clockMs+=20;ArmAxisTrialPoll();assert(trace.measured==40);
    ArmAxisTrialCancel();
}

int main(int argc,char **argv) {
    assert(argc==2);
    if(!strcmp(argv[1],"commands"))commands();
    else if(!strcmp(argv[1],"sampling"))sampling();
    else if(!strcmp(argv[1],"reverse"))reverse();
    else if(!strcmp(argv[1],"invalid"))invalid();
    else if(!strcmp(argv[1],"failures"))failures();
    else if(!strcmp(argv[1],"timing"))timing();
    else assert(false);
}
