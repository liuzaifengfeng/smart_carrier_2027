#pragma once
#include "../src/node_position_control.h"
#include "../src/emm_feedback.h"
#include "../src/scan_motion_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
using namespace continuous;
using namespace node_position;
#define CHECK(x) do{if(!(x)){fprintf(stderr,"position line %d: %s\n",__LINE__,#x);exit(2);}}while(0)
static Route route;
struct Sim : IO {
    uint32_t time=1000;double actual[4]={},goal[4]={},pending[4]={};
    uint16_t rpm[4]={},stagedRpm[4]={},peak=0;uint8_t lastAcceleration=0;
    bool frozen=false,falseArrived=false,cancel=false,missing=false,reject=false;
    bool disturb=false,injected=false,correctedMoving=false,differential=false;
    bool residualTest=false,residualInjected=false,residualActive=false,hardOverrun=false;
    bool persistentResidual=false;float disturbanceReference=0,disturbanceAngle=3;
    int residualEncoderUnits=6000;
    uint32_t cancelAt=0,missingAt=0;
    uint32_t rippleAt=0,firstZeroAt=0;bool rippleInjected=false;
    bool neverStops=false,badZero=false,stall=false;
    uint8_t stallMotor=4,stallFlags=133;
    int startupQueries=0;bool targetSent=false;
    uint32_t latencyMs=0;
    uint32_t flagDelayMs=0,rpmTailMs=0,doneAt[4]={};
    int sentMarks=0,arrivalMarks=0,waitReports=0;
    int recoveries=0,diagnostics=0;
    int actions=0,updates=0,syncs=0,halts=0,zeroGroups=0;float heading=0;Vec location={310,1200};
    uint32_t arrivalAt=0,maxGap=0;bool wasMoving=false,newAction=false;
    int encoderPolarity[4]={1,1,1,1};
    bool startupKick=false,kickInjected=false;
    bool velocityMode=false;int16_t velocityRpm[4]={};int velocityUpdates=0,curveReports=0;
    uint32_t now()override{return time;}
    bool cancelled()override{return cancel||(cancelAt&&time>=cancelAt);}
    void wait(uint32_t ms)override {
        double pulses[4]={};
        for(int i=0;i<4;++i) {
            double amount=rpm[i]*PULSES_PER_REV*ms/60000.0;
            double delta=fmax(-amount,fmin(amount,goal[i]-actual[i]));
            if(velocityMode)delta=velocityRpm[i]*PULSES_PER_REV*ms/60000.0;
            if(frozen)delta=0;
            actual[i]+=delta;pulses[i]=delta;
        }
        float angle=(pulses[0]+pulses[1]+pulses[2]+pulses[3])/(4*51.8f);
        Vec body={static_cast<float>((-pulses[0]-pulses[1]+pulses[2]+pulses[3])/(4*13.3)),
                  static_cast<float>((pulses[0]-pulses[1]+pulses[2]-pulses[3])/(4*13.6))};
        location=location+bodyToWorld(body,heading+angle/2);heading=wrap(heading+angle);time+=ms;
        bool moving=false;for(int i=0;i<4;++i)if(fabs(goal[i]-actual[i])>.001)moving=true;
        if(wasMoving&&!moving)arrivalAt=time;wasMoving=moving;
        if(disturb&&!injected&&location.x>500){heading=wrap(heading+disturbanceAngle);injected=true;}
        if(injected&&location.x>550&&location.x<1000&&fabsf(turn(heading,disturbanceReference))<1)correctedMoving=true;
    }
    const char *imu(H30Attitude &sample,float &value)override {
        if(missing||(missingAt&&time>=missingAt))return "MOTION_FIELDS_REQUIRED";
        sample.fields=7;sample.sequence=time/10;sample.receivedAtMs=time;sample.accel[2]=9.78f;value=heading;
        return nullptr;
    }
    const char *read(uint8_t id,Motor &m)override {
        if(!targetSent)startupQueries+=3;
        int i=id-1;wait(1+3*latencyMs);bool done=fabs(goal[i]-actual[i])<.001;
        m.position=llround(actual[i]*65536/PULSES_PER_REV);
        if(residualTest&&done&&pending[i]!=0&&!residualInjected){residualInjected=true;residualActive=true;}
        if(done&&pending[i]!=0&&residualActive)m.position+=(actual[i]>=0?1:-1)*(hardOverrun?12000:residualEncoderUnits);
        m.position*=encoderPolarity[i];
        if(startupKick&&!kickInjected&&actions==1&&id==1&&m.position!=0) {
            m.position=m.position>=0?-200:200;
            kickInjected=true;
        }
        if(done&&pending[i]!=0&&!doneAt[i])doneAt[i]=time;
        bool flagReady=done&&(!flagDelayMs||time-doneAt[i]>=flagDelayMs);
        m.flags=1|((flagReady||falseArrived)?2:0);m.rpm=(done||falseArrived)?0:rpm[i];
        if(done&&pending[i]!=0&&time-doneAt[i]<rpmTailMs)m.rpm=6;
        if(stall&&actions>0&&id==stallMotor)m.flags=stallFlags;
        if(neverStops){m.rpm=6;m.position+=time*10;}
        if(rippleAt&&time>=rippleAt&&!rippleInjected&&actions==0){m.rpm=6;rippleInjected=true;}
        return nullptr;
    }
    const char *readPosition(uint8_t id,int64_t &position)override {
        if(!targetSent)++startupQueries;
        wait(1+latencyMs);
        position=badZero?100:llround(actual[id-1]*65536/PULSES_PER_REV)*encoderPolarity[id-1];
        return nullptr;
    }
    const char *zero(uint8_t id)override {
        int i=id-1;CHECK(fabs(goal[i]-actual[i])<.01);actual[i]=goal[i]=pending[i]=0;
        if(id==1){if(!firstZeroAt)firstZeroAt=time;++zeroGroups;}return nullptr;
    }
    const char *target(uint8_t id,bool positive,uint16_t speed,uint8_t acceleration,uint32_t count)override {
        CHECK(acceleration>0); // 每条位置命令必须携带驱动板加减速曲线档位
        targetSent=true;
        if(reject)return "MOTOR_COMMAND_REJECTED";
        if(latencyMs)wait(latencyMs); // ACK后硬件适配器不再固定等待5ms
        int i=id-1;double next=positive?count:-double(count);
        CHECK(pending[i]==0); // 同一动作禁止重复装载位置指令，含低速补位重发
        pending[i]=next;stagedRpm[i]=speed;lastAcceleration=acceleration;peak=fmaxf(peak,speed);++updates;return nullptr;
    }
    const char *sync()override {
        velocityMode=false;
        if(newAction){if(arrivalAt&&time-arrivalAt>maxGap)maxGap=time-arrivalAt;newAction=false;}
        if(residualActive&&!persistentResidual)residualActive=false;
        if(stagedRpm[0]!=stagedRpm[1])differential=true;
        for(int i=0;i<4;++i){goal[i]=pending[i];rpm[i]=stagedRpm[i];}++syncs;return nullptr;
    }
    const char *velocity(const WheelCommand &command)override {
        if(cancelled())return "STOPPED";
        if(reject)return "MOTOR_COMMAND_REJECTED";
        velocityMode=true;targetSent=true;++velocityUpdates;
        for(int i=0;i<4;++i){velocityRpm[i]=command.rpm[i];rpm[i]=abs(command.rpm[i]);}
        return nullptr;
    }
    void beginAction()override{
        // 新动作（包括停止后校角）不能在上一动作仍运动时插入。
        for(int i=0;i<4;++i)CHECK(fabs(goal[i]-actual[i])<.01);
        ++actions;newAction=true;for(int i=0;i<4;++i){pending[i]=0;doneAt[i]=0;}
    }
    void timing(const char *stage)override {
        if(strstr(stage,"_SENT"))++sentMarks;
        if(strstr(stage,"_ARRIVED"))++arrivalMarks;
    }
    void arrivalDiagnostic(float,const Motor (&m)[4])override {
        for(int i=0;i<4;++i)CHECK(abs(m[i].rpm)<=3);
        ++waitReports;
    }
    void halt()override {velocityMode=false;for(int i=0;i<4;++i){goal[i]=actual[i];pending[i]=0;rpm[i]=0;}++halts;}
    void report(const char *stage,const Result&,float)override{
        if(!strcmp(stage,"POSITION_RECOVER"))++recoveries;
        if(!strcmp(stage,"OUTER_CURVE"))++curveReports;
    }
    void motorDiagnostic(uint8_t,const char*,double,const Motor&)override{++diagnostics;}
};
struct ScanSim : Sim {
    Vec start;float entryRpm=0,fastRpm=0,stopAt=100000;
    bool crossed=false,zeroWhileMoving=false;
    const char *velocity(const WheelCommand &command)override {
        CHECK(time<120000); // 测试控制循环必须有限完成。
        float distance=norm(location-start),speed=abs(command.rpm[0]);
        if(distance>=700&&!crossed){entryRpm=speed;crossed=true;}
        if(distance<600)fastRpm=fmaxf(fastRpm,speed);
        if(!speed&&velocityUpdates)zeroWhileMoving=true;
        return Sim::velocity(command);
    }
};
static ScanSim *scanCar=nullptr;
static bool scanReceived(){return norm(scanCar->location-scanCar->start)>=scanCar->stopAt;}
