#include "node_position_hardware.h"
#include "robot_runtime.h"
#include "h30_usb.h"
#include "emm_feedback.h"
#include "motor_bus.h"
#include "motion_executor.h"

namespace {
class Hardware : public node_position::IO {
    uint32_t version;bool (*cancel)();
    bool warned=false,referenceLost=false;
    const char *motionSpan=nullptr;
    const char *reply(uint8_t id,uint8_t fn,EmmReply &out) {
        EmmFeedbackParser parser;uint32_t start=millis();
        while(millis()-start<25) {
            if(cancelled())return "STOPPED";
            while(Serial1.available()) {
                if(parser.feed(Serial1.read(),out)) {
                    if(out.id!=id)continue;
                    if(out.function==0&&out.data[2]==0xEE)return "MOTOR_COMMAND_ERROR";
                    if(out.function!=fn)continue;
                    if((fn==0xFD||fn==0x0A)&&out.data[2]!=2)return "MOTOR_COMMAND_REJECTED";
                    return nullptr;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        return "MOTOR_FEEDBACK_TIMEOUT";
    }
    void drain(){for(int i=0;i<512&&Serial1.available();++i)Serial1.read();}
public:
    Hardware(uint32_t v,bool (*c)()):version(v),cancel(c){}
    void configure(uint32_t v,bool (*c)()){version=v;cancel=c;warned=referenceLost=false;motionSpan=nullptr;}
    uint32_t now()override{return millis();}
    void wait(uint32_t ms)override{vTaskDelay(pdMS_TO_TICKS(ms));}
    bool cancelled()override{return taskMotionAborted||(cancel&&cancel());}
    const char *imu(H30Attitude &sample,float &heading)override {
        uint32_t current=0;const char *error=H30UsbGetMotionSnapshot(sample,heading,current);
        if(!error&&(current!=version||!version))referenceLost=true;
        if(!error&&referenceLost)error="REFERENCE_CHANGED";
        if(!error&&millis()-sample.receivedAtMs>100)error="IMU_STALE";
        if(error) {
            if(!warned)Serial.printf("[Route] H30 assist unavailable (%s); continuing bounded encoder position control\n",error);
            warned=true;sample=H30Attitude{};return nullptr;
        }
        if(warned)Serial.println("[Route] H30 assist restored");
        warned=false;return nullptr;
    }
    const char *read(uint8_t id,node_position::Motor &state)override {
        MotorBusGuard transaction;
        const uint8_t commands[]={0x3A,0x36,0x35};
        for(uint8_t fn:commands) {
            if(cancelled())return "STOPPED";
            drain();uint8_t cmd[]={id,fn,0x6B};Serial1.write(cmd,3);Serial1.flush();
            EmmReply out;const char *error=reply(id,fn,out);if(error)return error;
            if(fn==0x3A)state.flags=out.data[2];
            else if(fn==0x36)state.position=EmmFeedbackParser::position(out);
            else state.rpm=EmmFeedbackParser::rpm(out);
        }
        return nullptr;
    }
    const char *readPosition(uint8_t id,int64_t &position)override {
        MotorBusGuard transaction;
        if(cancelled())return "STOPPED";
        drain();uint8_t cmd[]={id,0x36,0x6B};Serial1.write(cmd,3);Serial1.flush();
        EmmReply out;const char *error=reply(id,0x36,out);
        if(!error)position=EmmFeedbackParser::position(out);return error;
    }
    const char *zero(uint8_t id)override {
        MotorBusGuard transaction;
        if(cancelled())return "STOPPED";
        drain();Emm_V5_Reset_CurPos_To_Zero(id);Serial1.flush();EmmReply out;return reply(id,0x0A,out);
    }
    const char *target(uint8_t id,bool positive,uint16_t rpm,uint8_t acceleration,uint32_t pulses)override {
        MotorBusGuard transaction;
        if(cancelled())return "STOPPED";
        drain();
        Emm_V5_Pos_Control(id,positive?1:0,rpm,acceleration,pulses,true,true);Serial1.flush();
        // ACK已确认装载完成，不再每轮额外固定等待5ms。
        EmmReply out;return reply(id,0xFD,out);
    }
    const char *sync()override {
        if(cancelled())return "STOPPED";
        Emm_V5_Synchronous_motion(0);Serial1.flush();return nullptr;
    }
    const char *velocity(const continuous::WheelCommand &command)override {
        for(uint8_t id=1;id<=4;++id) {
            if(cancelled())return "STOPPED";
            int16_t rpm=command.rpm[id-1];
            Emm_V5_Vel_Control(id,rpm>=0?1:0,abs(rpm),0,true);
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        return sync();
    }
    void halt()override {
        for(uint8_t id=1;id<=4;++id){Emm_V5_Stop_Now(id,true);vTaskDelay(pdMS_TO_TICKS(5));}
        Emm_V5_Synchronous_motion(0);Serial1.flush();
        if(motionSpan){Serial.printf("{EVT,MOTION,SPAN,%lu,%lu,%s,END}\n",static_cast<unsigned long>(CurrentMotionId()),static_cast<unsigned long>(millis()),motionSpan);motionSpan=nullptr;}
    }
    void report(const char *stage,const node_position::Result &r,float reference)override {
        Serial.printf("{EVT,NAV,ROUTE_PHASE,0,%s}\n",stage);
        Serial.printf("{EVT,NAV,ROUTE_POSITION,%.2f,%.2f,%.2f,%.3f,%.3f}\n",r.s,r.estimated.x,r.estimated.y,reference,r.heading);
    }
    void motorDiagnostic(uint8_t id,const char *reason,double goal,const node_position::Motor &m)override {
        Serial.printf("{EVT,NAV,ROUTE_MOTOR,%u,%s,%.0f,%lld,%d,%u}\n",id,reason,goal,
            static_cast<long long>(m.position),m.rpm,m.flags);
    }
    void timing(const char *stage)override {
        Serial.printf("{EVT,NAV,ROUTE_TIMING,%s,%lu}\n",stage,static_cast<unsigned long>(millis()));
        const bool start=strstr(stage,"_SENT")!=nullptr;
        if(start)motionSpan=strncmp(stage,"TURN",4)==0?"CHASSIS_TURN":"CHASSIS_TRANSLATION";
        if(motionSpan)Serial.printf("{EVT,MOTION,SPAN,%lu,%lu,%s,%s}\n",static_cast<unsigned long>(CurrentMotionId()),
            static_cast<unsigned long>(millis()),motionSpan,start?"BEGIN":"END");
        if(!start)motionSpan=nullptr;
    }
    void arrivalDiagnostic(float residual,const node_position::Motor (&m)[4])override {
        Serial.printf("{EVT,NAV,ROUTE_WAIT,%lu,%.3f,%u,%d,%u,%d,%u,%d,%u,%d}\n",
            static_cast<unsigned long>(millis()),residual,
            m[0].flags,m[0].rpm,m[1].flags,m[1].rpm,m[2].flags,m[2].rpm,m[3].flags,m[3].rpm);
    }
};
Hardware routeHardware(0,nullptr);
node_position::Runner routeRunner;
Hardware scanHardware(0,nullptr);
ScanMotionRunner scanRunner;
}
void BeginContinuousScan(const ScanMotionSettings &settings,bool (*received)()){scanHardware.configure(0,nullptr);scanRunner.begin(settings,scanHardware,received);}
void TickContinuousScan(){scanRunner.tick();}
ScanMotionRunner &ContinuousScanRunner(){return scanRunner;}
void BeginPositionNodeRoute(const continuous::Route &route,const node_position::Settings &settings,
    uint32_t version,bool (*cancelled)(),node_position::Result &result) {
    routeHardware.configure(version,cancelled);routeRunner.begin(route,settings,routeHardware,result);
}
void TickPositionNodeRoute(){routeRunner.tick();}
node_position::Runner &PositionNodeRunner(){return routeRunner;}
const char *ReadMotorFeedback(uint8_t id,node_position::Motor &state){Hardware io(0,nullptr);return io.read(id,state);}
const char *RunPositionNodeRoute(const continuous::Route &route,const node_position::Settings &settings,
    uint32_t version,bool (*cancelled)(),node_position::Result &result) {
    Hardware io(version,cancelled);return node_position::run(route,settings,io,result);
}
bool ReadChassisEncoderPositions(int64_t (&positions)[4]) {
    Hardware io(0,nullptr);
    for(uint8_t id=1;id<=4;++id) {
        const char *error=io.readPosition(id,positions[id-1]);
        if(error){Serial.printf("[Scan] Encoder read failed: %s\n",error);return false;}
    }
    return true;
}
const char *RunContinuousScan(const ScanMotionSettings &settings,bool (*received)(),float &forwardTravel) {
    Hardware io(0,nullptr);
    return RunScanMotion(settings,io,received,forwardTravel);
}
