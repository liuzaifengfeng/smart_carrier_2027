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
        bool commandError=false;
        uint8_t received[64];size_t receivedSize=0;
        while(millis()-start<25) {
            if(cancelled())return "STOPPED";
            while(Serial1.available()) {
                const uint8_t byte=Serial1.read();
                if(receivedSize<sizeof(received))received[receivedSize++]=byte;
                if(parser.feed(byte,out)) {
                    if(out.id!=id)continue;
                    if(out.function==0&&out.data[2]==0xEE) {
                        Serial.printf("[MotorBus] id=%u request=0x%02X RX=",id,fn);
                        for(uint8_t i=0;i<out.size;++i)Serial.printf("%s%02X",i?" ":"",out.data[i]);
                        Serial.println("");
                        // 通用错误帧不含原请求功能码，可能来自该地址之前的命令。
                        // 保留错误记录，在原响应窗口内继续等匹配帧；不重发请求。
                        commandError=true;
                        continue;
                    }
                    if(out.function!=fn)continue;
                    if((fn==0xFD||fn==0x0A||fn==0x4A)&&out.data[2]!=2)return "MOTOR_COMMAND_REJECTED";
                    if(commandError)Serial.printf("[MotorBus] id=%u request=0x%02X matched after unassigned error\n",id,fn);
                    return nullptr;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        if(fn==0x4A||fn==0x21) {
            Serial.printf("[MotorBus] id=%u request=0x%02X timeout=25ms RX=",id,fn);
            for(size_t i=0;i<receivedSize;++i)Serial.printf("%s%02X",i?" ":"",received[i]);
            if(!receivedSize)Serial.printf("<none>");
            Serial.println("");
        }
        return commandError?"MOTOR_COMMAND_ERROR":"MOTOR_FEEDBACK_TIMEOUT";
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
    /** @brief 单独发送0x35转速查询，一次请求/应答，不附带状态和位置读取。 */
    const char *readSpeed(uint8_t id,int16_t &rpm)override {
        MotorBusGuard transaction;
        if(cancelled())return "STOPPED";
        drain();uint8_t cmd[]={id,0x35,0x6B};Serial1.write(cmd,3);Serial1.flush();
        EmmReply out;const char *error=reply(id,0x35,out);
        if(!error)rpm=EmmFeedbackParser::rpm(out);return error;
    }
    const char *readPosition(uint8_t id,int64_t &position)override {
        MotorBusGuard transaction;
        if(cancelled())return "STOPPED";
        drain();uint8_t cmd[]={id,0x36,0x6B};Serial1.write(cmd,3);Serial1.flush();
        EmmReply out;const char *error=reply(id,0x36,out);
        if(!error)position=EmmFeedbackParser::position(out);
        else Serial.printf("[MotorBus] position query TX=%02X 36 6B error=%s\n",id,error);
        return error;
    }
    const char *zero(uint8_t id)override {
        MotorBusGuard transaction;
        if(cancelled())return "STOPPED";
        drain();Emm_V5_Reset_CurPos_To_Zero(id);Serial1.flush();EmmReply out;return reply(id,0x0A,out);
    }
    const char *readEncoderSnapshot(uint8_t id,uint16_t &encoder,int64_t &position,int16_t &rpm,uint8_t &flags) {
        MotorBusGuard transaction;
        if(cancelled())return "STOPPED";
        drain();uint8_t cmd[]={id,0x43,0x7A,0x6B};Serial1.write(cmd,sizeof(cmd));Serial1.flush();
        EmmReply out;const char *error=reply(id,0x43,out);
        if(!error){encoder=EmmFeedbackParser::snapshotEncoder(out);position=EmmFeedbackParser::snapshotPosition(out);rpm=EmmFeedbackParser::snapshotRpm(out);flags=out.data[29];}
        else Serial.printf("[MotorBus] encoder snapshot TX=%02X 43 7A 6B error=%s\n",id,error);
        return error;
    }
    const char *readPositionPid(uint8_t id,uint32_t (&values)[3]) {
        MotorBusGuard transaction;
        if(cancelled())return "STOPPED";
        drain();uint8_t cmd[]={id,0x21,0x6B};Serial1.write(cmd,sizeof(cmd));Serial1.flush();
        EmmReply out;const char *error=reply(id,0x21,out);
        if(!error)for(uint8_t i=0;i<3;++i)values[i]=EmmFeedbackParser::pid(out,i);
        return error;
    }
    const char *writePositionPid(uint8_t id,const uint32_t (&values)[3],bool save=false) {
        MotorBusGuard transaction;
        if(cancelled())return "STOPPED";
        // 默认 svF=0 仅修改 RAM；调试页可显式选择保存 Flash。
        uint8_t cmd[17]={id,0x4A,0xC3,uint8_t(save)};
        for(uint8_t i=0;i<3;++i)for(uint8_t b=0;b<4;++b)cmd[4+4*i+b]=values[i]>>(24-8*b);
        cmd[16]=0x6B;drain();Serial1.write(cmd,sizeof(cmd));Serial1.flush();
        EmmReply out;const char *error=reply(id,0x4A,out);
        if(error) {
            Serial.printf("[MotorBus] PID write error=%s TX=",error);
            for(uint8_t i=0;i<sizeof(cmd);++i)Serial.printf("%s%02X",i?" ":"",cmd[i]);
            Serial.println("");
        }
        return error;
    }
    // 调试页的配置命令复用相同的事务锁和应答窗口。
    const char *configureMotor(uint8_t id,uint8_t function,uint8_t value,bool save) {
        MotorBusGuard transaction;
        drain();
        if(function==0x46)Emm_V5_Modify_Ctrl_Mode(id,save,value);
        else if(function==0xF3)Emm_V5_En_Control(id,value,false);
        else Emm_V5_Reset_Clog_Pro(id);
        Serial1.flush();EmmReply out;
        const char *error=reply(id,function,out);
        return error?error:out.data[2]==2?nullptr:"MOTOR_COMMAND_REJECTED";
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
    void headingStable(float heading)override {
        Serial.printf("{EVT,NAV,ROUTE_IMU_STABLE,%lu,%lu,3,%.3f}\n",
            static_cast<unsigned long>(CurrentMotionId()),static_cast<unsigned long>(millis()),heading);
    }
    void headingControl(float target,float actual,float omega,float speed)override {
        Serial.printf("{EVT,NAV,ROUTE_HOLD,%lu,%lu,%.3f,%.3f,%.3f,%.3f,%.1f}\n",
            static_cast<unsigned long>(CurrentMotionId()),static_cast<unsigned long>(millis()),
            target,actual,continuous::turn(actual,target),omega,speed);
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
    /** @brief 记录停转后完整检查耗时、额外等待及四轮转速/有符号位置残差；各轮依次采样。 */
    void arrivalComparison(uint8_t representative,uint32_t readMs,uint32_t waitMs,
        bool arrived,bool rotating,const float (&residual)[4],const node_position::Motor (&m)[4])override {
        Serial.printf("{EVT,NAV,ROUTE_CHECK,%lu,%lu,%u,%lu,%lu,%s,%s,%d,%.3f,%d,%.3f,%d,%.3f,%d,%.3f}\n",
            static_cast<unsigned long>(CurrentMotionId()),static_cast<unsigned long>(millis()),representative,
            static_cast<unsigned long>(readMs),static_cast<unsigned long>(waitMs),arrived?"PASS":"WAIT",rotating?"deg":"mm",
            m[0].rpm,residual[0],m[1].rpm,residual[1],m[2].rpm,residual[2],m[3].rpm,residual[3]);
    }
};
Hardware routeHardware(0,nullptr);
node_position::Runner routeRunner;
Hardware scanHardware(0,nullptr);
ScanMotionRunner scanRunner;
}
/**
 * @brief 绑定扫码硬件并启动 ScanMotionRunner；received 回调由 pollTaskCode 提供。
 */
void BeginContinuousScan(const ScanMotionSettings &settings,bool (*received)()){scanHardware.configure(0,nullptr);scanRunner.begin(settings,scanHardware,received);}
/**
 * @brief 调用扫码 Runner 的 tick，推进一次接近/搜索行程。
 */
void TickContinuousScan(){scanRunner.tick();}
/**
 * @brief 取得扫码 Runner，供执行器查询结束、错误及编码器行程。
 */
ScanMotionRunner &ContinuousScanRunner(){return scanRunner;}
/**
 * @brief 绑定路径、启动配置、航向参考代次及取消回调，初始化节点路径 Runner；不等待行程。
 */
void BeginPositionNodeRoute(const continuous::Route &route,const node_position::Settings &settings,
    uint32_t version,bool (*cancelled)(),node_position::Result &result) {
    routeHardware.configure(version,cancelled);routeRunner.begin(route,settings,routeHardware,result);
}
/**
 * @brief 推进节点位置路径一次，由统一运动任务周期调用。
 */
void TickPositionNodeRoute(){routeRunner.tick();}
/**
 * @brief 取得正式节点路径 Runner，供执行器查询进度和观察姿态提前触发阶段。
 */
node_position::Runner &PositionNodeRunner(){return routeRunner;}
/**
 * @brief 读取指定电机的到位标志、编码器位置和转速。
 * @param id 电机地址。
 * @param state 输出反馈，位置65536单位/圈，转速RPM。
 * @return nullptr 成功，否则为通信错误原因。
 */
const char *ReadMotorFeedback(uint8_t id,node_position::Motor &state){Hardware io(0,nullptr);return io.read(id,state);}
const char *ReadMotorEncoderPosition(uint8_t id,int64_t &position){Hardware io(0,nullptr);return io.readPosition(id,position);}
const char *ReadMotorEncoderSnapshot(uint8_t id,uint16_t &encoder,int64_t &position,int16_t &rpm,uint8_t &flags){Hardware io(0,nullptr);return io.readEncoderSnapshot(id,encoder,position,rpm,flags);}
const char *ReadMotorPositionPid(uint8_t id,uint32_t (&values)[3]){Hardware io(0,nullptr);return io.readPositionPid(id,values);}
const char *WriteMotorPositionPid(uint8_t id,const uint32_t (&values)[3],bool save){Hardware io(0,nullptr);return io.writePositionPid(id,values,save);}
// 部分驱动器写入生效但未在窗口内回复 ACK，以三项实际回读确认结果，不重发写入。
const char *SetMotorPositionPidVerified(uint8_t id,const uint32_t (&values)[3],bool save) {
    MotorBusGuard transaction;
    const char *error=WriteMotorPositionPid(id,values,save);
    if(error&&strcmp(error,"MOTOR_FEEDBACK_TIMEOUT")!=0)return error;
    uint32_t actual[3]={};
    error=ReadMotorPositionPid(id,actual);
    if(error)return error;
    for(uint8_t i=0;i<3;++i)if(actual[i]!=values[i])return "PID_VERIFY_MISMATCH";
    return nullptr;
}
const char *ConfigureEmmMotor(uint8_t id,uint8_t function,uint8_t value,bool save){Hardware io(0,nullptr);return io.configureMotor(id,function,value,save);}
const char *ZeroMotorEncoderPosition(uint8_t id){Hardware io(0,nullptr);return io.zero(id);}
/**
 * @brief 同步运行节点位置路径直至结束；调用者持有底盘运动锁。
 * @return nullptr 成功，否则为错误原因；result 为编码器/航向推算结果。
 */
const char *RunPositionNodeRoute(const continuous::Route &route,const node_position::Settings &settings,
    uint32_t version,bool (*cancelled)(),node_position::Result &result) {
    Hardware io(version,cancelled);return node_position::run(route,settings,io,result);
}
/**
 * @brief 回读四个底盘电机的编码器位置，数组顺序为电机1到4。
 * @return true 四轮读取成功；位置单位为65536/圈。
 */
bool ReadChassisEncoderPositions(int64_t (&positions)[4]) {
    Hardware io(0,nullptr);
    for(uint8_t id=1;id<=4;++id) {
        const char *error=io.readPosition(id,positions[id-1]);
        if(error){Serial.printf("[Scan] Encoder read failed: %s\n",error);return false;}
    }
    return true;
}
/**
 * @brief 同步运行连续扫码底盘行程；成功返回 nullptr，结束行程(mm)写入 forwardTravel。
 */
const char *RunContinuousScan(const ScanMotionSettings &settings,bool (*received)(),float &forwardTravel) {
    Hardware io(0,nullptr);
    return RunScanMotion(settings,io,received,forwardTravel);
}
