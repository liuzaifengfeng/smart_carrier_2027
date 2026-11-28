// 节点运动的串口和传感器适配。
// 把状态机的读写转换为电机命令，并提供圆盘基准编码器记录和回程位移计算。

#include "node_position_hardware.h"  // 引入 node_position_hardware.h 中的声明和定义。
#include "robot_runtime.h"  // 引入 robot_runtime.h 中的声明和定义。
#include "h30_usb.h"  // 引入 h30_usb.h 中的声明和定义。
#include "emm_feedback.h"  // 引入 emm_feedback.h 中的声明和定义。
#include "emm_drive_config.h"  // 引入 emm_drive_config.h 中的声明和定义。
#include "motor_bus.h"  // 引入 motor_bus.h 中的声明和定义。
#include "motion_executor.h"  // 引入 motion_executor.h 中的声明和定义。

namespace {  // 开始匿名命名空间，本文件内部共享其中的定义。
constexpr uint8_t DISC_FEEDBACK_ATTEMPTS=3;  // 将 3 写入或初始化 DISC_FEEDBACK_ATTEMPTS（圆盘反馈ATTEMPTS）。
constexpr uint32_t DISC_FEEDBACK_TIMEOUT_MS=50;  // 将 50 写入或初始化 DISC_FEEDBACK_TIMEOUT_MS（圆盘反馈超时毫秒）。
constexpr uint8_t DISC_RETURN_ACCEL_LEVEL=200;  // 将 200 写入或初始化 DISC_RETURN_ACCEL_LEVEL（圆盘RETURNACCELLEVEL）。
class Hardware : public node_position::IO {  // 定义 Hardware 的类型成员或作用域。
    uint32_t version;bool (*cancel)();  // 声明 version（版本），类型和可用范围由所在声明决定。然后调用 bool，传入 *cancel)(。
    uint8_t feedbackAttempts;  // 声明 feedbackAttempts（反馈Attempts），类型和可用范围由所在声明决定。
    uint32_t feedbackTimeoutMs;  // 声明 feedbackTimeoutMs（反馈超时毫秒），类型和可用范围由所在声明决定。
    bool warned=false,referenceLost=false;  // 将 false,referenceLost=false 写入或初始化 warned。
    const char *motionSpan=nullptr;  // 将 nullptr 写入或初始化 motionSpan。
    const char *reportedStage=nullptr;  // 将 nullptr 写入或初始化 reportedStage（reported阶段）。
    uint32_t lastPositionReport=0;  // 将 0 写入或初始化 lastPositionReport（最后位置Report）。
    const char *reply(uint8_t id,uint8_t fn,EmmReply &out) {  // 定义函数或成员 的入口 reply，参数由此处的签名确定。
        EmmFeedbackParser parser;uint32_t start=millis();  // 声明 parser，类型和可用范围由所在声明决定。然后将 millis() 写入或初始化 start（起点）。
        bool commandError=false;  // 将 false 写入或初始化 commandError（command错误）。
        uint8_t received[64];size_t receivedSize=0;  // 声明 received（已接收），类型和可用范围由所在声明决定。然后将 0 写入或初始化 receivedSize（已接收大小）。
        while(millis()-start<feedbackTimeoutMs) {  // 当 millis()-start<feedbackTimeoutMs 成立时继续循环。
            if(cancelled())return "STOPPED";  // 若 cancelled()，立即返回 "STOPPED"。
            while(Serial1.available()) {  // 当 Serial1.available() 成立时继续循环。
                const uint8_t byte=Serial1.read();  // 将 Serial1.read() 写入或初始化 byte（字节）。
                if(receivedSize<sizeof(received))received[receivedSize++]=byte;  // 当 receivedSize<sizeof(received) 成立时进入此分支，否则继续后续流程。
                if(parser.feed(byte,out)) {  // 当 parser.feed(byte,out) 成立时进入此分支，否则继续后续流程。
                    if(out.id!=id)continue;  // 若 out.id!=id，直接进入下一轮循环。
                    if(out.function==0&&out.data[2]==0xEE) {  // 当 out.function==0&&out.data[2]==0xEE 成立时进入此分支，否则继续后续流程。
                        Serial.printf("[MotorBus] id=%u request=0x%02X RX=",id,fn);  // 调用 Serial.printf，传入 "[MotorBus] id=%u request=0x%02X RX=",id,fn。
                        for(uint8_t i=0;i<out.size;++i)Serial.printf("%s%02X",i?" ":"",out.data[i]);  // 按 uint8_t i=0;i<out.size;++i)Serial.printf("%s%02X",i?" … 组织循环的初值、继续条件和步进。
                        Serial.println("");  // 调用 Serial.println，传入 ""。
                        // 通用错误帧不含原请求功能码，可能来自该地址之前的命令。
                        // 在本次应答窗口内继续等匹配帧，再由调用方决定是否重试。
                        commandError=true;  // 将 true 写入或初始化 commandError（command错误）。
                        continue;  // 跳过本次剩余处理，开始下一轮循环。
                    }  // 结束此条件分支。
                    if(out.function!=fn)continue;  // 若 out.function!=fn，直接进入下一轮循环。
                    if((fn==0xFD||fn==0x0A||fn==0x4A||fn==0x48)&&out.data[2]!=2)return "MOTOR_COMMAND_REJECTED";  // 若 (fn==0xFD||fn==0x0A||fn==0x4A||fn==0x48)&&out.data[2]!…，立即返回 "MOTOR_COMMAND_REJECTED"。
                    if(commandError)Serial.printf("[MotorBus] id=%u request=0x%02X matched after unassigned error\n",id,fn);  // 当 commandError 成立时进入此分支，否则继续后续流程。
                    return nullptr;  // 结束当前函数并返回 nullptr。
                }  // 结束此条件分支。
            }  // 结束此循环体。
            vTaskDelay(pdMS_TO_TICKS(1));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        }  // 结束此循环体。
        if(fn==0x4A||fn==0x21||fn==0x42||fn==0x48) {  // 当 fn==0x4A||fn==0x21||fn==0x42||fn==0x48 成立时进入此分支，否则继续后续流程。
            Serial.printf("[MotorBus] id=%u request=0x%02X timeout=%lums RX=",id,fn,  // 通过串口执行 printf，输出内容或参数由本行给出。
                static_cast<unsigned long>(feedbackTimeoutMs));  // 声明接口 long>，参数由此处的签名确定。
            for(size_t i=0;i<receivedSize;++i)Serial.printf("%s%02X",i?" ":"",received[i]);  // 按 size_t i=0;i<receivedSize;++i)Serial.printf("%s%02X",i… 组织循环的初值、继续条件和步进。
            if(!receivedSize)Serial.printf("<none>");  // 当 !receivedSize 成立时进入此分支，否则继续后续流程。
            Serial.println("");  // 调用 Serial.println，传入 ""。
        }  // 结束此条件分支。
        return commandError?"MOTOR_COMMAND_ERROR":"MOTOR_FEEDBACK_TIMEOUT";  // 结束当前函数并返回 commandError?"MOTOR_COMMAND_ERROR":"MOTOR_FEEDBACK_TIM…。
    }  // 结束此函数或控制块。
    void drain(){for(int i=0;i<512&&Serial1.available();++i)Serial1.read();}  // 定义函数或成员 drain，参数由此处的签名确定。
    template<class Send>  // 为下一条声明提供模板参数 <class Send>。
    const char *exchange(uint8_t id,uint8_t fn,EmmReply &out,Send send,bool readOnly=false) {  // 定义函数或成员 的入口 exchange，参数由此处的签名确定。
        const uint8_t attempts=readOnly&&feedbackAttempts<2?2:feedbackAttempts;  // 将 readOnly&&feedbackAttempts<2?2:feedbackAttempts 写入或初始化 attempts。
        return RetryEmmFeedback(attempts,[&]() -> const char * {  // 结束当前函数并返回 RetryEmmFeedback(attempts,[&]() -> const char * {。
            if(cancelled())return "STOPPED";  // 若 cancelled()，立即返回 "STOPPED"。
            drain();  // 调用 drain，使用无参形式推进相应操作。
            send();  // 调用 send，使用无参形式推进相应操作。
            Serial1.flush();  // 调用 Serial1.flush。
            return reply(id,fn,out);  // 结束当前函数并返回 reply(id,fn,out)。
        },readOnly);  // 续写当前声明或运算，提供 },readOnly);。
    }  // 结束此初始化列表或局部作用域。
    const char *query(uint8_t id,uint8_t fn,EmmReply &out) {  // 定义函数或成员 的入口 query，参数由此处的签名确定。
        return exchange(id,fn,out,[&] {  // 结束当前函数并返回 exchange(id,fn,out,[&] {。
            uint8_t cmd[]={id,fn,0x6B};  // 声明 cmd（命令），类型和可用范围由所在声明决定。
            Serial1.write(cmd,sizeof(cmd));  // 调用 Serial1.write，传入 cmd,sizeof(cmd)。
        },true);  // 续写当前声明或运算，提供 },true);。
    }  // 结束此函数或控制块。
public:  // 此后的成员采用 public 访问权限，直到下一个权限标签。
    Hardware(uint32_t v,bool (*c)(),uint8_t attempts=1,uint32_t timeoutMs=25)  // 调用 Hardware，传入 uint32_t v,bool (*c)(),uint8_t attempts=1,uint32_t tim…。
        :version(v),cancel(c),feedbackAttempts(attempts),feedbackTimeoutMs(timeoutMs){}  // 定义函数或成员 version（版本），参数由此处的签名确定。
    void configure(uint32_t v,bool (*c)()){  // 定义函数或成员 的入口 configure，参数由此处的签名确定。
        version=v;cancel=c;warned=referenceLost=false;motionSpan=nullptr;reportedStage=nullptr;  // 将 v 写入或初始化 version（版本）。然后将 c 写入或初始化 cancel。然后将 referenceLost=false 写入或初始化 warned。然后将 nullptr 写入或初始化 motionSpan。然后将 nullptr 写入或初始化 reportedStage（reported阶段）。
    }  // 结束此函数或控制块。
    uint32_t now()override{return millis();}  // 定义函数或成员 now，参数由此处的签名确定。
    void wait(uint32_t ms)override{vTaskDelay(pdMS_TO_TICKS(ms));}  // 定义函数或成员 wait，参数由此处的签名确定。
    bool cancelled()override{return taskMotionAborted||(cancel&&cancel());}  // 定义函数或成员 cancelled（已取消），参数由此处的签名确定。
    const char *imu(H30Attitude &sample,float &heading)override {  // 定义函数或成员 的入口 imu（惯性传感器），参数由此处的签名确定。
        uint32_t current=0;const char *error=H30UsbGetMotionSnapshot(sample,heading,current);  // 将 0 写入或初始化 current（当前）。然后将 H30UsbGetMotionSnapshot(sample,heading,current) 写入或初始化 error（错误）。
        if(!error&&(current!=version||!version))referenceLost=true;  // 当 !error&&(current!=version||!version) 成立时进入此分支，否则继续后续流程。
        if(!error&&referenceLost)error="REFERENCE_CHANGED";  // 当 !error&&referenceLost 成立时进入此分支，否则继续后续流程。
        if(!error&&millis()-sample.receivedAtMs>100)error="IMU_STALE";  // 当 !error&&millis()-sample.receivedAtMs>100 成立时进入此分支，否则继续后续流程。
        if(error) {  // 当 error 成立时进入此分支，否则继续后续流程。
            if(!warned)Serial.printf("[Route] H30 assist unavailable (%s); continuing bounded encoder position control\n",error);  // 当 !warned 成立时进入此分支，否则继续后续流程。
            warned=true;sample=H30Attitude{};return nullptr;  // 将 true;sample=H30Attitude{};return nullptr 写入或初始化 warned。
        }  // 结束此条件分支。
        if(warned)Serial.println("[Route] H30 assist restored");  // 当 warned 成立时进入此分支，否则继续后续流程。
        warned=false;return nullptr;  // 将 false 写入或初始化 warned。然后结束当前函数并返回 nullptr。
    }  // 结束此函数或控制块。
    const char *read(uint8_t id,node_position::Motor &state)override {  // 定义函数或成员 的入口 read，参数由此处的签名确定。
        MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
        const uint8_t commands[]={0x3A,0x36,0x35};  // 声明 commands，类型和可用范围由所在声明决定。
        for(uint8_t fn:commands) {  // 按 uint8_t fn:commands 组织循环的初值、继续条件和步进。
            EmmReply out;  // 声明 out，类型和可用范围由所在声明决定。
            if(const char *error=query(id,fn,out)) {  // 当 const char *error=query(id,fn,out) 成立时进入此分支，否则继续后续流程。
                if(feedbackAttempts>1)Serial.printf("[DiscReturn] feedback motor=%u function=0x%02X error=%s\n",id,fn,error);  // 当 feedbackAttempts>1 成立时进入此分支，否则继续后续流程。
                return error;  // 结束当前函数并返回 error。
            }  // 结束此条件分支。
            if(fn==0x3A)state.flags=out.data[2];  // 当 fn==0x3A 成立时进入此分支，否则继续后续流程。
            else if(fn==0x36)state.position=EmmFeedbackParser::position(out);  // 仅在先前条件均不成立时，当 fn==0x36 成立时进入此分支，否则继续后续流程。
            else state.rpm=EmmFeedbackParser::rpm(out);  // 前一个判断不成立时执行此分支。
        }  // 结束此循环体。
        return nullptr;  // 结束当前函数并返回 nullptr。
    }  // 结束此函数或控制块。
    /** @brief 单独发送0x35转速查询，一次请求/应答，不附带状态和位置读取。 */
    const char *readSpeed(uint8_t id,int16_t &rpm)override {  // 定义函数或成员 的入口 readSpeed（read速度），参数由此处的签名确定。
        MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
        EmmReply out;const char *error=query(id,0x35,out);  // 声明 out，类型和可用范围由所在声明决定。然后将 query(id,0x35,out) 写入或初始化 error（错误）。
        if(!error)rpm=EmmFeedbackParser::rpm(out);return error;  // 当 !error 成立时进入此分支，否则继续后续流程。
    }  // 结束此函数或控制块。
    const char *readPosition(uint8_t id,int64_t &position)override {  // 定义函数或成员 的入口 readPosition（read位置），参数由此处的签名确定。
        MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
        EmmReply out;const char *error=query(id,0x36,out);  // 声明 out，类型和可用范围由所在声明决定。然后将 query(id,0x36,out) 写入或初始化 error（错误）。
        if(!error)position=EmmFeedbackParser::position(out);  // 当 !error 成立时进入此分支，否则继续后续流程。
        else Serial.printf("[MotorBus] position query TX=%02X 36 6B error=%s\n",id,error);  // 前一个判断不成立时执行此分支。
        return error;  // 结束当前函数并返回 error。
    }  // 结束此函数或控制块。
    const char *zero(uint8_t id)override {  // 定义函数或成员 的入口 zero（零点），参数由此处的签名确定。
        MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
        if(cancelled())return "STOPPED";  // 若 cancelled()，立即返回 "STOPPED"。
        drain();Emm_V5_Reset_CurPos_To_Zero(id);Serial1.flush();EmmReply out;return reply(id,0x0A,out);  // 调用 drain，使用无参形式推进相应操作。然后调用 Emm_V5_Reset_CurPos_To_Zero（EmmV5ResetCur位置To零点），传入 id。然后调用 Serial1.flush。然后声明 out，类型和可用范围由所在声明决定。然后结束当前函数并返回 reply(id,0x0A,out)。
    }  // 结束此函数或控制块。
    const char *readEncoderSnapshot(uint8_t id,uint16_t &encoder,int64_t &position,int16_t &rpm,uint8_t &flags) {  // 定义函数或成员 的入口 readEncoderSnapshot（read编码器Snapshot），参数由此处的签名确定。
        MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
        EmmReply out;const char *error=exchange(id,0x43,out,[&] {  // 将 exchange(id,0x43,out,[&] { 写入或初始化 error（错误）。
            const uint8_t cmd[]={id,0x43,0x7A,0x6B};  // 声明 cmd（命令），类型和可用范围由所在声明决定。
            Serial1.write(cmd,sizeof(cmd));  // 调用 Serial1.write，传入 cmd,sizeof(cmd)。
        },true);  // 续写当前声明或运算，提供 },true);。
        if(!error){encoder=EmmFeedbackParser::snapshotEncoder(out);position=EmmFeedbackParser::snapshotPosition(out);rpm=EmmFeedbackParser::snapshotRpm(out);flags=out.data[29];}  // 当 !error 成立时进入此分支，否则继续后续流程。
        else Serial.printf("[MotorBus] encoder snapshot TX=%02X 43 7A 6B error=%s\n",id,error);  // 前一个判断不成立时执行此分支。
        return error;  // 结束当前函数并返回 error。
    }  // 结束此函数或控制块。
    const char *readPositionPid(uint8_t id,uint32_t (&values)[3]) {  // 定义函数或成员 的入口 readPositionPid（read位置PID），参数由此处的签名确定。
        MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
        if(cancelled())return "STOPPED";  // 若 cancelled()，立即返回 "STOPPED"。
        drain();uint8_t cmd[]={id,0x21,0x6B};Serial1.write(cmd,sizeof(cmd));Serial1.flush();  // 调用 drain，传入 );uint8_t cmd[]={id,0x21,0x6B};Serial1.write(cmd,sizeo…。
        EmmReply out;const char *error=reply(id,0x21,out);  // 声明 out，类型和可用范围由所在声明决定。然后将 reply(id,0x21,out) 写入或初始化 error（错误）。
        if(!error)for(uint8_t i=0;i<3;++i)values[i]=EmmFeedbackParser::pid(out,i);  // 当 !error 成立时进入此分支，否则继续后续流程。
        return error;  // 结束当前函数并返回 error。
    }  // 结束此函数或控制块。
    const char *writePositionPid(uint8_t id,const uint32_t (&values)[3],bool save=false) {  // 定义函数或成员 的入口 writePositionPid（write位置PID），参数由此处的签名确定。
        MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
        if(cancelled())return "STOPPED";  // 若 cancelled()，立即返回 "STOPPED"。
        // 默认 svF=0 仅修改 RAM；调试页可显式选择保存 Flash。
        uint8_t cmd[17]={id,0x4A,0xC3,uint8_t(save)};  // 将 {id,0x4A,0xC3,uint8_t(save)} 写入或初始化 cmd[17]。
        for(uint8_t i=0;i<3;++i)for(uint8_t b=0;b<4;++b)cmd[4+4*i+b]=values[i]>>(24-8*b);  // 按 uint8_t i=0;i<3;++i)for(uint8_t b=0;b<4;++b)cmd[4+4*i+… 组织循环的初值、继续条件和步进。
        cmd[16]=0x6B;drain();Serial1.write(cmd,sizeof(cmd));Serial1.flush();  // 将 0x6B 写入或初始化 cmd[16]。然后调用 drain，使用无参形式推进相应操作。然后调用 Serial1.write，传入 cmd,sizeof(cmd)。然后调用 Serial1.flush。
        EmmReply out;const char *error=reply(id,0x4A,out);  // 声明 out，类型和可用范围由所在声明决定。然后将 reply(id,0x4A,out) 写入或初始化 error（错误）。
        if(error) {  // 当 error 成立时进入此分支，否则继续后续流程。
            Serial.printf("[MotorBus] PID write error=%s TX=",error);  // 调用 Serial.printf，传入 "[MotorBus] PID write error=%s TX=",error。
            for(uint8_t i=0;i<sizeof(cmd);++i)Serial.printf("%s%02X",i?" ":"",cmd[i]);  // 按 uint8_t i=0;i<sizeof(cmd);++i)Serial.printf("%s%02X",i… 组织循环的初值、继续条件和步进。
            Serial.println("");  // 调用 Serial.println，传入 ""。
        }  // 结束此条件分支。
        return error;  // 结束当前函数并返回 error。
    }  // 结束此函数或控制块。
    const char *readDriveConfig(uint8_t id,EmmDriveConfig &config) {  // 定义函数或成员 的入口 readDriveConfig（readDrive配置），参数由此处的签名确定。
        MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
        drain();uint8_t cmd[]={id,0x42,0x6C,0x6B};Serial1.write(cmd,sizeof(cmd));Serial1.flush();  // 调用 drain，传入 );uint8_t cmd[]={id,0x42,0x6C,0x6B};Serial1.write(cmd,…。
        EmmReply out;const char *error=reply(id,0x42,out);  // 声明 out，类型和可用范围由所在声明决定。然后将 reply(id,0x42,out) 写入或初始化 error（错误）。
        if(!error) {  // 当 !error 成立时进入此分支，否则继续后续流程。
            Serial.printf("[MotorBus] config RX=");  // 调用 Serial.printf，传入 "[MotorBus] config RX="。
            for(uint8_t i=0;i<out.size;++i)Serial.printf("%s%02X",i?" ":"",out.data[i]);  // 按 uint8_t i=0;i<out.size;++i)Serial.printf("%s%02X",i?" … 组织循环的初值、继续条件和步进。
            Serial.println("");  // 调用 Serial.println，传入 ""。
            for(uint8_t i=0;i<28;++i)config.data[i]=out.data[4+i];  // 按 uint8_t i=0;i<28;++i)config.data[i]=out.data[4+i]; 组织循环的初值、继续条件和步进。
        }  // 结束此条件分支。
        return error;  // 结束当前函数并返回 error。
    }  // 结束此函数或控制块。
    const char *writeDriveConfig(uint8_t id,const EmmDriveConfig &config,bool save) {  // 定义函数或成员 的入口 writeDriveConfig（writeDrive配置），参数由此处的签名确定。
        MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
        uint8_t cmd[33];config.writeFrame(id,save,cmd);  // 声明 cmd（命令），类型和可用范围由所在声明决定。然后调用 config.writeFrame，传入 id,save,cmd。
        Serial.printf("[MotorBus] config TX=");  // 调用 Serial.printf，传入 "[MotorBus] config TX="。
        for(uint8_t i=0;i<sizeof(cmd);++i)Serial.printf("%s%02X",i?" ":"",cmd[i]);  // 按 uint8_t i=0;i<sizeof(cmd);++i)Serial.printf("%s%02X",i… 组织循环的初值、继续条件和步进。
        Serial.println("");  // 调用 Serial.println，传入 ""。
        drain();Serial1.write(cmd,sizeof(cmd));Serial1.flush();EmmReply out;  // 调用 drain，使用无参形式推进相应操作。然后调用 Serial1.write，传入 cmd,sizeof(cmd)。然后调用 Serial1.flush。然后声明 out，类型和可用范围由所在声明决定。
        const char *error=reply(id,0x48,out);  // 将 reply(id,0x48,out) 写入或初始化 error（错误）。
        if(out.function==0x48) {  // 当 out.function==0x48 成立时进入此分支，否则继续后续流程。
            Serial.printf("[MotorBus] config ACK RX=");  // 调用 Serial.printf，传入 "[MotorBus] config ACK RX="。
            for(uint8_t i=0;i<out.size;++i)Serial.printf("%s%02X",i?" ":"",out.data[i]);  // 按 uint8_t i=0;i<out.size;++i)Serial.printf("%s%02X",i?" … 组织循环的初值、继续条件和步进。
            Serial.println("");  // 调用 Serial.println，传入 ""。
        }  // 结束此条件分支。
        return error;  // 结束当前函数并返回 error。
    }  // 结束此函数或控制块。
    // 调试页的配置命令复用相同的事务锁和应答窗口。
    const char *configureMotor(uint8_t id,uint8_t function,uint8_t value,bool save) {  // 定义函数或成员 的入口 configureMotor，参数由此处的签名确定。
        MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
        drain();  // 调用 drain，使用无参形式推进相应操作。
        if(function==0x46)Emm_V5_Modify_Ctrl_Mode(id,save,value);  // 当 function==0x46 成立时进入此分支，否则继续后续流程。
        else if(function==0xF3)Emm_V5_En_Control(id,value,false);  // 仅在先前条件均不成立时，当 function==0xF3 成立时进入此分支，否则继续后续流程。
        else Emm_V5_Reset_Clog_Pro(id);  // 前一个判断不成立时执行此分支。
        Serial1.flush();EmmReply out;  // 调用 Serial1.flush。然后声明 out，类型和可用范围由所在声明决定。
        const char *error=reply(id,function,out);  // 将 reply(id,function,out) 写入或初始化 error（错误）。
        return error?error:out.data[2]==2?nullptr:"MOTOR_COMMAND_REJECTED";  // 结束当前函数并返回 error?error:out.data[2]==2?nullptr:"MOTOR_COMMAND_REJE…。
    }  // 结束此函数或控制块。
    const char *target(uint8_t id,bool positive,uint16_t rpm,uint8_t acceleration,uint32_t pulses)override {  // 定义函数或成员 的入口 target（目标），参数由此处的签名确定。
        MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
        const auto send=[&]() -> const char * {  // 将 [&]() -> const char * { 写入或初始化 send。
            if(cancelled())return "STOPPED";  // 若 cancelled()，立即返回 "STOPPED"。
            drain();  // 调用 drain，使用无参形式推进相应操作。
            Emm_V5_Pos_Control(id,positive?1:0,rpm,acceleration,pulses,true,true);  // 调用 Emm_V5_Pos_Control（EmmV5位置Control），传入 id,positive?1:0,rpm,acceleration,pulses,true,true。
            Serial1.flush();  // 调用 Serial1.flush。
            EmmReply out;  // 声明 out，类型和可用范围由所在声明决定。
            return reply(id,0xFD,out);  // 结束当前函数并返回 reply(id,0xFD,out)。
        };  // 结束此初始化列表或局部作用域。
        const char *error=send();  // 将 send() 写入或初始化 error（错误）。
        if(!error||strcmp(error,"MOTOR_FEEDBACK_TIMEOUT")!=0)return error;  // 若 !error||strcmp(error,"MOTOR_FEEDBACK_TIMEOUT")!=0，立即返回 error。

        EmmReply feedback;  // 声明 feedback（反馈），类型和可用范围由所在声明决定。
        if(const char *readError=query(id,0x33,feedback))return readError;  // 若 const char *readError=query(id,0x33,feedback)，立即返回 readError。
        const int64_t targetPosition=EmmFeedbackParser::position(feedback);  // 将 EmmFeedbackParser::position(feedback) 写入或初始化 targetPosition（目标位置）。
        if(const char *readError=query(id,0x36,feedback))return readError;  // 若 const char *readError=query(id,0x36,feedback)，立即返回 readError。
        const int64_t position=EmmFeedbackParser::position(feedback);  // 将 EmmFeedbackParser::position(feedback) 写入或初始化 position（位置）。
        if(const char *readError=query(id,0x35,feedback))return readError;  // 若 const char *readError=query(id,0x35,feedback)，立即返回 readError。
        if(abs(EmmFeedbackParser::rpm(feedback))>3)return error;  // 若 abs(EmmFeedbackParser::rpm(feedback))>3，立即返回 error。
        if(const char *readError=query(id,0x3A,feedback))return readError;  // 若 const char *readError=query(id,0x3A,feedback)，立即返回 readError。
        if(!(feedback.data[2]&1))return "MOTOR_DISABLED";  // 若 !(feedback.data[2]&1)，立即返回 "MOTOR_DISABLED"。
        const int64_t desired=llround(double(pulses)*65536/node_position::PULSES_PER_REV)*(positive?1:-1);  // 将 llround(double(pulses)*65536/node_position::PULSES_PER… 写入或初始化 desired（期望）。
        if(llabs(targetPosition-desired)<=1&&llabs(position-desired)<=1)return nullptr;  // 若 llabs(targetPosition-desired)<=1&&llabs(position-desir…，立即返回 nullptr。
        // 仍在同步触发前，补发同一个绝对目标，装载完成后由调用方统一触发。
        return send();  // 结束当前函数并返回 send()。
    }  // 结束此函数或控制块。
    const char *sync()override {  // 定义函数或成员 的入口 sync，参数由此处的签名确定。
        if(cancelled())return "STOPPED";  // 若 cancelled()，立即返回 "STOPPED"。
        Emm_V5_Synchronous_motion(0);Serial1.flush();return nullptr;  // 调用 Emm_V5_Synchronous_motion，传入 0。然后调用 Serial1.flush。然后结束当前函数并返回 nullptr。
    }  // 结束此函数或控制块。
    const char *velocity(const continuous::WheelCommand &command)override {  // 定义函数或成员 的入口 velocity（速度），参数由此处的签名确定。
        for(uint8_t id=1;id<=4;++id) {  // 按 uint8_t id=1;id<=4;++id 组织循环的初值、继续条件和步进。
            if(cancelled())return "STOPPED";  // 若 cancelled()，立即返回 "STOPPED"。
            int16_t rpm=command.rpm[id-1];  // 将 command.rpm[id-1] 写入或初始化 rpm（转速）。
            Emm_V5_Vel_Control(id,rpm>=0?1:0,abs(rpm),0,true);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 id,rpm>=0?1:0,abs(rpm),0,true。
            vTaskDelay(pdMS_TO_TICKS(5));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        }  // 结束此循环体。
        return sync();  // 结束当前函数并返回 sync()。
    }  // 结束此函数或控制块。
    void halt()override {  // 定义函数或成员 的入口 halt，参数由此处的签名确定。
        for(uint8_t id=1;id<=4;++id){Emm_V5_Stop_Now(id,true);vTaskDelay(pdMS_TO_TICKS(5));}  // 按 uint8_t id=1;id<=4;++id 组织循环的初值、继续条件和步进。
        Emm_V5_Synchronous_motion(0);Serial1.flush();  // 调用 Emm_V5_Synchronous_motion，传入 0。然后调用 Serial1.flush。
        if(motionSpan){Serial.printf("{EVT,MOTION,SPAN,%lu,%lu,%s,END}\n",static_cast<unsigned long>(CurrentMotionId()),static_cast<unsigned long>(millis()),motionSpan);motionSpan=nullptr;}  // 当 motionSpan 成立时进入此分支，否则继续后续流程。
    }  // 结束此函数或控制块。
    void report(const char *stage,const node_position::Result &r,float reference)override {  // 定义函数或成员 的入口 report，参数由此处的签名确定。
        const uint32_t now=millis();  // 将 millis() 写入或初始化 now。
        const bool changed=!reportedStage||strcmp(reportedStage,stage)!=0;  // 将 !reportedStage||strcmp(reportedStage,stage)!=0 写入或初始化 changed。
        if(changed) {  // 当 changed 成立时进入此分支，否则继续后续流程。
            reportedStage=stage;  // 将 stage 写入或初始化 reportedStage（reported阶段）。
            Serial.printf("{EVT,NAV,ROUTE_PHASE,0,%s}\n",stage);  // 调用 Serial.printf，传入 "{EVT,NAV,ROUTE_PHASE,0,%s}\n",stage。
        }  // 结束此条件分支。
        if(changed||now-lastPositionReport>=250) {  // 当 changed||now-lastPositionReport>=250 成立时进入此分支，否则继续后续流程。
            lastPositionReport=now;  // 将 now 写入或初始化 lastPositionReport（最后位置Report）。
            Serial.printf("{EVT,NAV,ROUTE_POSITION,%.2f,%.2f,%.2f,%.3f,%.3f}\n",r.s,r.estimated.x,r.estimated.y,reference,r.heading);  // 调用 Serial.printf，传入 "{EVT,NAV,ROUTE_POSITION,%.2f,%.2f,%.2f,%.3f,%.3f}\n",…。
        }  // 结束此条件分支。
    }  // 结束此函数或控制块。
    void headingStable(float heading)override {  // 定义函数或成员 的入口 headingStable（航向稳定），参数由此处的签名确定。
        Serial.printf("{EVT,NAV,ROUTE_IMU_STABLE,%lu,%lu,3,%.3f}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
            static_cast<unsigned long>(CurrentMotionId()),static_cast<unsigned long>(millis()),heading);  // 声明接口 long>，参数由此处的签名确定。
    }  // 结束此函数或控制块。
    void headingControl(float target,float actual,float omega,float speed)override {  // 定义函数或成员 的入口 headingControl（航向Control），参数由此处的签名确定。
        Serial.printf("{EVT,NAV,ROUTE_HOLD,%lu,%lu,%.3f,%.3f,%.3f,%.3f,%.1f}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
            static_cast<unsigned long>(CurrentMotionId()),static_cast<unsigned long>(millis()),  // 定义函数或成员 long>，参数由此处的签名确定。
            target,actual,continuous::turn(actual,target),omega,speed);  // 声明接口 continuous::turn，参数由此处的签名确定。
    }  // 结束此函数或控制块。
    void motorDiagnostic(uint8_t id,const char *reason,double goal,const node_position::Motor &m)override {  // 定义函数或成员 的入口 motorDiagnostic，参数由此处的签名确定。
        Serial.printf("{EVT,NAV,ROUTE_MOTOR,%u,%s,%.0f,%lld,%d,%u}\n",id,reason,goal,  // 通过串口执行 printf，输出内容或参数由本行给出。
            static_cast<long long>(m.position),m.rpm,m.flags);  // 声明接口 long>，参数由此处的签名确定。
    }  // 结束此函数或控制块。
    void timing(const char *stage)override {  // 定义函数或成员 的入口 timing，参数由此处的签名确定。
        Serial.printf("{EVT,NAV,ROUTE_TIMING,%s,%lu}\n",stage,static_cast<unsigned long>(millis()));  // 调用 Serial.printf，传入 "{EVT,NAV,ROUTE_TIMING,%s,%lu}\n",stage,static_cast<un…。
        const bool start=strstr(stage,"_SENT")!=nullptr;  // 将 strstr(stage,"_SENT")!=nullptr 写入或初始化 start（起点）。
        if(start)motionSpan=strncmp(stage,"TURN",4)==0?"CHASSIS_TURN":"CHASSIS_TRANSLATION";  // 当 start 成立时进入此分支，否则继续后续流程。
        if(motionSpan)Serial.printf("{EVT,MOTION,SPAN,%lu,%lu,%s,%s}\n",static_cast<unsigned long>(CurrentMotionId()),  // 当 motionSpan 成立时进入此分支，否则继续后续流程。
            static_cast<unsigned long>(millis()),motionSpan,start?"BEGIN":"END");  // 声明接口 long>，参数由此处的签名确定。
        if(!start)motionSpan=nullptr;  // 当 !start 成立时进入此分支，否则继续后续流程。
    }  // 结束此函数或控制块。
    void arrivalDiagnostic(float residual,const node_position::Motor (&m)[4])override {  // 定义函数或成员 的入口 arrivalDiagnostic，参数由此处的签名确定。
        Serial.printf("{EVT,NAV,ROUTE_WAIT,%lu,%.3f,%u,%d,%u,%d,%u,%d,%u,%d}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
            static_cast<unsigned long>(millis()),residual,  // 定义函数或成员 long>，参数由此处的签名确定。
            m[0].flags,m[0].rpm,m[1].flags,m[1].rpm,m[2].flags,m[2].rpm,m[3].flags,m[3].rpm);  // 续写当前声明或运算，提供 m[0].flags,m[0].rpm,m[1].flags,m[1].rpm,m[2].flags,m[2…。
    }  // 结束此函数或控制块。
    /** @brief 记录最终到位时的检查耗时、额外等待及四轮残差。 */
    void arrivalComparison(uint8_t representative,uint32_t readMs,uint32_t waitMs,  // 定义函数或成员 arrivalComparison，参数由此处的签名确定。
        bool arrived,bool rotating,const float (&residual)[4],const node_position::Motor (&m)[4])override {  // 定义函数或成员 的入口 float，参数由此处的签名确定。
        if(!arrived)return;  // 当 !arrived 成立时进入此分支，否则继续后续流程。
        Serial.printf("{EVT,NAV,ROUTE_CHECK,%lu,%lu,%u,%lu,%lu,PASS,%s,%d,%.3f,%d,%.3f,%d,%.3f,%d,%.3f}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
            static_cast<unsigned long>(CurrentMotionId()),static_cast<unsigned long>(millis()),representative,  // 定义函数或成员 long>，参数由此处的签名确定。
            static_cast<unsigned long>(readMs),static_cast<unsigned long>(waitMs),rotating?"deg":"mm",  // 定义函数或成员 long>，参数由此处的签名确定。
            m[0].rpm,residual[0],m[1].rpm,residual[1],m[2].rpm,residual[2],m[3].rpm,residual[3]);  // 续写当前声明或运算，提供 m[0].rpm,residual[0],m[1].rpm,residual[1],m[2].rpm,res…。
    }  // 结束此函数或控制块。
};  // 结束此类型定义。
Hardware routeHardware(0,nullptr);  // 声明接口 routeHardware（路线Hardware），参数由此处的签名确定。
node_position::Runner routeRunner;  // 声明 routeRunner（路线Runner），类型和可用范围由所在声明决定。
Hardware scanHardware(0,nullptr);  // 声明接口 scanHardware（扫码Hardware），参数由此处的签名确定。
ScanMotionRunner scanRunner;  // 声明 scanRunner（扫码Runner），类型和可用范围由所在声明决定。
}  // 结束命名空间。
/**
 * @brief 绑定扫码硬件并启动 ScanMotionRunner；received 回调由 pollTaskCode 提供。
 */
void BeginContinuousScan(const ScanMotionSettings &settings,bool (*received)()){scanHardware.configure(0,nullptr);scanRunner.begin(settings,scanHardware,received);}  // 定义函数或成员 BeginContinuousScan（BeginContinuous扫码），参数由此处的签名确定。
/**
 * @brief 调用扫码 Runner 的 tick，推进一次接近/搜索行程。
 */
void TickContinuousScan(){scanRunner.tick();}  // 定义函数或成员 TickContinuousScan（轮询Continuous扫码），参数由此处的签名确定。
/**
 * @brief 取得扫码 Runner，供执行器查询结束、错误及编码器行程。
 */
ScanMotionRunner &ContinuousScanRunner(){return scanRunner;}  // 定义函数或成员 ContinuousScanRunner（Continuous扫码Runner），参数由此处的签名确定。
/**
 * @brief 绑定路径、启动配置、航向参考代次及取消回调，初始化节点路径 Runner；不等待行程。
 */
void BeginPositionNodeRoute(const continuous::Route &route,const node_position::Settings &settings,  // 定义函数或成员 BeginPositionNodeRoute（Begin位置节点路线），参数由此处的签名确定。
    uint32_t version,bool (*cancelled)(),node_position::Result &result) {  // 定义函数或成员 的入口 bool，参数由此处的签名确定。
    routeHardware.configure(version,cancelled);routeRunner.begin(route,settings,routeHardware,result);  // 调用 routeHardware.configure，传入 version,cancelled);routeRunner.begin(route,settings,ro…。
}  // 结束此函数或控制块。
/**
 * @brief 推进节点位置路径一次，由统一运动任务周期调用。
 */
void TickPositionNodeRoute(){routeRunner.tick();}  // 定义函数或成员 TickPositionNodeRoute（轮询位置节点路线），参数由此处的签名确定。
/**
 * @brief 取得正式节点路径 Runner，供执行器查询进度和观察姿态提前触发阶段。
 */
node_position::Runner &PositionNodeRunner(){return routeRunner;}  // 定义函数或成员 PositionNodeRunner（位置节点Runner），参数由此处的签名确定。
/**
 * @brief 读取指定电机的到位标志、编码器位置和转速。
 * @param id 电机地址。
 * @param state 输出反馈，位置65536单位/圈，转速RPM。
 * @return nullptr 成功，否则为通信错误原因。
 */
const char *ReadMotorFeedback(uint8_t id,node_position::Motor &state){Hardware io(0,nullptr);return io.read(id,state);}  // 定义函数或成员 ReadMotorFeedback（ReadMotor反馈），参数由此处的签名确定。
const char *ReadMotorEncoderPosition(uint8_t id,int64_t &position){Hardware io(0,nullptr);return io.readPosition(id,position);}  // 定义函数或成员 ReadMotorEncoderPosition（ReadMotor编码器位置），参数由此处的签名确定。
const char *ReadMotorEncoderSnapshot(uint8_t id,uint16_t &encoder,int64_t &position,int16_t &rpm,uint8_t &flags){Hardware io(0,nullptr);return io.readEncoderSnapshot(id,encoder,position,rpm,flags);}  // 定义函数或成员 ReadMotorEncoderSnapshot（ReadMotor编码器Snapshot），参数由此处的签名确定。
const char *ReadMotorPositionPid(uint8_t id,uint32_t (&values)[3]){Hardware io(0,nullptr);return io.readPositionPid(id,values);}  // 定义函数或成员 ReadMotorPositionPid（ReadMotor位置PID），参数由此处的签名确定。
const char *WriteMotorPositionPid(uint8_t id,const uint32_t (&values)[3],bool save){Hardware io(0,nullptr);return io.writePositionPid(id,values,save);}  // 定义函数或成员 WriteMotorPositionPid（WriteMotor位置PID），参数由此处的签名确定。
// 部分驱动器写入生效但未在窗口内回复 ACK，以三项实际回读确认结果，不重发写入。
const char *SetMotorPositionPidVerified(uint8_t id,const uint32_t (&values)[3],bool save) {  // 定义函数或成员 的入口 SetMotorPositionPidVerified（SetMotor位置PIDVerified），参数由此处的签名确定。
    MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
    const char *error=WriteMotorPositionPid(id,values,save);  // 将 WriteMotorPositionPid(id,values,save) 写入或初始化 error（错误）。
    if(error&&strcmp(error,"MOTOR_FEEDBACK_TIMEOUT")!=0)return error;  // 若 error&&strcmp(error,"MOTOR_FEEDBACK_TIMEOUT")!=0，立即返回 error。
    // 经用户确认验证写后回读时序；ACK超时路径已等待25ms，不再追加间隔。
    if(!error)vTaskDelay(pdMS_TO_TICKS(5));  // 当 !error 成立时进入此分支，否则继续后续流程。
    uint32_t actual[3]={};  // 将 {} 写入或初始化 actual[3]。
    error=ReadMotorPositionPid(id,actual);  // 将 ReadMotorPositionPid(id,actual) 写入或初始化 error（错误）。
    if(error)return error;  // 若 error，立即返回 error。
    for(uint8_t i=0;i<3;++i)if(actual[i]!=values[i])return "PID_VERIFY_MISMATCH";  // 按 uint8_t i=0;i<3;++i)if(actual[i]!=values[i])return "PI… 组织循环的初值、继续条件和步进。
    return nullptr;  // 结束当前函数并返回 nullptr。
}  // 结束此函数或控制块。
const char *ConfigureEmmMotor(uint8_t id,uint8_t function,uint8_t value,bool save){Hardware io(0,nullptr);return io.configureMotor(id,function,value,save);}  // 定义函数或成员 ConfigureEmmMotor，参数由此处的签名确定。
const char *ReadMotorDriveParams(uint8_t id,uint16_t (&values)[7]) {  // 定义函数或成员 的入口 ReadMotorDriveParams，参数由此处的签名确定。
    Hardware io(0,nullptr);EmmDriveConfig config;  // 声明接口 io（设备接口），参数由此处的签名确定。然后声明 config（配置），类型和可用范围由所在声明决定。
    const char *error=io.readDriveConfig(id,config);  // 将 io.readDriveConfig(id,config) 写入或初始化 error（错误）。
    if(!error)for(uint8_t i=0;i<7;++i)values[i]=config.get(i);  // 当 !error 成立时进入此分支，否则继续后续流程。
    return error;  // 结束当前函数并返回 error。
}  // 结束此函数或控制块。
const char *SetMotorDriveParamsVerified(uint8_t id,uint16_t (&values)[7],bool save) {  // 定义函数或成员 的入口 SetMotorDriveParamsVerified，参数由此处的签名确定。
    MotorBusGuard transaction;Hardware io(0,nullptr);EmmDriveConfig config;  // 声明 transaction，类型和可用范围由所在声明决定。然后声明接口 io（设备接口），参数由此处的签名确定。然后声明 config（配置），类型和可用范围由所在声明决定。
    Serial.printf("[EMM] id=%u DRIVE_SET stage=BASE_READ save=%u\n",id,unsigned(save));  // 调用 Serial.printf，传入 "[EMM] id=%u DRIVE_SET stage=BASE_READ save=%u\n",id,u…。
    const char *error=io.readDriveConfig(id,config);  // 将 io.readDriveConfig(id,config) 写入或初始化 error（错误）。
    if(error){Serial.printf("[EMM] id=%u DRIVE_SET stage=BASE_READ error=%s (write not sent)\n",id,error);return error;}  // 当 error 成立时进入此分支，否则继续后续流程。
    for(uint8_t i=0;i<7;++i)config.set(i,values[i]);  // 按 uint8_t i=0;i<7;++i)config.set(i,values[i]); 组织循环的初值、继续条件和步进。
    // 经用户确认，整包读取到写入留5ms帧间隔。
    vTaskDelay(pdMS_TO_TICKS(5));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    Serial.printf("[EMM] id=%u DRIVE_SET stage=WRITE_ACK\n",id);  // 调用 Serial.printf，传入 "[EMM] id=%u DRIVE_SET stage=WRITE_ACK\n",id。
    error=io.writeDriveConfig(id,config,save);  // 将 io.writeDriveConfig(id,config,save) 写入或初始化 error（错误）。
    if(error)Serial.printf("[EMM] id=%u DRIVE_SET stage=WRITE_ACK error=%s\n",id,error);  // 当 error 成立时进入此分支，否则继续后续流程。
    if(error&&strcmp(error,"MOTOR_FEEDBACK_TIMEOUT")!=0)return error;  // 若 error&&strcmp(error,"MOTOR_FEEDBACK_TIMEOUT")!=0，立即返回 error。
    // 经用户确认验证Flash写入后100ms再回读；RAM仍5ms，ACK超时不追加等待。
    if(!error)vTaskDelay(pdMS_TO_TICKS(save?100:5));  // 当 !error 成立时进入此分支，否则继续后续流程。
    Serial.printf("[EMM] id=%u DRIVE_SET stage=READBACK\n",id);  // 调用 Serial.printf，传入 "[EMM] id=%u DRIVE_SET stage=READBACK\n",id。
    uint16_t actual[7]={};error=ReadMotorDriveParams(id,actual);  // 将 {};error=ReadMotorDriveParams(id,actual) 写入或初始化 actual[7]。
    if(error){Serial.printf("[EMM] id=%u DRIVE_SET stage=READBACK error=%s\n",id,error);return error;}  // 当 error 成立时进入此分支，否则继续后续流程。
    bool mismatch=false;  // 将 false 写入或初始化 mismatch。
    for(uint8_t i=0;i<7;++i)if(actual[i]!=values[i]) {  // 按 uint8_t i=0;i<7;++i)if(actual[i]!=values[i] 组织循环的初值、继续条件和步进。
        Serial.printf("[EMM] id=%u DRIVE_SET stage=VERIFY field=%u expected=%u actual=%u\n",id,i,values[i],actual[i]);  // 调用 Serial.printf，传入 "[EMM] id=%u DRIVE_SET stage=VERIFY field=%u expected=…。
        mismatch=true;  // 将 true 写入或初始化 mismatch。
    }  // 结束此循环体。
    for(uint8_t i=0;i<7;++i)values[i]=actual[i];  // 按 uint8_t i=0;i<7;++i)values[i]=actual[i]; 组织循环的初值、继续条件和步进。
    return mismatch?"DRIVE_VERIFY_MISMATCH":nullptr;  // 结束当前函数并返回 mismatch?"DRIVE_VERIFY_MISMATCH":nullptr。
}  // 结束此函数或控制块。
const char *ZeroMotorEncoderPosition(uint8_t id){Hardware io(0,nullptr);return io.zero(id);}  // 定义函数或成员 ZeroMotorEncoderPosition（零点Motor编码器位置），参数由此处的签名确定。
/**
 * @brief 同步运行节点位置路径直至结束；调用者持有底盘运动锁。
 * @return nullptr 成功，否则为错误原因；result 为编码器/航向推算结果。
 */
const char *RunPositionNodeRoute(const continuous::Route &route,const node_position::Settings &settings,  // 定义函数或成员 RunPositionNodeRoute（Run位置节点路线），参数由此处的签名确定。
    uint32_t version,bool (*cancelled)(),node_position::Result &result) {  // 定义函数或成员 的入口 bool，参数由此处的签名确定。
    Hardware io(version,cancelled);return node_position::run(route,settings,io,result);  // 声明接口 io（设备接口），参数由此处的签名确定。然后结束当前函数并返回 node_position::run(route,settings,io,result)。
}  // 结束此函数或控制块。
/**
 * @brief 回读四个底盘电机的编码器位置，数组顺序为电机1到4。
 * @return nullptr 表示四轮读取成功；位置单位为65536/圈。
 */
static const char *readChassisEncoderPositions(Hardware &io,int64_t (&positions)[4]) {  // 定义函数或成员 的入口 readChassisEncoderPositions（readChassis编码器Positions），参数由此处的签名确定。
    int64_t snapshot[4];  // 声明 snapshot，类型和可用范围由所在声明决定。
    for(uint8_t id=1;id<=4;++id) {  // 按 uint8_t id=1;id<=4;++id 组织循环的初值、继续条件和步进。
        const char *error=io.readPosition(id,snapshot[id-1]);  // 将 io.readPosition(id,snapshot[id-1]) 写入或初始化 error（错误）。
        if(error)return error;  // 若 error，立即返回 error。
    }  // 结束此循环体。
    memcpy(positions,snapshot,sizeof(snapshot));  // 调用 memcpy，传入 positions,snapshot,sizeof(snapshot)。
    return nullptr;  // 结束当前函数并返回 nullptr。
}  // 结束此函数或控制块。
bool ReadChassisEncoderPositions(int64_t (&positions)[4]) {  // 定义函数或成员 的入口 ReadChassisEncoderPositions（ReadChassis编码器Positions），参数由此处的签名确定。
    Hardware io(0,nullptr);  // 声明接口 io（设备接口），参数由此处的签名确定。
    const char *error=readChassisEncoderPositions(io,positions);  // 将 readChassisEncoderPositions(io,positions) 写入或初始化 error（错误）。
    if(error)Serial.printf("[Chassis] Encoder read failed: %s\n",error);  // 当 error 成立时进入此分支，否则继续后续流程。
    return !error;  // 结束当前函数并返回 !error。
}  // 结束此函数或控制块。
const char *RecordDiscEncoderPositions(int64_t (&positions)[4],bool (*cancelled)()) {  // 定义函数或成员 的入口 RecordDiscEncoderPositions（Record圆盘编码器Positions），参数由此处的签名确定。
    Hardware io(0,cancelled,DISC_FEEDBACK_ATTEMPTS,DISC_FEEDBACK_TIMEOUT_MS);  // 声明接口 io（设备接口），参数由此处的签名确定。
    return readChassisEncoderPositions(io,positions);  // 结束当前函数并返回 readChassisEncoderPositions(io,positions)。
}  // 结束此函数或控制块。
const char *ReadDiscReturnMotorFeedback(uint8_t id,node_position::Motor &state,bool (*cancelled)()) {  // 定义函数或成员 的入口 ReadDiscReturnMotorFeedback（Read圆盘ReturnMotor反馈），参数由此处的签名确定。
    Hardware io(0,cancelled,DISC_FEEDBACK_ATTEMPTS,DISC_FEEDBACK_TIMEOUT_MS);  // 声明接口 io（设备接口），参数由此处的签名确定。
    return io.read(id,state);  // 结束当前函数并返回 io.read(id,state)。
}  // 结束此函数或控制块。
// 驱动器命令每圈脉冲数与编码器每圈 65536 计数是两套比例。
// 圆盘退回计算用这个比例把位置变化换到命令脉冲尺度。
static constexpr double discPulsesPerCount=double(node_position::PULSES_PER_REV)/65536;  // 将 double(node_position::PULSES_PER_REV)/65536 写入或初始化 discPulsesPerCount（圆盘脉冲数Per计数）。
static const char *readDiscTravel(Hardware &io,const int64_t (&before)[4],float beforeHeading,  // 定义函数或成员 readDiscTravel（read圆盘Travel），参数由此处的签名确定。
    int64_t (&positions)[4],float &heading,float &yOffset) {  // 调用 int64_t，传入 &positions)[4],float &heading,float &yOffset) {。
    if(const char *error=readChassisEncoderPositions(io,positions))return error;  // 若 const char *error=readChassisEncoderPositions(io,posit…，立即返回 error。
    heading=beforeHeading;  // 将 beforeHeading 写入或初始化 heading（航向）。
    uint32_t ageMs=0;  // 将 0 写入或初始化 ageMs（样本年龄毫秒）。
    H30UsbGetHeading(heading,ageMs);  // 调用 H30UsbGetHeading（H30UsbGet航向），传入 heading,ageMs。
    float travel[4];  // 声明 travel，类型和可用范围由所在声明决定。
    for(uint8_t wheel=0;wheel<4;++wheel) {  // 按 uint8_t wheel=0;wheel<4;++wheel 组织循环的初值、继续条件和步进。
        const int sign=routeRunner.encoderToCommandSign(wheel);  // 将 routeRunner.encoderToCommandSign(wheel) 写入或初始化 sign。
        travel[wheel]=(positions[wheel]-before[wheel])*sign*discPulsesPerCount;  // 将 (positions[wheel]-before[wheel])*sign*discPulsesPerCou… 写入或初始化 travel[wheel]。
    }  // 结束此循环体。
    yOffset=node_position::discYOffset(travel,X_PULSE,Y_PULSE,beforeHeading,heading);  // 将 node_position::discYOffset(travel,X_PULSE,Y_PULSE,befo… 写入或初始化 yOffset（纵向偏移）。
    return nullptr;  // 结束当前函数并返回 nullptr。
}  // 结束此函数或控制块。
const char *ReadDiscYOffset(const int64_t (&before)[4],float beforeHeading,float &yOffset,bool (*cancelled)()) {  // 定义函数或成员 的入口 ReadDiscYOffset（Read圆盘纵向偏移），参数由此处的签名确定。
    Hardware io(0,cancelled,DISC_FEEDBACK_ATTEMPTS,DISC_FEEDBACK_TIMEOUT_MS);  // 声明接口 io（设备接口），参数由此处的签名确定。
    int64_t positions[4];  // 声明 positions，类型和可用范围由所在声明决定。
    float heading;  // 声明 heading（航向），类型和可用范围由所在声明决定。
    return readDiscTravel(io,before,beforeHeading,positions,heading,yOffset);  // 结束当前函数并返回 readDiscTravel(io,before,beforeHeading,positions,headi…。
}  // 结束此函数或控制块。
const char *BeginDiscYReturn(const int64_t (&before)[4],float beforeHeading,uint16_t maximumRpm,  // 定义函数或成员 BeginDiscYReturn（Begin圆盘纵向Return），参数由此处的签名确定。
    int64_t (&targets)[4],bool (*cancelled)()) {  // 调用 int64_t，传入 &targets)[4],bool (*cancelled)()) {。
    Hardware io(0,cancelled,DISC_FEEDBACK_ATTEMPTS,DISC_FEEDBACK_TIMEOUT_MS);  // 声明接口 io（设备接口），参数由此处的签名确定。
    MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
    int64_t positions[4],goals[4],correction[4],maximumTravel=0;  // 将 0 写入或初始化 maximumTravel。
    float heading,yOffset;  // 声明 heading（航向），类型和可用范围由所在声明决定。
    if(const char *error=readDiscTravel(io,before,beforeHeading,positions,heading,yOffset))return error;  // 若 const char *error=readDiscTravel(io,before,beforeHeadi…，立即返回 error。
    node_position::discReturnWheelPulses(yOffset,X_PULSE,Y_PULSE,heading,correction);  // 调用 node_position::discReturnWheelPulses（节点位置圆盘Return轮脉冲数），传入 yOffset,X_PULSE,Y_PULSE,heading,correction。
    for(uint8_t wheel=0;wheel<4;++wheel) {  // 按 uint8_t wheel=0;wheel<4;++wheel 组织循环的初值、继续条件和步进。
        const int sign=routeRunner.encoderToCommandSign(wheel);  // 将 routeRunner.encoderToCommandSign(wheel) 写入或初始化 sign。
        goals[wheel]=llround(positions[wheel]*sign*discPulsesPerCount)+correction[wheel];  // 将 llround(positions[wheel]*sign*discPulsesPerCount)+corr… 写入或初始化 goals[wheel]。
        targets[wheel]=correction[wheel]!=0?llround(goals[wheel]*sign/discPulsesPerCount):positions[wheel];  // 将 correction[wheel]!=0?llround(goals[wheel]*sign/discPul… 写入或初始化 targets[wheel]。
        const int64_t wheelTravel=llabs(correction[wheel]);  // 将 llabs(correction[wheel]) 写入或初始化 wheelTravel（轮Travel）。
        if(wheelTravel>maximumTravel)maximumTravel=wheelTravel;  // 当 wheelTravel>maximumTravel 成立时进入此分支，否则继续后续流程。
    }  // 结束此循环体。
    if(maximumTravel==0)return nullptr;  // 若 maximumTravel==0，立即返回 nullptr。
    for(uint8_t wheel=0;wheel<4;++wheel) {  // 按 uint8_t wheel=0;wheel<4;++wheel 组织循环的初值、继续条件和步进。
        if(correction[wheel]==0)continue;  // 若 correction[wheel]==0，直接进入下一轮循环。
        const auto rpm=static_cast<uint16_t>(fmax(1.0,round(double(llabs(correction[wheel]))*maximumRpm/maximumTravel)));  // 将 static_cast<uint16_t>(fmax(1.0,round(double(llabs(corr… 写入或初始化 rpm（转速）。
        const auto goal=goals[wheel];  // 将 goals[wheel] 写入或初始化 goal。
        if(const char *error=io.target(wheel+1,goal>=0,rpm,DISC_RETURN_ACCEL_LEVEL,llabs(goal))) {  // 当 const char *error=io.target(wheel+1,goal>=0,rpm,DISC_R… 成立时进入此分支，否则继续后续流程。
            Serial.printf("[DiscReturn] target motor=%u error=%s\n",wheel+1,error);  // 调用 Serial.printf，传入 "[DiscReturn] target motor=%u error=%s\n",wheel+1,error。
            return error;  // 结束当前函数并返回 error。
        }  // 结束此条件分支。
    }  // 结束此循环体。
    return io.sync();  // 结束当前函数并返回 io.sync()。
}  // 结束此函数或控制块。
/**
 * @brief 同步运行连续扫码底盘行程；成功返回 nullptr，结束行程(mm)写入 forwardTravel。
 */
const char *RunContinuousScan(const ScanMotionSettings &settings,bool (*received)(),float &forwardTravel) {  // 定义函数或成员 的入口 RunContinuousScan（RunContinuous扫码），参数由此处的签名确定。
    Hardware io(0,nullptr);  // 声明接口 io（设备接口），参数由此处的签名确定。
    return RunScanMotion(settings,io,received,forwardTravel);  // 结束当前函数并返回 RunScanMotion(settings,io,received,forwardTravel)。
}  // 结束此函数或控制块。
