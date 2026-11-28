// 电机反馈字节解析与编码器位置换算。
// 调用者拿到的状态用于运动反馈；查看位移时要区分原始编码器值与累计位置。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include <stdint.h>  // 引入 stdint.h 中的声明和定义。
#include <stddef.h>  // 引入 stddef.h 中的声明和定义。
#include <string.h>  // 引入 string.h 中的声明和定义。

// 查询及尚未触发的同步绝对位置指令可以原样重发。
template<class Exchange>  // 为下一条声明提供模板参数 <class Exchange>。
const char *RetryEmmFeedback(uint8_t attempts,Exchange exchange,bool retryUnassignedError=false) {  // 定义函数或成员 的入口 RetryEmmFeedback（重试Emm反馈），参数由此处的签名确定。
    const char *error;  // 声明 error（错误），类型和可用范围由所在声明决定。
    do {  // 进入至少执行一次的循环体。
        error=exchange();  // 将 exchange() 写入或初始化 error（错误）。
        if(!error)return nullptr;  // 若 !error，立即返回 nullptr。
        const bool retry=strcmp(error,"MOTOR_FEEDBACK_TIMEOUT")==0||  // 将 strcmp(error,"MOTOR_FEEDBACK_TIMEOUT")==0|| 写入或初始化 retry（重试）。
            (retryUnassignedError&&strcmp(error,"MOTOR_COMMAND_ERROR")==0);  // 继续上一行的组合条件，本行提供 (retryUnassignedError&&strcmp(error,"MOTOR_COMMAND_ERROR")==0);。
        if(!retry)return error;  // 若 !retry，立即返回 error。
    } while(--attempts);  // 当 --attempts 成立时继续循环。
    return error;  // 结束当前函数并返回 error。
}  // 结束此初始化列表或局部作用域。

// EMM V5固定0x6B校验模式。仅解析本控制器使用的回复，按固定长度防止载荷6B截帧。
struct EmmReply {uint8_t id=0,function=0,size=0,data[40]={};};  // 定义 EmmReply 的类型成员或作用域。
class EmmFeedbackParser {  // 定义 EmmFeedbackParser（Emm反馈Parser） 的类型成员或作用域。
public:  // 此后的成员采用 public 访问权限，直到下一个权限标签。
    /**
     * @brief 向EMM反馈解析器输入一个字节，完整合法回复写入out。
     * @return true 本次得到完整帧，false 还需后续字节或丢弃了无效数据。
     */
    bool feed(uint8_t value,EmmReply &out) {  // 定义函数或成员 的入口 feed，参数由此处的签名确定。
        if(size==sizeof(bytes))discard(1);bytes[size++]=value;  // 当 size==sizeof(bytes) 成立时进入此分支，否则继续后续流程。
        while(size>=2) {  // 当 size>=2 成立时继续循环。
            if(bytes[0]<1||bytes[0]>7){discard(1);continue;}  // 当 bytes[0]<1||bytes[0]>7 成立时进入此分支，否则继续后续流程。
            uint8_t n=length(bytes[1]);if(!n){discard(1);continue;}  // 将 length(bytes[1]);if(!n){discard(1);continue;} 写入或初始化 n。
            if(size<n)return false;  // 若 size<n，立即返回 false。
            if(bytes[n-1]!=0x6B||((bytes[1]==0x33||bytes[1]==0x35||bytes[1]==0x36)&&bytes[2]>1)){discard(1);continue;}  // 当 bytes[n-1]!=0x6B||((bytes[1]==0x33||bytes[1]==0x35||by… 成立时进入此分支，否则继续后续流程。
            if(bytes[1]==0x43&&(bytes[2]!=31||bytes[18]>1)){discard(1);continue;}  // 当 bytes[1]==0x43&&(bytes[2]!=31||bytes[18]>1) 成立时进入此分支，否则继续后续流程。
            if(bytes[1]==0x42&&(bytes[2]!=33||bytes[3]!=21)){discard(1);continue;}  // 当 bytes[1]==0x42&&(bytes[2]!=33||bytes[3]!=21) 成立时进入此分支，否则继续后续流程。
            out.id=bytes[0];out.function=bytes[1];out.size=n;  // 将 bytes[0] 写入或初始化 out.id。然后将 bytes[1] 写入或初始化 out.function。然后将 n 写入或初始化 out.size。
            for(uint8_t i=0;i<n;++i)out.data[i]=bytes[i];  // 按 uint8_t i=0;i<n;++i)out.data[i]=bytes[i]; 组织循环的初值、继续条件和步进。
            discard(n);return true;  // 调用 discard，传入 n。然后结束当前函数并返回 true。
        }  // 结束此循环体。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此函数或控制块。
    /**
     * @brief 从位置回复中解析有符号编码器位置，单位65536/圈。
     */
    static int64_t position(const EmmReply &v) {  // 定义函数或成员 的入口 position（位置），参数由此处的签名确定。
        uint32_t raw=(uint32_t(v.data[3])<<24)|(uint32_t(v.data[4])<<16)|(uint32_t(v.data[5])<<8)|v.data[6];  // 将 (uint32_t(v.data[3])<<24)|(uint32_t(v.data[4])<<16)|(u… 写入或初始化 raw。
        return v.data[2]?-int64_t(raw):int64_t(raw);  // 结束当前函数并返回 v.data[2]?-int64_t(raw):int64_t(raw)。
    }  // 结束此函数或控制块。
    /**
     * @brief 从速度回复中解析有符号转速RPM。
     */
    static int16_t rpm(const EmmReply &v){int16_t raw=(uint16_t(v.data[3])<<8)|v.data[4];return v.data[2]?-raw:raw;}  // 将 (uint16_t(v.data[3])<<8)|v.data[4];return v.data[2]?-r… 写入或初始化 raw。
    // 0x43系统状态：同一应答中的校准后单圈编码器及累计位置（65536单位/圈）。
    static uint16_t snapshotEncoder(const EmmReply &v){return (uint16_t(v.data[8])<<8)|v.data[9];}  // 定义函数或成员 snapshotEncoder（snapshot编码器），参数由此处的签名确定。
    static int64_t snapshotPosition(const EmmReply &v) {  // 定义函数或成员 的入口 snapshotPosition（snapshot位置），参数由此处的签名确定。
        uint32_t raw=(uint32_t(v.data[19])<<24)|(uint32_t(v.data[20])<<16)|(uint32_t(v.data[21])<<8)|v.data[22];  // 将 (uint32_t(v.data[19])<<24)|(uint32_t(v.data[20])<<16)|… 写入或初始化 raw。
        return v.data[18]?-int64_t(raw):int64_t(raw);  // 结束当前函数并返回 v.data[18]?-int64_t(raw):int64_t(raw)。
    }  // 结束此函数或控制块。
    static int16_t snapshotRpm(const EmmReply &v){int16_t rpm=(uint16_t(v.data[16])<<8)|v.data[17];return v.data[15]?-rpm:rpm;}  // 将 (uint16_t(v.data[16])<<8)|v.data[17];return v.data[15]… 写入或初始化 rpm（转速）。
    static uint32_t pid(const EmmReply &v,uint8_t index){const uint8_t n=2+4*index;return (uint32_t(v.data[n])<<24)|(uint32_t(v.data[n+1])<<16)|(uint32_t(v.data[n+2])<<8)|v.data[n+3];}  // 将 2+4*index;return (uint32_t(v.data[n])<<24)|(uint32_t(v… 写入或初始化 n。
private:  // 此后的成员采用 private 访问权限，直到下一个权限标签。
    uint8_t bytes[40]={};size_t size=0;  // 将 {};size_t size=0 写入或初始化 bytes[40]。
    /**
     * @brief 根据回复功能码确定固定帧长，未知功能返回0。
     */
    static uint8_t length(uint8_t f) {  // 定义函数或成员 的入口 length（长度），参数由此处的签名确定。
        if(f==0x42)return 33;if(f==0x21)return 15;if(f==0x43)return 31;if(f==0x33||f==0x36)return 8;if(f==0x35)return 6;  // 若 f==0x42，立即返回 33;if(f==0x21)return 15;if(f==0x43)return 31;if(f==0x3…。
        return f==0x48||f==0x46||f==0xF3||f==0x0E||f==0x4A||f==0x3A||f==0x0A||f==0xFD||f==0xFE||f==0xFF||f==0?4:0;  // 结束当前函数并返回 f==0x48||f==0x46||f==0xF3||f==0x0E||f==0x4A||f==0x3A||…。
    }  // 结束此函数或控制块。
    /**
     * @brief 移除已消费或无效的缓存前缀字节，保留后续待解析数据。
     */
    void discard(size_t n){size-=n;for(size_t i=0;i<size;++i)bytes[i]=bytes[i+n];}  // 将 n;for(size_t i=0;i<size;++i)bytes[i]=bytes[i+n];} 写入或初始化 size-。
};  // 结束此类型定义。
