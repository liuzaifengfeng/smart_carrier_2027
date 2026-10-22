#pragma once
#include <stdint.h>
#include <stddef.h>

// EMM V5固定0x6B校验模式。仅解析本控制器使用的回复，按固定长度防止载荷6B截帧。
struct EmmReply {uint8_t id=0,function=0,size=0,data[8]={};};
class EmmFeedbackParser {
public:
    /**
     * @brief 向EMM反馈解析器输入一个字节，完整合法回复写入out。
     * @return true 本次得到完整帧，false 还需后续字节或丢弃了无效数据。
     */
    bool feed(uint8_t value,EmmReply &out) {
        if(size==sizeof(bytes))discard(1);bytes[size++]=value;
        while(size>=2) {
            if(bytes[0]<1||bytes[0]>6){discard(1);continue;}
            uint8_t n=length(bytes[1]);if(!n){discard(1);continue;}
            if(size<n)return false;
            if(bytes[n-1]!=0x6B||((bytes[1]==0x35||bytes[1]==0x36)&&bytes[2]>1)){discard(1);continue;}
            out.id=bytes[0];out.function=bytes[1];out.size=n;
            for(uint8_t i=0;i<n;++i)out.data[i]=bytes[i];
            discard(n);return true;
        }
        return false;
    }
    /**
     * @brief 从位置回复中解析有符号编码器位置，单位65536/圈。
     */
    static int64_t position(const EmmReply &v) {
        uint32_t raw=(uint32_t(v.data[3])<<24)|(uint32_t(v.data[4])<<16)|(uint32_t(v.data[5])<<8)|v.data[6];
        return v.data[2]?-int64_t(raw):int64_t(raw);
    }
    /**
     * @brief 从速度回复中解析有符号转速RPM。
     */
    static int16_t rpm(const EmmReply &v){int16_t raw=(uint16_t(v.data[3])<<8)|v.data[4];return v.data[2]?-raw:raw;}
private:
    uint8_t bytes[32]={};size_t size=0;
    /**
     * @brief 根据回复功能码确定固定帧长，未知功能返回0。
     */
    static uint8_t length(uint8_t f) {
        if(f==0x36)return 8;if(f==0x35)return 6;
        return f==0x3A||f==0x0A||f==0xFD||f==0xFE||f==0xFF||f==0?4:0;
    }
    /**
     * @brief 移除已消费或无效的缓存前缀字节，保留后续待解析数据。
     */
    void discard(size_t n){size-=n;for(size_t i=0;i<size;++i)bytes[i]=bytes[i+n];}
};
