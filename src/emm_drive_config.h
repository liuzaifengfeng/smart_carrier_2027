#pragma once
#include <stdint.h>

// EMM V5 0x42回复中的28字节配置载荷；0x48按原样回写其余配置。
struct EmmDriveConfig {
    uint8_t data[28]={};
    // 最大输出电压、开环电流、堵转最大电流、检测转速/电流/时间、到达窗口。
    // 单位依次为mV、mA、mA、RPM、mA、ms、0.1度，全部大端uint16。
    static uint8_t offset(uint8_t field) {
        const uint8_t offsets[]={12,8,10,20,22,24,26};
        return offsets[field];
    }
    uint16_t get(uint8_t field)const {
        const uint8_t n=offset(field);return (uint16_t(data[n])<<8)|data[n+1];
    }
    void set(uint8_t field,uint16_t value) {
        const uint8_t n=offset(field);data[n]=value>>8;data[n+1]=value;
    }
    void writeFrame(uint8_t id,bool save,uint8_t (&cmd)[33])const {
        cmd[0]=id;cmd[1]=0x48;cmd[2]=0xD1;cmd[3]=save;
        for(uint8_t i=0;i<28;++i)cmd[4+i]=data[i];cmd[32]=0x6B;
    }
};
