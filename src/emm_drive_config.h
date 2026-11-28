// 描述张大头驱动器参数配置所用的协议字段。
// 参数在串口设置、驱动器写入和回读验证之间使用相同顺序。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include <stdint.h>  // 引入 stdint.h 中的声明和定义。

// EMM V5 0x42回复中的28字节配置载荷；0x48按原样回写其余配置。
struct EmmDriveConfig {  // 定义 EmmDriveConfig（EmmDrive配置） 的类型成员或作用域。
    uint8_t data[28]={};  // 将 {} 写入或初始化 data[28]。
    // 最大输出电压、开环电流、堵转最大电流、检测转速/电流/时间、到达窗口。
    // 单位依次为mV、mA、mA、RPM、mA、ms、0.1度，全部大端uint16。
    static uint8_t offset(uint8_t field) {  // 定义函数或成员 的入口 offset（偏移），参数由此处的签名确定。
        const uint8_t offsets[]={12,8,10,20,22,24,26};  // 声明 offsets，类型和可用范围由所在声明决定。
        return offsets[field];  // 结束当前函数并返回 offsets[field]。
    }  // 结束此函数或控制块。
    uint16_t get(uint8_t field)const {  // 定义函数或成员 的入口 get，参数由此处的签名确定。
        const uint8_t n=offset(field);return (uint16_t(data[n])<<8)|data[n+1];  // 将 offset(field) 写入或初始化 n。然后结束当前函数并返回 (uint16_t(data[n])<<8)|data[n+1]。
    }  // 结束此函数或控制块。
    void set(uint8_t field,uint16_t value) {  // 定义函数或成员 的入口 set，参数由此处的签名确定。
        const uint8_t n=offset(field);data[n]=value>>8;data[n+1]=value;  // 将 offset(field) 写入或初始化 n。然后将 value>>8 写入或初始化 data[n]。然后将 value 写入或初始化 data[n+1]。
    }  // 结束此函数或控制块。
    void writeFrame(uint8_t id,bool save,uint8_t (&cmd)[33])const {  // 定义函数或成员 的入口 writeFrame（write帧），参数由此处的签名确定。
        cmd[0]=id;cmd[1]=0x48;cmd[2]=0xD1;cmd[3]=save;  // 将 id 写入或初始化 cmd[0]。然后将 0x48 写入或初始化 cmd[1]。然后将 0xD1 写入或初始化 cmd[2]。然后将 save 写入或初始化 cmd[3]。
        for(uint8_t i=0;i<28;++i)cmd[4+i]=data[i];cmd[32]=0x6B;  // 按 uint8_t i=0;i<28;++i)cmd[4+i]=data[i];cmd[32]=0x6B; 组织循环的初值、继续条件和步进。
    }  // 结束此函数或控制块。
};  // 结束此类型定义。
