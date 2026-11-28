// 为运动控制整理 H30 样本中的姿态与运动信息。
// 路径控制使用样本时间及角度变化判断是否可以推进当前阶段。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include "h30_protocol.h"  // 引入 h30_protocol.h 中的声明和定义。
#include <math.h>  // 引入 math.h 中的声明和定义。

// “零轮速命令已生效 + 惯性安静”，不等同于外部定位或轮编码器到位。
// 不累计重复帧；任何扰动/断流/缺项重新开始观察，不用固定延时宣布成功。
class H30MotionQuiet {  // 定义 H30MotionQuiet 的类型成员或作用域。
public:  // 此后的成员采用 public 访问权限，直到下一个权限标签。
    /**
     * @brief 清空惯性停稳判定的样本计数、时间和安静状态。
     */
    void reset(){*this=H30MotionQuiet{};}  // 将 H30MotionQuiet{};} 写入或初始化 this。
    /**
     * @brief 输入一帧惯性数据，结合零轮速命令、加速度和角速度判定是否连续停稳。
     * @param v 同一帧的姿态、加速度和角速度样本，须含完整字段。
     * @param stopped 是否已下发并应用零轮速命令。
     * @return true 满足连续安静样本和持续时间要求，不代表外部定位确认。
     */
    bool update(const H30Attitude &v,bool stopped) {  // 定义函数或成员 的入口 update，参数由此处的签名确定。
        if(!stopped){reset();return false;}  // 当 !stopped 成立时进入此分支，否则继续后续流程。
        if(seen&&v.sequence==sequence)return quiet;  // 若 seen&&v.sequence==sequence，立即返回 quiet。
        if((v.fields&7)!=7){reset();return false;}  // 当 (v.fields&7)!=7 成立时进入此分支，否则继续后续流程。
        bool gap=seen&&(v.receivedAtMs-lastMs>100||v.receivedAtMs==lastMs);  // 将 seen&&(v.receivedAtMs-lastMs>100||v.receivedAtMs==last… 写入或初始化 gap。
        seen=true;sequence=v.sequence;lastMs=v.receivedAtMs;  // 将 true 写入或初始化 seen。然后将 v.sequence 写入或初始化 sequence。然后将 v.receivedAtMs 写入或初始化 lastMs（最后毫秒）。
        float g2=0,a2=0;  // 将 0,a2=0 写入或初始化 g2。
        for(int i=0;i<3;++i) {  // 按 int i=0;i<3;++i 组织循环的初值、继续条件和步进。
            if(!isfinite(v.gyro[i])||!isfinite(v.accel[i])){reset();return false;}  // 当 !isfinite(v.gyro[i])||!isfinite(v.accel[i]) 成立时进入此分支，否则继续后续流程。
            g2+=v.gyro[i]*v.gyro[i];a2+=v.accel[i]*v.accel[i];  // 把 v.gyro[i]*v.gyro[i] 累加到 g2。然后把 v.accel[i]*v.accel[i] 累加到 a2。
        }  // 结束此循环体。
        // 静止检查比较三轴模长的平方，阈值因此也用平方。
        // 后面再检查各轴相对窗口起点的漂移，并要求样本数与时间同时满足。
        bool calm=!gap&&g2<=4&&a2>=8.8f*8.8f&&a2<=10.8f*10.8f;  // 将 !gap&&g2<=4&&a2>=8.8f*8.8f&&a2<=10.8f*10.8f 写入或初始化 calm。
        if(!calm){count=0;quiet=false;return false;}  // 当 !calm 成立时进入此分支，否则继续后续流程。
        bool spread=false;  // 将 false 写入或初始化 spread。
        for(int i=0;i<3;++i)if(count&&fabsf(v.accel[i]-anchor[i])>0.25f)spread=true;  // 按 int i=0;i<3;++i)if(count&&fabsf(v.accel[i]-anchor[i])>… 组织循环的初值、继续条件和步进。
        if(!count||spread) {  // 当 !count||spread 成立时进入此分支，否则继续后续流程。
            firstMs=v.receivedAtMs;count=1;quiet=false;  // 将 v.receivedAtMs 写入或初始化 firstMs（首个毫秒）。然后将 1 写入或初始化 count（计数）。然后将 false 写入或初始化 quiet。
            for(int i=0;i<3;++i)anchor[i]=v.accel[i];  // 按 int i=0;i<3;++i)anchor[i]=v.accel[i]; 组织循环的初值、继续条件和步进。
        } else if(count<255)++count;  // 仅在先前条件均不成立时，当 count<255 成立时进入此分支，否则继续后续流程。
        quiet=count>=4&&v.receivedAtMs-firstMs>=120;return quiet;  // 将 count>=4&&v.receivedAtMs-firstMs>=120 写入或初始化 quiet。然后结束当前函数并返回 quiet。
    }  // 结束此函数或控制块。
    /**
     * @brief 读取已累计的停稳持续时间(ms)，没有样本时返回0。
     */
    uint32_t stableMs() const{return count?lastMs-firstMs:0;}  // 定义 stableMs（稳定毫秒），调用时返回 count?lastMs-firstMs:0。
private:  // 此后的成员采用 private 访问权限，直到下一个权限标签。
    bool seen=false,quiet=false;uint8_t count=0;uint16_t sequence=0;  // 将 false,quiet=false 写入或初始化 seen。然后将 0 写入或初始化 count（计数）。然后将 0 写入或初始化 sequence。
    uint32_t lastMs=0,firstMs=0;float anchor[3]={};  // 将 0,firstMs=0;float anchor[3]={} 写入或初始化 lastMs（最后毫秒）。
};  // 结束此类型定义。

// 保持既有地图参考的航向PI+D阻尼。D来自欧拉Yaw变化率，避免倒装轴符号猜测。
class H30HeadingHold {  // 定义 H30HeadingHold（H30航向保持） 的类型成员或作用域。
public:  // 此后的成员采用 public 访问权限，直到下一个权限标签。
    /**
     * @brief 清空航向保持的历史航向、积分和角速度滤波状态。
     */
    void reset(){*this=H30HeadingHold{};}  // 将 H30HeadingHold{};} 写入或初始化 this。
    /**
     * @brief 根据地图航向误差计算带积分和角速度阻尼的航向纠偏输出。
     * @param heading 当前地图航向(deg)。
     * @param error 目标与当前航向的有符号误差(deg)。
     * @param stamp 当前航向样本时间(ms)。
     * @param kp 比例系数。
     * @param limit 纠偏角速度的正负限幅(deg/s)。
     * @return 纠偏角速度(deg/s)。
     */
    float update(float heading,float error,uint32_t stamp,float kp,float limit) {  // 定义函数或成员 的入口 update，参数由此处的签名确定。
        if(!ready){previous=heading;previousMs=stamp;ready=true;return clamp(kp*error,limit);}  // 当 !ready 成立时进入此分支，否则继续后续流程。
        if(stamp!=previousMs) {  // 当 stamp!=previousMs 成立时进入此分支，否则继续后续流程。
            float dt=(stamp-previousMs)*.001f;  // 将 (stamp-previousMs)*.001f 写入或初始化 dt（时间差）。
            if(dt>0&&dt<=.2f) {  // 当 dt>0&&dt<=.2f 成立时进入此分支，否则继续后续流程。
                // 把相邻地图航向的差折回最短角度范围，再除以采样间隔得到角速度。
                // 低通后的角速度用于输出阻尼，积分只在误差较小时累加。
                float delta=fmodf(heading-previous+540,360)-180;  // 将 fmodf(heading-previous+540,360)-180 写入或初始化 delta。
                rate+=(delta/dt-rate)*dt/(.08f+dt);  // 把 (delta/dt-rate)*dt/(.08f+dt) 累加到 rate。
                if(fabsf(error)<=5)integral=clamp(integral+.3f*error*dt,5);  // 当 fabsf(error)<=5 成立时进入此分支，否则继续后续流程。
            } else {rate=0;integral=0;}  // 前一个判断不成立时执行此分支。
            previous=heading;previousMs=stamp;  // 将 heading 写入或初始化 previous（上一）。然后将 stamp 写入或初始化 previousMs（上一毫秒）。
        }  // 结束此条件分支。
        return clamp(kp*error+integral-.12f*rate,limit);  // 结束当前函数并返回 clamp(kp*error+integral-.12f*rate,limit)。
    }  // 结束此函数或控制块。
    /**
     * @brief 读取由相邻航向样本计算的角速度(deg/s)。
     */
    float measuredRate()const{return rate;}  // 定义 measuredRate，调用时返回 rate。
private:  // 此后的成员采用 private 访问权限，直到下一个权限标签。
    bool ready=false;float previous=0,rate=0,integral=0;uint32_t previousMs=0;  // 将 false 写入或初始化 ready（就绪）。然后将 0,rate=0,integral=0 写入或初始化 previous（上一）。然后将 0 写入或初始化 previousMs（上一毫秒）。
    /**
     * @brief 将航向控制输出对称限制在正负limit之间。
     */
    static float clamp(float x,float limit){return fmaxf(-limit,fminf(limit,x));}  // 定义函数或成员 clamp，参数由此处的签名确定。
};  // 结束此类型定义。
