#pragma once
#include "h30_protocol.h"
#include <math.h>

// “零轮速命令已生效 + 惯性安静”，不等同于外部定位或轮编码器到位。
// 不累计重复帧；任何扰动/断流/缺项重新开始观察，不用固定延时宣布成功。
class H30MotionQuiet {
public:
    void reset(){*this=H30MotionQuiet{};}
    bool update(const H30Attitude &v,bool stopped) {
        if(!stopped){reset();return false;}
        if(seen&&v.sequence==sequence)return quiet;
        if((v.fields&7)!=7){reset();return false;}
        bool gap=seen&&(v.receivedAtMs-lastMs>100||v.receivedAtMs==lastMs);
        seen=true;sequence=v.sequence;lastMs=v.receivedAtMs;
        float g2=0,a2=0;
        for(int i=0;i<3;++i) {
            if(!isfinite(v.gyro[i])||!isfinite(v.accel[i])){reset();return false;}
            g2+=v.gyro[i]*v.gyro[i];a2+=v.accel[i]*v.accel[i];
        }
        bool calm=!gap&&g2<=4&&a2>=8.8f*8.8f&&a2<=10.8f*10.8f;
        if(!calm){count=0;quiet=false;return false;}
        bool spread=false;
        for(int i=0;i<3;++i)if(count&&fabsf(v.accel[i]-anchor[i])>0.25f)spread=true;
        if(!count||spread) {
            firstMs=v.receivedAtMs;count=1;quiet=false;
            for(int i=0;i<3;++i)anchor[i]=v.accel[i];
        } else if(count<255)++count;
        quiet=count>=4&&v.receivedAtMs-firstMs>=120;return quiet;
    }
    uint32_t stableMs() const{return count?lastMs-firstMs:0;}
private:
    bool seen=false,quiet=false;uint8_t count=0;uint16_t sequence=0;
    uint32_t lastMs=0,firstMs=0;float anchor[3]={};
};

// 保持既有地图参考的航向PI+D阻尼。D来自欧拉Yaw变化率，避免倒装轴符号猜测。
class H30HeadingHold {
public:
    void reset(){*this=H30HeadingHold{};}
    float update(float heading,float error,uint32_t stamp,float kp,float limit) {
        if(!ready){previous=heading;previousMs=stamp;ready=true;return clamp(kp*error,limit);}
        if(stamp!=previousMs) {
            float dt=(stamp-previousMs)*.001f;
            if(dt>0&&dt<=.2f) {
                float delta=fmodf(heading-previous+540,360)-180;
                rate+=(delta/dt-rate)*dt/(.08f+dt);
                if(fabsf(error)<=5)integral=clamp(integral+.3f*error*dt,5);
            } else {rate=0;integral=0;}
            previous=heading;previousMs=stamp;
        }
        return clamp(kp*error+integral-.12f*rate,limit);
    }
    float measuredRate()const{return rate;}
private:
    bool ready=false;float previous=0,rate=0,integral=0;uint32_t previousMs=0;
    static float clamp(float x,float limit){return fmaxf(-limit,fminf(limit,x));}
};
