#include "h30_heading.h"
#include <assert.h>
static H30Attitude frame(float yaw,uint32_t now,float rate=0) {
    static uint16_t sequence=0;
    H30Attitude v;v.yaw=yaw;v.receivedAtMs=now;v.sequence=++sequence;
    v.fields=7;v.accel[2]=9.8f;v.gyro[2]=rate;return v;
}
static void stable(H30HeadingAverage &average,uint32_t start,float yaw) {
    for(uint32_t t=0;t<1000;t+=40)average.add(frame(yaw,start+t));
}
int main() {
    H30HeadingAverage average;average.begin(100,3000);
    average.add(frame(120,100));average.add(frame(120,3099));
    assert(average.count()==0&&!average.collecting(3099));
    // 359/0度跨界必须按圆周平均，不能算成180度。
    for(uint32_t t=0;t<3000;t+=40) {
        float yaw=t%80?1:359;
        average.add(frame(yaw,3100+t));
    }
    assert(average.elapsed(6100)&&average.count()==75);
    assert(fabsf(average.yaw())<0.1f);
    uint32_t count=average.count();average.add(frame(90,6100));assert(average.count()==count);
    H30HeadingReference reference;assert(reference.calibrate(average.yaw(),180));
    float heading=0;assert(reference.get(90,heading)&&fabsf(heading-270)<0.1f);

    average.begin(0);stable(average,0,30);assert(average.count()==25);
    auto repeated=frame(30,1000);average.add(repeated);count=average.count();
    average.add(repeated);assert(average.count()==count);
    average.add(frame(30,1040,3));assert(average.count()==count+1);
    // 角速度、加速度变化不再清空已采集样本。
    auto vibration=frame(60,2080);vibration.accel[0]=1;
    average.add(vibration);assert(average.count()==count+2);

    average.begin(0);for(uint32_t t=0;t<3000;t+=40)average.add(frame(30,t,3));
    assert(average.count()==75&&fabsf(average.yaw()-30)<0.001f);
    average.begin(0);stable(average,0,30);
    auto missing=frame(30,1000);missing.fields=4;average.add(missing);
    assert(average.count()==26); // Yaw平均不再依赖加速度、角速度字段。
    average.begin(0);stable(average,0,30);average.add(frame(30,1200));
    assert(average.count()==26); // 窗口内短暂断流不清空之前的Yaw。
    average.begin(0);stable(average,0,30);
    for(uint32_t t=1000;t<3000;t+=40)average.add(frame(60,t));
    assert(average.count()==75&&average.yaw()>50); // 朝向变化超过3度也直接参与平均。

    average.begin(0xFFFFFF00u);stable(average,0xFFFFFF00u,31);
    assert(average.count()==25&&fabsf(average.yaw()-31)<0.001f);
    assert(average.elapsed(2744));
}
