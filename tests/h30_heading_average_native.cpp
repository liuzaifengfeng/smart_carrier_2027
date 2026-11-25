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
    H30HeadingAverage average;average.begin(100,500);
    average.add(frame(120,100));average.add(frame(120,599));
    assert(average.result().count==0&&!average.collecting(599));
    // 359/0度跨界必须按圆周平均，不能算成180度。
    for(uint32_t t=0;t<3000;t+=40) {
        float yaw=t%80?1:359;
        average.add(frame(yaw,600+t));
    }
    assert(average.elapsed(3600)&&average.result().count==75);
    assert(fabsf(average.result().yaw)<0.1f);
    uint32_t count=average.result().count;average.add(frame(90,3600));assert(average.result().count==count);
    H30HeadingReference reference;assert(reference.calibrate(average.result().yaw,180));
    float heading=0;assert(reference.get(90,heading)&&fabsf(heading-270)<0.1f);

    average.begin(0);stable(average,0,30);assert(average.result().count==25);
    auto repeated=frame(30,1000);average.add(repeated);count=average.result().count;
    average.add(repeated);assert(average.result().count==count);
    average.add(frame(30,1040,3));assert(average.result().count==count+1);
    auto rotating=frame(32,1080,4);
    rotating.gyro[0]=4; // 单轴在阈值内，三轴模长超限。
    average.add(rotating);
    auto accelerated=frame(32,1120);accelerated.accel[2]=12;
    average.add(accelerated);
    auto missing=frame(32,1160);missing.fields=4;
    average.add(missing);
    assert(average.result().count==count+1);
    average.add(frame(30,1400));
    assert(average.result().count==count+2); // 扰动和短暂断流后继续采样。

    average.begin(0);
    average.add(frame(90,0));
    for(uint32_t t=40;t<440;t+=40)average.add(frame(8,t)); // 连续10帧跳变。
    for(uint32_t t=440;t<3000;t+=40)
        average.add(frame(t%80?359:1,t));
    assert(average.result().count==64&&fabsf(average.result().yaw)<0.1f);

    average.begin(0);stable(average,0,0);
    average.add(frame(355,1000));average.add(frame(5,1040));
    average.add(frame(354.9f,1080));average.add(frame(5.1f,1120));
    assert(average.result().count==27&&fabsf(average.result().yaw)<0.001f);

    average.begin(0);
    for(uint32_t t=0;t<3000;t+=40)average.add(frame(30,t,8));
    assert(average.elapsed(3000)&&average.result().count==0);

    average.begin(0xFFFFFF00u);stable(average,0xFFFFFF00u,31);
    assert(average.result().count==25&&fabsf(average.result().yaw-31)<0.001f);
    assert(average.elapsed(2744));
}
