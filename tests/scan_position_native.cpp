#include "../src/continuous_route.h"
#include "../src/position_move_timing.h"
#include <stdio.h>
#include <stdlib.h>
struct RobotPose {float x,y,theta;};
RobotPose currentPose;
float Y_PULSE=13.6f;
struct Logger {template<class... T>void printf(const char *,T...) {}} Serial;
static double traveled=0;
static bool complete=true;
static int reads=0;
constexpr double PI=3.14159265358979323846;
float normalizeHeading(float angle){return continuous::wrap(angle);}
RobotPose positionTarget(float x,float y) {
#include "position_target_actual.inc"
    return {static_cast<float>(targetX),static_cast<float>(targetY),currentPose.theta};
}
bool ReadChassisEncoderPositions(int64_t (&values)[4]) {
    for(int i=0;i<4;++i)values[i]=(i+1)*1000000+
        (reads%2?llround(traveled*Y_PULSE*65536/3200)*(i%2?1:-1):0);
    ++reads;return true;
}
bool MovePosition(float,float,float,float,bool (*)()) {
    currentPose={9999,9999,0}; // 提前停车时不能使用命令终点，也不能沿用旧位姿。
    return complete;
}
#include "scan_position_actual.inc"
#define CHECK(x) do{if(!(x)){fprintf(stderr,"scan pose line %d: %s\n",__LINE__,#x);return 2;}}while(0)
int main() {
    // 700mm@2000RPM和400mm@1000RPM为短程三角形曲线，不能按最高轮速匀速估时。
    CHECK(fabs(PositionMoveDurationMs(9520,2000,180)-1647.1794)<.01);
    CHECK(fabs(PositionMoveDurationMs(5440,1000,180)-1245.1506)<.01);
    CHECK(fabs(PositionMoveDurationMs(9520,60,180)-3203.0)<.01);
    for(int zone:{1,2}) {
        const float sign=zone==1?1:-1;
        currentPose={zone==1?2250.0f:150.0f,150,180};
        // 执行实际MovePosition终点换算，覆盖扫码前的雷达准备动作。
        currentPose=positionTarget(125,zone==1?100:-100);
        CHECK(fabsf(currentPose.x-(zone==1?2150:250))<.01f);
        CHECK(fabsf(currentPose.y-275)<.01f);
        const float initial=currentPose.x;
        traveled=700;complete=true;
        CHECK(MoveScanPosition(sign*700,60,nullptr));
        CHECK(fabsf(currentPose.x-(initial-sign*700))<.01f);
        traveled=125;complete=false;
        CHECK(!MoveScanPosition(sign*400,30,nullptr));
        CHECK(fabsf(currentPose.x-(initial-sign*825))<.01f);
        traveled=60;
        CHECK(!MoveScanPosition(-sign*400,30,nullptr));
        CHECK(fabsf(currentPose.x-(initial-sign*765))<.01f);
        CHECK(fabsf(currentPose.y-275)<.01f&&currentPose.theta==180);
    }
    // 四个朝向均遵循车身+Y为前方、车身+X为右侧。
    for(float heading:{0.0f,90.0f,180.0f,270.0f}) {
        currentPose={1200,1200,heading};
        const auto expected=continuous::bodyToWorld({125,100},heading);
        const auto target=positionTarget(125,100);
        CHECK(fabsf(target.x-1200-expected.x)<.01f&&fabsf(target.y-1200-expected.y)<.01f);
    }
    puts("scan encoder displacement, both start zones and early stop OK");return 0;
}
