#include "motion_hardware.h"
#include "motion_executor.h"
#include "node_position_control.h"
#include "chassis.h"
#include "material_transfer.h"
#include <assert.h>
#include <string.h>

NativeSerial Serial;
uint32_t clockMs=0;bool stop=false,servoAvailable=true;
float turretMeasured=0,clawMeasured=0;
uint8_t flags=3;
bool motorAtTarget=true;
int issued=0;
uint16_t issuedRamp=0;bool issuedSlowLower=false;
uint32_t millis(){return clockMs;}
volatile bool taskMotionAborted=false;
uint32_t ARM_FEEDBACK_ENABLED=0,ARM_GRIP_HOLD_MS=0,ARM_RELEASE_HOLD_MS=0;
float HEIGHT_PULSE=80,LENGTH_PULSE=30.2f;
ArmPose currentArm{100,20,90,50};
CargoPlatform cargoPlatforms[3]={{MATERIAL_NONE},{MATERIAL_NONE},{MATERIAL_NONE}};
bool cancelled(){return stop;}
bool MoveArm(float,float,float,float,float,uint16_t ramp,bool slowLower){++issued;issuedRamp=ramp;issuedSlowLower=slowLower;return true;}
bool MoveArmPlanned(float,float,float,float,float,uint16_t ramp,bool slowLower){++issued;issuedRamp=ramp;issuedSlowLower=slowLower;return true;}
bool BeginPositionMove(float,float,float,float){++issued;return true;}
void TickPositionMove(){}
bool PositionMoveActive(){return false;}
void GotoPose(float,float,float,bool){++issued;}
void Servo_SetAngle(uint8_t,float,uint16_t,uint16_t){++issued;}
void Emm_V5_En_Control_all(bool){++issued;}
void MovePose(int,float,bool){++issued;}
bool ReadArmTurretAngle(float &angle){angle=turretMeasured;return servoAvailable;}
bool Servo_QueryAngleMTurn(uint8_t id,float &angle,uint32_t){angle=id==2?turretMeasured:clawMeasured;return servoAvailable;}
const char *ReadMotorFeedback(uint8_t id,node_position::Motor &state) {
    state.flags=flags;
    const float target=id==5?100:20,ppm=id==5?HEIGHT_PULSE:LENGTH_PULSE;
    state.position=llround((target+(motorAtTarget?0:10))*ppm*65536/3200);return nullptr;
}
#include "motion_hardware_actual.inc"
void look3() {
    MotionHardware hardware;hardware.configure(1,cancelled);
    motion::Step s;s.kind=motion::StepKind::TURRET_READY;s.turret=90;s.waitMs=1500;
    assert(!hardware.issue(s)&&hardware.feedback(s));bool done=false;
    turretMeasured=84.9f;assert(!hardware.ready(s,done)&&!done);
    turretMeasured=85;assert(!hardware.ready(s,done)&&done);
    turretMeasured=95;assert(!hardware.ready(s,done)&&done);
    turretMeasured=95.1f;clockMs=1499;assert(!hardware.ready(s,done)&&!done);
    clockMs=1500;assert(hardware.ready(s,done)&&!done);
    servoAvailable=false;clockMs=100;hardware.issue(s);clockMs=1599;assert(!hardware.ready(s,done)&&!done);
    clockMs=1600;assert(hardware.ready(s,done));
}
void feedback() {
    MotionHardware hardware;hardware.configure(1,cancelled);
    motion::Step s;s.kind=motion::StepKind::ARM;s.high=100;s.length=20;s.turret=90;s.pawl=50;s.speed=200;
    assert(!hardware.feedback(s));ARM_FEEDBACK_ENABLED=1;assert(hardware.feedback(s));
    s.timed=true;assert(!hardware.feedback(s));s.timed=false;
    bool done=false;turretMeasured=90;clawMeasured=50;flags=1;
    assert(!hardware.ready(s,done)&&!done); // 目标缓存相同，驱动器尚未到位。
    flags=3;motorAtTarget=false;assert(!hardware.ready(s,done)&&!done);
    motorAtTarget=true;turretMeasured=70;assert(!hardware.ready(s,done)&&!done);
    turretMeasured=90;clawMeasured=30;assert(!hardware.ready(s,done)&&!done);
    clawMeasured=50;assert(!hardware.ready(s,done)&&done);
    s.turretRampMs=20;assert(!hardware.issue(s)&&issuedRamp==20&&!issuedSlowLower);
    s.planned=true;s.turretRampMs=500;s.slowLower=true;assert(!hardware.issue(s)&&issuedRamp==500&&issuedSlowLower);
    issued=0;
    stop=true;assert(hardware.issue(s)&&!issued);
    stop=false;s.kind=motion::StepKind::COMMIT;s.cargo=1;s.color=6;
    assert(!hardware.issue(s)&&cargoPlatforms[0].material==MATERIAL_LIGHT_BLUE);
}
int main(int argc,char **argv) {
    assert(argc==2);if(!strcmp(argv[1],"look3"))look3();else if(!strcmp(argv[1],"feedback"))feedback();else assert(false);
}
