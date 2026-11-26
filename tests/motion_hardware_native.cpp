#include "motion_hardware.h"
#include "motion_executor.h"
#include "node_position_control.h"
#include "chassis.h"
#include "material_transfer.h"
#include "arm_placement_log.h"
#include "servo.h"
#include <assert.h>
#include <string.h>

NativeSerial Serial;
uint32_t clockMs=0;bool stop=false,servoAvailable=true;
float turretMeasured=0,clawMeasured=0;
float trayMeasured=0,trayTarget=0;
float trayTargetAtArmIssue=0;
uint32_t trayInterval=0;
uint16_t trayPower=0;
bool trayHeld=false;
uint8_t flags=3;
bool motorAtTarget=true;
float extensionFeedbackMm=20;
int16_t extensionFeedbackRpm=0;
bool customMotor=false;
int64_t customPosition=0;
const char *motorError=nullptr;
int issued=0;
float issuedLength=-1;
float issuedTurret=0;
uint16_t issuedRamp=0;bool issuedSlowLower=false;
uint8_t issuedLengthAcceleration=0;
uint8_t issuedSlowLowerAcceleration=0;
uint16_t issuedTurretRpm=0;
uint32_t turretCommands=0,cancelledTurretCommand=0,activeTurretCommand=0;
bool turretSendFails=false;
TurretMotorPosition issuedTurretPosition{};
uint32_t millis(){return clockMs;}
volatile bool taskMotionAborted=false;
uint32_t ARM_FEEDBACK_ENABLED=0,ARM_GRIP_HOLD_MS=0,ARM_RELEASE_HOLD_MS=0;
float HEIGHT_PULSE=80,LENGTH_PULSE=30.2f;
ArmPose currentArm{100,20,90,50};
CargoPlatform cargoPlatforms[3]={{MATERIAL_NONE},{MATERIAL_NONE},{MATERIAL_NONE}};
float cargoTrayAngles[3]={150,213,276};
bool cancelled(){return stop;}
bool MoveArm(float,float length,float turret,float,float,uint8_t ramp,bool slowLower,uint8_t lengthAcceleration,uint8_t slowLowerAcceleration){++issued;issuedLength=length;issuedTurret=turret;issuedRamp=ramp;issuedSlowLower=slowLower;issuedLengthAcceleration=lengthAcceleration;issuedSlowLowerAcceleration=slowLowerAcceleration;trayTargetAtArmIssue=trayTarget;return true;}
void LedPwm_SetRingVisionActive(bool){}
bool MoveArmPlanned(float,float length,float turret,float,float,uint8_t ramp,bool slowLower,uint16_t rpm){++issued;issuedLength=length;issuedTurret=turret;issuedRamp=ramp;issuedSlowLower=slowLower;issuedTurretRpm=rpm;return true;}
TurretMotorPosition TurretMotorQuantizePosition(float angle) {
    return {static_cast<uint8_t>(angle<0?0:1),
        static_cast<uint32_t>(lroundf(fabsf(angle)*TURRET_MOTOR_PULSES_PER_REV/360.0f))};
}
bool TurretMotorMoveQuantized(float angle,const TurretMotorPosition &position,uint8_t ramp,uint16_t rpm,uint32_t &id) {
    if(turretSendFails)return false;
    id=++turretCommands;
    activeTurretCommand=id;
    issuedTurretPosition=position;
    issuedTurret=angle;
    issuedRamp=ramp;
    issuedTurretRpm=rpm;
    return true;
}
bool TurretMotorCommandCancelled(uint32_t id){return id==cancelledTurretCommand;}
bool TurretMotorCommandActive(uint32_t id){return id==activeTurretCommand;}
bool BeginPositionMove(float,float,float,float,uint8_t){++issued;return true;}
void TickPositionMove(){}
bool PositionMoveActive(){return false;}
void GotoPose(float,float,float,bool){++issued;}
bool updateDisplay(const char *,const char *,const char *){return true;}
void Servo_SetAngle(uint8_t,float,uint16_t,uint16_t){++issued;}
void Servo_SetAngleMTurn(uint8_t id,float angle,uint32_t interval,uint16_t power){
    assert(id==SERVO_ID_CARGO_TRAY);trayTarget=angle;trayInterval=interval;trayPower=power;
}
int smoothTrayCommands=0;
void Servo_MoveCargoTray(float angle){trayTarget=angle;++smoothTrayCommands;}
void Servo_HoldPosition(uint8_t id){assert(id==SERVO_ID_CARGO_TRAY);trayHeld=true;}
void Emm_V5_En_Control_all(bool){++issued;}
void MovePose(int,float,bool){++issued;}
bool ReadArmTurretAngle(float &angle){angle=turretMeasured;return servoAvailable;}
bool Servo_QueryAngleMTurn(uint8_t id,float &angle,uint32_t){
    angle=id==SERVO_ID_CARGO_TRAY?trayMeasured:id==2?turretMeasured:clawMeasured;return servoAvailable;
}
const char *ReadMotorFeedback(uint8_t id,node_position::Motor &state) {
    if(customMotor){assert(id==5);state.flags=flags;state.position=customPosition;return motorError;}
    state.flags=flags;
    const float target=id==5?100:extensionFeedbackMm,ppm=id==5?HEIGHT_PULSE:LENGTH_PULSE;
    if(id==6)state.rpm=extensionFeedbackRpm;
    state.position=llround((target+(motorAtTarget?0:10))*ppm*65536/3200);return nullptr;
}
int placementReads=0;
const char *ReadMotorEncoderPosition(uint8_t id,int64_t &position) {
    assert(id==6);++placementReads;position=customPosition;return motorError;
}
#include "motion_hardware_actual.inc"
void trayArrival() {
    MotionHardware hardware;hardware.configure(22,cancelled);
    motion::Step s;s.kind=motion::StepKind::TRAY_ROTATE;s.turret=260;s.waitMs=900;s.trayPowerMw=450;
    assert(!hardware.issue(s)&&hardware.feedback(s));
    assert(trayTarget==260&&trayInterval==900&&trayPower==450);
    bool done=false;trayMeasured=-100;
    assert(!hardware.ready(s,done)&&!done);
    trayMeasured=259.5f;
    assert(!hardware.ready(s,done)&&done);
    hardware.stopTray();assert(!trayHeld);
    assert(!hardware.issue(s));servoAvailable=false;
    assert(hardware.ready(s,done)&&!done);
    hardware.stopTray();assert(trayHeld);

    s=motion::Step{};
    s.kind=motion::StepKind::ARM;s.turretArrival=true;s.trayClearanceCargo=3;
    cargoPlatforms[0].material=MATERIAL_RED;
    assert(!hardware.issue(s)&&trayTarget==260); // 1 号有料，但 3 号只检查 2 号。
    cargoPlatforms[1].material=MATERIAL_BLUE;
    assert(!hardware.issue(s)&&trayTarget==306);
    s.trayClearanceCargo=2;
    servoAvailable=true;turretMeasured=currentArm.turret_angle;
    assert(!hardware.issue(s)&&trayTarget==243&&trayInterval==0);
    assert(!hardware.ready(s,done)&&done); // 沿用机械臂转向的到位条件。
    s.trayClearanceCargo=1;
    assert(!hardware.issue(s)&&trayTarget==243);

    // 任务码 513+312：黑色从料盘 3 去工位 3，此时两个较小盘位都有料。
    cargoPlatforms[2].material=MATERIAL_BLACK;
    s.trayClearanceCargo=3;
    assert(!hardware.issue(s)&&trayTarget==306);
    trayMeasured=276;
    assert(hardware.feedback(s)&&!hardware.ready(s,done)&&done);
    s=motion::Step{};
    s.kind=motion::StepKind::ARM;s.high=10;s.trayWork3LimitCargo=3;
    assert(!hardware.issue(s)&&trayTargetAtArmIssue==239);
    cargoPlatforms[1].material=MATERIAL_NONE;
    trayTarget=276;
    assert(!hardware.issue(s)&&trayTarget==239); // 仅 1 号有料也要避让。
    cargoPlatforms[0].material=MATERIAL_NONE;
    trayTarget=276;
    assert(!hardware.issue(s)&&trayTarget==276);

    s=motion::Step{};
    s.kind=motion::StepKind::TRAY_START;s.turret=150;
    assert(!hardware.issue(s)&&!hardware.feedback(s)&&trayTarget==150&&smoothTrayCommands==1);
    s.kind=motion::StepKind::TRAY_READY;
    trayTarget=123;trayMeasured=100;
    assert(!hardware.issue(s)&&hardware.feedback(s)&&trayTarget==123);
    assert(!hardware.ready(s,done)&&!done);
    trayMeasured=150;
    assert(!hardware.ready(s,done)&&done);
    trayMeasured=147;
    assert(!hardware.ready(s,done)&&!done);
    s.trayArrivalToleranceDeg=3;
    assert(!hardware.ready(s,done)&&done);
    trayMeasured=146.9f;
    assert(!hardware.ready(s,done)&&!done);
    s.kind=motion::StepKind::TRAY_START;s.turret=0;
    assert(!hardware.issue(s)&&!hardware.feedback(s)&&trayTarget==0&&smoothTrayCommands==2);

    // 到站时下发 0°→276°，跨作业后首件沿用同一条指令，但实测仍未到位。
    hardware.preselectTray(276);
    assert(trayTarget==276&&smoothTrayCommands==3);
    hardware.configure(23,cancelled);
    s.turret=276;
    assert(!hardware.issue(s)&&smoothTrayCommands==3);
    s.kind=motion::StepKind::TRAY_READY;
    trayMeasured=240;
    assert(!hardware.ready(s,done)&&!done);
    trayMeasured=276;
    assert(!hardware.ready(s,done)&&done);
    hardware.preselectTray(213);
    hardware.stopTray();
    s.kind=motion::StepKind::TRAY_START;s.turret=213;
    assert(!hardware.issue(s)&&smoothTrayCommands==5); // 停车后不能复用原先的预选。
}
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
void turretArrival() {
    MotionHardware hardware;hardware.configure(19,cancelled);
    motion::Step s;s.kind=motion::StepKind::ARM;s.turretArrival=true;s.turret=350;
    // 底层保留350°绝对目标，必须比较真实多圈值，不能将-10°当作同一目标。
    currentArm.turret_angle=350;turretMeasured=310;ARM_FEEDBACK_ENABLED=0;
    assert(!hardware.issue(s)&&hardware.feedback(s));bool done=false;
    assert(!hardware.ready(s,done)&&!done);
    currentArm.turret_angle=99; // 发令后缓存变化不应改变本步骤保存的目标。
    turretMeasured=344.9f;assert(!hardware.ready(s,done)&&!done);
    turretMeasured=345;assert(!hardware.ready(s,done)&&done);
    turretMeasured=355;assert(!hardware.ready(s,done)&&done);
    turretMeasured=355.1f;assert(!hardware.ready(s,done)&&!done);
    turretMeasured=-10;assert(!hardware.ready(s,done)&&!done); // 不把错圈当作到位。
    s.length=20;s.lengthArrival=true;
    turretMeasured=350;motorAtTarget=false;
    assert(!hardware.ready(s,done)&&!done);
    motorAtTarget=true;
    assert(!hardware.ready(s,done)&&done);
    servoAvailable=false;assert(hardware.ready(s,done));servoAvailable=true;
    stop=true;assert(hardware.ready(s,done));stop=false;

    s.name="LOOK4_READY";s.turret=90;s.length=0;s.lengthArrivalByPosition=true;
    currentArm.turret_angle=turretMeasured=90;
    flags=0;extensionFeedbackMm=9;extensionFeedbackRpm=8;
    assert(!hardware.issue(s));
    assert(!hardware.ready(s,done)&&done); // LOOK4 仅检查位置误差。
    extensionFeedbackMm=11;motorAtTarget=true;
    assert(!hardware.ready(s,done)&&!done);
    s.turret=270;s.length=150;
    currentArm.turret_angle=turretMeasured=270;
    assert(!hardware.issue(s));
    extensionFeedbackMm=159;
    assert(!hardware.ready(s,done)&&done);
    extensionFeedbackMm=139;
    assert(!hardware.ready(s,done)&&!done);
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
    s.turretAcceleration=TURRET_ACCELERATION_FAST;assert(!hardware.issue(s)&&issuedRamp==TURRET_ACCELERATION_FAST&&!issuedSlowLower);
    assert(issuedLengthAcceleration==240);
    s.lengthAcceleration=245;assert(!hardware.issue(s)&&issuedLengthAcceleration==245);
    s.slowLower=true;s.slowLowerAcceleration=200;
    assert(!hardware.issue(s)&&issuedSlowLower&&issuedSlowLowerAcceleration==200);
    s.planned=true;s.turretAcceleration=TURRET_ACCELERATION_SLOW;s.slowLower=true;assert(!hardware.issue(s)&&issuedRamp==TURRET_ACCELERATION_SLOW&&issuedSlowLower);
    assert(issuedTurretRpm==83);
    issued=0;
    stop=true;assert(hardware.issue(s)&&!issued);
    stop=false;s.kind=motion::StepKind::COMMIT;s.cargo=1;s.color=6;
    assert(!hardware.issue(s)&&cargoPlatforms[0].material==MATERIAL_LIGHT_BLUE);
}
void liftArrival() {
    MotionHardware hardware;hardware.configure(20,cancelled);
    motion::Step lift;lift.kind=motion::StepKind::ARM;lift.name="INIT_LIFT";
    lift.high=160;lift.liftArrival=true;
    assert(hardware.feedback(lift)&&!ARM_FEEDBACK_ENABLED);
    motion::Program program;program.append(lift);
    motion::Step turn;turn.kind=motion::StepKind::ARM;turn.name="INIT_TURN";turn.turret=-55;program.append(turn);
    motion::Sequence sequence;sequence.begin(program,hardware);
    customMotor=true;customPosition=0;currentArm.high=160;
    sequence.tick();assert(issued==1);
    clockMs=900;sequence.tick();assert(sequence.stepIndex()==0&&issued==1);
    customPosition=262144; // 反向160mm不允许结束抬升。
    clockMs+=50;sequence.tick();assert(sequence.stepIndex()==0&&issued==1);
    customPosition=-262144;flags=1; // 实测位置到了，但驱动器尚未报告到位。
    clockMs+=50;sequence.tick();assert(sequence.stepIndex()==0&&issued==1);
    flags=3;
    clockMs+=50;sequence.tick();assert(sequence.stepIndex()==1&&issued==1);
    sequence.tick();assert(issued==2);
    assert(Serial.text.find("LIFT_ARRIVED,20,160.000,160.000")!=std::string::npos);
    bool done=false;motorError="MOTOR_FEEDBACK_TIMEOUT";
    assert(hardware.ready(lift,done)&&!done);
    motorError=nullptr;assert(!hardware.ready(lift,done)&&done); // 通信恢复后可以再次检查。
    lift.liftArrival=false;lift.workPlaceLiftArrival=true;
    flags=1;
    customPosition=-245760; // 150 mm，误差正好 10 mm。
    assert(hardware.feedback(lift)&&!hardware.ready(lift,done)&&!done);
    customPosition=-247398; // 约 151 mm，进入误差范围。
    assert(!hardware.ready(lift,done)&&done);
    stop=true;assert(hardware.ready(lift,done)&&!done);
}
void placement() {
    MotionHardware hardware;hardware.configure(21,cancelled);
    motion::Step s;s.name="PLACE_DESCEND";s.placementPosition=2;
    currentArm.turret_angle=350;currentArm.length=20;turretMeasured=-10;
    customPosition=llround(-22.0*LENGTH_PULSE*65536/3200);
    hardware.event(0,s,true);assert(placementReads==0);
    hardware.event(0,s,false);assert(placementReads==1);
    assert(Serial.text.find("axis=turret target=350.000 actual=-10.000 error=-360.000 unit=deg")!=std::string::npos);
    assert(Serial.text.find("axis=length target=20.000 actual=22.000 error=+2.000 unit=mm")!=std::string::npos);
    servoAvailable=false;motorError="MOTOR_FEEDBACK_TIMEOUT";hardware.event(0,s,false);
    assert(Serial.text.find("axis=turret read=ERR")!=std::string::npos);
    assert(Serial.text.find("axis=length read=ERR reason=MOTOR_FEEDBACK_TIMEOUT")!=std::string::npos);
    // 日志读取失败不能中止序列，后续提交照常执行。
    s.placementPosition=0;s.kind=motion::StepKind::COMMIT;s.cargo=1;s.color=MATERIAL_RED;
    assert(!hardware.issue(s)&&cargoPlatforms[0].material==MATERIAL_RED);
    hardware.event(1,s,false);assert(placementReads==2);
}
void ringCorrection() {
    MotionHardware hardware;
    hardware.configure(22,cancelled);
    motion::Step hover;
    hover.kind=motion::StepKind::ARM;
    hover.ringVisionPosition=2;
    assert(!hardware.issue(hover));
    assert(Serial.text=="{EVT,VISION,RING_TARGET,2}\n{EVT,VISION,START_REQUEST,RING_CORRECTION}\n");
    ring::Frame frame;
    assert(!ring::input().submit(0,frame));

    motion::Step correct;
    correct.kind=motion::StepKind::RING_CORRECT;
    turretMeasured=86;
    customPosition=llround(-71.5*LENGTH_PULSE*65536/3200);
    assert(!hardware.issue(correct));
    assert(Serial.text.find("{EVT,VISION,RING_READY,1}")!=std::string::npos);
    assert(!ring::input().submit(0,frame));
    bool done=false;
    assert(!hardware.ready(correct,done)&&!done);
    frame.x=1;
    assert(ring::input().submit(1,frame));
    assert(!hardware.ready(correct,done)&&!done&&issued==2);
    assert(issuedRamp==160&&issuedTurretRpm==10);
    frame.x=0;
    frame.y=-4;
    assert(ring::input().submit(1,frame));
    assert(!hardware.ready(correct,done)&&!done&&issued==3&&issuedTurret==-1);
    assert(fabsf(issuedLength-71.288f)<0.002f);
    frame.y=4;
    assert(ring::input().submit(1,frame));
    assert(!hardware.ready(correct,done)&&!done&&issued==4&&issuedTurret==-1);
    assert(fabsf(issuedLength-71.712f)<0.002f);
    frame.x=8;
    frame.y=0.5f;
    assert(ring::input().submit(1,frame));
    assert(!hardware.ready(correct,done)&&!done&&issued==5&&issuedLength==-1);
    assert(fabsf(issuedTurret-85.840f)<0.001f);
    frame.x=-8;
    frame.y=-0.5f;
    assert(ring::input().submit(1,frame));
    assert(!hardware.ready(correct,done)&&!done&&issued==6&&issuedLength==-1);
    assert(fabsf(issuedTurret-86.160f)<0.001f);
    frame.y=4;
    assert(ring::input().submit(1,frame));
    assert(!hardware.ready(correct,done)&&!done&&issued==7);
    assert(fabsf(issuedTurret-86.160f)<0.001f&&fabsf(issuedLength-71.712f)<0.002f);
    frame.x=0.5f;
    frame.y=-0.5f;
    assert(ring::withinDeadzone(frame,ring::configuredDeadzone()));
    frame.y=-1;
    assert(!ring::withinDeadzone(frame,ring::configuredDeadzone()));
    frame.y=-0.5f;
    assert(ring::input().submit(1,frame));
    assert(!hardware.ready(correct,done)&&!done&&issued==7);
    frame.x=6;
    assert(ring::input().submit(1,frame));
    frame.x=0.5f;
    assert(ring::input().submit(1,frame));
    assert(!hardware.ready(correct,done)&&!done&&issued==7);
    assert(ring::input().submit(1,frame));
    frame.x=6;
    assert(ring::input().submit(1,frame));
    assert(!hardware.ready(correct,done)&&done&&issued==7);
    assert(Serial.text.find("{EVT,VISION,STOP_REQUEST,RING_CORRECTION}")!=std::string::npos);
    assert(!ring::input().submit(1,frame));

    assert(!hardware.issue(hover));
    assert(!hardware.issue(correct));
    frame.x=-600;
    frame.y=700;
    assert(ring::input().submit(2,frame));
    assert(!hardware.ready(correct,done)&&!done);
    assert(issuedTurret>95.9f&&issuedTurret<=96);
    assert(issuedLength>91.4f&&issuedLength<=91.501f);
    const float upperTurret=issuedTurret,upperLength=issuedLength;

    turretMeasured=90;
    customPosition=llround(-80.0*LENGTH_PULSE*65536/3200);
    assert(ring::input().submit(2,frame));
    assert(!hardware.ready(correct,done)&&!done);
    assert(issuedTurret==upperTurret&&issuedLength==upperLength);

    frame.x=1000;
    frame.y=-1000;
    assert(ring::input().submit(2,frame));
    assert(!hardware.ready(correct,done)&&!done);
    assert(issuedTurret>=76&&issuedTurret<76.1f);
    assert(issuedLength>=51.5f&&issuedLength<51.6f);
    hardware.stopRingVision();

    turretMeasured=357;
    customPosition=llround(-4.0*LENGTH_PULSE*65536/3200);
    ring::requiredStableFrames()=3;
    assert(!hardware.issue(hover));
    assert(!hardware.issue(correct));
    frame.x=-200;
    frame.y=-500;
    assert(ring::input().submit(3,frame));
    assert(!hardware.ready(correct,done)&&!done);
    assert(issuedTurret>359.9f&&issuedTurret<=360&&issuedLength==0);
    frame.x=frame.y=0;
    assert(ring::input().submit(3,frame));
    assert(ring::input().submit(3,frame));
    assert(!hardware.ready(correct,done)&&!done);
    assert(ring::input().submit(3,frame));
    assert(!hardware.ready(correct,done)&&done);
    ring::requiredStableFrames()=2;

    auto &deadzone=ring::configuredDeadzone();
    deadzone.x=2;
    deadzone.y=0.5f;
    turretMeasured=86;
    customPosition=llround(-71.5*LENGTH_PULSE*65536/3200);
    assert(!hardware.issue(hover)&&!hardware.issue(correct));
    frame.x=1.5f;
    frame.y=0.5f;
    assert(ring::input().submit(4,frame));
    assert(!hardware.ready(correct,done)&&!done&&issuedTurret==-1&&issuedLength>71.5f);
    frame.x=2;
    frame.y=0.4f;
    assert(ring::input().submit(4,frame));
    assert(!hardware.ready(correct,done)&&!done&&issuedTurret<86&&issuedLength==-1);
    frame.x=1.5f;
    assert(ring::input().submit(4,frame));
    assert(ring::input().submit(4,frame));
    assert(!hardware.ready(correct,done)&&done);
    deadzone=ring::Deadzone{};

    ring::configuredTimeoutMs()=100;
    clockMs=UINT32_MAX-40;
    assert(!hardware.issue(hover)&&!hardware.issue(correct));
    clockMs+=99;
    assert(!hardware.ready(correct,done)&&!done);
    ++clockMs;
    assert(!hardware.ready(correct,done)&&done);
    assert(Serial.text.find("{EVT,VISION,RING_TIMEOUT,100}\n{EVT,VISION,STOP_REQUEST,RING_CORRECTION}\n")!=std::string::npos);

    ring::configuredTimeoutMs()=0;
    assert(!hardware.issue(hover)&&!hardware.issue(correct));
    clockMs+=10000;
    assert(!hardware.ready(correct,done)&&!done);
    hardware.stopRingVision();
}

void ringFineTurret() {
    MotionHardware hardware;
    motion::Step hover,correct;
    hover.kind=motion::StepKind::ARM;
    hover.ringVisionPosition=1;
    correct.kind=motion::StepKind::RING_CORRECT;
    turretMeasured=90.0f;
    customPosition=llround(-20.0*LENGTH_PULSE*65536/3200);
    ring::Settings settings;
    settings.turret={0.1f,0.02f,0};
    ring::input().settings(settings);
    assert(!hardware.issue(hover)&&!hardware.issue(correct));
    uint32_t token=1;
    auto feed=[&](uint32_t interval,float x,float y=0.0f) {
        clockMs+=interval;
        ring::Frame frame;
        frame.x=x;frame.y=y;frame.timeMs=clockMs;
        assert(ring::input().submit(token,frame));
        bool done=false;
        const char *error=hardware.ready(correct,done);
        assert(!done);
        return error;
    };
    assert(!feed(0,1)&&turretCommands==1);
    assert(!feed(119,3,4)&&turretCommands==1&&issuedLength>20); // X等待期间Y仍发令。
    assert(!feed(1,2)&&turretCommands==2); // 满120 ms才使用新帧。
    assert(fabsf(issuedTurret-(90.0f-0.2f-0.02f*(3*0.119f+2*0.001f)))<0.001f);
    assert(!feed(120,2)&&turretCommands==2); // 目标角改变，但量化脉冲相同。
    assert(!feed(1,4)&&!feed(1,4)&&turretCommands==4); // 大误差照常逐帧发令。
    assert(!feed(1,3)&&turretCommands==4); // 从大误差发出的上一条也计入间隔。
    assert(!feed(119,3)&&turretCommands==5);
    cancelledTurretCommand=turretCommands;
    assert(!feed(1,3)&&turretCommands==6); // 已取消的命令不参与间隔或去重。
    turretSendFails=true;
    assert(feed(1,4)&&turretCommands==6);
    turretSendFails=false;
    assert(!feed(1,3)&&turretCommands==7);

    hardware.stopRingVision();
    assert(!hardware.issue(hover)&&!hardware.issue(correct));
    ++token;
    assert(!feed(1,3)&&turretCommands==8); // 新令牌立即允许首条命令。
    activeTurretCommand=0; // 转台已停稳，视觉误差仍在死区外。
    assert(!feed(1,3)&&turretCommands==8); // 停稳后仍保留 120 ms 发令间隔。
    assert(!feed(119,3)&&turretCommands==9); // 间隔已满，允许重发同一脉冲目标。
    hardware.stopRingVision();

    ring::input().settings(ring::Settings{});
    turretMeasured=0;
    assert(!hardware.issue(hover)&&!hardware.issue(correct));
    ++token;
    assert(!feed(0,0.5f,2)&&turretCommands==9); // X 在死区内，Y 可以继续。
    assert(!feed(0,1)&&turretCommands==10);
    assert(issuedTurretPosition.pulses==1&&issuedTurretPosition.direction!=TURRET_MOTOR_POSITIVE_DIR);
    assert(fabsf(issuedTurret+0.1125f)<0.001f);
    assert(!feed(120,-1)&&turretCommands==11);
    assert(issuedTurretPosition.pulses==1&&issuedTurretPosition.direction==TURRET_MOTOR_POSITIVE_DIR);
    turretMeasured=9.9f;
    assert(!feed(120,-1)&&turretCommands==11); // 下一脉冲超出本轮纠偏范围。
    hardware.stopRingVision();
}
int main(int argc,char **argv) {
    if(argc==2&&!strcmp(argv[1],"ring")){ringCorrection();return 0;}
    if(argc==2&&!strcmp(argv[1],"ring-fine")){ringFineTurret();return 0;}
    assert(argc==2);if(!strcmp(argv[1],"look3"))look3();else if(!strcmp(argv[1],"feedback"))feedback();else if(!strcmp(argv[1],"turret"))turretArrival();else if(!strcmp(argv[1],"lift"))liftArrival();else if(!strcmp(argv[1],"placement"))placement();else if(!strcmp(argv[1],"tray"))trayArrival();else assert(false);
}
