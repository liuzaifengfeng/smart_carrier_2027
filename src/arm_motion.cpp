#include "arm_motion.h"
#include "material_transfer.h"
#include "motion_executor.h"
#include "turret_path.h"
#include "disc_position_grab.h"
#include "node_route_heading.h"

using namespace motion;
namespace {
/**
 * @brief 向Program追加一个机械臂位姿步骤；只记录目标、速度和等待时间，不发令。
 * @param planned true 表示转台使用已规划的精确多圈目标。
 * @param hold 反馈完成后的夹持或释放保持时间(ms)。
 */
void arm(Program &p,const char *name,float h,float l,float t,float claw,float speed,uint32_t wait=0,bool planned=false,uint32_t hold=0) {
    Step s;s.kind=StepKind::ARM;s.name=name;s.high=h;s.length=l;s.turret=t;s.pawl=claw;s.speed=speed;s.waitMs=wait;s.planned=planned;s.holdMs=hold;s.turretRampMs=20;p.append(s);
}
/** @brief 追加转台到位检查步骤，忽略旧旋转延时；其他轴仍只发令。 */
void rotateArm(Program &p,const char *name,float h,float l,float t,float claw,float speed,bool planned=false) {
    arm(p,name,h,l,t,claw,speed,0,planned);
    p.steps[p.count-1].turretArrival=true;
}
/**
 * @brief 向Program追加纯等待步骤，ms为等待毫秒数，不操作机构。
 */
void wait(Program &p,const char *name,uint32_t ms){Step s;s.name=name;s.waitMs=ms;p.append(s);}
/**
 * @brief 追加载物台物料登记步骤；执行到这里才更新指定cargo的颜色。
 */
void commit(Program &p,uint8_t cargo,uint8_t color){Step s;s.kind=StepKind::COMMIT;s.name="CARGO_COMMIT";s.cargo=cargo;s.color=color;p.append(s);}
/**
 * @brief 追加底盘相对位置动作，x/y为mm、theta为转角deg、speed为最大RPM。
 */
void position(Program &p,float x,float y,float theta,float speed){Step s;s.kind=StepKind::POSITION;s.name="POSITION";s.high=x;s.length=y;s.turret=theta;s.speed=speed;p.append(s);}
/**
 * @brief 构建到工位上方的平移、稳定和转向步骤；angle可指定规划后的多圈转台角。
 */
void above(Program &p,const MaterialStationPose &pose,float claw,const MaterialApproachDelays &d,const float *angle) {
    arm(p,"APPROACH_TRANSLATE",materialTransferLayout.approachHeight,pose.length,-1,claw,materialMotionSteps.approachTranslateSpeed,d.translateMs);
    wait(p,"APPROACH_SETTLE",d.settleMs);
    rotateArm(p,"APPROACH_ROTATE",materialTransferLayout.approachHeight,pose.length,angle?*angle:pose.turretAngle,-1,materialMotionSteps.approachRotateSpeed,angle);
}
/**
 * @brief 构建到位、下降、夹紧、抬升及可选缩回的取料序列；retract和together选择缩回衔接方式。
 */
void pick(Program &p,const MaterialStationPose &pose,const MaterialTransferDelays &d,bool retract,bool together,const float *angle) {
    above(p,pose,materialTransferLayout.clawOpenAngle,d.pickAbove,angle);
    arm(p,"PICK_DESCEND",pose.high,-1,-1,-1,materialMotionSteps.pickDescendSpeed,d.pickDescendMs);
    arm(p,"PICK_GRIP",-1,-1,-1,materialTransferLayout.clawClosedAngle,materialMotionSteps.pickGripSpeed,d.pickGripMs,false,ARM_GRIP_HOLD_MS);
    arm(p,"PICK_LIFT",materialTransferLayout.approachHeight,together?0:-1,-1,-1,materialMotionSteps.pickLiftSpeed,d.pickLiftMs);
    if(retract&&!together)arm(p,"PICK_RETRACT",-1,0,-1,-1,materialMotionSteps.pickLiftSpeed);
}
/**
 * @brief 构建放料转向、下降、松爪及抬升序列；height为实际层高，retracted表示小臂已缩回。
 */
void place(Program &p,const MaterialStationPose &pose,float height,const MaterialTransferDelays &d,bool retracted,const float *angle,bool slowLower=false,uint8_t workPosition=0) {
    if(retracted) {
        wait(p,"PLACE_TRANSLATE_WAIT",d.placeAbove.translateMs);wait(p,"PLACE_SETTLE",d.placeAbove.settleMs);
        rotateArm(p,"PLACE_ROTATE",-1,-1,angle?*angle:pose.turretAngle,-1,materialMotionSteps.approachRotateSpeed,angle);
    } else above(p,pose,-1,d.placeAbove,angle); // 已夹紧物料，移位和下降不再重复操作夹爪。
    if(slowLower) {
        arm(p,"PLACE_DESCEND_10",10,retracted?pose.length:-1,-1,-1,materialMotionSteps.placeDescendSpeed,d.placeDescendMs);
        p.steps[p.count-1].slowLower=true;
        p.steps[p.count-1].timed=true; // 原下降延时结束后下发最后10mm，不查到位反馈。
        arm(p,"PLACE_DESCEND",0,-1,-1,-1,materialMotionSteps.placeDescendSpeed,100);
        p.steps[p.count-1].slowLower=true;
        p.steps[p.count-1].timed=true;
    } else arm(p,"PLACE_DESCEND",height,retracted?pose.length:-1,-1,-1,materialMotionSteps.placeDescendSpeed,d.placeDescendMs);
    p.steps[p.count-1].placementPosition=workPosition;
    if(slowLower) { // 仅工位第一层：先卸去夹紧力，300ms后再完全张开。
        arm(p,"PLACE_RELEASE_ZERO",-1,-1,-1,0,materialMotionSteps.placeReleaseSpeed,300);
        p.steps[p.count-1].timed=true;
    }
    arm(p,"PLACE_RELEASE",-1,-1,-1,materialTransferLayout.clawOpenAngle,materialMotionSteps.placeReleaseSpeed,d.placeReleaseMs,false,ARM_RELEASE_HOLD_MS);
    arm(p,"PLACE_LIFT",materialTransferLayout.approachHeight,-1,-1,-1,materialMotionSteps.placeLiftSpeed,d.placeLiftMs);
}
/**
 * @brief 回读转台实际角度并规划整组多圈转向目标，减少累计转角；成功返回true。
 */
bool plan(const float *targets,size_t count,TurretPathPlan &result) {
    float actual=0;if(!ReadArmTurretAngle(actual))return false;
    return PlanTurretPath(actual,targets,count,TURRET_CABLE_MIN_DEG,TURRET_CABLE_MAX_DEG,result);
}
/**
 * @brief 构建单件或整组三件的正向放料/反向取回序列，选择对应动作组延时并追加物料登记；成功返回nullptr。
 */
const char *transfer(const MotionRequest &r,Program &p) {
    const bool retrieve=r.kind==MotionKind::RETRIEVE_ROUND||r.kind==MotionKind::RETRIEVE||r.kind==MotionKind::DEMO_RETRIEVE;
    const bool single=r.kind==MotionKind::TRANSFER||r.kind==MotionKind::RETRIEVE;
    const bool demo=r.kind==MotionKind::DEMO_PLACE||r.kind==MotionKind::DEMO_RETRIEVE;
    const uint8_t count=single?1:3;
    float angles[6];uint8_t cargos[3],works[3];
    for(uint8_t i=0;i<count;++i) {
        cargos[i]=single?r.cargo:r.kind==MotionKind::DEMO_RETRIEVE?3-i:i+1;
        works[i]=single?r.work:demo?i+1:r.positions[i];
        const auto &cargo=materialTransferLayout.cargo[cargos[i]-1],&work=materialTransferLayout.workArea[works[i]-1];
        if(retrieve&&cargoPlatforms[cargos[i]-1].material!=MATERIAL_NONE)return "OCCUPIED";
        if(!retrieve&&!demo&&!r.debug&&cargoPlatforms[cargos[i]-1].material==MATERIAL_NONE)return "EXECUTION";
        angles[2*i]=retrieve?work.turretAngle:cargo.turretAngle;
        angles[2*i+1]=retrieve?cargo.turretAngle:work.turretAngle;
    }
    TurretPathPlan selected;if(!plan(angles,2*count,selected))return "EXECUTION";
    for(uint8_t i=0;i<count;++i) {
        const auto &cargo=materialTransferLayout.cargo[cargos[i]-1],&work=materialTransferLayout.workArea[works[i]-1];
        const auto &d=retrieve?workAreaToCargoDelays[works[i]-1][cargos[i]-1]:cargoToWorkAreaDelays[cargos[i]-1][works[i]-1];
        const auto &source=retrieve?work:cargo,&destination=retrieve?cargo:work;
        const float height=destination.high+(retrieve?0:(r.layer-1)*materialTransferLayout.secondLayerOffset);
        pick(p,source,d,!retrieve,retrieve,&selected.anglesDeg[2*i]);
        place(p,destination,height,d,true,&selected.anglesDeg[2*i+1],!retrieve&&r.layer==1,retrieve?0:works[i]);
        commit(p,cargos[i],retrieve?(demo?MATERIAL_RED:single?r.color:r.colors[i]):MATERIAL_NONE);
    }
    return nullptr;
}
/**
 * @brief 构建START或LOOK系列姿态；视觉观察抬升按记录目标高度差计算等待，不把目标缓存当实测。
 */
void observation(Program &p,ArmObservation pose,uint8_t zone) {
    // 仅观察姿态按记录的指令高度估算：170 mm 对应 1500 ms。
    const uint32_t liftWait=static_cast<uint32_t>(lroundf(fabsf(170.0f-currentArm.high)*1500.0f/170.0f));
    switch(pose) {
    case ArmObservation::START:
        arm(p,"INIT_LIFT",160,-1,-1,0,300,0);
        p.steps[p.count-1].liftArrival=true;
        rotateArm(p,"INIT_TURN",-1,-1,-55,-1,300);arm(p,"INIT_RETRACT",154,40,-1,0,300,500);break;
    case ArmObservation::LOOK:
        arm(p,"LOOK_LIFT",170,0,-1,60,200,liftWait);rotateArm(p,"LOOK_TURN",-1,0,90,80,200);arm(p,"LOOK_READY",170,-1,-1,80,200);break;
    case ArmObservation::LOOK2:
        arm(p,"LOOK2_LIFT",170,0,-1,-1,200,liftWait);rotateArm(p,"LOOK2_TURN",-1,0,90,80,200);break;
    case ArmObservation::LOOK3: {
        arm(p,"LOOK3_LIFT",170,40,-1,-1,300,liftWait);rotateArm(p,"LOOK3_TURN",-1,-1,90,80,300);
        Step s;s.kind=StepKind::TURRET_READY;s.name="LOOK3_MEASURED_READY";s.turret=90;s.waitMs=1500;p.append(s);break;
    }
    case ArmObservation::LOOK4:
        arm(p,"LOOK4_LIFT",170,120,-1,-1,200,liftWait);rotateArm(p,"LOOK4_TURN",-1,-1,90,80,200);
        rotateArm(p,"LOOK4_READY",-1,-1,zone==1?45:135,90,200);break;
    default: break;
    }
}
}
/**
 * @brief 将动作请求展开为机械臂步骤序列；不下发运动命令、不等待动作结束，转台路径规划会回读实际角度。
 * @param r 抓放料、观察姿态或原始控制请求。
 * @param p 输出步骤序列，后续由 motion::Sequence 和 MotionHardware 执行。
 * @return nullptr 构建成功，否则返回错误原因字符串。
 */
const char *BuildArmProgram(const MotionRequest &r,Program &p) {
    p.count=0;
    if((r.kind==MotionKind::LIDAR_POSE||r.kind==MotionKind::HOME_OBSERVE||
        (r.kind==MotionKind::ARM_POSE&&r.observation==ArmObservation::LOOK4))&&r.zone!=1&&r.zone!=2)return "EXECUTION";
    switch(r.kind) {
    case MotionKind::ARM_POSE: observation(p,r.observation,r.zone);break;
    case MotionKind::TRANSFER: case MotionKind::RETRIEVE: case MotionKind::PLACE_ROUND:
    case MotionKind::RETRIEVE_ROUND: case MotionKind::DEMO_PLACE: case MotionKind::DEMO_RETRIEVE: {
        const char *error=transfer(r,p);if(error)return error;break;
    }
    case MotionKind::HELD_DISC:
    case MotionKind::DISC_GRAB:
    case MotionKind::LOAD_ROUND: {
        const uint8_t count=r.kind==MotionKind::LOAD_ROUND?3:1;
        for(uint8_t i=0;i<count;++i) {
            const uint8_t cargo=r.kind==MotionKind::LOAD_ROUND?i+1:r.cargo;
            if(cargoPlatforms[cargo-1].material!=MATERIAL_NONE&&!r.force)return "OCCUPIED";
        }
        for(uint8_t i=0;i<count;++i) {
            const uint8_t cargo=r.kind==MotionKind::LOAD_ROUND?i+1:r.cargo,color=r.kind==MotionKind::LOAD_ROUND?r.colors[i]:r.color;
            const auto &destination=materialTransferLayout.cargo[cargo-1];
            if(r.kind!=MotionKind::HELD_DISC) {
                if(r.position>=1&&r.position<=3) {
                    const float length=r.position==2?0:150,angle=r.position==1?71:r.position==2?90:102;
                    arm(p,"DISC_APPROACH",80,length,angle,80,armSpeed,r.position==3?400:500);
                    arm(p,"DISC_GRIP",80,length,angle,-8,armSpeed,commonMaterialDelays.pickGripMs,false,ARM_GRIP_HOLD_MS);
                    arm(p,"DISC_LIFT",materialTransferLayout.approachHeight,0,-1,-1,armSpeed,commonMaterialDelays.pickLiftMs);
                } else if(r.position==4||r.kind==MotionKind::LOAD_ROUND)pick(p,materialTransferLayout.disc,commonMaterialDelays,false,true,nullptr);
                else {
                    const auto &d=materialMotionSteps.discApproach;
                    arm(p,"DISC_APPROACH",d.high,d.length,d.turretAngle,materialMotionSteps.discOpenAngle,materialMotionSteps.discApproachSpeed,materialAuxiliaryDelays.discApproachMs);
                    arm(p,"DISC_GRIP",d.high,d.length,d.turretAngle,materialTransferLayout.clawClosedAngle,materialMotionSteps.discGripSpeed,commonMaterialDelays.pickGripMs,false,ARM_GRIP_HOLD_MS);
                    arm(p,"DISC_LIFT",materialTransferLayout.approachHeight,0,-1,-1,materialMotionSteps.discLiftSpeed,commonMaterialDelays.pickLiftMs);
                }
            } else {
                arm(p,"DISC_LIFT",materialTransferLayout.approachHeight,0,-1,-1,materialMotionSteps.discLiftSpeed,commonMaterialDelays.pickLiftMs);
            }
            // 圆盘抓后与工作区取回一样：抬升缩回，再转向，同时伸出下降；共用公共延时表。
            place(p,destination,destination.high,commonMaterialDelays,true,nullptr);commit(p,cargo,color);
        }
        break;
    }
    case MotionKind::LIDAR_POSE:
        position(p,125,r.zone==1?100:-100,0,200);
        if(!STARTUP_LIDAR_SCAN_ENABLED)break; // 临时跳过雷达机械臂姿态，保留底盘斜移及其位姿更新。
        if(r.zone==1) {
            arm(p,"LIDAR_LIFT",170,100,-1,-1,200);rotateArm(p,"LIDAR_TURN",-1,-1,45,-1,200);
            arm(p,"LIDAR_LOWER",0,100,-1,-1,200,materialAuxiliaryDelays.lidarZone1LiftMs);
        } else {
            arm(p,"LIDAR_LIFT",170,100,135,-1,200,materialAuxiliaryDelays.lidarZone2LiftMs);
            arm(p,"LIDAR_LOWER",0,100,135,-1,200,materialAuxiliaryDelays.lidarZone2LiftMs);
        }
        break;
    case MotionKind::HOME_OBSERVE:
        position(p,0,0,NodeRouteTurn(currentPose.theta,180),80);
        if(!r.armPrepared)observation(p,ArmObservation::LOOK4,r.zone);
        break;
    case MotionKind::HOME_RETURN: {
        observation(p,ArmObservation::START,r.zone);
        position(p,r.zone==1?-90:-70,r.zone==1?-190:200,0,200);break;
    }
    case MotionKind::POSITION: position(p,r.x,r.y,r.theta,r.speed);break;
    case MotionKind::GOTO: {Step s;s.kind=StepKind::GOTO;s.name="GOTO";s.high=r.x;s.length=r.y;s.turret=r.theta;p.append(s);break;}
    case MotionKind::ARM_RAW: arm(p,"ARM_RAW",r.high,r.length,r.turret,r.pawl,r.speed,0,r.planned);break;
    case MotionKind::SERVO_RAW: {Step s;s.kind=StepKind::RAW_SERVO;s.name="SERVO";s.cargo=r.cargo;s.turret=r.turret;p.append(s);break;}
    case MotionKind::ENABLE: {Step s;s.kind=StepKind::ENABLE;s.name="ENABLE";s.cargo=r.force;p.append(s);break;}
    case MotionKind::VELOCITY: {Step s;s.kind=StepKind::VELOCITY;s.name="VELOCITY";s.cargo=r.position;s.speed=r.speed;s.color=r.force;p.append(s);break;}
    default: break;
    }
    // 整段搬运使用同一曲线：第一层放料慢，收料和第二层叠放快；圆盘全程快。
    uint16_t turretRampMs=20;
    switch(r.kind) {
    case MotionKind::DISC_GRAB: case MotionKind::LOAD_ROUND: case MotionKind::HELD_DISC:
        turretRampMs=20;break;
    case MotionKind::RETRIEVE: case MotionKind::RETRIEVE_ROUND: case MotionKind::DEMO_RETRIEVE:
        turretRampMs=20;break;
    case MotionKind::TRANSFER: case MotionKind::PLACE_ROUND:
        turretRampMs=r.layer==1?500:20;break;
    case MotionKind::DEMO_PLACE:
        turretRampMs=500;break;
    default: break;
    }
    for(size_t i=0;i<p.count;++i)if(p.steps[i].kind==StepKind::ARM)p.steps[i].turretRampMs=turretRampMs;
    return nullptr;
}
