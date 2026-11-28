// 把机械臂请求编译成动作序列。
// 这里组织伸缩、升降、转盘和夹爪的先后关系，具体下发与到位判断在 MotionHardware 中执行。

#include "arm_motion.h"  // 引入 arm_motion.h 中的声明和定义。
#include "material_transfer.h"  // 引入 material_transfer.h 中的声明和定义。
#include "motion_executor.h"  // 引入 motion_executor.h 中的声明和定义。
#include "turret_path.h"  // 引入 turret_path.h 中的声明和定义。
#include "disc_position_grab.h"  // 引入 disc_position_grab.h 中的声明和定义。
#include "disc_material_decision.h"  // 引入 disc_material_decision.h 中的声明和定义。
#include "node_route_heading.h"  // 引入 node_route_heading.h 中的声明和定义。
#include "ring_correction.h"  // 引入 ring_correction.h 中的声明和定义。

using namespace motion;  // 引入名称或类型别名 namespace motion。
namespace {  // 开始匿名命名空间，本文件内部共享其中的定义。
constexpr uint8_t WORK_TRANSFER_EXTENSION_ACCELERATION=250;  // 将 250 写入或初始化 WORK_TRANSFER_EXTENSION_ACCELERATION（工位TRANSFER伸缩加速度）。
/**
 * @brief 向Program追加一个机械臂位姿步骤；只记录目标、速度和等待时间，不发令。
 * @param planned true 表示转台使用已规划的精确多圈目标。
 * @param hold 反馈完成后的夹持或释放保持时间(ms)。
 */
void arm(Program &p,const char *name,float h,float l,float t,float claw,float speed,uint32_t wait=0,bool planned=false,uint32_t hold=0) {  // 定义函数或成员 的入口 arm（机械臂），参数由此处的签名确定。
    Step s;s.kind=StepKind::ARM;s.name=name;s.high=h;s.length=l;s.turret=t;s.pawl=LimitArmClawAngle(claw);s.speed=speed;s.waitMs=wait;s.planned=planned;s.holdMs=hold;s.turretAcceleration=TURRET_ACCELERATION_FAST;p.append(s);  // 声明 s，类型和可用范围由所在声明决定。然后将 StepKind::ARM 写入或初始化 s.kind。然后将 name 写入或初始化 s.name。然后将 h 写入或初始化 s.high。然后将 l 写入或初始化 s.length。然后将 t 写入或初始化 s.turret。然后将 LimitArmClawAngle(claw) 写入或初始化 s.pawl。然后将 speed 写入或初始化 s.speed。然后将 wait 写入或初始化 s.waitMs。然后将 planned 写入或初始化 s.planned。然后将 hold 写入或初始化 s.holdMs。然后将 TURRET_ACCELERATION_FAST 写入或初始化 s.turretAcceleration。然后调用 p.append，传入 s。
}  // 结束此初始化列表或局部作用域。
/** @brief 追加转台到位检查步骤，忽略旧旋转延时；其他轴仍只发令。 */
void rotateArm(Program &p,const char *name,float h,float l,float t,float claw,float speed,bool planned=false) {  // 定义函数或成员 的入口 rotateArm（rotate机械臂），参数由此处的签名确定。
    arm(p,name,h,l,t,claw,speed,0,planned);  // 调用 arm（机械臂），传入 p,name,h,l,t,claw,speed,0,planned。
    p.steps[p.count-1].turretArrival=true;  // 将 true 写入或初始化 turretArrival（转台Arrival）。
}  // 结束此初始化列表或局部作用域。
/**
 * @brief 向Program追加纯等待步骤，ms为等待毫秒数，不操作机构。
 */
void wait(Program &p,const char *name,uint32_t ms){Step s;s.name=name;s.waitMs=ms;p.append(s);}  // 将 name;s.waitMs=ms;p.append(s);} 写入或初始化 s.name。
/**
 * @brief 追加载物台物料登记步骤；执行到这里才更新指定cargo的颜色。
 */
void commit(Program &p,uint8_t cargo,uint8_t color){Step s;s.kind=StepKind::COMMIT;s.name="CARGO_COMMIT";s.cargo=cargo;s.color=color;p.append(s);}  // 将 StepKind::COMMIT;s.name="CARGO_COMMIT";s.cargo=cargo;s… 写入或初始化 s.kind。
void rotateTray(Program &p,float angle,uint32_t interval=0,uint16_t power=0) {  // 定义函数或成员 的入口 rotateTray（rotate料盘），参数由此处的签名确定。
    Step s;s.kind=StepKind::TRAY_ROTATE;s.name="TRAY_ROTATE";  // 声明 s，类型和可用范围由所在声明决定。然后将 StepKind::TRAY_ROTATE 写入或初始化 s.kind。然后将 "TRAY_ROTATE" 写入或初始化 s.name。
    s.turret=angle;s.waitMs=interval;s.trayPowerMw=power;p.append(s);  // 将 angle 写入或初始化 s.turret。然后将 interval 写入或初始化 s.waitMs。然后将 power 写入或初始化 s.trayPowerMw。然后调用 p.append，传入 s。
}  // 结束此初始化列表或局部作用域。
// 货位从 1 开始，角度数组从 0 开始。
// TRAY_START 用于开始旋转，TRAY_READY 用于带容差的到位步骤。
void prepareTray(Program &p,uint8_t cargo,bool checkArrival,float tolerance=CARGO_TRAY_ARRIVAL_TOLERANCE_DEG) {  // 定义函数或成员 的入口 prepareTray（prepare料盘），参数由此处的签名确定。
    Step s;  // 声明 s，类型和可用范围由所在声明决定。
    s.kind=checkArrival?StepKind::TRAY_READY:StepKind::TRAY_START;  // 将 checkArrival?StepKind::TRAY_READY:StepKind::TRAY_START 写入或初始化 s.kind。
    s.name=checkArrival?"TRAY_READY":"TRAY_START";  // 将 checkArrival?"TRAY_READY":"TRAY_START" 写入或初始化 s.name。
    s.turret=cargoTrayAngles[cargo-1];  // 将 cargoTrayAngles[cargo-1] 写入或初始化 s.turret。
    s.trayArrivalToleranceDeg=tolerance;  // 将 tolerance 写入或初始化 s.trayArrivalToleranceDeg。
    p.append(s);  // 调用 p.append，传入 s。
}  // 结束此初始化列表或局部作用域。
/**
 * @brief 追加底盘相对位置动作，x/y为mm、theta为转角deg、speed为最大RPM。
 */
void position(Program &p,float x,float y,float theta,float speed,uint8_t acceleration=POSITION_MOVE_DEFAULT_ACCELERATION) {  // 定义函数或成员 的入口 position（位置），参数由此处的签名确定。
    Step s;s.kind=StepKind::POSITION;s.name="POSITION";s.high=x;s.length=y;s.turret=theta;s.speed=speed;  // 声明 s，类型和可用范围由所在声明决定。然后将 StepKind::POSITION 写入或初始化 s.kind。然后将 "POSITION" 写入或初始化 s.name。然后将 x 写入或初始化 s.high。然后将 y 写入或初始化 s.length。然后将 theta 写入或初始化 s.turret。然后将 speed 写入或初始化 s.speed。
    s.positionAcceleration=acceleration;p.append(s);  // 将 acceleration 写入或初始化 s.positionAcceleration。然后调用 p.append，传入 s。
}  // 结束此初始化列表或局部作用域。
/**
 * @brief 构建到工位上方的平移、稳定和转向步骤；angle可指定规划后的多圈转台角。
 */
void above(Program &p,const MaterialStationPose &pose,float claw,const MaterialApproachDelays &d,const float *angle) {  // 定义函数或成员 的入口 above，参数由此处的签名确定。
    arm(p,"APPROACH_TRANSLATE",materialTransferLayout.approachHeight,pose.length,-1,claw,materialMotionSteps.approachTranslateSpeed,d.translateMs);  // 调用 arm（机械臂），传入 p,"APPROACH_TRANSLATE",materialTransferLayout.approach…。
    wait(p,"APPROACH_SETTLE",d.settleMs);  // 调用 wait，传入 p,"APPROACH_SETTLE",d.settleMs。
    rotateArm(p,"APPROACH_ROTATE",materialTransferLayout.approachHeight,pose.length,angle?*angle:pose.turretAngle,-1,materialMotionSteps.approachRotateSpeed,angle);  // 调用 rotateArm（rotate机械臂），传入 p,"APPROACH_ROTATE",materialTransferLayout.approachHei…。
}  // 结束此函数或控制块。
/**
 * @brief 构建到位、下降、夹紧、抬升及可选缩回的取料序列；retract和together选择缩回衔接方式。
 */
void pick(Program &p,const MaterialStationPose &pose,const MaterialTransferDelays &d,bool retract,bool together,const float *angle,  // 定义函数或成员 pick，参数由此处的签名确定。
          uint8_t cargo=0,uint8_t extensionAcceleration=ARM_EXTENSION_ACCELERATION) {  // 将 0,uint8_t extensionAcceleration=ARM_EXTENSION_ACCELERA… 写入或初始化 cargo（货位）。
    // 来源不同会影响张爪角。
    // 先保存接近步骤的下标，above 展开多个步骤后再通过该下标设置第一步的伸缩加速度。
    const float openAngle=cargo?materialTransferLayout.cargoClawOpenAngle:materialTransferLayout.clawOpenAngle;  // 将 cargo?materialTransferLayout.cargoClawOpenAngle:materi… 写入或初始化 openAngle（open角度）。
    const size_t approach=p.count;  // 将 p.count 写入或初始化 approach。
    above(p,pose,openAngle,d.pickAbove,angle);  // 调用 above，传入 p,pose,openAngle,d.pickAbove,angle。
    p.steps[approach].lengthAcceleration=extensionAcceleration;  // 将 extensionAcceleration 写入或初始化 lengthAcceleration（长度加速度）。
    if(cargo)prepareTray(p,cargo,true);  // 当 cargo 成立时进入此分支，否则继续后续流程。
    arm(p,"PICK_DESCEND",pose.high,-1,-1,-1,materialMotionSteps.pickDescendSpeed,d.pickDescendMs);  // 调用 arm（机械臂），传入 p,"PICK_DESCEND",pose.high,-1,-1,-1,materialMotionStep…。
    arm(p,"PICK_GRIP",-1,-1,-1,materialTransferLayout.clawClosedAngle,materialMotionSteps.pickGripSpeed,d.pickGripMs,false,ARM_GRIP_HOLD_MS);  // 调用 arm（机械臂），传入 p,"PICK_GRIP",-1,-1,-1,materialTransferLayout.clawClos…。
    arm(p,"PICK_LIFT",materialTransferLayout.approachHeight,together?0:-1,-1,-1,materialMotionSteps.pickLiftSpeed,d.pickLiftMs);  // 调用 arm（机械臂），传入 p,"PICK_LIFT",materialTransferLayout.approachHeight,to…。
    if(retract&&!together) {  // 当 retract&&!together 成立时进入此分支，否则继续后续流程。
        arm(p,"PICK_RETRACT",-1,0,-1,-1,materialMotionSteps.pickLiftSpeed,CARGO_RETRACT_BEFORE_TURN_MS);  // 调用 arm（机械臂），传入 p,"PICK_RETRACT",-1,0,-1,-1,materialMotionSteps.pickLi…。
        p.steps[p.count-1].timed=true;  // 将 true 写入或初始化 timed。
    }  // 结束此条件分支。
    if(together||retract)p.steps[p.count-1].lengthAcceleration=extensionAcceleration;  // 当 together||retract 成立时进入此分支，否则继续后续流程。
}  // 结束此初始化列表或局部作用域。
/**
 * @brief 构建放料转向、下降、松爪及抬升序列；height为实际层高，retracted表示小臂已缩回。
 */
void place(Program &p,const MaterialStationPose &pose,float height,const MaterialTransferDelays &d,bool retracted,const float *angle,bool slowLower=false,uint8_t workPosition=0,uint8_t sourceCargo=0,uint8_t destinationCargo=0,bool ringCorrection=false,uint8_t descendExtensionAcceleration=ARM_EXTENSION_ACCELERATION,float trayTolerance=CARGO_TRAY_ARRIVAL_TOLERANCE_DEG) {  // 定义函数或成员 的入口 place，参数由此处的签名确定。
    if(retracted) {  // 当 retracted 成立时进入此分支，否则继续后续流程。
        if(!sourceCargo) {  // 当 !sourceCargo 成立时进入此分支，否则继续后续流程。
            wait(p,"PLACE_TRANSLATE_WAIT",d.placeAbove.translateMs);  // 调用 wait，传入 p,"PLACE_TRANSLATE_WAIT",d.placeAbove.translateMs。
            wait(p,"PLACE_SETTLE",d.placeAbove.settleMs);  // 调用 wait，传入 p,"PLACE_SETTLE",d.placeAbove.settleMs。
        }  // 结束此条件分支。
        rotateArm(p,"PLACE_ROTATE",-1,destinationCargo?pose.length:-1,angle?*angle:pose.turretAngle,-1,materialMotionSteps.approachRotateSpeed,angle);  // 调用 rotateArm（rotate机械臂），传入 p,"PLACE_ROTATE",-1,destinationCargo?pose.length:-1,an…。
        p.steps[p.count-1].cargoPlaceTurn=destinationCargo!=0;  // 将 destinationCargo!=0 写入或初始化 cargoPlaceTurn（货位PlaceTurn）。
        p.steps[p.count-1].lengthArrival=destinationCargo!=0;  // 将 destinationCargo!=0 写入或初始化 lengthArrival（长度Arrival）。
        p.steps[p.count-1].trayClearanceCargo=sourceCargo;  // 将 sourceCargo 写入或初始化 trayClearanceCargo（料盘Clearance货位）。
    } else above(p,pose,-1,d.placeAbove,angle); // 已夹紧物料，移位和下降不再重复操作夹爪。
    if(destinationCargo)prepareTray(p,destinationCargo,true,trayTolerance);  // 当 destinationCargo 成立时进入此分支，否则继续后续流程。
    // 保存下一步下标，以便两种下降分支构建完后都能给首个下降步骤设置工位 3 的料盘限制。
    const size_t firstLower=p.count;  // 将 p.count 写入或初始化 firstLower（首个Lower）。
    if(slowLower) {  // 当 slowLower 成立时进入此分支，否则继续后续流程。
        // 第一层下降可拆成悬停、视觉修正和下探。
        // 悬停到位与启动相机分别受动作标志和视觉超时配置影响。
        const bool enableVision=ringCorrection&&ring::configuredTimeoutMs()!=0;  // 将 ringCorrection&&ring::configuredTimeoutMs()!=0 写入或初始化 enableVision（enable视觉）。
        arm(p,"PLACE_DESCEND_HOVER",ring::HOVER_MM,retracted?pose.length:-1,-1,-1,materialMotionSteps.placeDescendSpeed,d.placeDescendMs);  // 调用 arm（机械臂），传入 p,"PLACE_DESCEND_HOVER",ring::HOVER_MM,retracted?pose.…。
        Step &hover=p.steps[p.count-1];  // 将 p.steps[p.count-1] 写入或初始化 hover。
        hover.slowLower=true;  // 将 true 写入或初始化 hover.slowLower。
        hover.timed=!ringCorrection;  // 将 !ringCorrection 写入或初始化 hover.timed。
        if(ringCorrection) {  // 当 ringCorrection 成立时进入此分支，否则继续后续流程。
            hover.liftArrival=true;  // 将 true 写入或初始化 hover.liftArrival。
            hover.lengthArrival=retracted;  // 将 retracted 写入或初始化 hover.lengthArrival。
        }  // 结束此条件分支。
        if(enableVision) {  // 当 enableVision 成立时进入此分支，否则继续后续流程。
            hover.ringVisionPosition=workPosition;  // 将 workPosition 写入或初始化 hover.ringVisionPosition。
            Step correction;  // 声明 correction（修正量），类型和可用范围由所在声明决定。
            correction.kind=StepKind::RING_CORRECT;  // 将 StepKind::RING_CORRECT 写入或初始化 correction.kind。
            correction.name="RING_CORRECT";  // 将 "RING_CORRECT" 写入或初始化 correction.name。
            correction.speed=materialMotionSteps.placeDescendSpeed;  // 将 materialMotionSteps.placeDescendSpeed 写入或初始化 correction.speed。
            p.append(correction);  // 调用 p.append，传入 correction。
        }  // 结束此条件分支。
        arm(p,"PLACE_DESCEND",0,-1,-1,-1,materialMotionSteps.placeDescendSpeed,100);  // 调用 arm（机械臂），传入 p,"PLACE_DESCEND",0,-1,-1,-1,materialMotionSteps.place…。
        p.steps[p.count-1].slowLower=true;  // 将 true 写入或初始化 slowLower。
        p.steps[p.count-1].slowLowerAcceleration=200;  // 将 200 写入或初始化 slowLowerAcceleration（slowLower加速度）。
        p.steps[p.count-1].timed=true;  // 将 true 写入或初始化 timed。
    } else {  // 前一个判断不成立时执行此分支。
        // 料盘目标在放料层高的基础上加释放偏移。
        // 已缩回且目标是工位时，下降步骤同时带上目标伸长量。
        const float releaseHeight=height+(destinationCargo?CARGO_TRAY_RELEASE_OFFSET_MM:0.0f);  // 将 height+(destinationCargo?CARGO_TRAY_RELEASE_OFFSET_MM:… 写入或初始化 releaseHeight（release高度）。
        arm(p,"PLACE_DESCEND",releaseHeight,retracted&&!destinationCargo?pose.length:-1,-1,-1,materialMotionSteps.placeDescendSpeed,d.placeDescendMs);  // 调用 arm（机械臂），传入 p,"PLACE_DESCEND",releaseHeight,retracted&&!destinatio…。
        p.steps[p.count-1].lengthAcceleration=descendExtensionAcceleration;  // 将 descendExtensionAcceleration 写入或初始化 lengthAcceleration（长度加速度）。
    }  // 结束此条件分支。
    if(workPosition==3)p.steps[firstLower].trayWork3LimitCargo=sourceCargo;  // 当 workPosition==3 成立时进入此分支，否则继续后续流程。
    p.steps[p.count-1].placementPosition=workPosition;  // 将 workPosition 写入或初始化 placementPosition（placement位置）。
    if(slowLower) { // 仅工位第一层：先卸去夹紧力，300ms后再完全张开。
        arm(p,"PLACE_RELEASE_ZERO",-1,-1,-1,0,materialMotionSteps.placeReleaseSpeed,300);  // 调用 arm（机械臂），传入 p,"PLACE_RELEASE_ZERO",-1,-1,-1,0,materialMotionSteps.…。
        p.steps[p.count-1].timed=true;  // 将 true 写入或初始化 timed。
    }  // 结束此条件分支。
    const float openAngle=&pose==&materialTransferLayout.cargo  // 将 &pose==&materialTransferLayout.cargo 写入或初始化 openAngle（open角度）。
        ?materialTransferLayout.cargoClawOpenAngle:materialTransferLayout.clawOpenAngle;  // 续写当前声明或运算，提供 ?materialTransferLayout.cargoClawOpenAngle:materialTra…。
    arm(p,"PLACE_RELEASE",-1,-1,-1,openAngle,materialMotionSteps.placeReleaseSpeed,d.placeReleaseMs,false,ARM_RELEASE_HOLD_MS);  // 调用 arm（机械臂），传入 p,"PLACE_RELEASE",-1,-1,-1,openAngle,materialMotionSte…。
    arm(p,"PLACE_LIFT",materialTransferLayout.approachHeight,-1,-1,-1,materialMotionSteps.placeLiftSpeed,d.placeLiftMs);  // 调用 arm（机械臂），传入 p,"PLACE_LIFT",materialTransferLayout.approachHeight,-…。
}  // 结束此初始化列表或局部作用域。
/**
 * @brief 回读转台实际角度并规划整组多圈转向目标，减少累计转角；成功返回true。
 */
bool plan(const float *targets,size_t count,TurretPathPlan &result) {  // 定义函数或成员 的入口 plan，参数由此处的签名确定。
    float actual=0;if(!ReadArmTurretAngle(actual))return false;  // 将 0 写入或初始化 actual（实测）。然后若 !ReadArmTurretAngle(actual)，立即返回 false。
    return PlanTurretPath(actual,targets,count,TURRET_CABLE_MIN_DEG,TURRET_CABLE_MAX_DEG,result);  // 结束当前函数并返回 PlanTurretPath(actual,targets,count,TURRET_CABLE_MIN_D…。
}  // 结束此函数或控制块。
/**
 * @brief 构建单件或整组三件的正向放料/反向取回序列，选择对应动作组延时并追加物料登记；成功返回nullptr。
 */
const char *transfer(const MotionRequest &r,Program &p) {  // 定义函数或成员 的入口 transfer，参数由此处的签名确定。
    const bool retrieve=r.kind==MotionKind::RETRIEVE_ROUND||r.kind==MotionKind::RETRIEVE||r.kind==MotionKind::DEMO_RETRIEVE;  // 将 r.kind==MotionKind::RETRIEVE_ROUND||r.kind==MotionKind… 写入或初始化 retrieve。
    const bool single=r.kind==MotionKind::TRANSFER||r.kind==MotionKind::RETRIEVE;  // 将 r.kind==MotionKind::TRANSFER||r.kind==MotionKind::RETR… 写入或初始化 single。
    const bool demo=r.kind==MotionKind::DEMO_PLACE||r.kind==MotionKind::DEMO_RETRIEVE;  // 将 r.kind==MotionKind::DEMO_PLACE||r.kind==MotionKind::DE… 写入或初始化 demo。
    const uint8_t count=single?1:3;  // 将 single?1:3 写入或初始化 count（计数）。
    float angles[6];uint8_t cargos[3],works[3];  // 声明 angles，类型和可用范围由所在声明决定。然后声明 cargos，类型和可用范围由所在声明决定。
    // 正式搬运按抓取次序遍历载物台 3、2、1，位置码只指定目的工位。
    for(uint8_t i=0;i<count;++i) {  // 按 uint8_t i=0;i<count;++i 组织循环的初值、继续条件和步进。
        cargos[i]=single?r.cargo:r.kind==MotionKind::DEMO_PLACE?i+1:CargoForGrabIndex(i);  // 将 single?r.cargo:r.kind==MotionKind::DEMO_PLACE?i+1:Carg… 写入或初始化 cargos[i]。
        works[i]=single?r.work:demo?i+1:r.positions[i];  // 将 single?r.work:demo?i+1:r.positions[i] 写入或初始化 works[i]。
        const auto &cargo=materialTransferLayout.cargo,&work=materialTransferLayout.workArea[works[i]-1];  // 将 materialTransferLayout.cargo,&work=materialTransferLay… 写入或初始化 cargo（货位）。
        if(retrieve&&cargoPlatforms[cargos[i]-1].material!=MATERIAL_NONE)return "OCCUPIED";  // 若 retrieve&&cargoPlatforms[cargos[i]-1].material!=MATERI…，立即返回 "OCCUPIED"。
        if(!retrieve&&!demo&&!r.debug&&cargoPlatforms[cargos[i]-1].material==MATERIAL_NONE)return "EXECUTION";  // 若 !retrieve&&!demo&&!r.debug&&cargoPlatforms[cargos[i]-1…，立即返回 "EXECUTION"。
        angles[2*i]=retrieve?work.turretAngle:cargo.turretAngle;  // 将 retrieve?work.turretAngle:cargo.turretAngle 写入或初始化 angles[2*i]。
        angles[2*i+1]=retrieve?cargo.turretAngle:work.turretAngle;  // 将 retrieve?cargo.turretAngle:work.turretAngle 写入或初始化 angles[2*i+1]。
    }  // 结束此循环体。
    TurretPathPlan selected;if(!plan(angles,2*count,selected))return "EXECUTION";  // 声明 selected，类型和可用范围由所在声明决定。然后若 !plan(angles,2*count,selected)，立即返回 "EXECUTION"。
    for(uint8_t i=0;i<count;++i) {  // 按 uint8_t i=0;i<count;++i 组织循环的初值、继续条件和步进。
        const auto &cargo=materialTransferLayout.cargo,&work=materialTransferLayout.workArea[works[i]-1];  // 将 materialTransferLayout.cargo,&work=materialTransferLay… 写入或初始化 cargo（货位）。
        const auto &d=retrieve?workAreaToCargoDelays[works[i]-1][cargos[i]-1]:cargoToWorkAreaDelays[cargos[i]-1][works[i]-1];  // 将 retrieve?workAreaToCargoDelays[works[i]-1][cargos[i]-1… 写入或初始化 d。
        const auto &source=retrieve?work:cargo,&destination=retrieve?cargo:work;  // 将 retrieve?work:cargo,&destination=retrieve?cargo:work 写入或初始化 source。
        const float height=destination.high+(retrieve?0:(r.layer-1)*materialTransferLayout.secondLayerOffset);  // 将 destination.high+(retrieve?0:(r.layer-1)*materialTrans… 写入或初始化 height（高度）。
        prepareTray(p,cargos[i],false);  // 调用 prepareTray（prepare料盘），传入 p,cargos[i],false。
        pick(p,source,d,!retrieve,retrieve,&selected.anglesDeg[2*i],retrieve?0:cargos[i],WORK_TRANSFER_EXTENSION_ACCELERATION);  // 调用 pick，传入 p,source,d,!retrieve,retrieve,&selected.anglesDeg[2*i]…。
        place(p,destination,height,d,true,&selected.anglesDeg[2*i+1],!retrieve&&r.layer==1,  // 调用 place，传入 p,destination,height,d,true,&selected.anglesDeg[2*i+1]…。
              retrieve?0:works[i],retrieve?0:cargos[i],retrieve?cargos[i]:0,  // 为当前列表或跨行表达式提供这一项 retrieve?0:works[i],retrieve?0:cargos[i],retrieve?carg…。
              !retrieve&&r.ringCorrection,  // 为当前列表或跨行表达式提供这一项 !retrieve&&r.ringCorrection。
              !retrieve&&r.layer==2?WORK_TRANSFER_EXTENSION_ACCELERATION:ARM_EXTENSION_ACCELERATION);  // 续写当前声明或运算，提供 !retrieve&&r.layer==2?WORK_TRANSFER_EXTENSION_ACCELERA…。
        if(!retrieve) {  // 当 !retrieve 成立时进入此分支，否则继续后续流程。
            p.steps[p.count-1].workPlaceLiftArrival=true;  // 将 true 写入或初始化 workPlaceLiftArrival（工位Place升降Arrival）。
            p.steps[p.count-1].waitMs=0;  // 将 0 写入或初始化 waitMs（wait毫秒）。
        }  // 结束此条件分支。
        p.steps[p.count-1].materialEvent=retrieve?"WORK_PICK":"WORK_PLACE";  // 将 retrieve?"WORK_PICK":"WORK_PLACE" 写入或初始化 materialEvent（物料Event）。
        p.steps[p.count-1].materialPosition=works[i];  // 将 works[i] 写入或初始化 materialPosition（物料位置）。
        commit(p,cargos[i],retrieve?(demo?MATERIAL_RED:single?r.color:r.colors[i]):MATERIAL_NONE);  // 调用 commit，传入 p,cargos[i],retrieve?(demo?MATERIAL_RED:single?r.color…。
    }  // 结束此循环体。
    return nullptr;  // 结束当前函数并返回 nullptr。
}  // 结束此函数或控制块。
/**
 * @brief 构建START或LOOK系列姿态；视觉观察抬升按记录目标高度差计算等待，不把目标缓存当实测。
 */
void observation(Program &p,ArmObservation pose,uint8_t zone) {  // 定义函数或成员 的入口 observation，参数由此处的签名确定。
    // 仅观察姿态按记录的指令高度估算：170 mm 对应 1500 ms。
    const uint32_t liftWait=static_cast<uint32_t>(lroundf(fabsf(170.0f-currentArm.high)*1500.0f/170.0f));  // 将 static_cast<uint32_t>(lroundf(fabsf(170.0f-currentArm.… 写入或初始化 liftWait（升降Wait）。
    switch(pose) {  // 按 pose 的值选择对应分支。
    case ArmObservation::START:  // 当前值匹配 ArmObservation::START 时从这里执行。
        arm(p,"INIT_LIFT",160,-1,-1,0,300,0);  // 调用 arm（机械臂），传入 p,"INIT_LIFT",160,-1,-1,0,300,0。
        p.steps[p.count-1].liftArrival=true;  // 将 true 写入或初始化 liftArrival（升降Arrival）。
        rotateArm(p,"INIT_TURN",-1,-1,-58,-1,300);arm(p,"INIT_RETRACT",154,40,-1,0,300,500);break;  // 调用 rotateArm（rotate机械臂），传入 p,"INIT_TURN",-1,-1,-58,-1,300。然后调用 arm（机械臂），传入 p,"INIT_RETRACT",154,40,-1,0,300,500。然后退出当前循环或 switch 分支。
    case ArmObservation::LOOK:  // 当前值匹配 ArmObservation::LOOK 时从这里执行。
        arm(p,"LOOK_LIFT",170,0,-1,60,200,liftWait);rotateArm(p,"LOOK_TURN",-1,0,90,80,200);arm(p,"LOOK_READY",170,-1,-1,80,200);break;  // 调用 arm（机械臂），传入 p,"LOOK_LIFT",170,0,-1,60,200,liftWait。然后调用 rotateArm（rotate机械臂），传入 p,"LOOK_TURN",-1,0,90,80,200。然后调用 arm（机械臂），传入 p,"LOOK_READY",170,-1,-1,80,200。然后退出当前循环或 switch 分支。
    case ArmObservation::LOOK2:  // 当前值匹配 ArmObservation::LOOK2 时从这里执行。
        arm(p,"LOOK2_LIFT",170,0,-1,-1,200,liftWait);rotateArm(p,"LOOK2_TURN",-1,0,90,80,200);break;  // 调用 arm（机械臂），传入 p,"LOOK2_LIFT",170,0,-1,-1,200,liftWait。然后调用 rotateArm（rotate机械臂），传入 p,"LOOK2_TURN",-1,0,90,80,200。然后退出当前循环或 switch 分支。
    case ArmObservation::LOOK3: {  // 当前值匹配 ArmObservation::LOOK3 时从这里执行。
        arm(p,"LOOK3_LIFT",170,50,-1,-1,300,liftWait);rotateArm(p,"LOOK3_TURN",-1,-1,90,80,300);  // 调用 arm（机械臂），传入 p,"LOOK3_LIFT",170,50,-1,-1,300,liftWait。然后调用 rotateArm（rotate机械臂），传入 p,"LOOK3_TURN",-1,-1,90,80,300。
        Step s;s.kind=StepKind::TURRET_READY;s.name="LOOK3_MEASURED_READY";s.turret=90;s.waitMs=3000;p.append(s);break;  // 声明 s，类型和可用范围由所在声明决定。然后将 StepKind::TURRET_READY 写入或初始化 s.kind。然后将 "LOOK3_MEASURED_READY" 写入或初始化 s.name。然后将 90 写入或初始化 s.turret。然后将 3000 写入或初始化 s.waitMs。然后调用 p.append，传入 s。然后退出当前循环或 switch 分支。
    }  // 结束此条件分支。
    case ArmObservation::LOOK4:  // 当前值匹配 ArmObservation::LOOK4 时从这里执行。
        arm(p,"LOOK4_LIFT",170,-1,-1,-1,200,liftWait);  // 调用 arm（机械臂），传入 p,"LOOK4_LIFT",170,-1,-1,-1,200,liftWait。
        rotateArm(p,"LOOK4_READY",-1,zone==1?150:0,zone==1?270:90,80,200);  // 调用 rotateArm（rotate机械臂），传入 p,"LOOK4_READY",-1,zone==1?150:0,zone==1?270:90,80,200。
        p.steps[p.count-1].lengthArrival=true;  // 将 true 写入或初始化 lengthArrival（长度Arrival）。
        p.steps[p.count-1].lengthArrivalByPosition=true;  // 将 true 写入或初始化 lengthArrivalByPosition（长度ArrivalBy位置）。
        break;  // 退出当前循环或 switch 分支。
    default: break;  // 没有匹配的 case 时走此默认分支。
    }  // 结束此函数或控制块。
}  // 结束此函数或控制块。
}  // 结束命名空间。
/**
 * @brief 将动作请求展开为机械臂步骤序列；不下发运动命令、不等待动作结束，转台路径规划会回读实际角度。
 * @param r 抓放料、观察姿态或原始控制请求。
 * @param p 输出步骤序列，后续由 motion::Sequence 和 MotionHardware 执行。
 * @return nullptr 构建成功，否则返回错误原因字符串。
 */
const char *BuildArmProgram(const MotionRequest &r,Program &p) {  // 定义函数或成员 的入口 BuildArmProgram（Build机械臂Program），参数由此处的签名确定。
    p.count=0;  // 将 0 写入或初始化 p.count。
    if((r.kind==MotionKind::LIDAR_POSE||r.kind==MotionKind::HOME_OBSERVE||  // 当 (r.kind==MotionKind::LIDAR_POSE||r.kind==MotionKind::H… 成立时进入此分支，否则继续后续流程。
        (r.kind==MotionKind::ARM_POSE&&r.observation==ArmObservation::LOOK4))&&r.zone!=1&&r.zone!=2)return "EXECUTION";  // 继续上一行的组合条件，本行提供 (r.kind==MotionKind::ARM_POSE&&r.observation==ArmObservation::LOOK4))&&r.zone!=1&&r.zone!…。
    switch(r.kind) {  // 按 r.kind 的值选择对应分支。
    case MotionKind::ARM_POSE: observation(p,r.observation,r.zone);break;  // 当前值匹配 MotionKind::ARM_POSE 时从这里执行。
    case MotionKind::RING_PREPARE: {  // 当前值匹配 MotionKind::RING_PREPARE 时从这里执行。
        const auto &pose=materialTransferLayout.workArea[r.work-1];  // 将 materialTransferLayout.workArea[r.work-1] 写入或初始化 pose。
        TurretPathPlan selected;  // 声明 selected，类型和可用范围由所在声明决定。
        if(!plan(&pose.turretAngle,1,selected))return "EXECUTION";  // 若 !plan(&pose.turretAngle,1,selected)，立即返回 "EXECUTION"。
        arm(p,"RING_PREPARE_LIFT",materialTransferLayout.approachHeight,-1,-1,  // 调用 arm（机械臂），传入 p,"RING_PREPARE_LIFT",materialTransferLayout.approachH…。
            materialTransferLayout.clawClosedAngle,materialMotionSteps.approachTranslateSpeed);  // 续写当前声明或运算，提供 materialTransferLayout.clawClosedAngle,materialMotionS…。
        p.steps[p.count-1].liftArrival=true;  // 将 true 写入或初始化 liftArrival（升降Arrival）。
        rotateArm(p,"RING_PREPARE_ROTATE",-1,pose.length,selected.anglesDeg[0],-1,  // 调用 rotateArm（rotate机械臂），传入 p,"RING_PREPARE_ROTATE",-1,pose.length,selected.angles…。
            materialMotionSteps.approachRotateSpeed,true);  // 续写当前声明或运算，提供 materialMotionSteps.approachRotateSpeed,true);。
        p.steps[p.count-1].lengthArrival=true;  // 将 true 写入或初始化 lengthArrival（长度Arrival）。
        arm(p,"RING_PREPARE_HOVER",ring::HOVER_MM,-1,-1,-1,materialMotionSteps.placeDescendSpeed);  // 调用 arm（机械臂），传入 p,"RING_PREPARE_HOVER",ring::HOVER_MM,-1,-1,-1,materia…。
        p.steps[p.count-1].liftArrival=true;  // 将 true 写入或初始化 liftArrival（升降Arrival）。
        p.steps[p.count-1].slowLower=true;  // 将 true 写入或初始化 slowLower。
        break;  // 退出当前循环或 switch 分支。
    }  // 结束此条件分支。
    case MotionKind::TRANSFER: case MotionKind::RETRIEVE: case MotionKind::PLACE_ROUND:  // 当前值匹配 MotionKind::TRANSFER 时从这里执行。
    case MotionKind::RETRIEVE_ROUND: case MotionKind::DEMO_PLACE: case MotionKind::DEMO_RETRIEVE: {  // 当前值匹配 MotionKind::RETRIEVE_ROUND 时从这里执行。
        const char *error=transfer(r,p);if(error)return error;break;  // 将 transfer(r,p) 写入或初始化 error（错误）。然后若 error，立即返回 error。然后退出当前循环或 switch 分支。
    }  // 结束此条件分支。
    case MotionKind::HELD_DISC:  // 当前值匹配 MotionKind::HELD_DISC 时从这里执行。
    case MotionKind::DISC_GRAB:  // 当前值匹配 MotionKind::DISC_GRAB 时从这里执行。
    case MotionKind::LOAD_ROUND: {  // 当前值匹配 MotionKind::LOAD_ROUND 时从这里执行。
        const uint8_t count=r.kind==MotionKind::LOAD_ROUND?3:1;  // 将 r.kind==MotionKind::LOAD_ROUND?3:1 写入或初始化 count（计数）。
        for(uint8_t i=0;i<count;++i) {  // 按 uint8_t i=0;i<count;++i 组织循环的初值、继续条件和步进。
            const uint8_t cargo=r.kind==MotionKind::LOAD_ROUND?CargoForGrabIndex(i):r.cargo;  // 将 r.kind==MotionKind::LOAD_ROUND?CargoForGrabIndex(i):r.… 写入或初始化 cargo（货位）。
            if(cargoPlatforms[cargo-1].material!=MATERIAL_NONE&&!r.force)return "OCCUPIED";  // 若 cargoPlatforms[cargo-1].material!=MATERIAL_NONE&&!r.fo…，立即返回 "OCCUPIED"。
        }  // 结束此循环体。
        for(uint8_t i=0;i<count;++i) {  // 按 uint8_t i=0;i<count;++i 组织循环的初值、继续条件和步进。
            const uint8_t cargo=r.kind==MotionKind::LOAD_ROUND?CargoForGrabIndex(i):r.cargo,color=r.kind==MotionKind::LOAD_ROUND?r.colors[i]:r.color;  // 将 r.kind==MotionKind::LOAD_ROUND?CargoForGrabIndex(i):r.… 写入或初始化 cargo（货位）。
            const auto &destination=materialTransferLayout.cargo;  // 将 materialTransferLayout.cargo 写入或初始化 destination。
            const bool positionGrab=r.kind!=MotionKind::HELD_DISC&&r.position>=1&&r.position<=3;  // 将 r.kind!=MotionKind::HELD_DISC&&r.position>=1&&r.positi… 写入或初始化 positionGrab（位置Grab）。
            prepareTray(p,cargo,false);  // 调用 prepareTray（prepare料盘），传入 p,cargo,false。
            if(r.kind!=MotionKind::HELD_DISC) {  // 当 r.kind!=MotionKind::HELD_DISC 成立时进入此分支，否则继续后续流程。
                if(positionGrab) {  // 当 positionGrab 成立时进入此分支，否则继续后续流程。
                    const size_t approach=p.count;  // 将 p.count 写入或初始化 approach。
                    const float length=r.position==2?0:150,angle=r.position==1?71:r.position==2?90:102;  // 将 r.position==2?0:150,angle=r.position==1?71:r.position=… 写入或初始化 length（长度）。
                    arm(p,"DISC_APPROACH",80,length,angle,80,armSpeed,350);  // 调用 arm（机械臂），传入 p,"DISC_APPROACH",80,length,angle,80,armSpeed,350。
                    arm(p,"DISC_GRIP",80,length,angle,-8,armSpeed,400,false,ARM_GRIP_HOLD_MS);  // 调用 arm（机械臂），传入 p,"DISC_GRIP",80,length,angle,-8,armSpeed,400,false,AR…。
                    arm(p,"DISC_LIFT",materialTransferLayout.approachHeight,0,-1,-1,armSpeed,commonMaterialDelays.pickLiftMs);  // 调用 arm（机械臂），传入 p,"DISC_LIFT",materialTransferLayout.approachHeight,0,…。
                    for(size_t step=approach;step<p.count;++step)p.steps[step].lengthAcceleration=245;  // 按 size_t step=approach;step<p.count;++step)p.steps[step]… 组织循环的初值、继续条件和步进。
                } else if(r.position==4||r.kind==MotionKind::LOAD_ROUND)pick(p,materialTransferLayout.disc,commonMaterialDelays,false,true,nullptr);  // 仅在先前条件均不成立时，当 r.position==4||r.kind==MotionKind::LOAD_ROUND 成立时进入此分支，否则继续后续流程。
                else {  // 前一个判断不成立时执行此分支。
                    const auto &d=materialMotionSteps.discApproach;  // 将 materialMotionSteps.discApproach 写入或初始化 d。
                    arm(p,"DISC_APPROACH",d.high,d.length,d.turretAngle,materialMotionSteps.discOpenAngle,materialMotionSteps.discApproachSpeed,materialAuxiliaryDelays.discApproachMs);  // 调用 arm（机械臂），传入 p,"DISC_APPROACH",d.high,d.length,d.turretAngle,materi…。
                    arm(p,"DISC_GRIP",d.high,d.length,d.turretAngle,materialTransferLayout.clawClosedAngle,materialMotionSteps.discGripSpeed,commonMaterialDelays.pickGripMs,false,ARM_GRIP_HOLD_MS);  // 调用 arm（机械臂），传入 p,"DISC_GRIP",d.high,d.length,d.turretAngle,materialTr…。
                    arm(p,"DISC_LIFT",materialTransferLayout.approachHeight,0,-1,-1,materialMotionSteps.discLiftSpeed,commonMaterialDelays.pickLiftMs);  // 调用 arm（机械臂），传入 p,"DISC_LIFT",materialTransferLayout.approachHeight,0,…。
                }  // 结束此条件分支。
            } else {  // 前一个判断不成立时执行此分支。
                arm(p,"DISC_LIFT",materialTransferLayout.approachHeight,0,-1,-1,materialMotionSteps.discLiftSpeed,commonMaterialDelays.pickLiftMs);  // 调用 arm（机械臂），传入 p,"DISC_LIFT",materialTransferLayout.approachHeight,0,…。
            }  // 结束此条件分支。
            // 抬升缩回并转向料盘，确认目标盘到位后再伸出下降。
            auto delays=commonMaterialDelays;  // 将 commonMaterialDelays 写入或初始化 delays。
            if(positionGrab)delays.placeDescendMs=200;  // 当 positionGrab 成立时进入此分支，否则继续后续流程。
            place(p,destination,destination.high,delays,true,nullptr,false,0,0,cargo,false,  // 调用 place，传入 p,destination,destination.high,delays,true,nullptr,fal…。
                ARM_EXTENSION_ACCELERATION,positionGrab?3.0f:CARGO_TRAY_ARRIVAL_TOLERANCE_DEG);  // 续写当前声明或运算，提供 ARM_EXTENSION_ACCELERATION,positionGrab?3.0f:CARGO_TRA…。
            if(r.kind!=MotionKind::HELD_DISC) {  // 当 r.kind!=MotionKind::HELD_DISC 成立时进入此分支，否则继续后续流程。
                p.steps[p.count-1].materialEvent="DISC_PICK";p.steps[p.count-1].materialPosition=r.position;  // 将 "DISC_PICK" 写入或初始化 materialEvent（物料Event）。然后将 r.position 写入或初始化 materialPosition（物料位置）。
            }  // 结束此条件分支。
            commit(p,cargo,color);  // 调用 commit，传入 p,cargo,color。
        }  // 结束此循环体。
        break;  // 退出当前循环或 switch 分支。
    }  // 结束此条件分支。
    case MotionKind::LIDAR_POSE:  // 当前值匹配 MotionKind::LIDAR_POSE 时从这里执行。
        position(p,r.zone==1?100:-100,100,0,200);  // 调用 position（位置），传入 p,r.zone==1?100:-100,100,0,200。
        position(p,0,0,r.zone==1?-45:45,200);  // 调用 position（位置），传入 p,0,0,r.zone==1?-45:45,200。
        break;  // 退出当前循环或 switch 分支。
    case MotionKind::HOME_OBSERVE:  // 当前值匹配 MotionKind::HOME_OBSERVE 时从这里执行。
        position(p,0,0,NodeRouteTurn(currentPose.theta,HOME_ZONE_HEADING),80);  // 调用 position（位置），传入 p,0,0,NodeRouteTurn(currentPose.theta,HOME_ZONE_HEADIN…。
        if(!r.armPrepared)observation(p,ArmObservation::LOOK4,r.zone);  // 当 !r.armPrepared 成立时进入此分支，否则继续后续流程。
        if(r.zone==2)position(p,150,0,0,80); // 按校角后的车身方向右移，再开启角点视觉。
        break;  // 退出当前循环或 switch 分支。
    case MotionKind::HOME_RETURN: {  // 当前值匹配 MotionKind::HOME_RETURN 时从这里执行。
        observation(p,ArmObservation::START,r.zone);  // 调用 observation，传入 p,ArmObservation::START,r.zone。
        position(p,r.zone==1?-115:140,r.zone==1?125:155,0,200);break;  // 调用 position（位置），传入 p,r.zone==1?-115:140,r.zone==1?125:155,0,200。然后退出当前循环或 switch 分支。
    }  // 结束此条件分支。
    case MotionKind::POSITION: position(p,r.x,r.y,r.theta,r.speed,r.positionAcceleration);break;  // 当前值匹配 MotionKind::POSITION 时从这里执行。
    case MotionKind::GOTO: {Step s;s.kind=StepKind::GOTO;s.name="GOTO";s.high=r.x;s.length=r.y;s.turret=r.theta;p.append(s);break;}  // 当前值匹配 MotionKind::GOTO 时从这里执行。
    case MotionKind::ARM_RAW: arm(p,"ARM_RAW",r.high,r.length,r.turret,r.pawl,r.speed,0,r.planned);break;  // 当前值匹配 MotionKind::ARM_RAW 时从这里执行。
    case MotionKind::SERVO_RAW: {Step s;s.kind=StepKind::RAW_SERVO;s.name="SERVO";s.cargo=r.cargo;s.turret=r.turret;p.append(s);break;}  // 当前值匹配 MotionKind::SERVO_RAW 时从这里执行。
    case MotionKind::TRAY_ROTATE: rotateTray(p,r.turret,r.trayIntervalMs,r.trayPowerMw);break;  // 当前值匹配 MotionKind::TRAY_ROTATE 时从这里执行。
    case MotionKind::ENABLE: {Step s;s.kind=StepKind::ENABLE;s.name="ENABLE";s.cargo=r.force;p.append(s);break;}  // 当前值匹配 MotionKind::ENABLE 时从这里执行。
    case MotionKind::VELOCITY: {Step s;s.kind=StepKind::VELOCITY;s.name="VELOCITY";s.cargo=r.position;s.speed=r.speed;s.color=r.force;p.append(s);break;}  // 当前值匹配 MotionKind::VELOCITY 时从这里执行。
    default: break;  // 没有匹配的 case 时走此默认分支。
    }  // 结束此函数或控制块。
    // 整段搬运使用同一曲线：第一层放料慢，收料和第二层叠放快；圆盘全程快。
    uint8_t turretAcceleration=TURRET_ACCELERATION_FAST;  // 将 TURRET_ACCELERATION_FAST 写入或初始化 turretAcceleration（转台加速度）。
    switch(r.kind) {  // 按 r.kind 的值选择对应分支。
    case MotionKind::DISC_GRAB: case MotionKind::LOAD_ROUND: case MotionKind::HELD_DISC:  // 当前值匹配 MotionKind::DISC_GRAB 时从这里执行。
        turretAcceleration=TURRET_ACCELERATION_FAST;break;  // 将 TURRET_ACCELERATION_FAST 写入或初始化 turretAcceleration（转台加速度）。然后退出当前循环或 switch 分支。
    case MotionKind::RETRIEVE: case MotionKind::RETRIEVE_ROUND: case MotionKind::DEMO_RETRIEVE:  // 当前值匹配 MotionKind::RETRIEVE 时从这里执行。
        turretAcceleration=TURRET_ACCELERATION_FAST;break;  // 将 TURRET_ACCELERATION_FAST 写入或初始化 turretAcceleration（转台加速度）。然后退出当前循环或 switch 分支。
    case MotionKind::TRANSFER: case MotionKind::PLACE_ROUND:  // 当前值匹配 MotionKind::TRANSFER 时从这里执行。
        turretAcceleration=r.layer==1?TURRET_ACCELERATION_SLOW:TURRET_ACCELERATION_FAST;break;  // 将 r.layer==1?TURRET_ACCELERATION_SLOW:TURRET_ACCELERATIO… 写入或初始化 turretAcceleration（转台加速度）。然后退出当前循环或 switch 分支。
    case MotionKind::HOME_OBSERVE: // 返家角点观察姿态使用慢档。
    case MotionKind::DEMO_PLACE:  // 当前值匹配 MotionKind::DEMO_PLACE 时从这里执行。
        turretAcceleration=TURRET_ACCELERATION_SLOW;break;  // 将 TURRET_ACCELERATION_SLOW 写入或初始化 turretAcceleration（转台加速度）。然后退出当前循环或 switch 分支。
    case MotionKind::ARM_POSE:  // 当前值匹配 MotionKind::ARM_POSE 时从这里执行。
        if(r.observation==ArmObservation::LOOK4)turretAcceleration=TURRET_ACCELERATION_SLOW;  // 当 r.observation==ArmObservation::LOOK4 成立时进入此分支，否则继续后续流程。
        break;  // 退出当前循环或 switch 分支。
    default: break;  // 没有匹配的 case 时走此默认分支。
    }  // 结束此函数或控制块。
    for(size_t i=0;i<p.count;++i) {  // 按 size_t i=0;i<p.count;++i 组织循环的初值、继续条件和步进。
        auto &step=p.steps[i];  // 将 p.steps[i] 写入或初始化 step。
        if(step.kind==StepKind::ARM)  // 当 step.kind==StepKind::ARM 成立时进入此分支，否则继续后续流程。
            step.turretAcceleration=step.cargoPlaceTurn?TURRET_ACCELERATION_CARGO_PLACE:turretAcceleration;  // 将 step.cargoPlaceTurn?TURRET_ACCELERATION_CARGO_PLACE:tu… 写入或初始化 step.turretAcceleration。
    }  // 结束此循环体。
    return nullptr;  // 结束当前函数并返回 nullptr。
}  // 结束此函数或控制块。
