#include "arm_motion.h"
#include "material_transfer.h"
#include "motion_executor.h"
#include "disc_position_grab.h"
#include <assert.h>
#include <string.h>
#include <vector>
#include <set>
#include <string>
#include <float.h>

NativeSerial Serial;
RobotPose currentPose{1200,1200,180};ArmPose currentArm{};
volatile bool taskMotionAborted=false;
uint32_t ARM_FEEDBACK_ENABLED=0,ARM_GRIP_HOLD_MS=37,ARM_RELEASE_HOLD_MS=41;
float MOTION_LOOK_LEAD_MM=0,MOTION_LOOK2_LEAD_MM=0;
int armSpeed=500;
bool angleAvailable=true;
bool ReadArmTurretAngle(float &angle){angle=170;return angleAvailable;}
uint32_t SubmitMotion(const MotionRequest &,bool){return 1;}
bool WaitMotion(uint32_t){return false;}
std::vector<motion::Step> issuedArm;
std::vector<uint32_t> armWaits;
void vTaskDelay(uint32_t ticks){armWaits.push_back(ticks);}
bool MoveArm(float high,float length,float turret,float pawl,float speed) {
    motion::Step s;s.high=high;s.length=length;s.turret=turret;s.pawl=pawl;s.speed=speed;
    issuedArm.push_back(s);return true;
}
bool MoveArmPlanned(float high,float length,float turret,float pawl,float speed) {
    return MoveArm(high,length,turret,pawl,speed);
}
float NODE_PATH_ACCEL_MM_S2=2000,NODE_PATH_TURN_DEG_S=300,NODE_PATH_TURN_ACCEL_DEG_S2=400,NODE_PATH_HEADING_DEADBAND_DEG=1;
struct Entry {std::string name;void *address;bool integer;float minimum,maximum;};
std::vector<Entry> entries;
void RegisterRuntimeFloat(const char *name,float &value,float minimum,float maximum){entries.push_back({name,&value,false,minimum,maximum});}
void RegisterRuntimeUInt(const char *name,uint32_t &value,float minimum,float maximum){entries.push_back({name,&value,true,minimum,maximum});}
void RegisterCornerAlignmentParameters(){}
#include "delay_registry_actual.inc"

bool near(float a,float b){return fabsf(a-b)<.001f;}
void forward() {
    for(uint8_t c=1;c<=3;++c)for(uint8_t w=1;w<=3;++w)for(uint8_t layer=1;layer<=3;++layer) {
        cargoPlatforms[c-1].material=MATERIAL_BLUE;
        MotionRequest r;r.kind=MotionKind::TRANSFER;r.cargo=c;r.work=w;r.layer=layer;
        motion::Program p;assert(!BuildArmProgram(r,p));assert(p.count<=motion::Program::CAPACITY);
        assert(cargoPlatforms[c-1].material==MATERIAL_BLUE); // 构建、提交不是搬运完成。
        const auto &delays=cargoToWorkAreaDelays[c-1][w-1];
        assert(p.steps[0].waitMs==delays.pickAbove.translateMs);
        assert(near(p.steps[0].length,materialTransferLayout.cargo[c-1].length));
        bool release=false,retract=false;int commits=0;
        for(size_t i=0;i<p.count;++i) {
            const auto &s=p.steps[i];
            if(!strcmp(s.name,"PICK_RETRACT")){assert(s.length==0);retract=true;}
            if(!strcmp(s.name,"PLACE_DESCEND"))assert(near(s.high,materialTransferLayout.workArea[w-1].high+(layer-1)*materialTransferLayout.secondLayerOffset));
            if(!strcmp(s.name,"PLACE_RELEASE")){assert(s.waitMs==delays.placeReleaseMs&&s.holdMs==41);release=true;}
            if(s.kind==motion::StepKind::COMMIT){assert(release&&retract&&i==p.count-1&&s.cargo==c&&s.color==MATERIAL_NONE);++commits;}
            if(s.planned)assert(s.turret>=TURRET_CABLE_MIN_DEG&&s.turret<=TURRET_CABLE_MAX_DEG);
        }
        assert(commits==1);
    }
}
void reverse() {
    for(uint8_t c=1;c<=3;++c)for(uint8_t w=1;w<=3;++w) {
        cargoPlatforms[c-1].material=MATERIAL_NONE;
        MotionRequest r;r.kind=MotionKind::RETRIEVE;r.cargo=c;r.work=w;r.color=6;
        motion::Program p;assert(!BuildArmProgram(r,p));
        assert(p.steps[0].waitMs==workAreaToCargoDelays[w-1][c-1].pickAbove.translateMs);
        bool lift=false;
        for(size_t i=0;i<p.count;++i)if(!strcmp(p.steps[i].name,"PICK_LIFT")){assert(p.steps[i].length==0);lift=true;}
        assert(lift&&p.steps[p.count-1].color==6&&p.steps[p.count-1].cargo==c);
        assert(cargoPlatforms[c-1].material==MATERIAL_NONE);
    }
}
void rounds() {
    MotionRequest r;r.kind=MotionKind::PLACE_ROUND;r.layer=2;
    const int positions[3]={3,1,2};for(int i=0;i<3;++i){r.positions[i]=positions[i];cargoPlatforms[i].material=MATERIAL_RED;}
    motion::Program p;assert(!BuildArmProgram(r,p));assert(p.count==42);
    int commits=0;for(size_t i=0;i<p.count;++i)if(p.steps[i].kind==motion::StepKind::COMMIT){++commits;assert(p.steps[i].cargo==commits);}
    assert(commits==3);angleAvailable=false;assert(BuildArmProgram(r,p));
    for(const auto &cargo:cargoPlatforms)assert(cargo.material==MATERIAL_RED);
}
void discs() {
    const auto saved=commonMaterialDelays;
    const float savedHeight=materialTransferLayout.approachHeight;
    // 使用独特数值确认圆盘后半段绑定公共表，而非硬编码或辅助圆盘延时。
    commonMaterialDelays.pickGripMs=713;commonMaterialDelays.pickLiftMs=719;
    commonMaterialDelays.placeAbove={101,11,1021};
    commonMaterialDelays.placeDescendMs=733;commonMaterialDelays.placeReleaseMs=313;commonMaterialDelays.placeLiftMs=317;
    materialTransferLayout.approachHeight=165;
    const auto &d=commonMaterialDelays;
    auto checkTail=[&](const motion::Program &p,size_t start,uint8_t cargo) {
        const char *names[]={"PLACE_TRANSLATE_WAIT","PLACE_SETTLE","PLACE_ROTATE","PLACE_DESCEND","PLACE_RELEASE","PLACE_LIFT","CARGO_COMMIT"};
        const uint32_t waits[]={d.placeAbove.translateMs,d.placeAbove.settleMs,d.placeAbove.rotateMs,d.placeDescendMs,d.placeReleaseMs,d.placeLiftMs,0};
        for(size_t i=0;i<7;++i){assert(!strcmp(p.steps[start+i].name,names[i]));assert(p.steps[start+i].waitMs==waits[i]);}
        const auto &lift=p.steps[start-1],&turn=p.steps[start+2],&down=p.steps[start+3];
        const auto &target=materialTransferLayout.cargo[cargo-1];
        assert(lift.high==165&&lift.length==0&&lift.turret==-1&&lift.pawl==-1&&lift.waitMs==d.pickLiftMs);
        assert(turn.high==-1&&turn.length==-1&&turn.pawl==-1&&near(turn.turret,target.turretAngle)&&!turn.planned);
        assert(near(down.high,target.high)&&near(down.length,target.length)&&down.turret==-1);
        assert(p.steps[start+4].pawl==materialTransferLayout.clawOpenAngle&&p.steps[start+4].holdMs==41);
        assert(p.steps[start+5].high==165&&p.steps[start+5].length==-1);
        const auto &last=p.steps[start+6];assert(last.kind==motion::StepKind::COMMIT&&last.cargo==cargo&&last.color==6);
    };
    for(uint8_t cargo=1;cargo<=3;++cargo)for(uint8_t pos=0;pos<=4;++pos) {
        cargoPlatforms[cargo-1].material=MATERIAL_NONE;
        MotionRequest r;r.kind=MotionKind::DISC_GRAB;r.cargo=cargo;r.color=6;r.position=pos;
        motion::Program p;assert(!BuildArmProgram(r,p));
        checkTail(p,p.count-7,cargo);
        assert(p.steps[p.count-9].waitMs==d.pickGripMs&&p.steps[p.count-9].holdMs==37);
        if(pos>=1&&pos<=3) {
            const float angle=pos==1?71:pos==2?90:102,length=pos==2?0:150;
            assert(p.steps[0].high==120&&p.steps[1].high==80&&p.steps[2].pawl==-8);
            assert(p.steps[0].waitMs==(pos==3?200:300)&&p.steps[1].waitMs==300);
            for(int i=0;i<3;++i)assert(p.steps[i].turret==angle&&p.steps[i].length==length&&p.steps[i].speed==armSpeed);
            assert(p.steps[3].speed==armSpeed);
        } else if(pos==0) {
            assert(p.steps[0].high==materialMotionSteps.discApproach.high);
            assert(p.steps[0].waitMs==materialAuxiliaryDelays.discApproachMs);
            assert(p.steps[2].speed==materialMotionSteps.discLiftSpeed);
        } else assert(p.steps[p.count-8].speed==materialMotionSteps.pickLiftSpeed);
        assert(cargoPlatforms[cargo-1].material==MATERIAL_NONE);
        r.kind=MotionKind::HELD_DISC;assert(!BuildArmProgram(r,p));
        assert(p.count==8);checkTail(p,1,cargo);
    }
    for(auto &cargo:cargoPlatforms)cargo.material=MATERIAL_NONE;
    MotionRequest load;load.kind=MotionKind::LOAD_ROUND;
    for(int i=0;i<3;++i)load.colors[i]=6;
    motion::Program loaded;assert(!BuildArmProgram(load,loaded));assert(loaded.count==39);
    for(int i=0;i<3;++i)checkTail(loaded,13*i+6,i+1);
    // 同步旧入口仍在当前工程中，核对它们与执行器使用相同的夹后衔接。
    for(uint8_t cargo=1;cargo<=3;++cargo)for(int mode=0;mode<3;++mode) {
        cargoPlatforms[cargo-1].material=MATERIAL_NONE;issuedArm.clear();armWaits.clear();
        assert(mode==0?MoveDiscToCargo(6,cargo):mode==1?GrabDiscMaterial(6,cargo):PlaceHeldDiscMaterialToCargo(6,cargo));
        const auto &target=materialTransferLayout.cargo[cargo-1];
        const size_t start=issuedArm.size()-5;
        assert(issuedArm[start].high==165&&issuedArm[start].length==0&&issuedArm[start].turret==-1&&issuedArm[start].pawl==-1);
        assert(issuedArm[start+1].high==-1&&issuedArm[start+1].length==-1&&near(issuedArm[start+1].turret,target.turretAngle));
        assert(near(issuedArm[start+2].high,target.high)&&near(issuedArm[start+2].length,target.length)&&issuedArm[start+2].turret==-1);
        assert(issuedArm[start+3].pawl==materialTransferLayout.clawOpenAngle&&issuedArm[start+4].high==165);
        const uint32_t waits[]={d.pickLiftMs,d.placeAbove.translateMs,d.placeAbove.settleMs,d.placeAbove.rotateMs,d.placeDescendMs,d.placeReleaseMs,d.placeLiftMs};
        assert(armWaits.size()>=7);
        for(int i=0;i<7;++i)assert(armWaits[armWaits.size()-7+i]==waits[i]);
        assert(cargoPlatforms[cargo-1].material==6);
    }
    commonMaterialDelays=saved;materialTransferLayout.approachHeight=savedHeight;
}
void poses() {
    for(auto pose:{ArmObservation::START,ArmObservation::LOOK,ArmObservation::LOOK2,ArmObservation::LOOK3}) {
        MotionRequest r;r.kind=MotionKind::ARM_POSE;r.observation=pose;
        motion::Program p;assert(!BuildArmProgram(r,p));
        if(pose==ArmObservation::START)assert(p.steps[0].waitMs==800&&p.steps[1].waitMs==100&&p.steps[2].waitMs==500);
        if(pose==ArmObservation::LOOK)assert(p.steps[0].waitMs==1500&&p.steps[1].waitMs==100&&p.steps[2].waitMs==0);
        if(pose==ArmObservation::LOOK2)assert(p.steps[0].waitMs==1500&&p.count==2);
        if(pose==ArmObservation::LOOK3)assert(p.steps[p.count-1].kind==motion::StepKind::TURRET_READY&&p.steps[p.count-1].waitMs==1500);
    }
    const float heights[]={0,85,170};const uint32_t waits[]={1500,750,0};
    for(auto pose:{ArmObservation::LOOK,ArmObservation::LOOK2,ArmObservation::LOOK3,ArmObservation::LOOK4}) {
        for(int i=0;i<3;++i) {
            currentArm.high=heights[i];
            MotionRequest r;r.kind=MotionKind::ARM_POSE;r.observation=pose;r.zone=1;
            motion::Program p;assert(!BuildArmProgram(r,p));assert(p.steps[0].waitMs==waits[i]);
        }
    }
    currentArm.high=0;
    MotionRequest r;r.kind=MotionKind::HOME_OBSERVE;r.zone=2;motion::Program p;assert(!BuildArmProgram(r,p));
    assert(p.steps[0].kind==motion::StepKind::POSITION&&p.steps[p.count-1].turret==135);
    r.armPrepared=true;assert(!BuildArmProgram(r,p));
    assert(p.count==1&&p.steps[0].kind==motion::StepKind::POSITION); // 已提前 LOOK4，只保留航向修正。
    r.kind=MotionKind::HOME_RETURN;r.zone=1;assert(!BuildArmProgram(r,p));
    assert(p.steps[p.count-2].high==-70&&p.steps[p.count-1].length==-200&&p.steps[p.count-1].waitMs==1000);
}
void registry() {
    registrationTail();assert(entries.size()==244);
    std::set<std::string> names;
    for(const auto &e:entries)assert(names.insert(e.name).second);
    assert(entries[0].address==&commonMaterialDelays.pickAbove.translateMs);
    assert(entries[120].address==&materialAuxiliaryDelays.discApproachMs);
    assert(entries[126].address==&materialAuxiliaryDelays.lidarZone2TurnMs);
    assert(entries[127].name=="底盘/节点导航平移加速度(mm/s²)");
    assert(entries[130].name=="底盘/节点导航独立纠偏死区(°)");
    const char *steps[]={"取料平移","取料稳定","取料转向","取料下降","取料夹紧","取料抬升","放料平移","放料稳定","放料转向","放料下降","放料松爪","放料抬升"};
    for(uint8_t w=0;w<3;++w)for(uint8_t c=0;c<3;++c) {
        auto &d=workAreaToCargoDelays[w][c];uint32_t *values[]={&d.pickAbove.translateMs,&d.pickAbove.settleMs,&d.pickAbove.rotateMs,&d.pickDescendMs,&d.pickGripMs,&d.pickLiftMs,
            &d.placeAbove.translateMs,&d.placeAbove.settleMs,&d.placeAbove.rotateMs,&d.placeDescendMs,&d.placeReleaseMs,&d.placeLiftMs};
        for(size_t step=0;step<12;++step) {
            const auto &e=entries[131+(3*w+c)*12+step];char expected[100];
            snprintf(expected,sizeof(expected),"动作组延时/工作区%u到载物台%u/%s(ms)",w+1,c+1,steps[step]);
            assert(e.name==expected&&e.address==values[step]&&e.integer&&e.minimum==0&&e.maximum==60000);
        }
    }
    assert(entries[239].address==&MOTION_LOOK_LEAD_MM&&entries[241].address==&ARM_FEEDBACK_ENABLED);
}
int main(int argc,char **argv) {
    assert(argc==2);
    if(!strcmp(argv[1],"forward"))forward();else if(!strcmp(argv[1],"reverse"))reverse();
    else if(!strcmp(argv[1],"rounds"))rounds();else if(!strcmp(argv[1],"discs"))discs();
    else if(!strcmp(argv[1],"poses"))poses();else if(!strcmp(argv[1],"registry"))registry();else assert(false);
}
