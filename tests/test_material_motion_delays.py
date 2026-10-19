"""用实际搬运代码做编译期验证：9 组隔离、等待顺序、失败中止及层高。"""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class MaterialMotionDelayTests(unittest.TestCase):
    def test_actual_transfer_sequences(self):
        header = (ROOT / "src/material_transfer.h").read_text(encoding="utf-8")
        header = re.sub(r"^#include.*\n", "", header, flags=re.M)
        source = (ROOT / "src/material_transfer.cpp").read_text(encoding="utf-8")
        source = re.sub(r"^#include.*\n", "", source, flags=re.M)
        source = source.replace("namespace {", "").replace("} // namespace", "")
        # 默认参数引用全局配置；放进 Run 实例后，把省略的参数显式补齐。
        source = source.replace(" &delays = commonMaterialDelays", " &delays")
        source = source.replace("float durationMs = materialTransferLayout.motionWaitMs", "float durationMs")
        source = source.replace("float releaseHeights[MATERIAL_STATION_COUNT];",
                                "float releaseHeights[MATERIAL_STATION_COUNT] = {};")
        source = re.sub(r"pickAt\(([^(),\n]+)\)", r"pickAt(\1, commonMaterialDelays)", source)
        source = re.sub(r"placeAt\(([^(),\n]+), ([^(),\n]+)\)",
                        r"placeAt(\1, \2, commonMaterialDelays)", source)
        source = re.sub(r"^(bool|void) (\w+\()", r"constexpr \1 \2", source, flags=re.M)
        planner = (ROOT / "src/turret_path.h").read_text(encoding="utf-8")
        planner = planner.replace("#pragma once", "").replace("inline bool PlanTurretPath(",
                                                              "constexpr bool PlanTurretPath(")
        harness = r'''
#include <stdint.h>
PLANNER
constexpr bool isfinite(float v) { return __builtin_isfinite(v); }
constexpr long lroundf(float v) { return v < 0 ? long(v - 0.5f) : long(v + 0.5f); }
constexpr int pdMS_TO_TICKS(int v) { return v; }
constexpr float ARM_HEIGHT_LIMIT_MM=175, TURRET_CABLE_MIN_DEG=-150, TURRET_CABLE_MAX_DEG=360;
HEADER
struct Log {
    constexpr void println(const char*) {}
    template<class... A> constexpr void printf(const char*, A...) {}
};
struct Command { float high, length, turret, claw, speed; int waitsBefore; bool planned=false; };
struct Run {
    Log Serial;
    int moves=0, waits=0, failAt=0;
    int feedbackReads=0;
    float measuredTurret=0;
    bool feedbackOk=true;
    Command commands[64] = {};
    uint32_t delays[96] = {};
    constexpr bool MoveArm(float h, float l, float t, float c, float s) {
        commands[moves++]={h,l,t,c,s,waits};
        if (moves==failAt) return false;
        if (t!=-1) measuredTurret=t;
        return true;
    }
    constexpr bool MoveArmPlanned(float h, float l, float t, float c, float s) {
        commands[moves++]={h,l,t,c,s,waits,true};
        if (moves==failAt) return false;
        measuredTurret=t;
        return true;
    }
    constexpr bool ReadArmTurretAngle(float &angle) {
        ++feedbackReads;
        if (!feedbackOk) return false;
        angle=measuredTurret;
        return true;
    }
    constexpr void GotoPose(float, float, float, bool) {}
    constexpr bool MovePosition(float, float, float, float) { return true; }
    constexpr void vTaskDelay(int ms) { delays[waits++]=ms; }
    SOURCE
};
constexpr MaterialTransferDelays uniqueDelays(int base) {
    return {{base+1u,base+2u,base+3u},base+4u,base+5u,base+6u,
            {base+7u,base+8u,base+9u},base+10u,base+11u,base+12u};
}
constexpr void reverseProfiles(Run &r, int base) {
    for (int w=0;w<3;++w) for (int c=0;c<3;++c)
        r.workAreaToCargoDelays[w][c]=uniqueDelays(base+100*w+20*c);
}
constexpr bool matches(const Run &r, int offset, int base) {
    for (int k=0; k<12; ++k) if (r.delays[offset+k] != base+k+1u) return false;
    return true;
}
constexpr bool sameOrientation(float actual, float logical) {
    return actual==logical || actual==logical-360.0f || actual==logical+360.0f;
}
constexpr bool plannedRoute(const Run &r, float start, const float *targets, size_t count) {
    TurretPathPlan expected;
    if (!PlanTurretPath(start,targets,count,TURRET_CABLE_MIN_DEG,TURRET_CABLE_MAX_DEG,expected)
            || r.feedbackReads!=1) return false;
    size_t index=0;
    for (int i=0;i<r.moves;++i) if (r.commands[i].planned) {
        if (index==count || r.commands[i].turret!=expected.anglesDeg[index++]) return false;
    }
    return index==count && r.measuredTurret==expected.anglesDeg[count-1];
}
constexpr bool transferMotion(const Run &r, int offset, int waitOffset, int work, float height) {
    const auto &target=r.materialTransferLayout.workArea[work-1];
    const auto &lift=r.commands[offset+4], &retract=r.commands[offset+5];
    const auto &turn=r.commands[offset+6], &descend=r.commands[offset+7];
    return lift.high==r.materialTransferLayout.approachHeight && lift.length==-1
        && lift.turret==-1 && lift.claw==-1 && lift.speed==r.materialMotionSteps.pickLiftSpeed
        && lift.waitsBefore==waitOffset+5
        && retract.high==-1 && retract.length==0 && retract.turret==-1 && retract.claw==-1
        && retract.speed==r.materialMotionSteps.pickLiftSpeed && retract.waitsBefore==waitOffset+6
        && turn.high==-1 && turn.length==-1 && sameOrientation(turn.turret,target.turretAngle) && turn.claw==-1
        && turn.speed==r.materialMotionSteps.approachRotateSpeed && turn.waitsBefore==waitOffset+8
        && descend.high==height && descend.length==target.length && descend.turret==-1
        && descend.claw==r.materialTransferLayout.clawClosedAngle
        && descend.speed==r.materialMotionSteps.placeDescendSpeed && descend.waitsBefore==waitOffset+9
        && r.commands[offset+8].claw==r.materialTransferLayout.clawOpenAngle
        && r.commands[offset+8].waitsBefore==waitOffset+10
        && r.commands[offset+9].high==r.materialTransferLayout.approachHeight
        && r.commands[offset+9].length==-1 && r.commands[offset+9].waitsBefore==waitOffset+11;
}
constexpr bool nineRoutes(bool custom) {
    const uint32_t defaults[]={680,680,680,1020,680,1020,680,680,680,1360,680,1360};
    for (uint8_t c=1; c<=3; ++c) for (uint8_t w=1; w<=3; ++w) {
        Run r;
        r.cargoToWorkAreaDelays[c-1][w-1]={{680,680,680},1020,680,1020,{680,680,680},1360,680,1360};
        if (custom) r.cargoToWorkAreaDelays[c-1][w-1]=uniqueDelays(c*100+w*20);
        r.cargoPlatforms[c-1].material=MATERIAL_BLUE;
        if (!r.MoveCargoToWorkArea(c,w) || r.moves!=10 || r.waits!=12
                || r.cargoPlatforms[c-1].material!=MATERIAL_NONE) return false;
        if (custom) { if (!matches(r,0,c*100+w*20)) return false; }
        else for (int k=0;k<12;++k) if (r.delays[k]!=defaults[k]) return false;
        if (r.commands[2].high!=r.materialTransferLayout.cargo[c-1].high
                || r.commands[3].claw!=r.materialTransferLayout.clawClosedAngle
                || !transferMotion(r,0,0,w,r.materialTransferLayout.workArea[w-1].high)) return false;
        const float angles[]={r.materialTransferLayout.cargo[c-1].turretAngle,
                              r.materialTransferLayout.workArea[w-1].turretAngle};
        if (!plannedRoute(r,0,angles,2)) return false;
    }
    return true;
}
constexpr bool automaticPermutations() {
    constexpr int paths[][3]={{1,2,3},{1,3,2},{2,1,3},{2,3,1},{3,1,2},{3,2,1}};
    for (int p=0;p<6;++p) for (uint8_t layer=1; layer<=3; ++layer) {
        const int path[]={paths[p][0],paths[p][1],paths[p][2]};
        Run r;
        for (int c=0;c<3;++c) {
            r.cargoPlatforms[c].material=MATERIAL_RED;
            for (int w=0;w<3;++w) r.cargoToWorkAreaDelays[c][w]=uniqueDelays(100*c+20*w);
        }
        if (!r.PlaceTaskCargoToWorkArea(path,layer) || r.moves!=30 || r.waits!=36) return false;
        float angles[6]={};
        for (int c=0;c<3;++c) {
            angles[2*c]=r.materialTransferLayout.cargo[c].turretAngle;
            angles[2*c+1]=r.materialTransferLayout.workArea[path[c]-1].turretAngle;
            if (!matches(r,12*c,100*c+20*(path[c]-1))
                    || !transferMotion(r,10*c,12*c,path[c],r.materialTransferLayout.workArea[path[c]-1].high
                        +(layer-1)*r.materialTransferLayout.secondLayerOffset)
                    || r.cargoPlatforms[c].material!=MATERIAL_NONE) return false;
        }
        if (!plannedRoute(r,0,angles,6)) return false;
    }
    return true;
}
constexpr bool failuresAndZero() {
    for (int fail=1;fail<=10;++fail) {
        Run r; r.failAt=fail; r.cargoPlatforms[0].material=MATERIAL_RED;
        if (r.MoveCargoToWorkArea(1,2) || r.moves!=fail
                || r.cargoPlatforms[0].material!=MATERIAL_RED) return false;
    }
    Run r;
    if (r.MoveCargoToWorkArea(0,1) || r.MoveCargoToWorkArea(1,4)
            || r.MoveCargoToWorkArea(1,1) || r.moves) return false;
    r.cargoPlatforms[0].material=MATERIAL_RED;
    r.cargoToWorkAreaDelays[0][0]={};
    if (!r.MoveCargoToWorkArea(1,1) || r.waits || r.moves!=10) return false;
    r.waitForArm(-5);
    return r.waits==1 && r.delays[0]==r.materialTransferLayout.motionWaitMs;
}
constexpr bool reverseMotion(const Run &r, int offset, int waitOffset, int work, int cargo) {
    const auto &source=r.materialTransferLayout.workArea[work-1];
    const auto &target=r.materialTransferLayout.cargo[cargo-1];
    const auto &lift=r.commands[offset+4], &turn=r.commands[offset+5];
    const auto &descend=r.commands[offset+6];
    return r.commands[offset].length==source.length
        && sameOrientation(r.commands[offset+1].turret,source.turretAngle)
        && r.commands[offset+2].high==source.high
        && r.commands[offset+3].claw==r.materialTransferLayout.clawClosedAngle
        && lift.high==r.materialTransferLayout.approachHeight && lift.length==0
        && lift.turret==-1 && lift.claw==-1 && lift.speed==r.materialMotionSteps.pickLiftSpeed
        && lift.waitsBefore==waitOffset+5
        && turn.high==-1 && turn.length==-1 && sameOrientation(turn.turret,target.turretAngle) && turn.claw==-1
        && turn.speed==r.materialMotionSteps.approachRotateSpeed && turn.waitsBefore==waitOffset+8
        && descend.high==target.high && descend.length==target.length && descend.turret==-1
        && descend.claw==r.materialTransferLayout.clawClosedAngle
        && descend.speed==r.materialMotionSteps.placeDescendSpeed && descend.waitsBefore==waitOffset+9
        && r.commands[offset+7].claw==r.materialTransferLayout.clawOpenAngle
        && r.commands[offset+7].waitsBefore==waitOffset+10
        && r.commands[offset+8].high==r.materialTransferLayout.approachHeight
        && r.commands[offset+8].length==-1 && r.commands[offset+8].waitsBefore==waitOffset+11;
}
constexpr bool reverseRoutes() {
    for (uint8_t w=1;w<=3;++w) for (uint8_t c=1;c<=3;++c) {
        Run r; const int base=100+100*(w-1)+20*(c-1);
        reverseProfiles(r,100);
        r.commonMaterialDelays=uniqueDelays(800);
        r.cargoToWorkAreaDelays[c-1][w-1]=uniqueDelays(900);
        if (!r.MoveWorkAreaToCargo(MATERIAL_BLUE,w,c) || r.moves!=9 || r.waits!=12
                || !matches(r,0,base) || !reverseMotion(r,0,0,w,c)) return false;
        const float angles[]={r.materialTransferLayout.workArea[w-1].turretAngle,
                              r.materialTransferLayout.cargo[c-1].turretAngle};
        if (!plannedRoute(r,0,angles,2)) return false;
        for (int i=0;i<3;++i)
            if (r.cargoPlatforms[i].material!=(i==c-1 ? MATERIAL_BLUE : MATERIAL_NONE)) return false;
    }
    return true;
}
constexpr bool reverseBatches() {
    constexpr int paths[][3]={{1,2,3},{1,3,2},{2,1,3},{2,3,1},{3,1,2},{3,2,1}};
    constexpr int colors[]={1,2,3};
    for (int p=0;p<6;++p) {
        Run r; reverseProfiles(r,500); r.commonMaterialDelays=uniqueDelays(900);
        if (!r.RetrieveRoundToCargo(colors,paths[p]) || r.moves!=27 || r.waits!=36) return false;
        const float angles[]={r.materialTransferLayout.workArea[paths[p][0]-1].turretAngle,
            r.materialTransferLayout.cargo[0].turretAngle,r.materialTransferLayout.workArea[paths[p][1]-1].turretAngle,
            r.materialTransferLayout.cargo[1].turretAngle,r.materialTransferLayout.workArea[paths[p][2]-1].turretAngle,
            r.materialTransferLayout.cargo[2].turretAngle};
        if (!plannedRoute(r,0,angles,6)) return false;
        for (int c=0;c<3;++c)
            if (!matches(r,12*c,500+100*(paths[p][c]-1)+20*c) || !reverseMotion(r,9*c,12*c,paths[p][c],c+1)
                    || r.cargoPlatforms[c].material!=colors[c]) return false;
    }
    Run demo; reverseProfiles(demo,600); demo.commonMaterialDelays=uniqueDelays(900);
    if (!demo.DemoWorkAreaToCargo() || demo.moves!=27 || demo.waits!=36) return false;
    const float angles[]={48,300,86,271,125.3f,242};
    if (!plannedRoute(demo,0,angles,6)) return false;
    for (int w=0;w<3;++w)
        if (!matches(demo,12*w,600+100*w+20*(2-w)) || !reverseMotion(demo,9*w,12*w,w+1,3-w)
                || demo.cargoPlatforms[2-w].material!=MATERIAL_RED) return false;
    return true;
}
constexpr bool reverseFailuresAndZero() {
    for (int fail=1;fail<=9;++fail) {
        Run r; r.failAt=fail;
        if (r.MoveWorkAreaToCargo(MATERIAL_BLUE,2,3) || r.moves!=fail
                || r.cargoPlatforms[2].material!=MATERIAL_NONE) return false;
    }
    Run invalid;
    if (invalid.MoveWorkAreaToCargo(0,1,1) || invalid.MoveWorkAreaToCargo(1,0,1)
            || invalid.MoveWorkAreaToCargo(1,1,4) || invalid.moves) return false;
    invalid.cargoPlatforms[0].material=MATERIAL_RED;
    if (invalid.MoveWorkAreaToCargo(1,1,1) || invalid.moves
            || invalid.cargoPlatforms[0].material!=MATERIAL_RED) return false;
    Run zero; zero.workAreaToCargoDelays[1][2]={};
    return zero.MoveWorkAreaToCargo(MATERIAL_BLUE,2,3) && zero.moves==9 && zero.waits==0
        && zero.cargoPlatforms[2].material==MATERIAL_BLUE;
}
constexpr bool commonAndAuxiliary() {
    Run r; r.commonMaterialDelays=uniqueDelays(500);
    if (!r.MoveDiscToCargo(1,3) || !matches(r,0,500) || r.moves!=10
            || r.commands[4].length!=-1 || r.commands[5].length!=r.materialTransferLayout.cargo[2].length
            || r.commands[7].length!=-1) return false;
    r.waits=0; r.moves=0;
    r.materialAuxiliaryDelays={101,102,103,201,202,301,302};
    if (!r.GrabDiscMaterial(2,1,false) || r.waits!=9 || r.moves!=8
            || r.commands[3].length!=r.materialTransferLayout.cargo[0].length) return false;
    if (r.delays[0]!=101 || r.delays[1]!=102 || r.delays[2]!=103
            || r.delays[3]!=507 || r.delays[8]!=512) return false;
    r.waits=0;
    if (!r.PrepareLidarScanPose(1) || r.waits!=2
            || r.delays[0]!=202 || r.delays[1]!=201) return false;
    r.waits=0;
    return r.PrepareLidarScanPose(2) && r.waits==2
        && r.delays[0]==301 && r.delays[1]==301 && r.feedbackReads==0;
}
constexpr bool positionPlacement() {
    Run r; r.commonMaterialDelays=uniqueDelays(700);
    if (!r.ValidateDiscPositionTransfer(3,2)
            || !r.PlaceHeldDiscMaterialToCargo(3,2)
            || r.moves!=5 || r.waits!=6
            || r.feedbackReads!=0
            || r.cargoPlatforms[1].material!=MATERIAL_BLUE) return false;
    if (r.commands[0].high!=r.materialTransferLayout.approachHeight
            || r.commands[0].length!=r.materialTransferLayout.cargo[1].length
            || r.commands[1].turret!=r.materialTransferLayout.cargo[1].turretAngle
            || r.commands[2].high!=r.materialTransferLayout.cargo[1].high
            || r.commands[3].claw!=r.materialTransferLayout.clawOpenAngle
            || r.commands[4].high!=r.materialTransferLayout.approachHeight) return false;
    if (r.ValidateDiscPositionTransfer(1,2)) return false;
    Run failed; failed.commonMaterialDelays=uniqueDelays(800); failed.failAt=3;
    return !failed.PlaceHeldDiscMaterialToCargo(5,1)
        && failed.cargoPlatforms[0].material==MATERIAL_NONE;
}
constexpr bool stackEntrypoints() {
    for (uint8_t c=1;c<=3;++c) for (uint8_t w=1;w<=3;++w) {
        Run r; r.cargoPlatforms[c-1].material=MATERIAL_RED;
        r.cargoToWorkAreaDelays[c-1][w-1]=uniqueDelays(100*c+20*w);
        if (!r.StackCargoToWorkArea(c,w) || r.moves!=10 || !matches(r,0,100*c+20*w)
                || !transferMotion(r,0,0,w,r.materialTransferLayout.workArea[w-1].high
                    +r.materialTransferLayout.secondLayerOffset)) return false;
        const float angles[]={r.materialTransferLayout.cargo[c-1].turretAngle,
                              r.materialTransferLayout.workArea[w-1].turretAngle};
        if (!plannedRoute(r,0,angles,2)) return false;
    }
    for (int layer=2;layer<=3;++layer) {
        Run r;
        for (int c=0;c<3;++c) r.cargoToWorkAreaDelays[c][c]=uniqueDelays(100*c);
        if (!(layer==2 ? r.DemoStackCargoToWorkArea() : r.DemoStackCargoToWorkArea3())
                || r.moves!=30 || r.waits!=36) return false;
        const float angles[]={242,48,271,86,300,125.3f};
        if (!plannedRoute(r,0,angles,6)) return false;
        for (int c=0;c<3;++c) if (!matches(r,12*c,100*c)
                || !transferMotion(r,10*c,12*c,c+1,r.materialTransferLayout.workArea[c].high
                    +(layer-1)*r.materialTransferLayout.secondLayerOffset)) return false;
    }
    return true;
}
constexpr bool feedbackPlanning() {
    Run demo;
    const float batch[]={242,48,271,86,300,125.3f};
    if (!demo.DemoCargoToRoughArea() || demo.moves!=30 || !plannedRoute(demo,0,batch,6)) return false;
    Run r; r.measuredTurret=123.3f;
    r.materialTransferLayout.workArea[0].turretAngle=48.7f;
    r.cargoPlatforms[2].material=MATERIAL_RED;
    if (!r.MoveCargoToWorkArea(3,1) || r.commands[1].turret!=-60
            || r.commands[6].turret!=48.7f) return false;
    r.measuredTurret=350; // 模拟机构实际位置改变，下一次必须重读，不能沿用上次规划。
    r.cargoPlatforms[2].material=MATERIAL_BLUE;
    if (!r.MoveCargoToWorkArea(3,1) || r.feedbackReads!=2 || r.commands[11].turret!=300) return false;
    Run minusOne;
    minusOne.materialTransferLayout.cargo[0].turretAngle=359;
    minusOne.materialTransferLayout.workArea[0].turretAngle=0;
    minusOne.cargoPlatforms[0].material=MATERIAL_RED;
    return minusOne.MoveCargoToWorkArea(1,1) && minusOne.commands[1].planned
        && minusOne.commands[1].turret==-1;
}
constexpr bool planningFailures() {
    constexpr int codes[]={1,2,3};
    for (int mode=0;mode<8;++mode) {
        Run r; r.feedbackOk=false;
        const bool retrieve=mode==1 || mode==3 || mode==5;
        for (auto &cargo:r.cargoPlatforms) cargo.material=retrieve ? MATERIAL_NONE : MATERIAL_BLUE;
        const bool result=mode==0 ? r.MoveCargoToWorkArea(1,1)
            : mode==1 ? r.MoveWorkAreaToCargo(1,1,1)
            : mode==2 ? r.PlaceTaskCargoToWorkArea(codes,1)
            : mode==3 ? r.RetrieveRoundToCargo(codes,codes)
            : mode==4 ? r.DemoCargoToRoughArea() : mode==5 ? r.DemoWorkAreaToCargo()
            : mode==6 ? r.DemoStackCargoToWorkArea() : r.DemoStackCargoToWorkArea3();
        if (result || r.moves || r.waits || r.feedbackReads!=1) return false;
        for (auto cargo:r.cargoPlatforms)
            if (cargo.material!=(retrieve ? MATERIAL_NONE : MATERIAL_BLUE)) return false;
    }
    for (float angle:{-151.0f,361.0f,__builtin_nanf("")}) {
        Run r; r.measuredTurret=angle; r.cargoPlatforms[0].material=MATERIAL_RED;
        if (r.MoveCargoToWorkArea(1,1) || r.moves || r.waits
                || r.cargoPlatforms[0].material!=MATERIAL_RED) return false;
    }
    Run invalid;
    if (invalid.RetrieveRoundToCargo(codes,nullptr) || invalid.moves || invalid.feedbackReads) return false;
    invalid.materialTransferLayout.workArea[2].turretAngle=__builtin_nanf("");
    return !invalid.RetrieveRoundToCargo(codes,codes) && invalid.moves==0 && invalid.feedbackReads==0;
}
static_assert(feedbackPlanning(), "batch planning, greedy counterexample, reread actual angle and minus one");
static_assert(planningFailures(), "all entrypoints stop before motion and preserve cargo on planning failure");
static_assert(nineRoutes(false), "all 9 routes lift/wait/retract/turn/extend-descend in order");
static_assert(nineRoutes(true), "all 9 routes independently select their own 12 delays");
static_assert(automaticPermutations(), "all 6 task permutations and 3 layers use matching profiles");
static_assert(failuresAndZero(), "failure stops subsequent moves; zero skips waits");
static_assert(reverseRoutes(), "9 independent reverse profiles; lift-retract, turn and extend-descend order");
static_assert(reverseBatches(), "6 retrieval permutations and reverse cargo-order demo select matching profiles");
static_assert(reverseFailuresAndZero(), "reverse failure stops, occupied cargo stays unchanged, zero skips waits");
static_assert(commonAndAuxiliary(), "legacy disc, color grab and lidar keep their common/auxiliary sequences");
static_assert(positionPlacement(), "position pickup uses common cargo placement and updates only on success");
static_assert(stackEntrypoints(), "single stack and both stack demos follow the new motion order");
'''.replace("PLANNER", planner).replace("HEADER", header).replace("SOURCE", source)
        self.compile_harness("#include <initializer_list>\n" + harness)

    def test_delay_registry(self):
        header = re.sub(r"^#include.*\n", "", (ROOT / "src/material_transfer.h").read_text(encoding="utf-8"), flags=re.M)
        runtime = (ROOT / "src/runtime_parameters.cpp").read_text(encoding="utf-8")
        helpers = runtime[runtime.index("void registerTransferDelays("):runtime.index("void initialize()")]
        helpers = helpers.replace("void register", "constexpr void register")
        helpers = re.sub(r"char (name|prefix)\[80\];", r"char \1[80] = {};", helpers)
        tail = runtime[runtime.index("    registerMaterialDelays();"):
                       runtime.index("\n}\n\nbool parseUnsigned(")]
        capacity = int(re.search(r"MAX_PARAMETERS = (\d+)", runtime).group(1))
        defaults = (ROOT / "src/generated_parameter_defaults.h").read_text(encoding="utf-8")
        original_count = int(re.search(r"COMPILED_PARAMETER_DEFAULT_COUNT = (\d+)", defaults).group(1))
        self.assertGreaterEqual(capacity, original_count + 235)
        harness = r'''
#include <stdint.h>
#include <stddef.h>
HEADER
constexpr bool same(const char *a,const char *b) {
    while (*a && *a==*b) { ++a; ++b; } return *a==*b;
}
constexpr int append(char *out,int pos,const char *text) {
    while (*text) out[pos++]=*text++; out[pos]=0; return pos;
}
constexpr int snprintf(char *out,size_t,const char*,const char *prefix,const char *step) {
    int pos=append(out,0,prefix); pos=append(out,pos,"/");
    pos=append(out,pos,step); return append(out,pos,"(ms)");
}
constexpr int snprintf(char *out,size_t,const char *fmt,int c,int w) {
    int pos=0; bool first=true;
    while (*fmt) {
        if (fmt[0]=='%' && fmt[1]=='u') {
            out[pos++]=char('0'+(first?c:w)); first=false; fmt+=2;
        } else out[pos++]=*fmt++;
    }
    out[pos]=0; return pos;
}
struct Entry { char name[80]={}; uint32_t *address=nullptr; };
struct Run {
    MaterialTransferDelays commonMaterialDelays={}, cargoToWorkAreaDelays[3][3]={};
    MaterialTransferDelays workAreaToCargoDelays[3][3]={};
    MaterialAuxiliaryDelays materialAuxiliaryDelays={};
    float NODE_PATH_ACCEL_MM_S2=0, NODE_PATH_TURN_DEG_S=0;
    float NODE_PATH_TURN_ACCEL_DEG_S2=0, NODE_PATH_HEADING_DEADBAND_DEG=0;
    Entry entries[239]={}; int count=0; bool valid=true;
    constexpr void RegisterRuntimeUInt(const char *name,uint32_t &value,float low,float high) {
        valid=valid && low==0 && high==60000;
        append(entries[count].name,0,name); entries[count++].address=&value;
    }
    constexpr void RegisterRuntimeFloat(const char *name,float&,float,float) {
        append(entries[count++].name,0,name);
    }
    HELPERS
    constexpr void registrationTail() { TAIL }
};
constexpr bool registry() {
    Run r; r.registerMaterialDelays();
    if (!r.valid || r.count!=127 || !same(r.entries[0].name,"动作组延时/公共/取料平移(ms)")) return false;
    r.registerReverseMaterialDelays();
    if (!r.valid || r.count!=235) return false;
    for (int i=0;i<235;++i) for (int j=i+1;j<235;++j)
        if (same(r.entries[i].name,r.entries[j].name) || r.entries[i].address==r.entries[j].address) return false;
    for (int c=0;c<3;++c) for (int w=0;w<3;++w) {
        int first=12+12*(3*c+w);
        char prefix[80]={}, name[80]={};
        snprintf(prefix,80,"动作组延时/载物台%u到工作区%u",c+1,w+1);
        snprintf(name,80,"%s/%s(ms)",prefix,"放料松爪");
        if (!same(r.entries[first+10].name,name)
                || r.entries[first].address!=&r.cargoToWorkAreaDelays[c][w].pickAbove.translateMs
                || r.entries[first+10].address!=&r.cargoToWorkAreaDelays[c][w].placeReleaseMs
                || r.entries[first+11].address!=&r.cargoToWorkAreaDelays[c][w].placeLiftMs) return false;
    }
    for (int w=0;w<3;++w) for (int c=0;c<3;++c) {
        const int first=127+12*(3*w+c);
        const auto &d=r.workAreaToCargoDelays[w][c];
        const uint32_t *fields[]={&d.pickAbove.translateMs,&d.pickAbove.settleMs,&d.pickAbove.rotateMs,
            &d.pickDescendMs,&d.pickGripMs,&d.pickLiftMs,&d.placeAbove.translateMs,&d.placeAbove.settleMs,
            &d.placeAbove.rotateMs,&d.placeDescendMs,&d.placeReleaseMs,&d.placeLiftMs};
        const char *steps[]={"取料平移","取料稳定","取料转向","取料下降","取料夹紧","取料抬升",
            "放料平移","放料稳定","放料转向","放料下降","放料松爪","放料抬升"};
        char prefix[80]={}, name[80]={};
        snprintf(prefix,80,"动作组延时/工作区%u到载物台%u",w+1,c+1);
        for (int k=0;k<12;++k) {
            snprintf(name,80,"%s/%s(ms)",prefix,steps[k]);
            if (!same(r.entries[first+k].name,name) || r.entries[first+k].address!=fields[k]) return false;
        }
    }
    return r.entries[120].address==&r.materialAuxiliaryDelays.discApproachMs
        && r.entries[126].address==&r.materialAuxiliaryDelays.lidarZone2TurnMs;
}
constexpr bool appendOrder() {
    Run r; r.registrationTail();
    return r.count==239 && same(r.entries[127].name,"底盘/节点导航平移加速度(mm/s²)")
        && same(r.entries[130].name,"底盘/节点导航独立纠偏死区(°)")
        && same(r.entries[131].name,"动作组延时/工作区1到载物台1/取料平移(ms)")
        && r.entries[238].address==&r.workAreaToCargoDelays[2][2].placeLiftMs;
}
static_assert(registry(), "235 unique delay names/addresses, correct reverse bindings and 0..60000 bounds");
static_assert(appendOrder(), "append reverse delays after all legacy delay and chassis parameters");
'''.replace("HEADER", header).replace("HELPERS", helpers).replace("TAIL", tail)
        self.compile_harness(harness)

    def test_single_transfer_command(self):
        serial = (ROOT / "src/serial_commands.inc").read_text(encoding="utf-8")
        helper = serial[serial.index("static void serialTransfer("):serial.index("static void serialDispatch(")]
        helper = helper.replace("static void serial", "constexpr void serial")
        frame = (ROOT / "src/serial_frame.h").read_text(encoding="utf-8")
        parse = frame[frame.index("inline bool serialUInt("):frame.index("inline void serialError(")]
        parse = parse.replace("inline bool", "constexpr bool")
        self.assertIn('frame.is("ARM", "TRANSFER", 2)', serial)
        self.assertIn('{"ARM", "TRANSFER"}', serial)
        self.assertIn('frame.is("ARM", "RETRIEVE", 2)', serial)
        self.assertIn('{"ARM", "RETRIEVE"}', serial)
        harness = r'''
#include <stdint.h>
constexpr int MATERIAL_STATION_COUNT=3;
enum MaterialType { MATERIAL_NONE, MATERIAL_RED, MATERIAL_BLUE };
struct CargoPlatform { MaterialType material; };
struct SerialFrame { const char *fields[5]; };
PARSE
struct Log {
    const char *status="";
    unsigned long first=0, second=0;
    constexpr void printf(const char*,const char *s,unsigned long a,unsigned long b) {
        status=s; first=a; second=b;
    }
};
struct Run {
    bool serialDebugMode=true, success=true;
    int calls=0, acks=0; uint32_t cargo=0, work=0;
    int reverseCalls=0; uint32_t material=0;
    const char *error=""; Log Serial;
    CargoPlatform cargoPlatforms[3]={{MATERIAL_NONE},{MATERIAL_NONE},{MATERIAL_NONE}};
    constexpr void serialError(const SerialFrame&,const char *text) { error=text; }
    constexpr void serialReply(const SerialFrame&,const char*) { ++acks; }
    constexpr bool MoveCargoToWorkArea(uint32_t c,uint32_t w) {
        ++calls; cargo=c; work=w;
        if (success) cargoPlatforms[c-1].material=MATERIAL_NONE;
        return success;
    }
    constexpr bool MoveWorkAreaToCargo(uint32_t m,uint32_t w,uint32_t c) {
        ++reverseCalls; material=m; work=w; cargo=c;
        if (success) cargoPlatforms[c-1].material=static_cast<MaterialType>(m);
        return success;
    }
    HELPER
};
constexpr bool commands() {
    constexpr const char *codes[]={"1","2","3"};
    for (int c=0;c<3;++c) for (int w=0;w<3;++w) {
        Run r; SerialFrame f={{"CMD","ARM","TRANSFER",codes[c],codes[w]}};
        r.serialTransfer(f);
        if (r.calls!=1 || r.acks!=1 || r.cargo!=c+1u || r.work!=w+1u
                || r.Serial.status[0]!='D' || r.cargoPlatforms[c].material!=MATERIAL_NONE) return false;
    }
    constexpr const char *bad[]={"0","4","-1","1.0","nan","2x",""};
    for (auto text:bad) for (int index=3;index<=4;++index) {
        Run r; SerialFrame f={{"CMD","ARM","TRANSFER","1","1"}}; f.fields[index]=text;
        r.serialTransfer(f);
        if (r.calls || r.acks || r.error[0]!='R') return false;
    }
    SerialFrame f={{"CMD","ARM","TRANSFER","2","3"}};
    Run release; release.serialDebugMode=false; release.serialTransfer(f);
    if (release.calls || release.acks || release.error[0]!='M') return false;
    for (auto original:{MATERIAL_NONE,MATERIAL_BLUE}) {
        Run r; r.success=false; r.cargoPlatforms[1].material=original; r.serialTransfer(f);
        if (r.calls!=1 || r.Serial.status[0]!='F' || r.cargoPlatforms[1].material!=original) return false;
    }
    return true;
}
static_assert(commands(), "9 manual routes, invalid input, Debug gate and failure record restoration");
constexpr bool reverseCommands() {
    constexpr const char *codes[]={"1","2","3"};
    for (int w=0;w<3;++w) for (int c=0;c<3;++c) {
        Run r; SerialFrame f={{"CMD","ARM","RETRIEVE",codes[w],codes[c]}};
        r.cargoPlatforms[(c+1)%3].material=MATERIAL_BLUE;
        r.serialRetrieve(f);
        if (r.calls || r.reverseCalls!=1 || r.acks!=1 || r.material!=MATERIAL_RED
                || r.work!=w+1u || r.cargo!=c+1u || r.Serial.status[0]!='D'
                || r.Serial.first!=w+1u || r.Serial.second!=c+1u) return false;
        for (int i=0;i<3;++i) if (r.cargoPlatforms[i].material
                !=(i==c ? MATERIAL_RED : i==(c+1)%3 ? MATERIAL_BLUE : MATERIAL_NONE)) return false;
        r.serialRetrieve(f); // 成功后的重复命令必须被占用检查拦截。
        if (r.reverseCalls!=1 || r.acks!=1 || r.error[0]!='O') return false;
    }
    constexpr const char *bad[]={"0","4","-1","1.0","nan","2x","","4294967296"};
    for (auto text:bad) for (int index=3;index<=4;++index) {
        Run r; SerialFrame f={{"CMD","ARM","RETRIEVE","1","1"}}; f.fields[index]=text;
        r.serialRetrieve(f);
        if (r.calls || r.reverseCalls || r.acks || r.error[0]!='R') return false;
    }
    SerialFrame f={{"CMD","ARM","RETRIEVE","2","3"}};
    Run release; release.serialDebugMode=false; release.serialRetrieve(f);
    if (release.reverseCalls || release.acks || release.error[0]!='M') return false;
    Run failed; failed.success=false; failed.serialRetrieve(f);
    if (failed.reverseCalls!=1 || failed.acks!=1 || failed.Serial.status[0]!='F'
            || failed.Serial.first!=2 || failed.Serial.second!=3
            || failed.cargoPlatforms[2].material!=MATERIAL_NONE) return false;
    for (auto original:{MATERIAL_RED,MATERIAL_BLUE}) {
        Run occupied; occupied.cargoPlatforms[2].material=original; occupied.serialRetrieve(f);
        if (occupied.calls || occupied.reverseCalls || occupied.acks || occupied.error[0]!='O'
                || occupied.cargoPlatforms[2].material!=original) return false;
    }
    return true;
}
static_assert(reverseCommands(), "9 reverse routes, argument order, invalid input, Debug gate, occupancy and failure");
'''.replace("PARSE", parse).replace("HELPER", helper)
        self.compile_harness("#include <initializer_list>\n" + harness)

    def compile_harness(self, harness):
        compiler = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio")) / (
            "packages/toolchain-xtensa-esp32s3/bin/xtensa-esp32s3-elf-g++.exe")
        self.assertTrue(compiler.is_file(), "需要已安装的 ESP32-S3 编译器")
        with tempfile.TemporaryDirectory() as folder:
            fixture = Path(folder) / "material.cpp"
            fixture.write_text(harness, encoding="utf-8")
            result = subprocess.run([str(compiler), "-std=c++14", "-fsyntax-only", str(fixture)],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
