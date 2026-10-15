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
        harness = r'''
#include <stdint.h>
constexpr bool isfinite(float v) { return __builtin_isfinite(v); }
constexpr long lroundf(float v) { return v < 0 ? long(v - 0.5f) : long(v + 0.5f); }
constexpr int pdMS_TO_TICKS(int v) { return v; }
constexpr float ARM_HEIGHT_LIMIT_MM=175, TURRET_CABLE_MIN_DEG=-150, TURRET_CABLE_MAX_DEG=360;
HEADER
struct Log {
    constexpr void println(const char*) {}
    template<class... A> constexpr void printf(const char*, A...) {}
};
struct Command { float high, length, turret, claw, speed; int waitsBefore; };
struct Run {
    Log Serial;
    int moves=0, waits=0, failAt=0;
    Command commands[64] = {};
    uint32_t delays[96] = {};
    constexpr bool MoveArm(float h, float l, float t, float c, float s) {
        commands[moves++]={h,l,t,c,s,waits}; return moves != failAt;
    }
    constexpr void GotoPose(float, float, float, bool) {}
    constexpr void vTaskDelay(int ms) { delays[waits++]=ms; }
    SOURCE
};
constexpr MaterialTransferDelays uniqueDelays(int base) {
    return {{base+1u,base+2u,base+3u},base+4u,base+5u,base+6u,
            {base+7u,base+8u,base+9u},base+10u,base+11u,base+12u};
}
constexpr bool matches(const Run &r, int offset, int base) {
    for (int k=0; k<12; ++k) if (r.delays[offset+k] != base+k+1u) return false;
    return true;
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
        && turn.high==-1 && turn.length==-1 && turn.turret==target.turretAngle && turn.claw==-1
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
        for (int c=0;c<3;++c) {
            if (!matches(r,12*c,100*c+20*(path[c]-1))
                    || !transferMotion(r,10*c,12*c,path[c],r.materialTransferLayout.workArea[path[c]-1].high
                        +(layer-1)*r.materialTransferLayout.secondLayerOffset)
                    || r.cargoPlatforms[c].material!=MATERIAL_NONE) return false;
        }
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
constexpr bool commonAndAuxiliary() {
    Run r; r.commonMaterialDelays=uniqueDelays(500);
    if (!r.MoveWorkAreaToCargo(1,2,3) || !matches(r,0,500) || r.moves!=10
            || r.commands[4].length!=-1 || r.commands[5].length!=r.materialTransferLayout.cargo[2].length
            || r.commands[7].length!=-1) return false;
    r.waits=0; r.moves=0;
    r.materialAuxiliaryDelays={101,102,103,201,202,301,302};
    if (!r.GrabDiscMaterial(2,1,false) || r.waits!=9 || r.moves!=8
            || r.commands[3].length!=r.materialTransferLayout.cargo[0].length) return false;
    if (r.delays[0]!=101 || r.delays[1]!=102 || r.delays[2]!=103
            || r.delays[3]!=507 || r.delays[8]!=512) return false;
    r.waits=0;
    if (!r.PrepareLidarScanPose(1) || r.delays[0]!=201 || r.delays[1]!=202) return false;
    r.waits=0;
    return r.PrepareLidarScanPose(2) && r.delays[0]==301 && r.delays[1]==302;
}
constexpr bool stackEntrypoints() {
    for (uint8_t c=1;c<=3;++c) for (uint8_t w=1;w<=3;++w) {
        Run r; r.cargoPlatforms[c-1].material=MATERIAL_RED;
        r.cargoToWorkAreaDelays[c-1][w-1]=uniqueDelays(100*c+20*w);
        if (!r.StackCargoToWorkArea(c,w) || r.moves!=10 || !matches(r,0,100*c+20*w)
                || !transferMotion(r,0,0,w,r.materialTransferLayout.workArea[w-1].high
                    +r.materialTransferLayout.secondLayerOffset)) return false;
    }
    for (int layer=2;layer<=3;++layer) {
        Run r;
        for (int c=0;c<3;++c) r.cargoToWorkAreaDelays[c][c]=uniqueDelays(100*c);
        if (!(layer==2 ? r.DemoStackCargoToWorkArea() : r.DemoStackCargoToWorkArea3())
                || r.moves!=30 || r.waits!=36) return false;
        for (int c=0;c<3;++c) if (!matches(r,12*c,100*c)
                || !transferMotion(r,10*c,12*c,c+1,r.materialTransferLayout.workArea[c].high
                    +(layer-1)*r.materialTransferLayout.secondLayerOffset)) return false;
    }
    return true;
}
static_assert(nineRoutes(false), "all 9 routes lift/wait/retract/turn/extend-descend in order");
static_assert(nineRoutes(true), "all 9 routes independently select their own 12 delays");
static_assert(automaticPermutations(), "all 6 task permutations and 3 layers use matching profiles");
static_assert(failuresAndZero(), "failure stops subsequent moves; zero skips waits");
static_assert(commonAndAuxiliary(), "reverse, disc and lidar use their own common/auxiliary delays");
static_assert(stackEntrypoints(), "single stack and both stack demos follow the new motion order");
'''.replace("HEADER", header).replace("SOURCE", source)
        self.compile_harness(harness)

    def test_delay_registry(self):
        header = re.sub(r"^#include.*\n", "", (ROOT / "src/material_transfer.h").read_text(encoding="utf-8"), flags=re.M)
        runtime = (ROOT / "src/runtime_parameters.cpp").read_text(encoding="utf-8")
        helpers = runtime[runtime.index("void registerTransferDelays("):runtime.index("void initialize()")]
        helpers = helpers.replace("void register", "constexpr void register")
        helpers = re.sub(r"char (name|prefix)\[80\];", r"char \1[80] = {};", helpers)
        capacity = int(re.search(r"MAX_PARAMETERS = (\d+)", runtime).group(1))
        defaults = (ROOT / "src/generated_parameter_defaults.h").read_text(encoding="utf-8")
        original_count = int(re.search(r"COMPILED_PARAMETER_DEFAULT_COUNT = (\d+)", defaults).group(1))
        self.assertGreaterEqual(capacity, original_count + 127)
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
    MaterialAuxiliaryDelays materialAuxiliaryDelays={};
    Entry entries[127]={}; int count=0; bool valid=true;
    constexpr void RegisterRuntimeUInt(const char *name,uint32_t &value,float low,float high) {
        valid=valid && low==0 && high==60000;
        append(entries[count].name,0,name); entries[count++].address=&value;
    }
    HELPERS
};
constexpr bool registry() {
    Run r; r.registerMaterialDelays();
    if (!r.valid || r.count!=127 || !same(r.entries[0].name,"动作组延时/公共/取料平移(ms)")) return false;
    for (int i=0;i<127;++i) for (int j=i+1;j<127;++j)
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
    return r.entries[120].address==&r.materialAuxiliaryDelays.discApproachMs
        && r.entries[126].address==&r.materialAuxiliaryDelays.lidarZone2TurnMs;
}
static_assert(registry(), "127 unique names/addresses with 0..60000 ms bounds and correct route bindings");
'''.replace("HEADER", header).replace("HELPERS", helpers)
        self.compile_harness(harness)

    def test_single_transfer_command(self):
        serial = (ROOT / "src/serial_commands.inc").read_text(encoding="utf-8")
        helper = serial[serial.index("static void serialTransfer("):serial.index("static void serialDispatch(")]
        helper = helper.replace("static void serialTransfer", "constexpr void serialTransfer")
        frame = (ROOT / "src/serial_frame.h").read_text(encoding="utf-8")
        parse = frame[frame.index("inline bool serialUInt("):frame.index("inline void serialError(")]
        parse = parse.replace("inline bool", "constexpr bool")
        self.assertIn('frame.is("ARM", "TRANSFER", 2)', serial)
        self.assertIn('{"ARM", "TRANSFER"}', serial)
        harness = r'''
#include <stdint.h>
constexpr int MATERIAL_STATION_COUNT=3;
enum MaterialType { MATERIAL_NONE, MATERIAL_RED, MATERIAL_BLUE };
struct CargoPlatform { MaterialType material; };
struct SerialFrame { const char *fields[5]; };
PARSE
struct Log {
    const char *status="";
    constexpr void printf(const char*,const char *s,unsigned long,unsigned long) { status=s; }
};
struct Run {
    bool serialDebugMode=true, success=true;
    int calls=0, acks=0; uint32_t cargo=0, work=0;
    const char *error=""; Log Serial;
    CargoPlatform cargoPlatforms[3]={{MATERIAL_NONE},{MATERIAL_NONE},{MATERIAL_NONE}};
    constexpr void serialError(const SerialFrame&,const char *text) { error=text; }
    constexpr void serialReply(const SerialFrame&,const char*) { ++acks; }
    constexpr bool MoveCargoToWorkArea(uint32_t c,uint32_t w) {
        ++calls; cargo=c; work=w;
        if (success) cargoPlatforms[c-1].material=MATERIAL_NONE;
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
