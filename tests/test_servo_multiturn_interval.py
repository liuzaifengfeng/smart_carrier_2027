"""编译期执行实际转台控制及串口封包函数，不连接硬件。"""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


def function_source(source, signature):
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


class ServoMultiturnIntervalTests(unittest.TestCase):
    def compile_assertions(self, harness):
        compiler = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio")) / (
            "packages/toolchain-xtensa-esp32s3/bin/xtensa-esp32s3-elf-g++.exe")
        self.assertTrue(compiler.is_file(), "需要已安装的 ESP32-S3 编译器")
        with tempfile.TemporaryDirectory() as directory:
            fixture = Path(directory) / "servo.cpp"
            fixture.write_text(harness, encoding="utf-8")
            result = subprocess.run([str(compiler), "-std=c++14", "-fsyntax-only", str(fixture)],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_actual_packet_bytes(self):
        source = (ROOT / "src/servo.cpp").read_text(encoding="utf-8")
        header = (ROOT / "src/servo.h").read_text(encoding="utf-8")
        names = ("FSUS_PACK_REQ_HEADER", "FSUS_CMD_SET_ANGLE_MTURN",
                 "FSUS_CMD_SET_ANGLE_MTURN_INTERVAL", "FSUS_CMD_SET_ORIGIN", "FSUS_CMD_STOP")
        constants = "\n".join(re.search(r"(?m)^#define " + name + r"\s+0x[0-9A-Fa-f]+",
                                         header).group() for name in names)
        actual = "\n".join(function_source(source, signature) for signature in (
            "static void sendPacket(", "void Servo_SetAngleMTurn(",
            "void Servo_SetAngleMTurnByInterval(", "bool Servo_SetOrigin("))
        actual = actual.replace("static void sendPacket(", "constexpr void sendPacket(")
        actual = actual.replace("void Servo_SetAngle", "constexpr void Servo_SetAngle")
        actual = actual.replace("bool Servo_SetOrigin", "constexpr bool Servo_SetOrigin")
        # C++14 constexpr 要求局部数组初始化；运行时原函数会逐字节写满数组。
        actual = re.sub(r"uint8_t content\[(\d+)\];", r"uint8_t content[\1] = {};", actual)
        harness = r'''
#include <stdint.h>
CONSTANTS
constexpr uint32_t pdMS_TO_TICKS(uint32_t value){return value;}
constexpr void vTaskDelay(uint32_t){}
struct ServoBusGuard { bool locked=true; };
struct Port {
    uint8_t bytes[32]={};
    int count=0;
    constexpr void flush() {}
    constexpr int available() const { return 0; }
    constexpr int read() { return -1; }
    constexpr void write(uint8_t b) { bytes[count++]=b; }
    constexpr void write(const uint8_t* data, uint8_t n) {
        for (int i=0; i<n; ++i) write(data[i]);
    }
};
struct Run {
    Port port;
    Port* s_servoSerial=&port;
    ACTUAL
};
template<unsigned N> constexpr bool matches(const Port& p, const uint8_t (&expected)[N]) {
    if (p.count!=N) return false;
    for (unsigned i=0; i<N; ++i) if (p.bytes[i]!=expected[i]) return false;
    return true;
}
constexpr bool officialExample() {
    Run r;
    r.Servo_SetAngleMTurnByInterval(0, 600, 1200, 100, 100, 0);
    // 官方协议第11节的完整示例帧，含校验码：
    // https://fashionstar.com.cn/wiki/protocols/servo/uart-rs485/
    const uint8_t expected[]={0x12,0x4C,0x0E,0x0F,0,0x70,0x17,0,0,
        0xB0,4,0,0,0x64,0,0x64,0,0,0,0x7E};
    return matches(r.port, expected);
}
constexpr bool requestedExample() {
    Run r;
    r.Servo_SetAngleMTurnByInterval(2, 180, 1050, 100, 300, 0);
    const uint8_t expected[]={0x12,0x4C,0x0E,0x0F,2,8,7,0,0,
        0x1A,4,0,0,0x64,0,0x2C,1,0,0,0x3B};
    return matches(r.port, expected);
}
constexpr bool signedAngleAndWideTime() {
    Run r;
    r.Servo_SetAngleMTurnByInterval(2, -9000, 0x12345678u, 100, 300, 0x1234);
    const uint8_t expected[]={0x12,0x4C,0x0E,0x0F,2,0x70,0xA0,0xFE,0xFF,
        0x78,0x56,0x34,0x12,0x64,0,0x2C,1,0x34,0x12,0x75};
    return matches(r.port, expected);
}
constexpr bool legacyClawPacket() {
    Run r;
    r.Servo_SetAngleMTurn(1, 50, 500, 0);
    const uint8_t expected[]={0x12,0x4C,0x0D,0x0B,1,0xF4,1,0,0,0xF4,1,0,0,0,0,0x61};
    return matches(r.port, expected);
}
constexpr bool originPacket() {
    Run r;if(!r.Servo_SetOrigin(2))return false;
    const uint8_t expected[]={
        0x12,0x4C,0x18,0x04,0x02,0x10,0x00,0x00,0x8C,
        0x12,0x4C,0x17,0x02,0x02,0x00,0x79,
        0x12,0x4C,0x18,0x04,0x02,0x11,0x00,0x00,0x8D};
    return matches(r.port,expected);
}
static_assert(originPacket(), "set origin ID2, reset=0 and checksum");
static_assert(officialExample(), "official 0x0E example, length and checksum");
static_assert(requestedExample(), "ID 2, 180 degrees, 1050/100/300 ms");
static_assert(signedAngleAndWideTime(), "signed 32-bit angle and little-endian field widths");
static_assert(legacyClawPacket(), "ID 1 retains simple multiturn packet");
'''.replace("CONSTANTS", constants).replace("ACTUAL", actual)
        self.compile_assertions(harness)

    def test_actual_move_arm_timing_and_guards(self):
        source = (ROOT / "src/chassis.cpp").read_text(encoding="utf-8")
        header = (ROOT / "src/chassis.h").read_text(encoding="utf-8")
        names = ("ARM_HEIGHT_LIMIT_MM", "TURRET_CABLE_MIN_DEG", "TURRET_CABLE_MAX_DEG")
        limits = "\n".join(re.search(r"constexpr float " + name + r"\s*=[^;]+;",
                                      header).group() for name in names)
        actual = "\n".join(function_source(source, signature) for signature in (
            "bool ReadArmTurretAngle(", "static bool MoveArmInternal(",
            "bool MoveArm(", "bool MoveArmPlanned("))
        actual = re.sub(r"(?:static )?bool (ReadArmTurretAngle|MoveArmInternal|MoveArm|MoveArmPlanned)\(",
                        r"constexpr bool \1(", actual)
        actual = actual.replace("uint16_t turretRampMs, bool slowLower)", "uint16_t turretRampMs=600, bool slowLower=false)")
        harness = r'''
#include <stdint.h>
LIMITS
constexpr float INFINITY = __builtin_inff();
constexpr bool isfinite(float v) { return __builtin_isfinite(v); }
constexpr float fabsf(float v) { return v<0 ? -v : v; }
constexpr float roundf(float v) { return v<0 ? int(v-0.5f) : int(v+0.5f); }
constexpr int pdMS_TO_TICKS(int v) { return v; }
constexpr void vTaskDelay(int) {}
struct Log {
    constexpr void println(const char*) {}
    template<class... A> constexpr void printf(const char*, A...) {}
};
struct Pose { float high=0, length=0, turret_angle=0, pawl_angle=0; };
struct Command {
    int id=0; float angle=0; uint32_t time=0;
    uint16_t acc=0, dec=0, power=0; bool advanced=false;
};
struct Run {
    bool taskMotionAborted=false, queryOk=true;
    bool abortOnQuery=false;
    float measuredTurret=0, HEIGHT_PULSE=80, LENGTH_PULSE=30.2f;
    int count=0, turretQueries=0;
    Log Serial;
    Pose currentArm;
    Command commands[8]={};
    constexpr bool Servo_QueryAngleMTurn(int id, float& angle, int) {
        if (id==2) { ++turretQueries; if (abortOnQuery) taskMotionAborted=true; }
        angle=id==2 ? measuredTurret : 0;
        return queryOk;
    }
    constexpr void Servo_SetAngleMTurn(int id, float a, uint32_t t, uint16_t p) {
        commands[count++]={id,a,t,0,0,p,false};
    }
    constexpr void Servo_SetAngleMTurnByInterval(int id, float a, uint32_t t,
                                                uint16_t acc, uint16_t dec, uint16_t p) {
        commands[count++]={id,a,t,acc,dec,p,true};
    }
    constexpr void Emm_V5_Pos_Control(uint8_t id, uint8_t, uint16_t, uint8_t,
                                     uint32_t, bool, bool) {
        commands[count++]={id,0,0,0,0,0,false};
    }
    ACTUAL
};
constexpr bool timed(const Command& c, float angle, uint32_t time) {
    return c.id==2 && c.advanced && c.angle==angle && c.time==time
        && c.acc==600 && c.dec==600 && c.time>c.acc+c.dec && c.power==0;
}
constexpr bool turn(float current, float target, float finalTarget, uint32_t time, float speed=250) {
    Run r;
    r.measuredTurret=current; r.currentArm.turret_angle=999; // 故意污染目标缓存。
    return r.MoveArm(-1,-1,target,-1,speed) && r.count==1 && r.turretQueries==1
        && timed(r.commands[0],finalTarget,time) && r.currentArm.turret_angle==finalTarget;
}
constexpr bool grip(float target, float speed, float finalTarget, uint32_t time) {
    Run r;
    return r.MoveArm(-1,-1,-1,target,speed) && r.count==1
        && r.commands[0].id==1 && !r.commands[0].advanced
        && r.commands[0].angle==finalTarget && r.commands[0].time==time
        && r.commands[0].power==0 && r.currentArm.pawl_angle==target;
}
constexpr bool plannedTurn(float current, float target, uint32_t time) {
    Run r;
    r.measuredTurret=current; r.currentArm.turret_angle=999;
    return r.MoveArmPlanned(-1,-1,target,-1,250) && r.count==1 && r.turretQueries==1
        && timed(r.commands[0],target,time) && r.currentArm.turret_angle==target;
}
constexpr bool feedback(float measured, bool queryOk, bool abort, bool duringQuery,
                        bool expected) {
    Run r;
    r.measuredTurret=measured; r.queryOk=queryOk;
    r.taskMotionAborted=abort; r.abortOnQuery=duringQuery;
    r.currentArm.turret_angle=999;
    float angle=777;
    return r.ReadArmTurretAngle(angle)==expected && angle==(expected ? measured : 777)
        && r.currentArm.turret_angle==999 && r.count==0
        && r.turretQueries==(abort ? 0 : 1);
}
constexpr bool plannedGuards() {
    Run missing; missing.queryOk=false;
    Run invalid; invalid.measuredTurret=__builtin_nanf("");
    Run outside; outside.measuredTurret=TURRET_CABLE_MIN_DEG-1;
    Run aborted; aborted.taskMotionAborted=true;
    Run interrupted; interrupted.abortOnQuery=true;
    return !missing.MoveArmPlanned(10,20,90,50,250) && missing.count==0
        && !invalid.MoveArmPlanned(10,20,90,50,250) && invalid.count==0
        && !outside.MoveArmPlanned(10,20,90,50,250) && outside.count==0
        && !aborted.MoveArmPlanned(10,20,90,50,250) && aborted.count==0 && aborted.turretQueries==0
        && !interrupted.MoveArmPlanned(10,20,90,50,250) && interrupted.count==0;
}
constexpr bool guards() {
    Run missing; missing.queryOk=false;
    Run invalid; invalid.measuredTurret=__builtin_nanf("");
    Run outside; outside.measuredTurret=TURRET_CABLE_MAX_DEG+1;
    Run aborted; aborted.taskMotionAborted=true;
    Run skip;
    return !missing.MoveArm(-1,-1,90,-1,250) && missing.count==0
        && !invalid.MoveArm(-1,-1,90,-1,250) && invalid.count==0
        && !outside.MoveArm(-1,-1,90,-1,250) && outside.count==0
        && !aborted.MoveArm(-1,-1,90,-1,250) && aborted.count==0 && aborted.turretQueries==0
        && skip.MoveArm(-1,-1,-1,-1,250) && skip.count==0 && skip.turretQueries==0;
}
constexpr bool mixedAxes() {
    Run r;
    return r.MoveArm(10,20,180,50,250) && r.count==4
        && r.commands[0].id==5 && r.commands[1].id==6
        && timed(r.commands[2],180,1320)
        && r.commands[3].id==1 && !r.commands[3].advanced
        && r.commands[3].angle==50 && r.commands[3].time==89 && r.commands[3].power==0;
}
constexpr bool plannedMixedAxes() {
    Run r;
    return r.MoveArmPlanned(10,20,-1,50,250) && r.count==4
        && r.commands[0].id==5 && r.commands[1].id==6
        && timed(r.commands[2],-1,1201) && r.commands[3].id==1;
}
static_assert(turn(0,180,180,1320), "180 degrees: 250 deg/s peak, 600 ms acceleration and deceleration");
static_assert(turn(0,90,90,1201), "90 degrees: minimum 1201 ms total");
static_assert(turn(0,45,45,1201), "45 degrees: minimum 1201 ms total");
static_assert(turn(0,0,0,1201), "zero angle retains minimum interval above acceleration plus deceleration");
static_assert(turn(90,180,180,1201), "use measured displacement rather than target value");
static_assert(turn(180,90,90,1201), "reverse direction uses absolute displacement");
static_assert(turn(0,99,99,1201), "99 degrees: minimum 1201 ms total");
static_assert(turn(10,350,-10,1201), "choose nearest equivalent within cable limits");
static_assert(turn(350,10,10,1960), "reject shorter equivalent outside cable limits");
static_assert(turn(-150,180,180,1920), "large permitted displacement scales beyond reference");
static_assert(turn(0,180,180,1320,100) && turn(0,180,180,1320,300), "turret timing ignores old speed mapping");
static_assert(grip(90,200,90,200), "gripper duration is one fifth of original timing");
static_assert(grip(45,200,45,100), "gripper duration scales with angular travel");
static_assert(grip(10,200,10,40), "gripper short moves use one-fifth minimum duration");
static_assert(grip(0,200,0,0), "gripper zero travel needs no artificial duration");
static_assert(grip(350,200,-10,40), "gripper selects nearest equivalent angle before timing");
static_assert(guards(), "query failures, invalid angles, abort and sentinel");
static_assert(mixedAxes(), "retain motor and gripper command order and gripper timing");
static_assert(turn(0,180,180,1320,500) && grip(90,500,90,80), "accept new maximum command speed 500");
static_assert(turn(123.3f,-60,300,1307), "ordinary callers retain nearest-angle behavior");
static_assert(plannedTurn(123.3f,-60,1333), "planned branch overrides greedy equivalent");
static_assert(plannedTurn(0,360,2040) && plannedTurn(350,-10,2040), "preserve planned turn count");
static_assert(plannedTurn(0,-1,1201), "planned minus one is an actual angle");
static_assert(plannedTurn(360,-150,2640) && plannedTurn(-150,360,2640), "cable boundary travel");
static_assert(plannedGuards() && plannedMixedAxes(), "planned guards precede all commands");
constexpr bool areaRampTimes() {
    Run disc,work;
    return disc.MoveArm(-1,-1,180,-1,250,20)
        && disc.commands[0].acc==20 && disc.commands[0].dec==20 && disc.commands[0].time==740
        && work.MoveArmPlanned(-1,-1,180,-1,250,500)
        && work.commands[0].acc==500 && work.commands[0].dec==500 && work.commands[0].time==1220;
}
static_assert(areaRampTimes(), "per-action ramp times reach the servo protocol unchanged");
static_assert(feedback(-1,true,false,false,true), "actual minus one is valid feedback");
static_assert(feedback(360,true,false,false,true), "feedback keeps cache untouched");
static_assert(feedback(0,false,false,false,false), "missing feedback preserves output");
static_assert(feedback(__builtin_nanf(""),true,false,false,false), "reject nonfinite feedback");
static_assert(feedback(-151,true,false,false,false), "reject lower cable overflow");
static_assert(feedback(361,true,false,false,false), "reject upper cable overflow");
constexpr bool feedbackRecovery() {
    Run r; float angle=777;
    r.queryOk=false;
    if (r.ReadArmTurretAngle(angle) || angle!=777) return false;
    r.queryOk=true; r.measuredTurret=90;
    return r.MoveArm(-1,-1,180,-1,250) && r.count==1 && r.turretQueries==2;
}
static_assert(feedbackRecovery(), "failed reads do not latch future motion");
static_assert(feedback(0,true,true,false,false), "abort blocks query");
static_assert(feedback(0,true,false,true,false), "abort during query preserves output");
'''.replace("LIMITS", limits).replace("ACTUAL", actual)
        self.compile_assertions(harness)


if __name__ == "__main__":
    unittest.main()
