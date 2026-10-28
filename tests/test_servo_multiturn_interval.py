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

    def test_actual_move_arm_turret_routing(self):
        source = (ROOT / "src/chassis.cpp").read_text(encoding="utf-8")
        actual = "\n".join(function_source(source, signature) for signature in (
            "bool ReadArmTurretAngle(", "static bool MoveArmInternal(",
            "bool MoveArm(", "bool MoveArmPlanned("))
        actual = re.sub(r"(?:static )?bool (ReadArmTurretAngle|MoveArmInternal|MoveArm|MoveArmPlanned)\(",
                        r"constexpr bool \1(", actual)
        actual = actual.replace("uint16_t turretRampMs, bool slowLower)", "uint16_t turretRampMs=600, bool slowLower=false)")
        harness = r'''
#include <stdint.h>
constexpr float INFINITY=__builtin_inff();
constexpr bool isfinite(float v){return __builtin_isfinite(v);}
constexpr float fabsf(float v){return v<0?-v:v;}
constexpr float roundf(float v){return v<0?int(v-0.5f):int(v+0.5f);}
constexpr int pdMS_TO_TICKS(int v){return v;}
constexpr void vTaskDelay(int){}
constexpr float TURRET_CABLE_MIN_DEG=-150,TURRET_CABLE_MAX_DEG=360;
struct Log {constexpr void println(const char*){} template<class... A> constexpr void printf(const char*,A...) {}};
struct Pose {float high=0,length=0,turret_angle=0,pawl_angle=0;};
struct Run {
    bool taskMotionAborted=false,queryOk=true;
    float measured=0,HEIGHT_PULSE=80,LENGTH_PULSE=30;
    int commands=0,queries=0; float angle=0; uint16_t ramp=0;
    Log Serial; Pose currentArm;
    constexpr bool TurretMotorReadAngle(float &v){++queries;v=measured;return queryOk;}
    constexpr bool TurretMotorMove(float v,uint16_t r){++commands;angle=v;ramp=r;return true;}
    constexpr bool Servo_QueryAngleMTurn(int,float &v,int){v=0;return true;}
    constexpr void Servo_SetAngleMTurn(int,float,uint32_t,uint16_t){++commands;}
    constexpr void Emm_V5_Pos_Control(uint8_t,uint8_t,uint16_t,uint8_t,uint32_t,bool,bool){++commands;}
    ACTUAL
};
constexpr bool ordinary(){
    Run r;r.measured=10;r.currentArm.turret_angle=999;
    return r.MoveArm(-1,-1,350,-1,200,20)&&r.commands==1&&r.queries==1
        &&r.angle==350&&r.ramp==20&&r.currentArm.turret_angle==350;
}
constexpr bool winding(){
    Run r;r.measured=350;
    if(!r.MoveArm(-1,-1,0,-1,200)||r.angle!=0)return false;
    if(!r.MoveArm(-1,-1,360,-1,200)||r.angle!=360)return false;
    return r.MoveArm(-1,-1,-55,-1,200)&&r.angle==-55;
}
constexpr bool planned(){
    Run r;r.measured=350;
    if(!r.MoveArmPlanned(-1,-1,-10,-1,200,500)||r.angle!=-10||r.ramp!=500)return false;
    return r.MoveArmPlanned(-1,-1,-1,-1,200)&&r.angle==-1&&r.commands==2;
}
constexpr bool feedback(){
    Run r;float value=777;r.queryOk=false;
    if(r.ReadArmTurretAngle(value)||value!=777||r.MoveArm(1,2,90,50,200)||r.commands)return false;
    r.queryOk=true;r.measured=361;if(r.ReadArmTurretAngle(value))return false;
    r.measured=90;return r.ReadArmTurretAngle(value)&&value==90
        &&r.MoveArm(1,2,180,50,200)&&r.commands==4;
}
constexpr bool skip(){
    Run r;if(!r.MoveArm(-1,-1,-1,-1,200)||r.queries||r.commands)return false;
    r.taskMotionAborted=true;return !r.MoveArm(1,2,90,50,200)&&!r.commands;
}
static_assert(ordinary(),"ordinary targets preserve absolute angle and winding");
static_assert(winding(),"zero, 360 and negative targets remain distinct");
static_assert(planned(),"planned signed absolute angles and per-action ramps reach motor");
static_assert(feedback(),"read failures preserve output and do not latch future motion");
static_assert(skip(),"sentinel and cancellation remain effective");
'''.replace("ACTUAL",actual)
        self.compile_assertions(harness)


if __name__ == "__main__":
    unittest.main()
