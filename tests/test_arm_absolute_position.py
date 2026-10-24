"""编译期执行实际 MoveArm，验证绝对目标不依赖主控缓存；无需硬件。"""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class ArmAbsolutePositionTests(unittest.TestCase):
    def test_actual_move_arm_commands(self):
        source = (ROOT / "src/chassis.cpp").read_text(encoding="utf-8")
        start = source.index("bool ReadArmTurretAngle(")
        end = source.index("\n\n\n/**", start)
        move = re.sub(r"(?:static )?bool (ReadArmTurretAngle|MoveArmInternal|MoveArm|MoveArmPlanned)\(",
                      r"constexpr bool \1(", source[start:end])
        header = (ROOT / "src/chassis.h").read_text(encoding="utf-8")
        names = ("ARM_HEIGHT_LIMIT_MM", "TURRET_CABLE_MIN_DEG", "TURRET_CABLE_MAX_DEG")
        limits = "\n".join(re.search(r"constexpr float " + name + r"\s*=[^;]+;",
                                      header).group() for name in names)
        move = move.replace("uint16_t turretRampMs, bool slowLower)", "uint16_t turretRampMs=600, bool slowLower=false)")
        harness = r'''
#include <stdint.h>
#include <float.h>
constexpr float INFINITY = __builtin_inff();
constexpr bool isfinite(float v) { return __builtin_isfinite(v); }
constexpr float fabsf(float v) { return v < 0 ? -v : v; }
constexpr float roundf(float v) { return v < 0 ? int(v - 0.5f) : int(v + 0.5f); }
constexpr int pdMS_TO_TICKS(int v) { return v; }
constexpr void vTaskDelay(int) {}
LIMITS
struct Log {
    constexpr void println(const char*) {}
    template<class... A> constexpr void printf(const char*, A...) {}
};
struct Pose { float high=0, length=0, turret_angle=0, pawl_angle=0; };
struct Command { uint8_t addr=0, dir=0; uint32_t pulses=0; bool absolute=false, sync=false; uint16_t rpm=0; uint8_t acceleration=0; };
struct Run {
    bool taskMotionAborted=false;
    Log Serial;
    Pose currentArm;
    float HEIGHT_PULSE=10, LENGTH_PULSE=20;
    Command commands[8];
    int count=0;
    constexpr bool Servo_QueryAngleMTurn(int, float& angle, int) { angle=0; return true; }
    constexpr void Servo_SetAngleMTurn(int, float, float, int) {}
    constexpr void Servo_SetAngleMTurnByInterval(int, float, uint32_t,
                                                uint16_t, uint16_t, uint16_t) {}
    constexpr void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acceleration,
                                      uint32_t pulses, bool absolute, bool sync) {
        commands[count++]={addr, dir, pulses, absolute, sync, rpm, acceleration};
    }
    MOVE_ARM
};
constexpr bool target(const Command& c, int addr, int pulses) {
    return c.addr==addr && c.dir==1 && c.pulses==pulses && c.absolute && !c.sync;
}
constexpr bool cacheIndependent() {
    Run r;
    r.currentArm.high=150; r.currentArm.length=100;
    if (!r.MoveArm(50, 30, -1, -1, 100) || r.count!=2) return false;
    if (!target(r.commands[0], 5, 500) || !target(r.commands[1], 6, 600)) return false;
    if (!r.MoveArm(50, 30, -1, -1, 100) || r.count!=4) return false;
    if (!target(r.commands[2], 5, 500) || !target(r.commands[3], 6, 600)) return false;
    r.currentArm.high=999; r.currentArm.length=-123;
    if (!r.MoveArm(50, 30, -1, -1, 100) || r.count!=6) return false;
    return target(r.commands[4], 5, 500) && target(r.commands[5], 6, 600)
        && r.currentArm.high==50 && r.currentArm.length==30;
}
constexpr bool zeroAndSkip() {
    Run r;
    if (!r.MoveArm(0, 0, -1, -1, 100) || r.count!=2) return false;
    if (!target(r.commands[0], 5, 0) || !target(r.commands[1], 6, 0)) return false;
    if (!r.MoveArm(-1, -1, -1, -1, 100) || r.count!=2) return false;
    if (!r.MoveArm(-1, 170, -1, -1, 100) || r.count!=3) return false;
    if (!target(r.commands[2], 6, 3400)) return false;
    return r.MoveArm(175, -1, -1, -1, 100) && r.count==4
        && target(r.commands[3], 5, 1750);
}
constexpr bool rejectInvalid() {
    Run r;
    r.taskMotionAborted=true;
    return !r.MoveArm(10, 10, -1, -1, 100) && r.count==0;
}
constexpr bool speedWithoutArtificialLimit() {
    Run normal, large;
    if (!normal.MoveArm(10, 10, -1, -1, 1000) || normal.count!=2) return false;
    if (normal.commands[0].rpm!=15000 || normal.commands[1].rpm!=6000) return false;
    return large.MoveArm(10, 10, -1, -1, FLT_MAX) && large.count==2
        && large.commands[0].rpm==UINT16_MAX && large.commands[1].rpm==UINT16_MAX;
}
constexpr bool gentlePlacement() {
    Run normal, slow;
    if(!normal.MoveArm(2.5f,61,-1,-1,250,500) || !slow.MoveArm(2.5f,61,-1,-1,250,500,true))return false;
    return slow.commands[0].rpm==normal.commands[0].rpm/2
        && normal.commands[0].acceleration==252 && slow.commands[0].acceleration==248
        && slow.commands[1].rpm==normal.commands[1].rpm
        && slow.commands[0].pulses==normal.commands[0].pulses;
}
static_assert(gentlePlacement(), "only placement lift axis halves RPM and physical acceleration");
static_assert(speedWithoutArtificialLimit(), "valid speed maps directly to protocol RPM");
static_assert(cacheIndependent(), "descending/repeated/stale-cache absolute targets");
static_assert(zeroAndSkip(), "absolute zero, sentinel and travel limits");
static_assert(rejectInvalid(), "task abort remains effective");
'''.replace("LIMITS", limits).replace("MOVE_ARM", move)
        compiler = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio")) / (
            "packages/toolchain-xtensa-esp32s3/bin/xtensa-esp32s3-elf-g++.exe")
        self.assertTrue(compiler.is_file(), "需要已安装的 ESP32-S3 编译器")
        with tempfile.TemporaryDirectory() as directory:
            fixture = Path(directory) / "arm.cpp"
            fixture.write_text(harness, encoding="utf-8")
            result = subprocess.run([str(compiler), "-std=c++14", "-fsyntax-only", str(fixture)],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_startup_zero_precedes_command_tasks(self):
        from native_support import run_native
        from test_serial_motion import build_serial_motion
        run_native(build_serial_motion(), "startup")


if __name__ == "__main__":
    unittest.main()
