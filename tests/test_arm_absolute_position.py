"""编译期执行实际 MoveArm，验证绝对目标不依赖主控缓存；无需硬件。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class ArmAbsolutePositionTests(unittest.TestCase):
    def test_actual_move_arm_commands(self):
        source = (ROOT / "src/chassis.cpp").read_text(encoding="utf-8")
        start = source.index("bool MoveArm(")
        end = source.index("\n\n\n/**", start)
        move = source[start:end].replace("bool MoveArm(", "constexpr bool MoveArm(", 1)
        harness = r'''
#include <stdint.h>
#include <float.h>
constexpr float INFINITY = __builtin_inff();
constexpr bool isfinite(float v) { return __builtin_isfinite(v); }
constexpr float fabsf(float v) { return v < 0 ? -v : v; }
constexpr float roundf(float v) { return v < 0 ? int(v - 0.5f) : int(v + 0.5f); }
constexpr int pdMS_TO_TICKS(int v) { return v; }
constexpr void vTaskDelay(int) {}
constexpr float ARM_HEIGHT_LIMIT_MM=175, TURRET_CABLE_MIN_DEG=-180, TURRET_CABLE_MAX_DEG=360;
struct Log {
    constexpr void println(const char*) {}
    template<class... A> constexpr void printf(const char*, A...) {}
};
struct Pose { float high=0, length=0, turret_angle=0, pawl_angle=0; };
struct Command { uint8_t addr=0, dir=0; uint32_t pulses=0; bool absolute=false, sync=false; };
struct Run {
    bool taskMotionAborted=false, s_turretStartupReferenceFault=false;
    const char* s_turretStartupReferenceFaultReason="UNKNOWN";
    Log Serial;
    Pose currentArm;
    float HEIGHT_PULSE=10, LENGTH_PULSE=20;
    Command commands[8];
    int count=0;
    constexpr bool Servo_QueryAngleMTurn(int, float& angle, int) { angle=0; return true; }
    constexpr void Servo_SetAngleMTurn(int, float, float, int) {}
    constexpr void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t, uint8_t,
                                      uint32_t pulses, bool absolute, bool sync) {
        commands[count++]={addr, dir, pulses, absolute, sync};
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
    if (r.MoveArm(175.1f, -1, -1, -1, 100)) return false;
    if (r.MoveArm(-1, 170.1f, -1, -1, 100)) return false;
    if (r.MoveArm(-2, -1, -1, -1, 100)) return false;
    if (r.MoveArm(-1, -2, -1, -1, 100)) return false;
    if (r.MoveArm(INFINITY, -1, -1, -1, 100)) return false;
    if (r.MoveArm(-1, __builtin_nanf(""), -1, -1, 100)) return false;
    r.taskMotionAborted=true;
    return !r.MoveArm(10, 10, -1, -1, 100) && r.count==0;
}
static_assert(cacheIndependent(), "descending/repeated/stale-cache absolute targets");
static_assert(zeroAndSkip(), "absolute zero, sentinel and travel limits");
static_assert(rejectInvalid(), "range, nonfinite and task abort protections");
'''.replace("MOVE_ARM", move)
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
        driver = (ROOT / "src/Emm_V5.cpp").read_text(encoding="utf-8")
        init = driver[driver.index("void Emm_V5_Init(void)"):driver.index("void Emm_V5_Reset_CurPos_To_Zero(")]
        self.assertLess(init.index("Serial1.begin("), init.index("Emm_V5_Reset_CurPos_To_Zero(0)"))
        self.assertLess(init.index("Emm_V5_Reset_CurPos_To_Zero(0)"), init.index("Serial1.flush()"))
        main = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
        setup = main[main.index("void setup() {"):main.index("void loop() {")]
        self.assertLess(setup.index("Emm_V5_Init()"), setup.index("currentArm = {0, 0, 0, 0}"))
        self.assertLess(setup.index("currentArm = {0, 0, 0, 0}"), setup.index("xTaskCreate("))


if __name__ == "__main__":
    unittest.main()
