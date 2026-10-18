"""位置快照的封包、界面回显及固件实际处理函数，无需硬件。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import Mock

from upper_computer import (build_disc_positions_command,
                            parse_disc_positions_response, UpperComputerApp)

ROOT = Path(__file__).resolve().parents[1]


class DiscPositionsTests(unittest.TestCase):
    def test_each_completed_grab_restarts_position_recognition(self):
        main_source = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
        # 两轮首次识别也必须先保持 NOT_READY，经清旧边界后才开放。
        self.assertEqual(main_source.count(
            "firstDiscGrabReady = false;\n"
            "                // 启用物料识别接收状态"), 2)
        self.assertGreaterEqual(main_source.count(
            "requestVisionStart(VisionStartMode::DISC_MATERIAL);"), 4)
        self.assertGreaterEqual(main_source.count("discMessageResumeRequested = true;"), 4)
        for progress_update in (
                "discPreparedProgress = completedGrabs;",
                "disc2PreparedProgress = completedGrabs;"):
            start = main_source.index(progress_update)
            recovery = main_source[start:start + 600]
            vision_start = recovery.index(
                "requestVisionStart(VisionStartMode::DISC_MATERIAL);")
            resume = recovery.index("discMessageResumeRequested = true;")
            self.assertLess(vision_start, resume)

        handler_source = (ROOT / "src/serial_commands.inc").read_text(encoding="utf-8")
        selected = handler_source.index(
            "{EVT,VISION,DISC_POSITIONS,SELECTED,%u,%u,%u}")
        position_switch = handler_source.index("switch (candidate.position)", selected)
        stop_old_session = handler_source.index(
            "requestVisionStop(VisionStartMode::DISC_MATERIAL);", selected)
        self.assertLess(stop_old_session, position_switch)

    def test_look3_requires_measured_turret_within_five_degrees(self):
        source = (ROOT / "src/chassis.cpp").read_text(encoding="utf-8")
        start = source.index("bool InitArm_look3()")
        look3 = source[start:source.index("\n}\n", start) + 3]
        self.assertIn("Servo_QueryAngleMTurn(2, measuredAngle, 100)", look3)
        self.assertIn("LOOK3_TURRET_TOLERANCE_DEG = 5.0f", look3)
        self.assertIn("fabsf(measuredAngle - LOOK3_TURRET_TARGET_DEG)", look3)
        self.assertLess(look3.index("Servo_QueryAngleMTurn"), look3.index("return true;", 200))

    def test_build_and_reject_invalid_candidates(self):
        self.assertEqual(build_disc_positions_command([(2, 1)]),
                         "{CMD,VISION,DISC_POSITIONS,2,1}")
        self.assertEqual(build_disc_positions_command([(1, 5), (2, 1), (3, 6)]),
                         "{CMD,VISION,DISC_POSITIONS,1,5,2,1,3,6}")
        for candidates in ([], [(1, 1)] * 4, [(1, 1), (1, 2)], [(0, 1)],
                           [(4, 1)], [(True, 1)], [(1, True)], [(1, 0)],
                           [(1, 7)], [(1, "2")], [(1.5, 1)]):
            with self.subTest(candidates=candidates), self.assertRaises(ValueError):
                build_disc_positions_command(candidates)

    def test_replies_and_ui_report_real_completion(self):
        app = UpperComputerApp.__new__(UpperComputerApp)
        app.disc_material_status = Mock()
        app.serial_link = Mock()
        app.append_log = Mock()
        app.disc_position_vars = [Mock(), Mock(), Mock()]
        for variable, value in zip(app.disc_position_vars, ("1 红色", "不报送", "6 浅蓝色")):
            variable.get.return_value = value
        app.send_disc_positions()
        app.serial_link.send_line.assert_called_once_with("{CMD,VISION,DISC_POSITIONS,1,1,3,6}")
        for line, message in (
            ("{RSP,VISION,DISC_POSITIONS,ACK,2}", "等待选择"),
            ("{EVT,VISION,DISC_POSITIONS,SELECTED,3,6,1}", "圆盘位置 3"),
            ("{EVT,VISION,DISC_POSITIONS,DONE,3,6,1}", "抓放完成"),
            ("{EVT,VISION,DISC_POSITIONS,FAILED,3,6,1}", "抓放失败"),
            ("{EVT,VISION,DISC_POSITIONS,SKIP,1,1,COLOR_MISMATCH}", "等待新位置"),
            ("{RSP,VISION,DISC_POSITIONS,ERR,NOT_READY}", "被拒绝"),
        ):
            self.assertTrue(app._accept_disc_material_response(line))
            self.assertIn(message, app.disc_material_status.set.call_args.args[0])
        for line in ("{RSP,VISION,DISC_POSITIONS,DONE,3,6,1}",
                     "{EVT,VISION,DISC_POSITIONS,DONE,0,6,1}",
                     "{EVT,VISION,DISC_POSITIONS,DONE,3,7,1}",
                     "{EVT,VISION,DISC_POSITIONS,DONE,3,6,4}",
                     "{RSP,VISION,DISC_POSITIONS,ACK,4}",
                     "{EVT,VISION,DISC_POSITIONS,SELECTED,3,6}"):
            self.assertIsNone(parse_disc_positions_response(line))
        app.serial_link.send_line.side_effect = RuntimeError("disconnected")
        app.send_disc_positions()
        self.assertIn("未发送", app.disc_material_status.set.call_args.args[0])

    def test_actual_firmware_handler_and_position_actions(self):
        zig = ROOT / ".pio/host-test-tools/ziglang/zig.exe"
        compiler = os.environ.get("CONTINUOUS_TEST_CXX")
        if not compiler and not zig.is_file():
            self.skipTest("需要主机 C++ 编译器")
        source = (ROOT / "src/serial_commands.inc").read_text(encoding="utf-8")
        start = source.index("static void serialDiscPositions(")
        handler = source[start:source.index("\nstatic void serialTransfer(", start)]
        harness = r'''
#include "serial_frame.h"
#include "disc_material_decision.h"
#include "disc_position_grab.h"
#include <assert.h>
#include <string>
NativeSerial Serial;
struct ArmCall { float high, length, turret, claw, speed; };
ArmCall armCalls[32]; int armCallCount = 0;
bool MoveArm(float high, float length, float turret, float claw, float speed) {
    assert(armCallCount < 32);
    armCalls[armCallCount++] = {high, length, turret, claw, speed};
    return true;
}
enum RobotState { OTHER, STATE_GRAB_ROUND1, STATE_GRAB_ROUND2, STATE_TRANSFER_FAILED };
enum class VisionStartMode { DISC_MATERIAL };
RobotState currentState = STATE_GRAB_ROUND1;
bool discMaterialActive = true, serialDebugMode = false;
bool alignmentEnabled = false, firstDiscGrabReady = true;
bool discMessageResumeRequested = false, placementSucceeds = true;
int roundProgress = 0, discMaterialColor = 5;
struct { bool valid; int round1_colors[3]; int round2_colors[3]; }
    currentTask{true, {1,5,6}, {5,1,6}};
constexpr int MATERIAL_NONE = 0;
struct { int material = MATERIAL_NONE; } cargoPlatforms[3];
int visionStops = 0, displayErrors = 0;
void *xHomeTimer = nullptr;
void requestVisionStop(VisionStartMode) { ++visionStops; discMaterialActive = false; }
void updateDisplay(const char*, const char*, const char*) { ++displayErrors; }
void xTimerStop(void*, int) {}
bool ValidateDiscPositionTransfer(uint8_t color, uint8_t cargo) {
    return color >= 1 && color <= 6 && cargo >= 1 && cargo <= 3
        && cargoPlatforms[cargo - 1].material == MATERIAL_NONE;
}
bool PlaceHeldDiscMaterialToCargo(uint8_t color, uint8_t cargo) {
    if (!placementSucceeds || !ValidateDiscPositionTransfer(color, cargo)) return false;
    cargoPlatforms[cargo - 1].material = color;
    return true;
}
HANDLER
void send(const char *command, const char *expected) {
    Serial.text.clear();
    char line[160]; strcpy(line, command);
    SerialFrame frame;
    assert(parseSerialFrame(line, frame));
    serialDiscPositions(frame);
    if (Serial.text.find(expected) == std::string::npos) {
        fprintf(stderr, "command=%s\nexpected=%s\nactual=%s", command, expected, Serial.text.c_str());
        assert(false);
    }
    assert(roundProgress >= 0 && roundProgress <= 3);
    if (strstr(expected, ",ERR,")) {
        assert(Serial.text.find(",ACK,") == std::string::npos);
    }
}
int main() {
    const char *batch = "{CMD,VISION,DISC_POSITIONS,1,5,2,1,3,6}";
    send(batch, "SELECTED,2,1,1}");
    assert(Serial.text.find("DONE,2,1,1") != std::string::npos);
    assert(roundProgress == 1 && cargoPlatforms[0].material == 1);
    assert(!firstDiscGrabReady && !discMessageResumeRequested && discMaterialColor == 0);
    const int callsAfterFirstGrab = armCallCount;
    send(batch, ",ERR,NOT_READY}");
    assert(roundProgress == 1 && armCallCount == callsAfterFirstGrab);

    // 模拟主状态机完成look3、重新开启新识别会话并排空旧位置帧。
    discMaterialActive = true; firstDiscGrabReady = true;
    send(batch, "SELECTED,1,5,2}");
    assert(Serial.text.find("DONE,1,5,2") != std::string::npos);
    assert(roundProgress == 2 && cargoPlatforms[1].material == 5);
    assert(armCallCount >= 4);
    const ArmCall *step = armCalls + armCallCount - 4;
    assert(step[0].high == -1 && step[0].length == 0 && step[0].turret == -1 && step[0].claw == 80 && step[0].speed == 200);
    assert(step[1].high == 80 && step[1].length == 0 && step[1].turret == 90 && step[1].claw == 80 && step[1].speed == 200);
    assert(step[2].high == 80 && step[2].length == 0 && step[2].turret == 90 && step[2].claw == -8 && step[2].speed == 200);
    assert(step[3].high == 170 && step[3].length == 0 && step[3].turret == 90 && step[3].claw == -8 && step[3].speed == 200);

    discMaterialActive = true; firstDiscGrabReady = true;
    send(batch, "DONE,3,6,3}");
    assert(roundProgress == 3 && cargoPlatforms[2].material == 6 && visionStops == 3);

    // 新一轮开始后验证无匹配、协议边界以及失败不推进。
    for (auto &cargo : cargoPlatforms) cargo.material = MATERIAL_NONE;
    roundProgress = 0; currentState = STATE_GRAB_ROUND2;
    discMaterialActive = true; firstDiscGrabReady = true;
    send("{CMD,VISION,DISC_POSITIONS,1,1,2,6}", "SKIP,5,1,COLOR_MISMATCH}");
    send("{CMD,VISION,DISC_POSITIONS,1,5,1,6}", ",ERR,DUPLICATE_POSITION}");
    send("{CMD,VISION,DISC_POSITIONS,1,5,3,7}", ",ERR,RANGE}");
    send("{CMD,VISION,DISC_POSITIONS,0,1}", ",ERR,RANGE}");
    send("{CMD,VISION,DISC_POSITIONS,4,1}", ",ERR,RANGE}");
    send("{CMD,VISION,DISC_POSITIONS,1.0,1}", ",ERR,RANGE}");
    send("{CMD,VISION,DISC_POSITIONS}", ",ERR,FORMAT}");
    send("{CMD,VISION,DISC_POSITIONS,1,1,2}", ",ERR,FORMAT}");
    send("{CMD,VISION,DISC_POSITIONS,1,1,2,2,3,3,1,4}", ",ERR,FORMAT}");
    discMaterialActive = false; send(batch, ",ERR,NOT_ACTIVE}"); discMaterialActive = true;
    currentTask.valid = false; send(batch, ",ERR,NO_TASK}"); currentTask.valid = true;
    currentState = OTHER; send(batch, ",ERR,NOT_WAITING}"); currentState = STATE_GRAB_ROUND1;
    roundProgress = 3; send(batch, ",ERR,ROUND_COMPLETE}"); roundProgress = 0;
    alignmentEnabled = true; send(batch, ",ERR,ALIGN_ACTIVE}"); alignmentEnabled = false;
    firstDiscGrabReady = false; send(batch, ",ERR,NOT_READY}");
    currentState = STATE_GRAB_ROUND2;
    firstDiscGrabReady = true; placementSucceeds = false;
    send(batch, "FAILED,1,5,1}");
    assert(roundProgress == 0 && cargoPlatforms[0].material == MATERIAL_NONE);
    assert(currentState == STATE_TRANSFER_FAILED && !firstDiscGrabReady);
    assert(visionStops == 4 && displayErrors == 1);

    currentState = STATE_GRAB_ROUND1; discMaterialActive = true; firstDiscGrabReady = true;
    placementSucceeds = true; serialDebugMode = true;
    send("{CMD,VISION,DISC_POSITIONS,2,1}", "DONE,2,1,1}");
    assert(roundProgress == 1 && cargoPlatforms[0].material == 1);
    // 占用拒绝单独验证，保留原物料记录。
    roundProgress = 0; cargoPlatforms[0].material = 4;
    Serial.text.clear(); char line[] = "{CMD,VISION,DISC_POSITIONS,2,1}";
    SerialFrame frame; assert(parseSerialFrame(line, frame)); serialDiscPositions(frame);
    assert(Serial.text == "{RSP,VISION,DISC_POSITIONS,ERR,OCCUPIED}\n");
    assert(cargoPlatforms[0].material == 4 && roundProgress == 0);
}
'''.replace("HANDLER", handler)
        command = [compiler] if compiler else [str(zig), "c++"]
        with tempfile.TemporaryDirectory(dir=ROOT / ".pio") as directory:
            fixture = Path(directory) / "disc_positions.cpp"
            executable = Path(directory) / "disc_positions.exe"
            fixture.write_text(harness, encoding="utf-8")
            result = subprocess.run(command + ["-std=c++11", "-O1", "-I", str(ROOT / "src"),
                "-I", str(ROOT / "tests/native_stubs"), str(fixture),
                str(ROOT / "src/disc_position_grab.cpp"), "-o", str(executable)],
                capture_output=True, text=True, encoding="utf-8", errors="replace")
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True,
                                    encoding="utf-8", errors="replace")
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
