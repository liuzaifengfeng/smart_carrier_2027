"""编译期执行实际扫码移动分支，验证两启停区方向及提前停车。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from upper_computer import Pose, START_POSES


class ScanDirectionTests(unittest.TestCase):
    def test_start_heading_and_scan_to_left_boundary_turn(self):
        root = Path(__file__).resolve().parents[1]
        compiler = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio")) / (
            "packages/toolchain-xtensa-esp32s3/bin/xtensa-esp32s3-elf-g++.exe")
        self.assertTrue(compiler.is_file())
        main = (root / "src/main.cpp").read_text(encoding="utf-8")
        header = (root / "src/robot_runtime.h").read_text(encoding="utf-8")
        serial = (root / "src/serial_commands.inc").read_text(encoding="utf-8")
        heading = next(line for line in header.splitlines()
                       if line.startswith("constexpr float START_ZONE_HEADING ="))
        start = main[main.index("            switch (homeStartZone) {"):]
        start = start[:start.index("            // Release 开局自动执行一次。")]
        debug = serial[serial.index("        if (serialDebugMode) currentPose = zone == 1"):]
        debug = debug[:debug.index(";") + 1]
        harness = heading + r'''
#include "node_route_heading.h"
enum StartZone { START_ZONE_UNKNOWN, START_ZONE_1, START_ZONE_2 };
struct RobotPose { float x, y, theta; };
constexpr void updateDisplay(const char *, const char *, const char *) {}
constexpr RobotPose automatic(StartZone homeStartZone) {
    RobotPose currentPose{};
''' + start + r'''
    return currentPose;
}
constexpr RobotPose debugPose(int zone) {
    const bool serialDebugMode = true;
    RobotPose currentPose{};
''' + debug + r'''
    return currentPose;
}
constexpr bool check(StartZone zone) {
    const auto pose = automatic(zone);
    const auto debug = debugPose(zone);
    if (pose.theta != 180 || debug.theta != pose.theta || debug.x != pose.x
            || debug.y != pose.y || pose.y != 150
            || pose.x != (zone == START_ZONE_1 ? 2250 : 150)) return false;
    // 扫码平移不转车头。节点2向左到0，再从0向上到9：左转弯处须顺时针90°。
    const auto left = PlanNodeSegmentHeading(pose.theta, 180, false, 0);
    const auto up = PlanNodeSegmentHeading(left.heading, 90, false, 0);
    return !left.reverse && left.turn == 0 && left.heading == 180
        && !up.reverse && up.turn == -90 && up.heading == 90;
}
static_assert(check(START_ZONE_1), "zone 1 retains known working heading and route");
static_assert(check(START_ZONE_2), "zone 2 heading agrees with actual car; turn goes into field");
'''
        with tempfile.TemporaryDirectory() as directory:
            fixture = Path(directory) / "start_heading.cpp"
            fixture.write_text(harness, encoding="utf-8")
            result = subprocess.run([str(compiler), "-std=c++14", "-fsyntax-only",
                                     "-I", str(root / "src"), str(fixture)],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(START_POSES["启停区1"], Pose(2250, 150, 180))
        self.assertEqual(START_POSES["启停区2"], Pose(150, 150, 180))

    def test_scan_motion_for_both_start_zones(self):
        root = Path(__file__).resolve().parents[1]
        compiler = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio")) / (
            "packages/toolchain-xtensa-esp32s3/bin/xtensa-esp32s3-elf-g++.exe")
        self.assertTrue(compiler.is_file(), "需要已安装的 ESP32-S3 编译器")
        source = (root / "src/main.cpp").read_text(encoding="utf-8")
        scan = source[source.index("        case STATE_READ_TASK:"):
                      source.index("        case STATE_GRAB_ROUND1:")]
        declarations = scan[scan.index("            constexpr float SCAN_APPROACH_MM"):
                            scan.index("            const auto pollTaskCode")]
        motion = scan[scan.index("            if (!taskReceived) {"):
                      scan.index('                updateDisplay("DISPLAY", "DEBUG", "TASK ERR");')]
        # 去除紧接运动分支后的失败状态分支开头。
        motion = motion[:motion.rindex("            if (!taskReceived) {")]
        harness = r'''
#include <stdint.h>
enum StartZone { START_ZONE_UNKNOWN, START_ZONE_1, START_ZONE_2 };
struct Logger {
    constexpr void println(const char *) const {}
    template<class... Args> constexpr void printf(const char *, Args...) const {}
};
struct Run {
    StartZone currentStartZone;
    bool taskReceived = false;
    bool failApproach = false;
    bool failScan = false;
    int receiveAfter = -1;
    int count = 0;
    float distances[5] = {};
    float speeds[5] = {};
    Logger Serial;
    constexpr bool pollTaskCode() {
        if (receiveAfter >= 0 && count >= receiveAfter) taskReceived = true;
        return taskReceived;
    }
    constexpr bool MovePosition(float x, float y, float theta, float speed,
                                bool callback = false) {
        if (count >= 5 || x != 0 || theta != 0) return false;
        distances[count] = y;
        speeds[count++] = speed;
        if (callback && pollTaskCode()) return false;
        return !(count == 1 ? failApproach : failScan);
    }
    constexpr void execute() {
'''
        # 将实际回调实参映射为 mock 标志；保留真实分支和位移表达式。
        harness += declarations + motion.replace(
            "SCAN_SPEED_RPM/2, pollTaskCode)", "SCAN_SPEED_RPM/2, true)")
        harness += r'''
    }
};
constexpr bool check(StartZone zone, float sign) {
    Run r{zone}; r.execute();
    if (r.count != 5 || r.taskReceived || r.distances[0] != sign * 900
            || r.speeds[0] != 60) return false;
    for (int i = 1; i < 5; ++i) {
        if (r.distances[i] != sign * (i % 2 ? 400 : -400)
                || r.speeds[i] != 30) return false;
    }
    Run preset{zone}; preset.taskReceived = true; preset.execute();
    if (preset.count != 0) return false;
    Run stopped{zone}; stopped.receiveAfter = 2; stopped.execute();
    if (stopped.count != 2 || !stopped.taskReceived) return false;
    Run approachFailed{zone}; approachFailed.failApproach = true; approachFailed.execute();
    if (approachFailed.count != 1) return false;
    Run scanFailed{zone}; scanFailed.failScan = true; scanFailed.execute();
    return scanFailed.count == 2;
}
static_assert(check(START_ZONE_1, 1), "zone 1: left approach and alternating scan");
static_assert(check(START_ZONE_2, -1), "zone 2: right approach and alternating scan");
constexpr bool invalid() { Run r{START_ZONE_UNKNOWN}; r.execute(); return r.count == 0; }
static_assert(invalid(), "invalid start zone must not move");
'''
        with tempfile.TemporaryDirectory() as directory:
            fixture = Path(directory) / "scan.cpp"
            fixture.write_text(harness, encoding="utf-8")
            result = subprocess.run([str(compiler), "-std=c++14", "-fsyntax-only",
                                     str(fixture)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
