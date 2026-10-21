"""编译实际雷达入口，回归自动/串口重试及完成缓存。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class LidarPoseTests(unittest.TestCase):
    def test_release_retries_do_not_repeat_motion(self):
        source = (ROOT / "src/robot_runtime.cpp").read_text(encoding="utf-8")
        action = source[source.index("bool runLidarPoseAction("):
                        source.index("// ================= 视觉闭环对齐任务")]
        action = action.replace("bool runLidarPoseAction(", "constexpr bool runLidarPoseAction(")
        action = action.replace("    static bool releasePoseCompleted = false;", "    // cached member:")
        action = action.replace("    static bool releasePoseInitialized = false;", "    // initialized member:")
        harness = r'''
#include <stdint.h>
#include <stddef.h>
constexpr int pdTRUE=1;
constexpr int portMAX_DELAY=-1;
enum {START_ZONE_1=1,START_ZONE_2=2};
constexpr float START_ZONE_HEADING=180;
struct Pose {float x=0,y=0,theta=0;};
struct Log { constexpr void println(const char*) {} };
struct Run {
    Log Serial;
    int xLidarPoseMutex=1, currentStartZone=2, calls=0;
    bool releasePoseCompleted=false, locked=false, succeed=true, probeBusy=false;
    bool releasePoseInitialized=false;
    Pose currentPose;
    int calibrations=0,lastWait=0;
    constexpr void CalibrateH30MapHeading(float) {++calibrations;}
    constexpr int xSemaphoreTake(int, int wait) {
        lastWait=wait;
        if (locked) return 0;
        locked=true; return pdTRUE;
    }
    constexpr void xSemaphoreGive(int) { locked=false; }
    constexpr bool PrepareLidarScanPose(uint8_t) {
        ++calls;
        // 真实位置动作会在中途让出CPU；共用入口须先完成初始位姿设置。
        if (currentPose.theta!=180 || currentPose.y!=150
                || currentPose.x!=(currentStartZone==1?2250:150)) return false;
        // 模拟自动动作尚未结束时收到串口命令。
        if (probeBusy && runLidarPoseAction(true, false)) return false;
        return succeed;
    }
ACTION
};
constexpr bool retries() {
    Run r; r.probeBusy=true;
    if (!r.runLidarPoseAction(false, false) || r.calls!=1 || r.locked) return false;
    for (int i=0; i<5; ++i)
        if (!r.runLidarPoseAction(true, false) || r.calls!=1 || r.locked) return false;
    return r.runLidarPoseAction(false, false) && r.calls==1
        && r.calibrations==1 && r.lastWait==portMAX_DELAY;
}
constexpr bool manualFirstAndDebug() {
    Run r;
    r.currentPose={150,150,180}; // Debug位姿由START_ZONE命令设置。
    if (!r.runLidarPoseAction(true, true) || !r.runLidarPoseAction(true, true)
            || r.releasePoseCompleted || r.calls!=2) return false;
    if (!r.runLidarPoseAction(true, false) || r.calls!=3) return false;
    return r.runLidarPoseAction(false, false) && r.calls==3;
}
constexpr bool commandFirst() {
    Run r;r.currentStartZone=1;
    if(!r.runLidarPoseAction(true,false)||r.calls!=1||r.calibrations!=1||r.lastWait!=0)return false;
    r.currentPose={2150,275,180}; // 串口动作完成后的位姿不得被自动入口重置。
    if(!r.runLidarPoseAction(false,false)||r.calls!=1||r.calibrations!=1)return false;
    return r.currentPose.x==2150&&r.currentPose.y==275&&r.lastWait==portMAX_DELAY;
}
constexpr bool failureRetry() {
    Run r; r.succeed=false;
    if (r.runLidarPoseAction(false, false) || r.releasePoseCompleted || r.locked) return false;
    r.succeed=true;
    return r.runLidarPoseAction(true, false) && r.calls==2 && r.releasePoseCompleted;
}
static_assert(retries(), "busy and completed retries must not move twice");
static_assert(manualFirstAndDebug(), "debug may repeat; manual/auto Release share completion");
static_assert(failureRetry(), "failed actions must not be cached as successful");
static_assert(commandFirst(), "command-first initialization must precede movement and survive automatic entry");
'''.replace("ACTION", action)
        compiler = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio")) / (
            "packages/toolchain-xtensa-esp32s3/bin/xtensa-esp32s3-elf-g++.exe")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "lidar.cpp"
            path.write_text(harness, encoding="utf-8")
            result = subprocess.run([str(compiler), "-std=c++14", "-c", str(path),
                                     "-o", str(Path(directory) / "lidar.o")],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
