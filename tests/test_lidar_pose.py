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
        harness = r'''
#include <stdint.h>
#include <stddef.h>
constexpr int pdTRUE=1;
struct Log { constexpr void println(const char*) {} };
struct Run {
    Log Serial;
    int xLidarPoseMutex=1, currentStartZone=2, calls=0;
    bool releasePoseCompleted=false, locked=false, succeed=true, probeBusy=false;
    constexpr int xSemaphoreTake(int, int) {
        if (locked) return 0;
        locked=true; return pdTRUE;
    }
    constexpr void xSemaphoreGive(int) { locked=false; }
    constexpr bool PrepareLidarScanPose(uint8_t) {
        ++calls;
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
    return r.runLidarPoseAction(false, false) && r.calls==1;
}
constexpr bool manualFirstAndDebug() {
    Run r;
    if (!r.runLidarPoseAction(true, true) || !r.runLidarPoseAction(true, true)
            || r.releasePoseCompleted || r.calls!=2) return false;
    if (!r.runLidarPoseAction(true, false) || r.calls!=3) return false;
    return r.runLidarPoseAction(false, false) && r.calls==3;
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
