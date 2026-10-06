"""用 ESP32 C++ 编译器常量求值检查实际恢复逻辑，无需连接硬件。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class DiscResumeTests(unittest.TestCase):
    def test_resume_under_continuous_input_and_partial_frames(self):
        root = Path(__file__).resolve().parents[1]
        compiler = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio")) / (
            "packages/toolchain-xtensa-esp32s3/bin/xtensa-esp32s3-elf-g++.exe")
        self.assertTrue(compiler.is_file(), "需要已安装的 ESP32-S3 编译器")
        source = (root / "src/serial_commands.inc").read_text(encoding="utf-8")
        start = source.index("    const auto resumeDiscAtFrameBoundary = [&]() {")
        end = source.index("\n    };", start) + len("\n    };")
        actual_logic = source[start:end]
        harness = r'''
#include <stddef.h>
struct FakeSerial {
    int pending;
    int logs = 0;
    constexpr int available() const { return pending; }
    template<class... Args>
    constexpr void printf(const char*, Args...) { ++logs; }
};
enum State { STATE_GRAB_ROUND1, OTHER };
constexpr bool check(int mode) {
    FakeSerial Serial{20};
    bool discMessageResumeRequested = true, discResumeDraining = false;
    size_t discOldBytesRemaining = 0, length = 0;
    bool overflow = false, invalid = false, firstDiscGrabReady = false;
    bool serialDebugMode = mode == 3, discMaterialActive = mode != 4;
    int currentState = mode == 5 ? OTHER : STATE_GRAB_ROUND1;
    int roundProgress = mode == 6 ? 3 : 1, discMaterialColor = 4;
    struct { int round1_colors[3]; } currentTask{{4, 1, 2}};
LOGIC
    resumeDiscAtFrameBoundary();
    if (!discResumeDraining || discOldBytesRemaining != 20 || firstDiscGrabReady) return false;
    // 新数据始终存在；只消耗请求到达时的固定旧数据，不等 available()==0。
    Serial.pending = 100;
    length = 1;
    for (int i = 0; i < 20; ++i) {
        --discOldBytesRemaining;
        resumeDiscAtFrameBoundary();
        if (firstDiscGrabReady) return false;
    }
    // 旧数据结束于半帧：直到该帧终止并重置解析状态才能恢复。
    length = 0;
    overflow = mode == 1;
    invalid = mode == 2;
    resumeDiscAtFrameBoundary();
    if ((mode == 1 || mode == 2) && (!discResumeDraining || firstDiscGrabReady)) return false;
    overflow = invalid = false;
    resumeDiscAtFrameBoundary();
    bool allowed = mode < 3;
    if (discResumeDraining || discMessageResumeRequested || discMaterialColor != 0
            || firstDiscGrabReady != allowed || Serial.logs != (allowed ? 1 : 0)) return false;
    resumeDiscAtFrameBoundary();
    return Serial.logs == (allowed ? 1 : 0); // 恢复仅发生一次。
}
static_assert(check(0), "continuous input and partial frame");
static_assert(check(1), "overflow waits for frame reset");
static_assert(check(2), "invalid frame waits for reset");
static_assert(check(3), "debug does not resume automatic grab");
static_assert(check(4), "stopped vision does not resume");
static_assert(check(5), "changed task state does not resume");
static_assert(check(6), "completed round does not resume");
'''.replace("LOGIC", actual_logic)
        with tempfile.TemporaryDirectory() as directory:
            fixture = Path(directory) / "disc_resume.cpp"
            fixture.write_text(harness, encoding="utf-8")
            result = subprocess.run([str(compiler), "-std=c++17", "-fsyntax-only", str(fixture)],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
