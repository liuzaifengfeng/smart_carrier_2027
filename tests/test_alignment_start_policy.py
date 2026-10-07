"""编译期验证实际 C++ 启动策略，覆盖粗加工区重复启动回归。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class AlignmentStartPolicyTests(unittest.TestCase):
    def test_automatic_alignment_is_not_replaced(self):
        root = Path(__file__).resolve().parents[1]
        compiler = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio")) / (
            "packages/toolchain-xtensa-esp32s3/bin/xtensa-esp32s3-elf-g++.exe")
        self.assertTrue(compiler.is_file(), "需要已安装的 ESP32-S3 编译器")
        code = r'''
#include "alignment_start_policy.h"
constexpr bool verify() {
    const VisionStartMode modes[] = {VisionStartMode::DISC, VisionStartMode::WORK_AREA,
        VisionStartMode::WORK_AREA_LOADED, VisionStartMode::CORNER};
    for (auto owner : modes) {
        if (DecideAlignmentStart(true, false, owner, owner) != AlignmentStartAction::KEEP) return false;
        if (DecideAlignmentStart(true, false, owner, VisionStartMode::NONE) != AlignmentStartAction::KEEP) return false;
        if (DecideAlignmentStart(true, true, owner, owner) != AlignmentStartAction::BUSY) return false;
        if (DecideAlignmentStart(true, true, owner, VisionStartMode::NONE) != AlignmentStartAction::BUSY) return false;
        for (auto request : modes) {
            if (owner != request && DecideAlignmentStart(true, false, owner, request) != AlignmentStartAction::BUSY) return false;
            if (DecideAlignmentStart(false, false, owner, request) != AlignmentStartAction::RESET) return false;
        }
        if (DecideAlignmentStart(true, false, owner, VisionStartMode::DISC_MATERIAL) != AlignmentStartAction::BUSY) return false;
    }
    return DecideAlignmentStart(false, false, VisionStartMode::NONE, VisionStartMode::NONE) == AlignmentStartAction::RESET;
}
static_assert(verify(), "automatic wait/result must survive duplicate starts; manual start remains available");
'''
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "policy.cpp"
            source.write_text(code, encoding="utf-8")
            result = subprocess.run([str(compiler), "-std=c++14", "-I", str(root / "src"),
                                     "-c", str(source), "-o", str(Path(directory) / "policy.o")],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
