"""使用目标编译器实际计算解析器断言，无需连接硬件。"""
import os
from pathlib import Path
import subprocess
import unittest


class H30ProtocolTests(unittest.TestCase):
    def test_parser_at_compile_time(self):
        self.compile_fixture("h30_protocol_compile.cpp")

    def test_heading_reference_at_compile_time(self):
        self.compile_fixture("h30_heading_compile.cpp")

    def test_three_second_circular_heading_average(self):
        from native_support import build_native, run_native
        run_native(build_native("heading-average", ["tests/h30_heading_average_native.cpp"]))

    def compile_fixture(self, fixture):
        root = Path(__file__).resolve().parents[1]
        compiler = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio")) / (
            "packages/toolchain-xtensa-esp32s3/bin/xtensa-esp32s3-elf-g++.exe")
        result = subprocess.run([str(compiler), "-std=c++14", "-fsyntax-only",
                                 str(root / "tests" / fixture)],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
