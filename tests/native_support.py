"""编译并运行实际 C++ 控制代码；产物放在忽略的 .pio 目录。"""
import os
from pathlib import Path
import shutil
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]
_built = {}


def build_native(name, sources):
    key = (name, tuple(sources))
    if key in _built:
        return _built[key]
    compiler = os.environ.get("CONTINUOUS_TEST_CXX") or shutil.which("g++")
    zig = ROOT / ".pio/host-test-tools/ziglang/zig.exe"
    if not compiler and not zig.is_file():
        raise unittest.SkipTest("需要主机 C++ 编译器")
    command = [compiler] if compiler else [str(zig), "c++"]
    cache = ROOT / ".pio/motion-native"
    cache.mkdir(parents=True, exist_ok=True)
    exe = cache / (name + (".exe" if os.name == "nt" else ""))
    env = dict(os.environ)
    env["PATH"] = str(Path(command[0]).parent) + os.pathsep + env.get("PATH", "")
    # Zig优化构建可能定义NDEBUG；原生测试必须保留assert及其中的执行调用。
    result = subprocess.run(command + ["-std=c++14", "-O1", "-UNDEBUG", "-I", str(ROOT / "src"),
                            "-I", str(ROOT / "tests/native_stubs"), "-I", str(cache), *[str(ROOT / s) for s in sources],
                            "-o", str(exe)], capture_output=True, text=True, encoding="utf-8", errors="replace", env=env)
    if result.returncode:
        raise AssertionError(result.stdout + result.stderr)
    _built[key] = (exe, env)
    return _built[key]


def run_native(executable, *arguments):
    exe, env = executable
    result = subprocess.run([str(exe), *arguments], capture_output=True, text=True,
                            encoding="utf-8", errors="replace", env=env, timeout=30)
    if result.returncode:
        raise AssertionError(result.stdout + result.stderr + f"\nexit={result.returncode}")
    return result.stdout
