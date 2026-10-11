"""PlatformIO 预构建钩子：每次编译前从 JSON 生成固件默认参数。"""
from pathlib import Path
import sys

Import("env")
project_dir = Path(env.subst("$PROJECT_DIR"))
sys.path.insert(0, str(project_dir / "scripts"))
from generate_parameter_defaults import generate

generate(project_dir / "robot_parameters.json",
         project_dir / "src" / "generated_parameter_defaults.h")
