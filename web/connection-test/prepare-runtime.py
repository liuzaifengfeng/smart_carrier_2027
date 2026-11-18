"""Package the existing desktop algorithms for browser-local Python execution."""
from pathlib import Path
import json
import shutil
import ast
import sys

project = Path(__file__).resolve().parent
root = project.parent.parent
sys.path.insert(0, str(root))
import upper_computer
destination = project / "public" / "python"
destination.mkdir(parents=True, exist_ok=True)
for name in ("hybrid_plan.py", "field_geometry.py", "action_group_config.py"):
    shutil.copyfile(root / name, destination / name)
source = (root / "upper_computer.py").read_text(encoding="utf-8")
tree = ast.parse(source)
helpers = ["from __future__ import annotations", "import math"]
for name, value in vars(upper_computer).items():
    if name.isupper() and isinstance(value, (str, int, float, tuple, list, dict)):
        try:
            ast.literal_eval(repr(value))
        except (ValueError, SyntaxError):
            continue
        helpers.append(name + " = " + repr(value))
for node in tree.body:
    if isinstance(node, ast.FunctionDef) and (node.name.startswith(("build_", "validate_")) or node.name == "parameter_matches_stage"):
        helpers.append(ast.get_source_segment(source, node))
(destination / "desktop_helpers.py").write_text("\n\n".join(helpers), encoding="utf-8")
continuous = (root / "continuous_trial.py").read_text(encoding="utf-8")
continuous = continuous.split("class ContinuousTrialPanel")[0]
continuous = continuous.replace("import tkinter as tk", "").replace("from tkinter import ttk, messagebox, filedialog", "")
(destination / "continuous_trial.py").write_text(continuous, encoding="utf-8")
(destination / "src").mkdir(exist_ok=True)
shutil.copyfile(root / "src/field_geometry.def", destination / "src/field_geometry.def")
shutil.copyfile(root / "robot_parameters.json", destination / "robot_parameters.json")
runtime = project / "public" / "pyodide"
runtime.mkdir(exist_ok=True)
for name in ("pyodide.mjs", "pyodide.asm.mjs", "pyodide.asm.wasm", "python_stdlib.zip", "pyodide-lock.json"):
    shutil.copyfile(project / "node_modules/pyodide" / name, runtime / name)
for source_name, output_name in (("pyodide.mjs", "loader.js"), ("pyodide.asm.mjs", "pyodide.asm.js")):
    text = (runtime / source_name).read_text(encoding="utf-8")
    (runtime / output_name).write_text(text.replace("pyodide.asm.mjs", "pyodide.asm.js"), encoding="utf-8")
