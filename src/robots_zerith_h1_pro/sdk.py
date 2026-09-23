"""Lazy H1 robot SDK loading and stateless control-mode helpers."""

from __future__ import annotations

import importlib
import sys
from pathlib import Path
from types import ModuleType

MODE_NAME_TO_SDK_ATTR = {
    "uninitialized": "UNINITIALIZED",
    "low_level": "LOW_LEVEL",
    "gravity_compensation_level": "GRAVITY_COMPENSATION_LEVEL",
}

MODE_ALIASES = {
    "uninit": "uninitialized",
    "low": "low_level",
    "lowlevel": "low_level",
    "gravity": "gravity_compensation_level",
    "gravity_compensation": "gravity_compensation_level",
    "gravity_compensation_mode": "gravity_compensation_level",
    "gc": "gravity_compensation_level",
}


def resolve_sdk_python_root(sdk_root: str) -> Path:
    root = Path(sdk_root).expanduser().resolve()
    version_tag = f"py{sys.version_info.major}.{sys.version_info.minor}"
    candidates = [
        root,
        root / f"h1_sdk_v1.3.7_{version_tag}",
        root / "h1_sdk_v1.3.7_py3.12",
        root / "h1_sdk_v1.3.7_py3.10",
    ]

    for candidate in candidates:
        if any((candidate / "lib").glob("lib_h1_sdk_python*.so")):
            return candidate

    raise FileNotFoundError(
        "未找到 H1 Python SDK 绑定。期望在以下目录之一找到 "
        f"lib/lib_h1_sdk_python*.so: {', '.join(str(p) for p in candidates)}"
    )


def load_h1_sdk(sdk_root: str) -> ModuleType:
    py_root = resolve_sdk_python_root(sdk_root)
    py_root_str = str(py_root)
    if py_root_str not in sys.path:
        sys.path.insert(0, py_root_str)

    try:
        return importlib.import_module("lib.lib_h1_sdk_python")
    except Exception as exc:
        raise RuntimeError(
            "导入 H1 Python SDK 失败。"
            "请确认 Python 版本与 SDK so 匹配，且 sdk_root 指向 H1_SDK_1.3.7 或对应 Python SDK 目录。"
        ) from exc


def normalize_mode_name(mode: str) -> str:
    normalized = mode.strip().lower()
    normalized = MODE_ALIASES.get(normalized, normalized)
    if normalized not in MODE_NAME_TO_SDK_ATTR:
        supported = ", ".join(MODE_NAME_TO_SDK_ATTR.keys())
        raise ValueError(f"未知 control_mode: {mode}。支持: {supported}")
    return normalized


def mode_name_from_value(mode_value: int, sdk: ModuleType) -> str:
    enum_cls = sdk.MotorControlMode
    for mode_name, enum_attr in MODE_NAME_TO_SDK_ATTR.items():
        enum_val = int(getattr(enum_cls, enum_attr))
        if int(mode_value) == enum_val:
            return mode_name
    return f"unknown({int(mode_value)})"
