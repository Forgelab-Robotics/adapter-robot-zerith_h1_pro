# -*- mode: python ; coding: utf-8 -*-
# PyInstaller spec for Zerith H1 Pro Dora node (one-file bundle with vendored H1 SDK).

import os
import shutil
from pathlib import Path

from PyInstaller.utils.hooks import collect_submodules

_spec_dir = os.path.dirname(os.path.abspath(SPEC))
_repo_root = os.path.abspath(os.path.join(_spec_dir, ".."))
_sdk_root = Path(_repo_root) / "third_party" / "H1_SDK_1.3.7"


def _collect_sdk_datas() -> list[tuple[str, str]]:
    """Bundle H1 robot + camera Python SDK files under third_party/H1_SDK_1.3.7/."""
    if not _sdk_root.is_dir():
        raise FileNotFoundError(f"H1 SDK not found: {_sdk_root}")

    rel_paths = [
        "h1_sdk_v1.3.7_py3.12/lib",
        "camera_sdk_python/lib",
        "camera_sdk_python/proto",
        "camera_sdk_python/camera_client.py",
    ]
    datas: list[tuple[str, str]] = []
    prefix = "third_party/H1_SDK_1.3.7"

    for rel in rel_paths:
        src = _sdk_root / rel
        if src.is_file():
            dest_dir = str(Path(prefix) / src.parent.relative_to(_sdk_root))
            datas.append((str(src), dest_dir))
            continue
        if not src.is_dir():
            raise FileNotFoundError(f"Missing SDK path: {src}")
        for path in sorted(src.rglob("*")):
            if not path.is_file():
                continue
            dest_dir = str(
                Path(prefix) / path.parent.relative_to(_sdk_root)
            )
            datas.append((str(path), dest_dir))
    return datas


def _collect_camera_runtime_binaries() -> list[tuple[str, str]]:
    """Bundle PyAV's FFmpeg 8 runtime and aliases required by camera native SDK."""
    import av

    libs_dir = Path(av.__file__).resolve().parent.parent / "av.libs"
    if not libs_dir.is_dir():
        raise FileNotFoundError(f"PyAV runtime libraries not found: {libs_dir}")

    libraries = sorted(path for path in libs_dir.glob("*.so*") if path.is_file())
    binaries = [(str(path), ".") for path in libraries]
    alias_dir = Path(_repo_root) / "build" / "pyinstaller" / "camera_runtime_aliases"
    alias_dir.mkdir(parents=True, exist_ok=True)

    aliases = {
        "libavcodec.so.62": "libavcodec-*.so.62*",
        "libavutil.so.60": "libavutil-*.so.60*",
        "libswscale.so.9": "libswscale-*.so.9*",
    }
    for alias, pattern in aliases.items():
        matches = sorted(libs_dir.glob(pattern))
        if len(matches) != 1:
            raise FileNotFoundError(
                f"Expected exactly one PyAV runtime for {alias}, found: {matches}"
            )
        alias_path = alias_dir / alias
        shutil.copy2(matches[0], alias_path)
        binaries.append((str(alias_path), "."))
    return binaries


def _collect_package_datas() -> list[tuple[str, str]]:
    """Map src/robots_zerith_h1_pro -> zerith_h1_pro/ for setuptools package-dir alias."""
    pkg_src = Path(_repo_root) / "src" / "robots_zerith_h1_pro"
    if not pkg_src.is_dir():
        raise FileNotFoundError(f"Package source not found: {pkg_src}")

    datas: list[tuple[str, str]] = []
    for path in sorted(pkg_src.rglob("*")):
        if not path.is_file():
            continue
        if path.suffix not in {".py", ".pyi"}:
            continue
        dest_dir = str(Path("zerith_h1_pro") / path.parent.relative_to(pkg_src))
        datas.append((str(path), dest_dir))
    return datas


def _collect_zerith_hiddenimports() -> list[str]:
    pkg_src = Path(_repo_root) / "src" / "robots_zerith_h1_pro"
    names = ["zerith_h1_pro"]
    for path in sorted(pkg_src.rglob("*.py")):
        if path.name == "__init__.py":
            continue
        rel = path.relative_to(pkg_src).with_suffix("")
        names.append("zerith_h1_pro." + ".".join(rel.parts))
    return names


hiddenimports = (
    _collect_zerith_hiddenimports()
    + collect_submodules("forge_msgs")
    + collect_submodules("forge_common")
    + collect_submodules("forge_robot")
    + [
        "dora",
        "numpy",
        "pyarrow",
        "yaml",
        "typer",
        "cv2",
        "google.protobuf",
    ]
)

a = Analysis(
    [os.path.join(_spec_dir, "dora_entry.py")],
    pathex=[_repo_root, os.path.join(_repo_root, "src")],
    binaries=_collect_camera_runtime_binaries(),
    datas=_collect_sdk_datas() + _collect_package_datas(),
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name="robots_zerith_h1_pro",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
