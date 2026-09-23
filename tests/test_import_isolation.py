from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT / "src"
NATIVE_OR_CAMERA_MODULES = (
    "cv2",
    "camera_client",
    "lib.lib_h1_sdk_python",
)


def _assert_clean_import(module_name: str, *, driver_loaded: bool = False) -> None:
    script = f"""
import importlib
import sys
sys.path.insert(0, {str(SOURCE_ROOT)!r})
importlib.import_module({module_name!r})
for forbidden in {NATIVE_OR_CAMERA_MODULES!r}:
    assert forbidden not in sys.modules, (forbidden, sorted(sys.modules))
assert ("zerith_h1_pro.driver" in sys.modules) is {driver_loaded!r}, sorted(sys.modules)
"""
    subprocess.run(
        [sys.executable, "-c", script],
        cwd=PACKAGE_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )


@pytest.mark.parametrize(
    ("module_name", "driver_loaded"),
    (
        ("zerith_h1_pro", False),
        ("zerith_h1_pro.config", False),
        ("zerith_h1_pro.subprocess_node", False),
        ("zerith_h1_pro.workers", False),
        ("zerith_h1_pro.camera_driver", False),
        ("zerith_h1_pro.driver", True),
    ),
)
def test_imports_do_not_eagerly_load_native_or_camera_modules(
    module_name: str,
    driver_loaded: bool,
) -> None:
    _assert_clean_import(module_name, driver_loaded=driver_loaded)


def test_package_driver_export_is_lazy_and_compatible() -> None:
    script = f"""
import sys
sys.path.insert(0, {str(SOURCE_ROOT)!r})
import zerith_h1_pro
assert "zerith_h1_pro.driver" not in sys.modules
from zerith_h1_pro import ZerithH1ProDriver
assert ZerithH1ProDriver.__name__ == "ZerithH1ProDriver"
assert "zerith_h1_pro.driver" in sys.modules
assert "cv2" not in sys.modules
assert "camera_client" not in sys.modules
assert "lib.lib_h1_sdk_python" not in sys.modules
"""
    subprocess.run(
        [sys.executable, "-c", script],
        cwd=PACKAGE_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
