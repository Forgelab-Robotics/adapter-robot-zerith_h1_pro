"""Python wrapper for the Zerith camera SDK native extension."""

from __future__ import annotations

import importlib.machinery
import importlib.util
import sys
from pathlib import Path
from typing import Iterable

def _load_native_camera_client():
    lib_dir = Path(__file__).resolve().parent / "lib"
    candidates = []
    for suffix in importlib.machinery.EXTENSION_SUFFIXES:
        candidates.append(lib_dir / f"_camera_client_native{suffix}")

    native_path = next((path for path in candidates if path.exists()), None)
    if native_path is None:
        built_extensions = sorted(path.name for path in lib_dir.glob("_camera_client_native*.so"))
        raise RuntimeError(
            "Cannot import _camera_client_native. Build the Python 3.12 camera extension first. "
            f"Current interpreter is {sys.version.split()[0]} ({sys.executable}). "
            f"Importable extension suffixes are {importlib.machinery.EXTENSION_SUFFIXES}. "
            f"Built extension files are {built_extensions}. "
            "Run tools/build_py312.sh from camera_sdk_python, or rebuild for the active Python version."
        )

    module_name = "_camera_client_native"
    spec = importlib.util.spec_from_file_location(module_name, native_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Unable to load native camera extension: {native_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module.CameraClient


_NativeCameraClient = _load_native_camera_client()


class CameraClient:
    def __init__(self, grpc_target: str):
        self._client = _NativeCameraClient(grpc_target)

    def start(self) -> None:
        self._client.start()

    def stop(self) -> None:
        self._client.stop()

    def get_latest_frame(self, camera_name: str):
        return self._client.get_latest_frame(camera_name)

    def get_latest_depth(self, camera_name: str):
        return self._client.get_latest_depth(camera_name)

    def get_state(self, camera_names: Iterable[str] | None = None, timeout_sec: float = 5.0):
        from proto import robot_pb2

        payload = self._client.get_state_bytes(list(camera_names or []), float(timeout_sec))
        state = robot_pb2.RecorderStateResponse()
        state.ParseFromString(payload)
        return state
