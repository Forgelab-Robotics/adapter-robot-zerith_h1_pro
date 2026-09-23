#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ZERITH_SDK_ROOT="${ZERITH_SDK_ROOT:-$(cd "${SDK_DIR}/.." && pwd)}"
PYTHON_BIN="${PYTHON_BIN:-python3.12}"
BUILD_DIR="${BUILD_DIR:-${SDK_DIR}/build-py312}"

if ! "${PYTHON_BIN}" - <<'PY' >/dev/null 2>&1
import pybind11
PY
then
    VENV_DIR="${VENV_DIR:-${SDK_DIR}/.venv-py312-build}"
    echo "[*] pybind11 not found for ${PYTHON_BIN}; using local build venv: ${VENV_DIR}"
    if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
        "${PYTHON_BIN}" -m venv "${VENV_DIR}"
    fi
    PYTHON_BIN="${VENV_DIR}/bin/python"
    if ! "${PYTHON_BIN}" - <<'PY' >/dev/null 2>&1
import pybind11
PY
    then
        "${PYTHON_BIN}" -m pip install --upgrade pip pybind11
    fi
fi

echo "[*] Python: $("${PYTHON_BIN}" --version)"
echo "[*] SDK root: ${ZERITH_SDK_ROOT}"
echo "[*] Build dir: ${BUILD_DIR}"

cmake -S "${SDK_DIR}" -B "${BUILD_DIR}" \
    -DZERITH_SDK_ROOT="${ZERITH_SDK_ROOT}" \
    -DPython3_EXECUTABLE="$(command -v "${PYTHON_BIN}")"

cmake --build "${BUILD_DIR}" -j"$(nproc)"

"${PYTHON_BIN}" - <<PY
import sys
sys.path.insert(0, "${SDK_DIR}")
from camera_client import CameraClient

client = CameraClient("localhost:50051")
print("[*] import ok:", CameraClient, client)
PY
