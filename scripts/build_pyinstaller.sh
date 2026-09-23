#!/usr/bin/env bash
# Build Zerith H1 Pro as a one-file Dora node with vendored H1 SDK native libraries.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NODE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "==> [robots_zerith_h1_pro] 校验 H1 SDK 二进制..."
ROBOT_SO="${NODE_DIR}/third_party/H1_SDK_1.3.7/h1_sdk_v1.3.7_py3.12/lib/lib_h1_sdk_python"*.so
CAMERA_SO="${NODE_DIR}/third_party/H1_SDK_1.3.7/camera_sdk_python/lib/_camera_client_native"*.so
shopt -s nullglob
_robot=( ${ROBOT_SO} )
_camera=( ${CAMERA_SO} )
shopt -u nullglob
if [[ ${#_robot[@]} -eq 0 ]]; then
  echo "ERROR: 未找到 robot SDK .so，请先构建 third_party/H1_SDK_1.3.7/h1_sdk_v1.3.7_py3.12" >&2
  exit 1
fi
if [[ ${#_camera[@]} -eq 0 ]]; then
  echo "ERROR: 未找到 camera SDK .so，请先构建 third_party/H1_SDK_1.3.7/camera_sdk_python" >&2
  exit 1
fi

echo "==> [robots_zerith_h1_pro] 初始化隔离构建虚拟环境..."
VENV_DIR="${NODE_DIR}/.venv_build"
rm -rf "${VENV_DIR}"

FORGE_ROOT="$(cd "${NODE_DIR}/../../forge/packages" 2>/dev/null && pwd || true)"

if command -v uv >/dev/null 2>&1; then
  uv venv --no-workspace "${VENV_DIR}" --python 3.12
  echo "==> [robots_zerith_h1_pro] 安装包依赖与 PyInstaller (uv)..."
  uv pip install --python "${VENV_DIR}/bin/python" -e "${NODE_DIR}" pyinstaller
elif [[ -n "${FORGE_ROOT}" && -d "${FORGE_ROOT}/common" ]]; then
  python3.12 -m venv "${VENV_DIR}"
  "${VENV_DIR}/bin/python" -m pip install -U pip setuptools wheel -q
  echo "==> [robots_zerith_h1_pro] 安装包依赖与 PyInstaller (pip)..."
  "${VENV_DIR}/bin/pip" install -e "${NODE_DIR}" \
    -e "${FORGE_ROOT}/common" \
    -e "${FORGE_ROOT}/msgs" \
    -e "${FORGE_ROOT}/robot" \
    pyinstaller
else
  echo "ERROR: 需要 uv 或 ../../forge/packages 下的 forge 本地包。" >&2
  exit 1
fi

mkdir -p "${NODE_DIR}/dist" "${NODE_DIR}/build/pyinstaller/robots_zerith_h1_pro"

echo "==> [robots_zerith_h1_pro] PyInstaller 打包..."
(
  cd "${NODE_DIR}"
  "${VENV_DIR}/bin/pyinstaller" \
    --noconfirm \
    --clean \
    --distpath "${NODE_DIR}/dist" \
    --workpath "${NODE_DIR}/build/pyinstaller/robots_zerith_h1_pro" \
    "${SCRIPT_DIR}/zerith_h1_pro.spec"
)

rm -rf "${VENV_DIR}"

DIST_FILE="${NODE_DIR}/dist/robots_zerith_h1_pro"
if [[ -f "${DIST_FILE}" ]]; then
  echo "==> [robots_zerith_h1_pro] 构建成功：${DIST_FILE}"
else
  echo "WARNING: dist/robots_zerith_h1_pro 未找到，请检查 PyInstaller 输出。" >&2
  exit 1
fi
