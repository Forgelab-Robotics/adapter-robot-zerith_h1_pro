# Vendored Zerith Python 3.12 SDK

This directory contains the runtime Python 3.12 bindings used by `packages/zerith_h1_pro`:

- `h1_sdk_v1.3.7_py3.12/lib/lib_h1_sdk_python.cpython-312-x86_64-linux-gnu.so`
- `camera_sdk_python/lib/_camera_client_native.cpython-312-x86_64-linux-gnu.so`

Only the Python runtime packages are vendored here. The full C++ SDK trees (`robot_SDK` and `camera_sdk_cpp`) are not copied into this package to avoid carrying hundreds of MB of static build dependencies.

If you need to rebuild these extensions, use the `tools/build_py312.sh` scripts and pass `ZERITH_SDK_ROOT` pointing to a full `H1_SDK_1.3.7` checkout that includes `robot_SDK` and `camera_sdk_cpp`.
