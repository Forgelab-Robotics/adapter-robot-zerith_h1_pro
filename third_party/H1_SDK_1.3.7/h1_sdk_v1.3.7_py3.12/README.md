# H1 SDK v1.3.7 Python 3.12 Binding

这个目录是基于同级 `robot_SDK` 头文件和静态库自建的 Python 3.12 绑定工程，用来替代官方只交付 Python 3.10 ABI 的 `h1_sdk_v1.3.7_py3.10/lib/lib_h1_sdk_python.so`。

## 目录结构

```text
h1_sdk_v1.3.7_py3.12/
├── CMakeLists.txt
├── README.md
├── example/
│   └── h1_low_level_minimal.py
├── lib/
│   ├── __init__.py
│   └── lib_h1_sdk_python.pyi
├── src/
│   └── bindings.cpp
└── tools/
    └── build_py312.sh
```

编译成功后，扩展模块会输出到：

```text
lib/lib_h1_sdk_python.cpython-312-x86_64-linux-gnu.so
```

导入方式与官方 Python 3.10 包保持一致：

```python
from lib.lib_h1_sdk_python import H1Robot, MotorControlMode, Motor_Control
```

## 构建

推荐直接执行：

```bash
cd /home/mrgh/code/zerith_public_sdk/H1_SDK_1.3.7/h1_sdk_v1.3.7_py3.12
tools/build_py312.sh
```

脚本会在本目录创建本地构建虚拟环境 `.venv-py312-build` 并安装 `pybind11`，不会修改系统 Python 包。

如果你已有 Python 3.12 环境和 `pybind11`，也可以指定：

```bash
PYTHON_BIN=/path/to/python3.12 tools/build_py312.sh
```

## 可行性说明

官方 `h1_sdk_v1.3.7_py3.10` 目录不包含 pybind11 绑定源码，只包含示例、类型桩和预编译 `.so`。Python C 扩展受 CPython ABI 约束，Python 3.10 的 `.so` 不能直接在 Python 3.12 中导入。

本目录采用的方案是重新写一层 pybind11 绑定，直接链接同级 `robot_SDK/lib/librobot_core.a` 和 `robot_SDK/3rd_party` 中的静态依赖。它不依赖官方 Python 3.10 `.so`。

## 注意事项

- `robot_SDK` 中的 `.a` 必须是真实二进制，不可以是 Git LFS 指针文件。
- 本绑定只封装机器人 `robot_SDK`，不封装 `camera_sdk_python`。
- 如果链接阶段提示静态库不是 PIC，说明厂商交付的 `.a` 不能被链接进 Python 共享库，需要向厂商索取 `-fPIC` 构建的静态库或 Python 3.12 官方扩展。
- 如果系统缺少 `libzmq.so` 或 `libefivar.so`，Ubuntu/Debian 可安装 `libzmq3-dev` 和 `libefivar-dev`。
