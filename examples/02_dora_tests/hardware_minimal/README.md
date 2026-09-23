# Zerith H1 Pro Hardware Minimal

这个 demo 将真机 `JointState` 和相机 `Image` / `CompressedImage` 全部先送入 Forge runtime 的 `task_robot` 节点，并用本目录下的 `test/policy_action_source.py` 生成小幅夹爪动作来验证 policy -> task_robot -> robot 控制链路。

当前 H1 robot SDK 与 camera SDK 同进程加载会触发 native `exit 139`，因此这个硬件最小 demo 使用单 Dora 节点 + 内部 subprocess 隔离：

- `zerith_h1_pro.main`：对 Dora 暴露一个节点，统一输出 `joint_state`、`image/left`、`image/right`、`image/high`。
- robot worker subprocess：只加载 H1 robot SDK，负责状态采集和命令下发。
- camera worker subprocess：只加载 camera SDK，负责图像采集和编码。

这样 Dora 图中只有一个 Zerith 输出节点，同时避免两个 native SDK 在同一 Python 进程中共存。默认不启动 `image_viewer`，以避免无图形/OpenGL 环境中的 glutin 初始化错误；如需看图像可单独加 viewer 节点。

运行前修改 `zerith_h1_pro.yaml` 中的 `robot_ip` 和相机 `name`；相机地址自动使用 `robot_ip:50051`。无需配置 `sdk_root`（打包节点已内置 H1_SDK_1.3.7）。

先构建 Dora 二进制节点：

```bash
cd <本仓库根目录>
./scripts/build_pyinstaller.sh
```

再运行 hardware minimal demo（`dataflow.yaml` 使用 `dist/robots_zerith_h1_pro`）：

```bash
cd <本仓库根目录>/examples/02_dora_tests/hardware_minimal
uv sync --project ../../..
dora build dataflow.yaml --uv
dora run dataflow.yaml --uv
```

源码调试时可将 `dataflow.yaml` 中 `zerith_h1_pro.path` 改回 `../../../src/robots_zerith_h1_pro/main.py`。

## 关节与夹爪小幅位移测试

使用 `test/test_actuator_motion.py` 直连真机，按当前状态对每个 position actuator 做小幅插值往返测试。wheel 保留在通用合同中但不属于 position sweep；该测试覆盖 lift、腰部、头部、双臂和左右夹爪：

```bash
uv run --project ../../.. python test/test_actuator_motion.py --config zerith_h1_pro.yaml
```

默认位移较小：转动关节 `0.03rad`，lift `0.02m`，夹爪 `0.01m`。如需进一步减小夹爪活动范围：

```bash
uv run --project ../../.. python test/test_actuator_motion.py --config zerith_h1_pro.yaml --gripper-displacement 0.005
```

## 轮子 low_level 前后 0.5m 测试

轮子位移测试通过通用 wheel `JointCommand` 同时发送左右轮 velocity，并持续刷新以满足共享 watchdog；根据 wheel position 反馈与轮径估算前进 `0.5m`，停止，再后退 `0.5m`。默认轮半径使用交付模型的 `0.0835m`，真机验收后应替换为实测有效滚动半径。SDK 文档未明确 wheel `Speed` 单位，因此第一次只能在驱动轮架空且急停可用时运行，并对比 `Speed_Actual` 与 wheel position 导数；确认单位、比例和正负号之前不得落地：

```bash
uv run --project ../../.. python test/test_wheel_distance.py --config zerith_h1_pro.yaml
```

如果实际轮半径不同，调整 `--wheel-radius-m`；如需更保守，可以降低轮速：

```bash
uv run --project ../../.. python test/test_wheel_distance.py --config zerith_h1_pro.yaml --distance-m 0.5 --wheel-speed-radps 0.6 --wheel-radius-m 0.0835
```

如果 v2 二进制没有执行权限，先对 Forge runtime 的 `task_robot` 和 `image_viewer` 加执行权限。
