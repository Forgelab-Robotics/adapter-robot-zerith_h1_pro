# Zerith H1 Pro

[查看机器人构型说明](description.md)

Zerith H1 Pro 的独立 Forge robot 交付包，包含真机 driver、Python 3.12 SDK runtime、基础 CLI，以及最小 Dora 真机/仿真 demo。

## 快速检查

```bash
uv sync
uv run robots-zerith-h1-pro --help
uv run python -c "from zerith_h1_pro.driver import ZerithH1ProDriver"
```

真机配置从 `config/config.yaml` 或 `config/config.example.yaml` 开始修改。`sdk_root` 可省略：开发时默认使用本包 `third_party/H1_SDK_1.3.7`；PyInstaller 节点内置同版本 SDK。

`move-actuator` 是有意限制为 **LOW_LEVEL-only** 的真机安全命令；传入 `--control-mode gravity_compensation_level` 会在构造 driver 或加载 SDK 前直接拒绝，不会自动恢复或切换到 gravity compensation 模式。

源码按职责拆分：`config.py` 管公开配置与默认值，`actuators.py` 管执行器元数据/换算，`sdk.py` 与 `camera_io.py` 管惰性 SDK I/O，`ipc.py` 管纯 IPC 编解码，`workers.py` 管子进程 worker，`subprocess_node.py` 只管父进程 Dora 编排，状态与安全机仍完整保留在 `driver.py`。

### 生命周期位姿合同

`positions` 只接受 `initial` 和 `reset`。follower 连接初始化后移动到 `initial`；断开时先停止底盘 locomotion，确认停车后再通过 `move_to_reset_position()` 移动到 `reset`，最后释放控制。

### 夹爪方向合同

厂商 H1 SDK 1.3.7 示例定义 `MOTOR_LEFT_ARM_8` / `MOTOR_RIGHT_ARM_8` 的 `Position=0.0` 为张开、`Position=1.5` 为闭合。Forge 对外统一使用物理 opening：`0.08 m` 张开、`0.0 m` 闭合，因此 driver 会做反向线性换算：

| Forge opening | SDK Position | 状态 |
| ---: | ---: | --- |
| `0.08 m` | `0.0` | 张开 |
| `0.0 m` | `1.5` | 闭合 |

低层命令和状态继续使用厂商低层示例采用的 `setMotorControl_low` / `getMotorState`；反馈速度同步转换为米每秒并反向，使正速度始终表示 opening 增大。

### 移动底盘合同

通用 `JointCommand`/`JointState` 合同继续保留 `wheel_left`、`wheel_right` 两项 velocity actuator，加上 21 项 position actuator 共 23 项。节点同时接受标准 Forge `locomotion_command` 输入（`forge_msgs.LocomotionCommand`）：`vx` 为前向米每秒，`vy` 不受差速底盘支持并强制为 `0`，`wz` 为逆时针正方向弧度每秒。follower 只在实际 SDK 模式为 `LOW_LEVEL` 时动作，底盘只调用双轮 `setChassis_low`，不会使用厂商高层移动 API。

`vx/wz` 先按配置限速，再按差速模型换算轮速；混合命令超过轮速上限时按同一比例缩放左右轮以保持曲率。直接 wheel `JointCommand` 必须同时提供左右轮 velocity。两条入口共用 LOW_LEVEL wheel write、模式锁和 watchdog；非零命令必须以快于 `locomotion_command_timeout_seconds` 的频率持续刷新，显式零命令和 `disconnect()` 也会给双轮发送零速。

```yaml
wheel_radius_m: 0.0835
wheel_track_width_m: 0.379
locomotion_max_vx_mps: 0.08
locomotion_max_wz_radps: 0.4
locomotion_max_wheel_speed_radps: 1.0
locomotion_command_timeout_seconds: 0.25
```

轮径和轮距来自交付模型，真机启用前必须校验有效滚动半径、转向方向和打滑影响。vendored SDK 只把 wheel 字段称为 `Speed`，没有明确写出单位，也未声明示例使用的 `±1` 是硬件上限；当前按 wheel rad/s 换算，并把 `1.0` 作为项目保守安全上限，必须通过架空轮的 `Speed_Actual` 与位置导数实测确认。Dora 中应将 `LocomotionCommand` source 直接连接到 Zerith 节点的 `locomotion_command`；底盘命令使用独立 latest-only IPC queue，并优先于每轮最多一条关节命令处理。IPC 接收时间计入 watchdog freshness，过期非零命令不会重新启动底盘。

## 打包 Dora 节点

```bash
./scripts/build_pyinstaller.sh
# 产物: dist/robots_zerith_h1_pro
dist/robots_zerith_h1_pro --help
```

## Dora Demo

- 真机最小 demo：`examples/02_dora_tests/hardware_minimal`
- 仿真最小 demo：`examples/03_workflows/simulation/mujoco_H1_PRO/dataflow.minimal_v2.yaml`

两个 demo 都使用 Forge runtime 提供的二进制节点，并让 image/state 先进入 `task_robot` 后再输出到 viewer 或下游节点。

真机最小 demo 对 Dora 只暴露一个 `zerith_h1_pro` 节点。当前 H1 robot SDK 与 camera SDK 同进程加载会触发 native `exit 139`，所以 `src/robots_zerith_h1_pro/main.py` 内部启动 robot/camera 两个 subprocess 隔离 SDK，由父进程统一输出 `joint_state` 和 image topic 到 `task_robot`。`src/robots_zerith_h1_pro/camera_main.py` 保留为相机单独排障入口，不作为默认 demo 链路。
# 零次方 Zerith H1_PRO（Forge 机器人交付包）

## 交付内容

| 内容 | 路径 |
|------|------|
| **MJCF + meshes** | `assets/mjcf/`，仿真入口为 `scene.xml`，主模型为 `H1_PRO.xml` |
| **URDF** | `assets/urdf`，可用可视化工具（`https://viewer.robotsfan.com/`）查看
| **GLB / 可视化资产** | `assets/glb/`，由资产流水线生成
| **仿真配置示例** | `examples/03_workflows/simulation/mujoco_H1_PRO/` |
| **配置与标定模板** | `config/` |

`forge_runtime` 当前未发现上游内置的 零次方 Zerith H1_PRO 专用 `mujoco_assets` 示例；本仓已在 `examples/03_workflows/simulation/mujoco_H1_PRO/` 提供仿真配置示例，其 `simulator.yaml` 的 `model_path` 指向本仓 `assets/mjcf/scene.xml`。

## 目录结构

| 路径 | 说明 |
|------|------|
| `assets/urdf/` | URDF (规范落盘) |
| `assets/mjcf/` | MJCF、mesh、`scene.xml`、模型图片等 |
| `assets/glb/` | Web / 可视化 GLB |
| `examples/01_sdk_tests/` | 直连厂商/底层驱动的最小测试（与 Forge 解耦） |
| `examples/02_dora_tests/` | Dora 节点与 Topic 集成测试 |
| `examples/03_workflows/` | 端到端 Forge workflow（仿真 / 真机 / 同构 / VR） |
| `src/` | Forge 规范机器人节点源码（见下方说明） |
| `scripts/` | MJCF 校验、Dora 节点打包 |
| `config/` | 端口、相机、关节顺序、VR 标定等 |

## 资产状态

零次方 Zerith H1_PRO 的 MJCF、mesh 与 GLB 已完成同步并落盘到本仓，后续仿真与交付以本仓 `assets/` 下内容为准：

- `assets/mjcf/`：MuJoCo 仿真入口，使用 `scene.xml`，`attach` 前缀为 `item_1/`
- `assets/glb/H1_PRO.glb`：Web / 可视化交付资产（pipeline 产物）
- `assets/urdf/`：URDF与mjcf对应，可直接可视化

零次方 Zerith H1_PRO 的 MJCF、mesh 与 GLB 已完成同步并落盘到本仓，后续仿真与交付以本仓 `assets/` 下内容为准；常规开发、CI 与仿真示例不再依赖外部资产仓库。

## 仿真接入流程

分步说明、与 `forge_runtime` 对接要点及复现命令见：

**`examples/03_workflows/simulation/SIMULATION_INTEGRATION.md`**

摘要：资产齐 → `verify_mjcf` → 对齐约定 → 运行本仓 **`examples/03_workflows/simulation/mujoco_H1_PRO/`**（`model_path` 指本仓 `scene.xml`，`prefix: item_1/`，25 个 actuator 与 `task_robot` 一致）→ 自动 sweep + 人工观察 smoke check → 与真机对表。检查记录见 `examples/03_workflows/simulation/JOINT_SWEEP_CHECKLIST.md`。

## `src/` 与 Forge 节点

Forge / H1_PRO 专用节点源码应放在 `src/`，打包、校验、资产生成等辅助命令放在 `scripts/`。当前仿真示例复用 `forge_runtime` 的通用 MuJoCo / TaskRobot 节点；若后续增加真机节点、H1_PRO 专用适配层或 submodule，请在 `src/README.md` 记录实现位置、版本与入口。

## 交付与验收（摘要）

当前交付覆盖仿真资产与 Forge/Dora 仿真 workflow。真机节点、VR/同构参数、实机方向/零位/限位复核属于后续阶段；完成后在 `config/`、`examples/03_workflows/` 或 MR 中登记路径与命令。
