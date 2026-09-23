# MuJoCo + Zerith H1_PRO （Forge / dora-rs）

本目录放在 **`examples/03_workflows/simulation/`**，与 Zerith H1_PRO 资产、仿真说明同一套工作流；配置对应本仓 **`assets/mjcf/scene.xml`**（`attach` 前缀 `item_1/`）。

## 布局约定

- **`simulator.yaml`** 中 `model_path` 相对本目录指向 **`../../../../assets/mjcf/scene.xml`**（仅依赖本仓，不依赖工作区根）。
- **`dataflow*.yaml`** 里节点 `path` 指向并列克隆的 **`forge_runtime`**（相对本目录六级上到工作区根再进入 `forge_runtime/packages/nodes/...`）。请保证工作区根下 **`forge_runtime`** 与 **`robots`** 并列（与当前 `chenruobing` 布局一致）。

## 约定摘要

- **25 个执行器**：导轨 1 个 position + 身体 2 个 position + 七轴机械臂左右各 7 个 position + 左右夹爪各 2 个position + 颈部 2 个 position + 轮子 2 个 velocity。
- **Action / Proprio 的关节名**：与 MuJoCo joint 名一致（该本体关节数较多，此处不一一列举，具体参照 `simulator.yaml`）。
- **图像**：`item_1/left_d405`、`item_1/right_d405`、`item_1/neck_d435` → `image/left_d405`、`image/right_d405`、`image/neck_d435`。

## 运行（v2 minimal）

新版最小 demo 使用 `/home/mrgh/forge_binary/v2` 下的 `mujoco_sim`、`task_robot`、`test_action_source` 和 `image_viewer`。`joint_state`、`command` 和所有 image 都通过 `task_robot` 管理。

```bash
cd /home/mrgh/code/robots/zerith_h1_pro/examples/03_workflows/simulation/mujoco_H1_PRO
dora build dataflow.minimal_v2.yaml
dora run dataflow.minimal_v2.yaml
```

## 运行（旧 test_action_source）

在 **`forge_runtime` 根目录**已执行 `uv sync --all-packages` 的前提下：

```bash
cd robots/Zerith_H1_PRO/examples/03_workflows/simulation/mujoco_H1_PRO
dora build dataflow.test_action_source.yaml --uv
dora run dataflow.test_action_source.yaml --uv
```

（若当前目录已在 `mujoco_H1_PRO` 内，则无需再 `cd`。）

`--joint-order` 须与 `simulator.yaml` / `task_robot.yaml` 中 **joints 列表顺序**一致。

## 与 policy 端到端

可参照本目录 dataflow 自行复制并改 policy 节点、图像边与关节维；policy 侧需 **21** 维并与本目录命名约定一致。
