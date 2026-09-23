# 03 — Forge 端到端 workflow

**仿真配置示例**：`simulation/mujoco_H1_PRO/`（`dora` 配置 + README；节点脚本指向并列克隆的 `forge_runtime`）。模型路径仅依赖本仓 **`assets/mjcf/scene.xml`**。

自建时可复制 `simulation/mujoco_H1_PRO/` 作为基线，将 `model_path` 指向本仓 **`assets/mjcf/scene.xml`**，并按模型中 `attach` 的 `prefix`（`item_1/`）配置 MuJoCo 与 TaskRobot 的 `robots[].prefix` 及关节列表。

仿真节点配置骨架见 `simulation/simulator.example.yaml`（完整示例以 **`mujoco_H1_PRO/simulator.yaml`** 为准）。
