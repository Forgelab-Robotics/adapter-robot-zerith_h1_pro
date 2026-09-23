# MJCF（MuJoCo）

仿真用 **`scene.xml` / `H1_PRO.xml` / `meshes/`** 等直接放在 **`assets/mjcf/` 本目录**（与 `README.md` 同级），不再使用额外子包名。

## 资产状态

MJCF 与 mesh 已完成导入并作为本仓交付资产维护。常规仿真、CI 与示例运行均应直接使用本目录内容，不再依赖外部资产仓库。

如需重新导入外部资产，请确认来源版本，并在根 `README.md` 或 MR 中记录变更原因。

## 场景入口

| 文件 | 说明 |
|------|------|
| `scene.xml` | 地面 + 可移动机器人，`attach` 前缀 `item_1/` |
| `H1_PRO.xml` | 可移动机器人 + 双臂主体 |
| `assets` | 模型网格 |

## MJCF 校验

在仓库根目录：

```bash
python scripts/verify_mjcf.py --xml assets/mjcf/scene.xml
```

需要 `pip install mujoco`。

## 与 GLB 的关系

见 `assets/glb/README.md`。仿真入口使用 `assets/mjcf/scene.xml`，不要把 `assets/glb/H1_PRO.xml`（pipeline 归一化产物）误当仿真入口。
