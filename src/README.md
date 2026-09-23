# Forge 机器人节点源码（`src/`）

本目录是 零次方 Zerith H1_PRO 专用 Forge 节点源码的归宿。原则上：

- Zerith H1_PRO真机节点、专用适配层、消息转换、设备 SDK 封装等源码放在 `src/H1_PRO/`（或约定包名）；
- 打包、校验、资产导入等命令放在 `scripts/`；
- 端口、相机、关节顺序、VR 标定等参数放在 `config/`；
- dataflow / workflow 示例放在 `examples/`。

当前仿真示例复用 `forge_runtime` 的通用 MuJoCo / TaskRobot 节点，暂无独立 H1_PRO 节点源码落盘。如后续仍复用外部实现，请在本文件记录外部仓库、commit / 版本、入口命令与验收方式；如迁入本仓，请同步更新根 `README.md` 的目录说明。
