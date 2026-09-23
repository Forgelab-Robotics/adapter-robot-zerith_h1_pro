# 配置与标定

- **仿真 / 真机**：关节顺序、左右臂命名前缀、`item_1/` 等与 MuJoCo 模型一致的字段，放在此目录的 YAML 中（勿提交密钥）。
- **VR / 同构**：填写 `vr_calibration.yaml.example` 后另存为 `vr_calibration.yaml`（敏感标定勿入库）。

零次方 Zerith H1_PRO 含移动底盘、头部、双臂与夹爪，标定与验收需分系统记录。夹爪 YAML 值使用 opening 米制语义：`0.08` 张开、`0.0` 闭合；driver 内部反向换算为厂商 SDK 的 `Position=0.0` 张开、`Position=1.5` 闭合。

移动底盘必须显式配置 `wheel_radius_m`、`wheel_track_width_m`、`locomotion_max_vx_mps`、`locomotion_max_wz_radps`、`locomotion_max_wheel_speed_radps` 和 `locomotion_command_timeout_seconds`。示例中的 `0.0835m` 轮径与 `0.379m` 轮距来自交付模型；SDK 未明确说明 wheel `Speed` 的单位或上限，当前按 rad/s 换算并把 `1.0` 设为项目保守上限。真机验收需架空轮对比 `Speed_Actual` 与位置导数、测量有效滚动半径，并确认 `+vx` 前进、`+wz` 逆时针。wheel 保留在通用 `actuator_order`/`JointCommand` velocity 合同中，也可通过标准 `locomotion_command` 控制；两条路径共用实际 `LOW_LEVEL` 写入、锁和 watchdog。
