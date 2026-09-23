#!/usr/bin/env python3
"""Zerith H1 Pro 统一入口：dora 节点（run）。"""

from __future__ import annotations

import multiprocessing
import sys
import time
from pathlib import Path

import typer
from forge_common import get_logger

from .actuators import ACTUATOR_LIMITS

multiprocessing.freeze_support()

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    __package__ = "zerith_h1_pro"

logger = get_logger(__name__)

app = typer.Typer(
    name="robot_zerith_h1_pro",
    help="Zerith H1 Pro dora 节点入口。无子命令时默认运行节点；可用 run / get-mode / set-mode / list-modes / move-actuator。",
    no_args_is_help=False,
)


def _build_driver(
    config_path: str | None,
    *,
    auto_connect: bool,
    include_cameras: bool = True,
    init_on_connect: bool | None = None,
    switch_mode_on_connect: bool | None = None,
    control_mode: str | None = None,
    is_follower: bool | None = None,
):
    from .config import load_config
    from .driver import ZerithH1ProDriver

    config = load_config(config_path=config_path)
    final_init_on_connect = (
        init_on_connect if init_on_connect is not None else config.init_on_connect
    )
    final_switch_mode_on_connect = (
        switch_mode_on_connect
        if switch_mode_on_connect is not None
        else config.switch_mode_on_connect
    )
    driver = ZerithH1ProDriver(
        sdk_root=config.sdk_root,
        robot_ip=config.robot_ip,
        is_follower=config.is_follower if is_follower is None else is_follower,
        init_on_connect=final_init_on_connect,
        control_mode=control_mode if control_mode is not None else config.control_mode,
        switch_mode_on_connect=final_switch_mode_on_connect,
        actuator_order=config.actuator_order,
        positions={
            "initial": config.resolved_position("initial"),
            "reset": config.resolved_position("reset"),
        },
        lifecycle_position_move_seconds=config.lifecycle_position_move_seconds,
        wheel_radius_m=config.wheel_radius_m,
        wheel_track_width_m=config.wheel_track_width_m,
        locomotion_max_vx_mps=config.locomotion_max_vx_mps,
        locomotion_max_wz_radps=config.locomotion_max_wz_radps,
        locomotion_max_wheel_speed_radps=config.locomotion_max_wheel_speed_radps,
        locomotion_command_timeout_seconds=(
            config.locomotion_command_timeout_seconds
        ),
        auto_connect=auto_connect,
        camera_grpc_target=config.camera_grpc_target if include_cameras else None,
        cameras=config.cameras if include_cameras else None,
        image_fps=config.image_fps,
        joint_fps=config.joint_fps,
    )
    return config, driver


def _run_node(config_path: str | None) -> int:
    from .subprocess_node import run_subprocess_dora_node

    return run_subprocess_dora_node(config_path=config_path)


def _send_position_ramp(
    driver,
    actuator: str,
    *,
    start: float,
    target: float,
    move_seconds: float,
    rate_hz: float,
) -> None:
    """按 SDK Python 示例风格，将位置命令插值为多次 JointCommand 下发。"""
    from forge_msgs import JointCommand

    duration = max(float(move_seconds), 0.0)
    rate = max(float(rate_hz), 1.0)
    steps = max(int(duration * rate), 1)
    sleep_seconds = 1.0 / rate
    distance = target - start
    for i in range(1, steps + 1):
        value = start + distance * (i / steps)
        driver.set_command(JointCommand(name=[actuator], position=[value]))
        time.sleep(sleep_seconds)


@app.callback(invoke_without_command=True)
def _main(
    ctx: typer.Context,
    config: str | None = typer.Option(None, "--config", help="YAML 配置文件路径"),
) -> None:
    """无子命令时默认运行 Zerith H1 Pro dora 节点。"""
    if ctx.invoked_subcommand is not None:
        return
    sys.exit(_run_node(config_path=config))


@app.command()
def run(
    config: str | None = typer.Option(None, "--config", help="YAML 配置文件路径"),
) -> None:
    """运行 Zerith H1 Pro dora 节点。"""
    sys.exit(_run_node(config_path=config))


@app.command("get-mode")
def get_mode(
    config: str | None = typer.Option(None, "--config", help="YAML 配置文件路径"),
) -> None:
    """读取当前机器人控制模式。"""
    _, driver = _build_driver(
        config_path=config,
        auto_connect=True,
        include_cameras=False,
        init_on_connect=False,
        switch_mode_on_connect=False,
        is_follower=False,
    )
    try:
        typer.echo(driver.get_current_control_mode())
    finally:
        driver.disconnect()


@app.command("set-mode")
def set_mode(
    mode: str = typer.Argument(..., help="目标模式：uninitialized/low_level/gravity_compensation_level"),
    config: str | None = typer.Option(None, "--config", help="YAML 配置文件路径"),
    wait_timeout: float = typer.Option(3.0, "--wait-timeout", help="等待模式切换超时秒数"),
) -> None:
    """切换机器人控制模式。"""
    _, driver = _build_driver(
        config_path=config,
        auto_connect=True,
        include_cameras=False,
        init_on_connect=False,
        switch_mode_on_connect=False,
        is_follower=False,
    )
    try:
        new_mode = driver.switch_control_mode(mode, wait_timeout=wait_timeout)
        typer.echo(new_mode)
    finally:
        driver.disconnect()


@app.command("move-actuator")
def move_actuator(
    actuator: str = typer.Argument(..., help="要移动的 actuator 名称，如 lift"),
    target: float = typer.Option(..., "--target", help="目标位置，单位使用当前 actuator 状态里的 unit"),
    config: str | None = typer.Option(None, "--config", help="YAML 配置文件路径"),
    control_mode: str = typer.Option(
        "low_level",
        "--control-mode",
        help="安全策略：仅支持 LOW_LEVEL；不允许 gravity compensation 模式",
    ),
    move_seconds: float = typer.Option(5.0, "--move-seconds", help="插值移动秒数"),
    rate_hz: float = typer.Option(100.0, "--rate-hz", help="插值下发频率 Hz"),
    yes: bool = typer.Option(False, "--yes", "-y", help="跳过人工确认"),
) -> None:
    """仅在 LOW_LEVEL 模式下慢速移动单个 position actuator，不自动返回。"""
    if control_mode != "low_level":
        raise typer.BadParameter("move-actuator 安全策略只允许 LOW_LEVEL")

    _, driver = _build_driver(
        config_path=config,
        auto_connect=True,
        include_cameras=False,
        init_on_connect=True,
        switch_mode_on_connect=True,
        control_mode=control_mode,
    )
    try:
        order = driver.actuator_order
        if actuator not in order:
            supported = ", ".join(order)
            raise typer.BadParameter(f"未知 actuator: {actuator}。支持: {supported}")

        limits = ACTUATOR_LIMITS.get(actuator)
        if limits is None:
            raise typer.BadParameter(f"{actuator} 未配置限位，拒绝移动。")
        low, high = limits
        if not (low <= target <= high):
            raise typer.BadParameter(
                f"{actuator} target={target:.6f} 超出 SDK 限位 [{low:.6f}, {high:.6f}]"
            )

        current_mode = driver.get_current_control_mode()
        state = driver.get_state()
        if actuator not in state.name:
            raise RuntimeError(f"状态中未包含 actuator: {actuator}")
        if actuator in {"wheel_left", "wheel_right"}:
            raise typer.BadParameter("move-actuator 只移动 position actuator，不移动轮速。")

        current = float(state.to_np([actuator], "position")[0])
        unit = "meters" if actuator in {"lift", "left_gripper", "right_gripper"} else "radians"
        typer.echo(f"mode: {current_mode}")
        typer.echo(
            f"{actuator}: current={current:.6f}, target={target:.6f}, "
            f"unit={unit}, limit=[{low:.6f}, {high:.6f}]"
        )
        if not yes:
            typer.confirm(
                "确认急停可用、机器人周围安全后继续移动到目标位置？",
                abort=True,
            )

        _send_position_ramp(
            driver,
            actuator,
            start=current,
            target=float(target),
            move_seconds=move_seconds,
            rate_hz=rate_hz,
        )
        typer.echo("done")
    finally:
        driver.disconnect()


@app.command("list-modes")
def list_modes() -> None:
    """列出支持的控制模式。"""
    from .driver import ZerithH1ProDriver

    for mode in ZerithH1ProDriver.supported_control_modes():
        typer.echo(mode)


def main() -> None:
    app()


if __name__ == "__main__":
    main()
