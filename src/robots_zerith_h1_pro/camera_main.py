#!/usr/bin/env python3
"""Zerith H1 Pro camera-only Dora node."""

from __future__ import annotations

import sys
from pathlib import Path

import typer
from dora import Node
from forge_common import get_logger

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    __package__ = "zerith_h1_pro"

from .camera_driver import ZerithH1ProCameraDriver
from .config import load_config

logger = get_logger(__name__)

app = typer.Typer(
    name="robot_zerith_h1_pro_camera",
    help="Zerith H1 Pro camera-only Dora node.",
    no_args_is_help=False,
)


def _run_node(config_path: str | None) -> int:
    config = load_config(config_path=config_path)
    cameras = list(config.cameras or [])
    driver = ZerithH1ProCameraDriver(
        sdk_root=config.sdk_root,
        camera_grpc_target=config.camera_grpc_target,
        cameras=cameras,
        image_fps=config.image_fps,
        auto_connect=True,
    )
    node = Node()

    try:
        for event in node:
            match event.get("type"):
                case "INPUT":
                    if event.get("id") != "tick":
                        continue
                    for camera in cameras:
                        if camera.enable_color:
                            payload = driver.get_latest_encoded_image(camera.output_id)
                            if payload is not None:
                                node.send_output(camera.output_id, payload)
                        if camera.enable_depth and camera.depth_output_id:
                            payload = driver.get_latest_encoded_image(camera.depth_output_id)
                            if payload is not None:
                                node.send_output(camera.depth_output_id, payload)
                case "STOP":
                    break
                case "ERROR":
                    logger.error("camera node received ERROR: %s", event.get("error"))
                    break
                case _:
                    pass
    finally:
        driver.disconnect()
    return 0


@app.callback(invoke_without_command=True)
def _main(
    ctx: typer.Context,
    config: str | None = typer.Option(None, "--config", help="YAML 配置文件路径"),
) -> None:
    if ctx.invoked_subcommand is not None:
        return
    sys.exit(_run_node(config_path=config))


@app.command()
def run(
    config: str | None = typer.Option(None, "--config", help="YAML 配置文件路径"),
) -> None:
    sys.exit(_run_node(config_path=config))


def main() -> None:
    app()


if __name__ == "__main__":
    main()
