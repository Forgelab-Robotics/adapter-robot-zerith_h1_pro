#!/usr/bin/env python3
"""Read Zerith H1 Pro joint state and camera images through the integrated driver."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from zerith_h1_pro.config import load_config
from zerith_h1_pro.driver import ZerithH1ProDriver


def _describe_arrow_payload(payload: Any) -> str:
    schema = getattr(payload, "schema", None)
    names = list(getattr(schema, "names", []) or [])
    rows = getattr(payload, "num_rows", "?")
    nbytes = getattr(payload, "nbytes", None)
    if nbytes is None:
        return f"type={type(payload).__name__}, rows={rows}, fields={names}"
    return f"type={type(payload).__name__}, rows={rows}, bytes={nbytes}, fields={names}"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Use zerith_h1_pro driver to read joint state and camera frames."
    )
    parser.add_argument(
        "--config",
        default=str(PACKAGE_ROOT / "config" / "config.yaml"),
        help="Path to zerith_h1_pro config yaml.",
    )
    parser.add_argument("--duration", type=float, default=10.0, help="Read duration in seconds.")
    parser.add_argument("--interval", type=float, default=1.0, help="Print interval in seconds.")
    args = parser.parse_args()

    config = load_config(args.config)
    driver = ZerithH1ProDriver(
        sdk_root=config.sdk_root,
        robot_ip=config.robot_ip,
        is_follower=False,
        init_on_connect=config.init_on_connect,
        control_mode=config.control_mode,
        switch_mode_on_connect=config.switch_mode_on_connect,
        actuator_order=config.actuator_order,
        camera_grpc_target=config.camera_grpc_target,
        cameras=config.cameras,
        image_fps=config.image_fps,
        joint_fps=config.joint_fps,
        auto_connect=True,
    )

    image_output_ids: list[str] = []
    for camera in config.cameras or []:
        if camera.enable_color:
            image_output_ids.append(camera.output_id)
        if camera.enable_depth and camera.depth_output_id:
            image_output_ids.append(camera.depth_output_id)

    if not image_output_ids:
        print("未配置 cameras，示例将只读取关节状态。")

    try:
        deadline = time.monotonic() + max(args.duration, 0.0)
        while time.monotonic() < deadline:
            state = driver.get_state()
            sample = list(zip(state.name, state.position, strict=False))[:5]
            sample_text = ", ".join(
                f"{name}={float(position):.4f}" for name, position in sample
            )
            print(f"[joint] {sample_text}")

            for output_id in image_output_ids:
                payload = driver.get_latest_encoded_image(output_id)
                if payload is None:
                    print(f"[image:{output_id}] waiting for frame")
                else:
                    print(f"[image:{output_id}] {_describe_arrow_payload(payload)}")

            time.sleep(max(args.interval, 0.05))
    finally:
        driver.disconnect()


if __name__ == "__main__":
    main()
