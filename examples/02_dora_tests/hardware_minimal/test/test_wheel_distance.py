#!/usr/bin/env python3
"""Drive Zerith H1 Pro wheels forward and backward by an approximate distance."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[4]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from forge_msgs import JointCommand
from zerith_h1_pro.config import ZERITH_H1_PRO_WHEEL_RADIUS_M, load_config
from zerith_h1_pro.driver import ZerithH1ProDriver


WHEEL_ACTUATORS = ("wheel_left", "wheel_right")


def parse_args() -> argparse.Namespace:
    default_config = Path(__file__).resolve().parents[1] / "zerith_h1_pro.yaml"
    parser = argparse.ArgumentParser(
        description=(
            "Use low-level wheel velocity to move forward by distance_m, "
            "then backward by the same distance."
        )
    )
    parser.add_argument(
        "--config",
        default=str(default_config),
        help="Path to YAML config.",
    )
    parser.add_argument(
        "--distance-m",
        type=float,
        default=0.5,
        help="Approximate forward/backward travel distance in meters.",
    )
    parser.add_argument(
        "--wheel-speed-radps",
        type=float,
        default=1.0,
        help="Low-level wheel speed command in rad/s.",
    )
    parser.add_argument(
        "--wheel-radius-m",
        type=float,
        default=ZERITH_H1_PRO_WHEEL_RADIUS_M,
        help="Wheel radius used to convert wheel radians to chassis meters.",
    )
    parser.add_argument(
        "--max-seconds",
        type=float,
        default=15.0,
        help="Safety cap for each forward/backward segment.",
    )
    parser.add_argument(
        "--settle-seconds",
        type=float,
        default=1.0,
        help="Stop duration between forward and backward segments.",
    )
    parser.add_argument(
        "--yes",
        "-y",
        action="store_true",
        help="Skip safety confirmation.",
    )
    return parser.parse_args()


def build_driver(config_path: str) -> ZerithH1ProDriver:
    config = load_config(config_path=config_path)
    return ZerithH1ProDriver(
        sdk_root=config.sdk_root,
        robot_ip=config.robot_ip,
        is_follower=config.is_follower,
        init_on_connect=True,
        control_mode="low_level",
        switch_mode_on_connect=True,
        actuator_order=config.actuator_order,
        wheel_radius_m=config.wheel_radius_m,
        wheel_track_width_m=config.wheel_track_width_m,
        locomotion_max_vx_mps=config.locomotion_max_vx_mps,
        locomotion_max_wz_radps=config.locomotion_max_wz_radps,
        locomotion_max_wheel_speed_radps=config.locomotion_max_wheel_speed_radps,
        locomotion_command_timeout_seconds=config.locomotion_command_timeout_seconds,
        auto_connect=True,
        camera_grpc_target=None,
        cameras=None,
        image_fps=config.image_fps,
        joint_fps=config.joint_fps,
    )


def confirm_safety(args: argparse.Namespace, duration: float) -> bool:
    if args.yes:
        return True

    print(
        "This will drive the chassis approximately "
        f"{args.distance_m:.3f} m forward and {args.distance_m:.3f} m backward."
    )
    print(
        f"Low-level wheel speed={args.wheel_speed_radps:.3f} rad/s, "
        f"wheel radius={args.wheel_radius_m:.3f} m, "
        f"estimated duration per segment={duration:.3f} s."
    )
    response = input(
        "Confirm E-stop is available and at least 1 m of clear space exists "
        "in front/back? [y/N] "
    ).strip().lower()
    return response in {"y", "yes"}


def send_wheel_speed(driver: ZerithH1ProDriver, speed_radps: float) -> None:
    driver.set_command(
        JointCommand(
            name=list(WHEEL_ACTUATORS),
            velocity=[float(speed_radps), float(speed_radps)],
        )
    )


def stop_wheels(driver: ZerithH1ProDriver) -> None:
    send_wheel_speed(driver, 0.0)


def wheel_positions(driver: ZerithH1ProDriver) -> tuple[float, float]:
    state = driver.get_state()
    return tuple(float(v) for v in state.to_np(list(WHEEL_ACTUATORS), "position"))


def estimated_distance_m(
    start: tuple[float, float],
    current: tuple[float, float],
    wheel_radius_m: float,
) -> float:
    left_delta = abs(current[0] - start[0])
    right_delta = abs(current[1] - start[1])
    return 0.5 * (left_delta + right_delta) * wheel_radius_m


def drive_segment(
    driver: ZerithH1ProDriver,
    *,
    speed_radps: float,
    distance_m: float,
    wheel_radius_m: float,
    estimated_duration: float,
    max_seconds: float,
    label: str,
) -> None:
    deadline = time.monotonic() + min(estimated_duration, max_seconds)
    start = wheel_positions(driver)
    print(
        f"{label}: wheel_speed={speed_radps:.3f} rad/s, "
        f"target_distance={distance_m:.3f} m, "
        f"estimated_duration={estimated_duration:.3f} s"
    )
    send_wheel_speed(driver, speed_radps)
    travelled = 0.0
    try:
        while time.monotonic() < deadline:
            send_wheel_speed(driver, speed_radps)
            time.sleep(0.05)
            travelled = estimated_distance_m(
                start,
                wheel_positions(driver),
                wheel_radius_m,
            )
            if travelled >= distance_m:
                break
    finally:
        stop_wheels(driver)

    print(f"{label}: estimated travelled={travelled:.3f} m")


def main() -> int:
    args = parse_args()
    distance_m = abs(float(args.distance_m))
    wheel_speed_radps = abs(float(args.wheel_speed_radps))
    wheel_radius_m = abs(float(args.wheel_radius_m))
    if distance_m <= 0.0:
        raise ValueError("--distance-m must be positive.")
    if wheel_speed_radps <= 0.0:
        raise ValueError("--wheel-speed-radps must be positive.")
    if wheel_radius_m <= 0.0:
        raise ValueError("--wheel-radius-m must be positive.")

    duration = distance_m / (wheel_radius_m * wheel_speed_radps)
    if not confirm_safety(args, duration):
        print("Aborted.")
        return 1

    driver = build_driver(args.config)
    try:
        print(f"mode: {driver.get_current_control_mode()}")
        drive_segment(
            driver,
            speed_radps=wheel_speed_radps,
            distance_m=distance_m,
            wheel_radius_m=wheel_radius_m,
            estimated_duration=duration,
            max_seconds=max(float(args.max_seconds), 0.1),
            label="forward",
        )
        time.sleep(max(float(args.settle_seconds), 0.0))
        drive_segment(
            driver,
            speed_radps=-wheel_speed_radps,
            distance_m=distance_m,
            wheel_radius_m=wheel_radius_m,
            estimated_duration=duration,
            max_seconds=max(float(args.max_seconds), 0.1),
            label="backward",
        )
        time.sleep(max(float(args.settle_seconds), 0.0))
    finally:
        try:
            stop_wheels(driver)
        except Exception:
            pass
        driver.disconnect()

    print("done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
