#!/usr/bin/env python3
"""Small hardware motion smoke test for Zerith H1 Pro actuators."""

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
from zerith_h1_pro.actuators import ACTUATOR_LIMITS
from zerith_h1_pro.config import load_config
from zerith_h1_pro.driver import ZerithH1ProDriver


POSITION_ACTUATORS_TO_SKIP = {"wheel_left", "wheel_right"}
LINEAR_ACTUATORS = {"lift", "left_gripper", "right_gripper"}


def parse_args() -> argparse.Namespace:
    default_config = Path(__file__).resolve().parents[1] / "zerith_h1_pro.yaml"
    parser = argparse.ArgumentParser(
        description=(
            "Move each position actuator by a small displacement and return. "
            "Grippers are included by default."
        )
    )
    parser.add_argument(
        "--config",
        default=str(default_config),
        help="Path to YAML config.",
    )
    parser.add_argument(
        "--actuator",
        action="append",
        dest="actuators",
        help="Actuator to test; repeat to select multiple. Defaults to all position actuators.",
    )
    parser.add_argument(
        "--control-mode",
        default="low_level",
        choices=("low_level", "gravity_compensation_level"),
        help="Control mode used for the motion test.",
    )
    parser.add_argument(
        "--joint-displacement",
        type=float,
        default=0.1,
        help="Small revolute joint displacement in radians.",
    )
    parser.add_argument(
        "--lift-displacement",
        type=float,
        default=0.02,
        help="Small lift displacement in meters.",
    )
    parser.add_argument(
        "--gripper-displacement",
        type=float,
        default=0.1,
        help="Small gripper displacement in meters.",
    )
    parser.add_argument("--move-seconds", type=float, default=1.5)
    parser.add_argument("--hold-seconds", type=float, default=0.5)
    parser.add_argument("--settle-seconds", type=float, default=0.3)
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument(
        "--limit-tolerance",
        type=float,
        default=1e-3,
        help="Tolerance for tiny state readings outside configured limits.",
    )
    parser.add_argument(
        "--yes",
        "-y",
        action="store_true",
        help="Skip safety confirmation.",
    )
    return parser.parse_args()


def unit_for_actuator(actuator: str) -> str:
    return "meters" if actuator in LINEAR_ACTUATORS else "radians"


def displacement_for_actuator(args: argparse.Namespace, actuator: str) -> float:
    if actuator == "lift":
        return abs(float(args.lift_displacement))
    if actuator in {"left_gripper", "right_gripper"}:
        return abs(float(args.gripper_displacement))
    return abs(float(args.joint_displacement))


def clamp_current_to_limits(actuator: str, current: float, tolerance: float) -> float:
    limits = ACTUATOR_LIMITS.get(actuator)
    if limits is None:
        raise RuntimeError(f"{actuator} has no configured limit; refusing to test it.")
    low, high = limits
    tol = abs(float(tolerance))
    if low - tol <= current <= high + tol:
        return min(max(current, low), high)
    raise RuntimeError(
        f"{actuator} current={current:.6f} is outside "
        f"[{low:.6f}, {high:.6f}] beyond tolerance {tol:.6f}."
    )


def choose_target(actuator: str, current: float, displacement: float) -> float:
    limits = ACTUATOR_LIMITS.get(actuator)
    if limits is None:
        raise RuntimeError(f"{actuator} has no configured limit; refusing to test it.")
    low, high = limits
    if not (low <= current <= high):
        raise RuntimeError(
            f"{actuator} current={current:.6f} is outside [{low:.6f}, {high:.6f}]."
        )

    bounded = min(abs(float(displacement)), (high - low) / 4.0)
    if bounded <= 0.0:
        raise RuntimeError(f"{actuator} has no usable motion range.")

    upward = min(current + bounded, high)
    if upward - current >= bounded * 0.5:
        return upward

    downward = max(current - bounded, low)
    if current - downward >= bounded * 0.5:
        return downward

    raise RuntimeError(f"{actuator} is too close to its limits for this displacement.")


def send_position_ramp(
    driver: ZerithH1ProDriver,
    actuator: str,
    *,
    start: float,
    target: float,
    move_seconds: float,
    rate_hz: float,
) -> None:
    duration = max(float(move_seconds), 0.0)
    rate = max(float(rate_hz), 1.0)
    steps = max(int(duration * rate), 1)
    sleep_seconds = 1.0 / rate
    distance = target - start
    for i in range(1, steps + 1):
        value = start + distance * (i / steps)
        driver.set_command(JointCommand(name=[actuator], position=[value]))
        time.sleep(sleep_seconds)


def selected_position_actuators(
    driver: ZerithH1ProDriver,
    requested: list[str] | None,
) -> list[str]:
    order = driver.actuator_order
    selected = list(requested) if requested else [
        name for name in order if name not in POSITION_ACTUATORS_TO_SKIP
    ]
    unknown = [name for name in selected if name not in order]
    if unknown:
        raise ValueError(f"Unknown actuators: {unknown}. Supported: {order}")
    wheels = [name for name in selected if name in POSITION_ACTUATORS_TO_SKIP]
    if wheels:
        raise ValueError(
            f"Wheels are velocity actuators, not position actuators: {wheels}. "
            "Use test_wheel_distance.py for wheel motion testing."
        )
    return selected


def build_driver(args: argparse.Namespace) -> ZerithH1ProDriver:
    config = load_config(config_path=args.config)
    return ZerithH1ProDriver(
        sdk_root=config.sdk_root,
        robot_ip=config.robot_ip,
        is_follower=config.is_follower,
        init_on_connect=True,
        control_mode=args.control_mode,
        switch_mode_on_connect=True,
        actuator_order=config.actuator_order,
        auto_connect=True,
        camera_grpc_target=None,
        cameras=None,
        image_fps=config.image_fps,
        joint_fps=config.joint_fps,
    )


def main() -> int:
    args = parse_args()
    if not args.yes:
        response = input(
            "Confirm E-stop is available and the robot workspace is clear? [y/N] "
        ).strip().lower()
        if response not in {"y", "yes"}:
            print("Aborted.")
            return 1

    driver = build_driver(args)
    active_actuator: str | None = None
    active_start: float | None = None
    try:
        print(f"mode: {driver.get_current_control_mode()}")
        actuators = selected_position_actuators(driver, args.actuators)
        print("position test order: " + ", ".join(actuators))
        for index, actuator in enumerate(actuators, start=1):
            state = driver.get_state()
            if actuator not in state.name:
                raise RuntimeError(f"State does not include actuator: {actuator}")

            current = float(state.to_np([actuator], "position")[0])
            safe_current = clamp_current_to_limits(
                actuator,
                current,
                args.limit_tolerance,
            )
            displacement = displacement_for_actuator(args, actuator)
            target = choose_target(actuator, safe_current, displacement)
            unit = unit_for_actuator(actuator)
            print(
                f"[{index}/{len(actuators)}] {actuator}: "
                f"current={current:.6f}, start={safe_current:.6f}, "
                f"target={target:.6f}, unit={unit}"
            )

            active_actuator = actuator
            active_start = safe_current
            send_position_ramp(
                driver,
                actuator,
                start=safe_current,
                target=target,
                move_seconds=args.move_seconds,
                rate_hz=args.rate_hz,
            )
            time.sleep(max(float(args.hold_seconds), 0.0))
            send_position_ramp(
                driver,
                actuator,
                start=target,
                target=safe_current,
                move_seconds=args.move_seconds,
                rate_hz=args.rate_hz,
            )
            active_actuator = None
            active_start = None
            time.sleep(max(float(args.settle_seconds), 0.0))
    finally:
        if active_actuator is not None and active_start is not None:
            try:
                driver.set_command(
                    JointCommand(name=[active_actuator], position=[active_start])
                )
            except Exception as exc:
                print(f"Failed to return {active_actuator} to start: {exc}")
        driver.disconnect()

    print("done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
