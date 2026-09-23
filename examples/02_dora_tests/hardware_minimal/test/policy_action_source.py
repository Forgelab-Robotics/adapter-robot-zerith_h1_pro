#!/usr/bin/env python3
"""Small hardware-safe policy action source for Zerith H1 Pro Dora tests."""

from __future__ import annotations

import argparse
import math
import sys
import time

from dora import Node
from forge_msgs import JointCommand, JointState


GRIPPER_NAMES = {"left_gripper", "right_gripper"}
WHEEL_NAMES = {"wheel_left", "wheel_right"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a full JointCommand from the latest joint_state. "
            "All joints hold their current position; grippers get a small sine offset."
        )
    )
    parser.add_argument(
        "--gripper-amplitude",
        type=float,
        default=0.005,
        help="Gripper sine amplitude in meters.",
    )
    parser.add_argument(
        "--period-seconds",
        type=float,
        default=4.0,
        help="Sine period for gripper motion.",
    )
    parser.add_argument(
        "--gripper-min",
        type=float,
        default=0.0,
        help="Minimum gripper opening in meters.",
    )
    parser.add_argument(
        "--gripper-max",
        type=float,
        default=0.08,
        help="Maximum gripper opening in meters.",
    )
    return parser.parse_args()


def clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def build_action(
    state: JointState,
    *,
    baseline_position: dict[str, float],
    gripper_amplitude: float,
    period_seconds: float,
    gripper_min: float,
    gripper_max: float,
    started_at: float,
) -> JointCommand:
    phase = 2.0 * math.pi * ((time.monotonic() - started_at) / period_seconds)
    offset = gripper_amplitude * math.sin(phase)
    position_by_name = dict(zip(state.name, state.position, strict=False))

    positions: list[float] = []
    velocities: list[float] = []
    for name in state.name:
        current = float(position_by_name.get(name, 0.0))
        if name in GRIPPER_NAMES:
            baseline = float(baseline_position.get(name, current))
            positions.append(clamp(baseline + offset, gripper_min, gripper_max))
        else:
            positions.append(current)
        velocities.append(0.0)

    # Wheel actuators are velocity-controlled; keep them stopped for this policy test.
    for index, name in enumerate(state.name):
        if name in WHEEL_NAMES:
            velocities[index] = 0.0

    return JointCommand(
        name=list(state.name),
        position=positions,
        velocity=velocities,
        effort=[],
        kp=[],
        kd=[],
    )


def main() -> int:
    args = parse_args()
    period_seconds = max(float(args.period_seconds), 0.1)
    gripper_min = float(args.gripper_min)
    gripper_max = float(args.gripper_max)
    if gripper_min > gripper_max:
        raise ValueError("--gripper-min must be <= --gripper-max")

    node = Node()
    latest_state: JointState | None = None
    baseline_position: dict[str, float] | None = None
    started_at = time.monotonic()

    for event in node:
        match event["type"]:
            case "INPUT":
                input_id = event["id"]
                if input_id == "joint_state":
                    latest_state = JointState.from_arrow(event["value"])
                    if baseline_position is None:
                        baseline_position = dict(
                            zip(latest_state.name, latest_state.position, strict=False)
                        )
                    continue
                if input_id == "tick" and latest_state is not None and baseline_position is not None:
                    action = build_action(
                        latest_state,
                        baseline_position=baseline_position,
                        gripper_amplitude=abs(float(args.gripper_amplitude)),
                        period_seconds=period_seconds,
                        gripper_min=gripper_min,
                        gripper_max=gripper_max,
                        started_at=started_at,
                    )
                    node.send_output("action", action.to_arrow())
                    continue
            case "STOP":
                break
            case "ERROR":
                print(
                    f"[policy_action_source] error: {event.get('error', 'unknown')}",
                    file=sys.stderr,
                )
                break
            case _:
                pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
