"""Zerith H1 Pro actuator metadata, conversions, and stateless helpers."""

from __future__ import annotations

from forge_msgs import JointCommand, JointState, LocomotionCommand

from .config import ZERITH_H1_PRO_MAX_WHEEL_SPEED_RADPS

# actuator_name -> vendor SDK enum member name
ACTUATOR_ENUM_NAME_MAP: dict[str, str] = {
    "wheel_left": "MOTOR_WHEEL_LEFT",
    "wheel_right": "MOTOR_WHEEL_RIGHT",
    "lift": "MOTOR_LIFT",
    "waist_down": "MOTOR_WAIST_DOWN",
    "waist_up": "MOTOR_WAIST_UP",
    "head_down": "MOTOR_HEAD_DOWN",
    "head_up": "MOTOR_HEAD_UP",
    "left_arm_1": "MOTOR_LEFT_ARM_1",
    "left_arm_2": "MOTOR_LEFT_ARM_2",
    "left_arm_3": "MOTOR_LEFT_ARM_3",
    "left_arm_4": "MOTOR_LEFT_ARM_4",
    "left_arm_5": "MOTOR_LEFT_ARM_5",
    "left_arm_6": "MOTOR_LEFT_ARM_6",
    "left_arm_7": "MOTOR_LEFT_ARM_7",
    "left_gripper": "MOTOR_LEFT_ARM_8",
    "right_arm_1": "MOTOR_RIGHT_ARM_1",
    "right_arm_2": "MOTOR_RIGHT_ARM_2",
    "right_arm_3": "MOTOR_RIGHT_ARM_3",
    "right_arm_4": "MOTOR_RIGHT_ARM_4",
    "right_arm_5": "MOTOR_RIGHT_ARM_5",
    "right_arm_6": "MOTOR_RIGHT_ARM_6",
    "right_arm_7": "MOTOR_RIGHT_ARM_7",
    "right_gripper": "MOTOR_RIGHT_ARM_8",
}

# Wheels remain part of the generic joint contract in velocity mode. The
# independent LocomotionCommand path shares the same low-level writes/watchdog.
ACTUATOR_CONTROL_FIELD: dict[str, str] = {
    "wheel_left": "speed",
    "wheel_right": "speed",
}

WHEEL_ACTUATORS = {"wheel_left", "wheel_right"}
GRIPPER_ACTUATORS = {"left_gripper", "right_gripper"}
# Vendor H1 SDK examples define Position=0.0 as open and Position=1.5 as closed.
# The public Forge contract uses physical opening: 0.0 m closed, 0.08 m open.
GRIPPER_SDK_RANGE = (0.0, 1.5)
GRIPPER_OPENING_M_RANGE = (0.0, 0.08)
LIFT_ACTUATOR = "lift"
LIFT_SDK_RANGE = (0.0, 0.8)
LIFT_M_RANGE = (0.0, 0.8)

# Position actuator limits come from the SDK demo:
# H1_SDK_1.3.7/robot_SDK/example/h1_low_level_motor_limit.cpp.
# The vendored SDK does not document wheel Speed units or limits; the project
# uses the low-level demo's magnitude 1.0 as a conservative ceiling pending
# hardware validation. Linear axes are exposed in meters.
ACTUATOR_LIMITS: dict[str, tuple[float, float]] = {
    "wheel_left": (
        -ZERITH_H1_PRO_MAX_WHEEL_SPEED_RADPS,
        ZERITH_H1_PRO_MAX_WHEEL_SPEED_RADPS,
    ),
    "wheel_right": (
        -ZERITH_H1_PRO_MAX_WHEEL_SPEED_RADPS,
        ZERITH_H1_PRO_MAX_WHEEL_SPEED_RADPS,
    ),
    "lift": LIFT_M_RANGE,
    "waist_down": (0.0, 1.3),
    "waist_up": (-0.7, 0.7),
    "head_down": (-1.5, 1.5),
    "head_up": (-0.17, 0.25),
    "left_arm_1": (-2.7, 1.5),
    "left_arm_2": (-0.3, 2.0),
    "left_arm_3": (-2.9, 2.9),
    "left_arm_4": (-1.3, 1.5),
    "left_arm_5": (-2.9, 2.9),
    "left_arm_6": (-1.0, 1.0),
    "left_arm_7": (-1.0, 1.0),
    "left_gripper": GRIPPER_OPENING_M_RANGE,
    "right_arm_1": (-2.7, 1.5),
    "right_arm_2": (-2.0, 0.3),
    "right_arm_3": (-2.9, 2.9),
    "right_arm_4": (-1.3, 1.5),
    "right_arm_5": (-2.9, 2.9),
    "right_arm_6": (-1.0, 1.0),
    "right_arm_7": (-1.0, 1.0),
    "right_gripper": GRIPPER_OPENING_M_RANGE,
}


def scale_range(
    value: float,
    src: tuple[float, float],
    dst: tuple[float, float],
) -> float:
    src_lo, src_hi = src
    dst_lo, dst_hi = dst
    return dst_lo + (float(value) - src_lo) * (dst_hi - dst_lo) / (src_hi - src_lo)


def gripper_sdk_to_m(position: float) -> float:
    opening_min, opening_max = GRIPPER_OPENING_M_RANGE
    return scale_range(position, GRIPPER_SDK_RANGE, (opening_max, opening_min))


def gripper_m_to_sdk(opening_m: float) -> float:
    sdk_min, sdk_max = GRIPPER_SDK_RANGE
    return scale_range(opening_m, GRIPPER_OPENING_M_RANGE, (sdk_max, sdk_min))


def gripper_sdk_velocity_to_mps(speed: float) -> float:
    sdk_min, sdk_max = GRIPPER_SDK_RANGE
    opening_min, opening_max = GRIPPER_OPENING_M_RANGE
    return -float(speed) * (opening_max - opening_min) / (sdk_max - sdk_min)


def lift_sdk_to_m(position: float) -> float:
    return scale_range(position, LIFT_SDK_RANGE, LIFT_M_RANGE)


def lift_m_to_sdk(position_m: float) -> float:
    return scale_range(position_m, LIFT_M_RANGE, LIFT_SDK_RANGE)


def field_by_name(names: list[str], values: list[float]) -> dict[str, float]:
    if len(values) != len(names):
        return {}
    return dict(zip(names, (float(value) for value in values), strict=False))


def position_by_name(state: JointState) -> dict[str, float]:
    return field_by_name(state.name, state.position)


def joint_command_without_wheels(command: JointCommand) -> JointCommand:
    """Strip wheel entries while preserving the alignment semantics of all fields."""
    names = list(command.name)
    keep = [index for index, name in enumerate(names) if name not in WHEEL_ACTUATORS]

    def filtered(values: list[float]) -> list[float]:
        if len(values) != len(names):
            return list(values)
        return [values[index] for index in keep]

    return JointCommand(
        name=[names[index] for index in keep],
        mode=command.mode,
        position=filtered(command.position),
        velocity=filtered(command.velocity),
        effort=filtered(command.effort),
        kp=filtered(command.kp),
        kd=filtered(command.kd),
    )


def locomotion_to_wheel_speeds(
    command: LocomotionCommand,
    *,
    wheel_radius_m: float,
    wheel_track_width_m: float,
    max_wheel_speed_radps: float,
) -> tuple[float, float]:
    """Convert body-frame differential-drive velocity to saturated wheel speeds."""
    half_track = wheel_track_width_m / 2.0
    left = (command.vx - command.wz * half_track) / wheel_radius_m
    right = (command.vx + command.wz * half_track) / wheel_radius_m
    peak = max(abs(left), abs(right))
    if peak > max_wheel_speed_radps:
        scale = max_wheel_speed_radps / peak
        left *= scale
        right *= scale
    return left, right
