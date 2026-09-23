from __future__ import annotations

import threading
from types import MethodType, SimpleNamespace
import unittest

from forge_msgs import JointCommand

from zerith_h1_pro.actuators import (
    gripper_m_to_sdk,
    gripper_sdk_to_m,
    gripper_sdk_velocity_to_mps,
)
from zerith_h1_pro.config import ZerithH1ProNodeConfig
from zerith_h1_pro.driver import (
    ActuatorWriteError,
    JointCommandValidationError,
    ZerithH1ProDriver,
)


class _MotorControl:
    def __init__(self) -> None:
        self.Position = 0.0
        self.Speed = 0.0
        self.Torque = 0.0


class _FakeRobot:
    def __init__(self, *, fail_motor_ids: set[int] | None = None) -> None:
        self.commands: list[tuple[int, float]] = []
        self.positions: dict[int, float] = {}
        self.fail_motor_ids = fail_motor_ids or set()

    def setMotorControl_low(self, motor_id: int, control: _MotorControl) -> bool:
        self.commands.append((motor_id, float(control.Position)))
        return motor_id not in self.fail_motor_ids

    def getMotorState(self, motor_id: int):
        return True, SimpleNamespace(Position_Actual=self.positions[motor_id])


def _driver(robot: _FakeRobot) -> ZerithH1ProDriver:
    driver = object.__new__(ZerithH1ProDriver)
    driver._robot = robot
    driver._sdk = SimpleNamespace(Motor_Control=_MotorControl)
    driver._enum_map = {"left_gripper": 18, "right_gripper": 26}
    driver._actuator_order = ["left_gripper", "right_gripper"]
    driver._control_mode_lock = threading.RLock()
    driver._locomotion_lock = threading.Lock()
    driver._disconnecting = False
    driver._lifecycle_command_active = False
    driver._non_action_mode_warned = False
    driver.get_current_control_mode = MethodType(lambda _self: "low_level", driver)
    return driver


class GripperDriverTest(unittest.TestCase):
    def test_default_lifecycle_positions_leave_grippers_open(self) -> None:
        config = ZerithH1ProNodeConfig()

        self.assertAlmostEqual(config.resolved_position("initial")["left_gripper"], 0.08)
        self.assertAlmostEqual(config.resolved_position("initial")["right_gripper"], 0.08)
        self.assertAlmostEqual(config.resolved_position("reset")["left_gripper"], 0.08)
        self.assertAlmostEqual(config.resolved_position("reset")["right_gripper"], 0.08)

    def test_position_conversion_matches_vendor_open_close_direction(self) -> None:
        self.assertAlmostEqual(gripper_sdk_to_m(0.0), 0.08)
        self.assertAlmostEqual(gripper_sdk_to_m(0.75), 0.04)
        self.assertAlmostEqual(gripper_sdk_to_m(1.5), 0.0)

        self.assertAlmostEqual(gripper_m_to_sdk(0.08), 0.0)
        self.assertAlmostEqual(gripper_m_to_sdk(0.04), 0.75)
        self.assertAlmostEqual(gripper_m_to_sdk(0.0), 1.5)

    def test_velocity_is_scaled_and_reversed_to_opening_velocity(self) -> None:
        self.assertAlmostEqual(gripper_sdk_velocity_to_mps(1.5), -0.08)
        self.assertAlmostEqual(gripper_sdk_velocity_to_mps(-1.5), 0.08)
        self.assertAlmostEqual(gripper_sdk_velocity_to_mps(0.0), 0.0)

    def test_commands_use_vendor_sdk_endpoints(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot)

        driver.set_command(
            JointCommand(
                name=["left_gripper", "right_gripper"],
                position=[0.08, 0.0],
            )
        )

        self.assertEqual(robot.commands, [(18, 0.0), (26, 1.5)])

    def test_mixed_known_and_unknown_actuators_are_rejected_before_writes(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot)

        with self.assertRaisesRegex(JointCommandValidationError, "unknown_actuator"):
            driver.set_command(
                JointCommand(
                    name=["left_gripper", "unknown_actuator"],
                    position=[0.08, 0.0],
                )
            )

        self.assertEqual(robot.commands, [])

    def test_false_non_wheel_sdk_writes_are_collected_and_fatal(self) -> None:
        robot = _FakeRobot(fail_motor_ids={18, 26})
        driver = _driver(robot)

        with self.assertRaisesRegex(
            ActuatorWriteError,
            "left_gripper, right_gripper",
        ):
            driver.set_command(
                JointCommand(
                    name=["left_gripper", "right_gripper"],
                    position=[0.08, 0.0],
                )
            )

        self.assertEqual(robot.commands, [(18, 0.0), (26, 1.5)])

    def test_lifecycle_readback_uses_opening_semantics(self) -> None:
        robot = _FakeRobot()
        robot.positions = {18: 0.0, 26: 1.5}
        driver = _driver(robot)

        positions = driver._read_actuator_positions(
            ["left_gripper", "right_gripper"]
        )

        self.assertAlmostEqual(positions[0], 0.08)
        self.assertAlmostEqual(positions[1], 0.0)


if __name__ == "__main__":
    unittest.main()
