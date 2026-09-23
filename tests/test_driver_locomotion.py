from __future__ import annotations

import math
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from forge_msgs import JointCommand, LocomotionCommand
from forge_robot import LocomotionSpec, LocomotionRobotDriver

from zerith_h1_pro.config import ZERITH_H1_PRO_ACTUATOR_ORDER, ZerithH1ProNodeConfig
from zerith_h1_pro.driver import (
    ActuatorWriteError,
    ChassisStopUnconfirmedError,
    JointCommandValidationError,
    ZerithH1ProDriver,
)


class _MotorControl:
    def __init__(self) -> None:
        self.Speed = 0.0
        self.Position = 0.0


class _FakeRobot:
    def __init__(self, *, fail_calls: set[int] | None = None) -> None:
        self.calls: list[tuple[int, float]] = []
        self.motor_calls: list[tuple[int, float]] = []
        self.fail_calls = fail_calls or set()

    def setChassis_low(self, motor_id: int, control: _MotorControl) -> bool:
        self.calls.append((motor_id, float(control.Speed)))
        return len(self.calls) not in self.fail_calls

    def setMotorControl_low(self, motor_id: int, control: _MotorControl) -> bool:
        self.motor_calls.append((motor_id, float(control.Position)))
        return True


class _MotorIndex:
    MOTOR_WHEEL_LEFT = 10
    MOTOR_WHEEL_RIGHT = 11
    MOTOR_LIFT = 12


def _driver(
    robot: _FakeRobot,
    *,
    mode: str = "low_level",
    max_vx: float = 0.2,
    max_wz: float = 1.0,
) -> ZerithH1ProDriver:
    driver = object.__new__(ZerithH1ProDriver)
    driver._robot = robot
    driver._sdk = SimpleNamespace(Motor_Control=_MotorControl)
    driver._wheel_enum_map = {"wheel_left": 10, "wheel_right": 11}
    driver._lifecycle_lock = threading.Lock()
    driver._control_mode_lock = threading.RLock()
    driver._locomotion_lock = threading.Lock()
    driver._disconnecting = False
    driver._lifecycle_command_active = False
    driver._locomotion_spec = LocomotionSpec(
        max_vx=max_vx,
        max_wz=max_wz,
        allow_lateral=False,
    )
    driver.wheel_radius_m = 0.1
    driver.wheel_track_width_m = 0.4
    driver.locomotion_max_wheel_speed_radps = 1.0
    driver.locomotion_command_timeout_seconds = 0.25
    driver._last_locomotion_command_at = None
    driver._locomotion_active = False
    driver._locomotion_timeout_latched = False
    driver._chassis_stop_unconfirmed = False
    driver.get_current_control_mode = lambda: mode  # type: ignore[method-assign]
    return driver


class LocomotionConfigTest(unittest.TestCase):
    def test_conservative_defaults_are_explicit(self) -> None:
        config = ZerithH1ProNodeConfig.from_dict({})

        self.assertAlmostEqual(config.wheel_radius_m, 0.0835)
        self.assertAlmostEqual(config.wheel_track_width_m, 0.379)
        self.assertAlmostEqual(config.locomotion_max_vx_mps, 0.08)
        self.assertAlmostEqual(config.locomotion_max_wz_radps, 0.4)
        self.assertAlmostEqual(config.locomotion_max_wheel_speed_radps, 1.0)
        self.assertAlmostEqual(config.locomotion_command_timeout_seconds, 0.25)

    def test_wheels_remain_available_in_joint_actuator_order(self) -> None:
        config = ZerithH1ProNodeConfig.from_dict(
            {"actuator_order": ["wheel_left", "wheel_right", "lift"]}
        )

        self.assertEqual(
            config.actuator_order,
            ["wheel_left", "wheel_right", "lift"],
        )

    def test_invalid_or_infeasible_locomotion_config_is_rejected(self) -> None:
        for field, value in (
            ("wheel_radius_m", 0.0),
            ("wheel_track_width_m", -1.0),
            ("locomotion_max_vx_mps", math.inf),
            ("locomotion_max_wz_radps", math.nan),
            ("locomotion_max_wheel_speed_radps", 1.1),
            ("locomotion_command_timeout_seconds", 0.0),
        ):
            with self.subTest(field=field), self.assertRaises(ValueError):
                ZerithH1ProNodeConfig.from_dict({field: value})

        with self.assertRaisesRegex(ValueError, "locomotion_max_vx_mps"):
            ZerithH1ProNodeConfig.from_dict(
                {"wheel_radius_m": 0.01, "locomotion_max_vx_mps": 0.08}
            )
        with self.assertRaisesRegex(ValueError, "locomotion_max_wz_radps"):
            ZerithH1ProNodeConfig.from_dict(
                {"wheel_track_width_m": 1.0, "locomotion_max_wz_radps": 0.4}
            )


class LocomotionDriverTest(unittest.TestCase):
    def test_driver_satisfies_standard_locomotion_protocol(self) -> None:
        self.assertIsInstance(_driver(_FakeRobot()), LocomotionRobotDriver)

    def test_straight_and_counter_clockwise_commands_use_expected_signs(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot)

        driver.set_locomotion_command(LocomotionCommand(vx=0.05, vy=0.0, wz=0.0))
        self.assertEqual([call[0] for call in robot.calls], [10, 11])
        self.assertAlmostEqual(robot.calls[0][1], 0.5)
        self.assertAlmostEqual(robot.calls[1][1], 0.5)

        robot.calls.clear()
        driver.set_locomotion_command(LocomotionCommand(vx=0.0, vy=0.0, wz=0.5))
        self.assertAlmostEqual(robot.calls[0][1], -1.0)
        self.assertAlmostEqual(robot.calls[1][1], 1.0)

    def test_lateral_is_forced_zero_and_mixed_command_uses_common_saturation(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot)

        driver.set_locomotion_command(LocomotionCommand(vx=0.0, vy=1.0, wz=0.0))
        self.assertEqual(robot.calls, [(10, 0.0), (11, 0.0)])
        self.assertFalse(driver._locomotion_active)

        robot.calls.clear()
        driver.set_locomotion_command(LocomotionCommand(vx=0.1, vy=0.0, wz=0.5))
        self.assertAlmostEqual(robot.calls[0][1], 0.0)
        self.assertAlmostEqual(robot.calls[1][1], 1.0)

    def test_vx_and_wz_are_clipped_before_conversion(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot, max_vx=0.05, max_wz=0.5)

        driver.set_locomotion_command(LocomotionCommand(vx=1.0, vy=0.0, wz=0.0))

        self.assertAlmostEqual(robot.calls[0][1], 0.5)
        self.assertAlmostEqual(robot.calls[1][1], 0.5)

    def test_joint_wheel_command_shares_low_level_write_and_watchdog(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot)
        driver._actuator_order = ["wheel_left", "wheel_right"]
        driver._enum_map = {"wheel_left": 10, "wheel_right": 11}
        driver._non_action_mode_warned = False

        driver.set_command(
            JointCommand(
                name=["wheel_left", "wheel_right"],
                velocity=[0.5, -0.25],
            )
        )

        self.assertEqual(robot.calls, [(10, 0.5), (11, -0.25)])
        self.assertTrue(driver._locomotion_active)
        self.assertIsNotNone(driver._last_locomotion_command_at)

    def test_full_position_only_command_stops_wheels_and_writes_21_actuators(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot)
        names = list(ZERITH_H1_PRO_ACTUATOR_ORDER)
        driver._actuator_order = names
        driver._enum_map = {name: index for index, name in enumerate(names)}
        driver._non_action_mode_warned = False
        driver._locomotion_active = True
        driver._last_locomotion_command_at = time.monotonic()

        driver.set_command(
            JointCommand(
                name=names,
                position=[0.0 for _ in names],
            )
        )

        non_wheel_names = [
            name for name in names if name not in {"wheel_left", "wheel_right"}
        ]
        self.assertEqual(robot.calls, [(10, 0.0), (11, 0.0)])
        self.assertEqual(
            [motor_id for motor_id, _ in robot.motor_calls],
            [driver._enum_map[name] for name in non_wheel_names],
        )
        self.assertEqual(len(robot.motor_calls), 21)
        self.assertFalse(driver._locomotion_active)
        self.assertIsNone(driver._last_locomotion_command_at)
        self.assertFalse(driver._locomotion_timeout_latched)
        self.assertFalse(driver._chassis_stop_unconfirmed)

    def test_unknown_actuator_is_rejected_as_command_validation(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot)

        with self.assertRaisesRegex(JointCommandValidationError, "unknown JointCommand"):
            driver.set_command(JointCommand(name=["unknown_actuator"], position=[0.0]))

        self.assertEqual(robot.calls, [])

    def test_joint_wheel_command_requires_both_wheels(self) -> None:
        driver = _driver(_FakeRobot())
        driver._actuator_order = ["wheel_left", "wheel_right"]
        driver._enum_map = {"wheel_left": 10, "wheel_right": 11}
        driver._non_action_mode_warned = False

        with self.assertRaisesRegex(ValueError, "must include"):
            driver.set_command(
                JointCommand(name=["wheel_left"], velocity=[0.5])
            )

    def test_infinite_direct_wheel_velocity_is_rejected_before_clipping(self) -> None:
        for value in (math.inf, -math.inf):
            with self.subTest(value=value):
                robot = _FakeRobot()
                driver = _driver(robot)
                driver._actuator_order = ["wheel_left", "wheel_right"]
                driver._enum_map = {"wheel_left": 10, "wheel_right": 11}
                driver._non_action_mode_warned = False
                driver._locomotion_active = True

                with self.assertRaisesRegex(ValueError, "finite"):
                    driver.set_command(
                        JointCommand(
                            name=["wheel_left", "wheel_right"],
                            velocity=[value, 0.5],
                        )
                    )

                self.assertEqual(robot.calls, [(10, 0.0), (11, 0.0)])
                self.assertFalse(driver._locomotion_active)

    def test_invalid_direct_wheel_command_stops_existing_motion(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot)
        driver._actuator_order = ["wheel_left", "wheel_right"]
        driver._enum_map = {"wheel_left": 10, "wheel_right": 11}
        driver._non_action_mode_warned = False
        driver._locomotion_active = True

        with self.assertRaisesRegex(ValueError, "finite"):
            driver.set_command(
                JointCommand(
                    name=["wheel_left", "wheel_right"],
                    velocity=[math.nan, math.inf],
                )
            )

        self.assertEqual(robot.calls[-2:], [(10, 0.0), (11, 0.0)])
        self.assertFalse(driver._locomotion_active)

    def test_direct_wheel_command_in_non_low_mode_never_writes_and_requires_estop(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot, mode="gravity_compensation_level")
        driver._actuator_order = ["wheel_left", "wheel_right"]
        driver._enum_map = {"wheel_left": 10, "wheel_right": 11}
        driver._non_action_mode_warned = False
        driver._locomotion_active = True

        with self.assertRaisesRegex(
            ChassisStopUnconfirmedError,
            "Physical E-stop required",
        ):
            driver.set_command(
                JointCommand(
                    name=["wheel_left", "wheel_right"],
                    velocity=[0.5, 0.5],
                )
            )

        self.assertEqual(robot.calls, [])
        self.assertTrue(driver._locomotion_active)
        self.assertTrue(driver._chassis_stop_unconfirmed)

    def test_locomotion_rejects_commands_after_disconnect_starts(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot)
        driver._disconnecting = True

        with self.assertRaisesRegex(RuntimeError, "disconnecting"):
            driver.set_locomotion_command(
                LocomotionCommand(vx=0.05, vy=0.0, wz=0.0)
            )

        self.assertEqual(robot.calls, [])

    def test_locomotion_rejects_non_low_level_without_switching(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot, mode="gravity_compensation_level")

        with self.assertRaisesRegex(RuntimeError, "requires confirmed actual control mode"):
            driver.set_locomotion_command(
                LocomotionCommand(vx=0.05, vy=0.0, wz=0.0)
            )

        self.assertEqual(robot.calls, [])

    def test_invalid_timestamp_after_external_mode_change_never_writes(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot, mode="gravity_compensation_level")
        driver._locomotion_active = True

        with patch("zerith_h1_pro.driver.time.monotonic", return_value=10.0):
            with self.assertRaisesRegex(
                ChassisStopUnconfirmedError,
                "Physical E-stop required",
            ):
                driver.set_locomotion_command(
                    LocomotionCommand(vx=0.05, vy=0.0, wz=0.0),
                    received_at=10.1,
                )

        self.assertEqual(robot.calls, [])
        self.assertTrue(driver._chassis_stop_unconfirmed)

    def test_explicit_stop_in_non_low_mode_never_writes(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot, mode="gravity_compensation_level")
        driver._locomotion_active = True

        with self.assertRaisesRegex(
            ChassisStopUnconfirmedError,
            "Physical E-stop required",
        ):
            driver.stop_locomotion()

        self.assertEqual(robot.calls, [])
        self.assertTrue(driver._chassis_stop_unconfirmed)

    def test_unknown_mode_while_active_never_writes_and_requires_estop(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot)
        driver._locomotion_active = True

        def fail_mode_query() -> str:
            raise RuntimeError("mode unavailable")

        driver.get_current_control_mode = fail_mode_query  # type: ignore[method-assign]

        with self.assertRaisesRegex(
            ChassisStopUnconfirmedError,
            "actual control mode could not be read",
        ):
            driver.stop_locomotion()

        self.assertEqual(robot.calls, [])
        self.assertTrue(driver.chassis_stop_unconfirmed)

    def test_watchdog_external_mode_change_never_writes_and_latches_unconfirmed(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot, mode="gravity_compensation_level")
        driver._last_locomotion_command_at = 10.0
        driver._locomotion_active = True

        self.assertFalse(driver._apply_locomotion_watchdog(now=10.3))

        self.assertEqual(robot.calls, [])
        self.assertTrue(driver._locomotion_active)
        self.assertTrue(driver._chassis_stop_unconfirmed)

    def test_mode_is_rechecked_before_each_native_wheel_write(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot)
        modes = iter(["low_level", "low_level", "gravity_compensation_level"])
        driver.get_current_control_mode = lambda: next(modes)  # type: ignore[method-assign]

        with self.assertRaisesRegex(
            ChassisStopUnconfirmedError,
            "Physical E-stop required",
        ):
            driver.set_locomotion_command(
                LocomotionCommand(vx=0.05, vy=0.0, wz=0.0)
            )

        self.assertEqual(robot.calls, [(10, 0.5)])
        self.assertTrue(driver._chassis_stop_unconfirmed)

    def test_failed_sdk_write_attempts_zero_on_both_wheels_and_raises(self) -> None:
        robot = _FakeRobot(fail_calls={2})
        driver = _driver(robot)

        with self.assertRaisesRegex(RuntimeError, "setChassis_low failed"):
            driver.set_locomotion_command(
                LocomotionCommand(vx=0.05, vy=0.0, wz=0.0)
            )

        self.assertEqual(
            robot.calls,
            [(10, 0.5), (11, 0.5), (10, 0.0), (11, 0.0)],
        )

    def test_stale_ipc_command_is_stopped_without_restarting_wheels(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot)

        with patch("zerith_h1_pro.driver.time.monotonic", return_value=10.3):
            driver.set_locomotion_command(
                LocomotionCommand(vx=0.05, vy=0.0, wz=0.0),
                received_at=10.0,
            )

        self.assertEqual(robot.calls, [(10, 0.0), (11, 0.0)])
        self.assertFalse(driver._locomotion_active)
        self.assertTrue(driver._locomotion_timeout_latched)

    def test_invalid_ipc_timestamp_fails_closed(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot)
        driver._locomotion_active = True

        with patch("zerith_h1_pro.driver.time.monotonic", return_value=10.0):
            with self.assertRaisesRegex(ValueError, "received_at"):
                driver.set_locomotion_command(
                    LocomotionCommand(vx=0.05, vy=0.0, wz=0.0),
                    received_at=10.1,
                )

        self.assertEqual(robot.calls, [(10, 0.0), (11, 0.0)])

    def test_partial_write_and_failed_compensating_stop_arms_watchdog_retry(self) -> None:
        robot = _FakeRobot(fail_calls={2, 3})
        driver = _driver(robot)

        with self.assertRaisesRegex(RuntimeError, "setChassis_low failed"):
            driver.set_locomotion_command(
                LocomotionCommand(vx=0.05, vy=0.0, wz=0.0)
            )

        self.assertTrue(driver._locomotion_active)
        self.assertTrue(driver._apply_locomotion_watchdog(now=time.monotonic()))
        self.assertEqual(robot.calls[-2:], [(10, 0.0), (11, 0.0)])
        self.assertFalse(driver._locomotion_active)

    def test_watchdog_stops_once_and_fresh_command_recovers(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot)
        driver._last_locomotion_command_at = 10.0
        driver._locomotion_active = True

        self.assertFalse(driver._apply_locomotion_watchdog(now=10.24))
        self.assertEqual(robot.calls, [])
        self.assertTrue(driver._apply_locomotion_watchdog(now=10.26))
        self.assertEqual(robot.calls, [(10, 0.0), (11, 0.0)])
        self.assertTrue(driver._locomotion_timeout_latched)
        self.assertFalse(driver._apply_locomotion_watchdog(now=11.0))
        self.assertEqual(len(robot.calls), 2)

        driver.set_locomotion_command(LocomotionCommand(vx=0.05, vy=0.0, wz=0.0))
        self.assertFalse(driver._locomotion_timeout_latched)
        self.assertTrue(driver._locomotion_active)

    def test_disconnect_explicitly_zeros_wheels(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot)
        driver._stop_bg_threads = False
        driver._bg_threads = []
        driver.is_follower = False
        driver.init_on_connect = False
        driver._camera_client = None

        driver.disconnect()

        self.assertEqual(robot.calls, [(10, 0.0), (11, 0.0)])
        self.assertIsNone(driver._robot)

    def test_disconnect_skips_low_level_writes_in_non_low_modes(self) -> None:
        for mode in ("gravity_compensation_level", "uninitialized"):
            with self.subTest(mode=mode):
                robot = _FakeRobot()
                driver = _driver(robot, mode=mode)
                driver._stop_bg_threads = False
                driver._bg_threads = []
                driver.is_follower = False
                driver.init_on_connect = False
                driver._camera_client = None

                driver.disconnect()

                self.assertEqual(robot.calls, [])
                self.assertIsNone(driver._robot)

    def test_disconnect_non_low_active_requires_estop_without_low_level_write(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot, mode="gravity_compensation_level")
        driver._stop_bg_threads = False
        driver._bg_threads = []
        driver.is_follower = False
        driver.init_on_connect = False
        driver._camera_client = None
        driver._locomotion_active = True

        with self.assertRaisesRegex(RuntimeError, "physical E-stop required"):
            driver.disconnect()

        self.assertEqual(robot.calls, [])
        self.assertIsNone(driver._robot)
        self.assertTrue(driver.chassis_stop_unconfirmed)

        with self.assertRaisesRegex(
            ChassisStopUnconfirmedError,
            "robot is disconnected",
        ):
            driver.stop_locomotion()
        self.assertTrue(driver.chassis_stop_unconfirmed)

    def test_disconnect_mode_query_failure_avoids_low_level_writes(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot)
        driver._stop_bg_threads = False
        driver._bg_threads = []
        driver.is_follower = True
        driver.init_on_connect = False
        driver._camera_client = None
        reset_calls: list[str] = []
        driver.move_to_reset_position = lambda: reset_calls.append("reset")  # type: ignore[method-assign]

        def fail_mode_query() -> str:
            raise RuntimeError("mode unavailable")

        driver.get_current_control_mode = fail_mode_query  # type: ignore[method-assign]

        with self.assertRaisesRegex(RuntimeError, "physical E-stop required"):
            driver.disconnect()

        self.assertEqual(robot.calls, [])
        self.assertEqual(reset_calls, [])
        self.assertIsNone(driver._robot)
        self.assertTrue(driver.chassis_stop_unconfirmed)

    def test_disconnect_allows_internal_reset_after_successful_stop(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot)
        driver._stop_bg_threads = False
        driver._bg_threads = []
        driver.is_follower = True
        driver.init_on_connect = False
        driver._camera_client = None
        reset_calls: list[tuple[bool, list[tuple[int, float]]]] = []
        driver.move_to_reset_position = lambda: reset_calls.append(  # type: ignore[method-assign]
            (driver._lifecycle_command_active, list(robot.calls))
        )

        driver.disconnect()

        self.assertEqual(
            reset_calls,
            [(True, [(10, 0.0), (11, 0.0)])],
        )
        self.assertEqual(robot.calls, [(10, 0.0), (11, 0.0)])

    def test_disconnect_reset_actuator_failure_is_fatal_after_cleanup(self) -> None:
        robot = _FakeRobot()
        driver = _driver(robot)
        driver._stop_bg_threads = False
        driver._bg_threads = []
        driver.is_follower = True
        driver.init_on_connect = False
        driver._camera_client = None

        def fail_reset() -> None:
            raise ActuatorWriteError("setMotorControl_low failed for lift")

        driver.move_to_reset_position = fail_reset  # type: ignore[method-assign]

        with self.assertRaisesRegex(RuntimeError, "failed to complete reset movement"):
            driver.disconnect()

        self.assertIsNone(driver._robot)
        self.assertEqual(robot.calls, [(10, 0.0), (11, 0.0)])

    def test_disconnect_retries_failed_zero_and_skips_reset(self) -> None:
        robot = _FakeRobot(fail_calls=set(range(1, 13)))
        driver = _driver(robot)
        driver._stop_bg_threads = False
        driver._bg_threads = []
        driver.is_follower = True
        driver.init_on_connect = False
        driver._camera_client = None
        reset_calls: list[str] = []
        driver.move_to_reset_position = lambda: reset_calls.append("reset")  # type: ignore[method-assign]

        with self.assertRaisesRegex(RuntimeError, "failed to confirm chassis stop"):
            driver.disconnect()

        self.assertEqual(len(robot.calls), 12)
        self.assertEqual(reset_calls, [])
        self.assertIsNone(driver._robot)
        self.assertTrue(driver.chassis_stop_unconfirmed)

        with self.assertRaisesRegex(
            ChassisStopUnconfirmedError,
            "robot is disconnected",
        ):
            driver.stop_locomotion()
        self.assertTrue(driver.chassis_stop_unconfirmed)

    def test_wheel_enums_exist_when_actuator_order_excludes_wheels(self) -> None:
        fake_sdk = SimpleNamespace(EtherCAT_Motor_Index=_MotorIndex)
        with patch("zerith_h1_pro.sdk.load_h1_sdk", return_value=fake_sdk):
            driver = ZerithH1ProDriver(
                sdk_root="unused",
                actuator_order=["lift"],
                auto_connect=False,
            )

        self.assertNotIn("wheel_left", driver._enum_map)
        self.assertNotIn("wheel_right", driver._enum_map)
        self.assertEqual(driver._wheel_enum_map, {"wheel_left": 10, "wheel_right": 11})


if __name__ == "__main__":
    unittest.main()
