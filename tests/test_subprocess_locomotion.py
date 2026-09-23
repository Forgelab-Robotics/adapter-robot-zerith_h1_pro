from __future__ import annotations

import math
import multiprocessing as mp
import queue
import threading
import unittest
from types import SimpleNamespace

from forge_msgs import JointCommand, JointState, LocomotionCommand

from zerith_h1_pro.config import ZERITH_H1_PRO_ACTUATOR_ORDER
from zerith_h1_pro.driver import (
    JointCommandValidationError,
    LocomotionCommandValidationError,
    WheelCommandValidationError,
    ZerithH1ProDriver,
)
from zerith_h1_pro.ipc import (
    CommandMessageValidationError,
    command_payload_to_joint_command,
    command_payload_without_wheels,
    joint_command_arrow_to_payload,
    locomotion_arrow_to_payload,
    locomotion_command_payload,
    locomotion_payload_to_command,
    master_state_arrow_to_command_payload,
    replace_latest_locomotion,
)
from zerith_h1_pro.subprocess_node import (
    _reject_parent_control_message,
    coordinate_parent_shutdown,
)
from zerith_h1_pro.workers import (
    arbitrate_joint_payload,
    dispatch_joint_payload,
    dispatch_locomotion_payload,
)


class SubprocessLocomotionTest(unittest.TestCase):
    def test_payload_round_trip_uses_standard_locomotion_command(self) -> None:
        command = LocomotionCommand(vx=0.05, vy=0.02, wz=-0.1)

        payload = locomotion_command_payload(command, received_at=123.0)

        self.assertEqual(
            payload,
            {"vx": 0.05, "vy": 0.02, "wz": -0.1, "received_at": 123.0},
        )
        self.assertEqual(locomotion_payload_to_command(payload), command)

    def test_full_master_state_propagates_valid_wheel_velocity(self) -> None:
        names = list(ZERITH_H1_PRO_ACTUATOR_ORDER)
        velocity = [0.0 for _ in names]
        velocity[names.index("wheel_left")] = 0.4
        velocity[names.index("wheel_right")] = -0.3
        state = JointState(
            name=names,
            position=[0.0 for _ in names],
            velocity=velocity,
        )

        payload = master_state_arrow_to_command_payload(state.to_arrow())

        self.assertEqual(payload["name"], names)
        self.assertEqual(payload["velocity"], velocity)

    def test_full_master_state_without_velocity_fails_closed_with_wheel_zero(self) -> None:
        names = list(ZERITH_H1_PRO_ACTUATOR_ORDER)
        state = JointState(
            name=names,
            position=[0.0 for _ in names],
            velocity=[],
        )

        payload = master_state_arrow_to_command_payload(state.to_arrow())

        self.assertEqual(payload["name"], names)
        self.assertEqual(payload["velocity"], [0.0 for _ in names])
        command = command_payload_to_joint_command(payload)
        self.assertEqual(command.name, names)
        self.assertEqual(command.velocity, [0.0 for _ in names])

    def test_partial_master_wheel_is_stripped_but_non_wheel_fields_survive(self) -> None:
        state = JointState(
            name=["wheel_left", "lift"],
            position=[0.0, 0.5],
            velocity=[0.4, 0.0],
        )

        payload = master_state_arrow_to_command_payload(state.to_arrow())

        self.assertEqual(payload["name"], ["lift"])
        self.assertEqual(payload["position"], [0.5])
        self.assertEqual(payload["velocity"], [0.0])

    def test_worker_side_reconstruction_rejects_non_finite_payload(self) -> None:
        with self.assertRaises(ValueError):
            locomotion_payload_to_command(
                {"vx": math.inf, "vy": 0.0, "wz": 0.0}
            )

    def test_locomotion_ownership_can_strip_only_wheel_fields(self) -> None:
        payload = {
            "name": ["wheel_left", "lift", "wheel_right"],
            "position": [0.0, 0.5, 0.0],
            "velocity": [0.4, 0.0, 0.4],
            "effort": [],
            "kp": [],
            "kd": [],
            "received_at": 12.0,
        }

        stripped = command_payload_without_wheels(payload)

        self.assertEqual(stripped["name"], ["lift"])
        self.assertEqual(stripped["position"], [0.5])
        self.assertEqual(stripped["velocity"], [0.0])
        self.assertEqual(stripped["received_at"], 12.0)

    def test_worker_applies_21_positions_without_claiming_wheel_ownership(self) -> None:
        class _MotorControl:
            def __init__(self) -> None:
                self.Speed = 0.0
                self.Position = 0.0

        class _Robot:
            def __init__(self) -> None:
                self.chassis_calls: list[tuple[int, float]] = []
                self.motor_calls: list[tuple[int, float]] = []

            def setChassis_low(self, motor_id: int, control: _MotorControl) -> bool:
                self.chassis_calls.append((motor_id, float(control.Speed)))
                return True

            def setMotorControl_low(
                self,
                motor_id: int,
                control: _MotorControl,
            ) -> bool:
                self.motor_calls.append((motor_id, float(control.Position)))
                return True

        names = list(ZERITH_H1_PRO_ACTUATOR_ORDER)
        robot = _Robot()
        driver = object.__new__(ZerithH1ProDriver)
        driver._robot = robot
        driver._sdk = SimpleNamespace(Motor_Control=_MotorControl)
        driver._actuator_order = names
        driver._enum_map = {name: index for index, name in enumerate(names)}
        driver._wheel_enum_map = {"wheel_left": 0, "wheel_right": 1}
        driver._control_mode_lock = threading.RLock()
        driver._locomotion_lock = threading.Lock()
        driver._disconnecting = False
        driver._lifecycle_command_active = False
        driver._non_action_mode_warned = False
        driver._last_locomotion_command_at = None
        driver._locomotion_active = False
        driver._locomotion_timeout_latched = False
        driver._chassis_stop_unconfirmed = False
        driver.get_current_control_mode = lambda: "low_level"  # type: ignore[method-assign]
        payload = {
            "name": names,
            "position": [0.0 for _ in names],
            "velocity": [],
            "received_at": 10.0,
        }

        arbitrated = arbitrate_joint_payload(
            payload,
            now=10.1,
            locomotion_owner_until=9.0,
            command_timeout=0.25,
        )
        applied = dispatch_joint_payload(
            driver,
            arbitrated,
            validation_error_type=JointCommandValidationError,
        )

        self.assertTrue(applied)
        self.assertEqual(robot.chassis_calls, [])
        self.assertEqual(len(robot.motor_calls), 21)
        self.assertEqual(
            [motor_id for motor_id, _ in robot.motor_calls],
            list(range(2, 23)),
        )
        self.assertFalse(driver._locomotion_active)
        self.assertIsNone(driver._last_locomotion_command_at)
        self.assertFalse(driver._locomotion_timeout_latched)
        self.assertFalse(driver._chassis_stop_unconfirmed)

    def test_fresh_locomotion_lease_wins_but_preserves_position_command(self) -> None:
        payload = {
            "name": ["wheel_left", "wheel_right", "lift"],
            "position": [0.0, 0.0, 0.5],
            "velocity": [0.4, 0.4, 0.0],
            "received_at": 10.0,
        }

        result = arbitrate_joint_payload(
            payload,
            now=10.1,
            locomotion_owner_until=10.2,
            command_timeout=0.25,
        )

        self.assertEqual(result["name"], ["lift"])
        self.assertEqual(result["position"], [0.5])

    def test_stale_direct_wheels_are_dropped_and_fresh_wheels_can_take_over(self) -> None:
        payload = {
            "name": ["wheel_left", "wheel_right"],
            "velocity": [0.4, 0.4],
            "received_at": 10.0,
        }
        stale = arbitrate_joint_payload(
            payload,
            now=10.3,
            locomotion_owner_until=9.0,
            command_timeout=0.25,
        )
        fresh = arbitrate_joint_payload(
            {**payload, "received_at": 10.2},
            now=10.3,
            locomotion_owner_until=10.1,
            command_timeout=0.25,
        )

        self.assertEqual(stale["name"], [])
        self.assertEqual(fresh["name"], ["wheel_left", "wheel_right"])

    def test_worker_rejects_bad_wheel_message_and_processes_next_message(self) -> None:
        class _Driver:
            def __init__(self) -> None:
                self.commands: list[list[str]] = []
                self.rejections = 0

            def set_command(self, command: object) -> None:
                names = list(getattr(command, "name"))
                if "wheel_left" in names:
                    self.rejections += 1
                    raise WheelCommandValidationError("bad wheel command")
                self.commands.append(names)

        driver = _Driver()
        bad = {
            "name": ["wheel_left", "wheel_right"],
            "velocity": [0.4, 0.4],
        }
        good = {"name": ["lift"], "position": [0.5]}

        self.assertFalse(
            dispatch_joint_payload(
                driver,
                bad,
                validation_error_type=WheelCommandValidationError,
            )
        )
        self.assertTrue(
            dispatch_joint_payload(
                driver,
                good,
                validation_error_type=WheelCommandValidationError,
            )
        )
        self.assertEqual(driver.rejections, 1)
        self.assertEqual(driver.commands, [["lift"]])

    def test_worker_rejects_bad_locomotion_timestamp_and_processes_next(self) -> None:
        class _Driver:
            def __init__(self) -> None:
                self.commands = 0

            def set_locomotion_command(
                self,
                _command: object,
                *,
                received_at: float,
            ) -> None:
                if received_at > 10.0:
                    raise LocomotionCommandValidationError("future timestamp")
                self.commands += 1

        driver = _Driver()
        bad = {"vx": 0.05, "vy": 0.0, "wz": 0.0, "received_at": 10.1}
        good = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "received_at": 10.0}

        self.assertIsNone(
            dispatch_locomotion_payload(
                driver,
                bad,
                validation_error_type=LocomotionCommandValidationError,
            )
        )
        self.assertEqual(
            dispatch_locomotion_payload(
                driver,
                good,
                validation_error_type=LocomotionCommandValidationError,
            ),
            10.0,
        )
        self.assertEqual(driver.commands, 1)

    def test_unknown_actuator_validation_is_rejected_without_worker_failure(self) -> None:
        class _Driver:
            def __init__(self) -> None:
                self.commands = 0

            def set_command(self, command: object) -> None:
                if "unknown_actuator" in list(getattr(command, "name")):
                    raise JointCommandValidationError("unknown actuator")
                self.commands += 1

        driver = _Driver()
        self.assertFalse(
            dispatch_joint_payload(
                driver,
                {"name": ["unknown_actuator"], "position": [0.0]},
                validation_error_type=JointCommandValidationError,
            )
        )
        self.assertTrue(
            dispatch_joint_payload(
                driver,
                {"name": ["lift"], "position": [0.5]},
                validation_error_type=JointCommandValidationError,
            )
        )
        self.assertEqual(driver.commands, 1)

    def test_worker_does_not_swallow_sdk_or_control_failure(self) -> None:
        class _Driver:
            def set_command(self, _command: object) -> None:
                raise RuntimeError("SDK write failed")

        with self.assertRaisesRegex(RuntimeError, "SDK write failed"):
            dispatch_joint_payload(
                _Driver(),
                {"name": ["lift"], "position": [0.5]},
                validation_error_type=JointCommandValidationError,
            )

    def test_worker_does_not_swallow_locomotion_sdk_value_error(self) -> None:
        class _Driver:
            def set_locomotion_command(
                self,
                _command: object,
                *,
                received_at: float,
            ) -> None:
                raise ValueError(f"native SDK failure at {received_at}")

        with self.assertRaisesRegex(ValueError, "native SDK failure"):
            dispatch_locomotion_payload(
                _Driver(),
                {"vx": 0.0, "vy": 0.0, "wz": 0.0, "received_at": 10.0},
                validation_error_type=LocomotionCommandValidationError,
            )

    def test_malformed_worker_joint_payloads_are_stopped_and_rejected(self) -> None:
        class _Driver:
            def __init__(self) -> None:
                self.stop_calls = 0

            def stop_locomotion(self) -> None:
                self.stop_calls += 1

            def set_command(self, _command: object) -> None:
                raise AssertionError("malformed payload must not reach driver")

        driver = _Driver()
        malformed_payloads = (
            {"name": ["lift", "waist_down"], "position": [0.5]},
            {"name": ["lift"], "position": [math.nan]},
        )
        for payload in malformed_payloads:
            with self.subTest(payload=payload):
                self.assertFalse(
                    dispatch_joint_payload(
                        driver,
                        payload,
                        validation_error_type=JointCommandValidationError,
                    )
                )

        self.assertEqual(driver.stop_calls, len(malformed_payloads))

    def test_malformed_worker_locomotion_payload_is_stopped_and_rejected(self) -> None:
        class _Driver:
            def __init__(self) -> None:
                self.stop_calls = 0

            def stop_locomotion(self) -> None:
                self.stop_calls += 1

            def set_locomotion_command(self, *_args: object, **_kwargs: object) -> None:
                raise AssertionError("malformed payload must not reach driver")

        driver = _Driver()
        self.assertIsNone(
            dispatch_locomotion_payload(
                driver,
                {"vx": math.inf, "vy": 0.0, "wz": 0.0},
                validation_error_type=LocomotionCommandValidationError,
            )
        )
        self.assertEqual(driver.stop_calls, 1)

    def test_malformed_wheel_payload_is_stopped_and_rejected(self) -> None:
        class _Driver:
            def __init__(self) -> None:
                self.stop_calls = 0

            def stop_locomotion(self) -> None:
                self.stop_calls += 1

            def set_command(self, _command: object) -> None:
                raise AssertionError("malformed payload must not reach driver")

        driver = _Driver()
        result = dispatch_joint_payload(
            driver,
            {"name": ["wheel_left", "wheel_right"], "velocity": [0.4]},
            validation_error_type=WheelCommandValidationError,
        )

        self.assertFalse(result)
        self.assertEqual(driver.stop_calls, 1)

    def test_parent_arrow_decoders_reject_non_finite_joint_data(self) -> None:
        with self.assertRaises(CommandMessageValidationError):
            joint_command_arrow_to_payload(
                JointCommand(name=["lift"], position=[math.nan]).to_arrow()
            )
        with self.assertRaises(CommandMessageValidationError):
            master_state_arrow_to_command_payload(
                JointState(name=["lift"], position=[math.inf]).to_arrow()
            )

    def test_parent_arrow_decoders_wrap_all_malformed_control_inputs(self) -> None:
        for decoder in (
            joint_command_arrow_to_payload,
            master_state_arrow_to_command_payload,
            locomotion_arrow_to_payload,
        ):
            with self.subTest(decoder=decoder.__name__):
                with self.assertRaises(CommandMessageValidationError):
                    decoder(object())

    def test_parent_decode_rejection_queues_guarded_zero_and_continues(self) -> None:
        locomotion_queue: queue.Queue[tuple[str, dict[str, float]]] = queue.Queue(
            maxsize=1
        )

        self.assertTrue(
            _reject_parent_control_message(
                "locomotion_command",
                CommandMessageValidationError("bad Arrow payload"),
                locomotion_queue,
            )
        )
        kind, payload = locomotion_queue.get_nowait()
        self.assertEqual(kind, "locomotion_command")
        self.assertEqual(
            {key: payload[key] for key in ("vx", "vy", "wz")},
            {"vx": 0.0, "vy": 0.0, "wz": 0.0},
        )
        self.assertTrue(math.isfinite(payload["received_at"]))

    def test_latest_only_locomotion_queue_preserves_newer_stop(self) -> None:
        locomotion_queue: queue.Queue[object] = queue.Queue(maxsize=1)
        joint_queue: queue.Queue[object] = queue.Queue(maxsize=1)
        joint_message = ("command", {"name": ["lift"], "position": [0.5]})
        joint_queue.put_nowait(joint_message)

        replace_latest_locomotion(
            locomotion_queue,
            {"vx": 0.05, "vy": 0.0, "wz": 0.0},
        )
        replace_latest_locomotion(
            locomotion_queue,
            {"vx": 0.0, "vy": 0.0, "wz": 0.0},
        )

        self.assertEqual(joint_queue.get_nowait(), joint_message)
        self.assertEqual(
            locomotion_queue.get_nowait(),
            (
                "locomotion_command",
                {"vx": 0.0, "vy": 0.0, "wz": 0.0},
            ),
        )
        with self.assertRaises(queue.Empty):
            locomotion_queue.get_nowait()

    def test_parent_shutdown_replaces_pending_nonzero_with_fresh_zero(self) -> None:
        ctx = mp.get_context("spawn")
        locomotion_queue = ctx.Queue(maxsize=1)
        stop_event = ctx.Event()
        locomotion_gate = ctx.Lock()
        locomotion_queue.put_nowait(
            ("locomotion_command", {"vx": 0.05, "vy": 0.0, "wz": 0.0})
        )

        try:
            self.assertTrue(
                coordinate_parent_shutdown(
                    is_follower=True,
                    locomotion_queue=locomotion_queue,
                    locomotion_gate=locomotion_gate,
                    stop_event=stop_event,
                )
            )
            kind, payload = locomotion_queue.get(timeout=1.0)
            self.assertEqual(kind, "locomotion_command")
            self.assertEqual(
                {key: payload[key] for key in ("vx", "vy", "wz")},
                {"vx": 0.0, "vy": 0.0, "wz": 0.0},
            )
            self.assertTrue(math.isfinite(payload["received_at"]))
            self.assertTrue(stop_event.is_set())
        finally:
            locomotion_queue.close()
            locomotion_queue.join_thread()

    def test_parent_shutdown_waits_for_inflight_gate_before_setting_event(self) -> None:
        ctx = mp.get_context("spawn")
        locomotion_queue = ctx.Queue(maxsize=1)
        stop_event = ctx.Event()
        locomotion_gate = ctx.Lock()
        locomotion_queue.put_nowait(
            ("locomotion_command", {"vx": 0.05, "vy": 0.0, "wz": 0.0})
        )
        self.assertTrue(locomotion_gate.acquire())
        gate_held = True
        results: list[bool] = []
        thread = threading.Thread(
            target=lambda: results.append(
                coordinate_parent_shutdown(
                    is_follower=True,
                    locomotion_queue=locomotion_queue,
                    locomotion_gate=locomotion_gate,
                    stop_event=stop_event,
                    gate_timeout=1.0,
                )
            )
        )

        try:
            thread.start()
            self.assertFalse(stop_event.wait(0.05))
            self.assertTrue(thread.is_alive())
            locomotion_gate.release()
            gate_held = False
            thread.join(timeout=1.0)
            self.assertFalse(thread.is_alive())
            self.assertEqual(results, [True])
            self.assertTrue(stop_event.is_set())
            kind, payload = locomotion_queue.get(timeout=1.0)
            self.assertEqual(kind, "locomotion_command")
            self.assertEqual(
                {key: payload[key] for key in ("vx", "vy", "wz")},
                {"vx": 0.0, "vy": 0.0, "wz": 0.0},
            )
        finally:
            if gate_held:
                locomotion_gate.release()
            if thread.is_alive():
                thread.join(timeout=1.0)
            locomotion_queue.close()
            locomotion_queue.join_thread()

    def test_parent_shutdown_gate_timeout_fails_but_sets_stop_event(self) -> None:
        ctx = mp.get_context("spawn")
        locomotion_queue = ctx.Queue(maxsize=1)
        stop_event = ctx.Event()
        locomotion_gate = ctx.Lock()
        self.assertTrue(locomotion_gate.acquire())

        try:
            self.assertFalse(
                coordinate_parent_shutdown(
                    is_follower=True,
                    locomotion_queue=locomotion_queue,
                    locomotion_gate=locomotion_gate,
                    stop_event=stop_event,
                    gate_timeout=0.01,
                )
            )
            self.assertTrue(stop_event.is_set())
            kind, payload = locomotion_queue.get(timeout=1.0)
            self.assertEqual(kind, "locomotion_command")
            self.assertEqual(
                {key: payload[key] for key in ("vx", "vy", "wz")},
                {"vx": 0.0, "vy": 0.0, "wz": 0.0},
            )
        finally:
            locomotion_gate.release()
            locomotion_queue.close()
            locomotion_queue.join_thread()

    def test_parent_shutdown_zero_enqueue_failure_is_reported(self) -> None:
        class _FailingQueue:
            def put_nowait(self, _item: object) -> None:
                raise RuntimeError("queue unavailable")

        ctx = mp.get_context("spawn")
        stop_event = ctx.Event()
        locomotion_gate = ctx.Lock()

        self.assertFalse(
            coordinate_parent_shutdown(
                is_follower=True,
                locomotion_queue=_FailingQueue(),
                locomotion_gate=locomotion_gate,
                stop_event=stop_event,
            )
        )
        self.assertTrue(stop_event.is_set())


if __name__ == "__main__":
    unittest.main()
