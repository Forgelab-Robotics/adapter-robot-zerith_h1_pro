"""Zerith H1 Pro 硬件驱动，实现 forge_robot.RobotDriver 协议。"""

from __future__ import annotations

import logging
import math
import sys
import threading
import time
from typing import Any, NoReturn

from forge_msgs import JointCommand, JointState, LocomotionCommand
from forge_robot import (
    BaseRobotDriver,
    LocomotionSpec,
    clip_and_validate_command,
    clip_and_validate_locomotion_command,
    specs_by_name,
)

from . import sdk
from .actuators import (
    ACTUATOR_CONTROL_FIELD,
    ACTUATOR_ENUM_NAME_MAP,
    ACTUATOR_LIMITS,
    GRIPPER_ACTUATORS,
    LIFT_ACTUATOR,
    WHEEL_ACTUATORS,
    field_by_name,
    gripper_m_to_sdk,
    gripper_sdk_to_m,
    gripper_sdk_velocity_to_mps,
    joint_command_without_wheels,
    lift_m_to_sdk,
    lift_sdk_to_m,
    locomotion_to_wheel_speeds,
    position_by_name,
)
from .config import (
    ZERITH_H1_PRO_ACTUATOR_ORDER,
    ZERITH_H1_PRO_ACTUATOR_SPECS,
    ZERITH_H1_PRO_DEFAULT_POSITIONS,
    ZERITH_H1_PRO_LOCOMOTION_COMMAND_TIMEOUT_SECONDS,
    ZERITH_H1_PRO_MAX_VX_MPS,
    ZERITH_H1_PRO_MAX_WHEEL_SPEED_RADPS,
    ZERITH_H1_PRO_MAX_WZ_RADPS,
    ZERITH_H1_PRO_POSITION_ACTUATOR_ORDER,
    ZERITH_H1_PRO_WHEEL_RADIUS_M,
    ZERITH_H1_PRO_WHEEL_TRACK_WIDTH_M,
    ZerithCameraStreamConfig,
)

logger = logging.getLogger(__name__)


class JointCommandValidationError(ValueError):
    """A JointCommand was rejected before any actuator SDK write."""


class WheelCommandValidationError(JointCommandValidationError):
    """A malformed wheel command rejected after fail-closed handling."""


class LocomotionCommandValidationError(ValueError):
    """A LocomotionCommand was rejected before any chassis SDK write."""


class ActuatorWriteError(RuntimeError):
    """One or more non-wheel actuator SDK writes failed."""


class ChassisModeUnavailableError(RuntimeError):
    """A chassis command was rejected because LOW_LEVEL was not confirmed."""


class ChassisStopUnconfirmedError(RuntimeError):
    """The chassis may be active but cannot be stopped in the actual SDK mode."""


_ACTION_SUPPORTED_MODES = {
    "low_level",
    "gravity_compensation_level",
}

_INIT_STATE_NAMES = (
    "Uninit",
    "Initializing",
    "Init_Complete",
    "Deinitializing",
    "Deinit_Complete",
    "Error_State",
)
_INITIALIZABLE_STATES = {"Uninit", "Deinit_Complete"}
_DEINITIALIZED_STATES = {"Uninit", "Deinit_Complete"}
_INIT_TRANSITION_TIMEOUT_SECONDS = 5.0


class ZerithH1ProDriver(BaseRobotDriver):
    """Zerith H1 Pro 驱动，支持 SDK 全部控制模式切换。"""

    @property
    def actuator_order(self) -> list[str]:
        return list(self._actuator_order)

    @property
    def joint_order(self) -> list[str]:
        return self.actuator_order

    @property
    def chassis_stop_unconfirmed(self) -> bool:
        """Whether physical wheel stop requires operator/E-stop confirmation."""
        with self._locomotion_lock:
            return self._chassis_stop_unconfirmed

    def __init__(
        self,
        *,
        sdk_root: str,
        robot_ip: str | None = None,
        is_follower: bool = True,
        init_on_connect: bool = True,
        control_mode: str = "low_level",
        switch_mode_on_connect: bool = True,
        actuator_order: list[str] | None = None,
        positions: dict[str, dict[str, float]] | None = None,
        lifecycle_position_move_seconds: float = 3.0,
        wheel_radius_m: float = ZERITH_H1_PRO_WHEEL_RADIUS_M,
        wheel_track_width_m: float = ZERITH_H1_PRO_WHEEL_TRACK_WIDTH_M,
        locomotion_max_vx_mps: float = ZERITH_H1_PRO_MAX_VX_MPS,
        locomotion_max_wz_radps: float = ZERITH_H1_PRO_MAX_WZ_RADPS,
        locomotion_max_wheel_speed_radps: float = ZERITH_H1_PRO_MAX_WHEEL_SPEED_RADPS,
        locomotion_command_timeout_seconds: float = (
            ZERITH_H1_PRO_LOCOMOTION_COMMAND_TIMEOUT_SECONDS
        ),
        auto_connect: bool = True,
        camera_grpc_target: str | None = None,
        cameras: list[ZerithCameraStreamConfig] | None = None,
        image_fps: int = 30,
        joint_fps: int = 100,
    ) -> None:
        self.sdk_root = sdk_root
        self.robot_ip = robot_ip
        self.is_follower = is_follower
        self.init_on_connect = init_on_connect
        self.control_mode = sdk.normalize_mode_name(control_mode)
        self.switch_mode_on_connect = switch_mode_on_connect
        self.camera_grpc_target = camera_grpc_target
        self.cameras = cameras
        self.image_fps = image_fps
        self.joint_fps = joint_fps
        self._sdk = sdk.load_h1_sdk(sdk_root)
        self._robot = None
        self._state_cache: JointState | None = None
        self._non_action_mode_warned = False
        self._positions = self._resolve_positions(positions)
        self.lifecycle_position_move_seconds = float(lifecycle_position_move_seconds)
        if (
            not math.isfinite(self.lifecycle_position_move_seconds)
            or self.lifecycle_position_move_seconds <= 0
        ):
            raise ValueError("lifecycle_position_move_seconds must be finite and positive")

        self.wheel_radius_m = float(wheel_radius_m)
        self.wheel_track_width_m = float(wheel_track_width_m)
        self.locomotion_max_wheel_speed_radps = float(
            locomotion_max_wheel_speed_radps
        )
        self.locomotion_command_timeout_seconds = float(
            locomotion_command_timeout_seconds
        )
        locomotion_max_vx_mps = float(locomotion_max_vx_mps)
        locomotion_max_wz_radps = float(locomotion_max_wz_radps)
        locomotion_values = {
            "wheel_radius_m": self.wheel_radius_m,
            "wheel_track_width_m": self.wheel_track_width_m,
            "locomotion_max_vx_mps": locomotion_max_vx_mps,
            "locomotion_max_wz_radps": locomotion_max_wz_radps,
            "locomotion_max_wheel_speed_radps": self.locomotion_max_wheel_speed_radps,
            "locomotion_command_timeout_seconds": self.locomotion_command_timeout_seconds,
        }
        for field_name, value in locomotion_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{field_name} must be finite and positive")
        if self.locomotion_max_wheel_speed_radps > ZERITH_H1_PRO_MAX_WHEEL_SPEED_RADPS:
            raise ValueError(
                "locomotion_max_wheel_speed_radps exceeds the project safety ceiling"
            )
        max_feasible_vx = self.wheel_radius_m * self.locomotion_max_wheel_speed_radps
        if locomotion_max_vx_mps > max_feasible_vx:
            raise ValueError("locomotion_max_vx_mps exceeds the wheel-speed limit")
        max_feasible_wz = (
            2.0
            * self.wheel_radius_m
            * self.locomotion_max_wheel_speed_radps
            / self.wheel_track_width_m
        )
        if locomotion_max_wz_radps > max_feasible_wz:
            raise ValueError("locomotion_max_wz_radps exceeds the wheel-speed limit")
        self._locomotion_spec = LocomotionSpec(
            max_vx=locomotion_max_vx_mps,
            max_wz=locomotion_max_wz_radps,
            allow_lateral=False,
        )

        # 后台线程、相机及快照缓存初始化
        self._camera_client = None
        self._bg_threads: list[threading.Thread] = []
        self._stop_bg_threads = False
        self._joint_lock = threading.Lock()
        self._latest_joint_snapshot: JointState | None = None
        self._camera_lock = threading.Lock()
        self._latest_camera_frames: dict[str, Any] = {}
        self._lifecycle_lock = threading.RLock()
        self._control_mode_lock = threading.RLock()
        self._locomotion_lock = threading.Lock()
        self._disconnecting = False
        self._lifecycle_command_active = False
        self._last_locomotion_command_at: float | None = None
        self._locomotion_active = False
        self._locomotion_timeout_latched = False
        self._chassis_stop_unconfirmed = False

        order = list(actuator_order) if actuator_order else list(ZERITH_H1_PRO_ACTUATOR_ORDER)
        unknown = [name for name in order if name not in ACTUATOR_ENUM_NAME_MAP]
        if unknown:
            raise ValueError(f"actuator_order 包含未知执行器: {unknown}")
        if not order:
            raise ValueError("actuator_order 不能为空")
        self._actuator_order = order

        self._enum_map = {}
        enum_cls = self._sdk.EtherCAT_Motor_Index
        for name in self._actuator_order:
            self._enum_map[name] = getattr(enum_cls, ACTUATOR_ENUM_NAME_MAP[name])
        self._wheel_enum_map = {
            name: getattr(enum_cls, ACTUATOR_ENUM_NAME_MAP[name])
            for name in WHEEL_ACTUATORS
        }

        if auto_connect:
            self.connect()

    @staticmethod
    def _resolve_positions(
        positions: dict[str, dict[str, float]] | None,
    ) -> dict[str, dict[str, float]]:
        resolved = {
            name: dict(position)
            for name, position in ZERITH_H1_PRO_DEFAULT_POSITIONS.items()
        }
        if positions:
            for name, position in positions.items():
                if name not in resolved:
                    raise ValueError(f"unknown lifecycle position: {name}")
                for actuator, raw_value in position.items():
                    if actuator not in ZERITH_H1_PRO_POSITION_ACTUATOR_ORDER:
                        raise ValueError(f"unknown position actuator: {actuator}")
                    value = float(raw_value)
                    if not math.isfinite(value):
                        raise ValueError(
                            f"lifecycle position must be finite: {name}.{actuator}"
                        )
                    resolved[name][actuator] = value
        return resolved

    @classmethod
    def supported_control_modes(cls) -> list[str]:
        return list(sdk.MODE_NAME_TO_SDK_ATTR.keys())

    def get_current_control_mode(self) -> str:
        if self._robot is None:
            raise RuntimeError("Robot is not connected. Call connect() first.")
        mode_value = self._robot.getCurrentMode()
        return sdk.mode_name_from_value(int(mode_value), self._sdk)

    def get_init_state(self) -> str:
        if self._robot is None:
            raise RuntimeError("Robot is not connected. Call connect() first.")
        value = int(self._robot.getInitState())
        enum_cls = self._sdk.InitState
        for name in _INIT_STATE_NAMES:
            if value == int(getattr(enum_cls, name)):
                return name
        return f"Unknown({value})"

    def _wait_for_init_state(
        self,
        accepted: set[str],
        *,
        operation: str,
        wait_timeout: float = _INIT_TRANSITION_TIMEOUT_SECONDS,
        poll_interval: float = 0.05,
    ) -> str:
        deadline = time.monotonic() + max(wait_timeout, 0.0)
        while True:
            state = self.get_init_state()
            if state in accepted:
                return state
            if state.startswith("Unknown("):
                raise RuntimeError(f"{operation} 遇到未知 init state: {state}")
            if time.monotonic() >= deadline:
                raise TimeoutError(f"{operation} 超时，当前 init_state={state}")
            time.sleep(max(poll_interval, 0.01))

    def _ensure_deinitialized(self) -> str:
        robot = self._robot
        if robot is None:
            raise RuntimeError("Robot is not connected. Call connect() first.")
        state = self.get_init_state()
        if state == "Initializing":
            state = self._wait_for_init_state(
                {"Init_Complete", "Error_State"},
                operation="等待 robot_init 完成后执行 deinit",
            )
        if state == "Init_Complete":
            if not bool(robot.robot_deinit()):
                state = self.get_init_state()
                if state not in {"Deinitializing", *_DEINITIALIZED_STATES}:
                    raise RuntimeError(
                        f"H1 robot_deinit 失败，当前 init_state={state}"
                    )
            state = self._wait_for_init_state(
                _DEINITIALIZED_STATES | {"Error_State"},
                operation="等待 robot_deinit 完成",
            )
            if state == "Error_State":
                raise RuntimeError("robot_deinit 进入 Error_State")
            return state
        if state == "Deinitializing":
            state = self._wait_for_init_state(
                _DEINITIALIZED_STATES | {"Error_State"},
                operation="等待已有 robot_deinit 完成",
            )
            if state == "Error_State":
                raise RuntimeError("已有 robot_deinit 进入 Error_State")
            return state
        if state in _DEINITIALIZED_STATES:
            return state
        raise RuntimeError(f"无法从 init_state={state} 执行 robot_deinit")

    def _ensure_switchable(self) -> str:
        state = self.get_init_state()
        if state == "Error_State":
            logger.warning("控制器处于 Error_State，尝试通过模式切换恢复。")
            return state
        return self._ensure_deinitialized()

    def _ensure_initialized(self) -> str:
        robot = self._robot
        if robot is None:
            raise RuntimeError("Robot is not connected. Call connect() first.")
        state = self.get_init_state()
        if state == "Init_Complete":
            return state
        if state == "Initializing":
            state = self._wait_for_init_state(
                {"Init_Complete", "Error_State"},
                operation="等待已有 robot_init 完成",
            )
            if state == "Init_Complete":
                return state
            raise RuntimeError("已有 robot_init 进入 Error_State")
        if state == "Deinitializing":
            state = self._wait_for_init_state(
                _DEINITIALIZED_STATES,
                operation="等待 robot_deinit 后重新初始化",
            )
        if state not in _INITIALIZABLE_STATES:
            raise RuntimeError(f"无法从 init_state={state} 执行 robot_init")
        if not bool(robot.robot_init()):
            state = self.get_init_state()
            if state not in {"Initializing", "Init_Complete"}:
                raise RuntimeError(f"H1 robot_init 失败，当前 init_state={state}")
        state = self._wait_for_init_state(
            {"Init_Complete", "Error_State"},
            operation="等待 robot_init 完成",
        )
        if state != "Init_Complete":
            raise RuntimeError(f"H1 robot_init 未完成，当前 init_state={state}")
        return state

    def switch_control_mode(
        self,
        mode: str,
        *,
        wait_timeout: float = 3.0,
        poll_interval: float = 0.05,
    ) -> str:
        with self._lifecycle_lock:
            with self._control_mode_lock:
                with self._locomotion_lock:
                    return self._switch_control_mode_locked(
                        mode,
                        wait_timeout=wait_timeout,
                        poll_interval=poll_interval,
                    )

    def _switch_control_mode_locked(
        self,
        mode: str,
        *,
        wait_timeout: float,
        poll_interval: float,
    ) -> str:
        if self._robot is None:
            raise RuntimeError("Robot is not connected. Call connect() first.")
        mode_name = sdk.normalize_mode_name(mode)
        current = self.get_current_control_mode()
        current_init_state = self.get_init_state()
        if current == "low_level" and mode_name != "low_level":
            self._stop_wheels_locked()
        if current == mode_name and current_init_state != "Error_State":
            if mode_name == "low_level" and (
                self._locomotion_active or self._chassis_stop_unconfirmed
            ):
                self._stop_wheels_locked()
            self.control_mode = mode_name
            logger.info(
                "控制模式已是 %s，跳过 switchControlMode；init_state=%s。",
                mode_name,
                current_init_state,
            )
            return current

        restore_initialized = current_init_state in {"Initializing", "Init_Complete"}
        init_state = self._ensure_switchable()
        enum_attr = sdk.MODE_NAME_TO_SDK_ATTR[mode_name]
        enum_value = getattr(self._sdk.MotorControlMode, enum_attr)

        if not bool(self._robot.switchControlMode(enum_value)):
            raise RuntimeError(
                f"切换控制模式失败: {mode_name}，当前模式={current}，"
                f"init_state={self.get_init_state()}"
            )

        deadline = time.monotonic() + max(wait_timeout, 0.0)
        while True:
            current = self.get_current_control_mode()
            if current == mode_name:
                if mode_name == "low_level" and (
                    self._locomotion_active or self._chassis_stop_unconfirmed
                ):
                    self._stop_wheels_locked()
                self.control_mode = mode_name
                if restore_initialized and mode_name != "uninitialized":
                    try:
                        self._ensure_initialized()
                    except Exception:
                        try:
                            self._ensure_deinitialized()
                        except Exception:
                            logger.exception("模式切换后恢复 robot_init 失败，且清理异常。")
                        raise
                return current
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"等待控制模式切换超时，目标={mode_name}，当前={current}，"
                    f"init_state={init_state}"
                )
            time.sleep(max(poll_interval, 0.01))

    def _joint_loop(self) -> None:
        logger.info("Zerith H1 Pro 关节状态采集线程已启动。")
        while not self._stop_bg_threads:
            try:
                if self._robot is not None:
                    cached_positions = (
                        position_by_name(self._state_cache) if self._state_cache is not None else {}
                    )
                    names: list[str] = []
                    positions: list[float] = []
                    velocities: list[float] = []
                    for name in self._actuator_order:
                        if name in WHEEL_ACTUATORS:
                            ok, info = self._robot.getChassisState(self._enum_map[name])
                        else:
                            ok, info = self._robot.getMotorState(self._enum_map[name])
                        if not bool(ok):
                            names.append(name)
                            positions.append(cached_positions.get(name, 0.0))
                            velocities.append(0.0)
                            continue

                        field = ACTUATOR_CONTROL_FIELD.get(name, "position")
                        if field == "speed":
                            position = float(info.Position_Actual)
                            velocity = float(info.Speed_Actual)
                        elif name == LIFT_ACTUATOR:
                            position = lift_sdk_to_m(float(info.Position_Actual))
                            velocity = float(info.Speed_Actual)
                        elif name in GRIPPER_ACTUATORS:
                            position = gripper_sdk_to_m(float(info.Position_Actual))
                            velocity = gripper_sdk_velocity_to_mps(
                                float(info.Speed_Actual)
                            )
                        else:
                            position = float(info.Position_Actual)
                            velocity = float(info.Speed_Actual)
                        names.append(name)
                        positions.append(position)
                        velocities.append(velocity)

                    state = JointState(name=names, position=positions, velocity=velocities, effort=[])
                    self._state_cache = state
                    with self._joint_lock:
                        self._latest_joint_snapshot = state
            except Exception as e:
                logger.error("H1 Pro 关节采集线程异常: %s", e)
            time.sleep(1.0 / self.joint_fps)

    def _record_wheels_stopped_locked(self, *, timeout_latched: bool = False) -> None:
        self._last_locomotion_command_at = None
        self._locomotion_active = False
        self._locomotion_timeout_latched = timeout_latched
        self._chassis_stop_unconfirmed = False

    def _raise_stop_unconfirmed_locked(
        self,
        operation: str,
        *,
        cause: Exception | None = None,
    ) -> NoReturn:
        self._chassis_stop_unconfirmed = True
        message = (
            f"{operation}: a complete two-wheel stop could not be confirmed. "
            "Physical E-stop required."
        )
        logger.critical(message)
        raise ChassisStopUnconfirmedError(message) from cause

    def _raise_unavailable_chassis_mode_locked(
        self,
        operation: str,
        *,
        mode: str | None = None,
        cause: Exception | None = None,
    ) -> NoReturn:
        detail = (
            "actual control mode could not be read"
            if mode is None
            else f"actual mode is {mode}"
        )
        if self._locomotion_active or self._chassis_stop_unconfirmed:
            first_report = not self._chassis_stop_unconfirmed
            self._chassis_stop_unconfirmed = True
            message = (
                f"{operation}: chassis may still be active but {detail}; "
                "no setChassis_low command was sent. Physical E-stop required."
            )
            if first_report:
                logger.critical(message)
            raise ChassisStopUnconfirmedError(message) from cause
        raise ChassisModeUnavailableError(
            f"{operation} requires confirmed actual control mode low_level; {detail}; "
            "no setChassis_low command was sent"
        ) from cause

    def _require_low_level_for_chassis_write_locked(self, operation: str) -> None:
        """Re-check actual mode immediately before every setChassis_low call."""
        try:
            current_mode = self.get_current_control_mode()
        except Exception as exc:
            self._raise_unavailable_chassis_mode_locked(operation, cause=exc)
        if current_mode != "low_level":
            self._raise_unavailable_chassis_mode_locked(operation, mode=current_mode)

    def _set_chassis_low_confirmed_locked(
        self,
        name: str,
        speed: float,
        *,
        operation: str,
    ) -> bool:
        """The only native chassis-write primitive; actual mode is checked per call."""
        robot = self._robot
        if robot is None:
            raise RuntimeError("Robot is not connected. Call connect() first.")
        self._require_low_level_for_chassis_write_locked(operation)
        if speed != 0.0:
            # Once a non-zero native call is attempted, physical motion must be
            # considered possible until a complete two-wheel zero is confirmed.
            self._locomotion_active = True
            self._last_locomotion_command_at = (
                time.monotonic() - self.locomotion_command_timeout_seconds - 1.0
            )
            self._locomotion_timeout_latched = False
        control = self._sdk.Motor_Control()
        control.Speed = float(speed)
        return bool(robot.setChassis_low(self._wheel_enum_map[name], control))

    def _best_effort_zero_wheels_locked(self) -> bool:
        if self._robot is None:
            self._raise_stop_unconfirmed_locked(
                "compensating chassis stop while disconnected"
            )

        failures: list[str] = []
        for name in ("wheel_left", "wheel_right"):
            try:
                if not self._set_chassis_low_confirmed_locked(
                    name,
                    0.0,
                    operation=f"compensating stop for {name}",
                ):
                    failures.append(f"{name}: SDK returned false")
            except ChassisModeUnavailableError as exc:
                self._raise_stop_unconfirmed_locked(
                    f"compensating stop for {name}",
                    cause=exc,
                )
            except ChassisStopUnconfirmedError:
                raise
            except Exception as exc:
                failures.append(f"{name}: {exc}")

        if failures:
            logger.error("底盘故障收停失败: %s", "; ".join(failures))
            return False
        return True

    def _write_wheel_speeds_locked(self, left: float, right: float) -> None:
        """Write both wheels while control-mode and locomotion locks are held."""
        complete_stop_requested = left == 0.0 and right == 0.0
        if self._robot is None:
            if complete_stop_requested:
                self._raise_stop_unconfirmed_locked("chassis stop while disconnected")
            raise RuntimeError("Robot is not connected. Call connect() first.")

        failures: list[str] = []
        for name, speed in (
            ("wheel_left", float(left)),
            ("wheel_right", float(right)),
        ):
            try:
                if not self._set_chassis_low_confirmed_locked(
                    name,
                    speed,
                    operation=f"wheel write for {name}",
                ):
                    failures.append(f"{name}: SDK returned false")
            except ChassisModeUnavailableError as exc:
                if complete_stop_requested:
                    self._raise_stop_unconfirmed_locked(
                        f"wheel stop write for {name}",
                        cause=exc,
                    )
                raise
            except ChassisStopUnconfirmedError:
                raise
            except Exception as exc:
                failures.append(f"{name}: {exc}")

        if failures:
            stop_confirmed = self._best_effort_zero_wheels_locked()
            if stop_confirmed:
                self._record_wheels_stopped_locked()
            else:
                self._raise_stop_unconfirmed_locked(
                    "setChassis_low failed compensation"
                )
            raise RuntimeError(
                "setChassis_low failed; wheels were commanded to stop: "
                + "; ".join(failures)
            )

    def set_locomotion_command(
        self,
        command: LocomotionCommand,
        *,
        received_at: float | None = None,
    ) -> None:
        """Apply a fresh planar velocity command using only the LOW_LEVEL chassis API."""
        with self._control_mode_lock:
            with self._locomotion_lock:
                if self._disconnecting:
                    raise RuntimeError(
                        "Robot is disconnecting; locomotion commands are rejected."
                    )
                if self._robot is None:
                    raise RuntimeError("Robot is not connected. Call connect() first.")
                now = time.monotonic()
                try:
                    command_received_at = (
                        now if received_at is None else float(received_at)
                    )
                except (TypeError, ValueError, OverflowError) as exc:
                    if self._locomotion_active or self._chassis_stop_unconfirmed:
                        self._stop_wheels_locked()
                    raise LocomotionCommandValidationError(
                        "received_at must be a finite monotonic time not in the future"
                    ) from exc
                if (
                    not math.isfinite(command_received_at)
                    or command_received_at > now
                ):
                    if self._locomotion_active or self._chassis_stop_unconfirmed:
                        self._stop_wheels_locked()
                    raise LocomotionCommandValidationError(
                        "received_at must be a finite monotonic time not in the future"
                    )

                try:
                    safe_command = clip_and_validate_locomotion_command(
                        command,
                        self._locomotion_spec,
                    )
                except (TypeError, ValueError) as exc:
                    if self._locomotion_active or self._chassis_stop_unconfirmed:
                        self._stop_wheels_locked()
                    raise LocomotionCommandValidationError(
                        f"invalid LocomotionCommand: {exc}"
                    ) from exc

                self._require_low_level_for_chassis_write_locked(
                    "locomotion command"
                )
                if self._chassis_stop_unconfirmed:
                    self._stop_wheels_locked()
                left, right = locomotion_to_wheel_speeds(
                    safe_command,
                    wheel_radius_m=self.wheel_radius_m,
                    wheel_track_width_m=self.wheel_track_width_m,
                    max_wheel_speed_radps=self.locomotion_max_wheel_speed_radps,
                )
                if (
                    (left != 0.0 or right != 0.0)
                    and now - command_received_at
                    > self.locomotion_command_timeout_seconds
                ):
                    self._write_wheel_speeds_locked(0.0, 0.0)
                    self._record_wheels_stopped_locked(timeout_latched=True)
                    logger.warning("丢弃已过期的 locomotion_command，并保持底盘停止。")
                    return

                self._write_wheel_speeds_locked(left, right)
                if left == 0.0 and right == 0.0:
                    self._record_wheels_stopped_locked()
                else:
                    self._last_locomotion_command_at = command_received_at
                    self._locomotion_active = True
                    self._locomotion_timeout_latched = False
                    self._chassis_stop_unconfirmed = False

    def _stop_wheels_locked(self) -> None:
        if self._robot is None:
            self._raise_stop_unconfirmed_locked(
                "chassis stop while robot is disconnected"
            )
        self._write_wheel_speeds_locked(0.0, 0.0)
        self._record_wheels_stopped_locked()

    def stop_locomotion(self) -> None:
        """Explicitly stop both wheels, or raise if LOW_LEVEL cannot be confirmed."""
        with self._control_mode_lock:
            with self._locomotion_lock:
                self._stop_wheels_locked()

    def _apply_locomotion_watchdog(self, now: float | None = None) -> bool:
        """Stop stale locomotion once; return whether this call latched a timeout."""
        current_time = time.monotonic() if now is None else float(now)
        with self._control_mode_lock:
            with self._locomotion_lock:
                last_command_at = self._last_locomotion_command_at
                if (
                    self._disconnecting
                    or not self._locomotion_active
                    or self._locomotion_timeout_latched
                    or last_command_at is None
                    or current_time - last_command_at
                    <= self.locomotion_command_timeout_seconds
                ):
                    return False
                try:
                    self._write_wheel_speeds_locked(0.0, 0.0)
                except ChassisStopUnconfirmedError:
                    return False
                except Exception:
                    logger.exception("locomotion watchdog 无法停止底盘，将继续重试。")
                    return False
                self._record_wheels_stopped_locked(timeout_latched=True)

        logger.warning(
            "locomotion_command 超时 %.3f 秒，已停止底盘。",
            self.locomotion_command_timeout_seconds,
        )
        return True

    def _locomotion_watchdog_loop(self) -> None:
        logger.info("Zerith H1 Pro locomotion watchdog 已启动。")
        interval = min(0.05, self.locomotion_command_timeout_seconds / 2.0)
        while not self._stop_bg_threads:
            self._apply_locomotion_watchdog()
            time.sleep(max(interval, 0.005))

    def _camera_loop(self) -> None:
        from . import camera_io

        logger.info("Zerith H1 Pro 相机拉取线程已启动。")
        cv2 = camera_io.load_cv2()
        while not self._stop_bg_threads:
            try:
                if self._camera_client is not None:
                    for camera in self.cameras or []:
                        if camera.enable_color:
                            data = self._camera_client.get_latest_frame(camera.name)
                            if data:
                                frame, timestamp = data
                                payload = camera_io.build_color_payload(
                                    frame,
                                    timestamp,
                                    camera,
                                    cv2,
                                )
                                with self._camera_lock:
                                    self._latest_camera_frames[camera.output_id] = payload

                        if camera.enable_depth and camera.depth_output_id:
                            data = self._camera_client.get_latest_depth(camera.name)
                            if data:
                                depth, timestamp = data
                                payload = camera_io.build_depth_payload(
                                    depth,
                                    timestamp,
                                    camera,
                                    cv2,
                                )
                                with self._camera_lock:
                                    self._latest_camera_frames[
                                        camera.depth_output_id
                                    ] = payload
            except Exception as e:
                logger.error("H1 Pro 相机采集线程异常: %s", e)
            time.sleep(1.0 / self.image_fps)

    def get_latest_encoded_image(self, output_id: str) -> Any | None:
        with self._camera_lock:
            return self._latest_camera_frames.get(output_id)

    def connect(self) -> None:
        with self._lifecycle_lock:
            with self._control_mode_lock:
                self._connect_locked()

    def _connect_locked(self) -> None:
        if self.is_follower and self.control_mode != "low_level":
            raise ValueError("follower control_mode must be low_level")
        with self._locomotion_lock:
            self._disconnecting = False

        if self._robot is not None and bool(self._robot.isRobotConnected()):
            logger.warning("Robot is already connected.")
            return

        if sys.version_info[:2] != (3, 12):
            logger.warning(
                "当前 Python 版本是 %s.%s；包内 SDK 为 Python 3.12 构建，若导入/运行异常请切换到 Python 3.12。",
                sys.version_info.major,
                sys.version_info.minor,
            )

        self._robot = (
            self._sdk.H1Robot(self.robot_ip) if self.robot_ip else self._sdk.H1Robot()
        )
        if not bool(self._robot.robot_connect()):
            self._robot = None
            raise RuntimeError("H1 robot_connect 失败。")

        try:
            logger.info(
                "H1 connected state: control_mode=%s, init_state=%s.",
                self.get_current_control_mode(),
                self.get_init_state(),
            )
            if self.switch_mode_on_connect:
                current = self.switch_control_mode(self.control_mode)
            else:
                current = self.get_current_control_mode()

            if self.is_follower and current != "low_level":
                raise RuntimeError(
                    "follower control requires actual mode low_level; "
                    f"current mode is {current}"
                )
            if current == "low_level":
                with self._locomotion_lock:
                    if self._locomotion_active or self._chassis_stop_unconfirmed:
                        self._stop_wheels_locked()
            if self.init_on_connect and current != "uninitialized":
                self._ensure_initialized()
            logger.info(
                "Connected to Zerith H1 Pro, control_mode=%s, init_state=%s.",
                current,
                self.get_init_state(),
            )

            if self.is_follower:
                self.move_to_home_position()
        except Exception:
            if self.init_on_connect and self._robot is not None:
                try:
                    self._ensure_deinitialized()
                except Exception:
                    logger.exception("连接失败后 robot_deinit 异常。")
            self._robot = None
            raise

        # 启动后台线程
        self._stop_bg_threads = False
        self._bg_threads = []

        # 1. 启动关节状态线程
        joint_thread = threading.Thread(target=self._joint_loop, name="H1ProJointLoop", daemon=True)
        joint_thread.start()
        self._bg_threads.append(joint_thread)

        watchdog_thread = threading.Thread(
            target=self._locomotion_watchdog_loop,
            name="H1ProLocomotionWatchdog",
            daemon=True,
        )
        watchdog_thread.start()
        self._bg_threads.append(watchdog_thread)

        # 2. 启动相机线程
        if self.cameras:
            try:
                from .camera_io import connect_client

                self._camera_client = connect_client(
                    self.sdk_root,
                    self.camera_grpc_target or "",
                )
                camera_thread = threading.Thread(target=self._camera_loop, name="H1ProCameraLoop", daemon=True)
                camera_thread.start()
                self._bg_threads.append(camera_thread)
            except Exception as e:
                logger.error("连接相机服务失败: %s", e)
                self._camera_client = None

    def _read_actuator_positions(self, actuator_names: list[str]) -> list[float]:
        """同步读取位置执行器，生命周期移动不得使用虚构的零位作为起点。"""
        if self._robot is None:
            raise RuntimeError("Robot is not connected. Call connect() first.")

        positions: list[float] = []
        for actuator in actuator_names:
            ok, info = self._robot.getMotorState(self._enum_map[actuator])
            if not bool(ok):
                raise RuntimeError(f"读取生命周期位姿起点失败: {actuator}")

            position = float(info.Position_Actual)
            if not math.isfinite(position):
                raise RuntimeError(f"生命周期位姿起点不是有限数值: {actuator}")
            if actuator == LIFT_ACTUATOR:
                position = lift_sdk_to_m(position)
            elif actuator in GRIPPER_ACTUATORS:
                position = gripper_sdk_to_m(position)
            positions.append(position)
        return positions

    def move_to_home_position(self) -> None:
        """将位置执行器移动到配置的连接初始位。"""
        self._move_to_position("initial", self._positions["initial"])

    def move_to_reset_position(self) -> None:
        """将位置执行器移动到配置的断开安全位。"""
        self._move_to_position("reset", self._positions["reset"])

    def _move_to_position(self, name: str, target: dict[str, float]) -> None:
        """从真实当前位置平滑插值到生命周期位姿。"""
        if self._robot is None:
            raise RuntimeError("Robot is not connected. Call connect() first.")

        current_mode = self.get_current_control_mode()
        if current_mode not in _ACTION_SUPPORTED_MODES:
            raise RuntimeError(
                f"当前控制模式 {current_mode} 不支持生命周期位姿移动"
            )

        actuator_names = [
            actuator
            for actuator in ZERITH_H1_PRO_POSITION_ACTUATOR_ORDER
            if actuator in self._actuator_order
        ]
        if not actuator_names:
            logger.warning("actuator_order 中没有可执行 positions.%s 的位置执行器。", name)
            return

        start = self._read_actuator_positions(actuator_names)
        goal = [target.get(actuator, 0.0) for actuator in actuator_names]
        duration = self.lifecycle_position_move_seconds
        rate_hz = min(max(float(self.joint_fps), 10.0), 100.0)
        steps = max(int(duration * rate_hz), 1)
        sleep_seconds = duration / steps

        logger.info(
            "Moving Zerith H1 Pro to %s position over %.2fs: %s",
            name,
            duration,
            target,
        )
        for step in range(1, steps + 1):
            ratio = step / steps
            blend = ratio * ratio * (3.0 - 2.0 * ratio)
            command = JointCommand(
                name=actuator_names,
                position=[
                    start_value + (goal_value - start_value) * blend
                    for start_value, goal_value in zip(start, goal, strict=True)
                ],
                velocity=[],
                effort=[],
                kp=[],
                kd=[],
            )
            self.set_command(command)
            if step < steps:
                time.sleep(sleep_seconds)

    def disconnect(self) -> None:
        with self._lifecycle_lock:
            with self._control_mode_lock:
                self._disconnect_locked()

    def _disconnect_locked(self) -> None:
        logger.info("正在断开 Zerith H1 Pro 连接并释放后台线程...")

        with self._locomotion_lock:
            if self._disconnecting and self._robot is None:
                return
            self._disconnecting = True

        locomotion_stop_failed = False
        disconnect_mode: str | None = None
        if self._robot is not None:
            try:
                disconnect_mode = self.get_current_control_mode()
            except Exception:
                locomotion_stop_failed = True
                logger.exception(
                    "断开连接前无法确认实际控制模式；不发送 LOW_LEVEL 底盘命令。"
                )
                with self._locomotion_lock:
                    self._chassis_stop_unconfirmed = True
            else:
                if disconnect_mode == "low_level":
                    for attempt in range(1, 4):
                        try:
                            self.stop_locomotion()
                            locomotion_stop_failed = False
                            break
                        except Exception:
                            locomotion_stop_failed = True
                            logger.exception("断开连接前第 %d 次停止底盘失败。", attempt)
                            time.sleep(0.02)
                else:
                    with self._locomotion_lock:
                        locomotion_may_be_active = (
                            self._locomotion_active
                            or self._chassis_stop_unconfirmed
                        )
                        if locomotion_may_be_active:
                            self._chassis_stop_unconfirmed = True
                        else:
                            self._record_wheels_stopped_locked()
                    if locomotion_may_be_active:
                        locomotion_stop_failed = True
                        logger.critical(
                            "底盘可能仍在活动，但实际模式为 %s；"
                            "拒绝跨模式发送 LOW_LEVEL 停车命令，必须使用物理急停。",
                            disconnect_mode,
                        )
                    else:
                        logger.info(
                            "实际模式为 %s，跳过 LOW_LEVEL 底盘停车命令。",
                            disconnect_mode,
                        )
            if locomotion_stop_failed:
                logger.critical("无法确认底盘安全停车；跳过 reset movement 并继续释放控制。")

        # 先停止后台 SDK 读取，避免 lifecycle movement/deinit 与 getMotorState 并发。
        self._stop_bg_threads = True
        for thread in self._bg_threads:
            try:
                thread.join(timeout=2.0)
                if thread.is_alive():
                    logger.warning("后台线程 %s 未在超时内退出。", thread.name)
            except Exception:
                logger.exception("等待后台线程 %s 退出异常。", thread.name)
        self._bg_threads.clear()

        lifecycle_failure: Exception | None = None
        if (
            self._robot is not None
            and self.is_follower
            and disconnect_mode == "low_level"
            and not locomotion_stop_failed
        ):
            self._lifecycle_command_active = True
            try:
                self.move_to_reset_position()
            except Exception as exc:
                lifecycle_failure = exc
                logger.exception("Failed to move to reset position.")
            finally:
                self._lifecycle_command_active = False

        if self._robot is not None:
            if self.init_on_connect:
                try:
                    final_state = self._ensure_deinitialized()
                    logger.info("H1 robot_deinit completed, init_state=%s.", final_state)
                except Exception:
                    logger.exception("robot_deinit 异常。")
            self._robot = None
            logger.info("Disconnected from Zerith H1 Pro.")

        if hasattr(self, "_camera_client") and self._camera_client is not None:
            self._camera_client = None

        if locomotion_stop_failed:
            with self._locomotion_lock:
                self._chassis_stop_unconfirmed = True
            raise RuntimeError(
                "failed to confirm chassis stop during disconnect; physical E-stop required"
            )
        if lifecycle_failure is not None:
            raise RuntimeError(
                "failed to complete reset movement during disconnect"
            ) from lifecycle_failure

    def get_state(self) -> JointState:
        with self._joint_lock:
            snapshot = self._latest_joint_snapshot
        if snapshot is not None:
            return snapshot

        # 兜底：如果后台线程还没拿到，且连接正常，返回已缓存的或全零状态
        if self._robot is None:
            raise RuntimeError("Robot is not connected. Call connect() first.")

        if self._state_cache is not None:
            return self._state_cache

        return JointState(
            name=list(self._actuator_order),
            position=[0.0 for _ in self._actuator_order],
            velocity=[0.0 for _ in self._actuator_order],
            effort=[],
        )

    def set_command(self, command: JointCommand) -> None:
        with self._control_mode_lock:
            if self._disconnecting and not self._lifecycle_command_active:
                raise RuntimeError("Robot is disconnecting; commands are rejected.")
            if self._robot is None:
                raise RuntimeError("Robot is not connected. Call connect() first.")

            named_wheels = WHEEL_ACTUATORS & set(command.name)
            if named_wheels and not command.velocity:
                with self._locomotion_lock:
                    if self._locomotion_active or self._chassis_stop_unconfirmed:
                        self._stop_wheels_locked()
                command = joint_command_without_wheels(command)
                if not command.name:
                    return

            requested_wheels = (
                WHEEL_ACTUATORS & set(command.name) if command.velocity else set()
            )
            non_finite_fields = [
                field_name
                for field_name in ("position", "velocity", "effort", "kp", "kd")
                if any(
                    not math.isfinite(float(value))
                    for value in getattr(command, field_name)
                )
            ]
            if non_finite_fields:
                message = (
                    "JointCommand fields must contain only finite values: "
                    + ", ".join(non_finite_fields)
                )
                if requested_wheels:
                    with self._locomotion_lock:
                        self._stop_wheels_locked()
                    raise WheelCommandValidationError(message)
                raise JointCommandValidationError(message)

            actuator_specs = specs_by_name(ZERITH_H1_PRO_ACTUATOR_SPECS)
            unknown_actuators = [
                name for name in command.name if name not in actuator_specs
            ]
            if unknown_actuators:
                message = "unknown JointCommand actuators: " + ", ".join(
                    unknown_actuators
                )
                if requested_wheels:
                    with self._locomotion_lock:
                        self._stop_wheels_locked()
                    raise WheelCommandValidationError(message)
                raise JointCommandValidationError(message)

            try:
                safe_command = clip_and_validate_command(command, actuator_specs)
            except (KeyError, TypeError, ValueError) as exc:
                if requested_wheels:
                    with self._locomotion_lock:
                        self._stop_wheels_locked()
                    raise WheelCommandValidationError(
                        f"invalid wheel JointCommand: {exc}"
                    ) from exc
                raise JointCommandValidationError(
                    f"invalid JointCommand: {exc}"
                ) from exc

            try:
                current_mode = self.get_current_control_mode()
            except Exception as exc:
                if requested_wheels:
                    with self._locomotion_lock:
                        self._raise_unavailable_chassis_mode_locked(
                            "wheel JointCommand",
                            cause=exc,
                        )
                raise
            if requested_wheels and current_mode != "low_level":
                with self._locomotion_lock:
                    self._raise_unavailable_chassis_mode_locked(
                        "wheel JointCommand",
                        mode=current_mode,
                    )
            if current_mode not in _ACTION_SUPPORTED_MODES:
                if not self._non_action_mode_warned:
                    logger.warning(
                        "当前模式 %s 不执行 motor-level command。"
                        "可切换到 low_level 或 gravity_compensation_level。",
                        current_mode,
                    )
                    self._non_action_mode_warned = True
                return
            position_by_name = field_by_name(
                safe_command.name,
                safe_command.position,
            )
            velocity_by_name = field_by_name(
                safe_command.name,
                safe_command.velocity,
            )

            wheel_names = WHEEL_ACTUATORS & set(safe_command.name)
            if wheel_names:
                with self._locomotion_lock:
                    if self._chassis_stop_unconfirmed:
                        self._stop_wheels_locked()
                    if wheel_names != WHEEL_ACTUATORS:
                        self._stop_wheels_locked()
                        raise WheelCommandValidationError(
                            "wheel JointCommand must include wheel_left and wheel_right together"
                        )
                    if not WHEEL_ACTUATORS <= set(self._actuator_order):
                        self._stop_wheels_locked()
                        raise WheelCommandValidationError(
                            "wheel JointCommand requires both wheels in actuator_order"
                        )
                    if not WHEEL_ACTUATORS <= set(velocity_by_name):
                        self._stop_wheels_locked()
                        raise WheelCommandValidationError(
                            "wheel JointCommand requires velocity values for both wheels"
                        )

                    left = velocity_by_name["wheel_left"]
                    right = velocity_by_name["wheel_right"]
                    if not math.isfinite(left) or not math.isfinite(right):
                        self._stop_wheels_locked()
                        raise WheelCommandValidationError(
                            "wheel JointCommand velocities must be finite"
                        )
                    now = time.monotonic()
                    self._write_wheel_speeds_locked(left, right)
                    if left == 0.0 and right == 0.0:
                        self._record_wheels_stopped_locked()
                    else:
                        self._last_locomotion_command_at = now
                        self._locomotion_active = True
                        self._locomotion_timeout_latched = False
                        self._chassis_stop_unconfirmed = False

            failed: list[str] = []
            for name in safe_command.name:
                if name in WHEEL_ACTUATORS:
                    continue
                enum_value = self._enum_map.get(name)
                if enum_value is None or name not in position_by_name:
                    continue

                limits = ACTUATOR_LIMITS.get(name)
                if limits is None:
                    logger.warning(
                        "No actuator limit configured for %s; skipping command.",
                        name,
                    )
                    continue
                lo, hi = limits
                value = max(min(float(position_by_name[name]), hi), lo)

                ctrl = self._sdk.Motor_Control()
                if name == LIFT_ACTUATOR:
                    ctrl.Position = lift_m_to_sdk(value)
                elif name in GRIPPER_ACTUATORS:
                    ctrl.Position = gripper_m_to_sdk(value)
                else:
                    ctrl.Position = value

                if not bool(self._robot.setMotorControl_low(enum_value, ctrl)):
                    failed.append(name)

            if failed:
                logger.error("set actuator command failed for actuators: %s", failed)
                raise ActuatorWriteError(
                    "setMotorControl_low failed for actuators: " + ", ".join(failed)
                )
