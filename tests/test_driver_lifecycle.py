from __future__ import annotations

import threading
from types import MethodType, SimpleNamespace

import pytest

from zerith_h1_pro.config import ZerithH1ProNodeConfig
from zerith_h1_pro.driver import ZerithH1ProDriver
from zerith_h1_pro.subprocess_node import (
    _check_worker_exit,
    _discard_stale_camera_images,
    _graceful_worker_shutdown_timeout,
)


class _MotorControlMode:
    UNINITIALIZED = 0
    LOW_LEVEL = 1
    GRAVITY_COMPENSATION_LEVEL = 2


class _InitState:
    Uninit = 0
    Initializing = 1
    Init_Complete = 2
    Deinitializing = 3
    Deinit_Complete = 4
    Error_State = 5


class _FakeRobot:
    def __init__(self, *, mode: int, init_state: int) -> None:
        self.mode = mode
        self.init_state = init_state
        self.calls: list[str] = []

    def robot_connect(self) -> bool:
        self.calls.append("connect")
        return True

    def isRobotConnected(self) -> bool:
        return True

    def getCurrentMode(self) -> int:
        return self.mode

    def getInitState(self) -> int:
        return self.init_state

    def switchControlMode(self, mode: int) -> bool:
        self.calls.append("switch")
        self.mode = mode
        if self.init_state == _InitState.Error_State:
            self.init_state = _InitState.Uninit
        return True

    def robot_init(self) -> bool:
        self.calls.append("init")
        self.init_state = _InitState.Init_Complete
        return True

    def robot_deinit(self) -> bool:
        self.calls.append("deinit")
        self.init_state = _InitState.Deinit_Complete
        return True


class _StateSequenceRobot(_FakeRobot):
    def __init__(self, *, mode: int, init_states: list[int]) -> None:
        super().__init__(mode=mode, init_state=init_states[-1])
        self._init_states = list(init_states)

    def getInitState(self) -> int:
        if self._init_states:
            self.init_state = self._init_states.pop(0)
        return self.init_state


def _driver(robot: _FakeRobot) -> ZerithH1ProDriver:
    driver = object.__new__(ZerithH1ProDriver)
    driver._robot = robot
    driver._sdk = SimpleNamespace(
        MotorControlMode=_MotorControlMode,
        InitState=_InitState,
    )
    driver.control_mode = "low_level"
    driver._lifecycle_lock = threading.RLock()
    driver._control_mode_lock = threading.RLock()
    driver._locomotion_lock = threading.Lock()
    driver._last_locomotion_command_at = None
    driver._locomotion_active = False
    driver._locomotion_timeout_latched = False
    driver._chassis_stop_unconfirmed = False
    return driver


def test_reset_position_contract_rejects_legacy_name() -> None:
    config = ZerithH1ProNodeConfig.from_dict(
        {"positions": {"reset": {"lift": 0.25}}}
    )

    assert config.resolved_position("reset")["lift"] == 0.25
    with pytest.raises(ValueError, match="未知 positions 名称: rest"):
        ZerithH1ProNodeConfig.from_dict({"positions": {"rest": {"lift": 0.25}}})

    assert ZerithH1ProDriver._resolve_positions({"reset": {"lift": 0.25}})[
        "reset"
    ]["lift"] == 0.25
    with pytest.raises(ValueError, match="unknown lifecycle position: rest"):
        ZerithH1ProDriver._resolve_positions({"rest": {"lift": 0.25}})

    assert hasattr(ZerithH1ProDriver, "move_to_reset_position")
    assert not hasattr(ZerithH1ProDriver, "move_to_rest_position")


def test_same_mode_reconnect_does_not_switch_while_initialized() -> None:
    robot = _FakeRobot(
        mode=_MotorControlMode.LOW_LEVEL,
        init_state=_InitState.Init_Complete,
    )
    driver = _driver(robot)

    assert driver.switch_control_mode("low_level") == "low_level"
    assert robot.calls == []
    assert driver._ensure_initialized() == "Init_Complete"
    assert robot.calls == []


def test_reconnect_waits_for_interrupted_deinit_before_initializing() -> None:
    robot = _StateSequenceRobot(
        mode=_MotorControlMode.LOW_LEVEL,
        init_states=[_InitState.Deinitializing, _InitState.Deinit_Complete],
    )
    driver = _driver(robot)

    assert driver.switch_control_mode("low_level") == "low_level"
    assert driver._ensure_initialized() == "Init_Complete"
    assert robot.calls == ["init"]


def test_initialized_mode_change_deinitializes_switches_and_reinitializes() -> None:
    robot = _FakeRobot(
        mode=_MotorControlMode.GRAVITY_COMPENSATION_LEVEL,
        init_state=_InitState.Init_Complete,
    )
    driver = _driver(robot)

    assert driver.switch_control_mode("low_level") == "low_level"
    assert robot.calls == ["deinit", "switch", "init"]
    assert robot.init_state == _InitState.Init_Complete


def test_error_state_is_not_reported_as_deinitialized() -> None:
    robot = _FakeRobot(
        mode=_MotorControlMode.LOW_LEVEL,
        init_state=_InitState.Error_State,
    )
    driver = _driver(robot)

    with pytest.raises(RuntimeError, match="Error_State"):
        driver._ensure_deinitialized()


def test_error_state_cannot_initialize_without_mode_recovery() -> None:
    robot = _FakeRobot(
        mode=_MotorControlMode.LOW_LEVEL,
        init_state=_InitState.Error_State,
    )
    driver = _driver(robot)

    with pytest.raises(RuntimeError, match="Error_State"):
        driver._ensure_initialized()
    assert robot.calls == []


def test_same_mode_error_state_reissues_switch_before_init() -> None:
    robot = _FakeRobot(
        mode=_MotorControlMode.LOW_LEVEL,
        init_state=_InitState.Error_State,
    )
    driver = _driver(robot)

    assert driver.switch_control_mode("low_level") == "low_level"
    assert driver._ensure_initialized() == "Init_Complete"
    assert robot.calls == ["switch", "init"]


def test_mode_restore_initialization_failure_attempts_cleanup() -> None:
    robot = _FakeRobot(
        mode=_MotorControlMode.GRAVITY_COMPENSATION_LEVEL,
        init_state=_InitState.Init_Complete,
    )
    driver = _driver(robot)

    def fail_after_starting_init(_self: ZerithH1ProDriver) -> str:
        robot.init_state = _InitState.Init_Complete
        raise TimeoutError("restore initialization timed out")

    driver._ensure_initialized = MethodType(fail_after_starting_init, driver)

    with pytest.raises(TimeoutError, match="restore initialization timed out"):
        driver.switch_control_mode("low_level")

    assert robot.calls == ["deinit", "switch", "deinit"]
    assert robot.init_state == _InitState.Deinit_Complete


def test_connect_initialization_failure_attempts_cleanup() -> None:
    robot = _FakeRobot(
        mode=_MotorControlMode.LOW_LEVEL,
        init_state=_InitState.Initializing,
    )
    driver = object.__new__(ZerithH1ProDriver)
    driver._robot = None
    driver._sdk = SimpleNamespace(
        H1Robot=lambda *_args: robot,
        MotorControlMode=_MotorControlMode,
        InitState=_InitState,
    )
    driver.robot_ip = "10.10.0.94"
    driver.switch_mode_on_connect = False
    driver.control_mode = "low_level"
    driver.init_on_connect = True
    driver.is_follower = False
    driver._lifecycle_lock = threading.Lock()
    driver._control_mode_lock = threading.RLock()
    driver._locomotion_lock = threading.Lock()
    driver._disconnecting = False
    driver._locomotion_active = False
    driver._locomotion_timeout_latched = False
    driver._last_locomotion_command_at = None
    driver._chassis_stop_unconfirmed = False
    cleanup_calls: list[str] = []

    def fail_initialization(_self: ZerithH1ProDriver) -> str:
        raise TimeoutError("initialization timed out")

    def record_cleanup(_self: ZerithH1ProDriver) -> str:
        cleanup_calls.append("deinit")
        return "Deinit_Complete"

    driver._ensure_initialized = MethodType(fail_initialization, driver)
    driver._ensure_deinitialized = MethodType(record_cleanup, driver)

    with pytest.raises(TimeoutError, match="initialization timed out"):
        driver.connect()

    assert cleanup_calls == ["deinit"]
    assert driver._robot is None


def test_follower_connect_moves_to_initial_position() -> None:
    robot = _FakeRobot(
        mode=_MotorControlMode.LOW_LEVEL,
        init_state=_InitState.Deinit_Complete,
    )
    driver = object.__new__(ZerithH1ProDriver)
    driver._robot = None
    driver._sdk = SimpleNamespace(
        H1Robot=lambda *_args: robot,
        MotorControlMode=_MotorControlMode,
        InitState=_InitState,
    )
    driver.robot_ip = None
    driver.switch_mode_on_connect = False
    driver.control_mode = "low_level"
    driver.init_on_connect = False
    driver.is_follower = True
    driver.cameras = None
    driver._lifecycle_lock = threading.Lock()
    driver._control_mode_lock = threading.RLock()
    driver._locomotion_lock = threading.Lock()
    driver._disconnecting = False
    driver._locomotion_active = False
    driver._locomotion_timeout_latched = False
    driver._last_locomotion_command_at = None
    driver._chassis_stop_unconfirmed = False
    driver._joint_loop = lambda: None
    driver._locomotion_watchdog_loop = lambda: None
    initial_calls: list[str] = []
    driver.move_to_home_position = lambda: initial_calls.append("initial")

    driver.connect()
    for thread in driver._bg_threads:
        thread.join(timeout=1.0)

    assert initial_calls == ["initial"]


def test_follower_mode_mismatch_does_not_pollute_reconnect_target() -> None:
    robot = _FakeRobot(
        mode=_MotorControlMode.GRAVITY_COMPENSATION_LEVEL,
        init_state=_InitState.Deinit_Complete,
    )
    driver = object.__new__(ZerithH1ProDriver)
    driver._robot = None
    driver._sdk = SimpleNamespace(
        H1Robot=lambda *_args: robot,
        MotorControlMode=_MotorControlMode,
        InitState=_InitState,
    )
    driver.robot_ip = None
    driver.switch_mode_on_connect = False
    driver.control_mode = "low_level"
    driver.init_on_connect = False
    driver.is_follower = True
    driver._lifecycle_lock = threading.Lock()
    driver._control_mode_lock = threading.RLock()
    driver._locomotion_lock = threading.Lock()
    driver._disconnecting = False
    driver._locomotion_active = False
    driver._locomotion_timeout_latched = False
    driver._last_locomotion_command_at = None
    driver._chassis_stop_unconfirmed = False

    with pytest.raises(RuntimeError, match="actual mode low_level"):
        driver.connect()

    assert driver.control_mode == "low_level"
    assert driver._robot is None


def test_robot_worker_shutdown_budget_includes_lifecycle_and_deinit() -> None:
    config = ZerithH1ProNodeConfig(lifecycle_position_move_seconds=2.0)

    assert _graceful_worker_shutdown_timeout(config, "robot") == 17.0
    assert _graceful_worker_shutdown_timeout(config, "camera") == 3.0


def test_camera_worker_exit_is_reported_separately_from_robot() -> None:
    processes = {
        "robot": SimpleNamespace(exitcode=None),
        "camera": SimpleNamespace(exitcode=1),
    }
    reported: set[str] = set()

    assert _check_worker_exit(processes, reported) == {"camera"}
    assert _check_worker_exit(processes, reported) == set()


def test_camera_failure_discards_stale_images() -> None:
    images = {"image/head": object(), "image/left_wrist": object()}

    assert _discard_stale_camera_images(images) == 2
    assert images == {}


def test_camera_target_is_derived_from_robot_ip_and_fixed_port() -> None:
    remote = ZerithH1ProNodeConfig.from_dict({"robot_ip": "10.10.0.94"})
    local = ZerithH1ProNodeConfig.from_dict({"robot_ip": ""})

    assert remote.camera_grpc_target == "10.10.0.94:50051"
    assert local.camera_grpc_target == "127.0.0.1:50051"
