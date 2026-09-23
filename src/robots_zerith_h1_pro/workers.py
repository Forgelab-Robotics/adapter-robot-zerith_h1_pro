"""Spawn-safe Zerith robot and camera worker entry points."""

from __future__ import annotations

import logging
import math
import queue
import time
import traceback
from typing import Any

from .config import load_config
from .ipc import (
    CommandMessageValidationError,
    command_payload_to_joint_command,
    command_payload_without_wheels,
    joint_state_payload,
    locomotion_payload_to_command,
    put_latest,
)

logger = logging.getLogger(__name__)


def arbitrate_joint_payload(
    payload: dict[str, Any],
    *,
    now: float,
    locomotion_owner_until: float,
    command_timeout: float,
) -> dict[str, Any]:
    try:
        names = set(payload.get("name", []) or [])
        if not names & {"wheel_left", "wheel_right"}:
            return payload
        received_at = float(payload.get("received_at", now))
        direct_wheel_stale = (
            not math.isfinite(received_at)
            or received_at > now
            or now - received_at > command_timeout
        )
        if now <= locomotion_owner_until or direct_wheel_stale:
            return command_payload_without_wheels(payload)
        return payload
    except Exception as exc:
        raise CommandMessageValidationError(
            f"invalid joint command arbitration payload: {exc}"
        ) from exc


def dispatch_joint_payload(
    driver: Any,
    payload: dict[str, Any],
    *,
    validation_error_type: type[Exception],
) -> bool:
    """Reject malformed joint input without swallowing SDK/control failures."""
    try:
        command = command_payload_to_joint_command(payload)
    except CommandMessageValidationError as exc:
        driver.stop_locomotion()
        logger.warning("拒绝格式错误的 joint command: %s", exc)
        return False

    try:
        driver.set_command(command)
    except validation_error_type as exc:
        logger.warning("拒绝不安全的 joint command: %s", exc)
        return False
    return True


def dispatch_locomotion_payload(
    driver: Any,
    payload: dict[str, Any],
    *,
    validation_error_type: type[Exception],
) -> float | None:
    """Apply one locomotion message; malformed input is stopped and rejected."""
    try:
        try:
            received_at = float(payload.get("received_at", time.monotonic()))
        except Exception as exc:
            raise CommandMessageValidationError(
                f"invalid locomotion received_at: {exc}"
            ) from exc
        command = locomotion_payload_to_command(payload)
    except CommandMessageValidationError as exc:
        driver.stop_locomotion()
        logger.warning("拒绝格式错误的 locomotion command: %s", exc)
        return None

    try:
        driver.set_locomotion_command(command, received_at=received_at)
    except validation_error_type as exc:
        logger.warning("拒绝不安全的 locomotion command: %s", exc)
        return None
    return received_at


def robot_worker_main(
    config_path: str | None,
    command_queue: Any,
    locomotion_queue: Any,
    output_queue: Any,
    stop_event: Any,
    locomotion_gate: Any,
) -> None:
    """Robot SDK worker. This process must not load the camera SDK."""
    driver = None
    try:
        from .driver import (
            JointCommandValidationError,
            LocomotionCommandValidationError,
            ZerithH1ProDriver,
        )

        config = load_config(config_path=config_path)
        driver = ZerithH1ProDriver(
            sdk_root=config.sdk_root,
            robot_ip=config.robot_ip,
            is_follower=config.is_follower,
            init_on_connect=config.init_on_connect,
            control_mode=config.control_mode,
            switch_mode_on_connect=config.switch_mode_on_connect,
            actuator_order=config.actuator_order,
            positions={
                "initial": config.resolved_position("initial"),
                "reset": config.resolved_position("reset"),
            },
            lifecycle_position_move_seconds=config.lifecycle_position_move_seconds,
            wheel_radius_m=config.wheel_radius_m,
            wheel_track_width_m=config.wheel_track_width_m,
            locomotion_max_vx_mps=config.locomotion_max_vx_mps,
            locomotion_max_wz_radps=config.locomotion_max_wz_radps,
            locomotion_max_wheel_speed_radps=config.locomotion_max_wheel_speed_radps,
            locomotion_command_timeout_seconds=(
                config.locomotion_command_timeout_seconds
            ),
            auto_connect=False,
            camera_grpc_target=None,
            cameras=None,
            image_fps=config.image_fps,
            joint_fps=config.joint_fps,
        )
        driver.connect()

        period = 1.0 / max(int(config.joint_fps), 1)
        locomotion_owner_until = -math.inf
        while True:
            if stop_event.is_set():
                break

            if config.is_follower:
                try:
                    kind, payload = locomotion_queue.get_nowait()
                except queue.Empty:
                    pass
                else:
                    if kind == "locomotion_command":
                        with locomotion_gate:
                            if stop_event.is_set():
                                break
                            received_at = dispatch_locomotion_payload(
                                driver,
                                payload,
                                validation_error_type=LocomotionCommandValidationError,
                            )
                            if received_at is not None:
                                locomotion_owner_until = max(
                                    locomotion_owner_until,
                                    received_at
                                    + config.locomotion_command_timeout_seconds,
                                )

            try:
                kind, payload = command_queue.get_nowait()
            except queue.Empty:
                pass
            else:
                if kind == "stop":
                    stop_event.set()
                    driver.stop_locomotion()
                    break
                if config.is_follower and kind == "command":
                    try:
                        payload = arbitrate_joint_payload(
                            payload,
                            now=time.monotonic(),
                            locomotion_owner_until=locomotion_owner_until,
                            command_timeout=config.locomotion_command_timeout_seconds,
                        )
                    except CommandMessageValidationError as exc:
                        driver.stop_locomotion()
                        logger.warning(
                            "拒绝格式错误的 joint arbitration payload: %s",
                            exc,
                        )
                        continue
                    if payload.get("name"):
                        dispatch_joint_payload(
                            driver,
                            payload,
                            validation_error_type=JointCommandValidationError,
                        )

            if stop_event.is_set():
                break
            state = driver.get_state()
            put_latest(output_queue, ("joint_state", joint_state_payload(state)))
            stop_event.wait(period)
    except BaseException:
        put_latest(output_queue, ("error", "robot", traceback.format_exc()))
        raise
    finally:
        if driver is not None:
            driver.disconnect()


def camera_worker_main(
    config_path: str | None,
    output_queue: Any,
    stop_event: Any,
) -> None:
    """Camera SDK worker. This process must not load the robot SDK."""
    client = None
    try:
        from . import camera_io

        config = load_config(config_path=config_path)
        cameras = list(config.cameras or [])
        if not cameras:
            logger.info("未配置 cameras，相机 worker 直接退出。")
            return

        cv2 = camera_io.load_cv2()
        client = camera_io.connect_client(
            config.sdk_root,
            config.camera_grpc_target or "",
        )
        period = 1.0 / max(int(config.image_fps), 1)
        while not stop_event.is_set():
            for camera in cameras:
                if camera.enable_color:
                    data = client.get_latest_frame(camera.name)
                    if data:
                        frame, timestamp = data
                        payload = camera_io.build_color_payload(
                            frame,
                            timestamp,
                            camera,
                            cv2,
                        )
                        put_latest(output_queue, ("image", camera.output_id, payload))

                if camera.enable_depth and camera.depth_output_id:
                    data = client.get_latest_depth(camera.name)
                    if data:
                        depth, timestamp = data
                        payload = camera_io.build_depth_payload(
                            depth,
                            timestamp,
                            camera,
                            cv2,
                        )
                        put_latest(
                            output_queue,
                            ("image", camera.depth_output_id, payload),
                        )
            time.sleep(period)
    except BaseException:
        put_latest(output_queue, ("error", "camera", traceback.format_exc()))
        raise
    finally:
        client = None
