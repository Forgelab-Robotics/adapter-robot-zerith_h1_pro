"""Pure IPC payload codecs, validation, and bounded-queue helpers."""

from __future__ import annotations

import logging
import math
import queue
import time
from typing import Any

logger = logging.getLogger(__name__)


class CommandMessageValidationError(ValueError):
    """A command payload failed decoding before reaching the robot driver."""


def put_latest(target_queue: Any, item: Any) -> None:
    """Put a latest-value item into a bounded multiprocessing queue."""
    try:
        target_queue.put_nowait(item)
        return
    except queue.Full:
        pass

    try:
        target_queue.get_nowait()
    except queue.Empty:
        pass

    try:
        target_queue.put_nowait(item)
    except queue.Full:
        logger.debug("丢弃过期 IPC 消息: %s", item[0] if item else "unknown")


def joint_state_payload(state: Any) -> dict[str, list[Any]]:
    return {
        "name": list(getattr(state, "name", []) or []),
        "position": list(getattr(state, "position", []) or []),
        "velocity": list(getattr(state, "velocity", []) or []),
        "effort": list(getattr(state, "effort", []) or []),
    }


def joint_command_payload(command: Any) -> dict[str, Any]:
    return {
        "name": list(getattr(command, "name", []) or []),
        "position": list(getattr(command, "position", []) or []),
        "velocity": list(getattr(command, "velocity", []) or []),
        "effort": list(getattr(command, "effort", []) or []),
        "kp": list(getattr(command, "kp", []) or []),
        "kd": list(getattr(command, "kd", []) or []),
    }


def validate_finite_joint_fields(message: Any, fields: tuple[str, ...]) -> None:
    for field_name in fields:
        values = list(getattr(message, field_name, []) or [])
        if any(not math.isfinite(float(value)) for value in values):
            raise CommandMessageValidationError(
                f"{field_name} must contain only finite values"
            )


def command_payload_without_wheels(payload: dict[str, Any]) -> dict[str, Any]:
    names = list(payload.get("name", []) or [])
    keep = [
        index
        for index, name in enumerate(names)
        if name not in {"wheel_left", "wheel_right"}
    ]
    result = dict(payload)
    result["name"] = [names[index] for index in keep]
    for field in ("position", "velocity", "effort", "kp", "kd"):
        values = list(payload.get(field, []) or [])
        result[field] = (
            [values[index] for index in keep]
            if len(values) == len(names)
            else values
        )
    return result


def command_payload_to_joint_command(payload: dict[str, Any]) -> Any:
    from forge_msgs import JointCommand

    try:
        command = JointCommand(
            name=payload.get("name", []),
            position=payload.get("position", []),
            velocity=payload.get("velocity", []),
            effort=payload.get("effort", []),
            kp=payload.get("kp", []),
            kd=payload.get("kd", []),
        )
        validate_finite_joint_fields(
            command,
            ("position", "velocity", "effort", "kp", "kd"),
        )
        return command
    except Exception as exc:
        raise CommandMessageValidationError(
            f"invalid JointCommand payload: {exc}"
        ) from exc


def joint_command_arrow_to_payload(value: Any) -> dict[str, Any]:
    from forge_msgs import JointCommand

    try:
        command = JointCommand.from_arrow(value)
        validate_finite_joint_fields(
            command,
            ("position", "velocity", "effort", "kp", "kd"),
        )
        return joint_command_payload(command)
    except Exception as exc:
        raise CommandMessageValidationError(
            f"invalid JointCommand Arrow payload: {exc}"
        ) from exc


def locomotion_command_payload(
    command: Any,
    *,
    received_at: float | None = None,
) -> dict[str, float]:
    payload = {
        "vx": float(command.vx),
        "vy": float(command.vy),
        "wz": float(command.wz),
    }
    if received_at is not None:
        payload["received_at"] = float(received_at)
    return payload


def locomotion_payload_to_command(payload: dict[str, Any]) -> Any:
    from forge_msgs import LocomotionCommand

    try:
        return LocomotionCommand(
            vx=float(payload["vx"]),
            vy=float(payload["vy"]),
            wz=float(payload["wz"]),
        )
    except Exception as exc:
        raise CommandMessageValidationError(
            f"invalid LocomotionCommand payload: {exc}"
        ) from exc


def locomotion_arrow_to_payload(value: Any) -> dict[str, float]:
    from forge_msgs import LocomotionCommand

    try:
        command = LocomotionCommand.from_arrow(value)
        return locomotion_command_payload(command, received_at=time.monotonic())
    except Exception as exc:
        raise CommandMessageValidationError(
            f"invalid LocomotionCommand Arrow payload: {exc}"
        ) from exc


def state_payload_to_arrow(payload: dict[str, list[Any]]) -> Any:
    from forge_msgs import JointState

    return JointState(
        name=payload.get("name", []),
        position=payload.get("position", []),
        velocity=payload.get("velocity", []),
        effort=payload.get("effort", []),
    ).to_arrow()


def master_state_arrow_to_command_payload(value: Any) -> dict[str, list[Any]]:
    from forge_msgs import JointState

    try:
        state = JointState.from_arrow(value)
        validate_finite_joint_fields(state, ("position", "velocity", "effort"))
        return master_state_to_command_payload(state)
    except Exception as exc:
        raise CommandMessageValidationError(
            f"invalid master JointState Arrow payload: {exc}"
        ) from exc


def master_state_to_command_payload(state: Any) -> dict[str, list[Any]]:
    from forge_msgs import JointCommand

    names = list(state.name)
    velocity = list(state.velocity)
    wheel_indices = {
        name: names.index(name)
        for name in ("wheel_left", "wheel_right")
        if name in names
    }
    if wheel_indices:
        wheel_velocity_available = (
            set(wheel_indices) == {"wheel_left", "wheel_right"}
            and len(velocity) == len(names)
            and all(
                math.isfinite(velocity[index])
                for index in wheel_indices.values()
            )
        )
        if not wheel_velocity_available:
            velocity = [0.0 for _ in names]

    command = JointCommand(
        name=names,
        position=state.position,
        velocity=velocity,
        effort=[],
        kp=[],
        kd=[],
    )
    payload = joint_command_payload(command)
    if wheel_indices and set(wheel_indices) != {"wheel_left", "wheel_right"}:
        logger.warning(
            "master_joint_state only contains one wheel; rejecting wheel fields "
            "while preserving non-wheel joints."
        )
        return command_payload_without_wheels(payload)
    return payload


def enqueue_parent_timestamp(
    command_queue: Any,
    kind: str,
    payload: dict[str, Any],
) -> None:
    payload = dict(payload)
    payload["received_at"] = time.monotonic()
    put_latest(command_queue, (kind, payload))


def replace_latest_locomotion(
    locomotion_queue: Any,
    payload: dict[str, float],
) -> None:
    """Replace the pending chassis command; never silently discard the newer stop."""
    item = ("locomotion_command", payload)
    deadline = time.monotonic() + 0.1
    while True:
        try:
            locomotion_queue.put_nowait(item)
            return
        except queue.Full:
            try:
                locomotion_queue.get_nowait()
            except queue.Empty:
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        "无法更新 latest-only locomotion IPC queue；触发节点安全关闭。"
                    )
                time.sleep(0.001)
