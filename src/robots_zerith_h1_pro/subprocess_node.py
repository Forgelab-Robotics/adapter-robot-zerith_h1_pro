"""Parent Dora orchestration for isolated Zerith robot and camera workers."""

from __future__ import annotations

import multiprocessing as mp
import queue
import time
from typing import Any

from forge_common import get_logger

from .config import ZerithH1ProNodeConfig, load_config
from .ipc import (
    CommandMessageValidationError,
    enqueue_parent_timestamp,
    joint_command_arrow_to_payload,
    locomotion_arrow_to_payload,
    master_state_arrow_to_command_payload,
    put_latest,
    replace_latest_locomotion,
    state_payload_to_arrow,
)
from .workers import camera_worker_main, robot_worker_main

logger = get_logger(__name__)

# Includes background-thread joins, async robot_deinit and native SDK cleanup.
_INIT_SHUTDOWN_MARGIN_SECONDS = 15.0


def _graceful_worker_shutdown_timeout(
    config: ZerithH1ProNodeConfig,
    worker_name: str,
) -> float:
    if worker_name == "robot":
        return config.lifecycle_position_move_seconds + _INIT_SHUTDOWN_MARGIN_SECONDS
    return 3.0


def _drain_output_queue(
    output_queue: Any,
    *,
    latest_images: dict[str, Any],
) -> tuple[Any | None, bool]:
    latest_state_payload = None
    saw_error = False

    while True:
        try:
            message = output_queue.get_nowait()
        except queue.Empty:
            break

        kind = message[0]
        if kind == "joint_state":
            latest_state_payload = message[1]
        elif kind == "image":
            _, output_id, payload = message
            latest_images[output_id] = payload
        elif kind == "error":
            _, worker_name, error_text = message
            logger.error("%s worker 异常退出:\n%s", worker_name, error_text)
            saw_error = True
    return latest_state_payload, saw_error


def _check_worker_exit(processes: dict[str, Any], reported: set[str]) -> set[str]:
    exited: set[str] = set()
    for name, process in processes.items():
        if process.exitcode is None or name in reported:
            continue
        reported.add(name)
        exited.add(name)
        log = logger.error if name == "robot" else logger.warning
        log("%s worker 已退出，exitcode=%s", name, process.exitcode)
    return exited


def _discard_stale_camera_images(latest_images: dict[str, Any]) -> int:
    count = len(latest_images)
    latest_images.clear()
    return count


def _reject_parent_control_message(
    input_id: str,
    error: CommandMessageValidationError,
    locomotion_queue: Any,
) -> bool:
    """Reject one malformed control message and queue a guarded chassis stop."""
    logger.warning("拒绝格式错误的 %s: %s", input_id, error)
    try:
        replace_latest_locomotion(
            locomotion_queue,
            {
                "vx": 0.0,
                "vy": 0.0,
                "wz": 0.0,
                "received_at": time.monotonic(),
            },
        )
    except Exception:
        logger.exception("拒绝 %s 后无法排队底盘零命令。", input_id)
        return False
    return True


def coordinate_parent_shutdown(
    *,
    is_follower: bool,
    locomotion_queue: Any,
    locomotion_gate: Any,
    stop_event: Any,
    gate_timeout: float = 0.5,
) -> bool:
    """Queue a follower stop, close the locomotion gate, and signal shutdown."""
    success = True
    if is_follower:
        try:
            replace_latest_locomotion(
                locomotion_queue,
                {
                    "vx": 0.0,
                    "vy": 0.0,
                    "wz": 0.0,
                    "received_at": time.monotonic(),
                },
            )
        except Exception:
            success = False
            logger.exception("关闭节点时无法排队底盘零命令。")

    gate_acquired = locomotion_gate.acquire(timeout=max(float(gate_timeout), 0.0))
    if not gate_acquired:
        success = False
        logger.critical(
            "robot worker 未释放 locomotion gate；继续执行有界 join/terminate。"
        )

    try:
        stop_event.set()
    except Exception:
        success = False
        logger.exception("无法设置 worker stop event；继续执行有界 join/terminate。")
    finally:
        if gate_acquired:
            try:
                locomotion_gate.release()
            except Exception:
                success = False
                logger.exception("无法释放 locomotion gate。")

    return success


def run_subprocess_dora_node(config_path: str | None) -> int:
    """
    Run one Dora node while keeping Zerith robot and camera SDKs in separate processes.

    The parent process owns Dora I/O and never instantiates either native SDK. Worker
    processes communicate latest joint/image payloads through bounded queues.
    """
    from dora import Node

    config: ZerithH1ProNodeConfig = load_config(config_path=config_path)

    ctx = mp.get_context("spawn")
    stop_event = ctx.Event()
    locomotion_gate = ctx.Lock()
    command_queue = ctx.Queue(maxsize=8)
    # Locomotion has an independent latest-only channel so arm traffic cannot
    # displace a newer chassis stop command.
    locomotion_queue = ctx.Queue(maxsize=1)
    robot_output_queue = ctx.Queue(maxsize=4)
    camera_output_queue = ctx.Queue(maxsize=max(8, len(config.cameras or []) * 4))

    processes: dict[str, Any] = {
        "robot": ctx.Process(
            target=robot_worker_main,
            args=(
                config_path,
                command_queue,
                locomotion_queue,
                robot_output_queue,
                stop_event,
                locomotion_gate,
            ),
            name="ZerithH1ProRobotWorker",
        )
    }
    if config.cameras:
        processes["camera"] = ctx.Process(
            target=camera_worker_main,
            args=(config_path, camera_output_queue, stop_event),
            name="ZerithH1ProCameraWorker",
        )

    latest_joint_state_arrow = None
    latest_images: dict[str, Any] = {}
    reported_exits: set[str] = set()
    camera_available = "camera" in processes
    exit_code = 0

    try:
        for process in processes.values():
            process.start()

        node = Node()
        for event in node:
            state_payload, saw_robot_error = _drain_output_queue(
                robot_output_queue,
                latest_images=latest_images,
            )
            if state_payload is not None:
                latest_joint_state_arrow = state_payload_to_arrow(state_payload)

            _, saw_camera_error = _drain_output_queue(
                camera_output_queue,
                latest_images=latest_images,
            )
            exited_workers = _check_worker_exit(processes, reported_exits)
            if camera_available and (saw_camera_error or "camera" in exited_workers):
                discarded = _discard_stale_camera_images(latest_images)
                camera_available = False
                logger.warning(
                    "camera worker 不可用；停止图像输出并丢弃 %d 帧缓存，"
                    "机器人控制和关节反馈继续运行。",
                    discarded,
                )
            if saw_robot_error or "robot" in exited_workers:
                exit_code = 1
                break

            match event["type"]:
                case "INPUT":
                    input_id = event["id"]
                    value = event.get("value")
                    if input_id == "done":
                        logger.info("收到 done，退出 H1 聚合节点。")
                        break
                    if input_id == "tick":
                        if latest_joint_state_arrow is not None:
                            node.send_output("joint_state", latest_joint_state_arrow)
                        for output_id, payload in latest_images.items():
                            node.send_output(output_id, payload)
                        continue
                    if input_id == "command":
                        if config.is_follower:
                            try:
                                payload = joint_command_arrow_to_payload(value)
                            except CommandMessageValidationError as exc:
                                if not _reject_parent_control_message(
                                    input_id,
                                    exc,
                                    locomotion_queue,
                                ):
                                    exit_code = 1
                                    break
                                continue
                            enqueue_parent_timestamp(command_queue, "command", payload)
                        continue
                    if input_id == "master_joint_state":
                        if config.is_follower:
                            try:
                                payload = master_state_arrow_to_command_payload(value)
                            except CommandMessageValidationError as exc:
                                if not _reject_parent_control_message(
                                    input_id,
                                    exc,
                                    locomotion_queue,
                                ):
                                    exit_code = 1
                                    break
                                continue
                            enqueue_parent_timestamp(command_queue, "command", payload)
                        continue
                    if input_id == "locomotion_command":
                        if config.is_follower:
                            try:
                                payload = locomotion_arrow_to_payload(value)
                            except CommandMessageValidationError as exc:
                                if not _reject_parent_control_message(
                                    input_id,
                                    exc,
                                    locomotion_queue,
                                ):
                                    exit_code = 1
                                    break
                                continue
                            replace_latest_locomotion(locomotion_queue, payload)
                        continue
                case "STOP":
                    break
                case "ERROR":
                    logger.error("节点收到 ERROR: %s", event.get("error", "unknown"))
                    exit_code = 1
                    break
                case _:
                    pass
    finally:
        if not coordinate_parent_shutdown(
            is_follower=config.is_follower,
            locomotion_queue=locomotion_queue,
            locomotion_gate=locomotion_gate,
            stop_event=stop_event,
        ):
            exit_code = 1
        put_latest(command_queue, ("stop", {}))
        for worker_name, process in processes.items():
            # Robot shutdown includes reset interpolation, background thread joins and
            # asynchronous robot_deinit. A fixed 3 s timeout can kill it mid-deinit and
            # leave the controller in Init_Complete/Deinitializing for the next process.
            graceful_timeout = _graceful_worker_shutdown_timeout(
                config,
                worker_name,
            )
            process.join(timeout=graceful_timeout)
            if process.is_alive():
                logger.warning(
                    "%s 未在 %.1f 秒内退出，强制终止；下次连接将恢复 init state。",
                    process.name,
                    graceful_timeout,
                )
                process.terminate()
                process.join(timeout=1.0)
            if worker_name == "robot" and process.exitcode not in (None, 0):
                exit_code = 1

    return exit_code
