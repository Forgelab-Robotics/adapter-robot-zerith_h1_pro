"""Lazy Zerith camera SDK I/O and stateless image payload helpers."""

from __future__ import annotations

import importlib
import logging
import sys
from pathlib import Path
from types import ModuleType
from typing import Any

import numpy as np
from forge_msgs import CompressedImage, Image

from .config import ZerithCameraStreamConfig

logger = logging.getLogger(__name__)


def resolve_camera_sdk_root(sdk_root: str) -> Path:
    root = Path(sdk_root).expanduser().resolve()
    candidates = [root, root / "camera_sdk_python"]
    for candidate in candidates:
        if candidate.name != "camera_sdk_python":
            continue
        if candidate.exists():
            return candidate
    expected = root / "camera_sdk_python"
    raise FileNotFoundError(f"未找到 camera_sdk_python 目录: {expected}")


def load_camera_sdk(sdk_root: str) -> ModuleType:
    camera_root = resolve_camera_sdk_root(sdk_root)
    for path in (camera_root, camera_root / "proto"):
        path_str = str(path)
        if path_str not in sys.path:
            sys.path.insert(0, path_str)

    try:
        return importlib.import_module("camera_client")
    except Exception as exc:
        raise RuntimeError(
            "导入 Zerith Camera SDK 失败。请确认 camera_sdk_python 目录存在、"
            "Python 版本与 camera_client 扩展匹配，并已安装 requirements.txt 中的依赖。"
        ) from exc


def load_cv2() -> Any:
    try:
        return importlib.import_module("cv2")
    except Exception as exc:
        raise RuntimeError(
            "Zerith camera 需要 opencv-python 以编码 JPEG/PNG、缩放图像或处理 depth。"
        ) from exc


def raw_image_to_arrow(frame: np.ndarray, encoding: str) -> Any:
    return Image.from_numpy(np.ascontiguousarray(frame), encoding=encoding).to_arrow()


def compressed_image_to_arrow(image_format: str, data: bytes) -> Any:
    return CompressedImage(format=image_format, data=data).to_arrow()


def resize_if_needed(
    frame: np.ndarray,
    cfg: ZerithCameraStreamConfig,
    cv2: Any,
) -> np.ndarray:
    if cfg.width is None or cfg.height is None:
        return frame
    height, width = int(frame.shape[0]), int(frame.shape[1])
    if width == cfg.width and height == cfg.height:
        return frame
    return cv2.resize(frame, (cfg.width, cfg.height), interpolation=cv2.INTER_AREA)


def build_color_payload(
    frame: np.ndarray,
    timestamp: float | None,
    cfg: ZerithCameraStreamConfig,
    cv2: Any,
) -> Any:
    frame = np.asarray(frame)
    frame = resize_if_needed(frame, cfg, cv2)

    if frame.ndim != 3 or int(frame.shape[2]) != 3:
        raise ValueError(f"camera {cfg.name}: 彩色帧期望 HxWx3，实际 shape={frame.shape}")

    if cfg.image_format == "raw":
        encoding = "bgr8" if cfg.color_order == "bgr" else "rgb8"
        return raw_image_to_arrow(frame, encoding)

    bgr = frame if cfg.color_order == "bgr" else frame[..., ::-1]
    if cfg.image_format == "jpeg":
        ok, encoded = cv2.imencode(
            ".jpg",
            bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(cfg.image_jpeg_quality)],
        )
        image_format = "jpeg"
    else:
        ok, encoded = cv2.imencode(".png", bgr)
        image_format = "png"
    if not ok:
        raise RuntimeError(f"camera {cfg.name}: {cfg.image_format} 编码失败")

    return compressed_image_to_arrow(image_format, encoded.tobytes())


def build_depth_payload(
    depth: np.ndarray,
    timestamp: float | None,
    cfg: ZerithCameraStreamConfig,
    cv2: Any,
) -> Any:
    depth = np.asarray(depth)
    if depth.ndim == 3 and depth.shape[-1] == 1:
        depth = depth[:, :, 0]
    if depth.ndim != 2:
        raise ValueError(f"camera {cfg.name}: depth 期望 HxW，实际 shape={depth.shape}")

    depth = resize_if_needed(depth, cfg, cv2)

    if cfg.depth_format == "raw_u16":
        depth_u16 = np.ascontiguousarray(depth.astype(np.uint16, copy=False))
        return raw_image_to_arrow(depth_u16, "16UC1")

    depth_u8 = (depth.astype(np.float32) * cfg.depth_visualization_alpha).clip(0, 255)
    depth_color = cv2.applyColorMap(depth_u8.astype(np.uint8), cv2.COLORMAP_JET)
    ok, encoded = cv2.imencode(
        ".jpg",
        depth_color,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(cfg.depth_jpeg_quality)],
    )
    if not ok:
        raise RuntimeError(f"camera {cfg.name}: depth JPEG 编码失败")

    return compressed_image_to_arrow("jpeg", encoded.tobytes())


def connect_client(sdk_root: str, camera_grpc_target: str) -> Any:
    sdk = load_camera_sdk(sdk_root)
    client = sdk.CameraClient(grpc_target=camera_grpc_target)
    logger.info("Connecting Zerith camera service: %s", camera_grpc_target)
    client.start()
    logger.info("Connected Zerith camera service: %s", camera_grpc_target)
    return client


def poll_camera_once(
    client: Any,
    cameras: list[ZerithCameraStreamConfig],
    cv2: Any,
) -> dict[str, Any]:
    """Poll each configured stream once and return newly encoded output payloads."""
    payloads: dict[str, Any] = {}
    for camera in cameras:
        if camera.enable_color:
            data = client.get_latest_frame(camera.name)
            if data:
                frame, timestamp = data
                payloads[camera.output_id] = build_color_payload(
                    frame,
                    timestamp,
                    camera,
                    cv2,
                )

        if camera.enable_depth and camera.depth_output_id:
            data = client.get_latest_depth(camera.name)
            if data:
                depth, timestamp = data
                payloads[camera.depth_output_id] = build_depth_payload(
                    depth,
                    timestamp,
                    camera,
                    cv2,
                )
    return payloads
