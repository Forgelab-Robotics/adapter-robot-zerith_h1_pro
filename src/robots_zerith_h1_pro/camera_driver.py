"""Camera-only cached driver for Zerith H1 Pro."""

from __future__ import annotations

import logging
import threading
import time
from typing import Any

from . import camera_io
from .config import ZerithCameraStreamConfig

logger = logging.getLogger(__name__)


class ZerithH1ProCameraDriver:
    """Zerith camera cache driver.

    This mirrors the main robot driver's camera cache behavior while avoiding
    loading the H1 robot SDK in the camera process.
    """

    def __init__(
        self,
        *,
        sdk_root: str,
        camera_grpc_target: str,
        cameras: list[ZerithCameraStreamConfig],
        image_fps: int = 30,
        auto_connect: bool = True,
    ) -> None:
        if not cameras:
            raise ValueError("cameras 不能为空")
        self.sdk_root = sdk_root
        self.camera_grpc_target = camera_grpc_target
        self.cameras = cameras
        self.image_fps = image_fps
        self._camera_client = None
        self._camera_thread: threading.Thread | None = None
        self._stop_thread = False
        self._camera_lock = threading.Lock()
        self._latest_camera_frames: dict[str, Any] = {}

        if auto_connect:
            self.connect()

    def connect(self) -> None:
        if self._camera_client is not None:
            logger.warning("Camera driver is already connected.")
            return

        self._camera_client = camera_io.connect_client(
            self.sdk_root,
            self.camera_grpc_target,
        )
        self._stop_thread = False
        self._camera_thread = threading.Thread(
            target=self._camera_loop,
            name="H1ProCameraOnlyLoop",
            daemon=True,
        )
        self._camera_thread.start()

    def disconnect(self) -> None:
        self._stop_thread = True
        if self._camera_thread is not None:
            self._camera_thread.join(timeout=1.0)
            self._camera_thread = None
        if self._camera_client is not None:
            stop = getattr(self._camera_client, "stop", None)
            if callable(stop):
                try:
                    stop()
                except Exception:
                    logger.exception("停止 Zerith camera client 失败。")
            self._camera_client = None

    def _camera_loop(self) -> None:
        logger.info("Zerith H1 Pro camera cache thread started.")
        cv2 = camera_io.load_cv2()
        while not self._stop_thread:
            try:
                if self._camera_client is not None:
                    for camera in self.cameras:
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
            except Exception as exc:
                logger.error("H1 Pro camera cache thread error: %s", exc)
            time.sleep(1.0 / self.image_fps)

    def get_latest_encoded_image(self, output_id: str) -> Any | None:
        with self._camera_lock:
            return self._latest_camera_frames.get(output_id)
