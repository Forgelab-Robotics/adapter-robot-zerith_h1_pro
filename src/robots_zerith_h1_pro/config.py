"""Zerith H1 Pro 节点配置加载与校验。"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import yaml
from forge_robot import ActuatorSpec, joint_order

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
BUNDLED_SDK_REL = Path("third_party") / "H1_SDK_1.3.7"
CAMERA_GRPC_PORT = 50051
LOCAL_CAMERA_HOST = "127.0.0.1"

# Conservative low-level locomotion defaults. Geometry is derived from the
# delivered H1 Pro model. The SDK does not document wheel Speed units/limits;
# 1.0 is a project safety ceiling inferred from its low-level example.
ZERITH_H1_PRO_WHEEL_RADIUS_M = 0.0835
ZERITH_H1_PRO_WHEEL_TRACK_WIDTH_M = 0.379
ZERITH_H1_PRO_MAX_VX_MPS = 0.08
ZERITH_H1_PRO_MAX_WZ_RADPS = 0.4
ZERITH_H1_PRO_MAX_WHEEL_SPEED_RADPS = 1.0
ZERITH_H1_PRO_LOCOMOTION_COMMAND_TIMEOUT_SECONDS = 0.25


def resolve_default_sdk_root() -> str:
    """Return bundled SDK path (PyInstaller _MEIPASS) or repo third_party."""
    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        bundled = Path(meipass) / BUNDLED_SDK_REL
        if bundled.exists():
            return str(bundled.resolve())
    return str((PACKAGE_ROOT / BUNDLED_SDK_REL).resolve())


def resolve_sdk_root(
    raw: str | None,
    *,
    config_file_parent: Path | None = None,
) -> str:
    """Resolve sdk_root from YAML; omit or leave empty to use bundled/default SDK."""
    if raw is None or not str(raw).strip():
        return resolve_default_sdk_root()

    root = Path(str(raw)).expanduser()
    if not root.is_absolute():
        base = config_file_parent if config_file_parent is not None else Path.cwd()
        root = (base / root).resolve()
    else:
        root = root.resolve()
    return str(root)


DEFAULT_SDK_ROOT = resolve_default_sdk_root()

ZERITH_H1_PRO_ACTUATOR_SPECS = (
    ActuatorSpec(
        name="wheel_left",
        kind="continuous",
        mode="velocity",
        max_velocity=ZERITH_H1_PRO_MAX_WHEEL_SPEED_RADPS,
    ),
    ActuatorSpec(
        name="wheel_right",
        kind="continuous",
        mode="velocity",
        max_velocity=ZERITH_H1_PRO_MAX_WHEEL_SPEED_RADPS,
    ),
    ActuatorSpec(name="lift", kind="prismatic", min_position=0.0, max_position=0.8),
    ActuatorSpec(name="waist_down", kind="revolute", min_position=0.0, max_position=1.3),
    ActuatorSpec(name="waist_up", kind="revolute", min_position=-0.7, max_position=0.7),
    ActuatorSpec(name="head_down", kind="revolute", min_position=-1.5, max_position=1.5),
    ActuatorSpec(name="head_up", kind="revolute", min_position=-0.17, max_position=0.25),
    ActuatorSpec(name="left_arm_1", kind="revolute", min_position=-2.7, max_position=1.5),
    ActuatorSpec(name="left_arm_2", kind="revolute", min_position=-0.3, max_position=2.0),
    ActuatorSpec(name="left_arm_3", kind="revolute", min_position=-2.9, max_position=2.9),
    ActuatorSpec(name="left_arm_4", kind="revolute", min_position=-1.3, max_position=1.5),
    ActuatorSpec(name="left_arm_5", kind="revolute", min_position=-2.9, max_position=2.9),
    ActuatorSpec(name="left_arm_6", kind="revolute", min_position=-1.0, max_position=1.0),
    ActuatorSpec(name="left_arm_7", kind="revolute", min_position=-1.0, max_position=1.0),
    ActuatorSpec(name="left_gripper", kind="prismatic", min_position=0.0, max_position=0.08),
    ActuatorSpec(name="right_arm_1", kind="revolute", min_position=-2.7, max_position=1.5),
    ActuatorSpec(name="right_arm_2", kind="revolute", min_position=-2.0, max_position=0.3),
    ActuatorSpec(name="right_arm_3", kind="revolute", min_position=-2.9, max_position=2.9),
    ActuatorSpec(name="right_arm_4", kind="revolute", min_position=-1.3, max_position=1.5),
    ActuatorSpec(name="right_arm_5", kind="revolute", min_position=-2.9, max_position=2.9),
    ActuatorSpec(name="right_arm_6", kind="revolute", min_position=-1.0, max_position=1.0),
    ActuatorSpec(name="right_arm_7", kind="revolute", min_position=-1.0, max_position=1.0),
    ActuatorSpec(name="right_gripper", kind="prismatic", min_position=0.0, max_position=0.08),
)

ZERITH_H1_PRO_ACTUATOR_ORDER = joint_order(ZERITH_H1_PRO_ACTUATOR_SPECS)
ZERITH_H1_PRO_POSITION_ACTUATOR_ORDER = [
    spec.name for spec in ZERITH_H1_PRO_ACTUATOR_SPECS if spec.mode != "velocity"
]

ZERITH_H1_PRO_DEFAULT_POSITIONS: dict[str, dict[str, float]] = {
    "initial": {
        **{name: 0.0 for name in ZERITH_H1_PRO_POSITION_ACTUATOR_ORDER},
        "lift": 0.5,
        "left_gripper": 0.08,
        "right_gripper": 0.08,
    },
    "reset": {
        **{name: 0.0 for name in ZERITH_H1_PRO_POSITION_ACTUATOR_ORDER},
        "lift": 0.5,
        "left_gripper": 0.08,
        "right_gripper": 0.08,
    },
}


def _parse_positions(data: Any) -> dict[str, dict[str, float]] | None:
    if data is None:
        return None
    if not isinstance(data, dict):
        raise ValueError("positions 必须是字典。")

    allowed_position_names = set(ZERITH_H1_PRO_DEFAULT_POSITIONS)
    allowed_actuator_names = set(ZERITH_H1_PRO_POSITION_ACTUATOR_ORDER)
    parsed: dict[str, dict[str, float]] = {}

    for position_name, position in data.items():
        name = str(position_name)
        if name not in allowed_position_names:
            raise ValueError(
                f"未知 positions 名称: {name}，可用值: {sorted(allowed_position_names)}"
            )
        if not isinstance(position, dict):
            raise ValueError(f"positions.{name} 必须是字典。")

        parsed_position: dict[str, float] = {}
        for actuator_name, value in position.items():
            key = str(actuator_name)
            if key not in allowed_actuator_names:
                raise ValueError(
                    f"未知位置执行器名称: {key}，可用值: {ZERITH_H1_PRO_POSITION_ACTUATOR_ORDER}"
                )
            parsed_value = float(value)
            if not math.isfinite(parsed_value):
                raise ValueError(f"positions.{name}.{key} 必须是有限数值。")
            parsed_position[key] = parsed_value
        parsed[name] = parsed_position

    return parsed


ImageFormat = Literal["raw", "jpeg", "png"]
ColorOrder = Literal["bgr", "rgb"]
DepthFormat = Literal["raw_u16", "jpeg"]


@dataclass
class ZerithCameraStreamConfig:
    """Single camera stream configuration."""

    name: str
    output_id: str
    enable_color: bool = True
    enable_depth: bool = False
    depth_output_id: str | None = None
    image_format: ImageFormat = "jpeg"
    image_jpeg_quality: int = 85
    color_order: ColorOrder = "bgr"
    width: int | None = None
    height: int | None = None
    depth_format: DepthFormat = "raw_u16"
    depth_jpeg_quality: int = 85
    depth_visualization_alpha: float = 0.03

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ZerithCameraStreamConfig":
        name = str(data.get("name", "")).strip()
        if not name:
            raise ValueError("camera.name 不能为空")

        output_id = str(data.get("output_id", "")).strip()
        if not output_id:
            raise ValueError(f"camera {name}: output_id 不能为空")

        image_format = str(data.get("image_format", "jpeg"))
        if image_format not in {"raw", "jpeg", "png"}:
            raise ValueError(f"camera {name}: image_format 必须是 raw/jpeg/png")

        color_order = str(data.get("color_order", "bgr"))
        if color_order not in {"bgr", "rgb"}:
            raise ValueError(f"camera {name}: color_order 必须是 bgr/rgb")

        depth_format = str(data.get("depth_format", "raw_u16"))
        if depth_format not in {"raw_u16", "jpeg"}:
            raise ValueError(f"camera {name}: depth_format 必须是 raw_u16/jpeg")

        image_jpeg_quality = int(data.get("image_jpeg_quality", 85))
        depth_jpeg_quality = int(data.get("depth_jpeg_quality", 85))
        for key, value in {
            "image_jpeg_quality": image_jpeg_quality,
            "depth_jpeg_quality": depth_jpeg_quality,
        }.items():
            if value < 1 or value > 100:
                raise ValueError(f"camera {name}: {key} 必须在 [1, 100] 范围内")

        width = data.get("width")
        height = data.get("height")
        parsed_width = int(width) if width is not None else None
        parsed_height = int(height) if height is not None else None
        if (parsed_width is None) != (parsed_height is None):
            raise ValueError(f"camera {name}: width 和 height 必须同时设置或同时留空")
        if parsed_width is not None and (
            parsed_width <= 0 or parsed_height is None or parsed_height <= 0
        ):
            raise ValueError(f"camera {name}: width/height 必须为正整数")

        depth_output_id_raw = data.get("depth_output_id")
        depth_output_id = str(depth_output_id_raw).strip() if depth_output_id_raw else None

        return cls(
            name=name,
            output_id=output_id,
            enable_color=bool(data.get("enable_color", True)),
            enable_depth=bool(data.get("enable_depth", False)),
            depth_output_id=depth_output_id,
            image_format=image_format,  # type: ignore[arg-type]
            image_jpeg_quality=image_jpeg_quality,
            color_order=color_order,  # type: ignore[arg-type]
            width=parsed_width,
            height=parsed_height,
            depth_format=depth_format,  # type: ignore[arg-type]
            depth_jpeg_quality=depth_jpeg_quality,
            depth_visualization_alpha=float(data.get("depth_visualization_alpha", 0.03)),
        )


@dataclass
class ZerithH1ProNodeConfig:
    """Zerith H1 Pro 节点配置。"""

    sdk_root: str = DEFAULT_SDK_ROOT
    robot_ip: str | None = None
    is_follower: bool = True
    init_on_connect: bool = True
    control_mode: str = "low_level"
    switch_mode_on_connect: bool = True
    debug: bool = False
    actuator_order: list[str] | None = None
    positions: dict[str, dict[str, float]] | None = None
    lifecycle_position_move_seconds: float = 3.0
    wheel_radius_m: float = ZERITH_H1_PRO_WHEEL_RADIUS_M
    wheel_track_width_m: float = ZERITH_H1_PRO_WHEEL_TRACK_WIDTH_M
    locomotion_max_vx_mps: float = ZERITH_H1_PRO_MAX_VX_MPS
    locomotion_max_wz_radps: float = ZERITH_H1_PRO_MAX_WZ_RADPS
    locomotion_max_wheel_speed_radps: float = ZERITH_H1_PRO_MAX_WHEEL_SPEED_RADPS
    locomotion_command_timeout_seconds: float = (
        ZERITH_H1_PRO_LOCOMOTION_COMMAND_TIMEOUT_SECONDS
    )
    cameras: list[ZerithCameraStreamConfig] | None = None
    image_fps: int = 30
    joint_fps: int = 100

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ZerithH1ProNodeConfig":
        sdk_root_raw = data.get("sdk_root")
        sdk_root = resolve_sdk_root(
            str(sdk_root_raw) if sdk_root_raw is not None else None
        )
        robot_ip_raw = data.get("robot_ip")
        robot_ip = str(robot_ip_raw) if robot_ip_raw else None
        is_follower = bool(data.get("is_follower", True))
        init_on_connect = bool(data.get("init_on_connect", True))
        control_mode = str(data.get("control_mode", "low_level"))
        switch_mode_on_connect = bool(data.get("switch_mode_on_connect", True))
        debug = bool(data.get("debug", False))

        actuator_order_raw = data.get("actuator_order")
        actuator_order: list[str] | None = None
        if actuator_order_raw is not None:
            if not isinstance(actuator_order_raw, list) or not all(
                isinstance(x, str) for x in actuator_order_raw
            ):
                raise ValueError("actuator_order 必须是字符串列表")
            actuator_order = actuator_order_raw

        positions = _parse_positions(data.get("positions"))
        lifecycle_position_move_seconds = float(
            data.get("lifecycle_position_move_seconds", 3.0)
        )
        if (
            not math.isfinite(lifecycle_position_move_seconds)
            or lifecycle_position_move_seconds <= 0
        ):
            raise ValueError("lifecycle_position_move_seconds 必须是有限正数")

        wheel_radius_m = float(
            data.get("wheel_radius_m", ZERITH_H1_PRO_WHEEL_RADIUS_M)
        )
        wheel_track_width_m = float(
            data.get("wheel_track_width_m", ZERITH_H1_PRO_WHEEL_TRACK_WIDTH_M)
        )
        locomotion_max_vx_mps = float(
            data.get("locomotion_max_vx_mps", ZERITH_H1_PRO_MAX_VX_MPS)
        )
        locomotion_max_wz_radps = float(
            data.get("locomotion_max_wz_radps", ZERITH_H1_PRO_MAX_WZ_RADPS)
        )
        locomotion_max_wheel_speed_radps = float(
            data.get(
                "locomotion_max_wheel_speed_radps",
                ZERITH_H1_PRO_MAX_WHEEL_SPEED_RADPS,
            )
        )
        locomotion_command_timeout_seconds = float(
            data.get(
                "locomotion_command_timeout_seconds",
                ZERITH_H1_PRO_LOCOMOTION_COMMAND_TIMEOUT_SECONDS,
            )
        )
        locomotion_values = {
            "wheel_radius_m": wheel_radius_m,
            "wheel_track_width_m": wheel_track_width_m,
            "locomotion_max_vx_mps": locomotion_max_vx_mps,
            "locomotion_max_wz_radps": locomotion_max_wz_radps,
            "locomotion_max_wheel_speed_radps": locomotion_max_wheel_speed_radps,
            "locomotion_command_timeout_seconds": locomotion_command_timeout_seconds,
        }
        for key, value in locomotion_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{key} 必须是有限正数")
        if locomotion_max_wheel_speed_radps > ZERITH_H1_PRO_MAX_WHEEL_SPEED_RADPS:
            raise ValueError(
                "locomotion_max_wheel_speed_radps 不得超过项目保守轮速上限 "
                f"{ZERITH_H1_PRO_MAX_WHEEL_SPEED_RADPS}"
            )
        max_feasible_vx = wheel_radius_m * locomotion_max_wheel_speed_radps
        if locomotion_max_vx_mps > max_feasible_vx:
            raise ValueError(
                "locomotion_max_vx_mps 超过当前轮径和轮速允许的纯平移速度 "
                f"{max_feasible_vx:.6f}"
            )
        max_feasible_wz = (
            2.0
            * wheel_radius_m
            * locomotion_max_wheel_speed_radps
            / wheel_track_width_m
        )
        if locomotion_max_wz_radps > max_feasible_wz:
            raise ValueError(
                "locomotion_max_wz_radps 超过当前轮距、轮径和轮速允许的纯旋转速度 "
                f"{max_feasible_wz:.6f}"
            )

        cameras_raw = data.get("cameras")
        cameras: list[ZerithCameraStreamConfig] | None = None
        if cameras_raw is not None:
            if not isinstance(cameras_raw, list):
                raise ValueError("cameras 必须是列表")
            cameras = []
            for item in cameras_raw:
                if not isinstance(item, dict):
                    raise ValueError("cameras 中的每一项都必须是对象")
                cameras.append(ZerithCameraStreamConfig.from_dict(item))

        image_fps = int(data.get("image_fps", 30))
        joint_fps = int(data.get("joint_fps", 100))
        if image_fps <= 0:
            raise ValueError("image_fps 必须是正整数")
        if joint_fps <= 0:
            raise ValueError("joint_fps 必须是正整数")

        return cls(
            sdk_root=sdk_root,
            robot_ip=robot_ip,
            is_follower=is_follower,
            init_on_connect=init_on_connect,
            control_mode=control_mode,
            switch_mode_on_connect=switch_mode_on_connect,
            debug=debug,
            actuator_order=actuator_order,
            positions=positions,
            lifecycle_position_move_seconds=lifecycle_position_move_seconds,
            wheel_radius_m=wheel_radius_m,
            wheel_track_width_m=wheel_track_width_m,
            locomotion_max_vx_mps=locomotion_max_vx_mps,
            locomotion_max_wz_radps=locomotion_max_wz_radps,
            locomotion_max_wheel_speed_radps=locomotion_max_wheel_speed_radps,
            locomotion_command_timeout_seconds=locomotion_command_timeout_seconds,
            cameras=cameras,
            image_fps=image_fps,
            joint_fps=joint_fps,
        )

    @property
    def camera_grpc_target(self) -> str:
        """Derive the fixed camera endpoint from the robot host."""
        host = self.robot_ip or LOCAL_CAMERA_HOST
        return f"{host}:{CAMERA_GRPC_PORT}"

    def resolved_position(self, name: str) -> dict[str, float]:
        """返回完整生命周期位姿，未配置的执行器使用默认值。"""
        if name not in ZERITH_H1_PRO_DEFAULT_POSITIONS:
            raise ValueError(
                f"未知 positions 名称: {name}，"
                f"可用值: {sorted(ZERITH_H1_PRO_DEFAULT_POSITIONS)}"
            )
        position = dict(ZERITH_H1_PRO_DEFAULT_POSITIONS[name])
        if self.positions and name in self.positions:
            position.update(self.positions[name])
        return position

    @classmethod
    def from_yaml_path(cls, path: str | Path) -> "ZerithH1ProNodeConfig":
        p = Path(path)
        if not p.exists():
            raise FileNotFoundError(f"配置文件不存在: {p}")
        with open(p, encoding="utf-8") as f:
            data = yaml.safe_load(f)
        if not data:
            raise ValueError(f"配置文件为空: {p}")
        sdk_root_raw = data.get("sdk_root")
        config = cls.from_dict(data)
        config.sdk_root = resolve_sdk_root(
            str(sdk_root_raw) if sdk_root_raw is not None else None,
            config_file_parent=p.parent,
        )
        return config


def load_config(
    config_path: str | Path | None = None,
    dataflow_config: dict[str, Any] | None = None,
) -> ZerithH1ProNodeConfig:
    """
    加载配置，支持多种来源（优先级从高到低）：
    1. dataflow_config 中的 config_path 或内联 config
    2. config_path 参数
    3. 环境变量 ZERITH_H1_PRO_NODE_CONFIG
    """
    data: dict[str, Any] | None = None

    if dataflow_config:
        if "config" in dataflow_config:
            data = dataflow_config["config"]
        elif "config_path" in dataflow_config:
            return ZerithH1ProNodeConfig.from_yaml_path(dataflow_config["config_path"])

    path = config_path
    if path is None and data is None:
        path = os.environ.get("ZERITH_H1_PRO_NODE_CONFIG")

    if path:
        return ZerithH1ProNodeConfig.from_yaml_path(path)
    if data:
        return ZerithH1ProNodeConfig.from_dict(data)

    raise ValueError(
        "未找到配置。请设置 ZERITH_H1_PRO_NODE_CONFIG 环境变量、"
        "或通过 --config 指定配置文件、或使用 dataflow config。"
    )
