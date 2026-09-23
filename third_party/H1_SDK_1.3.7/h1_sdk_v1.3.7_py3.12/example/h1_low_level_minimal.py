import os
import sys
import time
import importlib.machinery
from pathlib import Path

root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if root not in sys.path:
    sys.path.insert(0, root)

try:
    from lib.lib_h1_sdk_python import (
        EtherCAT_Motor_Index,
        H1Robot,
        Motor_Control,
        MotorControlMode,
    )
except ModuleNotFoundError as exc:
    if exc.name != "lib.lib_h1_sdk_python":
        raise

    lib_dir = Path(root) / "lib"
    built_extensions = sorted(path.name for path in lib_dir.glob("lib_h1_sdk_python*.so"))
    raise RuntimeError(
        "Cannot import lib.lib_h1_sdk_python. This package was built for Python 3.12; "
        f"current interpreter is {sys.version.split()[0]} ({sys.executable}). "
        f"Importable extension suffixes are {importlib.machinery.EXTENSION_SUFFIXES}. "
        f"Built extension files are {built_extensions}. "
        "Run with python3.12 or rebuild the extension for the active Python version."
    ) from exc


def main() -> None:
    robot = H1Robot("10.20.0.67")
    # robot = H1Robot("172.31.200.1")

    if not robot.robot_connect():
        print("robot_connect failed")
        return

    if not robot.switchControlMode(MotorControlMode.LOW_LEVEL):
        print("switchControlMode LOW_LEVEL failed")
        return

    if not robot.robot_init():
        print("robot_init failed")
        return

    try:
        control = Motor_Control()
        control.Speed = 0.0
        robot.setMotorControl_low(EtherCAT_Motor_Index.MOTOR_WHEEL_LEFT, control)
        robot.setMotorControl_low(EtherCAT_Motor_Index.MOTOR_WHEEL_RIGHT, control)

        ok, left_state = robot.getMotorState(EtherCAT_Motor_Index.MOTOR_WHEEL_LEFT)
        print("left wheel state:", ok, left_state.Position_Actual, left_state.Speed_Actual)
        time.sleep(1.0)
    finally:
        robot.robot_deinit()


if __name__ == "__main__":
    main()
