import argparse
import importlib.machinery
import os
import sys
import time
from pathlib import Path

# 将 SDK 根目录加入 sys.path，以便导入 lib/lib_h1_sdk_python.so
root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if root not in sys.path:
    sys.path.insert(0, root)

try:
    from lib.lib_h1_sdk_python import EtherCAT_Motor_Index, H1Robot
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


DEFAULT_ROBOT_IP = "10.20.0.67"
DEFAULT_TARGET_SWEEP_HZ = 500.0

JOINT_NAMES = [
    "MOTOR_LIFT",
    "MOTOR_WAIST_DOWN",
    "MOTOR_WAIST_UP",
    "MOTOR_HEAD_DOWN",
    "MOTOR_HEAD_UP",
    "MOTOR_LEFT_ARM_1",
    "MOTOR_LEFT_ARM_2",
    "MOTOR_LEFT_ARM_3",
    "MOTOR_LEFT_ARM_4",
    "MOTOR_LEFT_ARM_5",
    "MOTOR_LEFT_ARM_6",
    "MOTOR_LEFT_ARM_7",
    "MOTOR_LEFT_ARM_8",
    "MOTOR_RIGHT_ARM_1",
    "MOTOR_RIGHT_ARM_2",
    "MOTOR_RIGHT_ARM_3",
    "MOTOR_RIGHT_ARM_4",
    "MOTOR_RIGHT_ARM_5",
    "MOTOR_RIGHT_ARM_6",
    "MOTOR_RIGHT_ARM_7",
    "MOTOR_RIGHT_ARM_8",
]

WHEEL_NAMES = [
    "MOTOR_WHEEL_LEFT",
    "MOTOR_WHEEL_RIGHT",
]


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="测试 H1 getMotorState 关节状态读取频率")
    parser.add_argument(
        "--host",
        default=DEFAULT_ROBOT_IP,
        help=f"机器人 IP；传空字符串则使用 H1Robot() 默认连接方式。默认: {DEFAULT_ROBOT_IP}",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="测试时长，单位秒；0 表示持续运行直到 Ctrl+C。默认: 0",
    )
    parser.add_argument(
        "--report-interval",
        type=float,
        default=1.0,
        help="控制台统计输出周期，单位秒。默认: 1.0",
    )
    parser.add_argument(
        "--sleep",
        type=float,
        default=0.0,
        help="每轮完整扫描后的额外休眠时间，单位秒；用于降低 CPU 占用。默认: 0",
    )
    parser.add_argument(
        "--target-sweep-hz",
        type=float,
        default=DEFAULT_TARGET_SWEEP_HZ,
        help=f"限制完整扫描频率；0 表示不限制。默认: {DEFAULT_TARGET_SWEEP_HZ}",
    )
    parser.add_argument(
        "--include-wheels",
        action="store_true",
        help="同时读取左右轮电机状态。",
    )
    return parser


def resolve_motor_indices(include_wheels: bool):
    names = list(JOINT_NAMES)
    if include_wheels:
        names = WHEEL_NAMES + names

    motors = []
    for name in names:
        motors.append((name, getattr(EtherCAT_Motor_Index, name)))
    return motors


def format_motor_sample(name, state) -> str:
    return (
        f"{name}: pos={state.Position_Actual:.4f}, "
        f"spd={state.Speed_Actual:.4f}, "
        f"tq={state.Torque_Actual:.4f}, "
        f"err={state.Error_flag}"
    )


def motor_state_signature(state):
    return (
        state.Position_Actual,
        state.Speed_Actual,
        state.Torque_Actual,
        state.KP_Actual,
        state.KD_Actual,
        state.Error_flag,
    )


def main() -> None:
    args = build_arg_parser().parse_args()
    motors = resolve_motor_indices(args.include_wheels)

    robot = H1Robot(args.host) if args.host else H1Robot()
    if not robot.robot_connect():
        print("robot_connect failed")
        return

    print("connected =", robot.isRobotConnected())
    print(f"[*] 开始读取 {len(motors)} 个关节/电机状态，按 Ctrl+C 退出。")
    print("[*] 统计口径: poll=缓存读取频率，changed=状态值变化频率，sweep=完整扫描频率。")
    if args.target_sweep_hz > 0:
        print(f"[*] 已限制完整扫描频率: {args.target_sweep_hz:.1f} Hz")

    start_time = time.perf_counter()
    report_start = start_time
    sweep_period = 1.0 / args.target_sweep_hz if args.target_sweep_hz > 0 else 0.0
    next_sweep_time = start_time
    total_calls = 0
    ok_calls = 0
    changed_calls = 0
    fail_calls = 0
    sweeps = 0
    changed_sweeps = 0
    last_states = {}
    sample_text = "no valid sample yet"

    try:
        while True:
            sweep_changed = False

            for name, idx in motors:
                total_calls += 1
                ok, state = robot.getMotorState(idx)
                if ok:
                    ok_calls += 1
                    sample_text = format_motor_sample(name, state)
                    signature = motor_state_signature(state)
                    if name in last_states and last_states[name] != signature:
                        changed_calls += 1
                        sweep_changed = True
                    last_states[name] = signature
                else:
                    fail_calls += 1

            sweeps += 1
            if sweep_changed:
                changed_sweeps += 1

            if args.sleep > 0:
                time.sleep(args.sleep)

            now = time.perf_counter()
            if sweep_period > 0:
                next_sweep_time += sweep_period
                sleep_time = next_sweep_time - now
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    now = time.perf_counter()
                elif -sleep_time > sweep_period:
                    next_sweep_time = now

            elapsed = now - report_start
            if elapsed >= args.report_interval:
                poll_hz = total_calls / elapsed
                ok_hz = ok_calls / elapsed
                changed_hz = changed_calls / elapsed
                sweep_hz = sweeps / elapsed
                changed_sweep_hz = changed_sweeps / elapsed
                print(
                    f"[RATE] poll={poll_hz:.1f} Hz, ok={ok_hz:.1f} Hz, "
                    f"changed={changed_hz:.1f} Hz, sweep={sweep_hz:.1f} Hz, "
                    f"changed_sweep={changed_sweep_hz:.1f} Hz, fail={fail_calls}, "
                    f"sample=({sample_text})"
                )
                report_start = now
                total_calls = 0
                ok_calls = 0
                changed_calls = 0
                fail_calls = 0
                sweeps = 0
                changed_sweeps = 0

            if args.duration > 0 and now - start_time >= args.duration:
                break
    except KeyboardInterrupt:
        print("\n[+] 用户中断。")


if __name__ == "__main__":
    main()
