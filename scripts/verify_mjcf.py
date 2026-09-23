#!/usr/bin/env python3
"""MJCF 快速核查：能否加载、关节名与限位、body 数量。"""
from __future__ import annotations

import argparse
import os
import sys


def main() -> int:
    p = argparse.ArgumentParser(description="Verify MuJoCo MJCF loads and print joint summary.")
    p.add_argument("--xml", required=True, help="Path to scene.xml or main mjcf file")
    args = p.parse_args()
    xml_path = os.path.abspath(args.xml)
    if not os.path.isfile(xml_path):
        print(f"File not found: {xml_path}", file=sys.stderr)
        return 2

    try:
        import mujoco
    except ImportError:
        print("Install mujoco: pip install mujoco", file=sys.stderr)
        return 1

    try:
        m = mujoco.MjModel.from_xml_path(xml_path)
    except Exception as e:
        print(f"Failed to load MJCF: {e}", file=sys.stderr)
        return 1

    print(f"ok load: {xml_path}")
    print(f"  nbody={m.nbody} njoint={m.njnt} nq={m.nq} nv={m.nv} nu={m.nu}")

    for j in range(m.njnt):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or f"<joint {j}>"
        jnt_type = m.jnt_type[j]
        type_str = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}.get(int(jnt_type), str(int(jnt_type)))
        qadr = int(m.jnt_qposadr[j])
        range_str = ""
        if m.jnt_limited[j]:
            lo, hi = m.jnt_range[j]
            range_str = f" range=[{lo:.6g}, {hi:.6g}]"
        print(f"  joint[{j}] {name!r} type={type_str} qposadr={qadr}{range_str}")

    print("  tip: compare names/order with simulator.yaml / TaskRobot actuator_order")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
