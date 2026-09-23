#!/usr/bin/env python3
"""
测试合并后的 Zerith H1 Pro 驱动。
此脚本在不移动机器人的前提下，启动后台线程，高频读取关节状态并拉取相机图像，输出采集统计数据（FPS 等）。
"""

import os
import sys
import time
from pathlib import Path

# 将父目录（即 packages/zerith_h1_pro）加入 sys.path
parent_dir = Path(__file__).resolve().parents[2]
if str(parent_dir) not in sys.path:
    sys.path.insert(0, str(parent_dir))

from zerith_h1_pro.config import load_config
from zerith_h1_pro.driver import ZerithH1ProDriver


def main():
    print("正在加载配置文件...")
    config_path = parent_dir / "config" / "config.yaml"
    if not config_path.exists():
        config_path = parent_dir / "config" / "config.example.yaml"
        print(f"config.yaml 不存在，将使用示例配置 {config_path}")

    try:
        config = load_config(config_path=config_path)
    except Exception as e:
        print(f"加载配置失败: {e}")
        return

    # 为了安全，默认只读不写
    print("\n--- 驱动初始化配置 ---")
    print(f"SDK 根目录: {config.sdk_root}")
    print(f"机器人 IP: {config.robot_ip}")
    print(f"控制模式 (只读验证不控制): {config.control_mode}")
    print(f"相机 gRPC 目标: {config.camera_grpc_target}")
    print(f"配置的关节读取线程 FPS: {config.joint_fps}")
    print(f"配置的图像读取线程 FPS: {config.image_fps}")
    if config.cameras:
        print("配置的相机流:")
        for cam in config.cameras:
            print(f"  - 名字: {cam.name}, 对应输出 ID: {cam.output_id}, 彩色: {cam.enable_color}, 深度: {cam.enable_depth}")
    else:
        print("未配置任何相机流。若需要测试相机，请在 config.yaml 中配置 cameras 项。")
    print("----------------------\n")

    print("正在建立连接，启动多线程高频读取（不发出动作指令）...")
    try:
        # 建立连接并开启后台双线程采集
        driver = ZerithH1ProDriver(
            sdk_root=config.sdk_root,
            robot_ip=config.robot_ip,
            is_follower=False, # 只读测试
            init_on_connect=config.init_on_connect,
            control_mode=config.control_mode,
            switch_mode_on_connect=config.switch_mode_on_connect,
            actuator_order=config.actuator_order,
            camera_grpc_target=config.camera_grpc_target,
            cameras=config.cameras,
            image_fps=config.image_fps,
            joint_fps=config.joint_fps,
            auto_connect=True,
        )
    except Exception as e:
        print(f"\n连接或加载 SDK 失败: {e}")
        print("请确认您的 python 运行环境（建议 Python 3.12）及 SDK 路径是否正确。")
        return

    print("\n连接建立成功！开始持续 10 秒的状态与相机图像读取频率统计...")
    
    start_time = time.time()
    last_print = start_time
    
    # 频率计数器
    state_read_count = 0
    camera_read_counts = {}
    if config.cameras:
        for cam in config.cameras:
            if cam.enable_color:
                camera_read_counts[cam.output_id] = 0
            if cam.enable_depth and cam.depth_output_id:
                camera_read_counts[cam.depth_output_id] = 0

    try:
        while time.time() - start_time < 10.0:
            now = time.time()
            
            # 1. 模拟控制政策周期读取关节状态（100Hz 模拟）
            try:
                state = driver.get_state()
                state_read_count += 1
            except Exception as e:
                pass

            # 2. 模拟读取最新相机数据并计数
            if config.cameras:
                for out_id in camera_read_counts.keys():
                    img_payload = driver.get_latest_encoded_image(out_id)
                    if img_payload is not None:
                        camera_read_counts[out_id] += 1

            # 3. 每 1 秒打印一次状态与 FPS
            if now - last_print >= 1.0:
                dt = now - last_print
                joint_fps = state_read_count / dt
                print(f"[{time.strftime('%H:%M:%S')}] 关节状态拉取 FPS: {joint_fps:.1f}")
                
                # 打印最新部分关节的角度样本
                try:
                    state_sample = driver.get_state()
                    sample = list(zip(state_sample.name, state_sample.position, strict=False))[:3]
                    sample_str = ", ".join(
                        [f"{name}: {float(position):.4f}" for name, position in sample]
                    )
                    print(f"    关节最新快照数据样本 -> {sample_str}")
                except Exception:
                    pass

                # 打印各相机流的帧率统计
                for out_id, count in camera_read_counts.items():
                    cam_fps = count / dt
                    print(f"    相机流 [{out_id}] 拉取 FPS: {cam_fps:.1f}")
                    # 归零重新计算当前周期的 FPS
                    camera_read_counts[out_id] = 0
                
                state_read_count = 0
                last_print = now
            
            time.sleep(0.01) # 100Hz 的控制/读取频率

    except KeyboardInterrupt:
        print("\n测试被用户中断。")
    finally:
        print("\n正在优雅停止后台线程、断开连接并清理资源...")
        driver.disconnect()
        print("清理完成，测试结束。")


if __name__ == "__main__":
    main()
