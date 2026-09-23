"""
@file 01_minimal_color.py
@brief 【示例 01】最简彩色图获取
* [功能介绍]
1. 展示如何初始化 CameraClient 并建立连接。
2. 演示非阻塞式获取最新彩色图像 (RGB) 并使用 OpenCV 显示。
* [使用说明]
1. 修改 GRPC_TARGET 为服务端真实的 IP 地址。
2. 修改 CAMERA_NAME 为服务端对应的相机 ID。
3. 运行脚本后，按 'q' 键退出预览。
"""

import os
import sys
import time
import threading
import cv2

# --- 路径兼容处理：确保能找到上一级目录中的 camera_client.py ---
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from camera_client import CameraClient

def camera_worker(client, camera_name, latest_frames, frame_lock, stop_event):
    """后台线程：持续获取指定相机的最新帧。"""
    print(f"[*] 线程启动，正在读取相机 [{camera_name}] ...")
    stats_interval = 1.0
    stats_start = time.monotonic()
    frame_count = 0
    last_ts = None

    while not stop_event.is_set():
        try:
            # 获取最新彩色帧 (返回: (numpy_bgr_array, timestamp) 或 None)
            data = client.get_latest_frame(camera_name)
            if data:
                img, ts = data
                # get_latest_frame 会反复返回缓存中的最新帧；只有时间戳变化才算新帧。
                if ts != last_ts:
                    last_ts = ts
                    frame_count += 1
                    with frame_lock:
                        latest_frames[camera_name] = (img.copy(), ts)
                else:
                    time.sleep(0.001)
            else:
                time.sleep(0.005)

            now = time.monotonic()
            elapsed = now - stats_start
            if elapsed >= stats_interval:
                fps = frame_count / elapsed
                print(f"[FPS] {camera_name}: {fps:.1f} Hz (new frames)")
                stats_start = now
                frame_count = 0
        except Exception as e:
            print(f"[!] 相机 [{camera_name}] 读取出错: {e}")
            stop_event.set()
            break

def main():
    # ---------------------------------------------------------
    # 【用户配置区】请根据实际环境修改以下参数
    # ---------------------------------------------------------
    GRPC_TARGET = "10.20.0.67:50051"  # 服务端 IP:端口
    CAMERA_NAMES = [
        "rs/cam_left_wrist",
        "rs/cam_high",
        "rs/cam_right_wrist",
    ]  # 彩色相机 ID
    # ---------------------------------------------------------

    # 1. 初始化客户端
    client = CameraClient(grpc_target=GRPC_TARGET)
    stop_event = threading.Event()
    frame_lock = threading.Lock()
    latest_frames = {}
    workers = []
    
    try:
        # 2. 启动客户端 (执行 gRPC 握手与 WebRTC 建立)
        print(f"[*] 正在连接服务 {GRPC_TARGET} ...")
        client.start()
        print(f"[+] 连接成功。正在获取相机 {CAMERA_NAMES} 的画面...")

        # 3. 为每路相机启动一个取流线程
        for camera_name in CAMERA_NAMES:
            worker = threading.Thread(
                target=camera_worker,
                args=(client, camera_name, latest_frames, frame_lock, stop_event),
                daemon=True,
            )
            worker.start()
            workers.append(worker)

        # 4. OpenCV 窗口渲染统一放在主线程，避免多线程 GUI 调用导致卡死
        while True:
            with frame_lock:
                frames_snapshot = latest_frames.copy()

            for camera_name, (img, ts) in frames_snapshot.items():
                cv2.imshow(f"Color Viewer - {camera_name}", img)
            
            # 按 'q' 键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n[+] 用户请求退出。")
                break

    except Exception as e:
        print(f"\n[!] 运行出错: {e}")
    finally:
        # 5. 停止线程和客户端，释放网络资源
        print("[*] 正在释放资源并断开连接...")
        stop_event.set()
        for worker in workers:
            worker.join(timeout=1.0)
        client.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()