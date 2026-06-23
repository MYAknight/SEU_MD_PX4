#!/usr/bin/env python3
"""
相机偏移标定工具

用法:
    rosrun ground_station calibrate_camera_offset.py
    或
    python3 ~/Projects/ground_station/scripts/calibrate_camera_offset.py

前提:
    YOLO 节点必须已在运行并发布 /yolo/pixel_error。
    例如先启动:
        ./start_ground_station_auto.sh
    或单独启动 YOLO:
        roslaunch YOLO4SEU_MD rtsp_pillar.launch rtsp_url:=rtsp://192.168.144.25:8554/main.264

流程:
    1. 将无人机机头正对柱子（此时无人机中心与柱子对齐）
    2. 观察终端中实时 pixel_error
    3. 按 Enter 记录当前 pixel_error 的负值作为相机偏移
    4. 结果保存到 ~/.config/ground_station/camera_offset.yaml

保存的参数会被 control_ground_station_auto.py 启动时自动加载。
"""
import os
import sys
import time
import threading
from pathlib import Path

import yaml
import rospy
from std_msgs.msg import Float32


CONFIG_PATH = Path.home() / ".config" / "ground_station" / "camera_offset.yaml"
IMAGE_WIDTH = 640.0  # 与检测节点默认宽度一致


def save_offset(offset_x_px: float, pixel_error: float):
    CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(CONFIG_PATH, "w") as f:
        yaml.safe_dump({
            "camera_offset_x": float(offset_x_px),
            "image_width": float(IMAGE_WIDTH),
            "pixel_error_at_calibration": float(pixel_error),
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        }, f, default_flow_style=False, allow_unicode=True)
    print(f"\n[OK] 相机偏移已保存到 {CONFIG_PATH}")
    print(f"     camera_offset_x = {offset_x_px:.1f} px")
    print(f"     pixel_error     = {pixel_error:+.4f}")


class StdinWatcher(threading.Thread):
    """Simple thread that waits for Enter key without blocking the ROS loop."""
    def __init__(self):
        super().__init__(daemon=True)
        self.triggered = False
        self.start()

    def run(self):
        try:
            sys.stdin.readline()
            self.triggered = True
        except Exception:
            pass


class CalibNode:
    def __init__(self):
        rospy.init_node("calibrate_camera_offset", anonymous=True)
        self.pixel_error = 0.0
        self.last_update = 0.0
        rospy.Subscriber("/yolo/pixel_error", Float32, self._on_error)

    def _on_error(self, msg):
        self.pixel_error = float(msg.data)
        self.last_update = time.time()

    def run(self):
        print("=" * 60)
        print("  相机偏移标定")
        print("=" * 60)
        print("前提：YOLO 节点已在运行并发布 /yolo/pixel_error")
        print("操作：将无人机机头正对柱子，按 Enter 记录偏移；按 Ctrl+C 退出。\n")

        watcher = StdinWatcher()
        rate = rospy.Rate(10)
        warned = False
        while not rospy.is_shutdown():
            active = (time.time() - self.last_update) < 1.0
            if active:
                print(f"\rpixel_error = {self.pixel_error:+.4f}  [按 Enter 标定]      ", end="", flush=True)
                warned = False
            else:
                print(f"\r等待 /yolo/pixel_error 数据...  ", end="", flush=True)
                if not warned and (time.time() - self.last_update) > 3.0 and self.last_update == 0.0:
                    print("\n[WARN] 尚未收到 /yolo/pixel_error，请确认 YOLO 节点已启动")
                    print("       例如: roslaunch YOLO4SEU_MD rtsp_pillar.launch")
                    warned = True

            if watcher.triggered:
                if not active:
                    print("\n[ERROR] 当前没有有效的 pixel_error 数据，无法标定")
                    watcher = StdinWatcher()  # reset for next try
                    continue
                offset_px = -self.pixel_error * (IMAGE_WIDTH / 2.0)
                save_offset(offset_px, self.pixel_error)
                return

            rate.sleep()


if __name__ == "__main__":
    try:
        CalibNode().run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n[EXIT] 用户取消")
