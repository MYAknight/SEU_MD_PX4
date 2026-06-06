#!/usr/bin/env python3
"""
huaqiccc_deform_control_gui.py
==============================
huaqiccc 机臂变形控制 GUI

用法:
    # 终端 1：启动仿真（自动启动 roscore + PX4 + Gazebo + MAVROS）
    roslaunch ~/PX4-Autopilot/launch/mavros_posix_sitl.launch

    # 终端 2：启动 GUI（先 source ROS 环境）
    source /opt/ros/noetic/setup.bash
    python3 huaqiccc_deform_control_gui.py

话题:
- 发布: /huaqiccc/arm_angle (std_msgs/Float64)
- 订阅: /huaqiccc/arm_status (std_msgs/String, JSON, 可选)
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import json
import math

try:
    import rospy
    from std_msgs.msg import Float64, String
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

# ===================== 参数配置 =====================
DEFAULT_OPEN_ANGLE = -0.3
DEFAULT_CLOSE_ANGLE = 0.0
ANGLE_MIN = -0.5
ANGLE_MAX = 0.0
SPEED_MIN = 0.05
SPEED_MAX = 1.0
SPEED_DEFAULT = 0.3
UPDATE_HZ = 20.0
DT = 1.0 / UPDATE_HZ

# 字体：使用系统通用字体，避免 Xlib 渲染错误
FONT_UI = "DejaVu Sans"
FONT_UI_BOLD = ("DejaVu Sans", 10, "bold")
FONT_MONO = ("DejaVu Sans Mono", 20, "bold")
FONT_BTN = ("DejaVu Sans", 12, "bold")
FONT_SMALL = ("DejaVu Sans", 9)


class ArmController:
    """后台 ROS 控制器：平滑插值 + 话题发布"""

    def __init__(self):
        self.current_angle = 0.0
        self.target_angle = 0.0
        self.speed = SPEED_DEFAULT
        self.running = False
        self.ros_connected = False
        self.real_angle_right = None
        self.real_angle_left = None

        self.pub = None
        self.sub = None

        if not ROS_AVAILABLE:
            print("[WARN] rospy 未安装。将以离线模式运行。")
            return

        try:
            rospy.init_node("huaqiccc_deform_gui", anonymous=True)
            self.pub = rospy.Publisher("/huaqiccc/arm_angle", Float64, queue_size=1)
            self.sub = rospy.Subscriber("/huaqiccc/arm_status", String, self._on_status)
            self.ros_connected = True
            print("[OK] ROS 节点初始化成功")
        except Exception as e:
            print(f"[WARN] ROS init failed: {e}")
            self.ros_connected = False

    def _on_status(self, msg):
        try:
            data = json.loads(msg.data)
            if "right_angle" in data and data["right_angle"] is not None:
                self.real_angle_right = float(data["right_angle"])
            if "left_angle" in data and data["left_angle"] is not None:
                self.real_angle_left = float(data["left_angle"])
        except Exception:
            pass

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False

    def set_target(self, angle):
        self.target_angle = max(ANGLE_MIN, min(ANGLE_MAX, float(angle)))

    def set_speed(self, speed):
        self.speed = max(SPEED_MIN, min(SPEED_MAX, float(speed)))

    def _loop(self):
        while self.running:
            diff = self.target_angle - self.current_angle
            if abs(diff) > 1e-6:
                step = self.speed * DT
                if abs(diff) <= step:
                    self.current_angle = self.target_angle
                else:
                    self.current_angle += math.copysign(step, diff)

                if self.pub and self.ros_connected:
                    try:
                        msg = Float64()
                        msg.data = self.current_angle
                        self.pub.publish(msg)
                    except Exception:
                        pass
            time.sleep(DT)


class DeformControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("huaqiccc 机臂变形控制")
        self.root.geometry("420x600")
        self.root.resizable(True, True)
        self.root.minsize(380, 520)
        self.root.configure(bg="#1e1e2e")

        self.bg_main = "#1e1e2e"
        self.bg_card = "#2d2d44"
        self.fg_main = "#cdd6f4"
        self.fg_dim = "#6c7086"
        self.accent = "#89b4fa"
        self.success = "#a6e3a1"
        self.danger = "#f38ba8"

        self.controller = ArmController()
        self.controller.start()

        self._build_ui()
        self._update_ui()

    def _build_ui(self):
        # ===== 顶部状态栏 =====
        self.status_frame = tk.Frame(self.root, bg=self.bg_main, padx=16, pady=8)
        self.status_frame.pack(fill="x")

        self.status_dot = tk.Label(self.status_frame, text="●", font=(FONT_UI, 14), bg=self.bg_main)
        self.status_dot.pack(side="left")
        self.status_text = tk.Label(self.status_frame, text="初始化中...", font=(FONT_UI, 11),
                                    bg=self.bg_main, fg=self.fg_main)
        self.status_text.pack(side="left", padx=(4, 0))

        # ===== 角度显示卡片 =====
        card = tk.Frame(self.root, bg=self.bg_card, padx=20, pady=12)
        card.pack(fill="x", padx=16, pady=6)

        tk.Label(card, text="当前目标角度", font=(FONT_UI, 10), bg=self.bg_card, fg=self.fg_dim).pack(anchor="w")
        self.lbl_current_angle = tk.Label(card, text="0.000 rad", font=FONT_MONO,
                                           bg=self.bg_card, fg=self.accent)
        self.lbl_current_angle.pack(anchor="w", pady=(2, 4))

        self.lbl_target_angle = tk.Label(card, text="目标: 0.000 rad", font=(FONT_UI, 10),
                                          bg=self.bg_card, fg=self.fg_dim)
        self.lbl_target_angle.pack(anchor="w")

        self.lbl_real_angles = tk.Label(card, text="实际角度: --", font=FONT_SMALL,
                                         bg=self.bg_card, fg=self.fg_dim)
        self.lbl_real_angles.pack(anchor="w", pady=(4, 0))

        # ===== 速度控制卡片 =====
        card_speed = tk.Frame(self.root, bg=self.bg_card, padx=20, pady=12)
        card_speed.pack(fill="x", padx=16, pady=6)

        tk.Label(card_speed, text="变形速度", font=FONT_UI_BOLD, bg=self.bg_card, fg=self.fg_main).pack(anchor="w")
        tk.Label(card_speed, text="控制机臂展开 / 收拢的快慢", font=(FONT_UI, 9),
                 bg=self.bg_card, fg=self.fg_dim).pack(anchor="w", pady=(0, 6))

        speed_row = tk.Frame(card_speed, bg=self.bg_card)
        speed_row.pack(fill="x")

        tk.Label(speed_row, text="慢", font=FONT_SMALL, bg=self.bg_card, fg=self.fg_dim).pack(side="left")
        self.scale_speed = tk.Scale(speed_row, from_=SPEED_MIN, to=SPEED_MAX, resolution=0.01,
                                    orient="horizontal", length=230, sliderlength=16,
                                    bg=self.bg_card, highlightthickness=0,
                                    troughcolor="#3d3d5c", activebackground=self.accent,
                                    command=self._on_speed_change)
        self.scale_speed.set(SPEED_DEFAULT)
        self.scale_speed.pack(side="left", padx=6, expand=True, fill="x")
        tk.Label(speed_row, text="快", font=FONT_SMALL, bg=self.bg_card, fg=self.fg_dim).pack(side="left")

        self.lbl_speed_val = tk.Label(card_speed, text=f"{SPEED_DEFAULT:.2f} rad/s", font=(FONT_UI, 11),
                                       bg=self.bg_card, fg=self.accent)
        self.lbl_speed_val.pack(anchor="e", pady=(4, 0))

        # ===== 目标角度微调卡片 =====
        card_angle = tk.Frame(self.root, bg=self.bg_card, padx=20, pady=12)
        card_angle.pack(fill="x", padx=16, pady=6)

        tk.Label(card_angle, text="目标角度微调", font=FONT_UI_BOLD, bg=self.bg_card, fg=self.fg_main).pack(anchor="w")
        tk.Label(card_angle, text="拖动滑块精确控制展开程度", font=(FONT_UI, 9),
                 bg=self.bg_card, fg=self.fg_dim).pack(anchor="w", pady=(0, 6))

        angle_row = tk.Frame(card_angle, bg=self.bg_card)
        angle_row.pack(fill="x")

        tk.Label(angle_row, text="收拢", font=FONT_SMALL, bg=self.bg_card, fg=self.fg_dim).pack(side="left")
        self.scale_angle = tk.Scale(angle_row, from_=ANGLE_MIN, to=ANGLE_MAX, resolution=0.01,
                                     orient="horizontal", length=230, sliderlength=16,
                                     bg=self.bg_card, highlightthickness=0,
                                     troughcolor="#3d3d5c", activebackground=self.accent,
                                     command=self._on_angle_change)
        self.scale_angle.set(DEFAULT_OPEN_ANGLE)
        self.scale_angle.pack(side="left", padx=6, expand=True, fill="x")
        tk.Label(angle_row, text="展开", font=FONT_SMALL, bg=self.bg_card, fg=self.fg_dim).pack(side="left")

        # ===== 主控制按钮区 =====
        btn_frame = tk.Frame(self.root, bg=self.bg_main, padx=16, pady=12)
        btn_frame.pack(fill="x", pady=(4, 0))

        self.btn_open = tk.Button(btn_frame, text="一键展开", font=FONT_BTN,
                                   bg=self.success, fg="#1e1e2e", activebackground="#81c8be",
                                   bd=0, padx=16, pady=14, cursor="hand2",
                                   command=self._on_open)
        self.btn_open.pack(side="left", expand=True, fill="x", padx=(0, 6))

        self.btn_close = tk.Button(btn_frame, text="一键收拢", font=FONT_BTN,
                                    bg=self.danger, fg="#1e1e2e", activebackground="#eba0ac",
                                    bd=0, padx=16, pady=14, cursor="hand2",
                                    command=self._on_close)
        self.btn_close.pack(side="left", expand=True, fill="x", padx=(6, 0))

        # ===== 紧急停止 =====
        stop_frame = tk.Frame(self.root, bg=self.bg_main, padx=16, pady=6)
        stop_frame.pack(fill="x")

        self.btn_stop = tk.Button(stop_frame, text="紧急停止", font=(FONT_UI, 11),
                                   bg="#313244", fg=self.danger, activebackground="#45475a",
                                   bd=0, padx=16, pady=10, cursor="hand2",
                                   command=self._on_stop)
        self.btn_stop.pack(fill="x")

        # ===== 底部提示 =====
        tk.Label(self.root, text="提示: 先启动仿真 (roslaunch mavros_posix_sitl.launch)，再打开此 GUI",
                 font=FONT_SMALL, bg=self.bg_main, fg=self.fg_dim).pack(pady=(8, 12))

    def _on_speed_change(self, val):
        s = float(val)
        self.controller.set_speed(s)
        self.lbl_speed_val.config(text=f"{s:.2f} rad/s")

    def _on_angle_change(self, val):
        a = float(val)
        self.controller.set_target(a)
        self.lbl_target_angle.config(text=f"目标: {a:.3f} rad")

    def _on_open(self):
        self.controller.set_target(DEFAULT_OPEN_ANGLE)
        self.scale_angle.set(DEFAULT_OPEN_ANGLE)
        self.lbl_target_angle.config(text=f"目标: {DEFAULT_OPEN_ANGLE:.3f} rad")

    def _on_close(self):
        self.controller.set_target(DEFAULT_CLOSE_ANGLE)
        self.scale_angle.set(DEFAULT_CLOSE_ANGLE)
        self.lbl_target_angle.config(text=f"目标: {DEFAULT_CLOSE_ANGLE:.3f} rad")

    def _on_stop(self):
        self.controller.target_angle = self.controller.current_angle
        self.scale_angle.set(self.controller.current_angle)
        self.lbl_target_angle.config(text=f"目标: {self.controller.current_angle:.3f} rad (已锁定)")

    def _update_ui(self):
        ca = self.controller.current_angle
        self.lbl_current_angle.config(text=f"{ca:.3f} rad")

        if not ROS_AVAILABLE:
            self.status_dot.config(fg=self.danger)
            self.status_text.config(text="ROS 未安装", fg=self.danger)
        elif not self.controller.ros_connected:
            self.status_dot.config(fg=self.danger)
            self.status_text.config(text="ROS 未连接", fg=self.danger)
        else:
            self.status_dot.config(fg=self.success)
            self.status_text.config(text="ROS 已连接", fg=self.success)

        if self.controller.real_angle_right is not None:
            r = self.controller.real_angle_right
            l = self.controller.real_angle_left if self.controller.real_angle_left is not None else r
            self.lbl_real_angles.config(text=f"实际角度: 右={r:.3f} 左={l:.3f} rad")
        else:
            self.lbl_real_angles.config(text="实际角度: -- (未收到 arm_status)")

        self.root.after(100, self._update_ui)

    def on_closing(self):
        self.controller.stop()
        self.root.destroy()


def main():
    if not ROS_AVAILABLE:
        print("[WARN] rospy 未安装。GUI 将以演示模式运行。")

    root = tk.Tk()
    app = DeformControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)

    root.update_idletasks()
    x = (root.winfo_screenwidth() // 2) - (420 // 2)
    y = (root.winfo_screenheight() // 2) - (600 // 2)
    root.geometry(f"+{x}+{y}")

    root.mainloop()


if __name__ == "__main__":
    main()
