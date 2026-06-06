#!/usr/bin/env python3
"""
huaqiccc_unified_control_gui.py
===============================
huaqiccc 机臂变形 + 动态飞行参数 统一控制 GUI

内置核心参数，无需外部 SDF 文件。参数可通过 GUI 文本区实时修改，
适配仿真或真实无人机。

用法:
    source /opt/ros/noetic/setup.bash
    python3 huaqiccc_unified_control_gui.py

工作流程:
    1. 用户点击"一键展开/收拢" → GUI 内部 20Hz 平滑插值
    2. 插值过程中，每隔 0.5s / 角度变化 0.05rad:
       a. 用当前插值角度 + 内置/用户修改的参数
       b. 重新计算整机质心和四个电机相对质心的位置
       c. 通过 MAVROS 31000 命令发送给 PX4
    3. 到达目标角度后，最终确认发送一次，停止更新
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import json
import math
import os
import sys

try:
    import rospy
    from std_msgs.msg import Float64
    from mavros_msgs.srv import CommandLong
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

# ===================== 默认参数（从 SDF 提取合并）====================
# 三部分: Base(机身+IMU), Left(左臂+两电机), Right(右臂+两电机)
DEFAULT_PARAMS_TEXT = """# === 部件质量 (kg) ===
base_mass=0.2204
left_mass=0.5195
right_mass=0.5208

# === 部件质心 (相对各自坐标系原点, m) ===
base_com=0.0313,0.0000,0.0048
left_com=0.1296,0.0997,-0.0190
right_com=0.1272,-0.1015,-0.0187

# === 关节位置 (相对base坐标系, m) ===
left_joint=0.036,0.023,0.0
right_joint=0.036,-0.023,0.0

# === 旋转轴 ===
left_axis=0,0,-1
right_axis=0,0,1

# === 电机推力位置 (相对各自机臂坐标系, m) ===
lb_motor=-0.0826,0.2250,0.0237
lf_motor=0.3765,0.2052,0.0237
rb_motor=-0.0872,-0.2234,0.0237
rf_motor=0.3724,-0.2109,0.0237"""

# ===================== 运动学计算引擎（纯 Python）====================

def parse_vec(s):
    """解析 '0.1,0.2,0.3' → [0.1, 0.2, 0.3]"""
    return [float(x.strip()) for x in s.split(',')]

def parse_params(text):
    """从 key=value 文本解析参数字典"""
    params = {}
    for line in text.split('\n'):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        if '=' not in line:
            continue
        key, val = line.split('=', 1)
        key = key.strip()
        val = val.strip()
        if ',' in val:
            params[key] = parse_vec(val)
        else:
            try:
                params[key] = float(val)
            except ValueError:
                params[key] = val
    return params

def vec_add(a, b):
    return [a[i] + b[i] for i in range(3)]

def vec_scale(a, s):
    return [a[i] * s for i in range(3)]

def vec_dot(a, b):
    return sum(a[i] * b[i] for i in range(3))

def vec_cross(a, b):
    return [
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    ]

def vec_norm(v):
    return math.sqrt(sum(x*x for x in v))

def rotate_vector(v, axis, angle):
    """罗德里格斯公式"""
    ax, ay, az = axis
    norm = math.sqrt(ax*ax + ay*ay + az*az)
    if norm < 1e-9:
        return v[:]
    ax, ay, az = ax/norm, ay/norm, az/norm
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    dot = ax*v[0] + ay*v[1] + az*v[2]
    cx = ay*v[2] - az*v[1]
    cy = az*v[0] - ax*v[2]
    cz = ax*v[1] - ay*v[0]
    return [
        v[0]*cos_a + cx*sin_a + ax*dot*(1-cos_a),
        v[1]*cos_a + cy*sin_a + ay*dot*(1-cos_a),
        v[2]*cos_a + cz*sin_a + az*dot*(1-cos_a)
    ]

def compute_rotor_params(params, arm_angle=0.0):
    """
    根据参数和机臂角度，计算 PX4 CA_ROTOR 参数
    返回: ({0:{'PX':x,'PY':y,'PZ':z},...}, total_mass, com_xyz)
    """
    # 读取参数
    base_mass = params.get('base_mass', 0.22)
    left_mass = params.get('left_mass', 0.52)
    right_mass = params.get('right_mass', 0.52)
    base_com = params.get('base_com', [0.0313, 0.0, 0.0048])
    left_com = params.get('left_com', [0.1296, 0.0997, -0.0190])
    right_com = params.get('right_com', [0.1272, -0.1015, -0.0187])
    left_joint = params.get('left_joint', [0.036, 0.023, 0.0])
    right_joint = params.get('right_joint', [0.036, -0.023, 0.0])
    left_axis = params.get('left_axis', [0, 0, -1])
    right_axis = params.get('right_axis', [0, 0, 1])
    lb_motor = params.get('lb_motor', [-0.0826, 0.2250, 0.0237])
    lf_motor = params.get('lf_motor', [0.3765, 0.2052, 0.0237])
    rb_motor = params.get('rb_motor', [-0.0872, -0.2234, 0.0237])
    rf_motor = params.get('rf_motor', [0.3724, -0.2109, 0.0237])

    base = [0.0, 0.0, 0.0]

    # 左臂
    laf = vec_add(base, left_joint)
    lac = vec_add(laf, rotate_vector(left_com, left_axis, arm_angle))
    lb_c = vec_add(laf, rotate_vector(lb_motor, left_axis, arm_angle))
    lf_c = vec_add(laf, rotate_vector(lf_motor, left_axis, arm_angle))

    # 右臂
    raf = vec_add(base, right_joint)
    rac = vec_add(raf, rotate_vector(right_com, right_axis, arm_angle))
    rb_c = vec_add(raf, rotate_vector(rb_motor, right_axis, arm_angle))
    rf_c = vec_add(raf, rotate_vector(rf_motor, right_axis, arm_angle))

    # 整机质心
    total_mass = base_mass + left_mass + right_mass
    com = vec_scale(vec_add(
        vec_add(vec_scale(base_com, base_mass), vec_scale(lac, left_mass)),
        vec_scale(rac, right_mass)
    ), 1.0 / total_mass)

    # 电机相对质心
    motors = {
        0: vec_add(lb_c, vec_scale(com, -1)),
        1: vec_add(lf_c, vec_scale(com, -1)),
        2: vec_add(rb_c, vec_scale(com, -1)),
        3: vec_add(rf_c, vec_scale(com, -1)),
    }

    px4_params = {}
    for i in range(4):
        pos = motors[i]
        px4_params[i] = {
            'PX': round(pos[0], 5),
            'PY': round(-pos[1], 5),   # SDF Y+左 → PX4 Y+右
            'PZ': round(-pos[2], 5),   # SDF Z+上 → PX4 Z+下
        }
    return px4_params, total_mass, com


# ===================== 统一控制器 =====================

class UnifiedController:
    def __init__(self):
        self.current_angle = 0.0
        self.target_angle = 0.0
        self.speed = 0.3
        self.running = False
        self.params = parse_params(DEFAULT_PARAMS_TEXT)

        self.ros_connected = False
        self.angle_pub = None
        self.cmd_long_srv = None

        if not ROS_AVAILABLE:
            print("[WARN] ROS 未安装，离线模式运行")
        else:
            try:
                if not rospy.core.is_initialized():
                    rospy.init_node('huaqiccc_unified_gui', anonymous=True)
                self.ros_connected = True
                self.angle_pub = rospy.Publisher('/huaqiccc/arm_angle', Float64, queue_size=1)
                try:
                    rospy.wait_for_service('/mavros/cmd/command', timeout=5.0)
                    self.cmd_long_srv = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
                    print("[OK] MAVROS /mavros/cmd/command 已连接")
                except Exception as e:
                    print(f"[WARN] MAVROS cmd/command: {e}")
            except Exception as e:
                print(f"[WARN] ROS init: {e}")

        self.last_param_angle = None
        self.last_param_time = 0.0
        self.param_update_count = 0

    def update_params(self, text):
        """从 GUI 文本区更新参数"""
        try:
            self.params = parse_params(text)
            print("[OK] 参数已更新")
            return True
        except Exception as e:
            print(f"[ERROR] 参数解析失败: {e}")
            return False

    def _send_rotor_31000(self, idx, px, py, pz):
        if not self.cmd_long_srv:
            return True
        try:
            from mavros_msgs.srv import CommandLongRequest
            req = CommandLongRequest()
            req.broadcast = False
            req.command = 31000
            req.confirmation = 0
            req.param1 = float(px)
            req.param2 = float(py)
            req.param3 = float(pz)
            req.param4 = 0.0
            req.param5 = 0.0
            req.param6 = -1.0
            req.param7 = float(idx)
            resp = self.cmd_long_srv(req)
            return getattr(resp, 'success', False)
        except Exception as e:
            print(f"[ERROR] 31000 motor{idx}: {e}")
            return False

    def _send_all_rotors(self, params, label=""):
        ok_all = True
        for i in range(4):
            p = params[i]
            ok = self._send_rotor_31000(i, p['PX'], p['PY'], p['PZ'])
            ok_all &= ok
            if i < 3:
                time.sleep(0.3)
        status = "OK" if ok_all else "PARTIAL"
        print(f"  [{status}] {label} 31000 已发送")
        return ok_all

    def _update_px4_params(self, arm_angle, force=False):
        now = time.time()
        if not force and (now - self.last_param_time) < 0.5:
            return False
        if not force and self.last_param_angle is not None:
            delta = abs(arm_angle - self.last_param_angle)
            if delta < 0.05:
                return False

        params, mass, com = compute_rotor_params(self.params, arm_angle)
        self.last_param_angle = arm_angle
        self.last_param_time = now
        self.param_update_count += 1

        label = f"step{self.param_update_count} angle={arm_angle:.3f}"
        print(f"\n[PARAM] {label} COM=({com[0]:.3f},{com[1]:.3f},{com[2]:.3f})")
        self._send_all_rotors(params, label)
        return True

    def set_target(self, angle):
        self.target_angle = max(-0.5, min(0.0, float(angle)))

    def set_speed(self, speed):
        self.speed = max(0.05, min(1.0, float(speed)))

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False

    def _loop(self):
        DT = 0.05
        is_moving = False
        while self.running:
            diff = self.target_angle - self.current_angle
            step = self.speed * DT

            if abs(diff) > 1e-6:
                is_moving = True
                if abs(diff) <= step:
                    self.current_angle = self.target_angle
                else:
                    self.current_angle += math.copysign(step, diff)

                if self.angle_pub and self.ros_connected:
                    try:
                        msg = Float64()
                        msg.data = self.current_angle
                        self.angle_pub.publish(msg)
                    except Exception:
                        pass

                self._update_px4_params(self.current_angle, force=False)
            else:
                if is_moving:
                    is_moving = False
                    self._update_px4_params(self.current_angle, force=True)
                    print(f"[DONE] 到达 {self.current_angle:.3f}rad，共 {self.param_update_count} 次")

            time.sleep(DT)

    def init_params(self, arm_angle=0.0):
        self._update_px4_params(arm_angle, force=True)
        print(f"[INIT] 初始参数已发送，角度={arm_angle:.3f}rad")


# ===================== GUI =====================

FONT_UI = "DejaVu Sans"
FONT_UI_BOLD = ("DejaVu Sans", 10, "bold")
FONT_MONO = ("DejaVu Sans Mono", 20, "bold")
FONT_BTN = ("DejaVu Sans", 12, "bold")
FONT_SMALL = ("DejaVu Sans", 9)

class UnifiedControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("huaqiccc 变形 + 飞控参数 统一控制")
        self.root.geometry("520x720")
        self.root.resizable(True, True)
        self.root.minsize(480, 600)
        self.root.configure(bg="#1e1e2e")

        self.bg_main = "#1e1e2e"
        self.bg_card = "#2d2d44"
        self.fg_main = "#cdd6f4"
        self.fg_dim = "#6c7086"
        self.accent = "#89b4fa"
        self.success = "#a6e3a1"
        self.danger = "#f38ba8"
        self.warning = "#f9e2af"

        self.controller = UnifiedController()
        self.controller.start()
        self.controller.init_params(0.0)

        self._build_ui()
        self._update_ui()

    def _build_ui(self):
        # ===== 状态栏 =====
        st = tk.Frame(self.root, bg=self.bg_main, padx=16, pady=8)
        st.pack(fill="x")
        self.status_dot = tk.Label(st, text="●", font=(FONT_UI, 14), bg=self.bg_main)
        self.status_dot.pack(side="left")
        self.status_text = tk.Label(st, text="初始化中...", font=(FONT_UI, 11),
                                     bg=self.bg_main, fg=self.fg_main)
        self.status_text.pack(side="left", padx=(4, 0))
        self.lbl_param_count = tk.Label(st, text="更新: 0", font=FONT_SMALL,
                                         bg=self.bg_main, fg=self.fg_dim)
        self.lbl_param_count.pack(side="right")

        # ===== 角度显示 =====
        card = tk.Frame(self.root, bg=self.bg_card, padx=20, pady=12)
        card.pack(fill="x", padx=16, pady=6)
        tk.Label(card, text="当前目标角度", font=(FONT_UI, 10), bg=self.bg_card, fg=self.fg_dim).pack(anchor="w")
        self.lbl_current_angle = tk.Label(card, text="0.000 rad", font=FONT_MONO,
                                           bg=self.bg_card, fg=self.accent)
        self.lbl_current_angle.pack(anchor="w", pady=(2, 4))
        self.lbl_target_angle = tk.Label(card, text="目标: 0.000 rad", font=(FONT_UI, 10),
                                          bg=self.bg_card, fg=self.fg_dim)
        self.lbl_target_angle.pack(anchor="w")

        # ===== 速度控制 =====
        cspd = tk.Frame(self.root, bg=self.bg_card, padx=20, pady=12)
        cspd.pack(fill="x", padx=16, pady=6)
        tk.Label(cspd, text="变形速度", font=FONT_UI_BOLD, bg=self.bg_card, fg=self.fg_main).pack(anchor="w")
        row = tk.Frame(cspd, bg=self.bg_card)
        row.pack(fill="x", pady=(6, 0))
        tk.Label(row, text="慢", font=FONT_SMALL, bg=self.bg_card, fg=self.fg_dim).pack(side="left")
        self.scale_speed = tk.Scale(row, from_=0.05, to=1.0, resolution=0.01,
                                     orient="horizontal", length=300, sliderlength=16,
                                     bg=self.bg_card, highlightthickness=0,
                                     troughcolor="#3d3d5c", activebackground=self.accent,
                                     command=self._on_speed)
        self.scale_speed.set(0.3)
        self.scale_speed.pack(side="left", padx=6, expand=True, fill="x")
        tk.Label(row, text="快", font=FONT_SMALL, bg=self.bg_card, fg=self.fg_dim).pack(side="left")
        self.lbl_speed = tk.Label(cspd, text="0.30 rad/s", font=(FONT_UI, 11),
                                   bg=self.bg_card, fg=self.accent)
        self.lbl_speed.pack(anchor="e", pady=(4, 0))

        # ===== 角度微调 =====
        cang = tk.Frame(self.root, bg=self.bg_card, padx=20, pady=12)
        cang.pack(fill="x", padx=16, pady=6)
        tk.Label(cang, text="目标角度微调", font=FONT_UI_BOLD, bg=self.bg_card, fg=self.fg_main).pack(anchor="w")
        row2 = tk.Frame(cang, bg=self.bg_card)
        row2.pack(fill="x", pady=(6, 0))
        tk.Label(row2, text="收拢", font=FONT_SMALL, bg=self.bg_card, fg=self.fg_dim).pack(side="left")
        self.scale_angle = tk.Scale(row2, from_=-0.5, to=0.0, resolution=0.01,
                                     orient="horizontal", length=300, sliderlength=16,
                                     bg=self.bg_card, highlightthickness=0,
                                     troughcolor="#3d3d5c", activebackground=self.accent,
                                     command=self._on_angle)
        self.scale_angle.set(-0.3)
        self.scale_angle.pack(side="left", padx=6, expand=True, fill="x")
        tk.Label(row2, text="展开", font=FONT_SMALL, bg=self.bg_card, fg=self.fg_dim).pack(side="left")

        # ===== 主控制按钮 =====
        btn = tk.Frame(self.root, bg=self.bg_main, padx=16, pady=12)
        btn.pack(fill="x", pady=(4, 0))
        self.btn_open = tk.Button(btn, text="一键展开", font=FONT_BTN,
                                   bg=self.success, fg="#1e1e2e", activebackground="#81c8be",
                                   bd=0, padx=16, pady=14, cursor="hand2",
                                   command=self._on_open)
        self.btn_open.pack(side="left", expand=True, fill="x", padx=(0, 6))
        self.btn_close = tk.Button(btn, text="一键收拢", font=FONT_BTN,
                                    bg=self.danger, fg="#1e1e2e", activebackground="#eba0ac",
                                    bd=0, padx=16, pady=14, cursor="hand2",
                                    command=self._on_close)
        self.btn_close.pack(side="left", expand=True, fill="x", padx=(6, 0))

        stop = tk.Frame(self.root, bg=self.bg_main, padx=16, pady=6)
        stop.pack(fill="x")
        self.btn_stop = tk.Button(stop, text="紧急停止", font=(FONT_UI, 11),
                                   bg="#313244", fg=self.danger, activebackground="#45475a",
                                   bd=0, padx=16, pady=10, cursor="hand2",
                                   command=self._on_stop)
        self.btn_stop.pack(fill="x")

        # ===== 参数配置区域（可折叠）=====
        self.param_collapsed = True
        self.param_frame = tk.Frame(self.root, bg=self.bg_main, padx=16, pady=4)
        self.param_frame.pack(fill="x")

        self.param_header = tk.Frame(self.param_frame, bg=self.bg_main, cursor="hand2")
        self.param_header.pack(fill="x")
        self.param_header.bind("<Button-1>", self._toggle_params)
        self.param_arrow = tk.Label(self.param_header, text="▶", font=(FONT_UI, 12),
                                     bg=self.bg_main, fg=self.accent)
        self.param_arrow.pack(side="left")
        tk.Label(self.param_header, text="核心参数配置（点击展开）", font=(FONT_UI, 11, "bold"),
                 bg=self.bg_main, fg=self.accent, cursor="hand2").pack(side="left", padx=(4, 0))
        self.param_header.bind("<Enter>", lambda e: self.param_header.config(bg="#252538"))
        self.param_header.bind("<Leave>", lambda e: self.param_header.config(bg=self.bg_main))

        # 参数内容（初始隐藏）
        self.param_content = tk.Frame(self.param_frame, bg=self.bg_card, padx=10, pady=10)
        # 不 pack，点击后展开

        tk.Label(self.param_content, text="参数格式: key=value，向量用逗号分隔",
                 font=FONT_SMALL, bg=self.bg_card, fg=self.fg_dim).pack(anchor="w")
        tk.Label(self.param_content, text="修改后点击 [应用参数]",
                 font=FONT_SMALL, bg=self.bg_card, fg=self.fg_dim).pack(anchor="w")

        self.text_params = tk.Text(self.param_content, height=16, width=55,
                                    font=("DejaVu Sans Mono", 10),
                                    bg="#1e1e2e", fg=self.fg_main,
                                    insertbackground=self.accent,
                                    relief="flat", padx=8, pady=8)
        self.text_params.pack(fill="both", expand=True, pady=(6, 6))
        self.text_params.insert("1.0", DEFAULT_PARAMS_TEXT)

        btn_row = tk.Frame(self.param_content, bg=self.bg_card)
        btn_row.pack(fill="x")
        tk.Button(btn_row, text="应用参数", font=FONT_UI_BOLD,
                  bg=self.accent, fg="#1e1e2e", activebackground=self.accent,
                  bd=0, padx=16, pady=6, cursor="hand2",
                  command=self._apply_params).pack(side="left", padx=(0, 8))
        tk.Button(btn_row, text="恢复默认", font=FONT_UI_BOLD,
                  bg="#313244", fg=self.fg_dim, activebackground="#45475a",
                  bd=0, padx=16, pady=6, cursor="hand2",
                  command=self._reset_params).pack(side="left")

        # ===== 底部提示 =====
        tk.Label(self.root, text="提示: 先启动仿真，再打开本 GUI",
                 font=FONT_SMALL, bg=self.bg_main, fg=self.fg_dim).pack(pady=(8, 12))

    def _toggle_params(self, event=None):
        if self.param_collapsed:
            self.param_content.pack(fill="both", expand=True, pady=(8, 0))
            self.param_arrow.config(text="▼")
            self.param_collapsed = False
        else:
            self.param_content.pack_forget()
            self.param_arrow.config(text="▶")
            self.param_collapsed = True

    def _apply_params(self):
        text = self.text_params.get("1.0", "end")
        ok = self.controller.update_params(text)
        if ok:
            # 立即用新参数计算一次
            self.controller._update_px4_params(self.controller.current_angle, force=True)
            messagebox.showinfo("参数更新", "参数已应用并重新计算")
        else:
            messagebox.showerror("参数错误", "解析失败，请检查格式")

    def _reset_params(self):
        self.text_params.delete("1.0", "end")
        self.text_params.insert("1.0", DEFAULT_PARAMS_TEXT)

    def _on_speed(self, val):
        self.controller.set_speed(float(val))
        self.lbl_speed.config(text=f"{float(val):.2f} rad/s")

    def _on_angle(self, val):
        a = float(val)
        self.controller.set_target(a)
        self.lbl_target_angle.config(text=f"目标: {a:.3f} rad")

    def _on_open(self):
        self.controller.set_target(-0.3)
        self.scale_angle.set(-0.3)
        self.lbl_target_angle.config(text="目标: -0.300 rad")

    def _on_close(self):
        self.controller.set_target(0.0)
        self.scale_angle.set(0.0)
        self.lbl_target_angle.config(text="目标: 0.000 rad")

    def _on_stop(self):
        self.controller.target_angle = self.controller.current_angle
        self.scale_angle.set(self.controller.current_angle)
        self.lbl_target_angle.config(text=f"目标: {self.controller.current_angle:.3f} rad (已锁定)")

    def _update_ui(self):
        ca = self.controller.current_angle
        self.lbl_current_angle.config(text=f"{ca:.3f} rad")
        self.lbl_param_count.config(text=f"更新: {self.controller.param_update_count}")

        if not ROS_AVAILABLE:
            self.status_dot.config(fg=self.danger)
            self.status_text.config(text="ROS 未安装", fg=self.danger)
        elif not self.controller.ros_connected:
            self.status_dot.config(fg=self.danger)
            self.status_text.config(text="ROS 未连接", fg=self.danger)
        else:
            self.status_dot.config(fg=self.success)
            self.status_text.config(text="ROS 已连接", fg=self.success)

        self.root.after(100, self._update_ui)

    def on_closing(self):
        self.controller.stop()
        self.root.destroy()


def main():
    if not ROS_AVAILABLE:
        print("[WARN] rospy 未安装，离线模式运行。")

    root = tk.Tk()
    app = UnifiedControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)

    root.update_idletasks()
    x = (root.winfo_screenwidth() // 2) - (520 // 2)
    y = (root.winfo_screenheight() // 2) - (720 // 2)
    root.geometry(f"+{x}+{y}")

    root.mainloop()


if __name__ == '__main__':
    main()
