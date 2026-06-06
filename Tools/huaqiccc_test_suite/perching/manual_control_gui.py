#!/usr/bin/env python3
"""
Manual Control GUI for huaqiccc perching drone.
All blocking operations run in background threads so the GUI never freezes.
"""

import math
import os
import signal
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Float64, Bool
from tf.transformations import euler_from_quaternion


class DroneGUI:
    def __init__(self, master):
        self.master = master
        master.title("huaqiccc Manual Control")
        master.geometry("640x820")
        master.resizable(True, True)

        # ---- state ----
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = 0.0
        self._target_lock = threading.Lock()

        self.current_state = None
        self.current_pose = None
        self.current_yaw = 0.0

        self.sp_pub = None
        self.arm_pub = None
        self.arming_client = None
        self.set_mode_client = None
        self.fix_pub = None
        self.perching_status = False

        self._stop_sender = threading.Event()
        self._control_thread = None
        self._busy = False

        self._build_gui()
        self.master.after(200, self._auto_start_sitl)

    def _build_gui(self):
        pad = {"pady": 4, "padx": 10}

        # Status
        status_frame = tk.Frame(self.master)
        status_frame.pack(fill=tk.X, **pad)
        tk.Label(status_frame, text="Status:").pack(side=tk.LEFT)
        self.status_label = tk.Label(status_frame, text="Idle", fg="gray",
                                     width=50, anchor=tk.W)
        self.status_label.pack(side=tk.LEFT, padx=5)

        # Top buttons
        top_frame = tk.Frame(self.master)
        top_frame.pack(fill=tk.X, **pad)
        self.btn_start = tk.Button(top_frame, text="启动仿真", width=12,
                                   command=self._start_sitl)
        self.btn_start.pack(side=tk.LEFT, padx=4)
        self.btn_takeoff = tk.Button(top_frame, text="起飞→2.5m", width=12,
                                     command=self._on_takeoff, state=tk.DISABLED)
        self.btn_takeoff.pack(side=tk.LEFT, padx=4)
        self.btn_land = tk.Button(top_frame, text="降落/Disarm", width=12,
                                  command=self._on_land, state=tk.DISABLED)
        self.btn_land.pack(side=tk.LEFT, padx=4)
        self.btn_kill = tk.Button(top_frame, text="强制清进程", width=12,
                                  command=self._force_kill, bg="#ffcccc")
        self.btn_kill.pack(side=tk.LEFT, padx=4)
        self.btn_fix = tk.Button(top_frame, text="固定到柱子", width=12,
                                 command=self._send_fix_toggle, bg="#ccffcc", state=tk.DISABLED)
        self.btn_fix.pack(side=tk.LEFT, padx=4)

        # Step sizes
        step_frame = tk.LabelFrame(self.master, text="步进幅度")
        step_frame.pack(fill=tk.X, **pad)
        tk.Label(step_frame, text="平移(m):").grid(row=0, column=0, padx=3)
        self.trans_entry = tk.Entry(step_frame, width=8)
        self.trans_entry.insert(0, "0.05")
        self.trans_entry.grid(row=0, column=1, padx=3)
        tk.Label(step_frame, text="旋转(°):").grid(row=0, column=2, padx=3)
        self.rot_entry = tk.Entry(step_frame, width=8)
        self.rot_entry.insert(0, "3.0")
        self.rot_entry.grid(row=0, column=3, padx=3)
        tk.Label(step_frame, text="高度(m):").grid(row=0, column=4, padx=3)
        self.z_entry = tk.Entry(step_frame, width=8)
        self.z_entry.insert(0, "2.5")
        self.z_entry.grid(row=0, column=5, padx=3)

        # Translation
        trans_frame = tk.LabelFrame(self.master, text="平移控制 (NED x=前  y=右)")
        trans_frame.pack(fill=tk.X, **pad)
        self.btn_fwd = tk.Button(trans_frame, text="↑ 前(x+)", width=12,
                                 command=lambda: self._move(1, 0), state=tk.DISABLED)
        self.btn_fwd.grid(row=0, column=1, pady=3)
        self.btn_left = tk.Button(trans_frame, text="← 左(y+)", width=12,
                                  command=lambda: self._move(0, 1), state=tk.DISABLED)
        self.btn_left.grid(row=1, column=0, padx=8, pady=3)
        self.btn_right = tk.Button(trans_frame, text="右(y-) →", width=12,
                                   command=lambda: self._move(0, -1), state=tk.DISABLED)
        self.btn_right.grid(row=1, column=2, padx=8, pady=3)
        self.btn_back = tk.Button(trans_frame, text="↓ 后(x-)", width=12,
                                  command=lambda: self._move(-1, 0), state=tk.DISABLED)
        self.btn_back.grid(row=2, column=1, pady=3)

        # Rotation & Height
        rot_frame = tk.LabelFrame(self.master, text="旋转 / 高度")
        rot_frame.pack(fill=tk.X, **pad)
        self.btn_rot_left = tk.Button(rot_frame, text="↺ 左转", width=12,
                                      command=lambda: self._rotate(1), state=tk.DISABLED)
        self.btn_rot_left.pack(side=tk.LEFT, padx=10, pady=4)
        self.btn_rot_right = tk.Button(rot_frame, text="右转 ↻", width=12,
                                       command=lambda: self._rotate(-1), state=tk.DISABLED)
        self.btn_rot_right.pack(side=tk.LEFT, padx=10, pady=4)
        self.btn_up = tk.Button(rot_frame, text="上升 +0.2m", width=12,
                                command=lambda: self._change_z(0.2), state=tk.DISABLED)
        self.btn_up.pack(side=tk.LEFT, padx=5, pady=4)
        self.btn_down = tk.Button(rot_frame, text="下降 -0.2m", width=12,
                                  command=lambda: self._change_z(-0.2), state=tk.DISABLED)
        self.btn_down.pack(side=tk.LEFT, padx=5, pady=4)

        # Arm control
        arm_frame = tk.LabelFrame(self.master, text="机臂开合 (弧度)")
        arm_frame.pack(fill=tk.X, **pad)
        tk.Label(arm_frame, text="角度:").pack(side=tk.LEFT, padx=4)
        self.arm_entry = tk.Entry(arm_frame, width=10)
        self.arm_entry.insert(0, "-0.15")
        self.arm_entry.pack(side=tk.LEFT, padx=4)
        self.btn_arm_go = tk.Button(arm_frame, text="发送", width=8,
                                    command=self._send_arm, state=tk.DISABLED)
        self.btn_arm_go.pack(side=tk.LEFT, padx=4)
        preset_frame = tk.Frame(self.master)
        preset_frame.pack(fill=tk.X, **pad)
        for val, label in [(-0.45, "展开-0.45"), (-0.30, "半开-0.30"),
                           (-0.15, "轻收-0.15"), (0.0, "收拢0.0")]:
            tk.Button(preset_frame, text=label, width=12,
                      command=lambda v=val: self._preset_arm(v)).pack(side=tk.LEFT, padx=3)

        # Preset positions
        preset2 = tk.LabelFrame(self.master, text="预设位置")
        preset2.pack(fill=tk.X, **pad)
        self.btn_goto_origin = tk.Button(preset2, text="原点(0,0,2.5)", width=15,
                                         command=self._goto_origin, state=tk.DISABLED)
        self.btn_goto_origin.pack(side=tk.LEFT, padx=6, pady=3)
        self.btn_goto_pole = tk.Button(preset2, text="杆前(4.5,0,2.5)", width=15,
                                       command=self._goto_pole_front, state=tk.DISABLED)
        self.btn_goto_pole.pack(side=tk.LEFT, padx=6, pady=3)
        self.btn_goto_surface = tk.Button(preset2, text="贴杆(4.91,0,2.5)", width=15,
                                          command=self._goto_pole_surface, state=tk.DISABLED)
        self.btn_goto_surface.pack(side=tk.LEFT, padx=6, pady=3)

        # Pose display
        pose_frame = tk.LabelFrame(self.master, text="当前位姿 / 目标位姿")
        pose_frame.pack(fill=tk.X, **pad)
        self.pose_label = tk.Label(
            pose_frame,
            text="act: x=0.00 y=0.00 z=0.00 yaw=0.0°\ntgt: x=0.00 y=0.00 z=0.00 yaw=0.0°",
            font=("Courier", 10), justify=tk.LEFT)
        self.pose_label.pack(pady=4)

        # Log
        log_frame = tk.LabelFrame(self.master, text="日志")
        log_frame.pack(fill=tk.BOTH, expand=True, **pad)
        scrollbar = tk.Scrollbar(log_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text = tk.Text(log_frame, height=10, state=tk.DISABLED,
                                font=("Courier", 9), yscrollcommand=scrollbar.set)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        scrollbar.config(command=self.log_text.yview)

    # ---- helpers ----
    def _log(self, msg):
        t = time.strftime("%H:%M:%S", time.localtime())
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{t}] {msg}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    def _set_status(self, text, color="black"):
        self.status_label.config(text=text, fg=color)
        self.master.update_idletasks()

    def _gui_safe(self, fn):
        """Run fn on the main GUI thread."""
        self.master.after(0, fn)

    # ---- SITL / ROS ----
    def _auto_start_sitl(self):
        self._start_sitl()

    def _start_sitl(self):
        self._log("[INFO] Starting SITL...")
        self._set_status("SITL starting...", "blue")
        self.btn_start.config(state=tk.DISABLED)
        cmd = (
            "cd /home/a/catkin_ws && source devel/setup.bash && "
            "roslaunch px4 mavros_posix_sitl_perching_16cm.launch "
            "fcu_url:=udp://:14540@localhost:14580"
        )
        subprocess.Popen(cmd, shell=True, executable="/bin/bash",
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True)
        self.master.after(18000, self._init_ros)

    def _force_kill(self):
        self._log("[KILL] pkill -9 px4/gzserver/gzclient/mavros/roslaunch")
        subprocess.run("pkill -9 -f 'px4|gzserver|gzclient|mavros|roslaunch'",
                       shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._set_status("All processes killed", "red")

    def _init_ros(self):
        try:
            rospy.init_node('huaqiccc_manual_gui', anonymous=True)
        except rospy.exceptions.ROSException:
            pass

        self.sp_pub = rospy.Publisher('/mavros/setpoint_raw/local',
                                      PositionTarget, queue_size=10)
        self.arm_pub = rospy.Publisher('/huaqiccc/arm_angle',
                                       Float64, queue_size=1)
        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/mavros/local_position/pose',
                         PoseStamped, self._pose_cb)

        try:
            rospy.wait_for_service('/mavros/cmd/arming', timeout=25.0)
            rospy.wait_for_service('/mavros/set_mode', timeout=25.0)
            self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming',
                                                     CommandBool)
            self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode',
                                                       SetMode)
        except Exception as e:
            self._log(f"[ERROR] MAVROS services: {e}")
            return

        self._log("[OK] MAVROS ready")
        self._set_status("Waiting for FCU...", "blue")
        self.master.after(500, self._wait_fcu)

    def _wait_fcu(self):
        if rospy.is_shutdown():
            return
        if self.current_state and self.current_state.connected:
            self._log("[OK] FCU connected")
            self._set_status("Ready", "green")
            self.btn_takeoff.config(state=tk.NORMAL)
            self.btn_land.config(state=tk.NORMAL)
            return
        self.master.after(500, self._wait_fcu)

    def _state_cb(self, msg):
        self.current_state = msg

    def _pose_cb(self, msg):
        self.current_pose = msg.pose
        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw
        with self._target_lock:
            fix_str = " [FIXED]" if self.perching_status else ""
            self.pose_label.config(
                text=(
                    f"act: x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f} "
                    f"z={msg.pose.position.z:.2f} yaw={math.degrees(yaw):.1f}°{fix_str}\n"
                    f"tgt: x={self.target_x:.2f} y={self.target_y:.2f} "
                    f"z={self.target_z:.2f} yaw={math.degrees(self.target_yaw):.1f}°"
                )
            )

    def _perching_status_cb(self, msg):
        self.perching_status = msg.data
        if msg.data:
            self._gui_safe(lambda: self._set_status("PERCHING FIXED", "green"))
            self._gui_safe(lambda: self.btn_fix.config(text="解除固定", bg="#ffcccc"))
        else:
            self._gui_safe(lambda: self._set_status("Fixed released", "blue"))
            self._gui_safe(lambda: self.btn_fix.config(text="固定到柱子", bg="#ccffcc"))

    # ---- background setpoint thread ----
    def _ensure_control_thread(self):
        if self._control_thread is None or not self._control_thread.is_alive():
            self._stop_sender.clear()
            self._control_thread = threading.Thread(
                target=self._control_loop, daemon=True)
            self._control_thread.start()

    def _control_loop(self):
        dt = 0.05
        while not self._stop_sender.is_set() and not rospy.is_shutdown():
            with self._target_lock:
                tx, ty, tz, tyaw = (self.target_x, self.target_y,
                                    self.target_z, self.target_yaw)
            msg = PositionTarget()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "map"
            msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            msg.type_mask = (
                PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                PositionTarget.IGNORE_VZ |
                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY |
                PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE
            )
            msg.position.x = tx
            msg.position.y = ty
            msg.position.z = tz
            msg.yaw = tyaw
            self.sp_pub.publish(msg)
            time.sleep(dt)

    # ---- actions (all run in background threads) ----
    def _on_takeoff(self):
        if self._busy:
            return
        self._busy = True
        self._set_status("Takeoff in progress...", "blue")
        self.btn_takeoff.config(state=tk.DISABLED)
        threading.Thread(target=self._takeoff_worker, daemon=True).start()

    def _takeoff_worker(self):
        try:
            z_target = float(self.z_entry.get())
        except ValueError:
            z_target = 2.5

        # burst pre-arm setpoints at current pose
        if self.current_pose:
            with self._target_lock:
                self.target_x = self.current_pose.position.x
                self.target_y = self.current_pose.position.y
                self.target_z = self.current_pose.position.z
                self.target_yaw = self.current_yaw

        for i in range(40):
            msg = PositionTarget()
            msg.header.stamp = rospy.Time.now()
            msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            msg.type_mask = (
                PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                PositionTarget.IGNORE_VZ |
                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY |
                PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE
            )
            with self._target_lock:
                msg.position.x = self.target_x
                msg.position.y = self.target_y
                msg.position.z = self.target_z
                msg.yaw = self.target_yaw
            self.sp_pub.publish(msg)
            time.sleep(0.05)

        # set offboard
        try:
            self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            time.sleep(0.5)
        except Exception as e:
            self._gui_safe(lambda: self._log(f"[ERROR] OFFBOARD: {e}"))
            self._gui_safe(lambda: self._set_status("OFFBOARD failed", "red"))
            self._gui_safe(lambda: self.btn_takeoff.config(state=tk.NORMAL))
            self._busy = False
            return

        # arm
        try:
            self.arming_client(True)
        except Exception as e:
            self._gui_safe(lambda: self._log(f"[ERROR] Arm: {e}"))
            self._gui_safe(lambda: self._set_status("Arm failed", "red"))
            self._gui_safe(lambda: self.btn_takeoff.config(state=tk.NORMAL))
            self._busy = False
            return

        # start background publisher
        self._ensure_control_thread()

        # set target height
        with self._target_lock:
            self.target_z = z_target
        self._gui_safe(lambda: self._log(f"[TAKEOFF] Target z={z_target}m"))

        # wait until close to target
        timeout = time.time() + 15.0
        while time.time() < timeout and not rospy.is_shutdown():
            if self.current_pose and abs(self.current_pose.position.z - z_target) < 0.3:
                self._gui_safe(lambda: self._log("[OK] Takeoff complete"))
                self._gui_safe(lambda: self._set_status("Manual control active", "green"))
                self._gui_safe(lambda: self._enable_controls(True))
                self._busy = False
                return
            time.sleep(0.3)

        self._gui_safe(lambda: self._log("[WARN] Takeoff timeout, controls enabled anyway"))
        self._gui_safe(lambda: self._set_status("Control active (timeout)", "orange"))
        self._gui_safe(lambda: self._enable_controls(True))
        self._busy = False

    def _on_land(self):
        if self._busy:
            return
        self._busy = True
        self._enable_controls(False)
        self.btn_takeoff.config(state=tk.NORMAL)
        threading.Thread(target=self._land_worker, daemon=True).start()

    def _land_worker(self):
        self._gui_safe(lambda: self._log("[LAND] Descending..."))
        with self._target_lock:
            self.target_z = 0.3
        time.sleep(4.0)
        try:
            self.arming_client(False)
            self._gui_safe(lambda: self._log("[OK] Disarmed"))
            self._gui_safe(lambda: self._set_status("Landed", "gray"))
        except Exception as e:
            self._gui_safe(lambda: self._log(f"[ERROR] Disarm: {e}"))
        self._busy = False

    def _enable_controls(self, en):
        state = tk.NORMAL if en else tk.DISABLED
        for b in (self.btn_fwd, self.btn_back, self.btn_left, self.btn_right,
                  self.btn_rot_left, self.btn_rot_right,
                  self.btn_up, self.btn_down,
                  self.btn_arm_go, self.btn_goto_origin,
                  self.btn_goto_pole, self.btn_goto_surface,
                  self.btn_fix):
            b.config(state=state)

    # ---- simple setpoint mutators (non-blocking) ----
    def _move(self, dx_ned, dy_ned):
        if self._busy:
            return
        try:
            step = float(self.trans_entry.get())
        except ValueError:
            messagebox.showwarning("Input error", "平移幅度必须是数字")
            return
        with self._target_lock:
            self.target_x += dx_ned * step
            self.target_y += dy_ned * step
        self._log(f"[MOVE] tgt x={self.target_x:.2f} y={self.target_y:.2f}")

    def _rotate(self, direction):
        if self._busy:
            return
        try:
            deg = float(self.rot_entry.get())
        except ValueError:
            messagebox.showwarning("Input error", "旋转幅度必须是数字")
            return
        rad = math.radians(deg)
        with self._target_lock:
            self.target_yaw += direction * rad
        self._log(f"[ROT] tgt yaw={math.degrees(self.target_yaw):.1f}°")

    def _change_z(self, dz):
        if self._busy:
            return
        with self._target_lock:
            self.target_z = max(0.2, self.target_z + dz)
        self._log(f"[HEIGHT] tgt z={self.target_z:.2f}")

    def _goto_origin(self):
        self._goto_preset(0.0, 0.0, 2.5)

    def _goto_pole_front(self):
        self._goto_preset(4.5, 0.0, 2.5)

    def _goto_pole_surface(self):
        self._goto_preset(4.91, 0.0, 2.5)

    def _goto_preset(self, x, y, z):
        if self._busy:
            return
        self._busy = True
        self._enable_controls(False)
        with self._target_lock:
            self.target_x, self.target_y, self.target_z = x, y, z
        self._log(f"[GOTO] {x:.2f},{y:.2f},{z:.2f}")
        self._set_status(f"Moving to ({x:.1f},{y:.1f},{z:.1f})...", "blue")

        def monitor():
            timeout = time.time() + 20.0
            while time.time() < timeout and not rospy.is_shutdown():
                if self.current_pose:
                    dx = abs(self.current_pose.position.x - x)
                    dy = abs(self.current_pose.position.y - y)
                    dz = abs(self.current_pose.position.z - z)
                    if dx < 0.15 and dy < 0.15 and dz < 0.15:
                        self._gui_safe(lambda: self._log("[OK] Arrived"))
                        self._gui_safe(lambda: self._set_status("Manual control active", "green"))
                        self._gui_safe(lambda: self._enable_controls(True))
                        self._busy = False
                        return
                time.sleep(0.3)
            self._gui_safe(lambda: self._log("[WARN] Preset timeout"))
            self._gui_safe(lambda: self._set_status("Control active (timeout)", "orange"))
            self._gui_safe(lambda: self._enable_controls(True))
            self._busy = False

        threading.Thread(target=monitor, daemon=True).start()

    def _preset_arm(self, val):
        self.arm_entry.delete(0, tk.END)
        self.arm_entry.insert(0, str(val))

    def _send_arm(self):
        if self._busy:
            return
        try:
            angle = float(self.arm_entry.get())
        except ValueError:
            messagebox.showwarning("Input error", "机臂角度必须是数字")
            return
        msg = Float64()
        msg.data = angle
        self.arm_pub.publish(msg)
        self._log(f"[ARM] angle={angle:.3f} rad")

    def _send_fix_toggle(self):
        if self.fix_pub:
            msg = Bool()
            msg.data = not self.perching_status
            self.fix_pub.publish(msg)
            action = "FIX" if msg.data else "UNFIX"
            self._log(f"[FIX] Sent {action}")

    def on_close(self):
        self._stop_sender.set()
        self._force_kill()
        self.master.destroy()


def main():
    root = tk.Tk()
    app = DroneGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == '__main__':
    main()
