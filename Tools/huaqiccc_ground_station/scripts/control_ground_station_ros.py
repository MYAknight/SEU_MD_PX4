#!/usr/bin/env python3
"""
Control Ground Station via ROS + MAVROS (NO direct serial connection)
Avoids serial port conflict with MAVROS

Updated 2026-06-11: Added morphing arm control panel
"""
import sys
import time
import math
import threading
import re

import rospy
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QPlainTextEdit, QGroupBox, QLineEdit,
    QSlider, QProgressBar
)
from PyQt5.QtCore import QTimer, pyqtSignal, QObject, Qt
from PyQt5.QtGui import QFont

from mavros_msgs.srv import CommandBool, SetMode, CommandLong, ParamSet, ParamGet
from mavros_msgs.msg import State
from mavros_msgs.msg import StatusText
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float64
from mavros_msgs.msg import ParamValue
from mavros_msgs.msg import DebugValue

# MAVLink command ID for huaqiccc morph angle
MAV_CMD_HUAQICCC_SET_ARM_ANGLE = 31440

# Morph angle limits (radians)
MORPH_ANGLE_MIN_RAD = -0.40  # fully expanded
MORPH_ANGLE_MAX_RAD = 0.00   # fully closed


class RosWorker(QObject):
    """ROS callbacks run in separate thread, emit signals to GUI"""
    state_sig = pyqtSignal(dict)
    pos_sig = pyqtSignal(dict)
    log_sig = pyqtSignal(str)
    morph_sig = pyqtSignal(dict)   # morph status updates
    contact_sig = pyqtSignal(dict) # contact/perching state updates
    
    def __init__(self):
        super().__init__()
        rospy.init_node('control_gs', anonymous=True)
        
        # Contact / perching state (parsed from PX4 DEBUG_FLOAT_ARRAY "perch"; STATUSTEXT as log fallback)
        self.pc_en = None            # current MPCA_PC_EN value from telemetry
        self.contact_state = {
            "state": "NO_CONTACT",      # NO_CONTACT / CANDIDATE / CONTACT_DETECTED / CONTACT /
                                        # COMPLIANT / GRASP_SECURE / RAMP_DOWN / PERCHED / ABORT
            "state_zh": "未接触",
            "err": None,
            "vel": None,
            "pitch_deg": None,
            "dx": None,
            "dz": None,
            "elapsed": None,
            "last_update": 0.0,
        }
        
        # Services
        self.srv_arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.srv_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.srv_cmd = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        self.srv_param_set = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        self.srv_param_get = rospy.ServiceProxy('/mavros/param/get', ParamGet)
        
        # Publishers
        self.pub_pos = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/mavros/state', State, self._on_state)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._on_pos)
        rospy.Subscriber('/mavros/statustext/recv', StatusText, self._on_statustext)
        rospy.Subscriber('/mavros/debug_value/debug_float_array', DebugValue, self._on_perch_debug)
        rospy.Subscriber('/mavros/battery', BatteryState, self._on_battery)
        
        self.current_state = {}
        self.current_pos = {"x": 0, "y": 0, "z": 0, "yaw": 0}
        self.sp_active = False
        self.sp = {"x": 0.0, "y": 0.0, "z": -1.0, "yaw": 0.0}
        self.sp_lock = threading.Lock()
        
        # Morph state
        self.morph_en = None
        self.morph_angle_rad = None
        self.morph_target_rad = None
        
        # Perching auto-retract state
        self._last_perching_phase = 0
        self._auto_retract_sent = False
        
        # Setpoint sender thread
        self.running = True
        threading.Thread(target=self._sp_sender, daemon=True).start()
        
        # Periodic morph status checker
        threading.Thread(target=self._morph_status_loop, daemon=True).start()
    
    def _on_state(self, msg):
        self.current_state = {
            "connected": msg.connected,
            "armed": msg.armed,
            "mode": msg.mode,
            "guided": msg.guided,
        }
        self.state_sig.emit(self.current_state)
    
    def _on_pos(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.current_pos = {"x": p.x, "y": p.y, "z": p.z, "yaw": yaw}
        self.pos_sig.emit(self.current_pos)
    
    def _on_statustext(self, msg):
        text = msg.text
        lower = text.lower()
        now = time.time()
        
        # Capture morph-related status text messages
        if 'huaqiccc_morph' in lower or 'morph' in lower or 'as5600' in lower:
            self.log_sig.emit(f"[MORPH] {text}")
            # Try to parse angle from text: supports "angle=-0.1234" or "a=-0.1234"
            angle_match = re.search(r'(?:angle|a)[=:]\s*([+-]?\d+\.?\d*)', lower)
            if angle_match:
                try:
                    self.morph_angle_rad = float(angle_match.group(1))
                    self.morph_sig.emit(self._get_morph_dict())
                except ValueError:
                    pass
            return
        
        # Capture contact / perching status text messages from mc_pos_control
        if 'contact' in lower or 'perching' in lower:
            self.log_sig.emit(f"[PERCH] {text}")
            self._update_contact_state(text, now)
            return
    
    def _on_perch_debug(self, msg):
        """Handle PX4 DEBUG_FLOAT_ARRAY telemetry for contact/perching state."""
        if msg.name != "perch":
            return
        if len(msg.data) < 5:
            return
        now = time.time()
        cs = self.contact_state
        cs["last_update"] = now

        contact_state = int(round(msg.data[0]))
        perching_phase = int(round(msg.data[1]))
        pc_en = int(round(msg.data[2]))
        arm_angle = msg.data[3]
        grasp_secure = int(round(msg.data[4]))
        perching_active = int(round(msg.data[5]))

        # Map contact detector state
        contact_labels = {
            0: ("NO_CONTACT", "未接触"),
            1: ("CANDIDATE", "接触候选"),
            2: ("CONTACT_DETECTED", "接触已确认"),
        }
        # Map perching phase (order from MulticopterPositionControl.hpp)
        phase_labels = {
            0: ("NO_CONTACT", "未接触"),
            1: ("APPROACH", "接近"),
            2: ("CONTACT", "接触阶段"),
            3: ("COMPLIANT", "柔顺接触"),
            4: ("RAMP_DOWN", "推力下降"),
            5: ("PERCHED", "已栖息"),
        }

        if perching_phase == 0:
            state_en, state_zh = contact_labels.get(contact_state, ("UNKNOWN", "未知"))
        else:
            state_en, state_zh = phase_labels.get(perching_phase, ("UNKNOWN", "未知"))

        # Grasp secure is reported while still in COMPLIANT phase
        if perching_phase == 3 and grasp_secure:
            state_en = "GRASP_SECURE"
            state_zh = "抓握确认"

        cs["state"] = state_en
        cs["state_zh"] = state_zh
        self.pc_en = pc_en
        cs["pc_en"] = pc_en
        cs["arm_angle"] = arm_angle if arm_angle < 90.0 else None
        cs["grasp_secure"] = grasp_secure
        cs["perching_active"] = perching_active
        self.contact_sig.emit(cs.copy())

        # Auto-retract is now handled inside PX4 (mc_pos_control publishes
        # huaqiccc_morph_cmd directly when entering CONTACT/COMPLIANT).
        # Keep the flag reset logic so the ground-station manual backup still works.
        if perching_phase == 0:
            self._auto_retract_sent = False

        self._last_perching_phase = perching_phase

        # Also update morph angle if valid arm telemetry is present
        if arm_angle < 90.0:
            self.morph_angle_rad = arm_angle
            self.morph_sig.emit(self._get_morph_dict())
    
    def _update_contact_state(self, text, now):
        """Parse mc_pos_control statustext and update contact/perching FSM state."""
        cs = self.contact_state
        cs["last_update"] = now
        
        # Contact candidate: Contact: candidate (err=0.303, vel=0.100, pitch=-10.30)
        m = re.search(r'contact:\s*candidate\s*\(err=([\d.]+),\s*vel=([\d.]+),\s*pitch=(-?[\d.]+)\)', text, re.IGNORECASE)
        if m:
            cs["state"] = "CANDIDATE"
            cs["state_zh"] = "接触候选"
            cs["err"] = float(m.group(1))
            cs["vel"] = float(m.group(2))
            cs["pitch_deg"] = float(m.group(3))
            self.contact_sig.emit(cs.copy())
            return
        
        # Contact confirmed: CONTACT_DETECTED: err=0.332 m, vel=0.090 m/s, pitch=-10.93 deg
        m = re.search(r'contact_detected:\s*err=([\d.]+)\s*m,\s*vel=([\d.]+)\s*m/s,\s*pitch=(-?[\d.]+)\s*deg', text, re.IGNORECASE)
        if m:
            cs["state"] = "CONTACT_DETECTED"
            cs["state_zh"] = "接触已确认"
            cs["err"] = float(m.group(1))
            cs["vel"] = float(m.group(2))
            cs["pitch_deg"] = float(m.group(3))
            self.contact_sig.emit(cs.copy())
            return
        
        # Perching phase transitions
        if 'contact detected, entering compliance' in lower:
            cs["state"] = "CONTACT"
            cs["state_zh"] = "接触阶段"
        elif 'entering soft contact' in lower:
            cs["state"] = "COMPLIANT"
            cs["state_zh"] = "柔顺接触"
        elif 'grasp secure confirmed' in lower:
            cs["state"] = "GRASP_SECURE"
            cs["state_zh"] = "抓握确认"
            # Perching: grasp secure confirmed (arms=0 have_data=0 pos=0 elapsed=6.0)
            m = re.search(r'elapsed=([\d.]+)', text)
            if m:
                cs["elapsed"] = float(m.group(1))
        elif 'grasp check failed' in lower:
            # Perching: grasp check failed, arms=0 have_data=0 pos=0 angle=99.000 dx=0.071 dz=0.033
            m = re.search(r'dx=([\d.-]+)\s+dz=([\d.-]+)', text)
            if m:
                cs["dx"] = float(m.group(1))
                cs["dz"] = float(m.group(2))
            # keep previous state, just update diagnostics
        elif 'thrust ramp-down started' in lower:
            cs["state"] = "RAMP_DOWN"
            cs["state_zh"] = "推力下降"
        elif 'zero thrust, arms holding' in lower:
            cs["state"] = "PERCHED"
            cs["state_zh"] = "已栖息"
        elif 'grasp timeout, aborting' in lower or 'abort, safety triggered' in lower:
            cs["state"] = "ABORT"
            cs["state_zh"] = "已中止"
        elif 'contact: reset' in lower:
            cs["state"] = "NO_CONTACT"
            cs["state_zh"] = "未接触"
        
        self.contact_sig.emit(cs.copy())
    
    def _on_battery(self, msg):
        voltage = msg.voltage
        percentage = msg.percentage
        if voltage > 0:
            self.log_sig.emit(f"[BATT] {voltage:.2f}V {percentage:.0f}%")
            # Update battery label directly (ROS callbacks run in separate thread,
            # but QLabel setText is generally thread-safe for simple text updates)
            batt_text = f"<span style='color:{'green' if percentage > 30 else 'red'}'>●</span> 电池: {voltage:.1f}V ({percentage:.0f}%)"
            # Emit via state_sig to ensure main-thread update
            self.state_sig.emit({"battery_voltage": voltage, "battery_percent": percentage, "battery_text": batt_text})
    
    def _get_morph_dict(self):
        return {
            "en": self.morph_en,
            "angle_rad": self.morph_angle_rad,
            "target_rad": self.morph_target_rad,
        }
    
    def _sp_sender(self):
        rate = rospy.Rate(20)
        while self.running and not rospy.is_shutdown():
            with self.sp_lock:
                if self.sp_active:
                    ps = PoseStamped()
                    ps.header.stamp = rospy.Time.now()
                    ps.header.frame_id = "map"
                    ps.pose.position.x = self.sp["x"]
                    ps.pose.position.y = self.sp["y"]
                    ps.pose.position.z = self.sp["z"]
                    cy = math.cos(self.sp["yaw"] * 0.5)
                    sy = math.sin(self.sp["yaw"] * 0.5)
                    ps.pose.orientation.w = cy
                    ps.pose.orientation.x = 0
                    ps.pose.orientation.y = 0
                    ps.pose.orientation.z = sy
                    self.pub_pos.publish(ps)
            rate.sleep()
    
    def _morph_status_loop(self):
        """Periodically read MORPH_EN parameter"""
        rate = rospy.Rate(0.2)  # every 5 seconds
        while self.running and not rospy.is_shutdown():
            self.read_morph_en()
            rate.sleep()
    
    def read_morph_en(self):
        try:
            resp = self.srv_param_get(param_id="MORPH_EN")
            if resp.success:
                self.morph_en = int(resp.value.integer)
                self.morph_sig.emit(self._get_morph_dict())
        except Exception as e:
            pass  # silent fail on periodic read
    
    def arm(self, arm=True):
        try:
            resp = self.srv_arm(arm)
            self.log_sig.emit(f"ARM={arm}: {'success' if resp.success else 'failed'}")
            return resp.success
        except Exception as e:
            self.log_sig.emit(f"ARM service error: {e}")
            return False
    
    def set_mode(self, mode_name):
        try:
            resp = self.srv_mode(custom_mode=mode_name)
            self.log_sig.emit(f"Mode {mode_name}: {'success' if resp.mode_sent else 'failed'}")
            return resp.mode_sent
        except Exception as e:
            self.log_sig.emit(f"Mode service error: {e}")
            return False
    
    def update_setpoint(self, x=None, y=None, z=None, yaw=None, relative=False):
        with self.sp_lock:
            if relative:
                if x is not None: self.sp["x"] += x
                if y is not None: self.sp["y"] += y
                if z is not None: self.sp["z"] += z
                if yaw is not None:
                    self.sp["yaw"] = (self.sp["yaw"] + yaw + math.pi) % (2 * math.pi) - math.pi
            else:
                if x is not None: self.sp["x"] = x
                if y is not None: self.sp["y"] = y
                if z is not None: self.sp["z"] = z
                if yaw is not None: self.sp["yaw"] = yaw
            self.sp_active = True
    
    def move_body(self, d_forward=0.0, d_right=0.0, d_up=0.0, d_yaw=0.0):
        """Move in body frame (FRD: Forward-Right-Down) relative to current setpoint.
        
        d_forward: + = forward along body X axis
        d_right:   + = right along body Y axis  
        d_up:      + = ascend (height increase)
        d_yaw:     + = counter-clockwise (CCW), radians
        
        Body frame is rotated by current yaw into ENU map frame.
        """
        yaw = self.current_pos.get("yaw", 0.0)
        with self.sp_lock:
            # Forward vector in ENU: (cos yaw, sin yaw)
            # Right vector in ENU:   (sin yaw, -cos yaw)
            self.sp["x"] += d_forward * math.cos(yaw) + d_right * math.sin(yaw)
            self.sp["y"] += d_forward * math.sin(yaw) - d_right * math.cos(yaw)
            self.sp["z"] += d_up  # ENU Z+ is up
            if d_yaw != 0.0:
                self.sp["yaw"] = (self.sp["yaw"] + d_yaw + math.pi) % (2 * math.pi) - math.pi
            self.sp_active = True
        
        # Log current setpoint for user feedback
        deg_yaw = math.degrees(self.sp["yaw"])
        self.log_sig.emit(f"Setpoint: X={self.sp['x']:.2f} Y={self.sp['y']:.2f} Z={self.sp['z']:.2f} Yaw={deg_yaw:.1f}°")
    
    def set_sp_active(self, active):
        with self.sp_lock:
            self.sp_active = active
    
    def set_controller(self, mode):
        try:
            pv = ParamValue()
            pv.integer = int(mode)
            resp = self.srv_param_set(param_id="MPCA_MODE", value=pv)
            names = {0: "PX4-PosCtl", 1: "GPID", 2: "LQR", 3: "MPC"}
            self.log_sig.emit(f"Controller set to {names.get(mode, mode)}: {'success' if resp.success else 'failed'}")
            return resp.success
        except Exception as e:
            self.log_sig.emit(f"Controller set error: {e}")
            return False
    
    def send_morph_angle(self, angle_rad):
        """Send target morph angle via MAVLink command 31440
        angle_rad: 0 = closed, negative = expanded (e.g. -0.4)
        """
        try:
            resp = self.srv_cmd(
                command=MAV_CMD_HUAQICCC_SET_ARM_ANGLE,
                param1=float(angle_rad),
                param2=0, param3=0, param4=0, param5=0, param6=0, param7=0
            )
            self.morph_target_rad = angle_rad
            self.morph_sig.emit(self._get_morph_dict())
            deg = math.degrees(angle_rad)
            self.log_sig.emit(f"Morph command sent: {deg:.1f}° ({angle_rad:.3f} rad) -> {'ACK' if resp.success else 'FAIL'}")
            return resp.success
        except Exception as e:
            self.log_sig.emit(f"Morph command error: {e}")
            return False
    
    def set_morph_en(self, enable):
        """Enable/disable morph control module"""
        try:
            pv = ParamValue()
            pv.integer = 1 if enable else 0
            resp = self.srv_param_set(param_id="MORPH_EN", value=pv)
            self.log_sig.emit(f"MORPH_EN set to {int(enable)}: {'success' if resp.success else 'failed'}")
            if resp.success:
                self.morph_en = int(enable)
                self.morph_sig.emit(self._get_morph_dict())
            return resp.success
        except Exception as e:
            self.log_sig.emit(f"MORPH_EN set error: {e}")
            return False
    
    def read_pc_en(self):
        """Read current MPCA_PC_EN parameter value."""
        try:
            resp = self.srv_param_get(param_id="MPCA_PC_EN")
            if resp.success:
                self.pc_en = int(resp.value.integer)
                # Emit via contact signal so UI updates
                cs = self.contact_state.copy()
                cs["pc_en"] = self.pc_en
                self.contact_sig.emit(cs)
                return True
        except Exception as e:
            self.log_sig.emit(f"MPCA_PC_EN read error: {e}")
        return False
    
    def set_pc_en(self, value):
        """Set MPCA_PC_EN in flight (1=DETECT only, 2=FULL perching FSM)."""
        try:
            pv = ParamValue()
            pv.integer = int(value)
            resp = self.srv_param_set(param_id="MPCA_PC_EN", value=pv)
            self.log_sig.emit(f"MPCA_PC_EN set to {int(value)}: {'success' if resp.success else 'failed'}")
            if resp.success:
                self.pc_en = int(value)
                cs = self.contact_state.copy()
                cs["pc_en"] = self.pc_en
                self.contact_sig.emit(cs)
                return True
            return False
        except Exception as e:
            self.log_sig.emit(f"MPCA_PC_EN set error: {e}")
            return False


class MainWindow(QMainWindow):
    def __init__(self, ros_worker):
        super().__init__()
        self.ros = ros_worker
        self.setWindowTitle("huaqiccc 控制地面站 (MAVROS模式) v2.0")
        self.setMinimumSize(560, 850)
        
        self.ros.state_sig.connect(self._on_state)
        self.ros.pos_sig.connect(self._on_pos)
        self.ros.log_sig.connect(self._on_log)
        self.ros.morph_sig.connect(self._on_morph)
        self.ros.contact_sig.connect(self._on_contact)
        
        self._build_ui()
        
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._update_sp_display)
        self.ui_timer.start(100)
    
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setSpacing(8)
        layout.setContentsMargins(12, 12, 12, 12)
        font_mono = QFont("Monospace", 10)
        font_mono.setStyleHint(QFont.Monospace)
        
        # ========== Status ==========
        sg = QGroupBox("状态")
        sl = QGridLayout()
        self.lbl_conn = QLabel("<span style='color:gray'>●</span> 飞控: 未连接")
        self.lbl_mode = QLabel("模式: --")
        self.lbl_arm = QLabel("锁定")
        self.lbl_batt = QLabel("电池: --")
        sl.addWidget(self.lbl_conn, 0, 0)
        sl.addWidget(self.lbl_mode, 0, 1)
        sl.addWidget(self.lbl_arm, 0, 2)
        sl.addWidget(self.lbl_batt, 1, 0)
        sg.setLayout(sl)
        layout.addWidget(sg)
        
        # ========== Morphing Control ==========
        mg = QGroupBox("🦾 变形控制 (huaqiccc_morph_control)")
        ml = QGridLayout()
        
        # Module status
        self.lbl_morph_en = QLabel("模块状态: 未知")
        self.lbl_morph_en.setStyleSheet("font-weight: bold;")
        ml.addWidget(self.lbl_morph_en, 0, 0, 1, 2)
        
        self.lbl_morph_angle = QLabel("当前角度: --")
        self.lbl_morph_angle.setFont(font_mono)
        ml.addWidget(self.lbl_morph_angle, 0, 2, 1, 2)
        
        self.lbl_morph_target = QLabel("目标角度: --")
        self.lbl_morph_target.setFont(font_mono)
        ml.addWidget(self.lbl_morph_target, 0, 4, 1, 2)
        
        # Angle slider (0° to -21° mapped to 0-21)
        ml.addWidget(QLabel("角度滑块:"), 1, 0)
        self.slider_morph = QSlider(Qt.Horizontal)
        self.slider_morph.setMinimum(0)
        self.slider_morph.setMaximum(21)  # 0 to 21 degrees (absolute value), max open ~21°
        self.slider_morph.setValue(0)
        self.slider_morph.setTickPosition(QSlider.TicksBelow)
        self.slider_morph.setTickInterval(5)
        self.slider_morph.valueChanged.connect(self._on_morph_slider)
        ml.addWidget(self.slider_morph, 1, 1, 1, 3)
        
        self.lbl_slider_val = QLabel("0° (收拢)")
        self.lbl_slider_val.setFont(font_mono)
        ml.addWidget(self.lbl_slider_val, 1, 4)
        
        # Manual angle input
        ml.addWidget(QLabel("精确角度(°):"), 2, 0)
        self.in_morph_deg = QLineEdit("0.0")
        self.in_morph_deg.setFont(font_mono)
        ml.addWidget(self.in_morph_deg, 2, 1)
        
        btn_morph_send = QPushButton("▶ 执行变形")
        btn_morph_send.clicked.connect(self._on_morph_send)
        btn_morph_send.setStyleSheet("background-color: #9C27B0; color: white; font-weight: bold;")
        ml.addWidget(btn_morph_send, 2, 2)
        
        btn_morph_close = QPushButton("⏹ 收拢 (0°)")
        btn_morph_close.clicked.connect(lambda: self._set_morph_preset(0.0))
        btn_morph_close.setStyleSheet("background-color: #607D8B; color: white;")
        ml.addWidget(btn_morph_close, 2, 3)
        
        btn_morph_open = QPushButton("⏏ 展开 (-21°)")
        btn_morph_open.clicked.connect(lambda: self._set_morph_preset(-21.0))
        btn_morph_open.setStyleSheet("background-color: #607D8B; color: white;")
        ml.addWidget(btn_morph_open, 2, 4)
        
        # Enable/Disable controls
        btn_morph_en_on = QPushButton("✅ 启用变形")
        btn_morph_en_on.clicked.connect(lambda: self.ros.set_morph_en(True))
        btn_morph_en_on.setStyleSheet("background-color: #4CAF50; color: white;")
        ml.addWidget(btn_morph_en_on, 3, 0, 1, 2)
        
        btn_morph_en_off = QPushButton("⛔ 禁用变形")
        btn_morph_en_off.clicked.connect(lambda: self.ros.set_morph_en(False))
        btn_morph_en_off.setStyleSheet("background-color: #f44336; color: white;")
        ml.addWidget(btn_morph_en_off, 3, 2, 1, 2)
        
        btn_morph_refresh = QPushButton("🔄 刷新状态")
        btn_morph_refresh.clicked.connect(self._on_morph_refresh)
        ml.addWidget(btn_morph_refresh, 3, 4)
        
        mg.setLayout(ml)
        layout.addWidget(mg)
        
        # ========== Contact / Perching State ==========
        cg = QGroupBox("🎯 接触 / 栖落状态")
        cgl = QGridLayout()
        self.lbl_contact_state = QLabel("<span style='color:gray;font-size:16px;font-weight:bold'>● 未接触</span>")
        self.lbl_contact_state.setStyleSheet("font-size: 16px; font-weight: bold;")
        cgl.addWidget(self.lbl_contact_state, 0, 0, 1, 2)
        
        self.lbl_contact_detail = QLabel("err: --  vel: --  pitch: --")
        self.lbl_contact_detail.setFont(font_mono)
        cgl.addWidget(self.lbl_contact_detail, 1, 0, 1, 2)
        
        self.lbl_contact_diag = QLabel("dx: --  dz: --  elapsed: --")
        self.lbl_contact_diag.setFont(font_mono)
        cgl.addWidget(self.lbl_contact_diag, 2, 0, 1, 2)
        
        self.btn_perch_en = QPushButton("自主抱柱: --")
        self.btn_perch_en.setStyleSheet("font-weight: bold; padding: 6px;")
        self.btn_perch_en.setToolTip("切换 MPCA_PC_EN：1=仅检测/记录，2=启用自主栖停 FSM\n可在飞行中实时切换")
        self.btn_perch_en.clicked.connect(self._toggle_perching_en)
        cgl.addWidget(self.btn_perch_en, 3, 0, 1, 2)
        
        cg.setLayout(cgl)
        layout.addWidget(cg)
        
        # ========== Position ==========
        pg = QGroupBox("位置")
        pl = QGridLayout()
        self.lbl_px = QLabel("X: 0.000"); self.lbl_px.setFont(font_mono)
        self.lbl_py = QLabel("Y: 0.000"); self.lbl_py.setFont(font_mono)
        self.lbl_pz = QLabel("Z: 0.000"); self.lbl_pz.setFont(font_mono)
        self.lbl_pyaw = QLabel("Yaw: 0.0°"); self.lbl_pyaw.setFont(font_mono)
        pl.addWidget(self.lbl_px, 0, 0)
        pl.addWidget(self.lbl_py, 0, 1)
        pl.addWidget(self.lbl_pz, 0, 2)
        pl.addWidget(self.lbl_pyaw, 0, 3)
        pg.setLayout(pl)
        layout.addWidget(pg)
        
        # ========== Setpoint ==========
        spg = QGroupBox("目标位置")
        spl = QGridLayout()
        self.lbl_spx = QLabel("X: 0.000"); self.lbl_spx.setFont(font_mono)
        self.lbl_spy = QLabel("Y: 0.000"); self.lbl_spy.setFont(font_mono)
        self.lbl_spz = QLabel("Z: 0.000"); self.lbl_spz.setFont(font_mono)
        spl.addWidget(self.lbl_spx, 0, 0)
        spl.addWidget(self.lbl_spy, 0, 1)
        spl.addWidget(self.lbl_spz, 0, 2)
        self.lbl_spyaw = QLabel("Yaw: 0.0°")
        self.lbl_spyaw.setFont(font_mono)
        spl.addWidget(self.lbl_spyaw, 0, 3)
        
        spl.addWidget(QLabel("目标X:"), 1, 0)
        self.in_x = QLineEdit("0.0")
        spl.addWidget(self.in_x, 1, 1)
        spl.addWidget(QLabel("目标Y:"), 2, 0)
        self.in_y = QLineEdit("0.0")
        spl.addWidget(self.in_y, 2, 1)
        spl.addWidget(QLabel("目标Z:"), 3, 0)
        self.in_z = QLineEdit("-1.0")
        spl.addWidget(self.in_z, 3, 1)
        
        btn_goto = QPushButton("→ 前往目标")
        btn_goto.clicked.connect(self._on_goto)
        btn_goto.setStyleSheet("background-color: #2196F3; color: white;")
        spl.addWidget(btn_goto, 1, 2, 3, 1)
        spg.setLayout(spl)
        layout.addWidget(spg)
        
        # ========== Controls ==========
        cg = QGroupBox("控制")
        cl = QGridLayout()
        
        btn_arm = QPushButton("ARM")
        btn_arm.clicked.connect(lambda: self.ros.arm(True))
        btn_arm.setStyleSheet("background-color: #4CAF50; color: white;")
        
        btn_disarm = QPushButton("DISARM")
        btn_disarm.clicked.connect(lambda: self.ros.arm(False))
        btn_disarm.setStyleSheet("background-color: #f44336; color: white;")
        
        btn_posctl = QPushButton("位置模式")
        btn_posctl.clicked.connect(lambda: self.ros.set_mode("POSCTL"))
        
        btn_offboard = QPushButton("OFFBOARD")
        btn_offboard.clicked.connect(lambda: self.ros.set_mode("OFFBOARD"))
        
        btn_manual = QPushButton("手动模式")
        btn_manual.clicked.connect(lambda: self.ros.set_mode("MANUAL"))
        
        btn_takeoff = QPushButton("一键起飞 1.5m")
        btn_takeoff.clicked.connect(self._on_takeoff)
        btn_takeoff.setStyleSheet("background-color: #FF9800; color: white;")
        
        btn_land = QPushButton("降落")
        btn_land.clicked.connect(self._on_land)
        
        btn_estop = QPushButton("急停 (DISARM)")
        btn_estop.clicked.connect(self._on_estop)
        btn_estop.setStyleSheet("background-color: #B71C1C; color: white;")
        
        btn_px4 = QPushButton("PX4 PosCtl")
        btn_px4.clicked.connect(lambda: self.ros.set_controller(0))
        btn_gpid = QPushButton("GPID")
        btn_gpid.clicked.connect(lambda: self.ros.set_controller(1))
        btn_lqr = QPushButton("LQR")
        btn_lqr.clicked.connect(lambda: self.ros.set_controller(2))
        btn_mpc = QPushButton("MPC")
        btn_mpc.clicked.connect(lambda: self.ros.set_controller(3))
        
        cl.addWidget(btn_arm, 0, 0)
        cl.addWidget(btn_disarm, 0, 1)
        cl.addWidget(btn_posctl, 0, 2)
        cl.addWidget(btn_manual, 1, 0)
        cl.addWidget(btn_offboard, 1, 1)
        cl.addWidget(btn_takeoff, 1, 2)
        cl.addWidget(btn_land, 2, 0)
        cl.addWidget(btn_px4, 2, 1)
        cl.addWidget(btn_gpid, 2, 2)
        cl.addWidget(btn_lqr, 3, 0)
        cl.addWidget(btn_mpc, 3, 1)
        cl.addWidget(btn_estop, 3, 2)
        cg.setLayout(cl)
        layout.addWidget(cg)
        
        # ========== Directional ==========
        dg = QGroupBox("方向控制 (Offboard / 机体坐标系)")
        dl = QGridLayout()
        
        # Step size inputs
        dl.addWidget(QLabel("平移步长(m):"), 0, 0)
        self.in_step_xy = QLineEdit("0.30")
        self.in_step_xy.setFont(font_mono)
        dl.addWidget(self.in_step_xy, 0, 1)
        
        dl.addWidget(QLabel("垂直步长(m):"), 0, 2)
        self.in_step_z = QLineEdit("0.20")
        self.in_step_z.setFont(font_mono)
        dl.addWidget(self.in_step_z, 0, 3)
        
        dl.addWidget(QLabel("旋转步长(°):"), 0, 4)
        self.in_step_yaw = QLineEdit("15.0")
        self.in_step_yaw.setFont(font_mono)
        dl.addWidget(self.in_step_yaw, 0, 5)
        
        # Directional buttons (body frame: F/R/U relative to current heading)
        btn_f = QPushButton("↑ 前")
        btn_b = QPushButton("↓ 后")
        btn_l = QPushButton("← 左")
        btn_r = QPushButton("→ 右")
        btn_u = QPushButton("▲ 上")
        btn_d = QPushButton("▼ 下")
        btn_cw = QPushButton("↻ 顺时针")
        btn_ccw = QPushButton("↺ 逆时针")
        
        # Auto-switch to OFFBOARD on directional command
        def _ensure_offboard():
            if self.ros.current_state.get("mode") != "OFFBOARD":
                self.ros.set_mode("OFFBOARD")
        
        btn_f.clicked.connect(lambda: (_ensure_offboard(), self._move_direction("forward")))
        btn_b.clicked.connect(lambda: (_ensure_offboard(), self._move_direction("backward")))
        btn_l.clicked.connect(lambda: (_ensure_offboard(), self._move_direction("left")))
        btn_r.clicked.connect(lambda: (_ensure_offboard(), self._move_direction("right")))
        btn_u.clicked.connect(lambda: (_ensure_offboard(), self._move_direction("up")))
        btn_d.clicked.connect(lambda: (_ensure_offboard(), self._move_direction("down")))
        btn_cw.clicked.connect(lambda: (_ensure_offboard(), self._move_direction("cw")))
        btn_ccw.clicked.connect(lambda: (_ensure_offboard(), self._move_direction("ccw")))
        
        dl.addWidget(btn_f, 1, 1)
        dl.addWidget(btn_l, 2, 0)
        dl.addWidget(btn_r, 2, 2)
        dl.addWidget(btn_b, 3, 1)
        dl.addWidget(btn_u, 1, 3)
        dl.addWidget(btn_d, 2, 3)
        dl.addWidget(btn_cw, 1, 5)
        dl.addWidget(btn_ccw, 2, 5)
        dg.setLayout(dl)
        layout.addWidget(dg)
        
        # ========== Log ==========
        self.log_edit = QPlainTextEdit()
        self.log_edit.setReadOnly(True)
        self.log_edit.setMaximumBlockCount(200)
        layout.addWidget(self.log_edit)
        
        self.show()
    
    # ========== Morph callbacks ==========
    def _on_morph_slider(self, value):
        """Slider: 0~40 maps to 0° ~ -40°"""
        deg = -float(value)
        status = "收拢" if value == 0 else f"展开 {abs(deg):.0f}°"
        self.lbl_slider_val.setText(f"{deg:.0f}° ({status})")
        # Sync input field so "execute" button sends the slider value
        self.in_morph_deg.setText(f"{deg:.1f}")
    
    def _on_morph_send(self):
        try:
            deg = float(self.in_morph_deg.text())
            # Clamp to valid range: 0° (closed) to -21° (max open)
            deg = max(-21.0, min(0.0, deg))
            rad = math.radians(deg)
            self.ros.send_morph_angle(rad)
        except ValueError:
            self._on_log("Invalid morph angle value")
    
    def _set_morph_preset(self, deg):
        """Preset button: set angle directly"""
        rad = math.radians(deg)
        self.in_morph_deg.setText(f"{deg:.1f}")
        self.slider_morph.setValue(int(abs(deg)))
        self.ros.send_morph_angle(rad)
    
    def _on_morph_refresh(self):
        self.ros.read_morph_en()
        self._on_log("Morph status refreshed")
    
    def _on_morph(self, morph):
        en = morph.get("en")
        angle = morph.get("angle_rad")
        target = morph.get("target_rad")
        
        # Update MORPH_EN status
        if en is not None:
            if en == 1:
                self.lbl_morph_en.setText("<span style='color:green'>●</span> 模块状态: 已启用 (MORPH_EN=1)")
            else:
                self.lbl_morph_en.setText("<span style='color:red'>●</span> 模块状态: 已禁用 (MORPH_EN=0)")
        
        # Update current angle
        if angle is not None:
            deg = math.degrees(angle)
            self.lbl_morph_angle.setText(f"当前角度: {deg:.1f}° ({angle:.3f} rad)")
        else:
            self.lbl_morph_angle.setText("当前角度: --")
        
        # Update target angle
        if target is not None:
            deg = math.degrees(target)
            self.lbl_morph_target.setText(f"目标角度: {deg:.1f}°")
        else:
            self.lbl_morph_target.setText("目标角度: --")
    
    # ========== Existing callbacks ==========
    def _on_contact(self, cs):
        state = cs.get("state", "NO_CONTACT")
        state_zh = cs.get("state_zh", "未接触")
        
        color_map = {
            "NO_CONTACT": "gray",
            "CANDIDATE": "orange",
            "CONTACT_DETECTED": "#FF9800",  # deep orange
            "CONTACT": "#2196F3",           # blue
            "COMPLIANT": "#03A9F4",         # light blue
            "GRASP_SECURE": "#4CAF50",      # green
            "RAMP_DOWN": "#9C27B0",         # purple
            "PERCHED": "#2E7D32",           # dark green
            "ABORT": "#f44336",             # red
        }
        color = color_map.get(state, "gray")
        self.lbl_contact_state.setText(f"<span style='color:{color};font-size:16px;font-weight:bold'>● {state_zh}</span>")
        
        # Detail line: err / vel / pitch (from statustext) or arm / pc_en (from debug telemetry)
        err = cs.get("err")
        vel = cs.get("vel")
        pitch = cs.get("pitch_deg")
        arm_angle = cs.get("arm_angle")
        pc_en = cs.get("pc_en")
        detail_parts = []
        if err is not None:
            detail_parts.append(f"err: {err:.3f}m")
        if vel is not None:
            detail_parts.append(f"vel: {vel:.3f}m/s")
        if pitch is not None:
            detail_parts.append(f"pitch: {pitch:.1f}°")
        if arm_angle is not None:
            detail_parts.append(f"arm: {arm_angle:.3f}rad")
        if pc_en is not None:
            detail_parts.append(f"pc_en: {pc_en}")
        self.lbl_contact_detail.setText("  ".join(detail_parts) if detail_parts else "err: --  vel: --  pitch: --")
        
        # Diagnostic line: dx / dz / elapsed (from statustext) or grasp / active (from debug telemetry)
        dx = cs.get("dx")
        dz = cs.get("dz")
        elapsed = cs.get("elapsed")
        grasp_secure = cs.get("grasp_secure")
        perching_active = cs.get("perching_active")
        diag_parts = []
        if dx is not None:
            diag_parts.append(f"dx: {dx:.3f}m")
        if dz is not None:
            diag_parts.append(f"dz: {dz:.3f}m")
        if elapsed is not None:
            diag_parts.append(f"elapsed: {elapsed:.1f}s")
        if grasp_secure is not None:
            diag_parts.append(f"grasp: {grasp_secure}")
        if perching_active is not None:
            diag_parts.append(f"active: {perching_active}")
        self.lbl_contact_diag.setText("  ".join(diag_parts) if diag_parts else "dx: --  dz: --  elapsed: --")
        
        # Update perching enable button to reflect current MPCA_PC_EN
        self._update_perch_en_button(pc_en)
    
    def _update_perch_en_button(self, pc_en):
        if pc_en is None:
            self.btn_perch_en.setText("自主抱柱: --")
            self.btn_perch_en.setStyleSheet("font-weight: bold; padding: 6px;")
        elif pc_en >= 2:
            self.btn_perch_en.setText("自主抱柱: 开启 (MPCA_PC_EN=2)")
            self.btn_perch_en.setStyleSheet("font-weight: bold; padding: 6px; background-color: #4CAF50; color: white;")
        else:
            self.btn_perch_en.setText("自主抱柱: 关闭 (MPCA_PC_EN=1)")
            self.btn_perch_en.setStyleSheet("font-weight: bold; padding: 6px; background-color: #f44336; color: white;")
    
    def _toggle_perching_en(self):
        """Toggle MPCA_PC_EN between 1 (DETECT only) and 2 (FULL perching FSM)."""
        current = self.ros.pc_en
        if current is None:
            self._on_log("[PERCH] 当前 MPCA_PC_EN 未知，先读取参数...")
            self.ros.read_pc_en()
            return
        new_val = 1 if current >= 2 else 2
        if self.ros.set_pc_en(new_val):
            self._on_log(f"[PERCH] MPCA_PC_EN 已切换为 {new_val} ({'FULL' if new_val == 2 else 'DETECT'})")
        else:
            self._on_log(f"[PERCH] MPCA_PC_EN 切换失败")
    
    def _on_state(self, state):
        # Battery update via state_sig from _on_battery
        if "battery_text" in state:
            self.lbl_batt.setText(state["battery_text"])
            return
        # Regular state update
        color = "green" if state.get("connected") else "gray"
        self.lbl_conn.setText(f"<span style='color:{color}'>●</span> 飞控: {'已连接' if state.get('connected') else '未连接'}")
        self.lbl_mode.setText(f"模式: {state.get('mode', '--')}")
        if state.get("armed"):
            self.lbl_arm.setText("<span style='color:red'>● ARMED</span>")
        else:
            self.lbl_arm.setText("锁定")
    
    def _on_pos(self, pos):
        self.lbl_px.setText(f"X: {pos['x']:.3f}")
        self.lbl_py.setText(f"Y: {pos['y']:.3f}")
        self.lbl_pz.setText(f"Z: {pos['z']:.3f}")
        self.lbl_pyaw.setText(f"Yaw: {math.degrees(pos['yaw']):.1f}°")
    
    def _update_sp_display(self):
        sp = self.ros.sp
        self.lbl_spx.setText(f"X: {sp['x']:.3f}")
        self.lbl_spy.setText(f"Y: {sp['y']:.3f}")
        self.lbl_spz.setText(f"Z: {sp['z']:.3f}")
        if hasattr(self, 'lbl_spyaw'):
            self.lbl_spyaw.setText(f"Yaw: {math.degrees(sp['yaw']):.1f}°")
    
    def _on_log(self, msg):
        self.log_edit.appendPlainText(f"[{time.strftime('%H:%M:%S')}] {msg}")
    
    def _on_takeoff(self):
        self.ros.update_setpoint(z=-1.5, relative=False)
        self.ros.set_sp_active(True)
        self.ros.set_mode("OFFBOARD")
        self._on_log("Takeoff to 1.5m + OFFBOARD")
    
    def _on_goto(self):
        try:
            x = float(self.in_x.text())
            y = float(self.in_y.text())
            z = float(self.in_z.text())
            self.ros.update_setpoint(x=x, y=y, z=z, relative=False)
            self.ros.set_sp_active(True)
            self.ros.set_mode("OFFBOARD")
            self._on_log(f"Goto ({x}, {y}, {z})")
        except ValueError:
            self._on_log("Invalid setpoint values")
    
    def _move_direction(self, direction):
        """Handle directional movement in body frame (FRD)."""
        try:
            step_xy = float(self.in_step_xy.text())
            step_z = float(self.in_step_z.text())
            step_yaw_deg = float(self.in_step_yaw.text())
            step_yaw_rad = math.radians(step_yaw_deg)
        except ValueError:
            self._on_log("Invalid step size value")
            return
        
        if direction == "forward":
            self.ros.move_body(d_forward=step_xy)
        elif direction == "backward":
            self.ros.move_body(d_forward=-step_xy)
        elif direction == "left":
            self.ros.move_body(d_right=-step_xy)
        elif direction == "right":
            self.ros.move_body(d_right=step_xy)
        elif direction == "up":
            self.ros.move_body(d_up=step_z)
        elif direction == "down":
            self.ros.move_body(d_up=-step_z)
        elif direction == "cw":
            # Clockwise = Yaw decreases
            self.ros.move_body(d_yaw=-step_yaw_rad)
        elif direction == "ccw":
            # Counter-clockwise = Yaw increases
            self.ros.move_body(d_yaw=step_yaw_rad)
    
    def _on_land(self):
        self.ros.set_sp_active(False)
        ok = self.ros.set_mode("LAND")
        self._on_log(f"LAND mode: {'sent' if ok else 'failed'}")
    
    def _on_estop(self):
        self.ros.arm(False)
        self.ros.set_sp_active(False)
        self._on_log("EMERGENCY STOP - DISARMED")
    
    def closeEvent(self, event):
        self.ros.running = False
        event.accept()


def main():
    app = QApplication(sys.argv)
    ros_worker = RosWorker()
    win = MainWindow(ros_worker)
    
    # ROS spin in separate thread
    ros_thread = threading.Thread(target=rospy.spin, daemon=True)
    ros_thread.start()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
