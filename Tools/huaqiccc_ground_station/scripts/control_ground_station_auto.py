#!/usr/bin/env python3
"""
Control Ground Station via ROS + MAVROS — Autonomous Perching Edition
Avoids serial port conflict with MAVROS

基于 control_ground_station_ros.py 扩展：
  - YOLO 视觉检测订阅与 YAW 对齐控制
  - 相机外参标定（补偿相机中心与无人机中心偏移）
  - 一键自主栖息任务状态机
  - YOLO 跳变保护（变化率限制、最大角速度限制）

Launch with:
  ./start_ground_station_auto.sh
"""
import sys
import time
import math
import threading
import re
import json
import os
import collections
from pathlib import Path

import yaml

import rospy
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QPlainTextEdit, QGroupBox, QLineEdit,
    QSlider, QProgressBar, QMessageBox, QComboBox, QSplitter, QSizePolicy
)
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import QTimer, pyqtSignal, QObject, Qt
from PyQt5.QtGui import QFont

from mavros_msgs.srv import CommandBool, SetMode, CommandLong, ParamSet, ParamGet
from mavros_msgs.msg import State
from mavros_msgs.msg import StatusText
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge
from std_msgs.msg import Float64, Bool, Float32, Int32, String
from mavros_msgs.msg import ParamValue
from mavros_msgs.msg import DebugValue

# MAVLink command ID for huaqiccc morph angle
MAV_CMD_HUAQICCC_SET_ARM_ANGLE = 31440

# Morph angle limits (radians)
MORPH_ANGLE_MIN_RAD = -0.40  # fully expanded
MORPH_ANGLE_MAX_RAD = 0.00   # fully closed

# Perching mission defaults
# Z sign convention: ENU frame used by MAVROS /local_position/pose in this setup.
# Positive Z = up, positive X = forward, positive Y = left.
PERCHING_HOVER_Z = 1.0           # hover height setpoint (m above ground)
PERCHING_APPROACH_SPEED = 0.05   # m/s forward speed during approach/blind push
PERCHING_EXPAND_ANGLE = -0.323    # radians, negative = arms expanded (~18.5°)

# Contact states that mean the pole has been physically detected
PERCHING_CONTACT_STATES = {
    "CONTACT_DETECTED", "CONTACT", "COMPLIANT",
    "GRASP_SECURE", "RAMP_DOWN", "PERCHED",
}

# Grasp states that mean the arms have closed securely around the pole
PERCHING_GRASP_STATES = {"GRASP_SECURE", "PERCHED"}


class RosWorker(QObject):
    """ROS callbacks run in separate thread, emit signals to GUI"""
    state_sig = pyqtSignal(dict)
    pos_sig = pyqtSignal(dict)
    log_sig = pyqtSignal(str)
    morph_sig = pyqtSignal(dict)   # morph status updates
    contact_sig = pyqtSignal(dict) # contact/perching state updates
    vision_sig = pyqtSignal(dict)  # YOLO vision updates
    image_sig = pyqtSignal(dict)   # detection image for camera view
    mission_sig = pyqtSignal(dict) # perching mission state updates
    
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
        self.pub_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.pub_lock_target = rospy.Publisher('/yolo/lock_target', Int32, queue_size=1, latch=True)
        
        # Subscribers
        rospy.Subscriber('/mavros/state', State, self._on_state)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._on_pos)
        rospy.Subscriber('/mavros/statustext/recv', StatusText, self._on_statustext)
        rospy.Subscriber('/mavros/debug_value/debug_float_array', DebugValue, self._on_perch_debug)
        rospy.Subscriber('/mavros/battery', BatteryState, self._on_battery)
        rospy.Subscriber('/yolo/yaw_aligned', Bool, self._on_yaw_aligned)
        rospy.Subscriber('/yolo/pixel_error', Float32, self._on_pixel_error)
        rospy.Subscriber('/yolo/detections_info', String, self._on_detections_info)
        rospy.Subscriber('/yolo/node_alive', Bool, self._on_video_alive)
        
        self.current_state = {}
        self.current_pos = {"x": 0, "y": 0, "z": 0, "yaw": 0}
        self.sp_active = False
        self.sp = {"x": 0.0, "y": 0.0, "z": 1.0, "yaw": 0.0}
        self.sp_lock = threading.Lock()
        
        # Morph state
        self.morph_en = None
        self.morph_angle_rad = None
        self.morph_target_rad = None

        # Vision state
        self.yaw_aligned = False
        self.pixel_error = 0.0
        self.detection_active = False
        self.detection_last_time = 0.0
        self.detections_count = 0
        self.video_alive = False
        self.video_alive_last_time = 0.0

        # Camera image for GUI
        self.bridge = CvBridge()
        rospy.Subscriber('/yolo/detection_image', SensorImage, self._on_detection_image)

        # YAW controller parameters
        self.camera_offset_x = 0.0      # pixels, positive = camera center is right of image center
        self._load_camera_offset()      # try loading persisted calibration
        self.image_width = 640.0
        self.yaw_Kp = 1.5
        self.yaw_max_rate = 0.3         # rad/s, user requested limit
        self.yaw_deadzone = 0.08
        self.yaw_rate_max_delta = 0.15  # rad/s per control step (jump protection)
        self.last_yaw_rate = 0.0
        self.yaw_control_active = False
        self.yaw_target = 0.0        # integrated yaw setpoint when yaw_control_active

        # Setpoint sender thread
        self.running = True
        threading.Thread(target=self._sp_sender, daemon=True).start()

        # Track flight mode changes for diagnostics
        self._last_mode = None

        # Periodic morph status checker
        threading.Thread(target=self._morph_status_loop, daemon=True).start()

        # MAVLink / vision topic rate diagnostics (helps debug HM30 bandwidth)
        self.topic_arrivals = collections.defaultdict(list)
        threading.Thread(target=self._telemetry_rate_loop, daemon=True).start()
    
    def _record_arrival(self, topic):
        now = time.time()
        self.topic_arrivals[topic].append(now)

    def _telemetry_rate_loop(self):
        """Log actual incoming MAVLink/vision rates to diagnose link saturation."""
        period = 5.0
        while self.running:
            rospy.sleep(period)
            now = time.time()
            rates = {}
            for topic, ts in list(self.topic_arrivals.items()):
                # keep only last period
                recent = [t for t in ts if now - t <= period]
                self.topic_arrivals[topic] = recent
                rates[topic] = len(recent) / period
            if rates:
                summary = ", ".join(f"{t.split('/')[-1]}={r:.1f}Hz" for t, r in rates.items())
                self.log_sig.emit(f"[LINK] rates: {summary}")

    def _on_state(self, msg):
        self._record_arrival('/mavros/state')
        self.current_state = {
            "connected": msg.connected,
            "armed": msg.armed,
            "mode": msg.mode,
            "guided": msg.guided,
        }
        if self._last_mode is not None and self._last_mode != msg.mode:
            if self._last_mode == "OFFBOARD":
                self.log_sig.emit(f"[STATE] 飞控退出 OFFBOARD -> {msg.mode}")
            elif msg.mode == "OFFBOARD":
                self.log_sig.emit(f"[STATE] 飞控进入 OFFBOARD (来自 {self._last_mode})")
        self._last_mode = msg.mode
        self.state_sig.emit(self.current_state)
    
    def _on_pos(self, msg):
        self._record_arrival('/mavros/local_position/pose')
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.current_pos = {"x": p.x, "y": p.y, "z": p.z, "yaw": yaw}
        self.pos_sig.emit(self.current_pos)

    def _on_yaw_aligned(self, msg):
        self.yaw_aligned = bool(msg.data)
        self._emit_vision_state()

    def _on_pixel_error(self, msg):
        self._record_arrival('/yolo/pixel_error')
        self.pixel_error = float(msg.data)
        self.detection_active = True
        self.detection_last_time = time.time()
        self._emit_vision_state()

    def _on_video_alive(self, msg):
        self.video_alive = bool(msg.data)
        if self.video_alive:
            self.video_alive_last_time = time.time()

    def is_video_stream_alive(self, timeout=1.0):
        """Return True if /yolo/node_alive is being received and True."""
        return self.video_alive and (time.time() - self.video_alive_last_time) < timeout

    def is_detection_active(self, timeout=1.0):
        """Return True if we are receiving recent pixel_error updates."""
        return self.detection_active and (time.time() - self.detection_last_time) < timeout

    def _on_detections_info(self, msg):
        try:
            info = json.loads(msg.data)
            targets = info.get("targets", [])
            self.detections_count = len(targets)
        except Exception:
            self.detections_count = 0

    def _on_detection_image(self, msg):
        self._record_arrival('/yolo/detection_image')
        """Convert YOLO detection image to RGB bytes and emit to GUI."""
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            h, w = cv_img.shape[:2]
            self.image_sig.emit({
                "data": cv_img.tobytes(),
                "width": w,
                "height": h,
                "bytes_per_line": 3 * w,
            })
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"detection_image conversion failed: {e}")

    def _emit_vision_state(self):
        self.vision_sig.emit({
            "yaw_aligned": self.yaw_aligned,
            "pixel_error": self.pixel_error,
            "detection_active": self.detection_active and (time.time() - self.detection_last_time) < 0.5,
            "detections_count": self.detections_count,
        })

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
        """Publish position setpoint at fixed rate.

        When yaw_control_active is True, the pixel-error-based yaw rate is
        integrated into self.sp['yaw'] so that only /mavros/setpoint_position/local
        is used.  This avoids mixing position and velocity setpoints on PX4.
        """
        # Lower setpoint rate to reduce HM30 telemetry/uplink congestion
        # that appears to starve the RTSP video stream on the same radio.
        rate_hz = 10.0
        dt = 1.0 / rate_hz
        rate = rospy.Rate(rate_hz)
        while self.running and not rospy.is_shutdown():
            with self.sp_lock:
                if self.sp_active:
                    if self.yaw_control_active:
                        yaw_rate = self._compute_yaw_rate(dt)
                        self.yaw_target += yaw_rate * dt
                        self.yaw_target = (self.yaw_target + math.pi) % (2.0 * math.pi) - math.pi
                        self.sp["yaw"] = self.yaw_target

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

    def _compute_yaw_rate(self, dt=None):
        """Compute yaw rate from pixel error with camera offset compensation and jump protection.

        Args:
            dt: time step in seconds; defaults to 1/30 s for backward compatibility.
        """
        if dt is None:
            dt = 1.0 / 30.0
        corrected_error = self.pixel_error + (self.camera_offset_x / (self.image_width / 2.0))

        if abs(corrected_error) < self.yaw_deadzone:
            yaw_rate = 0.0
        else:
            yaw_rate = -self.yaw_Kp * corrected_error * self.yaw_max_rate
            yaw_rate = max(-self.yaw_max_rate, min(self.yaw_max_rate, yaw_rate))

        # Rate-of-change limit (jump protection)
        max_delta = self.yaw_rate_max_delta * dt
        delta = yaw_rate - self.last_yaw_rate
        delta = max(-max_delta, min(max_delta, delta))
        yaw_rate = self.last_yaw_rate + delta
        self.last_yaw_rate = yaw_rate
        return yaw_rate

    def _camera_offset_path(self):
        """Path to persisted camera offset config."""
        p = Path.home() / ".config" / "ground_station" / "camera_offset.yaml"
        return p

    def _load_camera_offset(self):
        """Load camera offset from persistent config file if it exists."""
        try:
            p = self._camera_offset_path()
            if p.exists():
                with open(p, "r") as f:
                    cfg = yaml.safe_load(f)
                if cfg and "camera_offset_x" in cfg:
                    self.camera_offset_x = float(cfg["camera_offset_x"])
                    self.log_sig.emit(f"[VISION] 已加载相机偏移: {self.camera_offset_x:.1f} px ({p})")
        except Exception as e:
            self.log_sig.emit(f"[VISION] 加载相机偏移失败: {e}")

    def save_camera_offset(self):
        """Persist current camera offset to config file."""
        try:
            p = self._camera_offset_path()
            p.parent.mkdir(parents=True, exist_ok=True)
            with open(p, "w") as f:
                yaml.safe_dump({
                    "camera_offset_x": float(self.camera_offset_x),
                    "image_width": float(self.image_width),
                    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                }, f, default_flow_style=False, allow_unicode=True)
            self.log_sig.emit(f"[VISION] 相机偏移已保存: {p}")
            return True
        except Exception as e:
            self.log_sig.emit(f"[VISION] 保存相机偏移失败: {e}")
            return False

    def set_yaw_control(self, active):
        """Enable/disable YAW control via position setpoint yaw integration."""
        if active and not self.yaw_control_active:
            # Start integrating from current heading to avoid a step change
            self.yaw_target = self.current_pos.get("yaw", 0.0)
            self.last_yaw_rate = 0.0
        self.yaw_control_active = active
        if not active:
            self.last_yaw_rate = 0.0

    def set_lock_target(self, target_id):
        """Set YOLO target lock: -1 = auto, >=0 = lock specific ID."""
        self.pub_lock_target.publish(Int32(data=target_id))

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
    
    def set_message_intervals(self):
        """Reduce high-rate MAVLink downlink streams to free HM30 bandwidth for video."""
        # MAV_CMD_SET_MESSAGE_INTERVAL = 511
        # param1: message id, param2: interval_us
        targets = {
            32: 100000,   # LOCAL_POSITION_NED -> 10 Hz
            30: 100000,   # ATTITUDE -> 10 Hz
            31: 100000,   # ATTITUDE_QUATERNION -> 10 Hz
        }
        try:
            for msg_id, interval_us in targets.items():
                resp = self.srv_cmd(
                    broadcast=False,
                    command=511,
                    confirmation=0,
                    param1=float(msg_id),
                    param2=float(interval_us),
                    param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0,
                )
                ok = getattr(resp, 'success', False)
                self.log_sig.emit(f"[MAV] set message interval msg={msg_id} interval={interval_us}us: {'ok' if ok else 'failed'}")
        except Exception as e:
            self.log_sig.emit(f"[MAV] set message interval error: {e}")

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


    def disable_rtsp_cmd_vel(self):
        """Tell rtsp_pillar_node to stop publishing cmd_vel so we can take over."""
        try:
            rospy.set_param('/rtsp_pillar_node/publish_cmd_vel', False)
            self.log_sig.emit("[VISION] rtsp_pillar_node cmd_vel disabled")
        except Exception as e:
            self.log_sig.emit(f"[VISION] Failed to disable rtsp cmd_vel: {e}")

    def enable_rtsp_cmd_vel(self):
        """Allow rtsp_pillar_node to resume cmd_vel publishing."""
        try:
            rospy.set_param('/rtsp_pillar_node/publish_cmd_vel', True)
            self.log_sig.emit("[VISION] rtsp_pillar_node cmd_vel enabled")
        except Exception as e:
            self.log_sig.emit(f"[VISION] Failed to enable rtsp cmd_vel: {e}")



class PerchingMission(QObject):
    """Autonomous perching mission state machine."""
    mission_sig = pyqtSignal(dict)

    PHASE_NAMES = {
        "IDLE": "空闲",
        "TAKEOFF": "起飞",
        "ALIGN": "对齐",
        "EXPAND": "展开机臂",
        "APPROACH": "接近",
        "BLIND_PUSH": "盲推",
        "CONTACT_WAIT": "接触等待",
        "THROTTLE_RAMP": "油门下降",
        "DISARM": "锁定",
        "DONE": "完成",
        "ABORT": "中止",
        "HOLD_TAKEOFF": "悬停(起飞后)",
        "HOLD_ALIGN": "悬停(对齐后)",
        "HOLD_EXPAND": "悬停(展开后)",
        "HOLD_APPROACH": "悬停(接近后)",
        "HOLD_BLIND_PUSH": "悬停(盲推后)",
        "HOLD_CONTACT_WAIT": "悬停(接触等待后)",
    }

    # Ordered list of phases for the staged mission
    PHASE_ORDER = [
        "TAKEOFF", "ALIGN", "EXPAND", "APPROACH",
        "BLIND_PUSH", "CONTACT_WAIT", "THROTTLE_RAMP", "DISARM", "DONE",
    ]

    def __init__(self, ros_worker):
        super().__init__()
        self.ros = ros_worker
        self.phase = "IDLE"
        self.running = False
        self.thread = None
        self.abort_requested = False

        # Mission parameters
        self.hover_z = PERCHING_HOVER_Z
        self.approach_speed = PERCHING_APPROACH_SPEED
        self.expand_angle = PERCHING_EXPAND_ANGLE
        self.stable_threshold = 0.25     # m, vertical/horizontal position tolerance for takeoff
        self.stable_velocity_threshold = 0.15  # m/s, low-passed velocity tolerance
        self.stable_hold_time = 1.0      # s, require stability for this long before proceeding
        self.align_timeout = 60.0        # s, allow time for camera voltage-sag reboot + YOLO restart
        self.align_no_det_pause = 2.0    # s, pause timeout if no target is seen for this long
        self.phase_timeout = 60.0        # s
        self.yaw_align_hold_time = 1.0   # s
        self.detection_loss_threshold = 10  # frames
        self.morph_wait_timeout = 15.0   # s
        self.contact_wait_timeout = 30.0 # s
        self.throttle_ramp_duration = 5.0 # s
        self.video_recovery_timeout = 60.0 # s, max time to hover-wait for video recovery

        # ALIGN -> EXPAND altitude gate: require drone to be near hover_z before expanding arms
        self.altitude_check_tolerance = 0.15  # m, max |z - hover_z|
        self.altitude_check_hold_time = 1.0   # s, must stay within tolerance this long

        # Mission data
        self.point_a = None
        self.point_b = None
        self.push_yaw = None
        self.contact_triggered = False
        self.start_time = None
        self.phase_start_time = None
        self.last_pos = None
        self.last_pos_time = None
        self.vel = {"x": 0.0, "y": 0.0, "z": 0.0}

        # Staged execution: pause after this phase name (default = run to completion)
        # New perching flow is semi-autonomous: after contact PX4 takes over the
        # position setpoint and auto-retracts arms, then pauses for operator judgement.
        self.stop_after_phase = "CONTACT_WAIT"
        self.resume_event = threading.Event()

    def start(self):
        if self.running:
            self.ros.log_sig.emit("[MISSION] 任务已在运行")
            return
        self.running = True
        self.abort_requested = False
        self.phase = "IDLE"
        self.contact_triggered = False
        self.start_time = time.time()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        self.ros.log_sig.emit("[MISSION] 一键栖息任务启动")

    def abort(self):
        if not self.running:
            return
        self.abort_requested = True
        self.resume_event.set()  # wake hold loop so cleanup can run
        self.ros.log_sig.emit("[MISSION] 用户请求中止任务")

    def set_stop_after_phase(self, phase):
        """Set the phase after which the mission will pause and wait.

        Common values: "ALIGN", "EXPAND", "APPROACH", "BLIND_PUSH",
        "CONTACT_WAIT", "DONE".  Passing "DONE" means run to completion.
        """
        if phase not in self.PHASE_ORDER and phase != "DONE":
            self.ros.log_sig.emit(f"[MISSION] 无效的阶段: {phase}")
            return
        old = self.stop_after_phase
        self.stop_after_phase = phase
        if old != phase:
            self.ros.log_sig.emit(f"[MISSION] 暂停点设置为: {self.PHASE_NAMES.get(phase, phase)}")
        self.resume_event.set()  # wake hold loop if already paused

    def _maybe_hold(self, current_phase):
        """Pause here if the user requested to stop after current_phase."""
        if self.stop_after_phase != current_phase:
            return True
        hold_phase = f"HOLD_{current_phase}"
        self._set_phase(hold_phase)
        self.ros.log_sig.emit(f"[MISSION] 已暂停在 {self.PHASE_NAMES.get(current_phase, current_phase)}，可调整暂停点后继续")
        while not rospy.is_shutdown() and self.running and not self.abort_requested:
            if self.stop_after_phase != current_phase:
                self.resume_event.clear()
                self.ros.log_sig.emit(f"[MISSION] 继续执行，目标完成点: {self.PHASE_NAMES.get(self.stop_after_phase, self.stop_after_phase)}")
                return True
            self.resume_event.wait(timeout=0.2)
        return False

    def _set_phase(self, phase):
        if phase != self.phase:
            self.ros.log_sig.emit(f"[MISSION] {self.PHASE_NAMES.get(self.phase, self.phase)} -> {self.PHASE_NAMES.get(phase, phase)}")
        self.phase = phase
        self.phase_start_time = time.time()
        self.mission_sig.emit({"phase": phase, "phase_zh": self.PHASE_NAMES.get(phase, phase)})

    def _check_timeout(self, timeout):
        return (time.time() - self.phase_start_time) > timeout

    def _update_velocity(self):
        now = time.time()
        pos = self.ros.current_pos
        if self.last_pos is not None and (now - self.last_pos_time) > 0.0:
            dt = now - self.last_pos_time
            # Low-pass filter velocity to tolerate noisy/discrete pose updates (e.g. low telemetry rate).
            alpha = 0.3
            self.vel["x"] = alpha * (pos["x"] - self.last_pos["x"]) / dt + (1.0 - alpha) * self.vel["x"]
            self.vel["y"] = alpha * (pos["y"] - self.last_pos["y"]) / dt + (1.0 - alpha) * self.vel["y"]
            self.vel["z"] = alpha * (pos["z"] - self.last_pos["z"]) / dt + (1.0 - alpha) * self.vel["z"]
        self.last_pos = pos.copy()
        self.last_pos_time = now

    def _is_stable(self, target_z):
        pos = self.ros.current_pos
        # For takeoff, we mainly care about z and low velocity; xy origin drift tolerance is larger
        dz = abs(pos["z"] - target_z)
        dxy = math.hypot(pos["x"] - self.point_a["x"], pos["y"] - self.point_a["y"]) if self.point_a else 0.0
        vx = abs(self.vel["x"])
        vy = abs(self.vel["y"])
        vz = abs(self.vel["z"])
        return (dz < self.stable_threshold and dxy < 0.2 and
                vx < self.stable_velocity_threshold and vy < self.stable_velocity_threshold and vz < self.stable_velocity_threshold)

    def _wait_mode(self, target_mode, timeout=5.0):
        """Wait until MAVROS reports the requested flight mode."""
        rate = rospy.Rate(10)
        start = time.time()
        while not rospy.is_shutdown() and self.running and not self.abort_requested:
            if self.ros.current_state.get("mode") == target_mode:
                return True
            if (time.time() - start) > timeout:
                self.ros.log_sig.emit(f"[MISSION] 等待模式 {target_mode} 超时")
                return False
            rate.sleep()
        return False

    def _wait_stable(self, target_z, timeout=15.0):
        rate = rospy.Rate(10)
        stable_start = None
        while not rospy.is_shutdown() and self.running and not self.abort_requested:
            self._update_velocity()
            if self._is_stable(target_z):
                if stable_start is None:
                    stable_start = time.time()
                elif (time.time() - stable_start) >= self.stable_hold_time:
                    self.ros.log_sig.emit("[MISSION] 起飞到位并稳定")
                    return True
            else:
                stable_start = None
            if self._check_timeout(timeout):
                self.ros.log_sig.emit("[MISSION] 等待位姿稳定超时")
                return False
            rate.sleep()
        return False

    def _wait_video_recovery(self, timeout):
        """Hover in place and wait for the video stream to recover.

        Returns True on recovery, False if timeout or abort.
        Phase timeout is frozen while waiting.
        """
        self.ros.log_sig.emit("[MISSION] 视频流丢失，悬停等待恢复...")
        was_yaw_control = self.ros.yaw_control_active
        self.ros.set_yaw_control(False)
        # Freeze the current phase timeout so video loss does not consume alignment budget.
        elapsed_at_pause = time.time() - self.phase_start_time

        rate = rospy.Rate(10)
        start = time.time()
        while not rospy.is_shutdown() and self.running and not self.abort_requested:
            if self.ros.is_video_stream_alive() and self.ros.is_detection_active():
                self.ros.log_sig.emit("[MISSION] 视频流恢复")
                # Resume phase timeout from where it paused.
                self.phase_start_time = time.time() - elapsed_at_pause
                if was_yaw_control:
                    self.ros.set_yaw_control(True)
                return True
            if (time.time() - start) > timeout:
                self.ros.log_sig.emit("[MISSION] 视频流恢复超时")
                return False
            # Hold current position while waiting.
            current = self.ros.current_pos
            self.ros.update_setpoint(x=current["x"], y=current["y"], z=current["z"], yaw=current["yaw"])
            rate.sleep()
        return False

    def _wait_detection_recovery(self, timeout):
        """Hover in place and wait for YOLO to detect a target again.

        This is used when the video node is alive but no target is seen,
        e.g. during a camera voltage-sag reboot.  Phase timeout is frozen.
        Returns True on recovery, False if timeout or abort.
        """
        self.ros.log_sig.emit("[MISSION] 未检测到目标，悬停等待相机/检测恢复...")
        was_yaw_control = self.ros.yaw_control_active
        self.ros.set_yaw_control(False)
        elapsed_at_pause = time.time() - self.phase_start_time

        rate = rospy.Rate(10)
        start = time.time()
        while not rospy.is_shutdown() and self.running and not self.abort_requested:
            if self.ros.is_video_stream_alive() and self.ros.detections_count > 0:
                self.ros.log_sig.emit("[MISSION] 目标检测恢复")
                self.phase_start_time = time.time() - elapsed_at_pause
                if was_yaw_control:
                    self.ros.set_yaw_control(True)
                return True
            if (time.time() - start) > timeout:
                self.ros.log_sig.emit("[MISSION] 目标检测恢复超时")
                return False
            current = self.ros.current_pos
            self.ros.update_setpoint(x=current["x"], y=current["y"], z=current["z"], yaw=current["yaw"])
            rate.sleep()
        return False

    def _wait_yaw_aligned(self, timeout=60.0):
        rate = rospy.Rate(10)
        aligned_start = None
        no_det_start = None
        while not rospy.is_shutdown() and self.running and not self.abort_requested:
            # If video stream is lost, hover and wait rather than aborting immediately.
            if not self.ros.is_video_stream_alive():
                if not self._wait_video_recovery(self.video_recovery_timeout):
                    self.ros.log_sig.emit("[MISSION] YAW 对齐过程中视频无法恢复，中止")
                    return False
                aligned_start = None
                no_det_start = None
                continue

            # If the stream is alive but no target is seen for a while, pause the
            # timeout as well.  This covers camera voltage-sag reboots where the
            # YOLO node keeps publishing heartbeat/pixel_error=0 but has no real
            # detection.
            if not self.ros.yaw_aligned and self.ros.detections_count == 0:
                if no_det_start is None:
                    no_det_start = time.time()
                elif (time.time() - no_det_start) >= self.align_no_det_pause:
                    if not self._wait_detection_recovery(self.video_recovery_timeout):
                        self.ros.log_sig.emit("[MISSION] YAW 对齐过程中目标检测无法恢复，中止")
                        return False
                    aligned_start = None
                    no_det_start = None
                    continue
            else:
                no_det_start = None

            if self.ros.yaw_aligned:
                if aligned_start is None:
                    aligned_start = time.time()
                elif (time.time() - aligned_start) >= self.yaw_align_hold_time:
                    return True
            else:
                aligned_start = None
            if self._check_timeout(timeout):
                self.ros.log_sig.emit("[MISSION] YAW 对齐超时")
                return False
            rate.sleep()
        return False

    def _wait_altitude(self, target_z, tolerance=0.15, hold_time=1.0, timeout=15.0):
        """Wait until the drone's altitude is within tolerance of target_z for hold_time seconds.

        This guards against cases where the drone loses height during ALIGN (e.g. due to
        yaw motion or setpoint conflicts) before proceeding to EXPAND/approach.
        """
        rate = rospy.Rate(10)
        stable_start = None
        self.ros.log_sig.emit(f"[MISSION] 等待高度恢复到 {target_z:.2f}m (±{tolerance:.2f}m)...")
        while not rospy.is_shutdown() and self.running and not self.abort_requested:
            z = self.ros.current_pos.get("z", 0.0)
            dz = abs(z - target_z)
            if dz <= tolerance:
                if stable_start is None:
                    stable_start = time.time()
                elif (time.time() - stable_start) >= hold_time:
                    self.ros.log_sig.emit(f"[MISSION] 高度已稳定: z={z:.2f}m")
                    return True
            else:
                if stable_start is not None:
                    self.ros.log_sig.emit(f"[MISSION] 高度偏离: z={z:.2f}m, dz={dz:.2f}m, 重新等待...")
                stable_start = None
            if self._check_timeout(timeout):
                self.ros.log_sig.emit(f"[MISSION] 等待高度恢复超时: z={z:.2f}m, target={target_z:.2f}m")
                return False
            rate.sleep()
        return False

    def _wait_morph(self, target_angle, timeout=15.0):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and self.running and not self.abort_requested:
            angle = self.ros.morph_angle_rad
            if angle is not None and abs(angle - target_angle) < 0.05:
                return True
            if self._check_timeout(timeout):
                self.ros.log_sig.emit("[MISSION] 等待机臂到位超时")
                return False
            rate.sleep()
        return False

    def _set_pc_en_full(self):
        # Ensure MORPH_EN is enabled first
        if self.ros.morph_en != 1:
            self.ros.set_morph_en(True)
            rospy.sleep(1.0)
        self.ros.set_pc_en(2)

    def _move_forward_body(self, speed, duration):
        """Move forward in body frame at given speed for duration seconds."""
        rate = rospy.Rate(20)
        start = time.time()
        while not rospy.is_shutdown() and self.running and not self.abort_requested:
            elapsed = time.time() - start
            if elapsed > duration:
                break
            yaw = self.ros.current_pos.get("yaw", 0.0)
            dt = 1.0 / 20.0
            dx_body = speed * dt
            self.ros.move_body(d_forward=dx_body)
            rate.sleep()

    def _run(self):
        try:
            self._execute()
        except Exception as e:
            self.ros.log_sig.emit(f"[MISSION] 异常: {e}")
            import traceback
            self.ros.log_sig.emit(traceback.format_exc())
        finally:
            self._cleanup()

    def _execute(self):
        # Phase IDLE -> TAKEOFF
        self._set_phase("TAKEOFF")

        # Pre-checks
        if not self.ros.current_state.get("connected", False):
            self.ros.log_sig.emit("[MISSION] 飞控未连接，中止")
            self._set_phase("ABORT")
            return

        # Reduce MAVLink downlink rates to free HM30 bandwidth for RTSP video
        self.ros.set_message_intervals()

        # Disable rtsp_pillar_node cmd_vel, take over
        self.ros.disable_rtsp_cmd_vel()
        self.ros.set_lock_target(-1)

        # Takeoff: pre-publish setpoint, then OFFBOARD, then ARM
        takeoff_x = self.ros.current_pos["x"]
        takeoff_y = self.ros.current_pos["y"]
        self.ros.update_setpoint(x=takeoff_x, y=takeoff_y, z=self.hover_z, yaw=self.ros.current_pos["yaw"])
        self.ros.set_sp_active(True)

        # PX4 requires setpoints to be streamed before arming in OFFBOARD
        self.ros.log_sig.emit("[MISSION] 预发布 setpoint 2s，等待 OFFBOARD 准备...")
        rospy.sleep(2.0)

        if not self.ros.set_mode("OFFBOARD"):
            self.ros.log_sig.emit("[MISSION] 无法切换到 OFFBOARD，中止")
            self._set_phase("ABORT")
            return
        if not self._wait_mode("OFFBOARD", timeout=5.0):
            self.ros.log_sig.emit("[MISSION] 未确认 OFFBOARD 模式，中止")
            self._set_phase("ABORT")
            return

        if not self.ros.current_state.get("armed", False):
            self.ros.log_sig.emit("[MISSION] 执行 ARM...")
            self.ros.arm(True)
            rospy.sleep(0.5)

        # Wait until stable at hover_z (be tolerant to voltage-sag induced oscillation)
        if not self._wait_stable(self.hover_z, timeout=45.0):
            self._set_phase("ABORT")
            return

        self.ros.log_sig.emit("[MISSION] 记录 point A")
        self.point_a = self.ros.current_pos.copy()
        if not self._maybe_hold("TAKEOFF"):
            return

        # Phase ALIGN
        self._set_phase("ALIGN")
        self.ros.set_yaw_control(True)
        if not self._wait_yaw_aligned(timeout=self.align_timeout):
            self._set_phase("ABORT")
            return
        self.point_a = self.ros.current_pos.copy()
        self.push_yaw = self.point_a["yaw"]
        self.ros.log_sig.emit(f"[MISSION] YAW 对齐完成，point A = ({self.point_a['x']:.2f}, {self.point_a['y']:.2f}, {self.point_a['z']:.2f})")
        if not self._maybe_hold("ALIGN"):
            return

        # ALIGN -> EXPAND altitude gate: do not expand arms until height is recovered.
        if not self._wait_altitude(self.hover_z, tolerance=self.altitude_check_tolerance,
                                   hold_time=self.altitude_check_hold_time, timeout=15.0):
            self._set_phase("ABORT")
            return

        # Phase EXPAND
        self._set_phase("EXPAND")
        self.ros.send_morph_angle(self.expand_angle)
        self._set_pc_en_full()  # enable contact detection + auto close while expanding
        if not self._wait_morph(self.expand_angle, timeout=self.morph_wait_timeout):
            self._set_phase("ABORT")
            return
        self.ros.log_sig.emit("[MISSION] 机臂展开到位")
        if not self._maybe_hold("EXPAND"):
            return

        # Phase APPROACH
        self._set_phase("APPROACH")
        self.ros.log_sig.emit("[MISSION] 开始缓慢接近，启用接触检测")

        loss_count = 0
        rate = rospy.Rate(20)
        approach_start = time.time()
        while not rospy.is_shutdown() and self.running and not self.abort_requested:
            self._update_velocity()

            # If the entire video stream drops, hover and wait instead of blind-pushing.
            if not self.ros.is_video_stream_alive():
                if not self._wait_video_recovery(self.video_recovery_timeout):
                    self.ros.log_sig.emit("[MISSION] 接近阶段视频无法恢复，中止")
                    self._set_phase("ABORT")
                    return
                loss_count = 0
                continue

            # Check detection active
            if not (self.ros.detection_active and (time.time() - self.ros.detection_last_time) < 0.5):
                loss_count += 1
            else:
                loss_count = 0

            if loss_count >= self.detection_loss_threshold:
                self.point_b = self.ros.current_pos.copy()
                self.push_yaw = self.point_b["yaw"]
                self.ros.log_sig.emit(f"[MISSION] YOLO 检测丢失，记录 point B = ({self.point_b['x']:.2f}, {self.point_b['y']:.2f}), 进入盲推")
                break

            # Safety: if contact happens while the pole is still visible, stop approaching immediately.
            state = self.ros.contact_state.get("state", "NO_CONTACT")
            if state in PERCHING_CONTACT_STATES:
                self.contact_triggered = True
                self.point_b = self.ros.current_pos.copy()
                self.push_yaw = self.point_b["yaw"]
                self.ros.log_sig.emit(f"[MISSION] 接近阶段已触发接触: {state}，停止前进")
                break

            # Also advance forward slowly while aligning (direct setpoint update, no log spam)
            yaw = self.ros.current_pos.get("yaw", 0.0)
            dt = 1.0 / 20.0
            dx_body = self.approach_speed * dt
            with self.ros.sp_lock:
                self.ros.sp["x"] += dx_body * math.cos(yaw)
                self.ros.sp["y"] += dx_body * math.sin(yaw)
                self.ros.sp_active = True

            if self._check_timeout(self.phase_timeout):
                self.ros.log_sig.emit("[MISSION] 接近阶段超时")
                self._set_phase("ABORT")
                return

            rate.sleep()

        if not self._maybe_hold("APPROACH"):
            return

        # Phase BLIND_PUSH
        self._set_phase("BLIND_PUSH")
        self.ros.set_yaw_control(False)
        self.ros.log_sig.emit("[MISSION] 盲推阶段：保持 A->B 方向")

        # Compute direction from A to B
        if self.point_a is None:
            self.point_a = self.ros.current_pos.copy()
        if self.point_b is None:
            self.point_b = self.ros.current_pos.copy()
        push_dir_x = self.point_b["x"] - self.point_a["x"]
        push_dir_y = self.point_b["y"] - self.point_a["y"]
        dist = math.hypot(push_dir_x, push_dir_y)
        if dist > 0.01:
            push_dir_x /= dist
            push_dir_y /= dist
        else:
            # Fallback: use current yaw
            yaw = self.push_yaw if self.push_yaw is not None else self.ros.current_pos["yaw"]
            push_dir_x = math.cos(yaw)
            push_dir_y = math.sin(yaw)

        # Set setpoint yaw to push direction
        self.push_yaw = math.atan2(push_dir_y, push_dir_x)
        self.ros.update_setpoint(yaw=self.push_yaw)

        while not rospy.is_shutdown() and self.running and not self.abort_requested:
            self._update_velocity()

            # Advance setpoint along push direction
            dt = 1.0 / 20.0
            step = self.approach_speed * dt
            with self.ros.sp_lock:
                self.ros.sp["x"] += push_dir_x * step
                self.ros.sp["y"] += push_dir_y * step
                self.ros.sp_active = True

            # Check contact
            state = self.ros.contact_state.get("state", "NO_CONTACT")
            if state in PERCHING_CONTACT_STATES:
                self.ros.log_sig.emit(f"[MISSION] 接触触发: {state}")
                break

            if self._check_timeout(self.phase_timeout):
                self.ros.log_sig.emit("[MISSION] 盲推阶段超时")
                self._set_phase("ABORT")
                return

            rate.sleep()

        if not self._maybe_hold("BLIND_PUSH"):
            return

        # Phase CONTACT_WAIT
        self._set_phase("CONTACT_WAIT")
        self.ros.log_sig.emit("[MISSION] 接触触发，PX4 接管 setpoint；等待机臂收拢")
        rate = rospy.Rate(10)

        # Hold the last setpoint so the offboard link stays alive, but PX4 will
        # override it with the recorded contact point + preload every control cycle.
        current = self.ros.current_pos
        self.ros.update_setpoint(x=current["x"], y=current["y"], z=current["z"], yaw=self.push_yaw)

        while not rospy.is_shutdown() and self.running and not self.abort_requested:
            angle = self.ros.morph_angle_rad
            if angle is not None and angle >= -0.05:
                self.ros.log_sig.emit(f"[MISSION] 机臂已收拢 (angle={angle:.3f} rad)，等待操作员判断")
                break
            if self._check_timeout(self.contact_wait_timeout):
                self.ros.log_sig.emit("[MISSION] 等待机臂收拢超时")
                self._set_phase("ABORT")
                return
            rate.sleep()

        if not self._maybe_hold("CONTACT_WAIT"):
            return

        # Default semi-autonomous flow stops here.  Operator must explicitly choose
        # to continue (e.g. to THROTTLE_RAMP/DISARM) or perform manual disarm.

        # Phase THROTTLE_RAMP
        self._set_phase("THROTTLE_RAMP")
        self.ros.log_sig.emit("[MISSION] 油门逐渐下降")
        # IMPORTANT: exit PX4 full perching FSM before we lower the z setpoint,
        # otherwise PX4 keeps overriding the 3D setpoint to the contact point.
        self.ros.set_pc_en(1)
        rate = rospy.Rate(20)

        # Send descending throttle via cmd_vel (body z-down is positive in FRD? Actually cmd_vel z is up positive in ENU for MAVROS)
        # Use setpoint z decreasing slowly instead to avoid throttle semantics confusion
        ramp_start = time.time()
        start_z = self.ros.current_pos["z"]
        # Descend 0.5m over ramp duration to reduce throttle (Z+ is up)
        end_z = start_z - 0.5
        while not rospy.is_shutdown() and self.running and not self.abort_requested:
            elapsed = time.time() - ramp_start
            if elapsed >= self.throttle_ramp_duration:
                break
            ratio = elapsed / self.throttle_ramp_duration
            new_z = start_z + (end_z - start_z) * ratio
            self.ros.update_setpoint(z=new_z)
            rate.sleep()

        # Phase DISARM
        self._set_phase("DISARM")
        self.ros.log_sig.emit("[MISSION] 切换自稳并锁定")
        rate = rospy.Rate(10)
        self.ros.set_mode("STABILIZED")
        rospy.sleep(1.0)
        self.ros.arm(False)
        self._set_phase("DONE")
        self.ros.log_sig.emit("[MISSION] 任务完成")

    def _cleanup(self):
        self.ros.set_yaw_control(False)
        self.ros.enable_rtsp_cmd_vel()
        self.running = False
        if self.phase not in ("DONE", "ABORT"):
            self._set_phase("ABORT")

        # Safety fallback: if aborted mid-air, switch to POSCTL and retract arms
        if self.phase == "ABORT":
            self.ros.log_sig.emit("[MISSION] 中止：切 POSCTL 并收拢机臂")
            # Release PX4 perching controller so it stops overriding the setpoint.
            if getattr(self.ros, "pc_en", 0) >= 2:
                self.ros.set_pc_en(1)
            self.ros.set_mode("POSCTL")
            self.ros.send_morph_angle(0.0)

        self.ros.log_sig.emit("[MISSION] 任务线程结束")



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
        self.ros.vision_sig.connect(self._on_vision)
        self.ros.image_sig.connect(self._on_image)
        self.ros.mission_sig.connect(self._on_mission)

        self.mission = PerchingMission(self.ros)

        self._build_ui()
        
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._update_sp_display)
        self.ui_timer.start(100)
    
    def _build_ui(self):
        self.setWindowTitle("huaqiccc 自主栖息地面站 v2.1")
        self.setMinimumSize(1200, 800)
        self.resize(1400, 900)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setSpacing(6)
        main_layout.setContentsMargins(8, 8, 8, 8)

        font_mono = QFont("Monospace", 9)
        font_mono.setStyleHint(QFont.Monospace)

        splitter = QSplitter(Qt.Horizontal)

        # --- Left panel: camera + vision/mission + YAW params ---
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setSpacing(6)
        left_layout.setContentsMargins(0, 0, 0, 0)

        # Camera view
        cam_group = QGroupBox("摄像机视角 /yolo/detection_image")
        cam_layout = QVBoxLayout(cam_group)
        cam_layout.setContentsMargins(4, 4, 4, 4)
        self.lbl_camera = QLabel("等待图像...")
        self.lbl_camera.setAlignment(Qt.AlignCenter)
        self.lbl_camera.setMinimumSize(480, 270)
        self.lbl_camera.setStyleSheet("background-color: #1a1a1a; color: #888;")
        self.lbl_camera.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        cam_layout.addWidget(self.lbl_camera)
        left_layout.addWidget(cam_group, 4)

        # Vision / Mission status
        status_group = QGroupBox("视觉 / 任务状态")
        status_layout = QVBoxLayout(status_group)
        status_layout.setContentsMargins(6, 6, 6, 6)
        self.lbl_vision_state = QLabel("视觉: 未检测 | 对齐: 否 | 误差: 0.000")
        self.lbl_vision_state.setFont(font_mono)
        self.lbl_mission_phase = QLabel("阶段: 空闲")
        self.lbl_mission_phase.setStyleSheet("font-weight: bold; font-size: 13px;")
        status_layout.addWidget(self.lbl_mission_phase)
        status_layout.addWidget(self.lbl_vision_state)
        left_layout.addWidget(status_group, 0)

        # YAW params
        yaw_group = QGroupBox("YAW 对齐参数")
        yaw_layout = QGridLayout(yaw_group)
        yaw_layout.setContentsMargins(6, 6, 6, 6)
        yaw_layout.setVerticalSpacing(4)
        yaw_layout.setHorizontalSpacing(8)

        yaw_layout.addWidget(QLabel("相机偏移 X (px):"), 0, 0)
        self.in_camera_offset_x = QLineEdit(f"{self.ros.camera_offset_x:.1f}")
        self.in_camera_offset_x.setFont(font_mono)
        yaw_layout.addWidget(self.in_camera_offset_x, 0, 1)

        btn_calib = QPushButton("标定")
        btn_calib.setToolTip("当前机头正对柱子时，点击将当前 pixel_error 记录为偏移")
        btn_calib.clicked.connect(self._calibrate_camera_offset)
        yaw_layout.addWidget(btn_calib, 0, 2)

        yaw_layout.addWidget(QLabel("Kp:"), 1, 0)
        self.in_yaw_kp = QLineEdit("1.5")
        self.in_yaw_kp.setFont(font_mono)
        yaw_layout.addWidget(self.in_yaw_kp, 1, 1)

        yaw_layout.addWidget(QLabel("最大角速度:"), 1, 2)
        self.in_yaw_max_rate = QLineEdit("0.3")
        self.in_yaw_max_rate.setFont(font_mono)
        yaw_layout.addWidget(self.in_yaw_max_rate, 1, 3)

        yaw_layout.addWidget(QLabel("死区:"), 2, 0)
        self.in_yaw_deadzone = QLineEdit("0.08")
        self.in_yaw_deadzone.setFont(font_mono)
        yaw_layout.addWidget(self.in_yaw_deadzone, 2, 1)

        yaw_layout.addWidget(QLabel("跳变限制:"), 2, 2)
        self.in_yaw_max_delta = QLineEdit("0.15")
        self.in_yaw_max_delta.setFont(font_mono)
        yaw_layout.addWidget(self.in_yaw_max_delta, 2, 3)

        btn_update_yaw = QPushButton("应用参数")
        btn_update_yaw.clicked.connect(self._update_yaw_params)
        yaw_layout.addWidget(btn_update_yaw, 3, 0, 1, 4)

        left_layout.addWidget(yaw_group, 0)

        # --- Right panel: status + mission + morph + contact + pos/ctrl + log ---
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setSpacing(6)
        right_layout.setContentsMargins(0, 0, 0, 0)

        # Top status bar
        status_bar = QGroupBox("飞控状态")
        status_layout2 = QHBoxLayout(status_bar)
        status_layout2.setContentsMargins(6, 4, 6, 4)
        self.lbl_conn = QLabel("<span style='color:gray'>●</span> 未连接")
        self.lbl_mode = QLabel("模式: --")
        self.lbl_arm = QLabel("锁定")
        self.lbl_batt = QLabel("电池: --")
        status_layout2.addWidget(self.lbl_conn)
        status_layout2.addWidget(self.lbl_mode)
        status_layout2.addWidget(self.lbl_arm)
        status_layout2.addStretch()
        status_layout2.addWidget(self.lbl_batt)
        right_layout.addWidget(status_bar, 0)

        # Mission panel with phase selector
        mission_group = QGroupBox("自主栖息任务")
        mission_layout = QGridLayout(mission_group)
        mission_layout.setContentsMargins(6, 6, 6, 6)
        mission_layout.setVerticalSpacing(4)

        mission_layout.addWidget(QLabel("执行到阶段:"), 0, 0)
        self.combo_max_phase = QComboBox()
        self.combo_max_phase.addItem("1-2 起飞并对齐（记录A）", "ALIGN")
        self.combo_max_phase.addItem("3 展开机臂", "EXPAND")
        self.combo_max_phase.addItem("4 视觉接近", "APPROACH")
        self.combo_max_phase.addItem("5 盲推（记录B）", "BLIND_PUSH")
        self.combo_max_phase.addItem("6 接触等待", "CONTACT_WAIT")
        self.combo_max_phase.addItem("7 油门下降并锁定", "DONE")
        mission_layout.addWidget(self.combo_max_phase, 0, 1, 1, 3)

        btn_mission_start = QPushButton("开始")
        btn_mission_start.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 4px 12px;")
        btn_mission_start.clicked.connect(self._on_mission_start)
        mission_layout.addWidget(btn_mission_start, 0, 4)

        btn_mission_continue = QPushButton("继续")
        btn_mission_continue.setToolTip("继续执行到当前选择的阶段")
        btn_mission_continue.setStyleSheet("background-color: #2196F3; color: white; padding: 4px 12px;")
        btn_mission_continue.clicked.connect(self._on_mission_continue)
        mission_layout.addWidget(btn_mission_continue, 0, 5)

        btn_mission_abort = QPushButton("中止")
        btn_mission_abort.setStyleSheet("background-color: #f44336; color: white; padding: 4px 12px;")
        btn_mission_abort.clicked.connect(self._on_mission_abort)
        mission_layout.addWidget(btn_mission_abort, 0, 6)

        right_layout.addWidget(mission_group, 0)

        # Morph control
        morph_group = QGroupBox("变形控制")
        morph_layout = QGridLayout(morph_group)
        morph_layout.setContentsMargins(6, 6, 6, 6)
        morph_layout.setVerticalSpacing(4)

        self.lbl_morph_en = QLabel("模块状态: 未知")
        self.lbl_morph_en.setStyleSheet("font-weight: bold;")
        morph_layout.addWidget(self.lbl_morph_en, 0, 0)

        self.lbl_morph_angle = QLabel("当前: --")
        self.lbl_morph_angle.setFont(font_mono)
        morph_layout.addWidget(self.lbl_morph_angle, 0, 1)

        self.lbl_morph_target = QLabel("目标: --")
        self.lbl_morph_target.setFont(font_mono)
        morph_layout.addWidget(self.lbl_morph_target, 0, 2)

        self.slider_morph = QSlider(Qt.Horizontal)
        self.slider_morph.setMinimum(0)
        self.slider_morph.setMaximum(21)
        self.slider_morph.setValue(0)
        self.slider_morph.setTickPosition(QSlider.TicksBelow)
        self.slider_morph.setTickInterval(5)
        self.slider_morph.valueChanged.connect(self._on_morph_slider)
        morph_layout.addWidget(QLabel("角度:"), 1, 0)
        morph_layout.addWidget(self.slider_morph, 1, 1, 1, 2)

        self.lbl_slider_val = QLabel("0° (收拢)")
        self.lbl_slider_val.setFont(font_mono)
        morph_layout.addWidget(self.lbl_slider_val, 1, 3)

        self.in_morph_deg = QLineEdit("0.0")
        self.in_morph_deg.setFont(font_mono)
        morph_layout.addWidget(QLabel("精确(°):"), 2, 0)
        morph_layout.addWidget(self.in_morph_deg, 2, 1)

        btn_morph_send = QPushButton("执行")
        btn_morph_send.clicked.connect(self._on_morph_send)
        btn_morph_send.setStyleSheet("background-color: #9C27B0; color: white;")
        morph_layout.addWidget(btn_morph_send, 2, 2)

        btn_morph_close = QPushButton("收拢")
        btn_morph_close.clicked.connect(lambda: self._set_morph_preset(0.0))
        morph_layout.addWidget(btn_morph_close, 2, 3)

        btn_morph_open = QPushButton("展开")
        btn_morph_open.clicked.connect(lambda: self._set_morph_preset(-21.0))
        morph_layout.addWidget(btn_morph_open, 2, 4)

        btn_morph_en_on = QPushButton("启用")
        btn_morph_en_on.clicked.connect(lambda: self.ros.set_morph_en(True))
        btn_morph_en_on.setStyleSheet("background-color: #4CAF50; color: white;")
        morph_layout.addWidget(btn_morph_en_on, 3, 0)

        btn_morph_en_off = QPushButton("禁用")
        btn_morph_en_off.clicked.connect(lambda: self.ros.set_morph_en(False))
        btn_morph_en_off.setStyleSheet("background-color: #f44336; color: white;")
        morph_layout.addWidget(btn_morph_en_off, 3, 1)

        btn_morph_refresh = QPushButton("刷新")
        btn_morph_refresh.clicked.connect(self._on_morph_refresh)
        morph_layout.addWidget(btn_morph_refresh, 3, 2)

        right_layout.addWidget(morph_group, 0)

        # Contact state
        contact_group = QGroupBox("接触 / 栖落状态")
        contact_layout = QVBoxLayout(contact_group)
        contact_layout.setContentsMargins(6, 6, 6, 6)
        contact_layout.setSpacing(2)
        self.lbl_contact_state = QLabel("<span style='color:gray;font-size:14px;font-weight:bold'>● 未接触</span>")
        self.lbl_contact_detail = QLabel("err: --  vel: --  pitch: --")
        self.lbl_contact_detail.setFont(font_mono)
        self.lbl_contact_diag = QLabel("dx: --  dz: --  elapsed: --")
        self.lbl_contact_diag.setFont(font_mono)
        self.btn_perch_en = QPushButton("自主抱柱: --")
        self.btn_perch_en.setStyleSheet("font-weight: bold; padding: 4px;")
        self.btn_perch_en.setToolTip("切换 MPCA_PC_EN：1=仅检测/记录，2=启用自主栖停 FSM")
        self.btn_perch_en.clicked.connect(self._toggle_perching_en)
        contact_layout.addWidget(self.lbl_contact_state)
        contact_layout.addWidget(self.lbl_contact_detail)
        contact_layout.addWidget(self.lbl_contact_diag)
        contact_layout.addWidget(self.btn_perch_en)
        right_layout.addWidget(contact_group, 0)

        # Position / setpoint
        pos_group = QGroupBox("位置 / 目标")
        pos_layout = QGridLayout(pos_group)
        pos_layout.setContentsMargins(6, 6, 6, 6)
        pos_layout.setVerticalSpacing(2)

        self.lbl_px = QLabel("X: 0.000"); self.lbl_px.setFont(font_mono)
        self.lbl_py = QLabel("Y: 0.000"); self.lbl_py.setFont(font_mono)
        self.lbl_pz = QLabel("Z: 0.000"); self.lbl_pz.setFont(font_mono)
        self.lbl_pyaw = QLabel("Yaw: 0.0°"); self.lbl_pyaw.setFont(font_mono)
        pos_layout.addWidget(self.lbl_px, 0, 0)
        pos_layout.addWidget(self.lbl_py, 0, 1)
        pos_layout.addWidget(self.lbl_pz, 0, 2)
        pos_layout.addWidget(self.lbl_pyaw, 0, 3)

        self.lbl_spx = QLabel("tX: 0.000"); self.lbl_spx.setFont(font_mono)
        self.lbl_spy = QLabel("tY: 0.000"); self.lbl_spy.setFont(font_mono)
        self.lbl_spz = QLabel("tZ: 0.000"); self.lbl_spz.setFont(font_mono)
        self.lbl_spyaw = QLabel("tYaw: 0.0°"); self.lbl_spyaw.setFont(font_mono)
        pos_layout.addWidget(self.lbl_spx, 1, 0)
        pos_layout.addWidget(self.lbl_spy, 1, 1)
        pos_layout.addWidget(self.lbl_spz, 1, 2)
        pos_layout.addWidget(self.lbl_spyaw, 1, 3)

        pos_layout.addWidget(QLabel("X:"), 2, 0)
        self.in_x = QLineEdit("0.0"); self.in_x.setFont(font_mono)
        pos_layout.addWidget(self.in_x, 2, 1)
        pos_layout.addWidget(QLabel("Y:"), 2, 2)
        self.in_y = QLineEdit("0.0"); self.in_y.setFont(font_mono)
        pos_layout.addWidget(self.in_y, 2, 3)

        pos_layout.addWidget(QLabel("Z:"), 3, 0)
        self.in_z = QLineEdit("1.0"); self.in_z.setFont(font_mono)
        pos_layout.addWidget(self.in_z, 3, 1)

        btn_goto = QPushButton("前往")
        btn_goto.clicked.connect(self._on_goto)
        btn_goto.setStyleSheet("background-color: #2196F3; color: white;")
        pos_layout.addWidget(btn_goto, 3, 2, 1, 2)

        right_layout.addWidget(pos_group, 0)

        # Controls
        ctrl_group = QGroupBox("控制")
        ctrl_layout = QGridLayout(ctrl_group)
        ctrl_layout.setContentsMargins(6, 6, 6, 6)
        ctrl_layout.setVerticalSpacing(4)
        ctrl_layout.setHorizontalSpacing(4)

        btn_arm = QPushButton("ARM"); btn_arm.clicked.connect(lambda: self.ros.arm(True)); btn_arm.setStyleSheet("background-color: #4CAF50; color: white;")
        btn_disarm = QPushButton("DISARM"); btn_disarm.clicked.connect(lambda: self.ros.arm(False)); btn_disarm.setStyleSheet("background-color: #f44336; color: white;")
        btn_posctl = QPushButton("POSCTL"); btn_posctl.clicked.connect(lambda: self.ros.set_mode("POSCTL"))
        btn_manual = QPushButton("MANUAL"); btn_manual.clicked.connect(lambda: self.ros.set_mode("MANUAL"))
        btn_offboard = QPushButton("OFFBOARD"); btn_offboard.clicked.connect(lambda: self.ros.set_mode("OFFBOARD"))
        btn_takeoff = QPushButton(f"起飞{PERCHING_HOVER_Z}m"); btn_takeoff.clicked.connect(self._on_takeoff); btn_takeoff.setStyleSheet("background-color: #FF9800; color: white;")
        btn_land = QPushButton("降落"); btn_land.clicked.connect(self._on_land)
        btn_estop = QPushButton("急停"); btn_estop.clicked.connect(self._on_estop); btn_estop.setStyleSheet("background-color: #B71C1C; color: white;")
        btn_px4 = QPushButton("PX4"); btn_px4.clicked.connect(lambda: self.ros.set_controller(0))
        btn_gpid = QPushButton("GPID"); btn_gpid.clicked.connect(lambda: self.ros.set_controller(1))
        btn_lqr = QPushButton("LQR"); btn_lqr.clicked.connect(lambda: self.ros.set_controller(2))
        btn_mpc = QPushButton("MPC"); btn_mpc.clicked.connect(lambda: self.ros.set_controller(3))

        ctrl_layout.addWidget(btn_arm, 0, 0)
        ctrl_layout.addWidget(btn_disarm, 0, 1)
        ctrl_layout.addWidget(btn_posctl, 0, 2)
        ctrl_layout.addWidget(btn_manual, 0, 3)
        ctrl_layout.addWidget(btn_offboard, 1, 0)
        ctrl_layout.addWidget(btn_takeoff, 1, 1)
        ctrl_layout.addWidget(btn_land, 1, 2)
        ctrl_layout.addWidget(btn_estop, 1, 3)
        ctrl_layout.addWidget(btn_px4, 2, 0)
        ctrl_layout.addWidget(btn_gpid, 2, 1)
        ctrl_layout.addWidget(btn_lqr, 2, 2)
        ctrl_layout.addWidget(btn_mpc, 2, 3)

        right_layout.addWidget(ctrl_group, 0)

        # Directional control
        dir_group = QGroupBox("方向控制 (机体坐标系)")
        dir_layout = QGridLayout(dir_group)
        dir_layout.setContentsMargins(6, 6, 6, 6)
        dir_layout.setVerticalSpacing(4)
        dir_layout.setHorizontalSpacing(4)

        dir_layout.addWidget(QLabel("平移:"), 0, 0)
        self.in_step_xy = QLineEdit("0.30"); self.in_step_xy.setFont(font_mono)
        dir_layout.addWidget(self.in_step_xy, 0, 1)
        dir_layout.addWidget(QLabel("垂直:"), 0, 2)
        self.in_step_z = QLineEdit("0.20"); self.in_step_z.setFont(font_mono)
        dir_layout.addWidget(self.in_step_z, 0, 3)
        dir_layout.addWidget(QLabel("旋转°:"), 0, 4)
        self.in_step_yaw = QLineEdit("15.0"); self.in_step_yaw.setFont(font_mono)
        dir_layout.addWidget(self.in_step_yaw, 0, 5)

        btn_f = QPushButton("前")
        btn_b = QPushButton("后")
        btn_l = QPushButton("左")
        btn_r = QPushButton("右")
        btn_u = QPushButton("上")
        btn_d = QPushButton("下")
        btn_cw = QPushButton("顺")
        btn_ccw = QPushButton("逆")

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

        dir_layout.addWidget(btn_f, 1, 1)
        dir_layout.addWidget(btn_l, 2, 0)
        dir_layout.addWidget(btn_r, 2, 2)
        dir_layout.addWidget(btn_b, 3, 1)
        dir_layout.addWidget(btn_u, 1, 4)
        dir_layout.addWidget(btn_d, 2, 4)
        dir_layout.addWidget(btn_cw, 1, 5)
        dir_layout.addWidget(btn_ccw, 3, 5)

        right_layout.addWidget(dir_group, 0)

        # Log
        self.log_edit = QPlainTextEdit()
        self.log_edit.setReadOnly(True)
        self.log_edit.setMaximumBlockCount(300)
        self.log_edit.setMaximumHeight(160)
        right_layout.addWidget(self.log_edit, 0)

        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setSizes([700, 700])
        main_layout.addWidget(splitter)

        self.show()

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
        self.ros.update_setpoint(z=PERCHING_HOVER_Z, relative=False)
        self.ros.set_sp_active(True)
        self.ros.set_mode("OFFBOARD")
        self._on_log(f"Takeoff to {PERCHING_HOVER_Z}m + OFFBOARD")
    
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
    
    def _on_vision(self, state):
        aligned = "YES" if state.get("yaw_aligned") else "NO"
        det = "检测中" if state.get("detection_active") else "未检测"
        err = state.get("pixel_error", 0.0)
        self.lbl_vision_state.setText(f"视觉: {det} | 对齐: {aligned} | 误差: {err:+.3f}")

    def _on_mission(self, state):
        phase = state.get("phase", "IDLE")
        phase_zh = state.get("phase_zh", phase)
        self.lbl_mission_phase.setText(f"阶段: {phase_zh}")

    def _update_yaw_params(self):
        try:
            self.ros.camera_offset_x = float(self.in_camera_offset_x.text())
            self.ros.yaw_Kp = float(self.in_yaw_kp.text())
            self.ros.yaw_max_rate = float(self.in_yaw_max_rate.text())
            self.ros.yaw_deadzone = float(self.in_yaw_deadzone.text())
            self.ros.yaw_rate_max_delta = float(self.in_yaw_max_delta.text())
            self._on_log(f"[VISION] YAW 参数更新: offset_x={self.ros.camera_offset_x:.1f}, Kp={self.ros.yaw_Kp:.2f}, max_rate={self.ros.yaw_max_rate:.2f}")
        except ValueError:
            self._on_log("[VISION] YAW 参数格式错误")

    def _calibrate_camera_offset(self):
        # When UAV nose is pointing directly at pole, record negative of current pixel_error as offset
        offset = -self.ros.pixel_error
        self.ros.camera_offset_x = offset * (self.ros.image_width / 2.0)
        self.in_camera_offset_x.setText(f"{self.ros.camera_offset_x:.1f}")
        self.ros.save_camera_offset()
        self._on_log(f"[VISION] 相机偏移标定并保存: offset_x={self.ros.camera_offset_x:.1f} px (pixel_error={self.ros.pixel_error:+.3f})")

    def _on_mission_start(self):
        reply = QMessageBox.question(
            self, "确认", "启动一键栖息任务？\n请确保：\n1. 飞控已连接\n2. 无人机已上电\n3. 周围安全",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply != QMessageBox.Yes:
            return
        # Apply current YAW params
        self._update_yaw_params()
        # Configure stop point from dropdown
        phase = self.combo_max_phase.currentData()
        self.mission.set_stop_after_phase(phase)
        self.mission.start()

    def _on_mission_continue(self):
        """Continue a paused mission to the newly selected stop point."""
        phase = self.combo_max_phase.currentData()
        self.mission.set_stop_after_phase(phase)

    def _on_mission_abort(self):
        self.mission.abort()

    def _on_image(self, img):
        """Display detection image in the camera view label (throttled to 10Hz)."""
        try:
            now = time.time()
            if not hasattr(self, '_last_image_update'):
                self._last_image_update = 0.0
            if now - self._last_image_update < 0.1:
                return
            self._last_image_update = now

            qimg = QImage(
                img["data"], img["width"], img["height"],
                img["bytes_per_line"], QImage.Format_RGB888
            )
            pix = QPixmap.fromImage(qimg).scaled(
                self.lbl_camera.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            self.lbl_camera.setPixmap(pix)
        except Exception as e:
            self._on_log(f"[UI] 图像显示失败: {e}")

    def closeEvent(self, event):
        self.mission.abort()
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
