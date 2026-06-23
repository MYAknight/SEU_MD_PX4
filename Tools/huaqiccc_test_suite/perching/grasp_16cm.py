#!/usr/bin/env python3
"""
huaqiccc_grasp_test.py
======================
Gazebo Classic perching/grasping test for 16cm diameter pole.

Goal: The drone expands its arms, slowly pushes into the pole,
      detects contact, then DISARMS and relies on geometry/friction
      to remain perched on the pole.

Strategy:
  1. Takeoff to HOVER_Z
  2. Expand arms to EXPAND_ANGLE (widely)
  3. Approach to APPROACH_X
  4. Very slow push (PUSH_SPEED ~0.03 m/s) past the pole surface
  5. Detect contact via stall + EFO
  6. Upon contact:
       a. Maintain slight forward pressure for GRASP_HOLD_TIME
       b. Optionally contract arms slightly to GRASP_ANGLE (hug the pole)
       c. DISARM (stop motors)
       d. Monitor position for MONITOR_TIME seconds
  7. Success criterion: mean z > SUCCESS_Z_MIN and small z variance

Collision considerations:
  - Pole 16cm diameter, soft contact (kp=50000, kd=100, mu=10)
  - Very low push speed to avoid bouncing off
  - Gazebo physics iters=20, step=0.002s for stability
"""

import argparse
import csv
import math
import os
import sys
import threading
import time
from datetime import datetime

try:
    import rospy
    from geometry_msgs.msg import PoseStamped, Vector3
    from gazebo_msgs.msg import ModelStates
    from mavros_msgs.msg import State, PositionTarget
    from sensor_msgs.msg import Imu
    from mavros_msgs.srv import CommandBool, SetMode, CommandLong
    from std_msgs.msg import Float64, Bool
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


def _smoothstep(t):
    if t <= 0.0:
        return 0.0
    if t >= 1.0:
        return 1.0
    return t * t * (3.0 - 2.0 * t)


def _smoothstep_deriv(t):
    if t <= 0.0 or t >= 1.0:
        return 0.0
    return 6.0 * t * (1.0 - t)


class GraspFlightTest:

    # ---------- Trajectory parameters ----------
    HOVER_Z = 2.5
    POLE_X = 5.0
    POLE_Y = 0.0
    POLE_RADIUS = 0.09          # 18cm diameter
    APPROACH_X = 4.75           # Start slow push from here, closer to pole
    APPROACH_Y = 0.0
    T_HOVER0 = 8.0              # Initial hover after takeoff
    T_APPROACH = 12.0           # Fly to approach point
    T_PUSH_MAX = 15.0           # Max push duration
    PUSH_SPEED = 0.05           # Moderate speed to reach pole surface reliably (m/s)
    GRASP_HOLD_TIME = 4.0       # Keep pressure after contact
    MONITOR_TIME = 10.0         # Monitor after disarm
    SUCCESS_Z_MIN = 1.5         # Must stay above this (m)
    SUCCESS_Z_STD_MAX = 0.5     # Must not oscillate too much (m)
    RATE_HZ = 20.0

    # Arm morphing
    EXPAND_ANGLE = -0.45        # Wide open
    GRASP_ANGLE = 0.0           # Fully closed to grip pole tightly
    MORPH_DURATION_EXPAND = 4.0
    MORPH_DURATION_CONTRACT = 4.0

    def __init__(self, output_prefix='grasp_test', k_soft=None, preload=None):
        self.output_prefix = output_prefix
        self.k_soft = k_soft
        self.preload = preload
        self.px4_pc_en = int(os.environ.get('MPCA_PC_EN', '2'))
        self.adm_mass = float(os.environ.get('MPCA_PC_ADM_MASS', '1.5'))
        self.adm_ka = float(os.environ.get('MPCA_PC_ADM_KA', '0.0'))
        self.adm_fd = float(os.environ.get('MPCA_PC_ADM_FD', '1.0'))
        self.adm_lim = float(os.environ.get('MPCA_PC_ADM_LIM', '0.03'))
        self.adm_kp = float(os.environ.get('MPCA_PC_ADM_KP', '0.0'))
        self.adm_kv = float(os.environ.get('MPCA_PC_ADM_KV', '0.0'))
        self.adm_kt = float(os.environ.get('MPCA_PC_ADM_KT', '0.0'))
        self.adm_kc = float(os.environ.get('MPCA_PC_ADM_KC', '0.0'))
        self.adm_w1 = float(os.environ.get('MPCA_PC_ADM_W1', '1.0'))
        self.adm_w2 = float(os.environ.get('MPCA_PC_ADM_W2', '1.0'))
        self.rate_hz = self.RATE_HZ
        self.dt = 1.0 / self.rate_hz
        self.records = []

        self.current_state = None
        self.current_pose = None
        self.current_imu = None
        self.sp_pub = None
        self.arm_pub = None
        self.arming_client = None
        self.set_mode_client = None
        self.cmd_long_srv = None
        self.fix_pub = None

        # GMO telemetry
        self.last_efo_mag = 0.0
        self.last_contact_state = -1
        self.last_should_close = False

        # Spring model telemetry (extracted from PX4 status text)
        self.compliant_stats = None  # dict with avg_thrust, avg_motor, max_pitch, samples

        # Morph angle feedback from PX4 (for PX4-owned retraction check)
        self.last_morph_angle = None

        # PX4 contact-detection signal parsed from STATUSTEXT
        self.px4_contact_detected = False

        # PX4 perching phase from MAVROS debug_float_array (more reliable than STATUSTEXT)
        self.px4_perching_phase = 0

        # Admittance / compliance telemetry from PX4 debug array
        self.last_delta_p = 0.0
        self.last_f_est = 0.0
        self.last_pitch_deg = 0.0

        # 31440 background sender
        self._pending_angle_lock = threading.Lock()
        self._pending_angle = None
        self._sender_stop = threading.Event()
        self._sender_thread = None
        self._last_sent_angle = None

        if not ROS_AVAILABLE:
            print("[FATAL] ROS not available")
            sys.exit(1)

        self._init_ros()
        self._start_sender()

    def _init_ros(self):
        try:
            rospy.init_node('huaqiccc_grasp_test', anonymous=True)
        except rospy.exceptions.ROSException:
            pass  # Node already initialized

        self.sp_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.arm_pub = rospy.Publisher('/huaqiccc/arm_angle', Float64, queue_size=1)
        self.fix_pub = rospy.Publisher('/huaqiccc/fix_perching', Bool, queue_size=1)

        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb)
        rospy.Subscriber('/mavros/imu/data', Imu, self._imu_cb)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self._gazebo_state_cb)
        try:
            from mavros_msgs.msg import DebugValue
            rospy.Subscriber('/mavros/debug_value/debug_float_array', DebugValue, self._debug_array_cb)
            print("[OK] Subscribed to MAVROS debug_float_array")
        except Exception as e:
            print(f"[WARN] debug_float_array subscribe: {e}")
        try:
            from mavros_msgs.msg import StatusText
            rospy.Subscriber('/mavros/statustext/recv', StatusText, self._statustext_cb)
            print("[OK] Subscribed to MAVROS statustext")
        except Exception as e:
            print(f"[WARN] statustext subscribe: {e}")

        try:
            from px4_msgs.msg import ExternalForceEstimate, ContactState, HuaqicccMorphAngle
            rospy.Subscriber('/fmu/external_force_estimate/out', ExternalForceEstimate, self._efo_cb)
            rospy.Subscriber('/fmu/contact_state/out', ContactState, self._contact_cb)
            rospy.Subscriber('/fmu/huaqiccc_morph_angle/out', HuaqicccMorphAngle, self._morph_angle_cb)
            print("[OK] Subscribed to px4_msgs GMO topics")
        except Exception as e:
            print(f"[INFO] px4_msgs topics not available ({e})")

        rospy.wait_for_service('/mavros/cmd/arming', timeout=10.0)
        rospy.wait_for_service('/mavros/set_mode', timeout=10.0)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        try:
            rospy.wait_for_service('/mavros/cmd/command', timeout=5.0)
            self.cmd_long_srv = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
            print("[OK] MAVROS cmd/command connected")
        except Exception as e:
            print(f"[WARN] cmd/command: {e}")
            self.cmd_long_srv = None

        print("[WAIT] Waiting for FCU connection...")
        dt = 1.0 / self.rate_hz
        while not rospy.is_shutdown() and (self.current_state is None or not self.current_state.connected):
            time.sleep(dt)
        print("[OK] FCU connected")

        # MAVROS param cache sync
        print("[WAIT] Waiting for MAVROS parameter cache sync...")
        time.sleep(5.0)
        try:
            from mavros_msgs.srv import ParamPull
            rospy.wait_for_service('/mavros/param/pull', timeout=10.0)
            param_pull = rospy.ServiceProxy('/mavros/param/pull', ParamPull)
            pull_resp = param_pull(False)
            if pull_resp.success:
                print(f"[OK] Parameter sync complete, {pull_resp.param_received} params received")
            else:
                print("[WARN] Param pull not successful, continuing anyway...")
            time.sleep(3.0)
        except Exception as e:
            print(f"[WARN] Param pull skipped: {e}")
            time.sleep(5.0)

        mpca_mode = int(os.environ.get('MPCA_MODE', '0'))
        if mpca_mode != 0:
            self._set_param('MPCA_MODE', integer=mpca_mode)

    def _state_cb(self, msg):
        self.current_state = msg

    def _pose_cb(self, msg):
        self.current_pose = msg.pose

    def _imu_cb(self, msg):
        self.current_imu = msg.linear_acceleration

    def _gazebo_state_cb(self, msg):
        for i, name in enumerate(msg.name):
            if name == 'huaqiccc':
                self._gazebo_pose = msg.pose[i]
                break

    def _efo_cb(self, msg):
        self.last_efo_mag = math.sqrt(msg.force_x**2 + msg.force_y**2 + msg.force_z**2)

    def _contact_cb(self, msg):
        self.last_contact_state = msg.state
        self.last_should_close = msg.should_close

    def _morph_angle_cb(self, msg):
        self.last_morph_angle = msg.arm_angle

    def _debug_array_cb(self, msg):
        """Parse PX4 perching debug array published by mc_pos_control.

        Expected layout (matches MulticopterPositionControl.cpp):
          data[0] = contact_detect_state
          data[1] = perching_phase
          data[2] = pc_en
          data[3] = arm_angle (or 99 if unavailable)
          data[4] = grasp_secure
          data[5] = perching_active
          data[6] = admittance delta_p [m]
          data[7] = estimated contact force [N]
          data[8] = preload parameter [m]
          data[9] = pitch angle [deg]
        """
        try:
            if hasattr(msg, 'data') and len(msg.data) >= 6:
                self.px4_perching_phase = int(round(msg.data[1]))
                angle = msg.data[3]
                if angle < 90.0:
                    self.last_morph_angle = angle
                if self.px4_perching_phase >= 2 and not self.px4_contact_detected:
                    self.px4_contact_detected = True
                    print(f"[TELEM] PX4 perching phase={self.px4_perching_phase}, arm_angle={angle:.3f}")
                if len(msg.data) >= 10:
                    self.last_delta_p = float(msg.data[6])
                    self.last_f_est = float(msg.data[7])
                    self.last_pitch_deg = float(msg.data[9])
        except Exception:
            pass

    def _statustext_cb(self, msg):
        """Parse PX4 status text messages for perching state and morph angle."""
        try:
            text = msg.text if hasattr(msg, 'text') else str(msg)
            if 'COMPLIANT stats' in text:
                # Format: "Perching: COMPLIANT stats avg_thrust=0.XXX avg_motor=0.XXX max_pitch=XX.Xdeg samples=NNN"
                import re
                m = re.search(r'avg_thrust=([\d.]+).*avg_motor=([\d.]+).*max_pitch=([\d.]+)', text)
                if m:
                    self.compliant_stats = {
                        'avg_thrust': float(m.group(1)),
                        'avg_motor': float(m.group(2)),
                        'max_pitch_deg': float(m.group(3)),
                        'raw_text': text,
                    }
                    print(f"[TELEM] COMPLIANT stats: thrust={self.compliant_stats['avg_thrust']:.3f} "
                          f"motor={self.compliant_stats['avg_motor']:.3f} "
                          f"pitch={self.compliant_stats['max_pitch_deg']:.1f}°")
            if 'Perching: contact detected, entering compliance' in text or 'CONTACT_DETECTED' in text:
                if not self.px4_contact_detected:
                    self.px4_contact_detected = True
                    print(f"[TELEM] PX4 contact detected: {text.strip()}")
            m = re.search(r'morph angle=([+-]?\d+\.?\d*) rad', text)
            if m:
                try:
                    self.last_morph_angle = float(m.group(1))
                except ValueError:
                    pass
        except Exception as e:
            pass

    def _set_param(self, param_id, integer=None, real=None):
        try:
            from mavros_msgs.srv import ParamSet
            from mavros_msgs.msg import ParamValue
            rospy.wait_for_service('/mavros/param/set', timeout=10.0)
            param_set = rospy.ServiceProxy('/mavros/param/set', ParamSet)
            pv = ParamValue()
            if integer is not None:
                pv.integer = int(integer)
            if real is not None:
                pv.real = float(real)
            resp = param_set(param_id=param_id, value=pv)
            if resp.success:
                print(f"[OK] Set {param_id}")
                return True
            else:
                print(f"[WARN] Set {param_id} returned success=False")
        except Exception as e:
            print(f"[WARN] Param set {param_id} failed: {e}")
        return False

    def _set_k_soft(self):
        if self.k_soft is not None:
            print(f"[PARAM] Setting MPCA_PC_K_SOFT={self.k_soft}")
            self._set_param('MPCA_PC_K_SOFT', real=self.k_soft)

    def _set_preload(self):
        if self.preload is not None:
            print(f"[PARAM] Setting MPCA_PC_PRELOAD={self.preload}")
            self._set_param('MPCA_PC_PRELOAD', real=self.preload)

    def _set_pc_en(self):
        print(f"[PARAM] Setting MPCA_PC_EN={self.px4_pc_en}")
        self._set_param('MPCA_PC_EN', integer=self.px4_pc_en)

    def _set_adm_params(self):
        print("[PARAM] Setting admittance/compliance parameters...")
        print(f"  MPCA_PC_ADM_MASS={self.adm_mass}, KA={self.adm_ka}, FD={self.adm_fd}, LIM={self.adm_lim}")
        print(f"  MPCA_PC_ADM_KP={self.adm_kp}, KV={self.adm_kv}, KT={self.adm_kt}, KC={self.adm_kc}, W1={self.adm_w1}, W2={self.adm_w2}")
        self._set_param('MPCA_PC_ADM_MASS', real=self.adm_mass)
        self._set_param('MPCA_PC_ADM_KA', real=self.adm_ka)
        self._set_param('MPCA_PC_ADM_FD', real=self.adm_fd)
        self._set_param('MPCA_PC_ADM_LIM', real=self.adm_lim)
        self._set_param('MPCA_PC_ADM_KP', real=self.adm_kp)
        self._set_param('MPCA_PC_ADM_KV', real=self.adm_kv)
        self._set_param('MPCA_PC_ADM_KT', real=self.adm_kt)
        self._set_param('MPCA_PC_ADM_KC', real=self.adm_kc)
        self._set_param('MPCA_PC_ADM_W1', real=self.adm_w1)
        self._set_param('MPCA_PC_ADM_W2', real=self.adm_w2)

    # ---------- 31440 background sender ----------

    def _send_morph_31440(self, angle):
        if not self.cmd_long_srv:
            return False
        try:
            from mavros_msgs.srv import CommandLongRequest
            req = CommandLongRequest()
            req.broadcast = False
            req.command = 31440
            req.confirmation = 0
            req.param1 = float(angle)
            resp = self.cmd_long_srv(req)
            return getattr(resp, 'success', False)
        except Exception as e:
            print(f"[ERROR] 31440: {e}")
            return False

    def _start_sender(self):
        self._sender_thread = threading.Thread(target=self._sender_loop, daemon=True)
        self._sender_thread.start()
        print("[OK] Background 31440 sender started")

    def _sender_loop(self):
        while not self._sender_stop.is_set() and not rospy.is_shutdown():
            angle = None
            with self._pending_angle_lock:
                if self._pending_angle is not None:
                    angle = self._pending_angle
                    self._pending_angle = None
            if angle is not None:
                self._send_morph_31440(angle)
            else:
                time.sleep(0.02)

    def _stop_sender(self):
        self._sender_stop.set()
        if self._sender_thread and self._sender_thread.is_alive():
            self._sender_thread.join(timeout=2.0)

    def update_px4_morph(self, arm_angle, force=False):
        if not force and self._last_sent_angle is not None:
            if abs(arm_angle - self._last_sent_angle) < 0.03:
                return False
        self._last_sent_angle = arm_angle
        with self._pending_angle_lock:
            self._pending_angle = arm_angle
        print(f"[31440 QUEUE] angle={arm_angle:.3f}")
        return True

    # ---------- PositionTarget helpers ----------

    def _make_position_target(self, x, y, z, yaw, vx=0.0, vy=0.0, yaw_rate=0.0):
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        has_vel = abs(vx) > 1e-6 or abs(vy) > 1e-6
        has_yaw_rate = abs(yaw_rate) > 1e-6

        if has_vel and has_yaw_rate:
            msg.type_mask = (
                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
            )
            msg.velocity.x = vx
            msg.velocity.y = vy
            msg.velocity.z = 0.0
            msg.yaw_rate = yaw_rate
        elif has_vel:
            msg.type_mask = (
                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE
            )
            msg.velocity.x = vx
            msg.velocity.y = vy
            msg.velocity.z = 0.0
        else:
            msg.type_mask = (
                PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE
            )

        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.yaw = yaw
        return msg

    def _send_setpoint(self, x, y, z, yaw=0.0, vx=0.0, vy=0.0, yaw_rate=0.0):
        self.sp_pub.publish(self._make_position_target(x, y, z, yaw, vx, vy, yaw_rate))

    def _send_arm_angle_ros(self, angle):
        if self.arm_pub:
            msg = Float64()
            msg.data = float(angle)
            self.arm_pub.publish(msg)

    def _record(self, phase, t, sp_x, sp_y, sp_z, act_x=None, act_y=None, act_z=None):
        act = self.current_pose
        imu = self.current_imu
        if act is None and act_x is None:
            return
        ax = round(imu.x, 3) if imu else 0.0
        ay = round(imu.y, 3) if imu else 0.0
        az = round(imu.z, 3) if imu else 0.0
        # Use provided actual positions if available (e.g. Gazebo true pose)
        if act_x is None and act is not None:
            act_x = act.position.x
        if act_y is None and act is not None:
            act_y = act.position.y
        if act_z is None and act is not None:
            act_z = act.position.z
        self.records.append({
            'phase': phase,
            'time': round(t, 3),
            'sp_x': round(sp_x, 5),
            'sp_y': round(sp_y, 5),
            'sp_z': round(sp_z, 5),
            'act_x': round(act_x, 5),
            'act_y': round(act_y, 5),
            'act_z': round(act_z, 5),
            'ax': ax, 'ay': ay, 'az': az,
            'a_norm': round(math.sqrt(ax*ax + ay*ay + az*az), 3),
            'efo_mag': round(self.last_efo_mag, 4),
            'contact_state': self.last_contact_state,
            'should_close': 1 if self.last_should_close else 0,
            'delta_p': round(self.last_delta_p, 5),
            'f_est': round(self.last_f_est, 3),
            'pitch_deg': round(self.last_pitch_deg, 2),
            'px4_phase': self.px4_perching_phase,
        })

    def _save_csv(self, suffix=''):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_dir = os.path.join(os.path.expanduser("~"), "huaqiccc_logs")
        os.makedirs(out_dir, exist_ok=True)
        name = f"{self.output_prefix}{suffix}_{ts}.csv"
        out_path = os.path.join(out_dir, name)
        if not self.records:
            print("[WARN] No records to save")
            return None
        with open(out_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.records[0].keys())
            writer.writeheader()
            writer.writerows(self.records)
        print(f"[SAVE] {out_path}")
        return out_path

    def _ensure_offboard(self):
        if self.current_state and self.current_state.mode != "OFFBOARD":
            print("[GUARD] Re-enabling OFFBOARD mode")
            self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            rospy.sleep(0.5)

    # ---------- Flight phases ----------

    def _pre_send(self, x, y, z, count=100):
        rate = rospy.Rate(self.rate_hz)
        for _ in range(count):
            if rospy.is_shutdown():
                return False
            self._send_setpoint(x, y, z)
            rate.sleep()
        return True

    def _phase_hover(self, duration, x, y, z):
        print(f"[HOVER] {duration}s at ({x:.1f},{y:.1f},{z:.1f})")
        dt = 1.0 / self.rate_hz
        start = time.time()
        while not rospy.is_shutdown():
            t = time.time() - start
            if t > duration:
                break
            self._send_setpoint(x, y, z)
            self._record('hover', t, x, y, z)
            time.sleep(dt)
        return not rospy.is_shutdown()

    def _phase_approach(self, x1, y1, z, x2, y2, duration):
        print(f"[APPROACH] ({x1:.1f},{y1:.1f}) -> ({x2:.1f},{y2:.1f}) over {duration}s")
        dt = 1.0 / self.rate_hz
        start = time.time()
        while not rospy.is_shutdown():
            t = time.time() - start
            if t > duration:
                break
            s = min(1.0, t / duration)
            alpha = _smoothstep(s)
            alpha_d = _smoothstep_deriv(s) / duration
            x = x1 + (x2 - x1) * alpha
            y = y1 + (y2 - y1) * alpha
            vx = (x2 - x1) * alpha_d
            vy = (y2 - y1) * alpha_d
            self._send_setpoint(x, y, z, 0.0, vx, vy, 0.0)
            self._record('approach', t, x, y, z)
            time.sleep(dt)
        return not rospy.is_shutdown()

    def _morph_arm(self, target_angle, duration, hold_x=0.0, hold_y=0.0, hold_z=None):
        """Gradually morph arm angle while holding position."""
        if hold_z is None:
            hold_z = self.HOVER_Z
        print(f"[MORPH] {self._last_sent_angle if self._last_sent_angle is not None else 0.0:.2f} -> {target_angle:.2f} rad over {duration:.1f}s")
        dt_vis = 0.1
        start_angle = self._last_sent_angle if self._last_sent_angle is not None else 0.0
        start_time = time.time()
        last_px4_update = start_time
        while not rospy.is_shutdown():
            t = time.time() - start_time
            if t > duration:
                break
            s = t / duration
            s2 = s * s * (3.0 - 2.0 * s)
            angle = start_angle + (target_angle - start_angle) * s2
            self._send_arm_angle_ros(angle)
            if time.time() - last_px4_update >= 0.9:
                self.update_px4_morph(angle, force=False)
                last_px4_update = time.time()
            self._send_setpoint(hold_x, hold_y, hold_z)
            time.sleep(dt_vis)
        self._send_arm_angle_ros(target_angle)
        self.update_px4_morph(target_angle, force=True)
        self._send_setpoint(hold_x, hold_y, hold_z)
        print(f"[MORPH] Reached {target_angle:.2f} rad")

    def _phase_push_and_detect(self, x_start, y, z, x_target, timeout, speed):
        """Very slow push into pole with contact detection.

        In PX4-owned mode (MPCA_PC_EN >= 2) the script does NOT decide when
        contact occurs; it keeps pushing slowly and waits for PX4 to announce
        contact via STATUSTEXT.  The legacy stall/EFO detection is only used as
        a fallback when PX4 perching control is disabled.
        """
        px4_owned = self.px4_pc_en >= 2
        print(f"[PUSH] From x={x_start:.2f} toward x={x_target:.2f} at {speed}m/s "
              f"(PX4_owned={px4_owned})")
        dt = 1.0 / self.rate_hz
        start = time.time()
        contact_detected = False
        contact_time = None
        push_duration_est = abs(x_target - x_start) / max(speed, 0.001)

        # Thresholds for contact detection (legacy mode only)
        EFO_THRESHOLD = 1.0
        STALL_DIST = 0.15
        STALL_S = 0.70

        while not rospy.is_shutdown():
            t = time.time() - start
            if t > timeout:
                print("[PUSH] Timeout reached")
                break

            s = min(1.0, t / push_duration_est)
            alpha = _smoothstep(s)
            alpha_d = _smoothstep_deriv(s) / push_duration_est
            x = x_start + (x_target - x_start) * alpha
            vx = (x_target - x_start) * alpha_d
            self._send_setpoint(x, y, z, 0.0, vx, 0.0, 0.0)

            act_x = self.current_pose.position.x if self.current_pose else x_start
            efo = self.last_efo_mag
            cstate = self.last_contact_state

            if px4_owned:
                # Wait for PX4 to detect contact and take over the setpoint.
                if self.px4_contact_detected:
                    contact_detected = True
                    contact_time = t
                    print(f"[CONTACT] PX4 detected contact at t={t:.2f}s")
                    # Keep sending setpoints briefly to maintain OFFBOARD
                    for _ in range(20):
                        if rospy.is_shutdown():
                            break
                        self._send_setpoint(x, y, z)
                        self._record('push', t, x, y, z)
                        time.sleep(dt)
                    break
            else:
                # Legacy script-side contact detection.
                stalled = False
                if s > STALL_S:
                    pole_surface_x = self.POLE_X - self.POLE_RADIUS
                    if abs(act_x - pole_surface_x) < STALL_DIST:
                        stalled = True
                        print(f"  [STALL] act_x={act_x:.2f}, pole_surface={pole_surface_x:.2f}, sp_x={x:.2f}")

                if efo > EFO_THRESHOLD or cstate > 0 or stalled:
                    if not contact_detected:
                        contact_detected = True
                        contact_time = t
                        print(f"[CONTACT] Detected at t={t:.2f}s | efo={efo:.2f}N | cstate={cstate} | stalled={stalled}")
                        for _ in range(20):
                            if rospy.is_shutdown():
                                break
                            self._send_setpoint(x, y, z)
                            self._record('push', t, x, y, z)
                            time.sleep(dt)
                        break

            self._record('push', t, x, y, z)
            time.sleep(dt)

        return contact_detected, contact_time

    def _phase_hold_position(self, x, y, z, duration):
        """Hold position to maintain OFFBOARD and perching pressure."""
        print(f"[HOLD] ({x:.2f},{y:.2f},{z:.2f}) for {duration:.1f}s")
        dt = 1.0 / self.rate_hz
        start = time.time()
        while not rospy.is_shutdown():
            t = time.time() - start
            if t > duration:
                break
            self._send_setpoint(x, y, z)
            self._record('hold', t, x, y, z)
            time.sleep(dt)
        return not rospy.is_shutdown()

    def _wait_arm_retracted(self, timeout=15.0, threshold=-0.05):
        """Wait for PX4-driven arm retraction to finish (angle >= threshold)."""
        print(f"[WAIT] Waiting for arm retraction (angle >= {threshold:.3f} rad, timeout {timeout:.1f}s)")
        dt = 1.0 / self.rate_hz
        start = time.time()
        while not rospy.is_shutdown():
            t = time.time() - start
            if t > timeout:
                print("[WARN] Arm retraction wait timeout")
                return False
            angle = self.last_morph_angle
            if angle is not None and angle >= threshold:
                print(f"[OK] Arm retracted (angle={angle:.3f} rad)")
                return True
            time.sleep(dt)
        return False

    def _force_disarm(self):
        """Send force disarm command (stops motors even when not landed)."""
        if not self.cmd_long_srv:
            return False
        try:
            from mavros_msgs.srv import CommandLongRequest
            req = CommandLongRequest()
            req.broadcast = False
            req.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
            req.confirmation = 0
            req.param1 = 0.0   # disarm
            req.param2 = 21196.0  # force
            resp = self.cmd_long_srv(req)
            success = getattr(resp, 'success', False)
            if success:
                print("[OK] Force disarm accepted")
            else:
                print(f"[WARN] Force disarm result={getattr(resp, 'result', 'unknown')}")
            return success
        except Exception as e:
            print(f"[ERROR] Force disarm: {e}")
            return False

    def _send_fix(self, enable=True):
        if self.fix_pub:
            msg = Bool()
            msg.data = enable
            self.fix_pub.publish(msg)
            print(f"[FIX] Sent fix_perching={enable}")

    def _get_gazebo_model_pose(self):
        """Get cached true pose from Gazebo subscriber."""
        if hasattr(self, '_gazebo_pose') and self._gazebo_pose is not None:
            return self._gazebo_pose
        # Fallback to wait_for_message if cache empty
        try:
            from gazebo_msgs.msg import ModelStates
            msg = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=2.0)
            for i, name in enumerate(msg.name):
                if name == 'huaqiccc':
                    return msg.pose[i]
        except Exception:
            pass
        return None

    def _phase_monitor_after_disarm(self, duration):
        """Monitor true Gazebo pose after disarm (PX4 EKF drifts when disarmed)."""
        print(f"[MONITOR] Motors stopped. Monitoring TRUE Gazebo position for {duration:.1f}s...")
        dt = 1.0 / self.rate_hz
        start = time.time()
        xs, ys, zs = [], [], []
        while not rospy.is_shutdown():
            t = time.time() - start
            if t > duration:
                break
            pose = self._get_gazebo_model_pose()
            if pose:
                x, y, z = pose.position.x, pose.position.y, pose.position.z
                xs.append(x)
                ys.append(y)
                zs.append(z)
                self._record('monitor', t, x, y, z, act_x=x, act_y=y, act_z=z)
            else:
                # Fallback to MAVROS (EKF may drift after disarm)
                if self.current_pose:
                    z = self.current_pose.position.z
                    zs.append(z)
                    self._record('monitor', t,
                                 self.current_pose.position.x,
                                 self.current_pose.position.y, z)
            time.sleep(dt)

        if len(zs) < 5:
            print("[MONITOR] Not enough samples")
            return False, 0.0, 0.0

        z_mean = sum(zs) / len(zs)
        z_var = sum((z - z_mean)**2 for z in zs) / len(zs)
        z_std = math.sqrt(z_var)
        x_std = 0.0
        if len(xs) >= 5:
            x_mean = sum(xs) / len(xs)
            x_var = sum((x - x_mean)**2 for x in xs) / len(xs)
            x_std = math.sqrt(x_var)
        # Success: stays above threshold with very low oscillation in Gazebo
        success = z_mean > self.SUCCESS_Z_MIN and z_std < 0.05 and x_std < 0.05
        print(f"[MONITOR] z_mean={z_mean:.2f}m, z_std={z_std:.2f}m, x_std={x_std:.2f}m, n={len(zs)}")
        print(f"[RESULT] {'SUCCESS' if success else 'FAILURE'} - drone {'remains perched' if success else 'fell or oscillates'}")
        return success, z_mean, z_std

    # ---------- Main flight ----------

    def run(self):
        self._set_k_soft()
        self._set_preload()
        self._set_pc_en()
        self._set_adm_params()
        rate = rospy.Rate(self.rate_hz)

        # Pre-send hover setpoints
        print("[PRE] Pre-sending setpoints...")
        if not self._pre_send(0.0, 0.0, self.HOVER_Z, count=100):
            return None

        # OFFBOARD
        print("[MODE] OFFBOARD")
        self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")

        # ARM
        print("[ARM] Arming")
        self.arming_client(True)

        # Takeoff hover
        print("[TAKEOFF] Hover")
        if not self._pre_send(0.0, 0.0, self.HOVER_Z, count=100):
            return None

        # Phase 1: Initial hover
        if not self._phase_hover(self.T_HOVER0, 0.0, 0.0, self.HOVER_Z):
            return None

        self._ensure_offboard()

        # Phase 2: Expand arms wide open
        print("[EXPAND] Opening arms wide for perching...")
        self._morph_arm(self.EXPAND_ANGLE, self.MORPH_DURATION_EXPAND,
                        hold_x=0.0, hold_y=0.0, hold_z=self.HOVER_Z)

        self._ensure_offboard()
        print("[GUARD] Post-morph setpoint burst")
        for _ in range(40):
            if rospy.is_shutdown():
                return None
            self._send_setpoint(0.0, 0.0, self.HOVER_Z)
            rate.sleep()

        # Phase 3: Approach to pole front
        if not self._phase_approach(0.0, 0.0, self.HOVER_Z,
                                     self.APPROACH_X, self.APPROACH_Y,
                                     self.T_APPROACH):
            return None

        # Phase 4: Very slow push into pole
        # Target just past the pole surface so the drone presses against it
        x_push_target = self.POLE_X + 0.15
        contact_detected, contact_time = self._phase_push_and_detect(
            self.APPROACH_X, self.APPROACH_Y, self.HOVER_Z,
            x_push_target, self.T_PUSH_MAX, self.PUSH_SPEED
        )

        if not contact_detected:
            print("[RESULT] No contact detected. Aborting to land.")
            self._morph_arm(0.0, self.MORPH_DURATION_CONTRACT,
                           hold_x=self.APPROACH_X, hold_y=0.0, hold_z=self.HOVER_Z)
            print("[LAND] Landing")
            self.set_mode_client(base_mode=0, custom_mode="AUTO.LAND")
            rospy.sleep(8.0)
            self.arming_client(False)
            self._stop_sender()
            return self._save_csv(suffix='_no_contact')

        print(f"[RESULT] Contact detected at t={contact_time:.2f}s")

        # Phase 5: contact handling.
        # PX4 is now overriding the 3D setpoint to the recorded contact point + preload.
        # The script keeps the offboard link alive and (in SITL) drives arm retraction
        # because the morph-control module is disabled in simulation.
        perching_x = self.POLE_X + 0.25
        print(f"[PERCHING] PX4 owns setpoint; contracting arms near x={perching_x:.2f}")
        self._morph_arm(self.GRASP_ANGLE, self.MORPH_DURATION_CONTRACT,
                        hold_x=perching_x, hold_y=0.0, hold_z=self.HOVER_Z)
        hold_after_grasp = 6.0

        # Phase 6: Hold long enough for the grasp to settle.
        print(f"[HOLD] Holding position for {hold_after_grasp}s...")
        self._phase_hold_position(perching_x, self.APPROACH_Y, self.HOVER_Z, hold_after_grasp)

        # Manual fix trigger (fallback if auto-fix didn't fire)
        self._send_fix(True)
        time.sleep(1.0)

        # Phase 7: Disarm and monitor
        print("[DISARM] Stopping motors - relying on geometry/friction...")
        if not self._force_disarm():
            print("[WARN] Force disarm failed, trying normal disarm...")
            self.arming_client(False)
        time.sleep(2.0)  # Wait for motors to actually stop

        success, z_mean, z_std = self._phase_monitor_after_disarm(self.MONITOR_TIME)

        # If failed, try to recover by re-arming and landing
        if not success:
            print("[RECOVER] Re-arming and landing...")
            self.arming_client(True)
            rospy.sleep(1.0)
            self.set_mode_client(base_mode=0, custom_mode="AUTO.LAND")
            rospy.sleep(8.0)

        self.arming_client(False)
        self._stop_sender()

        # Print spring model telemetry summary
        if self.compliant_stats:
            print("\n" + "=" * 50)
            print("  SPRING MODEL TELEMETRY SUMMARY")
            print("=" * 50)
            print(f"  avg_thrust (norm): {self.compliant_stats['avg_thrust']:.4f}")
            print(f"  avg_motor  (norm): {self.compliant_stats['avg_motor']:.4f}")
            print(f"  max_pitch  (deg):  {self.compliant_stats['max_pitch_deg']:.1f}")
            print("=" * 50)
        else:
            print("\n[WARN] No COMPLIANT stats received from PX4")

        suffix = '_success' if success else '_fail'
        return self._save_csv(suffix=suffix)


def main():
    parser = argparse.ArgumentParser(description='huaqiccc Perching Grasp Test (16cm pole)')
    parser.add_argument('--output', default='grasp_test', help='Output CSV prefix')
    parser.add_argument('--k-soft', type=float, default=None, help='MPCA_PC_K_SOFT value (impedance stiffness ratio)')
    parser.add_argument('--preload', type=float, default=0.03, help='MPCA_PC_PRELOAD value (spring preload offset in m)')
    args = parser.parse_args()

    print("\n" + "=" * 60)
    print("  huaqiccc Perching Grasp Test (16cm pole)")
    print("  Pole position: x=5.0, y=0.0, radius=0.09m")
    print("  Strategy: expand -> slow push -> contact -> disarm -> monitor")
    print("=" * 60 + "\n")

    test = GraspFlightTest(output_prefix=args.output, k_soft=args.k_soft, preload=args.preload)
    csv_path = test.run()
    if csv_path:
        print(f"\n[RESULT] Log saved to: {csv_path}")
    else:
        print("\n[ERROR] Test did not complete or no log saved")


if __name__ == '__main__':
    main()
