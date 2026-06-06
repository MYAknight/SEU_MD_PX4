#!/usr/bin/env python3
"""
huaqiccc_perching_test.py v3
============================
Gazebo Classic pole collision test for end-to-end GMO verification.

Based on the proven huaqiccc_simplified_flight_test.py architecture:
  - PositionTarget with velocity feedforward (same as working flight test)
  - Background thread for 31440 morph commands (non-blocking)
  - Conservative trajectory to avoid failsafe

v3 fixes:
  1. MAVROS param cache sync before setting MPCA_MODE (prevents "Unknown parameter")
  2. Continuous position setpoint during morphing (prevents OFFBOARD timeout /失控)
  3. Extra OFFBOARD guards around mode transitions

Flight plan:
  1. Takeoff to HOVER_Z, hover for T_HOVER0
  2. Expand arms during hover (position hold maintained)
  3. Fly to approach position (x=APPROACH_X) with smooth trajectory
  4. Slow forward push into pole
  5. Monitor contact via position stall + EFO topic if available
  6. Retreat and land
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
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import State, PositionTarget
    from mavros_msgs.srv import CommandBool, SetMode, CommandLong
    from std_msgs.msg import Float64
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


class PerchingFlightTest:

    # ---------- Trajectory parameters (conservative) ----------
    HOVER_Z = 2.5
    POLE_X = 5.0
    POLE_Y = 0.0
    APPROACH_X = 4.0
    APPROACH_Y = 0.0
    T_HOVER0 = 8.0         # Initial hover after takeoff
    T_APPROACH = 10.0      # Fly to approach point
    T_PUSH = 12.0          # Max push duration
    T_RETREAT = 8.0        # Retreat duration
    T_HOVER1 = 3.0         # Hover before land
    PUSH_SPEED = 0.25      # Max forward speed during push (m/s)
    RATE_HZ = 20.0

    def __init__(self, output_prefix='perching_test'):
        self.output_prefix = output_prefix
        self.rate_hz = self.RATE_HZ
        self.dt = 1.0 / self.rate_hz
        self.records = []

        self.current_state = None
        self.current_pose = None
        self.sp_pub = None
        self.arm_pub = None
        self.arming_client = None
        self.set_mode_client = None
        self.cmd_long_srv = None

        # GMO telemetry placeholders
        self.last_efo_mag = 0.0
        self.last_contact_state = -1
        self.last_should_close = False

        # 31440 background sender (same pattern as working flight test)
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
        rospy.init_node('huaqiccc_perching_test', anonymous=True)

        self.sp_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.arm_pub = rospy.Publisher('/huaqiccc/arm_angle', Float64, queue_size=1)

        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb)

        # Try to subscribe GMO topics if bridged
        try:
            from px4_msgs.msg import ExternalForceEstimate, ContactState
            rospy.Subscriber('/fmu/external_force_estimate/out', ExternalForceEstimate, self._efo_cb)
            rospy.Subscriber('/fmu/contact_state/out', ContactState, self._contact_cb)
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
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and (self.current_state is None or not self.current_state.connected):
            rate.sleep()
        print("[OK] FCU connected")

        # ========== v3 FIX 1: MAVROS param cache sync ==========
        print("[WAIT] Waiting for MAVROS parameter cache sync...")
        rospy.sleep(5.0)
        try:
            from mavros_msgs.srv import ParamPull
            rospy.wait_for_service('/mavros/param/pull', timeout=10.0)
            param_pull = rospy.ServiceProxy('/mavros/param/pull', ParamPull)
            pull_resp = param_pull(False)
            if pull_resp.success:
                print(f"[OK] Parameter sync complete, {pull_resp.param_received} params received")
            else:
                print("[WARN] Param pull not successful, continuing anyway...")
            rospy.sleep(3.0)
        except Exception as e:
            print(f"[WARN] Param pull skipped: {e}")
            rospy.sleep(5.0)

        mpca_mode = int(os.environ.get('MPCA_MODE', '0'))
        if mpca_mode != 0:
            self._set_param('MPCA_MODE', integer=mpca_mode)
        self._set_param('MPCA_PC_EN', integer=3)
        self._set_param('MPCA_PC_GATE', real=0.0)
        self._set_param('MPCA_PC_TRIG', integer=1)
        # EFO parameters are now set statically in px4-rc.params

    def _state_cb(self, msg):
        self.current_state = msg

    def _pose_cb(self, msg):
        self.current_pose = msg.pose

    def _efo_cb(self, msg):
        self.last_efo_mag = math.sqrt(msg.force_x**2 + msg.force_y**2 + msg.force_z**2)

    def _contact_cb(self, msg):
        self.last_contact_state = msg.state
        self.last_should_close = msg.should_close

    def _set_param(self, param_id, integer=None, real=None):
        try:
            from mavros_msgs.srv import ParamSet
            from mavros_msgs.msg import ParamValue
            rospy.wait_for_service('/mavros/param/set', timeout=10.0)
            param_set = rospy.ServiceProxy('/mavros/param/set', ParamSet)
            pv = ParamValue()
            if integer is not None:
                pv.integer = integer
            if real is not None:
                pv.real = real
            resp = param_set(param_id=param_id, value=pv)
            if resp.success:
                print(f"[OK] Set {param_id}")
                return True
            else:
                print(f"[WARN] Set {param_id} returned success=False")
        except Exception as e:
            print(f"[WARN] Param set {param_id} failed: {e}")
        return False

    # ---------- 31440 background sender (non-blocking) ----------

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

    # ---------- PositionTarget with velocity feedforward ----------

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
        elif has_yaw_rate:
            msg.type_mask = (
                PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
            )
            msg.yaw_rate = yaw_rate
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

    def _record(self, phase, t, sp_x, sp_y, sp_z):
        act = self.current_pose
        if act is None:
            return
        self.records.append({
            'phase': phase,
            'time': round(t, 3),
            'sp_x': round(sp_x, 5),
            'sp_y': round(sp_y, 5),
            'sp_z': round(sp_z, 5),
            'act_x': round(act.position.x, 5),
            'act_y': round(act.position.y, 5),
            'act_z': round(act.position.z, 5),
            'efo_mag': round(self.last_efo_mag, 4),
            'contact_state': self.last_contact_state,
            'should_close': 1 if self.last_should_close else 0,
        })

    def _save_csv(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_dir = os.path.join(os.path.expanduser("~"), "huaqiccc_logs")
        os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.join(out_dir, f"{self.output_prefix}_{ts}.csv")
        if not self.records:
            print("[WARN] No records to save")
            return None
        with open(out_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.records[0].keys())
            writer.writeheader()
            writer.writerows(self.records)
        print(f"[SAVE] {out_path}")
        return out_path

    # ---------- OFFBOARD guard ----------

    def _ensure_offboard(self):
        """Check if still in OFFBOARD; if not, re-enable."""
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
        print(f"[HOVER] {duration}s at ({x:.1f}, {y:.1f}, {z:.1f})")
        rate = rospy.Rate(self.rate_hz)
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start).to_sec()
            if t > duration:
                break
            self._send_setpoint(x, y, z)
            self._record('hover', t, x, y, z)
            rate.sleep()
        return not rospy.is_shutdown()

    def _phase_approach(self, x1, y1, z, x2, y2, duration):
        print(f"[APPROACH] ({x1:.1f},{y1:.1f}) -> ({x2:.1f},{y2:.1f}) over {duration}s")
        rate = rospy.Rate(self.rate_hz)
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start).to_sec()
            if t > duration:
                break
            s = min(1.0, t / duration)
            # smoothstep for position, analytic derivative for velocity feedforward
            alpha = _smoothstep(s)
            alpha_d = _smoothstep_deriv(s) / duration
            x = x1 + (x2 - x1) * alpha
            y = y1 + (y2 - y1) * alpha
            vx = (x2 - x1) * alpha_d
            vy = (y2 - y1) * alpha_d
            self._send_setpoint(x, y, z, 0.0, vx, vy, 0.0)
            self._record('approach', t, x, y, z)
            rate.sleep()
        return not rospy.is_shutdown()

    def _morph_arm_slowly(self, target_angle, duration, hold_x=0.0, hold_y=0.0, hold_z=None, push_x=None, push_y=None, push_z=None):
        """
        Gradually morph arm angle while HOLDING position setpoint.

        v3 FIX 2: Always send position setpoint during morphing to prevent
        OFFBOARD timeout and subsequent失控.
        """
        if hold_z is None:
            hold_z = self.HOVER_Z
        print(f"[MORPH] Slow morph to {target_angle:.2f} rad over {duration:.1f}s (hold at {hold_x:.1f},{hold_y:.1f},{hold_z:.1f})")
        rate_vis = rospy.Rate(10)   # 10 Hz for Gazebo arm visual
        start_angle = self._last_sent_angle if self._last_sent_angle is not None else 0.0
        start_time = rospy.Time.now()
        last_px4_update = start_time
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start_time).to_sec()
            if t > duration:
                break
            s = t / duration
            s2 = s * s * (3.0 - 2.0 * s)  # smoothstep
            angle = start_angle + (target_angle - start_angle) * s2

            # Always update Gazebo visual at 10 Hz
            self._send_arm_angle_ros(angle)

            # Update PX4 allocator at 1 Hz (minimizes matrix rebuild thrashing)
            if (rospy.Time.now() - last_px4_update).to_sec() >= 0.9:
                self.update_px4_morph(angle, force=False)
                last_px4_update = rospy.Time.now()

            # v3 FIX 2: ALWAYS send position setpoint to keep OFFBOARD alive
            # Priority: push setpoint > hold setpoint
            sp_x = push_x if push_x is not None else hold_x
            sp_y = push_y if push_y is not None else hold_y
            sp_z = push_z if push_z is not None else hold_z
            self._send_setpoint(sp_x, sp_y, sp_z)

            rate_vis.sleep()

        self._send_arm_angle_ros(target_angle)
        self.update_px4_morph(target_angle, force=True)
        # Final hold setpoint
        sp_x = push_x if push_x is not None else hold_x
        sp_y = push_y if push_y is not None else hold_y
        sp_z = push_z if push_z is not None else hold_z
        self._send_setpoint(sp_x, sp_y, sp_z)
        print(f"[MORPH] Reached {target_angle:.2f} rad")

    def _phase_push(self, x_start, y, z, x_target, timeout, speed):
        print(f"[PUSH] From x={x_start:.2f} toward x={x_target:.2f} at {speed}m/s")
        rate = rospy.Rate(self.rate_hz)
        start = rospy.Time.now()
        contact_detected = False
        contact_time = None
        push_duration_est = abs(x_target - x_start) / max(speed, 0.01)

        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start).to_sec()
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

            # Stall detection: stuck near pole while setpoint is deep inside
            stalled = False
            if s > 0.9 and abs(act_x - self.POLE_X) < 0.15:
                stalled = True
                print(f"  [STALL] act_x={act_x:.2f}, sp_x={x:.2f}, s={s:.2f}")

            if efo > 1.5 or cstate > 0 or stalled:
                if not contact_detected:
                    contact_detected = True
                    contact_time = t
                    print(f"[CONTACT] Detected at t={t:.2f}s | efo={efo:.2f}N | cstate={cstate} | stalled={stalled}")
                    # Continue sending setpoint briefly to keep OFFBOARD alive while PX4 FSM reacts
                    for _ in range(20):  # 1s @ 20Hz
                        if rospy.is_shutdown():
                            break
                        self._send_setpoint(x, y, z)
                        self._record('push', t, x, y, z)
                        rate.sleep()
                    break

            self._record('push', t, x, y, z)
            rate.sleep()

        return contact_detected, contact_time

    def _phase_retreat(self, x_start, y, z, x_end, duration):
        print(f"[RETREAT] ({x_start:.1f},{y:.1f}) -> ({x_end:.1f},{y:.1f}) over {duration}s")
        rate = rospy.Rate(self.rate_hz)
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start).to_sec()
            if t > duration:
                break
            s = min(1.0, t / duration)
            alpha = _smoothstep(s)
            alpha_d = _smoothstep_deriv(s) / duration
            x = x_start + (x_end - x_start) * alpha
            vx = (x_end - x_start) * alpha_d
            self._send_setpoint(x, y, z, 0.0, vx, 0.0, 0.0)
            self._record('retreat', t, x, y, z)
            rate.sleep()
        return not rospy.is_shutdown()

    # ---------- Main flight ----------

    def run(self):
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

        # v3 FIX 3: Ensure OFFBOARD before morphing
        self._ensure_offboard()

        # Phase 2: Slowly expand arms during hover (with position hold)
        self._morph_arm_slowly(-0.3, 3.0, hold_x=0.0, hold_y=0.0, hold_z=self.HOVER_Z)

        # Re-ensure OFFBOARD after morph
        self._ensure_offboard()
        # Extra setpoint burst to stabilize after morph
        print("[GUARD] Post-morph setpoint burst")
        for _ in range(40):
            if rospy.is_shutdown():
                return None
            self._send_setpoint(0.0, 0.0, self.HOVER_Z)
            rate.sleep()

        # Phase 3: Approach to pole front
        if not self._phase_approach(0.0, 0.0, self.HOVER_Z, self.APPROACH_X, self.APPROACH_Y, self.T_APPROACH):
            return None

        # Phase 4: Push into pole
        contact_detected, contact_time = self._phase_push(
            self.APPROACH_X, self.APPROACH_Y, self.HOVER_Z,
            self.POLE_X, self.T_PUSH, self.PUSH_SPEED
        )

        if contact_detected:
            print(f"[RESULT] Contact detected at t={contact_time:.2f}s")
            # Phase 5: Keep pushing into pole while slowly closing arms
            push_x = self.POLE_X + 0.25  # 25cm past pole to maintain pressure (matches firmware override)
            print(f"[GRASP] Maintaining push at x={push_x:.2f} while closing arms...")
            self._morph_arm_slowly(0.0, 5.0, hold_x=push_x, hold_y=0.0, hold_z=self.HOVER_Z)
            print("[GRASP] Arms closed, maintaining contact...")
            rospy.sleep(2.0)
        else:
            print("[RESULT] No contact detected within timeout")
            self._morph_arm_slowly(0.0, 5.0, hold_x=self.APPROACH_X, hold_y=0.0, hold_z=self.HOVER_Z)

        # Phase 6: Land
        print("[LAND] Landing")
        self.set_mode_client(base_mode=0, custom_mode="AUTO.LAND")
        rospy.sleep(8.0)

        print("[DISARM]")
        self.arming_client(False)
        self._stop_sender()

        return self._save_csv()


def main():
    parser = argparse.ArgumentParser(description='huaqiccc Perching Pole Collision Test v3 (Gazebo Classic)')
    parser.add_argument('--output', default='perching_test', help='Output CSV prefix')
    args = parser.parse_args()

    print("\n" + "=" * 60)
    print("  huaqiccc Perching Pole Collision Test v3 (Gazebo Classic)")
    print("  Pole position: x=5.0, y=0.0")
    print("=" * 60 + "\n")

    test = PerchingFlightTest(output_prefix=args.output)
    csv_path = test.run()
    if csv_path:
        print(f"\n[RESULT] Log saved to: {csv_path}")
    else:
        print("\n[ERROR] Test did not complete or no log saved")


if __name__ == '__main__':
    main()
