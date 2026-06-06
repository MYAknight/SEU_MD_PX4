#!/usr/bin/env python3
"""
Long-contact motor comparison test using ROS/MAVROS.
Based on grasp_16cm.py but simplified for A/B comparison:
  - No arm closing (arms stay expanded)
  - No freeze plugin usage
  - Long contact recording (20-30s)
  - Records motor PWMs via /mavros/rc/out
"""

import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime

try:
    import rospy
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import State, PositionTarget, RCOut
    from mavros_msgs.srv import CommandBool, SetMode, CommandLong
    from sensor_msgs.msg import Imu
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("[FATAL] ROS not available")
    sys.exit(1)


class LongContactTestROS:
    HOVER_Z = 2.5
    POLE_X = 5.0
    POLE_RADIUS = 0.09
    APPROACH_X = 4.75
    T_HOVER0 = 8.0
    T_APPROACH = 12.0
    PUSH_SPEED = 0.05
    RECORD_DURATION = 25.0
    EXPAND_ANGLE = -0.45
    RATE_HZ = 20.0

    def __init__(self, output_prefix='contact_ros', contact_hold=None):
        self.output_prefix = output_prefix
        self.records = []
        if contact_hold is not None:
            self.RECORD_DURATION = float(contact_hold)
        self.rate_hz = self.RATE_HZ
        self.dt = 1.0 / self.rate_hz
        self.current_state = None
        self.current_pose = None
        self.current_imu = None
        self.motor_outputs = None
        self.sp_pub = None
        self.arm_pub = None
        self.arming_client = None
        self.set_mode_client = None
        self.cmd_long_srv = None
        self._last_sent_angle = None

        if not ROS_AVAILABLE:
            sys.exit(1)
        self._init_ros()

    def _init_ros(self):
        try:
            rospy.init_node('contact_compare_ros', anonymous=True)
        except rospy.exceptions.ROSException:
            pass

        self.sp_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        from std_msgs.msg import Float64
        self.arm_pub = rospy.Publisher('/huaqiccc/arm_angle', Float64, queue_size=1)

        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb)
        rospy.Subscriber('/mavros/imu/data', Imu, self._imu_cb)
        rospy.Subscriber('/mavros/rc/out', RCOut, self._rc_out_cb)

        rospy.wait_for_service('/mavros/cmd/arming', timeout=10.0)
        rospy.wait_for_service('/mavros/set_mode', timeout=10.0)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        try:
            rospy.wait_for_service('/mavros/cmd/command', timeout=5.0)
            self.cmd_long_srv = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        except Exception as e:
            print(f"[WARN] cmd/command: {e}")
            self.cmd_long_srv = None

        print("[WAIT] Waiting for FCU connection...")
        dt = 1.0 / self.rate_hz
        while not rospy.is_shutdown() and (self.current_state is None or not self.current_state.connected):
            time.sleep(dt)
        print("[OK] FCU connected")

        # Param cache sync
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

    def _state_cb(self, msg):
        self.current_state = msg

    def _pose_cb(self, msg):
        self.current_pose = msg.pose

    def _imu_cb(self, msg):
        self.current_imu = msg.linear_acceleration

    def _rc_out_cb(self, msg):
        if len(msg.channels) >= 4:
            self.motor_outputs = [msg.channels[0], msg.channels[1], msg.channels[2], msg.channels[3]]

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

    def _send_setpoint(self, x, y, z, yaw=0.0, vx=0.0, vy=0.0, yaw_rate=0.0):
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        if abs(vx) > 1e-6 or abs(vy) > 1e-6:
            msg.type_mask = (
                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE
            )
            msg.velocity.x = vx
            msg.velocity.y = vy
            msg.velocity.z = 0.0
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.yaw = yaw
        self.sp_pub.publish(msg)

    def _record(self, phase, t, sp_x, sp_y, sp_z):
        act = self.current_pose
        imu = self.current_imu
        motors = self.motor_outputs
        ax = round(imu.x, 3) if imu else 0.0
        ay = round(imu.y, 3) if imu else 0.0
        az = round(imu.z, 3) if imu else 0.0
        self.records.append({
            'phase': phase,
            'time': round(t, 3),
            'sp_x': round(sp_x, 5),
            'sp_y': round(sp_y, 5),
            'sp_z': round(sp_z, 5),
            'act_x': round(act.position.x, 5) if act else None,
            'act_y': round(act.position.y, 5) if act else None,
            'act_z': round(act.position.z, 5) if act else None,
            'm0': motors[0] if motors else None,
            'm1': motors[1] if motors else None,
            'm2': motors[2] if motors else None,
            'm3': motors[3] if motors else None,
            'motor_avg': round(sum(motors)/4.0, 2) if motors else None,
            'ax': ax, 'ay': ay, 'az': az,
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
            x = x1 + (x2 - x1) * s
            y = y1 + (y2 - y1) * s
            vx = (x2 - x1) / duration
            vy = (y2 - y1) / duration
            self._send_setpoint(x, y, z, 0.0, vx, vy, 0.0)
            self._record('approach', t, x, y, z)
            time.sleep(dt)
        return not rospy.is_shutdown()

    def _morph_arm(self, target_angle, duration, hold_x=0.0, hold_y=0.0, hold_z=None):
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
            if self.arm_pub:
                from std_msgs.msg import Float64
                msg = Float64()
                msg.data = float(angle)
                self.arm_pub.publish(msg)
            if time.time() - last_px4_update >= 0.9:
                self._send_morph_31440(angle)
                last_px4_update = time.time()
            self._send_setpoint(hold_x, hold_y, hold_z)
            time.sleep(dt_vis)
        self._send_morph_31440(target_angle)
        self._send_setpoint(hold_x, hold_y, hold_z)
        print(f"[MORPH] Reached {target_angle:.2f} rad")

    def _phase_push_and_contact(self, x_start, y, z, x_target, speed):
        print(f"[PUSH] From x={x_start:.2f} toward x={x_target:.2f} at {speed}m/s")
        dt = 1.0 / self.rate_hz
        start = time.time()
        contact_detected = False
        push_duration_est = abs(x_target - x_start) / max(speed, 0.001)
        pole_surface = self.POLE_X - self.POLE_RADIUS
        STALL_DIST = 0.15
        STALL_S = 0.70

        while not rospy.is_shutdown():
            t = time.time() - start
            if t > 20.0:
                print("[PUSH] Timeout reached")
                break

            s = min(1.0, t / push_duration_est)
            x = x_start + (x_target - x_start) * s
            self._send_setpoint(x, y, z, 0.0, speed, 0.0, 0.0)

            act_x = self.current_pose.position.x if self.current_pose else x_start
            if s > STALL_S and abs(act_x - pole_surface) < STALL_DIST:
                contact_detected = True
                print(f"[CONTACT] Detected at x={act_x:.2f}, stall after {t:.1f}s")
                break

            self._record('push', t, x, y, z)
            time.sleep(dt)

        return contact_detected

    def _phase_record_contact(self, x, y, z, duration):
        print(f"[RECORD] Holding contact at ({x:.2f},{y:.2f},{z:.2f}) for {duration:.1f}s")
        dt = 1.0 / self.rate_hz
        start = time.time()
        while not rospy.is_shutdown():
            t = time.time() - start
            if t > duration:
                break
            self._send_setpoint(x, y, z)
            self._record('contact', t, x, y, z)
            time.sleep(dt)
        return not rospy.is_shutdown()

    def run(self):
        rate = rospy.Rate(self.rate_hz)

        print("[PRE] Pre-sending setpoints...")
        if not self._pre_send(0.0, 0.0, self.HOVER_Z, count=100):
            return None

        print("[MODE] OFFBOARD")
        self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")

        print("[ARM] Arming")
        self.arming_client(True)

        if not self._phase_hover(self.T_HOVER0, 0.0, 0.0, self.HOVER_Z):
            return None

        print("[EXPAND] Opening arms wide...")
        self._morph_arm(self.EXPAND_ANGLE, 4.0, hold_x=0.0, hold_y=0.0, hold_z=self.HOVER_Z)

        if not self._phase_approach(0.0, 0.0, self.HOVER_Z, self.APPROACH_X, 0.0, self.T_APPROACH):
            return None

        contact_detected = self._phase_push_and_contact(
            self.APPROACH_X, 0.0, self.HOVER_Z,
            self.POLE_X + 0.15, self.PUSH_SPEED
        )

        if not contact_detected:
            print("[RESULT] No contact detected")
            self._morph_arm(0.0, 4.0, hold_x=self.APPROACH_X, hold_y=0.0, hold_z=self.HOVER_Z)
            self.set_mode_client(base_mode=0, custom_mode="AUTO.LAND")
            rospy.sleep(8.0)
            self.arming_client(False)
            return self._save_csv(suffix='_no_contact')

        print(f"[RESULT] Contact detected")

        # Hold contact position (do NOT close arms, do NOT freeze)
        hold_x = self.POLE_X + 0.25
        if not self._phase_record_contact(hold_x, 0.0, self.HOVER_Z, self.RECORD_DURATION):
            return None

        # Land
        print("[LAND] Landing...")
        self.set_mode_client(base_mode=0, custom_mode="AUTO.LAND")
        rospy.sleep(8.0)
        self.arming_client(False)

        path = self._save_csv(suffix='_contact')

        # Print summary
        contact_rows = [r for r in self.records if r['phase'] == 'contact']
        if contact_rows:
            motors_avg = [r['motor_avg'] for r in contact_rows if r['motor_avg'] is not None]
            pos_x = [r['act_x'] for r in contact_rows if r['act_x'] is not None]
            if motors_avg:
                print("\n" + "=" * 50)
                print("  CONTACT PHASE SUMMARY")
                print("=" * 50)
                print(f"  Duration:     {len(contact_rows) * self.dt:.1f}s")
                print(f"  Motor avg:    {sum(motors_avg)/len(motors_avg):.1f} PWM")
                print(f"  Motor min:    {min(motors_avg):.1f} PWM")
                print(f"  Motor max:    {max(motors_avg):.1f} PWM")
                if pos_x:
                    print(f"  Pos x mean:   {sum(pos_x)/len(pos_x):.3f}m")
                print("=" * 50)

        return path


def main():
    parser = argparse.ArgumentParser(description='Long-contact comparison test (ROS/MAVROS)')
    parser.add_argument('--output', default='contact_ros', help='Output CSV prefix')
    parser.add_argument('--contact-hold', type=float, default=None, help='Contact hold duration in seconds')
    args = parser.parse_args()

    test = LongContactTestROS(output_prefix=args.output, contact_hold=args.contact_hold)
    path = test.run()
    if path:
        print(f"\n[DONE] Log: {path}")
    else:
        print("\n[FAIL] Test did not complete")


if __name__ == '__main__':
    main()
