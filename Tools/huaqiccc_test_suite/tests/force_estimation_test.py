#!/usr/bin/env python3
"""
Minimal force-estimation validation for scheme A.
Drone hovers in OFFBOARD position hold; a known horizontal force is applied
via Gazebo /gazebo/apply_body_wrench. PX4 debug_float_array data[7] records
f_est in real time. The resulting CSV can be plotted to compare f_est with
the applied force.
"""
import csv
import os
import sys
import time
import math
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, Wrench, Vector3, Point
from mavros_msgs.msg import DebugValue, State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from gazebo_msgs.srv import ApplyBodyWrench
from std_msgs.msg import Header

class ForceEstimationTest:
    def __init__(self):
        rospy.init_node('force_estimation_test', anonymous=True)

        self.rate_hz = 50
        self.rate = rospy.Rate(self.rate_hz)

        # Subscribers
        self.state_sub = rospy.Subscriber('/mavros/state', State, self._state_cb)
        self.local_pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pos_cb)
        self.local_vel_sub = rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self._local_vel_cb)
        self.debug_sub = rospy.Subscriber('/mavros/debug_value/debug_float_array', DebugValue, self._debug_cb)

        # Publishers
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        # Services
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # Gazebo wrench service
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        self.apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

        self.current_state = State()
        self.current_pose = None
        self.current_vel = None
        self.f_est = 0.0
        self.delta_p = 0.0
        self.pitch_deg = 0.0
        self.px4_phase = -1
        self.f_total_forward = 0.0
        self.f_thrust_forward = 0.0
        self.acc_forward = 0.0
        self.thrust_z = 0.0

        self.records = []

    def _state_cb(self, msg):
        self.current_state = msg

    def _local_pos_cb(self, msg):
        self.current_pose = msg

    def _local_vel_cb(self, msg):
        self.current_vel = msg

    def _debug_cb(self, msg):
        if not hasattr(msg, 'data'):
            return
        if len(msg.data) < 14:
            rospy.logwarn_throttle(1.0, f"Debug array too short: len={len(msg.data)}")
            return
        self.px4_phase = int(round(msg.data[1]))
        self.delta_p = msg.data[6]
        self.f_est = msg.data[7]
        self.pitch_deg = msg.data[9]
        self.f_total_forward = msg.data[10]
        self.f_thrust_forward = msg.data[11]
        self.acc_forward = msg.data[12]
        self.thrust_z = msg.data[13]

    def _make_setpoint(self, x, y, z, yaw=0.0):
        sp = PositionTarget()
        sp.header = Header()
        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = "map"
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        sp.position.x = x
        sp.position.y = y
        sp.position.z = z
        sp.yaw = yaw
        return sp

    def _send_setpoint(self, x, y, z, yaw=0.0):
        self.setpoint_pub.publish(self._make_setpoint(x, y, z, yaw))

    def _arm(self):
        rospy.loginfo("Arming...")
        # Keep streaming setpoints while arming so OFFBOARD stays valid.
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < 15.0 and not rospy.is_shutdown():
            self._send_setpoint(0, 0, 2.0)
            if self.current_state.armed:
                return True
            if (rospy.Time.now() - start).to_sec() > 1.0:
                self.arming_client(value=True)
            self.rate.sleep()
        return self.current_state.armed

    def _set_offboard(self):
        rospy.loginfo("Pre-sending setpoints...")
        for _ in range(100):
            self._send_setpoint(0, 0, 2.0)
            self.rate.sleep()
        rospy.loginfo("Setting OFFBOARD...")
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < 10.0 and not rospy.is_shutdown():
            self._send_setpoint(0, 0, 2.0)
            self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            if self.current_state.mode == "OFFBOARD":
                rospy.loginfo("OFFBOARD enabled")
                return True
            self.rate.sleep()
        rospy.logerr("OFFBOARD failed, current mode: %s", self.current_state.mode)
        return False

    def _takeoff(self, target_z=2.0):
        rospy.loginfo(f"Taking off to z={target_z}...")
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < 20.0 and not rospy.is_shutdown():
            self._send_setpoint(0, 0, target_z)
            if self.current_pose and self.current_pose.pose.position.z > target_z * 0.85:
                rospy.loginfo("Takeoff complete")
                # Stream a bit more to stabilize
                for _ in range(50):
                    self._send_setpoint(0, 0, target_z)
                    self.rate.sleep()
                return True
            self.rate.sleep()
        return False

    def _apply_force(self, fx, fy, duration_sec):
        """Apply a body wrench in the world frame for duration_sec seconds."""
        rospy.loginfo(f"Applying force ({fx:.1f}, {fy:.1f}) N for {duration_sec:.1f}s...")
        body_name = "huaqiccc::base_link"
        # Empty reference_frame -> force expressed in world/inertial frame
        wrench = Wrench(force=Vector3(fx, fy, 0.0), torque=Vector3(0, 0, 0))
        try:
            self.apply_wrench(
                body_name=body_name,
                reference_frame="",
                reference_point=Point(0, 0, 0),
                wrench=wrench,
                start_time=rospy.Time.now(),
                duration=rospy.Duration(duration_sec)
            )
        except rospy.ServiceException as e:
            rospy.logerr(f"apply_body_wrench failed: {e}")

    def _record(self, applied_fx, applied_fy, label):
        if self.current_pose is None or self.current_vel is None:
            return
        p = self.current_pose.pose.position
        q = self.current_pose.pose.orientation
        v = self.current_vel.twist.linear
        self.records.append({
            'time': rospy.Time.now().to_sec(),
            'label': label,
            'applied_fx': applied_fx,
            'applied_fy': applied_fy,
            'f_est': self.f_est,
            'delta_p': self.delta_p,
            'pitch_deg': self.pitch_deg,
            'px4_phase': self.px4_phase,
            'f_total_forward': self.f_total_forward,
            'f_thrust_forward': self.f_thrust_forward,
            'acc_forward': self.acc_forward,
            'thrust_z': self.thrust_z,
            'x': p.x, 'y': p.y, 'z': p.z,
            'vx': v.x, 'vy': v.y, 'vz': v.z,
            'qw': q.w, 'qx': q.x, 'qy': q.y, 'qz': q.z,
        })

    def _hover(self, duration_sec, applied_fx=0.0, applied_fy=0.0, label="hover"):
        rospy.loginfo(f"Hovering for {duration_sec:.1f}s [{label}]...")
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < duration_sec and not rospy.is_shutdown():
            if self.current_pose:
                self._send_setpoint(0, 0, 2.0)
            self._record(applied_fx, applied_fy, label)
            self.rate.sleep()

    def _save_csv(self, path):
        if not self.records:
            rospy.logwarn("No records to save")
            return
        fieldnames = ['time', 'label', 'applied_fx', 'applied_fy', 'f_est',
                      'delta_p', 'pitch_deg', 'px4_phase',
                      'f_total_forward', 'f_thrust_forward', 'acc_forward', 'thrust_z',
                      'x', 'y', 'z', 'vx', 'vy', 'vz', 'qw', 'qx', 'qy', 'qz']
        with open(path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.records)
        rospy.loginfo(f"Saved {len(self.records)} records to {path}")

    def run(self, force_levels=None, force_duration=2.0):
        if force_levels is None:
            force_levels = [2.0, 5.0, 10.0]

        # Match grasp_16cm.py order: pre-send -> OFFBOARD -> arm -> takeoff
        if not self._set_offboard():
            rospy.logerr("OFFBOARD failed")
            return False
        if not self._arm():
            rospy.logerr("Arming failed")
            return False
        if not self._takeoff(2.0):
            rospy.logerr("Takeoff failed")
            return False

        # Baseline hover
        self._hover(3.0, label="baseline")

        # Apply each force level along body x (world +x because yaw=0)
        for fx in force_levels:
            self._hover(1.0, label=f"pre_{fx}N")
            self._apply_force(fx, 0.0, force_duration)
            self._hover(force_duration + 1.0, applied_fx=fx, label=f"force_{fx}N")
            self._hover(2.0, label=f"recover_{fx}N")

        rospy.loginfo("Test complete, landing...")
        # Descend
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < 5.0 and not rospy.is_shutdown():
            self._send_setpoint(0, 0, 0.3)
            self._record(0, 0, "land")
            self.rate.sleep()

        self.arming_client(value=False)

        # Save
        out_dir = os.path.expanduser("~/huaqiccc_force_test")
        os.makedirs(out_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(out_dir, f"force_est_test_{timestamp}.csv")
        self._save_csv(csv_path)
        return csv_path


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Force estimation validation test')
    parser.add_argument('--forces', nargs='+', type=float, default=[2.0, 5.0, 10.0],
                        help='List of force levels in Newtons')
    parser.add_argument('--duration', type=float, default=2.0,
                        help='Duration of each force application in seconds')
    args = parser.parse_args()

    test = ForceEstimationTest()
    csv_path = test.run(force_levels=args.forces, force_duration=args.duration)
    if csv_path:
        rospy.loginfo(f"CSV: {csv_path}")
        # Simple text summary
        import pandas as pd
        df = pd.read_csv(csv_path)
        print("\n=== Force estimation summary ===")
        for label, group in df.groupby('label'):
            if 'force_' in label:
                fx = group['applied_fx'].iloc[0]
                fmean = group['f_est'].mean()
                fmax = group['f_est'].max()
                print(f"{label}: applied={fx:.1f} N, f_est mean={fmean:.2f} N, max={fmax:.2f} N")
    else:
        sys.exit(1)


if __name__ == '__main__':
    main()
