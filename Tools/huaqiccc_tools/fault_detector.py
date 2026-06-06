#!/usr/bin/env python3
"""
huaqiccc 仿真失控实时监控与诊断脚本
=====================================
功能:
1. 实时对比 setpoint 与实际位置，计算跟踪误差
2. 检测姿态震荡、角速度异常
3. 检测电机饱和 / 停转
4. 检测 EKF-GroundTruth 不一致（SITL 专用）
5. 检测变形不同步（目标角度 vs 实际关节角度）
6. 异常时自动记录时间戳和事件类型

用法:
    # 终端1: 启动仿真
    roslaunch ~/PX4-Autopilot/launch/mavros_posix_sitl.launch

    # 终端2: 运行监控（先 source ROS 环境）
    source /opt/ros/noetic/setup.bash
    python3 ~/huaqiccc_fault_detector.py

    # 终端3: 运行飞行测试
    python3 ~/Downloads/huaqiccc_flight_test_v4.py
"""

import rospy
import numpy as np
import math
import time
import csv
import os
from datetime import datetime
from collections import deque

from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State, PositionTarget, RCOut
from std_msgs.msg import Float64, String
from sensor_msgs.msg import Imu


class FaultDetector:
    # 阈值配置
    POS_XY_WARN = 1.5       # m
    POS_XY_FAULT = 3.0      # m
    POS_Z_WARN = 1.0        # m
    VEL_MAX_WARN = 5.0      # m/s
    ROLL_WARN = 30.0        # deg
    PITCH_WARN = 30.0       # deg
    ROLL_FAULT = 60.0       # deg
    PITCH_FAULT = 60.0      # deg
    RATE_WARN = 90.0        # deg/s
    MOTOR_MIN_PWM = 950     # us
    MOTOR_MAX_PWM = 1950    # us
    ARM_ANGLE_ERR_WARN = 0.1  # rad
    EKF_GT_DRIFT_WARN = 2.0   # m

    def __init__(self, output_dir=None):
        rospy.init_node('huaqiccc_fault_detector', anonymous=True)
        self.rate_hz = 20.0
        self.dt = 1.0 / self.rate_hz

        self.output_dir = output_dir or os.path.expanduser('~/huaqiccc_logs')
        os.makedirs(self.output_dir, exist_ok=True)

        # 数据缓存（用于滑动窗口统计）
        self.pose_history = deque(maxlen=100)      # 5s @ 20Hz
        self.att_history = deque(maxlen=100)
        self.rate_history = deque(maxlen=100)
        self.motor_history = deque(maxlen=100)

        # 当前状态
        self.state = None
        self.pose = None
        self.setpoint = None
        self.imu = None
        self.motors = None
        self.arm_angle_target = None
        self.arm_angle_actual = None
        self.gt_pose = None
        self.ekf_pose = None

        self.start_time = rospy.Time.now()
        self.events = []
        self.fault_level = 0  # 0=OK, 1=WARN, 2=FAULT, 3=CRITICAL

        self._init_subscribers()
        self._init publishers()

    def _init_subscribers(self):
        rospy.Subscriber('/mavros/state', State, self._cb_state)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._cb_pose)
        rospy.Subscriber('/mavros/local_position/velocity_body', Twist, self._cb_velocity)
        rospy.Subscriber('/mavros/setpoint_raw/local', PositionTarget, self._cb_setpoint)
        rospy.Subscriber('/mavros/imu/data', Imu, self._cb_imu)
        rospy.Subscriber('/mavros/rc/out', RCOut, self._cb_motors)
        rospy.Subscriber('/huaqiccc/arm_angle', Float64, self._cb_arm_target)
        rospy.Subscriber('/huaqiccc/arm_status', String, self._cb_arm_status)
        # SITL ground truth (if available)
        rospy.Subscriber('/gazebo/model_states', PoseStamped, self._cb_gt)

    def _init_publishers(self):
        self.pub_diag = rospy.Publisher('/huaqiccc/fault_diag', String, queue_size=10)

    def _cb_state(self, msg):
        self.state = msg

    def _cb_pose(self, msg):
        self.ekf_pose = msg.pose
        p = msg.pose.position
        o = msg.pose.orientation
        self.pose = (p.x, p.y, p.z, o.x, o.y, o.z, o.w)
        self.pose_history.append((rospy.Time.now().to_sec(), p.x, p.y, p.z))

    def _cb_velocity(self, msg):
        self.velocity = (msg.linear.x, msg.linear.y, msg.linear.z)

    def _cb_setpoint(self, msg):
        self.setpoint = (msg.position.x, msg.position.y, msg.position.z)

    def _cb_imu(self, msg):
        self.imu = msg
        q = msg.orientation
        # quaternion to roll/pitch
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr, cosr)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        self.att_history.append((rospy.Time.now().to_sec(), roll, pitch))
        av = msg.angular_velocity
        self.rate_history.append((av.x, av.y, av.z))

    def _cb_motors(self, msg):
        self.motors = msg.channels[:4]
        self.motor_history.append(msg.channels[:4])

    def _cb_arm_target(self, msg):
        self.arm_angle_target = msg.data

    def _cb_arm_status(self, msg):
        try:
            import json
            d = json.loads(msg.data)
            self.arm_angle_actual = d.get('right_angle', d.get('left_angle'))
        except Exception:
            pass

    def _cb_gt(self, msg):
        self.gt_pose = msg.pose.position

    def _quat_to_rpy(self, qx, qy, qz, qw):
        sinr = 2.0 * (qw * qx + qy * qz)
        cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr, cosr)
        sinp = 2.0 * (qw * qy - qz * qx)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        siny = 2.0 * (qw * qz + qx * qy)
        cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny, cosy)
        return roll, pitch, yaw

    def _log_event(self, level, category, msg_text):
        t = (rospy.Time.now() - self.start_time).to_sec()
        event = {
            'time': round(t, 2),
            'level': level,
            'category': category,
            'message': msg_text,
        }
        self.events.append(event)
        prefix = {"WARN": "[WARN]", "FAULT": "[FAULT]", "CRITICAL": "[CRITICAL]"}.get(level, "[INFO]")
        rospy.logwarn(f"{prefix} t={t:.1f}s {category}: {msg_text}")

        # Publish for external monitoring
        diag = String()
        diag.data = f"{level}|{t:.1f}|{category}|{msg_text}"
        self.pub_diag.publish(diag)

    def _check_position_tracking(self):
        if self.pose is None or self.setpoint is None:
            return
        px, py, pz = self.pose[0], self.pose[1], self.pose[2]
        sx, sy, sz = self.setpoint
        err_xy = math.sqrt((px - sx)**2 + (py - sy)**2)
        err_z = abs(pz - sz)

        if err_xy > self.POS_XY_FAULT:
            self._log_event("CRITICAL", "TRACKING", f"XY deviation {err_xy:.2f}m > {self.POS_XY_FAULT}m")
            self.fault_level = max(self.fault_level, 3)
        elif err_xy > self.POS_XY_WARN:
            self._log_event("WARN", "TRACKING", f"XY deviation {err_xy:.2f}m > {self.POS_XY_WARN}m")
            self.fault_level = max(self.fault_level, 1)

        if err_z > self.POS_Z_WARN and pz < 0.5:
            self._log_event("FAULT", "TRACKING", f"Z deviation {err_z:.2f}m, alt={pz:.2f}m")
            self.fault_level = max(self.fault_level, 2)

    def _check_attitude(self):
        if len(self.att_history) < 10:
            return
        rolls = [a[1] for a in self.att_history]
        pitches = [a[2] for a in self.att_history]
        r_max = max(abs(np.degrees(r)) for r in rolls)
        p_max = max(abs(np.degrees(p)) for p in pitches)
        r_std = np.degrees(np.std(rolls))
        p_std = np.degrees(np.std(pitches))

        if r_max > self.ROLL_FAULT or p_max > self.PITCH_FAULT:
            self._log_event("CRITICAL", "ATTITUDE", f"Extreme attitude: roll={r_max:.1f}° pitch={p_max:.1f}°")
            self.fault_level = max(self.fault_level, 3)
        elif r_max > self.ROLL_WARN or p_max > self.PITCH_WARN:
            self._log_event("WARN", "ATTITUDE", f"Large attitude: roll={r_max:.1f}° pitch={p_max:.1f}°")
            self.fault_level = max(self.fault_level, 1)

        if r_std > 10 or p_std > 10:
            self._log_event("WARN", "ATTITUDE", f"High oscillation: roll_std={r_std:.1f}° pitch_std={p_std:.1f}°")
            self.fault_level = max(self.fault_level, 1)

    def _check_rates(self):
        if len(self.rate_history) < 10:
            return
        rates = np.array(self.rate_history)
        max_rate = np.degrees(np.max(np.abs(rates)))
        if max_rate > self.RATE_WARN:
            self._log_event("WARN", "RATE", f"High angular rate: {max_rate:.1f}°/s")
            self.fault_level = max(self.fault_level, 1)

    def _check_motors(self):
        if self.motors is None:
            return
        min_pwm = min(self.motors)
        max_pwm = max(self.motors)
        if min_pwm < self.MOTOR_MIN_PWM:
            self._log_event("FAULT", "MOTOR", f"Motor near stop: min PWM={min_pwm:.0f}us")
            self.fault_level = max(self.fault_level, 2)
        if max_pwm > self.MOTOR_MAX_PWM:
            self._log_event("WARN", "MOTOR", f"Motor saturation: max PWM={max_pwm:.0f}us")
            self.fault_level = max(self.fault_level, 1)

    def _check_morph_sync(self):
        if self.arm_angle_target is None or self.arm_angle_actual is None:
            return
        err = abs(self.arm_angle_target - self.arm_angle_actual)
        if err > self.ARM_ANGLE_ERR_WARN:
            self._log_event("WARN", "MORPH", f"Arm angle sync error: {err:.3f}rad (target={self.arm_angle_target:.3f}, actual={self.arm_angle_actual:.3f})")
            self.fault_level = max(self.fault_level, 1)

    def _check_ekf_gt_divergence(self):
        if self.ekf_pose is None or self.gt_pose is None:
            return
        dx = abs(self.ekf_pose.position.x - self.gt_pose.x)
        dy = abs(self.ekf_pose.position.y - self.gt_pose.y)
        dz = abs(self.ekf_pose.position.z - self.gt_pose.z)
        drift = math.sqrt(dx**2 + dy**2 + dz**2)
        if drift > self.EKF_GT_DRIFT_WARN:
            self._log_event("CRITICAL", "EKF", f"EKF-GT divergence: {drift:.2f}m")
            self.fault_level = max(self.fault_level, 3)

    def _save_report(self):
        if not self.events:
            return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.output_dir, f"fault_report_{ts}.csv")
        with open(path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['time', 'level', 'category', 'message'])
            writer.writeheader()
            writer.writerows(self.events)
        rospy.loginfo(f"[SAVE] Fault report: {path}")

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo("huaqiccc Fault Detector started. Monitoring...")
        while not rospy.is_shutdown():
            self._check_position_tracking()
            self._check_attitude()
            self._check_rates()
            self._check_motors()
            self._check_morph_sync()
            self._check_ekf_gt_divergence()
            rate.sleep()
        self._save_report()


def main():
    detector = FaultDetector()
    detector.run()


if __name__ == '__main__':
    main()
