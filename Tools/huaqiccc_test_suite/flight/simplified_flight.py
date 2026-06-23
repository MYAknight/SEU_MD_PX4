#!/usr/bin/env python3
"""
huaqiccc_flight_test.py v4.1 (Feedforward)
==========================================
变形无人机统一飞行测试脚本（v4.1 - 圆轨迹 + 31440 + 速度/加速度前馈）

【v4.1 更新】
1. Setpoint 从 PoseStamped 切换为 mavros_msgs/PositionTarget
2. 通过数值微分实时计算 vx,vy,ax,ay 作为前馈注入 PX4 MPC
3. 坐标系仍为 ENU（z向上），MAVROS 自动转换 NED 发给飞控
4. 发布 Topic 改为 /mavros/setpoint_raw/local

飞行流程：同 v4.0
"""

import argparse
import csv
import math
import os
import sys
import threading
import time
from datetime import datetime
from tf.transformations import quaternion_from_euler, euler_from_quaternion

try:
    import rospy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Imu
    from mavros_msgs.msg import State, PositionTarget
    from mavros_msgs.srv import CommandBool, SetMode
    from mavros_msgs.srv import CommandLong
    from std_msgs.msg import Float64
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


# ===================== 轨迹生成器 =====================

def _smoothstep(t):
    """C1 连续 smoothstep: [0,1] → [0,1], 边界导数为 0"""
    if t <= 0.0:
        return 0.0
    if t >= 1.0:
        return 1.0
    return t * t * (3.0 - 2.0 * t)


def _smoothstep_deriv(t):
    """smoothstep 对 t 的导数"""
    if t <= 0.0 or t >= 1.0:
        return 0.0
    return 6.0 * t * (1.0 - t)


class TrajectoryGenerator:
    """
    平滑轨迹：悬停 → 展开 → 圆(螺旋进→匀速→螺旋退) → 收拢 → 悬停 → 降落
    所有阶段的位置、速度、yaw、yaw_rate 均保证 C0/C1 连续
    """

    HOVER_Z = 2.0
    RADIUS = 1.0
    CIRCLE_PERIOD = 20.0

    # 阶段时间
    T_HOVER0 = 10.0
    T_EXPAND = 8.0
    T_CIRCLE = 20.0
    T_CLOSE = 8.0
    T_HOVER1 = 3.0
    T_DESCEND = 3.0
    T_RAMP = 3.0  # 螺旋进/退的过渡时间

    def __init__(self):
        self.total_time = (
            self.T_HOVER0 + self.T_EXPAND + self.T_CIRCLE +
            self.T_CLOSE + self.T_HOVER1 + self.T_DESCEND
        )
        omega = 2.0 * math.pi / self.CIRCLE_PERIOD
        # 圆结束时的切线方向 yaw = omega*T_CIRCLE + pi/2 = 2pi + pi/2
        # 选择最近的 2pi 整数倍作为着陆参考 yaw，避免 CLOSE/HOVER1/DESCEND 与 0 出现数值跳变
        yaw_circle_end = omega * self.T_CIRCLE + math.pi / 2.0
        self.yaw_land = 2.0 * math.pi * round(yaw_circle_end / (2.0 * math.pi))

    def get_state(self, t):
        """返回: x, y, z, arm_angle, yaw, vx, vy, yaw_rate"""
        z = self.HOVER_Z
        omega = 2.0 * math.pi / self.CIRCLE_PERIOD
        T_R = self.T_RAMP

        if t < self.T_HOVER0:
            # ========== HOVER0 ==========
            x, y, arm = 0.0, 0.0, 0.0
            yaw = 0.0
            vx, vy = 0.0, 0.0
            yaw_rate = 0.0

        elif t < self.T_HOVER0 + self.T_EXPAND:
            # ========== EXPAND ==========
            tau = t - self.T_HOVER0
            progress = tau / self.T_EXPAND
            arm = -0.3 * progress
            x, y = 0.0, 0.0
            yaw = 0.0
            vx, vy = 0.0, 0.0
            yaw_rate = 0.0

        elif t < self.T_HOVER0 + self.T_EXPAND + self.T_CIRCLE:
            # ========== CIRCLE ==========
            tau = t - self.T_HOVER0 - self.T_EXPAND  # tau in [0, T_CIRCLE]
            arm = -0.3
            theta = omega * tau

            if tau < T_R:
                # ---- ENTRY: 螺旋进入圆 + yaw 从 0 平滑过渡到切线方向 ----
                s = tau / T_R
                alpha = _smoothstep(s)
                alpha_d = _smoothstep_deriv(s) / T_R

                r = self.RADIUS * alpha
                r_d = self.RADIUS * alpha_d

                x = r * math.cos(theta)
                y = r * math.sin(theta)
                vx = r_d * math.cos(theta) - r * omega * math.sin(theta)
                vy = r_d * math.sin(theta) + r * omega * math.cos(theta)

                yaw_target = theta + math.pi / 2.0
                yaw = yaw_target * alpha          # 从 0 平滑开始
                yaw_rate = omega * alpha + yaw_target * alpha_d

            elif tau < self.T_CIRCLE - T_R:
                # ---- STEADY: 匀速圆 ----
                r = self.RADIUS
                x = r * math.cos(theta)
                y = r * math.sin(theta)
                vx = -r * omega * math.sin(theta)
                vy =  r * omega * math.cos(theta)
                yaw = theta + math.pi / 2.0
                yaw_rate = omega

            else:
                # ---- EXIT: 螺旋退回到原点 + yaw 平滑过渡到 yaw_land ----
                tau_exit = tau - (self.T_CIRCLE - T_R)
                s = tau_exit / T_R
                alpha = _smoothstep(s)
                alpha_d = _smoothstep_deriv(s) / T_R

                r = self.RADIUS * (1.0 - alpha)
                r_d = -self.RADIUS * alpha_d

                x = r * math.cos(theta)
                y = r * math.sin(theta)
                vx = r_d * math.cos(theta) - r * omega * math.sin(theta)
                vy = r_d * math.sin(theta) + r * omega * math.cos(theta)

                yaw_start = omega * (self.T_CIRCLE - T_R) + math.pi / 2.0
                yaw = yaw_start + (self.yaw_land - yaw_start) * alpha
                yaw_rate = (self.yaw_land - yaw_start) * alpha_d

        elif t < self.T_HOVER0 + self.T_EXPAND + self.T_CIRCLE + self.T_CLOSE:
            # ========== CLOSE ==========
            tau = t - self.T_HOVER0 - self.T_EXPAND - self.T_CIRCLE
            progress = tau / self.T_CLOSE
            arm = -0.3 * (1.0 - progress)
            x, y = 0.0, 0.0
            vx, vy = 0.0, 0.0
            yaw = self.yaw_land
            yaw_rate = 0.0

        elif t < self.T_HOVER0 + self.T_EXPAND + self.T_CIRCLE + self.T_CLOSE + self.T_HOVER1:
            # ========== HOVER1 ==========
            arm = 0.0
            x, y = 0.0, 0.0
            yaw = self.yaw_land
            vx, vy = 0.0, 0.0
            yaw_rate = 0.0

        elif t < self.total_time:
            # ========== DESCEND ==========
            tau = t - (self.T_HOVER0 + self.T_EXPAND + self.T_CIRCLE + self.T_CLOSE + self.T_HOVER1)
            progress = tau / self.T_DESCEND
            z = self.HOVER_Z - (self.HOVER_Z - 0.2) * progress
            arm = 0.0
            x, y = 0.0, 0.0
            yaw = self.yaw_land
            vx, vy = 0.0, 0.0
            yaw_rate = 0.0

        else:
            x, y, z, arm, yaw = 0.0, 0.0, 0.2, 0.0, self.yaw_land
            vx, vy = 0.0, 0.0
            yaw_rate = 0.0

        return x, y, z, arm, yaw, vx, vy, yaw_rate


# ===================== 核心测试类 =====================

class UnifiedFlightTest:

    def __init__(self, enable_algo=True, rate_hz=20.0, output_prefix='huaqiccc_flight'):
        self.enable_algo = enable_algo
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz
        self.output_prefix = output_prefix

        self.traj_gen = TrajectoryGenerator()
        self.records = []
        self.start_time = None

        self.current_state = None
        self.current_pose = None
        self.current_imu = None
        self.sp_pub = None
        self.arm_pub = None
        self.arming_client = None
        self.set_mode_client = None
        self.cmd_long_srv = None

        # v4.0: 31440 后台发送
        self._pending_angle_lock = threading.Lock()
        self._pending_angle = None
        self._sender_stop = threading.Event()
        self._sender_thread = None

        if not ROS_AVAILABLE:
            print("[FATAL] ROS 未安装")
            sys.exit(1)

        self._init_ros()
        self._start_sender()

    def _init_ros(self):
        rospy.init_node('huaqiccc_flight_test', anonymous=True)

        # v4.1: 改用 setpoint_raw/local 发布 PositionTarget
        self.sp_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.arm_pub = rospy.Publisher('/huaqiccc/arm_angle', Float64, queue_size=1)

        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb)
        rospy.Subscriber('/mavros/imu/data', Imu, self._imu_cb)

        rospy.wait_for_service('/mavros/cmd/arming', timeout=10.0)
        rospy.wait_for_service('/mavros/set_mode', timeout=10.0)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        if self.enable_algo:
            try:
                rospy.wait_for_service('/mavros/cmd/command', timeout=5.0)
                self.cmd_long_srv = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
                print("[OK] MAVROS cmd/command 已连接")
            except Exception as e:
                print(f"[WARN] cmd/command: {e}")
                self.cmd_long_srv = None

        print("[WAIT] 等待 FCU 连接...")
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and (self.current_state is None or not self.current_state.connected):
            rate.sleep()
        print("[OK] FCU 已连接")

        # Wait for MAVROS param cache to sync from PX4
        print("[WAIT] 等待 MAVROS 参数同步...")
        rospy.sleep(5.0)
        try:
            from mavros_msgs.srv import ParamPull
            rospy.wait_for_service('/mavros/param/pull', timeout=10.0)
            param_pull = rospy.ServiceProxy('/mavros/param/pull', ParamPull)
            pull_resp = param_pull(False)
            if pull_resp.success:
                print(f"[OK] 参数同步完成，共 {pull_resp.param_received} 个参数")
            else:
                print("[WARN] 参数 pull 未成功，继续等待...")
            rospy.sleep(3.0)
        except Exception as e:
            print(f"[WARN] 参数 pull 跳过: {e}")
            rospy.sleep(5.0)

        # Set MPCA_MODE from environment
        import os
        mpca_mode = int(os.environ.get('MPCA_MODE', '0'))
        
        # Attempt to tune MPC gains via MAVROS param service
        try:
            from mavros_msgs.srv import ParamSet
            from mavros_msgs.msg import ParamValue
            rospy.wait_for_service('/mavros/param/set', timeout=30.0)
            param_set = rospy.ServiceProxy('/mavros/param/set', ParamSet)
            tuned = []
            
            # Set MPCA_MODE first with retries
            for attempt in range(5):
                resp = param_set(param_id='MPCA_MODE', value=ParamValue(integer=mpca_mode))
                if resp.success:
                    tuned.append('MPCA_MODE')
                    print(f"[OK] Set MPCA_MODE = {mpca_mode} (attempt {attempt+1})")
                    break
                else:
                    print(f"[WARN] MPCA_MODE set attempt {attempt+1} returned success=False, retrying...")
                    rospy.sleep(1.0)
            else:
                print(f"[WARN] MPCA_MODE set failed after 5 attempts (PX4 may already have correct default from ROMFS)")
            
            # SITL can tolerate higher XY P gain than real hardware
            mpc_xy_p = float(os.environ.get('MPC_XY_P', '4.0'))
            mpc_xy_vel_p_acc = float(os.environ.get('MPC_XY_VEL_P_ACC', '1.8'))
            mpc_xy_vel_i_acc = float(os.environ.get('MPC_XY_VEL_I_ACC', '0.4'))
            mpc_xy_vel_d_acc = float(os.environ.get('MPC_XY_VEL_D_ACC', '0.2'))
            mpca_ff_mass = float(os.environ.get('MPCA_FF_MASS', '1.5'))
            mpca_mpc_alpha = float(os.environ.get('MPCA_MPC_ALPHA', '5.0'))
            mpca_ff_en = int(os.environ.get('MPCA_FF_EN', '1'))
            # INT32 params must be sent via ParamValue(integer=...)
            int_params = {'MPCA_FF_EN'}
            for pname, pval in [
                ('MPC_XY_P', mpc_xy_p),
                ('MPC_XY_VEL_P_ACC', mpc_xy_vel_p_acc),
                ('MPC_XY_VEL_I_ACC', mpc_xy_vel_i_acc),
                ('MPC_XY_VEL_D_ACC', mpc_xy_vel_d_acc),
                ('MPCA_FF_MASS', mpca_ff_mass),
                ('MPCA_MPC_ALPHA', mpca_mpc_alpha),
                ('MPCA_FF_EN', mpca_ff_en),
            ]:
                if pname in int_params:
                    resp = param_set(param_id=pname, value=ParamValue(integer=int(pval)))
                else:
                    resp = param_set(param_id=pname, value=ParamValue(real=float(pval)))
                if resp.success:
                    tuned.append(pname)
                    print(f"[OK] Set {pname} = {pval}")
                else:
                    print(f"[WARN] {pname} set failed (success=False)")
            if tuned:
                rospy.sleep(3.0)  # Allow params to propagate
        except Exception as e:
            print(f"[WARN] Param tuning skipped: {e}")

    def _state_cb(self, msg):
        self.current_state = msg

    def _pose_cb(self, msg):
        self.current_pose = msg.pose

    def _imu_cb(self, msg):
        self.current_imu = msg

    # ---------- v4.0: 31440 原子发送 ----------

    def send_morph_31440(self, arm_angle):
        if not self.cmd_long_srv:
            print(f"  [PRINT] 31440 angle={arm_angle:.3f}")
            return True
        try:
            from mavros_msgs.srv import CommandLongRequest
            req = CommandLongRequest()
            req.broadcast = False
            req.command = 31440
            req.confirmation = 0
            req.param1 = float(arm_angle)
            resp = self.cmd_long_srv(req)
            ok = getattr(resp, 'success', False)
            status = "OK" if ok else "FAIL"
            print(f"  [31440 {status}] angle={arm_angle:.3f}")
            return ok
        except Exception as e:
            print(f"  [ERROR] 31440: {e}")
            return False

    def _start_sender(self):
        self._sender_thread = threading.Thread(target=self._sender_loop, daemon=True)
        self._sender_thread.start()
        print("[OK] 后台 31440 发送线程已启动")

    def _sender_loop(self):
        while not self._sender_stop.is_set() and not rospy.is_shutdown():
            angle = None
            with self._pending_angle_lock:
                if self._pending_angle is not None:
                    angle = self._pending_angle
                    self._pending_angle = None
            if angle is not None:
                self.send_morph_31440(angle)
            else:
                time.sleep(0.02)

    def _stop_sender(self):
        self._sender_stop.set()
        if self._sender_thread and self._sender_thread.is_alive():
            self._sender_thread.join(timeout=2.0)

    def update_px4_params(self, arm_angle, force=False):
        if not self.enable_algo:
            return False
        if not force and getattr(self, '_last_sent_angle', None) is not None:
            if abs(arm_angle - self._last_sent_angle) < 0.03:
                return False
        self._last_sent_angle = arm_angle
        with self._pending_angle_lock:
            self._pending_angle = arm_angle
        print(f"\n[31440 QUEUE] angle={arm_angle:.3f}")
        return True

    # ---------- v4.1: PositionTarget 构造 ----------

    def _make_position_target(self, x, y, z, yaw, vx=0.0, vy=0.0, yaw_rate=0.0):
        """
        PositionTarget with optional velocity/yaw_rate feedforward
        """
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

    def _send_setpoint(self, x, y, z, yaw, vx=0.0, vy=0.0, yaw_rate=0.0):
        self.sp_pub.publish(self._make_position_target(x, y, z, yaw, vx, vy, yaw_rate))

    def send_arm_angle(self, angle):
        if self.arm_pub:
            msg = Float64()
            msg.data = float(angle)
            self.arm_pub.publish(msg)

    def _record(self, t, sp_x, sp_y, sp_z, arm_angle):
        act = self.current_pose
        if act is None:
            return
        record = {
            'time': round(t, 3),
            'sp_x': round(sp_x, 5), 'sp_y': round(sp_y, 5), 'sp_z': round(sp_z, 5),
            'act_x': round(act.position.x, 5), 'act_y': round(act.position.y, 5), 'act_z': round(act.position.z, 5),
            'err_x': round(act.position.x - sp_x, 5),
            'err_y': round(act.position.y - sp_y, 5),
            'err_z': round(act.position.z - sp_z, 5),
            'arm_angle': round(arm_angle, 4),
            'enable_algo': 1 if self.enable_algo else 0,
        }
        # Attitude data from IMU
        imu = self.current_imu
        if imu is not None:
            q = imu.orientation
            roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            record['roll_deg'] = round(math.degrees(roll), 4)
            record['pitch_deg'] = round(math.degrees(pitch), 4)
            record['yaw_deg'] = round(math.degrees(yaw), 4)
            record['p_dps'] = round(math.degrees(imu.angular_velocity.x), 4)
            record['q_dps'] = round(math.degrees(imu.angular_velocity.y), 4)
            record['r_dps'] = round(math.degrees(imu.angular_velocity.z), 4)
        else:
            record['roll_deg'] = record['pitch_deg'] = record['yaw_deg'] = None
            record['p_dps'] = record['q_dps'] = record['r_dps'] = None
        self.records.append(record)

    def _save_csv(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        algo_str = "with_algo" if self.enable_algo else "baseline"
        out_dir = os.path.join(os.path.expanduser("~"), "huaqiccc_logs")
        os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.join(out_dir, f"{self.output_prefix}_{algo_str}_{ts}.csv")
        if not self.records:
            return None
        with open(out_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.records[0].keys())
            writer.writeheader()
            writer.writerows(self.records)
        print(f"[SAVE] {out_path}")
        return out_path

    # ---------- 核心飞行流程 ----------

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        total_time = self.traj_gen.total_time
        print(f"[INFO] Total flight time: {total_time:.1f}s")

        # Step 1: Pre-send hover setpoints
        print("[PRE] Pre-sending setpoints...")
        for _ in range(100):
            if rospy.is_shutdown():
                return None
            self._send_setpoint(0.0, 0.0, 2.0, 0.0)
            rate.sleep()

        # Step 2: OFFBOARD
        print("[MODE] OFFBOARD")
        self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")

        # Step 3: ARM
        print("[ARM] Arming")
        self.arming_client(True)

        # Step 4: Takeoff hover
        print("[TAKEOFF] Takeoff hover")
        for _ in range(100):
            if rospy.is_shutdown():
                return None
            self._send_setpoint(0.0, 0.0, 2.0, 0.0)
            rate.sleep()

        # Step 5: Main flight loop (simplified trajectory, no feedforward)
        print("[FLIGHT] Starting trajectory")
        self.start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - self.start_time).to_sec()
            if elapsed > total_time:
                break

            x, y, z, arm_angle, yaw, vx, vy, yaw_rate = self.traj_gen.get_state(elapsed)

            self._send_setpoint(x, y, z, yaw, vx, vy, yaw_rate)
            self.send_arm_angle(arm_angle)
            self.update_px4_params(arm_angle, force=False)
            self._record(elapsed, x, y, z, arm_angle)
            rate.sleep()

        # Step 6: Land
        print("[LAND] Landing")
        for _ in range(60):
            if rospy.is_shutdown():
                break
            self._send_setpoint(0.0, 0.0, 0.2, 0.0)
            self.send_arm_angle(0.0)
            rate.sleep()

        print("[MODE] AUTO.LAND")
        self.set_mode_client(base_mode=0, custom_mode="AUTO.LAND")
        rospy.sleep(3.0)

        print("[DISARM] Disarm")
        try:
            self.arming_client(False)
        except Exception as e:
            print(f"[WARN] Disarm failed: {e}, continuing...")
        self._stop_sender()

        return self._save_csv()


def main():
    parser = argparse.ArgumentParser(description='huaqiccc 统一飞行测试 v4.1 (Feedforward)')
    parser.add_argument('--enable-algo', dest='enable_algo', action='store_true', default=True)
    parser.add_argument('--no-enable-algo', dest='enable_algo', action='store_false')
    parser.add_argument('--rate', type=int, default=20, help='发布频率(Hz)，建议 50')
    parser.add_argument('--output', default='huaqiccc_flight', help='输出前缀')

    args = parser.parse_args()
    mode_str = "with_algo" if args.enable_algo else "baseline"
    print(f"\n{'='*60}")
    print(f"  huaqiccc 统一飞行测试 v4.1 (Feedforward) | {mode_str}")
    print(f"  轨迹: 悬停→变形→圆×3 | 31440 原子更新 | 速度/加速度前馈")
    print(f"{'='*60}\n")

    test = UnifiedFlightTest(enable_algo=args.enable_algo, rate_hz=args.rate, output_prefix=args.output)
    csv_path = test.run()
    if csv_path:
        print(f"\n[RESULT] {csv_path}")


if __name__ == '__main__':
    main()
