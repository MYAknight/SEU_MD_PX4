#!/usr/bin/env python3
"""
广义动量观测器 (GMO) 离线验证脚本
使用 PX4 ulog 日志数据验证 GMO 算法和接触检测逻辑

运行方式:
    python3 gmo_offline_test.py <ulog_file>
    python3 gmo_offline_test.py /path/to/log.ulg --contact-time 10.0 --contact-force 2.0
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pyulog import ULog
# scipy not available; using pure numpy quaternion-to-rotation helper
import argparse
import os


class GMOEstimator:
    """广义动量观测器 - Python 原型"""

    def __init__(self, mass=1.173, inertia_diag=(0.015, 0.015, 0.025),
                 observer_gain=50.0, dt=0.001):
        self.m = mass
        self.I = np.diag(inertia_diag)
        self.K_O = observer_gain
        self.dt = dt
        self.alpha = np.exp(-self.K_O * self.dt)

        self.F_est = np.zeros(3)
        self.tau_est = np.zeros(3)
        self.gyro_prev = np.zeros(3)
        self.initialized = False

    def update(self, accel, gyro, gyro_dot, thrust_cmd, torque_cmd, R_bw):
        if not self.initialized:
            self.gyro_prev = gyro.copy()
            self.initialized = True
            return self.F_est.copy(), self.tau_est.copy()

        # 重力在 body frame 的分量
        gravity_world = np.array([0.0, 0.0, 9.80665])
        gravity_body = R_bw.T @ gravity_world

        # 力残差: m*a - F_thrust - m*g
        # 注意: PX4 NED 坐标系中 Z 轴向下，重力为 +9.81
        # accel 是 IMU 测得的比力 (specific force)，包含重力
        # 在 body frame 中: accel = (F_thrust + F_ext)/m - g_body
        # 因此: F_residual = m * accel + m * g_body - F_thrust = F_ext
        F_residual = self.m * accel + self.m * gravity_body - thrust_cmd

        # 力矩残差: I*ω̇ - τ_motor
        tau_residual = self.I @ gyro_dot - torque_cmd

        # 一阶低通滤波
        self.F_est = self.alpha * self.F_est + (1.0 - self.alpha) * F_residual
        self.tau_est = self.alpha * self.tau_est + (1.0 - self.alpha) * tau_residual

        self.gyro_prev = gyro.copy()
        return self.F_est.copy(), self.tau_est.copy()


class ContactDetector:
    """接触检测状态机"""

    NO_CONTACT = 0
    POSSIBLE_CONTACT = 1
    CONFIRMED_CONTACT = 2
    STABLE_PERCHING = 3
    SLIPPING = 4

    STATE_NAMES = ['无接触', '疑似接触', '确认接触', '稳定栖息', '滑动']

    def __init__(self, force_threshold=0.5, time_threshold=0.05,
                 stable_gyro_threshold=0.15, confidence_threshold=0.8):
        self.force_threshold = force_threshold
        self.time_threshold = time_threshold
        self.stable_gyro_threshold = stable_gyro_threshold
        self.confidence_threshold = confidence_threshold

        self.state = self.NO_CONTACT
        self.confidence = 0.0
        self.contact_start_time = None
        self.should_close = False
        self.closing_angle = 75.0

    def detect(self, F_est, tau_est, gyro, t):
        F_mag = np.linalg.norm(F_est)
        gyro_mag = np.linalg.norm(gyro)

        prev_state = self.state

        if self.state == self.NO_CONTACT:
            if F_mag > self.force_threshold:
                self.contact_start_time = t
                self.state = self.POSSIBLE_CONTACT
                self.confidence = 0.3

        elif self.state == self.POSSIBLE_CONTACT:
            duration = t - self.contact_start_time
            if F_mag < self.force_threshold * 0.5:
                self.state = self.NO_CONTACT
                self.confidence = 0.0
            elif duration > self.time_threshold:
                self.state = self.CONFIRMED_CONTACT
                self.confidence = 0.7

        elif self.state == self.CONFIRMED_CONTACT:
            if gyro_mag < self.stable_gyro_threshold and F_mag > self.force_threshold:
                self.state = self.STABLE_PERCHING
                self.confidence = 0.9
            elif F_mag < self.force_threshold * 0.3:
                self.state = self.NO_CONTACT
                self.confidence = 0.0

        elif self.state == self.STABLE_PERCHING:
            if gyro_mag > self.stable_gyro_threshold * 2:
                self.state = self.SLIPPING
                self.confidence = 0.4
            elif self.confidence > self.confidence_threshold:
                self.should_close = True

        elif self.state == self.SLIPPING:
            if gyro_mag < self.stable_gyro_threshold and F_mag > self.force_threshold:
                self.state = self.STABLE_PERCHING
            elif F_mag < 0.1:
                self.state = self.NO_CONTACT

        return self.state, self.should_close


def load_ulog_data(log_path):
    """从 ulog 读取 GMO 所需数据"""
    ulog = ULog(log_path)

    # 读取 IMU 数据 (sensor_accel, sensor_gyro)
    accel_data = ulog.get_dataset('sensor_accel')
    gyro_data = ulog.get_dataset('sensor_gyro')

    # 读取机体角速度和角加速度
    angvel_data = ulog.get_dataset('vehicle_angular_velocity')

    # 读取姿态
    att_data = ulog.get_dataset('vehicle_attitude')

    # 读取机体加速度 (可选，作为 sanity check)
    try:
        veh_accel = ulog.get_dataset('vehicle_acceleration')
    except:
        veh_accel = None

    return {
        'accel': accel_data,
        'gyro': gyro_data,
        'angvel': angvel_data,
        'attitude': att_data,
        'veh_accel': veh_accel,
    }


def interpolate_to_common_timestamps(data_dict, target_hz=250):
    """将所有数据插值到统一时间轴"""
    # 提取时间戳 (微秒 -> 秒)
    ts_accel = data_dict['accel'].data['timestamp'] / 1e6
    ts_gyro = data_dict['gyro'].data['timestamp'] / 1e6
    ts_angvel = data_dict['angvel'].data['timestamp'] / 1e6
    ts_att = data_dict['attitude'].data['timestamp'] / 1e6

    t_min = max(ts_accel.min(), ts_gyro.min(), ts_angvel.min(), ts_att.min())
    t_max = min(ts_accel.max(), ts_gyro.max(), ts_angvel.max(), ts_att.max())

    # 统一时间轴
    dt = 1.0 / target_hz
    t_common = np.arange(t_min, t_max, dt)

    def interp(ts, vals, t_out):
        return np.interp(t_out, ts, vals)

    # 加速度 (body frame, m/s^2, NED)
    ax = interp(ts_accel, data_dict['accel'].data['x'], t_common)
    ay = interp(ts_accel, data_dict['accel'].data['y'], t_common)
    az = interp(ts_accel, data_dict['accel'].data['z'], t_common)
    accel = np.column_stack([ax, ay, az])

    # 角速度 (body frame, rad/s, NED)
    gx = interp(ts_angvel, data_dict['angvel'].data['xyz[0]'], t_common)
    gy = interp(ts_angvel, data_dict['angvel'].data['xyz[1]'], t_common)
    gz = interp(ts_angvel, data_dict['angvel'].data['xyz[2]'], t_common)
    gyro = np.column_stack([gx, gy, gz])

    # 角加速度导数 (body frame, rad/s^2)
    gdx = interp(ts_angvel, data_dict['angvel'].data['xyz_derivative[0]'], t_common)
    gdy = interp(ts_angvel, data_dict['angvel'].data['xyz_derivative[1]'], t_common)
    gdz = interp(ts_angvel, data_dict['angvel'].data['xyz_derivative[2]'], t_common)
    gyro_dot = np.column_stack([gdx, gdy, gdz])

    # 姿态四元数
    q0 = interp(ts_att, data_dict['attitude'].data['q[0]'], t_common)
    q1 = interp(ts_att, data_dict['attitude'].data['q[1]'], t_common)
    q2 = interp(ts_att, data_dict['attitude'].data['q[2]'], t_common)
    q3 = interp(ts_att, data_dict['attitude'].data['q[3]'], t_common)
    quat = np.column_stack([q0, q1, q2, q3])

    return t_common, accel, gyro, gyro_dot, quat


def quat_to_rotation_matrix(q):
    """四元数 (w, x, y, z) -> 旋转矩阵 (world -> body), 纯 numpy"""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y + w*z),     2*(x*z - w*y)],
        [2*(x*y - w*z),       1 - 2*(x*x + z*z), 2*(y*z + w*x)],
        [2*(x*z + w*y),       2*(y*z - w*x),     1 - 2*(x*x + y*y)]
    ])

def compute_rotation_matrix(quat):
    """从四元数数组计算旋转矩阵 (world -> body)"""
    R = np.zeros((len(quat), 3, 3))
    for i in range(len(quat)):
        R[i] = quat_to_rotation_matrix(quat[i])  # q = [w, x, y, z]
    return R


def estimate_thrust_cmd(t, accel, mass):
    """简化推力估计: 悬停推力 + 垂直加速度补偿"""
    g = 9.80665
    # 简化模型: 假设推力主要用于抵消重力和提供垂直加速度
    # thrust_z = m * (g + az)  # NED: 向上加速需要负的 az
    thrust = np.zeros((len(t), 3))
    thrust[:, 2] = -mass * g  # 悬停推力 (NED: 向上为负)
    return thrust


def inject_contact_event(accel, t, contact_time, contact_duration, contact_force_body):
    """在指定时间段注入模拟接触力"""
    accel_injected = accel.copy()
    dt = t[1] - t[0] if len(t) > 1 else 0.004
    mass = 1.173

    for i in range(len(t)):
        if contact_time <= t[i] <= contact_time + contact_duration:
            # F = m*a, 所以接触力表现为加速度变化: a_contact = F_contact / m
            accel_injected[i] += contact_force_body / mass

    return accel_injected


def run_gmo_test(log_path, output_dir, contact_time=None, contact_duration=0.5,
                 contact_force=(1.0, 0.0, 0.0), observer_gain=50.0):
    """运行 GMO 离线测试"""
    print(f"读取日志: {log_path}")
    data = load_ulog_data(log_path)

    print("插值数据到统一时间轴...")
    t, accel, gyro, gyro_dot, quat = interpolate_to_common_timestamps(data, target_hz=250)
    print(f"  数据点: {len(t)}, 时长: {t[-1]-t[0]:.1f}s")

    # 计算旋转矩阵
    print("计算旋转矩阵...")
    R_bw = compute_rotation_matrix(quat)

    # 推力指令估计 (简化)
    mass = 1.173
    thrust_cmd = estimate_thrust_cmd(t, accel, mass)
    torque_cmd = np.zeros((len(t), 3))

    # 注入接触事件 (如果指定)
    if contact_time is not None:
        print(f"注入接触事件: t={contact_time}s, duration={contact_duration}s, force={contact_force}")
        accel = inject_contact_event(accel, t, contact_time, contact_duration,
                                     np.array(contact_force))

    # 运行 GMO
    print("运行 GMO 观测器...")
    dt = t[1] - t[0]
    gmo = GMOEstimator(mass=mass, observer_gain=observer_gain, dt=dt)
    detector = ContactDetector(force_threshold=0.3, time_threshold=0.05,
                               stable_gyro_threshold=0.2)

    N = len(t)
    F_est = np.zeros((N, 3))
    tau_est = np.zeros((N, 3))
    state_history = np.zeros(N)
    should_close_history = np.zeros(N, dtype=bool)

    for i in range(N):
        F, tau = gmo.update(accel[i], gyro[i], gyro_dot[i],
                           thrust_cmd[i], torque_cmd[i], R_bw[i])
        F_est[i] = F
        tau_est[i] = tau

        state, should_close = detector.detect(F, tau, gyro[i], t[i])
        state_history[i] = state
        should_close_history[i] = should_close

    # 绘制结果
    print("绘制结果...")
    plot_results(t, accel, gyro, F_est, tau_est, state_history, should_close_history,
                 output_dir, contact_time, contact_duration)

    # 统计
    F_mag = np.linalg.norm(F_est, axis=1)
    print(f"\n=== GMO 统计结果 ===")
    print(f"  最大估计力: {np.max(F_mag):.3f} N")
    print(f"  平均估计力: {np.mean(F_mag):.3f} N")
    print(f"  力标准差:   {np.std(F_mag):.3f} N")
    if contact_time is not None:
        contact_idx = (t >= contact_time) & (t <= contact_time + contact_duration)
        if np.any(contact_idx):
            print(f"  接触期间平均力: {np.mean(F_mag[contact_idx]):.3f} N")
    print(f"  状态变化次数: {np.sum(np.diff(state_history) != 0)}")
    print(f"  应闭合指令次数: {np.sum(should_close_history)}")
    print(f"  输出图片: {output_dir}/gmo_test.png")

    return F_est, tau_est, state_history


def plot_results(t, accel, gyro, F_est, tau_est, state_history, should_close_history,
                 output_dir, contact_time, contact_duration):
    """绘制 GMO 结果"""
    F_mag = np.linalg.norm(F_est, axis=1)
    tau_mag = np.linalg.norm(tau_est, axis=1)
    gyro_mag = np.linalg.norm(gyro, axis=1)

    fig, axes = plt.subplots(5, 1, figsize=(14, 14))

    # 1. 加速度
    axes[0].plot(t, accel[:, 0], label='ax', alpha=0.7, linewidth=0.8)
    axes[0].plot(t, accel[:, 1], label='ay', alpha=0.7, linewidth=0.8)
    axes[0].plot(t, accel[:, 2], label='az', alpha=0.7, linewidth=0.8)
    if contact_time:
        axes[0].axvspan(contact_time, contact_time + contact_duration, alpha=0.2, color='red', label='接触注入')
    axes[0].set_ylabel('IMU 加速度 (m/s²)')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)

    # 2. 角速度
    axes[1].plot(t, gyro[:, 0], label='p', alpha=0.7, linewidth=0.8)
    axes[1].plot(t, gyro[:, 1], label='q', alpha=0.7, linewidth=0.8)
    axes[1].plot(t, gyro[:, 2], label='r', alpha=0.7, linewidth=0.8)
    if contact_time:
        axes[1].axvspan(contact_time, contact_time + contact_duration, alpha=0.2, color='red')
    axes[1].set_ylabel('角速度 (rad/s)')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)

    # 3. 估计外力
    axes[2].plot(t, F_est[:, 0], label='Fx', alpha=0.7, linewidth=0.8)
    axes[2].plot(t, F_est[:, 1], label='Fy', alpha=0.7, linewidth=0.8)
    axes[2].plot(t, F_est[:, 2], label='Fz', alpha=0.7, linewidth=0.8)
    axes[2].plot(t, F_mag, 'k-', label='|F|', linewidth=1.5)
    axes[2].axhline(y=0.3, color='r', linestyle='--', alpha=0.7, label='检测阈值')
    if contact_time:
        axes[2].axvspan(contact_time, contact_time + contact_duration, alpha=0.2, color='red')
    axes[2].set_ylabel('估计外力 (N)')
    axes[2].legend(loc='upper right')
    axes[2].grid(True, alpha=0.3)

    # 4. 估计力矩
    axes[3].plot(t, tau_est[:, 0], label='τx', alpha=0.7, linewidth=0.8)
    axes[3].plot(t, tau_est[:, 1], label='τy', alpha=0.7, linewidth=0.8)
    axes[3].plot(t, tau_est[:, 2], label='τz', alpha=0.7, linewidth=0.8)
    axes[3].plot(t, tau_mag, 'k-', label='|τ|', linewidth=1.5)
    if contact_time:
        axes[3].axvspan(contact_time, contact_time + contact_duration, alpha=0.2, color='red')
    axes[3].set_ylabel('估计力矩 (Nm)')
    axes[3].legend(loc='upper right')
    axes[3].grid(True, alpha=0.3)

    # 5. 接触状态
    axes[4].fill_between(t, state_history, alpha=0.3, color='green')
    axes[4].plot(t, state_history, 'g-', linewidth=1.5)
    axes[4].scatter(t[should_close_history], state_history[should_close_history],
                    c='red', s=50, marker='x', label='应闭合关节', zorder=5)
    if contact_time:
        axes[4].axvspan(contact_time, contact_time + contact_duration, alpha=0.2, color='red')
    axes[4].set_ylabel('接触状态')
    axes[4].set_xlabel('时间 (s)')
    axes[4].set_yticks([0, 1, 2, 3, 4])
    axes[4].set_yticklabels(ContactDetector.STATE_NAMES)
    axes[4].legend(loc='upper right')
    axes[4].grid(True, alpha=0.3)

    plt.tight_layout()
    os.makedirs(output_dir, exist_ok=True)
    plt.savefig(f'{output_dir}/gmo_test.png', dpi=150, bbox_inches='tight')
    plt.close()


def main():
    parser = argparse.ArgumentParser(description='GMO 离线验证')
    parser.add_argument('log', help='PX4 ulog 文件路径')
    parser.add_argument('--output', '-o', default='/tmp/gmo_test', help='输出目录')
    parser.add_argument('--contact-time', type=float, help='模拟接触开始时间 (s)')
    parser.add_argument('--contact-duration', type=float, default=0.5, help='接触持续时间 (s)')
    parser.add_argument('--contact-force', type=float, nargs=3, default=[1.0, 0.0, 0.0],
                        help='接触力 body frame (Fx Fy Fz) N')
    parser.add_argument('--gain', '-k', type=float, default=50.0, help='观测器增益 K_O')
    args = parser.parse_args()

    run_gmo_test(args.log, args.output,
                 contact_time=args.contact_time,
                 contact_duration=args.contact_duration,
                 contact_force=args.contact_force,
                 observer_gain=args.gain)


if __name__ == '__main__':
    main()
