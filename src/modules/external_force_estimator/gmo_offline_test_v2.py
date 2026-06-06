#!/usr/bin/env python3
"""
广义动量观测器 (GMO) 离线验证 v2 - 动量法 + 基线补偿
使用 vehicle_local_position 速度，避免 sensor_accel 重力补偿问题
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pyulog import ULog
import argparse
import os


def quat_to_rotation_matrix(q):
    """四元数 (w, x, y, z) -> 旋转矩阵 (world -> body)"""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y + w*z),     2*(x*z - w*y)],
        [2*(x*y - w*z),       1 - 2*(x*x + z*z), 2*(y*z + w*x)],
        [2*(x*z + w*y),       2*(y*z - w*x),     1 - 2*(x*x + y*y)]
    ])


def compute_rotation_matrix(quat):
    R = np.zeros((len(quat), 3, 3))
    for i in range(len(quat)):
        R[i] = quat_to_rotation_matrix(quat[i])
    return R


class MomentumGMO:
    """
    广义动量观测器 - 动量法实现
    
    核心方程:
        p = m * v  (线动量, world frame)
        dp_hat/dt = F_thrust_estimate + m*g + K*(p - p_hat)
        F_ext_est = K*(p - p_hat)
    
    优点:
        - 不直接依赖加速度（避免噪声和重力补偿）
        - 速度来自 EKF，更平滑
        - 积分作用自动补偿推力估计偏差
    """

    def __init__(self, mass=1.173, observer_gain=30.0, dt=0.004):
        self.m = mass
        self.K = observer_gain
        self.dt = dt
        
        # 状态
        self.p_hat = np.zeros(3)      # 估计线动量
        self.h_hat = np.zeros(3)      # 估计角动量
        self.I = np.diag([0.015, 0.015, 0.025])
        
        self.initialized = False
        
        # 历史（用于基线补偿）
        self.force_history = []
        self.torque_history = []
        self.history_len = 100        # 约 0.4s 的滑动窗口

    def update(self, v_world, gyro_body, thrust_estimate, torque_estimate, R_bw):
        """
        参数:
            v_world:      world frame 速度 [vx, vy, vz] (m/s, NED)
            gyro_body:    body frame 角速度 [p, q, r] (rad/s)
            thrust_estimate: body frame 推力估计 [Fx, Fy, Fz] (N)
            torque_estimate: body frame 力矩估计 [tx, ty, tz] (Nm)
            R_bw:         world -> body 旋转矩阵
        """
        # 线动量
        p = self.m * v_world
        
        # 角动量 (body frame)
        h = self.I @ gyro_body
        
        # 重力 (world frame: NED, g = [0, 0, 9.81])
        g_world = np.array([0.0, 0.0, 9.80665])
        
        # 推力转到 world frame
        thrust_world = R_bw.T @ thrust_estimate
        
        if not self.initialized:
            self.p_hat = p.copy()
            self.h_hat = h.copy()
            self.initialized = True
            return np.zeros(3), np.zeros(3)
        
        # 线动量观测器
        # dp_hat/dt = F_thrust + m*g + K*(p - p_hat)
        dp_hat = thrust_world + self.m * g_world + self.K * (p - self.p_hat)
        self.p_hat += dp_hat * self.dt
        F_ext_world = self.K * (p - self.p_hat)
        
        # 角动量观测器
        # dh_hat/dt = τ + K*(h - h_hat)
        dh_hat = torque_estimate + self.K * (h - self.h_hat)
        self.h_hat += dh_hat * self.dt
        tau_ext_body = self.K * (h - self.h_hat)
        
        # 外力转到 body frame
        F_ext_body = R_bw @ F_ext_world
        
        # 保存历史
        self.force_history.append(F_ext_body.copy())
        self.torque_history.append(tau_ext_body.copy())
        if len(self.force_history) > self.history_len:
            self.force_history.pop(0)
            self.torque_history.pop(0)
        
        return F_ext_body, tau_ext_body

    def get_baseline_compensated_force(self):
        """返回减去滑动平均后的力（消除推力估计偏差）"""
        if len(self.force_history) < 10:
            return np.zeros(3)
        F_mean = np.mean(self.force_history, axis=0)
        return self.force_history[-1] - F_mean


class ContactDetector:
    """接触检测状态机 v2 - 基于动量法输出的跳变检测"""

    NO_CONTACT = 0
    POSSIBLE_CONTACT = 1
    CONFIRMED_CONTACT = 2
    STABLE_PERCHING = 3
    SLIPPING = 4
    STATE_NAMES = ['NO_CONTACT', 'POSSIBLE', 'CONFIRMED', 'STABLE', 'SLIPPING']

    def __init__(self, force_threshold=0.8, time_threshold=0.05,
                 stable_gyro_threshold=0.15, confidence_threshold=0.85):
        self.force_threshold = force_threshold
        self.time_threshold = time_threshold
        self.stable_gyro_threshold = stable_gyro_threshold
        self.confidence_threshold = confidence_threshold

        self.state = self.NO_CONTACT
        self.confidence = 0.0
        self.contact_start_time = None
        self.should_close = False

    def detect(self, F_est, tau_est, gyro, t):
        F_mag = np.linalg.norm(F_est)
        gyro_mag = np.linalg.norm(gyro)

        if self.state == self.NO_CONTACT:
            if F_mag > self.force_threshold:
                self.contact_start_time = t
                self.state = self.POSSIBLE_CONTACT
                self.confidence = 0.3

        elif self.state == self.POSSIBLE_CONTACT:
            duration = t - self.contact_start_time
            if F_mag < self.force_threshold * 0.4:
                self.state = self.NO_CONTACT
                self.confidence = 0.0
                self.should_close = False
            elif duration > self.time_threshold:
                self.state = self.CONFIRMED_CONTACT
                self.confidence = 0.7

        elif self.state == self.CONFIRMED_CONTACT:
            if gyro_mag < self.stable_gyro_threshold and F_mag > self.force_threshold * 0.6:
                self.state = self.STABLE_PERCHING
                self.confidence = 0.9
            elif F_mag < self.force_threshold * 0.3:
                self.state = self.NO_CONTACT
                self.confidence = 0.0
                self.should_close = False

        elif self.state == self.STABLE_PERCHING:
            if gyro_mag > self.stable_gyro_threshold * 2.5:
                self.state = self.SLIPPING
                self.confidence = 0.4
            elif self.confidence >= self.confidence_threshold:
                self.should_close = True

        elif self.state == self.SLIPPING:
            if gyro_mag < self.stable_gyro_threshold and F_mag > self.force_threshold:
                self.state = self.STABLE_PERCHING
            elif F_mag < 0.2:
                self.state = self.NO_CONTACT
                self.confidence = 0.0
                self.should_close = False

        return self.state, self.should_close


def load_and_interpolate(log_path, target_hz=250):
    """读取 ulog 并插值到统一时间轴"""
    ulog = ULog(log_path)
    
    lp = ulog.get_dataset('vehicle_local_position')
    av = ulog.get_dataset('vehicle_angular_velocity')
    att = ulog.get_dataset('vehicle_attitude')
    
    ts_lp = lp.data['timestamp'] / 1e6
    ts_av = av.data['timestamp'] / 1e6
    ts_att = att.data['timestamp'] / 1e6
    
    t_min = max(ts_lp.min(), ts_av.min(), ts_att.min())
    t_max = min(ts_lp.max(), ts_av.max(), ts_att.max())
    dt = 1.0 / target_hz
    t = np.arange(t_min, t_max, dt)
    
    def interp(ts, vals):
        return np.interp(t, ts, vals)
    
    # World frame 速度
    v = np.column_stack([
        interp(ts_lp, lp.data['vx']),
        interp(ts_lp, lp.data['vy']),
        interp(ts_lp, lp.data['vz'])
    ])
    
    # Body frame 角速度
    gyro = np.column_stack([
        interp(ts_av, av.data['xyz[0]']),
        interp(ts_av, av.data['xyz[1]']),
        interp(ts_av, av.data['xyz[2]'])
    ])
    
    # 姿态
    quat = np.column_stack([
        interp(ts_att, att.data['q[0]']),
        interp(ts_att, att.data['q[1]']),
        interp(ts_att, att.data['q[2]']),
        interp(ts_att, att.data['q[3]'])
    ])
    
    return t, v, gyro, quat


def inject_contact_force(v, t, contact_time, contact_duration, force_world):
    """在速度上注入接触力效应（通过加速度脉冲积分）"""
    v_injected = v.copy()
    mass = 1.173
    dt = t[1] - t[0]
    
    for i in range(len(t)):
        if contact_time <= t[i] <= contact_time + contact_duration:
            # 接触力导致加速度: a = F/m
            # 速度变化: dv = a * dt
            v_injected[i:] += (force_world / mass) * dt
    
    return v_injected


def run_test(log_path, output_dir, contact_time=None, contact_duration=0.3,
             contact_force=(3.0, 0.0, 0.0), K=30.0):
    print(f"读取日志: {log_path}")
    t, v, gyro, quat = load_and_interpolate(log_path, target_hz=250)
    print(f"  数据点: {len(t)}, 时长: {t[-1]-t[0]:.1f}s")
    
    # 旋转矩阵
    R = compute_rotation_matrix(quat)
    
    # 注入接触（可选）
    if contact_time is not None:
        print(f"注入接触: t={contact_time}s, dur={contact_duration}s, F_world={contact_force}")
        v = inject_contact_force(v, t, contact_time, contact_duration, np.array(contact_force))
    
    # 推力估计（简化: 悬停推力，body frame）
    mass = 1.173
    thrust_estimate = np.array([0.0, 0.0, -mass * 9.80665])
    torque_estimate = np.zeros(3)
    
    # 运行 GMO
    dt = t[1] - t[0]
    gmo = MomentumGMO(mass=mass, observer_gain=K, dt=dt)
    detector = ContactDetector(force_threshold=0.8, time_threshold=0.05,
                               stable_gyro_threshold=0.15)
    
    N = len(t)
    F_est = np.zeros((N, 3))
    F_comp = np.zeros((N, 3))  # 基线补偿后的力
    tau_est = np.zeros((N, 3))
    state_hist = np.zeros(N)
    close_hist = np.zeros(N, dtype=bool)
    
    for i in range(N):
        F, tau = gmo.update(v[i], gyro[i], thrust_estimate, torque_estimate, R[i])
        F_est[i] = F
        tau_est[i] = tau
        F_comp[i] = gmo.get_baseline_compensated_force()
        
        state, close = detector.detect(F_comp[i], tau, gyro[i], t[i])
        state_hist[i] = state
        close_hist[i] = close
    
    # 跳过前 2s（初始化）
    skip = int(2.0 / dt)
    
    # 统计（跳过初始化）
    F_mag = np.linalg.norm(F_est[skip:], axis=1)
    Fc_mag = np.linalg.norm(F_comp[skip:], axis=1)
    
    print(f"\n=== GMO 统计 (跳过前 2s) ===")
    print(f"  原始力 最大/均值/标准差: {np.max(F_mag):.2f} / {np.mean(F_mag):.2f} / {np.std(F_mag):.2f} N")
    print(f"  补偿力 最大/均值/标准差: {np.max(Fc_mag):.2f} / {np.mean(Fc_mag):.2f} / {np.std(Fc_mag):.2f} N")
    if contact_time is not None:
        cidx = (t[skip:] >= contact_time) & (t[skip:] <= contact_time + contact_duration)
        if np.any(cidx):
            print(f"  接触期间补偿力均值: {np.mean(Fc_mag[cidx]):.2f} N")
    print(f"  状态变化: {np.sum(np.diff(state_hist[skip:]) != 0)}")
    print(f"  闭合指令: {np.sum(close_hist[skip:])}")
    
    # 绘图
    plot(t, accel=None, gyro=gyro, F_raw=F_est, F_comp=F_comp, tau=tau_est,
         state=state_hist, close=close_hist, output_dir=output_dir,
         contact_time=contact_time, contact_duration=contact_duration, skip=skip)
    
    return F_comp, state_hist


def plot(t, accel, gyro, F_raw, F_comp, tau, state, close, output_dir,
         contact_time, contact_duration, skip):
    F_raw_mag = np.linalg.norm(F_raw, axis=1)
    F_comp_mag = np.linalg.norm(F_comp, axis=1)
    tau_mag = np.linalg.norm(tau, axis=1)
    gyro_mag = np.linalg.norm(gyro, axis=1)
    
    fig, axes = plt.subplots(5, 1, figsize=(14, 14))
    
    # 1. 角速度
    axes[0].plot(t[skip:], gyro[skip:, 0], label='p', alpha=0.7, lw=0.8)
    axes[0].plot(t[skip:], gyro[skip:, 1], label='q', alpha=0.7, lw=0.8)
    axes[0].plot(t[skip:], gyro[skip:, 2], label='r', alpha=0.7, lw=0.8)
    axes[0].plot(t[skip:], gyro_mag[skip:], 'k--', label='|omega|', lw=1)
    if contact_time:
        axes[0].axvspan(contact_time, contact_time+contact_duration, alpha=0.15, color='red')
    axes[0].set_ylabel('Gyro (rad/s)')
    axes[0].legend(loc='upper right', fontsize=8)
    axes[0].grid(True, alpha=0.3)
    
    # 2. 原始估计力
    axes[1].plot(t[skip:], F_raw[skip:, 0], label='Fx', alpha=0.7, lw=0.8)
    axes[1].plot(t[skip:], F_raw[skip:, 1], label='Fy', alpha=0.7, lw=0.8)
    axes[1].plot(t[skip:], F_raw[skip:, 2], label='Fz', alpha=0.7, lw=0.8)
    axes[1].plot(t[skip:], F_raw_mag[skip:], 'k-', label='|F_raw|', lw=1.5)
    if contact_time:
        axes[1].axvspan(contact_time, contact_time+contact_duration, alpha=0.15, color='red')
    axes[1].set_ylabel('Force raw (N)')
    axes[1].legend(loc='upper right', fontsize=8)
    axes[1].grid(True, alpha=0.3)
    
    # 3. 基线补偿后的力
    axes[2].plot(t[skip:], F_comp[skip:, 0], label='Fx', alpha=0.7, lw=0.8)
    axes[2].plot(t[skip:], F_comp[skip:, 1], label='Fy', alpha=0.7, lw=0.8)
    axes[2].plot(t[skip:], F_comp[skip:, 2], label='Fz', alpha=0.7, lw=0.8)
    axes[2].plot(t[skip:], F_comp_mag[skip:], 'k-', label='|F_comp|', lw=1.5)
    axes[2].axhline(y=1.0, color='r', linestyle='--', alpha=0.5, label='threshold')
    if contact_time:
        axes[2].axvspan(contact_time, contact_time+contact_duration, alpha=0.15, color='red', label='contact')
    axes[2].set_ylabel('Force compensated (N)')
    axes[2].legend(loc='upper right', fontsize=8)
    axes[2].grid(True, alpha=0.3)
    
    # 4. 力矩
    axes[3].plot(t[skip:], tau[skip:, 0], label='tx', alpha=0.7, lw=0.8)
    axes[3].plot(t[skip:], tau[skip:, 1], label='ty', alpha=0.7, lw=0.8)
    axes[3].plot(t[skip:], tau[skip:, 2], label='tz', alpha=0.7, lw=0.8)
    axes[3].plot(t[skip:], tau_mag[skip:], 'k-', label='|tau|', lw=1.5)
    if contact_time:
        axes[3].axvspan(contact_time, contact_time+contact_duration, alpha=0.15, color='red')
    axes[3].set_ylabel('Torque (Nm)')
    axes[3].legend(loc='upper right', fontsize=8)
    axes[3].grid(True, alpha=0.3)
    
    # 5. 接触状态
    axes[4].fill_between(t[skip:], state[skip:], alpha=0.3, color='green')
    axes[4].plot(t[skip:], state[skip:], 'g-', lw=1.5)
    axes[4].scatter(t[skip:][close[skip:]], state[skip:][close[skip:]],
                    c='red', s=50, marker='x', label='CLOSE_CMD', zorder=5)
    if contact_time:
        axes[4].axvspan(contact_time, contact_time+contact_duration, alpha=0.15, color='red')
    axes[4].set_ylabel('Contact State')
    axes[4].set_xlabel('Time (s)')
    axes[4].set_yticks([0, 1, 2, 3, 4])
    axes[4].set_yticklabels(ContactDetector.STATE_NAMES)
    axes[4].legend(loc='upper right', fontsize=8)
    axes[4].grid(True, alpha=0.3)
    
    plt.tight_layout()
    os.makedirs(output_dir, exist_ok=True)
    plt.savefig(f'{output_dir}/gmo_test.png', dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_dir}/gmo_test.png")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('log', help='PX4 ulog file')
    parser.add_argument('-o', '--output', default='/tmp/gmo_test')
    parser.add_argument('--contact-time', type=float, help='Inject contact at t (s)')
    parser.add_argument('--contact-duration', type=float, default=0.3)
    parser.add_argument('--contact-force', type=float, nargs=3, default=[3.0, 0.0, 0.0])
    parser.add_argument('-K', '--gain', type=float, default=30.0)
    args = parser.parse_args()
    
    run_test(args.log, args.output,
             contact_time=args.contact_time,
             contact_duration=args.contact_duration,
             contact_force=args.contact_force,
             K=args.gain)


if __name__ == '__main__':
    main()
