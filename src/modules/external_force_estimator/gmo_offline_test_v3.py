#!/usr/bin/env python3
"""
GMO 离线验证 v3 - 自适应基线法
使用 EKF 加速度 (vehicle_local_position.ax/ay/az) + 滑动基线补偿
避免速度数值微分的噪声放大问题
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pyulog import ULog
import argparse
import os


def quat_to_rotation_matrix(q):
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


class AdaptiveBaselineGMO:
    """
    自适应基线 GMO
    
    核心思想:
        - 使用 EKF 估计的机体加速度 (world frame)
        - 滑动窗口估计"无接触基线"加速度
        - 残差 = 当前加速度 - 基线加速度
        - 外力 = m * 残差 (world frame)
        - 低通滤波平滑
    
    推力估计隐含在基线中: 
        baseline_a = (F_thrust + m*g) / m  ( averaged over window )
    """

    def __init__(self, mass=1.173, dt=0.004, window_s=0.5, lpf_alpha=0.9):
        self.m = mass
        self.dt = dt
        self.window_size = int(window_s / dt)  # 滑动窗口样本数
        self.lpf_alpha = lpf_alpha
        
        self.a_history = []          # 加速度历史 (world frame)
        self.F_est = np.zeros(3)     # 估计外力 (world frame)
        self.F_est_lpf = np.zeros(3) # 低通滤波后的外力
        self.initialized = False

    def update(self, a_world):
        """
        a_world: EKF 估计的机体加速度 [ax, ay, az] (m/s^2, world/NED frame)
        """
        self.a_history.append(a_world.copy())
        if len(self.a_history) > self.window_size:
            self.a_history.pop(0)
        
        if len(self.a_history) < 10:
            return np.zeros(3)
        
        # 基线 = 滑动中位数 (对异常值更鲁棒)
        baseline = np.median(self.a_history, axis=0)
        
        # 残差 = 当前 - 基线
        residual = a_world - baseline
        
        # 外力 = m * 残差
        self.F_est = self.m * residual
        
        # 低通滤波
        self.F_est_lpf = self.lpf_alpha * self.F_est_lpf + (1 - self.lpf_alpha) * self.F_est
        
        return self.F_est_lpf.copy()


class ContactDetector:
    NO_CONTACT = 0
    POSSIBLE_CONTACT = 1
    CONFIRMED_CONTACT = 2
    STABLE_PERCHING = 3
    SLIPPING = 4
    STATE_NAMES = ['NO_CONTACT', 'POSSIBLE', 'CONFIRMED', 'STABLE', 'SLIPPING']

    def __init__(self, force_threshold=1.5, time_threshold=0.05,
                 stable_gyro_threshold=0.15, confidence_threshold=0.85):
        self.force_threshold = force_threshold
        self.time_threshold = time_threshold
        self.stable_gyro_threshold = stable_gyro_threshold
        self.confidence_threshold = confidence_threshold
        self.state = self.NO_CONTACT
        self.confidence = 0.0
        self.contact_start_time = None
        self.should_close = False

    def detect(self, F_est, gyro, t):
        F_mag = np.linalg.norm(F_est)
        gyro_mag = np.linalg.norm(gyro)

        if self.state == self.NO_CONTACT:
            if F_mag > self.force_threshold:
                self.contact_start_time = t
                self.state = self.POSSIBLE_CONTACT
                self.confidence = 0.3
                self.should_close = False

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
                self.should_close = False
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
    
    # EKF 加速度 (world frame)
    a_world = np.column_stack([
        interp(ts_lp, lp.data['ax']),
        interp(ts_lp, lp.data['ay']),
        interp(ts_lp, lp.data['az'])
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
    
    return t, a_world, gyro, quat


def inject_contact_acceleration(a, t, contact_time, contact_duration, force_world):
    """在加速度上注入接触力效应"""
    a_inj = a.copy()
    mass = 1.173
    for i in range(len(t)):
        if contact_time <= t[i] <= contact_time + contact_duration:
            a_inj[i] += force_world / mass
    return a_inj


def run_test(log_path, output_dir, contact_time=None, contact_duration=0.3,
             contact_force=(3.0, 0.0, 0.0), window_s=0.5, lpf_alpha=0.9,
             force_threshold=1.5):
    print(f"Reading log: {log_path}")
    t, a_world, gyro, quat = load_and_interpolate(log_path, target_hz=250)
    print(f"  Samples: {len(t)}, Duration: {t[-1]-t[0]:.1f}s")
    
    R = compute_rotation_matrix(quat)
    
    if contact_time is not None:
        print(f"Inject contact: t={contact_time}s, dur={contact_duration}s, F_world={contact_force}")
        a_world = inject_contact_acceleration(a_world, t, contact_time, contact_duration, np.array(contact_force))
    
    dt = t[1] - t[0]
    gmo = AdaptiveBaselineGMO(mass=1.173, dt=dt, window_s=window_s, lpf_alpha=lpf_alpha)
    detector = ContactDetector(force_threshold=force_threshold)
    
    N = len(t)
    F_world = np.zeros((N, 3))
    F_body = np.zeros((N, 3))
    state_hist = np.zeros(N)
    close_hist = np.zeros(N, dtype=bool)
    
    for i in range(N):
        F_w = gmo.update(a_world[i])
        F_world[i] = F_w
        F_body[i] = R[i] @ F_w
        
        state, close = detector.detect(F_w, gyro[i], t[i])
        state_hist[i] = state
        close_hist[i] = close
    
    skip = int(2.0 / dt)
    
    Fw_mag = np.linalg.norm(F_world[skip:], axis=1)
    Fb_mag = np.linalg.norm(F_body[skip:], axis=1)
    
    print(f"\n=== GMO Stats (skip first 2s) ===")
    print(f"  World force max/mean/std: {np.max(Fw_mag):.2f} / {np.mean(Fw_mag):.2f} / {np.std(Fw_mag):.2f} N")
    print(f"  Body  force max/mean/std: {np.max(Fb_mag):.2f} / {np.mean(Fb_mag):.2f} / {np.std(Fb_mag):.2f} N")
    if contact_time is not None:
        cidx = (t[skip:] >= contact_time) & (t[skip:] <= contact_time + contact_duration)
        if np.any(cidx):
            print(f"  Contact period world force mean: {np.mean(Fw_mag[cidx]):.2f} N")
    print(f"  State changes: {np.sum(np.diff(state_hist[skip:]) != 0)}")
    print(f"  Close commands: {np.sum(close_hist[skip:])}")
    
    plot(t, gyro, F_world, F_body, state_hist, close_hist, output_dir,
         contact_time, contact_duration, skip)
    return F_world, state_hist


def plot(t, gyro, F_world, F_body, state, close, output_dir,
         contact_time, contact_duration, skip):
    Fw_mag = np.linalg.norm(F_world, axis=1)
    Fb_mag = np.linalg.norm(F_body, axis=1)
    gyro_mag = np.linalg.norm(gyro, axis=1)
    
    fig, axes = plt.subplots(5, 1, figsize=(14, 14))
    
    # 1. World frame 加速度
    axes[0].plot(t[skip:], F_world[skip:, 0], label='Fx_world', alpha=0.7, lw=0.8)
    axes[0].plot(t[skip:], F_world[skip:, 1], label='Fy_world', alpha=0.7, lw=0.8)
    axes[0].plot(t[skip:], F_world[skip:, 2], label='Fz_world', alpha=0.7, lw=0.8)
    axes[0].plot(t[skip:], Fw_mag[skip:], 'k-', label='|F_world|', lw=1.5)
    if contact_time:
        axes[0].axvspan(contact_time, contact_time+contact_duration, alpha=0.15, color='red')
    axes[0].set_ylabel('Force world (N)')
    axes[0].legend(loc='upper right', fontsize=8)
    axes[0].grid(True, alpha=0.3)
    
    # 2. Body frame 力
    axes[1].plot(t[skip:], F_body[skip:, 0], label='Fx_body', alpha=0.7, lw=0.8)
    axes[1].plot(t[skip:], F_body[skip:, 1], label='Fy_body', alpha=0.7, lw=0.8)
    axes[1].plot(t[skip:], F_body[skip:, 2], label='Fz_body', alpha=0.7, lw=0.8)
    axes[1].plot(t[skip:], Fb_mag[skip:], 'k-', label='|F_body|', lw=1.5)
    axes[1].axhline(y=1.5, color='r', linestyle='--', alpha=0.5, label='threshold')
    if contact_time:
        axes[1].axvspan(contact_time, contact_time+contact_duration, alpha=0.15, color='red', label='contact')
    axes[1].set_ylabel('Force body (N)')
    axes[1].legend(loc='upper right', fontsize=8)
    axes[1].grid(True, alpha=0.3)
    
    # 3. 角速度
    axes[2].plot(t[skip:], gyro[skip:, 0], label='p', alpha=0.7, lw=0.8)
    axes[2].plot(t[skip:], gyro[skip:, 1], label='q', alpha=0.7, lw=0.8)
    axes[2].plot(t[skip:], gyro[skip:, 2], label='r', alpha=0.7, lw=0.8)
    axes[2].plot(t[skip:], gyro_mag[skip:], 'k--', label='|omega|', lw=1)
    if contact_time:
        axes[2].axvspan(contact_time, contact_time+contact_duration, alpha=0.15, color='red')
    axes[2].set_ylabel('Gyro (rad/s)')
    axes[2].legend(loc='upper right', fontsize=8)
    axes[2].grid(True, alpha=0.3)
    
    # 4. 力大小对比
    axes[3].plot(t[skip:], Fw_mag[skip:], label='|F_world|', lw=1)
    axes[3].plot(t[skip:], Fb_mag[skip:], label='|F_body|', lw=1)
    axes[3].axhline(y=1.5, color='r', linestyle='--', alpha=0.5, label='threshold')
    if contact_time:
        axes[3].axvspan(contact_time, contact_time+contact_duration, alpha=0.15, color='red')
    axes[3].set_ylabel('Force magnitude (N)')
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
    parser.add_argument('--window', type=float, default=0.5, help='Baseline window (s)')
    parser.add_argument('--lpf', type=float, default=0.9, help='LPF alpha')
    parser.add_argument('--threshold', type=float, default=1.5, help='Force threshold (N)')
    args = parser.parse_args()
    
    run_test(args.log, args.output,
             contact_time=args.contact_time,
             contact_duration=args.contact_duration,
             contact_force=args.contact_force,
             window_s=args.window,
             lpf_alpha=args.lpf,
             force_threshold=args.threshold)


if __name__ == '__main__':
    main()
