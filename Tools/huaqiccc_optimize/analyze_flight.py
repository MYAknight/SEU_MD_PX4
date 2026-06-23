#!/usr/bin/env python3
"""
analyze_flight.py — 自动分析飞行日志，生成自然语言报告和优化建议

用法：
    python3 analyze_flight.py [ulog_path]

如果不提供路径，默认分析 ~/Projects/optimize/logs/latest.ulg

输出：
    - 自然语言飞行情况描述
    - 关键指标表格
    - 优化建议
"""

import os
import sys
import math
import argparse
from collections import defaultdict

import numpy as np


def load_ulog(log_path: str):
    """加载 ulog 文件"""
    try:
        from pyulog import ULog
    except ImportError:
        print("[ERROR] 未安装 pyulog，请运行: pip3 install --user pyulog")
        sys.exit(1)

    if not os.path.exists(log_path):
        print(f"[ERROR] 日志文件不存在: {log_path}")
        sys.exit(1)

    print(f"[INFO] 加载日志: {log_path}")
    return ULog(log_path)


def get_dataset(ulog, name: str):
    """获取指定数据集"""
    for d in ulog.data_list:
        if d.name == name:
            return d
    return None


def extract_timeseries(dataset, field_name: str):
    """从数据集中提取时间序列"""
    if dataset is None:
        return None, None
    try:
        t = dataset.data['timestamp']
        y = dataset.data[field_name]
        # 转换为秒，以第一个点为 t=0
        t0 = t[0]
        t = (t - t0) / 1e6
        return t, y
    except Exception as e:
        print(f"[WARN] 无法提取 {field_name}: {e}")
        return None, None


def compute_rmse(err):
    return math.sqrt(np.mean(np.square(err)))


def compute_max_abs(err):
    return np.max(np.abs(err))


def compute_std(err):
    return np.std(err)


def find_nearest_idx(t_array, target):
    return np.argmin(np.abs(t_array - target))


def analyze_hover(ulog):
    """分析悬停段"""
    local_pos = get_dataset(ulog, 'vehicle_local_position')
    local_pos_sp = get_dataset(ulog, 'vehicle_local_position_setpoint')

    if local_pos is None or local_pos_sp is None:
        return None

    t, x = extract_timeseries(local_pos, 'x')
    _, y = extract_timeseries(local_pos, 'y')
    _, z = extract_timeseries(local_pos, 'z')

    t_sp, x_sp = extract_timeseries(local_pos_sp, 'x')
    _, y_sp = extract_timeseries(local_pos_sp, 'y')
    _, z_sp = extract_timeseries(local_pos_sp, 'z')

    if t is None:
        return None

    # 简单的悬停段检测：找 setpoint 变化较小的 5 秒窗口
    # 这里简化处理：取中间 1/3 时间段
    n = len(t)
    start_idx = n // 3
    end_idx = 2 * n // 3

    err_x = x[start_idx:end_idx] - np.interp(t[start_idx:end_idx], t_sp, x_sp)
    err_y = y[start_idx:end_idx] - np.interp(t[start_idx:end_idx], t_sp, y_sp)
    err_z = z[start_idx:end_idx] - np.interp(t[start_idx:end_idx], t_sp, z_sp)

    xy_err = np.sqrt(err_x**2 + err_y**2)

    return {
        'xy_rmse': compute_rmse(xy_err),
        'xy_max': compute_max_abs(xy_err),
        'z_rmse': compute_rmse(err_z),
        'z_max': compute_max_abs(err_z),
    }


def analyze_attitude(ulog):
    """分析姿态活动度"""
    attitude = get_dataset(ulog, 'vehicle_attitude')
    if attitude is None:
        return None

    t, roll = extract_timeseries(attitude, 'roll')
    _, pitch = extract_timeseries(attitude, 'pitch')
    _, yaw = extract_timeseries(attitude, 'yaw')

    if t is None:
        return None

    # 计算角速度（差分）
    dt = np.diff(t)
    roll_rate = np.diff(roll) / dt
    pitch_rate = np.diff(pitch) / dt
    yaw_rate = np.diff(yaw) / dt

    return {
        'roll_std_deg': np.degrees(np.std(roll_rate)),
        'pitch_std_deg': np.degrees(np.std(pitch_rate)),
        'yaw_std_deg': np.degrees(np.std(yaw_rate)),
        'max_tilt_deg': np.degrees(np.max(np.sqrt(roll**2 + pitch**2))),
    }


def analyze_motor_saturation(ulog):
    """分析电机输出饱和度"""
    outputs = get_dataset(ulog, 'actuator_outputs')
    if outputs is None:
        # 尝试 actuator_motors
        outputs = get_dataset(ulog, 'actuator_motors')

    if outputs is None:
        return None

    # 找到输出字段
    output_fields = [f for f in outputs.field_data if f.field_name.startswith('output[')]
    if not output_fields:
        return None

    all_outputs = []
    for f in output_fields:
        _, val = extract_timeseries(outputs, f.field_name)
        if val is not None:
            all_outputs.append(val)

    if not all_outputs:
        return None

    all_outputs = np.concatenate(all_outputs)
    # PWM 输出通常在 1000~2000 之间，这里归一化
    # 假设最大输出为 2000（PWM us）或 1.0（归一化）
    max_val = np.max(all_outputs)
    if max_val > 100:
        # PWM 模式
        saturation = np.max(all_outputs) / 2000.0
    else:
        # 归一化模式
        saturation = np.max(all_outputs)

    return {
        'max_output': float(np.max(all_outputs)),
        'saturation_ratio': float(saturation),
    }


def analyze_morph(ulog):
    """分析变形过程"""
    morph = get_dataset(ulog, 'huaqiccc_morph_angle')
    if morph is None:
        return None

    t, angle = extract_timeseries(morph, 'arm_angle')
    if t is None:
        return None

    return {
        'min_angle': float(np.min(angle)),
        'max_angle': float(np.max(angle)),
        'angle_range': float(np.max(angle) - np.min(angle)),
    }


def analyze_flight_mode(ulog):
    """分析飞行模式分布"""
    status = get_dataset(ulog, 'vehicle_status')
    if status is None:
        return None

    _, nav_state = extract_timeseries(status, 'nav_state')
    if nav_state is None:
        return None

    # PX4 nav_state 枚举（部分）
    mode_names = {
        0: 'MANUAL',
        1: 'ALTCTL',
        2: 'POSCTL',
        3: 'AUTO_MISSION',
        4: 'AUTO_LOITER',
        5: 'AUTO_RTL',
        6: 'ACRO',
        7: 'OFFBOARD',
        8: 'STABILIZED',
        9: 'RATTITUDE',
        10: 'AUTO_TAKEOFF',
        11: 'AUTO_LAND',
        12: 'AUTO_FOLLOW_TARGET',
        13: 'AUTO_PRECLAND',
        14: 'ORBIT',
        15: 'AUTO_VTOL_TAKEOFF',
    }

    unique, counts = np.unique(nav_state, return_counts=True)
    modes = {}
    for u, c in zip(unique, counts):
        name = mode_names.get(int(u), f'UNKNOWN({u})')
        modes[name] = int(c)

    return modes


def generate_report(ulog_path: str):
    ulog = load_ulog(ulog_path)

    hover = analyze_hover(ulog)
    attitude = analyze_attitude(ulog)
    motor = analyze_motor_saturation(ulog)
    morph = analyze_morph(ulog)
    modes = analyze_flight_mode(ulog)

    # ============================================================
    # 自然语言描述
    # ============================================================
    print("\n" + "=" * 60)
    print("飞行情况自然语言描述")
    print("=" * 60)

    print(f"\n本次飞行日志来自: {ulog_path}")
    print(f"日志时长: {ulog.start_timestamp / 1e6:.1f}s")

    if modes:
        mode_str = ", ".join([f"{k}({v}次采样)" for k, v in modes.items()])
        print(f"飞行模式分布: {mode_str}")

    if hover:
        print(f"\n悬停段位置跟踪表现:")
        print(f"  - XY 方向 RMSE: {hover['xy_rmse']*100:.1f} cm，最大误差: {hover['xy_max']*100:.1f} cm")
        print(f"  - Z 方向 RMSE: {hover['z_rmse']*100:.1f} cm，最大误差: {hover['z_max']*100:.1f} cm")

        if hover['xy_rmse'] < 0.05:
            print("  → XY 跟踪非常稳定。")
        elif hover['xy_rmse'] < 0.10:
            print("  → XY 跟踪良好，但仍有优化空间。")
        else:
            print("  → XY 跟踪较差，建议优先调优位置控制参数。")

        if hover['z_rmse'] < 0.08:
            print("  → Z 轴高度保持良好。")
        elif hover['z_rmse'] < 0.15:
            print("  → Z 轴有轻微波动，可适当增加 Z 轴积分或调整 hover thrust。")
        else:
            print("  → Z 轴波动明显，建议检查 MPC_Z_P、MPC_THR_HOVER 和 EKF2 高度融合。")

    if attitude:
        print(f"\n姿态稳定性:")
        print(f"  - 横滚角速度 std: {attitude['roll_std_deg']:.2f} °/s")
        print(f"  - 俯仰角速度 std: {attitude['pitch_std_deg']:.2f} °/s")
        print(f"  - 偏航角速度 std: {attitude['yaw_std_deg']:.2f} °/s")
        print(f"  - 最大倾斜角: {attitude['max_tilt_deg']:.1f} °")

        if attitude['roll_std_deg'] > 5 or attitude['pitch_std_deg'] > 5:
            print("  → 姿态高频抖动明显，可能 P 过大或 D 不足，建议降低姿态 P 或增加 D。")
        else:
            print("  → 姿态总体平稳。")

    if motor:
        print(f"\n电机输出:")
        print(f"  - 最大输出: {motor['max_output']:.1f}")
        print(f"  - 饱和度: {motor['saturation_ratio']*100:.1f}%")
        if motor['saturation_ratio'] > 0.9:
            print("  → 电机接近饱和，建议降低飞行速度或检查机体平衡。")
        else:
            print("  → 电机余量充足。")

    if morph:
        print(f"\n变形过程:")
        print(f"  - 变形角度范围: {morph['min_angle']:.3f} ~ {morph['max_angle']:.3f} rad")
        print(f"  - 变形幅度: {morph['angle_range']:.3f} rad")
        if morph['angle_range'] > 0.3:
            print("  → 本次飞行包含较大变形，可重点观察变形期间的位置漂移。")

    # ============================================================
    # 关键指标表格
    # ============================================================
    print("\n" + "=" * 60)
    print("关键指标汇总")
    print("=" * 60)

    if hover:
        print(f"{'XY RMSE (cm)':<25} {hover['xy_rmse']*100:>10.1f}")
        print(f"{'XY Max Err (cm)':<25} {hover['xy_max']*100:>10.1f}")
        print(f"{'Z RMSE (cm)':<25} {hover['z_rmse']*100:>10.1f}")
        print(f"{'Z Max Err (cm)':<25} {hover['z_max']*100:>10.1f}")

    if attitude:
        print(f"{'Roll Rate Std (°/s)':<25} {attitude['roll_std_deg']:>10.2f}")
        print(f"{'Pitch Rate Std (°/s)':<25} {attitude['pitch_std_deg']:>10.2f}")
        print(f"{'Yaw Rate Std (°/s)':<25} {attitude['yaw_std_deg']:>10.2f}")
        print(f"{'Max Tilt (°)':<25} {attitude['max_tilt_deg']:>10.1f}")

    if motor:
        print(f"{'Motor Saturation (%)':<25} {motor['saturation_ratio']*100:>10.1f}")

    # ============================================================
    # 优化建议
    # ============================================================
    print("\n" + "=" * 60)
    print("优化建议")
    print("=" * 60)

    suggestions = []

    if hover:
        if hover['xy_rmse'] > 0.10:
            suggestions.append("XY 跟踪误差偏大：尝试增大 MPC_XY_P 或 MPC_XY_VEL_P_ACC（每次 +10%~15%）。")
        if hover['xy_max'] > hover['xy_rmse'] * 3:
            suggestions.append("XY 存在明显尖峰：可能是 setpoint 突变或 D 项不足，可增加 MPC_XY_VEL_D_ACC 或检查轨迹平滑度。")
        if hover['z_rmse'] > 0.15:
            suggestions.append("Z 轴波动大：检查 MPC_Z_P、MPC_THR_HOVER 是否匹配实机，必要时标定 hover thrust。")
        if hover['z_rmse'] > 0.08 and hover['z_max'] < hover['z_rmse'] * 2:
            suggestions.append("Z 轴有稳态漂移：适当增加 Z 轴积分项 MPC_Z_VEL_I_ACC。")

    if attitude:
        if attitude['roll_std_deg'] > 5:
            suggestions.append("横滚轴高频抖动：降低 MC_ROLLRATE_P 或增加 MC_ROLLRATE_D。")
        if attitude['pitch_std_deg'] > 5:
            suggestions.append("俯仰轴高频抖动：降低 MC_PITCHRATE_P 或增加 MC_PITCHRATE_D。")
        if attitude['max_tilt_deg'] > 20:
            suggestions.append("飞行中最大倾斜角较大：可能是速度设定过快或位置 P 过大，可降低 cruise_speed 或 MPC_XY_P。")

    if motor and motor['saturation_ratio'] > 0.85:
        suggestions.append("电机输出接近饱和：降低飞行速度、减小机动幅度，或检查电池电压/螺旋桨效率。")

    if not suggestions:
        print("当前飞行表现良好，没有明显需要优化的项。")
        print("建议：保持当前参数，进行更多轨迹（圆、8字、变形）的重复测试。")
    else:
        for i, s in enumerate(suggestions, 1):
            print(f"{i}. {s}")

    print("\n注意：以上建议基于规则库自动生成，实际调参时请每次只改一个参数，幅度不超过 ±20%。")


def main():
    parser = argparse.ArgumentParser(description="分析飞行日志")
    # USER_CONFIG: change the default log path to your own path
    parser.add_argument("log_path", nargs="?", default="~/Projects/PX4/SEU_MD_PX4/Tools/huaqiccc_optimize/logs/latest.ulg",
                        help="ulog 文件路径，默认分析 latest.ulg")
    args = parser.parse_args()

    log_path = os.path.expanduser(args.log_path)
    generate_report(log_path)


if __name__ == "__main__":
    main()
