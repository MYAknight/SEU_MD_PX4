#!/usr/bin/env python3
"""
huaqiccc_flight_analyzer.py
===========================
huaqiccc 飞行数据后处理分析脚本

功能:
- 读取自动飞行测试生成的 CSV
- 计算轨迹跟踪精度指标（ATE, RMSE, Max Error, RPE 等）
- 生成对比图表（轨迹3D对比、误差时序、误差CDF）
- 输出 Markdown / 文本报告

用法:
    python3 huaqiccc_flight_analyzer.py ~/huaqiccc_logs/huaqiccc_flight_with_algo_figure8_*.csv

    # 对比两个实验结果
    python3 huaqiccc_flight_analyzer.py \
        ~/huaqiccc_logs/*_with_algo_*.csv \
        ~/huaqiccc_logs/*_baseline_*.csv \
        --compare --output report.md

指标说明:
    ATE (Absolute Trajectory Error):
        实际轨迹与参考轨迹对齐后的逐点位置误差
        ATE_RMSE = sqrt(mean(||p_actual - p_ref||^2))

    RPE (Relative Pose Error):
        评估局部一致性，短时间段内的相对位姿漂移
        RPE_RMSE = sqrt(mean(||(p_ref[i]-p_ref[j]) - (p_act[i]-p_act[j])||^2))

    XY_RMSE / Z_RMSE:
        分别评估水平面和垂直方向精度

    Max Error:
        最大绝对误差，反映最坏情况
"""

import argparse
import csv
import math
import os
import sys
from collections import defaultdict

try:
    import numpy as np
    NUMPY_OK = True
except ImportError:
    NUMPY_OK = False
    print("[WARN] numpy 未安装，将使用纯 Python 计算（性能较慢）")

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    MATPLOTLIB_OK = True
except ImportError:
    MATPLOTLIB_OK = False


# ===================== 核心计算函数 =====================

def load_csv(path):
    """加载 CSV 数据"""
    records = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            records.append({
                'time': float(row['time']),
                'sp_x': float(row['sp_x']),
                'sp_y': float(row['sp_y']),
                'sp_z': float(row['sp_z']),
                'act_x': float(row['act_x']),
                'act_y': float(row['act_y']),
                'act_z': float(row['act_z']),
                'err_x': float(row['err_x']),
                'err_y': float(row['err_y']),
                'err_z': float(row['err_z']),
                'arm_angle': float(row.get('arm_angle', 0)),
                'enable_algo': int(row.get('enable_algo', 0)),
            })
    return records


def compute_metrics(records):
    """
    计算轨迹跟踪指标
    返回 dict: {ate_rmse, xy_rmse, z_rmse, max_err, max_xy_err, max_z_err,
                rpe_rmse, mean_err, std_err, median_err, rms_xy, rms_z,
                percentile_95, percentile_90}
    """
    n = len(records)
    if n == 0:
        return {}

    # 逐点 3D 误差
    errors_3d = []
    errors_xy = []
    errors_z = []
    sp_points = []
    act_points = []

    for r in records:
        ex = r['err_x']
        ey = r['err_y']
        ez = r['err_z']
        e3d = math.sqrt(ex*ex + ey*ey + ez*ez)
        exy = math.sqrt(ex*ex + ey*ey)
        errors_3d.append(e3d)
        errors_xy.append(exy)
        errors_z.append(abs(ez))
        sp_points.append((r['sp_x'], r['sp_y'], r['sp_z']))
        act_points.append((r['act_x'], r['act_y'], r['act_z']))

    # ATE = 3D RMSE
    ate_rmse = math.sqrt(sum(e*e for e in errors_3d) / n)
    xy_rmse = math.sqrt(sum(e*e for e in errors_xy) / n)
    z_rmse = math.sqrt(sum(e*e for e in errors_z) / n)

    max_err = max(errors_3d)
    max_xy_err = max(errors_xy)
    max_z_err = max(errors_z)

    mean_err = sum(errors_3d) / n
    std_err = math.sqrt(sum((e - mean_err)**2 for e in errors_3d) / n)

    sorted_err = sorted(errors_3d)
    median_err = sorted_err[n // 2]
    p90 = sorted_err[int(n * 0.90)]
    p95 = sorted_err[int(n * 0.95)]

    # RPE: 1秒间隔的相对位姿误差
    rpe_errors = []
    dt_target = 1.0
    if n > 1:
        avg_dt = (records[-1]['time'] - records[0]['time']) / (n - 1)
        step = max(1, int(dt_target / avg_dt))
        for i in range(0, n - step):
            j = i + step
            # 参考相对位移
            dx_ref = sp_points[j][0] - sp_points[i][0]
            dy_ref = sp_points[j][1] - sp_points[i][1]
            dz_ref = sp_points[j][2] - sp_points[i][2]
            # 实际相对位移
            dx_act = act_points[j][0] - act_points[i][0]
            dy_act = act_points[j][1] - act_points[i][1]
            dz_act = act_points[j][2] - act_points[i][2]
            # 相对误差
            rpe = math.sqrt((dx_ref - dx_act)**2 + (dy_ref - dy_act)**2 + (dz_ref - dz_act)**2)
            rpe_errors.append(rpe)

    rpe_rmse = math.sqrt(sum(e*e for e in rpe_errors) / len(rpe_errors)) if rpe_errors else 0.0

    return {
        'ate_rmse': ate_rmse,
        'xy_rmse': xy_rmse,
        'z_rmse': z_rmse,
        'max_err': max_err,
        'max_xy_err': max_xy_err,
        'max_z_err': max_z_err,
        'rpe_rmse': rpe_rmse,
        'mean_err': mean_err,
        'std_err': std_err,
        'median_err': median_err,
        'p90': p90,
        'p95': p95,
        'n_samples': n,
        'duration': records[-1]['time'] - records[0]['time'],
    }


def format_metrics(metrics, label=""):
    """格式化指标为文本"""
    lines = [
        f"\n{'='*50}",
        f"  飞行测试结果: {label}",
        f"{'='*50}",
        f"  样本数: {metrics['n_samples']} | 飞行时长: {metrics['duration']:.1f}s",
        f"  ┌─────────────────────────────────────────┐",
        f"  │ ATE_RMSE (3D)      : {metrics['ate_rmse']:.4f} m │",
        f"  │ XY_RMSE (水平面)   : {metrics['xy_rmse']:.4f} m │",
        f"  │ Z_RMSE (垂直)      : {metrics['z_rmse']:.4f} m │",
        f"  │ RPE_RMSE (相对)    : {metrics['rpe_rmse']:.4f} m │",
        f"  │ Max Error (3D)     : {metrics['max_err']:.4f} m │",
        f"  │ Max XY Error       : {metrics['max_xy_err']:.4f} m │",
        f"  │ Max Z Error        : {metrics['max_z_err']:.4f} m │",
        f"  │ Mean Error         : {metrics['mean_err']:.4f} m │",
        f"  │ Median Error       : {metrics['median_err']:.4f} m │",
        f"  │ Std Dev            : {metrics['std_err']:.4f} m │",
        f"  │ 90th Percentile    : {metrics['p90']:.4f} m │",
        f"  │ 95th Percentile    : {metrics['p95']:.4f} m │",
        f"  └─────────────────────────────────────────┘",
    ]
    return '\n'.join(lines)


def plot_trajectory(records, save_path=None):
    """绘制3D轨迹对比图"""
    if not MATPLOTLIB_OK:
        print("[SKIP] matplotlib 未安装，跳过绘图")
        return

    fig = plt.figure(figsize=(12, 5))

    # 3D 轨迹
    ax1 = fig.add_subplot(131, projection='3d')
    sp_x = [r['sp_x'] for r in records]
    sp_y = [r['sp_y'] for r in records]
    sp_z = [r['sp_z'] for r in records]
    act_x = [r['act_x'] for r in records]
    act_y = [r['act_y'] for r in records]
    act_z = [r['act_z'] for r in records]

    ax1.plot(sp_x, sp_y, sp_z, 'b-', linewidth=1.5, label='Reference', alpha=0.7)
    ax1.plot(act_x, act_y, act_z, 'r--', linewidth=1.5, label='Actual', alpha=0.7)
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_zlabel('Z [m]')
    ax1.set_title('3D Trajectory')
    ax1.legend()

    # XY 平面投影
    ax2 = fig.add_subplot(132)
    ax2.plot(sp_x, sp_y, 'b-', linewidth=1.5, label='Reference', alpha=0.7)
    ax2.plot(act_x, act_y, 'r--', linewidth=1.5, label='Actual', alpha=0.7)
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Y [m]')
    ax2.set_title('XY Projection')
    ax2.legend()
    ax2.axis('equal')

    # 误差时序
    ax3 = fig.add_subplot(133)
    times = [r['time'] for r in records]
    errors = [math.sqrt(r['err_x']**2 + r['err_y']**2 + r['err_z']**2) for r in records]
    ax3.plot(times, errors, 'g-', linewidth=1.0)
    ax3.axhline(y=np.mean(errors) if NUMPY_OK else sum(errors)/len(errors),
                color='orange', linestyle='--', label='Mean')
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Error [m]')
    ax3.set_title('Tracking Error Over Time')
    ax3.legend()

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[PLOT] 轨迹图已保存: {save_path}")
    else:
        plt.savefig('trajectory_analysis.png', dpi=150, bbox_inches='tight')
        print("[PLOT] 轨迹图已保存: trajectory_analysis.png")
    plt.close()


def plot_comparison(records_list, labels, save_path='comparison.png'):
    """对比多个实验结果"""
    if not MATPLOTLIB_OK or len(records_list) < 2:
        return

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']

    # 1. XY 轨迹对比
    ax = axes[0, 0]
    for idx, (recs, label) in enumerate(zip(records_list, labels)):
        ax.plot([r['act_x'] for r in recs], [r['act_y'] for r in recs],
                color=colors[idx % len(colors)], linewidth=1.5,
                label=label, alpha=0.8)
    # 只画一次参考轨迹
    if records_list:
        ax.plot([r['sp_x'] for r in records_list[0]],
                [r['sp_y'] for r in records_list[0]],
                'k--', linewidth=1.0, label='Reference', alpha=0.5)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title('Trajectory Comparison (XY)')
    ax.legend()
    ax.axis('equal')

    # 2. 误差时序对比
    ax = axes[0, 1]
    for idx, (recs, label) in enumerate(zip(records_list, labels)):
        times = [r['time'] for r in recs]
        errs = [math.sqrt(r['err_x']**2 + r['err_y']**2 + r['err_z']**2) for r in recs]
        ax.plot(times, errs, color=colors[idx % len(colors)],
                linewidth=1.0, label=label, alpha=0.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('3D Error [m]')
    ax.set_title('Tracking Error Comparison')
    ax.legend()

    # 3. CDF 对比
    ax = axes[1, 0]
    for idx, (recs, label) in enumerate(zip(records_list, labels)):
        errs = sorted([math.sqrt(r['err_x']**2 + r['err_y']**2 + r['err_z']**2) for r in recs])
        n = len(errs)
        cdf = [i / n for i in range(n)]
        ax.plot(errs, cdf, color=colors[idx % len(colors)],
                linewidth=1.5, label=label)
    ax.set_xlabel('Error [m]')
    ax.set_ylabel('CDF')
    ax.set_title('Error Cumulative Distribution')
    ax.legend()

    # 4. 指标柱状图
    ax = axes[1, 1]
    metrics_list = [compute_metrics(recs) for recs in records_list]
    x = np.arange(len(labels))
    width = 0.25
    for idx, key in enumerate(['ate_rmse', 'xy_rmse', 'max_err']):
        vals = [m.get(key, 0) for m in metrics_list]
        ax.bar(x + idx*width, vals, width, label=key)
    ax.set_ylabel('Error [m]')
    ax.set_title('Key Metrics Comparison')
    ax.set_xticks(x + width)
    ax.set_xticklabels(labels, rotation=15, ha='right')
    ax.legend()

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"[PLOT] 对比图已保存: {save_path}")
    plt.close()


def generate_report(metrics_list, labels, output_path='report.md'):
    """生成 Markdown 报告"""
    lines = [
        "# huaqiccc 变形控制算法有效性验证报告",
        "",
        f"生成时间: {__import__('datetime').datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        "",
        "## 实验设计",
        "",
        "| 项目 | 说明 |",
        "|------|------|",
        "| 对比方法 | A/B 对照实验 |",
        "| A组 (with_algo) | 开启动态参数调整（31000命令实时更新电机位置）|",
        "| B组 (baseline) | 关闭动态参数调整（使用固定airframe参数）|",
        "| 评估指标 | ATE_RMSE, XY_RMSE, Z_RMSE, Max Error, RPE_RMSE, P95 |",
        "",
        "## 结果汇总",
        "",
    ]

    # 汇总表格
    lines.append("| 实验 | ATE_RMSE | XY_RMSE | Z_RMSE | Max Error | RPE | P95 |")
    lines.append("|------|----------|---------|--------|-----------|-----|-----|")
    for m, label in zip(metrics_list, labels):
        lines.append(f"| {label} | {m['ate_rmse']:.4f} | {m['xy_rmse']:.4f} | "
                     f"{m['z_rmse']:.4f} | {m['max_err']:.4f} | "
                     f"{m['rpe_rmse']:.4f} | {m['p95']:.4f} |")

    lines.extend(["", "## 详细结果", ""])
    for m, label in zip(metrics_list, labels):
        lines.append(format_metrics(m, label))
        lines.append("")

    # 结论自动判断
    if len(metrics_list) == 2:
        m_a, m_b = metrics_list[0], metrics_list[1]
        ate_improve = ((m_b['ate_rmse'] - m_a['ate_rmse']) / m_b['ate_rmse'] * 100) if m_b['ate_rmse'] > 0 else 0
        max_improve = ((m_b['max_err'] - m_a['max_err']) / m_b['max_err'] * 100) if m_b['max_err'] > 0 else 0

        lines.extend([
            "## 结论",
            "",
            f"- **ATE_RMSE 改善**: {ate_improve:+.1f}% "
            f"({'优于' if ate_improve > 0 else '劣于'}对照组)",
            f"- **Max Error 改善**: {max_improve:+.1f}% "
            f"({'优于' if max_improve > 0 else '劣于'}对照组)",
            "",
            "> 注: 正值表示开算法组精度更高（误差更小）",
            "",
        ])

    with open(output_path, 'w') as f:
        f.write('\n'.join(lines))
    print(f"[REPORT] Markdown 报告已保存: {output_path}")


def main():
    parser = argparse.ArgumentParser(description='huaqiccc 飞行数据分析')
    parser.add_argument('files', nargs='+', help='CSV 数据文件路径')
    parser.add_argument('--compare', action='store_true',
                        help='对比多个文件')
    parser.add_argument('--output', default='report.md',
                        help='输出报告文件名')
    parser.add_argument('--no-plot', action='store_true',
                        help='跳过绘图')

    args = parser.parse_args()

    records_list = []
    labels = []
    metrics_list = []

    for f in args.files:
        if not os.path.exists(f):
            print(f"[SKIP] 文件不存在: {f}")
            continue
        recs = load_csv(f)
        label = f"{'with_algo' if recs[0].get('enable_algo',0) else 'baseline'}_arm{recs[0].get('arm_angle',0):.2f}"
        labels.append(label)
        records_list.append(recs)

        metrics = compute_metrics(recs)
        metrics_list.append(metrics)
        print(format_metrics(metrics, label))

        if not args.no_plot and len(args.files) == 1:
            plot_trajectory(recs, save_path=f.replace('.csv', '.png'))

    # 对比模式
    if args.compare and len(records_list) >= 2:
        print("\n[COMPARE] 生成对比分析...")
        plot_comparison(records_list, labels, save_path='comparison.png')
        generate_report(metrics_list, labels, output_path=args.output)
    elif len(records_list) == 1:
        generate_report(metrics_list, labels, output_path=args.output)


if __name__ == '__main__':
    main()
