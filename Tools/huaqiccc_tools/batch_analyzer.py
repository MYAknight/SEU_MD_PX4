#!/usr/bin/env python3
"""
huaqiccc_batch_analyzer.py
==========================
批量分析多轮飞行实验数据

用法:
    python3 huaqiccc_batch_analyzer.py ~/Desktop/log1
    python3 huaqiccc_batch_analyzer.py ~/Desktop/log1 --output my_report

输出:
    - batch_analysis.png   (四图对比)
    - batch_analysis.md    (Markdown 统计报告)
"""

import argparse
import csv
import glob
import math
import os
from collections import defaultdict

try:
    import numpy as np
    NUMPY_OK = True
except ImportError:
    NUMPY_OK = False

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    MATPLOTLIB_OK = True
except ImportError:
    MATPLOTLIB_OK = False


def load_csv(path):
    records = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            records.append({
                'time': float(row['time']),
                'err_x': float(row['err_x']),
                'err_y': float(row['err_y']),
                'err_z': float(row['err_z']),
            })
    return records


def compute_metrics(records):
    n = len(records)
    if n == 0:
        return {}
    
    errors_3d, errors_xy, errors_z = [], [], []
    for r in records:
        ex, ey, ez = r['err_x'], r['err_y'], r['err_z']
        e3d = math.sqrt(ex*ex + ey*ey + ez*ez)
        exy = math.sqrt(ex*ex + ey*ey)
        errors_3d.append(e3d)
        errors_xy.append(exy)
        errors_z.append(abs(ez))
    
    mean_err = sum(errors_3d) / n
    variance = sum((e - mean_err)**2 for e in errors_3d) / n
    
    sorted_err = sorted(errors_3d)
    
    return {
        'ate_rmse': math.sqrt(sum(e*e for e in errors_3d) / n),
        'xy_rmse': math.sqrt(sum(e*e for e in errors_xy) / n),
        'z_rmse': math.sqrt(sum(e*e for e in errors_z) / n),
        'max_err': max(errors_3d),
        'mean_err': mean_err,
        'std_err': math.sqrt(variance),
        'median_err': sorted_err[n // 2],
        'p90': sorted_err[int(n * 0.90)],
        'p95': sorted_err[int(n * 0.95)],
        'n_samples': n,
    }


def stat_summary(values):
    if not values:
        return 0, 0
    n = len(values)
    mean = sum(values) / n
    variance = sum((v - mean)**2 for v in values) / n
    return mean, math.sqrt(variance)


def main():
    parser = argparse.ArgumentParser(description='huaqiccc 批量飞行数据分析')
    parser.add_argument('input_dir', help='CSV 文件所在目录')
    parser.add_argument('--output', '-o', default='batch_analysis', help='输出文件名前缀')
    args = parser.parse_args()

    # 1. 扫描并按组分类
    files = glob.glob(os.path.join(args.input_dir, '*.csv'))
    groups = defaultdict(list)
    
    for f in sorted(files):
        basename = os.path.basename(f)
        if 'baseline' in basename:
            group = 'baseline'
        elif 'with_algo' in basename:
            group = 'with_algo'
        else:
            continue
        
        records = load_csv(f)
        metrics = compute_metrics(records)
        groups[group].append({
            'file': basename,
            'records': records,
            'metrics': metrics
        })
    
    if len(groups) < 2:
        print(f"[ERROR] 需要至少 baseline 和 with_algo 两组数据，当前只有 {list(groups.keys())}")
        return
    
    print(f"\n{'='*60}")
    print("  huaqiccc 批量分析")
    print(f"{'='*60}")
    for g, items in groups.items():
        print(f"  {g}: {len(items)} 轮")
    
    # 2. 跨轮统计
    group_stats = {}
    for group, items in groups.items():
        stats = {}
        for k in ['ate_rmse', 'xy_rmse', 'z_rmse', 'max_err', 'mean_err', 'median_err', 'p95']:
            vals = [item['metrics'][k] for item in items]
            mean, std = stat_summary(vals)
            stats[k] = {'mean': mean, 'std': std, 'values': vals}
        group_stats[group] = stats
    
    # 3. 打印结果
    print(f"\n{'='*60}")
    print("  统计结果（均值 ± 标准差）")
    print(f"{'='*60}")
    print(f"{'指标':<15} {'baseline':>18} {'with_algo':>18}")
    print("-" * 55)
    for k in ['ate_rmse', 'xy_rmse', 'z_rmse', 'max_err', 'p95']:
        b = group_stats['baseline'][k]
        a = group_stats['with_algo'][k]
        print(f"{k:<15} {b['mean']:>8.4f}±{b['std']:<7.4f} {a['mean']:>8.4f}±{a['std']:<7.4f}")
    
    # 4. 对比
    print(f"\n{'='*60}")
    print("  对比分析（with_algo vs baseline）")
    print(f"{'='*60}")
    print(f"{'指标':<15} {'baseline':>12} {'with_algo':>12} {'改善率':>10}")
    print("-" * 55)
    
    improvements = {}
    for k in ['ate_rmse', 'xy_rmse', 'z_rmse', 'max_err', 'p95']:
        b = group_stats['baseline'][k]['mean']
        a = group_stats['with_algo'][k]['mean']
        improve = (b - a) / b * 100 if b > 0 else 0
        improvements[k] = improve
        print(f"{k:<15} {b:>12.4f} {a:>12.4f} {improve:>+9.1f}%")
    
    # 5. 绘图
    if MATPLOTLIB_OK:
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        # 图1: 关键指标柱状图（带误差棒）
        ax = axes[0, 0]
        keys = ['ate_rmse', 'xy_rmse', 'max_err', 'p95']
        x = np.arange(len(keys)) if NUMPY_OK else list(range(len(keys)))
        width = 0.35
        
        b_means = [group_stats['baseline'][k]['mean'] for k in keys]
        b_stds  = [group_stats['baseline'][k]['std'] for k in keys]
        a_means = [group_stats['with_algo'][k]['mean'] for k in keys]
        a_stds  = [group_stats['with_algo'][k]['std'] for k in keys]
        
        if NUMPY_OK:
            ax.bar(x - width/2, b_means, width, yerr=b_stds, label='baseline', capsize=3, color='#1f77b4')
            ax.bar(x + width/2, a_means, width, yerr=a_stds, label='with_algo', capsize=3, color='#ff7f0e')
        else:
            ax.bar([i - width/2 for i in x], b_means, width, label='baseline', color='#1f77b4')
            ax.bar([i + width/2 for i in x], a_means, width, label='with_algo', color='#ff7f0e')
        
        ax.set_ylabel('Error [m]')
        ax.set_title('Key Metrics (Mean ± Std)')
        ax.set_xticks(x)
        ax.set_xticklabels(keys, rotation=15, ha='right')
        ax.legend()
        
        # 图2: 每轮 ATE_RMSE 散点
        ax = axes[0, 1]
        b_vals = group_stats['baseline']['ate_rmse']['values']
        a_vals = group_stats['with_algo']['ate_rmse']['values']
        ax.scatter(range(1, len(b_vals)+1), b_vals, label='baseline', s=100, marker='o', color='#1f77b4')
        ax.scatter(range(1, len(a_vals)+1), a_vals, label='with_algo', s=100, marker='s', color='#ff7f0e')
        ax.set_xlabel('Run #')
        ax.set_ylabel('ATE_RMSE [m]')
        ax.set_title('ATE_RMSE per Run')
        ax.legend()
        
        # 图3: CDF（组内所有数据点聚合）
        ax = axes[1, 0]
        for group, color in [('baseline', '#1f77b4'), ('with_algo', '#ff7f0e')]:
            all_errors = []
            for item in groups[group]:
                for r in item['records']:
                    e = math.sqrt(r['err_x']**2 + r['err_y']**2 + r['err_z']**2)
                    all_errors.append(e)
            all_errors.sort()
            n = len(all_errors)
            cdf = [i/n for i in range(n)]
            ax.plot(all_errors, cdf, label=group, linewidth=1.5, color=color)
        ax.set_xlabel('3D Error [m]')
        ax.set_ylabel('CDF')
        ax.set_title('Error Cumulative Distribution')
        ax.legend()
        
        # 图4: 误差时序均值曲线
        ax = axes[1, 1]
        for group, color in [('baseline', '#1f77b4'), ('with_algo', '#ff7f0e')]:
            time_aligned = defaultdict(list)
            for item in groups[group]:
                for r in item['records']:
                    t = round(r['time'], 1)
                    e = math.sqrt(r['err_x']**2 + r['err_y']**2 + r['err_z']**2)
                    time_aligned[t].append(e)
            times = sorted(time_aligned.keys())
            means = [sum(time_aligned[t])/len(time_aligned[t]) for t in times]
            ax.plot(times, means, label=group, linewidth=1.2, color=color)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Mean 3D Error [m]')
        ax.set_title('Mean Tracking Error Over Time')
        ax.legend()
        
        plt.tight_layout()
        out_png = os.path.join(args.input_dir, f"{args.output}.png")
        plt.savefig(out_png, dpi=150, bbox_inches='tight')
        print(f"\n[PLOT] 图表已保存: {out_png}")
        plt.close()
    
    # 6. Markdown 报告
    report_path = os.path.join(args.input_dir, f"{args.output}.md")
    with open(report_path, 'w') as f:
        f.write("# huaqiccc 变形控制算法批量验证报告\n\n")
        f.write("| 组别 | 轮次数 | 说明 |\n")
        f.write("|------|--------|------|\n")
        f.write(f"| baseline | {len(groups['baseline'])} | 固定参数对照组 |\n")
        f.write(f"| with_algo | {len(groups['with_algo'])} | 动态参数调整组 |\n\n")
        
        f.write("## 统计结果（均值 ± 标准差）\n\n")
        f.write("| 指标 | baseline | with_algo |\n")
        f.write("|------|----------|-----------|\n")
        for k in ['ate_rmse', 'xy_rmse', 'z_rmse', 'max_err', 'mean_err', 'median_err', 'p95']:
            b = group_stats['baseline'][k]
            a = group_stats['with_algo'][k]
            f.write(f"| {k} | {b['mean']:.4f} ± {b['std']:.4f} | {a['mean']:.4f} ± {a['std']:.4f} |\n")
        f.write("\n")
        
        f.write("## 对比分析\n\n")
        f.write("| 指标 | baseline | with_algo | 改善率 |\n")
        f.write("|------|----------|-----------|--------|\n")
        for k in ['ate_rmse', 'xy_rmse', 'z_rmse', 'max_err', 'p95']:
            b = group_stats['baseline'][k]['mean']
            a = group_stats['with_algo'][k]['mean']
            imp = improvements[k]
            f.write(f"| {k} | {b:.4f} | {a:.4f} | {imp:+.1f}% |\n")
        f.write("\n")
        
        ate_imp = improvements['ate_rmse']
        max_imp = improvements['max_err']
        f.write("## 结论\n\n")
        if ate_imp > 0:
            f.write(f"动态参数调整使 ATE_RMSE 平均改善 **{ate_imp:.1f}%**，")
            f.write(f"最大误差改善 **{max_imp:.1f}%**。")
            f.write("实验结果表明算法对提升变形飞行跟踪精度具有积极作用。\n")
        else:
            f.write(f"ATE_RMSE 变化 {ate_imp:.1f}%，未观察到明显改善。")
            f.write("建议检查参数更新频率、混控矩阵计算或增加样本量。\n")
    
    print(f"[REPORT] 报告已保存: {report_path}")


if __name__ == '__main__':
    main()