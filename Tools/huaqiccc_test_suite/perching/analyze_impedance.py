#!/usr/bin/env python3
"""
analyze_impedance.py
====================
Analyze impedance control benchmark results from CSV logs.

Computes metrics:
  1. Contact Impact (CI):    max acceleration norm around stall detect
  2. Post-Contact Osc (PCO): position std during first 2s after contact
  3. Convergence Time (CT):  time from contact to disarm
  4. Final Pos Accuracy (FPA): mean position error in monitor phase
  5. Success Rate (SR):      fraction of SUCCESS trials

Usage:
    python3 analyze_impedance.py
    # or specify prefix:
    python3 analyze_impedance.py --prefix impedance
"""

import argparse
import csv
import glob
import math
import os
import statistics
from collections import defaultdict


def parse_csv(path):
    """Read CSV and return list of dicts."""
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        return [row for row in reader]


def find_contact_time(rows):
    """Find first row with phase == 'push' after which contact was detected.
    We use the heuristic: first 'hold' phase marks end of push/contact."""
    for i, row in enumerate(rows):
        if row['phase'] == 'hold':
            # contact happened during push, just before first hold
            return float(rows[i-1]['time']) if i > 0 else float(row['time'])
    return None


def find_stall_window(rows, contact_t):
    """Find stall detect window: push phase rows near contact."""
    window = []
    for row in rows:
        if row['phase'] == 'push':
            t = float(row['time'])
            if contact_t - 1.0 <= t <= contact_t + 1.0:
                window.append(row)
    return window


def compute_metrics(rows):
    """Compute all metrics for a single trial."""
    contact_t = find_contact_time(rows)
    if contact_t is None:
        return None

    # 1. Contact Impact (CI): max acceleration norm in [contact-0.5, contact+0.5]
    ci_window = [r for r in rows
                 if 'a_norm' in r and r['a_norm']
                 and contact_t - 0.5 <= float(r['time']) <= contact_t + 0.5]
    ci = max(float(r['a_norm']) for r in ci_window) if ci_window else 0.0

    # 2. Post-Contact Oscillation (PCO): std(x,z) in [contact, contact+3]
    pco_window = [r for r in rows
                  if contact_t <= float(r['time']) <= contact_t + 3.0]
    xs = [float(r['act_x']) for r in pco_window if r.get('act_x')]
    zs = [float(r['act_z']) for r in pco_window if r.get('act_z')]
    pco_x = statistics.stdev(xs) if len(xs) > 1 else 0.0
    pco_z = statistics.stdev(zs) if len(zs) > 1 else 0.0

    # 3. Convergence Time (CT): contact to last hold/disarm
    last_t = max(float(r['time']) for r in rows if r['phase'] in ('hold', 'monitor'))
    ct = last_t - contact_t

    # 4. Final Position Accuracy (FPA): monitor phase mean abs error vs target
    mon = [r for r in rows if r['phase'] == 'monitor']
    if mon:
        # Target is perching_x = 5.25 (POLE_X + 0.25)
        target_x = 5.25
        target_z = 2.5
        fpa_x = statistics.mean(abs(float(r['act_x']) - target_x) for r in mon)
        fpa_z = statistics.mean(abs(float(r['act_z']) - target_z) for r in mon)
    else:
        fpa_x = fpa_z = 0.0

    # 5. Success: monitor has data and z_mean > 1.5
    success = len(mon) > 5 and statistics.mean(float(r['act_z']) for r in mon) > 1.5

    return {
        'contact_t': contact_t,
        'ci': ci,
        'pco_x': pco_x,
        'pco_z': pco_z,
        'ct': ct,
        'fpa_x': fpa_x,
        'fpa_z': fpa_z,
        'success': success,
    }


def summarize(group_results):
    """Compute mean±std for each metric across trials in a group."""
    def stat(values):
        if not values:
            return (0.0, 0.0)
        return (statistics.mean(values), statistics.stdev(values) if len(values) > 1 else 0.0)

    metrics = ['ci', 'pco_x', 'pco_z', 'ct', 'fpa_x', 'fpa_z']
    summary = {}
    for m in metrics:
        vals = [r[m] for r in group_results]
        summary[m] = stat(vals)
    summary['success_rate'] = sum(1 for r in group_results if r['success']) / len(group_results)
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--prefix', default='impedance', help='CSV filename prefix')
    parser.add_argument('--dir', default=os.path.expanduser('~/huaqiccc_logs'), help='Log directory')
    args = parser.parse_args()

    pattern = os.path.join(args.dir, f"{args.prefix}_*.csv")
    files = sorted(glob.glob(pattern))
    if not files:
        print(f"[ERROR] No CSV files found matching {pattern}")
        return

    print(f"[INFO] Found {len(files)} CSV files")

    # Group by k_soft value extracted from filename
    groups = defaultdict(list)
    for f in files:
        basename = os.path.basename(f)
        # e.g., impedance_k020_trial_20260528_123456.csv
        # Extract group key (k100, k020, k005)
        parts = basename.split('_')
        group_key = parts[1] if len(parts) > 1 else 'unknown'
        rows = parse_csv(f)
        metrics = compute_metrics(rows)
        if metrics:
            groups[group_key].append(metrics)
            print(f"  [OK] {basename}: contact_t={metrics['contact_t']:.1f}s  CI={metrics['ci']:.2f}  success={metrics['success']}")
        else:
            print(f"  [WARN] {basename}: no contact detected")

    print("\n" + "=" * 70)
    print("IMPEDANCE CONTROL BENCHMARK REPORT")
    print("=" * 70)

    # Print per-group summary
    for group_key in sorted(groups.keys()):
        results = groups[group_key]
        if not results:
            continue
        summary = summarize(results)
        n = len(results)
        print(f"\n--- Group: {group_key} (n={n}) ---")
        print(f"  Success Rate:       {summary['success_rate']*100:.0f}%")
        print(f"  Contact Impact:     {summary['ci'][0]:.3f} ± {summary['ci'][1]:.3f}  (lower = softer)")
        print(f"  Post-Contact Osc X: {summary['pco_x'][0]:.4f} ± {summary['pco_x'][1]:.4f}  (lower = stabler)")
        print(f"  Post-Contact Osc Z: {summary['pco_z'][0]:.4f} ± {summary['pco_z'][1]:.4f}  (lower = stabler)")
        print(f"  Convergence Time:   {summary['ct'][0]:.1f} ± {summary['ct'][1]:.1f}s")
        print(f"  Final Pos Err X:    {summary['fpa_x'][0]:.3f} ± {summary['fpa_x'][1]:.3f}m")
        print(f"  Final Pos Err Z:    {summary['fpa_z'][0]:.3f} ± {summary['fpa_z'][1]:.3f}m")

    # Comparative analysis
    if len(groups) >= 2:
        print("\n--- Comparative Analysis ---")
        keys = sorted(groups.keys())
        baseline = keys[0]
        for k in keys[1:]:
            b = summarize(groups[baseline])
            t = summarize(groups[k])
            print(f"\n{k} vs {baseline}:")
            if t['ci'][0] < b['ci'][0]:
                delta = (b['ci'][0] - t['ci'][0]) / b['ci'][0] * 100 if b['ci'][0] > 0 else 0
                print(f"  ✓ Contact Impact reduced by {delta:.0f}%")
            else:
                print(f"  ✗ Contact Impact INCREASED")
            if t['pco_x'][0] < b['pco_x'][0]:
                print(f"  ✓ Post-Contact X-osc reduced")
            if t['pco_z'][0] < b['pco_z'][0]:
                print(f"  ✓ Post-Contact Z-osc reduced")
            if t['success_rate'] >= b['success_rate']:
                print(f"  ✓ Success rate maintained or improved")

    print("\n" + "=" * 70)
    print("INTERPRETATION GUIDE")
    print("=" * 70)
    print("""
Contact Impact (CI):
  - Measures peak IMU acceleration at contact
  - Impedance ON should show LOWER values than OFF
  - Ideal: < 15 m/s² (smooth contact)

Post-Contact Oscillation (PCO):
  - Measures position stability in first 3s after contact
  - Impedance ON should show LOWER std values
  - Ideal: x_std < 0.05m, z_std < 0.05m

Success Rate:
  - Fraction of trials where drone remains perched
  - Should be >= 66% (2/3) for viable parameter

If k_soft=0.20 shows lower CI/PCO than k_soft=1.00
while maintaining success rate, impedance control
is confirmed to have positive effect.
""")


if __name__ == '__main__':
    main()
