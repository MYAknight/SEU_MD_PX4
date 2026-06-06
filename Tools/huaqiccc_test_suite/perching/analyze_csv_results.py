#!/usr/bin/env python3
"""Analyze impedance control A/B test CSV results."""

import csv
import glob
import os
import sys
from collections import defaultdict

def analyze_csv(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    
    push_rows = [r for r in rows if r['phase'] == 'push']
    hold_rows = [r for r in rows if r['phase'] == 'hold']
    monitor_rows = [r for r in rows if r['phase'] == 'monitor']
    
    # Contact Impact (CI): peak acceleration during push
    ci = max(float(r['a_norm']) for r in push_rows) if push_rows else 0.0
    
    # Post-Contact Oscillation (PCO): max acceleration during hold
    pco = max(float(r['a_norm']) for r in hold_rows) if hold_rows else 0.0
    
    # Hold mean acceleration
    hold_mean = sum(float(r['a_norm']) for r in hold_rows) / len(hold_rows) if hold_rows else 0.0
    
    # Success: monitor z_mean > 1.5 and z_std < 0.5
    success = False
    z_mean = 0.0
    z_std = 0.0
    if monitor_rows:
        zs = [float(r['act_z']) for r in monitor_rows]
        z_mean = sum(zs) / len(zs)
        z_std = (sum((z - z_mean)**2 for z in zs) / len(zs)) ** 0.5
        success = z_mean > 1.5 and z_std < 0.5
    
    return {
        'ci': ci,
        'pco': pco,
        'hold_mean': hold_mean,
        'z_mean': z_mean,
        'z_std': z_std,
        'success': success,
    }

def main():
    log_dir = os.path.expanduser('~/huaqiccc_logs')
    files = glob.glob(os.path.join(log_dir, 'k*_r*_success_*.csv')) + \
            glob.glob(os.path.join(log_dir, 'k*_r*_fail_*.csv'))
    
    results = defaultdict(list)
    for f in sorted(files):
        basename = os.path.basename(f)
        # Parse k value from filename like k100_r1_success_...
        parts = basename.split('_')
        if len(parts) >= 2 and parts[0].startswith('k'):
            k_label = parts[0]  # e.g., k100, k020, k005
            k_val = float(k_label[1:]) / 100.0 if len(k_label) > 2 else float(k_label[1:]) / 10.0
            # Handle k100 -> 1.00, k020 -> 0.20, k005 -> 0.05
            if k_label == 'k100':
                k_val = 1.00
            elif k_label == 'k020':
                k_val = 0.20
            elif k_label == 'k005':
                k_val = 0.05
            
            metrics = analyze_csv(f)
            results[k_val].append(metrics)
    
    if not results:
        print("No result files found in", log_dir)
        sys.exit(1)
    
    print("=" * 70)
    print("  Impedance Control A/B Test Results")
    print("=" * 70)
    print("")
    print(f"{'k_soft':>8s} {'n':>3s} {'SR%':>5s} {'CI(mean)':>10s} {'PCO(mean)':>11s} {'Hold(mean)':>12s} {'z_mean':>8s} {'z_std':>8s}")
    print("-" * 70)
    
    for k_val in sorted(results.keys()):
        runs = results[k_val]
        n = len(runs)
        sr = sum(1 for r in runs if r['success']) / n * 100
        ci_mean = sum(r['ci'] for r in runs) / n
        pco_mean = sum(r['pco'] for r in runs) / n
        hold_mean = sum(r['hold_mean'] for r in runs) / n
        z_mean = sum(r['z_mean'] for r in runs) / n
        z_std = sum(r['z_std'] for r in runs) / n
        
        print(f"{k_val:>8.2f} {n:>3d} {sr:>5.1f} {ci_mean:>10.2f} {pco_mean:>11.2f} {hold_mean:>12.2f} {z_mean:>8.3f} {z_std:>8.4f}")
    
    print("-" * 70)
    print("CI  = Contact Impact (peak acceleration during push, g)")
    print("PCO = Post-Contact Oscillation (peak acceleration during hold, g)")
    print("SR  = Success Rate (%)")

if __name__ == '__main__':
    main()
