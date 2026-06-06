#!/usr/bin/env python3
"""Parse A/B test batch console log for results."""

import re
import sys
from collections import defaultdict

def parse_log(path):
    with open(path) as f:
        lines = f.readlines()
    
    results = []
    current = {}
    
    for line in lines:
        line = line.strip()
        
        # Detect test start
        m = re.search(r'Testing k_soft=(\S+)\s+run (\d+)/(\d+)', line)
        if m:
            if current:
                results.append(current)
            current = {
                'k_soft': float(m.group(1)),
                'run': int(m.group(2)),
                'success': False,
                'z_mean': 0.0,
                'z_std': 0.0,
                'x_std': 0.0,
            }
        
        # Detect monitor results
        m = re.search(r'MONITOR\] z_mean=([\d.]+)m, z_std=([\d.]+)m, x_std=([\d.]+)m', line)
        if m and current:
            current['z_mean'] = float(m.group(1))
            current['z_std'] = float(m.group(2))
            current['x_std'] = float(m.group(3))
        
        # Detect success/failure
        if 'SUCCESS - drone remains perched' in line and current:
            current['success'] = True
        elif 'FAILURE' in line and current:
            current['success'] = False
    
    if current:
        results.append(current)
    
    return results

def main():
    path = '/tmp/ab_test_batch.log'
    results = parse_log(path)
    
    grouped = defaultdict(list)
    for r in results:
        grouped[r['k_soft']].append(r)
    
    print("=" * 60)
    print("  A/B Test Results (from console log)")
    print("=" * 60)
    print(f"{'k_soft':>8s} {'n':>3s} {'SR%':>6s} {'z_mean':>8s} {'z_std':>8s}")
    print("-" * 60)
    
    for k in sorted(grouped.keys()):
        runs = grouped[k]
        n = len(runs)
        sr = sum(1 for r in runs if r['success']) / n * 100 if n else 0
        z_mean = sum(r['z_mean'] for r in runs) / n if n else 0
        z_std = sum(r['z_std'] for r in runs) / n if n else 0
        print(f"{k:>8.2f} {n:>3d} {sr:>6.1f} {z_mean:>8.3f} {z_std:>8.4f}")
    
    print("-" * 60)

if __name__ == '__main__':
    main()
