#!/usr/bin/env python3
"""
Analyze contact_compare_ros CSV outputs.
Compares motor PWMs between spring model and hard-push tests.
"""

import argparse
import csv
import glob
import math
import os
import sys


def load_contact_phase(csv_path):
    rows = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get('phase') == 'contact':
                rows.append(row)
    return rows


def analyze(rows, label):
    motors = []
    pos_x = []
    for r in rows:
        if r.get('motor_avg') not in (None, '', 'None'):
            motors.append(float(r['motor_avg']))
        if r.get('act_x') not in (None, '', 'None'):
            pos_x.append(float(r['act_x']))

    if not motors:
        print(f"[{label}] No contact data found")
        return None

    mean_m = sum(motors) / len(motors)
    min_m = min(motors)
    max_m = max(motors)
    var_m = sum((m - mean_m) ** 2 for m in motors) / len(motors)
    std_m = math.sqrt(var_m)
    mean_x = sum(pos_x) / len(pos_x) if pos_x else 0.0

    print(f"\n{'='*50}")
    print(f"  {label}")
    print(f"{'='*50}")
    print(f"  Samples:      {len(motors)}")
    print(f"  Motor mean:   {mean_m:.1f} PWM")
    print(f"  Motor min:    {min_m:.1f} PWM")
    print(f"  Motor max:    {max_m:.1f} PWM")
    print(f"  Motor std:    {std_m:.1f} PWM")
    print(f"  Pos x mean:   {mean_x:.3f} m")
    print(f"{'='*50}")

    return {
        'label': label,
        'mean': mean_m,
        'min': min_m,
        'max': max_m,
        'std': std_m,
        'samples': len(motors),
        'pos_x_mean': mean_x,
    }


def find_latest(prefix):
    log_dir = os.path.join(os.path.expanduser("~"), "huaqiccc_logs")
    pattern = os.path.join(log_dir, f"{prefix}_*.csv")
    files = glob.glob(pattern)
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def main():
    parser = argparse.ArgumentParser(description='Analyze contact comparison results')
    parser.add_argument('--spring', help='Spring model CSV path')
    parser.add_argument('--hard', help='Hard-push CSV path')
    args = parser.parse_args()

    spring_path = args.spring or find_latest('spring_model')
    hard_path = args.hard or find_latest('hard_push')

    if not spring_path or not os.path.exists(spring_path):
        print(f"[ERROR] Spring model CSV not found: {spring_path}")
        sys.exit(1)
    if not hard_path or not os.path.exists(hard_path):
        print(f"[ERROR] Hard-push CSV not found: {hard_path}")
        sys.exit(1)

    print(f"[LOAD] Spring: {spring_path}")
    print(f"[LOAD] Hard:   {hard_path}")

    spring_rows = load_contact_phase(spring_path)
    hard_rows = load_contact_phase(hard_path)

    spring_stats = analyze(spring_rows, "SPRING MODEL (MPCA_PC_SPRING=1)")
    hard_stats = analyze(hard_rows, "HARD PUSH (MPCA_PC_SPRING=0)")

    if spring_stats and hard_stats:
        diff = hard_stats['mean'] - spring_stats['mean']
        pct = (diff / spring_stats['mean']) * 100.0 if spring_stats['mean'] > 0 else 0
        print(f"\n{'='*50}")
        print(f"  COMPARISON")
        print(f"{'='*50}")
        print(f"  Hard-push mean - Spring mean = {diff:.1f} PWM")
        print(f"  Relative increase = {pct:.1f}%")
        print(f"{'='*50}")

        if diff > 50:
            print("\n[RESULT] Significant thrust reduction confirmed!")
            print("         Spring model produces lower motor output during contact.")
        elif diff > 10:
            print("\n[RESULT] Moderate thrust reduction observed.")
        else:
            print("\n[RESULT] Minimal difference. Parameters may need tuning.")


if __name__ == '__main__':
    main()
