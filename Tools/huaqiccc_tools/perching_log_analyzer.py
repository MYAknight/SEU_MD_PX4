#!/usr/bin/env python3
"""
analyze_perching_log.py
=======================
Quick analysis of perching test CSV to verify end-to-end GMO pipeline.

Usage:
    python3 analyze_perching_log.py ~/huaqiccc_logs/perching_test_*.csv
"""

import csv
import sys
import glob
import math


def analyze(csv_path):
    print(f"\nAnalyzing: {csv_path}\n")

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        print("[ERROR] Empty CSV")
        return

    # Phase counts
    phases = {}
    for r in rows:
        p = r.get('phase', 'unknown')
        phases[p] = phases.get(p, 0) + 1

    print("Phase summary:")
    for p, c in sorted(phases.items()):
        dur = c * 0.05  # 20 Hz
        print(f"  {p:15s}: {c:4d} samples (~{dur:5.1f}s)")

    # Contact detection
    has_contact = 'contact_hold' in phases
    print(f"\nContact detected: {'YES' if has_contact else 'NO'}")

    if has_contact:
        # Find first contact_hold row
        contact_rows = [r for r in rows if r['phase'] == 'contact_hold']
        first = contact_rows[0]
        last = contact_rows[-1]
        print(f"  Contact hold duration: {float(last['time']) - float(first['time']):.2f}s")
        print(f"  Position at contact: x={first['act_x']}, y={first['act_y']}, z={first['act_z']}")
        print(f"  Setpoint at contact: x={first['sp_x']}, y={first['sp_y']}, z={first['sp_z']}")

    # Push phase analysis
    push_rows = [r for r in rows if r['phase'] == 'push']
    if push_rows:
        # Min distance to pole (5,0)
        min_dist = min(
            math.sqrt((float(r['act_x']) - 5.0)**2 + (float(r['act_y']) - 0.0)**2)
            for r in push_rows
        )
        print(f"\nPush phase:")
        print(f"  Samples: {len(push_rows)}")
        print(f"  Min distance to pole: {min_dist:.3f}m")

        # Check if position stalled
        start_x = float(push_rows[0]['act_x'])
        end_x = float(push_rows[-1]['act_x'])
        print(f"  Push start x: {start_x:.3f}m, end x: {end_x:.3f}m")

    # EFO / contact_state availability
    efo_nonzero = sum(1 for r in rows if float(r.get('efo_mag', 0)) > 0.01)
    cs_nonzero = sum(1 for r in rows if int(r.get('contact_state', -1)) >= 0)
    print(f"\nGMO ROS topic availability:")
    print(f"  external_force_estimate non-zero: {efo_nonzero}/{len(rows)}")
    print(f"  contact_state valid (>=0): {cs_nonzero}/{len(rows)}")
    if efo_nonzero == 0:
        print("  [WARN] No EFO data - px4_msgs ROS bridge may not be running")
    if cs_nonzero == 0:
        print("  [WARN] No contact_state data - px4_msgs ROS bridge may not be running")

    print("\n" + "=" * 50)
    print("IMPORTANT: Check terminal output for:")
    print("  'Perching: contact stable, grasping triggered'")
    print("  'Perching: contact lost, releasing'")
    print("If these appear AFTER the [PUSH] phase starts, the GMO→mc_pos_control")
    print("pipeline is working end-to-end.")
    print("=" * 50 + "\n")


def main():
    if len(sys.argv) > 1:
        paths = sys.argv[1:]
    else:
        paths = glob.glob('/home/a/huaqiccc_logs/perching_test_*.csv')
        if not paths:
            print("No perching_test CSV found in ~/huaqiccc_logs/")
            sys.exit(1)
        paths.sort()
        paths = [paths[-1]]  # Latest

    for p in paths:
        analyze(p)


if __name__ == '__main__':
    main()
