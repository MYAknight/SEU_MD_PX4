#!/usr/bin/env python3
"""
Offline analysis of stall detection candidates using existing perching CSV logs.

Evaluates 4 candidate algorithms that do NOT rely on absolute position:
  A. Position Error Integration
  B. Velocity-Trajectory Consistency
  C. Projected Velocity Stagnation
  D. Motor Saturation + Stagnation

Usage:
  python3 analyze_stall_candidates.py ~/huaqiccc_logs/
"""

import argparse
import csv
import glob
import math
import os
import sys
from collections import defaultdict

# ---------------------------------------------------------------------------
# Candidate Algorithms
# ---------------------------------------------------------------------------

def algorithm_a_error_integration(rows, dt, eps_pos=0.10, eps_vel=0.08,
                                   I_thr=0.15, decay=0.95):
    """
    A. Position Error Integration.
    Integrate position error when |error| > eps_pos AND |vel| < eps_vel.
    Trigger when integral exceeds I_thr.
    """
    integral = 0.0
    trigger_time = None
    trigger_idx = None
    for i, r in enumerate(rows):
        if i == 0:
            continue
        sp = r['sp']
        act = r['act']
        vel = r['vel']
        err = abs(sp - act)
        if err > eps_pos and abs(vel) < eps_vel:
            integral += err * dt
        else:
            integral *= decay
        if trigger_time is None and integral > I_thr:
            trigger_time = r['time']
            trigger_idx = i
    return trigger_time, trigger_idx, integral


def algorithm_b_velocity_consistency(rows, dt, dv_thr=0.15, T=1.0):
    """
    B. Velocity-Trajectory Consistency.
    Use setpoint rate as expected velocity.
    Trigger when |v_actual - v_expected| > dv_thr for T seconds.
    """
    trigger_time = None
    trigger_idx = None
    consecutive = 0.0
    for i, r in enumerate(rows):
        if i == 0:
            continue
        v_exp = r['v_sp']  # setpoint velocity (computed from setpoint diff)
        v_act = r['vel']
        if abs(v_act - v_exp) > dv_thr:
            consecutive += dt
        else:
            consecutive = 0.0
        if trigger_time is None and consecutive >= T:
            trigger_time = r['time']
            trigger_idx = i
    return trigger_time, trigger_idx, consecutive


def algorithm_c_projected_stagnation(rows, dt, v_thr=0.05, eps=0.10, T=1.0):
    """
    C. Projected Velocity Stagnation.
    Compute velocity projected onto setpoint error direction.
    Trigger when v_proj < v_thr AND |error| > eps for T seconds.
    """
    trigger_time = None
    trigger_idx = None
    consecutive = 0.0
    for i, r in enumerate(rows):
        if i == 0:
            continue
        sp = r['sp']
        act = r['act']
        vel = r['vel']
        err = sp - act
        if abs(err) > eps:
            # 1D projection
            v_proj = vel if err > 0 else -vel
        else:
            v_proj = vel
        if v_proj < v_thr and abs(err) > eps:
            consecutive += dt
        else:
            consecutive = 0.0
        if trigger_time is None and consecutive >= T:
            trigger_time = r['time']
            trigger_idx = i
    return trigger_time, trigger_idx, consecutive


def algorithm_d_motor_saturation(rows, dt, motor_thr=1350.0, eps_vel=0.08, T=1.0):
    """
    D. Motor Saturation + Stagnation.
    Trigger when motor_avg > motor_thr AND |vel| < eps_vel for T seconds.
    """
    trigger_time = None
    trigger_idx = None
    consecutive = 0.0
    for i, r in enumerate(rows):
        if i == 0:
            continue
        motor = r.get('motor_avg', 0.0)
        vel = r['vel']
        if motor > motor_thr and abs(vel) < eps_vel:
            consecutive += dt
        else:
            consecutive = 0.0
        if trigger_time is None and consecutive >= T:
            trigger_time = r['time']
            trigger_idx = i
    return trigger_time, trigger_idx, consecutive


# ---------------------------------------------------------------------------
# Data Loading
# ---------------------------------------------------------------------------

def load_csv(path):
    """Load a perching CSV and return list of dicts with unified column names."""
    rows = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for r in reader:
            # Unify column names
            row = {
                'phase': r.get('phase', ''),
                'time': float(r.get('time', 0)),
                'sp_x': float(r.get('sp_x', r.get('sp_x', 0))),
                'sp_y': float(r.get('sp_y', r.get('sp_y', 0))),
                'sp_z': float(r.get('sp_z', r.get('sp_z', 0))),
                'act_x': float(r.get('act_x', r.get('pos_x', 0))),
                'act_y': float(r.get('act_y', r.get('pos_y', 0))),
                'act_z': float(r.get('act_z', r.get('pos_z', 0))),
                'motor_avg': float(r.get('motor_avg') or 0),
            }
            rows.append(row)
    # Compute velocities via numerical differentiation (central diff)
    for i in range(len(rows)):
        if i == 0:
            rows[i]['vel_x'] = 0.0
            rows[i]['vel_y'] = 0.0
            rows[i]['v_sp_x'] = 0.0
            rows[i]['v_sp_y'] = 0.0
        else:
            dt = rows[i]['time'] - rows[i-1]['time']
            if dt < 1e-6:
                dt = 0.05
            rows[i]['vel_x'] = (rows[i]['act_x'] - rows[i-1]['act_x']) / dt
            rows[i]['vel_y'] = (rows[i]['act_y'] - rows[i-1]['act_y']) / dt
            rows[i]['v_sp_x'] = (rows[i]['sp_x'] - rows[i-1]['sp_x']) / dt
            rows[i]['v_sp_y'] = (rows[i]['sp_y'] - rows[i-1]['sp_y']) / dt
    return rows


def extract_push_phase(rows):
    """Extract push-phase rows and compute 1D forward-motion vectors."""
    push = [r for r in rows if r['phase'] == 'push']
    if not push:
        return []
    # Determine primary axis: find which axis has largest setpoint change
    dx = push[-1]['sp_x'] - push[0]['sp_x']
    dy = push[-1]['sp_y'] - push[0]['sp_y']
    if abs(dx) >= abs(dy):
        axis = 'x'
    else:
        axis = 'y'
    out = []
    for r in push:
        out.append({
            'time': r['time'],
            'sp': r[f'sp_{axis}'],
            'act': r[f'act_{axis}'],
            'vel': r[f'vel_{axis}'],
            'v_sp': r[f'v_sp_{axis}'],
            'motor_avg': r['motor_avg'],
        })
    return out


def estimate_contact_time(push_rows):
    """Estimate actual contact time from position stagnation."""
    # Contact happens when position stops advancing toward setpoint
    for i in range(2, len(push_rows)):
        # Look for sustained position stagnation while setpoint keeps moving
        r = push_rows[i]
        if r['v_sp'] > 0.01 and r['vel'] < 0.02:
            # Sustained?
            if i >= 5:
                # Check previous 5 samples
                stalled = all(push_rows[j]['vel'] < 0.05 for j in range(i-4, i+1))
                if stalled:
                    return r['time'], i
    return None, None


# ---------------------------------------------------------------------------
# Evaluation
# ---------------------------------------------------------------------------

def evaluate_on_file(path, algorithms):
    rows = load_csv(path)
    push = extract_push_phase(rows)
    if not push:
        return None
    dt = push[1]['time'] - push[0]['time'] if len(push) > 1 else 0.05
    contact_t, contact_idx = estimate_contact_time(push)
    results = {
        'file': os.path.basename(path),
        'has_contact': 'contact' in {r['phase'] for r in rows},
        'push_duration': push[-1]['time'] - push[0]['time'] if push else 0,
        'contact_time': contact_t,
        'contact_idx': contact_idx,
    }
    for name, func, kwargs in algorithms:
        t_trigger, idx_trigger, _ = func(push, dt, **kwargs)
        delay = (t_trigger - contact_t) if (t_trigger and contact_t) else None
        # False positive = trigger before contact (or when no contact)
        fp = False
        if t_trigger and contact_t and t_trigger < contact_t - 0.3:
            fp = True
        if not results['has_contact'] and t_trigger is not None:
            fp = True
        results[name] = {
            'trigger_time': t_trigger,
            'trigger_idx': idx_trigger,
            'delay': delay,
            'false_positive': fp,
        }
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('logdir', nargs='?', default=os.path.expanduser('~/huaqiccc_logs'))
    args = parser.parse_args()

    files = sorted(glob.glob(os.path.join(args.logdir, '*.csv')))
    # Select perching-related files (exclude flatness/flight tests)
    files = [f for f in files if any(k in os.path.basename(f).lower()
             for k in ['contact', 'no_contact', 'grasp', 'push', 'spring', 'hard'])]

    print(f"Analyzing {len(files)} CSV files from {args.logdir}\n")

    algorithms = [
        ('A_err_int', algorithm_a_error_integration, {'eps_pos': 0.10, 'eps_vel': 0.08, 'I_thr': 0.15, 'decay': 0.95}),
        ('B_vel_cons', algorithm_b_velocity_consistency, {'dv_thr': 0.15, 'T': 1.0}),
        ('C_proj_stag', algorithm_c_projected_stagnation, {'v_thr': 0.05, 'eps': 0.10, 'T': 1.0}),
        ('D_motor', algorithm_d_motor_saturation, {'motor_thr': 1350.0, 'eps_vel': 0.08, 'T': 1.0}),
    ]

    all_results = []
    for f in files:
        res = evaluate_on_file(f, algorithms)
        if res:
            all_results.append(res)

    # Summarize
    print("=" * 90)
    print(f"{'File':<45} {'Contact?':<8} {'A_delay':<10} {'B_delay':<10} {'C_delay':<10} {'D_delay':<10}")
    print("=" * 90)
    for r in all_results:
        delays = []
        for name in ['A_err_int', 'B_vel_cons', 'C_proj_stag', 'D_motor']:
            d = r[name]['delay']
            delays.append(f"{d:.2f}s" if d is not None else "NO_TRIG")
        print(f"{r['file']:<45} {str(r['has_contact']):<8} {delays[0]:<10} {delays[1]:<10} {delays[2]:<10} {delays[3]:<10}")

    # False positive summary
    print("\n" + "=" * 90)
    print("FALSE POSITIVE SUMMARY")
    print("=" * 90)
    for name, _, _ in algorithms:
        fp_count = sum(1 for r in all_results if r[name]['false_positive'])
        total = len(all_results)
        print(f"  {name}: {fp_count}/{total} files had false positive ({fp_count/total*100:.1f}%)")

    # Detection rate on contact files
    contact_files = [r for r in all_results if r['has_contact']]
    print("\n" + "=" * 90)
    print("DETECTION RATE ON CONTACT FILES")
    print("=" * 90)
    for name, _, _ in algorithms:
        detected = sum(1 for r in contact_files if r[name]['trigger_time'] is not None)
        total = len(contact_files)
        delays = [r[name]['delay'] for r in contact_files if r[name]['delay'] is not None]
        mean_delay = sum(delays) / len(delays) if delays else float('nan')
        print(f"  {name}: {detected}/{total} detected ({detected/total*100:.1f}%), mean delay={mean_delay:.2f}s")

    print("\nDone.")


if __name__ == '__main__':
    main()
