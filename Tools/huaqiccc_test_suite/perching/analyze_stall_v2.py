#!/usr/bin/env python3
"""
Re-analyze stall candidates using CORRECT axis (X-axis, where flight happens).

Key finding: current PX4 Stall Detection checks Y-axis (position(1)), but
pole_collision.py flies along X-axis. This means current Stall Detection
has NEVER triggered in SITL — all contacts come from IMU-ICD only.
"""

import csv
import glob
import os

LOGDIR = os.path.expanduser('~/huaqiccc_logs')

def load_push_x(filepath):
    """Load push-phase X-axis trajectory from a CSV."""
    rows = []
    with open(filepath) as f:
        reader = csv.DictReader(f)
        for r in reader:
            if r.get('phase') != 'push':
                continue
            act_key = 'act_x' if 'act_x' in r else 'pos_x'
            rows.append({
                'time': float(r['time']),
                'sp': float(r['sp_x']),
                'act': float(r[act_key]),
                'motor': float(r.get('motor_avg') or 0),
            })
    # Smooth velocity with 5-sample moving average
    for i in range(len(rows)):
        if i == 0:
            rows[i]['vel'] = 0.0
        else:
            dt = rows[i]['time'] - rows[i-1]['time']
            if dt < 1e-6:
                dt = 0.05
            raw = (rows[i]['act'] - rows[i-1]['act']) / dt
            # moving average
            window = [rows[j]['act'] for j in range(max(0,i-2), min(len(rows),i+3))]
            t_window = [rows[j]['time'] for j in range(max(0,i-2), min(len(rows),i+3))]
            if len(t_window) > 1:
                dt_win = t_window[-1] - t_window[0]
                if dt_win > 1e-6:
                    rows[i]['vel'] = (window[-1] - window[0]) / dt_win
                else:
                    rows[i]['vel'] = raw
            else:
                rows[i]['vel'] = raw
    return rows

def detect_contact_manual(rows):
    """Manually estimate contact time: when position stops advancing."""
    for i in range(10, len(rows)):
        # Look for 5 consecutive samples with low velocity while setpoint keeps moving
        stalled = all(abs(rows[j]['vel']) < 0.10 for j in range(i-4, i+1))
        sp_moving = rows[i]['sp'] - rows[i-5]['sp'] > 0.05
        if stalled and sp_moving:
            return rows[i]['time'], i
    return None, None

def algo_error_int(rows, dt, eps_pos=0.05, eps_vel=0.10, I_thr=0.10, decay=0.90):
    """Position error integration."""
    integral = 0.0
    for i, r in enumerate(rows):
        if i == 0:
            continue
        err = r['sp'] - r['act']
        if err > eps_pos and abs(r['vel']) < eps_vel:
            integral += err * dt
        else:
            integral *= decay
        if integral > I_thr:
            return r['time'], i, integral
    return None, None, integral

def algo_proj_stag(rows, dt, v_thr=0.08, eps=0.05, T=0.8):
    """Projected velocity stagnation."""
    consec = 0.0
    for i, r in enumerate(rows):
        if i == 0:
            continue
        err = r['sp'] - r['act']
        if r['vel'] < v_thr and err > eps:
            consec += dt
        else:
            consec = 0.0
        if consec >= T:
            return r['time'], i, consec
    return None, None, consec

def algo_motor_stag(rows, dt, motor_thr=1450, eps_vel=0.10, T=0.8):
    """Motor saturation + stagnation."""
    consec = 0.0
    for i, r in enumerate(rows):
        if i == 0:
            continue
        if r['motor'] > motor_thr and abs(r['vel']) < eps_vel:
            consec += dt
        else:
            consec = 0.0
        if consec >= T:
            return r['time'], i, consec
    return None, None, consec

def analyze_file(filepath):
    basename = os.path.basename(filepath)
    rows = load_push_x(filepath)
    if not rows:
        return None
    # Filter: only files where drone actually reached x > 3.0
    if max(r['act'] for r in rows) < 3.0:
        return None
    dt = rows[1]['time'] - rows[0]['time'] if len(rows) > 1 else 0.05
    has_contact = False
    with open(filepath) as f:
        reader = csv.DictReader(f)
        for r in reader:
            if r.get('phase') == 'contact':
                has_contact = True
                break
    contact_t, contact_idx = detect_contact_manual(rows)
    res = {
        'file': basename,
        'has_contact': has_contact,
        'contact_t': contact_t,
        'push_len': len(rows),
    }
    for name, func, kwargs in [
        ('A_err_int', algo_error_int, {'eps_pos': 0.05, 'eps_vel': 0.10, 'I_thr': 0.10}),
        ('C_proj_stag', algo_proj_stag, {'v_thr': 0.08, 'eps': 0.05, 'T': 0.8}),
        ('D_motor', algo_motor_stag, {'motor_thr': 1450, 'eps_vel': 0.10, 'T': 0.8}),
    ]:
        t, idx, _ = func(rows, dt, **kwargs)
        delay = (t - contact_t) if (t and contact_t) else None
        fp = (t and contact_t and t < contact_t - 0.3) or (not has_contact and t is not None)
        res[name] = {'t': t, 'idx': idx, 'delay': delay, 'fp': fp}
    return res

def main():
    files = sorted(glob.glob(os.path.join(LOGDIR, '*.csv')))
    # Only analyze perching-related files
    files = [f for f in files if any(k in os.path.basename(f).lower()
             for k in ['contact', 'no_contact', 'grasp', 'push', 'spring', 'hard',
                       'baseline', 'relaxed', 'pole_pass', 'perching_test'])]
    results = []
    for f in files:
        r = analyze_file(f)
        if r:
            results.append(r)

    contact_files = [r for r in results if r['has_contact']]
    no_contact_files = [r for r in results if not r['has_contact']]

    print(f"Valid files: {len(results)} total ({len(contact_files)} contact, {len(no_contact_files)} no-contact)")
    print("\n" + "="*100)
    print(f"{'File':<50} {'Contact?':<8} {'A_delay':<10} {'C_delay':<10} {'D_delay':<10}")
    print("="*100)
    for r in results[:30]:
        ds = []
        for n in ['A_err_int', 'C_proj_stag', 'D_motor']:
            d = r[n]['delay']
            ds.append(f"{d:.2f}s" if d is not None else "NO")
        print(f"{r['file']:<50} {str(r['has_contact']):<8} {ds[0]:<10} {ds[1]:<10} {ds[2]:<10}")
    if len(results) > 30:
        print(f"... and {len(results)-30} more")

    print("\n" + "="*60)
    print("DETECTION RATE ON CONTACT FILES")
    print("="*60)
    for name in ['A_err_int', 'C_proj_stag', 'D_motor']:
        det = sum(1 for r in contact_files if r[name]['t'] is not None)
        delays = [r[name]['delay'] for r in contact_files if r[name]['delay'] is not None]
        mean_d = sum(delays)/len(delays) if delays else float('nan')
        print(f"  {name}: {det}/{len(contact_files)} detected, mean delay={mean_d:.2f}s")

    print("\n" + "="*60)
    print("FALSE POSITIVE RATE ON NO-CONTACT FILES")
    print("="*60)
    for name in ['A_err_int', 'C_proj_stag', 'D_motor']:
        fp = sum(1 for r in no_contact_files if r[name]['fp'])
        print(f"  {name}: {fp}/{len(no_contact_files)} FP ({fp/len(no_contact_files)*100:.1f}%)")

if __name__ == '__main__':
    main()
