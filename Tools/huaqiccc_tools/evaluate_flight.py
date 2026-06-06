#!/usr/bin/env python3
"""
Huaqiccc Flight Test Evaluation Script
Computes standardized metrics from CSV logs.
"""
import sys
import csv
import math

def evaluate(csv_path):
    rows = list(csv.DictReader(open(csv_path)))
    if not rows:
        print("EMPTY CSV")
        return

    # Time ranges
    hover = [r for r in rows if 20.0 <= float(r.get('time', 0)) <= 45.0]
    morph = [r for r in rows if 5.0 <= float(r.get('time', 0)) <= 15.0]
    landing = [r for r in rows if float(r.get('time', 0)) >= 48.0]
    all_rows = rows

    def stats(name, data, key):
        vals = [abs(float(r[key])) for r in data if r.get(key)]
        if not vals:
            return None
        mean = sum(vals) / len(vals)
        maxv = max(vals)
        rmse = math.sqrt(sum(v*v for v in vals) / len(vals))
        return {"name": name, "n": len(vals), "mean": mean, "max": maxv, "rmse": rmse}

    results = {}
    for phase, data in [("hover", hover), ("morph", morph), ("landing", landing), ("all", all_rows)]:
        for axis in ['err_x', 'err_y', 'err_z']:
            key = f"{phase}_{axis}"
            results[key] = stats(phase, data, axis)

    # Overall score: lower error = higher score
    hover_xy_mean = 0
    hover_z_mean = 0
    if results.get('hover_err_x') and results.get('hover_err_y') and results.get('hover_err_z'):
        hover_xy_mean = (results['hover_err_x']['mean'] + results['hover_err_y']['mean']) / 2.0
        hover_z_mean = results['hover_err_z']['mean']

    # Composite score: reward low XY and low Z error
    score = 1.0 / (hover_xy_mean + hover_z_mean * 0.5 + 0.005)

    print(f"=== {csv_path} ===")
    print(f"Total rows: {len(rows)}")
    print(f"HOVER (t=20-45s):")
    for axis in ['err_x', 'err_y', 'err_z']:
        s = results.get(f'hover_{axis}')
        if s:
            print(f"  {axis}: mean={s['mean']:.4f}m  max={s['max']:.4f}m  rmse={s['rmse']:.4f}m")
    print(f"MORPH (t=5-15s):")
    for axis in ['err_x', 'err_y', 'err_z']:
        s = results.get(f'morph_{axis}')
        if s:
            print(f"  {axis}: mean={s['mean']:.4f}m  max={s['max']:.4f}m")
    print(f"Composite Score: {score:.2f}  (hover_xy={hover_xy_mean:.4f}, hover_z={hover_z_mean:.4f})")
    print()

if __name__ == '__main__':
    for path in sys.argv[1:]:
        evaluate(path)
