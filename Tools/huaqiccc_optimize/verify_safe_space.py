#!/usr/bin/env python3
"""
verify_safe_space.py — 验证安全空间与轨迹是否满足安全裕量

用法：
    python3 verify_safe_space.py [--no-plot]

输出：
    - 安全多边形几何信息
    - home 到各边界距离
    - 每条轨迹的最小边界距离
    - 可视化图片（保存为 safe_space_verify.png，若 matplotlib 可用）
"""

import os
import sys
import math
import argparse
from typing import List, Tuple

import yaml
import numpy as np

# 从 flight_executor.py 导入 SafePolygon（不启动 ROS）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from flight_executor import SafePolygon


def point_to_segment_distance(px: float, py: float, x1: float, y1: float,
                              x2: float, y2: float) -> float:
    ex, ey = x2 - x1, y2 - y1
    len2 = ex * ex + ey * ey
    if len2 < 1e-12:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * ex + (py - y1) * ey) / len2))
    cx = x1 + t * ex
    cy = y1 + t * ey
    return math.hypot(px - cx, py - cy)


def compute_max_radius(safe: SafePolygon, home_xy: Tuple[float, float]) -> float:
    min_dist = float("inf")
    for i in range(safe.n):
        x1, y1 = safe.corners[i]
        x2, y2 = safe.corners[(i + 1) % safe.n]
        d = point_to_segment_distance(home_xy[0], home_xy[1], x1, y1, x2, y2)
        min_dist = min(min_dist, d)
    return min_dist


def generate_trajectory(name: str, home_xy: Tuple[float, float], ground_z: float,
                        flight_height: float, cruise_speed: float,
                        max_radius: float, setpoint_rate: float = 20.0,
                        hover_time: float = 10.0) -> List[Tuple[float, float, float]]:
    """简化版轨迹生成器（与 flight_executor.py 逻辑一致）。"""
    cx, cy = home_xy
    base_z = ground_z + flight_height

    R = min(0.30, max_radius)
    square_half = min(0.20, max_radius / math.sqrt(2))
    step_delta = min(0.22, max_radius / math.sqrt(2))

    def interp(p0, p1, speed: float) -> List[Tuple[float, float, float]]:
        dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(p0, p1)))
        if dist < 1e-6:
            return [p1]
        n = max(1, int(dist / (speed / setpoint_rate)))
        return [tuple(p0[i] + (p1[i] - p0[i]) * t / n for i in range(3)) for t in range(1, n + 1)]

    if name == "hover":
        return [(cx, cy, base_z)]

    elif name == "takeoff_land":
        p0 = (cx, cy, ground_z + 0.3)
        p1 = (cx, cy, base_z)
        return interp(p0, p1, 0.2) + [p1] * int(hover_time * setpoint_rate) + interp(p1, p0, 0.2)

    elif name == "square_small":
        corners = [
            (cx + square_half, cy + square_half, base_z),
            (cx + square_half, cy - square_half, base_z),
            (cx - square_half, cy - square_half, base_z),
            (cx - square_half, cy + square_half, base_z),
            (cx + square_half, cy + square_half, base_z),
        ]
        pts = []
        for i in range(len(corners) - 1):
            pts.extend(interp(corners[i], corners[i + 1], cruise_speed))
        return pts

    elif name == "circle_small":
        n = 100
        return [(cx + R * math.cos(2 * math.pi * i / n),
                 cy + R * math.sin(2 * math.pi * i / n),
                 base_z) for i in range(n + 1)]

    elif name == "figure8_small":
        n = 160
        return [(cx + R * math.sin(2 * math.pi * i / n),
                 cy + R * math.sin(2 * math.pi * i / n) * math.cos(2 * math.pi * i / n),
                 base_z) for i in range(n + 1)]

    elif name == "step_x":
        p0 = (cx, cy, base_z)
        p1 = (cx + step_delta, cy, base_z)
        p2 = (cx - step_delta, cy, base_z)
        hold = int(3.0 * setpoint_rate)
        return ([p0] * hold + interp(p0, p1, cruise_speed) + [p1] * hold +
                interp(p1, p2, cruise_speed) + [p2] * hold +
                interp(p2, p0, cruise_speed) + [p0] * hold)

    elif name == "step_xy":
        p0 = (cx, cy, base_z)
        p1 = (cx + step_delta, cy + step_delta, base_z)
        p2 = (cx - step_delta, cy - step_delta, base_z)
        hold = int(3.0 * setpoint_rate)
        return ([p0] * hold + interp(p0, p1, cruise_speed) + [p1] * hold +
                interp(p1, p2, cruise_speed) + [p2] * hold +
                interp(p2, p0, cruise_speed) + [p0] * hold)

    elif name == "morph_circle":
        return generate_trajectory("circle_small", home_xy, ground_z, flight_height,
                                   cruise_speed, max_radius, setpoint_rate, hover_time)

    else:
        raise ValueError(f"未知轨迹: {name}")


def main():
    parser = argparse.ArgumentParser(description="验证安全空间与轨迹")
    parser.add_argument("--no-plot", action="store_true", help="不生成可视化图片")
    args = parser.parse_args()

    cfg_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "safe_space.yaml")
    with open(cfg_path, "r") as f:
        cfg = yaml.safe_load(f)

    ss = cfg["safe_space"]
    ms = cfg["mission"]

    safe = SafePolygon(corners=ss["corners"], z_min=ss["z_min"], z_max=ss["z_max"])
    home_xy = (float(ss["home"][0]), float(ss["home"][1]))
    ground_z = float(ss["home"][2])
    flight_height = float(ms["flight_height"])
    cruise_speed = float(ms["cruise_speed"])
    safety_margin = float(ms["safety_margin"])

    print("=" * 60)
    print("安全空间验证报告")
    print("=" * 60)
    print(f"\n多边形角点（按顺序连接）:")
    for i, (x, y) in enumerate(safe.corners):
        print(f"  点{i}: ({x:.3f}, {y:.3f})")

    print(f"\nhome 点: ({home_xy[0]:.3f}, {home_xy[1]:.3f})")

    # home 到各边距离
    print(f"\nhome 到各边距离:")
    min_dist = float("inf")
    for i in range(safe.n):
        x1, y1 = safe.corners[i]
        x2, y2 = safe.corners[(i + 1) % safe.n]
        d = point_to_segment_distance(home_xy[0], home_xy[1], x1, y1, x2, y2)
        min_dist = min(min_dist, d)
        print(f"  边{i}-{(i+1)%safe.n}: {d:.3f} m")

    max_radius = min_dist - safety_margin
    print(f"\nhome 到边界最短距离: {min_dist:.3f} m")
    print(f"安全裕量: {safety_margin:.3f} m")
    print(f"轨迹可用最大半径: {max_radius:.3f} m")

    if max_radius < 0.15:
        print("\n[WARNING] 可用半径过小，请检查 home 点是否在安全区域中心！")

    trajectories = [
        "hover", "takeoff_land", "square_small", "circle_small",
        "figure8_small", "step_x", "step_xy", "morph_circle"
    ]

    print(f"\n各轨迹最小边界距离（要求 >= {safety_margin:.3f} m）:")
    print("-" * 60)
    all_safe = True
    traj_data = {}
    for name in trajectories:
        pts = generate_trajectory(name, home_xy, ground_z, flight_height,
                                  cruise_speed, max_radius)
        # 裁剪到安全区（与 flight_executor.py 一致）
        clipped = []
        for x, y, z in pts:
            sx, sy = safe.clip_point(x, y, home_xy, margin=safety_margin)
            sz = max(ground_z, min(ground_z + flight_height, z))
            clipped.append((sx, sy, sz))

        min_d = min(safe.distance_to_boundary(x, y) for x, y, _ in clipped)
        status = "OK" if min_d >= safety_margin - 1e-6 else "FAIL"
        if status == "FAIL":
            all_safe = False
        print(f"  {name:<18} 航点数={len(clipped):>5}  最小边距={min_d:.3f} m  [{status}]")
        traj_data[name] = clipped

    print("-" * 60)
    if all_safe:
        print("[PASS] 所有轨迹均满足安全裕量要求。")
    else:
        print("[FAIL] 存在轨迹不满足安全裕量，请减小轨迹尺寸或增大安全区域。")

    # 可视化
    if not args.no_plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            fig, ax = plt.subplots(figsize=(10, 10))

            # 绘制多边形
            xs = [c[0] for c in safe.corners] + [safe.corners[0][0]]
            ys = [c[1] for c in safe.corners] + [safe.corners[0][1]]
            ax.plot(xs, ys, "k-", linewidth=2, label="安全边界")

            # 绘制 home
            ax.plot(home_xy[0], home_xy[1], "r*", markersize=15, label="home (起飞/降落中心)")

            # 绘制内缩边界（margin）
            # 简化：用 home 为圆心、max_radius 为半径画圆示意
            theta = np.linspace(0, 2 * np.pi, 100)
            ax.plot(home_xy[0] + max_radius * np.cos(theta),
                    home_xy[1] + max_radius * np.sin(theta),
                    "r--", linewidth=1, label=f"可用半径 {max_radius:.2f}m")

            colors = plt.cm.tab10(np.linspace(0, 1, len(trajectories)))
            for name, color in zip(trajectories, colors):
                pts = traj_data[name]
                ax.plot([p[0] for p in pts], [p[1] for p in pts],
                        color=color, linewidth=1, alpha=0.7, label=name)

            ax.set_aspect("equal", adjustable="box")
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_title("Safe Space & Trajectories")
            ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0))
            ax.grid(True, alpha=0.3)

            out_path = os.path.join(os.path.dirname(cfg_path), "safe_space_verify.png")
            fig.tight_layout()
            fig.savefig(out_path, dpi=150)
            print(f"\n可视化已保存: {out_path}")
        except ImportError:
            print("\n[INFO] 未安装 matplotlib，跳过可视化。可运行 pip3 install matplotlib 后重试。")


if __name__ == "__main__":
    main()
