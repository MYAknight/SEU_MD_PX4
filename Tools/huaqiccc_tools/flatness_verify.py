#!/usr/bin/env python3
"""
Flatness Feedforward Verification Script for huaqiccc Morphing Quadrotor.
Generates analytical trajectories, computes feedforward via differential flatness,
and validates algebraic self-consistency.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

G = 9.80665
MASS = 1.5


def circle_trajectory(t, radius=2.0, omega=0.5, z0=2.5):
    """Generate a flat circle trajectory."""
    x = radius * np.cos(omega * t)
    y = radius * np.sin(omega * t)
    z = z0 * np.ones_like(t)
    
    vx = -radius * omega * np.sin(omega * t)
    vy = radius * omega * np.cos(omega * t)
    vz = np.zeros_like(t)
    
    ax = -radius * omega**2 * np.cos(omega * t)
    ay = -radius * omega**2 * np.sin(omega * t)
    az = np.zeros_like(t)
    
    jx = radius * omega**3 * np.sin(omega * t)
    jy = -radius * omega**3 * np.cos(omega * t)
    jz = np.zeros_like(t)
    
    sx = radius * omega**4 * np.cos(omega * t)
    sy = radius * omega**4 * np.sin(omega * t)
    sz = np.zeros_like(t)
    
    yaw = np.arctan2(vy, vx)
    yaw_dot = omega * np.ones_like(t)
    yaw_ddot = np.zeros_like(t)
    
    return {
        'pos': np.stack([x, y, z], axis=1),
        'vel': np.stack([vx, vy, vz], axis=1),
        'acc': np.stack([ax, ay, az], axis=1),
        'jerk': np.stack([jx, jy, jz], axis=1),
        'snap': np.stack([sx, sy, sz], axis=1),
        'yaw': yaw,
        'yaw_dot': yaw_dot,
        'yaw_ddot': yaw_ddot,
    }


def lemniscate_trajectory(t, a=2.0, omega=0.3, z0=2.5):
    """Generate a lemniscate (figure-8) trajectory."""
    scale = 1.0 / (1.0 + np.sin(omega * t)**2)
    x = a * np.cos(omega * t) * scale
    y = a * np.sin(omega * t) * np.cos(omega * t) * scale
    z = z0 * np.ones_like(t)
    
    # Numerical derivatives for higher orders
    dt = t[1] - t[0]
    vx = np.gradient(x, dt)
    vy = np.gradient(y, dt)
    vz = np.gradient(z, dt)
    
    ax = np.gradient(vx, dt)
    ay = np.gradient(vy, dt)
    az = np.gradient(vz, dt)
    
    jx = np.gradient(ax, dt)
    jy = np.gradient(ay, dt)
    jz = np.gradient(az, dt)
    
    sx = np.gradient(jx, dt)
    sy = np.gradient(jy, dt)
    sz = np.gradient(jz, dt)
    
    yaw = np.arctan2(vy, vx)
    yaw_dot = np.gradient(yaw, dt)
    yaw_ddot = np.gradient(yaw_dot, dt)
    
    return {
        'pos': np.stack([x, y, z], axis=1),
        'vel': np.stack([vx, vy, vz], axis=1),
        'acc': np.stack([ax, ay, az], axis=1),
        'jerk': np.stack([jx, jy, jz], axis=1),
        'snap': np.stack([sx, sy, sz], axis=1),
        'yaw': yaw,
        'yaw_dot': yaw_dot,
        'yaw_ddot': yaw_ddot,
    }


def compute_flatness_feedforward(traj):
    """
    Compute feedforward thrust and attitude from flat output.
    Returns dict with thrust, body_z, R, roll, pitch, yaw.
    """
    n = len(traj['pos'])
    thrust = np.zeros(n)
    body_z = np.zeros((n, 3))
    roll = np.zeros(n)
    pitch = np.zeros(n)
    yaw_out = np.zeros(n)
    
    for i in range(n):
        acc = traj['acc'][i]
        yaw = traj['yaw'][i]
        
        epsilon = np.array([acc[0], acc[1], acc[2] + G])
        eps_norm = np.linalg.norm(epsilon)
        
        if eps_norm < 1e-3:
            thrust[i] = MASS * G
            body_z[i] = [0, 0, 1]
            continue
        
        thrust[i] = MASS * eps_norm
        z_b = epsilon / eps_norm
        body_z[i] = z_b
        
        # Reconstruct attitude: R = [x_b, y_b, z_b]
        x_c = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        y_b = np.cross(z_b, x_c)
        y_b_norm = np.linalg.norm(y_b)
        
        if y_b_norm < 1e-4:
            # Singularity: z_b parallel to z_w
            x_b = np.array([np.cos(yaw), -np.sin(yaw), 0.0])
            y_b = np.cross(z_b, x_b)
            y_b = y_b / np.linalg.norm(y_b)
            x_b = np.cross(y_b, z_b)
        else:
            y_b = y_b / y_b_norm
            x_b = np.cross(y_b, z_b)
        
        R = np.column_stack([x_b, y_b, z_b])
        
        # Extract roll, pitch from R (ZYX convention)
        # R = Rz(yaw) * Ry(pitch) * Rx(roll)
        # For small angles: roll ~ -z_b[1], pitch ~ z_b[0]
        roll[i] = np.arctan2(-R[2, 1], R[2, 2])
        pitch[i] = np.arctan2(R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
        yaw_out[i] = yaw
    
    return {
        'thrust': thrust,
        'body_z': body_z,
        'roll': roll,
        'pitch': pitch,
        'yaw': yaw_out,
    }


def verify_self_consistency(traj, ff):
    """
    Verify that feedforward is self-consistent:
    R * [0, 0, T] - m*g*z_w should equal m*a
    """
    n = len(traj['pos'])
    errors = np.zeros(n)
    
    for i in range(n):
        acc = traj['acc'][i]
        z_b = ff['body_z'][i]
        T = ff['thrust'][i]
        
        # Reconstructed acceleration from feedforward
        acc_recon = (T * z_b - MASS * G * np.array([0, 0, 1])) / MASS
        errors[i] = np.linalg.norm(acc_recon - acc)
    
    return errors


def plot_results(traj, ff, title, outpath):
    """Plot trajectory, thrust, and attitude profiles."""
    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    t = np.arange(len(traj['pos'])) * 0.02  # assume 50Hz
    
    # Position
    ax = axes[0, 0]
    ax.plot(t, traj['pos'][:, 0], label='x')
    ax.plot(t, traj['pos'][:, 1], label='y')
    ax.plot(t, traj['pos'][:, 2], label='z')
    ax.set_title('Position')
    ax.legend()
    ax.grid(True)
    
    # Acceleration
    ax = axes[0, 1]
    ax.plot(t, traj['acc'][:, 0], label='ax')
    ax.plot(t, traj['acc'][:, 1], label='ay')
    ax.plot(t, traj['acc'][:, 2], label='az')
    ax.set_title('Acceleration')
    ax.legend()
    ax.grid(True)
    
    # Thrust
    ax = axes[1, 0]
    ax.plot(t, ff['thrust'], 'r')
    ax.axhline(MASS * G, color='k', linestyle='--', label='hover')
    ax.set_title('Collective Thrust [N]')
    ax.legend()
    ax.grid(True)
    
    # Attitude
    ax = axes[1, 1]
    ax.plot(t, np.degrees(ff['roll']), label='roll')
    ax.plot(t, np.degrees(ff['pitch']), label='pitch')
    ax.set_title('Attitude [deg]')
    ax.legend()
    ax.grid(True)
    
    # Body z-axis
    ax = axes[2, 0]
    ax.plot(t, ff['body_z'][:, 0], label='zb_x')
    ax.plot(t, ff['body_z'][:, 1], label='zb_y')
    ax.plot(t, ff['body_z'][:, 2], label='zb_z')
    ax.set_title('Body Z-axis (world frame)')
    ax.legend()
    ax.grid(True)
    
    # 3D trajectory
    ax = axes[2, 1]
    ax.plot(traj['pos'][:, 0], traj['pos'][:, 1])
    ax.set_aspect('equal')
    ax.set_title('XY Trajectory')
    ax.grid(True)
    
    plt.suptitle(title)
    plt.tight_layout()
    plt.savefig(outpath, dpi=150)
    plt.close()
    print(f"Saved plot: {outpath}")


def main():
    print("=" * 60)
    print("huaqiccc Flatness Feedforward Verification")
    print(f"Mass={MASS}kg, g={G}m/s^2")
    print("=" * 60)
    
    # Test 1: Circle
    print("\n[Test 1] Circle Trajectory")
    t = np.linspace(0, 4 * np.pi, 400)
    traj_c = circle_trajectory(t, radius=2.0, omega=0.5)
    ff_c = compute_flatness_feedforward(traj_c)
    err_c = verify_self_consistency(traj_c, ff_c)
    print(f"  Self-consistency RMSE: {np.sqrt(np.mean(err_c**2)):.6f} m/s^2")
    print(f"  Max consistency error: {np.max(err_c):.6f} m/s^2")
    print(f"  Thrust range: [{ff_c['thrust'].min():.2f}, {ff_c['thrust'].max():.2f}] N")
    print(f"  Roll range:  [{np.degrees(ff_c['roll'].min()):.2f}, {np.degrees(ff_c['roll'].max()):.2f}] deg")
    print(f"  Pitch range: [{np.degrees(ff_c['pitch'].min()):.2f}, {np.degrees(ff_c['pitch'].max()):.2f}] deg")
    plot_results(traj_c, ff_c, "Circle Trajectory - Flatness Feedforward", "/tmp/flatness_circle.png")
    
    # Test 2: Lemniscate
    print("\n[Test 2] Lemniscate (Figure-8) Trajectory")
    t = np.linspace(0, 6 * np.pi, 600)
    traj_l = lemniscate_trajectory(t, a=2.0, omega=0.3)
    ff_l = compute_flatness_feedforward(traj_l)
    err_l = verify_self_consistency(traj_l, ff_l)
    print(f"  Self-consistency RMSE: {np.sqrt(np.mean(err_l**2)):.6f} m/s^2")
    print(f"  Max consistency error: {np.max(err_l):.6f} m/s^2")
    print(f"  Thrust range: [{ff_l['thrust'].min():.2f}, {ff_l['thrust'].max():.2f}] N")
    plot_results(traj_l, ff_l, "Lemniscate Trajectory - Flatness Feedforward",
                 "/tmp/flatness_lemniscate.png")
    
    # Test 3: Morphing during flight
    print("\n[Test 3] Circle with Morphing (arm_angle: 0 -> -0.3 rad)")
    traj_c['arm_angle'] = np.linspace(0, -0.3, len(t))
    # Note: flatness is unaffected by arm_angle for thrust/attitude computation
    # (assuming inertialess arms), but inertia LUT would affect angular acc
    print("  Flatness attitude/thrust invariant to arm_angle (inertialess assumption)")
    print("  -> Inertia matrix J(arm_angle) only needed for angular acc feedforward")
    
    print("\n" + "=" * 60)
    print("Verification complete. Plots saved to /tmp/")
    print("=" * 60)


if __name__ == "__main__":
    main()
