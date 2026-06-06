#!/usr/bin/env python3
"""
Long-contact motor comparison test for perching impedance control.

Protocol:
  1. Takeoff to HOVER_Z
  2. Expand arms
  3. Approach to APPROACH_X
  4. Slow push until stall detected
  5. Hold contact for RECORD_DURATION seconds
  6. Record motor PWMs (SERVO_OUTPUT_RAW) and position throughout
  7. Land

This script uses pymavlink directly (no ROS dependency) to maximize reliability.
"""

import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime

try:
    from pymavlink import mavutil
except ImportError:
    print("[FATAL] pymavlink not installed. Run: pip install pymavlink")
    sys.exit(1)


class LongContactTest:
    # ---------- Config ----------
    SITL_UDP = "udp:127.0.0.1:14540"
    HOVER_Z = 2.5
    POLE_X = 5.0
    POLE_RADIUS = 0.09
    APPROACH_X = 4.75
    PUSH_SPEED = 0.05
    TAKEOFF_HOVER_TIME = 8.0
    APPROACH_TIME = 12.0
    EXPAND_ANGLE = -0.45
    RECORD_DURATION = 25.0   # seconds of contact recording
    STALL_DIST = 0.15
    STALL_S = 0.70
    DT = 0.05               # 20 Hz loop

    def __init__(self, output_prefix='contact_test'):
        self.output_prefix = output_prefix
        self.records = []
        self.master = None

    def _connect(self):
        print(f"[CONN] Connecting to {self.SITL_UDP}...")
        self.master = mavutil.mavlink_connection(self.SITL_UDP)
        self.master.wait_heartbeat()
        print(f"[CONN] System {self.master.target_system} ready")

    def _set_param(self, param_id, value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
        """Set PX4 parameter via MAVLink."""
        print(f"[PARAM] Setting {param_id}={value}")
        self.master.mav.param_set_send(
            self.master.target_system, self.master.target_component,
            param_id.encode(), float(value), param_type)
        # Wait for ACK
        t0 = time.time()
        while time.time() - t0 < 3.0:
            msg = self.master.recv_match(type='PARAM_VALUE', blocking=False)
            if msg and msg.param_id.rstrip('\x00') == param_id:
                print(f"[PARAM] {param_id} confirmed = {msg.param_value}")
                return True
            time.sleep(0.05)
        print(f"[WARN] {param_id} set timeout")
        return False

    def _send_pos(self, x, y, z, yaw=0.0):
        type_mask = 0x0FF8
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask, x, y, z, 0, 0, 0, 0, 0, 0, yaw, 0.0)

    def _send_heartbeat(self):
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def _arm(self):
        print("[ARM] Arming...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(2)

    def _set_mode_offboard(self):
        print("[MODE] OFFBOARD")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            1, 6, 0, 0, 0, 0, 0)
        time.sleep(1)

    def _expand_arms(self):
        print(f"[MORPH] Expanding arms to {self.EXPAND_ANGLE} rad")
        # Send custom command 31440 for morphing angle
        for _ in range(5):
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                31440, 0,
                float(self.EXPAND_ANGLE), 0, 0, 0, 0, 0, 0)
            time.sleep(0.2)
        time.sleep(2)

    def _get_pos(self):
        msg = self.master.messages.get('LOCAL_POSITION_NED')
        if msg:
            return msg.x, msg.y, msg.z
        return None

    def _get_motors(self):
        msg = self.master.messages.get('SERVO_OUTPUT_RAW')
        if msg:
            return [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw]
        return None

    def _record(self, phase, t, sp_x, sp_y, sp_z, pos, motors):
        ax, ay, az = 0.0, 0.0, 0.0
        imu = self.master.recv_match(type='HIGHRES_IMU', blocking=False)
        if imu:
            ax, ay, az = imu.xacc, imu.yacc, imu.zacc

        row = {
            'phase': phase,
            'time': round(t, 3),
            'sp_x': round(sp_x, 4),
            'sp_y': round(sp_y, 4),
            'sp_z': round(sp_z, 4),
            'pos_x': round(pos[0], 5) if pos else None,
            'pos_y': round(pos[1], 5) if pos else None,
            'pos_z': round(pos[2], 5) if pos else None,
            'm0': motors[0] if motors else None,
            'm1': motors[1] if motors else None,
            'm2': motors[2] if motors else None,
            'm3': motors[3] if motors else None,
            'motor_avg': round(sum(motors)/4.0, 2) if motors else None,
            'ax': round(ax, 3), 'ay': round(ay, 3), 'az': round(az, 3),
        }
        self.records.append(row)

    def _save_csv(self, suffix=''):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_dir = os.path.join(os.path.expanduser("~"), "huaqiccc_logs")
        os.makedirs(out_dir, exist_ok=True)
        name = f"{self.output_prefix}{suffix}_{ts}.csv"
        out_path = os.path.join(out_dir, name)
        if not self.records:
            print("[WARN] No records to save")
            return None
        with open(out_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.records[0].keys())
            writer.writeheader()
            writer.writerows(self.records)
        print(f"[SAVE] {out_path}")
        return out_path

    def run(self):
        self._connect()

        # Drain old messages
        for _ in range(200):
            self.master.recv_match(blocking=False)

        # Wait for initial position
        print("[WAIT] Waiting for position estimate...")
        while True:
            pos = self._get_pos()
            if pos:
                print(f"  pos: x={pos[0]:.2f} y={pos[1]:.2f} z={pos[2]:.2f}")
                break
            time.sleep(0.5)

        # Pre-stream setpoints
        print("[PRE] Pre-sending setpoints...")
        for _ in range(100):
            self._send_pos(0.0, 0.0, -0.5)
            self._send_heartbeat()
            time.sleep(0.02)

        self._set_mode_offboard()
        self._arm()

        # Takeoff hover
        print(f"[TAKEOFF] Hover at {self.HOVER_Z}m")
        t0 = time.time()
        while time.time() - t0 < self.TAKEOFF_HOVER_TIME:
            self._send_pos(0.0, 0.0, -self.HOVER_Z)
            self._send_heartbeat()
            pos = self._get_pos()
            motors = self._get_motors()
            self._record('hover', time.time()-t0, 0, 0, self.HOVER_Z, pos, motors)
            time.sleep(self.DT)

        # Expand arms
        self._expand_arms()

        # Approach
        print(f"[APPROACH] {0.0} -> {self.APPROACH_X}")
        t0 = time.time()
        while time.time() - t0 < self.APPROACH_TIME:
            t = time.time() - t0
            s = min(1.0, t / self.APPROACH_TIME)
            x = 0.0 + (self.APPROACH_X - 0.0) * s
            self._send_pos(x, 0.0, -self.HOVER_Z)
            self._send_heartbeat()
            pos = self._get_pos()
            motors = self._get_motors()
            self._record('approach', t, x, 0, self.HOVER_Z, pos, motors)
            time.sleep(self.DT)

        # Push and detect stall
        print(f"[PUSH] Slow push toward pole at {self.PUSH_SPEED}m/s")
        push_start = time.time()
        contact_detected = False
        push_duration_est = abs((self.POLE_X + 0.15) - self.APPROACH_X) / max(self.PUSH_SPEED, 0.001)
        pole_surface = self.POLE_X - self.POLE_RADIUS

        while time.time() - push_start < 20.0:
            t = time.time() - push_start
            s = min(1.0, t / push_duration_est)
            x = self.APPROACH_X + ((self.POLE_X + 0.15) - self.APPROACH_X) * s
            self._send_pos(x, 0.0, -self.HOVER_Z)
            self._send_heartbeat()

            pos = self._get_pos()
            motors = self._get_motors()
            self._record('push', t, x, 0, self.HOVER_Z, pos, motors)

            if pos and s > self.STALL_S:
                if abs(pos[0] - pole_surface) < self.STALL_DIST:
                    contact_detected = True
                    print(f"[CONTACT] Detected at x={pos[0]:.2f}, stall after {t:.1f}s")
                    break
            time.sleep(self.DT)

        if not contact_detected:
            print("[FAIL] No contact detected")
            self._save_csv(suffix='_no_contact')
            return None

        # Long contact recording
        print(f"[RECORD] Holding contact for {self.RECORD_DURATION}s...")
        hold_x = self.POLE_X + 0.25
        record_start = time.time()
        while time.time() - record_start < self.RECORD_DURATION:
            t = time.time() - record_start
            self._send_pos(hold_x, 0.0, -self.HOVER_Z)
            self._send_heartbeat()
            pos = self._get_pos()
            motors = self._get_motors()
            self._record('contact', t, hold_x, 0, self.HOVER_Z, pos, motors)
            time.sleep(self.DT)

        # Land
        print("[LAND] Descending...")
        land_start = time.time()
        while time.time() - land_start < 8.0:
            t = time.time() - land_start
            self._send_pos(hold_x, 0.0, 0.0)
            self._send_heartbeat()
            pos = self._get_pos()
            motors = self._get_motors()
            self._record('land', t, hold_x, 0, 0, pos, motors)
            time.sleep(self.DT)

        # Disarm
        print("[DISARM]")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)

        path = self._save_csv()

        # Print summary
        contact_rows = [r for r in self.records if r['phase'] == 'contact']
        if contact_rows:
            motors_avg = [r['motor_avg'] for r in contact_rows if r['motor_avg'] is not None]
            pos_x = [r['pos_x'] for r in contact_rows if r['pos_x'] is not None]
            if motors_avg:
                print("\n" + "=" * 50)
                print("  CONTACT PHASE SUMMARY")
                print("=" * 50)
                print(f"  Duration:     {len(contact_rows) * self.DT:.1f}s")
                print(f"  Motor avg:    {sum(motors_avg)/len(motors_avg):.1f} PWM")
                print(f"  Motor min:    {min(motors_avg):.1f} PWM")
                print(f"  Motor max:    {max(motors_avg):.1f} PWM")
                print(f"  Motor std:    {math.sqrt(sum((m-min(motors_avg))**2 for m in motors_avg)/len(motors_avg)):.1f}")
                if pos_x:
                    print(f"  Pos x mean:   {sum(pos_x)/len(pos_x):.3f}m")
                print("=" * 50)

        return path


def main():
    parser = argparse.ArgumentParser(description='Long-contact motor comparison test')
    parser.add_argument('--output', default='contact_test', help='Output CSV prefix')
    args = parser.parse_args()

    test = LongContactTest(output_prefix=args.output)
    path = test.run()
    if path:
        print(f"\n[DONE] Log: {path}")
    else:
        print("\n[FAIL] Test did not complete")


if __name__ == '__main__':
    main()
