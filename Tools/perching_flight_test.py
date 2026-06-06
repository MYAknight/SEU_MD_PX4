#!/usr/bin/env python3
"""
Automated perching collision test for huaqiccc morphing quadrotor.
Uses pymavlink OFFBOARD position control to fly into a pole.
"""

import time
from pymavlink import mavutil

SITL_UDP = "udp:127.0.0.1:14540"
TAKEOFF_ALT = 3.0
POLE_X = 5.0
FLY_VX = 0.3
MAX_TIME = 90


def send_pos(master, x, y, z, yaw=0.0):
    """Send local NED position setpoint (PX4 offboard)."""
    type_mask = 0x0FF8  # use position only
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask, x, y, z, 0, 0, 0, 0, 0, 0, yaw, 0.0
    )


def send_heartbeat(master):
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
    )


def get_pos(master):
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
    if msg:
        return msg.x, msg.y, msg.z
    return None


def arm(master):
    print("Arming...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(2)


def set_mode(master, mode_id):
    print(f"Setting mode {mode_id}...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        int(mode_id), 0, 0, 0, 0, 0, 0
    )
    time.sleep(1)


def main():
    print("=" * 60)
    print("Perching Collision Flight Test")
    print("=" * 60)

    print(f"\nConnecting to {SITL_UDP}...")
    master = mavutil.mavlink_connection(SITL_UDP)
    master.wait_heartbeat()
    print(f"System {master.target_system} ready")

    # Wait for position valid
    print("Waiting for position estimate...")
    while True:
        pos = get_pos(master)
        if pos:
            print(f"  pos: x={pos[0]:.2f} y={pos[1]:.2f} z={pos[2]:.2f}")
            break
        time.sleep(0.5)

    # Pre-stream offboard setpoints (PX4 requires this before arming in offboard)
    print("Pre-streaming offboard setpoints...")
    for _ in range(100):
        send_pos(master, 0, 0, -0.5)
        send_heartbeat(master)
        time.sleep(0.02)

    # Switch to OFFBOARD first, then arm
    set_mode(master, 6)  # OFFBOARD = 6
    arm(master)

    # Ascend to TAKEOFF_ALT using offboard position setpoints
    print(f"\nAscending to {TAKEOFF_ALT}m...")
    t0 = time.time()
    while time.time() - t0 < 20:
        send_pos(master, 0, 0, -TAKEOFF_ALT, yaw=0.0)
        send_heartbeat(master)
        pos = get_pos(master)
        if pos:
            if abs(pos[2] + TAKEOFF_ALT) < 0.3:
                print(f"  Reached altitude: z={pos[2]:.2f}")
                break
            if int((time.time() - t0) * 2) % 5 == 0:
                print(f"  z={pos[2]:.2f}")
        time.sleep(0.1)

    # Fly toward pole
    print(f"\nFlying toward pole at x={POLE_X}m...")
    t0 = time.time()
    contact = False
    pos_last = None

    try:
        while time.time() - t0 < MAX_TIME:
            elapsed = time.time() - t0
            target_x = min(FLY_VX * elapsed, POLE_X + 1.0)
            send_pos(master, target_x, 0, -TAKEOFF_ALT, yaw=0.0)
            send_heartbeat(master)

            pos = get_pos(master)
            if pos:
                x, y, z = pos
                if pos_last is None or abs(x - pos_last[0]) > 0.2:
                    print(f"  pos: x={x:.2f} y={y:.2f} z={z:.2f}")
                    pos_last = (x, y, z)

                # Check contact_state
                msg = master.recv_match(type='CONTACT_STATE', blocking=False)
                if msg:
                    print(f"  CONTACT: state={msg.state} close={msg.should_close}")
                    if msg.state == 3 and msg.should_close:
                        print("  *** GRASP TRIGGERED! ***")
                        contact = True
                        break

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nAborted")

    # Descend / land
    print("\nLanding...")
    if pos_last:
        for _ in range(100):
            send_pos(master, pos_last[0], 0, 0)
            send_heartbeat(master)
            time.sleep(0.02)

    set_mode(master, 4)  # HOLD = 4
    time.sleep(1)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
    )

    print("=" * 60)
    print("SUCCESS" if contact else "NO CONTACT")
    print("=" * 60)


if __name__ == '__main__':
    main()
