#!/usr/bin/env python3
"""
Inspect / tune PX4 MAVLink telemetry parameters for the HM30 datalink.

HM30 carries video + telemetry + RC over the same RF link.  If the serial
telemetry side (TTL to flight controller) is too fast for the configured baud
rate, the HM30 can drop or stall the RTSP video stream — especially after
ARM/OFFBOARD when PX4 starts streaming high-rate MAVLink messages.

Run without arguments to see the current configuration:
    rosrun ground_station tune_hm30_telemetry.py

Cap the telemetry data rate for the active MAVLink instance (reboot required):
    rosrun ground_station tune_hm30_telemetry.py --rate 2000

Recommended workflow:
1. Set HM30 "Datalink Baud Rate" to 230400 in the ground-unit OLED menu.
2. Match PX4 serial baud (e.g. SER_TEL1_BAUD or SER_TEL2_BAUD) to 230400.
3. Cap MAV_X_RATE for the active instance to 2000~4000 B/s.
4. Reboot the flight controller.
"""
import argparse
import sys

import rospy
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import ParamGet, ParamSet

# PX4 parameter mapping (common values, may differ slightly by firmware version)
PORT_MAP = {
    0: "Disabled",
    101: "TELEM1",
    102: "TELEM2",
    103: "TELEM3",
    104: "TELEM4",
    1000: "Ethernet",
}

SER_BAUD_PARAM = {
    101: "SER_TEL1_BAUD",
    102: "SER_TEL2_BAUD",
    103: "SER_TEL3_BAUD",
    104: "SER_TEL4_BAUD",
}

MAV_MODE_MAP = {
    0: "Normal",
    1: "Custom",
    2: "Onboard",
    3: "OSD",
    4: "Config",
    5: "Minimal",
    6: "ExtVision",
    7: "ExtVisionMin",
    8: "Iridium",
}


def get_param(get_srv, name):
    try:
        resp = get_srv(param_id=name)
    except rospy.ServiceException as e:
        print(f"  ERROR calling param/get for {name}: {e}")
        return None
    if not resp.success:
        print(f"  {name}: (not available)")
        return None
    val = resp.value
    if val.param_type == ParamValue.PARAM_INTEGER:
        return int(val.integer)
    return float(val.real)


def set_param(set_srv, name, value):
    val = ParamValue()
    if isinstance(value, int):
        val.param_type = ParamValue.PARAM_INTEGER
        val.integer = value
    else:
        val.param_type = ParamValue.PARAM_REAL
        val.real = float(value)
    try:
        resp = set_srv(param_id=name, value=val)
    except rospy.ServiceException as e:
        print(f"  ERROR setting {name}: {e}")
        return False
    print(f"  {name} -> {value}: {'OK' if resp.success else 'FAILED'}")
    return resp.success


def baud_enum_to_rate(enum_val):
    """Map SER_TELx_BAUD enum value to actual baud rate."""
    # PX4 enum values are usually 0: auto, then index into a table.
    # Common mapping for manual values (verify in your firmware):
    table = {
        0: "Auto",
        1: 9600,
        2: 19200,
        3: 38400,
        4: 57600,
        5: 115200,
        6: 230400,
        7: 460800,
        8: 921600,
        9: 1000000,
        10: 1500000,
        11: 2000000,
        12: 3000000,
        13: 4000000,
        14: 6000000,
        15: 7000000,
        16: 8000000,
    }
    return table.get(enum_val, f"unknown({enum_val})")


def main():
    parser = argparse.ArgumentParser(description="Inspect/tune PX4 MAVLink telemetry for HM30")
    parser.add_argument("--rate", type=int, default=None,
                        help="Target MAV_X_RATE in bytes/sec for the active instance")
    parser.add_argument("--mode", type=int, default=None,
                        help="Target MAV_X_MODE enum (0=Normal, 2=Onboard, 5=Minimal)")
    args = parser.parse_args()

    rospy.init_node("tune_hm30_telemetry", anonymous=True)
    rospy.wait_for_service("/mavros/param/get", timeout=10.0)
    rospy.wait_for_service("/mavros/param/set", timeout=10.0)
    get_srv = rospy.ServiceProxy("/mavros/param/get", ParamGet)
    set_srv = rospy.ServiceProxy("/mavros/param/set", ParamSet)

    print("PX4 MAVLink telemetry configuration for HM30 link\n")

    active_instances = []
    for i in range(3):
        config = get_param(get_srv, f"MAV_{i}_CONFIG")
        if config is None:
            continue
        port_name = PORT_MAP.get(config, f"value({config})")
        if config == 0:
            print(f"MAV_{i}: Disabled")
            continue

        mode = get_param(get_srv, f"MAV_{i}_MODE")
        rate = get_param(get_srv, f"MAV_{i}_RATE")
        forward = get_param(get_srv, f"MAV_{i}_FORWARD")
        ser_baud = None
        ser_param = SER_BAUD_PARAM.get(config)
        if ser_param:
            ser_baud = get_param(get_srv, ser_param)

        print(f"MAV_{i}: {port_name} (MAV_{i}_CONFIG={config})")
        print(f"  Mode      MAV_{i}_MODE  = {MAV_MODE_MAP.get(mode, mode)}")
        print(f"  Rate      MAV_{i}_RATE  = {rate} B/s")
        print(f"  Forward   MAV_{i}_FORWARD = {forward}")
        if ser_param:
            print(f"  Baud      {ser_param} = {baud_enum_to_rate(ser_baud)}")

        active_instances.append((i, config, ser_param))

    if not active_instances:
        print("No active MAVLink instance found!")
        return 1

    # Determine the instance most likely used by HM30 (the one with the highest
    # baud rate / non-USB port).  Usually the HM30 is wired to TELEM1/TELEM2.
    # We apply changes to ALL active instances unless user risk is a concern.
    # To be safe, we only change instances whose port has a matching SER_*_BAUD.
    if args.rate is not None or args.mode is not None:
        print("\nApplying requested changes (reboot required to take effect):")
        changed = False
        for i, config, ser_param in active_instances:
            if ser_param is None:
                print(f"  Skipping MAV_{i} (no serial baud param for config={config})")
                continue
            if args.rate is not None:
                if set_param(set_srv, f"MAV_{i}_RATE", args.rate):
                    changed = True
            if args.mode is not None:
                if set_param(set_srv, f"MAV_{i}_MODE", args.mode):
                    changed = True
        if changed:
            print("\nIMPORTANT: Reboot the flight controller for new rate/mode to take effect.")
            print("After reboot, also update start_ground_station_auto.sh with HM30_BAUD=230400")

    print("\nRecommendations:")
    print("- If HM30 is wired to TELEM2, prefer SER_TEL2_BAUD=230400 + MAV_1_RATE=2000~4000")
    print("- Keep MAVROS fcu_url baud in start_ground_station_auto.sh identical to HM30 setting.")
    print("- Use 'rostopic hz /mavros/local_position/pose /mavros/imu/data' to verify rates.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
