#!/usr/bin/env python3
"""
impedance_benchmark.py
======================
Single-process A/B benchmark. Runs trials sequentially in the same Python
process, restarting SITL between trials to ensure fair comparison.

Usage:
    python3 impedance_benchmark.py
"""

import os
import subprocess
import sys
import time

N_TRIALS = 3
GROUPS = [
    ("k100", 1.00, "Impedance OFF (k_soft=1.00)"),
    ("k020", 0.20, "Impedance ON  (k_soft=0.20)"),
    ("k005", 0.05, "Very soft     (k_soft=0.05)"),
]


def kill_all():
    for proc in ['px4', 'gzserver', 'gzclient', 'roslaunch', 'mavros_node']:
        subprocess.run(['pkill', '-9', proc], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(3)


def start_sitl():
    cmd = (
        "cd /home/a/catkin_ws && source devel/setup.bash && "
        "roslaunch px4 mavros_posix_sitl_perching_16cm.launch "
        "fcu_url:=udp://:14540@localhost:14580"
    )
    proc = subprocess.Popen(cmd, shell=True, executable="/bin/bash",
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            start_new_session=True)
    return proc


def wait_mavros(timeout=60):
    import rospy
    from mavros_msgs.msg import State
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            state = rospy.wait_for_message('/mavros/state', State, timeout=5.0)
            if state.connected:
                return True
        except Exception:
            pass
        time.sleep(1)
    return False


def set_k_soft(val):
    import rospy
    from mavros_msgs.srv import ParamSet
    from mavros_msgs.msg import ParamValue
    rospy.wait_for_service('/mavros/param/set', timeout=15.0)
    ps = rospy.ServiceProxy('/mavros/param/set', ParamSet)
    pv = ParamValue()
    pv.real = float(val)
    resp = ps(param_id='MPCA_PC_K_SOFT', value=pv)
    return resp.success


def shutdown_ros():
    import rospy
    try:
        rospy.signal_shutdown('benchmark trial complete')
    except Exception:
        pass
    time.sleep(1)


def run_one_trial(group_name, k_soft):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from grasp_16cm import GraspFlightTest

    # Ensure ROS is initialized for this trial
    import rospy
    try:
        rospy.init_node('huaqiccc_grasp_test', anonymous=True)
    except rospy.exceptions.ROSException:
        pass

    # Set parameter
    if not set_k_soft(k_soft):
        print(f"  [WARN] Failed to set k_soft={k_soft}, continuing with current value")
    time.sleep(1.0)

    test = GraspFlightTest(output_prefix=f"impedance_{group_name}")
    csv_path = test.run()
    return csv_path


def main():
    print("=" * 60)
    print("  Impedance Control A/B Benchmark")
    print("=" * 60)
    print(f"Groups: {[g[0] for g in GROUPS]}")
    print(f"Trials per group: {N_TRIALS}")
    print("=" * 60)

    results = []
    for group_name, k_soft, desc in GROUPS:
        print(f"\n{'='*60}")
        print(f"GROUP: {group_name} | {desc}")
        print(f"{'='*60}")
        for trial in range(1, N_TRIALS + 1):
            print(f"\n--- Trial {trial}/{N_TRIALS} ---")

            # Clean shutdown + kill
            shutdown_ros()
            kill_all()
            time.sleep(2)

            # Start fresh SITL
            proc = start_sitl()
            if not wait_mavros(timeout=60):
                print("  [ERROR] MAVROS not ready")
                results.append((group_name, k_soft, trial, None))
                continue
            time.sleep(5)

            # Run trial
            try:
                csv_path = run_one_trial(group_name, k_soft)
                results.append((group_name, k_soft, trial, csv_path))
                if csv_path:
                    print(f"  [RESULT] {csv_path}")
                else:
                    print(f"  [RESULT] FAIL (no CSV)")
            except Exception as e:
                print(f"  [ERROR] {e}")
                results.append((group_name, k_soft, trial, None))

    # Final cleanup
    shutdown_ros()
    kill_all()

    print("\n" + "=" * 60)
    print("BENCHMARK COMPLETE")
    print("=" * 60)
    for group_name, k_soft, trial, csv in results:
        status = "OK" if csv else "FAIL"
        print(f"  {group_name} trial-{trial}: {status}  {csv or ''}")
    print(f"\nNext: python3 analyze_impedance.py --prefix impedance")


if __name__ == '__main__':
    main()
