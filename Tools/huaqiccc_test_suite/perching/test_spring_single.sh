#!/bin/bash
# Quick single test for spring model validation

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/noetic/setup.bash
source /home/a/catkin_ws/devel/setup.bash

echo "[CLEAN] Killing old processes..."
ps aux | grep -E 'px4.*bin|gzserver|gzclient|rosmaster|roslaunch|mavros_node' | grep -v grep | awk '{print $2}' | xargs -r kill -9 2>/dev/null || true
rm -f /tmp/px4_lock-* /tmp/px4-sock-*
sleep 5

echo "[START] Launching SITL..."
cd /home/a/PX4-Autopilot
roslaunch px4 mavros_posix_sitl_perching_16cm.launch > /tmp/sitl_spring.log 2>&1 &
echo "[WAIT] 55s..."
sleep 55

echo "[PARAM] Setting spring model parameters..."
python3 -c "
import rospy
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import ParamValue
import time
rospy.init_node('param_setter', anonymous=True)
rospy.wait_for_service('/mavros/param/set', timeout=10)
ps = rospy.ServiceProxy('/mavros/param/set', ParamSet)
for p,v,t in [('MPCA_PC_SPRING', 1, True), ('MPCA_PC_PRELOAD', 0.02, False), ('MPCA_PC_K_SOFT', 0.20, False), ('MPCA_PC_EN', 1, True)]:
    pv = ParamValue()
    if t: pv.integer = int(v)
    else: pv.real = float(v)
    resp = ps(param_id=p, value=pv)
    time.sleep(0.3)
print('[OK] Params set')
"

echo "[RUN] Starting flight test..."
cd "$SCRIPT_DIR"
python3 contact_compare_ros.py --output spring_single --contact-hold 8

echo ""
echo "[DONE] Results saved to ~/huaqiccc_logs/spring_single_*.csv"
echo "Check the 'contact' phase motor_avg values:"
grep contact ~/huaqiccc_logs/spring_single_*.csv | head -20
