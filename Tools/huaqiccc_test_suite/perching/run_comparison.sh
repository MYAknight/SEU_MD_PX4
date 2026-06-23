#!/bin/bash
# Automated A/B comparison: Spring Model vs Hard-Push
# Restarts SITL between each test to avoid EKF drift.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="/home/a/Projects/PX4/SEU_MD_PX4"
OUTPUT_DIR="$HOME/huaqiccc_logs"
mkdir -p "$OUTPUT_DIR"

source /opt/ros/noetic/setup.bash
source /home/a/catkin_ws/devel_isolated/setup.bash
source /home/a/Projects/PX4/env_seu_md_px4.sh

# ---------- Helper functions ----------

kill_all() {
    echo "[CLEAN] Killing ALL old SITL/ROS processes..."
    ps aux | grep -E 'px4.*bin|gzserver|gzclient|rosmaster|roslaunch|mavros_node|rostopic' | grep -v grep | awk '{print $2}' | xargs -r kill -9 2>/dev/null || true
    rm -f /tmp/px4_lock-* /tmp/px4-sock-*
    sleep 5
}

start_sitl() {
    echo "[START] Launching SITL..."
    cd "$PX4_DIR"
    roslaunch px4 mavros_posix_sitl_perching_16cm.launch > /tmp/sitl_compare.log 2>&1 &
    SITL_PID=$!
    echo "[START] SITL PID=$SITL_PID"
    echo "[WAIT] Waiting 55s for full initialization..."
    sleep 55
    echo "[OK] SITL should be ready"
}

wait_for_mavros() {
    echo "[WAIT] Checking MAVROS connection..."
    for i in {1..20}; do
        state=$(rostopic echo -n 1 /mavros/state 2>/dev/null | grep "connected:" | awk '{print $2}')
        if [ "$state" = "True" ]; then
            echo "[OK] MAVROS connected"
            return 0
        fi
        sleep 2
    done
    echo "[WARN] MAVROS not confirmed connected, continuing anyway..."
}

# ---------- Test A: Spring Model ----------

echo ""
echo "========================================"
echo "TEST A: Spring Model"
echo "  MPCA_PC_SPRING = 1"
echo "  MPCA_PC_PRELOAD = 0.02"
echo "  MPCA_PC_K_SOFT = 0.20"
echo "========================================"
echo ""

kill_all
start_sitl
wait_for_mavros

cd "$SCRIPT_DIR"
# Set parameters before flight
python3 -c "
import rospy
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import ParamValue
import time
rospy.wait_for_service('/mavros/param/set', timeout=10)
ps = rospy.ServiceProxy('/mavros/param/set', ParamSet)
for p,v,t in [('MPCA_PC_SPRING', 1, True), ('MPCA_PC_PRELOAD', 0.02, False), ('MPCA_PC_K_SOFT', 0.20, False), ('MPCA_PC_EN', 1, True)]:
    pv = ParamValue()
    if t: pv.integer = int(v)
    else: pv.real = float(v)
    ps(param_id=p, value=pv)
    time.sleep(0.3)
print('[PARAM] Spring model params set')
"

# Run test
python3 contact_compare_ros.py --output spring_model

# Save SITL log
cp /tmp/sitl_compare.log "$OUTPUT_DIR/sitl_spring_model.log" 2>/dev/null || true
# Extract PX4 console stats
grep "COMPLIANT stats" "$OUTPUT_DIR/sitl_spring_model.log" 2>/dev/null || echo "No COMPLIANT stats in log"

# ---------- Test B: Hard Push ----------

echo ""
echo "========================================"
echo "TEST B: Hard Push (integral windup)"
echo "  MPCA_PC_SPRING = 0"
echo "  MPCA_PC_PRELOAD = 0.10"
echo "  MPCA_PC_K_SOFT = 1.00"
echo "========================================"
echo ""

kill_all
start_sitl
wait_for_mavros

cd "$SCRIPT_DIR"
python3 -c "
import rospy
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import ParamValue
import time
rospy.wait_for_service('/mavros/param/set', timeout=10)
ps = rospy.ServiceProxy('/mavros/param/set', ParamSet)
for p,v,t in [('MPCA_PC_SPRING', 0, True), ('MPCA_PC_PRELOAD', 0.10, False), ('MPCA_PC_K_SOFT', 1.00, False), ('MPCA_PC_EN', 1, True)]:
    pv = ParamValue()
    if t: pv.integer = int(v)
    else: pv.real = float(v)
    ps(param_id=p, value=pv)
    time.sleep(0.3)
print('[PARAM] Hard-push params set')
"

python3 contact_compare_ros.py --output hard_push

cp /tmp/sitl_compare.log "$OUTPUT_DIR/sitl_hard_push.log" 2>/dev/null || true
grep "COMPLIANT stats" "$OUTPUT_DIR/sitl_hard_push.log" 2>/dev/null || echo "No COMPLIANT stats in log"

# ---------- Cleanup ----------

kill_all

echo ""
echo "========================================"
echo "A/B COMPARISON COMPLETE"
echo "========================================"
echo "Logs saved to: $OUTPUT_DIR"
echo ""
echo "Next: run analysis to compare motor outputs"
echo "  python3 analyze_contact.py"
