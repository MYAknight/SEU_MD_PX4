#!/bin/bash
# Batch test: Baseline (MPCA_PC_THR_MAR=1.0) vs New (MPCA_PC_THR_MAR=0.2)

set -e

PX4_DIR=/home/a/PX4-Autopilot
TEST_DIR=/home/a/huaqiccc_test_suite/perching
LOG_DIR=/home/a/huaqiccc_logs

# Clean old logs
rm -f $LOG_DIR/baseline_*.csv $LOG_DIR/new_*.csv

run_test() {
    local label=$1
    local thr_mar=$2
    local run=$3

    echo ""
    echo ">>> $label run $run (MPCA_PC_THR_MAR=$thr_mar)"
    
    # Kill everything
    pkill -9 px4 || true
    pkill -9 gzserver || true
    pkill -9 gzclient || true
    pkill -9 rosmaster || true
    pkill -9 roslaunch || true
    sleep 3

    # Start SITL
    cd $PX4_DIR
    source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
    roslaunch px4 mavros_posix_sitl_perching_16cm.launch > /tmp/sitl_${label}_r${run}.log 2>&1 &
    SITL_PID=$!
    echo "SITL launched (PID=$SITL_PID), waiting 25s..."
    sleep 25

    # Verify EKF
    python3 -c "
import rospy
from geometry_msgs.msg import PoseStamped
rospy.init_node('ekf_check', anonymous=True)
msg = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped, timeout=10.0)
p = msg.pose.position
if abs(p.z) > 1.0:
    print(f'[WARN] EKF not converged: z={p.z:.2f}')
    exit(1)
else:
    print(f'[OK] EKF converged: z={p.z:.3f}')
" 2>/dev/null

    # Set parameter
    python3 -c "
import rospy
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import ParamValue
rospy.init_node('param_set', anonymous=True)
rospy.wait_for_service('/mavros/param/set', timeout=10.0)
param_set = rospy.ServiceProxy('/mavros/param/set', ParamSet)
pv = ParamValue()
pv.integer = 0
pv.real = $thr_mar
resp = param_set(param_id='MPCA_PC_THR_MAR', value=pv)
print(f'[OK] Set MPCA_PC_THR_MAR=$thr_mar: success={resp.success}')
" 2>/dev/null

    # Run test
    cd $TEST_DIR
    python3 grasp_16cm.py --output "${label}_r${run}" --k-soft 0.20 2>&1 | tee "/tmp/${label}_r${run}.log"

    sleep 2
}

# Baseline: 3 runs
for run in 1 2 3; do
    run_test "baseline" "1.0" "$run"
done

# New: 3 runs
for run in 1 2 3; do
    run_test "new" "0.2" "$run"
done

echo ""
echo "========================================"
echo "  All tests complete"
echo "========================================"

# Summary
echo ""
echo "Results:"
for f in $LOG_DIR/baseline_*.csv $LOG_DIR/new_*.csv; do
    if [ -f "$f" ]; then
        basename=$(basename "$f")
        echo "  $basename"
    fi
done
