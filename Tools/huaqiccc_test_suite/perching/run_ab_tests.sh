#!/bin/bash
# Automated A/B impedance control test runner
# Restarts SITL between each test to avoid EKF drift

set -e

PX4_DIR=/home/a/Projects/PX4/SEU_MD_PX4
TEST_DIR=/home/a/Projects/PX4/SEU_MD_PX4/Tools/huaqiccc_test_suite/perching
LOG_DIR=/home/a/huaqiccc_logs

K_VALUES=("1.00" "0.20" "0.05")
RUNS_PER_K=3

echo "========================================"
echo "  Impedance Control A/B Test Runner"
echo "========================================"
echo ""

for k in "${K_VALUES[@]}"; do
    for run in $(seq 1 $RUNS_PER_K); do
        echo ""
        echo ">>> Testing k_soft=$k  run $run/$RUNS_PER_K"
        echo ""

        # 1. Kill everything
        pkill -9 px4 || true
        pkill -9 gzserver || true
        pkill -9 gzclient || true
        pkill -9 rosmaster || true
        pkill -9 roslaunch || true
        sleep 3

        # 2. Start SITL
        cd $PX4_DIR
        source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
        roslaunch px4 mavros_posix_sitl_perching_16cm.launch > /tmp/sitl_ab_test.log 2>&1 &
        SITL_PID=$!
        echo "SITL launched (PID=$SITL_PID), waiting 25s for init..."
        sleep 25

        # 3. Verify EKF
        python3 -c "
import rospy
from geometry_msgs.msg import PoseStamped
rospy.init_node('ekf_check', anonymous=True)
msg = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped, timeout=10.0)
p = msg.pose.position
if abs(p.z) > 1.0:
    print(f'[WARN] EKF not converged: z={p.z:.2f}')
else:
    print(f'[OK] EKF converged: z={p.z:.3f}')
" 2>/dev/null

        # 4. Run test
        cd $TEST_DIR
        python3 grasp_16cm.py --output "k${k/./}_r${run}" --k-soft $k 2>&1 | tee "/tmp/k${k/./}_r${run}.log"

        # 5. Small delay before next cycle
        sleep 2
    done
done

echo ""
echo "========================================"
echo "  All tests complete"
echo "  Logs in: $LOG_DIR"
echo "========================================"
