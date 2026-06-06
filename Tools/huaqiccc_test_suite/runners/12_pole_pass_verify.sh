#!/bin/bash
set -e

export MPCA_MODE=${1:-0}

echo "========================================"
echo "  huaqiccc Pole Pass Verification Test"
echo "  MPCA_MODE=$MPCA_MODE"
echo "========================================"

source /opt/ros/noetic/setup.bash
source /home/a/catkin_ws/devel/setup.bash
export GAZEBO_PLUGIN_PATH="/home/a/huaqiccc_ws/devel/lib:$GAZEBO_PLUGIN_PATH"
export DISPLAY=:0

# Inject MPCA_MODE into ROMFS
ROMFS_PARAMS="/home/a/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.params"
BUILD_PARAMS="/home/a/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/px4-rc.params"
echo "[CONFIG] Setting MPCA_MODE=$MPCA_MODE in ROMFS..."
sed -i "s/^param set-default MPCA_MODE .*/param set-default MPCA_MODE $MPCA_MODE/" "$ROMFS_PARAMS"
if [ -f "$BUILD_PARAMS" ]; then
    sed -i "s/^param set-default MPCA_MODE .*/param set-default MPCA_MODE $MPCA_MODE/" "$BUILD_PARAMS"
fi

# Cleanup old ROS logs
if command -v rosclean >/dev/null 2>&1; then
    echo "[CLEAN] Running rosclean check..."
    rosclean check 2>/dev/null || true
    rosclean purge -y 2>/dev/null || true
fi
if [ -d "$HOME/.ros/log" ]; then
    LOG_SIZE=$(du -sm "$HOME/.ros/log" 2>/dev/null | cut -f1)
    if [ -n "$LOG_SIZE" ] && [ "$LOG_SIZE" -gt 100 ]; then
        echo "[CLEAN] Purging old ROS logs (${LOG_SIZE}MB)..."
        rm -rf "$HOME/.ros/log"/*
    fi
fi

# Cleanup
echo "[CLEAN] Thorough cleanup..."
pkill -9 -f "roslaunch|roscore|gzserver|gzclient|px4|mavros_node" 2>/dev/null || true
sleep 3

# Start roscore
if pgrep -x roscore > /dev/null 2>&1; then
    echo "[LAUNCH] roscore already running, skipping"
else
    echo "[LAUNCH] Starting roscore..."
    roscore &
    ROSCORE_PID=$!
    sleep 5
fi

# Start simulation (8cm pole, original perching world)
echo "[LAUNCH] Starting PX4 SITL with perching_pole.world (8cm pole)..."
roslaunch /home/a/PX4-Autopilot/launch/mavros_posix_sitl_perching.launch &
SIM_PID=$!
echo "[LAUNCH] PID=$SIM_PID"

# Wait for MAVROS
MAX_WAIT=90
echo "[WAIT] Waiting for MAVROS arming service (max ${MAX_WAIT}s)..."
MAVROS_READY=0
for i in $(seq 1 $MAX_WAIT); do
    if rosservice list 2>/dev/null | grep -q "/mavros/cmd/arming"; then
        echo "[OK] MAVROS ready at ${i}s"
        MAVROS_READY=1
        break
    fi
    sleep 1
done

if [ "$MAVROS_READY" -eq 0 ]; then
    echo "[ERROR] MAVROS did not become ready within ${MAX_WAIT}s"
    pkill -9 -f "roslaunch|roscore|gzserver|gzclient|px4|mavros_node" 2>/dev/null || true
    exit 1
fi

# Extra wait for EKF
echo "[WAIT] Extra 5s for EKF initialization..."
sleep 5

# Run flight test
echo "[FLIGHT] Starting pole pass verification test..."
PYTHONUNBUFFERED=1 stdbuf -oL python3 /home/a/huaqiccc_test_suite/perching/pole_pass_verify.py 2>&1 | tee /tmp/pole_pass_test.log

# Stop simulation
echo "[CLEAN] Stopping simulation..."
pkill -9 -f "roslaunch|roscore|gzserver|gzclient|px4|mavros_node" 2>/dev/null || true
sleep 2

echo "========================================"
echo "  Done"
echo "========================================"

# Show results
ls -lt ~/huaqiccc_logs/pole_pass_test_*.csv 2>/dev/null | head -1
