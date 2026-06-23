#!/bin/bash
set -e

export MPCA_MODE=${1:-0}  # 0=original PID, 1=GS-PID, 2=LQR, 3=MPC

# Robust cleanup for all ROS/Gazebo/SITL processes
cleanup_all() {
    echo "[CLEAN] Killing ROS/Gazebo/SITL processes..."
    # Kill by process names / command-line patterns
    sudo -n true 2>/dev/null || true
    pkill -9 -f "rosmaster" 2>/dev/null || true
    pkill -9 -f "rosout" 2>/dev/null || true
    pkill -9 -f "roslaunch" 2>/dev/null || true
    pkill -9 -f "roscore" 2>/dev/null || true
    pkill -9 -f "gzserver" 2>/dev/null || true
    pkill -9 -f "gzclient" 2>/dev/null || true
    pkill -9 -f "mavros_node" 2>/dev/null || true
    pkill -9 -f "px4" 2>/dev/null || true
    pkill -9 -f "simplified_flight.py" 2>/dev/null || true
    # Free ROS master port if still occupied
    if command -v fuser >/dev/null 2>&1; then
        fuser -k 11311/tcp 2>/dev/null || true
    fi
    sleep 2
}

# Ensure cleanup on exit regardless of how the script terminates
trap cleanup_all EXIT

echo "========================================"
echo "  huaqiccc Simplified Flight Test"
echo "  MPCA_MODE=$MPCA_MODE"
echo "========================================"

# Source environments
source /opt/ros/noetic/setup.bash
source /home/a/catkin_ws/devel_isolated/setup.bash
source /home/a/Projects/PX4/env_seu_md_px4.sh
export GAZEBO_PLUGIN_PATH="/home/a/Projects/PX4/SEU_MD_PX4/build/px4_sitl_default/build_gazebo-classic:$GAZEBO_PLUGIN_PATH"
export DISPLAY=:0

# Inject MPCA_MODE into ROMFS before starting PX4
ROMFS_PARAMS="/home/a/Projects/PX4/SEU_MD_PX4/ROMFS/px4fmu_common/init.d-posix/px4-rc.params"
BUILD_PARAMS="/home/a/Projects/PX4/SEU_MD_PX4/build/px4_sitl_default/etc/init.d-posix/px4-rc.params"
echo "[CONFIG] Setting MPCA_MODE=$MPCA_MODE in ROMFS..."
sed -i "s/^param set-default MPCA_MODE .*/param set-default MPCA_MODE $MPCA_MODE/" "$ROMFS_PARAMS"
if [ -f "$BUILD_PARAMS" ]; then
    sed -i "s/^param set-default MPCA_MODE .*/param set-default MPCA_MODE $MPCA_MODE/" "$BUILD_PARAMS"
fi

# Cleanup old ROS logs
if [ -d "$HOME/.ros/log" ]; then
    LOG_SIZE=$(du -sm "$HOME/.ros/log" 2>/dev/null | cut -f1)
    if [ -n "$LOG_SIZE" ] && [ "$LOG_SIZE" -gt 500 ]; then
        echo "[CLEAN] Purging old ROS logs (${LOG_SIZE}MB)..."
        rm -rf "$HOME/.ros/log"/*
    fi
fi

# Thorough cleanup
cleanup_all

# Wait until rosmaster port is truly free
for i in $(seq 1 15); do
    if ! ss -tln 2>/dev/null | grep -q ':11311'; then
        echo "[OK] ROS master port 11311 is free"
        break
    fi
    echo "[WAIT] Port 11311 still occupied, retrying..."
    sleep 1
done

# Start roscore first
echo "[LAUNCH] Starting roscore..."
roscore &
ROSCORE_PID=$!
sleep 5

# Start simulation
echo "[LAUNCH] Starting PX4 SITL..."
roslaunch /home/a/Projects/PX4/SEU_MD_PX4/launch/mavros_posix_sitl.launch &
SIM_PID=$!
echo "[LAUNCH] PID=$SIM_PID"

# Wait for MAVROS to be fully ready
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
echo "[FLIGHT] Starting simplified flight test..."
PYTHONUNBUFFERED=1 stdbuf -oL python3 /home/a/Projects/PX4/SEU_MD_PX4/Tools/huaqiccc_test_suite/flight/simplified_flight.py 2>&1 | tee /tmp/simplified_flight.log

# Stop simulation (cleanup_all runs via EXIT trap as well, but call it explicitly here)
echo "[CLEAN] Stopping simulation..."
cleanup_all

echo "========================================"
echo "  Done"
echo "========================================"

# Show results
ls -lt ~/huaqiccc_logs/huaqiccc_flight_with_algo_*.csv 2>/dev/null | head -1
ls -lt ~/.ros/log/*.ulg 2>/dev/null | head -1
