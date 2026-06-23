#!/bin/bash
set -e

export MPCA_MODE=${1:-0}

echo "========================================"
echo "  huaqiccc Perching Pole Collision Test"
echo "  MPCA_MODE=$MPCA_MODE"
echo "========================================"

source /opt/ros/noetic/setup.bash
source /home/a/catkin_ws/devel_isolated/setup.bash
source /home/a/Projects/PX4/env_seu_md_px4.sh
export GAZEBO_PLUGIN_PATH="/home/a/Projects/PX4/SEU_MD_PX4/build/px4_sitl_default/build_gazebo-classic:$GAZEBO_PLUGIN_PATH"
export DISPLAY=:0

# Inject MPCA_MODE into ROMFS
ROMFS_PARAMS="/home/a/Projects/PX4/SEU_MD_PX4/ROMFS/px4fmu_common/init.d-posix/px4-rc.params"
BUILD_PARAMS="/home/a/Projects/PX4/SEU_MD_PX4/build/px4_sitl_default/etc/init.d-posix/px4-rc.params"
echo "[CONFIG] Setting MPCA_MODE=$MPCA_MODE in ROMFS..."
sed -i "s/^param set-default MPCA_MODE .*/param set-default MPCA_MODE $MPCA_MODE/" "$ROMFS_PARAMS"
if [ -f "$BUILD_PARAMS" ]; then
    sed -i "s/^param set-default MPCA_MODE .*/param set-default MPCA_MODE $MPCA_MODE/" "$BUILD_PARAMS"
fi

# Cleanup old ROS logs (prevent >1GB warning)
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

# Start roscore (avoid duplication)
if pgrep -x roscore > /dev/null 2>&1; then
    echo "[LAUNCH] roscore already running, skipping"
else
    echo "[LAUNCH] Starting roscore..."
    roscore &
    ROSCORE_PID=$!
    sleep 5
fi

# Start simulation
echo "[LAUNCH] Starting PX4 SITL with perching_pole.world..."
roslaunch /home/a/Projects/PX4/SEU_MD_PX4/launch/mavros_posix_sitl_perching.launch &
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

# Run flight test (hard timeout 2 minutes)
echo "[FLIGHT] Starting perching pole collision test (timeout 120s)..."
PYTHONUNBUFFERED=1 stdbuf -oL timeout 120 python3 \
    /home/a/Projects/PX4/SEU_MD_PX4/Tools/huaqiccc_test_suite/perching/pole_collision.py 2>&1 | tee /tmp/perching_test.log
FLIGHT_EXIT=${PIPESTATUS[0]}
if [ "$FLIGHT_EXIT" -eq 124 ]; then
    echo "[WARN] Flight test hit 120s timeout"
elif [ "$FLIGHT_EXIT" -ne 0 ]; then
    echo "[WARN] Flight test exited with code $FLIGHT_EXIT"
fi

# Stop simulation
echo "[CLEAN] Stopping simulation..."
pkill -9 -f "roslaunch|roscore|gzserver|gzclient|px4|mavros_node" 2>/dev/null || true
sleep 2

echo "========================================"
echo "  Done"
echo "========================================"

# Show results
ls -lt ~/huaqiccc_logs/perching_test_*.csv 2>/dev/null | head -1
