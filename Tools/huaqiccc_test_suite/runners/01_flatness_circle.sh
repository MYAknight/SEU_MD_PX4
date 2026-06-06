#!/bin/bash
set -e

export MPCA_MODE=${1:-3}
export MPCA_FF_EN=${2:-1}
# Additional MPC tuning parameters from environment
export MPCA_MPC_ALPHA=${MPCA_MPC_ALPHA:-20.0}
export MPCA_MPC_R_DELTA=${MPCA_MPC_R_DELTA:-0.005}
export MPCA_FF_BLEND=${MPCA_FF_BLEND:-0.3}
export MPCA_FF_MASS=${MPCA_FF_MASS:-1.5}

echo "========================================"
echo "  huaqiccc Flatness Feedforward Test"
echo "  MPCA_MODE=$MPCA_MODE"
echo "  MPCA_FF_EN=$MPCA_FF_EN"
echo "========================================"

# Source environments
source /opt/ros/noetic/setup.bash
source /home/a/catkin_ws/devel/setup.bash
export GAZEBO_PLUGIN_PATH="/home/a/huaqiccc_ws/devel/lib:$GAZEBO_PLUGIN_PATH"
export DISPLAY=:0

# Inject params into ROMFS before starting PX4
ROMFS_PARAMS="/home/a/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.params"
BUILD_PARAMS="/home/a/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/px4-rc.params"

echo "[CONFIG] Setting MPCA_MODE=$MPCA_MODE, MPCA_FF_EN=$MPCA_FF_EN in ROMFS..."
for f in "$ROMFS_PARAMS" "$BUILD_PARAMS"; do
    if [ -f "$f" ]; then
        sed -i "s/^param set-default MPCA_MODE .*/param set-default MPCA_MODE $MPCA_MODE/" "$f"
        sed -i "s/^param set-default MPCA_FF_EN .*/param set-default MPCA_FF_EN $MPCA_FF_EN/" "$f"
        sed -i "s/^param set-default MPCA_MPC_ALPHA .*/param set-default MPCA_MPC_ALPHA $MPCA_MPC_ALPHA/" "$f"
        sed -i "s/^param set-default MPCA_MPC_R_DELTA .*/param set-default MPCA_MPC_R_DELTA $MPCA_MPC_R_DELTA/" "$f"
        sed -i "s/^param set-default MPCA_FF_BLEND .*/param set-default MPCA_FF_BLEND $MPCA_FF_BLEND/" "$f"
        sed -i "s/^param set-default MPCA_FF_MASS .*/param set-default MPCA_FF_MASS $MPCA_FF_MASS/" "$f"
        if ! grep -q "param set-default MPCA_MPC_ALPHA" "$f"; then
            echo "param set-default MPCA_MPC_ALPHA $MPCA_MPC_ALPHA" >> "$f"
        fi
        if ! grep -q "param set-default MPCA_MPC_R_DELTA" "$f"; then
            echo "param set-default MPCA_MPC_R_DELTA $MPCA_MPC_R_DELTA" >> "$f"
        fi
        if ! grep -q "param set-default MPCA_FF_BLEND" "$f"; then
            echo "param set-default MPCA_FF_BLEND $MPCA_FF_BLEND" >> "$f"
        fi
        if ! grep -q "param set-default MPCA_FF_MASS" "$f"; then
            echo "param set-default MPCA_FF_MASS $MPCA_FF_MASS" >> "$f"
        fi
        if ! grep -q "param set-default MPCA_FF_EN" "$f"; then
            echo "param set-default MPCA_FF_EN $MPCA_FF_EN" >> "$f"
        fi
    fi
done

# Remove saved parameter files to force ROMFS defaults
rm -f /home/a/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters.bson
rm -f /home/a/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters_backup.bson
rm -f /home/a/PX4-Autopilot/build/px4_sitl_default/parameters.bson
rm -f /home/a/PX4-Autopilot/build/px4_sitl_default/parameters_backup.bson
echo "[CONFIG] Cleared saved parameter files"

# Cleanup old ROS logs
if [ -d "$HOME/.ros/log" ]; then
    LOG_SIZE=$(du -sm "$HOME/.ros/log" 2>/dev/null | cut -f1)
    if [ -n "$LOG_SIZE" ] && [ "$LOG_SIZE" -gt 500 ]; then
        echo "[CLEAN] Purging old ROS logs (${LOG_SIZE}MB)..."
        rm -rf "$HOME/.ros/log"/*
    fi
fi

# Thorough cleanup
echo "[CLEAN] Thorough cleanup..."
pkill -9 -f "roslaunch|roscore|gzserver|gzclient|px4|mavros_node" 2>/dev/null || true
sleep 3

# Start roscore first
echo "[LAUNCH] Starting roscore..."
roscore &
ROSCORE_PID=$!
sleep 5

# Start simulation
echo "[LAUNCH] Starting PX4 SITL..."
roslaunch /home/a/PX4-Autopilot/launch/mavros_posix_sitl.launch &
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
echo "[FLIGHT] Starting flatness flight test..."
PYTHONUNBUFFERED=1 stdbuf -oL python3 /home/a/huaqiccc_test_suite/flight/flatness_circle.py --rate 50 --output "huaqiccc_flatness_m${MPCA_MODE}_ff${MPCA_FF_EN}" 2>&1 | tee /tmp/flatness_flight_m${MPCA_MODE}_ff${MPCA_FF_EN}.log

# Stop simulation
echo "[CLEAN] Stopping simulation..."
pkill -9 -f "roslaunch|roscore|gzserver|gzclient|px4|mavros_node" 2>/dev/null || true
sleep 2

echo "========================================"
echo "  Done"
echo "========================================"

# Show results
ls -lt ~/huaqiccc_logs/huaqiccc_flatness_*.csv 2>/dev/null | head -1
ls -lt ~/.ros/log/*.ulg 2>/dev/null | head -1
