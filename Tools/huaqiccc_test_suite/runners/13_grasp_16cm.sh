#!/bin/bash
set -e

export MPCA_MODE=${1:-0}

echo "========================================"
echo "  huaqiccc Perching Grasp Test (16cm)  "
echo "  MPCA_MODE=$MPCA_MODE"
echo "========================================"

source /opt/ros/noetic/setup.bash
source /home/a/catkin_ws/devel_isolated/setup.bash
source /home/a/Projects/PX4/env_seu_md_px4.sh
export GAZEBO_PLUGIN_PATH="/home/a/Projects/PX4/SEU_MD_PX4/build/px4_sitl_default/build_gazebo-classic:$GAZEBO_PLUGIN_PATH"
export GAZEBO_MODEL_PATH="/home/a/Projects/PX4/SEU_MD_PX4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$GAZEBO_MODEL_PATH"
export DISPLAY=:0

# Inject MPCA_MODE and admittance/compliance parameters into the build copy
# of the startup script. ROMFS is left untouched so default values remain clean.
ROMFS_PARAMS="/home/a/Projects/PX4/SEU_MD_PX4/ROMFS/px4fmu_common/init.d-posix/px4-rc.params"
BUILD_PARAMS="/home/a/Projects/PX4/SEU_MD_PX4/build/px4_sitl_default/etc/init.d-posix/px4-rc.params"
if [ ! -f "$BUILD_PARAMS" ]; then
    echo "[CONFIG] Build params script missing; copying from ROMFS..."
    mkdir -p "$(dirname "$BUILD_PARAMS")"
    cp "$ROMFS_PARAMS" "$BUILD_PARAMS"
fi

echo "[CONFIG] Setting MPCA_MODE=$MPCA_MODE in build params..."
sed -i "s/^param set-default MPCA_MODE .*/param set-default MPCA_MODE $MPCA_MODE/" "$BUILD_PARAMS"

# Admittance/compliance/adaptive parameters (scheme A/B/C)
MPCA_PC_ADM_MASS=${MPCA_PC_ADM_MASS:-1.5}
MPCA_PC_ADM_KA=${MPCA_PC_ADM_KA:-0.0}
MPCA_PC_ADM_FD=${MPCA_PC_ADM_FD:-1.0}
MPCA_PC_ADM_LIM=${MPCA_PC_ADM_LIM:-0.03}
MPCA_PC_ADM_KP=${MPCA_PC_ADM_KP:-0.0}
MPCA_PC_ADM_KV=${MPCA_PC_ADM_KV:-0.0}
MPCA_PC_ADM_KT=${MPCA_PC_ADM_KT:-0.0}
MPCA_PC_ADM_KC=${MPCA_PC_ADM_KC:-0.0}
MPCA_PC_ADM_W1=${MPCA_PC_ADM_W1:-1.0}
MPCA_PC_ADM_W2=${MPCA_PC_ADM_W2:-1.0}
echo "[CONFIG] Forcing admittance/compliance parameters in build params..."
# Remove any previously injected forced-param lines to avoid duplication.
sed -i "/^param set MPCA_PC_ADM_/d" "$BUILD_PARAMS"
sed -i "/# Admittance\/compliance\/adaptive position refinement (forced)/d" "$BUILD_PARAMS"
# Append forced param set lines. Using 'param set' instead of 'param set-default'
# prevents any previously-stored parameter value from overriding the test config.
cat >> "$BUILD_PARAMS" <<EOF
# Admittance/compliance/adaptive position refinement (forced for this test)
param set MPCA_PC_ADM_MASS $MPCA_PC_ADM_MASS
param set MPCA_PC_ADM_KA $MPCA_PC_ADM_KA
param set MPCA_PC_ADM_FD $MPCA_PC_ADM_FD
param set MPCA_PC_ADM_LIM $MPCA_PC_ADM_LIM
param set MPCA_PC_ADM_KP $MPCA_PC_ADM_KP
param set MPCA_PC_ADM_KV $MPCA_PC_ADM_KV
param set MPCA_PC_ADM_KT $MPCA_PC_ADM_KT
param set MPCA_PC_ADM_KC $MPCA_PC_ADM_KC
param set MPCA_PC_ADM_W1 $MPCA_PC_ADM_W1
param set MPCA_PC_ADM_W2 $MPCA_PC_ADM_W2
EOF

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

# Start simulation
echo "[LAUNCH] Starting PX4 SITL with perching_pole_16cm.world..."
roslaunch /home/a/Projects/PX4/SEU_MD_PX4/launch/mavros_posix_sitl_perching_16cm.launch &
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
echo "[FLIGHT] Starting perching grasp test..."
PYTHONUNBUFFERED=1 stdbuf -oL python3 /home/a/Projects/PX4/SEU_MD_PX4/Tools/huaqiccc_test_suite/perching/grasp_16cm.py 2>&1 | tee /tmp/grasp_test.log

# Stop simulation
echo "[CLEAN] Stopping simulation..."
pkill -9 -f "roslaunch|roscore|gzserver|gzclient|px4|mavros_node" 2>/dev/null || true
sleep 2

echo "========================================"
echo "  Done"
echo "========================================"

# Show results
ls -lt ~/huaqiccc_logs/grasp_test_*.csv 2>/dev/null | head -1
