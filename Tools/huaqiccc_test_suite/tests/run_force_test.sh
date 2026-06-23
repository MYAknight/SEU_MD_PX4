#!/bin/bash
# Launch SITL in empty world and run the force-estimation validation test.
set -e

source /opt/ros/noetic/setup.bash
source /home/a/catkin_ws/devel_isolated/setup.bash
source /home/a/Projects/PX4/env_seu_md_px4.sh
export GAZEBO_PLUGIN_PATH="/home/a/Projects/PX4/SEU_MD_PX4/build/px4_sitl_default/build_gazebo-classic:$GAZEBO_PLUGIN_PATH"
export GAZEBO_MODEL_PATH="/home/a/Projects/PX4/SEU_MD_PX4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$GAZEBO_MODEL_PATH"
export DISPLAY=:0

# Configure scheme A parameters in SITL build params.
# KA is set to 0 so the controller does not move the drone; we only observe f_est.
BUILD_PARAMS="/home/a/Projects/PX4/SEU_MD_PX4/build/px4_sitl_default/etc/init.d-posix/px4-rc.params"
if [ ! -f "$BUILD_PARAMS" ]; then
    mkdir -p "$(dirname "$BUILD_PARAMS")"
    cp "/home/a/Projects/PX4/SEU_MD_PX4/ROMFS/px4fmu_common/init.d-posix/px4-rc.params" "$BUILD_PARAMS"
fi
sed -i "/^param set MPCA_PC_ADM_/d" "$BUILD_PARAMS"
sed -i "/# Force-estimation test params/d" "$BUILD_PARAMS"
cat >> "$BUILD_PARAMS" <<'EOF'
# Force-estimation test params
param set MPCA_PC_ADM_MASS 1.5
param set MPCA_PC_ADM_KA 0.0
param set MPCA_PC_ADM_FD 0.0
param set MPCA_PC_ADM_LIM 0.03
param set MPCA_PC_ADM_KP 0.0
param set MPCA_PC_ADM_KV 0.0
param set MPCA_PC_ADM_KT 0.0
param set MPCA_PC_ADM_KC 0.0
param set MPCA_PC_ADM_W1 1.0
param set MPCA_PC_ADM_W2 1.0
EOF

echo "[CLEAN] Killing old ROS/Gazebo processes..."
pkill -9 -f "roslaunch|roscore|gzserver|gzclient|px4|mavros_node" 2>/dev/null || true
sleep 3

echo "[LAUNCH] Starting roscore..."
roscore &
ROSCORE_PID=$!
sleep 5

echo "[LAUNCH] Starting PX4 SITL (empty world)..."
roslaunch /home/a/Projects/PX4/SEU_MD_PX4/launch/mavros_posix_sitl_perching_16cm.launch \
    world:=/home/a/Projects/PX4/SEU_MD_PX4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world \
    gui:=true &
SIM_PID=$!
echo "[LAUNCH] SIM_PID=$SIM_PID"

echo "[WAIT] Waiting for MAVROS..."
MAX_WAIT=90
for i in $(seq 1 $MAX_WAIT); do
    if rosservice list 2>/dev/null | grep -q "/mavros/cmd/arming"; then
        echo "[OK] MAVROS ready at ${i}s"
        break
    fi
    sleep 1
done

echo "[WAIT] Extra 5s for EKF..."
sleep 5

echo "[TEST] Running force-estimation test..."
PYTHONUNBUFFERED=1 stdbuf -oL python3 \
    /home/a/Projects/PX4/SEU_MD_PX4/Tools/huaqiccc_test_suite/tests/force_estimation_test.py \
    --forces 2.0 5.0 10.0 --duration 2.0 2>&1 | tee /tmp/force_est_test.log

echo "[CLEAN] Stopping simulation..."
pkill -9 -f "roslaunch|roscore|gzserver|gzclient|px4|mavros_node" 2>/dev/null || true
sleep 2

echo "========================================"
echo "Done. Latest CSV:"
ls -lt ~/huaqiccc_force_test/force_est_test_*.csv 2>/dev/null | head -1
