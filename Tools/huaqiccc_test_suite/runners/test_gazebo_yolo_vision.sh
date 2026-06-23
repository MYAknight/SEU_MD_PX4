#!/bin/bash
# 临时验证脚本：启动带前视相机的 SITL + 白色 16cm pole，运行 Gazebo→YOLO 桥
# 仅用于 Phase 1-3 验证，不执行接近/接触任务。
set -e

echo "========================================"
echo "  Gazebo Camera + YOLO Vision Test"
echo "========================================"

source /opt/ros/noetic/setup.bash
source /home/a/catkin_ws/devel_isolated/setup.bash
source /home/a/Projects/PX4/env_seu_md_px4.sh
export GAZEBO_PLUGIN_PATH="/home/a/Projects/PX4/SEU_MD_PX4/build/px4_sitl_default/build_gazebo-classic:$GAZEBO_PLUGIN_PATH"
export GAZEBO_MODEL_PATH="/home/a/Projects/PX4/SEU_MD_PX4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$GAZEBO_MODEL_PATH"
export DISPLAY=:0

# Use front-camera SDF and white-pole world
SDF="/home/a/Projects/PX4/SEU_MD_PX4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/huaqiccc/huaqiccc_front_cam.sdf"
WORLD="/home/a/Projects/PX4/SEU_MD_PX4/Tools/huaqiccc_simulation/worlds/perching_pole_16cm_white.world"

# Cleanup old processes
pkill -9 -f "roslaunch|roscore|gzserver|gzclient|px4|mavros_node" 2>/dev/null || true
sleep 2

# Start roscore
if pgrep -x roscore > /dev/null 2>&1; then
    echo "[LAUNCH] roscore already running, skipping"
else
    echo "[LAUNCH] Starting roscore..."
    roscore &
    sleep 5
fi

# Start simulation with camera SDF and white world
echo "[LAUNCH] Starting PX4 SITL with front-camera model and white pole..."
roslaunch /home/a/Projects/PX4/SEU_MD_PX4/launch/mavros_posix_sitl_perching_16cm.launch \
    sdf:="$SDF" world:="$WORLD" gui:=true &
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

echo "[WAIT] Extra 5s for EKF initialization..."
sleep 5

# Check camera topic
echo "[CHECK] Listing camera image topics..."
rostopic list | grep -E "image_raw|camera" || true
CAMERA_TOPIC=$(rostopic list | grep -E "front_camera.*image_raw|camera.*image_raw" | head -1)
if [ -n "$CAMERA_TOPIC" ]; then
    echo "[CHECK] Found camera topic: $CAMERA_TOPIC"
    timeout 10 rostopic hz "$CAMERA_TOPIC" -w 30 || true
else
    echo "[WARN] No obvious camera image topic found; defaulting to /front_camera/image_raw"
    CAMERA_TOPIC="/front_camera/image_raw"
fi

# Start YOLO bridge
echo "[VISION] Starting Gazebo→YOLO bridge on topic $CAMERA_TOPIC..."
python3 /home/a/Projects/YOLO4SEU_MD/scripts/gazebo_yolo_bridge.py _image_topic:="$CAMERA_TOPIC" &
BRIDGE_PID=$!
echo "[VISION] Bridge PID=$BRIDGE_PID"

# Let it run for 30 s so we can observe YOLO detection output
echo "[INFO] Bridge is running. Inspecting /yolo/detection_image and /yolo/pixel_error for 30 s..."
sleep 30
kill $BRIDGE_PID 2>/dev/null || true
wait $BRIDGE_PID 2>/dev/null || true

# Cleanup on exit
pkill -9 -f "roslaunch|roscore|gzserver|gzclient|px4|mavros_node|gazebo_yolo_bridge" 2>/dev/null || true
echo "========================================"
echo "  Done"
echo "========================================"
