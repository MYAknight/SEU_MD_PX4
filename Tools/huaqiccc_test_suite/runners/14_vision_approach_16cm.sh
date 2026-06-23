#!/bin/bash
# 视觉引导栖落接近测试（16cm 红色 pole）
# 使用带前视相机的 SITL 模型，无人机初始 YAW 偏置约 20°，验证纯视觉 YAW 对齐 + 机臂展开 + 接近 + 盲推 + CONTACT。
set -e

export MPCA_MODE=${1:-0}
# 是否开启 GUI：默认关闭，避免 YOLO + GUI 并发导致 poll timeout
export GUI_ENABLED=${GUI_ENABLED:-true}

echo "========================================"
echo "  Vision Approach Test (16cm red)    "
echo "  MPCA_MODE=$MPCA_MODE"
echo "  GUI=$GUI_ENABLED"
echo "========================================"

source /opt/ros/noetic/setup.bash
source /home/a/catkin_ws/devel_isolated/setup.bash
source /home/a/Projects/PX4/env_seu_md_px4.sh
export GAZEBO_PLUGIN_PATH="/home/a/Projects/PX4/SEU_MD_PX4/build/px4_sitl_default/build_gazebo-classic:$GAZEBO_PLUGIN_PATH"
# 追加 SITL 模型路径（env_seu_md_px4.sh 已包含 huaqiccc_simulation/models）
export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:/home/a/Projects/PX4/SEU_MD_PX4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
export DISPLAY=:0

# Use front-camera SDF and red-pole world
SDF="/home/a/Projects/PX4/SEU_MD_PX4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/huaqiccc/huaqiccc_front_cam.sdf"

# Camera horizontal offset (px, positive = camera center is right of image center).
# 当前 SDF 中 front_camera_link 向 y=-0.10 m（向右平移 10 cm），光轴仍与机体前进方向平行。
# 该平移在 2.5m 距离约产生 15 px 像素偏移，因此默认 CAMERA_OFFSET_X=15.0 与之对应。
# 如需关闭偏移，把 SDF 的 y 改回 0 并在此处改回 0.0。
CAMERA_OFFSET_X=${CAMERA_OFFSET_X:-15.0}
WORLD="/home/a/Projects/PX4/SEU_MD_PX4/Tools/huaqiccc_simulation/worlds/perching_pole_16cm_red.world"
YOLO_DIR="/home/a/Projects/YOLO4SEU_MD"

# 优先使用 SITL 专用模型，若不存在则回退到实机模型
if [ -f "$YOLO_DIR/pillar_detector/weights/sitl_best.pt" ]; then
    MODEL_PATH="$YOLO_DIR/pillar_detector/weights/sitl_best.pt"
    CONF_THRESHOLD=0.25
    PROCESS_EVERY_N=1
    MIN_HITS=1
    echo "[VISION] 使用 SITL 专用模型: $MODEL_PATH"
else
    MODEL_PATH="$YOLO_DIR/pillar_detector/weights/best.pt"
    CONF_THRESHOLD=0.2
    PROCESS_EVERY_N=1
    MIN_HITS=1
    echo "[VISION] 使用实机模型: $MODEL_PATH (建议训练 SITL 专用模型)"
fi

# Inject MPCA_MODE into build params (same as 13_grasp_16cm.sh)
ROMFS_PARAMS="/home/a/Projects/PX4/SEU_MD_PX4/ROMFS/px4fmu_common/init.d-posix/px4-rc.params"
BUILD_PARAMS="/home/a/Projects/PX4/SEU_MD_PX4/build/px4_sitl_default/etc/init.d-posix/px4-rc.params"
if [ ! -f "$BUILD_PARAMS" ]; then
    echo "[CONFIG] Build params script missing; copying from ROMFS..."
    mkdir -p "$(dirname "$BUILD_PARAMS")"
    cp "$ROMFS_PARAMS" "$BUILD_PARAMS"
fi

echo "[CONFIG] Setting MPCA_MODE=$MPCA_MODE in build params..."
sed -i "s/^param set-default MPCA_MODE .*/param set-default MPCA_MODE $MPCA_MODE/" "$BUILD_PARAMS"

# Cleanup old ROS logs
if command -v rosclean >/dev/null 2>&1; then
    rosclean purge -y 2>/dev/null || true
fi
if [ -d "$HOME/.ros/log" ]; then
    LOG_SIZE=$(du -sm "$HOME/.ros/log" 2>/dev/null | cut -f1)
    if [ -n "$LOG_SIZE" ] && [ "$LOG_SIZE" -gt 100 ]; then
        rm -rf "$HOME/.ros/log"/*
    fi
fi

# Cleanup old processes
pkill -9 -f "roslaunch|roscore|gzserver|gzclient|px4|mavros_node|gazebo_yolo_bridge|vision_approach_test" 2>/dev/null || true
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

# Start simulation with front-camera SDF, red world, and an initial yaw offset.
# 初始 YAW 给一个偏移，便于展示纯视觉 YAW 跟踪能力（0.35 rad ≈ 20°）
INITIAL_YAW=${INITIAL_YAW:-0.35}
roslaunch /home/a/Projects/PX4/SEU_MD_PX4/launch/mavros_posix_sitl_perching_16cm.launch \
    sdf:="$SDF" world:="$WORLD" Y:="$INITIAL_YAW" gui:="$GUI_ENABLED" &
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
    pkill -9 -f "roslaunch|roscore|gzserver|gzclient|px4|mavros_node|gazebo_yolo_bridge|vision_approach_test" 2>/dev/null || true
    exit 1
fi

# Extra wait for EKF
echo "[WAIT] Extra 5s for EKF initialization..."
sleep 5

# Detect actual camera topic and start YOLO bridge
CAMERA_TOPIC=$(rostopic list | grep -E "front_camera.*image_raw" | head -1)
if [ -z "$CAMERA_TOPIC" ]; then
    CAMERA_TOPIC="/huaqiccc/front_camera/image_raw"
fi
echo "[VISION] Starting Gazebo→YOLO bridge on $CAMERA_TOPIC..."
PYTHONUNBUFFERED=1 stdbuf -oL python3 "$YOLO_DIR/scripts/gazebo_yolo_bridge.py" \
    _image_topic:="$CAMERA_TOPIC" \
    _model_path:="$MODEL_PATH" \
    _conf_threshold:="$CONF_THRESHOLD" \
    _process_every_n:="$PROCESS_EVERY_N" \
    _min_hits:="$MIN_HITS" > /tmp/gazebo_yolo_bridge.log 2>&1 &
BRIDGE_PID=$!
echo "[VISION] Bridge PID=$BRIDGE_PID"

# Give bridge time to load YOLO model
echo "[WAIT] 8s for YOLO model load..."
sleep 8

# Run vision approach test
echo "[FLIGHT] Starting vision approach test..."
PYTHONUNBUFFERED=1 stdbuf -oL python3 /home/a/Projects/PX4/SEU_MD_PX4/Tools/huaqiccc_test_suite/perching/vision_approach_test.py _camera_offset_x:="$CAMERA_OFFSET_X" 2>&1 | tee /tmp/vision_approach_test.log

# Stop simulation
echo "[CLEAN] Stopping simulation..."
kill $BRIDGE_PID 2>/dev/null || true
pkill -9 -f "roslaunch|roscore|gzserver|gzclient|px4|mavros_node|gazebo_yolo_bridge|vision_approach_test" 2>/dev/null || true
sleep 2

echo "========================================"
echo "  Done"
echo "========================================"

# Show results
ls -lt ~/huaqiccc_logs/vision_approach_*.csv 2>/dev/null | head -1
