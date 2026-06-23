#!/bin/bash
# =============================================================================
# One-click ground station launcher
# Vision pipeline: your 3 proven commands (unchanged)
# Control pipeline: ROS + MAVROS via HM30
# =============================================================================
#
# USER_CONFIG: This script contains hard-coded absolute paths from the original
# development environment (/home/a/Projects/...).  Search for "USER_CONFIG"
# below and update the paths to match your own machine layout before running.
# =============================================================================

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Config
HM30_PORT="/dev/ttyUSB0"
# HM30 supports 9600/38400/57600/115200/230400.  115200 is the default but easily
# saturates when PX4 streams high-rate telemetry in OFFBOARD.  If you raise the HM30
# "Datalink Baud Rate" to 230400 in the ground-unit OLED menu, set this env var too:
#   HM30_BAUD=230400 ./start_ground_station_auto.sh
HM30_BAUD="${HM30_BAUD:-115200}"
VRPN_SERVER="192.168.1.5"
TRACKER="Tracker1"

echo -e "${GREEN}=== huaqiccc Ground Station Launcher ===${NC}"
echo -e "${YELLOW}HM30 port: ${HM30_PORT} @ ${HM30_BAUD} baud${NC}"
if [ "$HM30_BAUD" = "115200" ]; then
    echo -e "${YELLOW}TIP: If video drops in OFFBOARD, raise HM30 datalink baud to 230400 and restart with HM30_BAUD=230400${NC}"
fi

# --- Check dependencies ---
if [ ! -e "$HM30_PORT" ]; then
    echo -e "${RED}ERROR: $HM30_PORT not found${NC}"
    echo "Please connect HM30 ground station USB"
    exit 1
fi

source /opt/ros/noetic/setup.bash 2>/dev/null || {
    echo -e "${RED}ERROR: ROS Noetic not found${NC}"
    exit 1
}

if ! pgrep -x "rosmaster" > /dev/null; then
    echo "[*] Starting roscore..."
    roscore &
    sleep 3
fi

# --- Cleanup function ---
cleanup() {
    echo ""
    echo -e "${YELLOW}[*] Cleaning up...${NC}"
    # Kill our background processes
    for pid in $VRPN_PID $MAVROS_PID $THROTTLE_PID $YOLO_PID; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
        fi
    done
    echo -e "${GREEN}[*] Done${NC}"
}
trap cleanup EXIT INT TERM

# --- 1. Start VRPN ---
echo "[*] Starting VRPN client..."
nohup roslaunch vrpn_client_ros sample.launch server:=$VRPN_SERVER > /tmp/vrpn_gs.log 2>&1 &
VRPN_PID=$!
sleep 3

if ! rosnode list 2>/dev/null | grep -q "vrpn_client_node"; then
    echo -e "${RED}ERROR: VRPN failed to start${NC}"
    cat /tmp/vrpn_gs.log
    exit 1
fi
echo -e "${GREEN}[✓] VRPN running (PID $VRPN_PID)${NC}"

# --- 2. Start MAVROS ---
echo "[*] Starting MAVROS..."
nohup roslaunch mavros px4.launch fcu_url:="${HM30_PORT}:${HM30_BAUD}" > /tmp/mavros_gs.log 2>&1 &
MAVROS_PID=$!
sleep 5

if ! rosnode list 2>/dev/null | grep -q "mavros"; then
    echo -e "${RED}ERROR: MAVROS failed to start${NC}"
    cat /tmp/mavros_gs.log
    exit 1
fi
echo -e "${GREEN}[✓] MAVROS running (PID $MAVROS_PID)${NC}"

# --- 3. Start topic_tools throttle ---
echo "[*] Starting vision throttle (40Hz)..."
nohup rosrun topic_tools throttle messages /vrpn_client_node/${TRACKER}/pose 40.0 /mavros/vision_pose/pose > /tmp/throttle_gs.log 2>&1 &
THROTTLE_PID=$!
sleep 2

if ! rostopic list 2>/dev/null | grep -q "/mavros/vision_pose/pose"; then
    echo -e "${YELLOW}WARNING: throttle topic not ready yet, will retry...${NC}"
    sleep 3
fi
echo -e "${GREEN}[✓] Vision throttle running (PID $THROTTLE_PID)${NC}"

# --- 4. Verify FCU heartbeat via MAVROS ---
echo "[*] Waiting for FCU heartbeat via MAVROS..."
for i in {1..10}; do
    if rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q "connected: True"; then
        echo -e "${GREEN}[✓] FCU connected via MAVROS${NC}"
        break
    fi
    sleep 1
    if [ $i -eq 10 ]; then
        echo -e "${YELLOW}WARNING: FCU heartbeat not confirmed, continuing anyway...${NC}"
    fi
done

# --- 5. Start YOLO vision node ---
echo ""
echo "[*] Starting YOLO vision node..."
# USER_CONFIG: change the following path to your YOLO4SEU_MD clone
export ROS_PACKAGE_PATH="${HOME}/Projects/YOLO4SEU_MD:${ROS_PACKAGE_PATH}"

# Default to SIYI main RTSP stream (same as run_yolo_preview.py).
# Override examples:
#   YOLO_URL=rtsp://192.168.144.25:8554/sub.264 ./start_ground_station_auto.sh
YOLO_URL="${YOLO_URL:-rtsp://192.168.144.25:8554/main.264}"
echo "[*] YOLO camera URL: ${YOLO_URL}"

nohup roslaunch YOLO4SEU_MD rtsp_pillar.launch rtsp_url:="${YOLO_URL}" publish_cmd_vel:=false > /tmp/yolo_gs.log 2>&1 &
YOLO_PID=$!

# Wait longer for model loading; check both node existence and pixel_error topic
YOLO_READY=false
for i in $(seq 1 20); do
    sleep 1
    if rosnode list 2>/dev/null | grep -q "rtsp_pillar_node" && \
       rostopic list 2>/dev/null | grep -q "/yolo/pixel_error"; then
        YOLO_READY=true
        break
    fi
done

if [ "$YOLO_READY" = true ]; then
    echo -e "${GREEN}[✓] YOLO vision node running (PID $YOLO_PID, URL ${YOLO_URL})${NC}"
else
    echo -e "${YELLOW}WARNING: YOLO node not ready after 20s${NC}"
    echo "YOLO log tail:"
    tail -n 20 /tmp/yolo_gs.log
    echo -e "${YELLOW}Continuing anyway...${NC}"
fi

# --- 6. Start control ground station ---
echo ""
echo -e "${GREEN}=== Starting Control Ground Station ===${NC}"
echo "Vision pipeline: VRPN → MAVROS → FCU (handled externally)"
echo "Control pipeline: Ground Station → ROS/MAVROS → HM30 → FCU"
echo ""

# USER_CONFIG: change the following path to where you cloned this repository
python3 ~/Projects/PX4/SEU_MD_PX4/Tools/huaqiccc_ground_station/scripts/control_ground_station_auto.py

# Ground station exited, cleanup will run via trap
