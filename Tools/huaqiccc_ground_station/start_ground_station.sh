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
HM30_BAUD="115200"
VRPN_SERVER="192.168.1.5"
TRACKER="Tracker1"

echo -e "${GREEN}=== huaqiccc Ground Station Launcher ===${NC}"

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
    for pid in $VRPN_PID $MAVROS_PID $THROTTLE_PID; do
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

# --- 5. Start control ground station ---
echo ""
echo -e "${GREEN}=== Starting Control Ground Station ===${NC}"
echo "Vision pipeline: VRPN → MAVROS → FCU (handled externally)"
echo "Control pipeline: Ground Station → ROS/MAVROS → HM30 → FCU"
echo ""

# USER_CONFIG: change the following path to where you cloned this repository
python3 ~/Projects/PX4/SEU_MD_PX4/Tools/huaqiccc_ground_station/scripts/control_ground_station_ros.py

# Ground station exited, cleanup will run via trap
