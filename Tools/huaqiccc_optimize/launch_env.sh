#!/bin/bash
# =============================================================================
# huaqiccc 变形无人机 —— 一键飞行测试环境启动脚本
# =============================================================================
# 功能:
#   1. 自动清理旧进程
#   2. 自动启动 roscore (如未运行)
#   3. 自动启动 VRPN 动捕客户端
#   4. 自动启动 MAVROS (独占 /dev/ttyUSB0)
#   5. 自动启动 topic_tools throttle (40Hz vision)
#   6. 等待 FCU 心跳确认
#
# 用法:
#   ./launch_env.sh              # 启动完整环境
#   ./launch_env.sh --kill       # 仅清理，不启动
#   ./launch_env.sh --stop       # 停止所有相关进程
# =============================================================================

set -e

# --- 配置 ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/safe_space.yaml" 2>/dev/null || true

# 默认配置（如果 YAML 读取失败）
VRPN_SERVER="${vrpn_server:-192.168.1.5}"
VRPN_TRACKER="${vrpn_tracker:-Tracker1}"
HM30_PORT="${serial_port:-/dev/ttyUSB0}"
HM30_BAUD="${baudrate:-115200}"
VISION_HZ="${vision_throttle_hz:-40.0}"

# --- 颜色输出 ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info()  { echo -e "${BLUE}[INFO]${NC}  $1"; }
log_ok()    { echo -e "${GREEN}[OK]${NC}    $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# --- 解析参数 ---
KILL_ONLY=false
STOP_ONLY=false

for arg in "$@"; do
    case "$arg" in
        --kill)  KILL_ONLY=true ;;
        --stop)  STOP_ONLY=true ;;
        --help|-h)
            echo "用法: $0 [--kill|--stop|--help]"
            echo "  (无参数) 启动完整环境"
            echo "  --kill    仅清理旧进程，不启动"
            echo "  --stop    停止所有相关进程"
            echo "  --help    显示帮助"
            exit 0
            ;;
    esac
done

# =============================================================================
# 1. 清理旧进程
# =============================================================================
cleanup() {
    log_info "正在清理旧进程..."

    # 杀死 MAVROS
    if pgrep -f "mavros_node" > /dev/null 2>&1; then
        pkill -f "mavros_node" 2>/dev/null && log_ok "已停止 MAVROS"
    fi

    # 杀死 VRPN 节点
    if rosnode list 2>/dev/null | grep -q "vrpn"; then
        rosnode kill $(rosnode list 2>/dev/null | grep "vrpn") 2>/dev/null || true
        log_ok "已停止 VRPN 节点"
    fi

    # 杀死 throttle
    if pgrep -f "topic_tools throttle" > /dev/null 2>&1; then
        pkill -f "topic_tools throttle" 2>/dev/null && log_ok "已停止 throttle"
    fi

    # 杀死 mocap 转换节点（如果存在）
    if pgrep -f "mocap_to_vision_pose.py" > /dev/null 2>&1; then
        pkill -f "mocap_to_vision_pose.py" 2>/dev/null && log_ok "已停止 mocap_to_vision_pose"
    fi

    # 杀死旧地面站/GUI
    if pgrep -f "auto_flight_gui.py" > /dev/null 2>&1; then
        pkill -f "auto_flight_gui.py" 2>/dev/null && log_ok "已停止 auto_flight_gui"
    fi

    # 杀死 flight executor
    if pgrep -f "flight_executor.py" > /dev/null 2>&1; then
        pkill -f "flight_executor.py" 2>/dev/null && log_ok "已停止 flight_executor"
    fi

    # 杀死 roscore（最后）
    if pgrep -x "roscore" > /dev/null 2>&1; then
        pkill -x "roscore" 2>/dev/null || true
        pkill -f "rosout" 2>/dev/null || true
    fi

    sleep 1
}

cleanup

if [ "$STOP_ONLY" = true ]; then
    log_ok "所有相关进程已停止"
    exit 0
fi

if [ "$KILL_ONLY" = true ]; then
    log_ok "清理完成"
    exit 0
fi

# =============================================================================
# 2. 环境检查
# =============================================================================
echo ""
echo "========================================"
echo "  huaqiccc 一键飞行测试环境启动器"
echo "========================================"
echo ""

# 检查 HM30
if [ -e "$HM30_PORT" ]; then
    log_ok "HM30 地面端已连接 ($HM30_PORT)"
else
    log_error "HM30 未检测到 ($HM30_PORT)，请检查连接"
    exit 1
fi

# 检查动捕网络连通性
if ping -c 1 -W 1 "$VRPN_SERVER" > /dev/null 2>&1; then
    log_ok "动捕服务器可达 ($VRPN_SERVER)"
else
    log_warn "动捕服务器不可达 ($VRPN_SERVER)，VRPN 可能无法连接"
fi

# 加载 ROS 环境
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
    log_ok "ROS Noetic 环境已加载"
else
    log_error "ROS Noetic 未安装"
    exit 1
fi

# 加载 catkin workspace
if [ -f "$HOME/catkin_ws/devel/setup.bash" ]; then
    source "$HOME/catkin_ws/devel/setup.bash"
    log_ok "catkin_ws 已加载"
elif [ -f "$HOME/catkin_ws/devel_isolated/setup.bash" ]; then
    source "$HOME/catkin_ws/devel_isolated/setup.bash"
    log_ok "catkin_ws (isolated) 已加载"
fi

# 确保 YOLO4SEU_MD 和 PX4 在 ROS_PACKAGE_PATH 中
export ROS_PACKAGE_PATH="$HOME/Projects/YOLO4SEU_MD:${ROS_PACKAGE_PATH}"
export ROS_PACKAGE_PATH="$HOME/Projects/PX4/SEU_MD_PX4:${ROS_PACKAGE_PATH}"

# 加载 PX4 环境
if [ -f "$HOME/Projects/PX4/env_seu_md_px4.sh" ]; then
    source "$HOME/Projects/PX4/env_seu_md_px4.sh" >/dev/null 2>&1
fi

# =============================================================================
# 3. 启动 roscore
# =============================================================================
log_info "检查 roscore..."
if rosnode list > /dev/null 2>&1; then
    log_ok "roscore 已在运行 (ROS_MASTER_URI=$ROS_MASTER_URI)"
else
    log_info "启动 roscore..."
    roscore > /tmp/roscore_optimize.log 2>&1 &
    ROSCORE_PID=$!
    for i in $(seq 1 20); do
        if rosnode list > /dev/null 2>&1; then
            log_ok "roscore 就绪"
            break
        fi
        sleep 0.5
    done
    if ! rosnode list > /dev/null 2>&1; then
        log_error "roscore 启动失败"
        exit 1
    fi
fi

# =============================================================================
# 4. 启动 VRPN 动捕客户端
# =============================================================================
log_info "启动 VRPN 客户端 (server=$VRPN_SERVER, tracker=$VRPN_TRACKER)..."

if [ ! -f "/opt/ros/noetic/share/vrpn_client_ros/launch/sample.launch" ]; then
    log_error "VRPN ROS 包未安装"
    log_info "请运行: sudo apt install ros-noetic-vrpn-client-ros"
    exit 1
fi

roslaunch vrpn_client_ros sample.launch server:="$VRPN_SERVER" > /tmp/vrpn_optimize.log 2>&1 &
VRPN_PID=$!
log_ok "VRPN 客户端已启动 (PID=$VRPN_PID)"

log_info "等待 VRPN topic..."
for i in $(seq 1 30); do
    if rostopic list 2>/dev/null | grep -q "/vrpn_client_node/$VRPN_TRACKER/pose"; then
        log_ok "VRPN topic 就绪"
        break
    fi
    sleep 0.5
done

if ! rostopic list 2>/dev/null | grep -q "/vrpn_client_node/$VRPN_TRACKER/pose"; then
    log_error "VRPN topic 未检测到，动捕未连接"
    exit 1
fi

# =============================================================================
# 5. 启动 topic_tools throttle
# =============================================================================
log_info "启动 vision throttle (${VISION_HZ}Hz)..."
rosrun topic_tools throttle messages "/vrpn_client_node/$VRPN_TRACKER/pose" "$VISION_HZ" /mavros/vision_pose/pose > /tmp/throttle_optimize.log 2>&1 &
THROTTLE_PID=$!
log_ok "throttle 已启动 (PID=$THROTTLE_PID)"

sleep 2

if ! rostopic list 2>/dev/null | grep -q "/mavros/vision_pose/pose"; then
    log_warn "/mavros/vision_pose/pose topic 未就绪"
fi

# =============================================================================
# 6. 启动 MAVROS
# =============================================================================
log_info "启动 MAVROS ($HM30_PORT @ $HM30_BAUD)..."

# 使用 gcs_url 转发到本地 UDP，方便后续日志下载等工具共享连接
roslaunch mavros px4.launch \
    fcu_url:="${HM30_PORT}:${HM30_BAUD}" \
    gcs_url:="udp://@127.0.0.1:14550" \
    > /tmp/mavros_optimize.log 2>&1 &
MAVROS_PID=$!
log_ok "MAVROS 已启动 (PID=$MAVROS_PID)"

# 等待 MAVROS 就绪
log_info "等待 MAVROS 就绪..."
for i in $(seq 1 60); do
    if rosservice list 2>/dev/null | grep -q "/mavros/cmd/arming"; then
        log_ok "MAVROS 服务就绪"
        break
    fi
    sleep 1
done

if ! rosservice list 2>/dev/null | grep -q "/mavros/cmd/arming"; then
    log_error "MAVROS 未就绪"
    cat /tmp/mavros_optimize.log
    exit 1
fi

# =============================================================================
# 7. 等待 FCU 心跳
# =============================================================================
log_info "等待 FCU 心跳..."
for i in $(seq 1 30); do
    if rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q "connected: True"; then
        log_ok "FCU 已连接"
        break
    fi
    sleep 1
done

if ! rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q "connected: True"; then
    log_error "FCU 未连接，请检查 HM30 和飞控"
    exit 1
fi

# =============================================================================
# 8. 启动 flight_executor
# =============================================================================
log_info "启动 flight_executor..."
cd "$SCRIPT_DIR"
python3 flight_executor.py > /tmp/flight_executor_optimize.log 2>&1 &
FLIGHT_EXEC_PID=$!
log_ok "flight_executor 已启动 (PID=$FLIGHT_EXEC_PID)"

# 等待服务可用
log_info "等待 flight_executor 服务..."
for i in $(seq 1 20); do
    if rosservice list 2>/dev/null | grep -q "/optimize/start_flight"; then
        log_ok "flight_executor 服务就绪"
        break
    fi
    sleep 0.5
done

if ! rosservice list 2>/dev/null | grep -q "/optimize/start_flight"; then
    log_error "flight_executor 服务未就绪"
    cat /tmp/flight_executor_optimize.log
    exit 1
fi

# =============================================================================
# 9. 打印状态
# =============================================================================
echo ""
echo "========================================"
log_ok "飞行测试环境已就绪"
echo "========================================"
echo ""
log_info "现在可以启动 GUI: python3 $SCRIPT_DIR/auto_flight_gui.py"
echo ""
log_info "或手动检查:"
echo "  rostopic echo -n 1 /mavros/state"
echo "  rosservice call /mavros/param/get \"param_id: 'MPCA_MODE'\""
echo "  rostopic hz /mavros/vision_pose/pose"
echo ""

# 保持脚本运行，捕获 Ctrl+C
trap cleanup EXIT INT TERM
wait
