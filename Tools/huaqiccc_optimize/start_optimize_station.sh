#!/bin/bash
# =============================================================================
# start_optimize_station.sh — 一键启动 optimize 飞行测试环境 + GUI
# =============================================================================
# 功能:
#   1. 后台启动 launch_env.sh（roscore + VRPN + MAVROS + throttle + flight_executor）
#   2. 等待 /optimize/start_flight 服务就绪
#   3. 前台启动 auto_flight_gui.py
#   4. GUI 关闭后自动清理所有环境进程
#
# 用法:
#   cd <your_repo_path>/Tools/huaqiccc_optimize
#   ./start_optimize_station.sh
#
# USER_CONFIG: This script and the tools in this directory originally lived at
# ~/Projects/optimize.  After moving into SEU_MD_PX4, the script now uses
# SCRIPT_DIR for relative paths.  If you copy this folder elsewhere, update the
# cd path in the usage comment above and any hard-coded paths you find below.
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

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

# --- 清理函数 ---
cleanup() {
    echo ""
    log_warn "正在关闭 GUI 并清理环境..."

    # 先停止后台的 launch_env.sh 进程（如果还在）
    if [ -n "$ENV_PID" ] && kill -0 "$ENV_PID" 2>/dev/null; then
        kill "$ENV_PID" 2>/dev/null || true
        wait "$ENV_PID" 2>/dev/null || true
    fi

    # 再调用 launch_env.sh --stop 确保所有子进程被清理
    if [ -x "$SCRIPT_DIR/launch_env.sh" ]; then
        "$SCRIPT_DIR/launch_env.sh" --stop >/dev/null 2>&1 || true
    fi

    log_ok "清理完成"
}
trap cleanup EXIT INT TERM

# --- 启动环境 ---
echo "========================================"
echo "  optimize 飞行测试环境 + GUI 启动器"
echo "========================================"
echo ""

log_info "启动飞行环境（后台）..."
"$SCRIPT_DIR/launch_env.sh" > /tmp/optimize_station_env.log 2>&1 &
ENV_PID=$!
log_ok "launch_env.sh 已启动 (PID=$ENV_PID)"

# --- 等待 flight_executor 服务就绪 ---
log_info "等待 flight_executor 服务就绪..."
SERVICE_READY=false
for i in $(seq 1 60); do
    if rosservice list 2>/dev/null | grep -q "/optimize/start_flight"; then
        SERVICE_READY=true
        break
    fi
    sleep 0.5
done

if [ "$SERVICE_READY" != true ]; then
    log_error "flight_executor 服务未就绪"
    echo ""
    log_info "环境日志: /tmp/optimize_station_env.log"
    cat /tmp/optimize_station_env.log
    exit 1
fi

log_ok "flight_executor 服务已就绪"

# --- 启动 GUI ---
echo ""
echo "========================================"
log_ok "环境已就绪，正在启动 GUI..."
echo "========================================"
echo ""

python3 "$SCRIPT_DIR/auto_flight_gui.py"

# GUI 退出后，trap cleanup 会自动执行清理
