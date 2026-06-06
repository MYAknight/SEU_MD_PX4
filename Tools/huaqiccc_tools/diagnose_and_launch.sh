#!/bin/bash
# huaqiccc 仿真链路一键诊断与启动脚本
# 用法: ./huaqiccc_diagnose_and_launch.sh [launch|gui|diag]

set -e

ROS_DISTRO="noetic"
PX4_DIR="$HOME/PX4-Autopilot"
CTRLF_DIR="$HOME/huaqiccc_tools/gui"
PLUGIN_SO="$PX4_DIR/build/px4_sitl_default/build_gazebo-classic/libhuaqiccc_arm_ros.so"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"

cmd="${1:-diag}"

# ========== 颜色 ==========
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_ok()  { echo -e "${GREEN}[OK]${NC} $1"; }
print_warn(){ echo -e "${YELLOW}[WARN]${NC} $1"; }
print_err() { echo -e "${RED}[ERR]${NC} $1"; }

# ========== 诊断 ==========
diagnose() {
    echo "========================================"
    echo "  huaqiccc 仿真链路诊断"
    echo "========================================"

    # 1. ROS
    echo ""
    echo "[1] ROS 环境"
    if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
        print_ok "ROS $ROS_DISTRO 已安装"
    else
        print_err "ROS $ROS_DISTRO 未找到"
    fi

    # 2. PX4 编译产物
    echo ""
    echo "[2] PX4 编译产物"
    if [ -f "$PX4_BIN" ]; then
        print_ok "px4 二进制存在 ($(date -r "$PX4_BIN" '+%Y-%m-%d %H:%M:%S'))"
    else
        print_err "px4 二进制缺失: $PX4_BIN"
    fi

    # 3. Gazebo Plugin
    echo ""
    echo "[3] Gazebo 变形插件"
    if [ -f "$PLUGIN_SO" ]; then
        print_ok "插件存在: $(basename $PLUGIN_SO) ($(date -r "$PLUGIN_SO" '+%m-%d %H:%M'))"
        ls -lh "$PLUGIN_SO" | awk '{print "    大小:", $5}'
    else
        print_err "插件缺失: $PLUGIN_SO"
        print_warn "请运行: cd ~/huaqiccc_ws && catkin_make"
    fi

    # 检查旧版本残留
    OLD_PLUGIN="$PX4_DIR/build/px4_sitl_default/build_gazebo-classic/libhuaqiccc_arm_ros_plugin.so"
    if [ -f "$OLD_PLUGIN" ]; then
        print_warn "发现旧版插件残留: libhuaqiccc_arm_ros_plugin.so"
        echo "    SDF 中引用的是 libhuaqiccc_arm_ros.so，旧版不影响"
    fi

    # 4. SDF 模型检查
    echo ""
    echo "[4] SDF 模型插件配置"
    SDF="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/huaqiccc/huaqiccc.sdf"
    if grep -q 'libhuaqiccc_arm_ros.so' "$SDF"; then
        print_ok "SDF 已挂载 libhuaqiccc_arm_ros.so"
    else
        print_err "SDF 未找到插件挂载！"
    fi

    # 5. launch 文件
    echo ""
    echo "[5] Launch 文件"
    LAUNCH="$PX4_DIR/launch/mavros_posix_sitl.launch"
    VEHICLE=$(grep 'arg name="vehicle"' "$LAUNCH" | grep -oP 'default="\K[^"]+')
    print_ok "默认 vehicle: $VEHICLE"
    if [ "$VEHICLE" != "huaqiccc" ]; then
        print_warn "launch 默认不是 huaqiccc！当前是 $VEHICLE"
    fi

    # 6. 机架配置
    echo ""
    echo "[6] 机架配置 (4400_huaqiccc)"
    AIRFRAME="$PX4_DIR/ROMFS/px4fmu_common/init.d/airframes/4400_huaqiccc"
    if [ -f "$AIRFRAME" ]; then
        print_ok "机架配置存在"
        echo "    电机数: $(grep -c 'CA_ROTOR' "$AIRFRAME" | head -1) 处参数"
    else
        print_err "机架配置缺失"
    fi

    # 7. PX4 源码修改确认
    echo ""
    echo "[7] PX4 源码修改确认"
    if grep -q 'MAV_CMD_SET_ROTOR_CONFIG' "$PX4_DIR/src/modules/mavlink/mavlink_receiver.cpp"; then
        print_ok "mavlink_receiver.cpp 已添加 31000 命令处理"
    else
        print_err "mavlink_receiver.cpp 未找到 31000 处理！"
    fi

    if grep -q 'huaqiccc_morph_angle' "$PX4_DIR/src/modules/control_allocator/ControlAllocator.cpp"; then
        print_ok "ControlAllocator.cpp 已添加变形支持"
    else
        print_err "ControlAllocator.cpp 未找到变形逻辑！"
    fi

    if grep -q 'dynamic_rotor_config' "$PX4_DIR/src/modules/control_allocator/ActuatorEffectiveness/ActuatorEffectivenessRotors.cpp"; then
        print_ok "ActuatorEffectivenessRotors.cpp 已添加动态旋翼支持"
    else
        print_err "ActuatorEffectivenessRotors.cpp 未找到动态旋翼逻辑！"
    fi

    # 8. ROS 包检查
    echo ""
    echo "[8] ROS 工作空间"
    if [ -d "$HOME/huaqiccc_ws/devel" ]; then
        print_ok "huaqiccc_ws 已编译"
    else
        print_warn "huaqiccc_ws 未编译"
    fi

    echo ""
    echo "========================================"
    echo "  诊断完成"
    echo "========================================"
}

# ========== 启动仿真 ==========
launch_sim() {
    echo "[INFO] 启动 huaqiccc SITL 仿真..."
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    roslaunch "$PX4_DIR/launch/mavros_posix_sitl.launch"
}

# ========== 启动 GUI ==========
launch_gui() {
    echo "[INFO] 启动统一控制 GUI..."
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    python3 "$CTRLF_DIR/huaqiccc_unified_control_gui.py"
}

# ========== 主入口 ==========
case "$cmd" in
    diag|d|"")
        diagnose
        ;;
    launch|l|sim)
        launch_sim
        ;;
    gui|g)
        launch_gui
        ;;
    all|a)
        # 只打印提示，不后台启动（避免嵌套问题）
        echo "请开两个终端分别执行:"
        echo "  终端1: $0 launch"
        echo "  终端2: $0 gui"
        ;;
    *)
        echo "用法: $0 [diag|launch|gui|all]"
        echo "  diag   - 运行诊断（默认）"
        echo "  launch - 启动 SITL 仿真"
        echo "  gui    - 启动统一控制 GUI"
        echo "  all    - 显示双终端启动提示"
        ;;
esac
