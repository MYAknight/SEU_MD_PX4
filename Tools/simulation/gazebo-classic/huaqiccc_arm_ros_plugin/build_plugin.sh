#!/bin/bash
# build_plugin.sh
# 一键编译 huaqiccc ROS Gazebo 插件
#
# 依赖：gazebo-dev, roscpp, std_msgs
# Ubuntu 安装：sudo apt install libgazebo-dev ros-noetic-roscpp ros-noetic-std-msgs

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"

echo "======================================"
echo "  huaqiccc Arm ROS Plugin Builder"
echo "======================================"

# 创建 build 目录
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# 配置并编译
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

echo ""
echo "编译完成！生成文件："
ls -la "$BUILD_DIR"/libhuaqiccc_arm_ros_plugin.so

echo ""
echo "======================================"
echo "安装步骤（二选一）："
echo ""
echo "A) 复制到 PX4 Gazebo 插件目录（推荐）："
echo "   cp $BUILD_DIR/libhuaqiccc_arm_ros_plugin.so \\"
echo "      ~/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic/"
echo ""
echo "B) 复制到系统 Gazebo 插件目录："
echo "   sudo cp $BUILD_DIR/libhuaqiccc_arm_ros_plugin.so /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/"
echo ""
echo "C) 添加到 GAZEBO_PLUGIN_PATH："
echo "   export GAZEBO_PLUGIN_PATH=$BUILD_DIR:\$GAZEBO_PLUGIN_PATH"
echo "======================================"
