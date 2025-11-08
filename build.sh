#!/bin/bash

# Fusion Perception C++ - Build Script
# 构建脚本 - 直接在项目目录编译

set -e  # 遇到错误立即退出

echo "========================================"
echo "Building Fusion Perception C++ Package"
echo "========================================"
cd /home/nvidia/liuwq/fusion_cpp
# 获取脚本所在目录作为项目目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"

# 检查是否在ROS2环境中
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS2 environment not sourced"
    echo "Please run: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "ROS2 Distribution: $ROS_DISTRO"
echo "Project Directory: $PROJECT_DIR"

# 进入项目目录
cd "$PROJECT_DIR"

# 创建构建目录
if [ ! -d "build" ]; then
    mkdir build
fi

cd build

# 安装依赖
echo ""
echo "Installing dependencies..."
rosdep install --from-paths .. --ignore-src -r -y || true

# CMake配置
echo ""
echo "Configuring CMake..."
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$PROJECT_DIR/install"

# 编译
echo ""
echo "Building package..."
make -j$(nproc)

# 安装
echo ""
echo "Installing..."
make install

# 检查编译结果
if [ $? -eq 0 ]; then
    echo ""
    echo "========================================"
    echo "Build Successful!"
    echo "========================================"
    echo ""
    echo "To run the node, use:"
    echo "  cd $PROJECT_DIR"
    echo "  source /opt/ros/humble/setup.bash"
    echo "  export LD_LIBRARY_PATH=$PROJECT_DIR/install/lib:\$LD_LIBRARY_PATH"
    echo "  ./install/lib/fusion_cpp/fusion_perception_node"
    echo ""
    echo "Or use the run script:"
    echo "  ./run.sh"
else
    echo ""
    echo "========================================"
    echo "Build Failed!"
    echo "========================================"
    exit 1
fi
