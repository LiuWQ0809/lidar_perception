#!/bin/bash

# Fusion Perception C++ - Clean Script
# 清理脚本

PROJECT_DIR="/home/nvidia/liuwq/lidar_perception"

echo "========================================"
echo "Cleaning Build Files"
echo "========================================"

cd "$PROJECT_DIR"

# 清理构建目录
if [ -d "build" ]; then
    echo "Removing build directory..."
    rm -rf build
fi

# 清理安装目录
if [ -d "install" ]; then
    echo "Removing install directory..."
    rm -rf install
fi

# 清理日志目录
if [ -d "log" ]; then
    echo "Removing log directory..."
    rm -rf log
fi

echo ""
echo "Clean complete!"
