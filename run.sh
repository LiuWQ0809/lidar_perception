#!/bin/bash

# Fusion Perception C++ - Run Script
# 运行脚本

PROJECT_DIR="/home/nvidia/liuwq/lidar_perception"
PROCESS_NAME="fusion_perception_node"

# 检查是否已编译
if [ ! -f "$PROJECT_DIR/install/lib/fusion_cpp/fusion_perception_node" ]; then
    echo "Error: Project not built yet!"
    echo "Please run: ./build.sh"
    exit 1
fi

# 检查进程是否已启动，如果启动则kill掉
echo "Checking for existing $PROCESS_NAME process..."
EXISTING_PID=$(pgrep -f "$PROCESS_NAME" | head -1)
if [ ! -z "$EXISTING_PID" ]; then
    echo "Found existing process with PID: $EXISTING_PID"
    echo "Killing existing process..."
    kill -9 "$EXISTING_PID" 2>/dev/null
    sleep 1
    
    # 验证进程是否已被kill
    if pgrep -f "$PROCESS_NAME" > /dev/null; then
        echo "Warning: Failed to kill existing process. Some processes may still be running."
    else
        echo "Successfully killed existing process."
    fi
fi

sleep 1

# 检查是否在ROS2环境中
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS2 environment..."
    source /opt/ros/humble/setup.bash
fi

echo "========================================"
echo "Running Fusion Perception Node"
echo "========================================"
echo ""

# 设置环境变量
export LD_LIBRARY_PATH="$PROJECT_DIR/install/lib:$LD_LIBRARY_PATH"

# 设置ROS2日志级别 (DEBUG, INFO, WARN, ERROR, FATAL)
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_COLORIZED_OUTPUT=1

# 设置默认日志级别为WARN (只显示警告和错误)
# 可选值: DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50
export RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED=1

# 设置日志目录到工程目录
export ROS_LOG_DIR="$PROJECT_DIR/logs"
mkdir -p "$ROS_LOG_DIR"

echo "Log directory: $ROS_LOG_DIR"
echo "Log level: INFO (showing debug information)"
echo ""
echo "Starting Fusion Perception Node..."
echo "========================================"
echo ""

# 运行节点 (使用--ros-args设置日志级别)
cd "$PROJECT_DIR"
"$PROJECT_DIR/install/lib/fusion_cpp/fusion_perception_node" \
    --ros-args --log-level INFO &

NODE_PID=$!
echo "Node started with PID: $NODE_PID"

# 保存PID到文件，方便后续管理
echo $NODE_PID > "$PROJECT_DIR/.fusion_node.pid"

wait $NODE_PID
